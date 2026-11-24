// ParkingVisionNode 구현이다. 노드 인터페이스와 스레드 구성은 parking_vision_node.hpp 머리 주석에 있다.
//
// 두 가지 일을 한다.
// - 실시간 상태(~/state): 타이머가 track_rate_hz(기본 10 Hz)로 가장 새 프레임 하나를 ParkingTracker에 넣고
//   NOT_IN_VIEW / MOVING / IN_POSITION / OFF_POSITION 등을 발행한다. 화면 표시용이며 팔 단계의 진행 조건이 아니다.
// - 1회 판정(~/check): 요청 뒤에 찍힌 프레임만 최대 3초, 15프레임 모아 중앙값으로 판정한다. 결과 status는
//   ARRIVED(허용 범위 안) / MISALIGNED(밖) / NOT_FOUND(마커를 못 찾았거나 쓸 수 있는 프레임이 5개 미만) /
//   AMBIGUOUS(같은 마커 ID가 둘 이상) / VIEW_UNVERIFIED(고정 마커로 카메라 시야를 확인하지 못함)다.
//   요청 전 프레임을 쓰지 않는 이유는, 움직이던 Pinky가 허용 범위를 잠깐 지나간 프레임을 도착으로 세지 않기 위해서다.
// 판정 로직 자체는 ROS 의존성 없는 parking_vision 라이브러리에 있고, 이 파일은 스레드·시계·메시지 변환만 맡는다.
#include "parking_vision_ros/parking_vision_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <opencv2/core/version.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "parking_vision/parking_observation.hpp"

namespace parking_vision_ros {
namespace {

using parking_vision::Corners;
using parking_vision::ParkingResult;
using parking_vision::ParkingSpot;

// 판정은 프레임이 오면 깨어나고 추적기는 가장 새 프레임만 쓰므로, 짧은 버퍼로 충분하다(30 fps면 약 0.33초 분량).
constexpr std::size_t kBufferedFrames = 10;
// 판정 스레드가 새 프레임을 한 번에 기다리는 최대 시간. 이 주기로 마감과 노드 종료도 확인한다.
constexpr auto kFrameWait = std::chrono::milliseconds(100);
// 판정마다 스레드 하나를 최대 3초 붙잡는다. 이보다 많이 진행 중이면 요청에 PARKING_CHECK_BUSY로 답한다.
constexpr std::size_t kMaxConcurrentChecks = 4;
// 정지 판정은 정지 창(1초)의 0.8 안에 kMinStillFrames(5)개 프레임이 필요하다. 8 Hz면 0.8초에 6.4프레임이라
// 타이머 흔들림에도 여유가 있다.
constexpr double kMinTrackRateHz = 8.0;
constexpr double kMaxTrackRateHz = 60.0;

// steady clock 등으로 계산한 초 값을 주어진 시계 종류의 rclcpp::Time으로 바꾼다.
rclcpp::Time to_time(double seconds, rcl_clock_type_t clock_type) {
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), clock_type);
}

// 네 꼭짓점(px)을 메시지의 평탄한 배열 [x0, y0, x1, y1, ...]로 옮긴다. 순서는 ArUco 순서(좌상, 우상, 우하, 좌하)다.
void copy_corners(const Corners& corners, std::array<double, 8>& out) {
  for (std::size_t i = 0; i < corners.size(); ++i) {
    out[2 * i] = corners[i].x;
    out[2 * i + 1] = corners[i].y;
  }
}

// ParkingState와 CheckParking 응답은 이 필드 이름들을 같이 쓴다. 관제에서 두 JSON 응답이 judgment_fields()와
// image_overlay()를 같이 쓰던 것과 같다. 그래서 아래 함수들은 Out 타입을 템플릿으로 받는다.
// JSON에서 null이던 값은 has_* 플래그로 나타낸다(has_* = false면 값 필드는 의미 없음).

// 주차 위치 자체의 정보(마커 ID, 전방 축, 허용 오차, 시야 확인 여부)와 시야 밀림 크기(px)를 채운다.
template <typename Out>
void fill_spot_fields(Out& out, const ParkingSpot& spot, std::optional<double> view_shift_px) {
  out.marker_id = spot.marker_id;
  out.marker_forward_axis = spot.marker_forward_axis.value_or("");
  out.tolerance_position_mm = spot.tolerance.position_mm;
  out.tolerance_yaw_deg = spot.tolerance.yaw_deg;
  out.view_check = !spot.anchors.empty();
  out.has_view_shift = view_shift_px.has_value();
  out.view_shift_px = view_shift_px.value_or(0.0);
}

// 판정 결과를 채운다. 편차 dx·dy(mm, 등록 마커 축 기준), dyaw(deg, 천장 카메라에서 볼 때 반시계가 +)와,
// 등록 때 marker_forward_axis가 기록돼 있으면 Pinky 자신의 앞(forward_mm)·왼쪽(left_mm) 기준 편차도 채운다.
// 허용 오차 대비 비율(position_ratio, yaw_ratio)과 WITHIN/NEAR/FAR 수준은 세 값이 다 있을 때만 싣는다.
template <typename Out>
void fill_judgment(Out& out, const ParkingResult& result, const ParkingSpot& spot) {
  fill_spot_fields(out, spot, result.view_shift_px);
  if (result.offset) {
    out.has_offset = true;
    out.dx_mm = result.offset->dx_mm;
    out.dy_mm = result.offset->dy_mm;
    out.distance_mm = result.offset->distance_mm();
    out.dyaw_deg = result.offset->dyaw_deg;
    if (spot.marker_forward_axis) {
      const auto [forward, left] = parking_vision::pinky_frame_offset(*result.offset, *spot.marker_forward_axis);
      out.has_pinky_frame = true;
      out.forward_mm = forward;
      out.left_mm = left;
    }
  }
  if (result.position_ratio && result.yaw_ratio && result.deviation_level) {
    out.has_ratios = true;
    out.position_ratio = *result.position_ratio;
    out.yaw_ratio = *result.yaw_ratio;
    out.deviation_level = parking_vision::to_string(*result.deviation_level);
  }
}

// 관제 화면이 영상 위에 그릴 값(px)을 채운다. 등록 위치의 마커 꼭짓점, 이번 프레임의 Pinky 마커 꼭짓점,
// 고정 마커의 등록 중심점(anchor_centers_px = [x0, y0, x1, y1, ...]), 빼낸 시야 밀림(x, y), 기준 해상도다.
template <typename Out>
void fill_overlay(Out& out, const ParkingSpot& spot, const std::optional<Corners>& marker_corners,
                  const std::optional<cv::Point2d>& view_shift_xy) {
  out.has_view_shift_xy = view_shift_xy.has_value();
  if (view_shift_xy) {
    out.view_shift_x_px = view_shift_xy->x;
    out.view_shift_y_px = view_shift_xy->y;
  }
  out.image_width = static_cast<uint32_t>(spot.intrinsics.width);
  out.image_height = static_cast<uint32_t>(spot.intrinsics.height);
  copy_corners(spot.reference_corners_px, out.registered_corners_px);
  out.has_marker_corners = marker_corners.has_value();
  if (marker_corners) copy_corners(*marker_corners, out.marker_corners_px);
  out.anchor_centers_px.clear();
  for (const auto& [id, center] : spot.anchors) {
    out.anchor_centers_px.push_back(center.x);
    out.anchor_centers_px.push_back(center.y);
  }
}

}  // namespace

ParkingVisionNode::ParkingVisionNode(const rclcpp::NodeOptions& options) : rclcpp::Node("pinky_parking", options) {
  repository_root_ = declare_parameter<std::string>("repository_root", ".");
  live_spot_id_ = declare_parameter<std::string>("spot_id", "");
  const std::string image_topic = declare_parameter<std::string>("image_topic", "image_raw");
  const double track_rate_hz = declare_parameter<double>("track_rate_hz", 1.0 / parking_vision::kTrackIntervalS);
  // 8 Hz 미만이면 1초 정지 창에 5프레임을 못 채워 IN_POSITION이 영영 나오지 않으므로, 시작하지 않고 던진다.
  if (!std::isfinite(track_rate_hz) || track_rate_hz < kMinTrackRateHz || track_rate_hz > kMaxTrackRateHz) {
    char message[160];
    std::snprintf(message, sizeof(message), "track_rate_hz는 %g~%g Hz여야 합니다 (받은 값 %g): 정지 판정에 1초 창 안 %d프레임이 필요",
                  kMinTrackRateHz, kMaxTrackRateHz, track_rate_hz, parking_vision::kMinStillFrames);
    throw std::invalid_argument(message);
  }

  // 카메라 쪽 발행자가 SensorDataQoS(best-effort)라 구독도 같은 QoS로 맞춘다. 늦은 프레임은 버려도 된다.
  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& message) { on_image(message); });
  feed_camera_id_ = declare_parameter<std::string>("feed_camera_id", image_subscription_->get_topic_name());
  // 관제 같은 늦게 붙은 구독자도 마지막 상태를 바로 받는다(reliable, keep-last 1, transient_local).
  // "~/"는 노드 이름 아래라, 노드 이름이 pinky_parking이면 /pinky_parking/state가 된다.
  state_publisher_ = create_publisher<ParkingState>("~/state", rclcpp::QoS(1).reliable().transient_local());
  // 콜백은 작업 스레드만 띄우고 바로 돌아온다. 응답은 나중에 작업 스레드가 보낸다(deferred response).
  check_service_ = create_service<CheckParking>(
      "~/check", [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<CheckParking::Request> request) {
        on_check(header, request);
      });

  if (live_spot_id_.empty()) {
    RCLCPP_INFO(get_logger(), "spot_id 파라미터가 없어 실시간 상태(~/state)는 발행하지 않습니다");
  } else {
    reloader_ = std::make_unique<parking_vision::SpotReloader>(
        parking_vision::spot_file(repository_root_, live_spot_id_), repository_root_);
    // wall timer라 use_sim_time일 때도 실제 시간 주기로 돈다. 추적기의 시간 판단은 어차피 steady clock이다.
    tick_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / track_rate_hz), [this] { on_tick(); });
  }
  RCLCPP_INFO(get_logger(), "주차 판정 노드 시작: spot_id=%s image_topic=%s (관측 전용, 로봇 명령 없음)",
              live_spot_id_.c_str(), image_subscription_->get_topic_name());
}

ParkingVisionNode::~ParkingVisionNode() {
  // 프레임을 기다리는 판정 스레드를 깨운다. 깨어난 스레드는 shutting_down_을 보고 실패 응답을 보낸 뒤 끝난다.
  // 알림을 놓치더라도 kFrameWait(100 ms)마다 다시 확인하므로 join이 오래 걸리지 않는다.
  shutting_down_ = true;
  frames_arrived_.notify_all();
  std::lock_guard<std::mutex> lock(workers_mutex_);
  for (auto& worker : workers_) worker.thread.join();
}

double ParkingVisionNode::steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

cv_bridge::CvImageConstPtr ParkingVisionNode::to_bgr(const Frame& frame) {
  // 이미 bgr8이면 메시지 버퍼를 공유한다. 변환 비용은 실제로 판정하는 프레임만 낸다.
  return cv_bridge::toCvShare(frame.message, sensor_msgs::image_encodings::BGR8);
}

void ParkingVisionNode::on_image(const sensor_msgs::msg::Image::ConstSharedPtr& message) {
  const double arrived_steady = steady_seconds();
  const rclcpp::Time received = now();
  const rclcpp::Time stamp(message->header.stamp, received.get_clock_type());
  // 프레임은 stamp 시각에 찍힌 것으로 보되, 도착 시각보다 늦게 찍혔다고 보지는 않는다. stamp가 도착보다 앞서면
  // (드라이버 시계가 빠름) 뒤처짐을 0으로 보고, stamp가 0이면 드라이버가 stamp를 주지 않은 것으로 본다. stamp가
  // 도착보다 한참 뒤처지면 프레임은 오래된 것으로 남으므로, 늦게 온 프레임이나 재생된 프레임을 요청 이후에 찍힌
  // 것으로 착각하지 않는다.
  // 뒤처짐은 노드 시계로 재고 steady clock 도착 시각에서 뺀다. 그래서 두 시계가 달라도 비교는 steady clock끼리 한다.
  const double lag = stamp.nanoseconds() == 0 ? 0.0 : std::max(0.0, (received - stamp).seconds());
  {
    std::lock_guard<std::mutex> lock(frames_mutex_);
    frames_.push_back({++sequence_, arrived_steady - lag, received - rclcpp::Duration::from_seconds(lag), lag, stamp,
                       message->header.frame_id, message});
    while (frames_.size() > kBufferedFrames) frames_.pop_front();  // 오래된 프레임부터 버린다
  }
  frames_arrived_.notify_all();  // 프레임을 기다리는 판정 스레드를 모두 깨운다
}

std::vector<ParkingVisionNode::Frame> ParkingVisionNode::frames_after(std::uint64_t sequence) const {
  std::lock_guard<std::mutex> lock(frames_mutex_);
  std::vector<Frame> newer;
  for (const auto& frame : frames_) {
    if (frame.sequence > sequence) newer.push_back(frame);
  }
  return newer;
}

std::vector<ParkingVisionNode::Frame> ParkingVisionNode::wait_for_frames_after(std::uint64_t sequence,
                                                                                std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(frames_mutex_);
  // predicate가 있어 가짜 깨움(spurious wakeup)은 다시 잠든다. 시간이 다 되면 빈 결과로 돌아갈 수 있다.
  frames_arrived_.wait_for(lock, timeout, [&] { return sequence_ > sequence || shutting_down_; });
  std::vector<Frame> newer;
  for (const auto& frame : frames_) {
    if (frame.sequence > sequence) newer.push_back(frame);
  }
  return newer;
}

bool ParkingVisionNode::has_image_publisher() const {
  return image_subscription_->get_publisher_count() > 0;
}

void ParkingVisionNode::on_tick() {
  const double now_s = steady_seconds();
  try {
    if (!parking_vision::is_valid_spot_id(live_spot_id_)) {
      tracker_.on_spot_invalid(now_s, "spot_id must be 1-80 letters, digits, '_', '.' or '-'");
    } else if (reloader_->poll(now_s, tracker_)) {  // 5초에 한 번 파일 변경을 확인해 다시 읽는다. 못 쓰면 false
      const std::vector<Frame> newer = frames_after(tracked_sequence_);
      if (!newer.empty()) {
        const Frame& frame = newer.back();  // 화면 표시에는 가장 새 프레임만 의미가 있다
        tracked_sequence_ = frame.sequence;
        tracked_frame_ = TrackedFrame{frame.sequence, frame.stamp, frame.frame_id};
        try {
          tracker_.on_frame(now_s, frame.sequence, to_bgr(frame)->image);
        } catch (const cv_bridge::Exception& error) {  // 추적기 결함이 아니라 카메라 영상 형식 문제다
          tracker_.on_camera_unavailable(now_s, "이미지 변환 실패 (" + frame.message->encoding + "): " + error.what());
        }
      } else if (!has_image_publisher()) {
        tracker_.on_camera_unavailable(now_s, "카메라 feed가 없습니다: " + feed_camera_id_);
      } else {
        tracker_.on_no_frame(now_s);  // 카메라가 아직 켜지는 중일 수 있다(2초 넘게 없으면 CAMERA_UNAVAILABLE)
      }
    }
  } catch (const std::exception& error) {  // 추적은 계속한다. 나쁜 프레임 하나가 화면 표시를 멈춰서는 안 된다
    tracker_.on_tracking_error(now_s, std::string("추적 오류: ") + error.what());
  }
  publish_state(now());  // 상태가 바뀌지 않아도 매 틱 발행한다
}

void ParkingVisionNode::publish_state(const rclcpp::Time& now) {
  const parking_vision::LiveSnapshot& snapshot = tracker_.snapshot();
  const auto& spot = tracker_.spot();
  // 추적기는 steady clock 초를 들고 있고, 메시지에는 노드 시계 시각을 싣는다. 추적기가 아직 아무것도 발행하지
  // 않았을 때는 시각이 0인데, 이것이 1970-01-01로 나가서는 안 된다.
  // 지금 두 시계의 차이(to_node_clock)를 더해 steady clock 시각을 노드 시계 시각으로 옮긴다.
  const double steady_now = steady_seconds();
  const double to_node_clock = now.seconds() - steady_now;
  const double updated_s = snapshot.updated_s > 0.0 ? snapshot.updated_s : steady_now;
  const double since_s = snapshot.since_s > 0.0 ? snapshot.since_s : updated_s;
  ParkingState message;
  message.header.stamp = now;
  message.spot_id = live_spot_id_;
  message.state = parking_vision::to_string(snapshot.state);
  message.reason = snapshot.reason;
  message.since = to_time(since_s + to_node_clock, now.get_clock_type());
  message.updated_at = to_time(updated_s + to_node_clock, now.get_clock_type());
  message.feed_camera_id = feed_camera_id_;
  if (snapshot.frame_sequence) {
    message.has_frame = true;
    message.frame_sequence = *snapshot.frame_sequence;
    // 스냅숏이 마지막으로 넣은 프레임에서 나왔으면 header를 그 프레임의 stamp·frame_id로 바꾼다.
    if (tracked_frame_ && tracked_frame_->sequence == *snapshot.frame_sequence) {
      message.header.stamp = tracked_frame_->stamp;
      message.header.frame_id = tracked_frame_->frame_id;
    }
  }
  if (spot) {
    message.has_spot = true;
    if (snapshot.judged) {
      fill_judgment(message, *snapshot.judged, *spot);
    } else {
      fill_spot_fields(message, *spot, snapshot.view_shift_px);
    }
    fill_overlay(message, *spot, snapshot.marker_corners_px, snapshot.view_shift_xy_px);
  }
  // 실시간 상태는 화면 표시용이다. 로봇 제어(팔 단계 진행 조건)에 쓰지 않는다는 것을 메시지에 명시한다.
  message.display_only = true;
  message.usable_for_robot_control = false;
  state_publisher_->publish(message);
}

void ParkingVisionNode::send_response(rmw_request_id_t& header, CheckParking::Response& response) {
  try {
    check_service_->send_response(header, response);
  } catch (const std::exception& error) {  // 예: 노드가 종료되는 중. 이것 때문에 프로세스가 끝나서는 안 된다
    RCLCPP_ERROR(get_logger(), "주차 판정 응답을 보내지 못했습니다: %s", error.what());
  }
}

void ParkingVisionNode::on_check(const std::shared_ptr<rmw_request_id_t>& header,
                                 const std::shared_ptr<CheckParking::Request>& request) {
  // 작업 스레드에서 답한다(deferred response). 그래서 프레임을 모으는 동안에도 executor가 막히지 않고, 단일 스레드
  // 컨테이너에서도 영상 구독 콜백(on_image)이 계속 돌아 판정 스레드에 프레임이 들어온다.
  std::lock_guard<std::mutex> lock(workers_mutex_);
  for (auto worker = workers_.begin(); worker != workers_.end();) {  // 이미 답한 판정 스레드는 join해서 정리한다
    if (*worker->finished) {
      worker->thread.join();
      worker = workers_.erase(worker);
    } else {
      ++worker;
    }
  }
  if (workers_.size() >= kMaxConcurrentChecks) {  // 자리가 없으면 executor 스레드에서 바로 거절한다(관제 API의 429)
    CheckParking::Response busy = failed(base_response(request->spot_id), "PARKING_CHECK_BUSY",
                                         "주차 판정 요청이 이미 " + std::to_string(workers_.size()) + "개 진행 중입니다");
    send_response(*header, busy);
    return;
  }
  // header(요청 ID)는 shared_ptr로 붙잡아 두고 spot_id는 값으로 복사한다. 콜백이 돌아간 뒤에도 스레드가 쓴다.
  auto finished = std::make_shared<std::atomic<bool>>(false);
  workers_.push_back({std::thread([this, header, spot_id = request->spot_id, finished] {
                        CheckParking::Response response;
                        try {
                          response = check(spot_id);
                        } catch (const std::exception& error) {  // 작업 스레드 밖으로는 아무것도 나가면 안 된다
                          response = failed(base_response(spot_id), "PARKING_OBSERVATION_ERROR", error.what());
                        } catch (...) {
                          response = failed(base_response(spot_id), "PARKING_OBSERVATION_ERROR", "알 수 없는 오류");
                        }
                        // 자리를 먼저 비운다. 이 응답을 받자마자 다음 판정을 보내는 클라이언트가 자리가 아직
                        // 차 있다고 거절당하면 안 된다. 그러면 on_check는 join할 때 응답 전송만 기다리면 된다.
                        *finished = true;
                        send_response(*header, response);
                      }),
                      finished});
}

ParkingVisionNode::CheckParking::Response ParkingVisionNode::base_response(const std::string& spot_id) const {
  CheckParking::Response response;
  response.observation_schema_version = parking_vision::kObservationSchemaVersion;
  response.spot_id = spot_id;
  response.feed_camera_id = feed_camera_id_;
  response.opencv_version = CV_VERSION;
  response.usable_for_robot_control = false;
  return response;
}

ParkingVisionNode::CheckParking::Response ParkingVisionNode::failed(CheckParking::Response response,
                                                                    const std::string& error_code,
                                                                    const std::string& message) const {
  response.ok = false;
  response.error_code = error_code;
  response.message = message;
  RCLCPP_WARN(get_logger(), "주차 판정 실패 %s: %s", error_code.c_str(), message.c_str());
  return response;
}

ParkingVisionNode::CheckParking::Response ParkingVisionNode::check(const std::string& spot_id) {
  CheckParking::Response response = base_response(spot_id);
  const auto fail = [&](const std::string& error_code, const std::string& message) {
    return failed(response, error_code, message);
  };

  if (!parking_vision::is_valid_spot_id(spot_id)) {
    return fail("PARKING_SPOT_ID_INVALID", "spot_id must be 1-80 letters, digits, '_', '.' or '-'");
  }
  const std::filesystem::path path = parking_vision::spot_file(repository_root_, spot_id);
  std::error_code filesystem_error;  // 예외를 던지지 않는 overload다. 읽을 수 없는 디렉터리는 죽지 않고 "없음"이 된다
  if (!std::filesystem::is_regular_file(path, filesystem_error)) {
    return fail("PARKING_SPOT_NOT_FOUND", "등록된 주차 위치가 없습니다: " + spot_id);
  }
  // 주차 위치는 요청마다 읽는다. SHA-256은 파싱한 바로 그 바이트로 계산하므로, 응답이 판정에 쓴 등록을 정확히
  // 가리킨다. 등록 뒤 내부 보정 파일이 바뀌었으면 load_spot이 던져 PARKING_SPOT_INVALID가 된다.
  ParkingSpot spot;
  try {
    spot = parking_vision::load_spot(path, repository_root_, &response.spot_registration_sha256);
  } catch (const std::exception& error) {
    return fail("PARKING_SPOT_INVALID", std::string("주차 위치 파일을 쓸 수 없습니다: ") + error.what());
  }
  response.spot_id = spot.spot_id;  // 파일에 기록된 ID. 관제 JSON도 이 값을 돌려준다
  response.calibration_camera_id = spot.camera_id;
  response.intrinsic_calibration_sha256 = spot.intrinsics.sha256;
  response.registration_opencv_version = spot.registration_opencv_version.value_or("");
  if (!has_image_publisher()) return fail("PARKING_CAMERA_UNAVAILABLE", "카메라 feed가 없습니다: " + feed_camera_id_);

  // 응답에는 노드 시계 시각을 싣고, "요청 이후" 비교와 수집 마감(3초)은 steady clock으로 한다.
  response.requested_at = now();
  const double requested = steady_seconds();
  parking_vision::ParkingObservationCollector collector(spot, requested);
  // 0부터 시작하므로 버퍼에 남은 최근 프레임도 한 번은 보지만, 요청 전에 찍힌 것은 아래에서 걸러진다.
  std::uint64_t last_sequence = 0;
  rclcpp::Time captured_from;
  rclcpp::Time captured_until;
  int captured_before_request = 0;  // 요청 뒤에 도착했지만 요청 전에 찍힌 프레임 수(오류 문구용)
  double largest_lag_s = 0.0;       // 그 프레임들의 stamp가 가장 많이 뒤처진 정도(초)
  try {
    // 15프레임을 채우거나 3초가 지나면 끝난다. 프레임이 없으면 kFrameWait마다 깨어나 마감과 종료를 확인한다.
    while (!collector.done(steady_seconds())) {
      if (shutting_down_) return fail("PARKING_CAMERA_UNAVAILABLE", "노드가 종료되는 중입니다");
      for (const Frame& frame : wait_for_frames_after(last_sequence, kFrameWait)) {
        last_sequence = frame.sequence;
        if (frame.captured_steady_s < requested) {  // 요청 전에 찍힌 프레임은 쓰지 않는다
          ++captured_before_request;
          largest_lag_s = std::max(largest_lag_s, frame.stamp_lag_s);
          continue;
        }
        cv_bridge::CvImageConstPtr image;
        try {
          image = to_bgr(frame);
        } catch (const cv_bridge::Exception& error) {
          return fail("PARKING_CAMERA_UNAVAILABLE",
                      "이미지 변환 실패 (" + frame.message->encoding + "): " + error.what());
        }
        // add는 해상도가 보정값과 다르면 ParkingObservationError(PARKING_CAMERA_UNAVAILABLE)를 던진다.
        if (collector.add(frame.sequence, frame.captured_steady_s, image->image)) {
          if (collector.frames_examined() == 1) captured_from = frame.captured;
          captured_until = frame.captured;
        }
        if (collector.done(steady_seconds())) break;
      }
    }
    if (collector.frames_examined() == 0 && captured_before_request > 0) {
      // 프레임은 들어왔지만 모두 요청 전 stamp였다. "새 프레임 없음" 대신 그렇다고 알린다.
      // use_sim_time 없이 rosbag을 재생하면 녹화된 stamp가 모두 과거라 이렇게 된다.
      char message[256];
      std::snprintf(message, sizeof(message),
                    "%d프레임이 들어왔지만 모두 요청 전에 찍힌 것이라 쓰지 않았습니다 (stamp가 최대 %.1f초 뒤처짐). "
                    "rosbag 재생이면 use_sim_time:=true로 실행하세요",
                    captured_before_request, largest_lag_s);
      return fail("PARKING_CAMERA_UNAVAILABLE", std::string(message) + ": " + feed_camera_id_);
    }
    // finish는 본 프레임이 하나도 없으면 던지고, 쓸 수 있는 프레임이 5개 미만이면 NOT_FOUND로 바꾼다.
    const parking_vision::ParkingObservation observation = collector.finish();
    response.ok = true;  // 판정을 마쳤다는 뜻이다. 도착 여부는 status로 본다
    response.marker_id = spot.marker_id;
    response.status = parking_vision::to_string(observation.result.status);
    response.reason = observation.result.reason;
    fill_judgment(response, observation.result, spot);
    fill_overlay(response, spot, observation.marker_corners_px, observation.view_shift_xy_px);
    response.frames_used = observation.frames_used;
    response.frames_examined = observation.frames_examined;
    response.first_frame_sequence = observation.first_sequence;
    response.last_frame_sequence = observation.last_sequence;
    response.captured_from = captured_from;
    response.captured_until = captured_until;
    RCLCPP_INFO(get_logger(), "주차 판정 %s (%d/%d 프레임): %s", response.status.c_str(), observation.frames_used,
                observation.frames_examined, observation.result.reason.c_str());
    return response;
  } catch (const parking_vision::ParkingObservationError& error) {
    return fail(error.error_code(), std::string(error.what()) + ": " + feed_camera_id_);
  } catch (const std::exception& error) {  // 예: 형식이 잘못된 프레임에서 난 OpenCV 오류
    return fail("PARKING_OBSERVATION_ERROR", error.what());
  }
}

}  // namespace parking_vision_ros

// component_container가 이 클래스를 plugin 이름으로 찾아 올릴 수 있게 등록한다.
RCLCPP_COMPONENTS_REGISTER_NODE(parking_vision_ros::ParkingVisionNode)
