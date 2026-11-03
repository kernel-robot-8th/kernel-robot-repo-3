#include "parking_vision_ros/parking_vision_node.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/version.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "parking_vision/parking_observation.hpp"

namespace parking_vision_ros {
namespace {

using parking_vision::Corners;
using parking_vision::ParkingResult;
using parking_vision::ParkingSpot;

// The check loop polls every 10 ms and the tracker takes only the newest frame, so a short buffer is enough.
constexpr std::size_t kBufferedFrames = 10;
constexpr auto kPollInterval = std::chrono::milliseconds(10);

rclcpp::Time to_time(double seconds, rcl_clock_type_t clock_type) {
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), clock_type);
}

void copy_corners(const Corners& corners, std::array<double, 8>& out) {
  for (std::size_t i = 0; i < corners.size(); ++i) {
    out[2 * i] = corners[i].x;
    out[2 * i + 1] = corners[i].y;
  }
}

// ParkingState and the CheckParking response share these field names, as the JSON payloads share
// judgment_fields() and image_overlay() in the control station.
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

  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& message) { on_image(message); });
  feed_camera_id_ = declare_parameter<std::string>("feed_camera_id", image_subscription_->get_topic_name());
  // Late subscribers such as the control station get the last state at once.
  state_publisher_ = create_publisher<ParkingState>("~/state", rclcpp::QoS(1).reliable().transient_local());
  check_service_ = create_service<CheckParking>(
      "~/check", [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<CheckParking::Request> request) {
        on_check(header, request);
      });

  if (live_spot_id_.empty()) {
    RCLCPP_INFO(get_logger(), "spot_id 파라미터가 없어 실시간 상태(~/state)는 발행하지 않습니다");
  } else {
    reloader_ = std::make_unique<parking_vision::SpotReloader>(
        parking_vision::spot_file(repository_root_, live_spot_id_), repository_root_);
    tick_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / track_rate_hz), [this] { on_tick(); });
  }
  RCLCPP_INFO(get_logger(), "주차 판정 노드 시작: spot_id=%s image_topic=%s (관측 전용, 로봇 명령 없음)",
              live_spot_id_.c_str(), image_subscription_->get_topic_name());
}

ParkingVisionNode::~ParkingVisionNode() {
  shutting_down_ = true;
  std::lock_guard<std::mutex> lock(workers_mutex_);
  for (auto& worker : workers_) worker.thread.join();
}

void ParkingVisionNode::on_image(const sensor_msgs::msg::Image::ConstSharedPtr& message) {
  const rclcpp::Time received = now();
  cv_bridge::CvImageConstPtr converted;
  try {
    converted = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception& error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "이미지 변환 실패 (%s): %s", message->encoding.c_str(),
                         error.what());
    return;
  }
  const rclcpp::Time stamp(message->header.stamp, received.get_clock_type());
  // A frame counts as captured at its stamp, but never later than it arrived: a driver clock running ahead must
  // not let a frame captured before a request look newer. Drivers on another host need a synchronised clock.
  const rclcpp::Time captured = stamp.nanoseconds() == 0 ? received : std::min(stamp, received);
  std::lock_guard<std::mutex> lock(frames_mutex_);
  frames_.push_back({++sequence_, captured, stamp, message->header.frame_id, converted->image});
  while (frames_.size() > kBufferedFrames) frames_.pop_front();
}

std::vector<ParkingVisionNode::Frame> ParkingVisionNode::frames_after(std::uint64_t sequence) const {
  std::lock_guard<std::mutex> lock(frames_mutex_);
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
  const rclcpp::Time now = this->now();
  const double now_s = now.seconds();
  try {
    if (!parking_vision::is_valid_spot_id(live_spot_id_)) {
      tracker_.on_spot_invalid(now_s, "spot_id must be 1-80 letters, digits, '_', '.' or '-'");
    } else if (reloader_->poll(now_s, tracker_)) {
      const std::vector<Frame> newer = frames_after(tracked_sequence_);
      if (!newer.empty()) {
        const Frame& frame = newer.back();  // only the newest frame matters for display
        tracked_sequence_ = frame.sequence;
        tracked_frame_ = frame;
        tracker_.on_frame(now_s, frame.sequence, frame.image);
      } else if (!has_image_publisher()) {
        tracker_.on_camera_unavailable(now_s, "카메라 feed가 없습니다: " + feed_camera_id_);
      } else {
        tracker_.on_no_frame(now_s);  // the camera may still be starting
      }
    }
  } catch (const std::exception& error) {  // keep tracking; one bad frame must not stop the display
    tracker_.on_tracking_error(now_s, std::string("추적 오류: ") + error.what());
  }
  publish_state(now);
}

void ParkingVisionNode::publish_state(const rclcpp::Time& now) {
  const parking_vision::LiveSnapshot& snapshot = tracker_.snapshot();
  const auto& spot = tracker_.spot();
  ParkingState message;
  message.header.stamp = now;
  message.spot_id = live_spot_id_;
  message.state = parking_vision::to_string(snapshot.state);
  message.reason = snapshot.reason;
  message.since = to_time(snapshot.since_s, now.get_clock_type());
  message.updated_at = to_time(snapshot.updated_s, now.get_clock_type());
  message.feed_camera_id = feed_camera_id_;
  if (snapshot.frame_sequence) {
    message.has_frame = true;
    message.frame_sequence = *snapshot.frame_sequence;
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
  message.display_only = true;
  message.usable_for_robot_control = false;
  state_publisher_->publish(message);
}

void ParkingVisionNode::on_check(const std::shared_ptr<rmw_request_id_t>& header,
                                 const std::shared_ptr<CheckParking::Request>& request) {
  // Answered from a worker thread (deferred response) so collecting frames never blocks the executor.
  std::lock_guard<std::mutex> lock(workers_mutex_);
  for (auto worker = workers_.begin(); worker != workers_.end();) {  // join checks that already answered
    if (*worker->finished) {
      worker->thread.join();
      worker = workers_.erase(worker);
    } else {
      ++worker;
    }
  }
  auto finished = std::make_shared<std::atomic<bool>>(false);
  workers_.push_back({std::thread([this, header, spot_id = request->spot_id, finished] {
                        CheckParking::Response response = check(spot_id);
                        check_service_->send_response(*header, response);
                        *finished = true;
                      }),
                      finished});
}

ParkingVisionNode::CheckParking::Response ParkingVisionNode::check(const std::string& spot_id) {
  CheckParking::Response response;
  response.observation_schema_version = parking_vision::kObservationSchemaVersion;
  response.spot_id = spot_id;
  response.feed_camera_id = feed_camera_id_;
  response.opencv_version = CV_VERSION;
  response.usable_for_robot_control = false;
  const auto fail = [&](const std::string& error_code, const std::string& message) {
    response.ok = false;
    response.error_code = error_code;
    response.message = message;
    RCLCPP_WARN(get_logger(), "주차 판정 실패 %s: %s", error_code.c_str(), message.c_str());
    return response;
  };

  if (!parking_vision::is_valid_spot_id(spot_id)) {
    return fail("PARKING_SPOT_ID_INVALID", "spot_id must be 1-80 letters, digits, '_', '.' or '-'");
  }
  const std::filesystem::path path = parking_vision::spot_file(repository_root_, spot_id);
  if (!std::filesystem::is_regular_file(path)) {
    return fail("PARKING_SPOT_NOT_FOUND", "등록된 주차 위치가 없습니다: " + spot_id);
  }
  // The spot is read per request, so the response names the exact files it used.
  ParkingSpot spot;
  try {
    response.spot_registration_sha256 = parking_vision::sha256_of(path);
    spot = parking_vision::load_spot(path, repository_root_);
  } catch (const std::exception& error) {
    return fail("PARKING_SPOT_INVALID", std::string("주차 위치 파일을 쓸 수 없습니다: ") + error.what());
  }
  response.calibration_camera_id = spot.camera_id;
  response.intrinsic_calibration_sha256 = spot.intrinsics.sha256;
  response.registration_opencv_version = spot.registration_opencv_version.value_or("");
  if (!has_image_publisher()) return fail("PARKING_CAMERA_UNAVAILABLE", "카메라 feed가 없습니다: " + feed_camera_id_);

  const rclcpp::Time requested = now();
  response.requested_at = requested;
  parking_vision::ParkingObservationCollector collector(spot, requested.seconds());
  std::uint64_t last_sequence = 0;
  try {
    while (!collector.done(now().seconds())) {
      if (shutting_down_) return fail("PARKING_CAMERA_UNAVAILABLE", "노드가 종료되는 중입니다");
      for (const Frame& frame : frames_after(last_sequence)) {
        last_sequence = frame.sequence;
        if (collector.add(frame.sequence, frame.captured.seconds(), frame.image) && collector.done(now().seconds())) {
          break;
        }
      }
      std::this_thread::sleep_for(kPollInterval);
    }
    const parking_vision::ParkingObservation observation = collector.finish();
    response.ok = true;
    response.marker_id = spot.marker_id;
    response.status = parking_vision::to_string(observation.result.status);
    response.reason = observation.result.reason;
    fill_judgment(response, observation.result, spot);
    fill_overlay(response, spot, observation.marker_corners_px, observation.view_shift_xy_px);
    response.frames_used = observation.frames_used;
    response.frames_examined = observation.frames_examined;
    response.first_frame_sequence = observation.first_sequence;
    response.last_frame_sequence = observation.last_sequence;
    response.captured_from = to_time(observation.captured_from_s, requested.get_clock_type());
    response.captured_until = to_time(observation.captured_until_s, requested.get_clock_type());
    RCLCPP_INFO(get_logger(), "주차 판정 %s (%d/%d 프레임): %s", response.status.c_str(), observation.frames_used,
                observation.frames_examined, observation.result.reason.c_str());
    return response;
  } catch (const parking_vision::ParkingObservationError& error) {
    return fail(error.error_code(), std::string(error.what()) + ": " + feed_camera_id_);
  } catch (const std::exception& error) {  // e.g. an OpenCV error on a malformed frame
    return fail("PARKING_OBSERVATION_ERROR", error.what());
  }
}

}  // namespace parking_vision_ros

RCLCPP_COMPONENTS_REGISTER_NODE(parking_vision_ros::ParkingVisionNode)
