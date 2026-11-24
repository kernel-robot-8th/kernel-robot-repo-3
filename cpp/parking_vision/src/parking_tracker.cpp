// ParkingTracker(화면 표시용 상태 기계)와 SpotReloader(주차 위치 파일 다시 읽기) 구현.
//
// 위치: parking_vision 라이브러리. ROS 노드의 10 Hz 타이머가 틱마다 on_* 하나와 SpotReloader::poll을 부른다.
#include "parking_vision/parking_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <system_error>
#include <vector>

namespace parking_vision {
namespace {

// 각 오프셋에서 창 안 중앙값을 뺀 것. 멈춘 Pinky와 움직이는 Pinky를 구분하는 데 쓴다.
std::vector<PlanarOffset> spread(const std::vector<PlanarOffset>& offsets) {
  const PlanarOffset center = median_offset(offsets);
  std::vector<PlanarOffset> result;
  for (const auto& o : offsets) {
    result.push_back({o.dx_mm - center.dx_mm, o.dy_mm - center.dy_mm, wrap_degrees(o.dyaw_deg - center.dyaw_deg)});
  }
  return result;
}

LiveSnapshot state_of(LiveState state, std::string reason) {
  LiveSnapshot snapshot;
  snapshot.state = state;
  snapshot.reason = std::move(reason);
  return snapshot;
}

}  // namespace

std::string to_string(LiveState state) {
  switch (state) {
    case LiveState::NotInView: return "NOT_IN_VIEW";
    case LiveState::Moving: return "MOVING";
    case LiveState::InPosition: return "IN_POSITION";
    case LiveState::OffPosition: return "OFF_POSITION";
    case LiveState::Ambiguous: return "AMBIGUOUS";
    case LiveState::ViewUnverified: return "VIEW_UNVERIFIED";
    case LiveState::CameraUnavailable: return "CAMERA_UNAVAILABLE";
    case LiveState::SpotInvalid: return "SPOT_INVALID";
    case LiveState::TrackingError: return "TRACKING_ERROR";
  }
  return "UNKNOWN";
}

ParkingTracker::ParkingTracker() { snapshot_.reason = "아직 프레임을 받지 않았습니다"; }

void ParkingTracker::set_spot(std::shared_ptr<const ParkingSpot> spot) {
  spot_ = std::move(spot);
  reset_samples();
}

void ParkingTracker::reset_samples() {
  samples_.clear();
  last_seen_.reset();
  marker_corners_.reset();
  view_shift_xy_.reset();
}

void ParkingTracker::start(double now_s) {
  if (!started_at_) started_at_ = now_s;
}

const LiveSnapshot& ParkingTracker::publish(double now_s, LiveSnapshot next) {
  next.updated_s = now_s;
  // 처음 발행하는 상태는 자기 `since`를 새로 시작한다. `since`가 없던 Python의 초기 snapshot과 같다.
  next.since_s = published_ && next.state == snapshot_.state ? snapshot_.since_s : now_s;
  next.marker_corners_px = marker_corners_;
  next.view_shift_xy_px = view_shift_xy_;
  published_ = true;
  snapshot_ = std::move(next);
  return snapshot_;
}

const LiveSnapshot& ParkingTracker::on_camera_unavailable(double now_s, const std::string& reason) {
  start(now_s);
  return publish(now_s, state_of(LiveState::CameraUnavailable, reason));
}

const LiveSnapshot& ParkingTracker::on_spot_invalid(double now_s, const std::string& reason) {
  start(now_s);
  spot_.reset();
  reset_samples();
  return publish(now_s, state_of(LiveState::SpotInvalid, reason));
}

const LiveSnapshot& ParkingTracker::on_tracking_error(double now_s, const std::string& reason) {
  start(now_s);
  reset_samples();
  return publish(now_s, state_of(LiveState::TrackingError, reason));
}

const LiveSnapshot& ParkingTracker::on_no_frame(double now_s) {
  start(now_s);
  // 마지막 프레임(없으면 시작 시각)부터 2초가 지나야 CAMERA_UNAVAILABLE로 바꾼다. 그 전에는 지금 상태를 유지한다.
  if (now_s - last_frame_at_.value_or(*started_at_) > kStaleFeedS) {
    return publish(now_s, state_of(LiveState::CameraUnavailable, "새 프레임이 들어오지 않습니다"));
  }
  return snapshot_;
}

const LiveSnapshot& ParkingTracker::on_frame(double now_s, std::uint64_t sequence, const cv::Mat& image) {
  start(now_s);
  if (!spot_) return on_spot_invalid(now_s, "주차 위치가 설정되지 않았습니다");
  const ParkingSpot& spot = *spot_;
  last_frame_at_ = now_s;
  if (image.cols != spot.intrinsics.width || image.rows != spot.intrinsics.height) {
    return publish(now_s, state_of(LiveState::CameraUnavailable, "프레임 해상도 " + std::to_string(image.cols) + "x" +
                                                                      std::to_string(image.rows) +
                                                                      "가 보정 해상도와 다릅니다"));
  }
  const FrameObservation observation = observe_frame(image, spot);

  LiveSnapshot next;
  next.frame_sequence = sequence;
  if (observation.blocking_status) {
    // 이전 프레임은 더 이상 지금 시야를 나타내지 않으므로, 그것으로 판정하거나 그려서는 안 된다.
    reset_samples();
    next.state = *observation.blocking_status == ParkingStatus::Ambiguous ? LiveState::Ambiguous
                                                                          : LiveState::ViewUnverified;
    next.reason = observation.reason;
    next.view_shift_px = observation.view_shift_px;
    return publish(now_s, std::move(next));
  }
  if (observation.offset) {  // 판정에 쓴 프레임만 화면 그림에 반영한다
    last_seen_ = now_s;
    samples_.emplace_back(now_s, *observation.offset);
    marker_corners_ = observation.corners;
    view_shift_xy_ = observation.view_shift_xy_px;
  }
  // 1초보다 오래된 샘플을 버린다. 남은 것이 없거나 1초 넘게 마커를 못 봤으면 화면에 없는 것으로 본다.
  // 이번 프레임이 고정 마커가 안 보여 건너뛴 것이면 NOT_IN_VIEW 대신 VIEW_UNVERIFIED로 보인다.
  while (!samples_.empty() && now_s - samples_.front().first > kStillWindowS) samples_.pop_front();
  if (samples_.empty() || !last_seen_ || now_s - *last_seen_ > kLostAfterS) {
    reset_samples();
    next.state = observation.skipped_status == ParkingStatus::ViewUnverified ? LiveState::ViewUnverified
                                                                             : LiveState::NotInView;
    next.reason = observation.reason;
    return publish(now_s, std::move(next));
  }

  std::vector<PlanarOffset> offsets;
  for (const auto& sample : samples_) offsets.push_back(sample.second);
  // 정지 조건: 프레임 5개 이상, 창의 0.8배(0.8초) 이상을 덮음, 모든 프레임이 중앙값에서 5 mm·2° 안.
  // 10 Hz 틱이라 창의 가장 오래된 샘플이 정확히 1초 전이 아니고 프레임이 빠질 수도 있어서 0.8배까지 허용한다.
  bool still = static_cast<int>(offsets.size()) >= kMinStillFrames &&
               now_s - samples_.front().first >= 0.8 * kStillWindowS;
  if (still) {
    for (const auto& o : spread(offsets)) {
      still = still && o.distance_mm() <= kStillPositionMm && std::abs(o.dyaw_deg) <= kStillYawDeg;
    }
  }
  // 멈췄으면 1초 중앙값으로, 움직이는 중이면 가장 최근 프레임으로 판정한다(움직이는 중의 판정은 표시용이다).
  ParkingResult result = judge(still ? median_offset(offsets) : offsets.back(), spot.tolerance);
  if (still) {
    next.state = result.status == ParkingStatus::Arrived ? LiveState::InPosition : LiveState::OffPosition;
    next.reason = "1초 이상 멈춰 있음";
  } else {
    next.state = LiveState::Moving;
    next.reason = "움직이는 중";
  }
  result.view_shift_px = observation.view_shift_px;
  next.view_shift_px = observation.view_shift_px;
  next.judged = std::move(result);
  return publish(now_s, std::move(next));
}

SpotReloader::SpotReloader(std::filesystem::path spot_path, std::filesystem::path repository_root)
    : path_(std::move(spot_path)), root_(std::move(repository_root)) {}

SpotReloader::Mtimes SpotReloader::mtimes(const ParkingSpot* spot) const {
  return {std::filesystem::last_write_time(path_),
          spot ? std::optional(std::filesystem::last_write_time(spot->intrinsics.path)) : std::nullopt};
}

bool SpotReloader::poll(double now_s, ParkingTracker& tracker) {
  // 쓸 수 없는 상태에서도 간격을 지킨다. 그래야 잘못된 파일을 틱마다 다시 읽지 않는다.
  if (now_s - checked_s_ < kSpotReloadS) return loaded_;
  checked_s_ = now_s;
  try {
    // 다시 등록하거나, 허용 오차를 고치거나, 카메라를 다시 보정하면 모두 주차 위치를 다시 읽는다. 보정이 바뀐
    // 경우에는 일회성 판정과 마찬가지로 load_spot의 SHA-256 확인에서 실패한다.
    if (!loaded_ || mtimes(tracker.spot().get()) != mtimes_) {
      auto spot = std::make_shared<const ParkingSpot>(load_spot(path_, root_));
      mtimes_ = mtimes(spot.get());
      tracker.set_spot(std::move(spot));
      loaded_ = true;
    }
  } catch (const std::exception& error) {  // ParkingSpotError, JSON 오류, 파일 없음
    loaded_ = false;
    mtimes_.reset();
    tracker.on_spot_invalid(now_s, std::string("주차 위치 파일을 쓸 수 없습니다: ") + error.what());
    return false;
  }
  return true;
}

}  // namespace parking_vision
