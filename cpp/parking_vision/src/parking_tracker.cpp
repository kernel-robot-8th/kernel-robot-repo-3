#include "parking_vision/parking_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <system_error>
#include <vector>

namespace parking_vision {
namespace {

// Each offset relative to the window median, to tell a stopped Pinky from a moving one.
std::vector<PlanarOffset> spread(const std::vector<PlanarOffset>& offsets) {
  const PlanarOffset center = median_offset(offsets);
  std::vector<PlanarOffset> result;
  for (const auto& o : offsets) {
    const double dyaw = o.dyaw_deg - center.dyaw_deg + 180.0;
    result.push_back({o.dx_mm - center.dx_mm, o.dy_mm - center.dy_mm, dyaw - 360.0 * std::floor(dyaw / 360.0) - 180.0});
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
  // The first published state starts its own `since`, like the initial Python snapshot without one.
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
    // Earlier frames no longer describe the current view, so nothing from them may be judged or drawn.
    reset_samples();
    next.state = *observation.blocking_status == ParkingStatus::Ambiguous ? LiveState::Ambiguous
                                                                          : LiveState::ViewUnverified;
    next.reason = observation.reason;
    next.view_shift_px = observation.view_shift_px;
    return publish(now_s, std::move(next));
  }
  if (observation.offset) {  // only judged frames feed the picture
    last_seen_ = now_s;
    samples_.emplace_back(now_s, *observation.offset);
    marker_corners_ = observation.corners;
    view_shift_xy_ = observation.view_shift_xy_px;
  }
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
  bool still = static_cast<int>(offsets.size()) >= kMinStillFrames &&
               now_s - samples_.front().first >= 0.8 * kStillWindowS;
  if (still) {
    for (const auto& o : spread(offsets)) {
      still = still && o.distance_mm() <= kStillPositionMm && std::abs(o.dyaw_deg) <= kStillYawDeg;
    }
  }
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
  // Also while invalid, so a bad file is not re-read every tick.
  if (now_s - checked_s_ < kSpotReloadS) return loaded_;
  checked_s_ = now_s;
  try {
    // A re-registration, an edited tolerance or a recalibrated camera all reload the spot. A changed calibration
    // then fails the SHA-256 check in load_spot, as the one-shot check does.
    if (!loaded_ || mtimes(tracker.spot().get()) != mtimes_) {
      auto spot = std::make_shared<const ParkingSpot>(load_spot(path_, root_));
      mtimes_ = mtimes(spot.get());
      tracker.set_spot(std::move(spot));
      loaded_ = true;
    }
  } catch (const std::exception& error) {  // ParkingSpotError, JSON errors, missing file
    loaded_ = false;
    mtimes_.reset();
    tracker.on_spot_invalid(now_s, std::string("주차 위치 파일을 쓸 수 없습니다: ") + error.what());
    return false;
  }
  return true;
}

}  // namespace parking_vision
