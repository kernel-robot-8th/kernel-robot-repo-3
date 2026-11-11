// Continuous, display-only Pinky parking state from the ceiling camera feed.
//
// C++ port of the state machine in the former src/control_station/parking_tracker.py: NOT_IN_VIEW -> MOVING ->
// IN_POSITION / OFF_POSITION. It never gates the arm step; ParkingObservationCollector judges only frames captured
// after an arrival result, so a frame where a moving Pinky briefly passes through the tolerance cannot count as an
// arrival. The caller owns the thread, the clock and the feed, and calls one on_* method per tick.
#pragma once

#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <opencv2/core.hpp>

#include "parking_vision/parking_check.hpp"

namespace parking_vision {

inline constexpr double kTrackIntervalS = 0.1;  // judge at most 10 frames per second; the feed runs near 30 fps
inline constexpr double kStillWindowS = 1.0;    // Pinky counts as stopped when every frame of the last second agrees
// Single raw frames of a parked Pinky scatter about ±1.5 mm and ±0.55° (2026-11-02, OpenCV 4.14);
// a Pinky that is actually driving moves tens of mm within the window.
inline constexpr double kStillPositionMm = 5.0;
inline constexpr double kStillYawDeg = 2.0;
inline constexpr int kMinStillFrames = 5;
inline constexpr double kLostAfterS = 1.0;  // marker unseen this long -> NOT_IN_VIEW
inline constexpr double kStaleFeedS = 2.0;  // no new frame this long -> CAMERA_UNAVAILABLE
inline constexpr double kSpotReloadS = 5.0;

enum class LiveState {
  NotInView,
  Moving,
  InPosition,
  OffPosition,
  Ambiguous,
  ViewUnverified,
  CameraUnavailable,
  SpotInvalid,
  TrackingError,  // a software fault in the tracker, not a camera or registration problem
};

// Same strings as the Python enum ("NOT_IN_VIEW", ...).
std::string to_string(LiveState state);

struct LiveSnapshot {
  LiveState state = LiveState::NotInView;
  std::string reason;
  std::optional<ParkingResult> judged;  // MOVING / IN_POSITION / OFF_POSITION only
  std::optional<double> view_shift_px;  // judged frames carry it in `judged` as well
  std::optional<std::uint64_t> frame_sequence;
  std::optional<Corners> marker_corners_px;  // Pinky marker of the last judged frame, cleared once it is lost
  std::optional<cv::Point2d> view_shift_xy_px;
  double updated_s = 0.0;
  double since_s = 0.0;  // when the current state began
};

class ParkingTracker {
 public:
  ParkingTracker();

  // A new or re-registered spot; earlier samples were measured against another reference and are dropped.
  void set_spot(std::shared_ptr<const ParkingSpot> spot);
  const std::shared_ptr<const ParkingSpot>& spot() const { return spot_; }

  // The newest frame of this tick (only the newest frame matters for display).
  const LiveSnapshot& on_frame(double now_s, std::uint64_t sequence, const cv::Mat& image);
  // The feed is open but had no new frame this tick; it may still be opening right after start-up.
  const LiveSnapshot& on_no_frame(double now_s);
  // The feed itself is missing or failed.
  const LiveSnapshot& on_camera_unavailable(double now_s, const std::string& reason);
  // The spot could not be loaded; tracking pauses until set_spot.
  const LiveSnapshot& on_spot_invalid(double now_s, const std::string& reason);
  // The caller caught an unexpected error while tracking.
  const LiveSnapshot& on_tracking_error(double now_s, const std::string& reason);

  const LiveSnapshot& snapshot() const { return snapshot_; }

 private:
  void start(double now_s);
  void reset_samples();
  const LiveSnapshot& publish(double now_s, LiveSnapshot next);

  std::shared_ptr<const ParkingSpot> spot_;
  std::optional<double> started_at_;
  std::optional<double> last_frame_at_;
  std::optional<double> last_seen_;
  std::deque<std::pair<double, PlanarOffset>> samples_;
  std::optional<Corners> marker_corners_;
  std::optional<cv::Point2d> view_shift_xy_;
  bool published_ = false;
  LiveSnapshot snapshot_;
};

// Re-reads a spot file when it or its intrinsic calibration changes, checking at most every kSpotReloadS.
// Mirrors ParkingTracker._reload_spot of the former Python tracker.
class SpotReloader {
 public:
  SpotReloader(std::filesystem::path spot_path, std::filesystem::path repository_root);

  // Feeds the tracker on change. Returns false when the spot is unusable (the tracker then shows SPOT_INVALID).
  bool poll(double now_s, ParkingTracker& tracker);

 private:
  std::filesystem::path path_;
  std::filesystem::path root_;
  using Mtimes = std::pair<std::filesystem::file_time_type, std::optional<std::filesystem::file_time_type>>;
  Mtimes mtimes(const ParkingSpot* spot) const;

  std::optional<Mtimes> mtimes_;
  double checked_s_ = -kSpotReloadS;
  bool loaded_ = false;
};

}  // namespace parking_vision
