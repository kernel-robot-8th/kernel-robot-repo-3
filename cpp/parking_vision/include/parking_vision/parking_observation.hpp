// One-shot parking observation over frames captured after a request.
//
// C++ port of the judging part of the former src/control_station/parking_observation.py: only frames captured after the
// request are judged, at most kObservationMaxFrames usable ones within the collection window, and fewer than
// kObservationMinFrames usable frames are not judged. The result is an observation, never a robot command.
// Spot lookup, the feed itself and the JSON contract stay with the caller; time is passed in by the caller.
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "parking_vision/parking_check.hpp"

namespace parking_vision {

inline constexpr int kObservationMaxFrames = 15;
inline constexpr int kObservationMinFrames = 5;
inline constexpr double kObservationCollectSeconds = 3.0;
inline constexpr int kObservationSchemaVersion = 1;
// Registered spots live at <repository root>/.control-station/parking-spots/<spot_id>.json.
inline constexpr const char* kSpotDir = ".control-station/parking-spots";

// 1-80 letters, digits, '_', '.' or '-', starting with a letter or digit (no path separators).
bool is_valid_spot_id(const std::string& spot_id);
std::filesystem::path spot_file(const std::filesystem::path& repository_root, const std::string& spot_id);

// Same error_code as the control-station contract (PARKING_CAMERA_UNAVAILABLE from the collector).
class ParkingObservationError : public std::runtime_error {
 public:
  ParkingObservationError(std::string error_code, const std::string& message)
      : std::runtime_error(message), error_code_(std::move(error_code)) {}
  const std::string& error_code() const { return error_code_; }

 private:
  std::string error_code_;
};

struct ParkingObservation {
  ParkingResult result;
  int frames_used = 0;      // usable frames behind the median
  int frames_examined = 0;  // frames captured after the request that were judged
  std::uint64_t first_sequence = 0;
  std::uint64_t last_sequence = 0;
  double captured_from_s = 0.0;
  double captured_until_s = 0.0;
  std::optional<Corners> marker_corners_px;     // Pinky marker of the latest judged frame, for drawing
  std::optional<cv::Point2d> view_shift_xy_px;  // fixed-marker drift of that frame
};

class ParkingObservationCollector {
 public:
  // requested_at_s and every captured_at_s / now_s share one monotonic clock.
  ParkingObservationCollector(ParkingSpot spot, double requested_at_s,
                              double collect_seconds = kObservationCollectSeconds);

  // Judges one frame. Returns false for a frame captured before the request (요청 전에 찍힌 프레임은 쓰지 않는다)
  // or one arriving after the collection is complete. Throws ParkingObservationError when the frame size is
  // not the calibrated size.
  bool add(std::uint64_t sequence, double captured_at_s, const cv::Mat& image);
  // Enough usable frames, or the collection window has passed.
  bool done(double now_s) const;
  int frames_used() const { return static_cast<int>(judgment_.recent().size()); }
  int frames_examined() const { return frames_examined_; }
  // Throws ParkingObservationError when no frame was examined.
  ParkingObservation finish() const;

  const ParkingSpot& spot() const { return spot_; }

 private:
  ParkingSpot spot_;
  double requested_at_s_;
  double deadline_s_;
  double collect_seconds_;
  ParkingJudge judgment_;
  int frames_examined_ = 0;
  std::uint64_t first_sequence_ = 0;
  std::uint64_t last_sequence_ = 0;
  double captured_from_s_ = 0.0;
  double captured_until_s_ = 0.0;
  std::optional<Corners> marker_corners_px_;
  std::optional<cv::Point2d> view_shift_xy_px_;
};

}  // namespace parking_vision
