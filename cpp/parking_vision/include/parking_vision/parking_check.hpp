// Ceiling-camera ArUco check of whether Pinky stopped on its registered parking spot.
//
// C++ port of the former src/control_station/parking_check.py (removed 2026-11-03; last in commit 8ea4c84). Same
// algorithm, thresholds and reasons; test/test_python_parity.cpp checks it against fixtures generated from it.
#pragma once

#include <array>
#include <filesystem>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace parking_vision {

inline constexpr int kSchemaVersion = 2;
inline constexpr const char* kPixelTransformId = "opencv-v4l2-capture-no-app-geometry-v1";
// A pose that reprojects the four corners worse than this is treated as a failed estimate.
inline constexpr double kMaxReprojectionErrorPx = 3.0;
// Default for a spot's view_check.max_anchor_shift_px. Fixed markers moving more than this means the frame
// is not the registered view (the ceiling camera sometimes opens wrapped sideways by ~96 px); smaller shifts
// are the camera mount drifting and are subtracted.
inline constexpr double kMaxViewShiftPx = 15.0;
// The drift is subtracted only when the fixed markers agree on it. Visible markers whose shifts differ from
// their median by more than this mean one of them was moved, not the camera (a tilted camera spread ~1.5 px).
inline constexpr double kAnchorAgreePx = 3.0;
// With a single visible fixed marker a moved marker and a drifted camera look the same, so only a shift up to
// this much is subtracted; beyond it the view is unverified. A spot whose operator confirmed the marker is
// fastened down (view_check.single_anchor_trusted) uses the full max_view_shift_px instead.
inline constexpr double kSingleAnchorMaxShiftPx = 3.0;
// The marker side measured on the registered plane must stay this close to the printed side;
// otherwise the marker is not on that plane (lifted, tilted or a different print).
inline constexpr double kMaxSideErrorRatio = 0.1;
// Up to this multiple of the tolerance a misalignment counts as NEAR, beyond it as FAR.
inline constexpr double kNearLimitRatio = 2.0;

enum class ParkingStatus { Arrived, Misaligned, NotFound, Ambiguous, ViewUnverified };
enum class DeviationLevel { Within, Near, Far };

// Same strings as the Python enums ("ARRIVED", "WITHIN", ...).
std::string to_string(ParkingStatus status);
std::string to_string(DeviationLevel level);

class ParkingSpotError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// ArUco order: top-left, top-right, bottom-right, bottom-left.
using Corners = std::array<cv::Point2d, 4>;
// Marker id -> every detection of that id.
using MarkerDetections = std::map<int, std::vector<Corners>>;

// Marker pose in the camera frame. Marker axes: +x to its right edge, +y to its top edge, +z out of its face.
struct MarkerPose {
  cv::Matx33d rotation;
  cv::Vec3d translation_mm;
};

// Marker corners back-projected onto the registered marker plane, in that plane's marker axes.
struct PlaneMarker {
  double x_mm;
  double y_mm;
  double yaw_deg;  // direction of the marker +x edge, counterclockwise as seen from the camera
  double side_mm;
};

// Current marker relative to the reference marker, expressed in the reference marker axes.
struct PlanarOffset {
  double dx_mm;     // + toward the reference marker's right edge
  double dy_mm;     // + toward the reference marker's top edge
  double dyaw_deg;  // + counterclockwise as seen from the camera

  double distance_mm() const;
};

struct ParkingTolerance {
  double position_mm;
  double yaw_deg;
};

struct ParkingResult {
  ParkingStatus status;
  std::optional<PlanarOffset> offset;
  std::string reason;
  std::optional<double> view_shift_px = std::nullopt;
  std::optional<double> position_ratio = std::nullopt;  // distance / position tolerance
  std::optional<double> yaw_ratio = std::nullopt;       // |dyaw| / yaw tolerance
  std::optional<DeviationLevel> deviation_level = std::nullopt;
};

struct Intrinsics {
  cv::Matx33d camera_matrix;
  std::vector<double> distortion;
  int width = 0;
  int height = 0;
  std::optional<std::string> calibration_id;
  std::filesystem::path path;
  std::string sha256;
};

struct ParkingSpot {
  std::string spot_id;
  std::string camera_id;
  std::string dictionary;
  int marker_id = 0;
  double marker_side_mm = 0.0;
  MarkerPose plane;  // reference marker pose; only its plane and axes are used for measuring
  PlaneMarker reference;
  Corners reference_corners_px;  // for drawing the registered spot
  // Fixed marker id -> registered center pixel. Empty only when registered with the view check off.
  std::map<int, cv::Point2d> anchors;
  ParkingTolerance tolerance;
  Intrinsics intrinsics;
  std::optional<std::string> marker_forward_axis;  // "+x", "-x", "+y", "-y" or not recorded
  double max_view_shift_px = kMaxViewShiftPx;
  bool single_anchor_trusted = false;
  std::optional<std::string> registration_opencv_version;
};

struct FrameObservation {
  std::optional<PlanarOffset> offset;
  std::optional<Corners> corners;
  std::string reason;
  // AMBIGUOUS or VIEW_UNVERIFIED invalidates earlier frames too.
  std::optional<ParkingStatus> blocking_status;
  std::optional<double> view_shift_px;
  // Status to report when this unusable frame leaves nothing to judge; a skipped frame keeps earlier ones.
  ParkingStatus skipped_status = ParkingStatus::NotFound;
  std::optional<cv::Point2d> view_shift_xy_px;  // fixed-marker drift subtracted from this frame
};

std::string sha256_of(const std::filesystem::path& path);
Intrinsics load_intrinsics(const std::filesystem::path& path, const std::string& camera_id);
ParkingSpot load_spot(const std::filesystem::path& path, const std::filesystem::path& repository_root);

// Accepts a BGR or grayscale image. Throws std::invalid_argument for an unknown dictionary name.
MarkerDetections detect_markers(const cv::Mat& image, const std::string& dictionary_name);
std::array<cv::Point3d, 4> marker_object_points(double side_mm);
// Throws std::runtime_error when no candidate pose faces the camera within kMaxReprojectionErrorPx.
MarkerPose estimate_marker_pose(const Corners& corners_px, double side_mm, const cv::Matx33d& camera_matrix,
                                const std::vector<double>& distortion);
PlaneMarker project_to_plane(const Corners& corners_px, const MarkerPose& plane, const cv::Matx33d& camera_matrix,
                             const std::vector<double>& distortion);
PlanarOffset plane_offset(const PlaneMarker& reference, const PlaneMarker& current);
// (forward, left) in mm from Pinky's own view; dyaw already reads as a left turn when positive.
std::pair<double, double> pinky_frame_offset(const PlanarOffset& offset, const std::string& forward_axis);
// (x, y) pixel shift of each registered fixed marker that is visible exactly once.
std::vector<cv::Point2d> anchor_shifts(const std::map<int, cv::Point2d>& anchors, const MarkerDetections& found);
// Median (x, y) pixel shift of the registered fixed markers visible exactly once, or nullopt if none are.
std::optional<cv::Point2d> anchor_shift_px(const std::map<int, cv::Point2d>& anchors, const MarkerDetections& found);
FrameObservation observe_frame(const cv::Mat& image, const ParkingSpot& spot);
// Median over frames; yaw is unwrapped around the first sample so ±180° does not average to 0.
PlanarOffset median_offset(const std::vector<PlanarOffset>& offsets);
ParkingResult judge(const std::optional<PlanarOffset>& offset, const ParkingTolerance& tolerance,
                    const std::string& reason = "");

// Median judgment over the most recent usable frames.
class ParkingJudge {
 public:
  ParkingJudge(ParkingTolerance tolerance, int frames);

  const ParkingResult& add(const FrameObservation& observation);
  const ParkingResult& result() const { return result_; }
  const std::vector<PlanarOffset>& recent() const { return recent_; }

 private:
  ParkingTolerance tolerance_;
  int frames_;
  std::vector<PlanarOffset> recent_;
  int misses_ = 0;
  ParkingResult result_;
};

}  // namespace parking_vision
