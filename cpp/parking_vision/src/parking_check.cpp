#include "parking_vision/parking_check.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <sstream>

#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <openssl/evp.h>

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#define PARKING_VISION_ARUCO_DETECTOR 1
#include <opencv2/objdetect/aruco_detector.hpp>
#else  // OpenCV < 4.7 keeps ArUco in the contrib module
#include <opencv2/aruco.hpp>
#endif

namespace parking_vision {
namespace {

using nlohmann::json;
namespace fs = std::filesystem;

// Marker axis that points to Pinky's front -> (forward, left) unit vectors in marker (x, y), with marker +z up.
const std::map<std::string, std::pair<double, double>> kMarkerForwardAxes = {
    {"+x", {1.0, 0.0}}, {"-x", {-1.0, 0.0}}, {"+y", {0.0, 1.0}}, {"-y", {0.0, -1.0}}};

const std::map<std::string, int> kArucoDictionaries = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
    {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
    {"DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
    {"DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
    {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11},
};

// Python's float %: the result takes the sign of the divisor, unlike std::fmod.
double floor_mod(double value, double divisor) { return value - divisor * std::floor(value / divisor); }


// numpy.median: the mean of the two middle values for an even count.
double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  return values.size() % 2 == 1 ? values[middle] : (values[middle - 1] + values[middle]) / 2.0;
}

template <typename... Args>
std::string format(const char* pattern, Args... args) {
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer), pattern, args...);
  return buffer;
}

cv::Point2d center_of(const Corners& corners) {
  return (corners[0] + corners[1] + corners[2] + corners[3]) / 4.0;
}

// Python repr of a JSON value, for error messages that name what was found.
std::string repr(const json& value) {
  if (value.is_null()) return "None";
  if (value.is_string()) return "'" + value.get<std::string>() + "'";
  return value.dump();
}

json value_or_null(const json& object, const char* key) {
  return object.is_object() && object.contains(key) ? object.at(key) : json();
}

void flatten_numbers(const json& value, std::vector<double>& out) {
  if (value.is_array()) {
    for (const auto& item : value) flatten_numbers(item, out);
  } else {
    out.push_back(value.get<double>());
  }
}

cv::Matx33d matx33(const json& rows) {
  std::vector<double> values;
  flatten_numbers(rows, values);
  if (values.size() != 9) throw ParkingSpotError("3x3 행렬이 아닙니다: " + rows.dump());
  return cv::Matx33d(values.data());
}

cv::Point2d point2(const json& xy) { return {xy.at(0).get<double>(), xy.at(1).get<double>()}; }

std::string read_bytes(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw ParkingSpotError("파일을 열 수 없습니다: " + path.string());
  return std::string((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
}

std::string sha256_hex(const std::string& bytes, const fs::path& path) {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  if (EVP_Digest(bytes.data(), bytes.size(), digest, &length, EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("SHA-256 계산 실패: " + path.string());
  }
  std::string hex;
  for (unsigned int i = 0; i < length; ++i) hex += format("%02x", digest[i]);
  return hex;
}

FrameObservation observe_marker(const MarkerDetections& found, const ParkingSpot& spot, cv::Point2d view_shift,
                                std::optional<double> shift) {
  FrameObservation observation;
  observation.view_shift_px = shift;
  const auto markers = found.find(spot.marker_id);
  if (markers == found.end() || markers->second.empty()) {
    observation.reason = "marker not visible";
    return observation;
  }
  if (markers->second.size() > 1) {  // 같은 ID가 둘이면 어느 쪽이 Pinky인지 모른다
    observation.reason = format("marker %d seen %zu times", spot.marker_id, markers->second.size());
    observation.blocking_status = ParkingStatus::Ambiguous;
    return observation;
  }
  // The fixed markers lie on the same floor, so their small shift is the view drift to undo.
  const Corners& corners = markers->second.front();
  Corners undone;
  for (std::size_t i = 0; i < corners.size(); ++i) undone[i] = corners[i] - view_shift;
  const PlaneMarker current =
      project_to_plane(undone, spot.plane, spot.intrinsics.camera_matrix, spot.intrinsics.distortion);
  observation.corners = corners;
  const bool finite = std::isfinite(current.x_mm) && std::isfinite(current.y_mm) && std::isfinite(current.yaw_deg) &&
                      std::isfinite(current.side_mm);
  // Written as "not within" so a NaN side from a degenerate projection is rejected instead of passing.
  if (!finite || !(std::abs(current.side_mm - spot.marker_side_mm) <= kMaxSideErrorRatio * spot.marker_side_mm)) {
    observation.reason = format("marker side %.1f mm on the registered plane (printed %g mm)", current.side_mm,
                                spot.marker_side_mm);
    return observation;
  }
  observation.offset = plane_offset(spot.reference, current);
  return observation;
}

}  // namespace

std::string to_string(ParkingStatus status) {
  switch (status) {
    case ParkingStatus::Arrived: return "ARRIVED";
    case ParkingStatus::Misaligned: return "MISALIGNED";
    case ParkingStatus::NotFound: return "NOT_FOUND";
    case ParkingStatus::Ambiguous: return "AMBIGUOUS";
    case ParkingStatus::ViewUnverified: return "VIEW_UNVERIFIED";
  }
  return "UNKNOWN";
}

std::string to_string(DeviationLevel level) {
  switch (level) {
    case DeviationLevel::Within: return "WITHIN";
    case DeviationLevel::Near: return "NEAR";
    case DeviationLevel::Far: return "FAR";
  }
  return "UNKNOWN";
}

double PlanarOffset::distance_mm() const { return std::hypot(dx_mm, dy_mm); }

double wrap_degrees(double degrees) { return floor_mod(degrees + 180.0, 360.0) - 180.0; }

std::string sha256_of(const fs::path& path) { return sha256_hex(read_bytes(path), path); }

Intrinsics load_intrinsics(const fs::path& path, const std::string& camera_id) {
  const std::string bytes = read_bytes(path);
  const json data = json::parse(bytes);
  const json found_camera = value_or_null(data, "camera_id");
  if (found_camera != json(camera_id)) {
    throw ParkingSpotError("내부 보정 파일의 camera_id가 " + repr(found_camera) + "입니다 ('" + camera_id +
                           "' 필요): " + path.string());
  }
  const json transform = value_or_null(data, "pixel_transform_id");
  if (transform != json(kPixelTransformId)) {
    throw ParkingSpotError("내부 보정 파일의 pixel_transform_id가 다릅니다: " + repr(transform));
  }
  Intrinsics intrinsics;
  intrinsics.camera_matrix = matx33(data.at("camera_matrix"));
  flatten_numbers(data.at("distortion_coefficients"), intrinsics.distortion);
  intrinsics.width = data.at("image_size_px").at("width").get<int>();
  intrinsics.height = data.at("image_size_px").at("height").get<int>();
  const json calibration_id = value_or_null(data, "calibration_id");
  if (calibration_id.is_string()) intrinsics.calibration_id = calibration_id.get<std::string>();
  intrinsics.path = path;
  intrinsics.sha256 = sha256_hex(bytes, path);
  return intrinsics;
}

ParkingSpot load_spot(const fs::path& path, const fs::path& repository_root, std::string* registration_sha256) {
  std::error_code error;
  if (!fs::is_regular_file(path, error)) {
    throw ParkingSpotError("등록된 주차 위치가 없습니다: " + path.string() + " (먼저 register)");
  }
  const std::string bytes = read_bytes(path);
  const json data = json::parse(bytes);
  const json schema = value_or_null(data, "schema_version");
  if (schema != json(kSchemaVersion)) {
    throw ParkingSpotError("지원하지 않는 주차 위치 schema " + (schema.is_null() ? "None" : schema.dump()) +
                           " (필요 " + std::to_string(kSchemaVersion) + "). 다시 등록하세요: " + path.string());
  }
  const json& calibration = data.at("intrinsic_calibration");
  Intrinsics intrinsics = load_intrinsics(repository_root / calibration.at("path").get<std::string>(),
                                          data.at("camera_id").get<std::string>());
  if (intrinsics.sha256 != calibration.at("sha256").get<std::string>()) {
    throw ParkingSpotError("등록 이후 내부 보정 파일이 바뀌었습니다. 주차 위치를 다시 등록하세요.");
  }
  const json& pose = data.at("reference_pose_in_camera");
  std::vector<double> translation;
  flatten_numbers(pose.at("translation_mm"), translation);
  if (translation.size() != 3) throw ParkingSpotError("translation_mm은 3개 값이어야 합니다: " + path.string());
  const MarkerPose plane{matx33(pose.at("rotation")), cv::Vec3d(translation[0], translation[1], translation[2])};

  const json& corner_rows = data.at("reference_corners_px");
  if (corner_rows.size() != 4) throw ParkingSpotError("reference_corners_px는 4개 점이어야 합니다: " + path.string());
  Corners corners;
  for (std::size_t i = 0; i < corners.size(); ++i) corners[i] = point2(corner_rows.at(i));

  const json view_check = value_or_null(data, "view_check");
  if (data.at("anchors").empty() && value_or_null(view_check, "mode") != json("OFF")) {
    throw ParkingSpotError("고정 마커가 없는데 view_check.mode가 OFF로 기록되지 않았습니다: " + path.string());
  }
  const json& tolerance_data = data.at("tolerance");
  const ParkingTolerance tolerance{tolerance_data.at("position_mm").get<double>(),
                                   tolerance_data.at("yaw_deg").get<double>()};
  if (!(tolerance.position_mm > 0 && tolerance.yaw_deg > 0)) {
    throw ParkingSpotError("허용 오차는 0보다 커야 합니다: " + tolerance_data.dump());
  }
  const json trusted = value_or_null(view_check, "single_anchor_trusted");
  if (!trusted.is_null() && !trusted.is_boolean()) {
    throw ParkingSpotError("view_check.single_anchor_trusted는 true/false여야 합니다: " + repr(trusted));
  }
  const json shift_data = value_or_null(view_check, "max_anchor_shift_px");
  const double max_view_shift = shift_data.is_null() ? kMaxViewShiftPx : shift_data.get<double>();
  if (!(max_view_shift > 0)) {
    throw ParkingSpotError("view_check.max_anchor_shift_px는 0보다 커야 합니다: " + format("%g", max_view_shift));
  }
  const json dictionary = data.at("aruco_dictionary");
  if (!dictionary.is_string() || kArucoDictionaries.count(dictionary.get<std::string>()) == 0) {
    throw ParkingSpotError("지원하지 않는 ArUco 사전입니다: " + repr(dictionary));
  }
  const json forward_axis = value_or_null(data, "marker_forward_axis");
  if (!forward_axis.is_null() &&
      (!forward_axis.is_string() || kMarkerForwardAxes.count(forward_axis.get<std::string>()) == 0)) {
    throw ParkingSpotError("marker_forward_axis는 ['+x', '+y', '-x', '-y'] 중 하나여야 합니다: " + repr(forward_axis));
  }

  ParkingSpot spot;
  spot.spot_id = data.at("spot_id").get<std::string>();
  spot.camera_id = data.at("camera_id").get<std::string>();
  spot.dictionary = dictionary.get<std::string>();
  spot.marker_id = data.at("marker_id").get<int>();
  spot.marker_side_mm = data.at("marker_side_mm").get<double>();
  spot.plane = plane;
  spot.reference = project_to_plane(corners, plane, intrinsics.camera_matrix, intrinsics.distortion);
  spot.reference_corners_px = corners;
  for (const auto& anchor : data.at("anchors")) {
    spot.anchors[anchor.at("marker_id").get<int>()] = point2(anchor.at("center_px"));
  }
  spot.tolerance = tolerance;
  spot.intrinsics = std::move(intrinsics);
  if (forward_axis.is_string()) spot.marker_forward_axis = forward_axis.get<std::string>();
  spot.max_view_shift_px = max_view_shift;
  spot.single_anchor_trusted = trusted.is_boolean() && trusted.get<bool>();
  const json opencv_version = value_or_null(data, "opencv_version");
  if (opencv_version.is_string()) spot.registration_opencv_version = opencv_version.get<std::string>();
  if (registration_sha256) *registration_sha256 = sha256_hex(bytes, path);
  return spot;
}

MarkerDetections detect_markers(const cv::Mat& image, const std::string& dictionary_name) {
  const auto dictionary_id = kArucoDictionaries.find(dictionary_name);
  if (dictionary_id == kArucoDictionaries.end()) {
    throw std::invalid_argument("unknown ArUco dictionary: " + dictionary_name);
  }
  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }
  // Integer corners on a ~40 px marker let the pose flip between two tilts (yaw jumped ~12° on the
  // ceiling camera); subpixel refinement kept it within 0.6°.
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
#ifdef PARKING_VISION_ARUCO_DETECTOR
  cv::aruco::DetectorParameters parameters;
  parameters.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  cv::aruco::ArucoDetector(cv::aruco::getPredefinedDictionary(dictionary_id->second), parameters)
      .detectMarkers(gray, corners, ids);
#else
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  cv::aruco::detectMarkers(gray, cv::aruco::getPredefinedDictionary(dictionary_id->second), corners, ids,
                           parameters);
#endif
  MarkerDetections found;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    Corners marker;
    for (std::size_t j = 0; j < marker.size(); ++j) marker[j] = cv::Point2d(corners[i][j]);
    found[ids[i]].push_back(marker);
  }
  return found;
}

std::array<cv::Point3d, 4> marker_object_points(double side_mm) {
  const double half = side_mm / 2.0;
  return {cv::Point3d(-half, half, 0.0), cv::Point3d(half, half, 0.0), cv::Point3d(half, -half, 0.0),
          cv::Point3d(-half, -half, 0.0)};
}

MarkerPose estimate_marker_pose(const Corners& corners_px, double side_mm, const cv::Matx33d& camera_matrix,
                                const std::vector<double>& distortion) {
  // IPPE returns two candidate poses for a square. Near a straight-down view the wrong one can win,
  // so keep only poses whose marker face points toward the camera and take the lowest reprojection error.
  // IPPE degenerates for an exactly fronto-parallel square at the principal point; ITERATIVE covers that case.
  const auto object = marker_object_points(side_mm);
  const std::vector<cv::Point3d> object_points(object.begin(), object.end());
  const std::vector<cv::Point2d> image_points(corners_px.begin(), corners_px.end());
  struct Candidate {
    double error;
    MarkerPose pose;
  };
  std::vector<Candidate> candidates;
  for (const int flags : {cv::SOLVEPNP_IPPE_SQUARE, cv::SOLVEPNP_ITERATIVE}) {
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::Mat errors;
    const int count = cv::solvePnPGeneric(object_points, image_points, camera_matrix, distortion, rvecs, tvecs,
                                          false, static_cast<cv::SolvePnPMethod>(flags), cv::noArray(),
                                          cv::noArray(), errors);
    errors.convertTo(errors, CV_64F);
    for (int index = 0; index < count; ++index) {
      cv::Matx33d rotation;
      cv::Rodrigues(rvecs[index], rotation);
      const cv::Vec3d translation(tvecs[index].reshape(1, 3));
      const double error = errors.at<double>(index);
      const cv::Vec3d normal(rotation(0, 2), rotation(1, 2), rotation(2, 2));
      if (translation[2] > 0 && normal.dot(-translation) > 0 && error < kMaxReprojectionErrorPx) {
        candidates.push_back({error, {rotation, translation}});
      }
    }
    if (!candidates.empty()) break;
  }
  if (candidates.empty()) throw std::runtime_error("marker pose estimation failed");
  return std::min_element(candidates.begin(), candidates.end(),
                          [](const Candidate& a, const Candidate& b) { return a.error < b.error; })
      ->pose;
}

PlaneMarker project_to_plane(const Corners& corners_px, const MarkerPose& plane, const cv::Matx33d& camera_matrix,
                             const std::vector<double>& distortion) {
  // Pinky's marker stays on the floor-parallel plane it was registered on, so intersect each corner ray
  // with that plane instead of trusting a per-frame PnP depth. On the ~40 px ceiling-camera marker the
  // PnP depth drifted ~45 mm and leaked ~20 mm into the in-plane offset; the plane keeps it under 0.1 mm.
  const std::vector<cv::Point2d> pixels(corners_px.begin(), corners_px.end());
  std::vector<cv::Point2d> rays;
  cv::undistortPoints(pixels, rays, camera_matrix, distortion);
  const cv::Vec3d normal(plane.rotation(0, 2), plane.rotation(1, 2), plane.rotation(2, 2));
  const double plane_distance = normal.dot(plane.translation_mm);
  const cv::Matx33d to_marker = plane.rotation.t();
  std::array<cv::Point2d, 4> local;
  for (std::size_t i = 0; i < local.size(); ++i) {
    const cv::Vec3d ray(rays[i].x, rays[i].y, 1.0);
    const cv::Vec3d point = ray * (plane_distance / ray.dot(normal));
    const cv::Vec3d in_marker = to_marker * (point - plane.translation_mm);
    local[i] = {in_marker[0], in_marker[1]};
  }
  const cv::Point2d x_edge = (local[1] - local[0]) + (local[2] - local[3]);  // both edges toward the right
  double side_sum = 0.0;
  for (std::size_t i = 0; i < local.size(); ++i) side_sum += cv::norm(local[i] - local[(i + 1) % local.size()]);
  const cv::Point2d center = (local[0] + local[1] + local[2] + local[3]) / 4.0;
  return {center.x, center.y, std::atan2(x_edge.y, x_edge.x) * 180.0 / CV_PI, side_sum / 4.0};
}

PlanarOffset plane_offset(const PlaneMarker& reference, const PlaneMarker& current) {
  return {current.x_mm - reference.x_mm, current.y_mm - reference.y_mm,
          wrap_degrees(current.yaw_deg - reference.yaw_deg)};
}

std::pair<double, double> pinky_frame_offset(const PlanarOffset& offset, const std::string& forward_axis) {
  const auto [fx, fy] = kMarkerForwardAxes.at(forward_axis);
  return {offset.dx_mm * fx + offset.dy_mm * fy, -offset.dx_mm * fy + offset.dy_mm * fx};
}

std::vector<cv::Point2d> anchor_shifts(const std::map<int, cv::Point2d>& anchors, const MarkerDetections& found) {
  std::vector<cv::Point2d> shifts;
  for (const auto& [marker_id, center] : anchors) {
    const auto markers = found.find(marker_id);
    if (markers != found.end() && markers->second.size() == 1) shifts.push_back(center_of(markers->second.front()) - center);
  }
  return shifts;
}

namespace {

cv::Point2d median_point(const std::vector<cv::Point2d>& points) {
  std::vector<double> xs;
  std::vector<double> ys;
  for (const auto& point : points) {
    xs.push_back(point.x);
    ys.push_back(point.y);
  }
  return {median(xs), median(ys)};
}

}  // namespace

std::optional<cv::Point2d> anchor_shift_px(const std::map<int, cv::Point2d>& anchors, const MarkerDetections& found) {
  const std::vector<cv::Point2d> shifts = anchor_shifts(anchors, found);
  if (shifts.empty()) return std::nullopt;
  return median_point(shifts);
}

FrameObservation observe_frame(const cv::Mat& image, const ParkingSpot& spot) {
  const MarkerDetections found = detect_markers(image, spot.dictionary);
  if (spot.anchors.empty()) {  // view check off: a wrapped or moved view goes unnoticed and shows up as an offset
    return observe_marker(found, spot, cv::Point2d(0.0, 0.0), std::nullopt);
  }
  const std::vector<cv::Point2d> shifts = anchor_shifts(spot.anchors, found);
  if (shifts.empty()) {  // a small fixed marker drops out for single frames; skip it like a missed Pinky marker
    FrameObservation observation;
    observation.reason = "no fixed marker visible to verify the camera view";
    observation.skipped_status = ParkingStatus::ViewUnverified;
    return observation;
  }
  const cv::Point2d view_shift = median_point(shifts);
  const double shift = cv::norm(view_shift);
  const auto unverified = [&](std::string reason) {
    FrameObservation observation;
    observation.reason = std::move(reason);
    observation.blocking_status = ParkingStatus::ViewUnverified;
    observation.view_shift_px = shift;
    observation.view_shift_xy_px = view_shift;
    return observation;
  };
  if (shift > spot.max_view_shift_px) return unverified(format("fixed markers moved %.1f px from registration", shift));
  double disagreement = 0.0;
  for (const auto& s : shifts) disagreement = std::max(disagreement, cv::norm(s - view_shift));
  if (disagreement > kAnchorAgreePx) {
    return unverified(format("fixed markers moved differently (%.1f px apart); one may have been moved", disagreement));
  }
  if (shifts.size() == 1 && !spot.single_anchor_trusted && shift > kSingleAnchorMaxShiftPx) {
    return unverified(format(
        "only one fixed marker visible and it moved %.1f px; cannot tell camera drift from a moved marker", shift));
  }
  FrameObservation observation = observe_marker(found, spot, view_shift, shift);
  observation.view_shift_xy_px = view_shift;
  return observation;
}

PlanarOffset median_offset(const std::vector<PlanarOffset>& offsets) {
  const double base = offsets.front().dyaw_deg;
  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> yaws;
  for (const auto& offset : offsets) {
    dxs.push_back(offset.dx_mm);
    dys.push_back(offset.dy_mm);
    yaws.push_back(base + wrap_degrees(offset.dyaw_deg - base));
  }
  return {median(dxs), median(dys), wrap_degrees(median(yaws))};
}

ParkingResult judge(const std::optional<PlanarOffset>& offset, const ParkingTolerance& tolerance,
                    const std::string& reason) {
  if (!offset) return {ParkingStatus::NotFound, std::nullopt, reason.empty() ? "marker not visible" : reason};
  if (!std::isfinite(offset->dx_mm) || !std::isfinite(offset->dy_mm) || !std::isfinite(offset->dyaw_deg)) {
    // NaN compares false against every limit and would otherwise read as ARRIVED.
    return {ParkingStatus::NotFound, std::nullopt, "non-finite offset (marker projection failed)"};
  }
  const double position_ratio = offset->distance_mm() / tolerance.position_mm;
  const double yaw_ratio = std::abs(offset->dyaw_deg) / tolerance.yaw_deg;
  const double worst = std::max(position_ratio, yaw_ratio);
  const DeviationLevel level = worst <= 1.0               ? DeviationLevel::Within
                               : worst <= kNearLimitRatio ? DeviationLevel::Near
                                                          : DeviationLevel::Far;
  std::vector<std::string> outside;
  if (position_ratio > 1.0) {
    outside.push_back(format("position %.0f mm > %g mm", offset->distance_mm(), tolerance.position_mm));
  }
  if (yaw_ratio > 1.0) {
    outside.push_back(format("yaw %+.1f deg > ±%g deg", offset->dyaw_deg, tolerance.yaw_deg));
  }
  ParkingResult result{ParkingStatus::Arrived, offset, "within tolerance", std::nullopt, position_ratio, yaw_ratio,
                       level};
  if (!outside.empty()) {
    result.status = ParkingStatus::Misaligned;
    result.reason = outside.front();
    for (std::size_t i = 1; i < outside.size(); ++i) result.reason += "; " + outside[i];
  }
  return result;
}

ParkingJudge::ParkingJudge(ParkingTolerance tolerance, int frames)
    : tolerance_(tolerance), frames_(frames), result_(judge(std::nullopt, tolerance, "no frame yet")) {}

const ParkingResult& ParkingJudge::add(const FrameObservation& observation) {
  if (observation.blocking_status) {
    recent_.clear();
    result_ = ParkingResult{*observation.blocking_status, std::nullopt, observation.reason, observation.view_shift_px};
    return result_;
  }
  if (!observation.offset) {
    if (++misses_ >= frames_) recent_.clear();  // 연속으로 못 찾으면 이전 값을 버린다
  } else {
    misses_ = 0;
    recent_.push_back(*observation.offset);
    if (static_cast<int>(recent_.size()) > frames_) recent_.erase(recent_.begin(), recent_.end() - frames_);
  }
  if (!recent_.empty()) {
    result_ = judge(median_offset(recent_), tolerance_);
  } else {
    result_ = ParkingResult{observation.skipped_status, std::nullopt,
                            observation.reason.empty() ? "marker not visible" : observation.reason};
  }
  result_.view_shift_px = observation.view_shift_px;
  return result_;
}

}  // namespace parking_vision
