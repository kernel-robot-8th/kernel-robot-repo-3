// 주차 판정 핵심 구현: 주차 위치·내부 보정 파일 읽기, ArUco 검출, 마커 평면 투영, 고정 마커 시야 확인, 판정.
//
// 위치: parking_vision 라이브러리(ROS 의존성 없음). 예전 parking_check.py를 그대로 옮긴 것이라 계산 순서, 중앙값 방식,
// reason 문구까지 Python과 같게 맞췄다(test/test_python_parity.cpp가 확인한다).
// 단위: 픽셀은 px, 평면 위 거리는 mm, 각도는 deg(내부 atan2 결과만 rad에서 deg로 바꾼다).
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
#else  // OpenCV 4.7 미만은 ArUco가 contrib 모듈에 있다
#include <opencv2/aruco.hpp>
#endif

namespace parking_vision {
namespace {

using nlohmann::json;
namespace fs = std::filesystem;

// Pinky 앞쪽을 가리키는 마커 축 -> 마커 (x, y) 좌표에서 앞쪽 단위 벡터(마커 +z가 위쪽일 때).
// 왼쪽 벡터는 앞쪽을 반시계로 90° 돌린 것이라 pinky_frame_offset에서 따로 계산한다.
const std::map<std::string, std::pair<double, double>> kMarkerForwardAxes = {
    {"+x", {1.0, 0.0}}, {"-x", {-1.0, 0.0}}, {"+y", {0.0, 1.0}}, {"-y", {0.0, -1.0}}};

// 주차 위치 파일의 aruco_dictionary 이름 -> OpenCV 사전 ID. 여기에 없는 이름은 load_spot이 거부한다.
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

// Python의 float %: std::fmod와 달리 결과의 부호가 나누는 수를 따른다.
double floor_mod(double value, double divisor) { return value - divisor * std::floor(value / divisor); }


// numpy.median: 개수가 짝수면 가운데 두 값의 평균이다.
double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  return values.size() % 2 == 1 ? values[middle] : (values[middle - 1] + values[middle]) / 2.0;
}

// Python의 % 서식과 같은 reason 문구를 만들기 위한 printf 서식. 256자를 넘으면 잘린다.
template <typename... Args>
std::string format(const char* pattern, Args... args) {
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer), pattern, args...);
  return buffer;
}

// 네 꼭짓점의 평균(마커 중심 px).
cv::Point2d center_of(const Corners& corners) {
  return (corners[0] + corners[1] + corners[2] + corners[3]) / 4.0;
}

// JSON 값의 Python repr. 오류 메시지에 실제로 무엇이 들어 있었는지 Python과 같은 모양으로 보여 준다.
std::string repr(const json& value) {
  if (value.is_null()) return "None";
  if (value.is_string()) return "'" + value.get<std::string>() + "'";
  return value.dump();
}

// Python의 dict.get(key)처럼, 객체가 아니거나 키가 없으면 null을 돌려준다.
json value_or_null(const json& object, const char* key) {
  return object.is_object() && object.contains(key) ? object.at(key) : json();
}

// 중첩 배열의 숫자를 순서대로 펼친다(numpy 배열을 flatten하는 것과 같다).
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

// 파일 전체를 바이트 그대로 읽는다. 해시와 파싱이 같은 바이트를 쓰도록 한 번만 읽는다.
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

// 시야 확인을 통과한 프레임에서 Pinky 마커를 찾아 기준 대비 오프셋을 구한다.
// view_shift: 꼭짓점에서 뺄 고정 마커 밀림(px), shift: 그 크기(결과에 기록만 한다).
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
  // 고정 마커는 같은 바닥에 있으므로, 그 작은 밀림은 되돌려야 할 시야 이동이다.
  const Corners& corners = markers->second.front();
  Corners undone;
  for (std::size_t i = 0; i < corners.size(); ++i) undone[i] = corners[i] - view_shift;
  const PlaneMarker current =
      project_to_plane(undone, spot.plane, spot.intrinsics.camera_matrix, spot.intrinsics.distortion);
  observation.corners = corners;
  const bool finite = std::isfinite(current.x_mm) && std::isfinite(current.y_mm) && std::isfinite(current.yaw_deg) &&
                      std::isfinite(current.side_mm);
  // "안에 있지 않으면"으로 쓴 이유: 투영이 퇴화해 변 길이가 NaN이면 비교가 모두 false라 통과해 버리기 때문이다.
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

  // 기준 위치는 저장된 숫자가 아니라 등록 꼭짓점을 지금 평면에 다시 투영해서 구한다(판정과 같은 계산 경로).
  const json& corner_rows = data.at("reference_corners_px");
  if (corner_rows.size() != 4) throw ParkingSpotError("reference_corners_px는 4개 점이어야 합니다: " + path.string());
  Corners corners;
  for (std::size_t i = 0; i < corners.size(); ++i) corners[i] = point2(corner_rows.at(i));

  const json view_check = value_or_null(data, "view_check");
  // 고정 마커가 없으면 시야 확인을 끄고 등록했다는 기록(mode OFF)이 있어야 한다. 실수로 빠진 것과 구분한다.
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
  // 약 40 px 크기 마커에서 정수 꼭짓점을 쓰면 자세가 두 기울기 사이를 오갔다(천장 카메라에서 yaw가 약 12° 튐).
  // subpixel 보정을 켜면 0.6° 안으로 유지됐다.
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

// 마커 축(+x 오른쪽, +y 위쪽)에서 왼쪽 위, 오른쪽 위, 오른쪽 아래, 왼쪽 아래 순서. ArUco 꼭짓점 순서와 맞춘다.
std::array<cv::Point3d, 4> marker_object_points(double side_mm) {
  const double half = side_mm / 2.0;
  return {cv::Point3d(-half, half, 0.0), cv::Point3d(half, half, 0.0), cv::Point3d(half, -half, 0.0),
          cv::Point3d(-half, -half, 0.0)};
}

MarkerPose estimate_marker_pose(const Corners& corners_px, double side_mm, const cv::Matx33d& camera_matrix,
                                const std::vector<double>& distortion) {
  // IPPE는 정사각형에 대해 후보 자세 두 개를 돌려준다. 거의 수직으로 내려다보면 틀린 쪽이 이길 수 있으므로,
  // 마커 앞면이 카메라를 향하는 자세만 남기고 그중 재투영 오차가 가장 작은 것을 고른다.
  // 주점(principal point)에서 정확히 정면을 보는 정사각형이면 IPPE가 퇴화하므로 ITERATIVE가 그 경우를 맡는다.
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
      // 회전 행렬의 세 번째 열 = 카메라 좌표계에서 본 마커 +z(앞면 법선). 마커가 카메라 앞(z > 0)에 있고
      // 법선이 카메라 쪽(-translation 방향)을 향해야 한다.
      const cv::Vec3d normal(rotation(0, 2), rotation(1, 2), rotation(2, 2));
      if (translation[2] > 0 && normal.dot(-translation) > 0 && error < kMaxReprojectionErrorPx) {
        candidates.push_back({error, {rotation, translation}});
      }
    }
    if (!candidates.empty()) break;  // IPPE로 찾았으면 ITERATIVE는 돌리지 않는다
  }
  if (candidates.empty()) throw std::runtime_error("marker pose estimation failed");
  return std::min_element(candidates.begin(), candidates.end(),
                          [](const Candidate& a, const Candidate& b) { return a.error < b.error; })
      ->pose;
}

PlaneMarker project_to_plane(const Corners& corners_px, const MarkerPose& plane, const cv::Matx33d& camera_matrix,
                             const std::vector<double>& distortion) {
  // Pinky 마커는 등록한 바닥과 평행한 평면 위에 머무르므로, 프레임마다 PnP 깊이를 믿는 대신 꼭짓점 광선을
  // 그 평면과 교차시킨다. 천장 카메라의 약 40 px 마커에서 PnP 깊이는 약 45 mm 흔들렸고 그중 약 20 mm가 평면 안
  // 오프셋으로 새어 들어왔다. 평면을 쓰면 0.1 mm 아래로 유지된다.
  //
  // 기하: undistortPoints로 왜곡을 없앤 정규화 좌표 (x, y)가 광선 방향 (x, y, 1)이다. 평면은 법선 n(기준 마커 +z)과
  // 거리 d = n·t로 n·p = d이므로, 광선 위 점 p = ray * d / (ray·n)이다. 그 점을 기준 마커 축으로 옮기면
  // (R^T (p - t)) 평면 위 2D 좌표(mm)가 된다.
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
  const cv::Point2d x_edge = (local[1] - local[0]) + (local[2] - local[3]);  // 오른쪽을 향하는 두 변(윗변, 아랫변)의 합
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
  // 앞쪽 단위 벡터 f = (fx, fy), 왼쪽 = f를 반시계 90° 회전한 (-fy, fx). 오프셋을 두 벡터에 내적한다.
  const auto [fx, fy] = kMarkerForwardAxes.at(forward_axis);
  return {offset.dx_mm * fx + offset.dy_mm * fy, -offset.dx_mm * fy + offset.dy_mm * fx};
}

std::vector<cv::Point2d> anchor_shifts(const std::map<int, cv::Point2d>& anchors, const MarkerDetections& found) {
  std::vector<cv::Point2d> shifts;
  for (const auto& [marker_id, center] : anchors) {
    const auto markers = found.find(marker_id);
    // 같은 ID가 둘 이상 보이면 어느 것이 고정 마커인지 모르므로 쓰지 않는다.
    if (markers != found.end() && markers->second.size() == 1) shifts.push_back(center_of(markers->second.front()) - center);
  }
  return shifts;
}

namespace {

// x와 y를 따로 중앙값을 낸다(numpy.median(axis=0)과 같다).
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
  if (spot.anchors.empty()) {  // 시야 확인 끔: 밀리거나 움직인 시야를 알아채지 못하고 그대로 오프셋으로 나타난다
    return observe_marker(found, spot, cv::Point2d(0.0, 0.0), std::nullopt);
  }
  const std::vector<cv::Point2d> shifts = anchor_shifts(spot.anchors, found);
  if (shifts.empty()) {  // 작은 고정 마커는 한두 프레임씩 빠진다. Pinky 마커를 놓친 프레임처럼 건너뛴다
    FrameObservation observation;
    observation.reason = "no fixed marker visible to verify the camera view";
    observation.skipped_status = ParkingStatus::ViewUnverified;
    return observation;
  }
  // 판단 순서: 밀림이 너무 큼(다른 시야) -> 고정 마커끼리 밀림이 다름(마커가 옮겨짐) -> 하나만 보이는데 밀림이 큼.
  // 셋 다 아니면 밀림을 빼고 Pinky 마커를 관측한다.
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
    // NaN은 어떤 한계값과 비교해도 false라, 그대로 두면 ARRIVED로 읽힌다.
    return {ParkingStatus::NotFound, std::nullopt, "non-finite offset (marker projection failed)"};
  }
  // 위치와 방향을 각자 허용 오차로 나눈 비율로 맞춰서, 더 나쁜 쪽으로 단계(WITHIN/NEAR/FAR)를 정한다.
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
  if (observation.blocking_status) {  // AMBIGUOUS / VIEW_UNVERIFIED: 모아 둔 프레임까지 버린다
    recent_.clear();
    result_ = ParkingResult{*observation.blocking_status, std::nullopt, observation.reason, observation.view_shift_px};
    return result_;
  }
  if (!observation.offset) {
    if (++misses_ >= frames_) recent_.clear();  // 연속으로 못 찾으면 이전 값을 버린다
  } else {
    misses_ = 0;
    recent_.push_back(*observation.offset);
    // 가장 최근 frames_개만 남긴다.
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
