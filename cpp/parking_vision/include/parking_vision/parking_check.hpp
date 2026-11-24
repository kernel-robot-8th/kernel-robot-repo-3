// 천장 카메라로 Pinky의 ArUco 마커를 찾아, Pinky가 등록된 주차 위치에 멈췄는지 판정한다.
//
// 예전 src/control_station/parking_check.py(2026-11-03에 삭제, 마지막 버전은 커밋 8ea4c84)를 C++로 옮긴 것이다.
// 알고리즘, 임계값, reason 문구가 같다. test/test_python_parity.cpp가 그 Python 모듈로 만든 기대값과 비교한다.
//
// 위치: parking_vision 라이브러리(ROS 의존성 없음)의 핵심 판정 부분이다. CLI(pinky_parking)와 ROS 노드가 함께 쓴다.
//
// 핵심 아이디어:
// - 평면 투영: 등록할 때 정한 마커 평면(바닥과 평행)에 프레임마다 마커 네 꼭짓점의 카메라 광선을 교차시켜
//   평면 위 위치(mm)와 방향(deg)을 구한다. 프레임마다 PnP로 깊이를 추정하는 것보다 훨씬 안정적이다.
// - 고정 마커 시야 확인: 바닥에 붙인 고정 마커(anchor)가 등록 때 픽셀 위치에서 얼마나 밀렸는지 보고,
//   작은 밀림은 카메라 거치대가 흔들린 것으로 보고 빼 주고, 큰 밀림은 다른 시야로 보고 판정하지 않는다.
// - 판정 결과(ParkingStatus):
//   ARRIVED(허용 오차 안), MISALIGNED(마커는 보이지만 허용 오차 밖), NOT_FOUND(마커가 안 보이거나 측정 실패),
//   AMBIGUOUS(같은 ID 마커가 둘 이상), VIEW_UNVERIFIED(고정 마커로 카메라 시야를 확인할 수 없음).
//   결과는 관측일 뿐이며 로봇 명령이나 Nav2 이동 성공을 대신하지 않는다.
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

// 주차 위치 파일(JSON)의 schema 버전. 다르면 load_spot이 거부하고 다시 등록하게 한다.
inline constexpr int kSchemaVersion = 2;
// 내부 보정과 주차 위치가 가정하는 픽셀 좌표계(앱이 영상을 자르거나 돌리지 않은 V4L2 캡처 그대로)를 나타내는 ID.
inline constexpr const char* kPixelTransformId = "opencv-v4l2-capture-no-app-geometry-v1";
// 네 꼭짓점의 재투영 오차가 이보다 크면(px) 자세 추정 실패로 본다.
inline constexpr double kMaxReprojectionErrorPx = 3.0;
// 주차 위치의 view_check.max_anchor_shift_px 기본값(px). 고정 마커가 이보다 많이 움직였으면 그 프레임은 등록한
// 시야가 아니다(천장 카메라가 가끔 영상이 옆으로 약 96 px 밀린 채 열린다). 그보다 작은 밀림은 카메라 거치대가
// 조금 움직인 것이므로 빼 준다.
inline constexpr double kMaxViewShiftPx = 15.0;
// 밀림은 고정 마커들이 같은 값을 가리킬 때만 뺀다. 보이는 마커의 밀림이 그 중앙값과 이보다(px) 많이 다르면
// 카메라가 아니라 마커 하나가 옮겨진 것이다(카메라를 기울였을 때 마커 사이 차이는 약 1.5 px였다).
inline constexpr double kAnchorAgreePx = 3.0;
// 고정 마커가 하나만 보이면 마커가 옮겨진 것과 카메라가 밀린 것을 구분할 수 없으므로, 이만큼(px)까지만 빼 주고
// 그보다 크면 시야를 확인하지 못한 것(VIEW_UNVERIFIED)으로 본다. 운영자가 마커가 고정돼 있다고 확인한 주차 위치
// (view_check.single_anchor_trusted)는 대신 max_view_shift_px 전체를 쓴다.
inline constexpr double kSingleAnchorMaxShiftPx = 3.0;
// 등록 평면에서 잰 마커 한 변이 인쇄된 크기와 이 비율(10%) 안으로 같아야 한다. 아니면 마커가 그 평면에 있지 않다
// (들렸거나, 기울었거나, 다른 크기로 인쇄한 마커).
inline constexpr double kMaxSideErrorRatio = 0.1;
// 허용 오차의 이 배수까지 벗어나면 NEAR, 그보다 많이 벗어나면 FAR로 본다(화면 표시용 단계).
inline constexpr double kNearLimitRatio = 2.0;

// 판정 결과. 의미는 파일 맨 위 설명을 참고한다.
enum class ParkingStatus { Arrived, Misaligned, NotFound, Ambiguous, ViewUnverified };
// 허용 오차 대비 벗어난 정도: WITHIN(1배 이하), NEAR(kNearLimitRatio배 이하), FAR(그 이상).
enum class DeviationLevel { Within, Near, Far };

// Python enum과 같은 문자열("ARRIVED", "WITHIN", ...)이다. 관제 JSON과 ROS 메시지에 그대로 들어간다.
std::string to_string(ParkingStatus status);
std::string to_string(DeviationLevel level);

// 주차 위치 파일이나 내부 보정 파일을 쓸 수 없을 때 던진다(ROS 노드에서는 PARKING_SPOT_INVALID가 된다).
class ParkingSpotError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// ArUco 순서의 네 꼭짓점(px): 왼쪽 위, 오른쪽 위, 오른쪽 아래, 왼쪽 아래.
using Corners = std::array<cv::Point2d, 4>;
// 마커 ID -> 그 ID로 검출된 모든 마커(같은 ID가 여러 번 보일 수 있다).
using MarkerDetections = std::map<int, std::vector<Corners>>;

// 카메라 좌표계에서 본 마커 자세. 마커 축: +x는 오른쪽 변 방향, +y는 윗변 방향, +z는 마커 앞면에서 나오는 방향이다.
// rotation은 마커 축 -> 카메라 축 회전, translation_mm은 카메라에서 본 마커 중심 위치(mm)다.
struct MarkerPose {
  cv::Matx33d rotation;
  cv::Vec3d translation_mm;
};

// 마커 꼭짓점을 등록된 마커 평면으로 역투영한 결과. 그 평면의 마커 축 기준이다(x, y, side는 mm).
struct PlaneMarker {
  double x_mm;
  double y_mm;
  double yaw_deg;  // 마커 +x 변의 방향. 카메라에서 볼 때 반시계가 +
  double side_mm;   // 평면 위에서 잰 네 변 길이의 평균. 인쇄 크기와 비교해 평면 위에 있는지 확인한다
};

// 등록(기준) 마커에 대한 현재 마커의 상대 위치. 기준 마커 축으로 나타낸다.
struct PlanarOffset {
  double dx_mm;     // +는 기준 마커의 오른쪽 변 방향
  double dy_mm;     // +는 기준 마커의 윗변 방향
  double dyaw_deg;  // +는 카메라에서 볼 때 반시계 방향

  // 평면 위 거리 sqrt(dx² + dy²) (mm). 위치 허용 오차와 비교한다.
  double distance_mm() const;
};

// 허용 오차. 거리(mm)와 방향 차이 절댓값(deg)이 모두 이 안이면 ARRIVED다.
struct ParkingTolerance {
  double position_mm;
  double yaw_deg;
};

// 판정 하나의 결과. offset은 판정에 쓴 (중앙값) 오프셋이고, 마커를 못 찾았으면 비어 있다.
struct ParkingResult {
  ParkingStatus status;
  std::optional<PlanarOffset> offset;
  std::string reason;
  std::optional<double> view_shift_px = std::nullopt;  // 고정 마커 밀림 크기(px), 시야 확인을 했을 때만
  std::optional<double> position_ratio = std::nullopt;  // 거리 / 위치 허용 오차
  std::optional<double> yaw_ratio = std::nullopt;       // |dyaw| / 방향 허용 오차
  std::optional<DeviationLevel> deviation_level = std::nullopt;
};

// 카메라 내부 보정(카메라 행렬, 왜곡 계수, 보정한 해상도)과 그 파일 정보.
struct Intrinsics {
  cv::Matx33d camera_matrix;
  std::vector<double> distortion;
  int width = 0;
  int height = 0;
  std::optional<std::string> calibration_id;
  std::filesystem::path path;
  std::string sha256;  // 파싱한 바이트의 SHA-256. 등록 때 값과 다르면 load_spot이 거부한다
};

// 등록된 주차 위치(schema 2 파일을 load_spot으로 읽은 것).
struct ParkingSpot {
  std::string spot_id;
  std::string camera_id;
  std::string dictionary;
  int marker_id = 0;
  double marker_side_mm = 0.0;
  MarkerPose plane;  // 기준 마커 자세. 측정에는 그 평면과 축만 쓴다
  PlaneMarker reference;  // 등록 꼭짓점을 그 평면에 투영한 기준 위치
  Corners reference_corners_px;  // 등록 위치를 화면에 그리는 데 쓴다
  // 고정 마커 ID -> 등록 때 중심 픽셀. 시야 확인을 끄고 등록했을 때만 비어 있다.
  std::map<int, cv::Point2d> anchors;
  ParkingTolerance tolerance;
  Intrinsics intrinsics;
  std::optional<std::string> marker_forward_axis;  // Pinky 앞쪽을 가리키는 마커 축 "+x", "-x", "+y", "-y" 또는 기록 없음
  double max_view_shift_px = kMaxViewShiftPx;
  bool single_anchor_trusted = false;
  std::optional<std::string> registration_opencv_version;
};

// 프레임 하나를 관측한 결과. ParkingJudge가 여러 프레임을 모아 판정한다.
struct FrameObservation {
  std::optional<PlanarOffset> offset;
  std::optional<Corners> corners;  // 검출한 Pinky 마커 꼭짓점(px, 밀림을 빼기 전)
  std::string reason;
  // AMBIGUOUS나 VIEW_UNVERIFIED는 이전 프레임까지 무효로 만든다.
  std::optional<ParkingStatus> blocking_status;
  std::optional<double> view_shift_px;
  // 쓸 수 없는 이 프레임 때문에 판정할 것이 하나도 남지 않았을 때 보고할 상태. 건너뛴 프레임은 이전 프레임을 지우지 않는다.
  ParkingStatus skipped_status = ParkingStatus::NotFound;
  std::optional<cv::Point2d> view_shift_xy_px;  // 이 프레임에서 뺀 고정 마커 밀림 (x, y) px
};

// 파일 내용의 SHA-256(16진 문자열).
std::string sha256_of(const std::filesystem::path& path);
// 두 로더 모두 파싱한 바로 그 바이트로 해시를 계산하므로, 기록된 SHA-256은 실제로 쓴 내용을 가리킨다.
// load_intrinsics는 camera_id와 pixel_transform_id가 맞지 않으면 ParkingSpotError를 던진다.
Intrinsics load_intrinsics(const std::filesystem::path& path, const std::string& camera_id);
// registration_sha256을 주면 파싱한 주차 위치 파일 바이트의 SHA-256을 받는다. 내부 보정 경로는 repository_root 기준이다.
ParkingSpot load_spot(const std::filesystem::path& path, const std::filesystem::path& repository_root,
                      std::string* registration_sha256 = nullptr);

// 각도를 [-180, 180)로 감는다. Python % 의미를 따르며 판정과 추적기가 함께 쓴다.
double wrap_degrees(double degrees);

// BGR이나 흑백 영상을 받는다. 모르는 사전 이름이면 std::invalid_argument를 던진다.
MarkerDetections detect_markers(const cv::Mat& image, const std::string& dictionary_name);
// 한 변 side_mm인 마커의 꼭짓점 3D 좌표(마커 축, z=0, 중심이 원점). ArUco 꼭짓점 순서와 같다.
std::array<cv::Point3d, 4> marker_object_points(double side_mm);
// 꼭짓점 픽셀로 마커 자세를 추정한다. 등록할 때 마커 평면을 정하는 데만 쓴다.
// 카메라를 향하면서 재투영 오차가 kMaxReprojectionErrorPx 안인 후보 자세가 없으면 std::runtime_error를 던진다.
MarkerPose estimate_marker_pose(const Corners& corners_px, double side_mm, const cv::Matx33d& camera_matrix,
                                const std::vector<double>& distortion);
// 꼭짓점 광선을 plane(기준 마커 평면)과 교차시켜 평면 위 마커 위치·방향·변 길이를 구한다.
PlaneMarker project_to_plane(const Corners& corners_px, const MarkerPose& plane, const cv::Matx33d& camera_matrix,
                             const std::vector<double>& distortion);
// current - reference. dyaw는 [-180, 180)로 감는다.
PlanarOffset plane_offset(const PlaneMarker& reference, const PlaneMarker& current);
// Pinky 자신이 보는 방향 기준 (앞, 왼쪽) mm. dyaw는 이미 양수면 왼쪽으로 돈 것으로 읽힌다.
std::pair<double, double> pinky_frame_offset(const PlanarOffset& offset, const std::string& forward_axis);
// 정확히 한 번 보인 등록 고정 마커마다 (x, y) 픽셀 밀림(현재 중심 - 등록 중심).
std::vector<cv::Point2d> anchor_shifts(const std::map<int, cv::Point2d>& anchors, const MarkerDetections& found);
// 정확히 한 번 보인 등록 고정 마커들의 (x, y) 픽셀 밀림 중앙값. 하나도 없으면 nullopt.
std::optional<cv::Point2d> anchor_shift_px(const std::map<int, cv::Point2d>& anchors, const MarkerDetections& found);
// 프레임 하나를 관측한다: 마커 검출 -> 고정 마커로 시야 확인·밀림 보정 -> 평면 투영 -> 기준 대비 오프셋.
FrameObservation observe_frame(const cv::Mat& image, const ParkingSpot& spot);
// 프레임들의 중앙값. yaw는 첫 샘플 기준으로 펼쳐서, ±180° 근처 값이 평균 0이 되지 않게 한다.
PlanarOffset median_offset(const std::vector<PlanarOffset>& offsets);
// 오프셋 하나를 허용 오차와 비교한다. offset이 없으면 NOT_FOUND(reason이 비면 "marker not visible")다.
ParkingResult judge(const std::optional<PlanarOffset>& offset, const ParkingTolerance& tolerance,
                    const std::string& reason = "");

// 최근의 쓸 수 있는 프레임들의 중앙값으로 판정한다.
// - AMBIGUOUS / VIEW_UNVERIFIED 프레임은 모아 둔 프레임을 모두 버리고 그 상태를 바로 결과로 낸다.
// - 마커를 못 찾은 프레임은 건너뛰되, frames번 연속으로 못 찾으면 모아 둔 프레임을 버린다.
class ParkingJudge {
 public:
  // frames: 중앙값을 낼 최근 프레임 수(최대 보관 개수).
  ParkingJudge(ParkingTolerance tolerance, int frames);

  // 프레임 하나를 더하고 갱신된 결과를 돌려준다.
  const ParkingResult& add(const FrameObservation& observation);
  const ParkingResult& result() const { return result_; }
  const std::vector<PlanarOffset>& recent() const { return recent_; }

 private:
  ParkingTolerance tolerance_;
  int frames_;
  std::vector<PlanarOffset> recent_;
  int misses_ = 0;  // 마커를 연속으로 못 찾은 프레임 수
  ParkingResult result_;
};

}  // namespace parking_vision
