// C++ 이식본을 같은 입력에서 예전 Python 모듈(parking_check, parking_observation, parking_tracker, 마지막 버전은
// 커밋 8ea4c84)의 기대값과 비교한다. 기대값은 2026-11-03에 그 모듈을 지우기 전에 실제로 돌려서 만들었고,
// 이제는 고정된 회귀 기준이다.
//
// 위치: parking_vision_tests(gtest). 확인 범위: 왜곡 렌즈의 평면 투영과 자세 추정, 프레임별 관측(고정 마커 하나만
// 밀림, 단일 마커 신뢰 포함), ParkingJudge 순서, 요청 이후 수집 시나리오, 추적기 틱 시퀀스.
// 판정 규칙을 의도적으로 바꾸면 기대값도 C++ 쪽에서 근거를 남기고 고쳐야 한다(생성 스크립트는 Python과 함께 지웠다).
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "parking_vision/parking_check.hpp"
#include "parking_vision/parking_observation.hpp"
#include "parking_vision/parking_tracker.hpp"

namespace parking_vision {
namespace {

namespace fs = std::filesystem;
using nlohmann::json;

// PARKING_VISION_FIXTURES 환경 변수로 저장소의 기대값 대신 다른 폴더를 쓴다. 예: 다른 OpenCV 버전으로 만든 기대값.
const fs::path kFixtures = std::getenv("PARKING_VISION_FIXTURES") ? std::getenv("PARKING_VISION_FIXTURES")
                                                                    : PARKING_VISION_FIXTURE_DIR;

// parity.json을 한 번만 읽어 둔다.
const json& parity() {
  static const json data = [] {
    std::ifstream stream(kFixtures / "parity.json");
    return json::parse(stream);
  }();
  return data;
}

// 같은 입력에서 하는 순수 기하 계산(undistortPoints, 평면 교차, solvePnP)의 허용 차이(mm, deg).
constexpr double kGeometryMm = 1e-6;
constexpr double kGeometryDeg = 1e-6;

// ArUco 꼭짓점 검출부터 시작하는 단계의 허용 차이. 기대값을 만든 OpenCV 버전과 같으면 반올림 수준까지 Python과
// 같아야 한다. 버전이 다르면 subpixel 꼭짓점이 움직인다. 2026-11-02에 4.14로 만든 기대값은 4.6에서 돈 이 이식본과
// 최대 0.12 px, 0.2 mm, 0.16° 달랐고, 서식화한 reason 문구는 0.1 mm 달랐다.
struct DetectionTolerance {
  double px;
  double mm;
  double deg;
  bool exact_reason;
};

// 같은 버전: 1e-9 이내 + reason 문자열 정확히 비교. 다른 버전: 0.2 px, 0.3 mm, 0.2° 이내, reason은 비교하지 않음.
const DetectionTolerance& detection() {
  static const DetectionTolerance tolerance =
      parity().at("python_opencv_version").get<std::string>() == CV_VERSION
          ? DetectionTolerance{1e-9, 1e-9, 1e-9, true}
          : DetectionTolerance{0.2, 0.3, 0.2, false};
  return tolerance;
}

TEST(PythonParity, FixtureOpenCVVersion) {
  // 실패하는 테스트가 아니다. 테스트 로그에 어떤 비교 방식(exact / cross-release)인지 남긴다.
  std::cout << "[ INFO     ] fixtures from Python OpenCV " << parity().at("python_opencv_version").get<std::string>()
            << ", linked OpenCV " << CV_VERSION << (detection().exact_reason ? " (exact)" : " (cross-release)")
            << std::endl;
}

// reason은 같은 OpenCV 버전일 때만 정확히 비교한다(숫자가 문구에 들어가므로).
void expect_reason(const std::string& actual, const json& expected, const std::string& where) {
  if (detection().exact_reason) EXPECT_EQ(actual, expected.get<std::string>()) << where;
}

Corners corners_of(const json& rows) {
  Corners corners;
  for (std::size_t i = 0; i < corners.size(); ++i) corners[i] = {rows.at(i).at(0), rows.at(i).at(1)};
  return corners;
}

MarkerPose pose_of(const json& data) {
  cv::Matx33d rotation;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) rotation(r, c) = data.at("rotation").at(r).at(c);
  }
  const json& t = data.at("translation_mm");
  return {rotation, cv::Vec3d(t.at(0), t.at(1), t.at(2))};
}

// 왜곡 계수가 0이 아닌 카메라. 평면 투영과 자세 추정이 왜곡 보정까지 Python과 같은지 보기 위한 것이다.
cv::Matx33d distorted_camera() {
  cv::Matx33d matrix;
  const json& rows = parity().at("distorted_camera").at("camera_matrix");
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) matrix(r, c) = rows.at(r).at(c);
  }
  return matrix;
}

std::vector<double> distortion() { return parity().at("distorted_camera").at("distortion"); }

std::optional<ParkingStatus> status_of(const json& value) {
  if (value.is_null()) return std::nullopt;
  for (const auto status : {ParkingStatus::Arrived, ParkingStatus::Misaligned, ParkingStatus::NotFound,
                            ParkingStatus::Ambiguous, ParkingStatus::ViewUnverified}) {
    if (to_string(status) == value.get<std::string>()) return status;
  }
  throw std::invalid_argument("unknown status " + value.dump());
}

void expect_offset(const std::optional<PlanarOffset>& actual, const json& expected, const std::string& where) {
  if (expected.is_null()) {
    EXPECT_FALSE(actual) << where;
    return;
  }
  ASSERT_TRUE(actual) << where;
  EXPECT_NEAR(actual->dx_mm, expected.at("dx_mm").get<double>(), detection().mm) << where;
  EXPECT_NEAR(actual->dy_mm, expected.at("dy_mm").get<double>(), detection().mm) << where;
  EXPECT_NEAR(actual->dyaw_deg, expected.at("dyaw_deg").get<double>(), detection().deg) << where;
}

void expect_optional_near(const std::optional<double>& actual, const json& expected, double tolerance,
                          const std::string& where) {
  if (expected.is_null()) {
    EXPECT_FALSE(actual) << where;
  } else {
    ASSERT_TRUE(actual) << where;
    EXPECT_NEAR(*actual, expected.get<double>(), tolerance) << where;
  }
}

cv::Mat frame(const json& image) {
  const cv::Mat loaded = cv::imread((kFixtures / image.get<std::string>()).string(), cv::IMREAD_UNCHANGED);
  if (loaded.empty()) throw std::runtime_error("cannot read fixture " + image.dump());
  return loaded;
}

// 기대값의 주차 위치 파일. 내부 보정 경로는 fixtures/spot 기준이다.
ParkingSpot fixture_spot() { return load_spot(kFixtures / parity().at("spot").at("path").get<std::string>(), kFixtures / "spot"); }

// 꼭짓점 광선과 평면 교차(project_to_plane) 결과가 1e-6 mm·deg 안에서 같다.
TEST(PythonParity, ProjectToPlaneWithDistortion) {
  for (const json& c : parity().at("geometry_cases")) {
    const std::string where = c.at("name");
    const PlaneMarker actual =
        project_to_plane(corners_of(c.at("corners_px")), pose_of(c.at("plane")), distorted_camera(), distortion());
    const json& expected = c.at("expected");
    EXPECT_NEAR(actual.x_mm, expected.at("x_mm").get<double>(), kGeometryMm) << where;
    EXPECT_NEAR(actual.y_mm, expected.at("y_mm").get<double>(), kGeometryMm) << where;
    EXPECT_NEAR(actual.yaw_deg, expected.at("yaw_deg").get<double>(), kGeometryDeg) << where;
    EXPECT_NEAR(actual.side_mm, expected.at("side_mm").get<double>(), kGeometryMm) << where;
  }
}

TEST(PythonParity, EstimateMarkerPoseWithDistortion) {
  const double side_mm = parity().at("marker_side_mm");
  for (const json& c : parity().at("pose_cases")) {
    const std::string where = c.at("name");
    const MarkerPose actual = estimate_marker_pose(corners_of(c.at("corners_px")), side_mm, distorted_camera(), distortion());
    const MarkerPose expected = pose_of(c.at("expected"));
    // solvePnP는 수렴 허용치까지만 반복하므로, 주차 판정이 실제로 의존하는 정밀도로 비교한다.
    EXPECT_LT(cv::norm(actual.rotation - expected.rotation, cv::NORM_INF), 1e-6) << where;
    EXPECT_LT(cv::norm(actual.translation_mm - expected.translation_mm, cv::NORM_INF), 1e-4) << where;
  }
}

// 주차 위치 파일을 읽어 다시 투영한 기준 위치와 고정 마커·전방 축이 Python과 같다.
TEST(PythonParity, LoadSpotReference) {
  const ParkingSpot spot = fixture_spot();
  const json& expected = parity().at("spot").at("reference");
  EXPECT_NEAR(spot.reference.x_mm, expected.at("x_mm").get<double>(), kGeometryMm);
  EXPECT_NEAR(spot.reference.y_mm, expected.at("y_mm").get<double>(), kGeometryMm);
  EXPECT_NEAR(spot.reference.yaw_deg, expected.at("yaw_deg").get<double>(), kGeometryDeg);
  EXPECT_NEAR(spot.reference.side_mm, expected.at("side_mm").get<double>(), kGeometryMm);
  EXPECT_EQ(spot.anchors.size(), 2u);
  EXPECT_EQ(spot.marker_forward_axis, "-x");
}

// 프레임별 관측. 같은 프레임을 여러 주차 위치 변형(시야 확인 끔, 밀림 한계 3 px, 고정 마커 하나, 그 하나를 신뢰)으로
// 관측해서 blocking/skipped 상태, reason, 오프셋, 밀림, 꼭짓점이 모두 같은지 본다.
TEST(PythonParity, ObserveFrame) {
  const ParkingSpot registered = fixture_spot();
  ParkingSpot view_check_off = registered;
  view_check_off.anchors.clear();
  ParkingSpot max_view_shift_3 = registered;
  max_view_shift_3.max_view_shift_px = 3.0;
  ParkingSpot single_anchor = registered;
  single_anchor.anchors = {{1, registered.anchors.at(1)}};
  ParkingSpot single_anchor_trusted = single_anchor;
  single_anchor_trusted.single_anchor_trusted = true;
  const std::map<std::string, const ParkingSpot*> spots = {{"registered", &registered},
                                                           {"view_check_off", &view_check_off},
                                                           {"max_view_shift_3", &max_view_shift_3},
                                                           {"single_anchor", &single_anchor},
                                                           {"single_anchor_trusted", &single_anchor_trusted}};

  for (const json& c : parity().at("frame_cases")) {
    const std::string where = c.at("name");
    const FrameObservation actual = observe_frame(frame(c.at("image")), *spots.at(c.at("spot")));
    const json& expected = c.at("expected");
    EXPECT_EQ(actual.blocking_status, status_of(expected.at("blocking_status"))) << where;
    EXPECT_EQ(to_string(actual.skipped_status), expected.at("skipped_status").get<std::string>()) << where;
    expect_reason(actual.reason, expected.at("reason"), where);
    expect_offset(actual.offset, expected.at("offset"), where);
    expect_optional_near(actual.view_shift_px, expected.at("view_shift_px"), detection().px, where);
    if (expected.at("view_shift_xy_px").is_null()) {
      EXPECT_FALSE(actual.view_shift_xy_px) << where;
    } else {
      ASSERT_TRUE(actual.view_shift_xy_px) << where;
      EXPECT_NEAR(actual.view_shift_xy_px->x, expected.at("view_shift_xy_px").at(0).get<double>(), detection().px) << where;
      EXPECT_NEAR(actual.view_shift_xy_px->y, expected.at("view_shift_xy_px").at(1).get<double>(), detection().px) << where;
    }
    if (expected.at("corners").is_null()) {
      EXPECT_FALSE(actual.corners) << where;
    } else {
      ASSERT_TRUE(actual.corners) << where;
      const Corners corners = corners_of(expected.at("corners"));
      for (std::size_t i = 0; i < corners.size(); ++i) {
        EXPECT_NEAR(cv::norm((*actual.corners)[i] - corners[i]), 0.0, detection().px) << where << " corner " << i;
      }
    }
  }
}

// 프레임을 순서대로 ParkingJudge에 넣었을 때 단계마다 결과(중앙값, 연속 누락 처리, blocking 초기화)가 같다.
TEST(PythonParity, ParkingJudgeSequence) {
  const ParkingSpot spot = fixture_spot();
  const json& sequence = parity().at("judge_sequence");
  ParkingJudge judgment(spot.tolerance, sequence.at("frames"));
  int step = 0;
  for (const json& s : sequence.at("steps")) {
    const std::string where = "step " + std::to_string(step++) + " " + s.at("image").get<std::string>();
    const ParkingResult actual = judgment.add(observe_frame(frame(s.at("image")), spot));
    const json& expected = s.at("expected");
    EXPECT_EQ(to_string(actual.status), expected.at("status").get<std::string>()) << where;
    expect_reason(actual.reason, expected.at("reason"), where);
    expect_offset(actual.offset, expected.at("offset"), where);
    expect_optional_near(actual.view_shift_px, expected.at("view_shift_px"), detection().px, where);
    if (expected.at("deviation_level").is_null()) {
      EXPECT_FALSE(actual.deviation_level) << where;
    } else {
      ASSERT_TRUE(actual.deviation_level) << where;
      EXPECT_EQ(to_string(*actual.deviation_level), expected.at("deviation_level").get<std::string>()) << where;
    }
  }
}

// Python payload는 오프셋을 0.1 mm / 0.01°로, 시야 밀림을 0.1 px로 반올림했다. 그래서 반올림 폭의 절반(0.05 등)을
// 허용 차이에 더한다.
void expect_payload_fields(const ParkingResult* judged, std::optional<double> view_shift_px, const std::string& reason,
                           const json& expected, const std::string& where) {
  expect_reason(reason, expected.at("reason"), where);
  const json& offset = expected.at("offset");
  if (offset.is_null()) {
    EXPECT_TRUE(judged == nullptr || !judged->offset) << where;
  } else {
    ASSERT_TRUE(judged != nullptr && judged->offset) << where;
    EXPECT_NEAR(judged->offset->dx_mm, offset.at("dx_mm").get<double>(), 0.05 + 1e-9 + detection().mm) << where;
    EXPECT_NEAR(judged->offset->dy_mm, offset.at("dy_mm").get<double>(), 0.05 + 1e-9 + detection().mm) << where;
    EXPECT_NEAR(judged->offset->dyaw_deg, offset.at("dyaw_deg").get<double>(), 0.005 + 1e-9 + detection().deg) << where;
  }
  const json& level = expected.at("deviation_level");
  if (level.is_null()) {
    EXPECT_TRUE(judged == nullptr || !judged->deviation_level) << where;
  } else {
    ASSERT_TRUE(judged != nullptr && judged->deviation_level) << where;
    EXPECT_EQ(to_string(*judged->deviation_level), level.get<std::string>()) << where;
  }
  const json& shift = expected.at("view_shift_px");
  if (shift.is_null()) {
    EXPECT_FALSE(view_shift_px) << where;
  } else {
    ASSERT_TRUE(view_shift_px) << where;
    EXPECT_NEAR(*view_shift_px, shift.get<double>(), 0.05 + 1e-9 + detection().px) << where;
  }
}

// Python payload의 image_overlay 점은 0.1 px로 반올림했다.
void expect_overlay(const std::optional<Corners>& corners, const std::optional<cv::Point2d>& view_shift_xy,
                    const json& expected, const std::string& where) {
  const double tolerance = 0.05 + 1e-9 + detection().px;
  const json& expected_corners = expected.at("marker_corners_px");
  if (expected_corners.is_null()) {
    EXPECT_FALSE(corners) << where;
  } else {
    ASSERT_TRUE(corners) << where;
    for (std::size_t i = 0; i < corners->size(); ++i) {
      EXPECT_NEAR((*corners)[i].x, expected_corners.at(i).at(0).get<double>(), tolerance) << where << " corner " << i;
      EXPECT_NEAR((*corners)[i].y, expected_corners.at(i).at(1).get<double>(), tolerance) << where << " corner " << i;
    }
  }
  const json& expected_shift = expected.at("view_shift_xy_px");
  if (expected_shift.is_null()) {
    EXPECT_FALSE(view_shift_xy) << where;
  } else {
    ASSERT_TRUE(view_shift_xy) << where;
    EXPECT_NEAR(view_shift_xy->x, expected_shift.at(0).get<double>(), tolerance) << where;
    EXPECT_NEAR(view_shift_xy->y, expected_shift.at(1).get<double>(), tolerance) << where;
  }
}

// 기대값 PNG는 흑백이다. 실제 feed처럼 BGR로 바꿔서 넣는다.
cv::Mat bgr_frame(const json& image) {
  cv::Mat color;
  cv::cvtColor(frame(image), color, cv::COLOR_GRAY2BGR);
  return color;
}

// 요청 이후 수집 시나리오들. "before:" 접두사가 붙은 프레임은 요청 시각(100 s)보다 앞선 50 s에 찍은 것으로 넣어
// 버려지는지 보고, 나머지는 요청 뒤 0.01 s 간격으로 넣는다. 결과 상태, 사용·검토 프레임 수, sequence 범위, 오류 코드가
// Python과 같아야 한다.
TEST(PythonParity, ParkingObservationCollector) {
  const ParkingSpot spot = fixture_spot();
  constexpr double kRequestedAt = 100.0;
  for (const json& c : parity().at("observation_cases")) {
    const std::string where = c.at("name");
    const json& expected = c.at("expected");
    ParkingObservationCollector collector(spot, kRequestedAt);
    std::uint64_t sequence = 0;
    try {
      for (const json& entry : c.at("frames")) {
        std::string name = entry;
        const bool stale = name.rfind("before:", 0) == 0;
        if (stale) name = name.substr(7);
        ++sequence;
        collector.add(sequence, stale ? 50.0 : kRequestedAt + 0.01 * static_cast<double>(sequence),
                      bgr_frame(json("frames/" + name + ".png")));
      }
      const ParkingObservation observation = collector.finish();
      ASSERT_FALSE(expected.contains("error_code")) << where;
      EXPECT_EQ(to_string(observation.result.status), expected.at("status").get<std::string>()) << where;
      expect_payload_fields(&observation.result, observation.result.view_shift_px, observation.result.reason, expected,
                            where);
      EXPECT_EQ(observation.frames_used, expected.at("frames_used").get<int>()) << where;
      EXPECT_EQ(observation.frames_examined, expected.at("frames_examined").get<int>()) << where;
      EXPECT_EQ(observation.first_sequence, expected.at("frame_sequence_range").at(0).get<std::uint64_t>()) << where;
      EXPECT_EQ(observation.last_sequence, expected.at("frame_sequence_range").at(1).get<std::uint64_t>()) << where;
      expect_overlay(observation.marker_corners_px, observation.view_shift_xy_px, expected, where);
    } catch (const ParkingObservationError& error) {
      ASSERT_TRUE(expected.contains("error_code")) << where << ": " << error.what();
      EXPECT_EQ(error.error_code(), expected.at("error_code").get<std::string>()) << where;
    }
  }
}

// 추적기 틱 시퀀스(feed가 frame / empty / 그 밖이면 카메라 없음)를 그대로 재생해서 틱마다 상태, sequence, 판정 값,
// 화면 그림 값이 Python 추적기와 같은지 본다.
TEST(PythonParity, ParkingTrackerTicks) {
  ParkingTracker tracker;
  tracker.set_spot(std::make_shared<const ParkingSpot>(fixture_spot()));
  for (const json& tick : parity().at("tracker_ticks")) {
    const double now = tick.at("now");
    const std::string feed = tick.at("feed");
    const std::string where = "t=" + std::to_string(now) + " " + feed + " " + tick.at("image").dump();
    const LiveSnapshot& snapshot =
        feed == "frame"   ? tracker.on_frame(now, tick.at("sequence"), bgr_frame(tick.at("image")))
        : feed == "empty" ? tracker.on_no_frame(now)
                          : tracker.on_camera_unavailable(now, "카메라 feed가 없습니다: head_camera");
    const json& expected = tick.at("expected");
    EXPECT_EQ(to_string(snapshot.state), expected.at("state").get<std::string>()) << where;
    if (expected.at("frame_sequence").is_null()) {
      EXPECT_FALSE(snapshot.frame_sequence) << where;
    } else {
      EXPECT_EQ(snapshot.frame_sequence, expected.at("frame_sequence").get<std::uint64_t>()) << where;
    }
    expect_payload_fields(snapshot.judged ? &*snapshot.judged : nullptr, snapshot.view_shift_px, snapshot.reason,
                          expected, where);
    expect_overlay(snapshot.marker_corners_px, snapshot.view_shift_xy_px, expected, where);
  }
}

}  // namespace
}  // namespace parking_vision
