// 예전 tests/test_parking_check.py를 옮긴 것이다. 같은 동작을 C++ 구현에서 확인한다.
//
// 위치: parking_vision 라이브러리 단위 테스트(gtest, parking_vision_tests 실행 파일)다.
// 확인하는 것:
// - PlaneProjection: 코너 광선을 등록 평면에 교차시키는 평면 투영이 기준 마커 축 기준 dx/dy(mm)·dyaw(deg)를
//   정확히 내는지. 기울어진 카메라, 등록 평면 깊이 오차, 픽셀에서 추정한 자세까지 다룬다.
// - ObserveFrame: 렌더링한 영상 한 장에서 고정 마커로 시야를 확인하는 규칙. 작은 카메라 밀림은 빼고,
//   96 px 감김·같은 ID 중복·평면을 벗어난 마커는 판정하지 않는다(VIEW_UNVERIFIED / AMBIGUOUS / 거부).
// - Judge·ParkingJudge: 허용 오차 비율과 WITHIN/NEAR/FAR 단계, 여러 프레임 중앙값 판정과 놓친 프레임 처리.
// - LoadSpot: 주차 위치 JSON(schema 2) 검증 규칙과 SHA-256 기록.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "test_support.hpp"

namespace parking_vision::test {
namespace {

// reference와 current 자세의 마커를 가상 카메라로 찍은 뒤, 둘 다 plane 평면에 투영해서 그 차이를 잰다.
// plane을 일부러 틀리게 주면 등록 평면 오차가 결과에 얼마나 새는지 볼 수 있다.
PlanarOffset measure(const MarkerPose& plane, const MarkerPose& reference, const MarkerPose& current) {
  const auto on_plane = [&](const MarkerPose& pose) {
    return project_to_plane(project_corners(pose), plane, kCamera, kNoDistortion);
  };
  return plane_offset(on_plane(reference), on_plane(current));
}

TEST(PlaneProjection, SameCornersAreZero) {
  const MarkerPose pose = floor_pose(0.0, 0.0, 15.0);
  const PlanarOffset offset = measure(pose, pose, pose);
  EXPECT_NEAR(offset.dx_mm, 0.0, 1e-6);
  EXPECT_NEAR(offset.dy_mm, 0.0, 1e-6);
  EXPECT_NEAR(offset.dyaw_deg, 0.0, 1e-6);
}

TEST(PlaneProjection, ShiftAndYawInReferenceAxes) {
  const MarkerPose reference = floor_pose(0.0, 0.0, 0.0);
  const PlanarOffset offset = measure(reference, reference, floor_pose(15.0, -30.0, 8.0));
  EXPECT_NEAR(offset.dx_mm, 15.0, 1e-4);
  EXPECT_NEAR(offset.dy_mm, -30.0, 1e-4);
  EXPECT_NEAR(offset.dyaw_deg, 8.0, 1e-4);
}

TEST(PlaneProjection, ShiftIsExpressedInReferenceMarkerAxes) {
  // 기준 마커가 반시계로 90° 돌아 있으므로, 그 오른쪽 변(+x)은 바닥 +y 방향을 가리킨다.
  // 바닥 +y로 50 mm 옮기면 기준 마커 축에서는 dx = +50 mm로 읽혀야 한다.
  const MarkerPose reference = floor_pose(0.0, 0.0, 90.0);
  const PlanarOffset offset = measure(reference, reference, floor_pose(0.0, 50.0, 90.0));
  EXPECT_NEAR(offset.dx_mm, 50.0, 1e-4);
  EXPECT_NEAR(offset.dy_mm, 0.0, 1e-4);
}

// 카메라가 26° 기울어도 바닥 평면 안의 이동·회전을 그대로 되찾는지 확인한다.
TEST(PlaneProjection, TiltedCamera) {
  const MarkerPose reference = floor_pose(0.0, 0.0, 0.0, 26.0);
  const PlanarOffset offset = measure(reference, reference, floor_pose(-12.0, 90.0, -3.0, 26.0));
  EXPECT_NEAR(offset.dx_mm, -12.0, 1e-4);
  EXPECT_NEAR(offset.dy_mm, 90.0, 1e-4);
  EXPECT_NEAR(offset.dyaw_deg, -3.0, 1e-4);
}

// 평면 위에서 잰 마커 한 변이 인쇄된 길이(70 mm)와 같아야 한다. observe_frame의 "평면 위 마커인가" 검사의 전제다.
TEST(PlaneProjection, MeasuredSideOnPlane) {
  const MarkerPose pose = floor_pose(20.0, 10.0, 30.0, 26.0);
  EXPECT_NEAR(project_to_plane(project_corners(pose), pose, kCamera, kNoDistortion).side_mm, kSideMm, 1e-4);
}

TEST(PlaneProjection, PlaneDepthErrorScalesOffsetOnlySlightly) {
  // 등록 평면이 3% 멀게 잡혀도 오프셋이 3% 커질 뿐이고, 수십 mm짜리 고정 편향이 더해지지는 않는다.
  const MarkerPose reference = floor_pose(0.0, 0.0, 0.0, 26.0);
  const MarkerPose biased{reference.rotation, reference.translation_mm * 1.03};
  const PlanarOffset offset = measure(biased, reference, floor_pose(0.0, 90.0, 0.0, 26.0));
  EXPECT_NEAR(offset.dx_mm, 0.0, 0.5);
  EXPECT_NEAR(offset.dy_mm, 90.0 * 1.03, 0.5);
}

// 실제 등록처럼 기준 평면을 픽셀 코너에서 PnP로 추정해도 0.5 mm, 0.2° 안에서 오프셋을 되찾는다.
TEST(PlaneProjection, RegisteredPoseFromPixels) {
  const MarkerPose reference = floor_pose(0.0, 0.0, 0.0, 26.0);
  const MarkerPose plane = estimate_marker_pose(project_corners(reference), kSideMm, kCamera, kNoDistortion);
  const PlanarOffset offset = measure(plane, reference, floor_pose(15.0, -30.0, 8.0, 26.0));
  EXPECT_NEAR(offset.dx_mm, 15.0, 0.5);
  EXPECT_NEAR(offset.dy_mm, -30.0, 0.5);
  EXPECT_NEAR(offset.dyaw_deg, 8.0, 0.2);
}

// 기본 장면: 21번이 Pinky 마커, 1번과 3번이 바닥에 고정된 마커다(값은 마커 좌상단 픽셀, 한 변 60 px).
const std::map<int, std::pair<int, int>> kScene = {{21, {420, 140}}, {1, {80, 60}}, {3, {120, 360}}};

// scene에서 마커 id 하나의 위치만 바꾼 사본을 돌려준다.
std::map<int, std::pair<int, int>> with(std::map<int, std::pair<int, int>> scene, int id, std::pair<int, int> xy) {
  scene[id] = xy;
  return scene;
}

// 기본 장면 그대로를 등록한 주차 위치에 대해 프레임 한 장씩 관측해 본다.
class ObserveFrame : public ::testing::Test {
 protected:
  ParkingSpot spot = spot_from_image(render_scene(kScene));
};

TEST_F(ObserveFrame, RegisteredFrameIsZeroOffset) {
  const FrameObservation observation = observe_frame(render_scene(kScene), spot);
  EXPECT_FALSE(observation.blocking_status);
  ASSERT_TRUE(observation.offset);
  EXPECT_NEAR(observation.offset->distance_mm(), 0.0, 0.5);
  EXPECT_NEAR(observation.view_shift_px.value_or(0.0), 0.0, 0.5);
}

// Pinky 마커만 20 px 옮기면 10 mm 넘는 오프셋으로 잡혀야 한다.
TEST_F(ObserveFrame, MovedMarkerIsMeasured) {
  const FrameObservation observation = observe_frame(render_scene(with(kScene, 21, {440, 140})), spot);
  ASSERT_TRUE(observation.offset);
  EXPECT_GT(observation.offset->distance_mm(), 10.0);
}

// 화면 전체가 아래 2 px, 오른쪽 1 px 밀리면 고정 마커로 그 밀림을 재서 빼므로 오프셋은 0에 가깝다.
TEST_F(ObserveFrame, SmallViewDriftIsSubtracted) {
  const FrameObservation observation = observe_frame(roll(render_scene(kScene), 2, 1), spot);
  EXPECT_FALSE(observation.blocking_status);
  ASSERT_TRUE(observation.offset);
  EXPECT_NEAR(observation.offset->distance_mm(), 0.0, 0.5);
  EXPECT_NEAR(observation.view_shift_px.value_or(0.0), std::hypot(2.0, 1.0), 0.3);
}

TEST_F(ObserveFrame, CameraDriftWithinTheSpotLimitIsSubtracted) {
  // 천장 거치대는 한 시간 안에 5 px 밀린 적이 있다. 그 정도로 제자리에 선 Pinky를 막으면 안 된다.
  // 여기서는 아래 10 px, 왼쪽 4 px(약 10.8 px)로 기본 한계 15 px 안쪽이다.
  const FrameObservation observation = observe_frame(roll(render_scene(kScene), 10, -4), spot);
  EXPECT_FALSE(observation.blocking_status);
  ASSERT_TRUE(observation.offset);
  EXPECT_NEAR(observation.offset->distance_mm(), 0.0, 0.5);
  ASSERT_TRUE(observation.view_shift_xy_px);
  EXPECT_NEAR(observation.view_shift_xy_px->x, -4.0, 0.3);
  EXPECT_NEAR(observation.view_shift_xy_px->y, 10.0, 0.3);
}

// 기본값 15 px가 아니라 주차 위치에 기록된 max_view_shift_px(여기서는 3 px)를 쓰는지 확인한다.
TEST_F(ObserveFrame, SpotLimitIsUsed) {
  spot.max_view_shift_px = 3.0;
  EXPECT_EQ(observe_frame(roll(render_scene(kScene), 10, 0), spot).blocking_status, ParkingStatus::ViewUnverified);
}

TEST_F(ObserveFrame, WrappedFrameIsNotJudged) {
  // 천장 카메라는 가끔 화면 전체가 옆으로 96 px 감겨 돌아간 상태로 열린다. 이런 프레임은 판정하지 않는다.
  const FrameObservation observation = observe_frame(roll(render_scene(kScene), 0, 96), spot);
  EXPECT_EQ(observation.blocking_status, ParkingStatus::ViewUnverified);
  EXPECT_FALSE(observation.offset);
  EXPECT_NEAR(observation.view_shift_px.value_or(0.0), 96.0, 1.0);
}

// 고정 마커가 하나도 안 보이는 프레임은 막지(blocking) 않고 건너뛴다. 이전 프레임은 그대로 살아 있다.
TEST_F(ObserveFrame, NoFixedMarkerFrameIsSkipped) {
  const FrameObservation observation = observe_frame(render_scene({{21, kScene.at(21)}}), spot);
  EXPECT_FALSE(observation.blocking_status);
  EXPECT_FALSE(observation.offset);
  EXPECT_EQ(observation.skipped_status, ParkingStatus::ViewUnverified);
}

// 고정 마커가 하나만 보여도, 밀림이 없으면(3 px 이하) 판정한다.
TEST_F(ObserveFrame, OneFixedMarkerIsEnough) {
  const FrameObservation observation = observe_frame(render_scene({{21, kScene.at(21)}, {1, kScene.at(1)}}), spot);
  EXPECT_FALSE(observation.blocking_status);
  EXPECT_TRUE(observation.offset);
}

// 같은 ID(21)가 두 번 보이면 어느 쪽이 Pinky인지 모르므로 AMBIGUOUS다.
// 흰 바탕 두 장면의 bitwise_and는 검은 마커를 합친 영상이 된다.
TEST_F(ObserveFrame, DuplicateMarkerIsAmbiguous) {
  cv::Mat image;
  cv::bitwise_and(render_scene(kScene), render_scene({{21, {300, 300}}}), image);
  EXPECT_EQ(observe_frame(image, spot).blocking_status, ParkingStatus::Ambiguous);
}

TEST_F(ObserveFrame, MarkerOffThePlaneIsRejected) {
  // 영상에서 더 큰 마커는 등록 평면 위에서도 더 긴 변으로 읽힌다(예: Pinky를 들어 올린 경우).
  // 60 px 대신 80 px로 그리면 약 93 mm로 읽혀 인쇄 크기 70 mm의 10% 한계를 넘는다.
  cv::Mat image;
  cv::bitwise_and(render_scene({{1, kScene.at(1)}, {3, kScene.at(3)}}), render_scene({{21, kScene.at(21)}}, 80),
                  image);
  const FrameObservation observation = observe_frame(image, spot);
  EXPECT_FALSE(observation.offset);
  EXPECT_NE(observation.reason.find("marker side"), std::string::npos);
}

// 시야 확인을 끈 주차 위치(anchors 비어 있음)는 Pinky 마커만 보고 판정하며 view_shift_px도 없다.
TEST_F(ObserveFrame, ViewCheckOffJudgesPinkyMarkerOnly) {
  spot.anchors.clear();
  const FrameObservation observation = observe_frame(render_scene({{21, kScene.at(21)}}), spot);
  ASSERT_TRUE(observation.offset);
  EXPECT_NEAR(observation.offset->distance_mm(), 0.0, 0.5);
  EXPECT_FALSE(observation.view_shift_px);
}

TEST_F(ObserveFrame, ViewCheckOffMissesWrappedFrame) {
  // 알려진 한계: 고정 마커가 없으면 감긴 화면을 Pinky가 움직인 것처럼 측정한다.
  spot.anchors.clear();
  const FrameObservation observation = observe_frame(roll(render_scene(kScene), 0, 96), spot);
  EXPECT_FALSE(observation.blocking_status);
  ASSERT_TRUE(observation.offset);
  EXPECT_GT(observation.offset->distance_mm(), 50.0);
}

// (170, 140)에서 시작하는 200 px 마커의 중심은 픽셀 중심 기준 (269.5, 239.5)다.
TEST(DetectMarkers, DetectsRenderedMarker) {
  const MarkerDetections found = detect_markers(render_scene({{20, {170, 140}}}, 200), "DICT_4X4_50");
  ASSERT_EQ(found.size(), 1u);
  ASSERT_EQ(found.begin()->first, 20);
  const Corners& c = found.at(20).front();
  const cv::Point2d center = (c[0] + c[1] + c[2] + c[3]) / 4.0;
  EXPECT_NEAR(center.x, 269.5, 1.0);
  EXPECT_NEAR(center.y, 239.5, 1.0);
}

TEST(DetectMarkers, UnknownDictionaryIsRejected) {
  EXPECT_THROW(detect_markers(render_scene({}), "DICT_UNKNOWN"), std::invalid_argument);
}

// 중심이 (1, 1) px인 2 px 정사각형이다. 고정 마커 밀림 계산만 시험하는 가짜 검출 결과에 쓴다.
const Corners kSquare = {cv::Point2d(0.0, 0.0), cv::Point2d(2.0, 0.0), cv::Point2d(2.0, 2.0), cv::Point2d(0.0, 2.0)};

// 코너 네 개를 모두 by만큼 옮긴다.
Corners moved(const Corners& corners, cv::Point2d by) {
  return {corners[0] + by, corners[1] + by, corners[2] + by, corners[3] + by};
}

TEST(AnchorShift, NoneWhenNoAnchorVisible) { EXPECT_FALSE(anchor_shift_px({{1, {10.0, 10.0}}}, {})); }

// 같은 고정 마커 ID가 두 번 보이면 어느 쪽이 진짜인지 모르므로 밀림 계산에서 뺀다.
TEST(AnchorShift, DuplicatedAnchorIsIgnored) {
  EXPECT_FALSE(anchor_shift_px({{1, {1.0, 1.0}}}, {{1, {kSquare, moved(kSquare, {50.0, 50.0})}}}));
}

// 밀림 (0, 0), (1, -2), (40, -3)의 x·y 중앙값은 (1, -2)다. 혼자 크게 움직인 마커 하나에 끌려가지 않는다.
TEST(AnchorShift, MedianOfVisibleAnchors) {
  const std::map<int, cv::Point2d> anchors = {{1, {1.0, 1.0}}, {2, {1.0, 1.0}}, {3, {1.0, 1.0}}};
  const MarkerDetections found = {
      {1, {kSquare}}, {2, {moved(kSquare, {1.0, -2.0})}}, {3, {moved(kSquare, {40.0, -3.0})}}};
  const auto shift = anchor_shift_px(anchors, found);
  ASSERT_TRUE(shift);
  EXPECT_DOUBLE_EQ(shift->x, 1.0);
  EXPECT_DOUBLE_EQ(shift->y, -2.0);
}

TEST(Judge, NotFound) { EXPECT_EQ(judge(std::nullopt, kTolerance).status, ParkingStatus::NotFound); }

TEST(Judge, ArrivedInsideBothTolerances) {
  EXPECT_EQ(judge(PlanarOffset{10.0, -12.0, 9.0}, kTolerance).status, ParkingStatus::Arrived);
}

// (15, 15) mm는 거리 약 21.2 mm라 20 mm를 넘는다. reason 문자열은 Python과 같은 형식이어야 한다.
TEST(Judge, MisalignedPosition) {
  const ParkingResult result = judge(PlanarOffset{15.0, 15.0, 0.0}, kTolerance);
  EXPECT_EQ(result.status, ParkingStatus::Misaligned);
  EXPECT_EQ(result.reason, "position 21 mm > 20 mm");
}

TEST(Judge, MisalignedYaw) {
  const ParkingResult result = judge(PlanarOffset{0.0, 0.0, -11.0}, kTolerance);
  EXPECT_EQ(result.status, ParkingStatus::Misaligned);
  EXPECT_EQ(result.reason, "yaw -11.0 deg > ±10 deg");
}

TEST(Judge, RatiosAndLevels) {
  // 허용 오차 20 mm / 10°: 두 비율 중 나쁜 쪽이 단계를 정한다.
  // 비율이 정확히 1.0이면 WITHIN(경계 포함), 2.0까지 NEAR, 그보다 크면 FAR다.
  struct Case {
    PlanarOffset offset;
    DeviationLevel level;
    double position_ratio;
    double yaw_ratio;
  };
  const Case cases[] = {
      {{12.0, 16.0, 5.0}, DeviationLevel::Within, 1.0, 0.5},
      {{30.0, 0.0, 0.0}, DeviationLevel::Near, 1.5, 0.0},
      {{0.0, 0.0, -20.0}, DeviationLevel::Near, 0.0, 2.0},
      {{0.0, 41.0, 0.0}, DeviationLevel::Far, 2.05, 0.0},
  };
  for (const Case& c : cases) {
    const ParkingResult result = judge(c.offset, kTolerance);
    EXPECT_EQ(result.deviation_level, c.level);
    EXPECT_DOUBLE_EQ(result.position_ratio.value_or(-1.0), c.position_ratio);
    EXPECT_DOUBLE_EQ(result.yaw_ratio.value_or(-1.0), c.yaw_ratio);
    EXPECT_EQ(result.status == ParkingStatus::Arrived, c.level == DeviationLevel::Within);
  }
}

TEST(Judge, NonFiniteOffsetIsNotFound) {
  // NaN은 어떤 한계와 비교해도 false라서, 따로 막지 않으면 ARRIVED로 읽힐 수 있다.
  for (const PlanarOffset& offset : {PlanarOffset{std::nan(""), 0.0, 0.0}, PlanarOffset{0.0, 0.0, std::nan("")},
                                     PlanarOffset{INFINITY, 0.0, 0.0}}) {
    const ParkingResult result = judge(offset, kTolerance);
    EXPECT_EQ(result.status, ParkingStatus::NotFound);
    EXPECT_FALSE(result.offset);
    EXPECT_EQ(result.reason, "non-finite offset (marker projection failed)");
  }
}

TEST(Judge, NotFoundHasNoLevel) { EXPECT_FALSE(judge(std::nullopt, kTolerance).deviation_level); }

TEST(PinkyFrame, MarkerRightEdgeIsFront) {
  // 마커 +x가 앞, +y가 왼쪽, +z가 위: Pinky 자신의 축이 마커 축과 같다.
  // pinky_frame_offset은 (앞, 왼쪽) mm를 돌려준다.
  EXPECT_EQ(pinky_frame_offset({30.0, -5.0, 0.0}, "+x"), std::make_pair(30.0, -5.0));
}

TEST(PinkyFrame, MarkerTopEdgeIsFront) {
  // 마커 +y가 앞이면 마커 -x가 Pinky의 왼쪽이 된다.
  EXPECT_EQ(pinky_frame_offset({30.0, -5.0, 0.0}, "+y"), std::make_pair(-5.0, -30.0));
}

TEST(PinkyFrame, MarkerLeftEdgeIsFront) {
  EXPECT_EQ(pinky_frame_offset({30.0, -5.0, 0.0}, "-x"), std::make_pair(-30.0, 5.0));
}

// ParkingJudge 테스트용 가짜 관측: dx_mm만큼 어긋난 마커가 보인 프레임.
FrameObservation seen(double dx_mm) {
  FrameObservation observation;
  observation.offset = PlanarOffset{dx_mm, 0.0, 0.0};
  observation.view_shift_px = 0.0;
  return observation;
}

// ParkingJudge 테스트용 가짜 관측: Pinky 마커를 놓친 프레임.
FrameObservation missed() {
  FrameObservation observation;
  observation.reason = "marker not visible";
  observation.view_shift_px = 0.0;
  return observation;
}

// 한두 프레임 놓쳐도 최근 값으로 판정을 유지한다.
TEST(ParkingJudgeTest, BriefMissKeepsRecentFrames) {
  ParkingJudge judgment(kTolerance, 3);
  judgment.add(seen(1.0));
  EXPECT_EQ(judgment.add(missed()).status, ParkingStatus::Arrived);
}

// 창 크기(frames)만큼 연속으로 놓치면 이전 값을 버리고 NOT_FOUND가 된다.
TEST(ParkingJudgeTest, MissesForAllFramesDropOldValues) {
  ParkingJudge judgment(kTolerance, 2);
  judgment.add(seen(1.0));
  judgment.add(missed());
  EXPECT_EQ(judgment.add(missed()).status, ParkingStatus::NotFound);
}

// 고정 마커가 잠깐 안 보인 프레임도 놓친 프레임처럼 다룬다. 남은 값이 없어지면 그때 skipped_status
// (VIEW_UNVERIFIED)를 보고한다.
TEST(ParkingJudgeTest, FixedMarkerDropoutKeepsRecentFrames) {
  ParkingJudge judgment(kTolerance, 2);
  FrameObservation unverified;
  unverified.reason = "no fixed marker";
  unverified.skipped_status = ParkingStatus::ViewUnverified;
  judgment.add(seen(1.0));
  EXPECT_EQ(judgment.add(unverified).status, ParkingStatus::Arrived);
  EXPECT_EQ(judgment.add(unverified).status, ParkingStatus::ViewUnverified);
}

// 시야가 바뀐 프레임(blocking_status)은 이전 프레임까지 무효로 만든다.
TEST(ParkingJudgeTest, ViewChangeClearsEarlierFrames) {
  ParkingJudge judgment(kTolerance, 3);
  judgment.add(seen(1.0));
  FrameObservation moved_view;
  moved_view.reason = "moved";
  moved_view.blocking_status = ParkingStatus::ViewUnverified;
  moved_view.view_shift_px = 96.0;
  const ParkingResult result = judgment.add(moved_view);
  EXPECT_EQ(result.status, ParkingStatus::ViewUnverified);
  EXPECT_TRUE(judgment.recent().empty());
  EXPECT_EQ(result.view_shift_px, 96.0);
}

// 창 크기 2: 처음의 100 mm는 밀려나고 1, 2 mm의 중앙값으로 ARRIVED가 된다.
TEST(ParkingJudgeTest, KeepsOnlyLatestFrames) {
  ParkingJudge judgment(kTolerance, 2);
  ParkingResult result = judgment.result();
  for (const double dx : {100.0, 1.0, 2.0}) result = judgment.add(seen(dx));
  EXPECT_EQ(result.status, ParkingStatus::Arrived);
}

// 179°, -179°, 178°를 단순 중앙값으로 하면 178°지만, 첫 값 기준으로 펴면 179°, 181°, 178°라서 중앙값은 ±179°다.
TEST(MedianOffset, YawMedianAcrossWraparound) {
  const PlanarOffset median = median_offset({{0, 0, 179.0}, {0, 0, -179.0}, {0, 0, 178.0}});
  EXPECT_NEAR(std::abs(median.dyaw_deg), 179.0, 1e-6);
}

namespace fs = std::filesystem;
using nlohmann::json;

// 테스트마다 임시 디렉터리에 내부 보정 파일과 주차 위치 파일을 직접 써서 load_spot의 검증 규칙을 확인한다.
class LoadSpot : public ::testing::Test {
 protected:
  void SetUp() override {
    directory = fs::temp_directory_path() /
                ("parking_vision_" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    fs::create_directories(directory);
  }
  void TearDown() override { fs::remove_all(directory); }

  static void write(const fs::path& path, const json& data) { std::ofstream(path) << data.dump(); }

  // 정상적인 schema 2 주차 위치 파일을 쓰고 경로를 돌려준다. extra의 키는 기본 내용을 덮어쓴다.
  // intrinsic_calibration.sha256은 방금 쓴 보정 파일의 해시라서 그대로 두면 통과한다.
  fs::path write_spot(const json& anchors, const json& view_check, const json& extra = json::object()) {
    const fs::path intrinsics = directory / "ceiling_camera.json";
    write(intrinsics, {{"camera_id", "ceiling_camera"},
                       {"pixel_transform_id", kPixelTransformId},
                       {"camera_matrix", {{980.0, 0.0, 320.0}, {0.0, 980.0, 240.0}, {0.0, 0.0, 1.0}}},
                       {"distortion_coefficients", kNoDistortion},
                       {"image_size_px", {{"width", 640}, {"height", 480}}}});
    const MarkerPose pose = floor_pose(0.0, 0.0, 0.0);
    json rotation = json::array();
    for (int r = 0; r < 3; ++r) rotation.push_back({pose.rotation(r, 0), pose.rotation(r, 1), pose.rotation(r, 2)});
    json corners = json::array();
    for (const auto& corner : project_corners(pose)) corners.push_back({corner.x, corner.y});
    json data = {{"schema_version", 2},
                 {"spot_id", "spot"},
                 {"camera_id", "ceiling_camera"},
                 {"intrinsic_calibration", {{"path", "ceiling_camera.json"}, {"sha256", sha256_of(intrinsics)}}},
                 {"aruco_dictionary", "DICT_4X4_50"},
                 {"marker_id", 21},
                 {"marker_side_mm", kSideMm},
                 {"reference_pose_in_camera",
                  {{"rotation", rotation},
                   {"translation_mm", {pose.translation_mm[0], pose.translation_mm[1], pose.translation_mm[2]}}}},
                 {"reference_corners_px", corners},
                 {"anchors", anchors},
                 {"view_check", view_check},
                 {"tolerance", {{"position_mm", 20.0}, {"yaw_deg", 2.0}}}};
    data.update(extra);
    const fs::path path = directory / "spot.json";
    write(path, data);
    return path;
  }

  // load_spot이 던진 ParkingSpotError 메시지를 돌려준다. 성공하면 빈 문자열이다.
  static std::string error_of(const fs::path& path, const fs::path& root) {
    try {
      load_spot(path, root);
    } catch (const ParkingSpotError& error) {
      return error.what();
    }
    return "";
  }

  fs::path directory;
};

TEST_F(LoadSpot, Schema1NeedsRegisteringAgain) {
  write(directory / "old.json", {{"schema_version", 1}});
  EXPECT_NE(error_of(directory / "old.json", directory).find("다시 등록"), std::string::npos);
}

TEST_F(LoadSpot, FixedMarkers) {
  const json anchors = {{{"marker_id", 3}, {"center_px", {500.0, 250.0}}}};
  const ParkingSpot spot =
      load_spot(write_spot(anchors, {{"mode", "FIXED_MARKERS"}, {"max_anchor_shift_px", 3.0}}), directory);
  EXPECT_EQ(spot.anchors.at(3), cv::Point2d(500.0, 250.0));
  EXPECT_EQ(spot.tolerance.position_mm, 20.0);
  EXPECT_EQ(spot.tolerance.yaw_deg, 2.0);
  EXPECT_EQ(spot.max_view_shift_px, 3.0);
  EXPECT_EQ(load_spot(write_spot(anchors, {{"mode", "FIXED_MARKERS"}}), directory).max_view_shift_px, 15.0);
}

// 고정 마커 없이 등록하려면 view_check.mode가 OFF로 기록돼 있어야 한다(실수로 빠진 것과 구분).
TEST_F(LoadSpot, ViewCheckOffMustBeRecorded) {
  EXPECT_TRUE(load_spot(write_spot(json::array(), {{"mode", "OFF"}}), directory).anchors.empty());
  EXPECT_NE(error_of(write_spot(json::array(), json::object()), directory).find("OFF"), std::string::npos);
}

TEST_F(LoadSpot, MarkerForwardAxis) {
  EXPECT_FALSE(load_spot(write_spot(json::array(), {{"mode", "OFF"}}), directory).marker_forward_axis);
  const ParkingSpot spot =
      load_spot(write_spot(json::array(), {{"mode", "OFF"}}, {{"marker_forward_axis", "+x"}}), directory);
  EXPECT_EQ(spot.marker_forward_axis, "+x");
  const fs::path bad = write_spot(json::array(), {{"mode", "OFF"}}, {{"marker_forward_axis", "front"}});
  EXPECT_NE(error_of(bad, directory).find("marker_forward_axis"), std::string::npos);
}

TEST_F(LoadSpot, ToleranceMustBePositive) {
  const fs::path path =
      write_spot(json::array(), {{"mode", "OFF"}}, {{"tolerance", {{"position_mm", 0}, {"yaw_deg", 10}}}});
  EXPECT_NE(error_of(path, directory).find("0보다"), std::string::npos);
}

TEST_F(LoadSpot, UnsupportedDictionaryIsRejected) {
  for (const json& dictionary : {json("DICT_UNKNOWN"), json("aruco"), json(5)}) {
    const fs::path path = write_spot(json::array(), {{"mode", "OFF"}}, {{"aruco_dictionary", dictionary}});
    EXPECT_NE(error_of(path, directory).find("지원하지 않는 ArUco 사전입니다"), std::string::npos) << dictionary;
  }
}

TEST_F(LoadSpot, SingleAnchorTrusted) {
  const json anchors = {{{"marker_id", 3}, {"center_px", {500.0, 250.0}}}};
  EXPECT_FALSE(load_spot(write_spot(anchors, {{"mode", "FIXED_MARKERS"}}), directory).single_anchor_trusted);
  EXPECT_TRUE(load_spot(write_spot(anchors, {{"mode", "FIXED_MARKERS"}, {"single_anchor_trusted", true}}), directory)
                  .single_anchor_trusted);
  const fs::path bad = write_spot(anchors, {{"mode", "FIXED_MARKERS"}, {"single_anchor_trusted", "yes"}});
  EXPECT_NE(error_of(bad, directory).find("single_anchor_trusted"), std::string::npos);
}

// 기록되는 SHA-256은 실제로 읽어 파싱한 바이트의 해시여야 한다.
TEST_F(LoadSpot, HashesTheBytesItParsed) {
  const fs::path path = write_spot(json::array(), {{"mode", "OFF"}});
  std::string registration_sha256;
  const ParkingSpot spot = load_spot(path, directory, &registration_sha256);
  EXPECT_EQ(registration_sha256, sha256_of(path));
  EXPECT_EQ(spot.intrinsics.sha256, sha256_of(directory / "ceiling_camera.json"));
}

// 등록 뒤 내부 보정 파일에 줄바꿈 하나만 붙여도 SHA-256이 달라져 거부된다.
TEST_F(LoadSpot, ChangedCalibrationIsRejected) {
  const fs::path path = write_spot(json::array(), {{"mode", "OFF"}});
  std::ofstream(directory / "ceiling_camera.json", std::ios::app) << "\n";
  EXPECT_NE(error_of(path, directory).find("내부 보정 파일이 바뀌었습니다"), std::string::npos);
}

}  // namespace
}  // namespace parking_vision::test
