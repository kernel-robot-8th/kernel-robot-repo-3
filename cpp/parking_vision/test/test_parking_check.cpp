// Port of the former tests/test_parking_check.py: the same behaviour checked on the C++ implementation.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "test_support.hpp"

namespace parking_vision::test {
namespace {

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
  // Reference turned 90° CCW, so its right edge (+x) points along the floor +y.
  const MarkerPose reference = floor_pose(0.0, 0.0, 90.0);
  const PlanarOffset offset = measure(reference, reference, floor_pose(0.0, 50.0, 90.0));
  EXPECT_NEAR(offset.dx_mm, 50.0, 1e-4);
  EXPECT_NEAR(offset.dy_mm, 0.0, 1e-4);
}

TEST(PlaneProjection, TiltedCamera) {
  const MarkerPose reference = floor_pose(0.0, 0.0, 0.0, 26.0);
  const PlanarOffset offset = measure(reference, reference, floor_pose(-12.0, 90.0, -3.0, 26.0));
  EXPECT_NEAR(offset.dx_mm, -12.0, 1e-4);
  EXPECT_NEAR(offset.dy_mm, 90.0, 1e-4);
  EXPECT_NEAR(offset.dyaw_deg, -3.0, 1e-4);
}

TEST(PlaneProjection, MeasuredSideOnPlane) {
  const MarkerPose pose = floor_pose(20.0, 10.0, 30.0, 26.0);
  EXPECT_NEAR(project_to_plane(project_corners(pose), pose, kCamera, kNoDistortion).side_mm, kSideMm, 1e-4);
}

TEST(PlaneProjection, PlaneDepthErrorScalesOffsetOnlySlightly) {
  // A registered plane 3% too far scales offsets by 3%, instead of adding a fixed tens-of-mm bias.
  const MarkerPose reference = floor_pose(0.0, 0.0, 0.0, 26.0);
  const MarkerPose biased{reference.rotation, reference.translation_mm * 1.03};
  const PlanarOffset offset = measure(biased, reference, floor_pose(0.0, 90.0, 0.0, 26.0));
  EXPECT_NEAR(offset.dx_mm, 0.0, 0.5);
  EXPECT_NEAR(offset.dy_mm, 90.0 * 1.03, 0.5);
}

TEST(PlaneProjection, RegisteredPoseFromPixels) {
  const MarkerPose reference = floor_pose(0.0, 0.0, 0.0, 26.0);
  const MarkerPose plane = estimate_marker_pose(project_corners(reference), kSideMm, kCamera, kNoDistortion);
  const PlanarOffset offset = measure(plane, reference, floor_pose(15.0, -30.0, 8.0, 26.0));
  EXPECT_NEAR(offset.dx_mm, 15.0, 0.5);
  EXPECT_NEAR(offset.dy_mm, -30.0, 0.5);
  EXPECT_NEAR(offset.dyaw_deg, 8.0, 0.2);
}

const std::map<int, std::pair<int, int>> kScene = {{21, {420, 140}}, {1, {80, 60}}, {3, {120, 360}}};

std::map<int, std::pair<int, int>> with(std::map<int, std::pair<int, int>> scene, int id, std::pair<int, int> xy) {
  scene[id] = xy;
  return scene;
}

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

TEST_F(ObserveFrame, MovedMarkerIsMeasured) {
  const FrameObservation observation = observe_frame(render_scene(with(kScene, 21, {440, 140})), spot);
  ASSERT_TRUE(observation.offset);
  EXPECT_GT(observation.offset->distance_mm(), 10.0);
}

TEST_F(ObserveFrame, SmallViewDriftIsSubtracted) {
  const FrameObservation observation = observe_frame(roll(render_scene(kScene), 2, 1), spot);
  EXPECT_FALSE(observation.blocking_status);
  ASSERT_TRUE(observation.offset);
  EXPECT_NEAR(observation.offset->distance_mm(), 0.0, 0.5);
  EXPECT_NEAR(observation.view_shift_px.value_or(0.0), std::hypot(2.0, 1.0), 0.3);
}

TEST_F(ObserveFrame, CameraDriftWithinTheSpotLimitIsSubtracted) {
  // The ceiling mount drifted 5 px within an hour; that must not block a correctly parked Pinky.
  const FrameObservation observation = observe_frame(roll(render_scene(kScene), 10, -4), spot);
  EXPECT_FALSE(observation.blocking_status);
  ASSERT_TRUE(observation.offset);
  EXPECT_NEAR(observation.offset->distance_mm(), 0.0, 0.5);
  ASSERT_TRUE(observation.view_shift_xy_px);
  EXPECT_NEAR(observation.view_shift_xy_px->x, -4.0, 0.3);
  EXPECT_NEAR(observation.view_shift_xy_px->y, 10.0, 0.3);
}

TEST_F(ObserveFrame, SpotLimitIsUsed) {
  spot.max_view_shift_px = 3.0;
  EXPECT_EQ(observe_frame(roll(render_scene(kScene), 10, 0), spot).blocking_status, ParkingStatus::ViewUnverified);
}

TEST_F(ObserveFrame, WrappedFrameIsNotJudged) {
  // The ceiling camera sometimes opens with the whole picture wrapped 96 px sideways.
  const FrameObservation observation = observe_frame(roll(render_scene(kScene), 0, 96), spot);
  EXPECT_EQ(observation.blocking_status, ParkingStatus::ViewUnverified);
  EXPECT_FALSE(observation.offset);
  EXPECT_NEAR(observation.view_shift_px.value_or(0.0), 96.0, 1.0);
}

TEST_F(ObserveFrame, NoFixedMarkerFrameIsSkipped) {
  const FrameObservation observation = observe_frame(render_scene({{21, kScene.at(21)}}), spot);
  EXPECT_FALSE(observation.blocking_status);
  EXPECT_FALSE(observation.offset);
  EXPECT_EQ(observation.skipped_status, ParkingStatus::ViewUnverified);
}

TEST_F(ObserveFrame, OneFixedMarkerIsEnough) {
  const FrameObservation observation = observe_frame(render_scene({{21, kScene.at(21)}, {1, kScene.at(1)}}), spot);
  EXPECT_FALSE(observation.blocking_status);
  EXPECT_TRUE(observation.offset);
}

TEST_F(ObserveFrame, DuplicateMarkerIsAmbiguous) {
  cv::Mat image;
  cv::bitwise_and(render_scene(kScene), render_scene({{21, {300, 300}}}), image);
  EXPECT_EQ(observe_frame(image, spot).blocking_status, ParkingStatus::Ambiguous);
}

TEST_F(ObserveFrame, MarkerOffThePlaneIsRejected) {
  // A bigger marker in the image reads as a bigger side on the registered plane (e.g. Pinky lifted).
  cv::Mat image;
  cv::bitwise_and(render_scene({{1, kScene.at(1)}, {3, kScene.at(3)}}), render_scene({{21, kScene.at(21)}}, 80),
                  image);
  const FrameObservation observation = observe_frame(image, spot);
  EXPECT_FALSE(observation.offset);
  EXPECT_NE(observation.reason.find("marker side"), std::string::npos);
}

TEST_F(ObserveFrame, ViewCheckOffJudgesPinkyMarkerOnly) {
  spot.anchors.clear();
  const FrameObservation observation = observe_frame(render_scene({{21, kScene.at(21)}}), spot);
  ASSERT_TRUE(observation.offset);
  EXPECT_NEAR(observation.offset->distance_mm(), 0.0, 0.5);
  EXPECT_FALSE(observation.view_shift_px);
}

TEST_F(ObserveFrame, ViewCheckOffMissesWrappedFrame) {
  // Known limit: without fixed markers a wrapped view is measured as if Pinky had moved.
  spot.anchors.clear();
  const FrameObservation observation = observe_frame(roll(render_scene(kScene), 0, 96), spot);
  EXPECT_FALSE(observation.blocking_status);
  ASSERT_TRUE(observation.offset);
  EXPECT_GT(observation.offset->distance_mm(), 50.0);
}

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

const Corners kSquare = {cv::Point2d(0.0, 0.0), cv::Point2d(2.0, 0.0), cv::Point2d(2.0, 2.0), cv::Point2d(0.0, 2.0)};

Corners moved(const Corners& corners, cv::Point2d by) {
  return {corners[0] + by, corners[1] + by, corners[2] + by, corners[3] + by};
}

TEST(AnchorShift, NoneWhenNoAnchorVisible) { EXPECT_FALSE(anchor_shift_px({{1, {10.0, 10.0}}}, {})); }

TEST(AnchorShift, DuplicatedAnchorIsIgnored) {
  EXPECT_FALSE(anchor_shift_px({{1, {1.0, 1.0}}}, {{1, {kSquare, moved(kSquare, {50.0, 50.0})}}}));
}

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
  // Tolerance 20 mm / 10°: the worse of the two ratios sets the level.
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
  // NaN compares false against every limit; it must not read as ARRIVED.
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
  // Marker +x forward, +y left, +z up: Pinky's own axes match the marker axes.
  EXPECT_EQ(pinky_frame_offset({30.0, -5.0, 0.0}, "+x"), std::make_pair(30.0, -5.0));
}

TEST(PinkyFrame, MarkerTopEdgeIsFront) {
  // Marker +y forward: marker -x is then Pinky's left.
  EXPECT_EQ(pinky_frame_offset({30.0, -5.0, 0.0}, "+y"), std::make_pair(-5.0, -30.0));
}

TEST(PinkyFrame, MarkerLeftEdgeIsFront) {
  EXPECT_EQ(pinky_frame_offset({30.0, -5.0, 0.0}, "-x"), std::make_pair(-30.0, 5.0));
}

FrameObservation seen(double dx_mm) {
  FrameObservation observation;
  observation.offset = PlanarOffset{dx_mm, 0.0, 0.0};
  observation.view_shift_px = 0.0;
  return observation;
}

FrameObservation missed() {
  FrameObservation observation;
  observation.reason = "marker not visible";
  observation.view_shift_px = 0.0;
  return observation;
}

TEST(ParkingJudgeTest, BriefMissKeepsRecentFrames) {
  ParkingJudge judgment(kTolerance, 3);
  judgment.add(seen(1.0));
  EXPECT_EQ(judgment.add(missed()).status, ParkingStatus::Arrived);
}

TEST(ParkingJudgeTest, MissesForAllFramesDropOldValues) {
  ParkingJudge judgment(kTolerance, 2);
  judgment.add(seen(1.0));
  judgment.add(missed());
  EXPECT_EQ(judgment.add(missed()).status, ParkingStatus::NotFound);
}

TEST(ParkingJudgeTest, FixedMarkerDropoutKeepsRecentFrames) {
  ParkingJudge judgment(kTolerance, 2);
  FrameObservation unverified;
  unverified.reason = "no fixed marker";
  unverified.skipped_status = ParkingStatus::ViewUnverified;
  judgment.add(seen(1.0));
  EXPECT_EQ(judgment.add(unverified).status, ParkingStatus::Arrived);
  EXPECT_EQ(judgment.add(unverified).status, ParkingStatus::ViewUnverified);
}

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

TEST(ParkingJudgeTest, KeepsOnlyLatestFrames) {
  ParkingJudge judgment(kTolerance, 2);
  ParkingResult result = judgment.result();
  for (const double dx : {100.0, 1.0, 2.0}) result = judgment.add(seen(dx));
  EXPECT_EQ(result.status, ParkingStatus::Arrived);
}

TEST(MedianOffset, YawMedianAcrossWraparound) {
  const PlanarOffset median = median_offset({{0, 0, 179.0}, {0, 0, -179.0}, {0, 0, 178.0}});
  EXPECT_NEAR(std::abs(median.dyaw_deg), 179.0, 1e-6);
}

namespace fs = std::filesystem;
using nlohmann::json;

class LoadSpot : public ::testing::Test {
 protected:
  void SetUp() override {
    directory = fs::temp_directory_path() /
                ("parking_vision_" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    fs::create_directories(directory);
  }
  void TearDown() override { fs::remove_all(directory); }

  static void write(const fs::path& path, const json& data) { std::ofstream(path) << data.dump(); }

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

TEST_F(LoadSpot, ChangedCalibrationIsRejected) {
  const fs::path path = write_spot(json::array(), {{"mode", "OFF"}});
  std::ofstream(directory / "ceiling_camera.json", std::ios::app) << "\n";
  EXPECT_NE(error_of(path, directory).find("내부 보정 파일이 바뀌었습니다"), std::string::npos);
}

}  // namespace
}  // namespace parking_vision::test
