// Port of tests/test_parking_tracker.py and the judging cases of tests/test_parking_observation.py.
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>

#include "parking_vision/parking_observation.hpp"
#include "parking_vision/parking_tracker.hpp"
#include "test_support.hpp"

namespace parking_vision::test {
namespace {

const std::pair<int, int> kPinkyAt = {420, 140};
const std::pair<int, int> kAnchorAt = {80, 60};

cv::Mat render(std::optional<std::pair<int, int>> pinky, bool anchor = true) {
  std::map<int, std::pair<int, int>> markers;
  if (pinky) markers[21] = *pinky;
  if (anchor) markers[1] = kAnchorAt;
  cv::Mat color;
  cv::cvtColor(render_scene(markers), color, cv::COLOR_GRAY2BGR);
  return color;
}

cv::Mat roll_color(const cv::Mat& image, int dy, int dx) {
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::Mat color;
  cv::cvtColor(roll(gray, dy, dx), color, cv::COLOR_GRAY2BGR);
  return color;
}

std::shared_ptr<const ParkingSpot> dock(double tolerance_mm = 50.0) {
  ParkingSpot spot = spot_from_image(render(kPinkyAt));
  spot.tolerance = {tolerance_mm, 10.0};
  spot.marker_forward_axis = "-x";
  return std::make_shared<const ParkingSpot>(spot);
}

// A tracker on a fake clock that gets one new frame per 0.1 s tick, like the Python Rig.
struct Rig {
  ParkingTracker tracker;
  double now = 0.0;
  std::uint64_t sequence = 0;

  Rig() { tracker.set_spot(dock()); }

  const LiveSnapshot& run(const cv::Mat& image, double seconds) {
    for (int i = 0; i < static_cast<int>(std::lround(seconds / kTrackIntervalS)); ++i) {
      now += kTrackIntervalS;
      tracker.on_frame(now, ++sequence, image);
    }
    return tracker.snapshot();
  }
};

TEST(ParkingTracker, NotInViewThenMovingThenInPosition) {
  Rig rig;
  EXPECT_EQ(rig.run(render(std::nullopt), 0.5).state, LiveState::NotInView);
  LiveSnapshot moving;
  for (int step = 0; step < 5; ++step) {  // driving in, 4 px per frame
    moving = rig.run(render(std::make_pair(kPinkyAt.first, kPinkyAt.second + 20 - 4 * step)), 0.1);
  }
  EXPECT_EQ(moving.state, LiveState::Moving);
  const LiveSnapshot settled = rig.run(render(kPinkyAt), 1.2);
  EXPECT_EQ(settled.state, LiveState::InPosition);
  ASSERT_TRUE(settled.judged);
  EXPECT_EQ(settled.judged->deviation_level, DeviationLevel::Within);
  EXPECT_TRUE(settled.marker_corners_px);
}

TEST(ParkingTracker, OverlayCarriesTheCameraDrift) {
  Rig rig;
  const LiveSnapshot drifted = rig.run(roll_color(render(kPinkyAt), 5, -2), 1.2);
  EXPECT_EQ(drifted.state, LiveState::InPosition);
  ASSERT_TRUE(drifted.view_shift_xy_px);
  EXPECT_NEAR(drifted.view_shift_xy_px->x, -2.0, 0.3);
  EXPECT_NEAR(drifted.view_shift_xy_px->y, 5.0, 0.3);
}

TEST(ParkingTracker, StoppedOutsideToleranceIsOffPosition) {
  Rig rig;
  const LiveSnapshot stopped = rig.run(render(std::make_pair(kPinkyAt.first + 60, kPinkyAt.second)), 1.2);
  EXPECT_EQ(stopped.state, LiveState::OffPosition);
  ASSERT_TRUE(stopped.judged && stopped.judged->deviation_level);
  EXPECT_NE(*stopped.judged->deviation_level, DeviationLevel::Within);
}

TEST(ParkingTracker, SinceKeepsTheTimeTheStateBegan) {
  Rig rig;
  const LiveSnapshot first = rig.run(render(kPinkyAt), 1.2);
  const LiveSnapshot later = rig.run(render(kPinkyAt), 0.5);
  EXPECT_EQ(first.state, later.state);
  EXPECT_EQ(first.since_s, later.since_s);
  EXPECT_GT(later.updated_s, later.since_s);
}

TEST(ParkingTracker, WrappedView) {
  Rig rig;
  const LiveSnapshot state = rig.run(roll_color(render(kPinkyAt), 0, 96), 0.3);
  EXPECT_EQ(state.state, LiveState::ViewUnverified);
  EXPECT_GT(state.view_shift_px.value_or(0.0), 90.0);
}

TEST(ParkingTracker, MarkerLost) {
  Rig rig;
  rig.run(render(kPinkyAt), 1.2);
  const LiveSnapshot lost = rig.run(render(std::nullopt), 1.5);
  EXPECT_EQ(lost.state, LiveState::NotInView);
  EXPECT_FALSE(lost.marker_corners_px);
}

TEST(ParkingTracker, CameraUnavailable) {
  Rig rig;
  EXPECT_EQ(rig.tracker.on_camera_unavailable(0.1, "카메라 feed가 없습니다: head_camera").state,
            LiveState::CameraUnavailable);
}

TEST(ParkingTracker, NoFramesYetWaitsBeforeReportingTheCamera) {
  ParkingTracker tracker;
  tracker.set_spot(dock());
  double now = 0.0;
  for (int i = 0; i < 10; ++i) tracker.on_no_frame(now += 0.1);
  EXPECT_EQ(tracker.snapshot().state, LiveState::NotInView);
  for (int i = 0; i < 15; ++i) tracker.on_no_frame(now += 0.1);
  EXPECT_EQ(tracker.snapshot().state, LiveState::CameraUnavailable);
}

TEST(ParkingTracker, ResolutionMismatch) {
  Rig rig;
  cv::Mat small;
  cv::resize(render(kPinkyAt), small, cv::Size(320, 240));
  EXPECT_EQ(rig.run(small, 0.1).state, LiveState::CameraUnavailable);
}

namespace fs = std::filesystem;
using nlohmann::json;

class SpotReloaderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root = fs::temp_directory_path() /
           ("parking_vision_reload_" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    fs::remove_all(root);
    fs::create_directories(root);
    const fs::path fixtures = fs::path(PARKING_VISION_FIXTURE_DIR) / "spot";
    fs::copy_file(fixtures / "ceiling_camera.json", root / "ceiling_camera.json");
    std::ifstream(fixtures / "spot.json") >> spot;
    write_spot();
  }
  void TearDown() override { fs::remove_all(root); }

  void write_spot() {
    std::ofstream(root / "spot.json") << spot.dump();
    // Make the change visible even within the file system's timestamp resolution.
    fs::last_write_time(root / "spot.json", fs::last_write_time(root / "spot.json") + std::chrono::seconds(++edits));
  }

  fs::path root;
  json spot;
  int edits = 0;
};

TEST_F(SpotReloaderTest, SpotFileChangeIsPickedUp) {
  ParkingTracker tracker;
  SpotReloader reloader(root / "spot.json", root);
  ASSERT_TRUE(reloader.poll(0.1, tracker));
  EXPECT_EQ(tracker.spot()->tolerance.position_mm, 20.0);
  spot["tolerance"]["position_mm"] = 500.0;
  write_spot();
  ASSERT_TRUE(reloader.poll(1.0, tracker));  // checked at most every kSpotReloadS
  EXPECT_EQ(tracker.spot()->tolerance.position_mm, 20.0);
  ASSERT_TRUE(reloader.poll(5.2, tracker));
  EXPECT_EQ(tracker.spot()->tolerance.position_mm, 500.0);
}

TEST_F(SpotReloaderTest, SpotMissing) {
  fs::remove(root / "spot.json");
  ParkingTracker tracker;
  SpotReloader reloader(root / "spot.json", root);
  EXPECT_FALSE(reloader.poll(0.1, tracker));
  EXPECT_EQ(tracker.snapshot().state, LiveState::SpotInvalid);
  EXPECT_FALSE(tracker.spot());
}

TEST_F(SpotReloaderTest, ChangedCalibrationInvalidatesTheSpot) {
  ParkingTracker tracker;
  SpotReloader reloader(root / "spot.json", root);
  ASSERT_TRUE(reloader.poll(0.1, tracker));
  std::ofstream(root / "ceiling_camera.json", std::ios::app) << "\n";
  write_spot();
  EXPECT_FALSE(reloader.poll(5.2, tracker));
  EXPECT_EQ(tracker.snapshot().state, LiveState::SpotInvalid);
  EXPECT_NE(tracker.snapshot().reason.find("내부 보정 파일이 바뀌었습니다"), std::string::npos);
}

TEST_F(SpotReloaderTest, RecalibrationAloneReloadsTheSpot) {
  ParkingTracker tracker;
  SpotReloader reloader(root / "spot.json", root);
  ASSERT_TRUE(reloader.poll(0.1, tracker));
  std::ofstream(root / "ceiling_camera.json", std::ios::app) << "\n";  // the spot file itself is untouched
  fs::last_write_time(root / "ceiling_camera.json", fs::last_write_time(root / "ceiling_camera.json") + std::chrono::seconds(5));
  EXPECT_FALSE(reloader.poll(5.2, tracker));
  EXPECT_EQ(tracker.snapshot().state, LiveState::SpotInvalid);
  EXPECT_FALSE(reloader.poll(5.3, tracker));  // an invalid spot is not re-read every tick either
}

TEST(ParkingTracker, TrackingErrorClearsTheSamples) {
  Rig rig;
  rig.run(render(kPinkyAt), 1.2);
  const LiveSnapshot error = rig.tracker.on_tracking_error(rig.now + 0.1, "추적 오류: test");
  EXPECT_EQ(error.state, LiveState::TrackingError);
  EXPECT_FALSE(error.marker_corners_px);
  EXPECT_EQ(rig.run(render(kPinkyAt), 0.1).state, LiveState::Moving);  // judging starts over
}

TEST(ParkingObservationCollector, ArrivedFromFramesAfterTheRequest) {
  ParkingObservationCollector collector(*dock(), 10.0);
  EXPECT_FALSE(collector.add(1, 9.9, render(std::make_pair(kPinkyAt.first + 60, kPinkyAt.second))));
  std::uint64_t sequence = 1;
  while (collector.add(++sequence, 10.0 + 0.03 * static_cast<double>(sequence), render(kPinkyAt))) {
  }
  EXPECT_TRUE(collector.done(10.5));
  const ParkingObservation observation = collector.finish();
  EXPECT_EQ(observation.result.status, ParkingStatus::Arrived);
  EXPECT_EQ(observation.frames_used, kObservationMaxFrames);
  EXPECT_EQ(observation.frames_examined, kObservationMaxFrames);
  EXPECT_EQ(observation.first_sequence, 2u);
  EXPECT_EQ(observation.last_sequence, 16u);
  EXPECT_TRUE(observation.marker_corners_px);
}

TEST(ParkingObservationCollector, OnlyStaleFramesMeansCameraUnavailable) {
  ParkingObservationCollector collector(*dock(), 10.0);
  EXPECT_FALSE(collector.add(1, 9.0, render(kPinkyAt)));
  EXPECT_FALSE(collector.done(12.9));
  EXPECT_TRUE(collector.done(13.0));
  try {
    collector.finish();
    FAIL() << "expected ParkingObservationError";
  } catch (const ParkingObservationError& error) {
    EXPECT_EQ(error.error_code(), "PARKING_CAMERA_UNAVAILABLE");
  }
}

TEST(ParkingObservationCollector, TooFewUsableFramesIsNotJudged) {
  ParkingObservationCollector collector(*dock(), 10.0);
  for (std::uint64_t sequence = 1; sequence <= 4; ++sequence) collector.add(sequence, 10.1, render(kPinkyAt));
  const ParkingObservation observation = collector.finish();
  EXPECT_EQ(observation.result.status, ParkingStatus::NotFound);
  EXPECT_EQ(observation.result.reason, "only 4 usable frames (< 5)");
}

TEST(ParkingObservationCollector, WrappedViewIsNotJudged) {
  ParkingObservationCollector collector(*dock(), 10.0);
  for (std::uint64_t sequence = 1; sequence <= 6; ++sequence) {
    collector.add(sequence, 10.1, roll_color(render(kPinkyAt), 0, 96));
  }
  EXPECT_EQ(collector.finish().result.status, ParkingStatus::ViewUnverified);
}

TEST(ParkingObservationCollector, ResolutionMismatch) {
  ParkingObservationCollector collector(*dock(), 10.0);
  cv::Mat small;
  cv::resize(render(kPinkyAt), small, cv::Size(320, 240));
  EXPECT_THROW(collector.add(1, 10.1, small), ParkingObservationError);
}

TEST(SpotId, SameRuleAsTheControlStation) {
  EXPECT_TRUE(is_valid_spot_id("pinky_dock_1"));
  EXPECT_TRUE(is_valid_spot_id("a.b-c_1"));
  EXPECT_TRUE(is_valid_spot_id(std::string(80, 'a')));
  EXPECT_FALSE(is_valid_spot_id(std::string(81, 'a')));
  EXPECT_FALSE(is_valid_spot_id(""));
  EXPECT_FALSE(is_valid_spot_id(".hidden"));
  EXPECT_FALSE(is_valid_spot_id("../etc"));
  EXPECT_FALSE(is_valid_spot_id("a/b"));
  EXPECT_EQ(spot_file("/repo", "dock"), std::filesystem::path("/repo/.control-station/parking-spots/dock.json"));
}

}  // namespace
}  // namespace parking_vision::test
