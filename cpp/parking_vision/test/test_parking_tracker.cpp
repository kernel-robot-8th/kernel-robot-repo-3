// 예전 tests/test_parking_tracker.py와 tests/test_parking_observation.py의 판정 케이스를 옮긴 것이다.
//
// 위치: parking_vision 라이브러리 단위 테스트(gtest)다. 실제 시계 대신 가짜 시계를 0.1 s씩 넘기며 돌린다.
// 확인하는 것:
// - ParkingTracker(화면 표시용 상태 기계): NOT_IN_VIEW -> MOVING -> IN_POSITION / OFF_POSITION 전이,
//   since 유지, 96 px 감김(VIEW_UNVERIFIED), 마커 사라짐, 카메라 없음·해상도 불일치(CAMERA_UNAVAILABLE).
// - SpotReloader: 주차 위치 파일이나 내부 보정 파일이 바뀌면 최대 kSpotReloadS(5 s)마다 다시 읽는지.
// - ParkingObservationCollector(일회성 판정): 요청 이후에 찍힌 프레임만 쓰는지, 최대 15프레임·최소 5프레임,
//   요청 전 프레임만 있으면 PARKING_CAMERA_UNAVAILABLE인지.
// - spot_id 규칙이 관제 서버와 같은지.
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

// 마커 좌상단 픽셀 위치다. 21번이 Pinky, 1번·2번이 고정 마커다.
const std::pair<int, int> kPinkyAt = {420, 140};
const std::pair<int, int> kAnchorAt = {80, 60};
const std::pair<int, int> kSecondAnchorAt = {100, 380};  // 고정 마커를 둘 두어 카메라 밀림과 옮겨진 마커를 구분한다

// Pinky 마커(없으면 nullopt)와 고정 마커를 그린 BGR 영상이다. 실제 feed처럼 3채널로 만든다.
cv::Mat render(std::optional<std::pair<int, int>> pinky, bool anchor = true) {
  std::map<int, std::pair<int, int>> markers;
  if (pinky) markers[21] = *pinky;
  if (anchor) {
    markers[1] = kAnchorAt;
    markers[2] = kSecondAnchorAt;
  }
  cv::Mat color;
  cv::cvtColor(render_scene(markers), color, cv::COLOR_GRAY2BGR);
  return color;
}

// roll()은 단일 채널만 다루므로 회색으로 바꿔 굴린 뒤 다시 BGR로 만든다.
cv::Mat roll_color(const cv::Mat& image, int dy, int dx) {
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::Mat color;
  cv::cvtColor(roll(gray, dy, dx), color, cv::COLOR_GRAY2BGR);
  return color;
}

// kPinkyAt에 선 Pinky로 등록한 주차 위치다. 허용 오차는 tolerance_mm(기본 50 mm)와 10°다.
std::shared_ptr<const ParkingSpot> dock(double tolerance_mm = 50.0) {
  ParkingSpot spot = spot_from_image(render(kPinkyAt));
  spot.tolerance = {tolerance_mm, 10.0};
  spot.marker_forward_axis = "-x";
  return std::make_shared<const ParkingSpot>(spot);
}

// 가짜 시계 위의 추적기다. Python의 Rig처럼 0.1 s 틱마다 새 프레임을 하나씩 받는다.
// run(image, seconds)는 같은 영상을 seconds 동안(= seconds / 0.1 틱) 넣고 마지막 상태를 돌려준다.
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

// 정상 흐름: 안 보임 -> 들어오는 중(MOVING) -> 1초 넘게 정지하면 IN_POSITION.
TEST(ParkingTracker, NotInViewThenMovingThenInPosition) {
  Rig rig;
  EXPECT_EQ(rig.run(render(std::nullopt), 0.5).state, LiveState::NotInView);
  LiveSnapshot moving;
  for (int step = 0; step < 5; ++step) {  // 프레임마다 4 px씩 들어오는 중
    moving = rig.run(render(std::make_pair(kPinkyAt.first, kPinkyAt.second + 20 - 4 * step)), 0.1);
  }
  EXPECT_EQ(moving.state, LiveState::Moving);
  const LiveSnapshot settled = rig.run(render(kPinkyAt), 1.2);
  EXPECT_EQ(settled.state, LiveState::InPosition);
  ASSERT_TRUE(settled.judged);
  EXPECT_EQ(settled.judged->deviation_level, DeviationLevel::Within);
  EXPECT_TRUE(settled.marker_corners_px);
}

// 화면이 (아래 5 px, 왼쪽 2 px) 밀리면 그 밀림을 빼고 판정하고, 화면 표시용으로 (x, y) 밀림을 함께 넘긴다.
TEST(ParkingTracker, OverlayCarriesTheCameraDrift) {
  Rig rig;
  const LiveSnapshot drifted = rig.run(roll_color(render(kPinkyAt), 5, -2), 1.2);
  EXPECT_EQ(drifted.state, LiveState::InPosition);
  ASSERT_TRUE(drifted.view_shift_xy_px);
  EXPECT_NEAR(drifted.view_shift_xy_px->x, -2.0, 0.3);
  EXPECT_NEAR(drifted.view_shift_xy_px->y, 5.0, 0.3);
}

// 60 px 옆에 멈추면 허용 오차 밖이므로 OFF_POSITION이고 단계는 WITHIN이 아니다.
TEST(ParkingTracker, StoppedOutsideToleranceIsOffPosition) {
  Rig rig;
  const LiveSnapshot stopped = rig.run(render(std::make_pair(kPinkyAt.first + 60, kPinkyAt.second)), 1.2);
  EXPECT_EQ(stopped.state, LiveState::OffPosition);
  ASSERT_TRUE(stopped.judged && stopped.judged->deviation_level);
  EXPECT_NE(*stopped.judged->deviation_level, DeviationLevel::Within);
}

// 상태가 그대로면 since_s는 처음 그 상태가 된 시각을 유지하고 updated_s만 앞으로 간다.
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

// 마커가 kLostAfterS(1 s)보다 오래 안 보이면 NOT_IN_VIEW가 되고, 그리던 마커 코너도 지운다.
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

// 시작 직후 카메라가 열리는 중일 수 있으므로, 1 s 동안 프레임이 없어도 바로 CAMERA_UNAVAILABLE로 바꾸지 않는다.
// kStaleFeedS(2 s)를 넘기면(여기서는 2.5 s) 그때 CAMERA_UNAVAILABLE이다.
TEST(ParkingTracker, NoFramesYetWaitsBeforeReportingTheCamera) {
  ParkingTracker tracker;
  tracker.set_spot(dock());
  double now = 0.0;
  for (int i = 0; i < 10; ++i) tracker.on_no_frame(now += 0.1);
  EXPECT_EQ(tracker.snapshot().state, LiveState::NotInView);
  for (int i = 0; i < 15; ++i) tracker.on_no_frame(now += 0.1);
  EXPECT_EQ(tracker.snapshot().state, LiveState::CameraUnavailable);
}

// 보정 해상도(640x480)와 다른 프레임은 판정하지 않고 CAMERA_UNAVAILABLE로 보고한다.
TEST(ParkingTracker, ResolutionMismatch) {
  Rig rig;
  cv::Mat small;
  cv::resize(render(kPinkyAt), small, cv::Size(320, 240));
  EXPECT_EQ(rig.run(small, 0.1).state, LiveState::CameraUnavailable);
}

namespace fs = std::filesystem;
using nlohmann::json;

// 고정 기대값의 주차 위치 파일(fixtures/spot)을 임시 디렉터리에 복사해 두고, 내용을 고쳐 가며 재로딩을 본다.
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
    // 파일 시스템의 시각 해상도 안에서 고쳐도 변경이 보이도록, 수정 시각을 고칠 때마다 1초씩 더 미룬다.
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
  ASSERT_TRUE(reloader.poll(1.0, tracker));  // 최대 kSpotReloadS(5 s)마다 한 번만 확인한다
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

// 내부 보정 파일이 바뀌면 다시 읽을 때 SHA-256 검사에 걸려 SPOT_INVALID가 된다(일회성 판정과 같은 규칙).
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

// 주차 위치 파일은 그대로 두고 내부 보정 파일만 바뀌어도 다시 읽는다(그 결과 SHA-256 불일치로 SPOT_INVALID).
TEST_F(SpotReloaderTest, RecalibrationAloneReloadsTheSpot) {
  ParkingTracker tracker;
  SpotReloader reloader(root / "spot.json", root);
  ASSERT_TRUE(reloader.poll(0.1, tracker));
  std::ofstream(root / "ceiling_camera.json", std::ios::app) << "\n";  // 주차 위치 파일 자체는 건드리지 않는다
  fs::last_write_time(root / "ceiling_camera.json", fs::last_write_time(root / "ceiling_camera.json") + std::chrono::seconds(5));
  EXPECT_FALSE(reloader.poll(5.2, tracker));
  EXPECT_EQ(tracker.snapshot().state, LiveState::SpotInvalid);
  EXPECT_FALSE(reloader.poll(5.3, tracker));  // 쓸 수 없는 주차 위치도 틱마다 다시 읽지 않는다
}

// 추적 중 예외(TRACKING_ERROR)가 나면 모아 둔 표본을 버리므로, 다음 프레임 하나로는 정지 판정을 못 하고 MOVING이다.
TEST(ParkingTracker, TrackingErrorClearsTheSamples) {
  Rig rig;
  rig.run(render(kPinkyAt), 1.2);
  const LiveSnapshot error = rig.tracker.on_tracking_error(rig.now + 0.1, "추적 오류: test");
  EXPECT_EQ(error.state, LiveState::TrackingError);
  EXPECT_FALSE(error.marker_corners_px);
  EXPECT_EQ(rig.run(render(kPinkyAt), 0.1).state, LiveState::Moving);  // 판정을 처음부터 다시 시작한다
}

// 요청 시각 10.0 s 전에 찍힌 어긋난 프레임(9.9 s)은 버리고, 그 뒤 프레임 15개(sequence 2~16)로 ARRIVED를 낸다.
// 15개를 채우면 add가 false를 돌려 수집이 끝난다.
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

// 요청 전 프레임만 들어오면 3 s 수집 창(10.0 ~ 13.0 s)이 지난 뒤 PARKING_CAMERA_UNAVAILABLE로 끝난다.
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

// 쓸 수 있는 프레임이 kObservationMinFrames(5)보다 적으면 오프셋이 있어도 NOT_FOUND로 바꾼다.
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

// 일회성 판정에서는 해상도 불일치를 상태가 아니라 예외(PARKING_CAMERA_UNAVAILABLE)로 알린다.
TEST(ParkingObservationCollector, ResolutionMismatch) {
  ParkingObservationCollector collector(*dock(), 10.0);
  cv::Mat small;
  cv::resize(render(kPinkyAt), small, cv::Size(320, 240));
  EXPECT_THROW(collector.add(1, 10.1, small), ParkingObservationError);
}

// 1~80자, 영문·숫자로 시작, 경로 구분자 없음: 파일 경로로 빠져나가는 spot_id(../etc, a/b)를 막는다.
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
