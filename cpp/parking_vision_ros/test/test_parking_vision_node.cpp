// 판정 노드를 ROS 2 위에서 처음부터 끝까지 시험한다. 합성 프레임을 영상 토픽으로 넣고, /pinky_parking/state와
// /pinky_parking/check로 나오는 것을 본다. CMake가 discovery를 localhost로 한정해서(ROS_DOMAIN_ID 97) 실행하므로
// 로봇에는 아무것도 닿지 않는다.
//
// 위치: parking_vision_ros 테스트(ament gtest). 노드와 도우미 노드(구독자·서비스 클라이언트·영상 발행자)를
// MultiThreadedExecutor 하나에 올려 별도 스레드에서 돌린다. 영상은 또 다른 스레드가 약 30 fps로 발행한다.
// 기대값 주차 위치(parity_dock)는 마커 21번, 허용 오차 20 mm, 고정 마커 두 개, marker_forward_axis -x다.
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "parking_vision_ros/parking_vision_node.hpp"

namespace parking_vision_ros {
namespace {

using namespace std::chrono_literals;
using ParkingState = parking_vision_interfaces::msg::ParkingState;
using CheckParking = parking_vision_interfaces::srv::CheckParking;
namespace fs = std::filesystem;

const fs::path kFixtures = PARKING_VISION_FIXTURE_DIR;
constexpr const char* kImageTopic = "/parking_vision_test/image";

// 기대값 흑백 PNG를 bgr8 영상으로 바꾼다.
cv::Mat fixture_frame(const std::string& name) {
  cv::Mat color;
  cv::cvtColor(cv::imread((kFixtures / "frames" / (name + ".png")).string(), cv::IMREAD_GRAYSCALE), color,
               cv::COLOR_GRAY2BGR);
  return color;
}

class ParkingVisionNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    // 관제와 같은 구조의 저장소 루트를 만들고, 기대값 주차 위치를 parity_dock으로 넣어 둔다.
    root = fs::temp_directory_path() /
           ("parking_vision_ros_" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    fs::remove_all(root);
    fs::create_directories(root / ".control-station" / "parking-spots");
    fs::copy_file(kFixtures / "spot" / "ceiling_camera.json", root / "ceiling_camera.json");
    fs::copy_file(kFixtures / "spot" / "spot.json", root / ".control-station" / "parking-spots" / "parity_dock.json");
  }

  // spot_id를 추적하는 판정 노드와 도우미 노드를 띄우고 서비스가 보일 때까지 기다린다.
  // 상태 구독 QoS는 발행 쪽과 같은 reliable·keep-last 1·transient_local이다.
  void start(const std::string& spot_id = "parity_dock") {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {{"repository_root", root.string()}, {"spot_id", spot_id}, {"image_topic", kImageTopic}});
    node = std::make_shared<ParkingVisionNode>(options);
    helper = std::make_shared<rclcpp::Node>("parking_vision_test_helper");
    state_subscription = helper->create_subscription<ParkingState>(
        "/pinky_parking/state", rclcpp::QoS(1).reliable().transient_local(),
        [this](ParkingState::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(state_mutex);
          state = *message;
        });
    client = helper->create_client<CheckParking>("/pinky_parking/check");
    executor.add_node(node);
    executor.add_node(helper);
    spinner = std::thread([this] { executor.spin(); });
    ASSERT_TRUE(client->wait_for_service(10s));
  }

  void TearDown() override {
    stop_publishing();
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    if (node) executor.remove_node(node);
    if (helper) executor.remove_node(helper);
    node.reset();
    fs::remove_all(root);
  }

  // 멈출 때까지 프레임을 약 30 fps로 발행한다. stamp_offset만큼 header stamp를 지금 시각에서 옮긴다.
  void start_publishing(const cv::Mat& frame, rclcpp::Duration stamp_offset = rclcpp::Duration(0, 0)) {
    stop_publishing();
    if (!publisher) publisher = helper->create_publisher<sensor_msgs::msg::Image>(kImageTopic, rclcpp::SensorDataQoS());
    publishing = true;
    publish_thread = std::thread([this, frame, stamp_offset] {
      while (publishing) {
        std_msgs::msg::Header header;
        header.stamp = helper->now() + stamp_offset;
        header.frame_id = "ceiling_camera";
        publisher->publish(*cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());
        std::this_thread::sleep_for(33ms);
      }
    });
    // 판정 요청이 feed가 있는지 묻기 전에 노드가 발행자를 발견할 시간을 준다(최대 5초).
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }

  void stop_publishing() {
    publishing = false;
    if (publish_thread.joinable()) publish_thread.join();
  }

  // 상태가 `expected`가 될 때까지 기다려 돌려준다. 시간이 지나면 마지막으로 본 상태를 돌려준다.
  std::optional<ParkingState> wait_for_state(const std::string& expected, std::chrono::seconds timeout = 5s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        if (state && state->state == expected) return state;
      }
      std::this_thread::sleep_for(20ms);
    }
    std::lock_guard<std::mutex> lock(state_mutex);
    return state;
  }

  rclcpp::Client<CheckParking>::SharedFuture send_check(const std::string& spot_id = "parity_dock") {
    auto request = std::make_shared<CheckParking::Request>();
    request->spot_id = spot_id;
    return client->async_send_request(request).future.share();
  }

  // 판정을 요청하고 응답을 기다린다(최대 15초. 수집은 최대 3초다).
  CheckParking::Response check(const std::string& spot_id = "parity_dock") {
    auto future = send_check(spot_id);
    EXPECT_EQ(future.wait_for(15s), std::future_status::ready);
    return *future.get();
  }

  fs::path root;
  std::shared_ptr<ParkingVisionNode> node;
  std::shared_ptr<rclcpp::Node> helper;
  rclcpp::Subscription<ParkingState>::SharedPtr state_subscription;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
  rclcpp::Client<CheckParking>::SharedPtr client;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
  std::thread publish_thread;
  std::atomic<bool> publishing{false};
  std::mutex state_mutex;
  std::optional<ParkingState> state;
};

TEST_F(ParkingVisionNodeTest, TrackRateOutsideTheStillWindowIsRejected) {
  // 8 Hz보다 낮으면 1초 정지 판정 창에 5프레임을 채울 수 없고, 0 Hz는 타이머조차 만들 수 없다. 1000 Hz는 상한 60을 넘는다.
  for (const double rate : {0.0, 3.0, 1000.0}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"spot_id", "parity_dock"}, {"track_rate_hz", rate}});
    EXPECT_THROW(std::make_shared<ParkingVisionNode>(options), std::invalid_argument) << rate;
  }
}

// 영상 발행자가 없으면 상태는 CAMERA_UNAVAILABLE(이유에 토픽 이름), 판정은 PARKING_CAMERA_UNAVAILABLE이다.
TEST_F(ParkingVisionNodeTest, NoImagePublisherIsCameraUnavailable) {
  start();
  const auto current = wait_for_state("CAMERA_UNAVAILABLE");
  ASSERT_TRUE(current);
  EXPECT_EQ(current->state, "CAMERA_UNAVAILABLE");
  EXPECT_NE(current->reason.find(kImageTopic), std::string::npos);
  // 추적기가 자기 상태를 갖기 전에도 시각은 노드 시계 기준이어야 하고, 1970-01-01(0)이 나오면 안 된다.
  EXPECT_GT(rclcpp::Time(current->updated_at).seconds(), 1e9);
  EXPECT_GT(rclcpp::Time(current->since).seconds(), 1e9);
  const auto response = check();
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.error_code, "PARKING_CAMERA_UNAVAILABLE");
}

// 등록 위치 영상이면 1초 정지 후 IN_POSITION이 되고, 메시지의 판정·그림·표시 전용 필드가 채워진다.
TEST_F(ParkingVisionNodeTest, StateSettlesInPosition) {
  start();
  start_publishing(fixture_frame("registered"));
  const auto current = wait_for_state("IN_POSITION");
  ASSERT_TRUE(current);
  EXPECT_EQ(current->state, "IN_POSITION");
  EXPECT_EQ(current->spot_id, "parity_dock");
  EXPECT_EQ(current->marker_id, 21);
  ASSERT_TRUE(current->has_offset);
  EXPECT_NEAR(current->distance_mm, 0.0, 0.5);
  ASSERT_TRUE(current->has_ratios);
  EXPECT_EQ(current->deviation_level, "WITHIN");
  EXPECT_EQ(current->marker_forward_axis, "-x");
  EXPECT_TRUE(current->has_pinky_frame);
  EXPECT_TRUE(current->view_check);
  EXPECT_EQ(current->tolerance_position_mm, 20.0);
  EXPECT_EQ(current->header.frame_id, "ceiling_camera");
  EXPECT_TRUE(current->has_marker_corners);
  EXPECT_EQ(current->image_width, 640u);
  EXPECT_EQ(current->anchor_centers_px.size(), 4u);  // 고정 마커 두 개(각각 x, y)
  EXPECT_TRUE(current->display_only);
  EXPECT_FALSE(current->usable_for_robot_control);
}

// transient_local QoS 덕분에 나중에 붙은 구독자도 마지막 상태를 바로 받는다.
TEST_F(ParkingVisionNodeTest, LateSubscriberGetsTheLastState) {
  start();
  start_publishing(fixture_frame("registered"));
  ASSERT_TRUE(wait_for_state("IN_POSITION"));
  std::promise<ParkingState> received;
  auto late = helper->create_subscription<ParkingState>(
      "/pinky_parking/state", rclcpp::QoS(1).reliable().transient_local(),
      [&received, done = std::make_shared<std::atomic<bool>>(false)](ParkingState::ConstSharedPtr message) {
        if (!done->exchange(true)) received.set_value(*message);
      });
  auto future = received.get_future();
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(future.get().state.empty());
}

// 약 96 px 옆으로 밀린 영상은 고정 마커 밀림이 커서 VIEW_UNVERIFIED가 되고 오프셋을 내지 않는다.
TEST_F(ParkingVisionNodeTest, StateRejectsWrappedView) {
  start();
  start_publishing(fixture_frame("wrapped_96"));
  const auto current = wait_for_state("VIEW_UNVERIFIED");
  ASSERT_TRUE(current);
  EXPECT_EQ(current->state, "VIEW_UNVERIFIED");
  EXPECT_FALSE(current->has_offset);
  ASSERT_TRUE(current->has_view_shift);
  EXPECT_NEAR(current->view_shift_px, 96.0, 1.0);
}

// 블러·노이즈를 넣은 등록 위치 영상으로 판정하면 ARRIVED, 15프레임 모두 요청 이후에 찍힌 것이다.
TEST_F(ParkingVisionNodeTest, CheckArrived) {
  start();
  start_publishing(fixture_frame("camera_like_registered"));
  const auto response = check();
  ASSERT_TRUE(response.ok) << response.error_code << " " << response.message;
  EXPECT_EQ(response.status, "ARRIVED");
  EXPECT_EQ(response.deviation_level, "WITHIN");
  EXPECT_EQ(response.frames_used, 15);
  EXPECT_EQ(response.frames_examined, 15);
  EXPECT_GE(rclcpp::Time(response.captured_from), rclcpp::Time(response.requested_at));
  EXPECT_EQ(response.observation_schema_version, 1);
  EXPECT_EQ(response.calibration_camera_id, "ceiling_camera");
  EXPECT_EQ(response.feed_camera_id, kImageTopic);
  EXPECT_EQ(response.spot_registration_sha256.size(), 64u);
  EXPECT_EQ(response.intrinsic_calibration_sha256.size(), 64u);
  EXPECT_TRUE(response.has_marker_corners);
  EXPECT_FALSE(response.usable_for_robot_control);
}

// 26 mm 옮긴 영상은 허용 오차 20 mm 밖이라 MISALIGNED다.
TEST_F(ParkingVisionNodeTest, CheckMisaligned) {
  start();
  start_publishing(fixture_frame("moved"));
  const auto response = check();
  ASSERT_TRUE(response.ok);
  EXPECT_EQ(response.status, "MISALIGNED");
  EXPECT_EQ(response.reason, "position 26 mm > 20 mm");
  // 기대값 주차 위치는 marker_forward_axis -x로 기록돼 있으므로 Pinky 자신의 앞쪽은 마커 -x다.
  ASSERT_TRUE(response.has_pinky_frame);
  EXPECT_NEAR(response.forward_mm, -response.dx_mm, 1e-9);
}

// 요청 뒤에 도착했어도 요청 전에 찍힌(stamp가 과거인) 프레임은 쓰지 않는다.
TEST_F(ParkingVisionNodeTest, CheckIgnoresFramesCapturedBeforeTheRequest) {
  start();
  // stamp가 5초 전인 프레임은 요청 뒤에 도착해도 요청 전에 찍힌 것이다.
  start_publishing(fixture_frame("registered"), rclcpp::Duration(-5s));
  const auto response = check();
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.error_code, "PARKING_CAMERA_UNAVAILABLE");
  // 프레임이 오래된 것이었다고 알린다. use_sim_time 없이 rosbag을 재생하면 이렇게 보인다.
  EXPECT_NE(response.message.find("요청 전에 찍힌"), std::string::npos) << response.message;
  EXPECT_NE(response.message.find("use_sim_time"), std::string::npos) << response.message;
}

// 응답의 spot_id는 요청한 이름(파일 이름)이 아니라 주차 위치 파일 안에 기록된 값이다.
TEST_F(ParkingVisionNodeTest, CheckReturnsTheSpotIdInTheFile) {
  fs::copy_file(kFixtures / "spot" / "spot.json", root / ".control-station" / "parking-spots" / "alias_dock.json");
  start();
  start_publishing(fixture_frame("camera_like_registered"));
  const auto response = check("alias_dock");
  ASSERT_TRUE(response.ok) << response.error_code << " " << response.message;
  EXPECT_EQ(response.spot_id, "parity_dock");
}

// 동시에 4개를 넘는 판정 요청은 PARKING_CHECK_BUSY로 바로 거절하고, 끝난 판정은 자리를 비운다.
TEST_F(ParkingVisionNodeTest, ChecksBeyondTheLimitAreBusy) {
  start();
  // 오래된 프레임만 들어오면 판정마다 3초를 꽉 채워 모으므로, 다섯 번째 요청은 네 개가 진행 중일 때 도착한다.
  start_publishing(fixture_frame("registered"), rclcpp::Duration(-5s));
  std::vector<rclcpp::Client<CheckParking>::SharedFuture> pending;
  for (int i = 0; i < 4; ++i) pending.push_back(send_check());
  std::this_thread::sleep_for(500ms);
  const auto busy = check();
  EXPECT_FALSE(busy.ok);
  EXPECT_EQ(busy.error_code, "PARKING_CHECK_BUSY");
  for (auto& future : pending) {
    ASSERT_EQ(future.wait_for(15s), std::future_status::ready);
    EXPECT_EQ(future.get()->error_code, "PARKING_CAMERA_UNAVAILABLE");
  }
  start_publishing(fixture_frame("camera_like_registered"));
  const auto after = check();  // 끝난 판정은 자리를 비운다
  EXPECT_TRUE(after.ok) << after.error_code << " " << after.message;
}

// spot_id 형식 오류(경로 조작 시도), 파일 없음, 등록 뒤 내부 보정 파일 변경을 각각의 error_code로 알린다.
TEST_F(ParkingVisionNodeTest, CheckSpotErrors) {
  start();
  EXPECT_EQ(check("../etc").error_code, "PARKING_SPOT_ID_INVALID");
  EXPECT_EQ(check("missing").error_code, "PARKING_SPOT_NOT_FOUND");
  std::ofstream(root / "ceiling_camera.json", std::ios::app) << "\n";  // 등록 이후 내부 보정 파일이 바뀜(SHA-256 다름)
  const auto response = check();
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.error_code, "PARKING_SPOT_INVALID");
}

// 추적할 주차 위치 파일이 없으면 노드는 살아 있고 상태는 SPOT_INVALID다.
TEST_F(ParkingVisionNodeTest, InvalidLiveSpot) {
  start("missing");
  const auto current = wait_for_state("SPOT_INVALID");
  ASSERT_TRUE(current);
  EXPECT_EQ(current->state, "SPOT_INVALID");
  EXPECT_FALSE(current->has_spot);
}

}  // namespace
}  // namespace parking_vision_ros
