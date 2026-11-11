// The node end to end over ROS 2: synthetic frames on an image topic in, /pinky_parking/state and
// /pinky_parking/check out. CMake runs this with discovery limited to localhost, so nothing reaches robots.
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
    // A repository root laid out like the control station's, holding the fixture spot as parity_dock.
    root = fs::temp_directory_path() /
           ("parking_vision_ros_" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    fs::remove_all(root);
    fs::create_directories(root / ".control-station" / "parking-spots");
    fs::copy_file(kFixtures / "spot" / "ceiling_camera.json", root / "ceiling_camera.json");
    fs::copy_file(kFixtures / "spot" / "spot.json", root / ".control-station" / "parking-spots" / "parity_dock.json");
  }

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

  // Publishes the frame at ~30 fps until stopped; stamp_offset shifts the header stamp from now.
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
    // Let the node discover the publisher before a check asks whether the feed exists.
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }

  void stop_publishing() {
    publishing = false;
    if (publish_thread.joinable()) publish_thread.join();
  }

  // Waits until the state reaches `expected` and returns it, or the last one seen.
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
  // Below 8 Hz the 1 s still window cannot hold its 5 frames; 0 Hz would not even make a timer.
  for (const double rate : {0.0, 3.0, 1000.0}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"spot_id", "parity_dock"}, {"track_rate_hz", rate}});
    EXPECT_THROW(std::make_shared<ParkingVisionNode>(options), std::invalid_argument) << rate;
  }
}

TEST_F(ParkingVisionNodeTest, NoImagePublisherIsCameraUnavailable) {
  start();
  const auto current = wait_for_state("CAMERA_UNAVAILABLE");
  ASSERT_TRUE(current);
  EXPECT_EQ(current->state, "CAMERA_UNAVAILABLE");
  EXPECT_NE(current->reason.find(kImageTopic), std::string::npos);
  // Times are on the node clock even before the tracker has a state of its own, never 1970-01-01.
  EXPECT_GT(rclcpp::Time(current->updated_at).seconds(), 1e9);
  EXPECT_GT(rclcpp::Time(current->since).seconds(), 1e9);
  const auto response = check();
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.error_code, "PARKING_CAMERA_UNAVAILABLE");
}

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
  EXPECT_EQ(current->anchor_centers_px.size(), 4u);  // two fixed markers
  EXPECT_TRUE(current->display_only);
  EXPECT_FALSE(current->usable_for_robot_control);
}

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

TEST_F(ParkingVisionNodeTest, CheckMisaligned) {
  start();
  start_publishing(fixture_frame("moved"));
  const auto response = check();
  ASSERT_TRUE(response.ok);
  EXPECT_EQ(response.status, "MISALIGNED");
  EXPECT_EQ(response.reason, "position 26 mm > 20 mm");
  // The fixture spot records marker_forward_axis -x, so Pinky's own forward is marker -x.
  ASSERT_TRUE(response.has_pinky_frame);
  EXPECT_NEAR(response.forward_mm, -response.dx_mm, 1e-9);
}

TEST_F(ParkingVisionNodeTest, CheckIgnoresFramesCapturedBeforeTheRequest) {
  start();
  // Frames stamped 5 s in the past were captured before the request even though they arrive after it.
  start_publishing(fixture_frame("registered"), rclcpp::Duration(-5s));
  const auto response = check();
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.error_code, "PARKING_CAMERA_UNAVAILABLE");
  // Says the frames were old, which is what a rosbag replayed without use_sim_time looks like.
  EXPECT_NE(response.message.find("요청 전에 찍힌"), std::string::npos) << response.message;
  EXPECT_NE(response.message.find("use_sim_time"), std::string::npos) << response.message;
}

TEST_F(ParkingVisionNodeTest, CheckReturnsTheSpotIdInTheFile) {
  fs::copy_file(kFixtures / "spot" / "spot.json", root / ".control-station" / "parking-spots" / "alias_dock.json");
  start();
  start_publishing(fixture_frame("camera_like_registered"));
  const auto response = check("alias_dock");
  ASSERT_TRUE(response.ok) << response.error_code << " " << response.message;
  EXPECT_EQ(response.spot_id, "parity_dock");
}

TEST_F(ParkingVisionNodeTest, ChecksBeyondTheLimitAreBusy) {
  start();
  // Old frames keep every check collecting for its full 3 s, so the fifth arrives while four are in flight.
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
  const auto after = check();  // finished checks free their slots
  EXPECT_TRUE(after.ok) << after.error_code << " " << after.message;
}

TEST_F(ParkingVisionNodeTest, CheckSpotErrors) {
  start();
  EXPECT_EQ(check("../etc").error_code, "PARKING_SPOT_ID_INVALID");
  EXPECT_EQ(check("missing").error_code, "PARKING_SPOT_NOT_FOUND");
  std::ofstream(root / "ceiling_camera.json", std::ios::app) << "\n";  // calibration changed since registration
  const auto response = check();
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.error_code, "PARKING_SPOT_INVALID");
}

TEST_F(ParkingVisionNodeTest, InvalidLiveSpot) {
  start("missing");
  const auto current = wait_for_state("SPOT_INVALID");
  ASSERT_TRUE(current);
  EXPECT_EQ(current->state, "SPOT_INVALID");
  EXPECT_FALSE(current->has_spot);
}

}  // namespace
}  // namespace parking_vision_ros
