// CeilingCameraNode feeding ParkingVisionNode, with a video file standing in for the camera: frames reach the
// judgment and the JPEG topic, a camera that disappears leaves both nodes running with CAMERA_UNAVAILABLE, and the
// capture picks the camera up again when it returns. CMake runs this with discovery limited to localhost.
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "parking_vision_ros/ceiling_camera_node.hpp"
#include "parking_vision_ros/parking_vision_node.hpp"

namespace parking_vision_ros {
namespace {

using namespace std::chrono_literals;
using ParkingState = parking_vision_interfaces::msg::ParkingState;
namespace fs = std::filesystem;

const fs::path kFixtures = PARKING_VISION_FIXTURE_DIR;

// An MJPEG AVI of one fixture frame, the format the real camera delivers.
void write_video(const fs::path& path, const std::string& frame_name, int frames = 90) {
  cv::Mat color;
  cv::cvtColor(cv::imread((kFixtures / "frames" / (frame_name + ".png")).string(), cv::IMREAD_GRAYSCALE), color,
               cv::COLOR_GRAY2BGR);
  cv::VideoWriter writer(path.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, color.size());
  ASSERT_TRUE(writer.isOpened());
  for (int i = 0; i < frames; ++i) writer.write(color);
}

class CeilingCameraNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    root = fs::temp_directory_path() /
           ("ceiling_camera_" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    fs::remove_all(root);
    fs::create_directories(root / ".control-station" / "parking-spots");
    fs::copy_file(kFixtures / "spot" / "ceiling_camera.json", root / "ceiling_camera.json");
    fs::copy_file(kFixtures / "spot" / "spot.json", root / ".control-station" / "parking-spots" / "parity_dock.json");
    video = root / "camera.avi";
  }

  void start(const fs::path& device) {
    rclcpp::NodeOptions camera_options;
    camera_options.arguments({"--ros-args", "-r", "__ns:=/cam_test"});
    camera_options.parameter_overrides({{"device", device.string()}, {"reopen_interval_s", 0.2}});
    camera = std::make_shared<CeilingCameraNode>(camera_options);
    rclcpp::NodeOptions parking_options;
    parking_options.parameter_overrides(
        {{"repository_root", root.string()}, {"spot_id", "parity_dock"}, {"image_topic", "/cam_test/image_raw"}});
    parking = std::make_shared<ParkingVisionNode>(parking_options);
    helper = std::make_shared<rclcpp::Node>("ceiling_camera_test_helper");
    state_subscription = helper->create_subscription<ParkingState>(
        "/pinky_parking/state", rclcpp::QoS(1).reliable().transient_local(), [this](ParkingState::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex);
          state = *m;
        });
    compressed_subscription = helper->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/cam_test/image_raw/compressed", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex);
          compressed = *m;
        });
    for (const auto& node : {std::static_pointer_cast<rclcpp::Node>(camera),
                             std::static_pointer_cast<rclcpp::Node>(parking), helper}) {
      executor.add_node(node);
    }
    spinner = std::thread([this] { executor.spin(); });
  }

  void TearDown() override {
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    camera.reset();
    parking.reset();
    helper.reset();
    fs::remove_all(root);
  }

  std::optional<ParkingState> wait_for_state(const std::string& expected, std::chrono::seconds timeout = 8s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (state && state->state == expected) return state;
      }
      std::this_thread::sleep_for(20ms);
    }
    std::lock_guard<std::mutex> lock(mutex);
    return state;
  }

  fs::path root;
  fs::path video;
  std::shared_ptr<CeilingCameraNode> camera;
  std::shared_ptr<ParkingVisionNode> parking;
  std::shared_ptr<rclcpp::Node> helper;
  rclcpp::Subscription<ParkingState>::SharedPtr state_subscription;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
  std::mutex mutex;
  std::optional<ParkingState> state;
  std::optional<sensor_msgs::msg::CompressedImage> compressed;
};

TEST_F(CeilingCameraNodeTest, FramesReachTheJudgmentAndTheJpegTopic) {
  write_video(video, "registered");
  start(video);
  const auto current = wait_for_state("IN_POSITION");
  ASSERT_TRUE(current);
  EXPECT_EQ(current->state, "IN_POSITION");
  EXPECT_EQ(current->header.frame_id, "ceiling_camera");
  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_TRUE(compressed);
  EXPECT_EQ(compressed->format, "jpeg");
  const cv::Mat decoded = cv::imdecode(compressed->data, cv::IMREAD_COLOR);
  EXPECT_EQ(decoded.cols, 640);
  EXPECT_EQ(decoded.rows, 480);
}

TEST_F(CeilingCameraNodeTest, CameraThatDisappearsAndReturnsIsFollowed) {
  write_video(video, "registered");
  start(video);
  ASSERT_TRUE(wait_for_state("IN_POSITION"));
  fs::remove(video);  // like the camera dropping off USB: both nodes stay up and say why nothing is judged
  const auto gone = wait_for_state("CAMERA_UNAVAILABLE");
  ASSERT_TRUE(gone);
  EXPECT_EQ(gone->state, "CAMERA_UNAVAILABLE");
  write_video(video, "moved");  // back, possibly as another device behind the same path
  const auto back = wait_for_state("OFF_POSITION", 12s);
  ASSERT_TRUE(back);
  EXPECT_EQ(back->state, "OFF_POSITION");
}

TEST_F(CeilingCameraNodeTest, MissingDeviceAtStartDoesNotStopTheNodes) {
  start(root / "no-such-camera");
  const auto current = wait_for_state("CAMERA_UNAVAILABLE");
  ASSERT_TRUE(current);
  EXPECT_EQ(current->state, "CAMERA_UNAVAILABLE");
}

}  // namespace
}  // namespace parking_vision_ros
