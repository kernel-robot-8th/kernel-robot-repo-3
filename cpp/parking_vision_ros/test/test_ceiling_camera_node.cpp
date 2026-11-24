// CeilingCameraNode가 ParkingVisionNode에 영상을 대는 구성을, 비디오 파일을 카메라 대신 써서 시험한다. 프레임이
// 판정과 JPEG 토픽에 닿는지, 카메라가 사라져도 두 노드가 CAMERA_UNAVAILABLE을 내며 계속 도는지, 카메라가 돌아오면
// 캡처가 다시 잡는지를 확인한다. CMake가 discovery를 localhost로 한정해서(ROS_DOMAIN_ID 98) 실행한다.
//
// 위치: parking_vision_ros 테스트(ament gtest). 두 노드와 도우미 노드를 한 MultiThreadedExecutor에 올려
// 별도 스레드에서 돌리고, 테스트 스레드는 mutex로 보호한 마지막 메시지를 폴링한다.
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

// 기대값 프레임 하나를 반복한 MJPEG AVI(실제 카메라가 보내는 형식). 30 fps, 기본 90프레임(약 3초).
void write_video(const fs::path& path, const std::string& frame_name, int frames = 90) {
  cv::Mat color;
  cv::cvtColor(cv::imread((kFixtures / "frames" / (frame_name + ".png")).string(), cv::IMREAD_GRAYSCALE), color,
               cv::COLOR_GRAY2BGR);
  cv::VideoWriter writer(path.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, color.size());
  ASSERT_TRUE(writer.isOpened());
  for (int i = 0; i < frames; ++i) writer.write(color);
}

// 테스트마다 임시 저장소 루트에 내부 보정 파일과 주차 위치(parity_dock)를 복사해 둔다.
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

  // device를 카메라로 여는 캡처 노드(namespace /cam_test, 다시 열기 0.2초로 짧게)와 판정 노드, 상태·JPEG 토픽을
  // 받는 도우미 노드를 띄운다. 상태 구독 QoS는 발행 쪽과 같은 reliable·keep-last 1·transient_local이다.
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

  // 상태가 expected가 될 때까지 20 ms 간격으로 기다린다. 시간이 지나면 마지막으로 받은 상태(없으면 nullopt)를 돌려준다.
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

// 등록 위치 영상이면 판정 노드가 IN_POSITION을 내고, JPEG 토픽에 640x480으로 풀리는 jpeg가 나온다.
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

// 영상 파일을 지웠다가 다른 내용(moved)으로 다시 만들면 CAMERA_UNAVAILABLE을 거쳐 새 영상의 판정(OFF_POSITION)이 나온다.
TEST_F(CeilingCameraNodeTest, CameraThatDisappearsAndReturnsIsFollowed) {
  write_video(video, "registered");
  start(video);
  ASSERT_TRUE(wait_for_state("IN_POSITION"));
  fs::remove(video);  // 카메라가 USB에서 빠진 것처럼: 두 노드 모두 살아 있고 왜 판정하지 못하는지 알린다
  const auto gone = wait_for_state("CAMERA_UNAVAILABLE");
  ASSERT_TRUE(gone);
  EXPECT_EQ(gone->state, "CAMERA_UNAVAILABLE");
  write_video(video, "moved");  // 돌아옴. 같은 경로 뒤에 다른 장치로 잡혔을 수도 있다
  const auto back = wait_for_state("OFF_POSITION", 12s);
  ASSERT_TRUE(back);
  EXPECT_EQ(back->state, "OFF_POSITION");
}

// 시작할 때 장치가 없어도 노드가 죽지 않고 CAMERA_UNAVAILABLE을 낸다.
TEST_F(CeilingCameraNodeTest, MissingDeviceAtStartDoesNotStopTheNodes) {
  start(root / "no-such-camera");
  const auto current = wait_for_state("CAMERA_UNAVAILABLE");
  ASSERT_TRUE(current);
  EXPECT_EQ(current->state, "CAMERA_UNAVAILABLE");
}

}  // namespace
}  // namespace parking_vision_ros
