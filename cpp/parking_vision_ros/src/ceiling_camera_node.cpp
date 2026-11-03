#include "parking_vision_ros/ceiling_camera_node.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <regex>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace parking_vision_ros {
namespace {

constexpr const char* kDefaultDevice = "/dev/v4l/by-path/pci-0000:80:14.0-usb-0:4:1.0-video-index0";
constexpr double kReadRetryS = 0.05;

}  // namespace

CeilingCameraNode::CeilingCameraNode(const rclcpp::NodeOptions& options) : rclcpp::Node("ceiling_camera", options) {
  // Point at a by-path link: two HCAM0 cameras share one by-id, and the link survives re-enumeration.
  device_ = declare_parameter<std::string>("device", kDefaultDevice);
  frame_id_ = declare_parameter<std::string>("frame_id", "ceiling_camera");
  width_ = declare_parameter<int>("width", 640);
  height_ = declare_parameter<int>("height", 480);
  fps_ = declare_parameter<double>("fps", 30.0);
  jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80);
  reopen_interval_s_ = declare_parameter<double>("reopen_interval_s", 2.0);
  max_failed_reads_ = declare_parameter<int>("max_failed_reads", 10);

  image_publisher_ = create_publisher<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS());
  // JPEG for consumers in other processes (the control station's stream and YOLO): raw 640x480 images were mostly
  // dropped between processes on 2026-11-02.
  compressed_publisher_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", rclcpp::SensorDataQoS());
  thread_ = std::thread([this] { capture_loop(); });
  RCLCPP_INFO(get_logger(), "천장 카메라 캡처 시작: %s (읽기 전용, 카메라 설정은 바꾸지 않음)", device_.c_str());
}

CeilingCameraNode::~CeilingCameraNode() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
}

void CeilingCameraNode::wait(double seconds) const {
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (running_ && std::chrono::steady_clock::now() < until) std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void CeilingCameraNode::set_status(const std::string& status, bool error) {
  if (status == status_) return;
  status_ = status;
  if (error) {
    RCLCPP_WARN(get_logger(), "%s", status.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "%s", status.c_str());
  }
}

void CeilingCameraNode::capture_loop() {
  static const std::regex video_index("/dev/video(\\d+)");
  while (running_) {
    try {
      // Resolved on every attempt, so a camera that comes back as another /dev/videoN is followed.
      std::error_code missing;
      const std::filesystem::path resolved = std::filesystem::canonical(device_, missing);
      if (missing) {
        set_status("카메라 장치 경로가 없습니다: " + device_ + " (" + missing.message() + ")", true);
      } else {
        std::smatch match;
        const std::string path = resolved.string();
        const bool v4l2 = std::regex_match(path, match, video_index);
        stream_once(path, v4l2, v4l2 ? std::stoi(match[1]) : -1);
      }
    } catch (const std::exception& error) {  // keep trying; a capture failure must not take down the container
      set_status(std::string("카메라 캡처 오류: ") + error.what(), true);
    }
    wait(reopen_interval_s_);
  }
}

void CeilingCameraNode::stream_once(const std::string& source, bool v4l2, int index) {
  // A /dev/videoN device opens by index through V4L2; any other path (a video file in tests) opens as a file.
  cv::VideoCapture capture;
  if (v4l2) {
    capture.open(index, cv::CAP_V4L2);
  } else {
    capture.open(source, cv::CAP_ANY);
  }
  if (!capture.isOpened()) {
    set_status("카메라를 열 수 없습니다: " + source + " (다른 프로그램 점유 여부 확인)", true);
    return;
  }
  if (v4l2) {
    // Both HCAM0 cameras share a USB 2.0 hub; MJPEG keeps two streams within its bandwidth.
    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    capture.set(cv::CAP_PROP_FPS, fps_);
  }
  const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
  const auto frame_period = std::chrono::duration<double>(1.0 / fps_);
  auto next_frame = std::chrono::steady_clock::now();
  int failed_reads = 0;
  bool streaming = false;
  cv::Mat frame;
  while (running_) {
    if (!capture.read(frame) || frame.empty()) {
      if (++failed_reads >= max_failed_reads_) break;
      wait(kReadRetryS);
      continue;
    }
    if (frame.type() != CV_8UC3) {
      set_status("카메라 프레임이 BGR8이 아닙니다: " + source, true);
      continue;
    }
    failed_reads = 0;
    if (!streaming) {
      streaming = true;
      set_status("카메라 프레임 수신 중: " + source + " " + std::to_string(frame.cols) + "x" +
                     std::to_string(frame.rows),
                 false);
    }
    std_msgs::msg::Header header;
    header.stamp = now();  // right after the read, as the control station stamped captured_at
    header.frame_id = frame_id_;
    auto image = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(header, "bgr8", frame).toImageMsg(*image);
    image_publisher_->publish(std::move(image));  // moved, so a component in the same container gets it without a copy
    if (compressed_publisher_->get_subscription_count() + compressed_publisher_->get_intra_process_subscription_count() >
        0) {
      auto compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
      compressed->header = header;
      compressed->format = "jpeg";
      cv::imencode(".jpg", frame, compressed->data, jpeg_params);
      compressed_publisher_->publish(std::move(compressed));
    }
    if (!v4l2) {  // a file reads as fast as it can; a camera paces itself
      next_frame += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_period);
      std::this_thread::sleep_until(next_frame);
    }
  }
  capture.release();
  if (running_) {
    char retry[64];
    std::snprintf(retry, sizeof(retry), "%g초 뒤 다시 엽니다", reopen_interval_s_);
    set_status("카메라 프레임 수신 중단: " + source + " (" + retry + ")", true);
  }
}

}  // namespace parking_vision_ros

RCLCPP_COMPONENTS_REGISTER_NODE(parking_vision_ros::CeilingCameraNode)
