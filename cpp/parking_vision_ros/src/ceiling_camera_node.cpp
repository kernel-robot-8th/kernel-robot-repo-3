// CeilingCameraNode 구현이다. 천장 카메라(HCAM0)를 V4L2로 읽어 image_raw(bgr8)와 image_raw/compressed(JPEG)를
// 발행한다. 설계 이유(usb_cam을 쓰지 않은 까닭, 다시 열기, MJPEG 그대로 전달)는 ceiling_camera_node.hpp에 있다.
//
// 흐름: capture_loop가 2초(reopen_interval_s)마다 장치 경로를 다시 풀고 stream_once로 연다. stream_once는
// 읽기가 max_failed_reads번 연속 실패하거나 노드가 멈출 때까지 프레임을 읽어 발행하고, 실패하면 장치를 닫고
// 돌아온다. 예외는 capture_loop에서 모두 잡으므로 캡처 스레드 밖으로 나가지 않고, 컨테이너도 죽지 않는다.
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

// 현장 PC의 USB 포트 4에 꽂힌 천장 카메라. launch는 CONTROL_STATION_CEILING_CAMERA가 있으면 그것을 넘긴다.
constexpr const char* kDefaultDevice = "/dev/v4l/by-path/pci-0000:80:14.0-usb-0:4:1.0-video-index0";
// 읽기가 실패했을 때 다음 읽기까지 쉬는 시간(초). 실패한 장치를 바쁘게 두드리지 않으려고 둔다.
constexpr double kReadRetryS = 0.05;

}  // namespace

CeilingCameraNode::CeilingCameraNode(const rclcpp::NodeOptions& options) : rclcpp::Node("ceiling_camera", options) {
  // by-path 링크를 가리킨다. HCAM0 카메라 두 대가 by-id 하나를 같이 쓰고, by-path 링크는 다시 잡혀도 유지된다.
  device_ = declare_parameter<std::string>("device", kDefaultDevice);
  frame_id_ = declare_parameter<std::string>("frame_id", "ceiling_camera");
  width_ = declare_parameter<int>("width", 640);
  height_ = declare_parameter<int>("height", 480);
  fps_ = declare_parameter<double>("fps", 30.0);
  jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80);  // 다시 압축할 때만 쓴다(카메라 자체 JPEG에는 안 씀)
  reopen_interval_s_ = declare_parameter<double>("reopen_interval_s", 2.0);
  max_failed_reads_ = declare_parameter<int>("max_failed_reads", 10);

  // 같은 컨테이너의 판정 노드용. best-effort라 구독자가 늦어도 캡처가 기다리지 않는다.
  image_publisher_ = create_publisher<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS());
  // 다른 프로세스의 소비자(관제 영상 스트림과 YOLO)용 JPEG다. 2026-11-02에 640x480 raw 영상은 프로세스 사이에서
  // 대부분 전달되지 않았다.
  compressed_publisher_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", rclcpp::SensorDataQoS());
  // 캡처는 전용 스레드에서 돈다. 블로킹 read가 executor를 막지 않게 하려는 것이다.
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
  // canonical()로 링크를 끝까지 푼 실제 장치가 /dev/videoN이면 그 번호로 V4L2를 연다.
  static const std::regex video_index("/dev/video(\\d+)");
  while (running_) {
    try {
      // 시도할 때마다 다시 풀어서, 카메라가 다른 /dev/videoN으로 돌아와도 따라간다.
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
    } catch (const std::exception& error) {  // 계속 다시 시도한다. 캡처 실패가 컨테이너를 죽여서는 안 된다
      set_status(std::string("카메라 캡처 오류: ") + error.what(), true);
    }
    wait(reopen_interval_s_);  // 실패든 정상 종료든 다시 열기 전에 기다린다(노드가 멈추면 바로 빠져나온다)
  }
}

void CeilingCameraNode::stream_once(const std::string& source, bool v4l2, int index) {
  // /dev/videoN 장치는 번호로 V4L2를 통해 열고, 그 밖의 경로(테스트의 영상 파일)는 파일로 연다.
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
  const int mjpeg = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  bool passthrough = false;
  if (v4l2) {
    // HCAM0 카메라 두 대가 USB 2.0 허브 하나를 같이 쓴다. MJPEG여야 두 스트림이 그 대역폭 안에 들어간다.
    // 형식·해상도·프레임률만 요청하고, 밝기 같은 카메라 설정값은 건드리지 않는다.
    capture.set(cv::CAP_PROP_FOURCC, mjpeg);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    capture.set(cv::CAP_PROP_FPS, fps_);
    // 카메라 자체 JPEG를 받는다. 그 바이트는 image_raw/compressed로 그대로 나가고 image_raw용으로 한 번만
    // 디코딩한다. OpenCV가 디코딩한 뒤 이 루프가 프레임마다 다시 압축하는 것을 피하려는 것이다. 2026-11-03에
    // OpenCV 4.6과 4.14 모두 HCAM0의 JPEG(Huffman 테이블을 포함한 baseline)를 바꾸지 않고 29.7 fps로 내줬다.
    // CONVERT_RGB=0이 실제로 받아들여졌고 형식이 MJPG일 때만 passthrough로 간다. 아니면 예전처럼 다시 압축한다.
    passthrough = static_cast<int>(capture.get(cv::CAP_PROP_FOURCC)) == mjpeg &&
                  capture.set(cv::CAP_PROP_CONVERT_RGB, 0) && capture.get(cv::CAP_PROP_CONVERT_RGB) == 0;
  }
  const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
  const auto frame_period = std::chrono::duration<double>(1.0 / fps_);  // 영상 파일 재생 간격
  auto next_frame = std::chrono::steady_clock::now();
  int failed_reads = 0;  // 연속 실패 횟수. 성공한 프레임에서 0으로 되돌린다
  bool streaming = false;  // 이번에 연 장치에서 첫 프레임을 받았는지(받으면 한 번만 로그)
  cv::Mat grabbed;  // passthrough면 카메라가 보낸 JPEG 바이트, 아니면 디코딩된 프레임
  cv::Mat frame;
  while (running_) {
    // 읽기 실패(USB가 빠졌거나 파일 끝)가 max_failed_reads번 이어지면 루프를 끝내고 장치를 다시 연다.
    if (!capture.read(grabbed) || grabbed.empty()) {
      if (++failed_reads >= max_failed_reads_) break;
      wait(kReadRetryS);
      continue;
    }
    // OpenCV도 카메라 MJPEG를 같은 imdecode로 푼다. 그래서 깨진 프레임은 거기서처럼 여기서도 실패한다.
    // 풀 수 없는 프레임은 두 토픽 어디에도 보내지 않는다(usb_cam처럼 예외로 죽지 않고 건너뛴다).
    frame = passthrough ? cv::imdecode(grabbed, cv::IMREAD_COLOR) : grabbed;
    if (frame.empty()) {
      if (++failed_reads >= max_failed_reads_) break;
      continue;
    }
    if (frame.type() != CV_8UC3) {  // bgr8로 낼 수 없는 프레임은 건너뛴다(연속 실패 횟수에는 세지 않는다)
      set_status("카메라 프레임이 BGR8이 아닙니다: " + source, true);
      continue;
    }
    failed_reads = 0;
    if (!streaming) {
      streaming = true;
      set_status("카메라 프레임 수신 중: " + source + " " + std::to_string(frame.cols) + "x" +
                     std::to_string(frame.rows) + (passthrough ? " (카메라 JPEG 그대로 전달)" : " (JPEG 다시 압축)"),
                 false);
    }
    std_msgs::msg::Header header;
    header.stamp = now();  // 읽은 직후의 노드 시계. 관제가 captured_at을 찍던 방식과 같다
    header.frame_id = frame_id_;
    auto image = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(header, "bgr8", frame).toImageMsg(*image);
    image_publisher_->publish(std::move(image));  // move하므로 같은 컨테이너의 컴포넌트는 복사 없이 받는다
    // JPEG는 받는 쪽(프로세스 밖 구독자 + intra-process 구독자)이 있을 때만 만든다.
    if (compressed_publisher_->get_subscription_count() + compressed_publisher_->get_intra_process_subscription_count() >
        0) {
      auto compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
      compressed->header = header;
      compressed->format = "jpeg";
      if (passthrough) {  // 카메라 JPEG 바이트를 그대로 복사한다(다시 압축하지 않음)
        compressed->data.assign(grabbed.datastart, grabbed.dataend);
      } else {
        cv::imencode(".jpg", frame, compressed->data, jpeg_params);
      }
      compressed_publisher_->publish(std::move(compressed));
    }
    if (!v4l2) {  // 파일은 할 수 있는 만큼 빨리 읽히므로 fps에 맞춰 쉰다. 카메라는 스스로 속도를 맞춘다
      next_frame += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_period);
      std::this_thread::sleep_until(next_frame);
    }
  }
  capture.release();
  if (running_) {  // 노드가 멈춘 것이 아니라 장치가 실패한 경우에만 다시 연다고 알린다
    char retry[64];
    std::snprintf(retry, sizeof(retry), "%g초 뒤 다시 엽니다", reopen_interval_s_);
    set_status("카메라 프레임 수신 중단: " + source + " (" + retry + ")", true);
  }
}

}  // namespace parking_vision_ros

// component_container가 이 클래스를 plugin 이름으로 찾아 올릴 수 있게 등록한다.
RCLCPP_COMPONENTS_REGISTER_NODE(parking_vision_ros::CeilingCameraNode)
