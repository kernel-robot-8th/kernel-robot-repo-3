// 천장 카메라 캡처 컴포넌트다. OpenCV로 V4L2 장치를 MJPEG 640x480으로 읽어 sensor_msgs/Image와
// sensor_msgs/CompressedImage로 발행한다.
//
// 위치: parking_vision_ros 패키지의 rclcpp_components 컴포넌트다. 단독 실행 파일은 ceiling_camera_node이고,
// ceiling_parking.launch.py가 주차 판정 컴포넌트(ParkingVisionNode)와 한 컨테이너에 올린다.
//
// usb_cam 0.8.1 대신 직접 만들었다. 2026-11-03에 usb_cam은 깨진 MJPEG 프레임 하나에 컨테이너 전체(그 안의 주차 판정
// 노드까지)를 종료시켰고, 카메라가 다른 /dev/videoN으로 다시 잡히면 따라가지 못했으며, 상대 경로 by-path 링크를
// 잘못 풀었고, 카메라에 brightness=50을 써 넣었다. 이 노드는 관제가 쓰던 캡처 루프
// (cameras/adapters.py의 OpenCVArmCameraStreamManager)를 그대로 옮긴 것이다. 캡처 스레드 밖으로 예외를 내보내지 않고,
// 실패하면 reopen_interval_s(기본 2초)마다 by-path 장치를 다시 풀어서 다시 열며, 카메라 설정값(controls)은
// 건드리지 않는다. 카메라는 읽기만 한다.
//
// 스레드와 토픽(토픽 이름은 namespace 기준 상대 이름이라, launch에서는 /ceiling_camera/... 가 된다):
// - 캡처는 생성자에서 띄운 전용 스레드(thread_)가 돈다. executor 스레드를 막지 않으므로, 단일 스레드
//   component_container에 함께 올라가도 판정 노드의 콜백은 계속 돈다. 소멸자가 running_을 내리고 join한다.
// - image_raw(bgr8)는 같은 컨테이너의 판정 노드가 intra-process로 받는다. unique_ptr로 move해서 발행하므로
//   복사가 없다.
// - image_raw/compressed는 다른 프로세스의 소비자(관제 영상 스트림, YOLO)용 JPEG다. 카메라가 MJPG를 받아들이면
//   카메라가 보낸 JPEG 바이트를 그대로 보내고(passthrough), image_raw용으로 한 번만 디코딩한다.
// - 두 토픽 모두 SensorDataQoS(best-effort, keep-last 5, volatile)다. 영상은 최신 프레임만 의미가 있어서
//   늦은 프레임을 다시 보내지 않는다.
#pragma once

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace parking_vision_ros {

/// 천장 카메라를 읽기 전용으로 열어 영상 토픽 두 개를 발행하는 노드다(노드 이름 ceiling_camera).
class CeilingCameraNode : public rclcpp::Node {
 public:
  // 파라미터를 읽고 발행자를 만든 뒤 캡처 스레드를 바로 시작한다.
  explicit CeilingCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  // running_을 내리고 캡처 스레드가 끝날 때까지 join한다.
  ~CeilingCameraNode() override;

 private:
  // 캡처 스레드 본체다. 장치 경로를 풀고 stream_once로 읽다가, 끝나면 reopen_interval_s 기다린 뒤 다시 연다.
  void capture_loop();
  // 장치가 실패하거나 노드가 멈추면 돌아온다. 예외를 던지지 않는다(혹시 나더라도 capture_loop가 잡는다).
  // v4l2_index가 true면 /dev/videoN의 번호 index로 V4L2를 열고, 아니면 source를 파일로 연다(테스트의 영상 파일).
  void stream_once(const std::string& source, bool v4l2_index, int index);
  // seconds만큼 기다리되 20 ms마다 running_을 확인해서, 노드가 멈추면 바로 돌아온다.
  void wait(double seconds) const;
  // 상태 문구가 바뀌었을 때만 로그를 남긴다. error면 WARN, 아니면 INFO다.
  void set_status(const std::string& status, bool error);

  std::string device_;      // 카메라 장치 경로(by-path 링크 권장) 또는 영상 파일 경로
  std::string frame_id_;    // 발행하는 header.frame_id
  int width_;               // V4L2에 요청하는 해상도(px)
  int height_;
  double fps_;              // V4L2에 요청하는 프레임률. 영상 파일은 이 속도로 재생한다
  int jpeg_quality_;        // 다시 압축할 때만 쓰는 JPEG 품질(0~100)
  double reopen_interval_s_;  // 장치가 실패한 뒤 다시 열기까지 기다리는 시간(초)
  int max_failed_reads_;    // 연속으로 이만큼 읽기·디코딩에 실패하면 장치를 닫고 다시 연다

  std::atomic<bool> running_{true};  // 소멸자가 false로 내리고 캡처 스레드가 읽는다
  std::string status_;  // 마지막으로 로그에 남긴 상태. 바뀔 때만 로그를 남기려고 둔다(캡처 스레드만 쓴다)
  std::thread thread_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;  // image_raw (bgr8)
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;  // image_raw/compressed
};

}  // namespace parking_vision_ros
