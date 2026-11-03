// Ceiling camera capture component: V4L2 MJPEG 640x480 through OpenCV, published as sensor_msgs/Image and
// sensor_msgs/CompressedImage.
//
// Written instead of using usb_cam 0.8.1, which on 2026-11-03 aborted the whole container (and the parking node
// in it) on a corrupt MJPEG frame, could not follow the camera when it re-enumerated as another /dev/videoN,
// mangled relative by-path links and wrote brightness=50 to the camera. This node mirrors the capture loop the
// control station used (camera_adapters.OpenCVArmCameraStreamManager): it never throws out of the capture thread,
// re-resolves the by-path device and reopens it every reopen_interval_s after a failure, and leaves the camera's
// controls alone. It only reads the camera.
#pragma once

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace parking_vision_ros {

class CeilingCameraNode : public rclcpp::Node {
 public:
  explicit CeilingCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~CeilingCameraNode() override;

 private:
  void capture_loop();
  // Returns after the device fails or the node stops; never throws.
  void stream_once(const std::string& source, bool v4l2_index, int index);
  void wait(double seconds) const;
  void set_status(const std::string& status, bool error);

  std::string device_;
  std::string frame_id_;
  int width_;
  int height_;
  double fps_;
  int jpeg_quality_;
  double reopen_interval_s_;
  int max_failed_reads_;

  std::atomic<bool> running_{true};
  std::string status_;  // last logged state, to log changes only
  std::thread thread_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
};

}  // namespace parking_vision_ros
