// ROS 2 component around the parking_vision library (docs/plans/pinky-parking-ros2-node.md).
//
// Subscribes to the ceiling camera image topic and offers what the control station does over HTTP:
//   /pinky_parking/state  parking_vision_interfaces/ParkingState, display-only live state (GET .../live)
//   /pinky_parking/check  parking_vision_interfaces/CheckParking, judging only frames captured after the request
//                         (GET .../check)
// It only observes. It sends no command to Pinky or the arm and does not open the camera itself.
#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "parking_vision/parking_tracker.hpp"
#include "parking_vision_interfaces/msg/parking_state.hpp"
#include "parking_vision_interfaces/srv/check_parking.hpp"

namespace parking_vision_ros {

class ParkingVisionNode : public rclcpp::Node {
 public:
  using ParkingState = parking_vision_interfaces::msg::ParkingState;
  using CheckParking = parking_vision_interfaces::srv::CheckParking;

  explicit ParkingVisionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ParkingVisionNode() override;

 private:
  struct Frame {
    std::uint64_t sequence;
    rclcpp::Time captured;  // earlier of the header stamp and the receive time
    rclcpp::Time stamp;
    std::string frame_id;
    cv::Mat image;  // BGR
  };

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& message);
  std::vector<Frame> frames_after(std::uint64_t sequence) const;
  bool has_image_publisher() const;
  void on_tick();
  void publish_state(const rclcpp::Time& now);

  void on_check(const std::shared_ptr<rmw_request_id_t>& header, const std::shared_ptr<CheckParking::Request>& request);
  CheckParking::Response check(const std::string& spot_id);

  std::string repository_root_;
  std::string live_spot_id_;
  std::string feed_camera_id_;

  mutable std::mutex frames_mutex_;
  std::deque<Frame> frames_;
  std::uint64_t sequence_ = 0;

  // Touched only from the timer callback.
  parking_vision::ParkingTracker tracker_;
  std::unique_ptr<parking_vision::SpotReloader> reloader_;
  std::uint64_t tracked_sequence_ = 0;
  std::optional<Frame> tracked_frame_;

  std::atomic<bool> shutting_down_{false};
  struct Worker {
    std::thread thread;
    std::shared_ptr<std::atomic<bool>> finished;
  };
  std::mutex workers_mutex_;
  std::list<Worker> workers_;  // one thread per pending check; the response is sent from it

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<ParkingState>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Service<CheckParking>::SharedPtr check_service_;
};

}  // namespace parking_vision_ros
