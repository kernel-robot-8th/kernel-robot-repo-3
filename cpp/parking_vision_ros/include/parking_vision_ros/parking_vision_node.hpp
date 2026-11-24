// parking_vision 라이브러리를 감싼 ROS 2 컴포넌트다(docs/plans/pinky-parking-ros2-node.md).
//
// 천장 카메라 영상 토픽을 구독하고, 관제가 HTTP로 제공하던 것을 ROS 2로 제공한다.
//   /pinky_parking/state  parking_vision_interfaces/ParkingState, 화면 표시 전용 실시간 상태 (GET .../live)
//   /pinky_parking/check  parking_vision_interfaces/CheckParking, 요청 이후에 찍힌 프레임만 판정한다
//                         (GET .../check)
// 관측만 한다. Pinky나 로봇팔에 명령을 보내지 않고, 카메라를 직접 열지도 않는다.
//
// 위치: parking_vision_ros 패키지의 rclcpp_components 컴포넌트(노드 이름 pinky_parking)다. 단독 실행 파일은
// parking_vision_node이고, ceiling_camera_node와 한 컨테이너에 올리면 영상을 intra-process로 받는다.
//
// 스레드 구성(단일 스레드 executor에서도 동작하도록 짰다):
// - on_image(구독 콜백): 메시지를 변환하지 않고 그대로 frames_ 버퍼(최근 10장)에 넣고 condition variable로 알린다.
// - on_tick(타이머, 기본 10 Hz): 가장 새 프레임 하나만 BGR로 바꿔 ParkingTracker에 넣고 ~/state를 발행한다.
//   tracker_와 관련 멤버는 이 콜백에서만 만진다.
// - on_check(서비스 콜백): deferred response 방식이다. 요청마다 작업 스레드를 하나 띄우고 바로 돌아오므로,
//   최대 3초 프레임을 모으는 동안에도 executor가 막히지 않아 영상 구독이 계속 돈다. 동시에 최대 4개까지 받고,
//   넘으면 PARKING_CHECK_BUSY로 바로 답한다. 작업 스레드가 send_response로 직접 응답한다.
// - ~/state QoS는 reliable, keep-last 1, transient_local이다. 나중에 붙은 구독자(관제)도 마지막 상태를 바로 받는다.
//
// 시계: "요청 이후 프레임" 판단, 수집 마감, 추적기의 정지·사라짐 판단은 모두 steady_clock으로 한다. 그래서 NTP가
// 시스템 시계를 옮겨도 수집이 늘어나거나 일찍 끝나지 않는다. 메시지와 응답에 싣는 시각은 노드 시계(ROS time)다.
// 프레임의 촬영 시각은 "받은 시각 - header stamp가 뒤처진 정도(stamp lag)"로 본다(Frame 주석 참고).
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "parking_vision/parking_tracker.hpp"
#include "parking_vision_interfaces/msg/parking_state.hpp"
#include "parking_vision_interfaces/srv/check_parking.hpp"

namespace parking_vision_ros {

/// 천장 카메라 영상으로 Pinky 주차 상태를 발행하고 주차 판정 서비스를 제공하는 노드다. 관측 전용이다.
class ParkingVisionNode : public rclcpp::Node {
 public:
  using ParkingState = parking_vision_interfaces::msg::ParkingState;
  using CheckParking = parking_vision_interfaces::srv::CheckParking;

  // 파라미터(repository_root, spot_id, image_topic, track_rate_hz, feed_camera_id)를 읽고 구독·발행·서비스를 만든다.
  // track_rate_hz가 8~60 Hz 밖이면 std::invalid_argument를 던져 노드가 시작하지 않는다.
  explicit ParkingVisionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  // 진행 중인 판정 스레드를 깨워 끝내고 모두 join한다.
  ~ParkingVisionNode() override;

 private:
  // 버퍼에 들어 있는 프레임 하나다. 메시지는 원본 그대로 들고 있다.
  struct Frame {
    std::uint64_t sequence;  // 노드가 받은 순서대로 1부터 매기는 번호
    // 노드의 steady clock 기준 촬영 시각이다. 도착 시각에서, 노드 시계로 잰 도착 시각보다 header stamp가 뒤처진
    // 만큼을 뺀다. 판정의 모든 시간 계산이 이 값을 쓰므로, 시스템 시계가 튀어도(NTP) 수집이 늘어나거나 일찍
    // 끝나지 않는다.
    double captured_steady_s;
    rclcpp::Time captured;  // 같은 순간을 노드 시계로 나타낸 값. 응답에 싣는 용도다
    double stamp_lag_s;     // 도착 때의 노드 시계 - header stamp (stamp가 없거나 미래면 0)
    rclcpp::Time stamp;
    std::string frame_id;
    sensor_msgs::msg::Image::ConstSharedPtr message;  // 판정에 쓰는 프레임만 BGR로 변환한다
  };
  // 추적기에 마지막으로 넣은 프레임. ~/state의 header를 그 프레임의 stamp·frame_id로 채우는 데 쓴다.
  struct TrackedFrame {
    std::uint64_t sequence;
    rclcpp::Time stamp;
    std::string frame_id;
  };

  // steady_clock의 현재 시각(초). 판정·추적의 시간 계산은 모두 이 시계로 한다.
  static double steady_seconds();
  // 메시지를 BGR8 cv::Mat으로 본다. 이미 bgr8이면 버퍼를 공유하고, 변환할 수 없으면 cv_bridge::Exception을 던진다.
  static cv_bridge::CvImageConstPtr to_bgr(const Frame& frame);

  // 영상 구독 콜백. 프레임을 버퍼에 넣고 기다리는 판정 스레드를 깨운다.
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& message);
  // sequence보다 새 프레임들을 기다리지 않고 돌려준다.
  std::vector<Frame> frames_after(std::uint64_t sequence) const;
  // `sequence`보다 새 프레임이 올 때까지(또는 종료할 때까지) 최대 `timeout` 기다렸다가 새 프레임들을 돌려준다.
  std::vector<Frame> wait_for_frames_after(std::uint64_t sequence, std::chrono::milliseconds timeout);
  // 구독 중인 영상 토픽에 발행자가 하나라도 있는지. 없으면 카메라 feed가 없는 것으로 본다.
  bool has_image_publisher() const;
  // 타이머 콜백. 주차 위치 파일을 필요하면 다시 읽고, 가장 새 프레임으로 추적기를 갱신한 뒤 상태를 발행한다.
  void on_tick();
  // 추적기의 현재 스냅숏을 ParkingState로 만들어 ~/state에 발행한다.
  void publish_state(const rclcpp::Time& now);

  // 서비스 콜백. 끝난 작업 스레드를 정리하고, 자리가 있으면 새 작업 스레드에 판정을 맡긴 뒤 바로 돌아온다.
  void on_check(const std::shared_ptr<rmw_request_id_t>& header, const std::shared_ptr<CheckParking::Request>& request);
  // 작업 스레드에서 도는 실제 판정. 주차 위치를 읽고 요청 이후 프레임을 모아 응답을 만든다.
  CheckParking::Response check(const std::string& spot_id);
  // 성공·실패 응답이 공통으로 갖는 필드(schema 버전, spot_id, feed, OpenCV 버전 등)만 채운 응답.
  CheckParking::Response base_response(const std::string& spot_id) const;
  // ok=false와 error_code·message를 채우고 경고 로그를 남긴 응답.
  CheckParking::Response failed(CheckParking::Response response, const std::string& error_code,
                                const std::string& message) const;
  // rclcpp가 둘 다 non-const 참조로 받는다. 작업 스레드에서 돌기 때문에 예외를 던지지 않고 로그만 남긴다.
  void send_response(rmw_request_id_t& header, CheckParking::Response& response);

  std::string repository_root_;  // 주차 위치 파일과 내부 보정 파일 경로의 기준 디렉터리
  std::string live_spot_id_;     // ~/state로 추적할 주차 위치. 비어 있으면 상태를 발행하지 않는다
  std::string feed_camera_id_;   // 응답·상태에 싣는 카메라 이름(기본은 구독 토픽 이름)

  // frames_와 sequence_를 지킨다. on_image(executor 스레드)와 판정 작업 스레드가 함께 쓴다.
  mutable std::mutex frames_mutex_;
  std::condition_variable frames_arrived_;  // 새 프레임이 들어오거나 노드가 끝날 때 판정 스레드를 깨운다
  std::deque<Frame> frames_;
  std::uint64_t sequence_ = 0;

  // 타이머 콜백에서만 만진다. 추적기는 steady clock(steady_seconds())으로 돈다.
  parking_vision::ParkingTracker tracker_;
  std::unique_ptr<parking_vision::SpotReloader> reloader_;  // spot_id가 있을 때만 만든다
  std::uint64_t tracked_sequence_ = 0;
  std::optional<TrackedFrame> tracked_frame_;

  std::atomic<bool> shutting_down_{false};  // 소멸자가 true로 바꾸면 판정 스레드가 수집을 멈춘다
  struct Worker {
    std::thread thread;
    std::shared_ptr<std::atomic<bool>> finished;  // 응답을 보내기 직전에 true가 되어 자리를 비운다
  };
  std::mutex workers_mutex_;
  // 대기 중인 판정마다 스레드 하나, 최대 kMaxConcurrentChecks개다. 응답은 그 스레드가 보낸다.
  std::list<Worker> workers_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;  // SensorDataQoS
  rclcpp::Publisher<ParkingState>::SharedPtr state_publisher_;  // ~/state, reliable·keep-last 1·transient_local
  rclcpp::TimerBase::SharedPtr tick_timer_;  // track_rate_hz 주기의 wall timer. spot_id가 있을 때만 만든다
  rclcpp::Service<CheckParking>::SharedPtr check_service_;  // ~/check
};

}  // namespace parking_vision_ros
