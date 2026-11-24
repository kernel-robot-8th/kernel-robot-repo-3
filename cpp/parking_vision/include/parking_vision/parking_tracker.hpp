// 천장 카메라 feed로 계속 갱신하는, 화면 표시 전용 Pinky 주차 상태.
//
// 예전 src/control_station/parking_tracker.py의 상태 기계를 C++로 옮긴 것이다: NOT_IN_VIEW -> MOVING ->
// IN_POSITION / OFF_POSITION. 팔 단계 진행 조건으로는 절대 쓰지 않는다. ParkingObservationCollector는 도착 결과
// 이후에 찍힌 프레임만 판정하므로, 움직이는 Pinky가 허용 범위를 잠깐 지나간 프레임이 도착으로 세어질 수 없다.
// 스레드, 시계, feed는 호출하는 쪽이 맡고, 틱마다 on_* 메서드 하나를 부른다.
//
// 위치: parking_vision 라이브러리. ROS 노드가 10 Hz 타이머로 부르고 결과를 /pinky_parking/state로 발행한다.
// 정지 판정: 최근 1초(kStillWindowS)의 프레임이 5개 이상이고 모두 그 중앙값에서 5 mm·2° 안이면 멈춘 것으로 보고,
// 그 중앙값을 허용 오차와 비교해 IN_POSITION / OFF_POSITION을 정한다. 아니면 MOVING이고 마지막 프레임으로 판정한다.
#pragma once

#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <opencv2/core.hpp>

#include "parking_vision/parking_check.hpp"

namespace parking_vision {

inline constexpr double kTrackIntervalS = 0.1;  // 초당 최대 10프레임만 판정한다. feed는 약 30 fps다
inline constexpr double kStillWindowS = 1.0;    // 최근 1초의 모든 프레임이 일치하면 Pinky가 멈춘 것으로 본다
// 주차한 Pinky의 단일 원본 프레임은 약 ±1.5 mm, ±0.55° 흩어진다(2026-11-02, OpenCV 4.14).
// 실제로 달리는 Pinky는 이 시간 안에 수십 mm를 움직인다. 그래서 그 사이 값인 5 mm·2°를 정지 기준으로 둔다.
inline constexpr double kStillPositionMm = 5.0;
inline constexpr double kStillYawDeg = 2.0;
inline constexpr int kMinStillFrames = 5;  // 정지 창 안에 최소 이만큼의 프레임이 있어야 정지로 본다
inline constexpr double kLostAfterS = 1.0;  // 마커가 이만큼 안 보이면 -> NOT_IN_VIEW
inline constexpr double kStaleFeedS = 2.0;  // 새 프레임이 이만큼 없으면 -> CAMERA_UNAVAILABLE
inline constexpr double kSpotReloadS = 5.0;  // 주차 위치 파일이 바뀌었는지 확인하는 최소 간격(초)

// 실시간 표시 상태. ParkingStatus와 달리 "움직이는 중"과 카메라·등록 문제를 구분해서 보여 준다.
enum class LiveState {
  NotInView,
  Moving,
  InPosition,
  OffPosition,
  Ambiguous,
  ViewUnverified,
  CameraUnavailable,
  SpotInvalid,
  TrackingError,  // 카메라나 등록 문제가 아니라 추적기 소프트웨어 오류
};

// Python enum과 같은 문자열("NOT_IN_VIEW", ...).
std::string to_string(LiveState state);

// 한 시점의 표시 상태. ROS 노드가 ParkingState 메시지로 바꿔 발행한다.
struct LiveSnapshot {
  LiveState state = LiveState::NotInView;
  std::string reason;
  std::optional<ParkingResult> judged;  // MOVING / IN_POSITION / OFF_POSITION에서만 있다
  std::optional<double> view_shift_px;  // 판정한 프레임은 `judged` 안에도 들어 있다
  std::optional<std::uint64_t> frame_sequence;
  std::optional<Corners> marker_corners_px;  // 마지막으로 판정한 프레임의 Pinky 마커. 놓치면 지운다
  std::optional<cv::Point2d> view_shift_xy_px;
  double updated_s = 0.0;  // 마지막으로 발행한 시각(호출하는 쪽의 단조 시계, 초)
  double since_s = 0.0;  // 지금 상태가 시작된 시각
};

// 주차 위치 하나를 추적하는 상태 기계. 스레드 안전하지 않으므로 한 스레드(ROS 타이머 콜백)에서만 부른다.
class ParkingTracker {
 public:
  ParkingTracker();

  // 새 주차 위치나 다시 등록한 주차 위치. 이전 샘플은 다른 기준으로 잰 것이라 버린다.
  void set_spot(std::shared_ptr<const ParkingSpot> spot);
  const std::shared_ptr<const ParkingSpot>& spot() const { return spot_; }

  // 이번 틱의 가장 새 프레임(화면 표시에는 가장 새 프레임만 의미가 있다).
  const LiveSnapshot& on_frame(double now_s, std::uint64_t sequence, const cv::Mat& image);
  // feed는 열려 있지만 이번 틱에 새 프레임이 없다. 시작 직후라 아직 여는 중일 수도 있다.
  const LiveSnapshot& on_no_frame(double now_s);
  // feed 자체가 없거나 실패했다.
  const LiveSnapshot& on_camera_unavailable(double now_s, const std::string& reason);
  // 주차 위치를 읽을 수 없다. set_spot을 부를 때까지 추적을 멈춘다.
  const LiveSnapshot& on_spot_invalid(double now_s, const std::string& reason);
  // 호출하는 쪽이 추적 중에 예상하지 못한 오류를 잡았다.
  const LiveSnapshot& on_tracking_error(double now_s, const std::string& reason);

  const LiveSnapshot& snapshot() const { return snapshot_; }

 private:
  void start(double now_s);  // 처음 불린 시각을 기록한다(새 프레임이 없을 때의 기준)
  void reset_samples();      // 정지 판정 샘플과 그림용 마커를 버린다
  // next를 지금 상태로 만든다. 상태가 같으면 since_s를 이어받는다.
  const LiveSnapshot& publish(double now_s, LiveSnapshot next);

  std::shared_ptr<const ParkingSpot> spot_;
  std::optional<double> started_at_;
  std::optional<double> last_frame_at_;
  std::optional<double> last_seen_;
  std::deque<std::pair<double, PlanarOffset>> samples_;  // (시각, 오프셋), 최근 kStillWindowS 동안
  std::optional<Corners> marker_corners_;
  std::optional<cv::Point2d> view_shift_xy_;
  bool published_ = false;
  LiveSnapshot snapshot_;
};

// 주차 위치 파일이나 그 내부 보정 파일이 바뀌면 다시 읽는다. 확인은 최대 kSpotReloadS마다 한 번이다.
// 예전 Python 추적기의 ParkingTracker._reload_spot과 같다. 변경은 두 파일의 수정 시각(mtime)으로 알아챈다.
class SpotReloader {
 public:
  SpotReloader(std::filesystem::path spot_path, std::filesystem::path repository_root);

  // 바뀌었으면 추적기에 넣는다. 주차 위치를 쓸 수 없으면 false를 돌려준다(그때 추적기는 SPOT_INVALID를 보인다).
  bool poll(double now_s, ParkingTracker& tracker);

 private:
  std::filesystem::path path_;
  std::filesystem::path root_;
  // (주차 위치 파일 mtime, 내부 보정 파일 mtime). 주차 위치를 읽지 못했으면 두 번째는 없다.
  using Mtimes = std::pair<std::filesystem::file_time_type, std::optional<std::filesystem::file_time_type>>;
  Mtimes mtimes(const ParkingSpot* spot) const;

  std::optional<Mtimes> mtimes_;
  double checked_s_ = -kSpotReloadS;  // 첫 poll(now_s = 0이어도)에서 바로 확인하도록 음수로 시작한다
  bool loaded_ = false;
};

}  // namespace parking_vision
