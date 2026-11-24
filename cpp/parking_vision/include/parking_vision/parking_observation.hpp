// 요청 이후에 찍힌 프레임으로 하는 일회성 주차 관측.
//
// 예전 src/control_station/parking_observation.py의 판정 부분을 C++로 옮긴 것이다. 요청 이후에 찍힌 프레임만
// 판정하고, 수집 시간 안에서 쓸 수 있는 프레임을 최대 kObservationMaxFrames개까지 모으며, 쓸 수 있는 프레임이
// kObservationMinFrames개보다 적으면 판정하지 않는다. 결과는 관측일 뿐이며 로봇 명령이 아니다.
// 주차 위치 찾기, 영상 feed 자체, JSON 계약은 호출하는 쪽이 맡고, 시각도 호출하는 쪽이 넘겨준다.
//
// 위치: parking_vision 라이브러리. ROS 노드의 /pinky_parking/check 서비스가 요청마다 하나씩 만들어 쓴다.
// 요청 이후 프레임만 쓰는 이유: 요청 직전 버퍼에 남은 프레임은 Pinky가 아직 움직이던 때일 수 있어서,
// 허용 범위를 잠깐 지나가던 순간이 "도착"으로 판정되면 안 되기 때문이다.
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "parking_vision/parking_check.hpp"

namespace parking_vision {

inline constexpr int kObservationMaxFrames = 15;  // 쓸 수 있는 프레임을 이만큼 모으면 바로 끝낸다
inline constexpr int kObservationMinFrames = 5;   // 이보다 적으면 오프셋이 있어도 NOT_FOUND로 바꾼다
inline constexpr double kObservationCollectSeconds = 3.0;  // 요청 시각부터 프레임을 모으는 최대 시간(초)
inline constexpr int kObservationSchemaVersion = 1;  // 응답의 observation_schema_version
// 등록된 주차 위치 파일은 <repository root>/.control-station/parking-spots/<spot_id>.json에 있다.
inline constexpr const char* kSpotDir = ".control-station/parking-spots";

// 1~80자의 영문자, 숫자, '_', '.', '-'이고 영문자나 숫자로 시작해야 한다(경로 구분자 없음).
// 요청으로 받은 spot_id가 파일 경로가 되므로 ../ 같은 경로 조작을 막는다.
bool is_valid_spot_id(const std::string& spot_id);
// <repository_root>/<kSpotDir>/<spot_id>.json. spot_id는 먼저 is_valid_spot_id로 확인해야 한다.
std::filesystem::path spot_file(const std::filesystem::path& repository_root, const std::string& spot_id);

// 관제 계약과 같은 error_code를 싣는다(수집기에서는 PARKING_CAMERA_UNAVAILABLE).
class ParkingObservationError : public std::runtime_error {
 public:
  ParkingObservationError(std::string error_code, const std::string& message)
      : std::runtime_error(message), error_code_(std::move(error_code)) {}
  const std::string& error_code() const { return error_code_; }

 private:
  std::string error_code_;
};

// 일회성 관측의 최종 결과. sequence와 captured_*는 판정에 쓴(요청 이후) 프레임의 범위다.
struct ParkingObservation {
  ParkingResult result;
  int frames_used = 0;      // 중앙값에 들어간 쓸 수 있는 프레임 수
  int frames_examined = 0;  // 요청 이후에 찍혀서 판정에 넣어 본 프레임 수
  std::uint64_t first_sequence = 0;
  std::uint64_t last_sequence = 0;
  double captured_from_s = 0.0;
  double captured_until_s = 0.0;
  std::optional<Corners> marker_corners_px;     // 마지막으로 판정한 프레임의 Pinky 마커(화면에 그리기용)
  std::optional<cv::Point2d> view_shift_xy_px;  // 그 프레임의 고정 마커 밀림(px)
};

// 요청 하나에 대한 프레임 수집기. 사용 순서: add를 프레임마다 부르고, done이 true가 되면 finish로 결과를 낸다.
// 스레드 안전하지 않으므로 한 스레드(ROS 노드에서는 그 요청의 작업 스레드)에서만 쓴다.
class ParkingObservationCollector {
 public:
  // requested_at_s와 모든 captured_at_s / now_s는 같은 단조 시계(steady clock, 초)를 쓴다.
  ParkingObservationCollector(ParkingSpot spot, double requested_at_s,
                              double collect_seconds = kObservationCollectSeconds);

  // 프레임 하나를 판정한다. 요청 전에 찍힌 프레임(요청 전에 찍힌 프레임은 쓰지 않는다)이나 수집이 끝난 뒤
  // 들어온 프레임이면 false를 돌려준다. 프레임 크기가 보정한 크기와 다르면 ParkingObservationError를 던진다.
  bool add(std::uint64_t sequence, double captured_at_s, const cv::Mat& image);
  // 쓸 수 있는 프레임이 충분하거나 수집 시간이 지났다.
  bool done(double now_s) const;
  int frames_used() const { return static_cast<int>(judgment_.recent().size()); }
  int frames_examined() const { return frames_examined_; }
  // 판정한 프레임이 하나도 없으면 ParkingObservationError(PARKING_CAMERA_UNAVAILABLE)를 던진다.
  ParkingObservation finish() const;

  const ParkingSpot& spot() const { return spot_; }

 private:
  ParkingSpot spot_;
  double requested_at_s_;
  double deadline_s_;  // requested_at_s + collect_seconds
  double collect_seconds_;
  ParkingJudge judgment_;  // 최근 kObservationMaxFrames개의 중앙값으로 판정한다
  int frames_examined_ = 0;
  std::uint64_t first_sequence_ = 0;
  std::uint64_t last_sequence_ = 0;
  double captured_from_s_ = 0.0;
  double captured_until_s_ = 0.0;
  std::optional<Corners> marker_corners_px_;
  std::optional<cv::Point2d> view_shift_xy_px_;
};

}  // namespace parking_vision
