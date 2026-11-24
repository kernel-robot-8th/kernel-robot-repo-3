// ParkingObservationCollector 구현: 요청 이후 프레임만 모아 ParkingJudge로 판정하고, 최소 프레임 수를 확인한다.
//
// 위치: parking_vision 라이브러리. 시각과 프레임은 호출하는 쪽(ROS 노드의 판정 스레드)이 넣어 준다.
#include "parking_vision/parking_observation.hpp"

#include <cstdio>
#include <regex>
#include <utility>

namespace parking_vision {
namespace {

const char* const kCameraUnavailable = "PARKING_CAMERA_UNAVAILABLE";

}  // namespace

bool is_valid_spot_id(const std::string& spot_id) {
  // regex_match는 문자열 전체가 맞아야 하므로 ^...$가 필요 없다. 첫 글자 1자 + 나머지 0~79자 = 1~80자.
  static const std::regex pattern("[A-Za-z0-9][A-Za-z0-9_.-]{0,79}");
  return std::regex_match(spot_id, pattern);
}

std::filesystem::path spot_file(const std::filesystem::path& repository_root, const std::string& spot_id) {
  return repository_root / kSpotDir / (spot_id + ".json");
}

ParkingObservationCollector::ParkingObservationCollector(ParkingSpot spot, double requested_at_s,
                                                         double collect_seconds)
    : spot_(std::move(spot)),
      requested_at_s_(requested_at_s),
      deadline_s_(requested_at_s + collect_seconds),
      collect_seconds_(collect_seconds),
      judgment_(spot_.tolerance, kObservationMaxFrames) {}

bool ParkingObservationCollector::add(std::uint64_t sequence, double captured_at_s, const cv::Mat& image) {
  // 요청 전에 찍혔거나 이미 15프레임을 채웠으면 쓰지 않는다.
  if (captured_at_s < requested_at_s_ || frames_used() >= kObservationMaxFrames) return false;
  if (image.cols != spot_.intrinsics.width || image.rows != spot_.intrinsics.height) {
    throw ParkingObservationError(kCameraUnavailable, "프레임 해상도 " + std::to_string(image.cols) + "x" +
                                                          std::to_string(image.rows) + "가 보정 해상도 " +
                                                          std::to_string(spot_.intrinsics.width) + "x" +
                                                          std::to_string(spot_.intrinsics.height) + "와 다릅니다");
  }
  const FrameObservation observation = observe_frame(image, spot_);
  judgment_.add(observation);
  if (observation.blocking_status) {  // 이전 프레임은 더 이상 지금 시야를 나타내지 않는다
    marker_corners_px_.reset();
    view_shift_xy_px_.reset();
  } else if (observation.offset) {  // 판정에 쓴 프레임만 화면 그림에 반영한다
    marker_corners_px_ = observation.corners;
    view_shift_xy_px_ = observation.view_shift_xy_px;
  }
  if (frames_examined_ == 0) {
    first_sequence_ = sequence;
    captured_from_s_ = captured_at_s;
  }
  last_sequence_ = sequence;
  captured_until_s_ = captured_at_s;
  ++frames_examined_;
  return true;
}

bool ParkingObservationCollector::done(double now_s) const {
  return frames_used() >= kObservationMaxFrames || now_s >= deadline_s_;
}

ParkingObservation ParkingObservationCollector::finish() const {
  if (frames_examined_ == 0) {
    char seconds[32];
    std::snprintf(seconds, sizeof(seconds), "%g", collect_seconds_);
    throw ParkingObservationError(kCameraUnavailable, std::string(seconds) + "초 안에 새 프레임이 없습니다");
  }
  ParkingObservation observation;
  observation.result = judgment_.result();
  // 오프셋은 나왔지만 근거 프레임이 5개 미만이면 믿지 않고 NOT_FOUND로 바꾼다. 결과에 오프셋이 없으면
  // (NOT_FOUND / AMBIGUOUS / VIEW_UNVERIFIED) 그 상태와 reason을 그대로 둔다.
  if (observation.result.offset && frames_used() < kObservationMinFrames) {
    observation.result = ParkingResult{ParkingStatus::NotFound, std::nullopt,
                                       "only " + std::to_string(frames_used()) + " usable frames (< " +
                                           std::to_string(kObservationMinFrames) + ")",
                                       observation.result.view_shift_px};
  }
  observation.frames_used = frames_used();
  observation.frames_examined = frames_examined_;
  observation.first_sequence = first_sequence_;
  observation.last_sequence = last_sequence_;
  observation.captured_from_s = captured_from_s_;
  observation.captured_until_s = captured_until_s_;
  observation.marker_corners_px = marker_corners_px_;
  observation.view_shift_xy_px = view_shift_xy_px_;
  return observation;
}

}  // namespace parking_vision
