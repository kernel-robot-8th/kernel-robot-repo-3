// 올바르게 주차한 Pinky의 프레임들로 주차 위치를 등록한다.
//
// 예전 scripts/pinky_parking.py의 build_registration()과 pick_anchors()를 C++로 옮긴 것이다. 마커 꼭짓점은
// 프레임들의 중앙값이고, 마커 평면은 그 중앙값 꼭짓점으로 추정한 자세에서 얻는다. 등록하는 동안 Pinky가 움직였거나
// 제자리에 머문 고정 마커가 없으면 등록을 거부한다. 결과는 load_spot()이 읽는 schema 2 주차 위치 파일이다.
//
// 위치: parking_vision 라이브러리. pinky_parking CLI의 register 명령이 프레임을 모아 이 함수를 부른다.
// 이후 판정은 모두 여기서 정한 평면 위에서 잰다(마커 높이가 바뀌지 않는다는 가정).
#pragma once

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "parking_vision/parking_check.hpp"

namespace parking_vision {

// 등록하는 동안 Pinky 마커가 이보다(mm, deg) 많이 움직였으면 밀렸거나 가려진 것이다. 아무것도 저장하지 않는다.
inline constexpr double kMaxRegisterJitterMm = 5.0;
inline constexpr double kMaxRegisterJitterDeg = 3.0;
// 등록하는 동안 이보다(px) 많이 움직인 고정 마커는 나중에 시야 확인에 쓰지 않는다.
inline constexpr double kMaxAnchorJitterPx = 1.0;

// 등록을 거부할 때 던진다. 메시지는 CLI가 그대로 보여 준다.
class RegistrationRejected : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// CLI register 인자에 해당하는 등록 옵션. 기본 허용 오차는 30 mm, 5°다.
struct RegistrationOptions {
  std::string spot_id;
  std::string camera_id = "ceiling_camera";
  std::string dictionary = "DICT_4X4_50";
  int marker_id = 0;
  double marker_side_mm = 0.0;
  std::optional<std::vector<int>> anchor_ids;  // nullopt: 제자리에 머문 다른 모든 마커
  std::optional<std::string> marker_forward_axis;  // Pinky 앞쪽을 가리키는 마커 축 ("+x" 등)
  bool trust_single_anchor = false;  // 고정 마커가 하나만 보여도 밀림을 max_view_shift_px까지 믿는다
  bool no_anchors = false;           // 시야 확인을 끄고 등록한다(view_check.mode OFF)
  double tolerance_mm = 30.0;
  double tolerance_deg = 5.0;
};

// 모은 프레임 하나: (정확히 한 번 보인) Pinky 마커 꼭짓점과, 그 프레임에서 검출한 모든 마커.
using CollectedFrame = std::pair<Corners, MarkerDetections>;

// 모든 프레임에서 한 번씩 보이고 kMaxAnchorJitterPx 안으로 흔들리지 않은 고정 마커(ID, 중앙값 중심 px).
// 건너뛴 마커는 `skipped`로 알려 준다. 지정한 고정 마커가 흔들렸거나 남은 것이 없으면 RegistrationRejected를 던진다.
std::vector<std::pair<int, cv::Point2d>> pick_anchors(const std::vector<CollectedFrame>& collected, int marker_id,
                                                      const std::optional<std::vector<int>>& anchor_ids,
                                                      std::vector<int>* skipped = nullptr);

// 주차 위치 파일 내용(키 순서 유지). intrinsics.path는 repository_root 기준 상대 경로로 기록한다.
// RegistrationRejected를 던진다.
nlohmann::ordered_json build_registration(const std::vector<CollectedFrame>& collected,
                                          const RegistrationOptions& options, const Intrinsics& intrinsics,
                                          const std::filesystem::path& repository_root,
                                          std::vector<int>* skipped_anchors = nullptr);

}  // namespace parking_vision
