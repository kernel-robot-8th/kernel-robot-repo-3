// Registering a parking spot from frames of a correctly parked Pinky.
//
// C++ port of build_registration() and pick_anchors() in the former scripts/pinky_parking.py: the marker corners
// are the median over the frames, the marker plane comes from that median pose, and the registration is rejected
// when Pinky moved while registering or no fixed marker stayed put. The result is the schema 2 spot file that
// load_spot() reads.
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

// Pinky's marker moving more than this while registering means it was pushed or covered; nothing is saved.
inline constexpr double kMaxRegisterJitterMm = 5.0;
inline constexpr double kMaxRegisterJitterDeg = 3.0;
// A fixed marker moving more than this while registering is not used to verify the view later.
inline constexpr double kMaxAnchorJitterPx = 1.0;

class RegistrationRejected : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct RegistrationOptions {
  std::string spot_id;
  std::string camera_id = "ceiling_camera";
  std::string dictionary = "DICT_4X4_50";
  int marker_id = 0;
  double marker_side_mm = 0.0;
  std::optional<std::vector<int>> anchor_ids;  // nullopt: every other marker that stayed put
  std::optional<std::string> marker_forward_axis;
  bool trust_single_anchor = false;
  bool no_anchors = false;
  double tolerance_mm = 30.0;
  double tolerance_deg = 5.0;
};

// One collected frame: Pinky's marker (seen exactly once) and every marker detected in it.
using CollectedFrame = std::pair<Corners, MarkerDetections>;

// Fixed markers seen once in every frame and steady within kMaxAnchorJitterPx. Skipped markers are reported
// through `skipped`. Throws RegistrationRejected when a requested anchor is unsteady or none is left.
std::vector<std::pair<int, cv::Point2d>> pick_anchors(const std::vector<CollectedFrame>& collected, int marker_id,
                                                      const std::optional<std::vector<int>>& anchor_ids,
                                                      std::vector<int>* skipped = nullptr);

// The spot file content. intrinsics.path is recorded relative to repository_root. Throws RegistrationRejected.
nlohmann::ordered_json build_registration(const std::vector<CollectedFrame>& collected,
                                          const RegistrationOptions& options, const Intrinsics& intrinsics,
                                          const std::filesystem::path& repository_root,
                                          std::vector<int>* skipped_anchors = nullptr);

}  // namespace parking_vision
