#include "parking_vision/registration.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <set>

#include <opencv2/core/version.hpp>

namespace parking_vision {
namespace {

using nlohmann::ordered_json;

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  return values.size() % 2 == 1 ? values[middle] : (values[middle - 1] + values[middle]) / 2.0;
}

double round_to(double value, int digits) {
  const double scale = std::pow(10.0, digits);
  return std::round(value * scale) / scale;
}

cv::Point2d center_of(const Corners& corners) { return (corners[0] + corners[1] + corners[2] + corners[3]) / 4.0; }

// datetime.now(timezone.utc).isoformat(): 2026-11-03T11:18:53.102653+00:00
std::string utc_now_iso() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
  const auto micros =
      std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
  std::tm utc{};
  gmtime_r(&seconds, &utc);
  char text[64];
  std::snprintf(text, sizeof(text), "%04d-%02d-%02dT%02d:%02d:%02d.%06lld+00:00", utc.tm_year + 1900, utc.tm_mon + 1,
                utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec, static_cast<long long>(micros));
  return text;
}

template <typename... Args>
std::string format(const char* pattern, Args... args) {
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer), pattern, args...);
  return buffer;
}

}  // namespace

std::vector<std::pair<int, cv::Point2d>> pick_anchors(const std::vector<CollectedFrame>& collected, int marker_id,
                                                      const std::optional<std::vector<int>>& anchor_ids,
                                                      std::vector<int>* skipped) {
  std::vector<int> candidates;
  if (anchor_ids && !anchor_ids->empty()) {
    candidates = *anchor_ids;
  } else {
    std::set<int> seen;
    for (const auto& [corners, found] : collected) {
      for (const auto& [id, detections] : found) {
        if (id != marker_id) seen.insert(id);
      }
    }
    candidates.assign(seen.begin(), seen.end());
  }
  std::vector<std::pair<int, cv::Point2d>> anchors;
  for (const int anchor_id : candidates) {
    std::vector<cv::Point2d> centers;
    for (const auto& [corners, found] : collected) {
      const auto detections = found.find(anchor_id);
      if (detections != found.end() && detections->second.size() == 1) centers.push_back(center_of(detections->second[0]));
    }
    if (centers.size() == collected.size() && !centers.empty()) {
      std::vector<double> xs;
      std::vector<double> ys;
      for (const auto& c : centers) {
        xs.push_back(c.x);
        ys.push_back(c.y);
      }
      const cv::Point2d center(median(xs), median(ys));
      double jitter = 0.0;
      for (const auto& c : centers) jitter = std::max(jitter, cv::norm(c - center));
      if (jitter <= kMaxAnchorJitterPx) {
        anchors.emplace_back(anchor_id, center);
        continue;
      }
    }
    if (anchor_ids && !anchor_ids->empty()) {
      throw RegistrationRejected(format("fixed marker %d was not seen steadily in every frame", anchor_id));
    }
    if (skipped) skipped->push_back(anchor_id);
  }
  if (anchors.empty()) throw RegistrationRejected("no fixed marker seen steadily - the camera view cannot be verified later");
  return anchors;
}

ordered_json build_registration(const std::vector<CollectedFrame>& collected, const RegistrationOptions& options,
                                const Intrinsics& intrinsics, const std::filesystem::path& repository_root,
                                std::vector<int>* skipped_anchors) {
  if (collected.empty()) throw RegistrationRejected("no frame collected");
  Corners corners;
  for (std::size_t i = 0; i < corners.size(); ++i) {
    std::vector<double> xs;
    std::vector<double> ys;
    for (const auto& frame : collected) {
      xs.push_back(frame.first[i].x);
      ys.push_back(frame.first[i].y);
    }
    corners[i] = {median(xs), median(ys)};
  }
  MarkerPose plane;
  try {
    plane = estimate_marker_pose(corners, options.marker_side_mm, intrinsics.camera_matrix, intrinsics.distortion);
  } catch (const std::runtime_error& error) {
    throw RegistrationRejected(error.what());
  }
  const PlaneMarker reference = project_to_plane(corners, plane, intrinsics.camera_matrix, intrinsics.distortion);
  double jitter_mm = 0.0;
  double jitter_deg = 0.0;
  for (const auto& frame : collected) {
    const PlanarOffset offset = plane_offset(
        reference, project_to_plane(frame.first, plane, intrinsics.camera_matrix, intrinsics.distortion));
    jitter_mm = std::max(jitter_mm, offset.distance_mm());
    jitter_deg = std::max(jitter_deg, std::abs(offset.dyaw_deg));
  }
  if (jitter_mm > kMaxRegisterJitterMm || jitter_deg > kMaxRegisterJitterDeg) {
    throw RegistrationRejected(
        format("moved while registering (%.1f mm / %.1f deg) - keep Pinky still", jitter_mm, jitter_deg));
  }
  const auto anchors = options.no_anchors
                           ? std::vector<std::pair<int, cv::Point2d>>{}
                           : pick_anchors(collected, options.marker_id, options.anchor_ids, skipped_anchors);
  ordered_json view_check = options.no_anchors
                                ? ordered_json{{"mode", "OFF"}}
                                : ordered_json{{"mode", "FIXED_MARKERS"}, {"max_anchor_shift_px", kMaxViewShiftPx}};
  if (options.trust_single_anchor) view_check["single_anchor_trusted"] = true;  // the operator confirmed it is fixed

  ordered_json rotation = ordered_json::array();
  for (int r = 0; r < 3; ++r) rotation.push_back({plane.rotation(r, 0), plane.rotation(r, 1), plane.rotation(r, 2)});
  ordered_json corner_rows = ordered_json::array();
  for (const auto& c : corners) corner_rows.push_back({c.x, c.y});
  ordered_json anchor_rows = ordered_json::array();
  for (const auto& [id, center] : anchors) {
    anchor_rows.push_back({{"marker_id", id}, {"center_px", {round_to(center.x, 2), round_to(center.y, 2)}}});
  }
  return ordered_json{
      {"schema_version", kSchemaVersion},
      {"spot_id", options.spot_id},
      {"camera_id", options.camera_id},
      {"pixel_transform_id", kPixelTransformId},
      {"image_size_px", {{"width", intrinsics.width}, {"height", intrinsics.height}}},
      {"intrinsic_calibration",
       {{"path", std::filesystem::relative(intrinsics.path, repository_root).string()},
        {"sha256", intrinsics.sha256},
        {"calibration_id", intrinsics.calibration_id ? ordered_json(*intrinsics.calibration_id) : ordered_json()}}},
      {"aruco_dictionary", options.dictionary},
      {"marker_id", options.marker_id},
      {"marker_side_mm", options.marker_side_mm},
      {"marker_forward_axis", options.marker_forward_axis ? ordered_json(*options.marker_forward_axis) : ordered_json()},
      {"position_method", "corner rays intersected with the registered marker plane"},
      {"opencv_version", CV_VERSION},
      {"reference_pose_in_camera",
       {{"rotation", rotation},
        {"translation_mm", {plane.translation_mm[0], plane.translation_mm[1], plane.translation_mm[2]}}}},
      {"reference_corners_px", corner_rows},
      {"reference_marker_side_on_plane_mm", round_to(reference.side_mm, 2)},
      {"reference_frame_count", collected.size()},
      {"reference_jitter", {{"position_mm", round_to(jitter_mm, 2)}, {"yaw_deg", round_to(jitter_deg, 3)}}},
      {"anchors", anchor_rows},
      {"view_check", view_check},
      {"tolerance", {{"position_mm", options.tolerance_mm}, {"yaw_deg", options.tolerance_deg}}},
      {"axes", "dx +: 마커 오른쪽 변 방향, dy +: 마커 윗변(화살표) 방향, dyaw +: 천장 카메라에서 볼 때 반시계"},
      {"created_at", utc_now_iso()},
      {"status", "DRAFT_ONLY"},
      {"review_required", true},
      {"usable_for_robot_control", false},
  };
}

}  // namespace parking_vision
