// Scene helpers shared by the C++ tests; mirror the helpers in tests/test_parking_check.py.
#pragma once

#include <cmath>
#include <map>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include "parking_vision/parking_check.hpp"

namespace parking_vision::test {

inline const cv::Matx33d kCamera(980.0, 0.0, 320.0, 0.0, 980.0, 240.0, 0.0, 0.0, 1.0);
inline const std::vector<double> kNoDistortion(5, 0.0);
inline constexpr double kSideMm = 70.0;
inline constexpr ParkingTolerance kTolerance{20.0, 10.0};

inline cv::Matx33d rotation_z(double degrees) {
  const double a = degrees * CV_PI / 180.0;
  return {std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a), 0.0, 0.0, 0.0, 1.0};
}

inline cv::Matx33d rotation_x(double degrees) {
  const double a = degrees * CV_PI / 180.0;
  return {1.0, 0.0, 0.0, 0.0, std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a)};
}

// A marker lying on the floor 1.5 m below the camera, moved by (x, y) and turned by yaw in the floor plane.
// With tilt 0 the camera looks straight down; a tilt turns the whole floor about the camera x axis.
inline MarkerPose floor_pose(double x_mm, double y_mm, double yaw_deg, double tilt_deg = 0.0) {
  const cv::Matx33d floor = rotation_x(tilt_deg) * cv::Matx33d::diag(cv::Vec3d(1.0, -1.0, -1.0));
  const cv::Vec3d origin = rotation_x(tilt_deg) * cv::Vec3d(40.0, -30.0, 1500.0);
  return {floor * rotation_z(yaw_deg), origin + floor * cv::Vec3d(x_mm, y_mm, 0.0)};
}

inline Corners project_corners(const MarkerPose& pose) {
  const auto object = marker_object_points(kSideMm);
  std::vector<cv::Point2d> points;
  cv::Mat rvec;
  cv::Rodrigues(pose.rotation, rvec);
  cv::projectPoints(std::vector<cv::Point3d>(object.begin(), object.end()), rvec, pose.translation_mm, kCamera,
                    kNoDistortion, points);
  return {points[0], points[1], points[2], points[3]};
}

// White 640x480 image with flat markers whose top-left corners sit at the given pixels.
inline cv::Mat render_scene(const std::map<int, std::pair<int, int>>& markers, int size = 60) {
  cv::Mat canvas(480, 640, CV_8UC1, cv::Scalar(255));
  for (const auto& [marker_id, xy] : markers) {
    cv::Mat marker;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), marker_id, size,
                                   marker);
#else
    cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), marker_id, size, marker);
#endif
    marker.copyTo(canvas(cv::Rect(xy.first, xy.second, size, size)));
  }
  return canvas;
}

// numpy.roll over (rows, cols): content moves down by dy and right by dx, wrapping around.
inline cv::Mat roll(const cv::Mat& image, int dy, int dx) {
  cv::Mat rolled(image.size(), image.type());
  for (int y = 0; y < image.rows; ++y) {
    for (int x = 0; x < image.cols; ++x) {
      const int ty = ((y + dy) % image.rows + image.rows) % image.rows;
      const int tx = ((x + dx) % image.cols + image.cols) % image.cols;
      rolled.at<uchar>(ty, tx) = image.at<uchar>(y, x);
    }
  }
  return rolled;
}

inline ParkingSpot spot_from_image(const cv::Mat& image, int marker_id = 21, double side_mm = kSideMm) {
  const MarkerDetections found = detect_markers(image, "DICT_4X4_50");
  const Corners& corners = found.at(marker_id).front();
  ParkingSpot spot;
  spot.spot_id = "test";
  spot.camera_id = "ceiling_camera";
  spot.dictionary = "DICT_4X4_50";
  spot.marker_id = marker_id;
  spot.marker_side_mm = side_mm;
  spot.plane = estimate_marker_pose(corners, side_mm, kCamera, kNoDistortion);
  spot.reference = project_to_plane(corners, spot.plane, kCamera, kNoDistortion);
  spot.reference_corners_px = corners;
  for (const auto& [id, detections] : found) {
    if (id == marker_id) continue;
    const Corners& c = detections.front();
    spot.anchors[id] = (c[0] + c[1] + c[2] + c[3]) / 4.0;
  }
  spot.tolerance = kTolerance;
  spot.intrinsics.camera_matrix = kCamera;
  spot.intrinsics.distortion = kNoDistortion;
  spot.intrinsics.width = 640;
  spot.intrinsics.height = 480;
  return spot;
}

}  // namespace parking_vision::test
