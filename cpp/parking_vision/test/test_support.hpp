// C++ 테스트들이 함께 쓰는 장면 생성 도우미다. 예전 tests/test_parking_check.py의 도우미를 그대로 옮겼다.
//
// 위치: parking_vision 라이브러리의 테스트 전용 헤더다(제품 코드에는 들어가지 않는다).
// 핵심 아이디어:
// - 실제 천장 카메라 대신 왜곡 없는 가상 핀홀 카메라(초점거리 980 px, 주점 (320, 240), 640x480)를 쓴다.
// - floor_pose()로 바닥 위 마커 자세를 만들고 project_corners()로 픽셀 코너를 계산하면, 영상 없이도
//   평면 투영·자세 추정을 기하학적으로 검증할 수 있다.
// - render_scene()은 흰 바탕에 ArUco 마커를 그려 실제 검출(detect_markers)까지 거치는 테스트에 쓴다.
// - roll()은 천장 카메라가 가끔 옆으로 ~96 px 밀려 열리는 현상(영상이 감겨 돌아감)을 재현한다.
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

// 가상 카메라 내부 행렬이다. fx = fy = 980 px, 주점 (cx, cy) = (320, 240) px로 640x480 영상의 가운데다.
inline const cv::Matx33d kCamera(980.0, 0.0, 320.0, 0.0, 980.0, 240.0, 0.0, 0.0, 1.0);
// 왜곡 계수 5개(k1, k2, p1, p2, k3)를 모두 0으로 둔 이상적인 렌즈다.
inline const std::vector<double> kNoDistortion(5, 0.0);
// 인쇄된 마커 한 변 길이(mm)다.
inline constexpr double kSideMm = 70.0;
// 테스트용 허용 오차: 위치 20 mm, 방향 10°.
inline constexpr ParkingTolerance kTolerance{20.0, 10.0};

// z축 기준 회전 행렬(각도는 deg)이다. 바닥 평면 안에서 마커를 돌릴 때 쓴다.
inline cv::Matx33d rotation_z(double degrees) {
  const double a = degrees * CV_PI / 180.0;
  return {std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a), 0.0, 0.0, 0.0, 1.0};
}

// x축 기준 회전 행렬(각도는 deg)이다. 카메라가 기울어진 상황을 만들 때 쓴다.
inline cv::Matx33d rotation_x(double degrees) {
  const double a = degrees * CV_PI / 180.0;
  return {1.0, 0.0, 0.0, 0.0, std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a)};
}

// 카메라 아래 1.5 m 바닥에 놓인 마커를 바닥 평면 안에서 (x, y)만큼 옮기고 yaw만큼 돌린 자세다.
// tilt가 0이면 카메라가 바로 아래를 내려다보고, tilt를 주면 바닥 전체가 카메라 x축을 중심으로 돈다.
// diag(1, -1, -1)은 마커 +z(앞면)가 카메라 쪽을 향하고 마커 +y가 영상 위쪽이 되도록 y·z축을 뒤집은 것이다.
// 기준 원점은 카메라 좌표 (40, -30, 1500) mm라서 마커가 주점 바로 아래가 아니라 조금 비켜난 곳에 놓인다.
inline MarkerPose floor_pose(double x_mm, double y_mm, double yaw_deg, double tilt_deg = 0.0) {
  const cv::Matx33d floor = rotation_x(tilt_deg) * cv::Matx33d::diag(cv::Vec3d(1.0, -1.0, -1.0));
  const cv::Vec3d origin = rotation_x(tilt_deg) * cv::Vec3d(40.0, -30.0, 1500.0);
  return {floor * rotation_z(yaw_deg), origin + floor * cv::Vec3d(x_mm, y_mm, 0.0)};
}

// 주어진 자세의 마커 네 코너를 가상 카메라로 투영한 픽셀 좌표다(ArUco 순서: 좌상, 우상, 우하, 좌하).
inline Corners project_corners(const MarkerPose& pose) {
  const auto object = marker_object_points(kSideMm);
  std::vector<cv::Point2d> points;
  cv::Mat rvec;
  cv::Rodrigues(pose.rotation, rvec);
  cv::projectPoints(std::vector<cv::Point3d>(object.begin(), object.end()), rvec, pose.translation_mm, kCamera,
                    kNoDistortion, points);
  return {points[0], points[1], points[2], points[3]};
}

// 흰 640x480 영상에 평평한 마커를 그린다. 각 마커의 좌상단 코너가 주어진 픽셀에 오고, 한 변은 size px이다.
// 사전은 DICT_4X4_50이며, OpenCV 4.7부터 바뀐 API(generateImageMarker / drawMarker)를 버전별로 고른다.
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

// (행, 열)에 대한 numpy.roll과 같다. 내용이 아래로 dy, 오른쪽으로 dx만큼 이동하고 가장자리는 반대편으로 감긴다.
// 음수 나머지를 피하려고 (a % n + n) % n 형태로 계산한다. 8비트 단일 채널 영상만 다룬다.
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

// 영상 한 장에서 바로 주차 위치를 만든다(등록 CLI를 거치지 않는 간이 등록).
// marker_id 마커의 자세로 기준 평면을 정하고, 그 밖에 보이는 마커는 모두 고정 마커(anchor)로 삼는다.
// 내부 보정은 가상 카메라 값이고, 파일 경로·SHA-256은 비워 둔다.
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
