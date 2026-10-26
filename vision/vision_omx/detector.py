"""ArUco 마커 검출. OpenCV 4.6(구 API)과 4.7+(ArucoDetector) 모두 지원한다."""
import cv2
import numpy as np


class MarkerDetector:
    def __init__(self, dictionary_name='DICT_4X4_50'):
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self._detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())
        else:
            self._detector = None
            self._params = cv2.aruco.DetectorParameters_create()

    def detect(self, frame):
        """{marker_id: (중심 u, 중심 v)}와 그리기용 원본 corners를 돌려준다."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self._params)
        centers = {}
        if ids is not None:
            for marker_id, marker_corners in zip(ids.flatten(), corners):
                u, v = np.mean(marker_corners[0], axis=0)
                centers[int(marker_id)] = (float(u), float(v))
        return centers, corners, ids

    def render(self, marker_id, size_px):
        """테스트·인쇄용 마커 이미지."""
        if hasattr(cv2.aruco, 'generateImageMarker'):
            return cv2.aruco.generateImageMarker(self.dictionary, marker_id, size_px)
        return cv2.aruco.drawMarker(self.dictionary, marker_id, size_px)
