"""카메라 열기. 디버그 창과 보정 스크립트가 같은 방식으로 연다."""
import cv2


def open_camera(cam):
    """config의 cameras 항목 하나로 카메라를 연다. 못 열거나 해상도가 다르면 SystemExit.

    OpenCV 기본 백엔드(GStreamer)로는 USB 카메라가 안 열리는 경우가 있어 V4L2를 쓴다.
    YUYV는 대개 640x480까지라 1280x720을 받으려면 MJPG를 지정해야 한다.
    """
    cap = cv2.VideoCapture(cam['device'], cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f'device {cam["device"]}를 열 수 없습니다. `v4l2-ctl --list-devices`로 번호를 확인하세요.')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cam.get('fourcc', 'MJPG')))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam['width'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam['height'])
    # 처리가 느리면 버퍼에 쌓인 오래된 프레임을 읽게 된다. 가장 최근 프레임만 둔다
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ok, frame = cap.read()
    if not ok:
        raise SystemExit(f'device {cam["device"]}에서 프레임을 읽지 못했습니다. 다른 프로그램이 쓰고 있는지 확인하세요.')
    height, width = frame.shape[:2]
    if (width, height) != (cam['width'], cam['height']):
        # 조용히 다른 해상도로 돌면 intrinsic·homography가 맞지 않는다
        raise SystemExit(f'device {cam["device"]}가 {cam["width"]}x{cam["height"]} 대신 {width}x{height}로 열렸습니다. '
                         f'`v4l2-ctl -d /dev/video{cam["device"]} --list-formats-ext`로 지원 해상도를 확인하세요.')
    return cap
