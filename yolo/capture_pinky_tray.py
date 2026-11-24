"""천장 카메라에서 핑키 쟁반 평가용 사진을 저장한다."""

import argparse
from datetime import datetime
import os
from pathlib import Path
import re

import cv2


DEFAULT_CAMERA = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "experiments" / "pinky_tray_eval"
WINDOW = "Pinky tray capture"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("layout", help="배치 이름과 개수. 예: carrot1_tissue1")
    parser.add_argument("--camera", default=os.environ.get("CONTROL_STATION_HEAD_CAMERA") or DEFAULT_CAMERA)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--weights", type=Path, help="미리보기에 표시할 YOLO 모델")
    parser.add_argument("--conf", type=float, default=0.5, help="검출 신뢰도 기준")
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9_-]+", args.layout):
        parser.error("배치 이름에는 영문, 숫자, 밑줄, 하이픈만 쓸 수 있습니다.")

    if not 0 < args.conf <= 1:
        parser.error("--conf는 0보다 크고 1 이하여야 합니다.")
    model = None
    if args.weights is not None:
        if not args.weights.is_file():
            parser.error(f"모델 파일을 찾을 수 없습니다: {args.weights}")
        from ultralytics import YOLO

        model = YOLO(args.weights)

    device = int(args.camera) if args.camera.isdecimal() else args.camera
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        if not camera.isOpened():
            raise SystemExit(f"카메라를 열 수 없습니다: {args.camera}\nUSB 연결과 다른 프로그램의 카메라 사용 여부를 확인하세요.")
        destination = args.output / args.layout
        destination.mkdir(parents=True, exist_ok=True)
        print("미리보기 창에서 스페이스바: 저장 / Q 또는 Esc: 종료", flush=True)
        while True:
            ok, frame = camera.read()
            if not ok:
                raise SystemExit("천장 카메라 영상을 읽지 못했습니다.")
            view = frame
            if model is not None:
                result = model.predict(frame, conf=args.conf, imgsz=640, verbose=False)[0]
                view = result.plot(img=frame.copy())
            cv2.imshow(WINDOW, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key == ord(" "):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                path = destination / f"{timestamp}.jpg"
                if not cv2.imwrite(str(path), frame):
                    raise SystemExit(f"사진을 저장하지 못했습니다: {path}")
                print(f"저장: {path}", flush=True)
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
