"""관제가 쓰는 카메라 여러 대에서 원본 프레임(검출 박스 없음)을 일정 간격으로 저장한다. 라벨은 붙이지 않는다.

관제와 같은 설정(MJPG 640x480 30fps)으로 장치를 직접 연다. V4L2 카메라는 한 프로그램만 열 수 있으므로,
다른 프로그램(관제 등)이 쓰고 있으면 비워질 때까지 기다린다.

    .venv/bin/python yolo/capture_frames.py --out yolo/dataset_carrot_20261107/raw \
        --camera shelf=/dev/v4l/by-path/...usb-0:10:1.0-video-index0 \
        --camera head=/dev/v4l/by-path/...usb-0:4.4:1.0-video-index0 --every-s 1.5 --minutes 3
"""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import cv2


def in_use(device: str) -> bool:
    target = os.path.realpath(device)
    me = str(os.getpid())
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit() or entry.name == me:
            continue
        try:
            if any(os.readlink(fd.path) == target for fd in os.scandir(f"/proc/{entry.name}/fd")):
                return True
        except OSError:
            continue
    return False


def open_camera(device: str) -> cv2.VideoCapture:
    index = int(os.path.realpath(device).removeprefix("/dev/video"))
    capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    capture.set(cv2.CAP_PROP_FPS, 30)
    if not capture.isOpened():
        raise SystemExit(f"카메라를 열 수 없습니다: {device}")
    return capture


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--camera", action="append", required=True, help="이름=장치경로 (여러 번)")
    parser.add_argument("--every-s", type=float, default=1.5)
    parser.add_argument("--minutes", type=float, default=3.0)
    args = parser.parse_args()
    cameras = dict(item.split("=", 1) for item in args.camera)
    args.out.mkdir(parents=True, exist_ok=True)

    while busy := [name for name, device in cameras.items() if in_use(device)]:
        print(f"다른 프로그램이 카메라를 쓰는 중 {busy}: 관제를 끄면 시작합니다", flush=True)
        time.sleep(2)
    captures = {name: open_camera(device) for name, device in cameras.items()}
    print(f"촬영 시작: {args.minutes:g}분, {args.every_s:g}초마다 → {args.out}", flush=True)
    end = time.monotonic() + args.minutes * 60
    count = 0
    try:
        while time.monotonic() < end:
            next_shot = time.monotonic() + args.every_s
            while time.monotonic() < next_shot:  # keep reading so the saved frame is the newest one
                for capture in captures.values():
                    capture.grab()
            stamp = time.strftime("%H%M%S") + f"{int(time.time() * 10) % 10}"
            for name, capture in captures.items():
                ok, image = capture.retrieve()
                if ok:
                    cv2.imwrite(str(args.out / f"{name}_{stamp}.jpg"), image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            count += 1
            if count % 10 == 0:
                print(f"  {count}회 촬영", flush=True)
    finally:
        for capture in captures.values():
            capture.release()
    print(f"촬영 끝: {count}회 × 카메라 {len(captures)}대", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
