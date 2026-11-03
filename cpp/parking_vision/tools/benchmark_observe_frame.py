#!/usr/bin/env python3
"""Per-frame latency of control_station.parking_check.observe_frame on the parity fixtures.

Counterpart of tools/benchmark_observe_frame.cpp; run both with the same OpenCV release and thread count:

    /usr/bin/python3 cpp/parking_vision/tools/benchmark_observe_frame.py [iterations] [opencv threads]
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from control_station.parking_check import load_spot, observe_frame  # noqa: E402

FIXTURES = Path(__file__).resolve().parents[1] / "test" / "fixtures"


def main() -> int:
    iterations = int(sys.argv[1]) if len(sys.argv) > 1 else 500
    if len(sys.argv) > 2:
        cv2.setNumThreads(int(sys.argv[2]))
    spot = load_spot(FIXTURES / "spot" / "spot.json", FIXTURES / "spot")
    print(f"Python {sys.version.split()[0]} OpenCV {cv2.__version__}, {cv2.getNumThreads()} threads, {iterations} iterations")
    for name in ("registered", "camera_like_registered", "moved", "wrapped_96"):
        gray = cv2.imread(str(FIXTURES / "frames" / f"{name}.png"), cv2.IMREAD_GRAYSCALE)
        frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)  # the feed delivers BGR
        for _ in range(50):
            observe_frame(frame, spot)
        ms = []
        for _ in range(iterations):
            start = time.perf_counter()
            observe_frame(frame, spot)
            ms.append((time.perf_counter() - start) * 1000.0)
        ms.sort()
        print(f"  {name:<24} median {ms[len(ms) // 2]:.3f} ms  p95 {ms[len(ms) * 95 // 100]:.3f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
