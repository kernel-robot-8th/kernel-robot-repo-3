#!/usr/bin/env python3
"""Read-only ROS header-stamp and host-arrival timing survey for MuJoCo RGB-D."""

from __future__ import annotations

import argparse
from bisect import bisect_left
from dataclasses import dataclass
import json
import math
import os
import statistics
import sys
import time
from typing import Any


TOPICS = {
    "rgb": ("/mujoco_omx/fixed_camera/color/image_raw", "image"),
    "depth": ("/mujoco_omx/fixed_camera/depth/image_raw", "image"),
    "camera_info": ("/mujoco_omx/fixed_camera/camera_info", "camera_info"),
    "point_cloud": ("/mujoco_omx/fixed_camera/points", "point_cloud"),
}
PAIRS = (
    ("rgb", "depth"),
    ("rgb", "camera_info"),
    ("rgb", "point_cloud"),
    ("depth", "camera_info"),
    ("depth", "point_cloud"),
)
MAX_SAMPLES_PER_TOPIC = 100_000


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    frame_id: str
    width: int
    height: int
    received_monotonic: float


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _summary(values: list[float]) -> dict[str, float | int | None]:
    return {
        "count": len(values),
        "min_ms": round(min(values), 3) if values else None,
        "p50_ms": round(_percentile(values, 0.50), 3) if values else None,
        "p95_ms": round(_percentile(values, 0.95), 3) if values else None,
        "max_ms": round(max(values), 3) if values else None,
        "mean_ms": round(statistics.fmean(values), 3) if values else None,
    }


def _signature(sample: Sample) -> tuple[str, int, int]:
    return sample.frame_id, sample.width, sample.height


def _pair_report(first: list[Sample], second: list[Sample]) -> dict[str, Any]:
    second_by_signature: dict[tuple[str, int, int], list[int]] = {}
    first_signatures: set[tuple[str, int, int]] = set()
    for sample in first:
        first_signatures.add(_signature(sample))
    for sample in second:
        second_by_signature.setdefault(_signature(sample), []).append(sample.stamp_ns)
    for stamps in second_by_signature.values():
        stamps.sort()

    compatible_groups = first_signatures.intersection(second_by_signature)
    compatible_first = [
        sample for sample in first if _signature(sample) in compatible_groups
    ]
    compatible_second_count = sum(
        len(second_by_signature[signature]) for signature in compatible_groups
    )
    deltas: list[float] = []
    exact_matches = 0
    for sample in compatible_first:
        candidates = second_by_signature[_signature(sample)]
        index = bisect_left(candidates, sample.stamp_ns)
        nearest = []
        if index < len(candidates):
            nearest.append(candidates[index])
        if index > 0:
            nearest.append(candidates[index - 1])
        if nearest:
            delta = min(abs(value - sample.stamp_ns) for value in nearest)
            deltas.append(delta / 1e6)
            if delta == 0:
                exact_matches += 1
    return {
        "comparison": "NEAREST_HEADER_STAMP_ONLY",
        "compatible_frame_and_dimensions_first_samples": len(compatible_first),
        "compatible_frame_and_dimensions_second_samples": compatible_second_count,
        "exact_stamp_matches_first_to_second": exact_matches,
        "nearest_absolute_header_stamp_delta": _summary(deltas),
        "synchronization_claim": False,
    }


def _arrival_report(samples: list[Sample]) -> dict[str, Any]:
    arrivals = [sample.received_monotonic for sample in samples]
    periods_ms = [
        (right - left) * 1000.0
        for left, right in zip(arrivals, arrivals[1:])
        if right >= left
    ]
    return _summary(periods_ms)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Subscribe read-only to MuJoCo fixed-camera ROS messages and report "
            "header-stamp skew and host-arrival intervals; does not decode image/cloud payloads."
        )
    )
    parser.add_argument("--duration", type=float, default=15.0, help="capture seconds (1–300)")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    domain_text = os.environ.get("ROS_DOMAIN_ID", "")
    if not domain_text or any(character not in "0123456789" for character in domain_text):
        print("Set ROS_DOMAIN_ID to an isolated decimal domain from 1 to 232.", file=sys.stderr)
        return 2
    if not 1 <= int(domain_text) <= 232:
        print("Set ROS_DOMAIN_ID to an isolated decimal domain from 1 to 232.", file=sys.stderr)
        return 2
    if not math.isfinite(args.duration) or not 1.0 <= args.duration <= 300.0:
        print("--duration must be a finite value from 1 to 300 seconds.", file=sys.stderr)
        return 2

    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile
        from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    except ImportError as exc:
        print(f"ROS 2 Jazzy sensor message packages are required: {exc}", file=sys.stderr)
        return 2

    context = rclpy.context.Context()
    rclpy.init(args=None, context=context)
    node = rclpy.create_node("mujoco_camera_sync_measurement", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    samples: dict[str, list[Sample]] = {name: [] for name in TOPICS}
    callbacks: dict[str, int] = {name: 0 for name in TOPICS}
    invalid_stamps: dict[str, int] = {name: 0 for name in TOPICS}
    zero_stamps: dict[str, int] = {name: 0 for name in TOPICS}
    overflow_topics: set[str] = set()

    def callback_for(topic_name: str):
        def callback(message) -> None:  # type: ignore[no-untyped-def]
            callbacks[topic_name] += 1
            stamp = message.header.stamp
            sec, nanosec = int(stamp.sec), int(stamp.nanosec)
            if sec < 0 or not 0 <= nanosec < 1_000_000_000:
                invalid_stamps[topic_name] += 1
                return
            if sec == 0 and nanosec == 0:
                zero_stamps[topic_name] += 1
            width = int(message.width)
            height = int(message.height)
            sample = Sample(
                stamp_ns=sec * 1_000_000_000 + nanosec,
                frame_id=str(message.header.frame_id),
                width=width,
                height=height,
                received_monotonic=time.monotonic(),
            )
            if len(samples[topic_name]) < MAX_SAMPLES_PER_TOPIC:
                samples[topic_name].append(sample)
            else:
                overflow_topics.add(topic_name)

        return callback

    message_types = {
        "image": Image,
        "camera_info": CameraInfo,
        "point_cloud": PointCloud2,
    }
    # Match the control station's create_subscription(..., 1): reliable,
    # volatile, KEEP_LAST depth 1. A sensor-data best-effort profile can drop
    # messages differently and would not measure the existing observer path.
    qos_profile = QoSProfile(depth=1)
    for name, (topic, message_type) in TOPICS.items():
        node.create_subscription(
            message_types[message_type], topic, callback_for(name), qos_profile
        )

    started = time.monotonic()
    try:
        while rclpy.ok(context=context) and time.monotonic() - started < args.duration:
            executor.spin_once(timeout_sec=min(0.1, args.duration))
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - started
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)

    required_topics = ("rgb", "depth", "camera_info")
    missing_required_topics = [name for name in required_topics if not samples[name]]
    report: dict[str, Any] = {
        "kind": "mujoco_camera_timing_observation",
        "measurement_status": "INCOMPLETE_REQUIRED_TOPICS"
        if missing_required_topics
        else "REQUIRED_TOPICS_OBSERVED",
        "missing_required_topics": missing_required_topics,
        "ros_domain_id": int(domain_text),
        "capture_duration_s": round(elapsed, 3),
        "mode": "READ_ONLY_NO_IMAGE_OR_POINT_CLOUD_PAYLOAD_DECODED",
        "topics": {
            name: {
                "topic": TOPICS[name][0],
                "callbacks_received": callbacks[name],
                "valid_header_samples": len(samples[name]),
                "observed_sample_rate_hz": round(len(samples[name]) / elapsed, 3)
                if elapsed > 0
                else None,
                "zero_source_stamp_count": zero_stamps[name],
                "invalid_stamp_count": invalid_stamps[name],
                "host_arrival_interval": _arrival_report(samples[name]),
                "sample_limit_reached": name in overflow_topics,
            }
            for name in TOPICS
        },
        "pairwise": {
            f"{first}_vs_{second}": _pair_report(samples[first], samples[second])
            for first, second in PAIRS
        },
        "interpretation_limits": [
            "Nearest stamp deltas are diagnostics, not synchronized sample pairs.",
            "Host arrival intervals use this process monotonic clock and do not share the ROS clock domain.",
            "No pixel, depth, XYZ coordinate, transform, object pose, or hardware command is read or produced.",
            "No acceptable skew threshold is assumed; operators must establish one from the task and measured setup.",
        ],
    }
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 1 if missing_required_topics or overflow_topics else 0


if __name__ == "__main__":
    raise SystemExit(main())
