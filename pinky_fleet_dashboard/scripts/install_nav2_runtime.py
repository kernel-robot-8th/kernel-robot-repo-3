#!/usr/bin/env python3
"""Create a dashboard-owned Nav2 launch/config copy on a Pinky."""

from __future__ import annotations

from pathlib import Path
import hashlib
import json


SHARE = Path("/home/pinky/pinky_pro/install/pinky_navigation/share/pinky_navigation")
RUNTIME = Path("/home/pinky/.cache/pinky-fleet/nav2")


def replace_once(text: str, old: str, new: str, expected: int = 1) -> str:
    count = text.count(old)
    if count != expected:
        raise RuntimeError(f"expected {expected} occurrence(s), found {count}: {old}")
    return text.replace(old, new)


def main() -> None:
    RUNTIME.mkdir(parents=True, exist_ok=True)

    bringup = (SHARE / "launch/bringup_launch.xml").read_text(encoding="utf-8")
    bringup = replace_once(
        bringup,
        'default="$(find-pkg-share pinky_navigation)/params/nav2_params.yaml"',
        'default="/home/pinky/.cache/pinky-fleet/nav2/nav2_params.yaml"',
    )
    bringup = replace_once(
        bringup,
        'file="$(find-pkg-share pinky_navigation)/launch/navigation_launch.xml"',
        'file="/home/pinky/.cache/pinky-fleet/nav2/navigation_launch.xml"',
    )

    navigation = (SHARE / "launch/navigation_launch.xml").read_text(encoding="utf-8")
    navigation = replace_once(
        navigation,
        '<remap from="cmd_vel_smoothed" to="cmd_vel"/>',
        '<remap from="cmd_vel_smoothed" to="cmd_vel_nav2"/>',
        expected=2,
    )
    behavior_start = '<node pkg="nav2_behaviors" exec="behavior_server"'
    prefix, behavior_tail = navigation.split(behavior_start, 1)
    behavior_block, suffix = behavior_tail.split("</node>", 1)
    behavior_block = replace_once(
        behavior_block,
        '<remap from="/tf_static" to="tf_static"/>',
        '<remap from="/tf_static" to="tf_static"/>\n            <remap from="cmd_vel" to="cmd_vel_nav2"/>',
    )
    navigation = prefix + behavior_start + behavior_block + "</node>" + suffix

    params = (SHARE / "params/nav2_params.yaml").read_text(encoding="utf-8")
    updates = [
        ("desired_linear_vel: 0.2", "desired_linear_vel: 0.1152", 1),
        ("rotate_to_heading_angular_vel: 1.0", "rotate_to_heading_angular_vel: 0.2", 1),
        ("min_approach_linear_velocity: 0.05", "min_approach_linear_velocity: 0.048", 1),
        ("xy_goal_tolerance: 0.25", "xy_goal_tolerance: 0.02", 1),
        ("update_min_d: 0.15", "update_min_d: 0.05", 1),
        ("yaw_goal_tolerance: 0.25", "yaw_goal_tolerance: 0.05", 1),
        ("movement_time_allowance: 10.0", "movement_time_allowance: 30.0", 1),
        ("required_movement_radius: 0.5", "required_movement_radius: 0.05", 1),
        ("inflation_radius: 0.15", "inflation_radius: 0.08", 2),
        ("footprint_padding: 0.03", "footprint_padding: 0.01", 1),
        ("tolerance: 0.5", "tolerance: 0.1", 1),
        ("max_velocity: [0.25, 0.0, 1.5]", "max_velocity: [0.12, 0.0, 0.3]", 1),
        ("min_velocity: [-0.25, 0.0, -1.5]", "min_velocity: [-0.12, 0.0, -0.3]", 1),
        ("max_accel: [2.5, 0.0, 3.2]", "max_accel: [0.5, 0.0, 0.64]", 1),
        ("max_decel: [-2.5, 0.0, -3.2]", "max_decel: [-0.5, 0.0, -0.64]", 1),
    ]
    for old, new, expected in updates:
        params = replace_once(params, old, new, expected)

    (RUNTIME / "bringup_launch.xml").write_text(bringup, encoding="utf-8")
    (RUNTIME / "navigation_launch.xml").write_text(navigation, encoding="utf-8")
    (RUNTIME / "localization_launch.xml").write_text(
        (SHARE / "launch/localization_launch.xml").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    (RUNTIME / "nav2_params.yaml").write_text(params, encoding="utf-8")
    sources = {
        'installer': RUNTIME.parent / 'install_nav2_runtime.py',
        'route': RUNTIME.parent / 'navigate_route.py',
        'ir': RUNTIME / 'line_following.py',
        'safety': RUNTIME / 'ultrasonic_safety.py',
        'mission': RUNTIME / 'mission_behaviors.py',
    }
    manifest = {name: hashlib.sha256(path.read_bytes()).hexdigest()
                for name, path in sources.items()}
    temporary = RUNTIME / 'source_manifest.json.tmp'
    temporary.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    temporary.replace(RUNTIME / 'source_manifest.json')
    print("NAV2_RUNTIME_READY")


if __name__ == "__main__":
    main()
