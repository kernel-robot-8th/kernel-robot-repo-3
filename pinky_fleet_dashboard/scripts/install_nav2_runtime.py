#!/usr/bin/env python3
"""Create a dashboard-owned Nav2 launch/config copy on a Pinky."""

from __future__ import annotations

from pathlib import Path
import hashlib
import json
import yaml


SHARE = Path("/home/pinky/pinky_pro/install/pinky_navigation/share/pinky_navigation")
RUNTIME = Path("/home/pinky/.cache/pinky-fleet/nav2")


def replace_once(text: str, old: str, new: str, expected: int = 1) -> str:
    count = text.count(old)
    if count != expected:
        raise RuntimeError(f"expected {expected} occurrence(s), found {count}: {old}")
    return text.replace(old, new)


ULTRASONIC_SOURCES = {
    # scripts/fusion_grid.py fuses the LiDAR and the ultrasonic in its own probabilistic grid and hands Nav2 the low
    # layer. Both clouds are in base_footprint at the ultrasonic's height (about 3.7 cm, the lowest 5 cm voxel row),
    # below the LiDAR plane (about 12.5 cm): the occupied low cells mark, and a ring at the grid's edge clears, so
    # Nav2's low cells inside the window follow the fused grid while the LiDAR's own cells are never touched.
    # 1.5 m covers the corners of the 2 m window.
    'fusion_low_mark': {'topic': '/fusion/low_marks', 'data_type': 'PointCloud2', 'marking': True,
                        'clearing': False, 'min_obstacle_height': 0.0, 'max_obstacle_height': 0.10,
                        'obstacle_max_range': 1.5, 'obstacle_min_range': 0.0},
    'fusion_low_clear': {'topic': '/fusion/low_clear', 'data_type': 'PointCloud2', 'marking': False,
                         'clearing': True, 'min_obstacle_height': 0.0, 'max_obstacle_height': 0.10,
                         'raytrace_max_range': 1.5, 'raytrace_min_range': 0.0},
}


def add_ultrasonic_fusion(parsed: dict, include_global: bool = False) -> None:
    """LiDAR + ultrasonic in height cells: the local costmap's VoxelLayer is fed by the scan and the fused low layer.

    Local only by default (2026-11-16 plan): the local costmap is a window that moves with the robot, so a wrong
    ultrasonic mark is forgotten once the robot has passed. In the global costmap it would stay until the robot looks
    that way again, and in the 2.2 x 2.6 m cell that can block the plan. With include_global (after the low-box test)
    the global 2D ObstacleLayer also becomes a VoxelLayer with the same sources, since a 2D layer lets LiDAR rays
    passing over a low box clear what the ultrasonic marked. A column is an obstacle when any of its cells within the
    robot's height is occupied. publish_voxel_map stays off: nothing reads the voxel grid, and on a Raspberry Pi
    publishing it every cycle costs CPU."""
    layers = [('local_costmap', 'voxel_layer')] + ([('global_costmap', 'obstacle_layer')] if include_global else [])
    for name, layer_name in layers:
        section = parsed[name][name]['ros__parameters']
        if layer_name not in section.get('plugins', []):
            raise RuntimeError(f"{name}에 {layer_name}가 없습니다")
        layer = section[layer_name]
        layer['plugin'] = 'nav2_costmap_2d::VoxelLayer'
        layer.setdefault('origin_z', 0.0)
        layer.setdefault('z_resolution', 0.05)
        layer.setdefault('z_voxels', 16)
        layer.setdefault('max_obstacle_height', 2.0)
        layer.setdefault('mark_threshold', 0)
        layer['publish_voxel_map'] = False
        sources = str(layer.get('observation_sources', 'scan')).split()
        for source, settings in ULTRASONIC_SOURCES.items():
            if source not in sources:
                sources.append(source)
            layer[source] = dict(settings)
        layer['observation_sources'] = ' '.join(sources)


def main() -> None:
    RUNTIME.mkdir(parents=True, exist_ok=True)
    runtime_config = json.loads((RUNTIME / "runtime_config.json").read_text(encoding="utf-8"))
    keepout_enabled = runtime_config.get("keepout_enabled", False)

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
    # The same for the composed behavior_server (use_composition:=True runs Nav2 as one process, which a busy
    # Raspberry Pi brings up far faster): its spin/backup must also reach the motors only through the guard chain.
    composed_start = '<composable_node pkg="nav2_behaviors" plugin="behavior_server::BehaviorServer" name="behavior_server">'
    prefix, composed_tail = navigation.split(composed_start, 1)
    composed_block, suffix = composed_tail.split("</composable_node>", 1)
    composed_block = replace_once(
        composed_block,
        '<remap from="/tf_static" to="tf_static"/>',
        '<remap from="/tf_static" to="tf_static"/>\n            <remap from="cmd_vel" to="cmd_vel_nav2"/>',
    )
    navigation = prefix + composed_start + composed_block + "</composable_node>" + suffix

    if keepout_enabled:
        keepout_nodes = '''
    <node pkg="nav2_map_server" exec="map_server" name="keepout_filter_mask_server" output="screen">
        <param from="/home/pinky/.cache/pinky-fleet/nav2/keepout_servers.yaml"/>
    </node>
    <node pkg="nav2_map_server" exec="costmap_filter_info_server" name="keepout_costmap_filter_info_server" output="screen">
        <param from="/home/pinky/.cache/pinky-fleet/nav2/keepout_servers.yaml"/>
    </node>
    <node pkg="nav2_lifecycle_manager" exec="lifecycle_manager" name="lifecycle_manager_keepout" output="screen">
        <param from="/home/pinky/.cache/pinky-fleet/nav2/keepout_servers.yaml"/>
    </node>
'''
        navigation = replace_once(navigation, "</launch>", keepout_nodes + "</launch>")

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

    dimensions=runtime_config['geometry']
    parsed=yaml.safe_load(params)
    x,y=dimensions['half_length_m'],dimensions['half_width_m']
    for name in ('local_costmap','global_costmap'):
        section=parsed[name][name]['ros__parameters']
        section['footprint']=json.dumps([[x,y],[x,-y],[-x,-y],[-x,y]])
        section['footprint_padding']=dimensions['padding_m']
        if keepout_enabled:
            section['filters'] = ['keepout_filter']
            section['keepout_filter'] = {
                'plugin': 'nav2_costmap_2d::KeepoutFilter',
                'enabled': True,
                'filter_info_topic': '/keepout_costmap_filter_info',
                'transform_tolerance': 0.1,
                'override_lethal_cost': True,
                'lethal_override_cost': 252,
            }
    navigation_settings = runtime_config.get('navigation') or {}
    if navigation_settings.get('ultrasonic_fusion'):
        add_ultrasonic_fusion(parsed, include_global=bool(navigation_settings.get('ultrasonic_fusion_global')))
    inflation = (runtime_config.get('navigation') or {}).get('inflation_radius_m')
    if isinstance(inflation, (int, float)) and 0.05 <= inflation <= 0.5:
        # Free Nav2 driving plans its own way: 0.08 m (tuned for line following) let paths hug the keepout corner
        # until the controller saw a collision ahead (2026-11-16). Pinky's own default is 0.15 m.
        for name in ('local_costmap', 'global_costmap'):
            parsed[name][name]['ros__parameters']['inflation_layer']['inflation_radius'] = float(inflation)
    rotate = navigation_settings.get('rotate_to_heading_min_angle_rad')
    if isinstance(rotate, (int, float)) and 0.1 <= rotate <= 1.6:
        # Wide turns (2026-11-16): below this heading error the controller curves onto the path instead of
        # stopping to turn in place, which it did right beside the walls.
        for plugin in parsed['controller_server']['ros__parameters'].values():
            if isinstance(plugin, dict) and 'rotate_to_heading_min_angle' in plugin:
                plugin['rotate_to_heading_min_angle'] = float(rotate)
    params=yaml.safe_dump(parsed,sort_keys=False)
    (RUNTIME / "bringup_launch.xml").write_text(bringup, encoding="utf-8")
    (RUNTIME / "navigation_launch.xml").write_text(navigation, encoding="utf-8")
    (RUNTIME / "localization_launch.xml").write_text(
        (SHARE / "launch/localization_launch.xml").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    (RUNTIME / "nav2_params.yaml").write_text(params, encoding="utf-8")
    if keepout_enabled:
        keepout_servers = {
            'keepout_filter_mask_server': {
                'ros__parameters': {
                    'topic_name': '/keepout_filter_mask',
                    'yaml_filename': str(RUNTIME / 'keepout_mask.yaml'),
                    'frame_id': 'map',
                    'use_sim_time': False,
                },
            },
            'keepout_costmap_filter_info_server': {
                'ros__parameters': {
                    'type': 0,
                    'filter_info_topic': '/keepout_costmap_filter_info',
                    'mask_topic': '/keepout_filter_mask',
                    'base': 0.0,
                    'multiplier': 1.0,
                    'use_sim_time': False,
                },
            },
            'lifecycle_manager_keepout': {
                'ros__parameters': {
                    'autostart': True,
                    'node_names': [
                        'keepout_filter_mask_server',
                        'keepout_costmap_filter_info_server',
                    ],
                    'use_sim_time': False,
                },
            },
        }
        (RUNTIME / 'keepout_servers.yaml').write_text(
            yaml.safe_dump(keepout_servers, sort_keys=False), encoding='utf-8'
        )
    sources = {
        'installer': RUNTIME.parent / 'install_nav2_runtime.py',
        'route': RUNTIME.parent / 'navigate_route.py',
        'ir': RUNTIME / 'line_following.py',
        'safety': RUNTIME / 'ultrasonic_safety.py',
        'mission': RUNTIME / 'mission_behaviors.py',
        'final_guard': RUNTIME / 'velocity_guard.py',
        'geometry': RUNTIME / 'runtime_config.json',
        'motion_gate': RUNTIME / 'motion_gate.py',
        'traffic': RUNTIME / 'traffic.py',
        'fusion_grid': RUNTIME / 'fusion_grid.py',
    }
    if keepout_enabled:
        sources.update({
            'keepout_mask': RUNTIME / 'keepout_mask.pgm',
            'keepout_metadata': RUNTIME / 'keepout_mask.yaml',
            'keepout_servers': RUNTIME / 'keepout_servers.yaml',
        })
    manifest = {name: hashlib.sha256(path.read_bytes()).hexdigest()
                for name, path in sources.items()}
    temporary = RUNTIME / 'source_manifest.json.tmp'
    temporary.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    temporary.replace(RUNTIME / 'source_manifest.json')
    print("NAV2_RUNTIME_READY")


if __name__ == "__main__":
    main()
