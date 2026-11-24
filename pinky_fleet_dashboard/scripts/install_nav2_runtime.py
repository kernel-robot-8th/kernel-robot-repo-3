#!/usr/bin/env python3
"""핑키에 관제 전용 Nav2 launch·파라미터 사본을 ~/.cache/pinky-fleet/nav2/에 만든다(원본 패키지는 그대로).

실행 위치: 로봇 (runtime_bundle 설치 명령의 마지막 단계, app.py가 NAV2_RUNTIME_READY를 확인).
- Nav2 출력을 /cmd_vel_nav2로 돌린다 → ultrasonic_safety → velocity_guard. /cmd_vel은 최종 감시기만 발행한다.
- 속도·허용 오차를 2.2 x 2.6 m 셀에 맞추고 footprint·keepout·초음파 퓨전 층을 넣는다.
- 원문이 예상과 다르면 실패한다. 끝에 소스 sha256을 source_manifest.json에 남긴다.
"""

from __future__ import annotations

from pathlib import Path
import hashlib
import json
import yaml


SHARE = Path("/home/pinky/pinky_pro/install/pinky_navigation/share/pinky_navigation")
RUNTIME = Path("/home/pinky/.cache/pinky-fleet/nav2")


def replace_once(text: str, old: str, new: str, expected: int = 1) -> str:
    """old가 정확히 expected번일 때만 바꾼다(아니면 RuntimeError: 원본 변경 감지)."""
    count = text.count(old)
    if count != expected:
        raise RuntimeError(f"expected {expected} occurrence(s), found {count}: {old}")
    return text.replace(old, new)


ULTRASONIC_SOURCES = {
    # fusion_grid.py가 라이다+초음파를 합친 낮은 층(약 3.7 cm, 라이다 평면 12.5 cm 아래)만 넘긴다.
    # 점유 칸은 mark, 격자 가장자리 고리는 clear. 1.5 m는 2 m 창 모서리까지의 거리.
    'fusion_low_mark': {'topic': '/fusion/low_marks', 'data_type': 'PointCloud2', 'marking': True,
                        'clearing': False, 'min_obstacle_height': 0.0, 'max_obstacle_height': 0.10,
                        'obstacle_max_range': 1.5, 'obstacle_min_range': 0.0},
    'fusion_low_clear': {'topic': '/fusion/low_clear', 'data_type': 'PointCloud2', 'marking': False,
                         'clearing': True, 'min_obstacle_height': 0.0, 'max_obstacle_height': 0.10,
                         'raytrace_max_range': 1.5, 'raytrace_min_range': 0.0},
}


def add_ultrasonic_fusion(parsed: dict, include_global: bool = False) -> None:
    """라이다 + 초음파를 지역 costmap VoxelLayer로 받는다(2026-11-16: 잘못 찍힌 초음파 표시가 전역에 남아
    경로를 막지 않도록 기본은 지역만). include_global이면 전역도 VoxelLayer로 바꾼다. publish_voxel_map은 끈다(Pi CPU)."""
    layers = [('local_costmap', 'voxel_layer')] + ([('global_costmap', 'obstacle_layer')] if include_global else [])
    for name, layer_name in layers:
        section = parsed[name][name]['ros__parameters']
        if layer_name not in section.get('plugins', []):
            raise RuntimeError(f"{name}에 {layer_name}가 없습니다")
        layer = section[layer_name]
        layer['plugin'] = 'nav2_costmap_2d::VoxelLayer'
        # 없을 때만 채움: 5 cm 칸 16개(0.8 m), mark_threshold 0 = 한 칸만 차도 장애물.
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

    # Nav2 속도 출력을 /cmd_vel_nav2로(일반·composition 두 곳이라 expected=2).
    navigation = (SHARE / "launch/navigation_launch.xml").read_text(encoding="utf-8")
    navigation = replace_once(
        navigation,
        '<remap from="cmd_vel_smoothed" to="cmd_vel"/>',
        '<remap from="cmd_vel_smoothed" to="cmd_vel_nav2"/>',
        expected=2,
    )
    # behavior_server(spin·backup)는 smoother를 거치지 않아 그 <node> 블록에만 따로 remap을 넣는다.
    behavior_start = '<node pkg="nav2_behaviors" exec="behavior_server"'
    prefix, behavior_tail = navigation.split(behavior_start, 1)
    behavior_block, suffix = behavior_tail.split("</node>", 1)
    behavior_block = replace_once(
        behavior_block,
        '<remap from="/tf_static" to="tf_static"/>',
        '<remap from="/tf_static" to="tf_static"/>\n            <remap from="cmd_vel" to="cmd_vel_nav2"/>',
    )
    navigation = prefix + behavior_start + behavior_block + "</node>" + suffix
    # composition 실행(Pi에서 빨리 뜸)의 behavior_server도 같은 감시기 사슬을 거치게 한다.
    composed_start = '<composable_node pkg="nav2_behaviors" plugin="behavior_server::BehaviorServer" name="behavior_server">'
    prefix, composed_tail = navigation.split(composed_start, 1)
    composed_block, suffix = composed_tail.split("</composable_node>", 1)
    composed_block = replace_once(
        composed_block,
        '<remap from="/tf_static" to="tf_static"/>',
        '<remap from="/tf_static" to="tf_static"/>\n            <remap from="cmd_vel" to="cmd_vel_nav2"/>',
    )
    navigation = prefix + composed_start + composed_block + "</composable_node>" + suffix

    # map-proto.yaml일 때만 keepout 마스크·필터 서버를 더한다.
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

    # 원본 nav2_params.yaml 값 → 관제 값(나와야 하는 횟수). 직선 0.1152 m/s·회전 0.2 rad/s, 도착 오차 2 cm·0.05 rad,
    # 막힘 판정 30초·5 cm(느린 속도라 원본 10초·0.5 m면 정상 주행도 실패), smoother 0.12 m/s·0.3 rad/s.
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

    # 이제 YAML을 파싱해 고친다. footprint는 robot_geometry.json 반치수(m)의 사각형.
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
        # 0.08 m는 경로가 keepout 모서리에 붙어 멈췄다(2026-11-16). 0.05~0.5 m만 받는다.
        for name in ('local_costmap', 'global_costmap'):
            parsed[name][name]['ros__parameters']['inflation_layer']['inflation_radius'] = float(inflation)
    # 48 cm 통로에 맞춘 navigation.json 값(2026-11-17), 범위 안일 때만 적용. 자세한 설명: pinky_fleet_dashboard/README.md
    controller_plugins = [plugin for plugin in parsed['controller_server']['ros__parameters'].values() if isinstance(plugin, dict)]
    scaling = navigation_settings.get('inflation_cost_scaling')
    if isinstance(scaling, (int, float)) and 1.0 <= scaling <= 20.0:
        for name in ('local_costmap', 'global_costmap'):
            parsed[name][name]['ros__parameters']['inflation_layer']['cost_scaling_factor'] = float(scaling)
        for plugin in controller_plugins:
            if 'inflation_cost_scaling_factor' in plugin:
                plugin['inflation_cost_scaling_factor'] = float(scaling)
    if isinstance(inflation, (int, float)) and 0.05 <= inflation <= 0.5:
        for plugin in controller_plugins:
            if 'cost_scaling_dist' in plugin:
                plugin['cost_scaling_dist'] = min(float(plugin['cost_scaling_dist']), float(inflation))
    lookahead = navigation_settings.get('lookahead_m')
    if isinstance(lookahead, dict) and all(isinstance(lookahead.get(k), (int, float)) for k in ('min', 'max')) \
            and 0.1 <= lookahead['min'] <= lookahead['max'] <= 1.0:
        for plugin in controller_plugins:
            if 'min_lookahead_dist' in plugin:
                plugin.update(min_lookahead_dist=float(lookahead['min']), max_lookahead_dist=float(lookahead['max']),
                              lookahead_dist=float((lookahead['min'] + lookahead['max']) / 2),
                              curvature_lookahead_dist=float(lookahead['max']))
    goal_xy = navigation_settings.get('goal_xy_tolerance_m')
    if isinstance(goal_xy, (int, float)) and 0.01 <= goal_xy <= 0.15:
        for plugin in controller_plugins:
            if 'xy_goal_tolerance' in plugin:
                plugin['xy_goal_tolerance'] = float(goal_xy)
    rotate = navigation_settings.get('rotate_to_heading_min_angle_rad')
    if isinstance(rotate, (int, float)) and 0.1 <= rotate <= 1.6:
        # 방향 오차가 이보다 작으면 제자리 회전 대신 곡선으로 경로에 오른다(2026-11-16).
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
        # type 0 = keepout. base 0·multiplier 1이면 마스크 그대로.
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
    # 관제가 로봇이 PC와 같은 코드를 돌리는지 이 해시로 확인한다.
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
