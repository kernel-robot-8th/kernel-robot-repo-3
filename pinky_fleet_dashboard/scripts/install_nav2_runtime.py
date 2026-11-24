#!/usr/bin/env python3
"""핑키에 관제 전용 Nav2 launch·파라미터 사본을 만든다.

실행 위치: 로봇 (주행준비 때 runtime_bundle로 ~/.cache/pinky-fleet/에 올라가고 압축 해제 직후 바로 실행된다)
사용처: runtime_bundle.install_command가 만든 SSH 명령의 마지막 단계. app.py는 출력의 NAV2_RUNTIME_READY를 확인한다.
tests/test_fusion.py가 add_ultrasonic_fusion을 import해 검사한다.

설치된 pinky_navigation 패키지 원본은 건드리지 않고 ~/.cache/pinky-fleet/nav2/에 고친 사본을 쓴다.
- 모터 명령 경로: Nav2의 출력(velocity_smoother, behavior_server)을 /cmd_vel 대신 /cmd_vel_nav2로 돌린다.
  그 뒤는 ultrasonic_safety.py(행동 선택, /cmd_vel_selected) → velocity_guard.py(최종 감시기, /cmd_vel)다.
  /cmd_vel은 최종 감시기 하나만 발행해야 모든 명령이 같은 안전 검사를 거친다.
- 속도·허용 오차를 핑키와 2.2 x 2.6 m 셀에 맞게 낮추고, 차체 footprint·keepout 필터·라이다+초음파 퓨전 층을 넣는다.
- 원문 문자열이 예상과 다르면(패키지 버전 변경 등) 조용히 넘어가지 않고 실패한다.
- 끝에 올라간 소스들의 sha256을 source_manifest.json에 기록한다.
"""

from __future__ import annotations

from pathlib import Path
import hashlib
import json
import yaml


SHARE = Path("/home/pinky/pinky_pro/install/pinky_navigation/share/pinky_navigation")
RUNTIME = Path("/home/pinky/.cache/pinky-fleet/nav2")  # 런타임 번들이 풀리는 곳이자 고친 사본을 쓰는 곳


def replace_once(text: str, old: str, new: str, expected: int = 1) -> str:
    """old가 정확히 expected번 나올 때만 모두 new로 바꾼다. 개수가 다르면 RuntimeError (원본이 바뀐 것을 잡는다)."""
    count = text.count(old)
    if count != expected:
        raise RuntimeError(f"expected {expected} occurrence(s), found {count}: {old}")
    return text.replace(old, new)


ULTRASONIC_SOURCES = {
    # scripts/fusion_grid.py가 라이다와 초음파를 자체 확률 격자에서 합치고 Nav2에는 낮은 층만 넘긴다.
    # 두 점구름 모두 base_footprint 기준 초음파 높이(약 3.7 cm, 5 cm voxel의 맨 아래 줄)에 있어 라이다 평면
    # (약 12.5 cm)보다 낮다. 점유된 낮은 칸은 mark하고, 격자 가장자리의 고리는 clear한다. 그래서 창 안의 Nav2 낮은 칸은
    # 퓨전 격자를 따르고, 라이다 자신의 칸은 건드리지 않는다. 1.5 m는 2 m 창의 모서리까지 닿는 거리다.
    'fusion_low_mark': {'topic': '/fusion/low_marks', 'data_type': 'PointCloud2', 'marking': True,
                        'clearing': False, 'min_obstacle_height': 0.0, 'max_obstacle_height': 0.10,
                        'obstacle_max_range': 1.5, 'obstacle_min_range': 0.0},
    'fusion_low_clear': {'topic': '/fusion/low_clear', 'data_type': 'PointCloud2', 'marking': False,
                         'clearing': True, 'min_obstacle_height': 0.0, 'max_obstacle_height': 0.10,
                         'raytrace_max_range': 1.5, 'raytrace_min_range': 0.0},
}


def add_ultrasonic_fusion(parsed: dict, include_global: bool = False) -> None:
    """라이다 + 초음파를 높이 칸으로: 지역 costmap의 VoxelLayer가 스캔과 퓨전된 낮은 층을 함께 받는다.

    기본은 지역 costmap만(2026-11-16 계획): 지역 costmap은 로봇과 함께 움직이는 창이라 잘못 찍힌 초음파 표시는
    로봇이 지나가면 잊힌다. 전역 costmap에서는 로봇이 그쪽을 다시 볼 때까지 남아, 2.2 x 2.6 m 셀에서는 경로를
    막을 수 있다. include_global이면(낮은 상자 시험 뒤) 전역 2D ObstacleLayer도 같은 소스의 VoxelLayer가 된다.
    2D 층에서는 낮은 상자 위를 지나는 라이다 광선이 초음파가 표시한 것을 지워 버리기 때문이다. 한 기둥(column)은
    로봇 높이 안의 칸 하나라도 점유되면 장애물이다. publish_voxel_map은 끈다: voxel 격자를 읽는 곳이 없고,
    Raspberry Pi에서 매 주기 발행하면 CPU를 쓴다."""
    layers = [('local_costmap', 'voxel_layer')] + ([('global_costmap', 'obstacle_layer')] if include_global else [])
    for name, layer_name in layers:
        section = parsed[name][name]['ros__parameters']
        if layer_name not in section.get('plugins', []):
            raise RuntimeError(f"{name}에 {layer_name}가 없습니다")
        layer = section[layer_name]
        layer['plugin'] = 'nav2_costmap_2d::VoxelLayer'
        # 원본에 값이 없을 때만 채운다: 0 m부터 5 cm 칸 16개(0.8 m)까지. mark_threshold 0 = 칸 하나만 차도 장애물.
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

    # bringup launch가 원본 대신 이 사본의 파라미터와 navigation launch를 쓰게 경로를 바꾼다.
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

    # Nav2 속도 출력을 /cmd_vel_nav2로 돌린다(일반 실행·composition 실행 두 곳이라 expected=2).
    # 바퀴로 가는 /cmd_vel은 최종 감시기 velocity_guard.py만 발행한다.
    navigation = (SHARE / "launch/navigation_launch.xml").read_text(encoding="utf-8")
    navigation = replace_once(
        navigation,
        '<remap from="cmd_vel_smoothed" to="cmd_vel"/>',
        '<remap from="cmd_vel_smoothed" to="cmd_vel_nav2"/>',
        expected=2,
    )
    # behavior_server(spin·backup 같은 복구 동작)는 smoother를 거치지 않고 cmd_vel에 직접 쓰므로 따로 remap을 넣는다.
    # 그 <node> 블록 안에서만 바꾸려고 블록을 잘라 낸 뒤 다시 붙인다.
    behavior_start = '<node pkg="nav2_behaviors" exec="behavior_server"'
    prefix, behavior_tail = navigation.split(behavior_start, 1)
    behavior_block, suffix = behavior_tail.split("</node>", 1)
    behavior_block = replace_once(
        behavior_block,
        '<remap from="/tf_static" to="tf_static"/>',
        '<remap from="/tf_static" to="tf_static"/>\n            <remap from="cmd_vel" to="cmd_vel_nav2"/>',
    )
    navigation = prefix + behavior_start + behavior_block + "</node>" + suffix
    # composition으로 실행되는 behavior_server도 똑같이 한다(use_composition:=True는 Nav2를 한 프로세스로 돌려
    # 바쁜 Raspberry Pi에서 훨씬 빨리 뜬다). 그 spin/backup도 감시기 사슬을 거쳐서만 모터에 닿아야 한다.
    composed_start = '<composable_node pkg="nav2_behaviors" plugin="behavior_server::BehaviorServer" name="behavior_server">'
    prefix, composed_tail = navigation.split(composed_start, 1)
    composed_block, suffix = composed_tail.split("</composable_node>", 1)
    composed_block = replace_once(
        composed_block,
        '<remap from="/tf_static" to="tf_static"/>',
        '<remap from="/tf_static" to="tf_static"/>\n            <remap from="cmd_vel" to="cmd_vel_nav2"/>',
    )
    navigation = prefix + composed_start + composed_block + "</composable_node>" + suffix

    # map-proto.yaml일 때만: 금지 구역 마스크 서버·필터 정보 서버와 그 둘을 켜는 lifecycle manager를 launch에 더한다.
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

    # 원본 nav2_params.yaml의 값을 문자열 그대로 찾아 바꾼다(원본 값 → 관제 값, 나와야 하는 횟수). 단위는 m, m/s, rad,
    # rad/s, s. 요지: 직선 0.1152 m/s, 회전 0.2 rad/s로 느리게. 도착 허용 오차 2 cm·0.05 rad. 막힘 판정은 30초 동안
    # 5 cm 이상 못 움직였을 때로 늘린다(느린 속도라 원본 10초·0.5 m면 멀쩡히 가는 중에도 실패로 본다).
    # update_min_d 0.05: AMCL이 5 cm마다 갱신. velocity_smoother 한도 0.12 m/s·0.3 rad/s, 가감속도 원본의 1/5.
    # inflation 0.08 m·footprint_padding 0.01 m는 좁은 통로용이고, 아래에서 navigation.json 값으로 다시 덮을 수 있다.
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

    # 이제부터는 YAML을 파싱해 고친다. footprint는 config/robot_geometry.json의 반길이·반폭(m)으로 만든 사각형.
    dimensions=runtime_config['geometry']
    parsed=yaml.safe_load(params)
    x,y=dimensions['half_length_m'],dimensions['half_width_m']
    for name in ('local_costmap','global_costmap'):
        section=parsed[name][name]['ros__parameters']
        section['footprint']=json.dumps([[x,y],[x,-y],[-x,-y],[-x,y]])
        section['footprint_padding']=dimensions['padding_m']
        if keepout_enabled:
            # 마스크 칸을 치명(lethal) 비용으로 덮어 Nav2가 금지 구역으로 경로를 만들지 않게 한다.
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
        # Nav2 자유 주행은 경로를 스스로 짠다. 라인 주행용으로 맞춘 0.08 m에서는 경로가 keepout 모서리에 바짝
        # 붙다가 controller가 앞에 충돌을 보고 멈췄다(2026-11-16). 핑키 기본값은 0.15 m다. 0.05~0.5 m만 받는다.
        for name in ('local_costmap', 'global_costmap'):
            parsed[name][name]['ros__parameters']['inflation_layer']['inflation_radius'] = float(inflation)
    # 48 cm 통로가 있는 이 2.2 x 2.6 m 셀에 맞춘 값(2026-11-17). inflation_cost_scaling: 비용이 더 가파르게 떨어져
    # 통로 한가운데가 뚜렷한 골짜기가 된다(controller가 따로 가진 값도 같아야 한다). lookahead_m: 라인 코스의
    # 0.3~0.9 m는 통로 너머를 봐서 모서리를 잘라 먹었다. goal_xy_tolerance_m: 2 cm는 멈출 때마다 핑키가 꼼지락거렸다.
    # 각 값은 navigation.json에 있고 범위 안일 때만 적용한다(이상한 값이 로봇에 가지 않게).
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
            if 'cost_scaling_dist' in plugin:  # costmap이 가진 inflation 반경보다 멀리 보면 안 된다
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
        # 넓게 돌기(2026-11-16): 방향 오차가 이 값보다 작으면 controller가 제자리 회전 대신 곡선으로 경로에 올라탄다.
        # 예전에는 벽 바로 옆에서 멈춰 제자리 회전을 했다.
        for plugin in parsed['controller_server']['ros__parameters'].values():
            if isinstance(plugin, dict) and 'rotate_to_heading_min_angle' in plugin:
                plugin['rotate_to_heading_min_angle'] = float(rotate)
    # 고친 사본들을 쓴다. localization launch는 바꿀 것이 없어 그대로 복사한다.
    params=yaml.safe_dump(parsed,sort_keys=False)
    (RUNTIME / "bringup_launch.xml").write_text(bringup, encoding="utf-8")
    (RUNTIME / "navigation_launch.xml").write_text(navigation, encoding="utf-8")
    (RUNTIME / "localization_launch.xml").write_text(
        (SHARE / "launch/localization_launch.xml").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    (RUNTIME / "nav2_params.yaml").write_text(params, encoding="utf-8")
    if keepout_enabled:
        # type 0 = keepout 필터. 마스크 값 = base + multiplier × 픽셀 값이라 base 0, multiplier 1이면 마스크 그대로다.
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
    # 지금 로봇에 깔린 소스의 해시. 관제는 이 매니페스트로 로봇이 PC와 같은 코드를 돌리는지 확인한다.
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
