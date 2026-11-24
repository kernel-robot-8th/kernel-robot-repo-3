#!/usr/bin/env python3
"""Nav2와 핑키 차체 사이의 초음파 정지·회전·저속 전진 게이트이자 행동 선택 노드.

실행 위치: 로봇 (runtime_bundle로 ~/.cache/pinky-fleet/nav2/에 올라가고 app.py가 `ultrasonic_safety`로 띄움)
/cmd_vel_nav2를 받아 고른 행동을 /cmd_vel_selected로만 낸다. /cmd_vel은 velocity_guard.py만 발행한다(최종 검사 우회 불가).
20 Hz 우선순위: 작업자 정지 > 초음파 신선도 > 경로 컨텍스트(1.5 s) > 양보 > 구역 허가 > 적재 자리 진입·탈출 >
초음파 회피 > 도킹·교차로 > Nav2 전달(IR 보조). self.state는 초음파 상태, self.line_state는 보이는 행동 이름.
센서·TF·컨텍스트가 오래되면(대개 0.6 s) 0 속도를 낸다(fail closed).
"""

from __future__ import annotations

import math
import time
import json
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import UInt16MultiArray, String
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from tf2_ros import Buffer, TransformListener, TransformException
if __package__:
    from .line_following import LineAssist, default_profile, swept_clear, TemplateEstimator, validate_profile, normalize, surface_observation
    from .mission_behaviors import BackOff, Crossing, Docking, DockEntry, DockExit, GiveWay, UltrasonicDetour
    from .motion_gate import geometry, swept_footprint
    from .traffic import required_zones, lease_allows, LeaseReceiver
    from .control_journal import ControlJournal
else:
    from line_following import LineAssist, default_profile, swept_clear, TemplateEstimator, validate_profile, normalize, surface_observation
    from mission_behaviors import BackOff, Crossing, Docking, DockEntry, DockExit, GiveWay, UltrasonicDetour
    from motion_gate import geometry, swept_footprint
    from traffic import required_zones, lease_allows, LeaseReceiver
    from control_journal import ControlJournal


class UltrasonicSafety(Node):
    def __init__(self) -> None:
        super().__init__("pinky_ultrasonic_safety")
        # navigation.json ultrasonic_stop_m / ultrasonic_clear_m(2026-11-16부터 5 / 8 cm). 정지 3~20 cm, 해제는 정지+2 cm 이상.
        try:
            _nav = json.loads(Path('/home/pinky/.cache/pinky-fleet/nav2/runtime_config.json').read_text()).get('navigation') or {}
        except (OSError, ValueError):
            _nav = {}
        _stop = _nav.get('ultrasonic_stop_m') if isinstance(_nav.get('ultrasonic_stop_m'), (int, float)) else 0.10
        _clear = _nav.get('ultrasonic_clear_m') if isinstance(_nav.get('ultrasonic_clear_m'), (int, float)) else 0.15
        _stop = min(0.20, max(0.03, float(_stop)))
        self.declare_parameter("stop_distance_m", _stop)
        self.declare_parameter("clear_distance_m", max(_stop + 0.02, min(0.30, float(_clear))))
        self.declare_parameter("clear_hold_s", 0.75)
        self.declare_parameter("sensor_timeout_s", 0.60)
        self.nav_command = Twist()
        self.last_command_at = 0.0
        self.range_m: float | None = None
        self.sensor_clear = False
        self.sensor_valid = False
        self.last_sensor_at = 0.0
        self.state = "idle"
        self.state_since = time.monotonic()
        self.clear_since: float | None = None
        self.ir_profile_path = Path('/home/pinky/.cache/pinky-fleet/nav2/ir_profile.json')
        self.ir_profile = default_profile()
        self.profile_error = ''
        self.profile_signature = None
        self.ir_raw = []
        self.last_ir_at = 0.0
        self.scan_points = []
        self.last_scan_at = 0.0
        self.line_assist = LineAssist()
        # route_start.json: 경로마다 한 번 0.03 m/s로 1 s 곧게 나간다.
        self.start_request_path = Path('/home/pinky/.cache/pinky-fleet/route_start.json')
        self.start_token = None
        self.start_deadline = 0.0
        self.start_elapsed = 0.0
        self.start_last_tick = None
        self.line_state = 'disabled'
        self.line_normalized = None
        self.line_error = None
        self.line_estimate = None
        self.observer_model = None
        self.observer_revision = None
        self.route_context_seen = False
        self.segment_key = None
        self.was_corner = False
        self.last_motion_at = 0.
        # 없으면 노드가 시작하지 않는다(차체 치수 필수).
        self.runtime = json.loads(Path('/home/pinky/.cache/pinky-fleet/nav2/runtime_config.json').read_text())
        self.geometry = geometry(self.runtime['geometry'])
        self.detour = UltrasonicDetour(self.geometry)
        self.traffic_request = {}
        self.lease_receiver = LeaseReceiver()
        self.journal = ControlJournal('/home/pinky/.cache/pinky-fleet/logs/behavior.jsonl')
        self.crossing = Crossing()
        self.docking = Docking()
        self.dock_exit = DockExit()
        self.dock_entry = DockEntry()
        # navigation.json ultrasonic_back_off_m: 초음파가 붙잡을 때 후진 거리(m, 0이면 안 함, 최대 0.25).
        _back = _nav.get('ultrasonic_back_off_m', 0.12)
        self.back_off = BackOff(distance_m=min(0.25, max(0.0, float(_back))) if isinstance(_back, (int, float)) else 0.12)
        self.mission_token = None
        self.control_token = None
        self.dock_saved_signature = None
        self.last_crossing_log = None
        self.restore_dock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.line_status_publisher = self.create_publisher(String, '/line/status', 10)
        # 관제 메시지는 자주 다시 오므로 best-effort. 끊기면 신선도 한도로 멈춘다.
        self.create_subscription(String,'/fleet/lease',self.on_lease,qos_profile_sensor_data)
        # 관제의 양보 지시(scripts/yielding.py, 2026-11-24): 오는 동안 조금 후진해 서서 상대가 지나가게 한다.
        self.give_way = GiveWay()
        self.yield_order = None
        self.create_subscription(String,'/fleet/yield',self.on_yield,qos_profile_sensor_data)
        # 5번 적재 자리 근처 천장 카메라가 본 등록 자리와의 어긋남(2026-11-18). 진입 주행만, 신선할 때만 쓴다.
        self.dock_guide = None
        self.create_subscription(String,'/fleet/dock_guide',self.on_dock_guide,qos_profile_sensor_data)
        self.create_subscription(UInt16MultiArray, '/ir_sensor/range', self.on_ir, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.reload_ir_profile)
        self.create_timer(0.2, self.publish_line_status)
        self.reload_ir_profile()
        self.publisher = self.create_publisher(Twist, "/cmd_vel_selected", 1)
        self.create_subscription(Twist, "/cmd_vel_nav2", self.on_command, 10)
        self.create_subscription(Range, "/us_sensor/range", self.on_range, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)
        self.get_logger().info("Ultrasonic safety gate ready; stale sensor data stops the base")

    def reload_ir_profile(self) -> None:
        """ir_profile.json이 바뀌었으면 다시 읽는다(1 s마다)."""
        try:
            if not self.ir_profile_path.exists():
                return
            signature = self.ir_profile_path.stat().st_mtime_ns
            if signature == self.profile_signature:
                return
            profile = json.loads(self.ir_profile_path.read_text(encoding='utf-8'))
            if not isinstance(profile, dict) or not isinstance(profile.get('enabled'), bool):
                raise ValueError('Invalid IR profile')
            self.ir_profile = profile
            self.profile_signature = signature
            self.profile_error = ''
            self.line_assist.reset()
            self.get_logger().info(f"IR profile loaded, enabled={profile['enabled']}")
        except (OSError, ValueError) as exc:
            self.profile_error = str(exc)

    def on_ir(self, msg: UInt16MultiArray) -> None:
        """IR 원시값을 받는다. 세 채널 모두 흰색(0.5 이상)이면 도킹에 알린다."""
        self.ir_raw = list(msg.data) if len(msg.data) == 3 else []
        self.last_ir_at = time.monotonic()
        strengths = normalize(self.ir_raw, self.ir_profile)
        self.docking.observe(strengths is not None and min(strengths) >= .5, self.last_ir_at)

    def on_scan(self, msg: LaserScan) -> None:
        """라이다 스캔을 base_footprint 기준 (x, y) 점(m)으로 바꿔 둔다."""
        try:
            age=(self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
            # stamp가 -0.2~0.6 s 밖이면 버린다(stamp 0이면 수신 시각을 믿음).
            if (msg.header.stamp.sec or msg.header.stamp.nanosec) and not -.2<=age<=.6:return
            if not msg.header.frame_id or not msg.ranges:
                return
            transform = self.tf_buffer.lookup_transform('base_footprint', msg.header.frame_id.lstrip('/'), Time()).transform
            q, t = transform.rotation, transform.translation
            m00, m01 = 1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.z*q.w)
            m10, m11 = 2*(q.x*q.y+q.z*q.w), 1-2*(q.x*q.x+q.z*q.z)
            points, usable = [], 0
            for i, distance in enumerate(msg.ranges):
                if math.isinf(distance) and distance > 0:
                    usable += 1
                elif math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                    usable += 1
                    angle = msg.angle_min+i*msg.angle_increment
                    x, y = distance*math.cos(angle), distance*math.sin(angle)
                    points.append((m00*x+m01*y+t.x, m10*x+m11*y+t.y))
            # +inf도 빈 것으로 쓴다. 쓸 수 있는 빔이 절반 미만이면 버린다.
            if usable < len(msg.ranges)*0.5:
                return
            self.scan_points = points
            self.last_scan_at = time.monotonic()
        except TransformException:
            # IR 보정을 켜려면 base 프레임 스캔 점이 있어야 한다.
            return

    def publish_line_status(self) -> None:
        """5 Hz로 /line/status를 내고 route_feedback.json(navigate_route.py용)을 쓴다."""
        now = time.monotonic()
        state = self.line_state
        if self.line_state.startswith(('docking_', 'crossing_', 'mission_', 'operator_', 'ultrasonic_', 'yield_', 'traffic_')):
            state = self.line_state
        elif self.line_state == 'route_start_straight' and self.state == 'idle' and now-self.last_command_at <= .5:
            state = 'route_start_straight'
        elif not self.ir_profile.get('enabled'):
            state = 'disabled'
        elif self.profile_error:
            state = 'profile_error'
        elif not self.sensor_fresh(now) or not self.sensor_valid:
            state = 'ultrasonic_unavailable'
        elif self.state != 'idle':
            state = 'ultrasonic_'+self.state
        elif now-self.last_command_at > .5:
            state = 'idle'
        estimate = None
        if self.ir_profile.get('mode') == 'template' and not validate_profile(self.ir_profile) and now-self.last_ir_at <= .6 and len(self.ir_raw) == 3:
            if self.observer_revision != self.ir_profile.get('revision'):
                self.observer_model = TemplateEstimator(self.ir_profile['template'])
                self.observer_revision = self.ir_profile.get('revision')
            estimate = self.observer_model.estimate([self.ir_raw[i] for i in self.ir_profile['channel_order']])
        status = dict(version=1, policy='center_first', line_algorithm='fleet_control_v4', dock_protocol='sponge_v1', recovery_policy='parking_mission_v2', traffic_config=self.runtime['traffic_zones'], traffic_request=self.traffic_request, avoidance=self.detour.status(), crossing=self.crossing.status(), docking=self.docking.status(), dock_exit=self.dock_exit.status(), dock_entry=self.dock_entry.status(), dock_guide_fresh=self.fresh_dock_guide(now) is not None, classification=self.ir_profile.get('mode'), estimate=estimate, control_estimate=self.line_estimate, cruise_speed_mps=.1152, enabled=bool(self.ir_profile.get('enabled')), state=state,
                      revision=self.ir_profile.get('revision'), normalized=normalize(self.ir_raw,self.ir_profile) if now-self.last_ir_at<=.6 else None, surface_reading=surface_observation(self.ir_raw,self.ir_profile) if self.ir_profile.get('mode')=='calibrated' and now-self.last_ir_at<=.6 else None,
                      error=self.line_error, ir_fresh=now-self.last_ir_at <= .6,
                      lidar_fresh=now-self.last_scan_at <= .6,
                      ultrasonic_fresh=self.sensor_fresh(now) and self.sensor_valid, profile_error=self.profile_error)
        self.line_status_publisher.publish(String(data=json.dumps(status, allow_nan=False)))
        self.write_json('route_feedback.json', dict(token=self.mission_token, monotonic=now,
                        traffic_config=self.runtime['traffic_zones'], traffic_request=self.traffic_request, avoidance=self.detour.status(), crossing=self.crossing.status(), docking=self.docking.status(), dock_exit=self.dock_exit.status(), dock_entry=self.dock_entry.status(), dock_guide=self.fresh_dock_guide(now), state=self.line_state))

    def write_json(self, name, data):
        """~/.cache/pinky-fleet/<name>에 원자적으로 쓴다. 실패하면 False."""
        path = Path('/home/pinky/.cache/pinky-fleet')/name
        try:
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(data, allow_nan=False))
            os.replace(temporary, path)
        except OSError as exc:
            self.get_logger().error(f'Mission state write failed: {exc}')
            return False
        return True

    def restore_dock(self):
        try:
            data = json.loads(Path('/home/pinky/.cache/pinky-fleet/dock_latch.json').read_text())
            if data.get('state') not in Docking.ACTIVE:
                return
            for key in ('token','dock_id','complete','anchored','origin','heading','reversed_once','command_id','message'):
                if key in data:
                    setattr(self.docking, key, data[key])
            self.docking.handled = set(data.get('handled', []))
            # 고정까지 끝난 정지만 그대로, 나머지는 운영자 확인을 기다리는 recovery_hold.
            self.docking.state = 'stopped' if data['state'] == 'stopped' and data.get('anchored') else 'recovery_hold'
        except (OSError, ValueError, TypeError):
            pass

    def fresh_pose(self, frame):
        """frame 기준 base_footprint의 (x m, y m, yaw rad). TF가 0.6 s보다 오래되면 None."""
        try:
            tf = self.tf_buffer.lookup_transform(frame, 'base_footprint', Time())
            age = (self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
            if -.2 <= age <= .6:
                t, q = tf.transform.translation, tf.transform.rotation
                pose = t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                if all(math.isfinite(v) for v in pose):
                    return pose
        except TransformException:
            pass
        return None

    def mission_tick(self, now):
        """임무 행동(적재 자리 진입·탈출, 도킹, 교차로)이 이번 tick 명령을 냈으면 True, 아니면 Nav2 전달로 넘어간다."""
        try:
            context = json.loads(Path('/home/pinky/.cache/pinky-fleet/route_context.json').read_text())
            valid = 0 <= now-float(context['monotonic']) < 1.5 and 'start_xy' in context
        except (OSError, ValueError, TypeError, KeyError):
            valid = False
        if not valid:
            if self.docking.state in Docking.ACTIVE:
                self.line_state = 'route_context_stale'
                self.send()
                return True
            return False
        if context.get('mission_wait'):
            self.line_state = 'mission_wait'
            self.send()
            return True
        if context.get('dock_entry'):
            # 5번 적재 자리로 곧게 들어감(DockEntry): 초음파·회피 없음, 옆은 최종 감시기가 본다.
            self.mission_token = context['token']
            self.state = 'idle'
            self.detour = UltrasonicDetour(self.geometry)
            odom = self.fresh_pose('odom')
            if odom is None or now-self.last_scan_at > .6:
                self.line_state = 'mission_sensor_unavailable'
                self.send()
                return True
            motion = self.dock_entry.update(context, odom, now,
                Path('/home/pinky/.cache/pinky-fleet/dock_entry_latch.json'), self.fresh_dock_guide(now))
            self.line_state = motion.state
            self.send(motion.linear, motion.angular)
            return True
        if context.get('dock_exit'):
            self.mission_token = context['token']
            self.state = 'idle'
            self.detour = UltrasonicDetour(self.geometry)
            odom = self.fresh_pose('odom')
            if odom is None or now-self.last_scan_at > .6:
                self.line_state = 'mission_sensor_unavailable'
                self.send()
                return True
            motion = self.dock_exit.update(context, odom, now,
                Path('/home/pinky/.cache/pinky-fleet/dock_exit_latch.json'))
            self.line_state = motion.state
            self.send(motion.linear, motion.angular)
            return True
        if context.get('transit') or not context.get('use_ir', True):
            return False
        self.mission_token = context['token']
        if self.docking.token != context['token']:
            self.docking.reset(context['token'])
        pose, odom = self.fresh_pose('map'), self.fresh_pose('odom')
        if pose is None or odom is None or now-self.last_ir_at > .6 or now-self.last_scan_at > .6:
            self.line_state = 'mission_sensor_unavailable'
            self.send()
            return True
        strengths = normalize(self.ir_raw, self.ir_profile)
        if strengths is None:
            self.line_state = 'profile_error'
            self.send()
            return True
        all_white = min(strengths) >= .5
        self.docking.observe(all_white, self.last_ir_at)
        # 관제가 SSH로 쓴 도킹 명령·고정 결과. 3 s 넘은 것은 무시한다(재시작 뒤 재실행 방지).
        try:
            command = json.loads(Path('/home/pinky/.cache/pinky-fleet/dock_command.json').read_text())
            if 0 <= now-float(command.get('monotonic', -100.)) <= 3.:
                self.docking.command(command)
        except (OSError, ValueError, TypeError):
            pass
        try:
            anchor = json.loads(Path('/home/pinky/.cache/pinky-fleet/dock_anchor.json').read_text())
            if (self.docking.state == 'anchor_wait' and anchor.get('token') == self.docking.token
                    and anchor.get('dock_id') == self.docking.dock_id and anchor.get('success') is True
                    and 0 <= now-float(anchor['monotonic']) < 3.):
                self.docking.anchored = True
                self.docking.state = 'stopped'
        except (OSError, ValueError, TypeError, KeyError):
            pass
        front_blocked = self.range_m is not None and self.range_m <= float(self.get_parameter('stop_distance_m').value)
        motion = self.docking.update(context, pose, odom, now, front_blocked)
        if self.docking.state in Docking.ACTIVE:
            self.crossing.reset(context['token'])
            self.state, self.clear_since = 'idle', None
        # 상태가 바뀔 때만 dock_latch.json에 저장하고, 실패하면 복구할 수 없으니 fault로 멈춘다.
        signature = (self.docking.state, self.docking.dock_id, self.docking.complete, self.docking.command_id, self.docking.anchored)
        if signature != self.dock_saved_signature:
            data = self.docking.status()
            data.update(origin=self.docking.origin, heading=self.docking.heading, reversed_once=self.docking.reversed_once)
            if self.write_json('dock_latch.json', data):
                self.dock_saved_signature = signature
            elif self.docking.state in Docking.ACTIVE:
                self.docking.state = 'fault'
                motion = type(motion)(0., 0., 'docking_fault') if motion else None
        if motion is None and self.docking.state == 'released' and context.get('end_label') == '5':
            self.line_state = 'docking_released'
            self.send()
            return True
        if motion is None:
            if self.state != 'idle' or front_blocked:
                return False  # 초음파가 더 높은 우선순위
            if now-self.last_command_at > .5 or (abs(self.nav_command.linear.x) <= .001 and abs(self.nav_command.angular.z) <= .001):
                return False
            if self.ir_profile.get('mode') == 'template':
                if self.observer_model is None or self.observer_revision != self.ir_profile.get('revision'):
                    self.observer_model = TemplateEstimator(self.ir_profile['template'])
                    self.observer_revision = self.ir_profile.get('revision')
                estimate = self.observer_model.estimate([self.ir_raw[i] for i in self.ir_profile['channel_order']])
                line_ok = estimate['usable'] and abs(estimate['offset_cm']) <= .5
            else:
                left, right = strengths[0] >= .55, strengths[2] >= .55
                line_ok = strengths[1] >= .55 and not (left and right)
            motion = self.crossing.update(context, pose, odom, line_ok, now,
                                          self.nav_command.linear.x, self.nav_command.angular.z,
                                          ir_required=bool(self.ir_profile.get('enabled')))
            log_key = (self.crossing.token, self.crossing.state, self.crossing.phase, self.crossing.failure_reason)
            if log_key != self.last_crossing_log:
                self.get_logger().info('Crossing transition: '+json.dumps(self.crossing.status()))
                self.last_crossing_log = log_key
        if motion is None:
            return False
        self.line_state = motion.state
        if motion.state == 'crossing_reacquire':
            # 교차로 tick과 직선 추종 사이(구간 변경 포함)에 같은 복구 기억을 유지한다.
            self.segment_key = (context.get('token'), context.get('segment'))
            decision = self.line_assist.decide(.015, 0., self.ir_raw,
                self.ir_profile, now, True, True, self.scan_points,
                route_corner=False, heading_yaw=odom[2])
            self.line_normalized, self.line_error = decision.normalized, decision.error
            self.line_estimate = decision.estimate
            self.line_state = decision.state
            motion.linear, motion.angular = decision.linear, decision.angular
            if decision.state == 'nav2_avoidance':
                # 인위적 전진 요청은 회피 경로가 아니다.
                motion.linear, motion.angular = 0., 0.
                self.line_state = 'lidar_stop'
            if decision.state == 'line_search_failed':
                self.crossing.state = 'fault'
                self.crossing.failure_reason = 'line_search_failed'
        else:
            self.line_assist.reset()
        # 도킹 중에는 Nav2 action을 일부러 취소해 Nav2 신선도가 필요 없다. 라이다 스윕 검사는 항상 한다.
        if not swept_clear(self.scan_points, motion.linear, motion.angular):
            self.line_state = 'lidar_stop'
            self.send()
        else:
            self.send(motion.linear, motion.angular)
        return True

    def on_command(self, msg: Twist) -> None:
        """Nav2 명령을 기억한다. NaN/inf면 지운다."""
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.nav_command, self.last_command_at = Twist(), 0.
            return
        self.nav_command = msg
        self.last_command_at = time.monotonic()

    def on_range(self, msg: Range) -> None:
        """초음파 거리(m)로 유효·해제를 갱신하고, 주행 중 정지 거리 안이면 'stopped'로 바꾼다.

        sensor_pose_measured=false면 Range 프레임 TF로 장착 좌표·시야각을 채운다.
        """
        age=(self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        self.range_timestamped=bool(msg.header.stamp.sec or msg.header.stamp.nanosec)
        if self.range_timestamped and not -.2<=age<=.6:return
        value = float(msg.range)
        if not self.geometry.get('sensor_pose_measured') and msg.header.frame_id:
            try:
                tf=self.tf_buffer.lookup_transform('base_footprint',msg.header.frame_id.lstrip('/'),Time()).transform
                q=tf.rotation
                self.geometry.update(ultrasound_x_m=tf.translation.x,ultrasound_y_m=tf.translation.y,
                    ultrasound_yaw_rad=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
                if 0<msg.field_of_view<=math.pi:self.geometry['ultrasound_fov_rad']=float(msg.field_of_view)
                self.detour.config=geometry(self.geometry)
            except (TransformException,ValueError):pass
        self.last_sensor_at = time.monotonic()
        self.range_m = value if math.isfinite(value) else None
        self.sensor_valid = (math.isinf(value) and value > 0.0) or (
            math.isfinite(value) and msg.min_range <= value <= msg.max_range
        )
        if not self.sensor_valid:
            self.clear_since = None
            return
        # +inf(반사 없음)는 해제, NaN은 무효라 정지 쪽으로 실패한다.
        self.sensor_clear = (math.isinf(value) and value > 0.0) or (
            math.isfinite(value) and value >= float(self.get_parameter("clear_distance_m").value)
        )
        stop_distance = float(self.get_parameter("stop_distance_m").value)
        # Nav2가 움직일 때만 정지로 들어간다. 도킹·적재 자리 진입/탈출 중에는 자리 자체가 보여 제외한다.
        nav_active = time.monotonic()-self.last_command_at <= .5 and (abs(self.nav_command.linear.x) > .001 or abs(self.nav_command.angular.z) > .001)
        if math.isfinite(value) and value <= stop_distance and self.state == "idle" and nav_active and self.docking.state not in Docking.ACTIVE and not getattr(self, "dock_exiting", False) and not getattr(self, "dock_entering", False):
            self.state = "stopped"
            self.state_since = time.monotonic()
            self.clear_since = None
            self.get_logger().warn(f"Obstacle at {value:.2f} m: stopping before turn/creep sequence")

    def sensor_fresh(self, now: float) -> bool:
        timeout = float(self.get_parameter("sensor_timeout_s").value)
        return self.last_sensor_at > 0.0 and now - self.last_sensor_at <= timeout

    def send(self, linear: float = 0.0, angular: float = 0.0) -> None:
        """고른 명령을 /cmd_vel_selected로 내고 behavior.jsonl에 기록한다. 인자 없으면 정지."""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.publisher.publish(cmd)
        self.journal.record('selected_velocity',state=self.line_state,robot_id=self.runtime['robot_id'],
                            run_id=self.mission_token,requested=[linear,angular],ir_raw=self.ir_raw,
                            normalized=self.line_normalized,ultrasound=self.range_m,odom=self.fresh_pose('odom'),
                            avoidance=self.detour.status(),traffic=self.traffic_request)

    def forward_nav_command(self) -> Twist:
        """Nav2 명령을 ±0.12 m/s, ±0.30 rad/s로 자르고 상황에 맞게 바꾼다(이송·IR 끔: 그대로, 경로 시작: 1 s 직진, 그 밖: LineAssist)."""
        safe = Twist()
        safe.linear.x = max(-0.12, min(0.12, self.nav_command.linear.x))
        safe.angular.z = max(-0.30, min(0.30, self.nav_command.angular.z))
        now = time.monotonic()
        route_corner = False
        try:
            context = json.loads(Path('/home/pinky/.cache/pinky-fleet/route_context.json').read_text())
            self.route_context_seen = True
            if not 0 <= now-float(context['monotonic']) < 1.5:
                self.line_state = 'route_context_stale'
                return Twist()
            if context.get('transit') or not context.get('use_ir',True):
                self.line_assist.reset()
                if now-self.last_scan_at > .6:
                    self.line_state='lidar_unavailable'
                    return Twist()
                if not swept_clear(self.scan_points,safe.linear.x,safe.angular.z):
                    self.line_state='lidar_stop'
                    return Twist()
                self.line_state='parking_transit' if context.get('transit') else 'nav2_only'
                return safe
            route_corner = False  # 래치된 모서리 전환은 Crossing이 맡는다.
            key = (context.get('token'), context.get('segment'))
            if key != self.segment_key or (self.was_corner and not route_corner):
                self.line_assist.reset()
                self.segment_key = key
            self.was_corner = route_corner
        except (OSError, ValueError, TypeError, KeyError):
            self.line_state = 'route_context_stale'
            return Twist()
        # 경로마다 한 번 켜지며 IR과 무관하다. 요청 후 10 s 안에 실제로 움직인 시간으로 1 s를 센다.
        try:
            request = json.loads(self.start_request_path.read_text())
            token = request['token']
            requested = float(request['monotonic'])
            if token != self.start_token:
                self.start_token = token
                self.start_deadline = requested + 10.0
                self.start_elapsed = 0.0
                self.start_last_tick = None
        except (OSError, ValueError, KeyError, TypeError):
            self.start_deadline = 0.0
        if now < self.start_deadline and self.start_elapsed < 1.0:
            if now-self.last_scan_at > .6:
                self.line_state = 'lidar_unavailable'
                self.start_last_tick = None
                return Twist()
            if not swept_clear(self.scan_points, .03, 0.0):
                self.line_state = 'lidar_stop'
                self.start_last_tick = None
                return Twist()
            if self.start_last_tick is not None:
                self.start_elapsed += min(.05, now-self.start_last_tick)
            self.start_last_tick = now
            if self.start_elapsed < 1.0:
                self.line_state = 'route_start_straight'
                safe.linear.x, safe.angular.z = .03, 0.0
                return safe
        if self.ir_profile.get('enabled') and self.profile_error:
            self.line_state = 'profile_error'
            return Twist()
        heading_yaw = None
        try:
            odom_tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', Time())
            age = (self.get_clock().now()-Time.from_msg(odom_tf.header.stamp)).nanoseconds/1e9
            if -.2 <= age <= .6:
                q = odom_tf.transform.rotation
                heading_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        except TransformException:
            pass
        decision = self.line_assist.decide(safe.linear.x, safe.angular.z, self.ir_raw,
            self.ir_profile, now, now-self.last_ir_at <= .6,
            now-self.last_scan_at <= .6, self.scan_points, route_corner=route_corner, heading_yaw=heading_yaw)
        self.line_state, self.line_normalized, self.line_error = decision.state, decision.normalized, decision.error
        self.line_estimate = decision.estimate
        safe.linear.x, safe.angular.z = decision.linear, decision.angular
        return safe

    def tick(self) -> None:
        """20 Hz 행동 선택(우선순위는 모듈 docstring). 매 tick 한 번 send()한다."""
        now = time.monotonic()
        if Path('/home/pinky/.cache/pinky-fleet/stop_latch.json').exists():
            self.nav_command = Twist()
            self.last_command_at = 0.
            self.state = 'idle'
            self.detour = UltrasonicDetour(self.geometry)
            self.line_state = 'operator_stopped'
            self.send()
            return
        if not self.sensor_fresh(now) or not self.sensor_valid:
            self.start_last_tick = None
            self.send()
            return
        try:
            context = json.loads(Path('/home/pinky/.cache/pinky-fleet/route_context.json').read_text())
            valid = 0 <= now-float(context['monotonic']) < 1.5
        except (OSError, ValueError, TypeError, KeyError):
            valid = False
        if valid and getattr(self,'control_token',None)!=context.get('token'):
            if getattr(self,'control_token',None) is not None:
                self.state='idle';self.detour=UltrasonicDetour(self.geometry);self.line_assist.reset()
            self.control_token=context.get('token')
        if valid:
            self.traffic_request = dict(token=context.get('token'),zones=required_zones(context,self.fresh_pose('map'),self.runtime['traffic_zones'])) if self.fresh_pose('map') else {}
        else:self.traffic_request = {}
        # 회피 도중 컨텍스트가 끊기면 fault(재개는 새 경로로).
        if not valid or context.get('mission_wait'):
            if self.detour.phase not in {'idle','fault'}:
                self.detour.phase,self.detour.failure='fault','route_context_expired'
                self.state='fault'
            else:self.state=self.detour.phase
            self.line_state = 'route_context_stale' if not valid else 'mission_wait'
            self.send()
            return
        # 양보 지시면 주행·대기 중 모두 후진해 선다(통로 입구에서 기다리는 쪽이 길을 막는다). 적재 자리 진입·탈출 중엔 안 함.
        if context.get('dock_exit') or context.get('dock_entry'):
            self.give_way.update(None, None, now, False)
        else:
            motion = self.give_way.update(
                self.fresh_yield(now), self.fresh_pose('odom'), now,
                rear_clear=now-self.last_scan_at <= .6 and swept_footprint(self.scan_points, -self.give_way.speed_mps, 0., self.geometry))
            if motion is not None:
                self.line_state = motion.state
                self.send(motion.linear, 0.)
                return
        if not self.traffic_ready(context, now):
            self.line_state='traffic_wait'
            self.send()
            return
        self.dock_exiting = bool(context.get('dock_exit'))
        # 적재 자리 진입 중: 초음파에 보이는 것이 자리 자체라 붙잡지 않는다.
        self.dock_entering = bool(context.get('dock_entry'))
        if self.dock_entering and self.state == 'stopped':
            self.state = 'idle'
        if self.dock_exiting or self.dock_entering:
            self.mission_tick(now)
            return
        # 취소·만료가 항상 이긴다(위에서 처리). 진행 중인 회피는 Nav2가 잠시 0을 내도 계속 맡는다.
        if self.state != 'idle':
            self.avoidance_tick(now, context)
            return
        if self.mission_tick(now):
            return
        if now-self.last_command_at > .5 or (abs(self.nav_command.linear.x) <= .001 and abs(self.nav_command.angular.z) <= .001):
            self.start_last_tick = None
            self.line_state = 'idle'
            if now-self.last_motion_at > 1.:
                self.line_assist.reset()
                self.line_estimate = None
            self.send()
            return
        self.last_motion_at = now
        command=self.forward_nav_command()
        self.send(command.linear.x,command.angular.z)

    def on_dock_guide(self,msg):
        """관제가 보낸 {"forward_mm","left_mm","dyaw_deg"}를 도착 시각과 함께 (m, m, rad)로 보관한다."""
        try:
            data=json.loads(msg.data)
            guide=(float(data['forward_mm'])/1000.,float(data['left_mm'])/1000.,math.radians(float(data['dyaw_deg'])))
        except (ValueError,TypeError,KeyError):return
        # 카메라 범위(앞뒤 0.60 m, 좌우 0.30 m) 밖이거나 숫자가 아니면 버린다.
        if all(math.isfinite(v) for v in guide) and abs(guide[0])<=.60 and abs(guide[1])<=.30 and abs(guide[2])<=math.pi+.01:
            self.dock_guide=(guide,time.monotonic())

    def fresh_dock_guide(self,now,max_age_s=.4):
        guide=getattr(self,'dock_guide',None)
        return guide[0] if guide is not None and 0<=now-guide[1]<=max_age_s else None

    def on_lease(self,msg):
        try:self.lease_receiver.receive(json.loads(msg.data),time.monotonic())
        except (ValueError,TypeError):pass

    def on_yield(self,msg):
        """양보 지시 {"id","to","back_m"}를 도착 시각과 함께 보관한다."""
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        if isinstance(data,dict) and isinstance(data.get('id'),str) and isinstance(data.get('back_m'),(int,float)) and not isinstance(data.get('back_m'),bool):
            self.yield_order=(dict(id=data['id'],to=data.get('to'),back_m=float(data['back_m'])),time.monotonic())

    # 1 s 안에 다시 와야 유효하다. 관제가 멈추면 양보도 풀린다.
    def fresh_yield(self,now,max_age_s=1.0):
        order=getattr(self,'yield_order',None)
        return order[0] if order is not None and 0<=now-order[1]<=max_age_s else None

    def traffic_ready(self,context,now):
        """필요한 공유 구역을 lease가 이 경로(token)에 허가하는지 본다."""
        pose=self.fresh_pose('map')
        if pose is None:return False
        self.traffic_request=dict(token=context['token'],zones=required_zones(context,pose,self.runtime['traffic_zones']))
        return lease_allows(self.lease_receiver.lease,context['token'],self.traffic_request['zones'],now)

    def avoidance_tick(self, now, context):
        """초음파가 막혔을 때의 동작. ultrasonic_detour가 꺼져 있으면 전진만 막고 Nav2 회피에 맡긴다(필요하면 BackOff),
        켜져 있으면 UltrasonicDetour가 직접 비켜 통과한다."""
        self.mission_token = context.get('token')
        if not (getattr(self, 'runtime', {}).get('navigation') or {}).get('ultrasonic_detour', True):
            # Nav2가 라이다+초음파 costmap으로 피한다. 전진만 막고 회전·후진은 통과시킨다.
            command = self.forward_nav_command()
            linear = command.linear.x
            # 너무 가까우면 조금 후진한다(라이다로 뒤가 비어야 함).
            scan_fresh = now-self.last_scan_at <= .6
            motion = self.back_off.update(
                blocked=self.sensor_valid and not self.sensor_clear, range_m=self.range_m if self.sensor_valid else None,
                odom=self.fresh_pose('odom'), now=now,
                rear_clear=scan_fresh and swept_footprint(self.scan_points, -self.back_off.speed_mps, 0., self.geometry))
            if motion is not None:
                self.line_state = motion.state
                self.send(motion.linear, 0.)
                return
            if not self.sensor_clear and linear > 0.0:
                linear = 0.0
                self.line_state = 'ultrasonic_hold_forward'
            if self.sensor_clear:
                self.state = 'idle'
            self.send(linear, command.angular.z)
            return
        odom = self.fresh_pose('odom')
        if odom is None or now-self.last_scan_at > .6:
            self.line_state = 'ultrasonic_geometry_unavailable'
            self.send()
            return
        if self.detour.phase == 'idle':
            self.detour.start(odom, self.range_m or .10, self.scan_points, now, require_line=bool(context.get('use_ir')))
            self.line_assist.reset()
        strengths=normalize(self.ir_raw,self.ir_profile) if now-self.last_ir_at<.6 else None
        center=bool(strengths and strengths[1]>=.55)
        motion = self.detour.update(odom,self.range_m,self.scan_points,now,
                    lambda p,v,w:swept_footprint(p,v,w,self.geometry),line_center=center,line_sample_at=self.last_ir_at)
        if self.detour.phase=='line_confirm' and self.detour.require_line:
            decision=self.line_assist.decide(.012,0.,self.ir_raw,self.ir_profile,now,
                        now-self.last_ir_at<.6,True,self.scan_points,heading_yaw=odom[2])
            motion.linear,motion.angular=decision.linear,decision.angular
            if decision.state=='line_search_failed':
                self.detour.phase,self.detour.failure='fault','line_rejoin_failed'
                motion.linear,motion.angular=0.,0.
                motion.state='ultrasonic_fault'

        self.state, self.line_state = self.detour.phase, motion.state
        self.send(motion.linear, motion.angular)
        if self.state == 'idle':
            # 멈춘 교차로 통과에서 회피 이동·회전·시간을 뺀다.
            if self.crossing.state not in {'idle', 'fault'}:
                elapsed = now-self.detour.started
                self.crossing.entered += elapsed
                self.crossing.phase_started += elapsed
                self.crossing.stable_since = None
                self.crossing.previous_odom = odom
            self.line_assist.reset()
            self.detour = UltrasonicDetour(self.geometry)


def main() -> None:
    rclpy.init()
    node = UltrasonicSafety()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.send()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
