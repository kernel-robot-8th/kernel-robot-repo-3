#!/usr/bin/env python3
"""기준점(anchor) 사이를 Nav2로 주행하고, 사이 체크포인트는 멈추지 않고 통과 위치만 기록한다.

실행 위치: 로봇 (app.py가 SSH로 실행, argv[1] = 경로 점 JSON base64). MISSION_STATUS·CHECKPOINT_POSE·ROUTE_* 줄을 관제가 읽는다.
/cmd_vel은 직접 내지 않는다(발행자는 pinky_final_guard 하나). 행동 계층과는 ~/.cache/pinky-fleet의 JSON 파일
(route_context.json ↔ route_feedback.json, 2초 넘게 낡으면 끊김)로 주고받는다. 종료 코드: 0 완료, 130 취소, 그 밖은 RouteFailure.code.
"""
from __future__ import annotations
import base64
import json
import math
from pathlib import Path
import signal
import sys
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

CACHE = Path('/home/pinky/.cache/pinky-fleet')


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


# 임시 파일 + rename: 반쯤 쓴 JSON을 읽지 않게.
def atomic_json(name, data):
    path = CACHE / name
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, allow_nan=False))
    temporary.replace(path)


class RouteFailure(Exception):
    """경로 실패. code는 종료 코드(130 = 취소)."""
    def __init__(self, code, message):
        self.code, self.message = code, message


class RouteRunner(Node):
    """Nav2 navigate_to_pose 액션 클라이언트. 5번 도킹 후 /initialpose로 AMCL을 다시 잡는다."""
    def __init__(self):
        super().__init__('pinky_dashboard_route')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.amcl_sample = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, qos_profile_sensor_data)
        self.interrupted = False
        self.run_id = None
        self.active_goal = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_amcl(self, msg):
        self.amcl_sample = (time.monotonic(), msg)

    def anchor_dock(self, context, target, token, dock_id=None):
        """AMCL 위치만 target으로 재설정한다(오도메트리는 유지). 3 cm·5도 안에 18초 내 수렴해야 성공."""
        self.cancel_active()
        last_pose = self.pose(.8)
        started, published = time.monotonic(), None
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.pose.pose.position.x, message.pose.pose.position.y = target[:2]
        message.pose.pose.orientation.z = math.sin(target[2]/2)
        message.pose.pose.orientation.w = math.cos(target[2]/2)
        # 표준편차 1 cm, yaw 5도: 도킹 자리는 정확하므로 AMCL이 강하게 믿게 한다.
        message.pose.covariance[0] = message.pose.covariance[7] = .0001
        message.pose.covariance[35] = math.radians(5)**2
        print('MISSION_STATUS '+json.dumps(dict(phase='docking_anchor', message='도킹완료 확인 · 5번 지도 위치 재설정 중'), ensure_ascii=False), flush=True)
        while rclpy.ok() and not self.interrupted and time.monotonic()-started < 18.:
            context(last_pose, mission_wait=True)
            now = time.monotonic()
            if self.initial_pose.get_subscription_count() and published is None:
                message.header.stamp = self.get_clock().now().to_msg()
                self.initial_pose.publish(message)
                published = now
            rclpy.spin_once(self, timeout_sec=.05)
            try:
                last_pose = self.pose(.1)
            except RouteFailure as exc:
                if exc.code != 9: raise
                continue
            sample = self.amcl_sample
            if published and sample and sample[0] >= published:
                pose = sample[1].pose.pose
                q = pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                if (sample[1].header.frame_id == 'map'
                        and math.hypot(pose.position.x-target[0],pose.position.y-target[1]) <= .03
                        and abs(angle(yaw-target[2])) <= math.radians(5)
                        and math.hypot(last_pose[0]-target[0],last_pose[1]-target[1]) <= .03
                        and abs(angle(last_pose[2]-target[2])) <= math.radians(5)):
                    atomic_json('dock_anchor.json', dict(token=token, dock_id=dock_id,
                        monotonic=time.monotonic(),success=True,target=list(target)))
                    return
        if self.interrupted: raise RouteFailure(130, 'cancelled')
        raise RouteFailure(18, '5번 위치 재설정 확인 실패: AMCL /initialpose 구독과 최신 AMCL·TF 좌표를 확인하세요. 정지를 유지합니다.')

    def cancel(self, *_args):
        self.interrupted = True

    def pose(self, timeout=1.):
        """최신 map→base_footprint TF (x, y, yaw). 이 run_id의 정지 래치면 취소(130), 시간 초과면 실패(9)."""
        deadline = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic() < deadline and not self.interrupted:
            try:
                stop=json.loads((CACHE/'stop_latch.json').read_text())
                if self.run_id and stop.get('run_id')==self.run_id:self.interrupted=True
            except (OSError,ValueError,TypeError):pass
            if self.interrupted:raise RouteFailure(130,'cancelled')
            rclpy.spin_once(self, timeout_sec=.025)
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
                age = (self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
                if -2 <= age <= 1.:
                    t, q = tf.transform.translation, tf.transform.rotation
                    return t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            except TransformException:
                pass
        if self.interrupted:
            raise RouteFailure(130, 'cancelled')
        raise RouteFailure(9, '최신 map 위치를 확인할 수 없습니다.')

    def cancel_active(self):
        if self.active_goal is not None:
            future = self.active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.)
            self.active_goal = None


def checkpoint(index, point, pose, passing=False, sampling=None):
    """점 도착/통과 보고. 멈추는 점은 오차 5 cm 초과면 실패(10), IR 도킹으로 맞춘 점은 검사하지 않는다."""
    x, y, yaw = pose
    error = math.hypot(x-point['x_m'], y-point['y_m'])
    print('CHECKPOINT_POSE '+json.dumps(dict(label=point['label'], x_m=x, y_m=y, yaw_rad=yaw,
        target_x_m=point['x_m'], target_y_m=point['y_m'], error_m=error,
        yaw_error_rad=angle(yaw-point['yaw_rad']), pass_through=passing,
        sampling=sampling or ('tf_crossing_interpolation' if passing else 'latest_tf'),
        within_tolerance=error <= .05), ensure_ascii=False), flush=True)
    if not passing and error > .05 and sampling != 'docking_service':
        raise RouteFailure(10, f"{point['label']}: 위치 오차 {error*100:.1f}cm가 도착 검증 한도 5cm를 넘었습니다.")
    print(f"WAYPOINT_DONE {index+1} {point['label']}", flush=True)


def mission_feedback(token):
    """이 회차의 행동 계층 피드백(2초 이내, 아니면 None). fault면 RouteFailure를 던진다."""
    try:
        feedback = json.loads((CACHE/'route_feedback.json').read_text())
        if feedback.get('token') != token or not 0 <= time.monotonic()-feedback['monotonic'] < 2.:
            return None
        if feedback.get('crossing', {}).get('state') == 'fault':
            crossing = feedback['crossing']
            raise RouteFailure(14, '교차로 통과 제한: '+crossing.get('failure_reason', 'unknown')+
                               f" (단계 {crossing.get('phase')}, {crossing.get('phase_elapsed_s', 0):.1f}초). 정지 후 위치를 확인하세요.")
        if feedback.get('state')=='line_search_failed':
            raise RouteFailure(16,'IR 차선 재탐색 한도(6초/약23도)에 도달했습니다. 정지 후 선 위치와 보정값을 확인하세요.')
        if feedback.get('avoidance', {}).get('phase') == 'fault':
            raise RouteFailure(17, '초음파 우회 공간/진행 한도 초과 · 정지 후 장애물과 주변 공간을 확인하세요.')
        return feedback
    except (OSError, ValueError, TypeError, KeyError):
        return None


def behaviour_feedback():
    """회차와 무관한 최신 피드백(2초 이내): 상태와 천장 카메라 안내 dock_guide."""
    try:
        feedback = json.loads((CACHE/'route_feedback.json').read_text())
        return feedback if 0 <= time.monotonic()-feedback['monotonic'] < 2. else {}
    except (OSError, ValueError, TypeError, KeyError):
        return {}


def capturable(guide):
    """카메라에 넘겨도 되는 위치: 자리 6~45 cm 뒤, 옆 오차 3 cm + 거리의 40%(최대 20 cm) 이내."""
    try:
        forward, left = float(guide[0]), float(guide[1])
    except (TypeError, ValueError, IndexError):
        return False
    return -.45 <= forward <= -.06 and abs(left) <= min(.20, .03+.4*abs(forward))


class Sightings:
    """카메라 프레임 수를 센다(같은 안내 재읽기는 한 번). capturable 아닌 안내가 오면 다시 센다."""

    def __init__(self):
        self.count, self.last = 0, None

    def see(self, guide):
        if not capturable(guide):
            self.count, self.last = 0, None
        elif list(guide) != self.last:
            self.count, self.last = self.count+1, list(guide)
        return self.count


NAV_RETRIES = 20  # 구간마다 Nav2 목표 재전송 횟수(교통 대기 제외)
TRAFFIC_WAIT_LIMIT_S = 900.  # 15분 넘는 대기는 사람이 봐야 하는 정체
NAV_RETRY_PAUSE_S = 3.
# 다른 핑키를 위해 서 있는 상태(scripts/yielding.py). 실패가 아니다.
WAIT_STATES = frozenset({'traffic_wait', 'yield_back', 'yield_wait'})


def detour_wait(node,context,token):
    # 우회 중에는 Nav2 목표를 취소한다(progress checker가 중단시키지 않게). idle이면 끝, 최대 90초.
    if node.active_goal is not None:
        node.active_goal.cancel_goal_async()
        node.active_goal=None
    print('MISSION_STATUS '+json.dumps(dict(phase='recovering',token=token,message='초음파 우회·차선 복귀 중 · Nav2 목표 일시 중단'),ensure_ascii=False),flush=True)
    started=time.monotonic()
    while rclpy.ok() and not node.interrupted:
        context(node.pose(.8))
        feedback=mission_feedback(token)
        if feedback is None:
            if time.monotonic()-started>2.:raise RouteFailure(17,'우회 감시 상태 미수신 · 정지')
        elif feedback.get('avoidance',{}).get('phase')=='idle':
            print('MISSION_STATUS '+json.dumps(dict(phase='driving',token=token,message='차선 복귀 확인 · Nav2 목표 재개'),ensure_ascii=False),flush=True)
            return
        if time.monotonic()-started>90.:raise RouteFailure(17,'우회·차선 복귀 대기 한도 초과')
        rclpy.spin_once(node,timeout_sec=.05)
    raise RouteFailure(130,'cancelled')


def docking_wait(node, context, token, require_marker=False):
    """IR 도킹 동안 기다린다. anchor_wait이면 AMCL 재설정, released면 끝. require_marker: IR 마커를 최대 20초 찾는다."""
    active = {'docking','reverse','await_confirmation','stopped','fault','recovery_hold','search','search_hold','anchor_wait'}
    node.cancel_active()
    started, previous = time.monotonic(), None
    anchored_id = None
    anchored_at = None
    while rclpy.ok() and not node.interrupted:
        context(node.pose(.8), dock_search=require_marker)
        feedback = mission_feedback(token)
        if feedback is None:
            if time.monotonic()-started > 2.:
                raise RouteFailure(15, '도킹 제어 상태가 끊겼습니다. 정지를 유지합니다.')
            continue
        dock = feedback.get('docking', {})
        state = dock.get('state')
        if state == 'anchor_wait' and anchored_id != dock.get('dock_id'):
            try:
                route = json.loads((CACHE/'route_context.json').read_text())
                target = (*route['dock_xy'], route['heading_rad'])
            except (OSError, ValueError, TypeError, KeyError):
                raise RouteFailure(18, '5번 위치 재설정 기준이 없습니다.')
            node.anchor_dock(context, target, token, dock.get('dock_id'))
            anchored_id = dock.get('dock_id')
            anchored_at = time.monotonic()
            context(node.pose(.8), dock_search=require_marker)
        if state == 'anchor_wait' and anchored_at and time.monotonic()-anchored_at > 4.:
            raise RouteFailure(18, '5번 위치 재설정 이후 도킹 정지 상태를 확인하지 못했습니다. 센서를 확인하세요.')
        if state == 'released':
            print('MISSION_STATUS '+json.dumps(dict(phase='driving', message='물건 수령 완료 · 경로 재개'), ensure_ascii=False), flush=True)
            return
        if state == 'fault':
            raise RouteFailure(15, '도킹 제어 오류로 정지했습니다.')
        if state not in active and (not require_marker or time.monotonic()-started > 20.):
            raise RouteFailure(15, '5번 도착 후 세 센서의 흰색 패턴을 확인하지 못했습니다. 위치를 확인하세요.')
        message = dock.get('message') if state in active else '5번 도착 · 세 센서 흰색 패턴 대기'
        if (state, message) != previous:
            print('MISSION_STATUS '+json.dumps(dict(phase='docking', message=message), ensure_ascii=False), flush=True)
            previous = state, message
        rclpy.spin_once(node, timeout_sec=.075)
    raise RouteFailure(130, 'cancelled')


REDOCK_BACK_M = .35  # 다시 들어오기 전 후진 거리(m)
REDOCK_LIMIT = 30  # 진입 반복 시험용으로 넉넉히
REDOCK_CAPTURE_BACK_M = .25  # 카메라 안내 시: 마커가 보이는 거리까지만


def maneuver(node, context, token, key, kind, extra, limit_s):
    """행동 계층 기동 하나(dock_exit, dock_entry)를 실행하고 complete를 기다린다. fault·시간 초과는 실패(19)."""
    started = time.monotonic()
    while rclpy.ok() and not node.interrupted:
        context(node.pose(.8), extra={**extra, 'segment': key})
        status = (mission_feedback(token) or {}).get(kind, {})
        if status.get('key') == [token, key]:
            if status.get('state') == 'fault':
                raise RouteFailure(19, status.get('message', '5번 다시 맞추기 실패'))
            if status.get('state') == 'complete':
                return
        if time.monotonic()-started > limit_s:
            raise RouteFailure(19, '5번 다시 맞추기 제한 시간 초과 · 주변을 확인하세요.')
        rclpy.spin_once(node, timeout_sec=.05)
    raise RouteFailure(130, 'cancelled')


def nav_goal(node, context, extra, x, y, yaw, limit_s=90.):
    """방향을 정한 Nav2 목표 하나(재도킹 정면 맞추기용)."""
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    context(node.pose(.8), extra=extra)
    future = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
    if not future.done() or future.exception() is not None:
        raise RouteFailure(4, 'Nav2 목표 전달 응답을 받지 못했습니다.')
    node.active_goal = future.result()
    if not node.active_goal.accepted:
        node.active_goal = None
        raise RouteFailure(5, '5번 정면 다시 맞추기 목표가 거부됐습니다.')
    result = node.active_goal.get_result_async()
    deadline = time.monotonic()+limit_s
    while rclpy.ok() and not node.interrupted and not result.done():
        if time.monotonic() > deadline:
            raise RouteFailure(6, '5번 정면 다시 맞추기 시간 초과')
        context(node.pose(.8), extra=extra)
        rclpy.spin_once(node, timeout_sec=.075)
    if node.interrupted: raise RouteFailure(130, 'cancelled')
    response = result.result()
    node.active_goal = None
    if response.status != GoalStatus.STATUS_SUCCEEDED:
        raise RouteFailure(7, f'5번 정면 다시 맞추기 실패 (status {response.status})')


def guided_dock(correction):
    """관제가 카메라 오프셋으로 계산한 정면 맞추기 점과 적재 자리. 없거나 잘못되면 None."""
    try:
        points = [dict(x_m=float(correction[name]['x_m']), y_m=float(correction[name]['y_m']),
                       yaw_rad=float(correction[name]['yaw_rad'])) for name in ('square', 'dock')]
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for point in points for v in point.values()):
        return None
    return points


def redock(node, context, token, segment, square, dock, attempt, correction=None):
    """카메라가 어긋남을 보고 다시 들이기(2026-11-17): 후진 → 정면 맞추기 → 직진 진입. 오프셋이 있으면 보정된 점을 쓴다."""
    guided = guided_dock(correction)
    how = ''
    if guided:
        how = f" · 카메라 보정 {math.hypot(guided[1]['x_m']-dock['x_m'], guided[1]['y_m']-dock['y_m'])*100:.1f}cm"
        square, dock = guided
    heading = dock.get('yaw_rad', 0.)
    print('MISSION_STATUS '+json.dumps(dict(phase='redocking', token=token,
          message=f'5번 위치가 어긋나 다시 맞추는 중 ({attempt}/{REDOCK_LIMIT}){how} · {REDOCK_BACK_M*100:.0f}cm 후진'), ensure_ascii=False), flush=True)
    node.cancel_active()
    # 카메라가 보이면 짧게 후진하고 카메라가 다시 들인다(Nav2 정면 맞추기는 몇 분씩 헤맴, 2026-11-19).
    # 프레임 하나 빠진 것으로 먼 길로 돌지 않게 최대 1.5초 찾는다(2026-11-21).
    layer, until = behaviour_feedback(), time.monotonic()+1.5
    while rclpy.ok() and not node.interrupted and layer.get('dock_guide') is None and time.monotonic() < until:
        context(node.pose(.8), mission_wait=True)
        rclpy.spin_once(node, timeout_sec=.075)
        layer = behaviour_feedback()
    camera_led = layer.get('dock_guide') is not None and 'capture' in (layer.get('dock_entry') or {})
    back_m = REDOCK_CAPTURE_BACK_M if camera_led else REDOCK_BACK_M
    if camera_led:
        print('MISSION_STATUS '+json.dumps(dict(phase='redocking', token=token,
              message=f'5번 다시 들어오는 중 ({attempt}/{REDOCK_LIMIT}) · {back_m*100:.0f}cm 후진'), ensure_ascii=False), flush=True)
    maneuver(node, context, token, f'{segment}r{attempt}b', 'dock_exit',
             dict(dock_exit=True, dock_exit_m=back_m, dock_entry=False), 40.)
    if camera_led:
        # 3초 안에 프레임 4개 연속 capturable이면 카메라 진입, 아니면 Nav2.
        seen, until = Sightings(), time.monotonic()+3.
        while rclpy.ok() and not node.interrupted and time.monotonic() < until and seen.count < 4:
            context(node.pose(.8), mission_wait=True)
            seen.see(behaviour_feedback().get('dock_guide'))
            rclpy.spin_once(node, timeout_sec=.075)
        if seen.count >= 4:
            print('MISSION_STATUS '+json.dumps(dict(phase='redocking', token=token,
                  message=f'5번 다시 들어오는 중 ({attempt}/{REDOCK_LIMIT}) · 천장 캠과 주고받으며 들어감'), ensure_ascii=False), flush=True)
            maneuver(node, context, token, f'{segment}r{attempt}e', 'dock_entry',
                     dict(dock_entry=True, dock_exit=False, dock_capture=True, dock_turn_rad=0., dock_side_m=0.), 90.)
            return
    print('MISSION_STATUS '+json.dumps(dict(phase='redocking', token=token,
          message=f'5번 다시 맞추는 중 ({attempt}/{REDOCK_LIMIT}) · 정면 진입점에서 방향 맞추기'), ensure_ascii=False), flush=True)
    nav_goal(node, context, dict(segment=f'{segment}r{attempt}n', dock_exit=False, dock_entry=False),
             square['x_m'], square['y_m'], heading)
    print('MISSION_STATUS '+json.dumps(dict(phase='redocking', token=token,
          message=f'5번 다시 맞추는 중 ({attempt}/{REDOCK_LIMIT}) · 정면 직진'), ensure_ascii=False), flush=True)
    node.cancel_active()
    maneuver(node, context, token, f'{segment}r{attempt}e', 'dock_entry',
             dict(dock_entry=True, dock_exit=False, dock_capture=False, dock_turn_rad=0., dock_side_m=0.,
                  dock_entry_m=math.hypot(dock['x_m']-square['x_m'], dock['y_m']-square['y_m'])), 40.)


def operator_wait(node,context,token,phase,action,message):
    """관제가 이 관문에 action을 보낼 때까지 정지해 기다린다(같은 gate_id, 대기 시작 뒤·10초 이내 신호만)."""
    actions=(action,) if isinstance(action,str) else tuple(action)
    node.cancel_active()
    gate_id=uuid.uuid4().hex
    # 대기마다 고유 토큰: 늦거나 중복된 클릭이 다음 관문을 풀지 못하게.
    print('MISSION_STATUS '+json.dumps(dict(phase=phase,token=gate_id,message=message),ensure_ascii=False),flush=True)
    started=time.monotonic()
    while rclpy.ok() and not node.interrupted:
        context(node.pose(.8),mission_wait=True)
        try:
            command=json.loads((CACHE/'operator_confirm.json').read_text())
            if command.get('token')==gate_id and command.get('action') in actions and float(command.get('monotonic',0))>=started and 0<=time.monotonic()-float(command['monotonic'])<=10.:
                print('MISSION_STATUS '+json.dumps(dict(phase='driving',token=token,message='확인 완료 · 다음 단계 진행'),ensure_ascii=False),flush=True)
                node.confirmed=command
                return command['action']
        except (OSError,ValueError,TypeError,KeyError):pass
        rclpy.spin_once(node,timeout_sec=.075)
    raise RouteFailure(130,'cancelled')


def run(node, points):
    """경로 전체 실행. anchor 사이가 한 구간이고 구간마다 Nav2 목표 하나. 특수: dock_exit, '5 정면'→'5', course_end."""
    if not node.client.wait_for_server(timeout_sec=15.):
        raise RouteFailure(3, 'Nav2 navigate_to_pose 액션 서버를 찾을 수 없습니다.')
    # /cmd_vel 발행자는 pinky_final_guard 하나여야 한다(다른 발행자가 있으면 거부). 바쁜 Pi에서 발견이
    # 1초 넘게 걸려 임무가 거부된 적이 있어(2026-11-16) 최대 8초 기다린다.
    deadline = time.monotonic()+8.
    publishers = []
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        publishers = node.get_publishers_info_by_topic('/cmd_vel')
        if any(p.node_name == 'pinky_final_guard' for p in publishers):
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.05)
            publishers = node.get_publishers_info_by_topic('/cmd_vel')
            break
    competitors = sorted({p.node_name for p in publishers if p.node_name != 'pinky_final_guard'})
    if competitors:
        raise RouteFailure(11, '수동/다른 cmd_vel 발행자를 종료한 뒤 시작하세요: '+', '.join(competitors))
    if not any(p.node_name == 'pinky_final_guard' for p in publishers):
        raise RouteFailure(11, '통합 안전 감시기가 없습니다. 주행준비를 다시 실행하세요.')
    anchors = [i for i, p in enumerate(points) if p.get('anchor')]
    if not anchors or anchors[0] != 0 or anchors[-1] != len(points)-1:
        raise RouteFailure(2, '시작·끝 기준점이 포함된 경로가 필요합니다.')
    initial = node.pose(3.)
    print(f"WAYPOINT_START 1 {points[0]['label']}", flush=True)
    if math.hypot(initial[0]-points[0]['x_m'], initial[1]-points[0]['y_m']) > .05:
        raise RouteFailure(10, '기체를 자기 주차장에 놓고 AMCL 주차장 위치를 설정한 뒤 시작하세요.')
    if abs(angle(initial[2]-points[0].get('parking_yaw_rad',initial[2]))) > math.radians(15):
        raise RouteFailure(10,'주차장 출발 방향 오차가 15도를 넘습니다. 실제 방향과 AMCL 추정을 맞춰주세요.')
    checkpoint(0, points[0], initial)
    token, startup_armed = uuid.uuid4().hex, False
    # token: 이번 실행의 회차 ID.
    mission_phase = 'driving'
    # DockEntry에 넘길 남은 회전(rad)·직진 거리(m)·옆 오차(m)
    dock_turn, dock_entry_distance, dock_side = 0., None, 0.
    dock_captured = False
    visits = [points[i]['label'] for i in anchors]
    # 두 번 이상 지나는 점(출발점 제외) = 교차로.
    junctions = {label for label in visits if label != visits[0] and visits.count(label) > 1}
    for segment, (start_index, end_index) in enumerate(zip(anchors, anchors[1:])):
        start, end = points[start_index], points[end_index]
        dx, dy = end['x_m']-start['x_m'], end['y_m']-start['y_m']
        length = math.hypot(dx, dy)
        if length < 1e-6:
            checkpoint(end_index, end, node.pose())
            continue
        heading = math.atan2(dy, dx)
        start_turn = False
        if segment:
            previous = points[anchors[segment-1]]
            incoming = math.atan2(start['y_m']-previous['y_m'], start['x_m']-previous['x_m'])
            incoming = start.get('arrival_yaw_rad', incoming)
            start_turn = abs(angle(heading-incoming)) > .35
        course_departure = bool(segment and previous['label'] == '주차장' and start['label'] != '주차장')
        end_turn = False
        if segment+2 < len(anchors):
            following = points[anchors[segment+2]]
            outgoing = math.atan2(following['y_m']-end['y_m'], following['x_m']-end['x_m'])
            end_turn = abs(angle(outgoing-heading)) > .35
        if end['label'] == '5':
            end_turn = False
        cursor = start_index+1
        print(f"WAYPOINT_START {cursor+1} {points[cursor]['label']}", flush=True)
        dock_turn, dock_entry_distance, dock_side = 0., None, 0.
        if start['label'] == '5 정면' and end['label'] == '5':
            # 정면 맞추기 점 근처(가깝거나 카메라가 잡음): Nav2를 멈추고 남은 회전·진입(5~40 cm)을 DockEntry에 넘긴다.
            node.cancel_active()
            settle = time.monotonic()+.8
            while rclpy.ok() and not node.interrupted and time.monotonic() < settle:
                rclpy.spin_once(node, timeout_sec=.05)
            stood, dock_yaw = node.pose(.8), end.get('yaw_rad', heading)
            dock_turn = angle(dock_yaw-stood[2])
            dock_entry_distance = max(.05, min(.40, (end['x_m']-stood[0])*math.cos(dock_yaw)+(end['y_m']-stood[1])*math.sin(dock_yaw)))
            # 도킹선 옆 오차는 DockEntry가 바로잡는다(±8 cm로 자름).
            dock_side = max(-.08, min(.08, -(stood[0]-end['x_m'])*math.sin(dock_yaw)+(stood[1]-end['y_m'])*math.cos(dock_yaw)))
        # route_context.json을 쓴다. 루프마다 불러 heartbeat를 겸한다(1.5초 넘게 낡으면 감시기가 정지).
        def context(pose, dock_search=False, mission_wait=False, extra=None):
            near_start = math.hypot(pose[0]-start['x_m'], pose[1]-start['y_m']) < .12
            near_end = math.hypot(pose[0]-end['x_m'], pose[1]-end['y_m']) < .12
            atomic_json('route_context.json', {**dict(token=token, segment=segment,
                monotonic=time.monotonic(), dock_exit=bool(end.get('dock_exit')), dock_search=dock_search, mission_wait=mission_wait, use_ir=bool(end.get("use_ir")), transit=start["label"]=="주차장" or end["label"]=="주차장", corner=(start_turn and near_start) or (end_turn and near_end),
                start_transition=not course_departure and (start_turn or start['label'] in junctions),
                end_transition=end_turn or end['label'] in junctions,
                start_xy=[start['x_m'],start['y_m']], end_xy=[end['x_m'],end['y_m']],
                start_label=start['label'], end_label=end['label'],
                dock_xy=[end['x_m'],end['y_m']] if end['label'] == '5' else None,
                # 5번 마지막 직선: 낮은 적재대가 바로 앞이라 초음파는 끄고 라이다만 본다.
                dock_entry=start['label'] == '5 정면' and end['label'] == '5',
                dock_entry_m=dock_entry_distance if dock_entry_distance is not None else math.hypot(end['x_m']-start['x_m'], end['y_m']-start['y_m']),
                dock_turn_rad=dock_turn, dock_side_m=dock_side,
                dock_capture=dock_captured and start['label'] == '5 정면' and end['label'] == '5',
                outgoing_heading_rad=outgoing if end_turn else heading,
                label=f"{start['label']}→{end['label']}", heading_rad=heading), **(extra or {})})
        pose = node.pose()
        context(pose)
        if not startup_armed and start["label"]!="주차장" and end.get("use_ir"):
            atomic_json('route_start.json', dict(token=token, monotonic=time.monotonic()))
            startup_armed = True
        if end.get('dock_exit'):
            node.cancel_active()
            print('MISSION_STATUS '+json.dumps(dict(phase='dock_exit', message='도킹 탈출 · 20cm 후진 후 6번으로 이동'), ensure_ascii=False), flush=True)
            started = time.monotonic()
            while rclpy.ok() and not node.interrupted:
                pose = node.pose(.8)
                context(pose)
                feedback = mission_feedback(token)
                exit_status = (feedback or {}).get('dock_exit', {})
                if exit_status.get('key') != [token, segment]:
                    exit_status = {}
                if exit_status.get('state') == 'fault':
                    raise RouteFailure(19, exit_status.get('message', '도킹 후진 실패'))
                if exit_status.get('state') == 'complete':
                    checkpoint(end_index, end, pose)
                    break
                if time.monotonic()-started > 32.:
                    raise RouteFailure(19, '20cm 도킹 후진 제한 시간 초과 · 후방 장애물/센서를 확인하세요.')
                rclpy.spin_once(node, timeout_sec=.05)
            if node.interrupted: raise RouteFailure(130, 'cancelled')
            continue
        if start['label'] == '5 정면' and end['label'] == '5':
            # 5번 직진 진입은 행동 계층 DockEntry가 한다(적재대가 Nav2에는 충돌로 보임).
            node.cancel_active()
            print('MISSION_STATUS '+json.dumps(dict(phase='dock_entry', message='5번 진입 · 천장 캠과 주고받으며 들어감' if dock_captured else '5번 진입 · 정면 직진'), ensure_ascii=False), flush=True)
            started = time.monotonic()
            entry_limit_s = 90. if dock_captured else 60.
            while rclpy.ok() and not node.interrupted:
                pose = node.pose(.8)
                context(pose)
                feedback = mission_feedback(token)
                entry_status = (feedback or {}).get('dock_entry', {})
                if entry_status.get('key') != [token, segment]:
                    entry_status = {}
                if entry_status.get('state') == 'fault':
                    raise RouteFailure(19, entry_status.get('message', '5번 진입 실패'))
                if entry_status.get('state') == 'complete':
                    break
                if time.monotonic()-started > entry_limit_s:
                    raise RouteFailure(19, '5번 진입 제한 시간 초과 · 옆 장애물/센서를 확인하세요.')
                rclpy.spin_once(node, timeout_sec=.05)
            if node.interrupted: raise RouteFailure(130, 'cancelled')
            dock_captured, dock_turn, dock_entry_distance, dock_side = False, 0., None, 0.
            pose = node.pose(2.)
            while cursor < end_index:
                checkpoint(cursor, points[cursor], pose, passing=True, sampling='end_pose_no_crossing_sample')
                cursor += 1
            checkpoint(end_index, end, pose)
            attempt = 0
            while True:
                note = f' (다시 맞춘 {attempt}번째)' if attempt else ''
                action = operator_wait(node, context, token, 'docking_manual', ('dock_complete', 'redock'),
                                       '5번 도착 · 정지 · 도킹완료 확인 대기'+note)
                if action == 'dock_complete':
                    node.anchor_dock(context, (end['x_m'],end['y_m'],end.get('yaw_rad',heading)), token)
                    # dock_complete 뒤에도 관제가 재도킹을 요청할 수 있다(2026-11-19).
                    action = operator_wait(node, context, token, 'docked_manual', ('received', 'redock'),
                                           '도킹완료 · 정지 유지 · 물건 수령 완료 대기')
                    if action == 'received':
                        break
                attempt += 1
                if attempt > REDOCK_LIMIT:
                    raise RouteFailure(19, f'5번 다시 맞추기를 {REDOCK_LIMIT}번 했는데도 또 요청받았습니다 · 현장을 확인하세요.')
                redock(node, context, token, segment, start, end, attempt,
                       (getattr(node, 'confirmed', None) or {}).get('dock_correction'))
            continue
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = end['x_m'], end['y_m']
        # 들어오는 구간 방향으로 도착(주차 복귀면 주차 방향).
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin((end["yaw_rad"] if end.get("parking_return") else end.get("arrival_yaw_rad",heading))/2), math.cos((end["yaw_rad"] if end.get("parking_return") else end.get("arrival_yaw_rad",heading))/2)
        future = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
        if not future.done() or future.exception() is not None:
            raise RouteFailure(4, 'Nav2 목표 전달 응답을 받지 못했습니다.')
        node.active_goal = future.result()
        if not node.active_goal.accepted:
            node.active_goal = None
            raise RouteFailure(5, f"{end['label']} 목표가 거부됐습니다.")
        # 정지 래치는 새 목표가 수락된 뒤에만 지운다. 이 run_id의 정지면 취소한다.
        try:
            stop=json.loads((CACHE/'stop_latch.json').read_text())
            if node.run_id and stop.get('run_id')==node.run_id:raise RouteFailure(130,'cancelled')
        except (OSError,ValueError,TypeError):pass
        (CACHE/'stop_latch.json').unlink(missing_ok=True)
        result = node.active_goal.get_result_async()
        deadline = time.monotonic()+240.  # 구간당 240초(교통 대기 중에는 연장)
        previous_pose = pose
        dock_serviced = False
        passed = False
        retries, waited_at, wait_began, seen = 0, None, None, Sightings()
        while rclpy.ok() and time.monotonic() < deadline and not node.interrupted:
            pose = node.pose(.8)
            context(pose)
            layer = behaviour_feedback()
            if layer.get('state') in WAIT_STATES:
                # 다른 핑키 대기는 실패가 아니다: 한도를 다시 시작하고 포기된 목표는 다시 보낸다
                # (2026-11-19: 대기 중 임무가 끝나버린 사고).
                if waited_at is None:
                    why = '다른 핑키에게 길을 비켜 주는 중' if str(layer.get('state')).startswith('yield') else '다른 핑키가 지나가길 기다리는 중'
                    print('MISSION_STATUS '+json.dumps(dict(phase=mission_phase, token=token, message=f"{end['label']}로 가는 길 · {why}"), ensure_ascii=False), flush=True)
                waited_at = time.monotonic()
                wait_began = wait_began or waited_at
                if waited_at-wait_began > TRAFFIC_WAIT_LIMIT_S:
                    raise RouteFailure(6, f"{end['label']}로 가는 길에서 다른 핑키를 {TRAFFIC_WAIT_LIMIT_S/60:.0f}분 넘게 기다렸습니다 · 길을 막은 핑키와 자리 예약을 확인하세요.")
                deadline = waited_at+240.
            elif waited_at is not None and time.monotonic()-waited_at > 3. and layer:
                wait_began = None
                print('MISSION_STATUS '+json.dumps(dict(phase=mission_phase, token=token, message=f"웨이포인트 {end['label']}로 이동 중"), ensure_ascii=False), flush=True)
                waited_at = None
            if end['label'] == '5 정면':
                # 카메라가 자리 앞 핑키를 본다(프레임 4개): 여기서부터 카메라가 들인다.
                if seen.see(layer.get('dock_guide')) >= 4:
                    dock_captured = passed = True
                    break
            # 통과 정류지는 pass_radius_m 안이면 도착(멈추지 않음). '5 정면'은 행동 계층이 squaring을 할 때만 통과.
            squares = end['label'] != '5 정면' or 'squaring' in (layer.get('dock_entry') or {})
            if end.get('pass_radius_m') and squares and math.hypot(pose[0]-end['x_m'], pose[1]-end['y_m']) <= end['pass_radius_m']:
                passed = True
                break
            feedback = mission_feedback(token)
            if feedback and feedback.get('avoidance',{}).get('phase') not in {None,'idle','fault'}:
                detour_wait(node,context,token)
                goal.pose.header.stamp=node.get_clock().now().to_msg()
                future=node.client.send_goal_async(goal)
                rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
                if not future.done() or future.exception() or not future.result().accepted:
                    raise RouteFailure(4,'차선 복귀 후 Nav2 목표 재개 실패')
                node.active_goal=future.result();result=node.active_goal.get_result_async()
                deadline=time.monotonic()+240.
                previous_pose=node.pose(.8)
                continue
            if result.done():
                response = result.result()
                waited = waited_at is not None and time.monotonic()-waited_at < 30.
                if response.status == GoalStatus.STATUS_SUCCEEDED or node.interrupted or not (waited or retries < NAV_RETRIES):
                    break
                # Nav2가 포기해도 임무는 계속하고 목표를 다시 보낸다. 대기 직후(30초) 실패는 횟수에 넣지 않는다.
                if not waited:
                    retries += 1
                    print('MISSION_STATUS '+json.dumps(dict(phase=mission_phase, token=token, message=f"웨이포인트 {end['label']} 구간 다시 시도 ({retries}/{NAV_RETRIES})"), ensure_ascii=False), flush=True)
                pause = time.monotonic()+(0. if waited else NAV_RETRY_PAUSE_S)
                while rclpy.ok() and not node.interrupted and (time.monotonic() < pause or behaviour_feedback().get('state') in WAIT_STATES):
                    context(node.pose(.8))
                    if behaviour_feedback().get('state') in WAIT_STATES:
                        waited_at = time.monotonic()
                        wait_began = wait_began or waited_at
                        if waited_at-wait_began > TRAFFIC_WAIT_LIMIT_S:
                            raise RouteFailure(6, f"{end['label']}로 가는 길에서 다른 핑키를 {TRAFFIC_WAIT_LIMIT_S/60:.0f}분 넘게 기다렸습니다 · 길을 막은 핑키와 자리 예약을 확인하세요.")
                    rclpy.spin_once(node, timeout_sec=.075)
                if node.interrupted:
                    break
                goal.pose.header.stamp = node.get_clock().now().to_msg()
                future = node.client.send_goal_async(goal)
                rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
                if not future.done() or future.exception() or not future.result().accepted:
                    raise RouteFailure(4, f"{end['label']} 구간 Nav2 목표 다시 보내기 실패")
                node.active_goal = future.result(); result = node.active_goal.get_result_async()
                deadline = time.monotonic()+240.
                previous_pose = node.pose(.8)
                continue
            if end.get('use_ir') and feedback and feedback.get('docking', {}).get('state') in {'docking','reverse','await_confirmation','stopped','fault','recovery_hold','search','search_hold','anchor_wait'}:
                docking_wait(node, context, token)
                dock_serviced = True
                break
            # 구간 직선 위 진행률(0~1). 체크포인트 진행률을 지나면 통과.
            progress = ((pose[0]-start['x_m'])*dx+(pose[1]-start['y_m'])*dy)/(length*length)
            old_progress = ((previous_pose[0]-start['x_m'])*dx+(previous_pose[1]-start['y_m'])*dy)/(length*length)
            while cursor < end_index:
                point = points[cursor]
                at = ((point['x_m']-start['x_m'])*dx+(point['y_m']-start['y_m'])*dy)/(length*length)
                if progress < at:
                    break
                t = max(0., min(1., (at-old_progress)/(progress-old_progress))) if progress > old_progress else 1.
                crossing = (previous_pose[0]+t*(pose[0]-previous_pose[0]),
                            previous_pose[1]+t*(pose[1]-previous_pose[1]),
                            previous_pose[2]+t*angle(pose[2]-previous_pose[2]))
                checkpoint(cursor, point, crossing, passing=True)
                cursor += 1
                print(f"WAYPOINT_START {cursor+1} {points[cursor]['label']}", flush=True)
            previous_pose = pose
            rclpy.spin_once(node, timeout_sec=.075)
        if node.interrupted:
            raise RouteFailure(130, 'cancelled')
        if not passed and not result.done() and not dock_serviced:
            raise RouteFailure(6, f"{end['label']} 구간 주행 대기 시간 초과")
        if not dock_serviced and not passed:
            response = result.result()
            node.active_goal = None
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                detail = getattr(response.result, 'error_msg', '')
                raise RouteFailure(7, f"{end['label']} 구간 주행 실패 (status {response.status}, {detail})")
            if end['label'] == '5' and end.get('use_ir'):
                docking_wait(node, context, token, require_marker=True)
                dock_serviced = True
        pose = node.pose(2.)
        # 마지막 체크포인트 전에 성공이 올 수 있다: 남은 점은 끝 위치로 기록.
        while cursor < end_index:
            checkpoint(cursor, points[cursor], pose, passing=True, sampling='end_pose_no_crossing_sample')
            cursor += 1
        checkpoint(end_index, end, pose, passing=passed, sampling='docking_service' if dock_serviced else None)
        if end['label']=='5' and not end.get('use_ir'):
            operator_wait(node,context,token,'docking_manual','dock_complete','5번 도착 · 정지 · IR 보정 전 수동 도킹완료 확인 대기')
            # 도킹 자리 자체의 방향으로 재설정한다.
            node.anchor_dock(context, (end['x_m'],end['y_m'],end.get('yaw_rad',heading)), token)
            operator_wait(node,context,token,'docked_manual','received','도킹완료 · 정지 유지 · 물건 수령 완료 대기')
        if end.get('course_end'):
            node.cancel_active()
            print('MISSION_STATUS '+json.dumps(dict(phase='course_complete',token=token,message='코스 완주 · 정지 · 주행완료 신호 대기'),ensure_ascii=False),flush=True)
            while rclpy.ok() and not node.interrupted:
                context(node.pose(.8), mission_wait=True)
                try:
                    signal_data=json.loads((CACHE/'mission_complete.json').read_text())
                    if signal_data.get('token')==token and 0<=time.monotonic()-float(signal_data.get('monotonic',0))<=10.:
                        break
                except (OSError,ValueError,TypeError): pass
                rclpy.spin_once(node,timeout_sec=.075)
            if node.interrupted: raise RouteFailure(130,'cancelled')
            print('MISSION_STATUS '+json.dumps(dict(phase='returning',token=token,message='주행완료 수신 · 자기 주차장 복귀중'),ensure_ascii=False),flush=True)
            mission_phase = 'returning'
    print('ROUTE_DONE', flush=True)


def main():
    """argv[1] 경로를 검사(점 1~500개, 유한 좌표)하고 실행한다."""
    try:
        points = json.loads(base64.b64decode(sys.argv[1], validate=True))
        if not isinstance(points, list) or not 1 <= len(points) <= 500:
            raise ValueError('empty/oversized route')
        for p in points:
            if not isinstance(p.get('label'), str) or any(not math.isfinite(float(p[k])) for k in ('x_m','y_m','yaw_rad')):
                raise ValueError('invalid waypoint')
            for k in ('x_m','y_m','yaw_rad'):
                p[k] = float(p[k])
    except (IndexError, KeyError, ValueError, TypeError) as exc:
        print(f'ROUTE_FAILED 경로 입력 오류: {exc}', flush=True)
        return 2
    CACHE.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = RouteRunner()
    node.run_id=points[0].get("run_id")
    signal.signal(signal.SIGINT, node.cancel)
    signal.signal(signal.SIGTERM, node.cancel)
    try:
        run(node, points)
        return 0
    except RouteFailure as exc:
        print('ROUTE_CANCELLED' if exc.code == 130 else 'ROUTE_FAILED '+exc.message, flush=True)
        return exc.code
    except Exception as exc:
        print(f'ROUTE_FAILED 경로 오류: {exc}', flush=True)
        return 8
    finally:
        try:
            node.cancel_active()
        finally:
            for name in ('route_start.json','route_context.json','route.pid','mission_complete.json','operator_confirm.json','dock_anchor.json'):
                (CACHE/name).unlink(missing_ok=True)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
