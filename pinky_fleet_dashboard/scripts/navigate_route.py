#!/usr/bin/env python3
"""Navigate between anchors; observe dense checkpoints without stopping."""
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
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

CACHE = Path('/home/pinky/.cache/pinky-fleet')


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def atomic_json(name, data):
    path = CACHE / name
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, allow_nan=False))
    temporary.replace(path)


class RouteFailure(Exception):
    def __init__(self, code, message):
        self.code, self.message = code, message


class RouteRunner(Node):
    def __init__(self):
        super().__init__('pinky_dashboard_route')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.interrupted = False
        self.run_id = None
        self.active_goal = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def cancel(self, *_args):
        self.interrupted = True

    def pose(self, timeout=1.):
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
        return feedback
    except (OSError, ValueError, TypeError, KeyError):
        return None


def docking_wait(node, context, token, require_marker=False):
    active = {'docking','reverse','await_confirmation','stopped','fault','recovery_hold','search','search_hold'}
    node.cancel_active()
    started, previous = time.monotonic(), None
    while rclpy.ok() and not node.interrupted:
        context(node.pose(.8), dock_search=require_marker)
        feedback = mission_feedback(token)
        if feedback is None:
            if time.monotonic()-started > 2.:
                raise RouteFailure(15, '도킹 제어 상태가 끊겼습니다. 정지를 유지합니다.')
            continue
        dock = feedback.get('docking', {})
        state = dock.get('state')
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


def operator_wait(node,context,token,phase,action,message):
    node.cancel_active()
    gate_id=uuid.uuid4().hex
    # A unique token per wait prevents a delayed/double click from releasing a later gate.
    print('MISSION_STATUS '+json.dumps(dict(phase=phase,token=gate_id,message=message),ensure_ascii=False),flush=True)
    started=time.monotonic()
    while rclpy.ok() and not node.interrupted:
        context(node.pose(.8),mission_wait=True)
        try:
            command=json.loads((CACHE/'operator_confirm.json').read_text())
            if command.get('token')==gate_id and command.get('action')==action and float(command.get('monotonic',0))>=started and 0<=time.monotonic()-float(command['monotonic'])<=10.:
                print('MISSION_STATUS '+json.dumps(dict(phase='driving',token=token,message='확인 완료 · 다음 단계 진행'),ensure_ascii=False),flush=True)
                return
        except (OSError,ValueError,TypeError,KeyError):pass
        rclpy.spin_once(node,timeout_sec=.075)
    raise RouteFailure(130,'cancelled')


def run(node, points):
    if not node.client.wait_for_server(timeout_sec=15.):
        raise RouteFailure(3, 'Nav2 navigate_to_pose 액션 서버를 찾을 수 없습니다.')
    # Two publishers used to race on cmd_vel. Do not silently override teleop.
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=.05)
    publishers = node.get_publishers_info_by_topic('/cmd_vel')
    competitors = sorted({p.node_name for p in publishers if p.node_name != 'pinky_ultrasonic_safety'})
    if competitors:
        raise RouteFailure(11, '수동/다른 cmd_vel 발행자를 종료한 뒤 시작하세요: '+', '.join(competitors))
    if not any(p.node_name == 'pinky_ultrasonic_safety' for p in publishers):
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
    visits = [points[i]['label'] for i in anchors]
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
        cursor = start_index+1
        print(f"WAYPOINT_START {cursor+1} {points[cursor]['label']}", flush=True)
        def context(pose, dock_search=False, mission_wait=False):
            near_start = math.hypot(pose[0]-start['x_m'], pose[1]-start['y_m']) < .12
            near_end = math.hypot(pose[0]-end['x_m'], pose[1]-end['y_m']) < .12
            atomic_json('route_context.json', dict(token=token, segment=segment,
                monotonic=time.monotonic(), dock_search=dock_search, mission_wait=mission_wait, use_ir=bool(end.get("use_ir")), transit=start["label"]=="주차장" or end["label"]=="주차장", corner=(start_turn and near_start) or (end_turn and near_end),
                start_transition=not course_departure and (start_turn or start['label'] in junctions),
                end_transition=end_turn or end['label'] in junctions,
                start_xy=[start['x_m'],start['y_m']], end_xy=[end['x_m'],end['y_m']],
                start_label=start['label'], end_label=end['label'],
                dock_xy=[end['x_m'],end['y_m']] if end['label'] == '5' else None,
                outgoing_heading_rad=outgoing if end_turn else heading,
                label=f"{start['label']}→{end['label']}", heading_rad=heading))
        pose = node.pose()
        context(pose)
        if not startup_armed and start["label"]!="주차장" and end.get("use_ir"):
            atomic_json('route_start.json', dict(token=token, monotonic=time.monotonic()))
            startup_armed = True
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = end['x_m'], end['y_m']
        # Arrive facing the incoming segment; next segment owns the corner turn.
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin((end["yaw_rad"] if end.get("parking_return") else end.get("arrival_yaw_rad",heading))/2), math.cos((end["yaw_rad"] if end.get("parking_return") else end.get("arrival_yaw_rad",heading))/2)
        future = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
        if not future.done() or future.exception() is not None:
            raise RouteFailure(4, 'Nav2 목표 전달 응답을 받지 못했습니다.')
        node.active_goal = future.result()
        if not node.active_goal.accepted:
            node.active_goal = None
            raise RouteFailure(5, f"{end['label']} 목표가 거부됐습니다.")
        # Clear a persistent operator stop only after the new goal is accepted.
        # At that point Nav2 has preempted the previous goal and context is fresh.
        try:
            stop=json.loads((CACHE/'stop_latch.json').read_text())
            if node.run_id and stop.get('run_id')==node.run_id:raise RouteFailure(130,'cancelled')
        except (OSError,ValueError,TypeError):pass
        (CACHE/'stop_latch.json').unlink(missing_ok=True)
        result = node.active_goal.get_result_async()
        deadline = time.monotonic()+240.
        previous_pose = pose
        dock_serviced = False
        while rclpy.ok() and not result.done() and time.monotonic() < deadline and not node.interrupted:
            pose = node.pose(.8)
            context(pose)
            feedback = mission_feedback(token)
            if end.get('use_ir') and feedback and feedback.get('docking', {}).get('state') in {'docking','reverse','await_confirmation','stopped','fault','recovery_hold','search','search_hold'}:
                docking_wait(node, context, token)
                dock_serviced = True
                break
            progress = ((pose[0]-start['x_m'])*dx+(pose[1]-start['y_m'])*dy)/(length*length)
            old_progress = ((previous_pose[0]-start['x_m'])*dx+(previous_pose[1]-start['y_m'])*dy)/(length*length)
            while cursor < end_index:
                point = points[cursor]
                at = ((point['x_m']-start['x_m'])*dx+(point['y_m']-start['y_m'])*dy)/(length*length)
                if progress < at:
                    break
                # Interpolate the measured trajectory at the checkpoint plane.
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
        if not result.done() and not dock_serviced:
            raise RouteFailure(6, f"{end['label']} 구간 주행 대기 시간 초과")
        if not dock_serviced:
            response = result.result()
            node.active_goal = None
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                detail = getattr(response.result, 'error_msg', '')
                raise RouteFailure(7, f"{end['label']} 구간 주행 실패 (status {response.status}, {detail})")
            if end['label'] == '5' and end.get('use_ir'):
                docking_wait(node, context, token, require_marker=True)
                dock_serviced = True
        pose = node.pose(2.)
        # A controller may report success just before the last checkpoint plane.
        while cursor < end_index:
            checkpoint(cursor, points[cursor], pose, passing=True, sampling='end_pose_no_crossing_sample')
            cursor += 1
        checkpoint(end_index, end, pose, sampling='docking_service' if dock_serviced else None)
        if end['label']=='5' and not end.get('use_ir'):
            operator_wait(node,context,token,'docking_manual','dock_complete','5번 도착 · 정지 · IR 보정 전 수동 도킹완료 확인 대기')
            operator_wait(node,context,token,'docked_manual','received','도킹완료 · 정지 유지 · 물건 수령 완료 대기')
        if end.get('course_end'):
            node.cancel_active()
            print('MISSION_STATUS '+json.dumps(dict(phase='course_complete',token=token,message='코스 완주 · 정지 · 주행완료 신호 대기'),ensure_ascii=False),flush=True)
            while rclpy.ok() and not node.interrupted:
                context(node.pose(.8), mission_wait=True)
                try:
                    signal_data=json.loads((CACHE/'mission_complete.json','operator_confirm.json').read_text())
                    if signal_data.get('token')==token and 0<=time.monotonic()-float(signal_data.get('monotonic',0))<=10.:
                        break
                except (OSError,ValueError,TypeError): pass
                rclpy.spin_once(node,timeout_sec=.075)
            if node.interrupted: raise RouteFailure(130,'cancelled')
            print('MISSION_STATUS '+json.dumps(dict(phase='returning',token=token,message='주행완료 수신 · 자기 주차장 복귀중'),ensure_ascii=False),flush=True)
    print('ROUTE_DONE', flush=True)


def main():
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
            for name in ('route_start.json','route_context.json','route.pid','mission_complete.json','operator_confirm.json'):
                (CACHE/name).unlink(missing_ok=True)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
