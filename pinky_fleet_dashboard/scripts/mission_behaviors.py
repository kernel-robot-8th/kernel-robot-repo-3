"""Bounded junction traversal and an operator-gated docking state machine."""
from __future__ import annotations
from dataclasses import dataclass
import math
import uuid


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


@dataclass
class Motion:
    linear: float
    angular: float
    state: str


class Crossing:
    def __init__(self):
        self.token = None
        self.state = 'idle'
        self.anchor = None
        self.departure_segment = None
        self.target_heading = None
        self.entered = 0.
        self.travel = 0.
        self.rotation = 0.
        self.previous_odom = None
        self.stable_since = None
        self.done = set()
        self.phase = 'idle'
        self.phase_started = 0.
        self.phase_elapsed = 0.
        self.failure_reason = ''
        self.ir_required = True

    def reset(self, token=None):
        self.__init__()
        self.token = token

    def update(self, context, pose, odom, line_ok, now, linear, angular, ir_required=True):
        token = context.get('token')
        if token != self.token:
            self.reset(token)
        segment = context['segment']
        self.ir_required = ir_required
        if self.state == 'fault':
            return Motion(0., 0., 'crossing_fault')
        if self.state == 'idle':
            start_key, end_key = ('start', segment), ('end', segment)
            near_start = distance(pose, context['start_xy']) <= .25
            near_end = distance(pose, context['end_xy']) <= .15
            if context.get('start_transition') and near_start and start_key not in self.done:
                self.anchor = context['start_xy']
                self.departure_segment = segment
                self.target_heading = context['heading_rad']
                self.done.add(start_key)
            elif context.get('end_transition') and near_end and end_key not in self.done:
                self.anchor = context['end_xy']
                self.departure_segment = segment+1
                self.target_heading = context['outgoing_heading_rad']
                self.done.add(end_key)
                self.done.add(('start', segment+1))
            else:
                return None
            self.state = 'traverse'
            self.entered, self.travel, self.rotation = now, 0., 0.
            self.previous_odom = odom
            self.phase = 'turn' if segment >= self.departure_segment else 'approach'
            self.phase_started = now
        if self.previous_odom is not None:
            self.travel += distance(odom, self.previous_odom)
            self.rotation += abs(angle(odom[2]-self.previous_odom[2]))
        self.previous_odom = odom
        departed = segment >= self.departure_segment
        along = ((pose[0]-self.anchor[0])*math.cos(self.target_heading)
                 +(pose[1]-self.anchor[1])*math.sin(self.target_heading))
        aligned = abs(angle(pose[2]-self.target_heading)) < .20
        geometry_complete = departed and along >= .05 and aligned
        phase = ('reacquire' if geometry_complete or self.phase == 'reacquire'
                 else 'turn' if departed else 'approach')
        if phase != self.phase:
            self.phase, self.phase_started = phase, now
        self.phase_elapsed = now-self.phase_started
        # Approach, turn and line reacquisition must not consume one timer.
        limit = {'approach': 20., 'turn': 20., 'reacquire': 12.}[self.phase]
        if self.travel > .65:
            self.failure_reason = 'distance_limit'
        elif self.rotation > 3.6:
            self.failure_reason = 'rotation_limit'
        elif self.phase_elapsed > limit:
            self.failure_reason = self.phase+'_timeout'
        elif now-self.entered > 60.:
            self.failure_reason = 'overall_timeout'
        if self.failure_reason:
            self.state = 'fault'
            return Motion(0., 0., 'crossing_fault')
        # Keep reacquisition latched while IR steers away from the map heading.
        # Rechecking alignment here used to switch back to Nav2 mid-search.
        if self.phase == 'reacquire':
            self.state = 'reacquire'
            if line_ok or not ir_required:
                if self.stable_since is None:
                    self.stable_since = now
                if now-self.stable_since >= .3:
                    self.state = 'idle'
                    self.phase = 'idle'
                    self.stable_since = None
                    return None
            else:
                self.stable_since = None
            # The safety controller supplies the shared IR recovery controller.
            # Do not blindly advance while no sensor can see the white line.
            return Motion(0., 0., 'crossing_reacquire')
        self.stable_since = None
        return Motion(max(-.02, min(.02, linear)), max(-.20, min(.20, angular)), 'crossing_traverse')

    def status(self):
        return dict(state=self.state, travel_m=self.travel, rotation_rad=self.rotation,
                    target_heading_rad=self.target_heading, phase=self.phase,
                    phase_elapsed_s=self.phase_elapsed, failure_reason=self.failure_reason,
                    ir_required=self.ir_required)


class Docking:
    ACTIVE = {'docking', 'reverse', 'await_confirmation', 'stopped', 'fault', 'recovery_hold', 'search', 'search_hold'}

    def __init__(self):
        self.token = None
        self.state = 'idle'
        self.dock_id = None
        self.complete = False
        self.handled = set()
        self.sample_at = None
        self.white_count = 0
        self.black_count = 0
        self.all_white = False
        self.origin = None
        self.reverse_origin = None
        self.started = 0.
        self.reverse_started = 0.
        self.heading = 0.
        self.reversed_once = False
        self.command_id = None
        self.command_error = ''
        self.message = ''
        self.search_origin = None
        self.search_started = 0.
        self.search_leg = 0
        self.search_travel = 0.
        self.search_previous = None

    def reset(self, token=None):
        self.__init__()
        self.token = token

    def observe(self, white, sample_at):
        self.all_white = white
        if sample_at != self.sample_at:
            self.white_count = self.white_count+1 if white else 0
            self.black_count = 0 if white else self.black_count+1
            self.sample_at = sample_at

    def command(self, command):
        if command.get('id') == self.command_id:
            return
        if command.get('token') != self.token or command.get('dock_id') != self.dock_id:
            return
        self.command_id, self.command_error = command.get('id'), ''
        if command.get('action') == 'complete' and self.state in {'docking','await_confirmation','recovery_hold'}:
            self.complete = True
            self.white_count = 0  # Confirm fresh white samples after the signal.
            self.message = '도킹완료 신호 수신 · 흰색 유지 확인 중'
        elif command.get('action') == 'received' and self.state == 'stopped':
            self.state = 'released'
            self.message = '물건 수령 완료 · 경로 재개'
        else:
            self.command_error = '현재 도킹 상태에서 처리할 수 없는 명령입니다.'

    def update(self, context, pose, odom, now, front_blocked):
        if context.get('token') != self.token:
            self.reset(context.get('token'))
            return None
        eligible = (context.get('end_label') == '5' and context.get('dock_xy') is not None
                    and context['segment'] not in self.handled
                    and distance(pose, context['dock_xy']) <= (.10 if context.get('dock_search') else .05))
        if self.state in {'idle', 'released'}:
            if not eligible:
                return None
            if self.white_count < 3:
                if not context.get('dock_search'):
                    return None
                self.state, self.dock_id = 'search', uuid.uuid4().hex
                self.search_origin, self.heading = odom, odom[2]
                self.search_started, self.search_leg, self.search_travel = now, 0, 0.
                self.search_previous = odom
            else:
                self.state, self.dock_id = 'docking', uuid.uuid4().hex
                self.handled.add(context['segment'])
                self.complete, self.reversed_once = False, False
                self.origin, self.heading, self.started = odom, odom[2], now
                self.message = '도킹중 · 저속 진입 · 도킹완료 신호 대기'
        if self.state == 'search':
            self.search_travel += distance(odom, self.search_previous)
            self.search_previous = odom
            if self.white_count >= 3:
                self.state = 'docking'
                self.handled.add(context['segment'])
                self.complete, self.reversed_once = False, False
                self.origin, self.started = odom, now
                self.message = '흰색 패턴 확인 · 도킹중 · 도킹완료 신호 대기'
            else:
                offset = ((odom[0]-self.search_origin[0])*math.cos(self.heading)
                          +(odom[1]-self.search_origin[1])*math.sin(self.heading))
                targets = (.03, -.03, 0.)
                target = targets[self.search_leg]
                if abs(target-offset) <= .003:
                    self.search_leg += 1
                if (self.search_leg >= len(targets) or now-self.search_started >= 26.
                        or self.search_travel >= .15 or distance(odom, self.search_origin) > .045
                        or abs(angle(odom[2]-self.heading)) > .20):
                    self.state = 'search_hold'
                    self.message = '흰색 탐색 종료 · 정지 유지 · IR 보정/센서 높이와 도킹 위치를 확인하세요'
                    return Motion(0., 0., 'docking_wait')
                target = targets[self.search_leg]
                velocity = math.copysign(.01, target-offset)
                self.message = '도킹 흰색 탐색 · 1cm/s · 출발점 기준 앞뒤 3cm 이내'
                if velocity > 0 and front_blocked:
                    return Motion(0., 0., 'docking_wait')
                return Motion(velocity, max(-.08, min(.08, angle(self.heading-odom[2]))), 'docking_search')
        if self.state == 'search_hold':
            if self.white_count >= 3:
                self.state = 'docking'
                self.handled.add(context['segment'])
                self.complete, self.reversed_once = False, False
                self.origin, self.started = odom, now
                self.message = '흰색 패턴 확인 · 도킹중'
            else:
                return Motion(0., 0., 'docking_wait')
        if self.state == 'released':
            return None
        if self.state == 'fault':
            return Motion(0., 0., 'docking_fault')
        if self.state == 'stopped':
            self.message = ('도킹완료 · 물건 수령 완료 버튼 대기' if self.all_white
                            else '도킹완료 · IR 변화 감지 · 정지 유지')
            return Motion(0., 0., 'docking_stopped')
        if self.complete and self.white_count >= 3 and self.state != 'reverse':
            self.state = 'stopped'
            self.message = '도킹완료 · 물건 수령 완료 버튼 대기'
            return Motion(0., 0., 'docking_stopped')
        if self.complete and self.all_white and self.state != 'reverse':
            return Motion(0., 0., 'docking_wait')
        if self.state == 'recovery_hold':
            return Motion(0., 0., 'docking_wait')
        if self.state == 'reverse':
            if self.white_count >= 3 or distance(odom, self.reverse_origin) >= .02 or now-self.reverse_started >= 2.:
                self.state = 'await_confirmation'
                self.message = '후진 보정 종료 · 정지 · 흰색/도킹완료 확인 대기'
                return Motion(0., 0., 'docking_wait')
            return Motion(-.01, max(-.08, min(.08, angle(self.heading-odom[2]))), 'docking_reverse')
        if not self.all_white and self.black_count < 3:
            return Motion(0., 0., 'docking_wait')
        if not self.all_white and not self.reversed_once:
            self.state, self.reverse_origin, self.reverse_started = 'reverse', odom, now
            self.reversed_once = True
            self.message = '도킹중 IR 이탈 · 최대 2cm 후진 보정'
            return Motion(0., 0., 'docking_reverse')
        if self.state in {'await_confirmation','recovery_hold'}:
            return Motion(0., 0., 'docking_wait')
        if not self.all_white or front_blocked or distance(odom, self.origin) >= .10 or now-self.started >= 10.:
            self.state = 'await_confirmation'
            self.message = '도킹 저속 이동 제한 도달/장애물 · 정지 · 도킹완료 확인 대기'
            return Motion(0., 0., 'docking_wait')
        return Motion(.01, max(-.08, min(.08, angle(self.heading-odom[2]))), 'docking_creep')

    def status(self):
        return dict(state=self.state, token=self.token, dock_id=self.dock_id,
                    complete=self.complete, all_white=self.all_white, white_count=self.white_count,
                    black_count=self.black_count,
                    command_id=self.command_id, command_error=self.command_error,
                    message=self.message, handled=sorted(self.handled))
