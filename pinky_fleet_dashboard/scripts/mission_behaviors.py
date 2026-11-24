"""제한을 둔 교차 구간 통과와 운영자 신호로 진행하는 도킹 상태 기계 등, 임무 중 특수 행동 모음.

실행 위치: 로봇 (주행준비 때 runtime_bundle로 올라감)
사용처: ultrasonic_safety.py가 import해 행동 선택에 쓴다 (Crossing, UltrasonicDetour, Docking, DockEntry,
DockExit, BackOff, GiveWay). ROS import가 없어 tests/에서 오프라인으로 시험한다.

핵심: 각 클래스는 매 주기 update()로 Motion(선속도 m/s, 각속도 rad/s, 상태 이름) 또는 None(Nav2 명령을
그대로 둠)을 돌려준다. 모든 동작에 거리·회전·시간 한도가 있고, 넘으면 fault로 정지한다(fail closed).
pose는 지도 (x, y, yaw), odom은 바퀴 오도메트리 (x, y, yaw), 단위 m·rad·s.
try/except import는 PC에서는 패키지(scripts.)로, 로봇에서는 같은 폴더의 일반 모듈로 읽히기 때문이다.
"""
from __future__ import annotations
from dataclasses import dataclass
import json
import math
import uuid
try:
    from .motion_gate import geometry, range_arc, detour_clearance
except ImportError:
    from motion_gate import geometry, range_arc, detour_clearance


# 각도를 -pi~pi로 감싼다.
def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


@dataclass
class Motion:
    """행동이 내는 속도 명령: linear(m/s), angular(rad/s), state(로그·화면용 이름)."""
    linear: float
    angular: float
    state: str


class Crossing:
    """경로 모퉁이(교차점)를 Nav2 명령을 저속(±0.02 m/s, ±0.2 rad/s)으로 제한해 지나고, 이후 IR로 선을 다시 잡는다.

    state: idle(관여 안 함) → traverse(통과 중) → reacquire(선 재확인) → idle, 한도 초과 시 fault(정지 유지).
    phase: approach(꺾는 점까지) → turn(다음 구간으로 회전) → reacquire. done은 이미 처리한 (start|end, 구간)이다.
    """
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
            # 구간 시작점 25 cm, 끝점 15 cm 안에 들어오면 통과 동작을 시작한다.
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
        # 접근·회전·선 재확인이 타이머 하나를 나눠 쓰면 안 된다(단계별 20/20/12초).
        # 그 밖의 한도: 이동 0.65 m, 누적 회전 3.6 rad, 전체 60초.
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
        # IR이 지도 방향에서 벗어나게 조향하는 동안에도 재확인 상태를 유지한다.
        # 여기서 정렬을 다시 검사하던 예전 코드는 탐색 도중 Nav2로 되돌아갔다.
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
            # 선 찾기 조향은 안전 제어기(ultrasonic_safety)의 공용 IR 복구 제어기가 맡는다.
            # 어느 센서도 흰 선을 못 보는 동안 눈감고 전진하지 않는다. 선이 0.3초 안정되면 Nav2로 돌려준다.
            return Motion(0., 0., 'crossing_reacquire')
        self.stable_since = None
        return Motion(max(-.02, min(.02, linear)), max(-.20, min(.20, angular)), 'crossing_traverse')

    def status(self):
        return dict(state=self.state, travel_m=self.travel, rotation_rad=self.rotation,
                    target_heading_rad=self.target_heading, phase=self.phase,
                    phase_elapsed_s=self.phase_elapsed, failure_reason=self.failure_reason,
                    ir_required=self.ir_required)


class UltrasonicDetour:
    """오도메트리로 범위를 제한한 초음파 장애물 우회. 거리 센서 하나로는 물체 폭을 잴 수 없다.

    phase: stopped(0.5초 정지) → turning(60° 회전) → offset(옆으로 offset만큼) → aligning(원래 방향)
    → passing(장애물 지나 pass_distance 전진) → return_turn(반대로 60°) → returning(선 쪽으로)
    → line_align → line_confirm(IR 확인) → idle(완료). 60초 또는 출발점에서 0.70 m를 넘으면 fault.
    low_points는 초음파가 본 장애물 점(지도 좌표)을 기억해 LiDAR 점과 함께 궤적 검사에 쓴다.
    """
    def __init__(self, config=None):
        self.config = geometry(config)
        self.phase = 'idle'
        self.origin = None
        self.started = 0.
        self.clear_since = None
        self.direction = 1.
        self.low_points = []
        self.failure = ''
        self.line_since = None
        self.line_count = 0
        self.line_sample_at = None
        self.require_line = False

    def start(self, pose, range_m, points, now, require_line=False):
        self.__init__(self.config)
        self.require_line = require_line
        self.origin, self.started, self.phase = pose, now, 'stopped'
        # 통과 거리: 센서 위치 + 측정 거리 + 차체 반길이 + 8 cm, 0.25~0.50 m로 자른다.
        # 옆 이동 폭: 설정값과 차체 반폭 + 3 cm 중 큰 값. 좌우 중 여유(clearance)가 큰 쪽으로 돈다.
        self.pass_distance = min(.50, max(.25, self.config['ultrasound_x_m']+range_m + self.config['half_length_m']+.08))
        self.offset = max(self.config['detour_offset_m'], self.config['half_width_m']+.03)
        obstacle = points+range_arc(range_m,self.config)
        candidates = [(detour_clearance(obstacle,d,self.config,self.pass_distance),d) for d in (1.,-1.)]
        score,self.direction = max(candidates,key=lambda candidate:candidate[0])
        if score < 0:self.phase,self.failure='fault','no_clear_detour'
        self.remember(pose, range_m)

    def remember(self, pose, distance_m):
        if distance_m is None or not math.isfinite(distance_m):return
        c,s=math.cos(pose[2]),math.sin(pose[2])
        for x,y in range_arc(distance_m,self.config):
            hit = pose[0]+c*x-s*y,pose[1]+s*x+c*y
            # 8 mm보다 가까운 점은 중복으로 보고 버리며, 최근 200개만 남긴다.
            if not self.low_points or min(distance(hit,p) for p in self.low_points)>.008:self.low_points.append(hit)
        self.low_points=self.low_points[-200:]

    def update(self, pose, range_m, points, now, clear_path, line_center=False, line_sample_at=None):
        if self.phase == 'fault':
            return Motion(0., 0., 'ultrasonic_fault')
        if now-self.started > 60. or distance(pose, self.origin) > .70:
            self.phase, self.failure = 'fault', 'detour_limit'
            return Motion(0., 0., 'ultrasonic_fault')
        # 초음파 10 cm 이하면 막힘(이동 멈춤), 회전 후 15 cm 이상이 0.75초 이어지면 다음 단계로 간다.
        blocked = range_m is not None and math.isfinite(range_m) and range_m <= .10
        if blocked:
            self.remember(pose, range_m)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        # 기억한 초음파 점을 로봇 기준 좌표로 바꿔 LiDAR 점과 합친다. forward·lateral은 출발 자세 기준 이동량이다.
        obstacles = points + [(c*(x-pose[0])+s*(y-pose[1]), -s*(x-pose[0])+c*(y-pose[1]))
                              for x, y in self.low_points]
        ox, oy, heading = self.origin
        forward = (pose[0]-ox)*math.cos(heading)+(pose[1]-oy)*math.sin(heading)
        lateral = self.direction*(-(pose[0]-ox)*math.sin(heading)+(pose[1]-oy)*math.cos(heading))
        v, w = 0., 0.
        if self.phase == 'stopped':
            if now-self.started >= .5:
                self.phase = 'turning'
        elif self.phase in {'turning', 'aligning', 'return_turn', 'line_align'}:
            target = heading + (self.direction*math.pi/3 if self.phase == 'turning' else -self.direction*math.pi/3 if self.phase == 'return_turn' else 0.)
            error = angle(target-pose[2])
            if abs(error) > .035:
                w = math.copysign(min(.25, max(.08, 1.2*abs(error))), error)
                self.clear_since = None
            elif range_m is None or range_m >= .15:
                if self.clear_since is None:
                    self.clear_since = now
                if now-self.clear_since >= .75:
                    self.phase = {'turning':'offset','aligning':'passing','return_turn':'returning','line_align':'line_confirm'}[self.phase]
                    self.clear_since = None
            else:
                self.clear_since = None
        elif self.phase == 'offset':
            if lateral >= self.offset:
                self.phase = 'aligning'
            elif not blocked:
                v, w = .03, max(-.12, min(.12, angle(heading+self.direction*math.pi/3-pose[2])))
        elif self.phase == 'passing':
            if forward >= self.pass_distance:
                self.phase = 'return_turn'
            elif not blocked:
                v, w = .03, max(-.12, min(.12, angle(heading-pose[2])))
        elif self.phase == 'returning':
            if line_center or lateral <= .008:
                self.phase = 'line_align'
            elif not blocked:
                v,w=.02,max(-.12,min(.12,angle(heading-self.direction*math.pi/3-pose[2])))
        elif self.phase == 'line_confirm':
            if not self.require_line:self.phase='idle'
            elif line_center:
                if self.line_since is None:self.line_since=now
                sample=now if line_sample_at is None else line_sample_at
                if sample!=self.line_sample_at:self.line_count+=1;self.line_sample_at=sample
                # 새 IR 샘플 3개 이상 + 가운데 흰색 0.3초 연속이어야 복귀 완료로 본다.
                if now-self.line_since>=.3 and self.line_count>=3:self.phase='idle'
            else:self.line_since=None;self.line_count=0
        if not clear_path(obstacles, v, w):
            return Motion(0., 0., 'ultrasonic_clearance_wait')
        return Motion(v, w, 'ultrasonic_'+self.phase)

    def status(self):
        return dict(phase=self.phase, failure=self.failure, lateral_target_m=getattr(self,'offset',self.config['detour_offset_m']),
                    footprint_width_m=2*self.config['half_width_m'], direction=self.direction)


class Docking:
    """5번 자리(IR 흰색 패턴)에서 저속 진입 후 관제 신호(complete → received)를 기다리는 도킹 상태 기계.

    state: idle → (흰색 샘플 3개 미만이고 dock_search면) search(앞뒤 3 cm, 1 cm/s 탐색) / search_hold(탐색 실패
    정지) → docking(1 cm/s 직진, 10 cm·10초 한도) → reverse(IR 이탈 시 한 번만 최대 2 cm 후진) →
    await_confirmation(정지, 완료 신호 대기) → anchor_wait(완료 신호 + 흰색 확인, 지도 위치 재설정 대기) →
    stopped(정지, 수령 대기) → released(received 수신, 경로 재개). fault·recovery_hold는 정지 유지다.
    stopped·recovery_hold는 ultrasonic_safety가 지도 위치 재설정 뒤·재시작 복구 때 직접 넣는다.
    """
    ACTIVE = {'docking', 'reverse', 'await_confirmation', 'stopped', 'fault', 'recovery_hold', 'search', 'search_hold', 'anchor_wait'}

    def __init__(self):
        self.token = None
        self.state = 'idle'
        self.dock_id = None
        self.complete = False
        self.anchored = False
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
        """IR 샘플마다 연속 흰색·검정 횟수를 센다. 같은 sample_at(같은 샘플)은 두 번 세지 않는다."""
        self.all_white = white
        if sample_at != self.sample_at:
            self.white_count = self.white_count+1 if white else 0
            self.black_count = 0 if white else self.black_count+1
            self.sample_at = sample_at

    def command(self, command):
        """관제 명령(complete·received)을 받는다. 같은 id는 무시하고, 토큰·dock_id가 다르면 오래된 명령으로 버린다."""
        if command.get('id') == self.command_id:
            return
        if command.get('token') != self.token or command.get('dock_id') != self.dock_id:
            return
        self.command_id, self.command_error = command.get('id'), ''
        if command.get('action') == 'complete' and self.state in {'docking','await_confirmation','recovery_hold'}:
            self.complete = True
            self.white_count = 0  # 신호 뒤의 새 흰색 샘플로 다시 확인한다.
            self.message = '도킹완료 신호 수신 · 흰색 유지 확인 중'
        elif command.get('action') == 'received' and self.state == 'stopped' and self.anchored:
            self.state = 'released'
            self.message = '물건 수령 완료 · 경로 재개'
        else:
            self.command_error = '현재 도킹 상태에서 처리할 수 없는 명령입니다.'

    def update(self, context, pose, odom, now, front_blocked):
        """Motion 또는 None(도킹과 무관, Nav2 그대로). 5번 끝점 5 cm(탐색 허용 시 10 cm) 안에서만 시작한다."""
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
                self.complete, self.reversed_once, self.anchored = False, False, False
                self.origin, self.heading, self.started = odom, odom[2], now
                self.message = '도킹중 · 저속 진입 · 도킹완료 신호 대기'
        if self.state == 'search':
            self.search_travel += distance(odom, self.search_previous)
            self.search_previous = odom
            if self.white_count >= 3:
                self.state = 'docking'
                self.handled.add(context['segment'])
                self.complete, self.reversed_once, self.anchored = False, False, False
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
                self.complete, self.reversed_once, self.anchored = False, False, False
                self.origin, self.started = odom, now
                self.message = '흰색 패턴 확인 · 도킹중'
            else:
                return Motion(0., 0., 'docking_wait')
        if self.state == 'released':
            return None
        if self.state == 'fault':
            return Motion(0., 0., 'docking_fault')
        if self.state == 'anchor_wait':
            return Motion(0., 0., 'docking_anchor_wait')
        if self.state == 'stopped':
            self.message = ('도킹완료 · 물건 수령 완료 버튼 대기' if self.all_white
                            else '도킹완료 · IR 변화 감지 · 정지 유지')
            return Motion(0., 0., 'docking_stopped')
        if self.complete and self.white_count >= 3 and self.state != 'reverse':
            self.state = 'anchor_wait'
            self.message = '도킹완료 확인 · 5번 지도 위치 재설정 중'
            return Motion(0., 0., 'docking_anchor_wait')
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
                    complete=self.complete, anchored=self.anchored, all_white=self.all_white, white_count=self.white_count,
                    black_count=self.black_count,
                    command_id=self.command_id, command_error=self.command_error,
                    message=self.message, handled=sorted(self.handled))


class DockExit:
    """연속된 바퀴 오도메트리로 스펀지 스테이션에서 후진해 빠져나온다(2 cm/s, 0.10~0.50 m, 기본 0.20 m).

    동작 중 재시작되면 실패로 닫는다(fail closed): 시작점을 새로 잡지 않는다.
    persist 파일에 (token, segment) 키를 남겨, 재시작 후 같은 키가 보이면 fault로 둔다.
    뒤·옆 충돌 검사는 독립 최종 감시기(velocity_guard)가 계속 맡는다.
    옆으로 2 cm, 회전 0.12 rad, 30초를 넘으면 fault다.
    """
    def __init__(self):
        self.key = None
        self.origin = None
        self.started = 0.
        self.state = 'idle'
        self.progress = 0.
        self.message = ''

    def update(self, context, odom, now, persist):
        key = [context['token'], context['segment']]
        if key != self.key:
            self.key, self.origin, self.started = key, odom, now
            try:
                previous = json.loads(persist.read_text())
            except FileNotFoundError:
                previous = {}
            except (OSError, ValueError):
                previous = {'key': key}
            self.state = 'fault' if previous.get('key') == key else 'reversing'
            try:
                self.target = max(.10, min(.50, float(context.get('dock_exit_m') or .20)))
            except (TypeError, ValueError):
                self.target = .20
            try:
                temporary = persist.with_suffix('.tmp')
                temporary.write_text(json.dumps(dict(key=key, state=self.state)))
                temporary.replace(persist)
            except OSError:
                self.state = 'fault'
        dx, dy = odom[0]-self.origin[0], odom[1]-self.origin[1]
        self.progress = -(dx*math.cos(self.origin[2])+dy*math.sin(self.origin[2]))
        lateral = abs(-dx*math.sin(self.origin[2])+dy*math.cos(self.origin[2]))
        if self.state == 'reversing':
            if lateral > .02 or abs(angle(odom[2]-self.origin[2])) > .12 or now-self.started > 30.:
                self.state = 'fault'
            elif self.progress >= getattr(self, 'target', .20)-.002:
                self.state = 'complete'
        target_cm = getattr(self, 'target', .20)*100
        self.message = {'reversing':f'도킹 스테이션 탈출 · 2cm/s로 {target_cm:.0f}cm 후진',
                        'complete':f'{target_cm:.0f}cm 후진 완료 · 다음 이동 준비',
                        'fault':'도킹 후진 중단 · 재시작/방향/이동 제한 확인'}[self.state]
        return Motion(-.02 if self.state == 'reversing' else 0., 0., 'docking_exit_'+self.state)

    def status(self):
        return dict(key=self.key, state=self.state, progress_m=self.progress, target_m=getattr(self, 'target', .20),
                    message=self.message)


class DockEntry:
    """정렬점에서 바퀴 오도메트리로 5번 자리에 곧게 들어간다(2026-11-16).

    바로 앞에 스테이션이 있으므로 초음파와 정면 LiDAR는 보지 않는다. 좌우 LiDAR는 최종 감시기가 계속 본다.
    핑키가 맞춘 방향(Nav2, 3° 이내)을 유지한다. 동작 중 재시작되면 DockExit처럼 실패로 닫는다.
    옆으로 3 cm 밀리거나 0.15 rad 넘게 돌거나 25초를 넘으면 fault로 멈춘다.

    천장 카메라 안내(2026-11-18): 들어가는 동안 카메라가 본 대로 조향한다. `guide`는 /fleet/dock_guide에서 온
    최신 값으로, 등록 자리에 대한 핑키의 오프셋을 핑키 자신의 축으로 나타낸 (forward, left, yaw) m·rad다
    (자리에 못 미쳤으면 forward가 음수). 자리의 선 쪽으로 돌아 그 선에 맞추고(2차 선 추종:
    yaw rate = -(K_YAW * yaw + K_LEFT * left)), 센 거리 대신 카메라가 자리라고 하는 곳에서 멈춘다.
    카메라는 정지점을 센 거리 기준 몇 cm만 옮긴다(앞으로 GUIDE_SHORT_M, 뒤로 GUIDE_OVER_M). 새 안내가 없으면
    예전처럼 맞춘 방향으로 달리므로, 마커가 가려지거나 PC가 조용해도 손해가 없다.

    카메라가 넘겨받기(context dock_capture, 2026-11-19): 카메라가 자리 앞의 핑키를 보는 즉시 경로 실행기가
    넘겨준다. Nav2가 어디에 어느 방향으로 세웠든 상관없다. 그때는 지킬 정렬선이 없으므로 먼저 제자리에서 돌아
    자리의 선(카메라 yaw)을 향한 뒤 위처럼 조향하며 들어가, 카메라가 자리를 보는 곳에서 멈춘다. 카메라만
    움직인다: 안내가 끊기면 서 있고(최대 CAPTURE_BLIND_S), CAPTURE_TRAVEL_M나 CAPTURE_TIME_S를 넘으면 포기한다.

    스스로 정렬하기(context dock_turn_rad, 2026-11-19): Nav2는 정렬점에 cm 단위로 서서 돌려고 몇 분씩 헤맸다
    (스테이션 옆 충돌 검사, 3 cm 허용 오차 때문). 이제 실행기는 핑키가 그 점 근처에 오면 도킹 방향까지 남은
    회전량과 함께 넘겨주고, 핑키는 오도메트리로 그만큼 제자리 회전한 뒤 예전처럼 들어간다."""
    # 진입 속도 3 cm/s
    SPEED_MPS = .03
    TAKEOVER_LEFT_M = .06          # 직진 조향으로 되돌릴 수 있는 것보다 선에서 옆으로 먼 거리(m)
    SQUARE_DONE_RAD, SQUARE_TIME_S = .03, 20.        # 도킹 방향으로 스스로 도는 회전(context dock_turn_rad)의 완료 오차 rad, 제한 시간 s
    CAPTURE_FACE_RAD, CAPTURE_FACED_RAD = .35, .12   # 첫 값 넘으면 제자리 회전, 둘째 값 아래면 다시 전진(rad, 히스테리시스)
    CAPTURE_TURN_LIMIT, CAPTURE_TURN_MIN = .5, .25   # 제자리 회전 rad/s. 최소값보다 느리면 바퀴가 웅 소리만 내고 안 돈다
    CAPTURE_LEFT_LIMIT = .75       # rad/s: 약 24°까지 비스듬히 간다. 되돌아올 거리가 더 있기 때문이다
    CAPTURE_BLIND_S, CAPTURE_TIME_S, CAPTURE_TRAVEL_M = 8., 75., .70  # 안내 없이 최대 8초, 전체 75초, 이동 0.70 m
    K_YAW, K_LEFT = 1.8, 24.       # 1/s, m당 rad/s: 3 cm 어긋남을 약 15 cm 이동 안에 바로잡는다
    GUIDE_LEFT_LIMIT = .45         # 옆 오차 항이 요구할 수 있는 rad/s: 최대 약 14° 비스듬히
    GUIDE_TURN_LIMIT = .35         # rad/s
    GUIDE_SHORT_M, GUIDE_OVER_M = .08, .06  # 카메라가 센 거리보다 8 cm 앞·6 cm 뒤까지 정지점을 옮길 수 있다
    GUIDE_STOP_M = .005            # 카메라 기준 이만큼 앞에서 멈춘다: 카메라 영상은 약 0.2초 늦다

    def __init__(self):
        self.key = None
        self.origin = None
        self.started = 0.
        self.state = 'idle'
        self.progress = 0.
        self.target = .25
        self.message = ''
        self.guided = False
        self.guide = None
        self.capture = False
        self.turn, self.turn_from, self.squaring = 0., 0., False
        self.side = 0.  # 넘겨받을 때 도킹 선에서 왼쪽으로 떨어진 거리 m (context dock_side_m)
        self.facing = False
        self.travel = 0.
        self.last_odom = None
        self.heard = 0.

    def update(self, context, odom, now, persist, guide=None):
        key = [context['token'], context['segment']]
        if key != self.key:
            self.key, self.origin, self.started = key, odom, now
            self.guided, self.guide = False, None
            self.capture, self.facing = bool(context.get('dock_capture')), False
            try:
                self.turn = angle(float(context.get('dock_turn_rad') or 0.))
            except (TypeError, ValueError):
                self.turn = 0.
            self.turn_from, self.squaring = odom[2], not self.capture and abs(self.turn) > .04
            try:
                self.side = max(-.08, min(.08, float(context.get('dock_side_m') or 0.)))
            except (TypeError, ValueError):
                self.side = 0.
            self.travel, self.last_odom, self.heard = 0., odom, now
            try:
                self.target = max(.05, min(.40, float(context.get('dock_entry_m') or .25)))
            except (TypeError, ValueError):
                self.target = .25
            try:
                previous = json.loads(persist.read_text())
            except FileNotFoundError:
                previous = {}
            except (OSError, ValueError):
                previous = {'key': key}
            self.state = 'fault' if previous.get('key') == key else 'entering'
            try:
                temporary = persist.with_suffix('.tmp')
                temporary.write_text(json.dumps(dict(key=key, state=self.state)))
                temporary.replace(persist)
            except OSError:
                self.state = 'fault'
        dx, dy = odom[0]-self.origin[0], odom[1]-self.origin[1]
        self.progress = dx*math.cos(self.origin[2])+dy*math.sin(self.origin[2])
        lateral = abs(-dx*math.sin(self.origin[2])+dy*math.cos(self.origin[2]))
        heading_error = angle(self.origin[2]-odom[2])
        self.guide = guide
        if guide is not None:
            self.guided = True
        if (not self.capture and self.state == 'entering' and guide is not None
                and (abs(guide[2]) > self.CAPTURE_FACE_RAD or abs(guide[1]) > self.TAKEOVER_LEFT_M)):
            # 카메라가 보기에 돌아서 있거나 선에서 옆으로 많이 벗어났다(2026-11-21: Nav2가 틀린 지도 방향으로
            # "정렬"한 뒤 93° 어긋남). 직진으로는 고칠 수 없어 예전엔 fault로 멈췄다. 이제 카메라가 넘겨받아
            # 먼저 제자리에서 돌린다.
            self.capture, self.squaring, self.facing = True, False, False
            self.travel, self.last_odom, self.heard, self.started = 0., odom, now, now
        if self.capture:
            return self.captured(odom, now, guide)
        if self.state == 'entering' and self.squaring:
            remaining = angle(self.turn_from+self.turn-odom[2])
            if now-self.started > self.SQUARE_TIME_S:
                self.state, self.message = 'fault', '5번 진입 중단 · 제자리에서 방향을 못 맞춤 (바퀴/바닥 확인)'
                return Motion(0., 0., 'docking_entry_fault')
            if abs(remaining) > self.SQUARE_DONE_RAD:
                self.message = f'5번 진입 · 제자리에서 방향 맞춤 (남은 각도 {abs(math.degrees(remaining)):.0f}도)'
                rate = max(self.CAPTURE_TURN_MIN, min(self.CAPTURE_TURN_LIMIT, 1.5*abs(remaining)))
                return Motion(0., math.copysign(rate, remaining), 'docking_entry_entering')
            # 정렬 완료: 요청받은 방향 위에서, 여기서부터 직진을 시작한다.
            self.squaring = False
            self.origin, self.started = (odom[0], odom[1], angle(self.turn_from+self.turn)), now
            dx = dy = self.progress = lateral = 0.
            heading_error = angle(self.origin[2]-odom[2])
        # 도킹 선 옆(dock_side_m)에서 넘겨받았고 카메라가 없으면: 같은 조향을 오도메트리로 해서, 바퀴가 아는
        # 범위 안에서 선 위로 데려간다.
        if guide is None and abs(self.side) > .008:
            steer = (self.progress-self.target, self.side-dx*math.sin(self.origin[2])+dy*math.cos(self.origin[2]), -heading_error)
        else:
            steer = guide
        # 한 번이라도 조향했다면 경로가 처음의 직선이 아니므로 한도를 넓힌다(옆 m, 방향 rad, 시간 s).
        lateral_limit, heading_limit, time_limit = (.10, .50, 35.) if self.guided or steer is not None else (.03, .15, 25.)
        if self.state == 'entering':
            if lateral > lateral_limit or abs(heading_error) > heading_limit or now-self.started > time_limit:
                self.state = 'fault'
            elif guide is not None and guide[0] >= -self.GUIDE_STOP_M and self.progress >= self.target-self.GUIDE_SHORT_M:
                self.state = 'complete'  # 카메라가 자리 위에 있다고 본다
            elif self.progress >= self.target+(self.GUIDE_OVER_M if guide is not None else 0.)-.003:
                self.state = 'complete'
        how = ' · 천장 캠을 보며 방향 맞춤' if guide is not None else ''
        self.message = {'entering': f'5번 진입 · 3cm/s로 {self.target*100:.0f}cm 직진 (앞은 안 보고 좌우 라이다만){how}',
                        'complete': '5번 진입 완료 · 정지' + (' (천장 캠이 본 자리)' if self.guided else ''),
                        'fault': '5번 진입 중단 · 옆으로 밀림/방향/시간 제한 확인'}[self.state]
        if self.state == 'entering':
            if steer is not None:
                sideways = max(-self.GUIDE_LEFT_LIMIT, min(self.GUIDE_LEFT_LIMIT, self.K_LEFT*steer[1]))
                turn = -(self.K_YAW*steer[2]+sideways)
                return Motion(self.SPEED_MPS, max(-self.GUIDE_TURN_LIMIT, min(self.GUIDE_TURN_LIMIT, turn)),
                              'docking_entry_entering')
            return Motion(self.SPEED_MPS, max(-.1, min(.1, 1.5*heading_error)), 'docking_entry_entering')
        return Motion(0., 0., 'docking_entry_'+self.state)

    def captured(self, odom, now, guide):
        """처음부터 끝까지 카메라만 보고 들어간다(클래스 설명 참고). guide: (forward, left, yaw) 또는 None."""
        self.travel += math.hypot(odom[0]-self.last_odom[0], odom[1]-self.last_odom[1])
        self.last_odom = odom
        if guide is not None:
            self.heard = now
        turn = wanted = 0.
        if guide is not None:
            sideways = max(-self.CAPTURE_LEFT_LIMIT, min(self.CAPTURE_LEFT_LIMIT, self.K_LEFT*guide[1]))
            turn = -(self.K_YAW*guide[2]+sideways)
            wanted = abs(turn)/self.K_YAW  # 자리의 선으로 이어지는 방향에서 얼마나 벗어났는지
        if self.state == 'entering':
            if now-self.started > self.CAPTURE_TIME_S or self.travel > self.CAPTURE_TRAVEL_M or now-self.heard > self.CAPTURE_BLIND_S:
                self.state = 'fault'
            elif guide is not None and guide[0] >= -self.GUIDE_STOP_M:
                self.state = 'complete'  # 카메라가 자리 위에 있다고 본다
        if self.state == 'entering' and guide is not None:
            self.facing = wanted > (self.CAPTURE_FACED_RAD if self.facing else self.CAPTURE_FACE_RAD)
        self.message = {'entering': ('5번 진입 · 천장 캠이 안 보여 서서 기다림' if guide is None else
                                     '5번 진입 · 천장 캠을 보며 제자리에서 방향 맞춤' if self.facing else
                                     f'5번 진입 · 천장 캠을 보며 들어감 (남은 거리 {max(0., -guide[0])*100:.0f}cm)'),
                        'complete': '5번 진입 완료 · 정지 (천장 캠이 본 자리)',
                        'fault': '5번 진입 중단 · 천장 캠이 오래 안 보였거나 시간/거리 제한을 넘음'}[self.state]
        if self.state != 'entering':
            return Motion(0., 0., 'docking_entry_'+self.state)
        if guide is None:
            return Motion(0., 0., 'docking_entry_entering')
        if self.facing:
            rate = max(self.CAPTURE_TURN_MIN, min(self.CAPTURE_TURN_LIMIT, abs(turn)))
            return Motion(0., math.copysign(rate, turn), 'docking_entry_entering')
        return Motion(self.SPEED_MPS, max(-self.GUIDE_TURN_LIMIT, min(self.GUIDE_TURN_LIMIT, turn)), 'docking_entry_entering')

    def status(self):
        return dict(key=self.key, state=self.state, progress_m=self.progress, target_m=self.target, message=self.message,
                    guided=self.guided, guide=list(self.guide) if self.guide is not None else None,
                    capture=self.capture, travel_m=self.travel, squaring=self.squaring)


class BackOff:
    """앞 물체에 너무 가까울 때(초음파) 서 있기만 하지 않는다. hold_s 동안 서 있었으면 초음파가 clear_m을 읽거나
    distance_m을 갈 때까지 곧게 후진해, Nav2가 장애물을 도는 경로(퓨전 costmap에 이제 들어 있음)로 돌 공간을
    만든다. LiDAR가 뒤가 비었다고 볼 때만 하며, 연속 `tries`번까지 한 뒤에는 기다린다(앞이 reset_s 동안 비면
    횟수 초기화). 2026-11-17. 기본값: 12 cm, 15 cm, 1초, 4 cm/s, 3번, 6초, 10초."""

    def __init__(self, distance_m=.12, clear_m=.15, hold_s=1.0, speed_mps=.04, tries=3, timeout_s=6., reset_s=10.):
        self.distance_m, self.clear_m, self.hold_s = distance_m, clear_m, hold_s
        self.speed_mps, self.tries, self.timeout_s, self.reset_s = speed_mps, tries, timeout_s, reset_s
        self.blocked_since = None
        self.clear_since = None
        self.origin = None
        self.started = 0.
        self.count = 0

    @property
    def active(self):
        return self.origin is not None

    def update(self, blocked, range_m, odom, now, rear_clear):
        """물러나는 중이면 후진 Motion, 아니면 None(호출한 쪽이 예전대로 서 있거나 달린다)."""
        if self.active:
            done = (odom is None or not rear_clear or now-self.started > self.timeout_s
                    or distance(odom, self.origin) >= self.distance_m
                    or (range_m is not None and range_m >= self.clear_m))
            if done:
                self.origin, self.blocked_since = None, None
                self.count += 1
                return None
            return Motion(-self.speed_mps, 0., 'ultrasonic_back_off')
        if not blocked:
            self.blocked_since = None
            if self.clear_since is None:
                self.clear_since = now
            if now-self.clear_since >= self.reset_s:
                self.count = 0
            return None
        self.clear_since = None
        if self.blocked_since is None:
            self.blocked_since = now
        if (now-self.blocked_since >= self.hold_s and self.count < self.tries and odom is not None and rear_clear
                and self.distance_m > 0):
            self.origin, self.started = odom, now
            return Motion(-self.speed_mps, 0., 'ultrasonic_back_off')
        return None


class GiveWay:
    """관제의 지시로 다른 핑키에게 길을 비킨다(scripts/yielding.py, 2026-11-24): 지시한 거리까지만 곧게 후진한 뒤
    지시가 끊길 때까지 서 있는다. 후진은 LiDAR가 뒤가 비었다고 볼 때만, timeout_s를 넘지 않게 한다. 후진할 수
    없으면 그냥 서 있는데, 그것도 상대 핑키에게 필요한 것(가만히 있는 장애물)이다. 지시 하나에 한 번만
    물러나며, 같은 id가 다시 오면 "계속 서 있으라"는 뜻이다. 기본 5 cm/s, 최대 0.40 m, 10초."""

    def __init__(self, speed_mps=.05, limit_m=.40, timeout_s=10.):
        self.speed_mps, self.limit_m, self.timeout_s = speed_mps, limit_m, timeout_s
        self.order_id = None
        self.origin = None      # 후진을 시작한 오도메트리 위치. 후진이 끝나면 None
        self.started = 0.
        self.backed_m = 0.

    @property
    def active(self):
        return self.order_id is not None

    def update(self, order, odom, now, rear_clear):
        """order: 관제가 양보하라는 동안 {'id', 'back_m'}, 아니면 None. 후진 또는 정지 Motion을 돌려주고,
        지시가 없으면 None(호출한 쪽이 예전대로 달린다)."""
        if not isinstance(order, dict) or not isinstance(order.get('id'), str):
            self.order_id = self.origin = None
            return None
        if order['id'] != self.order_id:
            self.order_id, self.origin, self.started, self.backed_m = order['id'], odom, now, 0.
        if self.origin is not None:
            try:
                wanted = min(self.limit_m, max(0., float(order.get('back_m', 0.))))
            except (TypeError, ValueError):
                wanted = 0.
            self.backed_m = distance(odom, self.origin) if odom is not None else self.backed_m
            if (odom is None or not rear_clear or now-self.started > self.timeout_s or self.backed_m >= wanted):
                self.origin = None
            else:
                return Motion(-self.speed_mps, 0., 'yield_back')
        return Motion(0., 0., 'yield_wait')

    def status(self):
        return dict(order=self.order_id, backing=self.origin is not None, backed_m=round(self.backed_m, 3))
