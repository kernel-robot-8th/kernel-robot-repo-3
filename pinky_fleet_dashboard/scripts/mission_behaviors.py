"""Bounded junction traversal and an operator-gated docking state machine."""
from __future__ import annotations
from dataclasses import dataclass
import json
import math
import uuid
try:
    from .motion_gate import geometry, range_arc, detour_clearance
except ImportError:
    from motion_gate import geometry, range_arc, detour_clearance


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


class UltrasonicDetour:
    """Odometry-bounded detour; a single range sensor cannot measure object width."""
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
            if not self.low_points or min(distance(hit,p) for p in self.low_points)>.008:self.low_points.append(hit)
        self.low_points=self.low_points[-200:]

    def update(self, pose, range_m, points, now, clear_path, line_center=False, line_sample_at=None):
        if self.phase == 'fault':
            return Motion(0., 0., 'ultrasonic_fault')
        if now-self.started > 60. or distance(pose, self.origin) > .70:
            self.phase, self.failure = 'fault', 'detour_limit'
            return Motion(0., 0., 'ultrasonic_fault')
        blocked = range_m is not None and math.isfinite(range_m) and range_m <= .10
        if blocked:
            self.remember(pose, range_m)
        c, s = math.cos(pose[2]), math.sin(pose[2])
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
                if now-self.line_since>=.3 and self.line_count>=3:self.phase='idle'
            else:self.line_since=None;self.line_count=0
        if not clear_path(obstacles, v, w):
            return Motion(0., 0., 'ultrasonic_clearance_wait')
        return Motion(v, w, 'ultrasonic_'+self.phase)

    def status(self):
        return dict(phase=self.phase, failure=self.failure, lateral_target_m=getattr(self,'offset',self.config['detour_offset_m']),
                    footprint_width_m=2*self.config['half_width_m'], direction=self.direction)


class Docking:
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
        elif command.get('action') == 'received' and self.state == 'stopped' and self.anchored:
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
    """Reverse out of the sponge station using continuous wheel odometry.

    A restart during the maneuver fails closed; the origin is never rebased.
    The independent final guard retains rear and side collision checks.
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
    """Drive straight into station 5 from its square-up point on wheel odometry (2026-11-16).

    The station stands right in front there, so the ultrasonic and the LiDAR straight ahead are not looked at;
    the final guard keeps the LiDAR to both sides. The heading the Pinky squared up to (Nav2, within 3 degrees)
    is held. A restart during the maneuver fails closed, as DockExit does; drifting sideways 3 cm, turning
    more than 0.15 rad or taking over 25 s stops it as a fault.

    With the ceiling camera's guide (2026-11-18) it steers on what the camera sees while it drives in: `guide` is
    the Pinky's offset from its registered spot in its own axes, in metres and radians (forward, left, yaw; forward
    negative while it is still short of the spot), fresh from /fleet/dock_guide. It turns toward the spot's line and
    lines up on it (a second-order line follower: yaw rate = -(K_YAW * yaw + K_LEFT * left)), and stops where the
    camera says the spot is instead of after a counted distance. The camera only moves the stop a few centimetres
    either way of the counted one (GUIDE_SHORT_M before it, GUIDE_OVER_M past it). While no fresh guide arrives it drives
    as before, back on the heading it squared up to, so a hidden marker or a silent PC costs nothing.

    Taken over by the camera (context dock_capture, 2026-11-19): the route runner hands over as soon as the camera
    sees the Pinky in front of the spot, wherever Nav2 had brought it and whichever way it faces. Then there is no
    squared-up line to hold: it first turns on the spot until it faces the spot's line (the camera's yaw), then drives
    in steering as above, and stops where the camera sees the spot. Only the camera moves it: while the guide is
    silent it stands still (CAPTURE_BLIND_S at most), and it gives up after CAPTURE_TRAVEL_M or CAPTURE_TIME_S.

    Squaring up by itself (context dock_turn_rad, 2026-11-19): Nav2 kept hunting for minutes to stand on the
    square-up point to the centimetre and turn there (its collision check next to the station, its 3 cm tolerance).
    Now the runner hands over once the Pinky is near that point, with how far it still has to turn to the dock's
    heading; the Pinky turns that much on the spot on its odometry, then drives in as before."""
    SPEED_MPS = .03
    TAKEOVER_LEFT_M = .06          # farther beside the line than the straight run's steering can bring back
    SQUARE_DONE_RAD, SQUARE_TIME_S = .03, 20.        # its own turn to the dock's heading (context dock_turn_rad)
    CAPTURE_FACE_RAD, CAPTURE_FACED_RAD = .35, .12   # turn on the spot above the first, drive again below the second
    CAPTURE_TURN_LIMIT, CAPTURE_TURN_MIN = .5, .25   # rad/s on the spot; below the minimum the wheels only hum
    CAPTURE_LEFT_LIMIT = .75       # rad/s: up to about 24 degrees of crab, there is more room to come back
    CAPTURE_BLIND_S, CAPTURE_TIME_S, CAPTURE_TRAVEL_M = 8., 75., .70
    K_YAW, K_LEFT = 1.8, 24.       # 1/s and rad/s per metre: settles a 3 cm miss within about 15 cm of travel
    GUIDE_LEFT_LIMIT = .45         # rad/s the sideways term may ask for: at most about 14 degrees of crab
    GUIDE_TURN_LIMIT = .35         # rad/s
    GUIDE_SHORT_M, GUIDE_OVER_M = .08, .06
    GUIDE_STOP_M = .005            # stop this far short by the camera: what it shows is about 0.2 s old

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
        self.side = 0.  # how far left of the dock's line it stood at the hand-over (context dock_side_m)
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
            # The camera sees it turned away or well beside the line (2026-11-21: 93 degrees off after Nav2 had
            # "squared it up" on a map heading that was wrong). The straight run cannot mend that and used to stop
            # as a fault; the camera takes it over instead, turning it on the spot first.
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
            # Squared up: the straight run starts here, on the heading it was asked to reach.
            self.squaring = False
            self.origin, self.started = (odom[0], odom[1], angle(self.turn_from+self.turn)), now
            dx = dy = self.progress = lateral = 0.
            heading_error = angle(self.origin[2]-odom[2])
        # Handed over beside the dock's line (dock_side_m) and no camera: the same steering on odometry brings it onto
        # the line, as far as the wheels know where that is.
        if guide is None and abs(self.side) > .008:
            steer = (self.progress-self.target, self.side-dx*math.sin(self.origin[2])+dy*math.cos(self.origin[2]), -heading_error)
        else:
            steer = guide
        # Once it has been steered, the path is no longer the straight line it started on: wider limits.
        lateral_limit, heading_limit, time_limit = (.10, .50, 35.) if self.guided or steer is not None else (.03, .15, 25.)
        if self.state == 'entering':
            if lateral > lateral_limit or abs(heading_error) > heading_limit or now-self.started > time_limit:
                self.state = 'fault'
            elif guide is not None and guide[0] >= -self.GUIDE_STOP_M and self.progress >= self.target-self.GUIDE_SHORT_M:
                self.state = 'complete'  # the camera sees it on the spot
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
        """The whole way in on the camera (see the class text). guide: (forward, left, yaw) or None."""
        self.travel += math.hypot(odom[0]-self.last_odom[0], odom[1]-self.last_odom[1])
        self.last_odom = odom
        if guide is not None:
            self.heard = now
        turn = wanted = 0.
        if guide is not None:
            sideways = max(-self.CAPTURE_LEFT_LIMIT, min(self.CAPTURE_LEFT_LIMIT, self.K_LEFT*guide[1]))
            turn = -(self.K_YAW*guide[2]+sideways)
            wanted = abs(turn)/self.K_YAW  # how far it is from the heading that leads onto the spot's line
        if self.state == 'entering':
            if now-self.started > self.CAPTURE_TIME_S or self.travel > self.CAPTURE_TRAVEL_M or now-self.heard > self.CAPTURE_BLIND_S:
                self.state = 'fault'
            elif guide is not None and guide[0] >= -self.GUIDE_STOP_M:
                self.state = 'complete'  # the camera sees it on the spot
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
    """Too close to something in front (the ultrasonic): do not just stand there. After hold_s of standing, reverse
    straight until the ultrasonic reads clear_m or distance_m is covered, so Nav2 has room to turn onto the path
    round the obstacle (which the fused costmap now holds). Only while the LiDAR shows the way back clear; at most
    `tries` times in a row, then it waits (reset once the front has been clear for reset_s). 2026-11-17."""

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
        """A reversing Motion while backing off, else None (the caller holds or drives as before)."""
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
    """Give way to another Pinky on the dashboard's word (scripts/yielding.py, 2026-11-24): reverse straight, at most
    the distance the order names, then stand until the order stops coming. Reversing goes on only while the LiDAR
    shows the way back clear, and never longer than timeout_s; when it cannot reverse it simply stands, which is
    still what the other Pinky needs (an obstacle that keeps still). One order is backed off for once: the same id
    again means "keep standing"."""

    def __init__(self, speed_mps=.05, limit_m=.40, timeout_s=10.):
        self.speed_mps, self.limit_m, self.timeout_s = speed_mps, limit_m, timeout_s
        self.order_id = None
        self.origin = None      # where the reversing began (odom), None once it is over
        self.started = 0.
        self.backed_m = 0.

    @property
    def active(self):
        return self.order_id is not None

    def update(self, order, odom, now, rear_clear):
        """order: {'id', 'back_m'} while the dashboard says give way, else None. A Motion (reversing, or standing),
        or None when there is no order and the caller drives as before."""
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
