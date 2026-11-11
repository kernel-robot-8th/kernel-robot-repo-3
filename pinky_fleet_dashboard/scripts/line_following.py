"""Calibrated IR steering assistance with a LiDAR footprint check; no ROS imports."""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import Any

MAX_LINEAR = 0.1152
MAX_ANGULAR = 0.30
MIN_CONTRAST = 100.0
CENTER_ON = 0.55
CENTER_OFF = 0.35
SIDE_ON = 0.25
SIDE_OFF = 0.18


def validate_template(model):
    try:
        rows = model['rows']
        if not 3 <= len(rows) <= 1000:
            return '횡방향 보정 지점이 부족합니다.'
        offsets = [float(row['offset_cm']) for row in rows]
        if any(not math.isfinite(v) or abs(v) > 10 for v in offsets):
            return '횡방향 보정 범위는 ±10cm 이내여야 합니다.'
        if offsets != sorted(set(offsets)) or not offsets[0] < 0 < offsets[-1]:
            return '횡방향 보정은 중복 없는 음수·0·양수 지점이 필요합니다.'
        if 0. not in offsets or model.get('positive_offset') != 'line_right':
            return '보정 중앙 또는 좌우 부호 정보가 없습니다.'
        for row in rows:
            if not math.isfinite(float(row['offset_cm'])) or len(row['mean']) != 3 or any(
                    not math.isfinite(float(v)) or not 0 <= float(v) <= 4095 for v in row['mean']):
                return '횡방향 보정값이 유효하지 않습니다.'
        if any(max(row['mean'][i] for row in rows)-min(row['mean'][i] for row in rows) < 100 for i in range(3)):
            return '횡방향 보정의 채널 대비가 부족합니다.'
    except (KeyError, TypeError, ValueError):
        return '횡방향 보정 모델이 유효하지 않습니다.'
    return None


class TemplateEstimator:
    """Match a measured sweep; interpolation resolution is not accuracy."""
    def __init__(self, model):
        reason = validate_template(model)
        if reason:
            raise ValueError(reason)
        rows = model['rows']
        self.scale = [max(r['mean'][i] for r in rows)-min(r['mean'][i] for r in rows) for i in range(3)]
        self.grid = []
        lo, hi = rows[0]['offset_cm'], rows[-1]['offset_cm']
        count = int(round((hi-lo)/.1))
        j = 0
        for k in range(count+1):
            offset = lo+(hi-lo)*k/count
            while j+1 < len(rows)-1 and rows[j+1]['offset_cm'] < offset:
                j += 1
            a, b = rows[j], rows[j+1]
            t = (offset-a['offset_cm'])/(b['offset_cm']-a['offset_cm'])
            self.grid.append((offset, [a['mean'][i]+t*(b['mean'][i]-a['mean'][i]) for i in range(3)]))
        self.zero = next(r['mean'] for r in rows if r['offset_cm'] == 0)

    def estimate(self, raw):
        rms = lambda template: math.sqrt(sum(((template[i]-raw[i])/self.scale[i])**2 for i in range(3))/3)
        fits = [(rms(template), offset) for offset, template in self.grid]
        best, offset = min(fits)
        candidates = sorted(off for error, off in fits if error <= best+.025)
        lower, upper = candidates[0], candidates[-1]
        reason = 'matched'
        # Saturated side channels do not reveal sub-mm position in this band.
        if rms(self.zero) <= .035:
            offset, lower, upper, reason = 0., -.5, .5, 'center_band'
        elif best > .12:
            reason = 'pattern_mismatch'
        elif upper-lower > .8 or any(b-a > .3 for a, b in zip(candidates, candidates[1:])):
            reason = 'ambiguous'
        elif abs(offset-self.grid[0][0]) < .01 or abs(offset-self.grid[-1][0]) < .01:
            reason = 'range_edge'
        usable = reason in ('matched', 'center_band')
        return dict(offset_cm=offset, lower_cm=lower, upper_cm=upper, fit_rms=best,
                    confidence=max(0., 1-best/.12) if usable else 0., usable=usable, reason=reason)


def default_profile() -> dict[str, Any]:
    return dict(enabled=False, floor=[None]*3, line=[None]*3,
                floor_noise=[0.0]*3, line_noise=[0.0]*3,
                channel_order=[0, 1, 2], mapping_confirmed=True, revision='unconfigured',
                mode='calibrated', geometry=dict(sensor_pitch_m=.01, sensor_span_m=.02,
                    white_width_m=.02, black_left_width_m=.02, black_right_width_m=.02))


def validate_profile(profile: dict[str, Any]) -> str | None:
    """Return why calibration cannot be used, or None for a valid profile."""
    try:
        if sorted(profile['channel_order']) != [0, 1, 2] or not profile.get('mapping_confirmed'):
            return '채널 배치를 확인하세요.'
        if profile.get('mode') == 'template':
            return validate_template(profile.get('template', {}))
        if profile.get('mode', 'calibrated') != 'calibrated':
            return '고정 기준은 종료됐습니다. 바닥·선 실측값을 다시 저장하세요.'
        for key in ('floor', 'line', 'floor_noise', 'line_noise'):
            if len(profile[key]) != 3:
                return '세 채널의 보정값이 필요합니다.'
        for i, (floor, line) in enumerate(zip(profile['floor'], profile['line'])):
            if not all(isinstance(v, (int, float)) and math.isfinite(v) and 0 <= v <= 4095 for v in (floor, line)):
                return f'CH{i+1} 바닥·선값을 저장하세요.'
            noise = max(float(profile['floor_noise'][i]), float(profile['line_noise'][i]))
            if not math.isfinite(noise) or noise < 0:
                return '유효하지 않은 보정 잡음값입니다.'
            if abs(floor-line) < max(MIN_CONTRAST, 6*noise):
                return f'CH{i+1} 바닥·선 대비가 작거나 신호가 불안정합니다.'
    except (KeyError, TypeError, ValueError):
        return '유효하지 않은 IR 보정 파일입니다.'
    return None


def capture_profile(profile: dict[str, Any], samples: list[list[int]], surface: str, channel: int | None = None) -> dict[str, Any]:
    """Use a median and robust noise estimate; require multiple live samples."""
    if profile.get('enabled'):
        raise ValueError('IR 보조를 끈 뒤 보정하세요.')
    if len(samples) < 6 or any(len(row) != 3 for row in samples):
        raise ValueError('IR 실시간 샘플이 부족합니다. 위치를 유지하고 다시 누르세요.')
    if surface not in ('floor', 'line') or (surface == 'line' and channel not in (0, 1, 2)):
        raise ValueError('보정 종류와 채널을 확인하세요.')
    result = {**profile, **{key: list(profile[key]) for key in ('floor', 'line', 'floor_noise', 'line_noise')}}
    for i in range(3) if surface == 'floor' else [channel]:
        values = [float(row[i]) for row in samples]
        if not all(math.isfinite(v) and 0 <= v <= 4095 for v in values):
            raise ValueError('IR 원시값이 ADC 범위를 벗어났습니다.')
        middle = statistics.median(values)
        noise = 1.4826*statistics.median(abs(v-middle) for v in values)
        if noise > 100:
            raise ValueError(f'CH{i+1} 값이 불안정합니다. 센서를 고정한 뒤 다시 보정하세요.')
        result[surface][i], result[surface+'_noise'][i] = middle, noise
    return result


def surface_observation(raw, profile):
    """Three-surface membership from measured floor and polarity-aware endpoints.

    Legacy outer `line` entries above floor represent BLACK, not white.
    Until each channel has a white reference, use the measured low endpoint
    from the other channels and explicitly report that shared reference.
    """
    if len(raw)!=3 or validate_profile(profile): return None
    if any(not isinstance(v,(int,float)) or not math.isfinite(v) or not 0<=v<=4095 for v in raw): return None
    floor=profile['floor'];line=profile['line']
    lows=[v for v,f in zip(line,floor) if v<f-MIN_CONTRAST]
    highs=[v for v,f in zip(line,floor) if v>f+MIN_CONTRAST]
    if not lows:return None
    white_ref=[];black_ref=[];shared=[]
    for i,f in enumerate(floor):
        explicit=profile.get('white',[None]*3)[i]
        white=explicit if isinstance(explicit,(int,float)) and 0<=explicit<f-MIN_CONTRAST else line[i] if line[i]<f-MIN_CONTRAST else min(lows)
        explicit_black=profile.get('black',[None]*3)[i]
        black=explicit_black if isinstance(explicit_black,(int,float)) and f+MIN_CONTRAST<explicit_black<=4095 else line[i] if line[i]>f+MIN_CONTRAST else max(highs) if highs else 4095.
        white_ref.append(white);black_ref.append(black);shared.append(line[i]>=f-MIN_CONTRAST and explicit is None)
    white=[max(0.,min(1.,(f-v)/max(1.,f-w))) for v,f,w in zip(raw,floor,white_ref)]
    black=[max(0.,min(1.,(v-f)/max(1.,b-f))) for v,f,b in zip(raw,floor,black_ref)]
    surfaces=['white' if w>=.55 else 'black' if b>=.6 else 'floor' if max(w,b)<=.25 else 'uncertain' for w,b in zip(white,black)]
    order=profile['channel_order']
    return dict(white=[white[i] for i in order],black=[black[i] for i in order],
                surfaces=[surfaces[i] for i in order],white_reference_shared=[shared[i] for i in order],
                white_reference=[white_ref[i] for i in order],black_reference=[black_ref[i] for i in order])


def normalize(raw: list[int], profile: dict[str, Any]) -> list[float] | None:
    """White strength, with black borders distinguished from the white track."""
    if profile.get('mode')!='template':
        observation=surface_observation(raw,profile)
        return observation['white'] if observation else None
    if len(raw)!=3 or validate_profile(profile):return None
    if not all(isinstance(v,(int,float)) and math.isfinite(v) and 0<=v<=4095 for v in raw):return None
    rows=profile['template']['rows']
    floor=[max(r['mean'][i] for r in rows) for i in range(3)]
    line=[min(r['mean'][i] for r in rows) for i in range(3)]
    values=[max(0.,min(1.,(f-v)/(f-l))) for v,f,l in zip(raw,floor,line)]
    return [values[i] for i in profile['channel_order']]


def swept_clear(points: list[tuple[float, float]], linear: float, angular: float) -> bool:
    """Check a 14 cm square footprint along a 1 s candidate trajectory."""
    for step in range(11):
        t = step / 10
        yaw = angular*t
        if abs(angular) < 1e-6:
            x, y = linear*t, 0.0
        else:
            x = linear*math.sin(yaw)/angular
            y = linear*(1-math.cos(yaw))/angular
        c, s = math.cos(yaw), math.sin(yaw)
        for px, py in points:
            dx, dy = px-x, py-y
            if abs(c*dx+s*dy) <= .07 and abs(-s*dx+c*dy) <= .07:
                return False
    return True


@dataclass
class LineDecision:
    linear: float
    angular: float
    state: str
    normalized: list[float] | None = None
    error: float | None = None
    estimate: dict[str, Any] | None = None


class LineAssist:
    """Apply bounded PD only to forward commands on a visible, unblocked line."""
    def __init__(self) -> None:
        self.last_error = 0.0
        self.last_time: float | None = None
        self.center_locked = False
        self.side_locked = [False, False]
        self.started_at = None
        self.side_seen_at = None
        self.side_direction = 0.0
        self.last_steering = 0.0
        self.steering_at = None
        self.hold_yaw = None
        self.line_was_seen = False
        self.search_start = None
        self.search_yaw = None
        self.search_phase = 0
        self.model_revision = None
        self.model = None
        self.offset_filtered = None
        self.offset_at = None
        self.yaw_at = None
        self.previous_yaw = None
        self.yaw_rate = 0.
        self.center_since = None
        self.recovery_start = None
        self.recovery_yaw = None
        self.recovery_direction = 0.

    def reset(self) -> None:
        self.started_at = None
        self.side_seen_at = None
        self.side_direction = 0.0
        self.last_steering = 0.0
        self.steering_at = None
        self.hold_yaw = None
        self.line_was_seen = False
        self.search_start = None
        self.search_yaw = None
        self.search_phase = 0
        self.offset_filtered = None
        self.offset_at = None
        self.yaw_at = None
        self.previous_yaw = None
        self.yaw_rate = 0.
        self.center_since = None
        self.recovery_start = None
        self.recovery_yaw = None
        self.recovery_direction = 0.
        self._clear_tracking()

    def _clear_tracking(self) -> None:
        self.last_time = None
        self.last_error = 0.0
        self.center_locked = False
        self.side_locked = [False, False]

    def decide(self, linear: float, angular: float, raw: list[int], profile: dict[str, Any],
               now: float, ir_fresh: bool, scan_fresh: bool,
               scan_points: list[tuple[float, float]], route_corner: bool = False,
               heading_yaw: float | None = None) -> LineDecision:
        if not profile.get('enabled'):
            self.reset()
            return LineDecision(linear, angular, 'disabled')
        d = normalize(raw, profile)
        if d is None or not ir_fresh:
            self.reset()
            return LineDecision(0., 0., 'ir_unavailable', d)
        if not scan_fresh:
            self.reset()
            return LineDecision(0., 0., 'lidar_unavailable', d)
        if profile.get('mode') == 'template':
            return self._template_decide(linear, angular, raw, profile, d, now, scan_points, route_corner, heading_yaw)
        return self._calibrated_decide(linear, angular, raw, profile, d, now, scan_points, route_corner, heading_yaw)

    def _template_decide(self, linear, angular, raw, profile, d, now, points, corner, yaw):
        if self.model_revision != profile.get('revision') or self.model is None:
            self.model = TemplateEstimator(profile['template'])
            self.model_revision = profile.get('revision')
        estimate = self.model.estimate([raw[i] for i in profile['channel_order']])
        def result(v, w, state, error=None):
            if not swept_clear(points, v, w):
                v, w, state = 0., 0., 'lidar_stop'
            return LineDecision(v, w, state, d, error, estimate)
        if linear < -.001 or (abs(linear) <= .001 and abs(angular) <= .001):
            self.reset()
            return result(linear, angular, 'nav2_turn')
        blocked = any(px > 0 and abs(math.atan2(py, px)) <= math.pi/4 and math.hypot(px, py) < .25 for px, py in points)
        if corner or blocked:
            self.reset()
            return result(max(-.024, min(.024, linear)), max(-.2, min(.2, angular)),
                          'nav2_avoidance' if blocked else 'route_corner_gap')
        if yaw is None:
            return result(0., 0., 'heading_unavailable')
        if not estimate['usable']:
            self.offset_filtered, self.offset_at, self.center_since = None, None, None
            if not self.line_was_seen:
                return result(0., 0., 'template_unreliable')
            if self.search_start is None:
                self.search_start, self.search_yaw, self.search_phase = now, yaw, 0
            delta = math.atan2(math.sin(yaw-self.search_yaw), math.cos(yaw-self.search_yaw))
            direction = self.side_direction or 1.
            targets = [.20*direction, -.20*direction, 0.]
            if now-self.search_start > 6 or abs(delta) > .24 or self.search_phase >= 3:
                return result(0., 0., 'line_search_failed')
            if abs(targets[self.search_phase]-delta) < .025:
                self.search_phase += 1
            if self.search_phase >= 3:
                return result(0., 0., 'line_search_failed')
            return result(0., math.copysign(.16, targets[self.search_phase]-delta), 'line_search')
        self.line_was_seen = True
        self.search_start, self.search_phase = None, 0
        error = estimate['offset_cm']
        if abs(error) > .5:
            self.side_direction = -math.copysign(1., error)  # +offset means line RIGHT.
        dt = .05 if self.offset_at is None else max(.02, min(.2, now-self.offset_at))
        previous = error if self.offset_filtered is None else self.offset_filtered
        filtered = previous+(error-previous)*dt/(.15+dt)
        derivative = max(-4., min(4., (filtered-previous)/dt))
        self.offset_filtered, self.offset_at = filtered, now
        if self.previous_yaw is not None and self.yaw_at is not None and 0 < now-self.yaw_at < .3:
            rate = math.atan2(math.sin(yaw-self.previous_yaw), math.cos(yaw-self.previous_yaw))/(now-self.yaw_at)
            self.yaw_rate = .7*self.yaw_rate+.3*max(-1., min(1., rate))
        self.previous_yaw, self.yaw_at = yaw, now
        neutral = estimate['reason'] == 'center_band'
        if neutral:
            if self.center_since is None:
                self.center_since = now
            if now-self.center_since >= 1.:
                if self.hold_yaw is None:
                    self.hold_yaw = yaw
                difference = math.atan2(math.sin(yaw-self.hold_yaw), math.cos(yaw-self.hold_yaw))
                self.hold_yaw += min(1., dt/5.)*difference
        else:
            self.center_since = None
        # Reference yaw is learned only on stable centered runs. It never
        # competes with lateral feedback; angular-rate damping limits weave.
        target = max(-.22, min(.22, -.10*filtered-.025*derivative-.20*self.yaw_rate))
        if neutral and abs(filtered) < .25 and abs(self.yaw_rate) < .02:
            target = 0.
        step = .6*dt
        w = max(self.last_steering-step, min(self.last_steering+step, target))
        self.last_steering = w
        v = min(MAX_LINEAR, linear) if linear > .005 else .024
        v = min(v, MAX_LINEAR if neutral else .036)
        estimate.update(filtered_offset_cm=filtered, offset_rate_cm_s=derivative,
                        heading_reference_rad=self.hold_yaw, yaw_rate_rad_s=self.yaw_rate)
        return result(v, w, 'template_centered' if neutral else 'template_tracking', filtered)

    def _calibrated_decide(self, linear, angular, raw, profile, d, now, points, corner, heading_yaw):
        observation=surface_observation(raw,profile)
        def result(v,w,state,error=None):
            if not swept_clear(points,v,w):v,w,state=0.,0.,'lidar_stop'
            return LineDecision(v,w,state,d,error,observation)
        if linear<-.001 or (abs(linear)<=.001 and abs(angular)<=.001):
            self.reset();return result(linear,angular,'nav2_turn')
        blocked=any(px>0 and abs(math.atan2(py,px))<=math.pi/4 and math.hypot(px,py)<.25 for px,py in points)
        if blocked or corner:
            self.reset();return result(max(-.024,min(.024,linear)),max(-.2,min(.2,angular)),'nav2_avoidance' if blocked else 'route_corner_gap')
        if heading_yaw is None:return result(0.,0.,'heading_unavailable')
        if observation is None:return result(0.,0.,'ir_unavailable')
        surfaces=observation['surfaces'];black=observation['black']
        center=d[1]>=(CENTER_OFF if self.center_locked else CENTER_ON)
        self.center_locked=center
        left,right=d[0]>=.55,d[2]>=.55
        recovering=self.recovery_start is not None
        if center:
            self.line_was_seen=True
            if self.center_since is None:self.center_since=now
            # Release the recovery budget only after continuous centering;
            # one noisy white sample cannot restart an unlimited search.
            if now-self.center_since>=.25:
                if recovering:self.hold_yaw=heading_yaw
                self.recovery_start,self.recovery_yaw=None,None
                self.search_start,self.search_yaw,self.search_phase=None,None,0
            if self.hold_yaw is None:self.hold_yaw=heading_yaw
            dt=.05 if self.steering_at is None else max(.01,min(.1,now-self.steering_at))
            drift=0. if recovering else math.atan2(math.sin(heading_yaw-self.hold_yaw),math.cos(heading_yaw-self.hold_yaw))
            if not left and not right and now-self.center_since>=.25:
                self.hold_yaw+=min(1.,dt/3.)*drift
            error=(d[0]-d[2])/max(.1,sum(d))
            derivative=(error-self.last_error)/dt if self.last_time is not None else 0.
            target=max(-.12,min(.12,.18*error+.015*max(-2.,min(2.,derivative))-.6*drift))
            self.last_error,self.last_time=error,now
            if abs(error)<.05 and abs(drift)<.01:target=0.
            v=min(MAX_LINEAR,max(0.,linear)) if linear>.005 else .012
            if recovering:v=min(v,.012)
            if abs(error)>.15:v=min(v,.036)
            state='centered' if abs(error)<.05 else 'tracking'
        else:
            self.center_since=None
            # White on one side is the strongest direction cue; a black border
            # with gray floor on the other two channels is the next cue.
            direction=1. if left and not right else -1. if right and not left else 0.
            edge=(surfaces[0]=='black' and surfaces[1]=='floor' and surfaces[2]=='floor') or (surfaces[2]=='black' and surfaces[1]=='floor' and surfaces[0]=='floor')
            if not direction and edge:direction=1. if surfaces[0]=='black' else -1.
            if self.recovery_start is None:
                self.recovery_start,self.recovery_yaw=now,heading_yaw
                self.recovery_direction=direction or self.side_direction or 1.
                self.search_phase=0
            delta=math.atan2(math.sin(heading_yaw-self.recovery_yaw),math.cos(heading_yaw-self.recovery_yaw))
            if now-self.recovery_start>6. or abs(delta)>.40:
                return result(0.,0.,'line_search_failed')
            if direction:
                self.side_direction=direction;self.side_seen_at=now
                target=.16*direction
                v=.012;state='black_edge_reacquire' if edge and not (left or right) else 'white_side_reacquire'
                error=direction
            else:
                # No direction cue: pivot toward the remembered side, then scan
                # the other side. Never drive blindly on three gray readings.
                # Complete both sides within the six-second recovery budget:
                # the old +/- .35 rad sweep needed >8 s at .16 rad/s.
                targets=[.20*self.recovery_direction,-.20*self.recovery_direction,0.]
                if self.search_phase>=len(targets):return result(0.,0.,'line_search_failed')
                if abs(targets[self.search_phase]-delta)<.025:self.search_phase+=1
                if self.search_phase>=len(targets):return result(0.,0.,'line_search_failed')
                target=math.copysign(.16,targets[self.search_phase]-delta)
                v,state,error=0.,'line_search',delta
        dt=.05 if self.steering_at is None else max(.01,min(.1,now-self.steering_at))
        step=.6*dt
        w=max(self.last_steering-step,min(self.last_steering+step,target))
        self.last_steering,self.steering_at=w,now
        return result(v,w,state,error)
