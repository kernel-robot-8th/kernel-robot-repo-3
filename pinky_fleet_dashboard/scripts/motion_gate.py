"""공용 차체 기하와 최종 속도 게이트. 행동 선택과는 독립이다.

실행 위치: PC·로봇 양쪽 (로봇에는 주행준비 때 runtime_bundle로 올라감)
사용처: 로봇 velocity_guard.py(gate, LowObstacleMemory), ultrasonic_safety.py(swept_footprint),
mission_behaviors.py(range_arc, detour_clearance), PC app.py(geometry로 robot_geometry.json 검증).

좌표는 로봇 기준(x 앞·y 왼쪽, m·rad)이고 차체는 중심 기준 반치수의 직사각형이다.
"""
import math


def geometry(config=None):
    """robot_geometry.json 값을 기본값 위에 덮어쓰고 범위를 검사한다(틀리면 ValueError). 초음파 시야 0.35 rad는 실측값이 아니다."""
    result = dict(half_length_m=.07, half_width_m=.07, padding_m=0.,
                  ultrasound_x_m=0., ultrasound_y_m=0., ultrasound_yaw_rad=0.,
                  ultrasound_fov_rad=.35, sensor_pose_measured=False, detour_offset_m=.10)
    result.update(config or {})
    for key in result:
        if key == 'sensor_pose_measured':
            if not isinstance(result[key], bool):raise ValueError('sensor_pose_measured must be boolean')
        elif not isinstance(result[key], (float,int)) or not math.isfinite(result[key]):
            raise ValueError('Invalid geometry: '+key)
    if not .05 <= result['half_length_m'] <= .3 or not .05 <= result['half_width_m'] <= .3:
        raise ValueError('Footprint must include wheels')
    if not 0 <= result['padding_m'] <= .05 or not .01 <= result['ultrasound_fov_rad'] <= math.pi:
        raise ValueError('Invalid margin/FOV')
    if abs(result['ultrasound_x_m'])>.3 or abs(result['ultrasound_y_m'])>.3:
        raise ValueError('Invalid sensor mounting offset')
    if not .08 <= result['detour_offset_m'] <= .3:raise ValueError('Invalid detour offset')
    return result


def swept_footprint(points, linear, angular, config=None, reach_m=None):
    """(linear, angular)로 다음 1초를 움직이는 동안 차체가 `points`에 닿지 않으면 True. reach_m이 있으면
    차체가 그만큼 움직인 지점에서 미리보기를 멈춘다. 그래서 1초 주행 거리 밖이 아니라 장애물에 그만큼 가까이
    가서야 멈춘다(navigation.json guard_front_reach_m, 2026-11-16부터 0.05 m)."""
    g = geometry(config)
    length, width = g['half_length_m']+g['padding_m'],g['half_width_m']+g['padding_m']
    horizon = 1.
    if reach_m is not None and abs(linear) > 1e-6:
        horizon = min(1., reach_m/abs(linear))
    # 등속 원호 궤적의 자세 21개마다, 점을 차체 좌표로 돌려 반길이·반폭 안에 들면 충돌이다.
    for step in range(21):
        t=horizon*step/20;theta=angular*t
        x,y=(linear*t,0.) if abs(angular)<1e-6 else (linear*math.sin(theta)/angular,linear*(1-math.cos(theta))/angular)
        c,s=math.cos(theta),math.sin(theta)
        if any(abs(c*(px-x)+s*(py-y))<=length and abs(-s*(px-x)+c*(py-y))<=width for px,py in points):return False
    return True


def range_arc(distance, config):
    """초음파 거리값 하나를 시야각에 걸친 호 위의 점 9개(로봇 좌표)로 바꾼다. 물체가 시야 어디인지 모르니 전부 찍는다."""
    g=geometry(config)
    return [(g['ultrasound_x_m']+distance*math.cos(g['ultrasound_yaw_rad']+g['ultrasound_fov_rad']*(i/8-.5)),
             g['ultrasound_y_m']+distance*math.sin(g['ultrasound_yaw_rad']+g['ultrasound_fov_rad']*(i/8-.5))) for i in range(9)]


class LowObstacleMemory:
    """라이다가 못 보는 낮은 장애물을 초음파가 본 자리(odom 좌표)로 기억한다. 시야를 벗어나도 계속 피하게 한다."""
    # keep_s / mark_m: 초음파가 본 낮은 장애물을 얼마나 오래 기억하는지, 얼마나 가까워야 기억하는지
    # (기본 90 s / 25 cm). 2026-11-17에 다른 핑키가 앞을 지나간 뒤 라이다가 아니라 이 기억된 점들이 핑키를
    # 1분 동안 세워 두었다. 그래서 navigation.json에서 6 s / 12 cm로 정한다.
    def __init__(self, config=None, keep_s=90., mark_m=.25):
        self.config=geometry(config);self.points=[];self.keep_s=keep_s;self.mark_m=mark_m
    def update(self, pose, reading, now):
        # pose: odom (x, y, yaw), reading: 초음파 거리(m) 또는 None. (sx, sy)는 odom 좌표의 센서 위치다.
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);g=self.config
        sx=x+c*g['ultrasound_x_m']-s*g['ultrasound_y_m'];sy=y+s*g['ultrasound_x_m']+c*g['ultrasound_y_m']
        # 시야 안에서 지금 측정값보다 2 cm 넘게 가까운 기억점은 이제 비었으니 지운다(측정 없음 = 0.5 m까지 빔).
        limit=.5 if reading is None else reading
        retained=[]
        for px,py,at in self.points:
            bearing=math.atan2(py-sy,px-sx)-yaw-g['ultrasound_yaw_rad']
            bearing=math.atan2(math.sin(bearing),math.cos(bearing))
            cleared=abs(bearing)<=g['ultrasound_fov_rad']/2+1e-6 and math.hypot(px-sx,py-sy)<limit-.02
            if now-at<self.keep_s and not cleared:retained.append((px,py,at))
        self.points=retained
        # mark_m 이내 반사만 기억하고, 기존 점과 8 mm 안이면 중복이라 넣지 않는다.
        if reading is not None and math.isfinite(reading) and 0 < reading <= self.mark_m:
            for bx,by in range_arc(reading,g):
                px,py=x+c*bx-s*by,y+s*bx+c*by
                if not any(math.hypot(px-qx,py-qy)<.008 for qx,qy,_ in self.points):self.points.append((px,py,now))
        # 검사 시간을 묶어 두려고 최근 400개만 남긴다.
        self.points=self.points[-400:]
    # 기억점을 현재 자세 기준 로봇 좌표로 돌려준다.
    def local(self,pose):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        return [(c*(px-x)+s*(py-y),-s*(px-x)+c*(py-y)) for px,py,_ in self.points]


def gate(linear, angular, *, fresh, stop, context_valid, points, config=None, reach_m=None):
    """/cmd_vel 직전 마지막 검사(velocity_guard가 20 Hz로 부름). (linear, angular, 이유)를 돌려준다.

    순서가 우선순위다: 비정상 숫자 → 작업자 정지 → 경로·구역 허가 없음 → 센서·명령 오래됨 → 속도 상한 → 궤적 충돌.
    """
    if not all(math.isfinite(v) for v in (linear,angular)):return 0.,0.,'invalid_command'
    if stop:return 0.,0.,'operator_stopped'
    if not context_valid:return 0.,0.,'route_or_traffic_wait'
    if not fresh:return 0.,0.,'sensor_or_command_stale'
    # 위 단계가 무엇을 보내도 0.12 m/s, 0.30 rad/s를 넘지 않는다.
    linear=max(-.12,min(.12,linear));angular=max(-.30,min(.30,angular))
    if not swept_footprint(points,linear,angular,config,reach_m):return 0.,0.,'collision_stop'
    return linear,angular,'allowed'


def detour_clearance(points,direction,config,pass_distance):
    """나가는 우회 동작 전체를, 회전하는 바퀴 모서리까지 포함해 점수 매긴다.

    direction +1은 왼쪽, -1은 오른쪽. 60도 회전 → 측면 offset까지 이동 → 방향 복원 → pass_distance까지 직진하는
    자세 81개의 장애물 여유(m) 평균이고, 한 번이라도 닿으면 -1이다. UltrasonicDetour가 큰 쪽을 고른다.
    """
    g=geometry(config);theta=direction*math.pi/3
    # 측면 폭은 설정값과 반폭+3 cm 중 큰 값. 60도로 그만큼 옆으로 가면 앞으로는 offset/tan(60°) 간다.
    offset=max(g['detour_offset_m'],g['half_width_m']+.03)
    x,y=offset/math.tan(math.pi/3),direction*offset
    poses=[]
    for i in range(21):poses.append((0.,0.,theta*i/20))
    for i in range(1,21):poses.append((x*i/20,y*i/20,theta))
    for i in range(1,21):poses.append((x,y,theta*(1-i/20)))
    for i in range(1,21):poses.append((x+(pass_distance-x)*i/20,y,0.))
    score=0.
    for px,py,yaw in poses:
        local_score=1.
        c,s=math.cos(yaw),math.sin(yaw)
        for ox,oy in points:
            dx,dy=c*(ox-px)+s*(oy-py),-s*(ox-px)+c*(oy-py)
            clearance=max(abs(dx)-g['half_length_m']-g['padding_m'],abs(dy)-g['half_width_m']-g['padding_m'])
            if clearance<=0:return -1.
            local_score=min(local_score,clearance)
        score+=local_score
    return score/len(poses)
