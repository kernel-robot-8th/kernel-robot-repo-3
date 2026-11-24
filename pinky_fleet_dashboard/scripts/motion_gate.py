"""공용 차체 기하와 최종 속도 게이트(행동 선택과 독립).

실행 위치: PC·로봇 양쪽. 좌표는 로봇 기준(x 앞·y 왼쪽, m·rad), 차체는 중심 기준 직사각형.
"""
import math


def geometry(config=None):
    """robot_geometry.json을 기본값에 덮어쓰고 범위 검사(ValueError). 초음파 시야 0.35 rad는 실측 아님."""
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
    """다음 1초 (linear, angular) 궤적에서 차체가 points에 안 닿으면 True. reach_m이 있으면 그 거리까지만
    본다(guard_front_reach_m, 2026-11-16부터 0.05 m)."""
    g = geometry(config)
    length, width = g['half_length_m']+g['padding_m'],g['half_width_m']+g['padding_m']
    horizon = 1.
    if reach_m is not None and abs(linear) > 1e-6:
        horizon = min(1., reach_m/abs(linear))
    for step in range(21):
        t=horizon*step/20;theta=angular*t
        x,y=(linear*t,0.) if abs(angular)<1e-6 else (linear*math.sin(theta)/angular,linear*(1-math.cos(theta))/angular)
        c,s=math.cos(theta),math.sin(theta)
        if any(abs(c*(px-x)+s*(py-y))<=length and abs(-s*(px-x)+c*(py-y))<=width for px,py in points):return False
    return True


def range_arc(distance, config):
    """초음파 거리 하나를 시야각 호 위 점 9개(로봇 좌표)로 바꾼다."""
    g=geometry(config)
    return [(g['ultrasound_x_m']+distance*math.cos(g['ultrasound_yaw_rad']+g['ultrasound_fov_rad']*(i/8-.5)),
             g['ultrasound_y_m']+distance*math.sin(g['ultrasound_yaw_rad']+g['ultrasound_fov_rad']*(i/8-.5))) for i in range(9)]


class LowObstacleMemory:
    """라이다가 못 보는 낮은 장애물을 초음파가 본 odom 자리로 기억한다."""
    # 기본 90 s / 25 cm. 2026-11-17 기억점이 지나간 핑키 뒤로 1분 정지시켜 navigation.json에서 6 s / 12 cm로 둔다.
    def __init__(self, config=None, keep_s=90., mark_m=.25):
        self.config=geometry(config);self.points=[];self.keep_s=keep_s;self.mark_m=mark_m
    def update(self, pose, reading, now):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);g=self.config
        sx=x+c*g['ultrasound_x_m']-s*g['ultrasound_y_m'];sy=y+s*g['ultrasound_x_m']+c*g['ultrasound_y_m']
        # 시야 안에서 측정값보다 2 cm 넘게 가까운 기억점은 지운다(측정 없음 = 0.5 m).
        limit=.5 if reading is None else reading
        retained=[]
        for px,py,at in self.points:
            bearing=math.atan2(py-sy,px-sx)-yaw-g['ultrasound_yaw_rad']
            bearing=math.atan2(math.sin(bearing),math.cos(bearing))
            cleared=abs(bearing)<=g['ultrasound_fov_rad']/2+1e-6 and math.hypot(px-sx,py-sy)<limit-.02
            if now-at<self.keep_s and not cleared:retained.append((px,py,at))
        self.points=retained
        if reading is not None and math.isfinite(reading) and 0 < reading <= self.mark_m:
            for bx,by in range_arc(reading,g):
                px,py=x+c*bx-s*by,y+s*bx+c*by
                if not any(math.hypot(px-qx,py-qy)<.008 for qx,qy,_ in self.points):self.points.append((px,py,now))
        self.points=self.points[-400:]
    def local(self,pose):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        return [(c*(px-x)+s*(py-y),-s*(px-x)+c*(py-y)) for px,py,_ in self.points]


def gate(linear, angular, *, fresh, stop, context_valid, points, config=None, reach_m=None):
    """/cmd_vel 직전 마지막 검사(20 Hz). 순서가 우선순위: 숫자 오류 → 정지 → 허가 없음 → 오래됨 → 속도 상한 → 충돌."""
    if not all(math.isfinite(v) for v in (linear,angular)):return 0.,0.,'invalid_command'
    if stop:return 0.,0.,'operator_stopped'
    if not context_valid:return 0.,0.,'route_or_traffic_wait'
    if not fresh:return 0.,0.,'sensor_or_command_stale'
    linear=max(-.12,min(.12,linear));angular=max(-.30,min(.30,angular))
    if not swept_footprint(points,linear,angular,config,reach_m):return 0.,0.,'collision_stop'
    return linear,angular,'allowed'


def detour_clearance(points,direction,config,pass_distance):
    """우회 동작 전체(바퀴 모서리 회전 포함)의 장애물 여유 평균(m), 닿으면 -1. direction +1 왼쪽, -1 오른쪽."""
    g=geometry(config);theta=direction*math.pi/3
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
