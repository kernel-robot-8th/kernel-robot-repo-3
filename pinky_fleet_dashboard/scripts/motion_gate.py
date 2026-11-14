"""Shared geometry and final velocity gate, independent of behavior selection."""
import math


def geometry(config=None):
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


def swept_footprint(points, linear, angular, config=None):
    g = geometry(config)
    length, width = g['half_length_m']+g['padding_m'],g['half_width_m']+g['padding_m']
    for step in range(21):
        t=step/20;theta=angular*t
        x,y=(linear*t,0.) if abs(angular)<1e-6 else (linear*math.sin(theta)/angular,linear*(1-math.cos(theta))/angular)
        c,s=math.cos(theta),math.sin(theta)
        if any(abs(c*(px-x)+s*(py-y))<=length and abs(-s*(px-x)+c*(py-y))<=width for px,py in points):return False
    return True


def range_arc(distance, config):
    g=geometry(config)
    return [(g['ultrasound_x_m']+distance*math.cos(g['ultrasound_yaw_rad']+g['ultrasound_fov_rad']*(i/8-.5)),
             g['ultrasound_y_m']+distance*math.sin(g['ultrasound_yaw_rad']+g['ultrasound_fov_rad']*(i/8-.5))) for i in range(9)]


class LowObstacleMemory:
    def __init__(self, config=None):self.config=geometry(config);self.points=[]
    def update(self, pose, reading, now):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);g=self.config
        sx=x+c*g['ultrasound_x_m']-s*g['ultrasound_y_m'];sy=y+s*g['ultrasound_x_m']+c*g['ultrasound_y_m']
        limit=.5 if reading is None else reading
        retained=[]
        for px,py,at in self.points:
            bearing=math.atan2(py-sy,px-sx)-yaw-g['ultrasound_yaw_rad']
            bearing=math.atan2(math.sin(bearing),math.cos(bearing))
            cleared=abs(bearing)<=g['ultrasound_fov_rad']/2+1e-6 and math.hypot(px-sx,py-sy)<limit-.02
            if now-at<90 and not cleared:retained.append((px,py,at))
        self.points=retained
        if reading is not None and math.isfinite(reading) and 0 < reading <= .25:
            for bx,by in range_arc(reading,g):
                px,py=x+c*bx-s*by,y+s*bx+c*by
                if not any(math.hypot(px-qx,py-qy)<.008 for qx,qy,_ in self.points):self.points.append((px,py,now))
        self.points=self.points[-400:]
    def local(self,pose):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        return [(c*(px-x)+s*(py-y),-s*(px-x)+c*(py-y)) for px,py,_ in self.points]


def gate(linear, angular, *, fresh, stop, context_valid, points, config=None):
    if not all(math.isfinite(v) for v in (linear,angular)):return 0.,0.,'invalid_command'
    if stop:return 0.,0.,'operator_stopped'
    if not context_valid:return 0.,0.,'route_or_traffic_wait'
    if not fresh:return 0.,0.,'sensor_or_command_stale'
    linear=max(-.12,min(.12,linear));angular=max(-.30,min(.30,angular))
    if not swept_footprint(points,linear,angular,config):return 0.,0.,'collision_stop'
    return linear,angular,'allowed'


def detour_clearance(points,direction,config,pass_distance):
    """Score the entire outbound maneuver, including rotating wheel corners."""
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
