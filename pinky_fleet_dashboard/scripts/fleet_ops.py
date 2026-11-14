"""Present current readiness independently of the last successful preparation."""
REQUIRED=('map','tf_pose','scan','odom','ultrasonic','ir','line_control','safety_status')

def operational_status(robot,topics):
    missing=[k for k in REQUIRED if not topics.get(k,{}).get('fresh')]
    control=topics.get('line_control',{}).get('value') or {}
    final=topics.get('safety_status',{}).get('value') or {}
    updated=control.get('dock_protocol')=='sponge_v1' and control.get('line_algorithm')=='fleet_control_v4' and final.get('version')=='final_gate_v1'
    state='ready';label='현재 주행 가능'
    if robot.get('stage')=='fault':state,label='fault','준비 오류'
    elif not robot.get('ssh_reachable'):state,label='disconnected','연결 끊김'
    elif robot.get('stage')!='prepared':state,label='unprepared','주행준비 필요'
    elif missing:state,label='stale','센서·위치 수신 대기'
    # A freshly started AMCL may publish a default pose (the map origin); the map pose counts only once LiDAR
    # localization or the operator set it after the latest Nav2 start (app.py records both times).
    elif robot.get('nav_started_at') and (robot.get('localized_at') or 0)<robot['nav_started_at']:state,label='unlocalized','위치 확인 필요'
    elif not updated:state,label='update_required','새 감시기 적용 필요'
    return dict(state=state,label=label,ready_for_motion=state=='ready',missing=missing,
                last_preparation_ok=bool(robot.get('last_prepared_at') or robot.get('previously_prepared') or robot.get('stage')=='prepared'),runtime_updated=updated)


def indicator(robot,route,operational):
    """The lamp colour for a Pinky (scripts/status_lamp.py shows it): red for a stop being applied or a fault,
    green while a route runs, blue when ready and waiting, orange while connecting, preparing or without a pose.
    The robot itself blinks orange when this stops arriving."""
    route=route or {}
    if route.get('stop_pending') or robot.get('stage') in ('fault','stopping'):return dict(color='red',mode='on')
    if route.get('status') in ('starting','running','cancel_requested'):return dict(color='green',mode='on')
    if operational.get('ready_for_motion'):return dict(color='blue',mode='on')
    return dict(color='orange',mode='on')
