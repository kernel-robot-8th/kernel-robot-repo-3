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
    elif not updated:state,label='update_required','새 감시기 적용 필요'
    return dict(state=state,label=label,ready_for_motion=state=='ready',missing=missing,
                last_preparation_ok=bool(robot.get('last_prepared_at') or robot.get('previously_prepared') or robot.get('stage')=='prepared'),runtime_updated=updated)
