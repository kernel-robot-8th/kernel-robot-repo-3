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


# What a Pinky is doing, for its own screen (scripts/status_display.py): (Korean, English) per mission phase. The
# English one is shown when the robot has no Korean font. Places are called what they are for, not by their waypoint
# number (2026-11-24: "5번" said nothing to somebody standing next to the robot): 적재 장소, 수령 장소 1, 주차장.
PHASE_ACTIVITY = {
    'recovering': ('장애물 피하는 중', 'AVOIDING'),
    'docking': ('적재 장소 들어가는 중', 'DOCKING'),
    'dock_entry': ('적재 장소 들어가는 중', 'DOCKING'),
    'redocking': ('적재 장소 다시 맞추는 중', 'RE-DOCKING'),
    'docking_manual': ('적재 장소 도착', 'AT LOADING'),
    'docking_anchor': ('적재 장소 도착', 'AT LOADING'),
    'docked_manual': ('물건 싣는 중', 'LOADING'),
    'dock_exit': ('적재 장소에서 나가는 중', 'LEAVING'),
    'course_complete': ('수령 장소 도착', 'AT PICKUP'),
    'returning': ('주차장으로 복귀 중', 'GOING HOME'),
}
IDLE_ACTIVITY = {
    'ready': ('대기 중', 'READY'), 'fault': ('준비 오류', 'FAULT'), 'disconnected': ('연결 끊김', 'NO LINK'),
    'unprepared': ('주행준비 필요', 'NOT READY'), 'stale': ('센서 기다리는 중', 'NO SENSOR'),
    'unlocalized': ('위치 확인 필요', 'NO POSE'), 'update_required': ('주행준비 다시 필요', 'NOT READY'),
}
PLACES = {'5': ('적재 장소', 'LOADING'), '5 정면': ('적재 장소', 'LOADING'), '4': ('적재 대기 자리', 'QUEUE'),
          '통로 아래': ('통로', 'PASSAGE'), '통로 위': ('통로', 'PASSAGE'), '주차장': ('주차장', 'HOME'),
          '원점': ('출발 자리', 'START'), '출발': ('출발 자리', 'START')}


def _place(label):
    """(Korean, English) name of a waypoint label for the screen, by what the place is for."""
    label = str(label).strip()
    if label in PLACES:
        return PLACES[label]
    if label.startswith('수령'):   # '수령 1', '수령 A'
        which = label[2:].strip()
        return (f'수령 장소 {which}'.strip(), f'PICKUP {which}'.strip() if which.isascii() else 'PICKUP')
    if label.isdigit():
        return f'{label}번 지점', f'POINT {label}'
    return label, (label if label.isascii() else '')


# The screen shows a face while the Pinky drives (which face, by phase; any other driving is 'happy'), the battery
# once it is this low, STOPPED after a stop, and otherwise its name with what it is doing.
MOVING_FACE = {'recovering': 'surprised', 'docking': 'focus', 'dock_entry': 'focus', 'redocking': 'focus',
               'dock_exit': 'focus', 'returning': 'excited'}   # going home: the happiest one (2026-11-24)
STANDING_PHASES = ('docking_manual', 'docking_anchor', 'docked_manual', 'course_complete')
LOW_BATTERY_PERCENT = 20
LOW_BATTERY_RELEASE_PERCENT = 25


class BatteryView:
    """The battery as the screen shows it. The reading sags and jumps while a Pinky drives (65, 59, 58 within a
    minute, 2026-11-24), and every change redraws the screen: the middle of the last readings is shown, it moves
    only by 2 points or more, and the low-battery warning stays on until LOW_BATTERY_RELEASE_PERCENT."""

    def __init__(self, window=20):
        self.readings, self.window, self.shown, self.low = [], window, None, False

    def update(self, percent):
        """(percent to show or None, low battery?) after this reading; None keeps what was shown."""
        try:
            value = max(0.0, min(100.0, float(percent)))
        except (TypeError, ValueError):
            return self.shown, self.low
        self.readings = (self.readings + [value])[-self.window:]
        middle = int(round(sorted(self.readings)[len(self.readings) // 2]))
        if self.shown is None or abs(middle - self.shown) >= 2:
            self.shown = middle
        self.low = self.shown <= LOW_BATTERY_PERCENT or (self.low and self.shown < LOW_BATTERY_RELEASE_PERCENT)
        return self.shown, self.low


def _toward(place):
    """'<place>로' or '<place>으로' by how the last syllable ends (no final consonant or ㄹ takes 로); a digit by how
    it is read (0 영, 3 삼, 6 육 take 으로)."""
    last = ord(place[-1]) - 0xAC00
    if place[-1].isdigit():
        return place + ('으로' if place[-1] in '036' else '로')
    return place + ('으로' if 0 <= last < 11172 and last % 28 not in (0, 8) else '로')


def activity(robot, route, operational, battery=None, low_battery=None, waiting=False):
    """What this Pinky is doing now, for its own screen: text/word (headline, Korean/English), detail/detail_en
    (a second line, may be empty), battery (whole percent or None), and which screen to draw: screen is 'battery'
    (LOW_BATTERY_PERCENT or less, whatever else it does), 'stop' (being stopped, or stopped and waiting for a
    command), 'face' with face (driving) or 'text'. Same order of precedence as indicator(). low_battery comes
    from a BatteryView, which keeps the warning from flickering around the threshold. waiting: it stands for
    another Pinky to pass (the behaviour layer's traffic_wait); route['yield_to']: it gives way to that Pinky."""
    route = route or {}
    detail = detail_en = ''
    screen, face = 'text', None
    if route.get('stop_pending') or robot.get('stage') == 'stopping' or route.get('status') == 'cancel_requested':
        text, word, screen = 'STOPPED', 'STOPPED', 'stop'
        detail, detail_en = '멈추는 중', 'STOPPING'
    elif robot.get('stage') == 'fault':
        text, word = IDLE_ACTIVITY['fault']
    elif route.get('status') == 'cancelled' and operational.get('state') == 'ready':
        text, word, screen = 'STOPPED', 'STOPPED', 'stop'
        detail, detail_en = '명령 기다리는 중', 'WAITING'
    elif route.get('status') in ('starting', 'running'):
        phase, label = route.get('phase'), route.get('current_label')
        if phase not in STANDING_PHASES:
            screen, face = 'face', MOVING_FACE.get(phase, 'happy')
        if phase in PHASE_ACTIVITY:
            text, word = PHASE_ACTIVITY[phase]
        elif route.get('return_home') or label == '주차장':
            text, word = PHASE_ACTIVITY['returning']
            face = 'excited' if screen == 'face' else face
        elif label:
            place, place_en = _place(label)
            text, word = f'{_toward(place)} 이동 중', ('TO ' + place_en if place_en else 'DRIVING')
        else:
            text, word = '출발 준비 중', 'STARTING'
        dropoff = route.get('dropoff')
        delivery = _place(dropoff) if dropoff and dropoff not in ('주차장', '원점') else None
        if delivery and phase == 'course_complete':
            text, word = f'{delivery[0]} 도착', ('AT ' + delivery[1] if delivery[1] else word)
        elif delivery and phase != 'returning' and not route.get('return_home'):
            detail, detail_en = f'배달: {delivery[0]}', ('FOR ' + delivery[1] if delivery[1] else '')
        # Standing for another Pinky (not at the station, where it stands for its own reasons): say so instead of
        # "moving", and where it was going moves to the second line.
        if phase not in STANDING_PHASES and (route.get('yield_to') or waiting):
            going = (label and _place(label)) or ('', '')
            detail, detail_en = (f'{going[0]} 가는 길' if going[0] else ''), ('TO ' + going[1] if going[1] else '')
            text, word = ('양보 중', 'GIVING WAY') if route.get('yield_to') else ('교차로 대기', 'WAITING')
            screen, face = 'text', None
    else:
        text, word = IDLE_ACTIVITY.get(operational.get('state'), IDLE_ACTIVITY['unprepared'])
    try:
        battery = max(0, min(100, int(round(float(battery)))))
    except (TypeError, ValueError):
        battery = None
    if low_battery is None:  # no BatteryView: the plain threshold
        low_battery = battery is not None and battery <= LOW_BATTERY_PERCENT
    if battery is not None and low_battery:
        # What it was doing moves to the second line.
        screen, face, detail, detail_en = 'battery', None, text, word
        text, word = '배터리 부족', 'LOW BATTERY'
    return dict(text=text, word=word, detail=detail, detail_en=detail_en, battery=battery, screen=screen, face=face)
