"""핑키의 '지금' 주행 가능 상태를 마지막 주행준비 성공 기록과 따로 판단하고, 램프 색·화면 내용을 정한다.

실행 위치: PC 관제(app.py가 operational_status, indicator, activity, BatteryView를 import).
사용처: 관제 화면의 상태 표시와 출발 버튼 활성화(operational_status), 그리고 app.py가 0.25초마다 각 로봇 도메인의
/fleet/indicator로 보내는 램프 색(indicator → 로봇의 status_lamp.py)과 화면 내용(activity → status_display.py).

핵심: 준비 성공 기록만으로는 출발할 수 없다(OPERATIONS.md). 필수 토픽이 모두 신선하고, 위치가 확인됐고,
로봇에 최신 감시기 코드가 올라가 있어야 'ready'다. 모두 순수 함수라 ROS 없이 테스트된다.
"""
# 출발 전에 모두 신선(fresh)해야 하는 텔레메트리 항목. line_control은 행동 선택 계층(ultrasonic_safety.py)
# 상태, safety_status는 최종 감시기(velocity_guard.py) 상태다.
REQUIRED=('map','tf_pose','scan','odom','ultrasonic','ir','line_control','safety_status')

def operational_status(robot,topics):
    """robot(관제 상태)과 topics(텔레메트리 항목별 {fresh, value})로 지금 상태를 판정한다.

    state는 아래 순서에서 처음 걸리는 것: fault → disconnected → unprepared → stale(센서 대기) →
    unlocalized(위치 확인 필요) → update_required(로봇 코드가 옛 버전) → ready. ready일 때만 ready_for_motion이다.
    """
    missing=[k for k in REQUIRED if not topics.get(k,{}).get('fresh')]
    control=topics.get('line_control',{}).get('value') or {}
    final=topics.get('safety_status',{}).get('value') or {}
    # 로봇이 보고하는 버전 문자열이 이 관제가 기대하는 것과 같아야 '최신 감시기 적용됨'으로 본다
    # (line_control은 ultrasonic_safety.py, safety_status는 velocity_guard.py가 보낸다).
    # 로봇 코드를 바꿔 이 문자열이 바뀌면 여기도 같이 바꿔야 한다.
    updated=control.get('dock_protocol')=='sponge_v1' and control.get('line_algorithm')=='fleet_control_v4' and final.get('version')=='final_gate_v1'
    state='ready';label='현재 주행 가능'
    if robot.get('stage')=='fault':state,label='fault','준비 오류'
    elif not robot.get('ssh_reachable'):state,label='disconnected','연결 끊김'
    elif robot.get('stage')!='prepared':state,label='unprepared','주행준비 필요'
    elif missing:state,label='stale','센서·위치 수신 대기'
    # 막 시작한 AMCL은 기본 위치(지도 원점)를 발행할 수 있다. 그래서 지도 위치는 최근 Nav2 시작 뒤에 라이다 위치
    # 찾기나 작업자가 위치를 정해 준 경우에만 인정한다(app.py가 두 시각을 모두 기록한다).
    elif robot.get('nav_started_at') and (robot.get('localized_at') or 0)<robot['nav_started_at']:state,label='unlocalized','위치 확인 필요'
    elif not updated:state,label='update_required','새 감시기 적용 필요'
    return dict(state=state,label=label,ready_for_motion=state=='ready',missing=missing,
                last_preparation_ok=bool(robot.get('last_prepared_at') or robot.get('previously_prepared') or robot.get('stage')=='prepared'),runtime_updated=updated)


def indicator(robot,route,operational):
    """핑키 램프 색(scripts/status_lamp.py가 켠다): 정지 적용 중이거나 오류면 빨강, 경로 주행 중이면 초록,
    준비되어 기다리면 파랑, 연결·준비 중이거나 위치가 없으면 주황. 이 값이 더 오지 않으면 로봇이 스스로
    주황으로 깜빡인다."""
    route=route or {}
    if route.get('stop_pending') or robot.get('stage') in ('fault','stopping'):return dict(color='red',mode='on')
    if route.get('status') in ('starting','running','cancel_requested'):return dict(color='green',mode='on')
    if operational.get('ready_for_motion'):return dict(color='blue',mode='on')
    return dict(color='orange',mode='on')


# 핑키 자기 화면(scripts/status_display.py)에 보일 '지금 하는 일': 임무 단계마다 (한국어, 영어). 영어는 로봇에
# 한글 글꼴이 없을 때 보인다. 장소는 웨이포인트 번호가 아니라 쓰임새로 부른다(2026-11-24: "5번"은 로봇 옆에 선
# 사람에게 아무 뜻이 없었다): 적재 장소, 수령 장소 1, 주차장.
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
# 경로를 달리지 않을 때: operational_status의 state별 문구.
IDLE_ACTIVITY = {
    'ready': ('대기 중', 'READY'), 'fault': ('준비 오류', 'FAULT'), 'disconnected': ('연결 끊김', 'NO LINK'),
    'unprepared': ('주행준비 필요', 'NOT READY'), 'stale': ('센서 기다리는 중', 'NO SENSOR'),
    'unlocalized': ('위치 확인 필요', 'NO POSE'), 'update_required': ('주행준비 다시 필요', 'NOT READY'),
}
# 웨이포인트 라벨 → 화면용 장소 이름 (한국어, 영어).
PLACES = {'5': ('적재 장소', 'LOADING'), '5 정면': ('적재 장소', 'LOADING'), '4': ('적재 대기 자리', 'QUEUE'),
          '통로 아래': ('통로', 'PASSAGE'), '통로 위': ('통로', 'PASSAGE'), '주차장': ('주차장', 'HOME'),
          '원점': ('출발 자리', 'START'), '출발': ('출발 자리', 'START')}


def _place(label):
    """웨이포인트 라벨을 화면용 (한국어, 영어) 장소 이름으로 바꾼다. 장소의 쓰임새로 부른다."""
    label = str(label).strip()
    if label in PLACES:
        return PLACES[label]
    if label.startswith('수령'):   # '수령 1', '수령 A'
        which = label[2:].strip()
        return (f'수령 장소 {which}'.strip(), f'PICKUP {which}'.strip() if which.isascii() else 'PICKUP')
    if label.isdigit():
        return f'{label}번 지점', f'POINT {label}'
    return label, (label if label.isascii() else '')


# 화면은 핑키가 달리는 동안 표정을 보인다(단계별 표정, 그 밖의 주행은 'happy'). 배터리가 이만큼 낮으면 배터리,
# 정지 뒤에는 STOPPED, 그 밖에는 이름과 지금 하는 일을 보인다.
MOVING_FACE = {'recovering': 'surprised', 'docking': 'focus', 'dock_entry': 'focus', 'redocking': 'focus',
               'dock_exit': 'focus', 'returning': 'excited'}   # 집으로 가는 길: 가장 신난 표정(2026-11-24)
# 서 있는 단계(적재 장소 도착·싣는 중·수령 장소 도착): 표정 대신 글자 화면을 보인다.
STANDING_PHASES = ('docking_manual', 'docking_anchor', 'docked_manual', 'course_complete')
# 저전압 경고는 20% 이하에서 켜고 25% 이상이 돼야 끈다(히스테리시스: 경계에서 깜빡이지 않게).
LOW_BATTERY_PERCENT = 20
LOW_BATTERY_RELEASE_PERCENT = 25


class BatteryView:
    """화면에 보이는 배터리 값. 핑키가 달리는 동안 측정값은 처졌다 튀었다 하고(1분 안에 65, 59, 58,
    2026-11-24), 값이 바뀔 때마다 화면을 다시 그린다. 그래서 최근 측정값의 중앙값을 보이고, 2 포인트 이상
    바뀔 때만 움직이며, 저전압 경고는 LOW_BATTERY_RELEASE_PERCENT가 될 때까지 켜 둔다."""

    def __init__(self, window=20):
        # window: 중앙값을 낼 최근 측정 개수. shown: 지금 화면 값, low: 저전압 경고 켜짐 여부.
        self.readings, self.window, self.shown, self.low = [], window, None, False

    def update(self, percent):
        """이번 측정값을 넣은 뒤 (보일 퍼센트 또는 None, 저전압인가?)를 돌려준다. 숫자가 아니면 보이던 값을 유지한다."""
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
    """마지막 음절의 받침에 따라 '<장소>로' 또는 '<장소>으로'를 붙인다(받침이 없거나 ㄹ이면 로). 숫자는 읽는
    소리로 정한다(0 영, 3 삼, 6 육은 으로)."""
    # 한글 음절 = 0xAC00 + (초성*21 + 중성)*28 + 종성. 종성 0은 받침 없음, 8은 ㄹ. 11172는 한글 음절 수.
    last = ord(place[-1]) - 0xAC00
    if place[-1].isdigit():
        return place + ('으로' if place[-1] in '036' else '로')
    return place + ('으로' if 0 <= last < 11172 and last % 28 not in (0, 8) else '로')


def activity(robot, route, operational, battery=None, low_battery=None, waiting=False):
    """이 핑키가 지금 하는 일을 자기 화면용으로 돌려준다: text/word(제목, 한국어/영어), detail/detail_en(둘째 줄,
    비어 있을 수 있음), battery(정수 퍼센트 또는 None), 그리고 그릴 화면 screen: 'battery'(무엇을 하든
    LOW_BATTERY_PERCENT 이하), 'stop'(멈추는 중, 또는 멈춘 뒤 명령 대기), 'face'와 face(주행 중), 'text'.
    우선순위는 indicator()와 같다. low_battery는 BatteryView에서 오며 경계값 근처에서 경고가 깜빡이지 않게 한다.
    waiting: 다른 핑키가 지나가길 기다리며 서 있다(행동 계층의 traffic_wait). route['yield_to']: 그 핑키에게
    길을 비키는 중이다."""
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
        # 다른 핑키 때문에 서 있으면(자기 일로 서 있는 적재 장소는 제외) "이동 중" 대신 그렇게 말하고,
        # 가던 곳은 둘째 줄로 옮긴다.
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
    if low_battery is None:  # BatteryView가 없으면 단순 임계값으로 판단
        low_battery = battery is not None and battery <= LOW_BATTERY_PERCENT
    if battery is not None and low_battery:
        # 하던 일은 둘째 줄로 옮긴다.
        screen, face, detail, detail_en = 'battery', None, text, word
        text, word = '배터리 부족', 'LOW BATTERY'
    return dict(text=text, word=word, detail=detail, detail_en=detail_en, battery=battery, screen=screen, face=face)
