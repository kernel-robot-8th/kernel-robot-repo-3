"""실측한 map 프레임 기준점(anchor)들로 순서가 정해지고 점 간격이 제한된 경로를 만든다.

실행 위치: PC 관제(app.py가 import). 오프라인 도구 export_dense_route.py도 쓴다.
사용처: app.py의 임무 시작(build_mission_route), 적재 자리 카메라 보정(corrected_dock, clean_dock_guide),
DOCK_APPROACH_M. 결과 goal 목록은 JSON으로 로봇의 navigate_route.py에 넘어간다.

핵심:
- 경로 점은 {label, map, frame_id, x_m, y_m, yaw_rad, ...} dict다. 좌표는 m, 각도는 rad(map 프레임).
- build_dense_route: 예전 방식. 정해진 순서(ROUTE_LABELS)의 웨이포인트 사이를 10 cm 간격 점으로 채운다.
- build_mission_route: 지금 임무 방식. 정차점만 두고 그 사이 길은 Nav2가 스스로 계획한다. 5번(적재 자리)
  앞에서 정면으로 맞추는 점, 5번에서 뒤로 빠지는 점(도킹 탈출), 벽 통로를 곧게 지나는 두 점을 끼워 넣는다.
- 천장 카메라가 잰 5번 자리와의 어긋남(offset)을 검사해 보정 위치를 계산한다.
"""
import math

# 초기 10번 핑키 순회 경로의 웨이포인트 순서(README의 1 → 2 → 3 → 7 → 4 → 5 → 6 → 7 → 9 → 8 → 1).
ROUTE_LABELS = ['1', '2', '3', '7', '4', '5', '6', '7', '9', '8', '1']
# 촘촘한 경로의 점 간격(m). 10 cm.
CHECKPOINT_SPACING_M = 0.10


def build_dense_route(anchors, labels=ROUTE_LABELS, spacing=CHECKPOINT_SPACING_M):
    """웨이포인트(anchors)를 labels 순서로 이어 spacing(m) 간격 점으로 채운 경로를 돌려준다.

    5번(도킹) 바로 다음이 6번이면, 그 사이에 5번에서 들어온 방향 반대로 20 cm 후진하는 '도킹 탈출' 점을 넣는다.
    """
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('Invalid checkpoint spacing')
    by_label = {p['label']: p for p in anchors}
    selected = []
    for label in labels:
        if label == '6' and selected and selected[-1]['label'] == '5':
            dock = selected[-1]
            previous = selected[-2] if len(selected) > 1 else None
            heading = (math.atan2(dock['y_m']-previous['y_m'], dock['x_m']-previous['x_m'])
                       if previous else dock['yaw_rad'])
            selected.append(dict(dock, label='도킹 탈출',
                x_m=dock['x_m']-.20*math.cos(heading), y_m=dock['y_m']-.20*math.sin(heading),
                yaw_rad=heading, arrival_yaw_rad=heading, dock_exit=True, reverse=True))
        selected.append(by_label[label])
    return _densify(selected, spacing)


# 5번에서 뒤로 빠지는 거리(m).
DOCK_EXIT_M = .20
DOCK_APPROACH_M = 0.25  # 5번 자리 앞 정면 맞추기 지점까지의 거리(m). 여기서 방향을 맞춘 뒤 곧장 들어간다.


# 카메라 어긋남은 이 정도로 작을 때만 보정에 쓴다. 더 크면 카메라나 위치 추정이 크게 틀렸다는 뜻이므로
# 핑키는 보정 없이 예전처럼 다시 들어온다. 앞 줄: 카메라 값 한도(mm, 도), 뒷 줄: 보정 결과 한도(m, rad).
DOCK_OFFSET_LIMIT_MM, DOCK_OFFSET_LIMIT_DEG = 150., 30.
DOCK_CORRECTION_LIMIT_M, DOCK_CORRECTION_LIMIT_RAD = .12, math.radians(25.)


def clean_dock_offset(offset):
    """천장 카메라가 잰, 등록된 적재 자리에서 핑키가 어긋난 정도. 관제 서버가 redock과 함께 보낸다.

    forward_mm·left_mm은 자리 자신의 축 기준, dyaw_deg는 + 가 왼쪽으로 돈 쪽이다. 값이 없거나 숫자가 아니거나
    위 한도보다 크면 None을 돌려준다."""
    # bool은 int의 하위 클래스라 True/False가 숫자로 통과하지 않도록 따로 막는다.
    if not isinstance(offset, dict):
        return None
    values = [offset.get(key) for key in ('forward_mm', 'left_mm', 'dyaw_deg')]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        return None
    forward, left, dyaw = (float(v) for v in values)
    if math.hypot(forward, left) > DOCK_OFFSET_LIMIT_MM or abs(dyaw) > DOCK_OFFSET_LIMIT_DEG:
        return None
    return dict(forward_mm=forward, left_mm=left, dyaw_deg=dyaw)


def clean_dock_guide(offset):
    """같은 세 값을 5번으로 들어가는 동안의 안내로 쓴다(핑키가 들어오는 중에 실시간으로 중계됨).

    그때 핑키는 아직 자리에서 최대 45 cm 못 미친 곳에 있으므로 forward에는 따로 더 긴 한도(500 mm)를 둔다.
    쓸 수 없으면 None."""
    if not isinstance(offset, dict):
        return None
    values = [offset.get(key) for key in ('forward_mm', 'left_mm', 'dyaw_deg')]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        return None
    forward, left, dyaw = (float(v) for v in values)
    # 방향은 어떤 값이든 받는다(±180도): 카메라가 마커를 처음 본 순간 핑키는 아직 돌아서 있을 수 있다(2026-11-19).
    if abs(forward) > 500. or abs(left) > 250. or abs(dyaw) > 180.:
        return None
    return dict(forward_mm=round(forward, 1), left_mm=round(left, 1), dyaw_deg=round(dyaw, 2))


def corrected_dock(pose, offset, dock, approach_m):
    """핑키 자신의 지도 추정 안에서 적재 자리가 실제로 어디인지 구한다(2026-11-17).

    핑키는 `pose`(스스로 믿는 x, y, yaw)에 서 있고, 천장 카메라는 이 자리가 등록된 자리에서 `offset`만큼
    떨어져 있다고 쟀다. 그러므로 실제 자리는 `pose`가 그로부터 forward_mm 앞, left_mm 왼쪽, dyaw_deg 돌아간
    곳이 되는 포즈다. 어긋난 원인(위치 추정 편향, 허용 오차 끝에서 도달한 목표, 바퀴 미끄러짐)이 무엇이든
    거기로 다시 들어가면 된다. (정면 맞추기 점, 보정된 자리)를 돌려주고, offset을 쓸 수 없거나 보정된 자리가
    저장된 웨이포인트에서 보정 한도보다 멀면 None을 돌려준다."""
    # 아래는 2D 좌표 변환의 역산이다: 자리의 yaw = 핑키 yaw - dyaw, 자리 좌표 = 핑키 좌표 - R(yaw)·(forward, left).
    offset = clean_dock_offset(offset)
    if offset is None or pose is None:
        return None
    yaw = pose[2]-math.radians(offset['dyaw_deg'])
    forward, left = offset['forward_mm']/1000., offset['left_mm']/1000.
    x = pose[0]-(forward*math.cos(yaw)-left*math.sin(yaw))
    y = pose[1]-(forward*math.sin(yaw)+left*math.cos(yaw))
    # 각도 차이를 [-pi, pi)로 감싼다.
    turn = (yaw-dock.get('yaw_rad', yaw)+math.pi) % (2*math.pi)-math.pi
    if math.hypot(x-dock['x_m'], y-dock['y_m']) > DOCK_CORRECTION_LIMIT_M or abs(turn) > DOCK_CORRECTION_LIMIT_RAD:
        return None
    spot = dict(dock, x_m=x, y_m=y, yaw_rad=yaw)
    square = dict(x_m=x-approach_m*math.cos(yaw), y_m=y-approach_m*math.sin(yaw), yaw_rad=yaw)
    return square, spot


def _through_passages(stops, passages):
    """벽 양쪽에 놓인 두 정차점 사이에 통로의 두 점을 넣는다: 통로 바로 앞에서 정면을 맞추고, 통로 한가운데를
    곧게 지나간다(2026-11-16: Nav2가 스스로 만든 길이 벽 끝을 스쳐 로봇이 거기서 멈췄다).
    passages: navigation.json wall_passages의 [{x_m, wall_y_m, approach_m}]."""
    if not passages:
        return list(stops)
    result = [stops[0]]
    for a, b in zip(stops, stops[1:]):
        for passage in passages:
            x, wall, gap = float(passage['x_m']), float(passage['wall_y_m']), float(passage.get('approach_m', .18))
            # 벽 선(y = wall)에서 approach의 0.6배보다 더 떨어져 있어야 벽 아래/위로 본다. 벽에 붙은 점은 건너는 것으로 치지 않는다.
            below_a, below_b = a['y_m'] < wall-gap*.6, b['y_m'] < wall-gap*.6
            above_a, above_b = a['y_m'] > wall+gap*.6, b['y_m'] > wall+gap*.6
            # 두 점이 벽 반대편에 있으면, 통로 x에서 벽 앞뒤 approach_m 지점 두 개를 진행 방향(+y 또는 -y)으로 넣는다.
            if (below_a and above_b) or (above_a and below_b):
                up = below_a
                heading = math.pi/2 if up else -math.pi/2
                first, second = (wall-gap, wall+gap) if up else (wall+gap, wall-gap)
                for label, y in ((('통로 아래', first), ('통로 위', second)) if up else (('통로 위', first), ('통로 아래', second))):
                    result.append(dict(label=label, map=b['map'], frame_id=b['frame_id'], x_m=x, y_m=y,
                                       yaw_rad=heading, arrival_yaw_rad=heading))
        result.append(b)
    return result


def build_mission_route(stops, spacing=10.0, passages=None, reverse_exit=True):
    """자유 임무 경로: 정차점 사이 길은 Nav2가 스스로 계획하고, 그 사이 점을 따라가지 않는다.

    `stops`는 순서대로 놓인 점 dict다(출발, '5', 내려놓는 곳, 주차장). 5번 다음에는 어디서 들어왔든 도킹의
    기록된 방향을 따라 DOCK_EXIT_M만큼 후진해 빠지고, 도킹은 그 방향을 바라보고 한다.
    spacing이 크면(기본 10 m) 정차점만 남으므로 화면의 경로 띠가 체크포인트 줄이 아니라 임무를 보여 준다."""
    if len(stops) < 2:
        raise ValueError('A mission needs a start and at least one stop')
    stops = _through_passages(stops, passages)
    selected = []
    for index, stop in enumerate(stops):
        if stop['label'] == '5' and index > 0:
            # 먼저 정면을 맞춘다(2026-11-16): 5번에 비스듬히 다가가다 앞 모서리가 18 cm 못 미쳐 거치대에 닿았고
            # 최종 감시기가 멈춰 세웠다. 그래서 DOCK_APPROACH_M 앞에서 멈춰 도킹 방향으로 돈 뒤 마지막 구간을
            # 곧게 들어간다.
            heading = stop['yaw_rad']
            selected.append(dict(stop, label='5 정면',
                x_m=stop['x_m']-DOCK_APPROACH_M*math.cos(heading), y_m=stop['y_m']-DOCK_APPROACH_M*math.sin(heading),
                yaw_rad=heading, arrival_yaw_rad=heading))
        selected.append(stop)
        # reverse_exit가 False면(navigation.json dock_exit_reverse, 2026-11-17) 5번에서 후진하지 않는다. Nav2가
        # 그 자리에서 핑키를 돌려 앞으로 나간다.
        if reverse_exit and stop['label'] == '5' and index + 1 < len(stops):
            heading = stop['yaw_rad']
            selected.append(dict(stop, label='도킹 탈출',
                x_m=stop['x_m']-DOCK_EXIT_M*math.cos(heading), y_m=stop['y_m']-DOCK_EXIT_M*math.sin(heading),
                yaw_rad=heading, arrival_yaw_rad=heading, dock_exit=True, reverse=True))
    result = _densify(selected, spacing)
    for point in result:
        if point.get('anchor') and point['label'] == '5':
            point['yaw_rad'] = point['recorded_yaw_rad']  # 기록된 방향 그대로 곧게 도킹한다
    return result


def _densify(selected, spacing):
    """정차점 사이를 spacing(m) 이하 간격의 중간 점(anchor=False)으로 채우고, 각 정차점(anchor=True)의 yaw를 정한다.

    모든 점은 같은 지도의 'map' 프레임이어야 한다. 중간 점의 yaw는 구간 진행 방향이고, 도킹 탈출 구간은
    후진(reverse=True)이므로 도착 방향(arrival_yaw_rad)을 유지한다. 점이 500개를 넘으면 거부한다.
    """
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('Invalid checkpoint spacing')
    result = []
    for index, end in enumerate(selected):
        if index:
            start = selected[index - 1]
            if start['map'] != end['map'] or start['frame_id'] != 'map' or end['frame_id'] != 'map':
                raise ValueError('Route coordinates must use the same map frame')
            dx, dy = end['x_m'] - start['x_m'], end['y_m'] - start['y_m']
            steps = max(1, math.ceil(math.hypot(dx, dy) / spacing))
            heading = math.atan2(dy, dx)
            for step in range(1, steps):
                result.append(dict(label=f"{start['label']}→{end['label']}:{step:02d}",
                    map=end['map'], frame_id='map', x_m=start['x_m'] + dx*step/steps,
                    y_m=start['y_m'] + dy*step/steps, yaw_rad=end.get('arrival_yaw_rad', heading) if end.get('dock_exit') else heading,
                    anchor=False, segment_index=index, reverse=bool(end.get("dock_exit"))))
        point = {key: end[key] for key in ('label', 'map', 'frame_id', 'x_m', 'y_m', 'yaw_rad')}
        if not all(math.isfinite(float(point[k])) for k in ('x_m','y_m','yaw_rad')):
            raise ValueError('Non-finite waypoint coordinate')
        # 같은 좌표를 반대 방향으로 두 번 지날 수 있다. 그래서 yaw는 방문마다 나가는 구간 방향으로 고르고,
        # 마지막 정차점에서는 출발 방향으로 되돌아 돌지 않고 들어온 방향을 유지한다.
        # 원래 기록된 yaw는 recorded_yaw_rad에 남긴다.
        point["recorded_yaw_rad"] = point["yaw_rad"]
        heading = None
        for following in selected[index + 1:]:
            dx, dy = following["x_m"] - end["x_m"], following["y_m"] - end["y_m"]
            if math.hypot(dx, dy) > 1e-9:
                heading = math.atan2(dy, dx)
                break
        if heading is None:
            for previous in reversed(selected[:index]):
                dx, dy = end["x_m"] - previous["x_m"], end["y_m"] - previous["y_m"]
                if math.hypot(dx, dy) > 1e-9:
                    heading = math.atan2(dy, dx)
                    break
        if heading is not None:
            point["yaw_rad"] = heading
        # 5번 다음이 후진 탈출이면 '나가는 방향'은 뒤쪽이므로, 대신 들어온 방향을 yaw로 쓴다.
        if end['label'] == '5' and index and index+1 < len(selected) and selected[index+1].get('dock_exit'):
            previous = selected[index-1]
            point['yaw_rad'] = math.atan2(end['y_m']-previous['y_m'],end['x_m']-previous['x_m'])
        if end.get('dock_exit'):
            point.update(dock_exit=True, reverse=True, arrival_yaw_rad=end['arrival_yaw_rad'])
            point['yaw_rad'] = end['arrival_yaw_rad']
        elif end.get('arrival_yaw_rad') is not None:
            # 도착할 때 정해진 방향을 바라봐야 하는 정차점(5번 앞 정면 맞추기, 출발했을 때 방향 그대로 원점 복귀).
            point['arrival_yaw_rad'] = end['arrival_yaw_rad']
        point.update(anchor=True, segment_index=index)
        result.append(point)
    if len(result) > 500:
        raise ValueError('Route has more than 500 checkpoints')
    return result
