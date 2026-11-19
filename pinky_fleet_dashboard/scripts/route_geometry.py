"""Build an ordered, bounded-spacing route from measured map-frame anchors."""
import math

ROUTE_LABELS = ['1', '2', '3', '7', '4', '5', '6', '7', '9', '8', '1']
CHECKPOINT_SPACING_M = 0.10


def build_dense_route(anchors, labels=ROUTE_LABELS, spacing=CHECKPOINT_SPACING_M):
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


DOCK_EXIT_M = .20
DOCK_APPROACH_M = 0.25  # where the Pinky squares up before driving straight into station 5


# A camera offset is used as a correction only when it is this small: a larger one means the camera or the
# localization is far off, and then the Pinky just comes in again as before.
DOCK_OFFSET_LIMIT_MM, DOCK_OFFSET_LIMIT_DEG = 150., 30.
DOCK_CORRECTION_LIMIT_M, DOCK_CORRECTION_LIMIT_RAD = .12, math.radians(25.)


def clean_dock_offset(offset):
    """The ceiling camera's offset of the Pinky from its registered loading spot, as the control station sends it with
    redock: forward_mm and left_mm along the spot's own axes, dyaw_deg (+ = turned left). None when it is missing,
    not numbers, or larger than the limits above."""
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
    """The same three numbers as a guide for the drive into station 5 (relayed live while the Pinky comes in): it is
    still up to 45 cm short of the spot then, so forward has its own, longer limit. None when unusable."""
    if not isinstance(offset, dict):
        return None
    values = [offset.get(key) for key in ('forward_mm', 'left_mm', 'dyaw_deg')]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        return None
    forward, left, dyaw = (float(v) for v in values)
    # Any heading: from the moment the camera sees the marker the Pinky may still be turned away (2026-11-19).
    if abs(forward) > 500. or abs(left) > 250. or abs(dyaw) > 180.:
        return None
    return dict(forward_mm=round(forward, 1), left_mm=round(left, 1), dyaw_deg=round(dyaw, 2))


def corrected_dock(pose, offset, dock, approach_m):
    """Where the loading spot really is in the Pinky's own map estimate (2026-11-17).

    The Pinky stands at `pose` (x, y, yaw as it believes) and the ceiling camera measured that this is `offset`
    away from the registered spot. So the spot is the pose from which `pose` lies forward_mm ahead, left_mm to the
    left and dyaw_deg turned: that is where to drive in again, whatever caused the miss (localization bias, a goal
    reached at the edge of its tolerance, wheel slip). Returns (square-up point, corrected dock), or None when the
    offset is unusable or the corrected spot is further from the saved waypoint than the correction limit."""
    offset = clean_dock_offset(offset)
    if offset is None or pose is None:
        return None
    yaw = pose[2]-math.radians(offset['dyaw_deg'])
    forward, left = offset['forward_mm']/1000., offset['left_mm']/1000.
    x = pose[0]-(forward*math.cos(yaw)-left*math.sin(yaw))
    y = pose[1]-(forward*math.sin(yaw)+left*math.cos(yaw))
    turn = (yaw-dock.get('yaw_rad', yaw)+math.pi) % (2*math.pi)-math.pi
    if math.hypot(x-dock['x_m'], y-dock['y_m']) > DOCK_CORRECTION_LIMIT_M or abs(turn) > DOCK_CORRECTION_LIMIT_RAD:
        return None
    spot = dict(dock, x_m=x, y_m=y, yaw_rad=yaw)
    square = dict(x_m=x-approach_m*math.cos(yaw), y_m=y-approach_m*math.sin(yaw), yaw_rad=yaw)
    return square, spot


def _through_passages(stops, passages):
    """Between two stops on opposite sides of a wall, the passage's two points: square up just before it, cross
    it straight through its middle (2026-11-16: Nav2's own path brushed the wall's end and the robot stopped there).
    passages: [{x_m, wall_y_m, approach_m}] from navigation.json wall_passages."""
    if not passages:
        return list(stops)
    result = [stops[0]]
    for a, b in zip(stops, stops[1:]):
        for passage in passages:
            x, wall, gap = float(passage['x_m']), float(passage['wall_y_m']), float(passage.get('approach_m', .18))
            below_a, below_b = a['y_m'] < wall-gap*.6, b['y_m'] < wall-gap*.6
            above_a, above_b = a['y_m'] > wall+gap*.6, b['y_m'] > wall+gap*.6
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
    """A free mission: Nav2 plans its own way between the stops; nothing in between is followed.

    `stops` are point dicts in order (start, '5', drop-off, parking). After station 5 the robot backs out
    DOCK_EXIT_M along the dock's recorded heading, wherever it came from, and it docks facing that heading.
    The large spacing leaves only the stops, so the route strip shows the mission, not a line of checkpoints."""
    if len(stops) < 2:
        raise ValueError('A mission needs a start and at least one stop')
    stops = _through_passages(stops, passages)
    selected = []
    for index, stop in enumerate(stops):
        if stop['label'] == '5' and index > 0:
            # Square up first (2026-11-16): coming at 5 on a slant, the front corner met the station 18 cm short and the
            # final guard stopped it. So it stops DOCK_APPROACH_M in front, turns to the dock's heading there, and
            # drives the last stretch straight in.
            heading = stop['yaw_rad']
            selected.append(dict(stop, label='5 정면',
                x_m=stop['x_m']-DOCK_APPROACH_M*math.cos(heading), y_m=stop['y_m']-DOCK_APPROACH_M*math.sin(heading),
                yaw_rad=heading, arrival_yaw_rad=heading))
        selected.append(stop)
        # reverse_exit False (navigation.json dock_exit_reverse, 2026-11-17): no backing out of station 5; Nav2 turns
        # the Pinky round where it stands and drives out forward.
        if reverse_exit and stop['label'] == '5' and index + 1 < len(stops):
            heading = stop['yaw_rad']
            selected.append(dict(stop, label='도킹 탈출',
                x_m=stop['x_m']-DOCK_EXIT_M*math.cos(heading), y_m=stop['y_m']-DOCK_EXIT_M*math.sin(heading),
                yaw_rad=heading, arrival_yaw_rad=heading, dock_exit=True, reverse=True))
    result = _densify(selected, spacing)
    for point in result:
        if point.get('anchor') and point['label'] == '5':
            point['yaw_rad'] = point['recorded_yaw_rad']  # dock straight in along the recorded heading
    return result


def _densify(selected, spacing):
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
        # A coordinate can be visited in opposite directions. Choose yaw per
        # occurrence, using the outgoing segment; retain the incoming heading
        # at the final stop rather than rotating back to the start heading.
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
        if end['label'] == '5' and index and index+1 < len(selected) and selected[index+1].get('dock_exit'):
            previous = selected[index-1]
            point['yaw_rad'] = math.atan2(end['y_m']-previous['y_m'],end['x_m']-previous['x_m'])
        if end.get('dock_exit'):
            point.update(dock_exit=True, reverse=True, arrival_yaw_rad=end['arrival_yaw_rad'])
            point['yaw_rad'] = end['arrival_yaw_rad']
        elif end.get('arrival_yaw_rad') is not None:
            # A stop that must be faced a set way on arrival (squared up in front of 5, back at the start as it stood).
            point['arrival_yaw_rad'] = end['arrival_yaw_rad']
        point.update(anchor=True, segment_index=index)
        result.append(point)
    if len(result) > 500:
        raise ValueError('Route has more than 500 checkpoints')
    return result
