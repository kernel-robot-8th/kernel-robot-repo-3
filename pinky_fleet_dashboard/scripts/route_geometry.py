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


def build_mission_route(stops, spacing=10.0, passages=None):
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
        if stop['label'] == '5' and index + 1 < len(stops):
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
