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
        point.update(anchor=True, segment_index=index)
        result.append(point)
    if len(result) > 500:
        raise ValueError('Route has more than 500 checkpoints')
    return result
