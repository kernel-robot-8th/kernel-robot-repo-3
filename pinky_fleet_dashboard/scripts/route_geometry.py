"""Build an ordered, bounded-spacing route from measured map-frame anchors."""
import math

ROUTE_LABELS = ['1', '2', '3', '7', '4', '5', '6', '7', '9', '8', '1']
CHECKPOINT_SPACING_M = 0.10


def build_dense_route(anchors, labels=ROUTE_LABELS, spacing=CHECKPOINT_SPACING_M):
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('Invalid checkpoint spacing')
    by_label = {p['label']: p for p in anchors}
    selected = [by_label[label] for label in labels]
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
                    y_m=start['y_m'] + dy*step/steps, yaw_rad=heading,
                    anchor=False, segment_index=index))
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
        point.update(anchor=True, segment_index=index)
        result.append(point)
    if len(result) > 500:
        raise ValueError('Route has more than 500 checkpoints')
    return result
