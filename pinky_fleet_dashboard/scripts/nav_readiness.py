#!/usr/bin/env python3
"""Check Nav2 lifecycle activation without sending a movement goal."""
import json
import time
import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def main():
    rclpy.init()
    node = rclpy.create_node('pinky_dashboard_nav_readiness')
    names = ('map_server', 'amcl', 'controller_server', 'planner_server', 'bt_navigator', 'behavior_server')
    clients = {name: node.create_client(GetState, '/'+name+'/get_state') for name in names}
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    states = {}
    pending = {}
    deadline = time.monotonic()+75.
    previous = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            for name, client in clients.items():
                future = pending.get(name)
                if future is not None and future.done():
                    try:
                        state = future.result().current_state
                        states[name] = (state.id, state.label)
                    except Exception as exc:
                        states[name] = (0, str(exc))
                    del pending[name]
                if name not in pending and client.service_is_ready():
                    pending[name] = client.call_async(GetState.Request())
            summary = json.dumps(states, sort_keys=True)
            if summary != previous:
                print('NAV_LIFECYCLE '+summary, flush=True)
                previous = summary
            if all(states.get(name, (0,))[0] == 3 for name in names) and action.server_is_ready():
                print('NAV_ACTION_READY · lifecycle active', flush=True)
                return 0
            rclpy.spin_once(node, timeout_sec=.1)
        print('NAV_ACTION_MISSING_OR_INACTIVE '+json.dumps(states), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
