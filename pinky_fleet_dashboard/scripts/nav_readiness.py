#!/usr/bin/env python3
"""Check Nav2 lifecycle activation without sending a movement goal.

argv: [deadline seconds] [Nav2 log path]. On Jazzy 1.3 a lifecycle manager waits a fixed 2 s for a node's get_state
and gives the whole bringup up when it misses one; a Raspberry Pi busy starting the base, the safety nodes and two
lifecycle managers at once misses it often (planner_server, amcl, 2026-11-16), while the nodes themselves are fine.
With the log, this probe sees the manager give up and finishes that manager's bringup itself: its node_names are
configured, then activated, in its own order, each call waiting as long as the Pi needs (NAV_MANAGER_GAVE_UP,
NAV_BRINGUP_FINISHED). Only when that fails too is NAV_BRINGUP_ABORTED reported, at once rather than after the
deadline, and the dashboard starts Nav2 again."""
import json
import re
import sys
import time

START_MARK = '=== PINKY_FLEET_START'  # written by the dashboard before each launch; the log is appended to
ABORT_MARKS = ('Aborting bringup', 'Failed to bring up all requested nodes')


def aborted_managers(log_text):
    """{lifecycle manager: its first error line} for the managers that gave their bringup up in the latest launch."""
    start = log_text.rfind(START_MARK)
    tail = log_text[start:] if start >= 0 else log_text
    found = {}
    for line in tail.splitlines():
        match = re.search(r'\[(lifecycle_manager\w*)\]: (Failed to change state.*|Failed to bring up all requested nodes.*)', line)
        if match and match.group(1) not in found:
            found[match.group(1)] = match.group(2).strip()[-300:]
    return {name: why for name, why in found.items()
            if any(mark in line and '['+name+']' in line for line in tail.splitlines() for mark in ABORT_MARKS)}


def bringup_abort(log_text):
    """The first error line of a manager that gave up in the latest launch, else None."""
    managers = aborted_managers(log_text)
    return next(iter(managers.values()), None)


UNCONFIGURED, INACTIVE, ACTIVE = 1, 2, 3
CONFIGURE, ACTIVATE = 1, 3  # lifecycle_msgs/Transition ids


def read_tail(path, size=200000):
    try:
        with open(path, 'rb') as handle:
            handle.seek(0, 2)
            handle.seek(max(0, handle.tell() - size))
            return handle.read().decode('utf-8', 'replace')
    except OSError:
        return ''


def main():
    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    from lifecycle_msgs.srv import ChangeState
    from rcl_interfaces.srv import GetParameters

    rclpy.init(args=[])  # sys.argv carries the deadline, not ROS arguments
    node = rclpy.create_node('pinky_dashboard_nav_readiness')
    lifecycle_clients = {}

    def call(service_type, name, request, timeout):
        key = (service_type.__name__, name)
        if key not in lifecycle_clients:
            lifecycle_clients[key] = node.create_client(service_type, name)
        client = lifecycle_clients[key]
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        end = time.monotonic()+timeout
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        return future.result() if future.done() else None

    def primary_state(name, timeout=15.):
        """The node's primary lifecycle state, waiting out a transition in progress; None when it does not answer."""
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            answer = call(GetState, '/'+name+'/get_state', GetState.Request(), 10.)
            if answer is None:
                return None
            if answer.current_state.id <= 4:
                return answer.current_state.id
            time.sleep(.5)
        return None

    def finish_bringup(manager):
        """Configure, then activate, every node the manager was to bring up and has not; (ok, detail)."""
        answer = call(GetParameters, '/'+manager+'/get_parameters', GetParameters.Request(names=['node_names']), 15.)
        if answer is None or not answer.values:
            return False, manager+'의 node_names를 읽지 못했습니다'
        names = list(answer.values[0].string_array_value)
        for transition, before, after in ((CONFIGURE, UNCONFIGURED, INACTIVE), (ACTIVATE, INACTIVE, ACTIVE)):
            for name in names:
                state = primary_state(name)
                if state is None:
                    return False, name+' 상태를 읽지 못했습니다'
                if state == after or (after == INACTIVE and state == ACTIVE):
                    continue
                if state != before:
                    return False, f'{name} 상태가 {state}라 이어서 켤 수 없습니다'
                request = ChangeState.Request()
                request.transition.id = transition
                result = call(ChangeState, '/'+name+'/change_state', request, 40.)
                if result is None or not result.success:
                    return False, f'{name} {"configure" if transition == CONFIGURE else "activate"} 실패'
        return True, ', '.join(names)

    handled = set()
    names = ('map_server', 'amcl', 'controller_server', 'planner_server', 'bt_navigator', 'behavior_server')
    clients = {name: node.create_client(GetState, '/'+name+'/get_state') for name in names}
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    states = {}
    pending = {}
    # Raspberry Pi Pinkys bring the Nav2 lifecycle up slowly while the base, sensors and DDS discovery are busy
    # (controller active, the rest still inactive at 75 s on 2026-11-16).
    deadline = time.monotonic()+(float(sys.argv[1]) if len(sys.argv) > 1 else 150.)
    log_path = sys.argv[2] if len(sys.argv) > 2 else None
    next_log_check = 0.
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
            # The lifecycle manager activates the navigation nodes in order (controller, planner, behavior, then
            # bt_navigator) and aborts on any failure, so an active bt_navigator means the ones before it are active.
            # A fresh probe sometimes never discovers one node's get_state service (planner_server, 2026-11-16)
            # while Nav2 reports 'Managed nodes are active'; that must not fail the preparation.
            # amcl is left out (2026-11-16): a fresh probe sometimes never finds amcl/get_state either, while AMCL
            # runs; the preparation's next step gives AMCL a pose and fails unless AMCL answers it.
            required = ('map_server', 'bt_navigator')
            if all(states.get(name, (0,))[0] == 3 for name in required) and action.server_is_ready():
                print('NAV_ACTION_READY · lifecycle active', flush=True)
                return 0
            if log_path and time.monotonic() >= next_log_check:
                next_log_check = time.monotonic()+1.
                for manager, why in aborted_managers(read_tail(log_path)).items():
                    if manager in handled:
                        continue
                    handled.add(manager)
                    print(f'NAV_MANAGER_GAVE_UP {manager} · {why}', flush=True)
                    ok, detail = finish_bringup(manager)
                    if not ok:
                        print('NAV_BRINGUP_ABORTED '+json.dumps(states)+f' · {manager}: {why} · 이어서 켜기 실패: {detail}', flush=True)
                        return 2
                    print(f'NAV_BRINGUP_FINISHED {manager} · {detail}', flush=True)
            rclpy.spin_once(node, timeout_sec=.1)
        print('NAV_ACTION_MISSING_OR_INACTIVE '+json.dumps(states), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
