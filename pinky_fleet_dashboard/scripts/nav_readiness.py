#!/usr/bin/env python3
"""이동 목표를 보내지 않고 Nav2 lifecycle 노드가 모두 활성(active)이 됐는지 확인한다.

실행 위치: 로봇 (app._nav_action_ready가 base64로 실어 python3 -c로 exec한다. 파일로 남지 않는다)
사용처: 주행준비의 Nav2 단계. app.py는 표준 출력의 NAV_* 표시 줄과 종료 코드로 성공·재시작을 정한다.

argv: [마감 시간(초)] [Nav2 로그 경로]. Jazzy 1.3의 lifecycle manager는 노드의 get_state 응답을 고정 2초만
기다리고, 한 번이라도 놓치면 bringup 전체를 포기한다. 베이스·안전 노드·lifecycle manager 두 개를 한꺼번에 띄우느라
바쁜 Raspberry Pi에서는 노드 자체는 멀쩡한데도 이 응답을 자주 놓친다(planner_server, amcl, 2026-11-16).
로그가 주어지면 이 프로브는 manager가 포기한 것을 보고 그 manager의 bringup을 직접 마무리한다: 그 manager의
node_names를 자기 순서대로 configure한 뒤 activate하며, 호출마다 Pi가 필요한 만큼 기다린다
(NAV_MANAGER_GAVE_UP, NAV_BRINGUP_FINISHED). 그것마저 실패했을 때만 NAV_BRINGUP_ABORTED를 마감까지 기다리지
않고 바로 알리고, 관제는 Nav2를 다시 띄운다.

종료 코드: 0 = 준비됨(NAV_ACTION_READY), 1 = 마감까지 준비 안 됨, 2 = bringup 포기 후 이어서 켜기도 실패.
"""
import json
import re
import sys
import time

START_MARK = '=== PINKY_FLEET_START'  # 관제가 launch마다 로그에 먼저 쓰는 표시. 로그는 이어 쓰기라 이 표시로 이번 launch를 가른다
# lifecycle manager가 bringup을 포기할 때 Nav2 로그에 남기는 문구.
ABORT_MARKS = ('Aborting bringup', 'Failed to bring up all requested nodes')


def aborted_managers(log_text):
    """최근 launch에서 bringup을 포기한 manager마다 {lifecycle manager 이름: 첫 오류 줄}을 돌려준다."""
    start = log_text.rfind(START_MARK)
    tail = log_text[start:] if start >= 0 else log_text
    found = {}
    # 먼저 manager별 첫 오류 줄을 모으고(300자까지), 아래에서 실제로 포기 문구까지 남긴 manager만 남긴다.
    for line in tail.splitlines():
        match = re.search(r'\[(lifecycle_manager\w*)\]: (Failed to change state.*|Failed to bring up all requested nodes.*)', line)
        if match and match.group(1) not in found:
            found[match.group(1)] = match.group(2).strip()[-300:]
    return {name: why for name, why in found.items()
            if any(mark in line and '['+name+']' in line for line in tail.splitlines() for mark in ABORT_MARKS)}


def bringup_abort(log_text):
    """최근 launch에서 포기한 manager의 첫 오류 줄. 없으면 None."""
    managers = aborted_managers(log_text)
    return next(iter(managers.values()), None)


# lifecycle_msgs/State의 primary state id. 0=UNKNOWN, 4=FINALIZED, 10 이상은 전이 중인 상태다.
UNCONFIGURED, INACTIVE, ACTIVE = 1, 2, 3
CONFIGURE, ACTIVATE = 1, 3  # lifecycle_msgs/Transition id


def read_tail(path, size=200000):
    """로그 파일의 마지막 size 바이트(기본 200 kB)를 문자열로. 파일이 없으면 빈 문자열."""
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

    rclpy.init(args=[])  # sys.argv에는 ROS 인자가 아니라 마감 시간이 들어 있다
    node = rclpy.create_node('pinky_dashboard_nav_readiness')
    lifecycle_clients = {}

    def call(service_type, name, request, timeout):
        """서비스를 한 번 동기 호출한다(클라이언트는 재사용). 서비스가 없거나 timeout 초 안에 답이 없으면 None."""
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
        """노드의 primary lifecycle state. 전이 중(id > 4)이면 끝날 때까지 기다린다. 응답이 없으면 None."""
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
        """manager가 켜야 했지만 못 켠 노드를 모두 configure한 뒤 activate한다. (ok, 설명)을 돌려준다.

        노드 목록은 manager의 node_names 파라미터에서 읽는다. 이미 목표 상태면 건너뛰고, 예상 밖 상태면 멈춘다.
        change_state는 40초까지 기다린다: Pi에서는 configure가 manager의 2초 한도보다 훨씬 오래 걸릴 수 있다."""
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

    handled = set()  # 이미 이어서 켜기를 시도한 manager (같은 manager를 두 번 손대지 않는다)
    names = ('map_server', 'amcl', 'controller_server', 'planner_server', 'bt_navigator', 'behavior_server',
             'velocity_smoother')
    bt_active_at = None  # bt_navigator가 처음 active로 보인 시각 (monotonic s)
    finished_tail = False  # velocity_smoother를 위해 이어서 켜기를 이미 한 번 했는지
    clients = {name: node.create_client(GetState, '/'+name+'/get_state') for name in names}
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    states = {}  # 노드 이름 → (state id, label). 응답 오류는 (0, 오류 문자열)
    pending = {}  # 노드 이름 → 아직 답을 기다리는 get_state future (노드마다 한 번에 하나만 보낸다)
    # Raspberry Pi 핑키는 베이스·센서·DDS 디스커버리로 바쁜 동안 Nav2 lifecycle을 느리게 올린다
    # (2026-11-16, 75초 시점에 controller만 active이고 나머지는 아직 inactive). 그래서 기본 마감이 150초다.
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
            # 상태가 바뀔 때만 한 줄 찍는다. app.py가 진행 상황을 화면에 보여 준다.
            summary = json.dumps(states, sort_keys=True)
            if summary != previous:
                print('NAV_LIFECYCLE '+summary, flush=True)
                previous = summary
            # lifecycle manager는 주행 노드를 순서대로(controller, planner, behavior, 그다음 bt_navigator) activate하고
            # 하나라도 실패하면 중단한다. 그래서 bt_navigator가 active면 그 앞의 노드도 active다.
            # 새로 뜬 프로브가 어떤 노드의 get_state 서비스를 끝내 못 찾는 일이 있는데(planner_server, 2026-11-16)
            # 그동안 Nav2는 'Managed nodes are active'를 알린다. 이것 때문에 준비가 실패하면 안 된다.
            # amcl도 뺀다(2026-11-16): AMCL이 돌고 있어도 amcl/get_state를 못 찾는 일이 있다. 준비의 다음 단계가
            # AMCL에 위치를 주고, AMCL이 응답하지 않으면 그 단계가 실패하므로 거기서 걸러진다.
            required = ('map_server', 'bt_navigator')
            ready = all(states.get(name, (0,))[0] == 3 for name in required) and action.server_is_ready()
            # velocity smoother는 manager가 bt_navigator 다음, 마지막으로 activate하는 노드다. bt_navigator에서 포기한
            # manager(activate는 됐는데 get_state 답이 늦게 옴)는 smoother를 inactive로 남긴다. 그러면 Nav2는 경로를
            # 계획하는데 바퀴에는 아무 명령도 가지 않는다(핑키 12, 2026-11-17). 그래서 준비 완료에는 smoother active도
            # 필요하고, bt_navigator 뒤 12초가 지나도 inactive이면 이 프로브가 manager의 남은 노드를 직접 켠다.
            # bt_navigator 뒤 40초가 지나도 smoother가 active로 안 보이면 더는 막지 않고 준비 완료로 넘어간다.
            if ready:
                bt_active_at = bt_active_at or time.monotonic()
                smoother = states.get('velocity_smoother', (0,))[0]
                if smoother != 3 and time.monotonic()-bt_active_at < 40.:
                    if smoother in (1, 2) and time.monotonic()-bt_active_at > 12. and not finished_tail:
                        finished_tail = True
                        ok, detail = finish_bringup('lifecycle_manager_navigation')
                        print(('NAV_BRINGUP_FINISHED ' if ok else 'NAV_BRINGUP_FINISH_FAILED ')+'lifecycle_manager_navigation · '+detail, flush=True)
                    ready = False
                    # smoother 상태를 이미 한 번 받았으면 기다리던 요청을 버려 다음 루프에서 새로 묻게 한다.
                    pending.pop('velocity_smoother', None) if states.get('velocity_smoother') else None
            if ready:
                print('NAV_ACTION_READY · lifecycle active', flush=True)
                return 0
            # 1초마다 Nav2 로그 끝부분을 읽어 bringup을 포기한 manager가 있는지 본다.
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
