#!/usr/bin/env python3
"""운영자 정지: 최종 감시기를 잠그고, 관제가 띄운 경로 실행기만 멈추고, 이 도메인의 Nav2 목표를 취소한다.

실행 위치: 로봇 (app.py가 base64로 ~/.cache/pinky-fleet/stop_robot.py에 올리고 바로 실행한다)
사용처: app._cancel_waypoint_route(정지 버튼, 인자 = 진행 중이던 run_id)와 app._reset_robot(초기 세팅, 인자 "reset").

순서와 이유:
1. stop_latch.json을 먼저 쓴다. velocity_guard.py는 이 파일이 있는 동안 /cmd_vel을 0으로 묶고
   ultrasonic_safety.py도 이를 본다. 프로세스를 죽이기 전에 잠가야 그 사이 명령이 새어 나가지 않는다.
   잠금은 다음 navigate_route.py가 시작할 때 푼다.
2. route.pid의 프로세스가 정말 navigate_route.py일 때만 SIGINT(정리 기회) → 2초 뒤에도 살아 있으면 SIGTERM.
   PID 재사용으로 엉뚱한 프로세스를 죽이지 않도록 /proc/<pid>/cmdline을 확인한다.
3. /cmd_vel에 0을 0.5초 동안 20 Hz로 보내고, /navigate_to_pose 목표 전체 취소를 요청한다.
Nav2·AMCL·감시기는 그대로 두므로 다음 임무는 주행준비 없이 시작할 수 있다.
마지막 줄 `STOP_LATCHED {...}`를 app.py가 성공 표시로 읽는다.
"""
import json,os,signal,time,sys
from pathlib import Path
CACHE=Path('/home/pinky/.cache/pinky-fleet')
def main():
    CACHE.mkdir(parents=True,exist_ok=True)
    # 임시 파일에 쓰고 replace로 바꿔 넣는다(원자적 교체): 읽는 쪽이 반쯤 쓴 JSON을 보지 않는다.
    path=CACHE/'stop_latch.json';temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps({'reason':'operator_stop','run_id':sys.argv[1] if len(sys.argv)>1 else None}));temporary.replace(path)
    pidfile=CACHE/'route.pid'
    try:
        pid=int(pidfile.read_text());cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
        # 관제가 띄운 경로 실행기일 때만 신호를 보낸다. SIGINT를 받으면 navigate_route.py가 목표를 취소하고 끝낸다.
        if b'navigate_route.py' in cmd:
            os.kill(pid,signal.SIGINT)
            # 0.1 s × 20 = 최대 2초 동안 스스로 끝나기를 기다린다.
            for _ in range(20):
                if not Path(f'/proc/{pid}').exists():break
                time.sleep(.1)
            if Path(f'/proc/{pid}').exists() and b'navigate_route.py' in Path(f'/proc/{pid}/cmdline').read_bytes():os.kill(pid,signal.SIGTERM)
    # pidfile이 없거나 프로세스가 이미 끝났으면 넘어간다. 아래 0 속도·목표 취소는 어차피 한다.
    except (OSError,ValueError):pass
    import rclpy
    from rclpy.node import Node
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import Twist
    rclpy.init();node=Node('pinky_operator_stop')
    # /cmd_vel의 유일한 평소 발행자는 velocity_guard.py다. 여기서는 예외로 0 속도만 직접 보낸다.
    # 0은 어떤 명령과 섞여도 더 위험해지지 않고, 감시기가 이미 잠금으로 0을 내고 있으므로 이중 안전장치다.
    publisher=node.create_publisher(Twist,'/cmd_vel',10)
    timer=node.create_timer(.05,lambda:publisher.publish(Twist()))
    # 0.05 s 타이머로 0 Twist를 내보내며 약 0.5초 돈다.
    for _ in range(10):rclpy.spin_once(node,timeout_sec=.05)
    # 빈 CancelGoal 요청(goal_id 0, stamp 0)은 액션 서버의 모든 목표를 취소하라는 뜻이다.
    client=node.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
    canceled=False
    if client.wait_for_service(timeout_sec=2.):
        future=client.call_async(CancelGoal.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=2.)
        canceled=future.done() and future.exception() is None
    for _ in range(10):rclpy.spin_once(node,timeout_sec=.05)
    print('STOP_LATCHED '+json.dumps({'nav_cancel_service_replied':canceled}),flush=True)
    node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
