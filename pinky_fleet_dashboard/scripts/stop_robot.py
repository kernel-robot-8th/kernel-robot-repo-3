#!/usr/bin/env python3
"""Latch the velocity guard, stop only the managed runner and cancel this domain's goals."""
import json,os,signal,time,sys
from pathlib import Path
CACHE=Path('/home/pinky/.cache/pinky-fleet')
def main():
    CACHE.mkdir(parents=True,exist_ok=True)
    path=CACHE/'stop_latch.json';temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps({'reason':'operator_stop','run_id':sys.argv[1] if len(sys.argv)>1 else None}));temporary.replace(path)
    pidfile=CACHE/'route.pid'
    try:
        pid=int(pidfile.read_text());cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
        if b'navigate_route.py' in cmd:
            os.kill(pid,signal.SIGINT)
            for _ in range(20):
                if not Path(f'/proc/{pid}').exists():break
                time.sleep(.1)
            if Path(f'/proc/{pid}').exists() and b'navigate_route.py' in Path(f'/proc/{pid}/cmdline').read_bytes():os.kill(pid,signal.SIGTERM)
    except (OSError,ValueError):pass
    import rclpy
    from rclpy.node import Node
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import Twist
    rclpy.init();node=Node('pinky_operator_stop')
    publisher=node.create_publisher(Twist,'/cmd_vel',10)
    timer=node.create_timer(.05,lambda:publisher.publish(Twist()))
    for _ in range(10):rclpy.spin_once(node,timeout_sec=.05)
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
