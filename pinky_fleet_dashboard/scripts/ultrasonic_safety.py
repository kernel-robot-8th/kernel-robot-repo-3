#!/usr/bin/env python3
"""Ultrasonic stop/turn/creep gate between Nav2 and Pinky's base."""

from __future__ import annotations

import math
import time
import json
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import UInt16MultiArray, String
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from tf2_ros import Buffer, TransformListener, TransformException
if __package__:
    from .line_following import LineAssist, default_profile, swept_clear, TemplateEstimator, validate_profile, normalize, surface_observation
    from .mission_behaviors import Crossing, Docking
else:
    from line_following import LineAssist, default_profile, swept_clear, TemplateEstimator, validate_profile, normalize, surface_observation
    from mission_behaviors import Crossing, Docking


class UltrasonicSafety(Node):
    def __init__(self) -> None:
        super().__init__("pinky_ultrasonic_safety")
        self.declare_parameter("stop_distance_m", 0.10)
        self.declare_parameter("clear_distance_m", 0.15)
        self.declare_parameter("clear_hold_s", 0.75)
        self.declare_parameter("sensor_timeout_s", 0.60)
        self.declare_parameter("turn_speed_rad_s", 0.25)
        self.declare_parameter("creep_speed_m_s", 0.03)
        self.declare_parameter("creep_duration_s", 0.50)
        self.nav_command = Twist()
        self.last_command_at = 0.0
        self.range_m: float | None = None
        self.sensor_clear = False
        self.sensor_valid = False
        self.last_sensor_at = 0.0
        self.state = "idle"
        self.state_since = time.monotonic()
        self.clear_since: float | None = None
        self.ir_profile_path = Path('/home/pinky/.cache/pinky-fleet/nav2/ir_profile.json')
        self.ir_profile = default_profile()
        self.profile_error = ''
        self.profile_signature = None
        self.ir_raw = []
        self.last_ir_at = 0.0
        self.scan_points = []
        self.last_scan_at = 0.0
        self.line_assist = LineAssist()
        self.start_request_path = Path('/home/pinky/.cache/pinky-fleet/route_start.json')
        self.start_token = None
        self.start_deadline = 0.0
        self.start_elapsed = 0.0
        self.start_last_tick = None
        self.line_state = 'disabled'
        self.line_normalized = None
        self.line_error = None
        self.line_estimate = None
        self.observer_model = None
        self.observer_revision = None
        self.route_context_seen = False
        self.segment_key = None
        self.was_corner = False
        self.last_motion_at = 0.
        self.crossing = Crossing()
        self.docking = Docking()
        self.mission_token = None
        self.dock_saved_signature = None
        self.last_crossing_log = None
        self.restore_dock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.line_status_publisher = self.create_publisher(String, '/line/status', 10)
        self.create_subscription(UInt16MultiArray, '/ir_sensor/range', self.on_ir, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.reload_ir_profile)
        self.create_timer(0.2, self.publish_line_status)
        self.reload_ir_profile()
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel_nav2", self.on_command, 10)
        self.create_subscription(Range, "/us_sensor/range", self.on_range, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info("Ultrasonic safety gate ready; stale sensor data stops the base")

    def reload_ir_profile(self) -> None:
        try:
            if not self.ir_profile_path.exists():
                return
            signature = self.ir_profile_path.stat().st_mtime_ns
            if signature == self.profile_signature:
                return
            profile = json.loads(self.ir_profile_path.read_text(encoding='utf-8'))
            if not isinstance(profile, dict) or not isinstance(profile.get('enabled'), bool):
                raise ValueError('Invalid IR profile')
            self.ir_profile = profile
            self.profile_signature = signature
            self.profile_error = ''
            self.line_assist.reset()
            self.get_logger().info(f"IR profile loaded, enabled={profile['enabled']}")
        except (OSError, ValueError) as exc:
            self.profile_error = str(exc)

    def on_ir(self, msg: UInt16MultiArray) -> None:
        self.ir_raw = list(msg.data) if len(msg.data) == 3 else []
        self.last_ir_at = time.monotonic()
        strengths = normalize(self.ir_raw, self.ir_profile)
        self.docking.observe(strengths is not None and min(strengths) >= .5, self.last_ir_at)

    def on_scan(self, msg: LaserScan) -> None:
        try:
            if not msg.header.frame_id or not msg.ranges:
                return
            transform = self.tf_buffer.lookup_transform('base_footprint', msg.header.frame_id.lstrip('/'), Time()).transform
            q, t = transform.rotation, transform.translation
            m00, m01 = 1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.z*q.w)
            m10, m11 = 2*(q.x*q.y+q.z*q.w), 1-2*(q.x*q.x+q.z*q.z)
            points, usable = [], 0
            for i, distance in enumerate(msg.ranges):
                if math.isinf(distance) and distance > 0:
                    usable += 1
                elif math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                    usable += 1
                    angle = msg.angle_min+i*msg.angle_increment
                    x, y = distance*math.cos(angle), distance*math.sin(angle)
                    points.append((m00*x+m01*y+t.x, m10*x+m11*y+t.y))
            if usable < len(msg.ranges)*0.5:
                return
            self.scan_points = points
            self.last_scan_at = time.monotonic()
        except TransformException:
            # An enabled IR correction requires scan points in the base frame.
            return

    def publish_line_status(self) -> None:
        now = time.monotonic()
        state = self.line_state
        if self.line_state.startswith(('docking_', 'crossing_', 'mission_', 'operator_')):
            state = self.line_state
        elif self.line_state == 'route_start_straight' and self.state == 'idle' and now-self.last_command_at <= .5:
            state = 'route_start_straight'
        elif not self.ir_profile.get('enabled'):
            state = 'disabled'
        elif self.profile_error:
            state = 'profile_error'
        elif not self.sensor_fresh(now) or not self.sensor_valid:
            state = 'ultrasonic_unavailable'
        elif self.state != 'idle':
            state = 'ultrasonic_'+self.state
        elif now-self.last_command_at > .5:
            state = 'idle'
        estimate = None
        if self.ir_profile.get('mode') == 'template' and not validate_profile(self.ir_profile) and now-self.last_ir_at <= .6 and len(self.ir_raw) == 3:
            if self.observer_revision != self.ir_profile.get('revision'):
                self.observer_model = TemplateEstimator(self.ir_profile['template'])
                self.observer_revision = self.ir_profile.get('revision')
            estimate = self.observer_model.estimate([self.ir_raw[i] for i in self.ir_profile['channel_order']])
        status = dict(version=1, policy='center_first', line_algorithm='three_surface_pd_v2', recovery_policy='parking_mission_v2', crossing=self.crossing.status(), docking=self.docking.status(), classification=self.ir_profile.get('mode'), estimate=estimate, control_estimate=self.line_estimate, cruise_speed_mps=.1152, enabled=bool(self.ir_profile.get('enabled')), state=state,
                      revision=self.ir_profile.get('revision'), normalized=normalize(self.ir_raw,self.ir_profile) if now-self.last_ir_at<=.6 else None, surface_reading=surface_observation(self.ir_raw,self.ir_profile) if self.ir_profile.get('mode')=='calibrated' and now-self.last_ir_at<=.6 else None,
                      error=self.line_error, ir_fresh=now-self.last_ir_at <= .6,
                      lidar_fresh=now-self.last_scan_at <= .6,
                      ultrasonic_fresh=self.sensor_fresh(now) and self.sensor_valid, profile_error=self.profile_error)
        self.line_status_publisher.publish(String(data=json.dumps(status, allow_nan=False)))
        self.write_json('route_feedback.json', dict(token=self.mission_token, monotonic=now,
                        crossing=self.crossing.status(), docking=self.docking.status(), state=self.line_state))

    def write_json(self, name, data):
        path = Path('/home/pinky/.cache/pinky-fleet')/name
        try:
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(data, allow_nan=False))
            os.replace(temporary, path)
        except OSError as exc:
            self.get_logger().error(f'Mission state write failed: {exc}')
            return False
        return True

    def restore_dock(self):
        try:
            data = json.loads(Path('/home/pinky/.cache/pinky-fleet/dock_latch.json').read_text())
            if data.get('state') not in Docking.ACTIVE:
                return
            for key in ('token','dock_id','complete','origin','heading','reversed_once','command_id','message'):
                if key in data:
                    setattr(self.docking, key, data[key])
            self.docking.handled = set(data.get('handled', []))
            self.docking.state = 'stopped' if data['state'] == 'stopped' else 'recovery_hold'
        except (OSError, ValueError, TypeError):
            pass

    def fresh_pose(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(frame, 'base_footprint', Time())
            age = (self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
            if -.2 <= age <= .6:
                t, q = tf.transform.translation, tf.transform.rotation
                pose = t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                if all(math.isfinite(v) for v in pose):
                    return pose
        except TransformException:
            pass
        return None

    def mission_tick(self, now):
        try:
            context = json.loads(Path('/home/pinky/.cache/pinky-fleet/route_context.json').read_text())
            valid = 0 <= now-float(context['monotonic']) < 1.5 and 'start_xy' in context
        except (OSError, ValueError, TypeError, KeyError):
            valid = False
        if not valid:
            if self.docking.state in Docking.ACTIVE:
                self.line_state = 'route_context_stale'
                self.send()
                return True
            return False
        if context.get('mission_wait'):
            self.line_state = 'mission_wait'
            self.send()
            return True
        if context.get('transit') or not context.get('use_ir', True):
            return False
        self.mission_token = context['token']
        if self.docking.token != context['token']:
            self.docking.reset(context['token'])
        pose, odom = self.fresh_pose('map'), self.fresh_pose('odom')
        if pose is None or odom is None or now-self.last_ir_at > .6 or now-self.last_scan_at > .6:
            self.line_state = 'mission_sensor_unavailable'
            self.send()
            return True
        strengths = normalize(self.ir_raw, self.ir_profile)
        if strengths is None:
            self.line_state = 'profile_error'
            self.send()
            return True
        all_white = min(strengths) >= .5
        self.docking.observe(all_white, self.last_ir_at)
        try:
            command = json.loads(Path('/home/pinky/.cache/pinky-fleet/dock_command.json').read_text())
            if 0 <= now-float(command.get('monotonic', -100.)) <= 3.:
                self.docking.command(command)
        except (OSError, ValueError, TypeError):
            pass
        front_blocked = self.range_m is not None and self.range_m <= float(self.get_parameter('stop_distance_m').value)
        motion = self.docking.update(context, pose, odom, now, front_blocked)
        if self.docking.state in Docking.ACTIVE:
            self.crossing.reset(context['token'])
            self.state, self.clear_since = 'idle', None
        signature = (self.docking.state, self.docking.dock_id, self.docking.complete, self.docking.command_id)
        if signature != self.dock_saved_signature:
            data = self.docking.status()
            data.update(origin=self.docking.origin, heading=self.docking.heading, reversed_once=self.docking.reversed_once)
            if self.write_json('dock_latch.json', data):
                self.dock_saved_signature = signature
            elif self.docking.state in Docking.ACTIVE:
                self.docking.state = 'fault'
                motion = type(motion)(0., 0., 'docking_fault') if motion else None
        if motion is None and self.docking.state == 'released' and context.get('end_label') == '5':
            self.line_state = 'docking_released'
            self.send()
            return True
        if motion is None:
            if self.state != 'idle' or front_blocked:
                return False  # US stop/turn/creep remains the higher priority.
            if now-self.last_command_at > .5 or (abs(self.nav_command.linear.x) <= .001 and abs(self.nav_command.angular.z) <= .001):
                return False
            if self.ir_profile.get('mode') == 'template':
                if self.observer_model is None or self.observer_revision != self.ir_profile.get('revision'):
                    self.observer_model = TemplateEstimator(self.ir_profile['template'])
                    self.observer_revision = self.ir_profile.get('revision')
                estimate = self.observer_model.estimate([self.ir_raw[i] for i in self.ir_profile['channel_order']])
                line_ok = estimate['usable'] and abs(estimate['offset_cm']) <= .5
            else:
                left, right = strengths[0] >= .55, strengths[2] >= .55
                line_ok = strengths[1] >= .55 and not (left and right)
            motion = self.crossing.update(context, pose, odom, line_ok, now,
                                          self.nav_command.linear.x, self.nav_command.angular.z,
                                          ir_required=bool(self.ir_profile.get('enabled')))
            log_key = (self.crossing.token, self.crossing.state, self.crossing.phase, self.crossing.failure_reason)
            if log_key != self.last_crossing_log:
                self.get_logger().info('Crossing transition: '+json.dumps(self.crossing.status()))
                self.last_crossing_log = log_key
        if motion is None:
            return False
        self.line_state = motion.state
        if motion.state == 'crossing_reacquire':
            # Preserve the same recovery memory across crossing ticks and the
            # handoff to straight-line tracking, including the segment change.
            self.segment_key = (context.get('token'), context.get('segment'))
            decision = self.line_assist.decide(.015, 0., self.ir_raw,
                self.ir_profile, now, True, True, self.scan_points,
                route_corner=False, heading_yaw=odom[2])
            self.line_normalized, self.line_error = decision.normalized, decision.error
            self.line_estimate = decision.estimate
            self.line_state = decision.state
            motion.linear, motion.angular = decision.linear, decision.angular
            if decision.state == 'nav2_avoidance':
                # A synthetic forward request is not an obstacle avoidance path.
                motion.linear, motion.angular = 0., 0.
                self.line_state = 'lidar_stop'
            if decision.state == 'line_search_failed':
                self.crossing.state = 'fault'
                self.crossing.failure_reason = 'line_search_failed'
        else:
            self.line_assist.reset()
        # No Nav2 freshness is needed during explicit docking, whose route
        # heartbeat remains live while its action is intentionally cancelled.
        if not swept_clear(self.scan_points, motion.linear, motion.angular):
            self.line_state = 'lidar_stop'
            self.send()
        else:
            self.send(motion.linear, motion.angular)
        return True

    def on_command(self, msg: Twist) -> None:
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.nav_command, self.last_command_at = Twist(), 0.
            return
        self.nav_command = msg
        self.last_command_at = time.monotonic()

    def on_range(self, msg: Range) -> None:
        value = float(msg.range)
        self.last_sensor_at = time.monotonic()
        self.range_m = value if math.isfinite(value) else None
        self.sensor_valid = (math.isinf(value) and value > 0.0) or (
            math.isfinite(value) and msg.min_range <= value <= msg.max_range
        )
        if not self.sensor_valid:
            self.state = "sensor_fault"
            self.clear_since = None
            return
        if self.state == "sensor_fault":
            self.state = "idle" if self.sensor_clear else "stopped"
            self.state_since = time.monotonic()
        # Range drivers commonly use +inf to mean no return; accept that as
        # clear, while NaN remains invalid and therefore fails closed.
        self.sensor_clear = (math.isinf(value) and value > 0.0) or (
            math.isfinite(value) and value >= float(self.get_parameter("clear_distance_m").value)
        )
        stop_distance = float(self.get_parameter("stop_distance_m").value)
        nav_active = time.monotonic()-self.last_command_at <= .5 and (abs(self.nav_command.linear.x) > .001 or abs(self.nav_command.angular.z) > .001)
        if math.isfinite(value) and value <= stop_distance and self.state == "idle" and nav_active:
            self.state = "stopped"
            self.state_since = time.monotonic()
            self.clear_since = None
            self.get_logger().warn(f"Obstacle at {value:.2f} m: stopping before turn/creep sequence")

    def sensor_fresh(self, now: float) -> bool:
        timeout = float(self.get_parameter("sensor_timeout_s").value)
        return self.last_sensor_at > 0.0 and now - self.last_sensor_at <= timeout

    def send(self, linear: float = 0.0, angular: float = 0.0) -> None:
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.publisher.publish(cmd)

    def forward_nav_command(self) -> Twist:
        safe = Twist()
        safe.linear.x = max(-0.12, min(0.12, self.nav_command.linear.x))
        safe.angular.z = max(-0.30, min(0.30, self.nav_command.angular.z))
        now = time.monotonic()
        route_corner = False
        try:
            context = json.loads(Path('/home/pinky/.cache/pinky-fleet/route_context.json').read_text())
            self.route_context_seen = True
            if not 0 <= now-float(context['monotonic']) < 1.5:
                self.line_state = 'route_context_stale'
                return Twist()
            if context.get('transit') or not context.get('use_ir',True):
                self.line_assist.reset()
                if now-self.last_scan_at > .6:
                    self.line_state='lidar_unavailable'
                    return Twist()
                if not swept_clear(self.scan_points,safe.linear.x,safe.angular.z):
                    self.line_state='lidar_stop'
                    return Twist()
                self.line_state='parking_transit' if context.get('transit') else 'nav2_only'
                return safe
            route_corner = False  # Crossing owns the latched transition now.
            key = (context.get('token'), context.get('segment'))
            if key != self.segment_key or (self.was_corner and not route_corner):
                self.line_assist.reset()
                self.segment_key = key
            self.was_corner = route_corner
        except (OSError, ValueError, TypeError, KeyError):
            self.line_state = 'route_context_stale'
            return Twist()
        # Explicitly armed once per route, independent of IR enable/detection.
        # The outer tick still requires a fresh nonzero Nav2 command and US.
        try:
            request = json.loads(self.start_request_path.read_text())
            token = request['token']
            requested = float(request['monotonic'])
            if token != self.start_token:
                self.start_token = token
                self.start_deadline = requested + 10.0
                self.start_elapsed = 0.0
                self.start_last_tick = None
        except (OSError, ValueError, KeyError, TypeError):
            self.start_deadline = 0.0
        if now < self.start_deadline and self.start_elapsed < 1.0:
            if now-self.last_scan_at > .6:
                self.line_state = 'lidar_unavailable'
                self.start_last_tick = None
                return Twist()
            if not swept_clear(self.scan_points, .03, 0.0):
                self.line_state = 'lidar_stop'
                self.start_last_tick = None
                return Twist()
            if self.start_last_tick is not None:
                self.start_elapsed += min(.05, now-self.start_last_tick)
            self.start_last_tick = now
            if self.start_elapsed < 1.0:
                self.line_state = 'route_start_straight'
                safe.linear.x, safe.angular.z = .03, 0.0
                return safe
        if self.ir_profile.get('enabled') and self.profile_error:
            self.line_state = 'profile_error'
            return Twist()
        heading_yaw = None
        try:
            odom_tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', Time())
            age = (self.get_clock().now()-Time.from_msg(odom_tf.header.stamp)).nanoseconds/1e9
            if -.2 <= age <= .6:
                q = odom_tf.transform.rotation
                heading_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        except TransformException:
            pass
        decision = self.line_assist.decide(safe.linear.x, safe.angular.z, self.ir_raw,
            self.ir_profile, now, now-self.last_ir_at <= .6,
            now-self.last_scan_at <= .6, self.scan_points, route_corner=route_corner, heading_yaw=heading_yaw)
        self.line_state, self.line_normalized, self.line_error = decision.state, decision.normalized, decision.error
        self.line_estimate = decision.estimate
        safe.linear.x, safe.angular.z = decision.linear, decision.angular
        return safe

    def tick(self) -> None:
        now = time.monotonic()
        if Path('/home/pinky/.cache/pinky-fleet/stop_latch.json').exists():
            self.nav_command=Twist()
            self.last_command_at=0.
            self.state='idle'
            self.line_state='operator_stopped'
            self.send()
            return
        if not self.sensor_fresh(now) or not self.sensor_valid:
            self.start_last_tick = None
            if self.state == "turning":
                self.clear_since = None
            self.send()
            return

        if self.mission_tick(now):
            return

        if now-self.last_command_at > .5 or (abs(self.nav_command.linear.x) <= .001 and abs(self.nav_command.angular.z) <= .001):
            self.start_last_tick = None
            self.state = "idle"
            self.clear_since = None
            self.line_state = "idle"
            if now-self.last_motion_at > 1.:
                self.line_assist.reset()
                self.line_estimate = None
            else:
                self.line_assist.offset_at = None
                self.line_assist.previous_yaw = None
            self.send()
            return

        self.last_motion_at = now

        # A cancelled/dead route must also stop an in-progress US recovery,
        # even if an orphaned Nav2 action keeps publishing commands.
        try:
            context = json.loads(Path('/home/pinky/.cache/pinky-fleet/route_context.json').read_text())
            valid_context = 0 <= now-float(context['monotonic']) < 1.5
        except (OSError, ValueError, TypeError, KeyError):
            valid_context = False
        if not valid_context:
            self.state = 'idle'
            self.line_state = 'route_context_stale'
            self.start_last_tick = None
            self.send()
            return

        if self.state == "idle":
            if self.last_command_at > 0.0 and now - self.last_command_at <= 0.50:
                self.publisher.publish(self.forward_nav_command())
            else:
                self.send()
            return

        if self.state == "stopped":
            self.send()
            if now - self.state_since >= 0.50:
                self.state = "turning"
                self.state_since = now
                self.clear_since = None
            return

        if self.state == "turning":
            if self.sensor_clear:
                if self.clear_since is None:
                    self.clear_since = now
                hold = float(self.get_parameter("clear_hold_s").value)
                if now - self.clear_since >= hold:
                    self.state = "creeping"
                    self.state_since = now
                    self.send()
                    return
            else:
                self.clear_since = None
            turn_speed = float(self.get_parameter("turn_speed_rad_s").value)
            self.send(angular=turn_speed)
            return

        if self.state == "creeping":
            value = self.range_m
            stop_distance = float(self.get_parameter("stop_distance_m").value)
            if value is not None and value <= stop_distance:
                self.state = "stopped"
                self.state_since = now
                self.clear_since = None
                self.send()
                return
            duration = float(self.get_parameter("creep_duration_s").value)
            if now - self.state_since >= duration:
                self.state = "idle"
                self.get_logger().info("Low-speed creep complete; returning control to Nav2")
                self.send()
                return
            speed = float(self.get_parameter("creep_speed_m_s").value)
            self.send(linear=speed)


def main() -> None:
    rclpy.init()
    node = UltrasonicSafety()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.send()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
