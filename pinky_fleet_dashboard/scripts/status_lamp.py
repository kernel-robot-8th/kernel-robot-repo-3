#!/usr/bin/env python3
"""Robot side: show the fleet dashboard's state for this Pinky on its lamp and LED strip.

The dashboard publishes /fleet/indicator (std_msgs/String, JSON {"color": "green", "mode": "on"}) about twice a
second. This node turns the colour into the Pinky's own services: pinky_lamp_control's /set_lamp and pinky_led's
/set_led, whichever are running. With no message for SILENCE_S the dashboard is out of reach, and the lamp blinks
orange on its own. What was applied is reported on /fleet/indicator_status. The lamp is a signal for people
nearby; it is not a safety function.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String

try:
    from pinky_interfaces.srv import SetLamp, SetLed
except ImportError:  # an image without the Pinky interfaces: only the status is reported
    SetLamp = SetLed = None

COLORS = {"orange": (255, 110, 0), "green": (0, 255, 0), "blue": (0, 90, 255), "red": (255, 0, 0), "off": (0, 0, 0)}
SILENCE_S = 3.0
# pinky_lamp_control drives its WS2812B with strip_type WS2811_STRIP_GBR, so red and blue come out swapped
# (red showed blue on Pinky C76C, 2026-11-14). Swap them here instead of changing the Pinky's own package.
LAMP_SWAP_RED_BLUE = True
BLINK_MS = 500


class StatusLamp(Node):
    def __init__(self):
        super().__init__("pinky_status_lamp")
        self.wanted = ("orange", "blink")
        self.heard_at = 0.0
        self.applied = None
        self.lamp = self.create_client(SetLamp, "/set_lamp") if SetLamp else None
        self.led = self.create_client(SetLed, "/set_led") if SetLed else None
        self.status = self.create_publisher(String, "/fleet/indicator_status", 1)
        self.create_subscription(String, "/fleet/indicator", self.on_indicator,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.report)

    def on_indicator(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        color, mode = data.get("color"), data.get("mode", "on")
        if color in COLORS and mode in ("on", "blink"):
            self.wanted, self.heard_at = (color, mode), time.monotonic()

    def target(self):
        if time.monotonic() - self.heard_at > SILENCE_S:
            return ("orange", "blink")  # the dashboard went quiet
        return self.wanted

    def tick(self):
        target = self.target()
        if target == self.applied:
            return
        color, mode = target
        r, g, b = COLORS[color]
        sent = False
        if self.lamp is not None and self.lamp.service_is_ready():
            request = SetLamp.Request()
            lr, lb = (b, r) if LAMP_SWAP_RED_BLUE else (r, b)
            request.color = ColorRGBA(r=lr / 255.0, g=g / 255.0, b=lb / 255.0, a=1.0)
            request.mode = 2 if mode == "blink" else (0 if color == "off" else 1)
            request.time = BLINK_MS
            self.lamp.call_async(request)
            sent = True
        if self.led is not None and self.led.service_is_ready():
            request = SetLed.Request()
            request.command = "fill" if color != "off" else "clear"
            request.r, request.g, request.b = r, g, b
            self.led.call_async(request)
            sent = True
        if sent:
            self.applied = target

    def report(self):
        devices = [name for name, client in (("lamp", self.lamp), ("led", self.led))
                   if client is not None and client.service_is_ready()]
        color, mode = self.applied or (None, None)
        self.status.publish(String(data=json.dumps({"devices": devices, "color": color, "mode": mode,
                                                    "dashboard_heard": time.monotonic() - self.heard_at <= SILENCE_S})))


def main():
    rclpy.init()
    node = StatusLamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
