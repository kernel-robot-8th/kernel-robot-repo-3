#!/usr/bin/env python3
"""로봇 쪽: 관제가 본 이 핑키의 상태를 램프와 LED 띠 색으로 보여 준다.

실행 위치: 로봇 (runtime_bundle로 올라가고 app.py가 `status_lamp` 단계로 띄움). 램프·LED 패키지가 없어도 주행준비는 계속된다.
/fleet/indicator(JSON {"color", "mode"}, app.py `_indicator_loop`가 2026-11-24부터 초당 4번)를 받아
/set_lamp·/set_led 중 떠 있는 서비스로 보낸다. SILENCE_S 동안 조용하면 주황 깜박임. 램프는 안전 기능이 아니다.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String

try:
    from pinky_interfaces.srv import SetLamp, SetLed
except ImportError:  # 상태 보고만 한다
    SetLamp = SetLed = None

COLORS = {"orange": (255, 110, 0), "green": (0, 255, 0), "blue": (0, 90, 255), "red": (255, 0, 0), "off": (0, 0, 0)}
SILENCE_S = 3.0  # 초. 관제 메시지가 이만큼 없으면 연결 끊김으로 본다
# pinky_lamp_control이 WS2811_STRIP_GBR로 구동해 빨강·파랑이 뒤바뀐다(2026-11-14 C76C). 여기서 바꿔 보낸다.
LAMP_SWAP_RED_BLUE = True
BLINK_MS = 500  # ms


class StatusLamp(Node):
    """/fleet/indicator를 받아 바뀐 것이 있을 때만 램프·LED 서비스를 부른다."""
    def __init__(self):
        super().__init__("pinky_status_lamp")
        self.wanted = ("orange", "blink")
        self.heard_at = 0.0
        self.applied = None
        self.lamp = self.create_client(SetLamp, "/set_lamp") if SetLamp else None
        self.led = self.create_client(SetLed, "/set_led") if SetLed else None
        self.status = self.create_publisher(String, "/fleet/indicator_status", 1)
        # best effort·깊이 1: 최신 색만 의미가 있다.
        self.create_subscription(String, "/fleet/indicator", self.on_indicator,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.5, self.tick)
        self.create_timer(1.0, self.report)

    def on_indicator(self, msg):
        """알려진 색·모드일 때만 원하는 상태와 받은 시각을 고친다."""
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        color, mode = data.get("color"), data.get("mode", "on")
        if color in COLORS and mode in ("on", "blink"):
            self.wanted, self.heard_at = (color, mode), time.monotonic()

    def target(self):
        """지금 보여야 할 (색, 모드). 관제가 조용하면 주황 깜박임."""
        if time.monotonic() - self.heard_at > SILENCE_S:
            return ("orange", "blink")
        return self.wanted

    def tick(self):
        """0.5초마다: 보일 상태가 바뀌었으면 서비스를 비동기로 부른다(응답은 기다리지 않음)."""
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
            # mode: 0 끄기, 1 켜기, 2 깜박임
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
        # 둘 다 준비 전이면 다음 틱에 다시 시도한다.
        if sent:
            self.applied = target

    def report(self):
        """1초마다 /fleet/indicator_status로 장치·적용 상태·관제 수신 여부를 알린다."""
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
