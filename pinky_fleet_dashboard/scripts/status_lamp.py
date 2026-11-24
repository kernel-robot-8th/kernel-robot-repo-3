#!/usr/bin/env python3
"""로봇 쪽: 관제가 본 이 핑키의 상태를 핑키의 램프와 LED 띠에 색으로 보여 준다.

실행 위치: 로봇 (주행준비 때 runtime_bundle로 올라가고, app.py가 `status_lamp` 단계로 따로 띄움)
사용처: app.py `_indicator_loop`가 보내는 색(scripts/fleet_ops.indicator)을 받는다. 램프나 LED 패키지가 없는
핑키에서도 주행준비는 이것 없이 계속된다.

관제는 /fleet/indicator(std_msgs/String, JSON {"color": "green", "mode": "on"})를 초당 두 번쯤 보낸다(2026-11-24부터
app.py는 초당 네 번 보낸다). 이 노드는 그 색을 핑키 자체 서비스로 바꿔 보낸다: pinky_lamp_control의 /set_lamp와
pinky_led의 /set_led 중 떠 있는 것. SILENCE_S 동안 메시지가 없으면 관제와 연결이 끊긴 것으로 보고 램프가 스스로
주황색으로 깜박인다. 실제로 적용한 것은 /fleet/indicator_status로 알린다. 램프는 주변 사람에게 보내는 신호일 뿐
안전 기능이 아니다.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String

try:
    from pinky_interfaces.srv import SetLamp, SetLed
except ImportError:  # 핑키 인터페이스가 없는 이미지: 상태 보고만 한다
    SetLamp = SetLed = None

COLORS = {"orange": (255, 110, 0), "green": (0, 255, 0), "blue": (0, 90, 255), "red": (255, 0, 0), "off": (0, 0, 0)}
SILENCE_S = 3.0  # 초. 관제 메시지가 이만큼 없으면 연결 끊김으로 본다
# pinky_lamp_control은 WS2812B를 strip_type WS2811_STRIP_GBR로 구동해 빨강과 파랑이 뒤바뀌어 나온다
# (2026-11-14 핑키 C76C에서 빨강이 파랑으로 보였다). 핑키 자체 패키지를 고치지 않고 여기서 둘을 바꿔 보낸다.
LAMP_SWAP_RED_BLUE = True
BLINK_MS = 500  # 깜박임 주기(ms), /set_lamp의 time 필드


class StatusLamp(Node):
    """/fleet/indicator를 받아 램프·LED 서비스를 부르는 노드. 바뀐 것이 있을 때만 서비스를 부른다."""
    def __init__(self):
        super().__init__("pinky_status_lamp")
        # wanted: 관제가 마지막으로 원한 (색, 모드). applied: 서비스에 실제로 보낸 것(None이면 아직 없음).
        self.wanted = ("orange", "blink")
        self.heard_at = 0.0
        self.applied = None
        self.lamp = self.create_client(SetLamp, "/set_lamp") if SetLamp else None
        self.led = self.create_client(SetLed, "/set_led") if SetLed else None
        self.status = self.create_publisher(String, "/fleet/indicator_status", 1)
        # 관제는 best effort, 깊이 1로 보낸다. 램프는 최신 색만 의미가 있어 잃은 메시지는 다시 받을 필요가 없다.
        self.create_subscription(String, "/fleet/indicator", self.on_indicator,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.5, self.tick)
        self.create_timer(1.0, self.report)

    def on_indicator(self, msg):
        """알려진 색과 모드("on"·"blink")일 때만 원하는 상태와 받은 시각을 고친다."""
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        color, mode = data.get("color"), data.get("mode", "on")
        if color in COLORS and mode in ("on", "blink"):
            self.wanted, self.heard_at = (color, mode), time.monotonic()

    def target(self):
        """지금 보여야 할 (색, 모드). 관제가 SILENCE_S 넘게 조용하면 주황 깜박임."""
        if time.monotonic() - self.heard_at > SILENCE_S:
            return ("orange", "blink")  # 관제가 조용해졌다
        return self.wanted

    def tick(self):
        """0.5초마다: 보일 상태가 바뀌었으면 준비된 램프·LED 서비스를 비동기로 부른다. 응답은 기다리지 않는다."""
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
            # /set_lamp mode: 0 끄기, 1 켜기, 2 깜박임(BLINK_MS 주기)
            request.mode = 2 if mode == "blink" else (0 if color == "off" else 1)
            request.time = BLINK_MS
            self.lamp.call_async(request)
            sent = True
        if self.led is not None and self.led.service_is_ready():
            request = SetLed.Request()
            # LED 띠는 깜박임 없이 한 색으로 채운다("fill") 또는 끈다("clear"). 0~255 정수 RGB.
            request.command = "fill" if color != "off" else "clear"
            request.r, request.g, request.b = r, g, b
            self.led.call_async(request)
            sent = True
        # 둘 다 준비되지 않았으면 applied를 그대로 두어 다음 틱에 다시 시도한다.
        if sent:
            self.applied = target

    def report(self):
        """1초마다 /fleet/indicator_status로 쓸 수 있는 장치, 적용한 색·모드, 관제 수신 여부를 알린다."""
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
