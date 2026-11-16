#!/usr/bin/env python3
"""Robot side: show which Pinky this is, and its state, on the Pinky's own screen.

Big: the name the control screens use (P0, P1, P2) with the robot id under it. Background: the same colour as the
status lamp (scripts/status_lamp.py), from the dashboard's /fleet/indicator. A short English word for the state,
since the robot may have no Korean font. With no indicator for SILENCE_S the dashboard is out of reach: NO LINK.

The screen is driven through pinkylib's LCD, whose drawing call was not confirmed on this PC (2026-11-16), so the
known names are tried in turn and what worked is reported on /fleet/display_status. A screen problem never
affects driving: this node only draws.
"""
import json
import time

COLORS = {"orange": (255, 110, 0), "green": (0, 170, 60), "blue": (0, 90, 255), "red": (220, 0, 0)}
WORDS = {"orange": "NO POSE", "green": "DRIVING", "blue": "READY", "red": "STOP"}
NAMES = {"10": "P0", "11": "P1", "12": "P2"}  # the names on the control screens (control_station.store.PINKY_NAMES)
SILENCE_S = 3.0
FALLBACK_SIZE = (320, 240)
FONT_PATHS = ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf")


def screen_state(wanted, heard_at, now):
    """(colour, word) to show: the dashboard's colour, or NO LINK once it has been quiet for SILENCE_S."""
    if wanted is None or now - heard_at > SILENCE_S:
        return "orange", "NO LINK"
    color, mode = wanted
    return color, ("NO LINK" if mode == "blink" else WORDS.get(color, color.upper()))


def render(size, name, robot_id, color, word):
    """A PIL image: name big in the middle, '#id' and the state word below, on the state colour."""
    from PIL import Image, ImageDraw, ImageFont

    def font(px):
        for path in FONT_PATHS:
            try:
                return ImageFont.truetype(path, px)
            except OSError:
                continue
        return ImageFont.load_default()

    width, height = size
    image = Image.new("RGB", size, COLORS.get(color, (60, 60, 60)))
    draw = ImageDraw.Draw(image)
    for text, px, y in ((name, int(height * 0.45), height * 0.36), (f"#{robot_id}", int(height * 0.11), height * 0.70),
                        (word, int(height * 0.13), height * 0.87)):
        f = font(px)
        box = draw.textbbox((0, 0), text, font=f)
        draw.text(((width - (box[2] - box[0])) / 2, y - (box[3] - box[1]) / 2), text, font=f, fill=(255, 255, 255))
    return image


class Screen:
    """pinkylib's LCD behind one call; the first drawing method that works is kept."""

    METHODS = ("img_show", "show_image", "display", "show")

    def __init__(self):
        from pinkylib import LCD  # only on the robot
        self.lcd = LCD()
        self.method = None
        width = getattr(self.lcd, "width", None) or getattr(self.lcd, "WIDTH", None)
        height = getattr(self.lcd, "height", None) or getattr(self.lcd, "HEIGHT", None)
        self.size = (int(width), int(height)) if width and height else FALLBACK_SIZE

    def draw(self, image):
        import numpy as np
        candidates = [self.method] if self.method else [m for m in self.METHODS if hasattr(self.lcd, m)]
        errors = []
        for method in candidates:
            for payload in (image, np.asarray(image)):
                try:
                    getattr(self.lcd, method)(payload)
                    self.method = method
                    return method
                except Exception as exc:  # noqa: BLE001 - unknown library, try the next way
                    errors.append(f"{method}: {exc}")
        raise RuntimeError("; ".join(errors) or "LCD에 그리기 함수가 없습니다")


def main():
    import rclpy
    from pathlib import Path
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    try:
        robot_id = str(json.loads(Path("/home/pinky/.cache/pinky-fleet/nav2/runtime_config.json").read_text())["robot_id"])
    except (OSError, ValueError, KeyError):
        robot_id = "?"

    class StatusDisplay(Node):
        def __init__(self):
            super().__init__("pinky_status_display")
            self.wanted, self.heard_at, self.shown, self.problem = None, 0.0, None, None
            try:
                self.screen = Screen()
            except Exception as exc:  # noqa: BLE001 - no screen library: report it, keep running
                self.screen, self.problem = None, f"LCD 열기 실패: {exc}"
            self.status = self.create_publisher(String, "/fleet/display_status", 1)
            self.create_subscription(String, "/fleet/indicator", self.on_indicator,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_timer(0.5, self.tick)
            self.create_timer(1.0, self.report)

        def on_indicator(self, msg):
            try:
                data = json.loads(msg.data)
            except ValueError:
                return
            if data.get("color") in COLORS:
                self.wanted, self.heard_at = (data["color"], data.get("mode", "on")), time.monotonic()

        def tick(self):
            state = screen_state(self.wanted, self.heard_at, time.monotonic())
            if self.screen is None or state == self.shown:
                return
            try:
                self.screen.draw(render(self.screen.size, NAMES.get(robot_id, robot_id), robot_id, *state))
                self.shown, self.problem = state, None
            except Exception as exc:  # noqa: BLE001
                self.problem = str(exc)[:300]

        def report(self):
            self.status.publish(String(data=json.dumps({
                "shown": list(self.shown) if self.shown else None, "method": getattr(self.screen, "method", None),
                "size": list(self.screen.size) if self.screen else None, "problem": self.problem})))

    rclpy.init()
    node = StatusDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
