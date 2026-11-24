#!/usr/bin/env python3
"""Robot side: show which Pinky this is, and what it is doing, on the Pinky's own screen.

Top: the name the control screens use (P0, P1, P2) with the robot id, and the battery. Middle, big: what it is doing
now ("적재 자리로 이동 중", "물건 싣는 중", "대기 중"). Below: a second line when there is one ("배달 A"). Background:
the same colour as the status lamp (scripts/status_lamp.py). All of it comes from the dashboard's /fleet/indicator
(scripts/fleet_ops.indicator and activity). Korean is shown when the robot has a Korean font, else the short English
word. With no indicator for SILENCE_S the dashboard is out of reach: NO LINK.

The screen: pinky_pro's own driver (pinky_emotion/pinky_lcd.py: LCD().img_show(PIL image), a 320x240 landscape
picture) is tried first, then the older names. Read from the pinky_pro sources on 2026-11-24, not yet seen working
on a Pinky; what worked, or why nothing did, is reported on /fleet/display_status. pinky_emotion's emotion_server
draws on the same screen ten times a second, so the two must not run together. With no screen this node reports
the reason for a while (trying to open it again meanwhile) and ends. A screen problem never affects driving: this
node only draws.

Sending the picture (2026-11-24): pinky_lcd.img_show writes the whole frame (153,600 bytes) in one go, and on a
Pinky that is driving that write often ended in "[Errno 110] Connection timed out", so the screen stayed on the old
picture for a long time. With pinky_lcd's own primitives this node sends only the bands of rows that changed, a few
rows per write, and sends a band again when its write fails; what is still to go is finished on the next ticks.
"""
import json
import time
from pathlib import Path

COLORS = {"orange": (255, 110, 0), "green": (0, 170, 60), "blue": (0, 90, 255), "red": (220, 0, 0)}
WORDS = {"orange": "NO POSE", "green": "DRIVING", "blue": "READY", "red": "STOP"}
NAMES = {"10": "P0", "11": "P1", "12": "P2"}  # the names on the control screens (control_station.store.PINKY_NAMES)
SILENCE_S = 3.0
NO_SCREEN_REPORT_S = 90.0   # how long opening the screen is tried (every OPEN_RETRY_S) before this node ends
OPEN_RETRY_S = 2.0
OPEN_TRIES = 3              # in a row, each time the screen is opened
TICK_S = 0.2
BAND_ROWS = 8        # panel rows per write: 8 x 240 px x 2 bytes = 3840, one spidev write (its buffer is 4096)
BAND_TRIES = 3       # a band whose write fails is sent again, with its window set again
SMALL_CHUNK = 32     # bytes per write after BIG_FAILURES_TO_SMALL failed band writes in a row (short writes
BIG_FAILURES_TO_SMALL = 6  # stay inside the SPI controller's FIFO; slower, tried only when big ones keep failing)
DRAW_BUDGET_S = 1.5  # per tick; bands left over go out on the next ticks
FALLBACK_SIZE = (320, 240)
FONT_PATHS = ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf")
KOREAN_FONT_PATHS = ("/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf",
                     "/usr/share/fonts/truetype/nanum/NanumSquareB.ttf",
                     "/usr/share/fonts/truetype/nanum/NanumBarunGothicBold.ttf",
                     "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
                     "/usr/share/fonts/truetype/noto/NotoSansCJK-Bold.ttc",
                     "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                     "/usr/share/fonts/truetype/unfonts-core/UnDotumBold.ttf",
                     "/usr/share/fonts/truetype/baekmuk/dotum.ttf")
# (module, class, picture size): pinky_lcd's panel is 240x320 and img_show turns a landscape picture onto it.
BACKENDS = (("pinky_emotion.pinky_lcd", "LCD", (320, 240)), ("pinky_lcd", "LCD", (320, 240)), ("pinkylib", "LCD", None))


def korean_font(paths=KOREAN_FONT_PATHS):
    """The first Korean font file on this machine, or None."""
    return next((path for path in paths if Path(path).is_file()), None)


def screen_state(wanted, heard_at, now, korean=False):
    """(colour, screen, face, headline, second line, battery) to show from the dashboard's last indicator (a dict),
    or NO LINK once it has been quiet for SILENCE_S. screen: 'face' (driving), 'battery' (low), 'stop', 'text'."""
    if wanted is None or now - heard_at > SILENCE_S or wanted.get("mode") == "blink":
        return "orange", "text", None, ("관제 연결 끊김" if korean else "NO LINK"), "", None
    color = wanted["color"]
    headline = (wanted.get("text") if korean else None) or wanted.get("word") or WORDS.get(color, color.upper())
    detail = (wanted.get("detail") if korean else wanted.get("detail_en")) or ""
    screen = wanted.get("screen") if wanted.get("screen") in ("face", "battery", "stop") else "text"
    return color, screen, (wanted.get("face") if screen == "face" else None), str(headline), str(detail), wanted.get("battery")


def draw_face(draw, box, face, ink):
    """A face inside box (x0, y0, x1, y1): 'happy' round eyes and a smile, 'excited' eyes squeezed into arches and a
    wide open laugh (going home), 'focus' narrowed eyes and a straight mouth (docking), 'surprised' wide eyes and an
    open mouth (an obstacle)."""
    x0, y0, x1, y1 = box
    w, h = x1 - x0, y1 - y0
    cx, eye_y, gap = (x0 + x1) / 2, y0 + h * 0.36, w * 0.21
    stroke = max(4, int(h * 0.07))
    for ex in (cx - gap, cx + gap):
        if face == "excited":   # ^ ^
            r = h * 0.17
            draw.arc((ex - r, eye_y - r * 0.7, ex + r, eye_y + r * 1.3), start=200, end=340, fill=ink, width=max(6, int(h * 0.09)))
        elif face == "focus":
            draw.line((ex - h * 0.17, eye_y, ex + h * 0.17, eye_y), fill=ink, width=max(6, int(h * 0.10)))
        else:
            r = h * (0.20 if face == "surprised" else 0.15)
            draw.ellipse((ex - r, eye_y - r, ex + r, eye_y + r), fill=ink)
    mouth_y = y0 + h * 0.78
    if face == "excited":       # an open laugh: the lower half of a wide ellipse, filled
        rx, ry = w * 0.24, h * 0.26
        draw.pieslice((cx - rx, mouth_y - ry * 1.1, cx + rx, mouth_y + ry * 0.9), start=0, end=180, fill=ink)
    elif face == "surprised":
        r = h * 0.10
        draw.ellipse((cx - r, mouth_y - r, cx + r, mouth_y + r), outline=ink, width=stroke)
    elif face == "focus":
        draw.line((cx - w * 0.10, mouth_y, cx + w * 0.10, mouth_y), fill=ink, width=stroke)
    else:
        rx, ry = w * 0.20, h * 0.22
        draw.arc((cx - rx, mouth_y - ry * 1.4, cx + rx, mouth_y + ry * 0.6), start=20, end=160, fill=ink, width=stroke)


def draw_battery(draw, box, percent, ink, paper):
    """A battery outline inside box, filled to percent (at least a sliver, so an empty one still reads)."""
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    w, h = x1 - x0, y1 - y0
    stroke, cap = max(4, int(h * 0.09)), max(4, int(w * 0.06))
    body = (x0, y0, x1 - cap, y1)
    draw.rounded_rectangle(body, radius=int(h * 0.16), outline=ink, width=stroke)
    draw.rectangle((x1 - cap, y0 + int(h * 0.30), x1, y1 - int(h * 0.30)), fill=ink)
    pad = stroke * 2
    inner = body[2] - body[0] - 2 * pad
    fill = int(max(inner * 0.06, inner * max(0, min(100, percent or 0)) / 100))
    draw.rectangle((x0 + pad, y0 + pad, x0 + pad + fill, y1 - pad), fill=ink)


def render(size, name, robot_id, color, screen="text", face=None, headline="", detail="", battery=None,
           korean_path=None):
    """A PIL image on the state colour. Always: the name top left and the battery top right. Then by screen:
    'text' the name big with '#id' and the headline big in the middle (shrunk to fit), the second line under it;
    'face' a face with the headline as a caption; 'battery' a battery with the percent beside it; 'stop' the
    headline (STOPPED) across the screen."""
    from PIL import Image, ImageDraw, ImageFont

    def font(px, text=""):
        paths = FONT_PATHS if text.isascii() or not korean_path else (korean_path,) + FONT_PATHS
        for path in paths:
            try:
                return ImageFont.truetype(path, px)
            except OSError:
                continue
        return ImageFont.load_default()

    width, height = size
    paper = COLORS.get(color, (60, 60, 60))
    image = Image.new("RGB", size, paper)
    draw = ImageDraw.Draw(image)
    white, margin = (255, 255, 255), int(width * 0.04)

    def centred(text, px, min_px, y):
        """text centred on (width / 2, y), shrunk until it fits the width."""
        if not text:
            return
        room = width - 2 * margin
        while True:
            f = font(px, text)
            box = draw.textbbox((0, 0), text, font=f)
            if box[2] - box[0] <= room or px <= min_px:
                break
            px -= 2
        draw.text(((width - (box[2] - box[0])) / 2 - box[0], y - (box[3] - box[1]) / 2 - box[1]), text, font=f, fill=white)

    big = screen == "text"
    name_font = font(int(height * (0.30 if big else 0.15)), name)
    box = draw.textbbox((0, 0), name, font=name_font)
    draw.text((margin - box[0], int(height * 0.04) - box[1]), name, font=name_font, fill=white)
    if big:
        draw.text((margin + box[2] - box[0] + int(width * 0.03), int(height * 0.22)), f"#{robot_id}",
                  font=font(int(height * 0.10)), fill=white)
    if battery is not None and screen != "battery":
        text = f"{battery}%"
        f = font(int(height * 0.13))
        box = draw.textbbox((0, 0), text, font=f)
        draw.text((width - margin - box[2], int(height * 0.06) - box[1]), text, font=f, fill=white)

    if screen == "face":
        draw_face(draw, (width * 0.18, height * 0.20, width * 0.82, height * 0.78), face, white)
        centred(headline, int(height * 0.12), int(height * 0.08), height * 0.89)
    elif screen == "battery":
        draw_battery(draw, (width * 0.08, height * 0.27, width * 0.52, height * 0.60), battery, white, paper)
        text = f"{battery if battery is not None else '?'}%"
        f = font(int(height * 0.24))
        box = draw.textbbox((0, 0), text, font=f)
        draw.text((width * 0.76 - (box[2] - box[0]) / 2 - box[0], height * 0.435 - (box[3] - box[1]) / 2 - box[1]),
                  text, font=f, fill=white)
        centred(headline, int(height * 0.14), int(height * 0.08), height * 0.74)
        centred(detail, int(height * 0.10), int(height * 0.07), height * 0.90)
    elif screen == "stop":
        centred(headline, int(height * 0.30), int(height * 0.12), height * (0.52 if detail else 0.56))
        centred(detail, int(height * 0.12), int(height * 0.08), height * 0.84)
    else:
        draw.line((margin, int(height * 0.40), width - margin, int(height * 0.40)), fill=white, width=2)
        centred(headline, int(height * 0.22), int(height * 0.09), height * (0.62 if detail else 0.68))
        centred(detail, int(height * 0.12), int(height * 0.08), height * 0.87)
    return image


def panel_bytes(image, width, height):
    """The picture as the panel takes it, an array (rows, columns, 2) of big-endian RGB565: the same turn and
    packing as pinky_lcd.img_show."""
    import numpy as np
    from PIL import Image
    turned = image.transpose(Image.FLIP_LEFT_RIGHT).transpose(Image.ROTATE_270)
    if turned.size != (width, height):
        turned = turned.resize((width, height), Image.LANCZOS)
    rgb = np.asarray(turned.convert("RGB")).astype(np.uint16)
    high = (rgb[..., 0] & 0xF8) | (rgb[..., 1] >> 5)
    low = ((rgb[..., 1] << 3) & 0xE0) | (rgb[..., 2] >> 3)
    return np.stack([high, low], axis=-1).astype(np.uint8)


class Screen:
    """The Pinky's LCD behind one call: the first library that opens. With pinky_lcd's primitives only the changed
    bands are sent (draw returns False while some are still to go); otherwise the library's own whole-frame call."""

    METHODS = ("img_show", "show_image", "display", "show")
    PRIMITIVES = ("spi", "w", "h", "_write_cmd", "_write_data", "_set_windows", "_write_data_buffer")

    def __init__(self, lcd=None, backend="given", size=None):
        import importlib
        errors = []
        candidates = [(backend, None, size)] if lcd is not None else BACKENDS
        for module, name, picture_size in candidates:  # importing: only on the robot
            try:
                self.lcd = lcd if lcd is not None else self._open(getattr(importlib.import_module(module), name))
            except Exception as exc:  # noqa: BLE001 - not installed, or the screen is held by something else
                errors.append(f"{module}: {exc}")
                continue
            self.backend, self.method, self.payload = module, None, None
            width = getattr(self.lcd, "width", None) or getattr(self.lcd, "WIDTH", None)
            height = getattr(self.lcd, "height", None) or getattr(self.lcd, "HEIGHT", None)
            self.size = picture_size or ((int(width), int(height)) if width and height else FALLBACK_SIZE)
            self.banded = all(hasattr(self.lcd, name) for name in self.PRIMITIVES)
            self.sent = self.valid = None       # what the panel holds, and which rows of that are known
            self.failures = self.big_failures = 0
            self.small, self.last_error, self.last_draw_s = False, None, None
            return
        raise RuntimeError("; ".join(errors))

    @classmethod
    def _open(cls, lcd_class):
        """An instance of the library's LCD. pinky_lcd's constructor ends with clear(), 38 writes of 4096 bytes, and
        on a Pinky with Nav2 running one of them timed out nearly every time, so the screen could not even be opened
        (P1, 2026-11-24). With its primitives the bands are sent here anyway, so that clear() is left out; the rest
        of the constructor (pins, reset, panel settings: single bytes) is tried OPEN_TRIES times."""
        if all(hasattr(lcd_class, name) for name in ("_write_cmd", "_write_data", "_set_windows", "_write_data_buffer", "clear")):
            lcd_class = type("BandedLCD", (lcd_class,), {"clear": lambda self, color=0x0000: None})
        error = None
        for _ in range(OPEN_TRIES):
            try:
                return lcd_class()
            except OSError as exc:  # a write timed out: again
                error = exc
                time.sleep(0.2)
        raise error

    def draw(self, image, budget_s=DRAW_BUDGET_S):
        """Put the picture on the screen. True when all of it is there, False when some bands are still to go."""
        started = time.monotonic()
        try:
            return self._draw_bands(image, started + budget_s) if self.banded else self._draw_whole(image)
        finally:
            self.last_draw_s = round(time.monotonic() - started, 3)

    def _draw_whole(self, image):
        import numpy as np
        candidates = [self.method] if self.method else [m for m in self.METHODS if hasattr(self.lcd, m)]
        errors = []
        for method in candidates:
            for kind in ([self.payload] if self.payload else ["image", "array"]):
                try:
                    getattr(self.lcd, method)(image if kind == "image" else np.asarray(image))
                    self.method, self.payload = method, kind
                    return True
                except Exception as exc:  # noqa: BLE001 - unknown library, try the next way
                    self.failures += 1
                    self.last_error = f"{method}: {exc}"
                    errors.append(self.last_error)
        raise RuntimeError("; ".join(errors) or "LCD에 그리기 함수가 없습니다")

    def _draw_bands(self, image, deadline):
        import numpy as np
        lcd = self.lcd
        target = panel_bytes(image, lcd.w, lcd.h)
        if self.sent is None or self.sent.shape != target.shape:
            self.sent, self.valid = np.zeros_like(target), np.zeros(target.shape[0], dtype=bool)
        dirty = ~self.valid | (self.sent != target).any(axis=(1, 2))
        bands = sorted({int(row) // BAND_ROWS for row in np.flatnonzero(dirty)})
        if not bands:
            return True
        self.method = "bands"
        try:
            lcd._write_cmd(0x36)   # memory access control, as img_show sets it before every frame
            lcd._write_data(0x08)
        except OSError as exc:
            self.failures += 1
            self.last_error = str(exc)
            return False
        for band in bands:
            if time.monotonic() > deadline:
                break
            first, last = band * BAND_ROWS, min(target.shape[0], (band + 1) * BAND_ROWS)
            data = target[first:last].tobytes()
            for _ in range(BAND_TRIES):
                try:
                    lcd._set_windows(0, first, lcd.w, last)
                    if self.small:
                        for at in range(0, len(data), SMALL_CHUNK):
                            lcd._write_data_buffer(data[at:at + SMALL_CHUNK])
                    else:
                        lcd._write_data_buffer(data)
                    self.sent[first:last], self.valid[first:last] = target[first:last], True
                    self.big_failures = 0
                    break
                except OSError as exc:  # the write timed out: the band may be half written, so it goes again
                    self.valid[first:last] = False
                    self.failures += 1
                    self.last_error = str(exc)
                    if not self.small:
                        self.big_failures += 1
                        self.small = self.big_failures >= BIG_FAILURES_TO_SMALL
                if time.monotonic() > deadline:
                    break
        return bool(self.valid.all() and (self.sent == target).all())

    def blank(self):
        """Dark screen when this node ends, so a stale 'READY' is not left standing."""
        try:
            from PIL import Image
            self.draw(Image.new("RGB", self.size, (0, 0, 0)), budget_s=3.0)
        except Exception:  # noqa: BLE001
            pass


def main():
    import rclpy
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
            self.target, self.picture = None, None  # the state being drawn and its picture
            self.korean_path, self.started = korean_font(), time.monotonic()
            self.screen, self.opened_at = None, None
            self.open_screen()
            self.status = self.create_publisher(String, "/fleet/display_status", 1)
            self.create_subscription(String, "/fleet/indicator", self.on_indicator,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_timer(TICK_S, self.tick)
            self.create_timer(1.0, self.report)

        def on_indicator(self, msg):
            try:
                data = json.loads(msg.data)
            except ValueError:
                return
            if isinstance(data, dict) and data.get("color") in COLORS:
                self.wanted, self.heard_at = data, time.monotonic()

        def open_screen(self):
            self.opened_at = time.monotonic()
            try:
                self.screen = Screen()
                self.problem = None
            except Exception as exc:  # noqa: BLE001 - no screen library, or its writes time out: tried again
                self.screen, self.problem = None, f"LCD 열기 실패: {exc}"[:400]
                self.get_logger().error(self.problem)

        def tick(self):
            if self.screen is None:
                if time.monotonic() - self.started > NO_SCREEN_REPORT_S:
                    raise SystemExit(0)
                if time.monotonic() - self.opened_at >= OPEN_RETRY_S:
                    self.open_screen()
                if self.screen is None:
                    return
            state = screen_state(self.wanted, self.heard_at, time.monotonic(), korean=bool(self.korean_path))
            if state == self.shown:
                return
            try:
                if state != self.target:
                    self.picture = render(self.screen.size, NAMES.get(robot_id, robot_id), robot_id, *state,
                                          korean_path=self.korean_path)
                    self.target = state
                if self.screen.draw(self.picture):
                    self.shown, self.problem = state, None
                else:  # the rest goes out on the next ticks
                    self.problem = f"화면 일부를 아직 못 보냄: {self.screen.last_error}"
            except Exception as exc:  # noqa: BLE001
                self.problem = str(exc)[:300]

        def report(self):
            screen = self.screen
            self.status.publish(String(data=json.dumps({
                "shown": list(self.shown) if self.shown else None, "backend": getattr(screen, "backend", None),
                "method": getattr(screen, "method", None), "size": list(screen.size) if screen else None,
                "korean_font": self.korean_path, "problem": self.problem,
                "write_failures": getattr(screen, "failures", None), "small_writes": getattr(screen, "small", None),
                "last_draw_s": getattr(screen, "last_draw_s", None)}, ensure_ascii=False)))

    rclpy.init()
    node = StatusDisplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as exc:  # noqa: BLE001 - e.g. rclpy's shutdown exception on SIGTERM
        if type(exc).__name__ != "ExternalShutdownException":
            raise
    finally:
        if node.screen is not None:
            node.screen.blank()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
