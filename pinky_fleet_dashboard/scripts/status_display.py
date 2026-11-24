#!/usr/bin/env python3
"""핑키 자기 LCD에 이 핑키가 누구인지, 지금 무엇을 하는지 보여 준다.

실행 위치: 로봇 (runtime_bundle로 올라가고 app.py가 별도 프로세스로 띄움, app._reload_display가 재시작)
위: 이름(P0~P2)·로봇 id·배터리, 가운데: 하는 일, 아래: 둘째 줄, 배경: 상태 램프와 같은 색. 모두 관제
/fleet/indicator에서 온다. 한글 글꼴이 없으면 영어. SILENCE_S 동안 조용하면 NO LINK.

화면: pinky_pro 드라이버(pinky_emotion/pinky_lcd.py, 320x240)를 먼저, 그다음 옛 이름들을 시도한다(2026-11-24
소스만 보고 만듦). 상태는 /fleet/display_status로 알린다. emotion_server와 함께 돌리면 안 된다. 그리기만 하므로
주행에 영향이 없다. 2026-11-24: 프레임 전체 쓰기가 주행 중 자주 "[Errno 110]"으로 끝나, 바뀐 행 묶음(band)만
보내고 실패한 band는 다시 보낸다.
"""
import json
import time
from pathlib import Path

COLORS = {"orange": (255, 110, 0), "green": (0, 170, 60), "blue": (0, 90, 255), "red": (220, 0, 0)}
WORDS = {"orange": "NO POSE", "green": "DRIVING", "blue": "READY", "red": "STOP"}
NAMES = {"10": "P0", "11": "P1", "12": "P2"}  # 관제 화면의 이름
SILENCE_S = 3.0             # s. indicator는 0.25 s마다 온다
NO_SCREEN_REPORT_S = 90.0   # s. 화면 열기를 이만큼 시도한 뒤 끝낸다
OPEN_RETRY_S = 2.0
OPEN_TRIES = 3
TICK_S = 0.2                # s, 5 Hz
BAND_ROWS = 8        # 8 x 240 px x 2 바이트 = 3840, spidev 한 번(버퍼 4096)
BAND_TRIES = 3
SMALL_CHUNK = 32     # 큰 쓰기가 연달아 실패하면 쓰는 바이트 수(SPI FIFO 안에 듦)
BIG_FAILURES_TO_SMALL = 6
DRAW_BUDGET_S = 1.5  # s, tick당
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
# (모듈, 클래스, 그림 크기). 패널은 240x320이고 img_show가 가로 그림을 돌려 올린다.
BACKENDS = (("pinky_emotion.pinky_lcd", "LCD", (320, 240)), ("pinky_lcd", "LCD", (320, 240)), ("pinkylib", "LCD", None))


def korean_font(paths=KOREAN_FONT_PATHS):
    """이 기기에 있는 첫 한글 글꼴 파일, 없으면 None."""
    return next((path for path in paths if Path(path).is_file()), None)


def screen_state(wanted, heard_at, now, korean=False):
    """마지막 indicator로 (색, 화면, 표정, 제목, 둘째 줄, 배터리). 조용하면 NO LINK.
    화면: 'face'(주행), 'battery'(배터리 낮음), 'stop', 'text'."""
    if wanted is None or now - heard_at > SILENCE_S or wanted.get("mode") == "blink":
        return "orange", "text", None, ("관제 연결 끊김" if korean else "NO LINK"), "", None
    color = wanted["color"]
    headline = (wanted.get("text") if korean else None) or wanted.get("word") or WORDS.get(color, color.upper())
    detail = (wanted.get("detail") if korean else wanted.get("detail_en")) or ""
    screen = wanted.get("screen") if wanted.get("screen") in ("face", "battery", "stop") else "text"
    return color, screen, (wanted.get("face") if screen == "face" else None), str(headline), str(detail), wanted.get("battery")


def draw_face(draw, box, face, ink):
    """box 안의 얼굴: 'happy', 'excited'(귀가), 'focus'(도킹), 'surprised'(장애물)."""
    x0, y0, x1, y1 = box
    w, h = x1 - x0, y1 - y0
    cx, eye_y, gap = (x0 + x1) / 2, y0 + h * 0.36, w * 0.21
    stroke = max(4, int(h * 0.07))
    for ex in (cx - gap, cx + gap):
        if face == "excited":
            r = h * 0.17
            draw.arc((ex - r, eye_y - r * 0.7, ex + r, eye_y + r * 1.3), start=200, end=340, fill=ink, width=max(6, int(h * 0.09)))
        elif face == "focus":
            draw.line((ex - h * 0.17, eye_y, ex + h * 0.17, eye_y), fill=ink, width=max(6, int(h * 0.10)))
        else:
            r = h * (0.20 if face == "surprised" else 0.15)
            draw.ellipse((ex - r, eye_y - r, ex + r, eye_y + r), fill=ink)
    mouth_y = y0 + h * 0.78
    if face == "excited":
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
    """배터리 외곽을 그리고 percent만큼 채운다(최소 한 줄)."""
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
    """상태 색 바탕의 PIL 이미지. 왼쪽 위 이름, 오른쪽 위 배터리, 화면('text'·'face'·'battery'·'stop')별 본문."""
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
        """text를 가운데에 놓고 폭에 맞을 때까지 글자를 줄인다."""
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
    """패널 형태의 그림: big-endian RGB565 (행, 열, 2) 배열. pinky_lcd.img_show와 같은 회전·패킹."""
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
    """핑키 LCD를 한 호출 뒤에 감춘다. 저수준 함수가 있으면 바뀐 band만 보내고(남으면 draw가 False),
    없으면 프레임 전체 호출을 쓴다."""

    METHODS = ("img_show", "show_image", "display", "show")
    PRIMITIVES = ("spi", "w", "h", "_write_cmd", "_write_data", "_set_windows", "_write_data_buffer")

    def __init__(self, lcd=None, backend="given", size=None):
        import importlib
        errors = []
        candidates = [(backend, None, size)] if lcd is not None else BACKENDS
        for module, name, picture_size in candidates:
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
            self.sent = self.valid = None       # 패널 내용, 그중 확실히 아는 행
            self.failures = self.big_failures = 0
            self.small, self.last_error, self.last_draw_s = False, None, None
            return
        raise RuntimeError("; ".join(errors))

    @classmethod
    def _open(cls, lcd_class):
        """라이브러리 LCD 인스턴스. 생성자의 clear()가 Nav2가 도는 핑키에서 거의 매번 시간 초과돼(P1, 2026-11-24)
        저수준 함수가 있으면 뺀다. 나머지 설정은 OPEN_TRIES번 시도한다."""
        if all(hasattr(lcd_class, name) for name in ("_write_cmd", "_write_data", "_set_windows", "_write_data_buffer", "clear")):
            lcd_class = type("BandedLCD", (lcd_class,), {"clear": lambda self, color=0x0000: None})
        error = None
        for _ in range(OPEN_TRIES):
            try:
                return lcd_class()
            except OSError as exc:
                error = exc
                time.sleep(0.2)
        raise error

    def draw(self, image, budget_s=DRAW_BUDGET_S):
        """그림을 올린다. 다 올라가면 True, 남은 band가 있으면 False."""
        started = time.monotonic()
        try:
            return self._draw_bands(image, started + budget_s) if self.banded else self._draw_whole(image)
        finally:
            self.last_draw_s = round(time.monotonic() - started, 3)

    def _draw_whole(self, image):
        # 메서드·인자 형식을 몰라 차례로 시도하고 성공한 조합을 기억한다.
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
            lcd._write_cmd(0x36)   # img_show처럼 매 프레임 전에 설정
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
                except OSError as exc:  # 반쯤 쓰였을 수 있어 다시 보낸다
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
        """끝날 때 화면을 검게 해 오래된 'READY'가 남지 않게 한다."""
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
            self.target, self.picture = None, None
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
                else:
                    self.problem = f"화면 일부를 아직 못 보냄: {self.screen.last_error}"
            except Exception as exc:  # noqa: BLE001
                self.problem = str(exc)[:300]

        def report(self):
            # 1 s마다 /fleet/display_status로 알린다.
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
