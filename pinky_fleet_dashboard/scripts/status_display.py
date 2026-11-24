#!/usr/bin/env python3
"""핑키 자기 LCD에 이 핑키가 누구인지, 지금 무엇을 하는지 보여 준다.

실행 위치: 로봇 (주행준비 때 runtime_bundle로 올라감)
사용처: app.py가 주행준비 때 별도 프로세스로 띄우고, app._reload_display가 이 파일만 다시 올려 재시작한다.

위: 관제 화면의 이름(P0, P1, P2)과 로봇 id, 배터리. 가운데 크게: 지금 하는 일("적재 자리로 이동 중",
"물건 싣는 중", "대기 중"). 아래: 둘째 줄이 있으면("배달 A"). 배경: 상태 램프(scripts/status_lamp.py)와 같은 색.
모두 관제의 /fleet/indicator(scripts/fleet_ops.indicator와 activity)에서 온다. 로봇에 한글 글꼴이 있으면 한국어,
없으면 짧은 영어 단어를 쓴다. SILENCE_S 동안 indicator가 없으면 관제와 끊긴 것: NO LINK.

화면: pinky_pro 자체 드라이버(pinky_emotion/pinky_lcd.py: LCD().img_show(PIL 이미지), 320x240 가로 그림)를
먼저 시도하고, 그다음 옛 이름들을 시도한다. 2026-11-24 pinky_pro 소스를 읽고 만든 것으로 핑키에서 동작을 아직
보지 못했다. 무엇이 됐는지, 왜 안 됐는지는 /fleet/display_status로 알린다. pinky_emotion의 emotion_server가
같은 화면에 초당 10번 그리므로 둘을 함께 돌리면 안 된다. 화면이 없으면 한동안 이유를 알리며(그동안 다시 열기를
시도) 끝난다. 화면 문제는 주행에 영향을 주지 않는다: 이 노드는 그리기만 한다.

그림 보내기(2026-11-24): pinky_lcd.img_show는 프레임 전체(153,600 바이트)를 한 번에 쓰는데, 주행 중인
핑키에서 그 쓰기가 자주 "[Errno 110] Connection timed out"으로 끝나 화면이 오래 옛 그림에 머물렀다.
pinky_lcd의 저수준 함수로 바뀐 행 묶음(band)만 몇 행씩 보내고, 쓰기가 실패한 묶음은 다시 보낸다.
남은 것은 다음 tick들에서 마저 보낸다.
"""
import json
import time
from pathlib import Path

COLORS = {"orange": (255, 110, 0), "green": (0, 170, 60), "blue": (0, 90, 255), "red": (220, 0, 0)}
WORDS = {"orange": "NO POSE", "green": "DRIVING", "blue": "READY", "red": "STOP"}
NAMES = {"10": "P0", "11": "P1", "12": "P2"}  # 관제 화면의 이름(control_station.storage.store.PINKY_NAMES)
SILENCE_S = 3.0             # s. 관제 indicator는 0.25 s마다 오므로 3 s 조용하면 끊긴 것
NO_SCREEN_REPORT_S = 90.0   # s. 화면 열기를(OPEN_RETRY_S마다) 이만큼 시도한 뒤 노드를 끝낸다
OPEN_RETRY_S = 2.0
OPEN_TRIES = 3              # 화면을 열 때마다 연달아 시도하는 횟수
TICK_S = 0.2                # s. 5 Hz로 그릴 것이 바뀌었는지 본다
BAND_ROWS = 8        # 한 번 쓰는 패널 행 수: 8 x 240 px x 2 바이트 = 3840, spidev 한 번(버퍼 4096)
BAND_TRIES = 3       # 쓰기 실패한 band는 창(window)을 다시 잡고 다시 보낸다
SMALL_CHUNK = 32     # band 쓰기가 BIG_FAILURES_TO_SMALL번 연달아 실패한 뒤의 쓰기당 바이트(짧은 쓰기는
BIG_FAILURES_TO_SMALL = 6  # SPI 컨트롤러 FIFO 안에 들어감. 느려서 큰 쓰기가 계속 실패할 때만)
DRAW_BUDGET_S = 1.5  # s, tick당. 남은 band는 다음 tick들에서 보낸다
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
# (모듈, 클래스, 그림 크기): pinky_lcd 패널은 240x320이고 img_show가 가로 그림을 돌려 올린다.
BACKENDS = (("pinky_emotion.pinky_lcd", "LCD", (320, 240)), ("pinky_lcd", "LCD", (320, 240)), ("pinkylib", "LCD", None))


def korean_font(paths=KOREAN_FONT_PATHS):
    """이 기기에 있는 첫 한글 글꼴 파일, 없으면 None."""
    return next((path for path in paths if Path(path).is_file()), None)


def screen_state(wanted, heard_at, now, korean=False):
    """관제의 마지막 indicator(dict)로 보여 줄 (색, 화면, 표정, 제목, 둘째 줄, 배터리). SILENCE_S 동안 조용하면
    NO LINK. 화면: 'face'(주행 중), 'battery'(배터리 낮음), 'stop', 'text'."""
    if wanted is None or now - heard_at > SILENCE_S or wanted.get("mode") == "blink":
        return "orange", "text", None, ("관제 연결 끊김" if korean else "NO LINK"), "", None
    color = wanted["color"]
    headline = (wanted.get("text") if korean else None) or wanted.get("word") or WORDS.get(color, color.upper())
    detail = (wanted.get("detail") if korean else wanted.get("detail_en")) or ""
    screen = wanted.get("screen") if wanted.get("screen") in ("face", "battery", "stop") else "text"
    return color, screen, (wanted.get("face") if screen == "face" else None), str(headline), str(detail), wanted.get("battery")


def draw_face(draw, box, face, ink):
    """box (x0, y0, x1, y1) 안의 얼굴: 'happy' 둥근 눈과 미소, 'excited' 아치 모양 눈과 활짝 웃는 입(귀가),
    'focus' 가는 눈과 일자 입(도킹), 'surprised' 큰 눈과 벌린 입(장애물)."""
    x0, y0, x1, y1 = box
    w, h = x1 - x0, y1 - y0
    cx, eye_y, gap = (x0 + x1) / 2, y0 + h * 0.36, w * 0.21
    stroke = max(4, int(h * 0.07))
    for ex in (cx - gap, cx + gap):
        if face == "excited":   # ^ ^ 모양 눈
            r = h * 0.17
            draw.arc((ex - r, eye_y - r * 0.7, ex + r, eye_y + r * 1.3), start=200, end=340, fill=ink, width=max(6, int(h * 0.09)))
        elif face == "focus":
            draw.line((ex - h * 0.17, eye_y, ex + h * 0.17, eye_y), fill=ink, width=max(6, int(h * 0.10)))
        else:
            r = h * (0.20 if face == "surprised" else 0.15)
            draw.ellipse((ex - r, eye_y - r, ex + r, eye_y + r), fill=ink)
    mouth_y = y0 + h * 0.78
    if face == "excited":       # 활짝 웃는 입: 넓은 타원의 아래 절반을 채움
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
    """box 안에 배터리 외곽을 그리고 percent만큼 채운다(빈 배터리도 보이게 최소 한 줄은 채움)."""
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
    """상태 색 바탕의 PIL 이미지. 항상 왼쪽 위에 이름, 오른쪽 위에 배터리. 화면별로: 'text'는 큰 이름과 '#id',
    가운데 큰 제목(맞게 줄임)과 그 아래 둘째 줄. 'face'는 얼굴과 제목 자막. 'battery'는 배터리와 옆의 퍼센트.
    'stop'은 화면 가득 제목(STOPPED)."""
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
        """text를 (width / 2, y) 가운데에 놓고, 폭에 맞을 때까지 글자 크기를 줄인다."""
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
    """패널이 받는 형태의 그림: big-endian RGB565의 (행, 열, 2) 배열. pinky_lcd.img_show와 같은 회전·패킹이다."""
    import numpy as np
    from PIL import Image
    turned = image.transpose(Image.FLIP_LEFT_RIGHT).transpose(Image.ROTATE_270)
    if turned.size != (width, height):
        turned = turned.resize((width, height), Image.LANCZOS)
    rgb = np.asarray(turned.convert("RGB")).astype(np.uint16)
    # RGB565: 빨강 5비트 | 초록 6비트 | 파랑 5비트를 상위·하위 바이트로 나눈다.
    high = (rgb[..., 0] & 0xF8) | (rgb[..., 1] >> 5)
    low = ((rgb[..., 1] << 3) & 0xE0) | (rgb[..., 2] >> 3)
    return np.stack([high, low], axis=-1).astype(np.uint8)


class Screen:
    """핑키 LCD를 한 호출 뒤에 감춘다: 처음 열리는 라이브러리를 쓴다. pinky_lcd 저수준 함수가 있으면 바뀐 band만
    보내고(남은 것이 있으면 draw가 False), 없으면 라이브러리의 프레임 전체 호출을 쓴다."""

    METHODS = ("img_show", "show_image", "display", "show")
    PRIMITIVES = ("spi", "w", "h", "_write_cmd", "_write_data", "_set_windows", "_write_data_buffer")

    def __init__(self, lcd=None, backend="given", size=None):
        import importlib
        errors = []
        candidates = [(backend, None, size)] if lcd is not None else BACKENDS
        for module, name, picture_size in candidates:  # import는 로봇에서만 된다
            try:
                self.lcd = lcd if lcd is not None else self._open(getattr(importlib.import_module(module), name))
            except Exception as exc:  # noqa: BLE001 - not installed, or the screen is held by something else
                # 설치 안 됨, 또는 다른 프로세스가 화면을 잡고 있음
                errors.append(f"{module}: {exc}")
                continue
            self.backend, self.method, self.payload = module, None, None
            width = getattr(self.lcd, "width", None) or getattr(self.lcd, "WIDTH", None)
            height = getattr(self.lcd, "height", None) or getattr(self.lcd, "HEIGHT", None)
            self.size = picture_size or ((int(width), int(height)) if width and height else FALLBACK_SIZE)
            self.banded = all(hasattr(self.lcd, name) for name in self.PRIMITIVES)
            self.sent = self.valid = None       # 패널에 들어 있는 것, 그중 확실히 아는 행
            self.failures = self.big_failures = 0
            self.small, self.last_error, self.last_draw_s = False, None, None
            return
        raise RuntimeError("; ".join(errors))

    @classmethod
    def _open(cls, lcd_class):
        """라이브러리 LCD 인스턴스. pinky_lcd 생성자는 clear()(4096 바이트 쓰기 38번)로 끝나는데, Nav2가 도는
        핑키에서는 그중 하나가 거의 매번 시간 초과되어 화면을 열지도 못했다(P1, 2026-11-24). 저수준 함수가 있으면
        어차피 여기서 band로 보내므로 그 clear()를 뺀다. 나머지(핀, 리셋, 패널 설정: 1바이트씩)는 OPEN_TRIES번 시도한다."""
        if all(hasattr(lcd_class, name) for name in ("_write_cmd", "_write_data", "_set_windows", "_write_data_buffer", "clear")):
            lcd_class = type("BandedLCD", (lcd_class,), {"clear": lambda self, color=0x0000: None})
        error = None
        for _ in range(OPEN_TRIES):
            try:
                return lcd_class()
            except OSError as exc:  # 쓰기 시간 초과: 다시
                error = exc
                time.sleep(0.2)
        raise error

    def draw(self, image, budget_s=DRAW_BUDGET_S):
        """그림을 화면에 올린다. 전부 올라가면 True, 아직 남은 band가 있으면 False."""
        started = time.monotonic()
        try:
            return self._draw_bands(image, started + budget_s) if self.banded else self._draw_whole(image)
        finally:
            self.last_draw_s = round(time.monotonic() - started, 3)

    def _draw_whole(self, image):
        # 메서드 이름·인자 형식(PIL/배열)을 몰라 차례로 시도하고, 성공한 조합을 기억한다.
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
        # 바뀌었거나 상태를 모르는 행이 든 band만 보낸다.
        dirty = ~self.valid | (self.sent != target).any(axis=(1, 2))
        bands = sorted({int(row) // BAND_ROWS for row in np.flatnonzero(dirty)})
        if not bands:
            return True
        self.method = "bands"
        try:
            lcd._write_cmd(0x36)   # 메모리 접근 제어. img_show가 매 프레임 전에 하듯 설정
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
                except OSError as exc:  # 쓰기 시간 초과: band가 반쯤 쓰였을 수 있어 다시 보낸다
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
        """노드가 끝날 때 화면을 검게 해 오래된 'READY'가 남지 않게 한다."""
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
            # 화면은 운영 편의 기능이라 열리지 않아도 노드는 이유를 알리며 계속 돈다.
            super().__init__("pinky_status_display")
            self.wanted, self.heard_at, self.shown, self.problem = None, 0.0, None, None
            self.target, self.picture = None, None  # 그리는 중인 상태와 그 그림
            self.korean_path, self.started = korean_font(), time.monotonic()
            self.screen, self.opened_at = None, None
            self.open_screen()
            self.status = self.create_publisher(String, "/fleet/display_status", 1)
            # indicator는 0.25 s마다 새로 오므로 best-effort, 마지막 하나(depth 1)만 있으면 된다.
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
                # 화면 라이브러리가 없거나 쓰기가 시간 초과: 다시 시도한다
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
                else:  # 나머지는 다음 tick들에서 보낸다
                    self.problem = f"화면 일부를 아직 못 보냄: {self.screen.last_error}"
            except Exception as exc:  # noqa: BLE001
                self.problem = str(exc)[:300]

        def report(self):
            # 1 s마다 화면 상태를 /fleet/display_status로 알린다(관제가 원인 파악에 씀).
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
        # 예: SIGTERM 때 rclpy의 종료 예외
        if type(exc).__name__ != "ExternalShutdownException":
            raise
    finally:
        if node.screen is not None:
            node.screen.blank()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
