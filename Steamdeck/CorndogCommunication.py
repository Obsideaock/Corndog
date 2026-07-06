#!/usr/bin/env python3
"""
CorndogCommunication.py (Steam Deck client) — v4

New vs v3:
- LAUNCHER: after the IP is known you pick a session type:
      DRIVE            camera + joystick (lightest, no LiDAR spin-up)
      DRIVE + MINIMAP  same, but the Pi boots the SLAM stack and the corner
                       minimap starts on
      MAP MODE         fullscreen native map: cursor/touch to send goals,
                       save spots, roam — the robot drives itself
  The Pi's supervisor is told which flavor to boot via the presence hello
  ("STEAMDECK_READY MODE=drive|slam"), so the right script starts on the robot
  automatically. Last choice is remembered and preselected.
- SETTINGS screen (X on the launcher): every gait-engine knob (step rate, step
  height, speeds, slew, finish-step, soft touchdown, inactivity reset, ...) plus
  deck-local prefs (debug HUD, minimap size). Robot-scope settings are pushed
  over the control socket as "GAIT <key> <value>" on every (re)connect.
- TOASTS: slide-in notifications for snapshots, reconnects, emotes, errors.
- RECONNECT, DON'T DIE: losing the video stream or the control socket now shows
  a spinner/toast and retries forever instead of exiting the app.
- DEBUG HUD (D-pad Down): fps, control ping, robot pose, raw + snapped stick
  values, Wi-Fi signal.
- SNAPSHOT (Back/Select): saves the current camera frame to ~/Pictures/Corndog.
- AUTO-DISCOVERY: the Pi broadcasts a UDP beacon; the IP-setup screen listens
  and offers the found address (press Y) — no more typing IPs on new networks.
- D-pad: Up = minimap on/off, Right = minimap size S/M/L, Down = debug HUD.
- Start = back to the launcher from a drive session.

Requires (same folder): cdc_emotes.py, cdc_minimap.py, cdc_mapmode.py
"""

import cv2
import numpy as np
import threading
import socket
import select
import math
import os
from time import sleep, time
from datetime import datetime
from urllib.request import urlopen
from urllib.error import URLError
import socket as pysocket

from evdev import InputDevice, ecodes, list_devices

import json
import re
import subprocess
from pathlib import Path

from cdc_emotes import EMOTES, EMOTE_TOKENS, label_for
from cdc_minimap import Minimap
import cdc_mapmode


# -------------------- SETTINGS --------------------
CONTROL_PORT = 65432
PRESENCE_PORT = 65431
SLAM_PORT = 8001
DISCOVERY_PORT = 65430

DEVICE_PATH_FALLBACK = "/dev/input/event9"

SCREEN_W = 1280
SCREEN_H = 800

RETRY_SEC = 1.0
MJPEG_IO_TIMEOUT = 2.0
CONTROL_IO_TIMEOUT = 3.0

PRESENCE_CONNECT_TIMEOUT = 2.0
PRESENCE_HEARTBEAT_S = 2.0

PING_PERIOD_S = 1.0

# Battery (placeholder until you have voltage telemetry)
BATTERY_ENABLED = False

# Right-stick drift / sensitivity
RIGHT_RADIAL_DEADZONE = 0.60

# Wheel
WHEEL_SELECT_DZ = 0.50

# Minimap sizes (S/M/L) — px square
MINIMAP_SIZES = {"S": 190, "M": 240, "L": 320}
MINIMAP_MARGIN = 16

SNAPSHOT_DIR = Path.home() / "Pictures" / "Corndog"

RESET_BTN_RECT = None

# Button codes
BTN_A = 304
BTN_B = 305
BTN_X = 307
BTN_Y = 308
BTN_LB = 310
BTN_RB = 311
BTN_BACK = 312
BTN_START = 313
BTN_GUIDE = 314

# Axis codes
ABS_LX, ABS_LY = 0, 1
ABS_RX, ABS_RY = 3, 4
ABS_HAT_X, ABS_HAT_Y = 16, 17

# Palette (BGR)
BG_DARK = (13, 17, 23)
BG_PANEL = (22, 27, 34)
ACCENT     = (30, 30, 200)
ACCENT_DIM = (15, 15, 60)
ACCENT_GREEN     = (121, 161, 33)
ACCENT_GREEN_DIM = (25, 45, 10)
TEXT_HI = (230, 237, 243)
TEXT_MID = (139, 148, 158)
TEXT_ERR = (248, 81, 73)
BORDER = (33, 39, 47)
FONT = cv2.FONT_HERSHEY_SIMPLEX

WINDOW = "Corndog"

CONFIG_DIR = Path.home() / ".config" / "cdc"
CONFIG_PATH = CONFIG_DIR / "config.json"
IP_RE = re.compile(r"^(?:\d{1,3}\.){3}\d{1,3}$")


# ==================== SETTINGS SPEC ====================
# scope "gait"  -> pushed to the Pi as "GAIT <key> <value>" on control connect
# scope "local" -> deck-side only
SETTINGS_SPEC = [
    {"key": "debug_hud",    "label": "Debug HUD",            "type": "bool",  "default": False, "scope": "local"},
    {"key": "minimap_size", "label": "Minimap size",         "type": "choice","choices": ["S", "M", "L"], "default": "M", "scope": "local"},

    {"key": "gait",         "label": "Gait pattern",         "type": "choice","choices": ["diagonal", "creep", "wave"], "default": "diagonal", "scope": "gait"},
    {"key": "step_hz",      "label": "Step rate (Hz)",       "type": "float", "default": 1.15, "min": 0.2,  "max": 2.5,  "step": 0.05, "scope": "gait"},
    {"key": "swing_frac",   "label": "Swing fraction",       "type": "float", "default": 0.24, "min": 0.05, "max": 0.60, "step": 0.01, "scope": "gait"},
    {"key": "step_height",  "label": "Step height (m)",      "type": "float", "default": 0.045,"min": 0.005,"max": 0.055,"step": 0.005,"scope": "gait"},
    {"key": "speed_scale",  "label": "Speed scale",          "type": "float", "default": 1.0,  "min": 0.05, "max": 3.0,  "step": 0.05, "scope": "gait"},
    {"key": "height_offset","label": "Body height offset",   "type": "float", "default": 0.0,  "min": -0.10,"max": 0.10, "step": 0.005,"scope": "gait"},
    {"key": "vx_speed",     "label": "Fwd/back speed (m/s)", "type": "float", "default": 0.15, "min": 0.01, "max": 0.40, "step": 0.01, "scope": "gait"},
    {"key": "vy_speed",     "label": "Strafe speed (m/s)",   "type": "float", "default": 0.11, "min": 0.01, "max": 0.40, "step": 0.01, "scope": "gait"},
    {"key": "diag_speed",   "label": "Diagonal speed (m/s)", "type": "float", "default": 0.13, "min": 0.01, "max": 0.40, "step": 0.01, "scope": "gait"},
    {"key": "wz_max",       "label": "Turn rate max (rad/s)","type": "float", "default": 1.0,  "min": 0.05, "max": 3.0,  "step": 0.05, "scope": "gait"},

    {"key": "real_dt",      "label": "Real-time phase (dt)", "type": "bool",  "default": True,  "scope": "gait"},
    {"key": "finish_step",  "label": "Finish step on stop",  "type": "bool",  "default": True,  "scope": "gait"},
    {"key": "slew",         "label": "Velocity ramping",     "type": "bool",  "default": True,  "scope": "gait"},
    {"key": "slew_time",    "label": "Ramp time (s)",        "type": "float", "default": 0.25, "min": 0.02, "max": 2.0,  "step": 0.01, "scope": "gait"},
    {"key": "step_clamp",   "label": "Step-length clamp",    "type": "bool",  "default": True,  "scope": "gait"},
    {"key": "max_step_len", "label": "Max step length (m)",  "type": "float", "default": 0.135,"min": 0.03, "max": 0.30, "step": 0.005,"scope": "gait"},
    {"key": "soft_td",      "label": "Soft touchdown",       "type": "bool",  "default": False, "scope": "gait"},
    {"key": "inactivity_reset", "label": "Idle reset delay (s)", "type": "float", "default": 1.0, "min": 0.1, "max": 10.0, "step": 0.1, "scope": "gait"},

    {"key": "write_deadband","label": "Servo write deadband","type": "float", "default": 0.0,  "min": 0.0,  "max": 3.0,  "step": 0.1,  "scope": "gait"},
    {"key": "move_easing",  "label": "Emote move easing",    "type": "bool",  "default": False, "scope": "gait"},
    {"key": "speed_mode",   "label": "SPEEDMODE (all-out)",  "type": "bool",  "default": False, "scope": "gait"},
]
SETTINGS_BY_KEY = {s["key"]: s for s in SETTINGS_SPEC}

MODES = [
    ("drive",     "DRIVE",           "Camera + joystick. Fastest boot,", "no LiDAR."),
    ("drive_map", "DRIVE + MINIMAP", "Drive with the SLAM minimap in",   "the corner. LiDAR spins up."),
    ("map",       "MAP MODE",        "Fullscreen map. Tap or aim to",    "send him places. He drives."),
]
MODE_TOKEN = {"drive": "drive", "drive_map": "slam", "map": "slam"}


# ==================== CONFIG ====================
def load_config():
    try:
        cfg = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception:
        cfg = {}
    cfg.setdefault("wifi_to_ip", {})
    cfg.setdefault("last_ip", None)
    cfg.setdefault("mode", "drive")
    cfg.setdefault("settings", {})
    return cfg


def save_config(cfg):
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2), encoding="utf-8")


def get_setting(cfg, key):
    spec = SETTINGS_BY_KEY[key]
    return cfg.get("settings", {}).get(key, spec["default"])


def set_setting(cfg, key, value):
    cfg.setdefault("settings", {})[key] = value


def gait_settings_lines(cfg):
    """The 'GAIT key value' lines to push to the Pi on control connect."""
    out = []
    for s in SETTINGS_SPEC:
        if s["scope"] != "gait":
            continue
        v = get_setting(cfg, s["key"])
        if s["type"] == "bool":
            v = 1 if v else 0
        out.append(f"GAIT {s['key']} {v}")
    return out


def get_active_ssid():
    try:
        out = subprocess.check_output(
            ["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"], text=True)
        for line in out.splitlines():
            if line.startswith("yes:"):
                ssid = line.split("yes:", 1)[1].strip()
                return ssid or None
    except Exception:
        pass
    return None


def get_wifi_signal():
    """Signal % of the in-use network, or None."""
    try:
        out = subprocess.check_output(
            ["nmcli", "-t", "-f", "IN-USE,SIGNAL", "dev", "wifi"], text=True, timeout=2)
        for line in out.splitlines():
            if line.startswith("*:"):
                return int(line.split(":", 1)[1])
    except Exception:
        pass
    return None


def validate_ip(ip):
    if not IP_RE.match(ip):
        return False
    parts = ip.split(".")
    if len(parts) != 4:
        return False
    try:
        return all(0 <= int(p) <= 255 for p in parts)
    except ValueError:
        return False


def _is_gamepad(dev):
    caps = dev.capabilities()
    keys = caps.get(ecodes.EV_KEY, [])
    abss = caps.get(ecodes.EV_ABS, [])
    return (BTN_A in keys) and (len(abss) > 0)


def pick_controller_device():
    try:
        dev = InputDevice(DEVICE_PATH_FALLBACK)
        ok = _is_gamepad(dev)
        name = dev.name
        dev.close()
        if ok:
            return DEVICE_PATH_FALLBACK
        print(f"[Init] {DEVICE_PATH_FALLBACK} is '{name}', not a gamepad — scanning...")
    except Exception:
        pass

    candidates = []
    for path in list_devices():
        try:
            dev = InputDevice(path)
            if _is_gamepad(dev):
                candidates.append((path, dev.name))
            dev.close()
        except Exception:
            continue
    if not candidates:
        return None

    def score(item):
        name = item[1].lower()
        for i, kw in enumerate(("steam deck", "valve", "x-box", "xbox", "gamepad", "controller")):
            if kw in name:
                return i
        return 99

    candidates.sort(key=score)
    path, name = candidates[0]
    print(f"[Init] Selected controller: {name} ({path})")
    return path


# ==================== TOASTS ====================
class ToastManager:
    """Thread-safe slide-in notifications; any thread adds, video loop draws."""
    LIFE = 2.6
    SLIDE = 0.18

    def __init__(self):
        self._lock = threading.Lock()
        self._items = []           # (msg, color, t_added)

    def add(self, msg, color=None):
        with self._lock:
            self._items.append((str(msg)[:70], color or TEXT_HI, time()))
            if len(self._items) > 4:
                self._items = self._items[-4:]
        print(f"[Toast] {msg}")

    def __call__(self, msg, color=None):
        self.add(msg, color)

    def draw(self, canvas):
        now = time()
        with self._lock:
            self._items = [it for it in self._items if now - it[2] < self.LIFE]
            items = list(self._items)
        W = canvas.shape[1]
        y = 70
        for msg, color, t0 in items:
            age = now - t0
            # slide down on entry, fade near the end
            slide = min(1.0, age / self.SLIDE)
            alpha = 1.0 if age < self.LIFE - 0.5 else max(0.0, (self.LIFE - age) / 0.5)
            (tw, th), _ = cv2.getTextSize(msg, FONT, 0.6, 1)
            x1 = (W - tw - 40) // 2
            yy = int(y - (1.0 - slide) * 24)
            overlay = canvas.copy()
            cv2.rectangle(overlay, (x1, yy - th - 12), (x1 + tw + 40, yy + 12), BG_PANEL, -1)
            cv2.rectangle(overlay, (x1, yy - th - 12), (x1 + tw + 40, yy + 12), color, 1)
            cv2.putText(overlay, msg, (x1 + 20, yy), FONT, 0.6, color, 1, cv2.LINE_AA)
            a = 0.92 * alpha
            cv2.addWeighted(overlay, a, canvas, 1 - a, 0, canvas)
            y += th + 34


TOASTS = ToastManager()


# ==================== SHARED UI STATE (threads <-> video) ====================
class UIState:
    def __init__(self):
        self._lock = threading.Lock()
        self.wheel_open = False
        self.wheel_sel = -1
        self.minimap_on = False
        # HUD / status
        self.hud_on = False
        self.ping_ms = None
        self.pose = "?"
        self.act = False               # robot activation state (from PONG)
        self.control_ok = False
        self.sticks = (0.0, 0.0, 0.0, 0.0)
        self.wifi_sig = None
        self.snapshot_flag = False
        self.back_to_launcher = False

    def set_wheel(self, is_open, sel):
        with self._lock:
            self.wheel_open = is_open
            self.wheel_sel = sel

    def get_wheel(self):
        with self._lock:
            return self.wheel_open, self.wheel_sel

    def set_minimap(self, v):
        with self._lock:
            self.minimap_on = v

    def get_minimap(self):
        with self._lock:
            return self.minimap_on

    def set(self, **kw):
        with self._lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def get(self, name):
        with self._lock:
            return getattr(self, name)

    def pop_snapshot(self):
        with self._lock:
            v = self.snapshot_flag
            self.snapshot_flag = False
            return v


# ==================== BATTERY ====================
def get_battery():
    """Return battery percent 0-100, or None if unknown/disabled.
    Wire this up once the Pi reports voltage (e.g. over a small status feed)."""
    if not BATTERY_ENABLED:
        return None
    return None


def draw_battery(canvas):
    """Small battery glyph, top-right of any screen."""
    W = canvas.shape[1]
    bx2 = W - 16
    bx1 = bx2 - 56
    by1, by2 = 14, 40
    pct = get_battery()

    body = TEXT_MID if pct is None else (
        TEXT_ERR if pct <= 15 else ACCENT_GREEN if pct >= 35 else (60, 180, 220))
    cv2.rectangle(canvas, (bx1, by1), (bx2, by2), body, 1)
    cv2.rectangle(canvas, (bx2, by1 + 7), (bx2 + 5, by2 - 7), body, -1)

    if pct is None:
        cv2.putText(canvas, "--", (bx1 + 16, by2 - 8), FONT, 0.5, TEXT_MID, 1, cv2.LINE_AA)
    else:
        fillw = int((bx2 - bx1 - 6) * max(0, min(100, pct)) / 100.0)
        cv2.rectangle(canvas, (bx1 + 3, by1 + 3), (bx1 + 3 + fillw, by2 - 3), body, -1)
        cv2.putText(canvas, f"{pct:d}%", (bx1 - 44, by2 - 8), FONT, 0.5, body, 1, cv2.LINE_AA)


# ==================== MJPEG STREAM ====================
class MjpegClient:
    def __init__(self, url, io_timeout=2.0):
        self.url = url
        self.io_timeout = io_timeout
        self.resp = None
        self.buf = bytearray()

    def open(self):
        self.close()
        self.resp = urlopen(self.url, timeout=self.io_timeout)

    def close(self):
        try:
            if self.resp is not None:
                self.resp.close()
        except Exception:
            pass
        self.resp = None
        self.buf = bytearray()

    def read_frame(self):
        if self.resp is None:
            raise RuntimeError("MJPEG stream not open")
        SOI = b"\xff\xd8"
        EOI = b"\xff\xd9"
        while True:
            start = self.buf.find(SOI)
            if start != -1:
                end = self.buf.find(EOI, start + 2)
                if end != -1:
                    jpg = bytes(self.buf[start:end + 2])
                    del self.buf[:end + 2]
                    img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if img is None:
                        continue
                    return img
                if start > 0:
                    del self.buf[:start]
            try:
                chunk = self.resp.read(4096)
                if not chunk:
                    raise ConnectionError("MJPEG stream closed")
                self.buf.extend(chunk)
                if len(self.buf) > 2_000_000:
                    self.buf = self.buf[-500_000:]
            except (pysocket.timeout, TimeoutError) as e:
                raise TimeoutError("MJPEG read timeout") from e


# ==================== DRAW HELPERS ====================
def filled_rect(img, pt1, pt2, color, alpha=1.0):
    if alpha >= 1.0:
        cv2.rectangle(img, pt1, pt2, color, -1)
    else:
        overlay = img.copy()
        cv2.rectangle(overlay, pt1, pt2, color, -1)
        cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


def draw_text(img, text, x, y, scale=0.8, color=None, thick=1):
    color = color or TEXT_HI
    cv2.putText(img, text, (int(x), int(y)), FONT, scale, color, thick, cv2.LINE_AA)


def draw_center_text(img, text, y, scale=0.8, color=None, thick=1):
    color = color or TEXT_HI
    (tw, _), _ = cv2.getTextSize(text, FONT, scale, thick)
    x = (img.shape[1] - tw) // 2
    cv2.putText(img, text, (x, int(y)), FONT, scale, color, thick, cv2.LINE_AA)


def draw_header(canvas, title, ssid=None, connected=False):
    W = canvas.shape[1]
    filled_rect(canvas, (0, 0), (W, 60), BG_PANEL)
    cv2.line(canvas, (0, 60), (W, 60), ACCENT if connected else BORDER, 1)
    dot_color = ACCENT if connected else TEXT_MID
    cv2.circle(canvas, (32, 30), 6, dot_color, -1)
    draw_text(canvas, title, 50, 38, scale=0.85, color=TEXT_HI, thick=2)
    if ssid:
        badge_text = ssid if connected else f"{ssid} (new)"
        badge_color = ACCENT if connected else TEXT_MID
        (bw, _), _ = cv2.getTextSize(badge_text, FONT, 0.55, 1)
        bx = W - bw - 36 - 80
        filled_rect(canvas, (bx - 8, 18), (bx + bw + 8, 44), ACCENT_DIM if connected else BORDER)
        cv2.rectangle(canvas, (bx - 8, 18), (bx + bw + 8, 44), badge_color, 1)
        draw_text(canvas, badge_text, bx, 37, scale=0.55, color=badge_color)
    draw_battery(canvas)


def draw_hint_grid(canvas, hints, x, y, col_w=280):
    for i, (btn, desc) in enumerate(hints):
        col = i % 2
        row = i // 2
        rx = x + col * col_w
        ry = y + row * 48
        filled_rect(canvas, (rx, ry), (rx + col_w - 12, ry + 38), BG_PANEL)
        cv2.rectangle(canvas, (rx, ry), (rx + col_w - 12, ry + 38), BORDER, 1)
        (bw, _), _ = cv2.getTextSize(btn, FONT, 0.5, 1)
        filled_rect(canvas, (rx + 8, ry + 8), (rx + 8 + bw + 10, ry + 30), (31, 41, 55))
        cv2.rectangle(canvas, (rx + 8, ry + 8), (rx + 8 + bw + 10, ry + 30), (55, 65, 81), 1)
        draw_text(canvas, btn, rx + 13, ry + 25, scale=0.5, color=TEXT_MID)
        draw_text(canvas, desc, rx + 8 + bw + 18, ry + 25, scale=0.55, color=TEXT_MID)


def point_in_rect(x, y, rect):
    x1, y1, x2, y2 = rect
    return x1 <= x <= x2 and y1 <= y <= y2


def draw_connecting_overlay(canvas, url, subtitle="Connecting to camera stream"):
    """Header/panel style spinner; also used as the RECONNECTING overlay."""
    global RESET_BTN_RECT
    W, H = canvas.shape[1], canvas.shape[0]
    draw_center_text(canvas, subtitle, H // 2 - 40, scale=0.9, color=TEXT_HI, thick=2)
    draw_center_text(canvas, url, H // 2, scale=0.6, color=TEXT_MID)

    t = time()
    dot_r, gap, total = 7, 26, 3
    start_x = W // 2 - (total - 1) * gap // 2
    for i in range(total):
        phase = t * 2.2 - i * 0.6
        b = 0.35 + 0.65 * (0.5 + 0.5 * math.sin(phase))
        color = (int(ACCENT[0] * b), int(ACCENT[1] * b), int(ACCENT[2] * b))
        cv2.circle(canvas, (start_x + i * gap, H // 2 + 50), dot_r, color, -1)

    btn_text = "Not loading? Tap here to change IP"
    (bw, bh), _ = cv2.getTextSize(btn_text, FONT, 0.58, 1)
    bx = (W - bw - 32) // 2
    by = H - 96
    btn_rect = (bx, by, bx + bw + 32, by + bh + 18)
    RESET_BTN_RECT = btn_rect
    filled_rect(canvas, (btn_rect[0], btn_rect[1]), (btn_rect[2], btn_rect[3]), BG_PANEL)
    cv2.rectangle(canvas, (btn_rect[0], btn_rect[1]), (btn_rect[2], btn_rect[3]), BORDER, 1)
    draw_text(canvas, btn_text, bx + 16, by + bh + 8, scale=0.58, color=TEXT_MID)


# ==================== EMOTE WHEEL ====================
def wheel_select_from_stick(lx, ly):
    mag = math.hypot(lx, ly)
    if mag < WHEEL_SELECT_DZ:
        return -1
    n = len(EMOTES)
    ang = (math.degrees(math.atan2(lx, -ly)) + 360.0) % 360.0
    seg = 360.0 / n
    return int((ang + seg / 2) // seg) % n


def draw_emote_wheel(canvas, selected):
    W, H = canvas.shape[1], canvas.shape[0]
    cx, cy = W // 2, H // 2
    R_out = 250
    R_in = 110
    R_lbl = 185
    n = len(EMOTES)

    overlay = canvas.copy()
    cv2.circle(overlay, (cx, cy), R_out + 26, (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.45, canvas, 0.55, 0, canvas)

    seg = 2 * math.pi / n
    for i, (token, label) in enumerate(EMOTES):
        a0 = -math.pi / 2 + (i - 0.5) * seg
        a1 = -math.pi / 2 + (i + 0.5) * seg
        is_sel = (i == selected)

        pts = []
        steps = 8
        for k in range(steps + 1):
            a = a0 + (a1 - a0) * k / steps
            pts.append((int(cx + R_out * math.cos(a)), int(cy + R_out * math.sin(a))))
        for k in range(steps + 1):
            a = a1 - (a1 - a0) * k / steps
            pts.append((int(cx + R_in * math.cos(a)), int(cy + R_in * math.sin(a))))
        poly = np.array(pts, np.int32)
        cv2.fillPoly(canvas, [poly], ACCENT_DIM if is_sel else BG_PANEL)
        cv2.polylines(canvas, [poly], True, ACCENT if is_sel else BORDER,
                      2 if is_sel else 1, cv2.LINE_AA)

        am = -math.pi / 2 + i * seg
        lx = int(cx + R_lbl * math.cos(am))
        ly = int(cy + R_lbl * math.sin(am))
        sc = 0.5
        (tw, th), _ = cv2.getTextSize(label, FONT, sc, 1)
        draw_text(canvas, label, lx - tw // 2, ly + th // 2,
                  scale=sc, color=TEXT_HI if is_sel else TEXT_MID,
                  thick=2 if is_sel else 1)

    cv2.circle(canvas, (cx, cy), R_in - 6, BG_DARK, -1)
    cv2.circle(canvas, (cx, cy), R_in - 6, ACCENT if selected >= 0 else BORDER, 2)
    if selected >= 0:
        name = label_for(EMOTE_TOKENS[selected])
        draw_center_text(canvas, name, cy - 4, scale=0.8, color=TEXT_HI, thick=2)
        draw_center_text(canvas, "release LB to play", cy + 26, scale=0.45, color=TEXT_MID)
    else:
        draw_center_text(canvas, "Emotes", cy - 4, scale=0.8, color=TEXT_MID, thick=2)
        draw_center_text(canvas, "left stick to aim", cy + 26, scale=0.45, color=TEXT_MID)


# ==================== CUSTOM NUMERIC KEYPAD ====================
KEYPAD_KEYS = ["1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "Enter"]


def keypad_layout(screen_w, screen_h):
    panel_w, panel_h = 520, 450
    panel_x = (screen_w - panel_w) // 2
    panel_y = 360
    cols, cell_w, cell_h, gap_x, gap_y = 3, 140, 58, 18, 16
    total_w = cols * cell_w + (cols - 1) * gap_x
    start_x = panel_x + (panel_w - total_w) // 2
    start_y = panel_y + 95
    rects = []
    for idx in range(len(KEYPAD_KEYS)):
        r, c = idx // cols, idx % cols
        x1 = start_x + c * (cell_w + gap_x)
        y1 = start_y + r * (cell_h + gap_y)
        rects.append((x1, y1, x1 + cell_w, y1 + cell_h))
    panel_rect = (panel_x, panel_y, panel_x + panel_w, panel_y + panel_h)
    return panel_rect, rects


def draw_keypad(canvas, selected_idx):
    panel_rect, rects = keypad_layout(canvas.shape[1], canvas.shape[0])
    x1, y1, x2, y2 = panel_rect
    filled_rect(canvas, (x1, y1), (x2, y2), BG_PANEL, alpha=0.97)
    filled_rect(canvas, (x1, y1), (x2, y1 + 52), (28, 34, 42))
    cv2.line(canvas, (x1, y1 + 52), (x2, y1 + 52), BORDER, 1)
    draw_center_text(canvas, "Enter IP Address", y1 + 32, scale=0.75, color=TEXT_HI, thick=2)
    cv2.rectangle(canvas, (x1, y1), (x2, y2), ACCENT, 2)
    draw_center_text(canvas, "Stick / D-pad to move   A = select   B = delete",
                     y1 + 75, scale=0.48, color=TEXT_MID)
    for idx, label in enumerate(KEYPAD_KEYS):
        kx1, ky1, kx2, ky2 = rects[idx]
        is_selected = (idx == selected_idx)
        is_enter = (label == "Enter")
        if is_enter:
            fill = ACCENT if is_selected else (35, 35, 80); border = ACCENT; txt = TEXT_HI
        elif is_selected:
            fill = ACCENT_DIM; border = ACCENT; txt = TEXT_HI
        else:
            fill = BG_DARK; border = BORDER; txt = TEXT_MID
        filled_rect(canvas, (kx1, ky1), (kx2, ky2), fill)
        cv2.rectangle(canvas, (kx1, ky1), (kx2, ky2), border, 2 if (is_selected or is_enter) else 1)
        scale = 0.95 if label.isdigit() or label == "." else 0.7
        thick = 2 if is_selected else 1
        (tw, th), _ = cv2.getTextSize(label, FONT, scale, thick)
        tx = kx1 + ((kx2 - kx1) - tw) // 2
        ty = ky1 + ((ky2 - ky1) + th) // 2 - 2
        draw_text(canvas, label, tx, ty, scale=scale, color=txt, thick=thick)
        if is_selected and not is_enter:
            cv2.circle(canvas, (kx2 - 8, ky1 + 8), 3, ACCENT, -1)


def keypad_move(selected_idx, direction):
    row, col = selected_idx // 3, selected_idx % 3
    if direction == "left":   col = max(0, col - 1)
    elif direction == "right": col = min(2, col + 1)
    elif direction == "up":    row = max(0, row - 1)
    elif direction == "down":  row = min(3, row + 1)
    return row * 3 + col


def keypad_apply_key(current_text, selected_idx):
    key = KEYPAD_KEYS[selected_idx]
    if key == "Enter":
        return current_text, "enter"
    if key == ".":
        if not current_text or current_text.endswith(".") or current_text.count(".") >= 3:
            return current_text, None
        return current_text + ".", None
    candidate = current_text + key
    if len(candidate) <= 15:
        return candidate, None
    return current_text, None


def hit_test_keypad(x, y, screen_w, screen_h):
    _, rects = keypad_layout(screen_w, screen_h)
    for idx, rect in enumerate(rects):
        if point_in_rect(x, y, rect):
            return idx
    return None


# ==================== UDP DISCOVERY ====================
class BeaconListener:
    """Listens for the Pi's 'CORNDOG <hostname>' UDP broadcast. The Pi's IP
    is simply the packet's source address."""

    def __init__(self, port=DISCOVERY_PORT):
        self._lock = threading.Lock()
        self.found_ip = None
        self.found_name = None
        self._run = True
        self._sock = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("", port))
            s.settimeout(0.3)
            self._sock = s
            threading.Thread(target=self._loop, daemon=True).start()
        except Exception as e:
            print(f"[Discovery] listener unavailable: {e}")

    def _loop(self):
        while self._run and self._sock is not None:
            try:
                data, addr = self._sock.recvfrom(128)
                text = data.decode("utf-8", errors="ignore")
                if text.startswith("CORNDOG"):
                    name = text.split(None, 1)[1].strip() if " " in text else "corndog"
                    with self._lock:
                        self.found_ip = addr[0]
                        self.found_name = name
            except socket.timeout:
                continue
            except Exception:
                sleep(0.3)

    def get(self):
        with self._lock:
            return self.found_ip, self.found_name

    def stop(self):
        self._run = False
        try:
            if self._sock is not None:
                self._sock.close()
        except Exception:
            pass


# ==================== IP SETUP UI (MAIN THREAD, shared window) ====================
def choose_pi_ip_with_ui(controller_dev, force_manual=False):
    ssid = get_active_ssid()
    cfg = load_config()
    if not force_manual and ssid and ssid in cfg.get("wifi_to_ip", {}):
        return cfg["wifi_to_ip"][ssid]

    beacon = BeaconListener()

    typed = (cfg.get("last_ip") or "")
    keypad_open = False
    keypad_selected = 0
    last_nav_time = 0.0
    nav_repeat_delay = 0.18
    ip_box_rect = (40, 120, SCREEN_W - 40, 175)
    found_rect = None
    touch_state = {"clicked": False, "x": 0, "y": 0}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            touch_state["clicked"] = True
            touch_state["x"], touch_state["y"] = x, y

    cv2.setMouseCallback(WINDOW, on_mouse)

    def save_and_return(ip):
        beacon.stop()
        new_cfg = load_config()
        new_cfg["last_ip"] = ip
        if ssid:
            new_cfg.setdefault("wifi_to_ip", {})[ssid] = ip
        save_config(new_cfg)
        return ip

    while True:
        canvas = np.full((SCREEN_H, SCREEN_W, 3), BG_DARK, dtype=np.uint8)
        ssid_known = bool(ssid and ssid in cfg.get("wifi_to_ip", {}))
        draw_header(canvas, "Corndog Communication", ssid=ssid or "Wi-Fi not detected",
                    connected=ssid_known)
        draw_text(canvas, "RASPBERRY PI IP ADDRESS", 40, 110, scale=0.5, color=TEXT_MID)
        box_y1, box_y2 = 120, 175
        filled_rect(canvas, (40, box_y1), (SCREEN_W - 40, box_y2), BG_PANEL)
        border_col = ACCENT_GREEN if validate_ip(typed) else (TEXT_ERR if typed else BORDER)
        cv2.rectangle(canvas, (40, box_y1), (SCREEN_W - 40, box_y2), border_col, 2)
        cursor_char = "|" if int(time() * 2) % 2 == 0 else " "
        draw_text(canvas, (typed or "") + cursor_char, 60, 160, scale=1.3, color=TEXT_HI, thick=2)
        if typed:
            ok = validate_ip(typed)
            draw_text(canvas, "Valid IP" if ok else "Invalid IP", 40, 205, scale=0.6,
                      color=ACCENT_GREEN if ok else TEXT_ERR)

        # discovery banner
        found_ip, found_name = beacon.get()
        found_rect = None
        if found_ip:
            msg = f"Found '{found_name}' at {found_ip}  —  Y (or tap) to use it"
            (tw, th), _ = cv2.getTextSize(msg, FONT, 0.62, 1)
            fx = (SCREEN_W - tw - 36) // 2
            fy = 212
            found_rect = (fx, fy, fx + tw + 36, fy + th + 20)
            filled_rect(canvas, (found_rect[0], found_rect[1]),
                        (found_rect[2], found_rect[3]), ACCENT_GREEN_DIM)
            cv2.rectangle(canvas, (found_rect[0], found_rect[1]),
                          (found_rect[2], found_rect[3]), ACCENT_GREEN, 1)
            draw_text(canvas, msg, fx + 18, fy + th + 8, scale=0.62, color=ACCENT_GREEN)

        cv2.line(canvas, (40, 254), (SCREEN_W - 40, 254), BORDER, 1)
        hints = [("OPTIONS", "Open keypad"), ("A", "Select key"),
                 ("B", "Delete character"), ("Y", "Use discovered IP"),
                 ("GUIDE", "Quit")]
        draw_hint_grid(canvas, hints, x=40, y=268, col_w=(SCREEN_W - 80) // 2)
        if keypad_open:
            draw_keypad(canvas, keypad_selected)
            draw_center_text(canvas, "Tap a key, or select Enter to save & connect",
                             SCREEN_H - 36, scale=0.58, color=TEXT_MID)
        else:
            draw_center_text(canvas, "Listening for Corndog's beacon... "
                             "tap the IP box or press OPTIONS to type instead",
                             SCREEN_H - 36, scale=0.55, color=TEXT_MID)

        cv2.imshow(WINDOW, canvas)

        if touch_state["clicked"]:
            tx, ty = touch_state["x"], touch_state["y"]
            touch_state["clicked"] = False
            if found_rect and point_in_rect(tx, ty, found_rect) and found_ip:
                return save_and_return(found_ip)
            if point_in_rect(tx, ty, ip_box_rect):
                keypad_open = True
            elif keypad_open:
                hit = hit_test_keypad(tx, ty, SCREEN_W, SCREEN_H)
                if hit is not None:
                    keypad_selected = hit
                    typed, action = keypad_apply_key(typed, keypad_selected)
                    if action == "enter" and validate_ip(typed):
                        return save_and_return(typed)

        if controller_dev is not None:
            try:
                r, _, _ = select.select([controller_dev.fd], [], [], 0)
                if r:
                    for ev in controller_dev.read():
                        if ev.type == ecodes.EV_KEY and ev.value == 1:
                            if ev.code == BTN_START:
                                keypad_open = not keypad_open
                            elif ev.code == BTN_GUIDE:
                                beacon.stop()
                                raise SystemExit(0)
                            elif ev.code == BTN_Y and found_ip:
                                return save_and_return(found_ip)
                            elif ev.code == BTN_A and keypad_open:
                                typed, action = keypad_apply_key(typed, keypad_selected)
                                if action == "enter" and validate_ip(typed):
                                    return save_and_return(typed)
                            elif ev.code == BTN_B:
                                typed = typed[:-1]
                        elif keypad_open and ev.type == ecodes.EV_ABS:
                            now = time()
                            if now - last_nav_time < nav_repeat_delay:
                                continue
                            moved = False
                            if ev.code == ABS_LX:
                                if ev.value < -12000: keypad_selected = keypad_move(keypad_selected, "left"); moved = True
                                elif ev.value > 12000: keypad_selected = keypad_move(keypad_selected, "right"); moved = True
                            elif ev.code == ABS_LY:
                                if ev.value < -12000: keypad_selected = keypad_move(keypad_selected, "up"); moved = True
                                elif ev.value > 12000: keypad_selected = keypad_move(keypad_selected, "down"); moved = True
                            elif ev.code == ABS_HAT_X:
                                if ev.value == -1: keypad_selected = keypad_move(keypad_selected, "left"); moved = True
                                elif ev.value == 1: keypad_selected = keypad_move(keypad_selected, "right"); moved = True
                            elif ev.code == ABS_HAT_Y:
                                if ev.value == -1: keypad_selected = keypad_move(keypad_selected, "up"); moved = True
                                elif ev.value == 1: keypad_selected = keypad_move(keypad_selected, "down"); moved = True
                            if moved:
                                last_nav_time = now
            except OSError:
                controller_dev = None

        key = cv2.waitKey(16) & 0xFF
        if key == 27:
            beacon.stop()
            raise SystemExit(0)
        if key in (8, 127):
            typed = typed[:-1]; continue
        if key in (10, 13):
            if validate_ip(typed):
                return save_and_return(typed)
            continue
        if key in tuple(ord(c) for c in "0123456789"):
            candidate = typed + chr(key)
            if len(candidate) <= 15:
                typed = candidate
            continue
        if key == ord('.'):
            if typed and not typed.endswith('.') and typed.count('.') < 3:
                typed += '.'
            continue
        if key in (ord('o'), ord('O')):
            keypad_open = not keypad_open; continue


# ==================== LAUNCHER (mode select) ====================
def launcher_screen(controller_dev, pi_ip):
    """Pick a session mode. Returns 'drive' | 'drive_map' | 'map' |
    'change_ip' | None (quit). X opens Settings."""
    cfg = load_config()
    sel = next((i for i, m in enumerate(MODES) if m[0] == cfg.get("mode")), 0)
    ssid = get_active_ssid()

    card_w, card_h = 350, 330
    gap = 40
    total = 3 * card_w + 2 * gap
    x0 = (SCREEN_W - total) // 2
    y0 = 170
    card_rects = [(x0 + i * (card_w + gap), y0,
                   x0 + i * (card_w + gap) + card_w, y0 + card_h) for i in range(3)]
    settings_rect = (SCREEN_W // 2 - 190, y0 + card_h + 60,
                     SCREEN_W // 2 + 190, y0 + card_h + 116)

    touch = {"clicked": False, "x": 0, "y": 0}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            touch["clicked"] = True
            touch["x"], touch["y"] = x, y

    cv2.setMouseCallback(WINDOW, on_mouse)

    def commit(mode_id):
        c = load_config()
        c["mode"] = mode_id
        save_config(c)
        return mode_id

    last_nav = 0.0
    while True:
        canvas = np.full((SCREEN_H, SCREEN_W, 3), BG_DARK, dtype=np.uint8)
        draw_header(canvas, "Corndog Communication", ssid=ssid, connected=True)
        draw_center_text(canvas, f"Robot: {pi_ip}", 100, scale=0.6, color=TEXT_MID)
        draw_center_text(canvas, "CHOOSE SESSION", 140, scale=0.75, color=TEXT_HI, thick=2)

        for i, (mode_id, title, d1, d2) in enumerate(MODES):
            x1, y1, x2, y2 = card_rects[i]
            hot = (i == sel)
            filled_rect(canvas, (x1, y1), (x2, y2), ACCENT_DIM if hot else BG_PANEL)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), ACCENT if hot else BORDER,
                          2 if hot else 1)
            # icon block
            icx, icy = (x1 + x2) // 2, y1 + 92
            if mode_id == "drive":
                cv2.circle(canvas, (icx, icy), 40, TEXT_MID if not hot else TEXT_HI, 2)
                cv2.circle(canvas, (icx, icy), 12, ACCENT_GREEN if hot else TEXT_MID, -1)
            elif mode_id == "drive_map":
                cv2.circle(canvas, (icx - 18, icy), 34, TEXT_MID if not hot else TEXT_HI, 2)
                cv2.rectangle(canvas, (icx + 8, icy + 2), (icx + 52, icy + 44),
                              ACCENT_GREEN if hot else TEXT_MID, 2)
            else:
                cv2.rectangle(canvas, (icx - 44, icy - 34), (icx + 44, icy + 40),
                              TEXT_MID if not hot else TEXT_HI, 2)
                cv2.circle(canvas, (icx - 14, icy + 12), 6, ACCENT_GREEN if hot else TEXT_MID, -1)
                cv2.line(canvas, (icx - 14, icy + 12), (icx + 22, icy - 14),
                         ACCENT_GREEN if hot else TEXT_MID, 2)
                cv2.circle(canvas, (icx + 22, icy - 14), 8,
                           (255, 92, 255) if hot else TEXT_MID, 2)

            (tw, _), _ = cv2.getTextSize(title, FONT, 0.72, 2)
            draw_text(canvas, title, (x1 + x2 - tw) // 2, y1 + 190, scale=0.72,
                      color=TEXT_HI if hot else TEXT_MID, thick=2)
            for j, dline in enumerate((d1, d2)):
                (tw2, _), _ = cv2.getTextSize(dline, FONT, 0.48, 1)
                draw_text(canvas, dline, (x1 + x2 - tw2) // 2, y1 + 228 + j * 26,
                          scale=0.48, color=TEXT_MID)
            if cfg.get("mode") == mode_id:
                draw_text(canvas, "last used", x1 + 12, y2 - 14, scale=0.42,
                          color=ACCENT_GREEN)

        # settings button
        sx1, sy1, sx2, sy2 = settings_rect
        filled_rect(canvas, (sx1, sy1), (sx2, sy2), BG_PANEL)
        cv2.rectangle(canvas, (sx1, sy1), (sx2, sy2), BORDER, 1)
        draw_center_text(canvas, "X / tap:  SETTINGS & GAIT TUNING", sy1 + 36,
                         scale=0.6, color=TEXT_MID)

        draw_center_text(canvas, "Stick/D-pad: choose    A: start    Y: change IP    Guide: quit",
                         SCREEN_H - 30, scale=0.55, color=TEXT_MID)
        cv2.imshow(WINDOW, canvas)

        if touch["clicked"]:
            tx, ty = touch["x"], touch["y"]
            touch["clicked"] = False
            for i, rect in enumerate(card_rects):
                if point_in_rect(tx, ty, rect):
                    return commit(MODES[i][0])
            if point_in_rect(tx, ty, settings_rect):
                settings_screen(controller_dev)
                cv2.setMouseCallback(WINDOW, on_mouse)
                cfg = load_config()

        if controller_dev is not None:
            try:
                r, _, _ = select.select([controller_dev.fd], [], [], 0)
                if r:
                    for ev in controller_dev.read():
                        if ev.type == ecodes.EV_KEY and ev.value == 1:
                            if ev.code == BTN_GUIDE:
                                return None
                            if ev.code == BTN_A:
                                return commit(MODES[sel][0])
                            if ev.code == BTN_X:
                                settings_screen(controller_dev)
                                cv2.setMouseCallback(WINDOW, on_mouse)
                                cfg = load_config()
                            if ev.code == BTN_Y:
                                return "change_ip"
                        elif ev.type == ecodes.EV_ABS:
                            now = time()
                            if now - last_nav < 0.18:
                                continue
                            v = ev.value
                            if ev.code == ABS_LX and abs(v) > 12000:
                                sel = (sel + (1 if v > 0 else -1)) % 3
                                last_nav = now
                            elif ev.code == ABS_HAT_X and v != 0:
                                sel = (sel + (1 if v > 0 else -1)) % 3
                                last_nav = now
            except OSError:
                controller_dev = None

        key = cv2.waitKey(16) & 0xFF
        if key == 27:
            return None
        if key in (10, 13):
            return commit(MODES[sel][0])


# ==================== SETTINGS SCREEN ====================
def _fmt_setting(spec, v):
    if spec["type"] == "bool":
        return "ON" if v else "OFF"
    if spec["type"] == "choice":
        return str(v)
    step = spec.get("step", 0.01)
    return f"{v:.3f}".rstrip("0").rstrip(".") if step < 1 else f"{v:g}"


def _adjust_setting(cfg, spec, direction):
    key = spec["key"]
    cur = get_setting(cfg, key)
    if spec["type"] == "bool":
        set_setting(cfg, key, not cur)
    elif spec["type"] == "choice":
        ch = spec["choices"]
        i = (ch.index(cur) if cur in ch else 0) + direction
        set_setting(cfg, key, ch[i % len(ch)])
    else:
        v = cur + direction * spec["step"]
        v = max(spec["min"], min(spec["max"], round(v, 4)))
        set_setting(cfg, key, v)


def settings_screen(controller_dev):
    """Scrollable settings list. Left/right (or A) adjusts, B saves & exits,
    X resets everything to defaults. Robot-scope rows take effect on the next
    control (re)connect."""
    cfg = load_config()
    sel = 0
    scroll = 0
    ROWS = 11
    row_h = 56
    top = 120
    last_nav = 0.0

    touch = {"clicked": False, "x": 0, "y": 0}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            touch["clicked"] = True
            touch["x"], touch["y"] = x, y

    cv2.setMouseCallback(WINDOW, on_mouse)

    while True:
        canvas = np.full((SCREEN_H, SCREEN_W, 3), BG_DARK, dtype=np.uint8)
        draw_header(canvas, "Settings", connected=True)
        draw_text(canvas, "robot-scope rows push to the Pi as GAIT commands on connect",
                  40, 92, scale=0.5, color=TEXT_MID)

        if sel < scroll:
            scroll = sel
        if sel >= scroll + ROWS:
            scroll = sel - ROWS + 1

        for r in range(ROWS):
            idx = scroll + r
            if idx >= len(SETTINGS_SPEC):
                break
            spec = SETTINGS_SPEC[idx]
            v = get_setting(cfg, spec["key"])
            y1 = top + r * row_h
            hot = (idx == sel)
            filled_rect(canvas, (40, y1), (SCREEN_W - 40, y1 + row_h - 8),
                        ACCENT_DIM if hot else BG_PANEL)
            cv2.rectangle(canvas, (40, y1), (SCREEN_W - 40, y1 + row_h - 8),
                          ACCENT if hot else BORDER, 1)
            draw_text(canvas, spec["label"], 64, y1 + 33, scale=0.62,
                      color=TEXT_HI if hot else TEXT_MID, thick=2 if hot else 1)
            tag = "robot" if spec["scope"] == "gait" else "deck"
            tag_col = ACCENT_GREEN if spec["scope"] == "gait" else TEXT_MID
            draw_text(canvas, tag, 480, y1 + 33, scale=0.45, color=tag_col)

            val_text = _fmt_setting(spec, v)
            is_default = (v == spec["default"])
            (tw, _), _ = cv2.getTextSize(val_text, FONT, 0.65, 2)
            vx = SCREEN_W - 120 - tw // 2
            if hot:
                draw_text(canvas, "<", vx - 60, y1 + 34, scale=0.7, color=TEXT_MID, thick=2)
                draw_text(canvas, ">", vx + tw + 44, y1 + 34, scale=0.7, color=TEXT_MID, thick=2)
            draw_text(canvas, val_text, vx, y1 + 34, scale=0.65,
                      color=(TEXT_HI if hot else TEXT_MID) if not is_default else
                            (ACCENT_GREEN if hot else TEXT_MID),
                      thick=2 if hot else 1)

        # scrollbar
        if len(SETTINGS_SPEC) > ROWS:
            bar_h = int(ROWS / len(SETTINGS_SPEC) * (ROWS * row_h))
            bar_y = top + int(scroll / len(SETTINGS_SPEC) * (ROWS * row_h))
            cv2.rectangle(canvas, (SCREEN_W - 30, bar_y), (SCREEN_W - 24, bar_y + bar_h),
                          BORDER, -1)

        draw_center_text(canvas,
                         "Up/Down: row    Left/Right: adjust    A: toggle    X: reset all    B: done",
                         SCREEN_H - 26, scale=0.55, color=TEXT_MID)
        cv2.imshow(WINDOW, canvas)

        if touch["clicked"]:
            tx, ty = touch["x"], touch["y"]
            touch["clicked"] = False
            for r in range(ROWS):
                idx = scroll + r
                if idx >= len(SETTINGS_SPEC):
                    break
                y1 = top + r * row_h
                if 40 <= tx <= SCREEN_W - 40 and y1 <= ty <= y1 + row_h - 8:
                    if idx == sel:
                        _adjust_setting(cfg, SETTINGS_SPEC[idx],
                                        -1 if tx < SCREEN_W - 130 - 60 and tx > SCREEN_W // 2 else 1)
                    sel = idx

        if controller_dev is not None:
            try:
                r, _, _ = select.select([controller_dev.fd], [], [], 0)
                if r:
                    for ev in controller_dev.read():
                        if ev.type == ecodes.EV_KEY and ev.value == 1:
                            if ev.code in (BTN_B, BTN_GUIDE, BTN_START):
                                save_config(cfg)
                                return
                            if ev.code == BTN_A:
                                _adjust_setting(cfg, SETTINGS_SPEC[sel], +1)
                            if ev.code == BTN_X:
                                cfg["settings"] = {}
                                TOASTS.add("settings reset to defaults", TEXT_HI)
                        elif ev.type == ecodes.EV_ABS:
                            now = time()
                            v = ev.value
                            repeat = 0.16
                            if now - last_nav < repeat:
                                continue
                            if ev.code in (ABS_LY,) and abs(v) > 12000:
                                sel = max(0, min(len(SETTINGS_SPEC) - 1,
                                                 sel + (1 if v > 0 else -1)))
                                last_nav = now
                            elif ev.code == ABS_HAT_Y and v != 0:
                                sel = max(0, min(len(SETTINGS_SPEC) - 1, sel + v))
                                last_nav = now
                            elif ev.code in (ABS_LX,) and abs(v) > 12000:
                                _adjust_setting(cfg, SETTINGS_SPEC[sel], 1 if v > 0 else -1)
                                last_nav = now
                            elif ev.code == ABS_HAT_X and v != 0:
                                _adjust_setting(cfg, SETTINGS_SPEC[sel], v)
                                last_nav = now
            except OSError:
                controller_dev = None

        key = cv2.waitKey(16) & 0xFF
        if key == 27:
            save_config(cfg)
            return


# ==================== PRESENCE (WORKER THREAD, mode-aware) ====================
def presence_socket(stop_event, raspi_ip, mode_token):
    """Keeps the supervisor's presence port fed; the hello line tells the Pi
    which script family to boot (drive vs slam)."""
    hello = f"STEAMDECK_READY MODE={mode_token}\n".encode()
    while not stop_event.is_set():
        s = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(PRESENCE_CONNECT_TIMEOUT)
            s.connect((raspi_ip, PRESENCE_PORT))
            s.settimeout(None)
            try:
                s.sendall(hello)
            except Exception:
                pass
            while not stop_event.is_set():
                try:
                    s.sendall(b".")
                except Exception:
                    break
                sleep(PRESENCE_HEARTBEAT_S)
        except Exception:
            sleep(1.0)
        finally:
            try:
                if s is not None:
                    s.close()
            except Exception:
                pass


# ==================== WIFI SIGNAL POLLER ====================
def wifi_poller(stop_event, ui):
    while not stop_event.is_set():
        ui.set(wifi_sig=get_wifi_signal())
        stop_event.wait(5.0)


# ==================== LOCAL SNAP PREVIEW (for the debug HUD) ====================
def local_joystick_to_cmd(lx, ly, rx, ry, cfg):
    """Deck-side mirror of the Pi's snap logic, using the CONFIGURED speeds, so
    the HUD can show what the robot should be doing with the current sticks."""
    VX = get_setting(cfg, "vx_speed"); VY = get_setting(cfg, "vy_speed")
    DG = get_setting(cfg, "diag_speed"); WZ = get_setting(cfg, "wz_max")
    vx = vy = wz = 0.0
    mag = math.hypot(lx, ly)
    if mag >= 0.5:
        ang = (math.degrees(math.atan2(lx, -ly)) + 360.0) % 360.0
        dirs = [0, 45, 90, 135, 180, 225, 270, 315]
        nearest = min(dirs, key=lambda d: min((ang - d) % 360, (d - ang) % 360))
        if min((ang - nearest) % 360, (nearest - ang) % 360) <= 30.0:
            f = nearest in (0, 45, 315); b = nearest in (180, 135, 225)
            r = nearest in (90, 45, 135); l = nearest in (270, 225, 315)
            if (f or b) and (r or l):
                base = math.hypot(VX, VY)
                k = DG / base if base > 1e-9 else 0.0
                vx = VX * k * (1 if f else -1)
                vy = VY * k * (-1 if r else 1)
            else:
                vx = VX if f else (-VX if b else 0.0)
                vy = -VY if r else (VY if l else 0.0)
    ax = abs(rx)
    if ax >= 0.5:
        nx = min(1.0, (ax - 0.5) / 0.5)
        fade = 1.0 - min(1.0, abs(ry))
        wz = -WZ * math.copysign(nx * fade, rx)
    return vx, vy, wz


# ==================== CONTROLS (WORKER THREAD) ====================
def send_controls(stop_event, raspi_ip, device_path, ui, minimap, cfg):
    button_names = {
        304: "A", 305: "B", 307: "X", 308: "Y",
        310: "LB", 311: "RB",
        315: "LS_Press", 316: "RS_Press"
    }
    axis_names = {
        0: "Left Stick X", 1: "Left Stick Y", 2: "Left Trigger",
        3: "Right Stick X", 4: "Right Stick Y", 5: "Right Trigger"
    }
    last_vals = {}

    lx_raw = ly_raw = rx_raw = ry_raw = 0.0
    wheel_open = False
    right_zeroed = True
    last_hat_time = 0.0
    minimap_sizes = list(MINIMAP_SIZES.keys())

    try:
        dev = InputDevice(device_path)
        print(f"[Input] Using device: {dev.name} ({device_path})")
    except Exception as e:
        print(f"[Input] Failed to open controller device {device_path}: {e}")
        return

    sock_obj = None
    ping_seq = 0
    ping_sent = {}
    last_ping_t = 0.0
    rx_buf = b""

    def connect_socket(announce=True):
        """(Re)connect forever until stopped. Pushes saved gait config."""
        nonlocal sock_obj, rx_buf
        try:
            if sock_obj is not None:
                sock_obj.close()
        except Exception:
            pass
        rx_buf = b""
        ui.set(control_ok=False, ping_ms=None)
        while not stop_event.is_set():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(CONTROL_IO_TIMEOUT)
                s.connect((raspi_ip, CONTROL_PORT))
                s.settimeout(None)
                sock_obj = s
                ui.set(control_ok=True)
                if announce:
                    TOASTS.add("controls connected", ACCENT_GREEN)
                # push saved robot-scope settings (allowed pre-activation)
                try:
                    payload = "".join(line + "\n" for line in gait_settings_lines(cfg))
                    s.sendall(payload.encode())
                    print("[Input] pushed gait config")
                except Exception:
                    pass
                return True
            except (ConnectionRefusedError, TimeoutError, OSError):
                sleep(RETRY_SEC)
        return False

    if not connect_socket(announce=False):
        return

    def send(msg):
        sock_obj.sendall((msg + "\n").encode())

    def send_drive_stop():
        try:
            send("Left Stick X 0.0"); send("Left Stick Y 0.0")
            send("Right Stick X 0.0"); send("Right Stick Y 0.0")
        except Exception:
            pass
        last_vals[0] = last_vals[1] = last_vals[3] = last_vals[4] = 0.0

    def handle_pong(line):
        # "PONG <seq> pose=<pose> act=<0|1>"
        parts = line.split()
        if len(parts) < 2:
            return
        try:
            seq = int(parts[1])
        except ValueError:
            return
        t0 = ping_sent.pop(seq, None)
        kw = {}
        if t0 is not None:
            kw["ping_ms"] = (time() - t0) * 1000.0
        for p in parts[2:]:
            if p.startswith("pose="):
                kw["pose"] = p[5:]
            elif p.startswith("act="):
                kw["act"] = (p[4:] == "1")
        ui.set(**kw)

    try:
        while not stop_event.is_set():
            try:
                r, _, _ = select.select([dev.fd, sock_obj.fileno()], [], [], 0.1)
            except (OSError, ValueError):
                r = []

            now = time()
            # periodic PING for the HUD
            if now - last_ping_t >= PING_PERIOD_S:
                last_ping_t = now
                ping_seq += 1
                ping_sent[ping_seq] = now
                if len(ping_sent) > 8:
                    for k in sorted(ping_sent)[:-8]:
                        ping_sent.pop(k, None)
                try:
                    send(f"PING {ping_seq}")
                except Exception:
                    TOASTS.add("controls lost — reconnecting", TEXT_ERR)
                    if not connect_socket():
                        break
                    continue

            if not r:
                continue

            # ---- socket readable: PONG lines ----
            if sock_obj.fileno() in r:
                try:
                    data = sock_obj.recv(1024)
                except OSError:
                    data = b""
                if not data:
                    TOASTS.add("controls lost — reconnecting", TEXT_ERR)
                    if not connect_socket():
                        break
                    continue
                rx_buf += data
                while b"\n" in rx_buf:
                    line, rx_buf = rx_buf.split(b"\n", 1)
                    text = line.decode("utf-8", errors="ignore").strip()
                    if text.startswith("PONG"):
                        handle_pong(text)

            if dev.fd not in r:
                continue

            for ev in dev.read():
                if stop_event.is_set():
                    break
                try:
                    # ---------------- BUTTONS ----------------
                    if ev.type == ecodes.EV_KEY:
                        code, val = ev.code, ev.value

                        if code == BTN_GUIDE and val == 1:
                            stop_event.set()
                            break

                        if code == BTN_START and val == 1:
                            ui.set(back_to_launcher=True)
                            stop_event.set()
                            break

                        # Back/Select = deck-local snapshot
                        if code == BTN_BACK:
                            if val == 1:
                                ui.set(snapshot_flag=True)
                            continue

                        # LB = wheel modifier (never forwarded)
                        if code == BTN_LB:
                            if val == 1:
                                wheel_open = True
                                ui.set_wheel(True, wheel_select_from_stick(lx_raw, ly_raw))
                                send_drive_stop()
                            else:
                                _, sel = ui.get_wheel()
                                wheel_open = False
                                ui.set_wheel(False, -1)
                                if sel is not None and sel >= 0:
                                    token = EMOTE_TOKENS[sel]
                                    TOASTS.add(f"emote: {label_for(token)}", TEXT_HI)
                                    try:
                                        send(f"EMOTE {token}")
                                    except Exception:
                                        pass
                            continue

                        if wheel_open:
                            continue

                        if code in button_names:
                            name = button_names[code]
                            state = 'pressed' if val else 'released'
                            send(f"{name} {state}")
                            if code == BTN_A and val == 1 and not ui.get("act"):
                                ui.set(act=True)   # optimistic; PONG confirms

                    # ---------------- AXES ----------------
                    elif ev.type == ecodes.EV_ABS:
                        code = ev.code

                        # D-pad: Deck-local view config (NOT forwarded)
                        if code in (ABS_HAT_X, ABS_HAT_Y):
                            now = time()
                            if (now - last_hat_time) <= 0.30:
                                continue
                            if code == ABS_HAT_Y and ev.value == -1:
                                last_hat_time = now
                                if minimap is not None:
                                    if minimap.active:
                                        minimap.stop(); ui.set_minimap(False)
                                        TOASTS.add("minimap off")
                                    else:
                                        minimap.start(); ui.set_minimap(True)
                                        TOASTS.add("minimap on")
                            elif code == ABS_HAT_Y and ev.value == 1:
                                last_hat_time = now
                                new_hud = not ui.get("hud_on")
                                ui.set(hud_on=new_hud)
                                TOASTS.add("debug HUD " + ("on" if new_hud else "off"))
                            elif code == ABS_HAT_X and ev.value == 1:
                                last_hat_time = now
                                cur = get_setting(cfg, "minimap_size")
                                nxt = minimap_sizes[(minimap_sizes.index(cur) + 1)
                                                    % len(minimap_sizes)]
                                set_setting(cfg, "minimap_size", nxt)
                                save_config(cfg)
                                TOASTS.add(f"minimap size: {nxt}")
                            continue

                        if code == ABS_LX:   lx_raw = max(min(ev.value, 30000), -30000) / 30000.0
                        elif code == ABS_LY: ly_raw = max(min(ev.value, 30000), -30000) / 30000.0
                        elif code == ABS_RX: rx_raw = max(min(ev.value, 30000), -30000) / 30000.0
                        elif code == ABS_RY: ry_raw = max(min(ev.value, 30000), -30000) / 30000.0
                        ui.set(sticks=(lx_raw, ly_raw, rx_raw, ry_raw))

                        if wheel_open:
                            if code in (ABS_LX, ABS_LY):
                                ui.set_wheel(True, wheel_select_from_stick(lx_raw, ly_raw))
                            continue

                        # ---- LEFT stick: forward as-is (Pi does 8-dir snap) ----
                        if code in (ABS_LX, ABS_LY):
                            norm = round((lx_raw if code == ABS_LX else ly_raw), 1)
                            if abs(norm) < 0.1:
                                norm = 0.0
                            prev = last_vals.get(code)
                            if prev is None or abs(norm - prev) >= 0.1:
                                send(f"{axis_names[code]} {norm}")
                                last_vals[code] = norm

                        # ---- RIGHT stick: radial deadzone + clean zero ----
                        elif code in (ABS_RX, ABS_RY):
                            mag = math.hypot(rx_raw, ry_raw)
                            if mag < RIGHT_RADIAL_DEADZONE:
                                if not right_zeroed:
                                    send("Right Stick X 0.0")
                                    send("Right Stick Y 0.0")
                                    last_vals[ABS_RX] = 0.0
                                    last_vals[ABS_RY] = 0.0
                                    right_zeroed = True
                            else:
                                right_zeroed = False
                                norm = round(rx_raw if code == ABS_RX else ry_raw, 1)
                                if abs(norm) < 0.1:
                                    norm = 0.0
                                prev = last_vals.get(code)
                                if prev is None or abs(norm - prev) >= 0.1:
                                    send(f"{axis_names[code]} {norm}")
                                    last_vals[code] = norm

                        elif code in (2, 5):
                            pass

                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                    TOASTS.add("controls lost — reconnecting", TEXT_ERR)
                    if not connect_socket():
                        stop_event.set()
                    break
    finally:
        try:
            if sock_obj is not None:
                sock_obj.close()
        except Exception:
            pass
        try:
            dev.close()
        except Exception:
            pass


# ==================== DEBUG HUD ====================
def draw_debug_hud(canvas, ui, cfg, fps):
    x, y = 16, 76
    lines = []
    lines.append(f"video fps: {fps:4.1f}")
    ping = ui.get("ping_ms")
    ok = ui.get("control_ok")
    lines.append(f"control:  {'OK' if ok else 'RECONNECTING'}   "
                 f"ping: {f'{ping:.0f} ms' if ping is not None else '--'}")
    lines.append(f"robot:    pose={ui.get('pose')}  act={'yes' if ui.get('act') else 'no'}")
    lx, ly, rx, ry = ui.get("sticks")
    lines.append(f"sticks:   L({lx:+.2f},{ly:+.2f})  R({rx:+.2f},{ry:+.2f})")
    vx, vy, wz = local_joystick_to_cmd(lx, ly, rx, ry, cfg)
    lines.append(f"cmd(est): vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.2f}")
    sig = ui.get("wifi_sig")
    lines.append(f"wifi:     {f'{sig}%' if sig is not None else '--'}   "
                 f"minimap: {get_setting(cfg, 'minimap_size')}")

    w = 430
    h = 16 + len(lines) * 26 + 8
    filled_rect(canvas, (x - 6, y - 20), (x + w, y - 20 + h), BG_DARK, alpha=0.78)
    cv2.rectangle(canvas, (x - 6, y - 20), (x + w, y - 20 + h), BORDER, 1)
    for i, ln in enumerate(lines):
        draw_text(canvas, ln, x + 4, y + i * 26, scale=0.52,
                  color=TEXT_HI if i < 3 else TEXT_MID)


def save_snapshot(frame):
    try:
        SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime("%Y%m%d_%H%M%S") + ".png"
        path = SNAPSHOT_DIR / name
        cv2.imwrite(str(path), frame)
        TOASTS.add(f"snapshot saved: {name}", ACCENT_GREEN)
    except Exception as e:
        TOASTS.add(f"snapshot failed ({type(e).__name__})", TEXT_ERR)


# ==================== VIDEO LOOP (MAIN THREAD, shared window) ====================
def video_loop(stop_event, mjpeg_url, reset_requested, ui, minimap, cfg):
    mj = MjpegClient(mjpeg_url, io_timeout=MJPEG_IO_TIMEOUT)
    last_frame = None
    was_streaming = False
    fps = 0.0
    fps_t = time()
    fps_n = 0

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and RESET_BTN_RECT:
            rx1, ry1, rx2, ry2 = RESET_BTN_RECT
            if rx1 <= x <= rx2 and ry1 <= y <= ry2:
                reset_requested.set()

    cv2.setMouseCallback(WINDOW, on_mouse)

    while not stop_event.is_set():
        # ---------- (re)connect ----------
        if mj.resp is None:
            canvas = np.full((SCREEN_H, SCREEN_W, 3), BG_DARK, dtype=np.uint8)
            draw_header(canvas, "Corndog Communication", connected=False)
            draw_connecting_overlay(canvas, mjpeg_url,
                                    subtitle="Reconnecting to camera" if was_streaming
                                    else "Connecting to camera stream")
            TOASTS.draw(canvas)
            cv2.imshow(WINDOW, canvas)
            cv2.waitKey(16)
            if reset_requested.is_set():
                stop_event.set()
                break
            try:
                mj.open()
                if was_streaming:
                    TOASTS.add("camera back", ACCENT_GREEN)
                print("[Video] Connected to MJPEG stream")
            except (URLError, ConnectionError, OSError, TimeoutError):
                t_wait = time()
                while time() - t_wait < RETRY_SEC and not stop_event.is_set():
                    canvas = np.full((SCREEN_H, SCREEN_W, 3), BG_DARK, dtype=np.uint8)
                    draw_header(canvas, "Corndog Communication", connected=False)
                    draw_connecting_overlay(canvas, mjpeg_url,
                                            subtitle="Reconnecting to camera" if was_streaming
                                            else "Connecting to camera stream")
                    TOASTS.draw(canvas)
                    cv2.imshow(WINDOW, canvas)
                    cv2.waitKey(16)
                    if reset_requested.is_set():
                        stop_event.set()
                        break
            continue

        # ---------- read a frame ----------
        try:
            frame = mj.read_frame()
            if not was_streaming:
                was_streaming = True
            last_frame = frame
        except Exception as e:
            print(f"[Video] Stream lost ({e}) — will reconnect")
            TOASTS.add("video lost — reconnecting", TEXT_ERR)
            mj.close()
            continue

        fps_n += 1
        now = time()
        if now - fps_t >= 1.0:
            fps = fps_n / (now - fps_t)
            fps_n = 0
            fps_t = now

        if ui.pop_snapshot() and last_frame is not None:
            save_snapshot(last_frame)

        # ---------- compose ----------
        h, w = frame.shape[:2]
        scale = SCREEN_H / h
        new_w = int(w * scale)
        resized = cv2.resize(frame, (new_w, SCREEN_H))
        canvas = np.zeros((SCREEN_H, SCREEN_W, 3), dtype=np.uint8)
        if new_w <= SCREEN_W:
            x_off = (SCREEN_W - new_w) // 2
            canvas[:, x_off:x_off + new_w] = resized
        else:
            start = (new_w - SCREEN_W) // 2
            canvas[:, :] = resized[:, start:start + SCREEN_W]

        draw_battery(canvas)

        if minimap is not None and minimap.active:
            mm = MINIMAP_SIZES.get(get_setting(cfg, "minimap_size"), 240)
            mx2 = SCREEN_W - MINIMAP_MARGIN
            my2 = SCREEN_H - MINIMAP_MARGIN
            minimap.render(canvas, (mx2 - mm, my2 - mm, mx2, my2))

        if ui.get("hud_on"):
            draw_debug_hud(canvas, ui, cfg, fps)

        wheel_open, wheel_sel = ui.get_wheel()
        if wheel_open:
            draw_emote_wheel(canvas, wheel_sel)
        elif not ui.get("act"):
            bar_h = 48
            filled_rect(canvas, (0, SCREEN_H - bar_h), (SCREEN_W, SCREEN_H), BG_DARK, alpha=0.88)
            cv2.line(canvas, (0, SCREEN_H - bar_h), (SCREEN_W, SCREEN_H - bar_h), BORDER, 1)
            cx, cy = SCREEN_W // 2 - 140, SCREEN_H - bar_h // 2
            cv2.circle(canvas, (cx, cy), 14, ACCENT_GREEN_DIM, -1)
            cv2.circle(canvas, (cx, cy), 14, ACCENT_GREEN, 2)
            draw_text(canvas, "A", cx - 6, cy + 6, scale=0.6, color=ACCENT_GREEN, thick=2)
            draw_text(canvas, "Press A to enable controls", cx + 24, cy + 5,
                      scale=0.7, color=TEXT_HI)
        else:
            draw_text(canvas, "LB: emotes   dpad U: minimap  R: size  D: HUD   "
                      "Select: snapshot   Start: menu",
                      16, SCREEN_H - 16, scale=0.48, color=TEXT_MID)

        TOASTS.draw(canvas)
        cv2.imshow(WINDOW, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            stop_event.set()
            break

    mj.close()


# ==================== DRIVE SESSION ====================
def run_drive_session(pi_ip, controller_path, minimap_on_start, cfg):
    """One drive session (with or without minimap). Returns 'back', 'reset_ip',
    or 'exit'."""
    mjpeg_url = f"http://{pi_ip}:8000/stream.mjpg"

    ui = UIState()
    minimap = Minimap(pi_ip, port=SLAM_PORT, crop_radius_m=3.0, poll_hz=5.0)
    if minimap_on_start:
        minimap.start()
        ui.set_minimap(True)

    stop = threading.Event()
    reset_requested = threading.Event()

    threads = []
    t = threading.Thread(target=wifi_poller, args=(stop, ui), daemon=True)
    t.start(); threads.append(t)

    input_thread = None
    if controller_path:
        input_thread = threading.Thread(
            target=send_controls,
            args=(stop, pi_ip, controller_path, ui, minimap, cfg),
            daemon=False)
        input_thread.start()

    try:
        video_loop(stop, mjpeg_url, reset_requested, ui, minimap, cfg)
    finally:
        stop.set()
        minimap.stop()
        if input_thread is not None:
            input_thread.join(timeout=1.5)

    if reset_requested.is_set():
        return "reset_ip"
    if ui.get("back_to_launcher"):
        return "back"
    return "exit"


# ==================== MAIN ====================
if __name__ == "__main__":
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        force_manual_ip = False
        while True:
            controller_path = pick_controller_device()
            if controller_path:
                print(f"[Init] Controller device: {controller_path}")
                try:
                    controller_dev = InputDevice(controller_path)
                except Exception:
                    controller_dev = None
            else:
                print("[Init] WARNING: no controller device found. Video will still run.")
                controller_dev = None

            RASPI_IP = choose_pi_ip_with_ui(controller_dev, force_manual=force_manual_ip)
            force_manual_ip = False

            # -------- launcher loop (mode picking + sessions) --------
            presence_stop = None
            presence_mode = None
            leave_app = False

            while True:
                choice = launcher_screen(controller_dev, RASPI_IP)

                if choice is None:
                    leave_app = True
                    break
                if choice == "change_ip":
                    force_manual_ip = True
                    break

                cfg = load_config()
                mode_token = MODE_TOKEN[choice]

                # (re)start presence with the right mode token
                if presence_mode != mode_token:
                    if presence_stop is not None:
                        presence_stop.set()
                        sleep(0.2)
                    presence_stop = threading.Event()
                    threading.Thread(target=presence_socket,
                                     args=(presence_stop, RASPI_IP, mode_token),
                                     daemon=True).start()
                    presence_mode = mode_token

                # release the launcher's controller handle for the session
                try:
                    if controller_dev is not None:
                        controller_dev.close()
                except Exception:
                    pass
                controller_dev = None

                if choice == "map":
                    result = cdc_mapmode.run(WINDOW, SCREEN_W, SCREEN_H, RASPI_IP,
                                             controller_path, SLAM_PORT, TOASTS,
                                             TOASTS.draw, draw_battery)
                else:
                    result = run_drive_session(RASPI_IP, controller_path,
                                               minimap_on_start=(choice == "drive_map"),
                                               cfg=cfg)

                # re-open controller for the launcher
                if controller_path:
                    try:
                        controller_dev = InputDevice(controller_path)
                    except Exception:
                        controller_dev = None

                if result == "back":
                    continue
                if result == "reset_ip":
                    c = load_config()
                    ssid = get_active_ssid()
                    if ssid and ssid in c.get("wifi_to_ip", {}):
                        del c["wifi_to_ip"][ssid]
                    c["last_ip"] = None
                    save_config(c)
                    force_manual_ip = True
                    break
                leave_app = True
                break

            if presence_stop is not None:
                presence_stop.set()
            try:
                if controller_dev is not None:
                    controller_dev.close()
            except Exception:
                pass

            if leave_app:
                break
    finally:
        cv2.destroyAllWindows()
        for _ in range(5):
            cv2.waitKey(20)
