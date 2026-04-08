#!/usr/bin/env python3
"""
CorndogCommunication.py (Steam Deck client)

This is only to run on the steamdeck, do not run this on the pi!!!

- Launches fullscreen MJPEG video from Raspberry Pi
- Forwards Steam Deck controller events to Pi over TCP
- Remembers Wi-Fi SSID -> Pi IP (saved to ~/.config/cdc/config.json)
- If SSID unknown, shows fullscreen IP entry screen
- OPTIONS/START button requests the Steam on-screen keyboard (steam://open/keyboard)
  NOTE: On some SteamOS builds, the keyboard request can succeed but not visually appear
  unless another overlay element is open. Steam+X always works as a fallback.

Stability changes vs my previous version:
- Uses your known-good DEVICE_PATH by default
- Only tries auto-detect if DEVICE_PATH fails
- OpenCV windows are created/destroyed ONLY on the main thread (reduces Qt mutex/segfault issues)

Added for Pi auto-detect / launcher:
- Opens a second TCP connection (presence socket) to the Pi on PRESENCE_PORT.
  The Pi can detect "Steam Deck is here" without ever needing the Steam Deck's IP.
"""

import cv2
import numpy as np
import threading
import socket
import select
from time import sleep, time
from urllib.request import urlopen
from urllib.error import URLError
import socket as pysocket

from evdev import InputDevice, ecodes, list_devices

import json
import re
import subprocess
from pathlib import Path


# -------------------- SETTINGS --------------------
CONTROL_PORT = 65432

# NEW: presence connection (for Pi to detect Deck without needing Deck IP)
PRESENCE_PORT = 65431

# Your known-good controller device. Keep this.
DEVICE_PATH_FALLBACK = "/dev/input/event9"

# Steam Deck is 1280x800; your original used 1200x800
SCREEN_W = 1200
SCREEN_H = 800

RETRY_SEC          = 1.0
MJPEG_IO_TIMEOUT   = 2.0
CONTROL_IO_TIMEOUT = 3.0

# Presence behavior
PRESENCE_CONNECT_TIMEOUT = 2.0
PRESENCE_HEARTBEAT_S     = 2.0

# Button codes from your working script
BTN_A     = 304
BTN_START = 313   # Options/Start (what you want to trigger keyboard)
BTN_GUIDE = 314   # Guide (your exit button)

# -------------------- CONFIG (Wi-Fi -> IP) --------------------
CONFIG_DIR  = Path.home() / ".config" / "cdc"
CONFIG_PATH = CONFIG_DIR / "config.json"
IP_RE = re.compile(r"^(?:\d{1,3}\.){3}\d{1,3}$")


def get_active_ssid() -> str | None:
    try:
        out = subprocess.check_output(
            ["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"],
            text=True
        )
        for line in out.splitlines():
            if line.startswith("yes:"):
                ssid = line.split("yes:", 1)[1].strip()
                return ssid or None
    except Exception:
        pass
    return None


def load_config() -> dict:
    try:
        return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception:
        return {"wifi_to_ip": {}, "last_ip": None}


def save_config(cfg: dict) -> None:
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2), encoding="utf-8")


def validate_ip(ip: str) -> bool:
    if not IP_RE.match(ip):
        return False
    parts = ip.split(".")
    try:
        return all(0 <= int(p) <= 255 for p in parts)
    except ValueError:
        return False


def open_steam_keyboard() -> bool:
    """
    Returns True if we managed to run a keyboard-open command without throwing.
    This does NOT guarantee the keyboard becomes visible (SteamOS overlay quirk).
    """
    candidates = [
        ["steam", "-ifrunning", "steam://open/keyboard"],
        ["steam", "steam://open/keyboard"],
        ["xdg-open", "steam://open/keyboard"],
    ]
    for cmd in candidates:
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1, check=False)
            return True
        except Exception:
            continue
    return False


def pick_controller_device() -> str | None:
    """
    Prefer your known-good DEVICE_PATH_FALLBACK.
    If it isn't openable, try to auto-detect by scanning /dev/input/event*.
    """
    # 1) Try the known-good path first
    try:
        dev = InputDevice(DEVICE_PATH_FALLBACK)
        caps = dev.capabilities().get(ecodes.EV_KEY, [])
        dev.close()
        return DEVICE_PATH_FALLBACK
    except Exception:
        pass

    # 2) Auto-detect fallback
    for path in list_devices():
        try:
            dev = InputDevice(path)
            caps = dev.capabilities()
            keys = caps.get(ecodes.EV_KEY, [])
            abss = caps.get(ecodes.EV_ABS, [])
            if (BTN_A in keys) and (BTN_GUIDE in keys) and (len(abss) > 0):
                dev.close()
                return path
            dev.close()
        except Exception:
            continue

    return None


# -------------------- MJPEG STREAM --------------------
class MjpegClient:
    def __init__(self, url: str, io_timeout: float = 2.0):
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


def _draw_center_text(canvas, text, y, scale=1.0, thick=2):
    (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    x = (canvas.shape[1] - tw) // 2
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), thick, cv2.LINE_AA)


# -------------------- IP SETUP UI (MAIN THREAD) --------------------
def choose_pi_ip_with_ui(screen_w: int, screen_h: int, controller_dev: InputDevice | None) -> str:
    ssid = get_active_ssid()
    cfg = load_config()

    # Known SSID -> immediate
    if ssid and ssid in cfg.get("wifi_to_ip", {}):
        return cfg["wifi_to_ip"][ssid]

    typed = (cfg.get("last_ip") or "")
    status = "New Wi-Fi detected" if ssid else "Wi-Fi not detected"
    last_kb_request = 0.0
    kb_requested_ok = False

    cv2.namedWindow("CDC Setup", cv2.WINDOW_NORMAL)
    cv2.setWindowProperty("CDC Setup", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    while True:
        canvas = np.zeros((screen_h, screen_w, 3), dtype=np.uint8)

        def put(y: int, text: str, scale: float = 0.9, thick: int = 2):
            cv2.putText(canvas, text, (60, y), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, (255, 255, 255), thick, cv2.LINE_AA)

        put(120, "Corndog Communication", 1.4, 3)
        put(190, f"{status}: {ssid or '(unknown)'}", 0.9, 2)
        put(260, "Enter Raspberry Pi IP address", 1.0, 2)

        cv2.rectangle(canvas, (60, 300), (screen_w - 60, 380), (40, 40, 40), -1)
        cv2.rectangle(canvas, (60, 300), (screen_w - 60, 380), (120, 120, 120), 2)
        cv2.putText(canvas, typed or " ", (80, 355),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2, cv2.LINE_AA)

        put(450, "Options/Start = request Steam keyboard", 0.85, 2)
        put(500, "Enter = save & connect   Backspace = delete   Guide/Esc = quit", 0.85, 2)

        if last_kb_request and (time() - last_kb_request) < 4.0:
            hint = "If keyboard doesn't show: use Steam+X, or open '...' menu then Keyboard."
            put(560, hint, 0.75, 2)
            put(600, f"Keyboard request command {'ran' if kb_requested_ok else 'failed to run'}.", 0.75, 2)

        if typed:
            ok = validate_ip(typed)
            cv2.putText(canvas, "OK" if ok else "Invalid IP", (60, 420),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 255, 0) if ok else (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow("CDC Setup", canvas)

        # Controller buttons
        if controller_dev is not None:
            try:
                r, _, _ = select.select([controller_dev.fd], [], [], 0)
                if r:
                    for ev in controller_dev.read():
                        if ev.type == ecodes.EV_KEY and ev.value == 1:
                            if ev.code == BTN_START:
                                last_kb_request = time()
                                kb_requested_ok = open_steam_keyboard()
                            elif ev.code == BTN_GUIDE:
                                raise SystemExit(0)
            except OSError:
                controller_dev = None

        # Keyboard input (Steam+X typing)
        key = cv2.waitKey(16) & 0xFF
        if key == 27:  # ESC
            raise SystemExit(0)

        if key in (10, 13):  # Enter
            if validate_ip(typed):
                cfg = load_config()
                cfg["last_ip"] = typed
                if ssid:
                    cfg.setdefault("wifi_to_ip", {})[ssid] = typed
                save_config(cfg)

                cv2.destroyWindow("CDC Setup")
                cv2.waitKey(1)
                sleep(0.05)
                return typed

        if key in (8, 127):  # Backspace/delete
            typed = typed[:-1]
            continue

        ch = chr(key) if 0 <= key <= 255 else ""
        if ch.isdigit() or ch == ".":
            typed += ch


# -------------------- PRESENCE (WORKER THREAD) --------------------
def presence_socket(stop_event: threading.Event, raspi_ip: str):
    """
    Keep an open socket to the Pi so the Pi can detect the Steam Deck
    without knowing the Deck's IP. If the Pi restarts, we'll reconnect.
    """
    while not stop_event.is_set():
        s = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(PRESENCE_CONNECT_TIMEOUT)
            s.connect((raspi_ip, PRESENCE_PORT))
            s.settimeout(None)

            try:
                s.sendall(b"STEAMDECK_READY\n")
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


# -------------------- CONTROLS (WORKER THREAD) --------------------
def send_controls(stop_event: threading.Event, a_pressed_event: threading.Event, raspi_ip: str, device_path: str):
    button_names = {
        304: "A", 305: "B", 307: "X", 308: "Y",
        310: "LB", 311: "RB", 312: "Back", 313: "Start",
        314: "Guide", 315: "LS_Press", 316: "RS_Press"
    }
    axis_names = {
        0: "Left Stick X", 1: "Left Stick Y", 2: "Left Trigger",
        3: "Right Stick X", 4: "Right Stick Y", 5: "Right Trigger"
    }
    last_vals = {}

    try:
        dev = InputDevice(device_path)
        print(f"[Input] Using device: {dev.name} ({device_path})")
    except Exception as e:
        print(f"[Input] Failed to open controller device {device_path}: {e}")
        return

    sock_obj = None
    connected_once = False

    def connect_socket():
        nonlocal sock_obj, connected_once
        try:
            if sock_obj is not None:
                sock_obj.close()
        except Exception:
            pass

        sock_obj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_obj.settimeout(CONTROL_IO_TIMEOUT)

        while not stop_event.is_set():
            try:
                sock_obj.connect((raspi_ip, CONTROL_PORT))
                sock_obj.settimeout(None)
                print("[Input] Connected to Pi for controls")
                connected_once = True
                return True
            except (ConnectionRefusedError, TimeoutError, OSError):
                if not connected_once:
                    print("[Input] waiting... (control server not available yet)")
                    sleep(RETRY_SEC)
                    continue
                print("[Input] Control connection lost. Exiting.")
                stop_event.set()
                return False
        return False

    if not connect_socket():
        return

    try:
        while not stop_event.is_set():
            r, _, _ = select.select([dev.fd], [], [], 0.1)
            if not r:
                continue

            for ev in dev.read():
                if stop_event.is_set():
                    break

                try:
                    # buttons
                    if ev.type == ecodes.EV_KEY and ev.code in button_names:
                        name = button_names[ev.code]
                        state = 'pressed' if ev.value else 'released'
                        msg = f"{name} {state}"
                        sock_obj.sendall(msg.encode())

                        if ev.code == BTN_A and ev.value == 1:
                            a_pressed_event.set()

                        if ev.code == BTN_GUIDE and ev.value == 1:
                            stop_event.set()
                            break

                    # axes
                    elif ev.type == ecodes.EV_ABS and ev.code in axis_names:
                        raw = ev.value
                        axis = ev.code

                        if axis in (0, 1, 3, 4):
                            val = max(min(raw, 30000), -30000)
                            norm = round(val / 30000.0, 1)
                        else:
                            norm = round(raw / 255.0, 1)

                        if abs(norm) < 0.1:
                            norm = 0.0

                        prev = last_vals.get(axis)
                        if prev is None or abs(norm - prev) >= 0.1:
                            msg = f"{axis_names[axis]} {norm}"
                            sock_obj.sendall(msg.encode())
                            last_vals[axis] = norm

                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError) as e:
                    print(f"[Input] Connection lost ({e}). Exiting.")
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


# -------------------- VIDEO LOOP (MAIN THREAD) --------------------
def video_loop(stop_event: threading.Event, a_pressed_event: threading.Event, mjpeg_url: str):
    mj = MjpegClient(mjpeg_url, io_timeout=MJPEG_IO_TIMEOUT)
    connected_once = False

    cv2.namedWindow("LiveView", cv2.WINDOW_NORMAL)
    cv2.setWindowProperty("LiveView", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    while not stop_event.is_set():
        if mj.resp is None:
            canvas = np.zeros((SCREEN_H, SCREEN_W, 3), dtype=np.uint8)
            _draw_center_text(canvas, "Connecting to camera stream...", SCREEN_H // 2 - 10, 1.1, 3)
            _draw_center_text(canvas, mjpeg_url, SCREEN_H // 2 + 35, 0.7, 2)
            cv2.imshow("LiveView", canvas)
            cv2.waitKey(1)

            try:
                mj.open()
                print("[Video] Connected to MJPEG stream")
            except (URLError, ConnectionError, OSError, TimeoutError):
                sleep(RETRY_SEC)
                continue

        try:
            frame = mj.read_frame()
            connected_once = True
        except Exception as e:
            mj.close()
            if not connected_once:
                sleep(RETRY_SEC)
                continue
            print(f"[Video] Stream lost ({e}). Exiting.")
            stop_event.set()
            break

        # Preserve aspect ratio: scale to screen height, then center/crop to width
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

        if not a_pressed_event.is_set():
            bar_h = 50
            cv2.rectangle(canvas, (0, SCREEN_H - bar_h), (SCREEN_W, SCREEN_H), (0, 0, 0), -1)
            text = "Press A to enable (stand up)"
            (txt_w, txt_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
            txt_x = (SCREEN_W - txt_w) // 2
            txt_y = SCREEN_H - (bar_h - txt_h) // 2
            cv2.putText(canvas, text, (txt_x, txt_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("LiveView", canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            stop_event.set()
            break

    mj.close()
    cv2.destroyAllWindows()
    cv2.waitKey(1)


# -------------------- MAIN --------------------
if __name__ == "__main__":
    controller_path = pick_controller_device()
    if controller_path:
        print(f"[Init] Controller device: {controller_path}")
        controller_dev_for_setup = None
        try:
            controller_dev_for_setup = InputDevice(controller_path)
        except Exception:
            controller_dev_for_setup = None
    else:
        print("[Init] WARNING: Could not find a controller device. Video will still run; use Steam+X for keyboard.")
        controller_dev_for_setup = None

    # Get IP (saved per Wi-Fi)
    RASPI_IP = choose_pi_ip_with_ui(SCREEN_W, SCREEN_H, controller_dev_for_setup)

    # Close setup controller handle (setup is done)
    try:
        if controller_dev_for_setup is not None:
            controller_dev_for_setup.close()
    except Exception:
        pass

    MJPEG_URL = f"http://{RASPI_IP}:8000/stream.mjpg"

    stop = threading.Event()
    a_pressed = threading.Event()

    # NEW: start presence thread so Pi can detect us
    presence_thread = threading.Thread(
        target=presence_socket,
        args=(stop, RASPI_IP),
        daemon=True
    )
    presence_thread.start()

    input_thread = None
    if controller_path:
        input_thread = threading.Thread(
            target=send_controls,
            args=(stop, a_pressed, RASPI_IP, controller_path),
            daemon=False
        )
        input_thread.start()

    print("Running. Press Guide or ESC to exit.")
    try:
        # Video loop runs on MAIN THREAD (more stable)
        video_loop(stop, a_pressed, MJPEG_URL)
    finally:
        stop.set()
        if input_thread is not None:
            input_thread.join(timeout=1.0)
