#!/usr/bin/env python3
"""
lcd_supervisor.py (Raspberry Pi) — v2

Manages the robot controller scripts:
  - Flipper2.py               : runs by default at all times, auto-restarts on crash
  - SteamDeckCommunication.py : "drive" mode — camera + joystick control only
  - Slam/run_robot.py         : "slam" mode — SLAM stack + camera + joystick
                                control in ONE process (map at :8001)

Steam Deck always takes priority — only one script owns GPIO at a time.

v2 changes:
  * The Deck's presence hello now carries a mode:
        "STEAMDECK_READY MODE=drive\n"   -> SteamDeckCommunication.py
        "STEAMDECK_READY MODE=slam\n"    -> Slam/run_robot.py
    (a bare "STEAMDECK_READY" still works and means drive — old clients fine.)
    "Drive + Minimap" and "Map Mode" on the Deck both request slam, since the
    minimap and the map UI are fed by the SLAM web server on :8001.
  * If the Deck reconnects asking for a DIFFERENT mode, the running script is
    swapped without waiting for the Deck to fully disappear.
  * UDP discovery beacon: broadcasts "CORNDOG <hostname>" on UDP :65430 every
    2 s while idle or serving, so the Deck can auto-discover the Pi's IP on any
    shared network (no more typing IPs).
"""

import os
import re
import time
import socket
import threading
import subprocess
import signal

import sys
sys.path.insert(0, "/home/Corndog")
from lcd import lcd_library as lcd


# -------------------- CONFIG --------------------
PRESENCE_PORT = 65431
BEACON_PORT   = 65430
BEACON_PERIOD = 2.0

_PY = "/home/Corndog/Desktop/RobotOperationScripts/robo/bin/python"
_BASE = "/home/Corndog/Desktop/RobotOperationScripts/Corndog"

STEAMDECK_MODE_CMD = [_PY, f"{_BASE}/SteamDeckCommunication.py"]

# SLAM entry point (runs SLAM + control listener + camera in one process).
# run_robot.py expects to be launched from inside the Slam folder.
SLAM_MODE_CMD = [_PY, f"{_BASE}/Slam/run_robot.py"]
SLAM_MODE_CWD = f"{_BASE}/Slam"

FLIPPER2_CMD = [_PY, f"{_BASE}/Flipper2.py"]

POLL_S          = 0.5
LCD_REFRESH_S   = 2.0
RESTART_DELAY_S = 3.0


# -------------------- WiFi + Connect status --------------------
def get_wifi_status():
    try:
        result = subprocess.run(['nmcli', 'radio', 'wifi'], stdout=subprocess.PIPE, text=True)
        if result.stdout.strip() == "disabled":
            return "Wifi: Off"
        result = subprocess.run(['nmcli', 'device', 'status'], stdout=subprocess.PIPE, text=True)
        for line in result.stdout.strip().split("\n"):
            if "wifi" in line:
                columns = line.split()
                state = columns[2] if len(columns) > 2 else ""
                if state == "connected":
                    ssid_result = subprocess.run(
                        ['nmcli', '-t', '-f', 'active,ssid', 'dev', 'wifi'],
                        stdout=subprocess.PIPE, text=True,
                    )
                    for ssid_line in ssid_result.stdout.strip().split("\n"):
                        if ssid_line.startswith("yes:"):
                            return f"Wifi: {ssid_line.split(':', 1)[1]}"
                    return "Wifi: Connected"
                elif state == "disconnected":
                    return "Wifi: Searching"
        return "Wifi: Searching"
    except Exception as e:
        return f"WiFi Err: {e}"


CONNECT_USER = os.environ.get("CONNECT_USER")


def _run_rpi_connect_status():
    cmd = ["rpi-connect", "status"]
    if os.geteuid() == 0 and CONNECT_USER:
        cmd = ["sudo", "-u", CONNECT_USER, *cmd]
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return result.returncode, result.stdout


def get_connect_info():
    try:
        rc, out = _run_rpi_connect_status()
        if rc != 0:
            return False, 0

        def yn(field):
            m = re.search(rf"^{re.escape(field)}:\s*(yes|no)\s*$", out, flags=re.MULTILINE)
            return bool(m and m.group(1) == "yes")

        def sessions(kind):
            m = re.search(
                rf"^{re.escape(kind)}:.*\((\d+)\s+sessions?\s+active\)\s*$",
                out, flags=re.MULTILINE,
            )
            return int(m.group(1)) if m else 0

        online = yn("Signed in") and yn("Subscribed to events")
        active = sessions("Screen sharing") + sessions("Remote shell")
        return online, active
    except FileNotFoundError:
        return False, 0
    except Exception:
        return False, 0


# -------------------- UDP discovery beacon --------------------
def run_beacon(stop_event):
    """Broadcast a small hello so the Deck can find our IP without typing it.
    The Deck learns the IP from the packet's source address."""
    host = socket.gethostname()
    payload = f"CORNDOG {host}".encode()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    while not stop_event.is_set():
        try:
            s.sendto(payload, ("255.255.255.255", BEACON_PORT))
        except Exception:
            pass
        stop_event.wait(BEACON_PERIOD)
    try:
        s.close()
    except Exception:
        pass


# -------------------- Steam Deck presence listener (mode-aware) --------------------
class SteamDeckPresence:
    def __init__(self, port):
        self.port = port
        self._lock = threading.Lock()
        self._connected = False
        self._mode = "drive"
        self._conn = None
        self._stop = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._stop.set()
        with self._lock:
            try:
                if self._conn:
                    self._conn.close()
            except Exception:
                pass
            self._conn = None
            self._connected = False

    def is_connected(self):
        with self._lock:
            return self._connected

    def mode(self):
        with self._lock:
            return self._mode

    def _set_connected(self, v, conn=None, mode=None):
        with self._lock:
            self._connected = v
            self._conn = conn
            if mode is not None:
                self._mode = mode

    @staticmethod
    def _parse_mode(data: bytes) -> str:
        try:
            text = data.decode("utf-8", errors="ignore")
        except Exception:
            return "drive"
        m = re.search(r"MODE=(\w+)", text)
        if m and m.group(1).lower() in ("drive", "slam"):
            return m.group(1).lower()
        return "drive"

    def _run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", self.port))
        s.listen(1)
        s.settimeout(0.5)

        while not self._stop.is_set():
            if self.is_connected():
                with self._lock:
                    conn = self._conn
                if conn is None:
                    self._set_connected(False)
                    continue
                try:
                    conn.settimeout(0.1)
                    data = conn.recv(64)
                    if not data:
                        self._set_connected(False)
                        try:
                            conn.close()
                        except Exception:
                            pass
                except socket.timeout:
                    pass
                except Exception:
                    self._set_connected(False)
                    try:
                        conn.close()
                    except Exception:
                        pass
                continue

            try:
                conn, addr = s.accept()
                mode = "drive"
                try:
                    conn.settimeout(0.8)
                    hello = conn.recv(128)
                    mode = self._parse_mode(hello)
                except Exception:
                    pass
                print(f"[Supervisor] Steam Deck hello (mode={mode}) from {addr}")
                self._set_connected(True, conn, mode)
            except socket.timeout:
                pass
            except Exception:
                pass

        try:
            s.close()
        except Exception:
            pass


# -------------------- Process helpers --------------------
def _popen(cmd, cwd=None):
    """Launch cmd in its own process group, capturing stdout+stderr."""
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    return subprocess.Popen(
        cmd,
        cwd=cwd,
        preexec_fn=os.setsid,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=env,
    )


def _kill_proc(p, grace_s=2.0):
    if p is None:
        return
    try:
        if p.poll() is None:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            t0 = time.time()
            while time.time() - t0 < grace_s:
                if p.poll() is not None:
                    return
                time.sleep(0.05)
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except Exception:
        pass


# -------------------- Crash-restart watcher --------------------
def _watch_for_crash(name, proc, restart_fn):
    """
    Waits for a process to exit. If it wasn't killed intentionally,
    waits RESTART_DELAY_S then calls restart_fn() to bring it back.
    """
    # Drain stdout so the pipe doesn't fill and block the child.
    try:
        for raw in iter(proc.stdout.readline, b''):
            line = raw.decode("utf-8", errors="ignore").strip()
            if line:
                print(f"[{name}] {line}")
    except Exception:
        pass

    restart_fn()


# -------------------- Main --------------------
def main():
    _flipper_proc   = None
    _deck_proc      = None       # whichever deck-facing script is running
    _deck_mode      = None       # "drive" | "slam" | None
    _steamdeck_present = False

    def start_flipper():
        nonlocal _flipper_proc
        p = _popen(FLIPPER2_CMD)
        p._suppress = False          # per-process flag: no cross-restart races
        _flipper_proc = p
        print("[Supervisor] Flipper2.py started")

        def on_exit():
            nonlocal _flipper_proc
            if _flipper_proc is p:
                _flipper_proc = None
            if p._suppress:
                print("[Supervisor] Flipper2.py stopped (intentional)")
                return
            print(f"[Supervisor] Flipper2.py crashed — restarting in {RESTART_DELAY_S}s")
            time.sleep(RESTART_DELAY_S)
            if not _steamdeck_present and _flipper_proc is None:
                start_flipper()

        threading.Thread(target=_watch_for_crash, args=("flipper", p, on_exit), daemon=True).start()

    def kill_flipper():
        nonlocal _flipper_proc
        if _flipper_proc is not None:
            _flipper_proc._suppress = True
        _kill_proc(_flipper_proc)
        _flipper_proc = None
        print("[Supervisor] Flipper2.py killed (Steam Deck priority)")

    def start_deck_script(mode):
        nonlocal _deck_proc, _deck_mode
        if mode == "slam":
            p = _popen(SLAM_MODE_CMD, cwd=SLAM_MODE_CWD)
            print("[Supervisor] Slam/run_robot.py started (slam mode)")
        else:
            p = _popen(STEAMDECK_MODE_CMD)
            print("[Supervisor] SteamDeckCommunication.py started (drive mode)")
        p._suppress = False
        _deck_proc = p
        _deck_mode = mode

        def on_exit():
            nonlocal _deck_proc
            if _deck_proc is p:
                _deck_proc = None
            if p._suppress:
                print("[Supervisor] deck script stopped (intentional)")
                return
            print(f"[Supervisor] deck script crashed — restarting in {RESTART_DELAY_S}s")
            time.sleep(RESTART_DELAY_S)
            if _steamdeck_present and _deck_proc is None:
                start_deck_script(mode)

        threading.Thread(target=_watch_for_crash, args=(f"deck:{mode}", p, on_exit), daemon=True).start()

    def kill_deck_script():
        nonlocal _deck_proc, _deck_mode
        if _deck_proc is not None:
            _deck_proc._suppress = True
        _kill_proc(_deck_proc, grace_s=3.0 if _deck_mode == "slam" else 1.5)
        _deck_proc = None
        _deck_mode = None
        print("[Supervisor] deck script killed")

    try:
        lcd.lcd(get_wifi_status())
    except Exception:
        pass

    last_lcd_t   = 0.0
    last_lcd_msg = None

    beacon_stop = threading.Event()
    threading.Thread(target=run_beacon, args=(beacon_stop,), daemon=True).start()

    presence = SteamDeckPresence(PRESENCE_PORT)
    presence.start()

    # Flipper2 runs from boot by default.
    start_flipper()

    try:
        while True:
            deck_now = presence.is_connected()
            want_mode = presence.mode()

            # ── Steam Deck just appeared ──────────────────────────
            if deck_now and not _steamdeck_present:
                _steamdeck_present = True
                print(f"[Supervisor] Steam Deck detected (mode={want_mode}) — "
                      f"killing Flipper2, launching deck script")
                kill_flipper()
                start_deck_script(want_mode)
                try:
                    lcd.lcd("Steam Deck      " + ("SLAM" if want_mode == "slam" else "Active"))
                except Exception:
                    pass

            # ── Deck present but asked for a different mode ───────
            elif deck_now and _steamdeck_present and _deck_mode is not None \
                    and want_mode != _deck_mode:
                print(f"[Supervisor] Steam Deck mode switch {_deck_mode} -> {want_mode}")
                kill_deck_script()
                start_deck_script(want_mode)
                try:
                    lcd.lcd("Steam Deck      " + ("SLAM" if want_mode == "slam" else "Active"))
                except Exception:
                    pass

            # ── Steam Deck just disappeared ───────────────────────
            elif not deck_now and _steamdeck_present:
                _steamdeck_present = False
                print("[Supervisor] Steam Deck gone — killing deck script, restoring Flipper2")
                kill_deck_script()
                start_flipper()
                try:
                    lcd.lcd("Flipper Mode")
                except Exception:
                    pass

            # ── LCD idle status ───────────────────────────────────
            now = time.time()
            if now - last_lcd_t >= LCD_REFRESH_S:
                if not deck_now:
                    online, sessions = get_connect_info()
                    if online:
                        msg = (f"Connect: {sessions} sess" if sessions else "Raspi Connect   Online")
                    else:
                        msg = get_wifi_status()
                    if msg != last_lcd_msg:
                        try:
                            lcd.lcd(msg)
                        except Exception:
                            pass
                        last_lcd_msg = msg
                last_lcd_t = now

            time.sleep(POLL_S)

    finally:
        kill_flipper()
        kill_deck_script()
        presence.stop()
        beacon_stop.set()
        try:
            lcd.clear()
        except Exception:
            pass


if __name__ == "__main__":
    main()
