#!/usr/bin/env python3
"""
lcd_supervisor.py (Raspberry Pi)

Manages two robot controller scripts:
  - Flipper2.py               : runs by default at all times, auto-restarts on crash
  - SteamDeckCommunication.py : launched when Steam Deck presence detected;
                                Flipper2 is killed while Steam Deck is present
                                and restarted when it leaves.

Steam Deck always takes priority — only one script owns GPIO at a time.
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

STEAMDECK_MODE_CMD = [
    "/home/Corndog/Desktop/RobotOperationScripts/robo/bin/python",
    "/home/Corndog/Desktop/RobotOperationScripts/Corndog/SteamDeckCommunication.py",
]

FLIPPER2_CMD = [
    "/home/Corndog/Desktop/RobotOperationScripts/robo/bin/python",
    "/home/Corndog/Desktop/RobotOperationScripts/Corndog/Flipper2.py",
]

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


# -------------------- Steam Deck presence listener --------------------
class SteamDeckPresence:
    def __init__(self, port):
        self.port = port
        self._lock = threading.Lock()
        self._connected = False
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

    def _set_connected(self, v, conn=None):
        with self._lock:
            self._connected = v
            self._conn = conn

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
                    data = conn.recv(1)
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
                try:
                    conn.settimeout(0.5)
                    conn.recv(64)
                except Exception:
                    pass
                self._set_connected(True, conn)
            except socket.timeout:
                pass
            except Exception:
                pass

        try:
            s.close()
        except Exception:
            pass


# -------------------- Process helpers --------------------
def _popen(cmd):
    """Launch cmd in its own process group, capturing stdout+stderr."""
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    return subprocess.Popen(
        cmd,
        preexec_fn=os.setsid,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=env,
    )


def _kill_proc(p, grace_s=1.0):
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
    restart_fn receives one argument: was_intentional (bool).
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
    _steamdeck_proc = None
    _steamdeck_present = False

    # Suppress flags: set True before an intentional kill so the watcher
    # thread doesn't schedule a restart for that death.
    _flipper_suppress   = [False]   # wrapped in list so closure can mutate
    _steamdeck_suppress = [False]

    def start_flipper():
        nonlocal _flipper_proc
        _flipper_suppress[0] = False
        p = _popen(FLIPPER2_CMD)
        _flipper_proc = p
        print("[Supervisor] Flipper2.py started")

        def on_exit():
            nonlocal _flipper_proc
            _flipper_proc = None
            if _flipper_suppress[0]:
                print("[Supervisor] Flipper2.py stopped (intentional)")
                return
            print(f"[Supervisor] Flipper2.py crashed — restarting in {RESTART_DELAY_S}s")
            time.sleep(RESTART_DELAY_S)
            # Only restart if Steam Deck isn't currently using the Pi
            if not _steamdeck_present:
                start_flipper()

        threading.Thread(target=_watch_for_crash, args=("flipper", p, on_exit), daemon=True).start()

    def kill_flipper():
        nonlocal _flipper_proc
        _flipper_suppress[0] = True
        _kill_proc(_flipper_proc)
        _flipper_proc = None
        print("[Supervisor] Flipper2.py killed (Steam Deck priority)")

    def start_steamdeck():
        nonlocal _steamdeck_proc
        _steamdeck_suppress[0] = False
        p = _popen(STEAMDECK_MODE_CMD)
        _steamdeck_proc = p
        print("[Supervisor] SteamDeckCommunication.py started")

        def on_exit():
            nonlocal _steamdeck_proc
            _steamdeck_proc = None
            if _steamdeck_suppress[0]:
                print("[Supervisor] SteamDeckCommunication.py stopped (intentional)")
                return
            print(f"[Supervisor] SteamDeckCommunication.py crashed — restarting in {RESTART_DELAY_S}s")
            time.sleep(RESTART_DELAY_S)
            if _steamdeck_present:
                start_steamdeck()

        threading.Thread(target=_watch_for_crash, args=("steamdeck", p, on_exit), daemon=True).start()

    def kill_steamdeck():
        nonlocal _steamdeck_proc
        _steamdeck_suppress[0] = True
        _kill_proc(_steamdeck_proc)
        _steamdeck_proc = None
        print("[Supervisor] SteamDeckCommunication.py killed")

    try:
        lcd.lcd(get_wifi_status())
    except Exception:
        pass

    last_lcd_t   = 0.0
    last_lcd_msg = None

    presence = SteamDeckPresence(PRESENCE_PORT)
    presence.start()

    # Flipper2 runs from boot by default.
    start_flipper()

    try:
        while True:
            deck_now = presence.is_connected()

            # ── Steam Deck just appeared ──────────────────────────
            if deck_now and not _steamdeck_present:
                _steamdeck_present = True
                print("[Supervisor] Steam Deck detected — killing Flipper2, launching SteamDeck")
                kill_flipper()
                start_steamdeck()
                try:
                    lcd.lcd("Steam Deck      Active")
                except Exception:
                    pass

            # ── Steam Deck just disappeared ───────────────────────
            elif not deck_now and _steamdeck_present:
                _steamdeck_present = False
                print("[Supervisor] Steam Deck gone — killing SteamDeck, restoring Flipper2")
                kill_steamdeck()
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
        kill_steamdeck()
        presence.stop()
        try:
            lcd.clear()
        except Exception:
            pass


if __name__ == "__main__":
    main()
