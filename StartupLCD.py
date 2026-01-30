#!/usr/bin/env python3
"""
lcd_supervisor.py (Raspberry Pi)

Default: shows WiFi + Raspi Connect status on LCD.
Watches for:
  - Flipper keyboard presence (via /dev/input and evdev)
  - Steam Deck presence socket (incoming TCP on PRESENCE_PORT)

If Flipper present (and no Steam Deck present): launches FLIPPER_MODE_CMD
If Steam Deck present: launches STEAMDECK_MODE_CMD
On disconnect (device disappears / socket closes): kills launched script and returns to LCD status.
"""

import os
import re
import time
import socket
import threading
import subprocess
import signal

# LCD
import sys
sys.path.insert(0, "/home/Corndog")
from lcd import lcd_library as lcd

# Optional evdev (needed for Flipper detect)
try:
    from evdev import InputDevice, list_devices
except Exception:
    InputDevice = None
    list_devices = None


# -------------------- CONFIG YOU MUST SET --------------------
# Steam Deck presence port (must match the Deck script)
PRESENCE_PORT = 65431

# What to launch when Steam Deck is present (Pi-side server/controller script)
STEAMDECK_MODE_CMD = ["/home/Corndog/Desktop/RobotOperationScripts/robo/bin/python", "/home/Corndog/Desktop/RobotOperationScripts/Corndog/SteamDeckCommunication.py"]  # <-- CHANGE THIS

# What to launch when Flipper is present (your flipper menu script)
FLIPPER_MODE_CMD = ["/home/Corndog/Desktop/RobotOperationScripts/robo/bin/python", "/home/Corndog/Desktop/RobotOperationScripts/Corndog/flipper_menu.py"]  # <-- CHANGE THIS

# Flipper name match (same idea as your evdev script)
FLIPPER_NAME_MATCHES = ("flipper", "keynote", "control")

# Polling / UI timing
POLL_S = 0.5
LCD_REFRESH_S = 2.0

# If both are present: Steam Deck wins (set False to prefer Flipper instead)
STEAMDECK_PRIORITY = True

# Raspi Connect check
CONNECT_USER = os.environ.get("CONNECT_USER")  # e.g. export CONNECT_USER=pi


# -------------------- WiFi + Connect status --------------------
def get_wifi_status():
    try:
        result = subprocess.run(['nmcli', 'radio', 'wifi'], stdout=subprocess.PIPE, text=True)
        wifi_state = result.stdout.strip()
        if wifi_state == "disabled":
            return "Wifi: Off"

        result = subprocess.run(['nmcli', 'device', 'status'], stdout=subprocess.PIPE, text=True)
        lines = result.stdout.strip().split("\n")

        for line in lines:
            if "wifi" in line:
                columns = line.split()
                state = columns[2] if len(columns) > 2 else ""
                if state == "connected":
                    ssid_result = subprocess.run(
                        ['nmcli', '-t', '-f', 'active,ssid', 'dev', 'wifi'],
                        stdout=subprocess.PIPE,
                        text=True
                    )
                    for ssid_line in ssid_result.stdout.strip().split("\n"):
                        if ssid_line.startswith("yes:"):
                            ssid = ssid_line.split(":", 1)[1]
                            return f"Wifi: Connected {ssid}"
                    return "Wifi: Connected"
                elif state == "disconnected":
                    return "Wifi: Searching"

        return "Wifi: Searching"
    except Exception as e:
        return f"WiFi Err: {e}"


def _run_rpi_connect_status():
    cmd = ["rpi-connect", "status"]
    if os.geteuid() == 0 and CONNECT_USER:
        cmd = ["sudo", "-u", CONNECT_USER, *cmd]
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return result.returncode, result.stdout


def get_connect_info():
    """
    Returns (online_bool, active_sessions_int)
    Online/ready check:
      Signed in: yes AND Subscribed to events: yes
    Active session check:
      Parses 'Screen sharing: ... (N sessions active)'
      and 'Remote shell: ... (N sessions active)'
    """
    try:
        rc, out = _run_rpi_connect_status()
        if rc != 0:
            return False, 0

        def yn(field_name: str) -> bool:
            m = re.search(rf"^{re.escape(field_name)}:\s*(yes|no)\s*$", out, flags=re.MULTILINE)
            return (m.group(1) == "yes") if m else False

        def sessions(kind: str) -> int:
            m = re.search(
                rf"^{re.escape(kind)}:.*\((\d+)\s+sessions?\s+active\)\s*$",
                out,
                flags=re.MULTILINE
            )
            return int(m.group(1)) if m else 0

        online = yn("Signed in") and yn("Subscribed to events")
        active = sessions("Screen sharing") + sessions("Remote shell")
        return online, active

    except FileNotFoundError:
        return False, 0
    except Exception:
        return False, 0


# -------------------- Flipper presence detect --------------------
def flipper_present() -> bool:
    if list_devices is None or InputDevice is None:
        return False
    try:
        for p in list_devices():
            try:
                dev = InputDevice(p)
                name = (dev.name or "").lower()
                dev.close()
                if any(s in name for s in FLIPPER_NAME_MATCHES):
                    return True
            except Exception:
                continue
    except Exception:
        return False
    return False


# -------------------- Steam Deck presence listener --------------------
class SteamDeckPresence:
    def __init__(self, port: int):
        self.port = port
        self._lock = threading.Lock()
        self._connected = False
        self._conn = None
        self._stop = threading.Event()

    def start(self):
        t = threading.Thread(target=self._run, daemon=True)
        t.start()

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

    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def _set_connected(self, v: bool, conn=None):
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
            # If already connected, poll the connection for liveness
            if self.is_connected():
                with self._lock:
                    conn = self._conn
                if conn is None:
                    self._set_connected(False, None)
                    continue
                try:
                    conn.settimeout(0.1)
                    data = conn.recv(1)
                    if not data:
                        # clean close
                        self._set_connected(False, None)
                        try:
                            conn.close()
                        except Exception:
                            pass
                    else:
                        # got heartbeat byte, still alive
                        pass
                except socket.timeout:
                    pass
                except Exception:
                    self._set_connected(False, None)
                    try:
                        conn.close()
                    except Exception:
                        pass
                continue

            # Not connected: accept new
            try:
                conn, addr = s.accept()
                try:
                    conn.settimeout(0.5)
                    _ = conn.recv(64)  # read "STEAMDECK_READY\n" or heartbeat
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


# -------------------- Subprocess management --------------------
def _popen(cmd):
    # put child in its own process group so we can kill the whole thing
    return subprocess.Popen(cmd, preexec_fn=os.setsid)

def _kill_proc(p: subprocess.Popen, grace_s: float = 1.0):
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


# -------------------- Supervisor loop --------------------
def main():
    lcd.lcd(get_wifi_status())
    last_lcd_t = 0.0
    last_lcd_msg = None

    presence = SteamDeckPresence(PRESENCE_PORT)
    presence.start()

    mode = "lcd"  # "lcd" | "steamdeck" | "flipper"
    proc = None

    try:
        while True:
            steamdeck = presence.is_connected()
            flipper = flipper_present()

            # Decide target mode
            target = "lcd"
            if steamdeck and flipper:
                if STEAMDECK_PRIORITY:
                    target = "steamdeck"
                else:
                    target = "flipper"
            elif steamdeck:
                target = "steamdeck"
            elif flipper:
                target = "flipper"

            # Mode switch?
            if target != mode:
                # stop current
                if proc is not None:
                    _kill_proc(proc)
                    proc = None

                mode = target

                if mode == "steamdeck":
                    lcd.lcd("Steam Deck      Mode")
                    proc = _popen(STEAMDECK_MODE_CMD)

                elif mode == "flipper":
                    lcd.lcd("Flipper Mode")
                    proc = _popen(FLIPPER_MODE_CMD)

                else:
                    # back to LCD status immediately
                    last_lcd_t = 0.0
                    last_lcd_msg = None

            # If running a mode script, and it died, fall back to LCD
            if proc is not None and proc.poll() is not None:
                proc = None
                mode = "lcd"
                last_lcd_t = 0.0
                last_lcd_msg = None

            # LCD status mode updates
            if mode == "lcd":
                now = time.time()
                if now - last_lcd_t >= LCD_REFRESH_S:
                    online, active_sessions = get_connect_info()
                    wifi_msg = get_wifi_status()
                    if online:
                        msg = "Raspi Connect   Online"
                        # If someone connects via Connect, you *could* clear LCD or show sessions
                        if active_sessions > 0:
                            msg = f"Connect Session {active_sessions}"
                    else:
                        # show WiFi
                        # If too long, LCD lib usually truncates; keep it simple
                        msg = wifi_msg

                    if msg != last_lcd_msg:
                        lcd.lcd(msg)
                        last_lcd_msg = msg

                    last_lcd_t = now

            time.sleep(POLL_S)

    finally:
        try:
            if proc is not None:
                _kill_proc(proc)
        except Exception:
            pass
        try:
            presence.stop()
        except Exception:
            pass
        try:
            lcd.clear()
        except Exception:
            pass


if __name__ == "__main__":
    main()
