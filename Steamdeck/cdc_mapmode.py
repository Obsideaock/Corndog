"""
cdc_minimap.py  —  LIDAR minimap for the Steam Deck client (display only).

Talks to the Corndog SLAM web server (web_server.py) running on the Pi at
http://<pi-ip>:8001 :

  GET /state   -> JSON {pose, scan[], trail[], map{ox,oy,res,n,rev}, robot[L,W], ...}
  GET /map.png -> n x n occupancy image, already flipped so +y is up.

It renders a small, robot-centered, north-up crop into a corner of the live
view. No interaction (no click-to-go) by design — this is a glance-able minimap,
the full interactive map still lives in a browser at :8001.

Threading model:
  - One daemon poller thread fetches /state (~5 Hz) and re-fetches /map.png only
    when map.rev changes (cheap; the Pi only bumps rev when the grid changes).
  - The video thread calls render(canvas, rect) every frame; it only reads the
    latest cached state, so it never blocks on the network.
  - start()/stop() let the d-pad toggle gate both the display AND the polling, so
    "minimap off" means the Deck stops hitting the Pi entirely.

NOTE: turning the minimap off here stops the Deck's polling/Display. It does NOT
stop the SLAM process on the Pi (that's a separate process). Reclaiming the Pi's
compute/battery needs a Pi-side hook — see the notes that ship with this build.
"""

import json
import threading
from io import BytesIO
from time import sleep, time
from urllib.request import urlopen
from urllib.error import URLError

import cv2
import numpy as np


# ----- palette (kept consistent with the main client) -----
_BG        = (13, 17, 23)
_PANEL     = (22, 27, 34)
_BORDER    = (44, 52, 62)
_FREE      = (40, 24, 20)      # free space (BGR of the SLAM's free colour-ish)
_WALL      = (210, 210, 210)
_UNKNOWN   = (46, 46, 46)
_TRAIL     = (70, 190, 255)    # amber-ish (BGR)
_SCAN      = (240, 230, 80)    # cyan (BGR)
_ROBOT     = (107, 107, 255)   # red (BGR)
_TEXT_HI   = (230, 237, 243)
_TEXT_MID  = (139, 148, 158)
_FONT      = cv2.FONT_HERSHEY_SIMPLEX


class Minimap:
    def __init__(self, host_ip: str, port: int = 8001,
                 crop_radius_m: float = 3.0, poll_hz: float = 5.0,
                 http_timeout: float = 0.6):
        self.base = f"http://{host_ip}:{port}"
        self.crop_radius_m = float(crop_radius_m)
        self.poll_period = 1.0 / max(0.5, poll_hz)
        self.http_timeout = http_timeout

        self._lock = threading.Lock()
        self._state = None            # latest /state dict
        self._map_img = None          # latest decoded /map.png as np.uint8 [n,n,3]
        self._map_rev = -1
        self._last_ok = 0.0           # monotonic-ish time of last good /state

        self._run = False
        self._thread = None

    # ---- lifecycle ----
    def start(self):
        if self._run:
            return
        self._run = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._run = False
        # leave cached frame so a quick re-toggle shows last image instantly

    @property
    def active(self) -> bool:
        return self._run

    # ---- network ----
    def _poll_loop(self):
        while self._run:
            t0 = time()
            try:
                with urlopen(self.base + "/state", timeout=self.http_timeout) as r:
                    st = json.loads(r.read().decode("utf-8", "ignore"))
                rev = int(st.get("map", {}).get("rev", -1))
                with self._lock:
                    self._state = st
                    self._last_ok = time()
                if rev != self._map_rev:
                    try:
                        with urlopen(self.base + f"/map.png?{rev}",
                                     timeout=self.http_timeout) as r:
                            raw = r.read()
                        arr = cv2.imdecode(np.frombuffer(raw, np.uint8),
                                           cv2.IMREAD_COLOR)
                        if arr is not None:
                            with self._lock:
                                self._map_img = arr
                                self._map_rev = rev
                    except (URLError, OSError, ValueError):
                        pass
            except (URLError, OSError, ValueError, json.JSONDecodeError):
                pass

            dt = time() - t0
            sleep(max(0.0, self.poll_period - dt))

    def _fresh(self) -> bool:
        with self._lock:
            return self._state is not None and (time() - self._last_ok) < 2.0

    # ---- rendering ----
    def render(self, canvas, rect):
        """Draw the minimap into rect=(x1,y1,x2,y2) on canvas (BGR)."""
        x1, y1, x2, y2 = rect
        w, h = x2 - x1, y2 - y1

        # panel + border
        sub = canvas[y1:y2, x1:x2]
        overlay = sub.copy()
        overlay[:] = _PANEL
        cv2.addWeighted(overlay, 0.82, sub, 0.18, 0, sub)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), _BORDER, 1)

        # title strip
        cv2.rectangle(canvas, (x1, y1), (x2, y1 + 22), (28, 34, 42), -1)
        cv2.putText(canvas, "LIDAR", (x1 + 8, y1 + 16), _FONT, 0.45, _TEXT_HI, 1, cv2.LINE_AA)

        with self._lock:
            st = self._state
            mp = self._map_img
            fresh = st is not None and (time() - self._last_ok) < 2.0

        if not fresh or st is None or mp is None:
            msg = "no data" if self._run else "off"
            (tw, _), _ = cv2.getTextSize(msg, _FONT, 0.5, 1)
            cv2.putText(canvas, msg, (x1 + (w - tw) // 2, y1 + h // 2),
                        _FONT, 0.5, _TEXT_MID, 1, cv2.LINE_AA)
            return

        meta = st["map"]
        ox, oy, res, n = meta["ox"], meta["oy"], meta["res"], meta["n"]
        px, py, th = st["pose"]["x"], st["pose"]["y"], st["pose"]["theta"]

        # robot position in *png pixel* space (png row 0 = top = max y)
        rix = (px - ox) / res
        riy = (n - 1) - ((py - oy) / res)
        cr = int(round(self.crop_radius_m / res))      # crop half-size in cells

        cx0 = int(round(rix - cr)); cx1 = int(round(rix + cr))
        cy0 = int(round(riy - cr)); cy1 = int(round(riy + cr))

        # build a (2cr x 2cr) crop, padding where the crop runs off the grid
        side = cy1 - cy0
        crop = np.full((side, side, 3), _UNKNOWN, np.uint8)
        sx0, sy0 = max(0, cx0), max(0, cy0)
        sx1, sy1 = min(n, cx1), min(n, cy1)
        if sx1 > sx0 and sy1 > sy0:
            crop[sy0 - cy0:sy1 - cy0, sx0 - cx0:sx1 - cx0] = mp[sy0:sy1, sx0:sx1]

        view_area = (x1 + 2, y1 + 24, x2 - 2, y2 - 2)
        vw = view_area[2] - view_area[0]
        vh = view_area[3] - view_area[1]
        disp = cv2.resize(crop, (vw, vh), interpolation=cv2.INTER_NEAREST)
        canvas[view_area[1]:view_area[3], view_area[0]:view_area[2]] = disp

        # helpers: world -> minimap-view pixel
        scale = vw / float(side)

        def w2v(wx, wy):
            cix = (wx - ox) / res - cx0
            ciy = ((n - 1) - ((wy - oy) / res)) - cy0
            return (int(view_area[0] + cix * scale),
                    int(view_area[1] + ciy * scale))

        # trail (amber)
        trail = st.get("trail", [])
        if len(trail) > 1:
            pts = np.array([w2v(p[0], p[1]) for p in trail], np.int32)
            cv2.polylines(canvas, [pts], False, _TRAIL, 1, cv2.LINE_AA)

        # live scan (cyan)
        for sxy in st.get("scan", []):
            vx, vy = w2v(sxy[0], sxy[1])
            if view_area[0] <= vx < view_area[2] and view_area[1] <= vy < view_area[3]:
                canvas[vy, vx] = _SCAN

        # robot oriented box + heading tick
        L, W = st.get("robot", [0.35, 0.24])
        hl, hw = L / 2.0, W / 2.0
        cs, sn = np.cos(th), np.sin(th)
        corners = []
        for bx, by in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):
            wx = px + bx * cs - by * sn
            wy = py + bx * sn + by * cs
            corners.append(w2v(wx, wy))
        cv2.polylines(canvas, [np.array(corners, np.int32)], True, _ROBOT, 1, cv2.LINE_AA)
        hx, hy = w2v(px + hl * 1.6 * cs, py + hl * 1.6 * sn)
        c0 = w2v(px, py)
        cv2.line(canvas, c0, (hx, hy), _ROBOT, 1, cv2.LINE_AA)
