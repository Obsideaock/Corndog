"""
slam_map_server.py — Serves the live SLAM map as an MJPEG stream, mirroring the
camera-stream pattern in SteamDeckCommunication.py. Open http://<pi-ip>:8001/
in any browser on your laptop or Steam Deck.

  /            -> tiny HTML page embedding the stream
  /map.mjpg    -> multipart MJPEG of the rendered map
"""

from __future__ import annotations
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import render

MAP_PORT = 8001
_core = None
_fps = 8

_PAGE = b"""<!doctype html><html><head><title>Corndog SLAM</title>
<style>body{margin:0;background:#11131a;display:flex;justify-content:center;
align-items:center;height:100vh}img{max-width:98vw;max-height:98vh;
image-rendering:pixelated;border-radius:8px}</style></head>
<body><img src="/map.mjpg"></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(_PAGE)
            return
        if self.path != "/map.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                if _core is not None and _core._field is not None:
                    im = render.render(_core, upscale=3)
                    jpg = render.to_jpeg(im, quality=70)
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                time.sleep(1.0 / _fps)
        except (BrokenPipeError, ConnectionResetError):
            pass


def serve(core, port=MAP_PORT, fps=8):
    global _core, _fps
    _core, _fps = core, fps
    srv = ThreadingHTTPServer(("0.0.0.0", port), _Handler)
    print(f"[Map] http://0.0.0.0:{port}/  (MJPEG)")
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv
