"""
web_server.py — Canvas/JSON web UI for Corndog SLAM + navigation (v2).

Replaces the old MJPEG view. The browser:
  * GET /map.png   lossless occupancy image (crisp, refetched only when it changes)
  * GET /state     JSON: pose, scan, trail, path, goal, nav state, waypoints, meta
  * POST /goal     {x,y}  -> plan + drive there  (rejected goals return a reason)
  * POST /stop     cancel navigation, halt Corndog
  * POST /waypoint {name,x,y} ; POST /goto {name} ; POST /delwaypoint {name}

Lighter on the Pi than MJPEG (no continuous JPEG re-encode) and renders the map
crisply on a canvas, so no upscaling artifacts. The robot is drawn as a
35 x 24 cm oriented box so you can judge clearance.
"""

from __future__ import annotations
import json
import time
import threading
from io import BytesIO
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image

ROBOT_L = 0.35
ROBOT_W = 0.24

_core = None
_nav = None
_wp = None
_png_lock = threading.Lock()
_png_cache = (0.0, None)        # (timestamp, bytes)


def _png_loop():
    """Render the map in ONE background thread (~3x/sec), regardless of how many
    viewers are connected. Request handlers just hand back the latest bytes, so
    serving the map never blocks on a render and never steals a render per client."""
    global _png_cache
    while True:
        try:
            data = _map_png(_core.grid)
            with _png_lock:
                _png_cache = (time.time(), data)
        except Exception:
            pass
        time.sleep(0.35)


def _cached_png(grid):
    global _png_cache
    with _png_lock:
        data = _png_cache[1]
    if data is not None:
        return data
    data = _map_png(grid)            # first request before the loop has run
    with _png_lock:
        _png_cache = (time.time(), data)
    return data


# 3-colour map: free (dark blue), unknown (grey), wall (white). With only three
# distinct colours the RGB PNG compresses to ~2 KB and encodes fast.
def _map_png(grid):
    p = grid.prob()
    img = np.full((grid.n, grid.n, 3), 46, np.uint8)   # unknown grey
    img[p < 0.35] = (20, 24, 40)                        # free
    img[p > 0.65] = (235, 235, 235)                     # wall
    img = img[::-1]                                     # flip so +y is up
    buf = BytesIO()
    Image.fromarray(img, "RGB").save(buf, "PNG")
    return buf.getvalue()


def _state():
    g = _core.grid
    with _core.lock:
        pose = _core.pose
        sx, sy = _core.last_scan_world
        trail = [(x, y) for _, x, y in _core.trail]
    scan = list(zip([float(v) for v in sx], [float(v) for v in sy])) if len(sx) else []
    if len(scan) > 220:                          # cap for bandwidth over network
        step = (len(scan) // 220) + 1
        scan = scan[::step]
    nav_snap = _nav.snapshot() if _nav else dict(state="idle", goal=None, path=[], reason="")
    return {
        "pose": {"x": float(pose[0]), "y": float(pose[1]), "theta": float(pose[2])},
        "scan": scan,
        "trail": trail[-160:],
        "path": [list(p) for p in nav_snap["path"]],
        "goal": list(nav_snap["goal"]) if nav_snap["goal"] else None,
        "nav_state": nav_snap["state"],
        "reason": nav_snap["reason"],
        "waypoints": _wp.all() if _wp else [],
        "robot": [ROBOT_L, ROBOT_W],
        "map": {"ox": g.ox, "oy": g.oy, "res": g.res, "n": g.n, "rev": int(_core._frame)},
    }


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, "text/html", _PAGE.encode())
        elif self.path.startswith("/map.png"):
            self._send(200, "image/png", _cached_png(_core.grid))
        elif self.path.startswith("/state"):
            self._send(200, "application/json", json.dumps(_state(), default=float).encode())
        else:
            self._send(404, "text/plain", b"not found")

    def _body(self):
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(n) or b"{}")

    def do_POST(self):
        try:
            b = self._body()
        except Exception:
            b = {}
        if self.path == "/goal":
            ok, reason = (_nav.set_goal(float(b["x"]), float(b["y"]))
                          if _nav else (False, "nav off"))
            self._send(200, "application/json",
                       json.dumps({"ok": ok, "reason": reason}).encode())
        elif self.path == "/stop":
            if _nav:
                _nav.cancel()
            self._send(200, "application/json", b'{"ok":true}')
        elif self.path == "/waypoint":
            if _wp:
                _wp.add(str(b["name"]), float(b["x"]), float(b["y"]))
            self._send(200, "application/json", b'{"ok":true}')
        elif self.path == "/goto":
            wp = _wp.get(str(b["name"])) if _wp else None
            if wp and _nav:
                ok, reason = _nav.set_goal(wp[0], wp[1])
            else:
                ok, reason = False, "unknown waypoint"
            self._send(200, "application/json",
                       json.dumps({"ok": ok, "reason": reason}).encode())
        elif self.path == "/delwaypoint":
            if _wp:
                _wp.delete(str(b["name"]))
            self._send(200, "application/json", b'{"ok":true}')
        else:
            self._send(404, "text/plain", b"not found")


def serve(core, nav_ctrl=None, wp_store=None, port=8001):
    global _core, _nav, _wp
    _core, _nav, _wp = core, nav_ctrl, wp_store
    ThreadingHTTPServer.allow_reuse_address = True
    try:
        srv = ThreadingHTTPServer(("0.0.0.0", port), _Handler)
    except OSError as e:
        print(f"[Web] could not bind port {port} ({e}). "
              f"Another SLAM instance may still be running — stop it, or pass a different port.")
        raise
    print(f"[Web] http://0.0.0.0:{port}/   (click the map to send Corndog there)")
    threading.Thread(target=_png_loop, daemon=True).start()
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


_PAGE = r"""<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Corndog SLAM</title><style>
html,body{margin:0;height:100%;background:#11131a;color:#cdd;font:14px system-ui;overflow:hidden}
#wrap{position:fixed;inset:0}
canvas{display:block;width:100%;height:100%;touch-action:none;cursor:crosshair}
#hud{position:fixed;top:10px;left:10px;background:#0d0f16cc;padding:10px 12px;border-radius:10px;min-width:170px}
#stop{position:fixed;top:10px;right:10px;background:#c0392b;color:#fff;border:0;border-radius:10px;
 padding:14px 20px;font-size:18px;font-weight:700;cursor:pointer}
#stop:active{transform:scale(.96)}
.row{margin:4px 0}.wp{cursor:pointer;color:#7fd3ff;margin:2px 0}.wp b:hover{text-decoration:underline}
.del{color:#e88;cursor:pointer;margin-left:6px}
#msg{color:#ffce6b;min-height:1.2em;max-width:200px}
button.sm{background:#243;color:#cfe;border:0;border-radius:6px;padding:4px 8px;cursor:pointer}
</style></head><body><div id=wrap><canvas id=c></canvas></div>
<button id=stop>STOP</button>
<div id=hud>
 <div class=row><b>state:</b> <span id=st>—</span></div>
 <div class=row id=msg></div>
 <div class=row><button class=sm id=savewp>save spot here</button></div>
 <div class=row><b>spots</b></div><div id=wps></div>
 <div class=row style="opacity:.6;font-size:12px">click map = go · drag = pan · wheel = zoom</div>
 <div class=row style="opacity:.45;font-size:11px">ui build v2.0.1 · <span id=fps>—</span></div>
</div>
<script>
const cv=document.getElementById('c'),ctx=cv.getContext('2d');
let MAP=null, S=null, mapRev=-1, mapBusy=false, lastMapT=0;
const mapCanvas=document.createElement('canvas'); let mapReady=false;
let view={scale:4,ox:0,oy:0}, drag=null, fitted=false;
function resize(){cv.width=innerWidth*devicePixelRatio;cv.height=innerHeight*devicePixelRatio;
 ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);}
addEventListener('resize',resize);resize();

function w2i(x,y){return [(x-MAP.ox)/MAP.res, (MAP.n-1)-((y-MAP.oy)/MAP.res)];}
function i2s(ix,iy){return [view.ox+ix*view.scale, view.oy+iy*view.scale];}
function s2i(sx,sy){return [(sx-view.ox)/view.scale,(sy-view.oy)/view.scale];}
function i2w(ix,iy){return [MAP.ox+ix*MAP.res, MAP.oy+((MAP.n-1)-iy)*MAP.res];}
function w2s(x,y){const a=w2i(x,y);return i2s(a[0],a[1]);}

// Refresh the map into a PERSISTENT offscreen canvas. We blit that canvas every
// frame; because a canvas is never "incomplete" (unlike a reloading <img>), the
// on-screen map can never blank — no flicker, no multi-second dropouts.
function maybeRefreshMap(rev){
 const now=performance.now();
 if(mapBusy || now-lastMapT<450) return;
 mapBusy=true; lastMapT=now;
 const im=new Image();
 im.onload=()=>{
   if(mapCanvas.width!==im.width){mapCanvas.width=im.width; mapCanvas.height=im.height;}
   const g=mapCanvas.getContext('2d');
   g.clearRect(0,0,im.width,im.height); g.drawImage(im,0,0);
   mapReady=true; mapRev=rev; mapBusy=false;
 };
 im.onerror=()=>{mapBusy=false;};
 im.src='/map.png?'+rev;
}

function fit(){if(!MAP)return;const W=innerWidth,H=innerHeight;
 view.scale=Math.min(W,H)/MAP.n*0.9; view.ox=(W-MAP.n*view.scale)/2; view.oy=(H-MAP.n*view.scale)/2; fitted=true;}

async function poll(){try{
 const r=await fetch('/state'); S=await r.json(); MAP=S.map;
 if(!fitted)fit();
 maybeRefreshMap(MAP.rev);
 document.getElementById('st').textContent=S.nav_state;
 const wd=document.getElementById('wps');wd.innerHTML='';
 S.waypoints.forEach(w=>{const d=document.createElement('div');d.className='wp';
  d.innerHTML='<b>▸ '+w.name+'</b><span class=del>✕</span>';
  d.querySelector('b').onclick=()=>fetch('/goto',{method:'POST',body:JSON.stringify({name:w.name})});
  d.querySelector('.del').onclick=()=>fetch('/delwaypoint',{method:'POST',body:JSON.stringify({name:w.name})});
  wd.appendChild(d);});
}catch(e){}}
setInterval(poll,200);

let _fc=0,_ft=performance.now();
function fpsTick(){_fc++;const n=performance.now();if(n-_ft>=1000){
 document.getElementById('fps').textContent=_fc+' fps';_fc=0;_ft=n;}}
function draw(){requestAnimationFrame(draw);fpsTick();if(!MAP||!S)return;
 ctx.clearRect(0,0,cv.width,cv.height);
 if(mapReady){const a=i2s(0,0);
  ctx.imageSmoothingEnabled=false;
  ctx.drawImage(mapCanvas,a[0],a[1],MAP.n*view.scale,MAP.n*view.scale);}
 // trail
 if(S.trail.length>1){ctx.strokeStyle='#ffbe46';ctx.lineWidth=2;ctx.beginPath();
  S.trail.forEach((p,i)=>{const s=w2s(p[0],p[1]);i?ctx.lineTo(s[0],s[1]):ctx.moveTo(s[0],s[1]);});ctx.stroke();}
 // planned path
 if(S.path.length>1){ctx.strokeStyle='#28e06e';ctx.lineWidth=3;ctx.beginPath();
  S.path.forEach((p,i)=>{const s=w2s(p[0],p[1]);i?ctx.lineTo(s[0],s[1]):ctx.moveTo(s[0],s[1]);});ctx.stroke();}
 // scan points
 ctx.fillStyle='#50e6f0';S.scan.forEach(p=>{const s=w2s(p[0],p[1]);ctx.fillRect(s[0]-1,s[1]-1,2,2);});
 // goal
 if(S.goal){const s=w2s(S.goal[0],S.goal[1]);ctx.strokeStyle='#ff5cff';ctx.lineWidth=2;
  ctx.beginPath();ctx.arc(s[0],s[1],8,0,7);ctx.stroke();}
 // robot oriented box (35x24cm)
 const px=S.pose.x,py=S.pose.y,th=S.pose.theta,L=S.robot[0]/2,Wd=S.robot[1]/2;
 const cs=Math.cos(th),sn=Math.sin(th);
 const corners=[[L,Wd],[L,-Wd],[-L,-Wd],[-L,Wd]].map(c=>{
   const wx=px+c[0]*cs-c[1]*sn, wy=py+c[0]*sn+c[1]*cs;return w2s(wx,wy);});
 ctx.fillStyle='#ff6b6b33';ctx.strokeStyle='#ff6b6b';ctx.lineWidth=2;ctx.beginPath();
 corners.forEach((s,i)=>i?ctx.lineTo(s[0],s[1]):ctx.moveTo(s[0],s[1]));ctx.closePath();ctx.fill();ctx.stroke();
 // heading tick
 const h=w2s(px+L*1.4*cs,py+L*1.4*sn),ctr=w2s(px,py);
 ctx.beginPath();ctx.moveTo(ctr[0],ctr[1]);ctx.lineTo(h[0],h[1]);ctx.stroke();
}
draw();

// interaction
let moved=false;
cv.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,ox:view.ox,oy:view.oy};moved=false;});
cv.addEventListener('pointermove',e=>{if(!drag)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;
 if(Math.abs(dx)+Math.abs(dy)>4)moved=true;view.ox=drag.ox+dx;view.oy=drag.oy+dy;});
cv.addEventListener('pointerup',e=>{const wasDrag=moved;drag=null;if(wasDrag||!MAP)return;
 const r=cv.getBoundingClientRect();const ii=s2i(e.clientX-r.left,e.clientY-r.top);const w=i2w(ii[0],ii[1]);
 fetch('/goal',{method:'POST',body:JSON.stringify({x:w[0],y:w[1]})}).then(r=>r.json()).then(j=>{
   document.getElementById('msg').textContent=j.ok?'':('✗ '+j.reason);});});
cv.addEventListener('wheel',e=>{e.preventDefault();const r=cv.getBoundingClientRect();
 const mx=e.clientX-r.left,my=e.clientY-r.top;const f=e.deltaY<0?1.15:1/1.15;
 view.ox=mx-(mx-view.ox)*f;view.oy=my-(my-view.oy)*f;view.scale*=f;},{passive:false});
document.getElementById('stop').onclick=()=>fetch('/stop',{method:'POST'});
document.getElementById('savewp').onclick=()=>{if(!S)return;const n=prompt('Name this spot:');if(!n)return;
 fetch('/waypoint',{method:'POST',body:JSON.stringify({name:n,x:S.pose.x,y:S.pose.y})});};
</script></body></html>"""
