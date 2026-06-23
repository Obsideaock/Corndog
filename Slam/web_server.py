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
_ko = None
_lc = None
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
    nav_snap = _nav.snapshot() if _nav else dict(state="idle", goal=None, path=[], reason="",
                                                 roam=False, route=[], route_i=0, loop=False)
    return {
        "pose": {"x": float(pose[0]), "y": float(pose[1]), "theta": float(pose[2])},
        "scan": scan,
        "trail": trail[-160:],
        "path": [list(p) for p in nav_snap["path"]],
        "goal": list(nav_snap["goal"]) if nav_snap["goal"] else None,
        "nav_state": nav_snap["state"],
        "reason": nav_snap["reason"],
        "roam": nav_snap.get("roam", False),
        "route": [list(p) for p in nav_snap.get("route", [])],
        "route_i": nav_snap.get("route_i", 0),
        "nogo": _ko.all() if _ko else [],
        "waypoints": _wp.all() if _wp else [],
        "slam": (_lc.status() if _lc else {}),
        "stats": dict(getattr(_core, "stats", {})),
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
        elif self.path == "/route":
            pts = b.get("points", [])
            ok, reason = (_nav.set_route(pts, bool(b.get("loop", True)))
                          if _nav else (False, "nav off"))
            self._send(200, "application/json",
                       json.dumps({"ok": ok, "reason": reason}).encode())
        elif self.path == "/roam":
            if _nav:
                _nav.set_roam(bool(b.get("on", False)))
            self._send(200, "application/json", b'{"ok":true}')
        elif self.path == "/nogo":
            if _ko:
                _ko.add_rect(float(b["x0"]), float(b["y0"]),
                             float(b["x1"]), float(b["y1"]))
            self._send(200, "application/json", b'{"ok":true}')
        elif self.path == "/delnogo":
            if _ko:
                _ko.delete(int(b["id"]))
            self._send(200, "application/json", b'{"ok":true}')
        elif self.path == "/nogo_clear":
            if _ko:
                _ko.clear(auto_only=bool(b.get("auto_only", False)))
            self._send(200, "application/json", b'{"ok":true}')
        elif self.path == "/reset":
            if _nav:
                _nav.cancel()
            if _core:
                _core.reset_map()
            if _ko:
                _ko.clear()
            if _wp:
                for w in _wp.all():
                    _wp.delete(w["name"])
            if _lc and hasattr(_lc, "reset"):
                _lc.reset()
            self._send(200, "application/json", b'{"ok":true}')
        else:
            self._send(404, "text/plain", b"not found")


def serve(core, nav_ctrl=None, wp_store=None, keepout=None, loop_closer=None, port=8001):
    global _core, _nav, _wp, _ko, _lc
    _core, _nav, _wp, _ko, _lc = core, nav_ctrl, wp_store, keepout, loop_closer
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
<meta name=viewport content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Corndog SLAM</title><style>
html,body{margin:0;height:100%;background:#11131a;color:#cdd;font:14px system-ui;overflow:hidden;
 -webkit-user-select:none;user-select:none}
#wrap{position:fixed;inset:0}
canvas{display:block;width:100%;height:100%;touch-action:none;cursor:crosshair}
#hud{position:fixed;top:10px;left:10px;background:#0d0f16e0;padding:10px 12px;border-radius:12px;
 min-width:172px;max-width:46vw}
#stop{position:fixed;top:10px;right:10px;background:#c0392b;color:#fff;border:0;border-radius:12px;
 padding:14px 22px;font-size:18px;font-weight:700;cursor:pointer}
#stop:active{transform:scale(.96)}
.row{margin:5px 0}
.mb{background:#1c2030;color:#bcd;border:1px solid #2c3346;border-radius:8px;padding:7px 10px;cursor:pointer;margin-right:4px}
.mb.sel{background:#2d6cdf;color:#fff;border-color:#2d6cdf}
button.sm{background:#243049;color:#cfe;border:0;border-radius:8px;padding:7px 10px;cursor:pointer;margin:2px 4px 2px 0}
button.go{background:#1f6f43;color:#dfe}
.wp{cursor:pointer;color:#7fd3ff;margin:2px 0}.wp b:hover{text-decoration:underline}
.del{color:#e88;cursor:pointer;margin-left:6px}
#msg{color:#ffce6b;min-height:1.1em}
.hint{opacity:.55;font-size:12px}.ver{opacity:.4;font-size:11px}
#roamtag{color:#7fe0a0;font-weight:700}
@media (max-width:760px){
 html,body{font-size:16px}
 #hud{max-width:62vw;padding:12px}
 .mb,button.sm{padding:11px 13px}
 #stop{padding:18px 26px;font-size:22px}
}
</style></head><body><div id=wrap><canvas id=c></canvas></div>
<button id=stop>STOP</button>
<div id=hud>
 <div class=row><b>state:</b> <span id=st>—</span> <span id=roamtag></span></div>
 <div class=row id=msg></div>
 <div class=row id=modes>
   <button class="mb sel" data-m="go">Go</button>
   <button class="mb" data-m="route">Route</button>
   <button class="mb" data-m="nogo">No-go</button>
 </div>
 <div class=row><button class=sm id=roam>Roam: off</button></div>
 <div class=row id=ctx></div>
 <div class=row><button class=sm id=savewp>save spot here</button>
   <button class=sm id=clearnogo>clear no-go</button></div>
 <div class=row><button class=sm id=resetmap style="background:#7a2230;color:#fdd">⟲ reset map</button></div>
 <div class=row><b>spots</b></div><div id=wps></div>
 <div class=row hint id=hint>tap = go · drag = pan · pinch/wheel = zoom</div>
 <div class=row ver>ui build v2.3.0 · <span id=fps>—</span> · <span id=slam></span></div>
 <div class=row ver id=fe></div>
</div>
<script>
const cv=document.getElementById('c'),ctx=cv.getContext('2d');
let MAP=null,S=null,mapRev=-1,mapBusy=false,lastMapT=0;
const mapCanvas=document.createElement('canvas');let mapReady=false;
let view={scale:4,ox:0,oy:0},fitted=false;
let mode='go', routePts=[], rectStart=null, rectEnd=null;
let lastRev=-1,lastRevT=0,slamHz=0;
function resize(){cv.width=innerWidth*devicePixelRatio;cv.height=innerHeight*devicePixelRatio;
 ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);}
addEventListener('resize',resize);resize();

function w2i(x,y){return [(x-MAP.ox)/MAP.res,(MAP.n-1)-((y-MAP.oy)/MAP.res)];}
function i2s(ix,iy){return [view.ox+ix*view.scale,view.oy+iy*view.scale];}
function s2i(sx,sy){return [(sx-view.ox)/view.scale,(sy-view.oy)/view.scale];}
function i2w(ix,iy){return [MAP.ox+ix*MAP.res,MAP.oy+((MAP.n-1)-iy)*MAP.res];}
function w2s(x,y){const a=w2i(x,y);return i2s(a[0],a[1]);}
function evWorld(e){const r=cv.getBoundingClientRect();const ii=s2i(e.clientX-r.left,e.clientY-r.top);return i2w(ii[0],ii[1]);}
function dist(a,b){return Math.hypot(a.x-b.x,a.y-b.y);}

function maybeRefreshMap(rev){const now=performance.now();
 if(mapBusy||now-lastMapT<450)return;mapBusy=true;lastMapT=now;
 const im=new Image();
 im.onload=()=>{if(mapCanvas.width!==im.width){mapCanvas.width=im.width;mapCanvas.height=im.height;}
  const g=mapCanvas.getContext('2d');g.clearRect(0,0,im.width,im.height);g.drawImage(im,0,0);
  mapReady=true;mapRev=rev;mapBusy=false;};
 im.onerror=()=>{mapBusy=false;};im.src='/map.png?'+rev;}

function fit(){if(!MAP)return;const W=innerWidth,H=innerHeight;
 view.scale=Math.min(W,H)/MAP.n*0.9;view.ox=(W-MAP.n*view.scale)/2;view.oy=(H-MAP.n*view.scale)/2;fitted=true;}

async function poll(){try{
 const r=await fetch('/state');S=await r.json();MAP=S.map;if(!fitted)fit();
 maybeRefreshMap(MAP.rev);
 document.getElementById('st').textContent=S.nav_state;
 document.getElementById('roamtag').textContent=S.roam?'· ROAM':'';
 document.getElementById('roam').textContent='Roam: '+(S.roam?'on':'off');
 const sl=S.slam||{};document.getElementById('slam').textContent=
   (sl.keyframes!==undefined)?('kf '+sl.keyframes+' · loops '+(sl.loops||0)):'';
 const st=S.stats||{},tn=performance.now();
 if(lastRev>=0 && tn>lastRevT+1) slamHz=0.7*slamHz+0.3*((MAP.rev-lastRev)*1000/(tn-lastRevT));
 lastRev=MAP.rev;lastRevT=tn;
 document.getElementById('fe').textContent='SLAM '+slamHz.toFixed(1)+'Hz · match '
   +(st.score!==undefined?st.score:'—')+' · gated '+(st.gated||0)+' · mapskip '+(st.moving||0);
 const wd=document.getElementById('wps');wd.innerHTML='';
 (S.waypoints||[]).forEach(w=>{const d=document.createElement('div');d.className='wp';
  d.innerHTML='<b>▸ '+w.name+'</b><span class=del>✕</span>';
  d.querySelector('b').onclick=()=>fetch('/goto',{method:'POST',body:JSON.stringify({name:w.name})});
  d.querySelector('.del').onclick=()=>fetch('/delwaypoint',{method:'POST',body:JSON.stringify({name:w.name})});
  wd.appendChild(d);});
}catch(e){}}
setInterval(poll,200);

let _fc=0,_ft=performance.now();
function fpsTick(){_fc++;const n=performance.now();if(n-_ft>=1000){
 document.getElementById('fps').textContent=_fc+' fps';_fc=0;_ft=n;}}

function poly(pp,close){ctx.beginPath();pp.forEach((s,i)=>i?ctx.lineTo(s[0],s[1]):ctx.moveTo(s[0],s[1]));if(close)ctx.closePath();}

function draw(){requestAnimationFrame(draw);fpsTick();if(!MAP||!S)return;
 ctx.clearRect(0,0,cv.width,cv.height);
 if(mapReady){const a=i2s(0,0);ctx.imageSmoothingEnabled=false;
  ctx.drawImage(mapCanvas,a[0],a[1],MAP.n*view.scale,MAP.n*view.scale);}
 // no-go zones
 (S.nogo||[]).forEach(z=>{
  if(z.kind==='rect'){const a=w2s(z.x0,z.y0),b=w2s(z.x1,z.y1);
   ctx.fillStyle='#e0405533';ctx.strokeStyle='#e04055';ctx.lineWidth=2;
   ctx.fillRect(Math.min(a[0],b[0]),Math.min(a[1],b[1]),Math.abs(b[0]-a[0]),Math.abs(b[1]-a[1]));
   ctx.strokeRect(Math.min(a[0],b[0]),Math.min(a[1],b[1]),Math.abs(b[0]-a[0]),Math.abs(b[1]-a[1]));}
  else{const c=w2s(z.x,z.y);const rr=z.r/MAP.res*view.scale;
   ctx.fillStyle=z.auto?'#e8902033':'#e0405533';ctx.strokeStyle=z.auto?'#e89020':'#e04055';ctx.lineWidth=2;
   ctx.beginPath();ctx.arc(c[0],c[1],rr,0,7);ctx.fill();ctx.stroke();}});
 // rectangle being drawn
 if(rectStart&&rectEnd){const a=w2s(rectStart[0],rectStart[1]),b=w2s(rectEnd[0],rectEnd[1]);
  ctx.setLineDash([6,5]);ctx.strokeStyle='#ff6680';ctx.lineWidth=2;
  ctx.strokeRect(Math.min(a[0],b[0]),Math.min(a[1],b[1]),Math.abs(b[0]-a[0]),Math.abs(b[1]-a[1]));ctx.setLineDash([]);}
 // trail
 if(S.trail.length>1){ctx.strokeStyle='#ffbe46';ctx.lineWidth=2;poly(S.trail.map(p=>w2s(p[0],p[1])),false);ctx.stroke();}
 // active route (loop)
 if(S.route&&S.route.length){const pp=S.route.map(p=>w2s(p[0],p[1]));
  ctx.strokeStyle='#28e06e';ctx.lineWidth=2;ctx.setLineDash([4,4]);poly(pp,S.route.length>2);ctx.stroke();ctx.setLineDash([]);
  pp.forEach((s,i)=>{ctx.fillStyle=(i===S.route_i)?'#fff':'#28e06e';ctx.beginPath();ctx.arc(s[0],s[1],6,0,7);ctx.fill();
   ctx.fillStyle='#062';ctx.font='10px system-ui';ctx.fillText(i+1,s[0]-3,s[1]+3);});}
 // planned path
 if(S.path.length>1){ctx.strokeStyle='#28e06e';ctx.lineWidth=3;poly(S.path.map(p=>w2s(p[0],p[1])),false);ctx.stroke();}
 // route being built locally
 if(routePts.length){const pp=routePts.map(p=>w2s(p[0],p[1]));
  ctx.strokeStyle='#50e6f0';ctx.lineWidth=2;ctx.setLineDash([3,4]);poly(pp,false);ctx.stroke();ctx.setLineDash([]);
  pp.forEach((s,i)=>{ctx.fillStyle='#50e6f0';ctx.beginPath();ctx.arc(s[0],s[1],7,0,7);ctx.fill();
   ctx.fillStyle='#023';ctx.font='11px system-ui';ctx.fillText(i+1,s[0]-3,s[1]+4);});}
 // scan
 ctx.fillStyle='#50e6f0';S.scan.forEach(p=>{const s=w2s(p[0],p[1]);ctx.fillRect(s[0]-1,s[1]-1,2,2);});
 // goal
 if(S.goal){const s=w2s(S.goal[0],S.goal[1]);ctx.strokeStyle='#ff5cff';ctx.lineWidth=2;ctx.beginPath();ctx.arc(s[0],s[1],8,0,7);ctx.stroke();}
 // robot 35x24cm oriented box
 const px=S.pose.x,py=S.pose.y,th=S.pose.theta,L=S.robot[0]/2,Wd=S.robot[1]/2,cs=Math.cos(th),sn=Math.sin(th);
 const corners=[[L,Wd],[L,-Wd],[-L,-Wd],[-L,Wd]].map(c=>w2s(px+c[0]*cs-c[1]*sn,py+c[0]*sn+c[1]*cs));
 ctx.fillStyle='#ff6b6b33';ctx.strokeStyle='#ff6b6b';ctx.lineWidth=2;poly(corners,true);ctx.fill();ctx.stroke();
 const h=w2s(px+L*1.4*cs,py+L*1.4*sn),ctr=w2s(px,py);ctx.beginPath();ctx.moveTo(ctr[0],ctr[1]);ctx.lineTo(h[0],h[1]);ctx.stroke();
}
draw();

// ---- modes ----
function setMode(m){mode=m;routePts=[];rectStart=rectEnd=null;
 document.querySelectorAll('.mb').forEach(b=>b.classList.toggle('sel',b.dataset.m===m));
 const hint={go:'tap = go · drag = pan · pinch/wheel = zoom',
  route:'tap to add stops · then Start loop · drag = pan',
  nogo:'drag a box over areas to avoid · drag = pan elsewhere'}[m];
 document.getElementById('hint').textContent=hint;renderCtx();}
function renderCtx(){const c=document.getElementById('ctx');c.innerHTML='';
 if(mode==='route'){
  const s=document.createElement('button');s.className='sm go';s.textContent='Start loop ('+routePts.length+')';
  s.onclick=()=>{if(routePts.length<2){msg('add at least 2 stops');return;}
   fetch('/route',{method:'POST',body:JSON.stringify({points:routePts,loop:true})}).then(r=>r.json())
    .then(j=>{msg(j.ok?'':('✗ '+j.reason));if(j.ok){routePts=[];renderCtx();}});};
  const cl=document.createElement('button');cl.className='sm';cl.textContent='clear stops';
  cl.onclick=()=>{routePts=[];renderCtx();};
  c.appendChild(s);c.appendChild(cl);}}
document.querySelectorAll('.mb').forEach(b=>b.onclick=()=>setMode(b.dataset.m));
function msg(t){document.getElementById('msg').textContent=t;}

// ---- unified pointer (mouse + touch + pinch) ----
let pmap=new Map(),last=null,pinch=null,movedFar=false;
cv.addEventListener('pointerdown',e=>{cv.setPointerCapture(e.pointerId);
 pmap.set(e.pointerId,{x:e.clientX,y:e.clientY});movedFar=false;
 if(pmap.size===2){const a=[...pmap.values()];
  pinch={d:dist(a[0],a[1]),cx:(a[0].x+a[1].x)/2,cy:(a[0].y+a[1].y)/2,scale:view.scale,ox:view.ox,oy:view.oy};rectStart=rectEnd=null;}
 else{last={x:e.clientX,y:e.clientY,ox:view.ox,oy:view.oy};
  if(mode==='nogo'&&MAP){rectStart=evWorld(e);rectEnd=rectStart;}}});
cv.addEventListener('pointermove',e=>{if(!pmap.has(e.pointerId))return;
 pmap.set(e.pointerId,{x:e.clientX,y:e.clientY});
 if(pmap.size>=2&&pinch){const a=[...pmap.values()];const f=dist(a[0],a[1])/(pinch.d||1);
  view.scale=pinch.scale*f;view.ox=pinch.cx-(pinch.cx-pinch.ox)*f;view.oy=pinch.cy-(pinch.cy-pinch.oy)*f;movedFar=true;return;}
 if(pmap.size===1&&last){const dx=e.clientX-last.x,dy=e.clientY-last.y;if(Math.abs(dx)+Math.abs(dy)>5)movedFar=true;
  if(mode==='nogo'&&rectStart){rectEnd=evWorld(e);return;}
  view.ox=last.ox+dx;view.oy=last.oy+dy;}});
function endPointer(e){const m=mode,rs=rectStart,wasMoved=movedFar;
 pmap.delete(e.pointerId);
 if(pmap.size>0)return;
 pinch=null;
 if(m==='nogo'&&rs){const re=evWorld(e);rectStart=rectEnd=null;
  if(Math.hypot(re[0]-rs[0],re[1]-rs[1])>0.1){fetch('/nogo',{method:'POST',
   body:JSON.stringify({x0:rs[0],y0:rs[1],x1:re[0],y1:re[1]})}).then(()=>msg('no-go added'));}
  return;}
 if(wasMoved||!MAP)return;
 const w=evWorld(e);
 if(m==='go'){fetch('/goal',{method:'POST',body:JSON.stringify({x:w[0],y:w[1]})}).then(r=>r.json())
   .then(j=>msg(j.ok?'':('✗ '+j.reason)));}
 else if(m==='route'){routePts.push([w[0],w[1]]);renderCtx();}}
cv.addEventListener('pointerup',endPointer);
cv.addEventListener('pointercancel',endPointer);
cv.addEventListener('wheel',e=>{e.preventDefault();const r=cv.getBoundingClientRect();
 const mx=e.clientX-r.left,my=e.clientY-r.top,f=e.deltaY<0?1.15:1/1.15;
 view.ox=mx-(mx-view.ox)*f;view.oy=my-(my-view.oy)*f;view.scale*=f;},{passive:false});

document.getElementById('stop').onclick=()=>{routePts=[];renderCtx();fetch('/stop',{method:'POST'});};
document.getElementById('roam').onclick=()=>fetch('/roam',{method:'POST',body:JSON.stringify({on:!(S&&S.roam)})});
document.getElementById('savewp').onclick=()=>{if(!S)return;const n=prompt('Name this spot:');if(!n)return;
 fetch('/waypoint',{method:'POST',body:JSON.stringify({name:n,x:S.pose.x,y:S.pose.y})});};
document.getElementById('clearnogo').onclick=()=>fetch('/nogo_clear',{method:'POST',body:JSON.stringify({})}).then(()=>msg('no-go cleared'));
document.getElementById('resetmap').onclick=()=>{
 if(!confirm('Erase the map, trail, saved spots and no-go zones and start fresh from here?'))return;
 routePts=[];rectStart=rectEnd=null;renderCtx();
 fetch('/reset',{method:'POST'}).then(()=>msg('map reset — remapping from here'));};
setMode('go');
</script></body></html>"""
