"""
render.py — Render the occupancy grid + robot + live scan + trail to a PIL
image. Pure Pillow/numpy so it runs headless on the Pi and is fast enough to
push as MJPEG. Also used by the simulation demo to overlay ground truth.

Colour scheme:
  occupied -> light,  free -> dark,  unknown -> mid grey
  accumulated map (occupied) ..... white-ish
  live current scan .............. cyan
  robot ......................... red dot + heading line
  trajectory trail (~30 s) ....... amber
  (sim only) ground-truth path ... green
"""

from __future__ import annotations
import io
import math
import numpy as np
from PIL import Image, ImageDraw


def _grid_to_rgb(grid, upscale):
    p = grid.prob()                      # 0..1, 0.5 = unknown
    img = np.full((grid.n, grid.n, 3), 40, np.uint8)        # unknown grey
    free = p < 0.35
    occ = p > 0.65
    img[free] = (24, 28, 46)             # dark navy free space
    shade = np.clip((p - 0.65) / 0.35, 0, 1)
    occv = (120 + 135 * shade).astype(np.uint8)
    img[occ] = np.stack([occv, occv, (occv.astype(np.int16) + 20).clip(0, 255).astype(np.uint8)], -1)[occ]

    im = Image.fromarray(img, "RGB")
    if upscale > 1:
        im = im.resize((grid.n * upscale, grid.n * upscale), Image.NEAREST)
    # grid is [row=y, col=x] with y increasing downward in array terms; flip so
    # +y is up on screen (conventional map view).
    return im.transpose(Image.FLIP_TOP_BOTTOM)


def _w2px(grid, x, y, upscale):
    cx = (x - grid.ox) / grid.res * upscale
    cy = (y - grid.oy) / grid.res * upscale
    return cx, grid.n * upscale - cy        # flip y for screen


def render(core, upscale=3, scan_pts=None, true_path=None):
    grid = core.grid
    im = _grid_to_rgb(grid, upscale).convert("RGB")
    d = ImageDraw.Draw(im)

    # ground-truth path (sim only)
    if true_path is not None and len(true_path) > 1:
        pts = [ _w2px(grid, x, y, upscale) for x, y in true_path ]
        d.line(pts, fill=(80, 220, 120), width=1)

    # trajectory trail
    with core.lock:
        trail = list(core.trail)
        sx, sy = core.last_scan_world
        pose = core.pose
    if len(trail) > 1:
        pts = [ _w2px(grid, x, y, upscale) for _, x, y in trail ]
        d.line(pts, fill=(255, 190, 70), width=2)

    # live scan
    if sx is not None and len(sx):
        for x, y in zip(sx, sy):
            px, py = _w2px(grid, x, y, upscale)
            d.point((px, py), fill=(80, 230, 240))

    # robot + heading
    rx, ry = _w2px(grid, pose[0], pose[1], upscale)
    r = 4
    d.ellipse([rx - r, ry - r, rx + r, ry + r], fill=(255, 80, 80))
    hx = rx + 14 * math.cos(pose[2])
    hy = ry - 14 * math.sin(pose[2])
    d.line([rx, ry, hx, hy], fill=(255, 80, 80), width=2)
    return im


def to_jpeg(im, quality=70):
    buf = io.BytesIO()
    im.save(buf, "JPEG", quality=quality)
    return buf.getvalue()
