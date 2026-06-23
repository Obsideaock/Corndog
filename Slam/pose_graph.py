"""
pose_graph.py — 2D (SE(2)) pose-graph optimisation in pure numpy. No scipy.

This is the maths core of loop closure. Nodes are robot poses (x, y, theta);
edges are relative-pose measurements with an information matrix. Consecutive
keyframes give "odometry" edges (from the SLAM frontend); when the robot
re-recognises an old place, a "loop" edge ties the two together. Optimising the
graph distributes the accumulated drift so the trajectory (and the rebuilt map)
snaps back into global consistency.

Gauss-Newton with the standard closed-form SE(2) Jacobians (Grisetti et al.,
"A Tutorial on Graph-Based SLAM"). Node 0 is anchored. For the keyframe counts
we see indoors (tens to low hundreds) a dense solve is plenty fast and runs only
when a loop is actually found.
"""
from __future__ import annotations
import math
import numpy as np


def v2t(p):
    c, s = math.cos(p[2]), math.sin(p[2])
    return np.array([[c, -s, p[0]],
                     [s,  c, p[1]],
                     [0,  0, 1.0]])


def t2v(T):
    return np.array([T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Edge:
    __slots__ = ("i", "j", "z", "omega", "loop")

    def __init__(self, i, j, z, omega=None, loop=False):
        self.i = int(i)
        self.j = int(j)
        self.z = np.asarray(z, float)          # relative pose i->j (3,)
        self.omega = np.eye(3) if omega is None else np.asarray(omega, float)
        self.loop = loop


def relative(xi, xj):
    """Relative pose measurement i->j given world poses xi, xj (for odom edges)."""
    return t2v(np.linalg.inv(v2t(xi)) @ v2t(xj))


def optimize(nodes, edges, iterations=12, anchor=0, tol=1e-5):
    """
    nodes: (N,3) initial world poses. edges: list[Edge].
    Returns (optimised (N,3), info dict). Pure Gauss-Newton; node `anchor` fixed.
    """
    x = np.array(nodes, float).reshape(-1, 3).copy()
    N = x.shape[0]
    n = 3 * N
    last_chi = None
    for it in range(iterations):
        H = np.zeros((n, n))
        b = np.zeros(n)
        chi = 0.0
        for e in edges:
            i, j = e.i, e.j
            xi, xj = x[i], x[j]
            Xi, Xj, Z = v2t(xi), v2t(xj), v2t(e.z)
            E = np.linalg.inv(Z) @ (np.linalg.inv(Xi) @ Xj)
            err = t2v(E)
            err[2] = _wrap(err[2])

            si, ci = math.sin(xi[2]), math.cos(xi[2])
            Ri = Xi[:2, :2]
            Rz = Z[:2, :2]
            dRiT = np.array([[-si, ci], [-ci, -si]])     # d(Ri^T)/dtheta
            dt = xj[:2] - xi[:2]

            A = np.zeros((3, 3))
            A[:2, :2] = -Rz.T @ Ri.T
            A[:2, 2] = Rz.T @ dRiT @ dt
            A[2, 2] = -1.0
            B = np.zeros((3, 3))
            B[:2, :2] = Rz.T @ Ri.T
            B[2, 2] = 1.0

            Om = e.omega
            chi += float(err @ Om @ err)
            ii, jj = slice(3 * i, 3 * i + 3), slice(3 * j, 3 * j + 3)
            H[ii, ii] += A.T @ Om @ A
            H[ii, jj] += A.T @ Om @ B
            H[jj, ii] += B.T @ Om @ A
            H[jj, jj] += B.T @ Om @ B
            b[ii] += A.T @ Om @ err
            b[jj] += B.T @ Om @ err

        # anchor: pin one node so H is non-singular
        a = slice(3 * anchor, 3 * anchor + 3)
        H[a, a] += np.eye(3) * 1e6

        try:
            dx = np.linalg.solve(H, -b)
        except np.linalg.LinAlgError:
            dx = np.linalg.lstsq(H, -b, rcond=None)[0]

        x = x.reshape(-1)
        x = x + dx
        x = x.reshape(-1, 3)
        x[:, 2] = (x[:, 2] + np.pi) % (2 * np.pi) - np.pi

        if last_chi is not None and abs(last_chi - chi) < tol * max(1.0, last_chi):
            last_chi = chi
            break
        last_chi = chi

    return x, {"chi2": float(last_chi), "iters": it + 1, "nodes": N, "edges": len(edges)}
