"""waypoints.py — named map locations, saved next to the map file."""
from __future__ import annotations
import os
import json
import threading


class WaypointStore:
    def __init__(self, map_path=None):
        self.path = (map_path + ".waypoints.json") if map_path else None
        self._wp = {}
        self._lock = threading.Lock()
        self.load()

    def load(self):
        if self.path and os.path.exists(self.path):
            try:
                with open(self.path) as f:
                    self._wp = json.load(f)
            except Exception:
                self._wp = {}

    def save(self):
        if not self.path:
            return
        try:
            with open(self.path, "w") as f:
                json.dump(self._wp, f)
        except Exception:
            pass

    def add(self, name, x, y):
        with self._lock:
            self._wp[name] = [float(x), float(y)]
        self.save()

    def delete(self, name):
        with self._lock:
            self._wp.pop(name, None)
        self.save()

    def get(self, name):
        with self._lock:
            return self._wp.get(name)

    def all(self):
        with self._lock:
            return [{"name": k, "x": v[0], "y": v[1]} for k, v in self._wp.items()]
