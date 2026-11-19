"""Profiles and test records for the web UI, stored as JSON files (no ROS imports).

    <data_dir>/profiles/<name>.json   {"name", "note", "guard": {...}, "drive": {...}, "updated"}
    <data_dir>/profiles/_default      name of the profile applied when the stack starts
    <data_dir>/tests/<id>.json        one recorded test: params, events, 5 Hz samples, summary

On the car data_dir is /data/nav/ui (a volume: survives restarts and image
updates); in the simulator /data/sim/ui (= run_logs/sim/ui on the laptop).
"""
from __future__ import annotations

import json
import math
import os
import re
import time
from dataclasses import dataclass, field

_NAME = re.compile(r"[^A-Za-z0-9_.\- ]+")


def safe_name(name: str) -> str:
    """A file-safe profile / test name (letters, digits, space, _ . -), max 60 chars."""
    n = _NAME.sub("", str(name)).strip().strip(".")
    if not n:
        raise ValueError("empty name")
    return n[:60]


def _write(path: str, data: dict) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=1)
    os.replace(tmp, path)


class ProfileStore:
    def __init__(self, data_dir: str) -> None:
        self.dir = os.path.join(data_dir, "profiles")
        os.makedirs(self.dir, exist_ok=True)

    def _path(self, name: str) -> str:
        return os.path.join(self.dir, safe_name(name) + ".json")

    def list(self) -> list[dict]:
        out = []
        for f in sorted(os.listdir(self.dir)):
            if f.endswith(".json"):
                try:
                    with open(os.path.join(self.dir, f), encoding="utf-8") as fh:
                        d = json.load(fh)
                    out.append({"name": d.get("name", f[:-5]), "note": d.get("note", ""),
                                "updated": d.get("updated")})
                except (OSError, ValueError):
                    continue
        default = self.default()
        for p in out:
            p["default"] = p["name"] == default
        return out

    def get(self, name: str) -> dict | None:
        try:
            with open(self._path(name), encoding="utf-8") as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return None

    def put(self, name: str, body: dict) -> dict:
        name = safe_name(name)
        d = {"name": name, "note": str(body.get("note", ""))[:500],
             "guard": dict(body.get("guard") or {}), "drive": dict(body.get("drive") or {}),
             "updated": time.strftime("%Y-%m-%d %H:%M:%S")}
        _write(self._path(name), d)
        return d

    def delete(self, name: str) -> bool:
        try:
            os.remove(self._path(name))
        except OSError:
            return False
        if self.default() == safe_name(name):
            self.set_default(None)
        return True

    def default(self) -> str | None:
        try:
            with open(os.path.join(self.dir, "_default"), encoding="utf-8") as fh:
                n = fh.read().strip()
            return n or None
        except OSError:
            return None

    def set_default(self, name: str | None) -> None:
        with open(os.path.join(self.dir, "_default"), "w", encoding="utf-8") as fh:
            fh.write("" if name is None else safe_name(name))


@dataclass
class TestRecorder:
    """Collects one test run: samples (5 Hz), events, then a summary."""
    name: str
    note: str = ""
    params: dict = field(default_factory=dict)
    started: float = field(default_factory=time.time)
    samples: list = field(default_factory=list)
    events: list = field(default_factory=list)
    run: str = ""          # id of the rosbag run that was recording when the test started ('' = none)

    def event(self, kind: str, **data) -> None:
        self.events.append({"t": round(time.time() - self.started, 2), "kind": kind, **data})

    def sample(self, pose, guard_state: str | None, nearest, wheels, n_dyn: int) -> None:
        self.samples.append({"t": round(time.time() - self.started, 2), "pose": pose, "guard": guard_state,
                             "nearest": nearest, "wheels": wheels, "n_dyn": n_dyn})

    def summary(self) -> dict:
        dist, prev, vmax = 0.0, None, 0.0
        for s in self.samples:
            if s["pose"] and prev:
                dist += math.hypot(s["pose"][0] - prev[0], s["pose"][1] - prev[1])
            if s["pose"]:
                prev = s["pose"]
            if s["wheels"]:
                vmax = max(vmax, 0.5 * abs(s["wheels"][0] + s["wheels"][1]))
        near = [s["nearest"] for s in self.samples if s["nearest"] is not None]
        stops = sum(1 for e in self.events if e["kind"] == "guard" and e.get("to") in ("stop", "estop"))
        return {"duration_s": round(time.time() - self.started, 1), "distance_m": round(dist, 2),
                "max_cmd_speed_mps": round(vmax, 3), "guard_stops": stops,
                "min_person_distance_m": round(min(near), 3) if near else None,
                "goals": sum(1 for e in self.events if e["kind"] == "goal")}

    def to_dict(self) -> dict:
        return {"name": self.name, "note": self.note, "verdict": "",
                "started": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.started)),
                "run": self.run, "params": self.params, "summary": self.summary(), "events": self.events,
                "samples": self.samples}


class TestStore:
    def __init__(self, data_dir: str) -> None:
        self.dir = os.path.join(data_dir, "tests")
        os.makedirs(self.dir, exist_ok=True)

    def save(self, rec: TestRecorder) -> str:
        tid = time.strftime("%Y%m%d-%H%M%S", time.localtime(rec.started)) + "_" + safe_name(rec.name).replace(" ", "_")
        _write(os.path.join(self.dir, tid + ".json"), rec.to_dict())
        return tid

    def list(self) -> list[dict]:
        out = []
        for f in sorted(os.listdir(self.dir), reverse=True):
            if not f.endswith(".json"):
                continue
            try:
                with open(os.path.join(self.dir, f), encoding="utf-8") as fh:
                    d = json.load(fh)
            except (OSError, ValueError):
                continue
            out.append({"id": f[:-5], "name": d.get("name"), "note": d.get("note", ""),
                        "verdict": d.get("verdict", ""), "started": d.get("started"), "summary": d.get("summary"),
                        "run": d.get("run", "")})
        return out

    def _path(self, tid: str) -> str:
        return os.path.join(self.dir, safe_name(tid) + ".json")

    def get(self, tid: str) -> dict | None:
        try:
            with open(self._path(tid), encoding="utf-8") as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return None

    def update(self, tid: str, body: dict) -> dict | None:
        d = self.get(tid)
        if d is None:
            return None
        for k in ("note", "verdict"):
            if k in body:
                d[k] = str(body[k])[:2000]
        _write(self._path(tid), d)
        return d

    def delete(self, tid: str) -> bool:
        try:
            os.remove(self._path(tid))
            return True
        except OSError:
            return False
