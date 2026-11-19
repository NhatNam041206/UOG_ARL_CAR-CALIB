"""Recorded runs (rosbag2) of the web UI: folders, sizes, zip download, disk guard (no ROS imports).

    <data_dir>/runs/<id>/meta.json   {"id","name","note","started","topics","storage","state","stop_reason", ...}
    <data_dir>/runs/<id>/bag/        the rosbag2 directory (`ros2 bag play <dir>/bag`)
    <data_dir>/runs/<id>/record.log  what `ros2 bag record` printed

A *test record* (Tests tab, <data_dir>/tests/*.json) is a light JSON summary of one test (events, 5 Hz
samples, verdict). A *run* is the full bag: every message of the important topics, to replay or analyse
later. A test started while a run records is linked to it (both hold the other's id).
"""
from __future__ import annotations

import json
import os
import shutil
import time
import zipfile

from .ui_store import safe_name

DEFAULT_TOPICS = [
    "/scan_nav", "/odom", "/tf", "/tf_static", "/amcl_pose", "/plan", "/cmd_vel", "/cmd_vel_nav",
    "/cmd_vel_guarded", "/cmd_vel_teleop", "/car/wheels/cmd", "/car/wheels/applied", "/safety_guard/state",
    "/depth_obstacles", "/imu/data_raw", "/rosout", "/teleop_mux/state", "/goal_pose", "/initialpose", "/map",
]
MIN_FREE_MB = 1000          # refuse to start below this, and stop a recording that reaches it
MAX_RUN_S = 3600.0          # one run is stopped after an hour


def dir_size(path: str) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


def free_mb(path: str) -> float:
    return shutil.disk_usage(path).free / 1e6


def disk_blocker(path: str, min_free_mb: float = MIN_FREE_MB) -> str | None:
    """A reason not to record (too little free space), or None."""
    free = free_mb(path)
    if free < min_free_mb:
        return f"only {free:.0f} MB free on the data disk (need {min_free_mb:.0f} MB): delete old runs first"
    return None


def clean_topics(topics) -> list[str]:
    out = []
    for t in topics or []:
        t = str(t).strip()
        if t.startswith("/") and all(c.isalnum() or c in "/_" for c in t) and t not in out:
            out.append(t)
    return out


class RunStore:
    def __init__(self, data_dir: str) -> None:
        self.root = os.path.join(data_dir, "runs")
        os.makedirs(self.root, exist_ok=True)

    def _dir(self, rid: str) -> str:
        return os.path.join(self.root, safe_name(rid))

    def new(self, name: str, note: str = "", topics=None, storage: str = "mcap") -> dict:
        name = safe_name(name or "run")
        rid = time.strftime("%Y%m%d-%H%M%S") + "_" + name.replace(" ", "_")
        path = self._dir(rid)
        os.makedirs(path)
        meta = {"id": rid, "name": name, "note": str(note)[:2000], "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                "started_ts": time.time(), "topics": clean_topics(topics) or list(DEFAULT_TOPICS),
                "storage": storage, "state": "recording", "stop_reason": "", "tests": []}
        self._write(rid, meta)
        return meta

    def _write(self, rid: str, meta: dict) -> None:
        p = os.path.join(self._dir(rid), "meta.json")
        tmp = p + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(meta, fh, indent=1)
        os.replace(tmp, p)

    def get(self, rid: str) -> dict | None:
        try:
            with open(os.path.join(self._dir(rid), "meta.json"), encoding="utf-8") as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return None

    def update(self, rid: str, **fields) -> dict | None:
        m = self.get(rid)
        if m is None:
            return None
        for k, v in fields.items():
            m[k] = str(v)[:2000] if k in ("note", "stop_reason") else v
        self._write(rid, m)
        return m

    def link_test(self, rid: str, test_id: str) -> None:
        m = self.get(rid)
        if m is not None and test_id not in m.get("tests", []):
            m.setdefault("tests", []).append(test_id)
            self._write(rid, m)

    def finish(self, rid: str, reason: str, duration_s: float | None = None) -> dict | None:
        m = self.get(rid)
        if m is None:
            return None
        m["state"] = "saved"
        m["stop_reason"] = reason
        m["duration_s"] = round(duration_s if duration_s is not None else time.time() - m.get("started_ts", time.time()), 1)
        self._write(rid, m)
        return m

    def list(self) -> list[dict]:
        out = []
        for rid in sorted(os.listdir(self.root), reverse=True):
            m = self.get(rid)
            if m is None:
                continue
            m = dict(m)
            m["size_mb"] = round(dir_size(self._dir(rid)) / 1e6, 2)
            out.append(m)
        return out

    def delete(self, rid: str) -> bool:
        path = self._dir(rid)
        if not os.path.isdir(path):
            return False
        shutil.rmtree(path, ignore_errors=True)
        return not os.path.isdir(path)

    def zip_to(self, rid: str, fileobj) -> bool:
        """Write the run folder as a zip (stored, the bag is already compressed) to a file-like object."""
        path = self._dir(rid)
        if not os.path.isdir(path):
            return False
        with zipfile.ZipFile(fileobj, "w", zipfile.ZIP_STORED, allowZip64=True) as z:
            for root, _dirs, files in os.walk(path):
                for f in sorted(files):
                    full = os.path.join(root, f)
                    z.write(full, os.path.join(rid, os.path.relpath(full, path)))
        return True

    def mark_unfinished(self) -> None:
        """At start-up: a run still marked 'recording' lost its recorder (crash, power cut)."""
        for rid in os.listdir(self.root):
            m = self.get(rid)
            if m and m.get("state") == "recording":
                self.update(rid, state="interrupted", stop_reason="the recorder did not stop cleanly "
                                                                   "(restart / power loss); the bag may need `ros2 bag reindex`")
