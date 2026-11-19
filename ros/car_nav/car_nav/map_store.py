"""Stored maps for the web UI's Load / Create modes (no ROS imports).

    <data_dir>/maps/<name>.pgm    occupancy image (nav2_map_server format)
    <data_dir>/maps/<name>.yaml   nav2_map_server map file ("image: <name>.pgm", resolution, origin, ...)
    <data_dir>/maps/<name>.json   {"name", "note", "start_pose": {"x","y","yaw"} | null, "created",
                                   "updated", "source", "width", "height", "resolution"}
    <data_dir>/maps/_seeded       names the simulator seeded once (so a deleted seed stays deleted)

On the car data_dir is /data/nav/ui (maps in /data/nav/ui/maps); in the simulator
/data/sim/ui. A map that has a .yaml and a .pgm is listed even without the .json
(a map saved with map_saver_cli, copied in by hand): it just has no note / start pose.
"""
from __future__ import annotations

import json
import math
import os
import re
import shutil
import time

import numpy as np
import yaml

from .ui_store import _write

_NAME = re.compile(r"[^A-Za-z0-9_.\-]+")
RESERVED = {"save", "_seeded"}
UNKNOWN_PIXEL = 205     # nav2 map_saver's "unknown" grey
FREE_PIXEL = 254
OCC_PIXEL = 0


def safe_map_name(name: str) -> str:
    """A file-safe map name: letters, digits, _ . -  (spaces become _), max 60 chars."""
    n = _NAME.sub("", str(name).strip().replace(" ", "_")).strip(".")
    if not n:
        raise ValueError("empty map name")
    n = n[:60]
    if n.lower() in RESERVED:
        raise ValueError(f"{n!r} is a reserved name")
    return n


def parse_start_pose(d) -> dict | None:
    """{"x","y","yaw"} (floats, yaw in rad) from a request body, or None to clear it."""
    if d is None or d == {}:
        return None
    try:
        x, y, yaw = float(d["x"]), float(d["y"]), float(d.get("yaw", 0.0))
    except (KeyError, TypeError, ValueError):
        raise ValueError("start_pose needs numbers x, y, yaw") from None
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError("start_pose must be finite")
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4)}


def grid_to_image(data: np.ndarray) -> np.ndarray:
    """Occupancy values (H x W, row 0 = lowest y, -1 unknown, 0..100) -> pgm pixels (row 0 = top)."""
    d = np.asarray(data, dtype=np.int16)
    img = np.full(d.shape, UNKNOWN_PIXEL, dtype=np.uint8)
    img[(d >= 0) & (d < 50)] = FREE_PIXEL
    img[d >= 50] = OCC_PIXEL
    return img[::-1, :]


def write_pgm(path: str, img: np.ndarray) -> None:
    h, w = img.shape
    tmp = path + ".tmp"
    with open(tmp, "wb") as fh:
        fh.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        fh.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())
    os.replace(tmp, path)


def read_pgm(path: str) -> np.ndarray:
    with open(path, "rb") as fh:
        raw = fh.read()
    toks, pos = [], 0
    while len(toks) < 4:                 # magic, width, height, maxval (comments allowed)
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b"#":
            pos = raw.index(b"\n", pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        toks.append(raw[pos:end])
        pos = end
    pos += 1
    if toks[0] != b"P5":
        raise ValueError("not a binary pgm")
    w, h = int(toks[1]), int(toks[2])
    return np.frombuffer(raw[pos:pos + w * h], dtype=np.uint8).reshape(h, w)


def _now() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


class MapStore:
    def __init__(self, data_dir: str) -> None:
        self.dir = os.path.join(data_dir, "maps")
        os.makedirs(self.dir, exist_ok=True)

    # ---------------------------------------------------------------- paths
    def _p(self, name: str, ext: str) -> str:
        return os.path.join(self.dir, safe_map_name(name) + ext)

    def yaml_path(self, name: str) -> str | None:
        """Absolute path of the map's yaml (for map_server), or None if there is no such map."""
        p = self._p(name, ".yaml")
        return p if os.path.isfile(p) and os.path.isfile(self._p(name, ".pgm")) else None

    def exists(self, name: str) -> bool:
        try:
            return self.yaml_path(name) is not None
        except ValueError:
            return False

    # ----------------------------------------------------------------- read
    def get(self, name: str) -> dict | None:
        try:
            ypath = self.yaml_path(name)
        except ValueError:
            return None
        if ypath is None:
            return None
        name = safe_map_name(name)
        meta: dict = {}
        try:
            with open(self._p(name, ".json"), encoding="utf-8") as fh:
                meta = json.load(fh)
        except (OSError, ValueError):
            pass
        info = {}
        try:
            with open(ypath, encoding="utf-8") as fh:
                info = yaml.safe_load(fh) or {}
        except (OSError, yaml.YAMLError):
            pass
        stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(os.path.getmtime(ypath)))
        return {"name": name, "note": meta.get("note", ""), "start_pose": meta.get("start_pose"),
                "created": meta.get("created", stamp), "updated": meta.get("updated", stamp),
                "source": meta.get("source", ""), "width": meta.get("width"), "height": meta.get("height"),
                "resolution": info.get("resolution", meta.get("resolution")),
                "origin": info.get("origin")}

    def list(self) -> list[dict]:
        out = []
        for f in sorted(os.listdir(self.dir)):
            if f.endswith(".yaml"):
                d = self.get(f[:-5])
                if d:
                    out.append(d)
        return out

    # ---------------------------------------------------------------- write
    def _write_meta(self, name: str, meta: dict) -> None:
        _write(self._p(name, ".json"), meta)

    def _meta(self, name: str) -> dict:
        d = self.get(name)
        if d is None:
            raise KeyError(name)
        return {k: d.get(k) for k in ("name", "note", "start_pose", "created", "updated", "source",
                                      "width", "height", "resolution")}

    def update(self, name: str, note: str | None = None, start_pose=...) -> dict | None:
        """Change the note and / or the default start pose (None clears it; omit to keep)."""
        if not self.exists(name):
            return None
        name = safe_map_name(name)
        meta = self._meta(name)
        if note is not None:
            meta["note"] = str(note)[:500]
        if start_pose is not ...:
            meta["start_pose"] = parse_start_pose(start_pose)
        meta["updated"] = _now()
        self._write_meta(name, meta)
        return self.get(name)

    def rename(self, old: str, new: str) -> dict:
        old, new = safe_map_name(old), safe_map_name(new)
        if not self.exists(old):
            raise KeyError(old)
        if old == new:
            return self.get(old)
        if any(os.path.exists(self._p(new, e)) for e in (".yaml", ".pgm", ".json")):
            raise ValueError(f"a map named {new!r} already exists")
        meta = self._meta(old)
        with open(self._p(old, ".yaml"), encoding="utf-8") as fh:
            info = yaml.safe_load(fh) or {}
        info["image"] = new + ".pgm"
        os.replace(self._p(old, ".pgm"), self._p(new, ".pgm"))
        _write_yaml(self._p(new, ".yaml"), info)
        os.remove(self._p(old, ".yaml"))
        try:
            os.remove(self._p(old, ".json"))
        except OSError:
            pass
        meta["name"] = new
        meta["updated"] = _now()
        self._write_meta(new, meta)
        return self.get(new)

    def delete(self, name: str) -> bool:
        if not self.exists(name):
            return False
        for ext in (".yaml", ".pgm", ".json"):
            try:
                os.remove(self._p(name, ext))
            except OSError:
                pass
        return True

    def save_grid(self, name: str, data: np.ndarray, resolution: float, origin: tuple[float, float],
                  note: str = "", start_pose=None, source: str = "slam", overwrite: bool = False) -> dict:
        """Store an occupancy grid (H x W int, row 0 = lowest y) as pgm + yaml + json."""
        name = safe_map_name(name)
        if self.exists(name) and not overwrite:
            raise ValueError(f"a map named {name!r} already exists")
        data = np.asarray(data)
        if data.ndim != 2 or data.size == 0:
            raise ValueError("empty map")
        write_pgm(self._p(name, ".pgm"), grid_to_image(data))
        _write_yaml(self._p(name, ".yaml"), {
            "image": name + ".pgm", "mode": "trinary", "resolution": float(resolution),
            "origin": [float(origin[0]), float(origin[1]), 0.0], "negate": 0,
            "occupied_thresh": 0.65, "free_thresh": 0.196})
        old = {}
        if overwrite:
            try:
                with open(self._p(name, ".json"), encoding="utf-8") as fh:
                    old = json.load(fh)
            except (OSError, ValueError):
                pass
        meta = {"name": name, "note": str(note)[:500] or old.get("note", ""),
                "start_pose": parse_start_pose(start_pose) or old.get("start_pose"),
                "created": old.get("created", _now()), "updated": _now(), "source": source,
                "width": int(data.shape[1]), "height": int(data.shape[0]), "resolution": float(resolution)}
        self._write_meta(name, meta)
        return self.get(name)

    def import_yaml(self, yaml_file: str, name: str, note: str = "", start_pose=None,
                    source: str = "imported", overwrite: bool = False) -> dict:
        """Copy an existing nav2 map (yaml + its image) into the store."""
        name = safe_map_name(name)
        if self.exists(name) and not overwrite:
            raise ValueError(f"a map named {name!r} already exists")
        with open(yaml_file, encoding="utf-8") as fh:
            info = yaml.safe_load(fh) or {}
        src_img = info["image"]
        if not os.path.isabs(src_img):
            src_img = os.path.join(os.path.dirname(yaml_file), src_img)
        shutil.copyfile(src_img, self._p(name, ".pgm"))
        info["image"] = name + ".pgm"
        _write_yaml(self._p(name, ".yaml"), info)
        try:
            h, w = read_pgm(self._p(name, ".pgm")).shape
        except (OSError, ValueError):
            h = w = None
        self._write_meta(name, {"name": name, "note": note, "start_pose": parse_start_pose(start_pose),
                                "created": _now(), "updated": _now(), "source": source,
                                "width": w, "height": h, "resolution": info.get("resolution")})
        return self.get(name)

    # ----------------------------------------------------------------- seed
    def seed(self, yaml_file: str, name: str, start_pose=None, note: str = "") -> bool:
        """Import a map once (the simulator's rasterised world). True if it was added now."""
        name = safe_map_name(name)
        marker = os.path.join(self.dir, "_seeded")
        try:
            with open(marker, encoding="utf-8") as fh:
                seeded = fh.read().split()
        except OSError:
            seeded = []
        if name in seeded:
            return False
        if not self.exists(name):
            self.import_yaml(yaml_file, name, note=note, start_pose=start_pose, source="sim world")
        with open(marker, "w", encoding="utf-8") as fh:
            fh.write("\n".join(seeded + [name]))
        return True


def _write_yaml(path: str, data: dict) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, sort_keys=False)
    os.replace(tmp, path)
