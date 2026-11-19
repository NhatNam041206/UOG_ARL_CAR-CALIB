"""Car profiles and car.yaml writing for the web UI's Car tab (no ROS imports).

    <data_dir>/cars/<name>.json   {"name", "note", "props": {...}, "updated"}
    <data_dir>/cars/_applied      name of the profile applied last (informational)

`props` is a flat dict of the values the Car tab edits (CAR_FIELDS). They map onto car.yaml:

    x_front, x_rear, half_width, height_m      footprint.*   (body, centred on base_link)
    track_width_m, cruise_speed_mps,
    max_wheel_speed_mps                        drive.*
    laser_x/y/z/roll/pitch/yaw                 laser.*       (m, rad)
    camera_x/y/z/roll/yaw, camera_pitch_deg    camera.*

`ActiveCar` reads and writes the active car.yaml: the writable copy (CAR_NAV_CONFIG_OVERRIDE,
on the car /data/nav/ui/car.yaml) shadows the shipped file; a write patches the VALUES of the
source text in place (comments stay), writes atomically and keeps a timestamped backup of the
previous file next to it (`car.yaml.20261119-101500.bak`, the 10 newest are kept).
"""
from __future__ import annotations

import json
import math
import os
import re
import shutil
import time
from dataclasses import dataclass

import yaml

from .ui_store import safe_name

BACKUPS_KEPT = 10
VFOV_DEG = 45.0     # the Astra Pro's vertical field of view (depth_obstacles.blind_distance, README)


@dataclass(frozen=True)
class Field:
    section: str
    key: str
    lo: float
    hi: float
    nullable: bool = False
    default: float | None = 0.0
    label: str = ""
    unit: str = "m"


CAR_FIELDS: dict[str, Field] = {
    "x_front": Field("footprint", "x_front", 0.05, 2.0, default=0.35, label="Front edge from centre"),
    "x_rear": Field("footprint", "x_rear", -2.0, -0.05, default=-0.35, label="Rear edge from centre"),
    "half_width": Field("footprint", "half_width", 0.05, 1.5, default=0.30, label="Half width"),
    "height_m": Field("footprint", "height_m", 0.05, 3.0, nullable=True, default=None, label="Body height"),
    "track_width_m": Field("drive", "track_width_m", 0.10, 2.0, nullable=True, default=None, label="Track width"),
    "laser_x": Field("laser", "x", -2.0, 2.0, label="LiDAR x"),
    "laser_y": Field("laser", "y", -1.5, 1.5, label="LiDAR y"),
    "laser_z": Field("laser", "z", 0.0, 3.0, default=0.24, label="LiDAR height"),
    "laser_roll": Field("laser", "roll", -3.2, 3.2, label="LiDAR roll", unit="rad"),
    "laser_pitch": Field("laser", "pitch", -3.2, 3.2, label="LiDAR pitch", unit="rad"),
    "laser_yaw": Field("laser", "yaw", -3.2, 3.2, nullable=True, label="LiDAR yaw", unit="rad"),
    "camera_x": Field("camera", "x", -2.0, 2.0, label="Camera x"),
    "camera_y": Field("camera", "y", -1.5, 1.5, label="Camera y"),
    "camera_z": Field("camera", "z", 0.0, 3.0, default=1.2, label="Camera height"),
    "camera_roll": Field("camera", "roll", -3.2, 3.2, label="Camera roll", unit="rad"),
    "camera_pitch_deg": Field("camera", "pitch_deg", -90.0, 90.0, default=15.0, label="Camera pitch down", unit="deg"),
    "camera_yaw": Field("camera", "yaw", -3.2, 3.2, label="Camera yaw", unit="rad"),
    "cruise_speed_mps": Field("drive", "cruise_speed_mps", 0.05, 2.0, default=0.4, label="Cruise speed", unit="m/s"),
    "max_wheel_speed_mps": Field("drive", "max_wheel_speed_mps", 0.05, 3.0, default=0.6, label="Max wheel speed",
                                 unit="m/s"),
}


def props_from_cfg(cfg: dict) -> dict:
    """The editable values of a loaded car.yaml (null / missing -> None or the field's default)."""
    out = {}
    for name, f in CAR_FIELDS.items():
        v = (cfg.get(f.section) or {}).get(f.key)
        out[name] = float(v) if v is not None else f.default
    return out


def validate_props(props: dict, partial: bool = False) -> dict:
    """Clean and range-check props. Unknown keys are rejected; missing keys are an error unless partial."""
    if not isinstance(props, dict):
        raise ValueError("props must be an object")
    unknown = sorted(set(props) - set(CAR_FIELDS))
    if unknown:
        raise ValueError(f"unknown car properties: {', '.join(unknown)}")
    out = {}
    for name, f in CAR_FIELDS.items():
        if name not in props:
            if partial:
                continue
            raise ValueError(f"missing {name}")
        v = props[name]
        if v is None or v == "":
            if not f.nullable:
                raise ValueError(f"{f.label or name} is needed")
            out[name] = None
            continue
        try:
            v = float(v)
        except (TypeError, ValueError):
            raise ValueError(f"{f.label or name}: not a number") from None
        if not math.isfinite(v) or not f.lo <= v <= f.hi:
            raise ValueError(f"{f.label or name}: {v} is outside {f.lo} .. {f.hi} {f.unit}")
        out[name] = round(v, 4)
    if not partial:
        if out["x_front"] <= out["x_rear"]:
            raise ValueError("front edge must be ahead of the rear edge")
        if out["cruise_speed_mps"] > out["max_wheel_speed_mps"]:
            raise ValueError("cruise speed cannot be above the max wheel speed")
    return out


def derived(props: dict, guard: dict | None = None) -> dict:
    """Numbers the Car tab shows next to the drawings (uses depth_obstacles.blind_distance)."""
    from .depth_obstacles import blind_distance
    from .geometry import Pose3D

    p = {**props_from_cfg({}), **{k: v for k, v in props.items() if v is not None}}
    cam = Pose3D(p["camera_x"], p["camera_y"], p["camera_z"], p["camera_roll"],
                 math.radians(p["camera_pitch_deg"]), p["camera_yaw"])
    down_deg = p["camera_pitch_deg"] + VFOV_DEG / 2.0
    blind = blind_distance(cam, VFOV_DEG)
    top_down = p["camera_pitch_deg"] - VFOV_DEG / 2.0     # the highest ray: positive = still looking down
    return {
        "length_m": round(p["x_front"] - p["x_rear"], 3),
        "width_m": round(2 * p["half_width"], 3),
        "spin_radius_m": round(math.hypot(max(p["x_front"], -p["x_rear"]), p["half_width"]), 3),
        "camera_blind_m": round(blind, 3),
        "camera_lowest_ray_deg": round(down_deg, 1),
        "camera_highest_ray_deg": round(top_down, 1),
        "camera_floor_visible": down_deg > 0.06,
        "vfov_deg": VFOV_DEG,
        "lidar_plane_m": p["laser_z"],
        "lidar_upside_down": abs(p["laser_roll"]) > 1.0 or abs(p["laser_pitch"]) > 1.0,
        "guard": guard or {},
    }


def apply_blocker(moving: bool, goal_active: bool, switching: bool, mode: str, map_in_progress: bool,
                  writable: bool, force: bool = False) -> str | None:
    """Why "Apply to the car" must be refused (the web UI answers HTTP 409 with this text), or None.
    Applying restarts the nav stack, so the car must be stopped and no goal / mission active."""
    if switching:
        return "the nav stack is restarting: wait until it is ready"
    if moving:
        return "the car is being driven (wheel command not 0): stop it first"
    if goal_active:
        return "a goal / mission is active: cancel it first"
    if mode == "create" and map_in_progress and not force:
        return "Create mode would lose the map being built: save it first (or force)"
    if not writable:
        return "car.yaml is read-only here (CAR_NAV_CONFIG_OVERRIDE is not set)"
    return None


# ----------------------------------------------------------------------- yaml text patch
_SECTION = re.compile(r"^([A-Za-z_]\w*):\s*(#.*)?$")
_KEY = re.compile(r"^(?P<ind>[ \t]+)(?P<key>[A-Za-z_]\w*):(?P<sp>[ \t]*)(?P<val>[^#\n]*?)(?P<tail>[ \t]*)(?P<cmt>#.*)?$")


def _fmt(v) -> str:
    if v is None:
        return "null"
    if isinstance(v, bool):
        return "true" if v else "false"
    return repr(round(float(v), 6))


def patch_yaml_text(text: str, updates: dict[tuple[str, str], object]) -> str:
    """Set section.key values in the text of a car.yaml, keeping comments and layout.
    A missing key is added at the end of its section, a missing section at the end of the file."""
    lines = text.split("\n")
    pending = dict(updates)
    section = None
    last_key_line: dict[str, int] = {}
    indent_of: dict[str, str] = {}
    for i, line in enumerate(lines):
        m = _SECTION.match(line)
        if m:
            section = m.group(1)
            continue
        if line and not line[0].isspace() and not line.startswith("#"):
            section = None            # some other top-level construct
            continue
        km = _KEY.match(line)
        if section is None or km is None:
            continue
        last_key_line[section] = i
        indent_of.setdefault(section, km.group("ind"))
        k = (section, km.group("key"))
        if k in pending and km.group("ind") == indent_of[section]:
            new = _fmt(pending.pop(k))
            old_w = len(km.group("val")) + len(km.group("tail"))
            if km.group("cmt"):
                pad = " " * max(2, old_w - len(new))
                lines[i] = f"{km.group('ind')}{km.group('key')}:{km.group('sp') or ' '}{new}{pad}{km.group('cmt')}"
            else:
                lines[i] = f"{km.group('ind')}{km.group('key')}:{km.group('sp') or ' '}{new}"
    # keys / sections that were not there: insert from the bottom up so the indexes stay valid
    missing: dict[str, list[tuple[str, object]]] = {}
    for (sec, key), v in pending.items():
        missing.setdefault(sec, []).append((key, v))
    for sec, items in sorted(missing.items(), key=lambda kv: -last_key_line.get(kv[0], 10 ** 9)):
        if sec in last_key_line:
            at = last_key_line[sec] + 1
            ind = indent_of[sec]
            lines[at:at] = [f"{ind}{k}: {_fmt(v)}" for k, v in items]
        else:
            while lines and lines[-1] == "":
                lines.pop()
            lines += ["", f"{sec}:"] + [f"  {k}: {_fmt(v)}" for k, v in items] + [""]
    return "\n".join(lines)


def props_to_updates(props: dict) -> dict[tuple[str, str], object]:
    return {(CAR_FIELDS[k].section, CAR_FIELDS[k].key): v for k, v in props.items() if k in CAR_FIELDS}


def patched_text(text: str, props: dict) -> str:
    """car.yaml text with `props` applied; checked by parsing it back."""
    new = patch_yaml_text(text, props_to_updates(props))
    cfg = yaml.safe_load(new) or {}
    for k, v in props.items():
        f = CAR_FIELDS[k]
        got = (cfg.get(f.section) or {}).get(f.key)
        if (v is None and got is not None) or (v is not None and (got is None or abs(float(got) - float(v)) > 1e-9)):
            raise ValueError(f"car.yaml could not be updated at {f.section}.{f.key} (got {got!r}, wanted {v!r})")
    return new


class ActiveCar:
    """The active car.yaml: the writable `override` if it exists, else the `shipped` file."""

    def __init__(self, shipped: str, override: str = "", keep: int = BACKUPS_KEPT) -> None:
        self.shipped = shipped
        self.override = override
        self.keep = keep

    @property
    def writable(self) -> bool:
        return bool(self.override)

    def path(self) -> str:
        return self.override if self.override and os.path.isfile(self.override) else self.shipped

    def is_override(self) -> bool:
        return bool(self.override) and os.path.isfile(self.override)

    def load(self) -> dict:
        with open(self.path(), "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}

    def read_props(self) -> dict:
        return props_from_cfg(self.load())

    def backups(self) -> list[str]:
        if not self.override:
            return []
        d, base = os.path.split(self.override)
        try:
            names = sorted(n for n in os.listdir(d or ".") if n.startswith(base + ".") and n.endswith(".bak"))
        except OSError:
            return []
        return [os.path.join(d, n) for n in names]

    def _backup(self, now: float | None = None) -> str:
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
        dst = f"{self.override}.{stamp}.bak"
        n = 1
        while os.path.exists(dst):
            n += 1
            dst = f"{self.override}.{stamp}-{n}.bak"
        shutil.copyfile(self.path(), dst)
        for old in self.backups()[:-self.keep]:
            try:
                os.remove(old)
            except OSError:
                pass
        return dst

    def write_props(self, props: dict, now: float | None = None) -> dict:
        """Patch the active file's text with props (validated), back up the previous file, write the
        override atomically. Returns {"path", "backup"}."""
        if not self.writable:
            raise RuntimeError("car.yaml is read-only here: CAR_NAV_CONFIG_OVERRIDE is not set")
        props = validate_props(props)
        with open(self.path(), "r", encoding="utf-8") as fh:
            text = fh.read()
        new = patched_text(text, props)
        os.makedirs(os.path.dirname(self.override) or ".", exist_ok=True)
        backup = self._backup(now)
        tmp = self.override + ".tmp"
        with open(tmp, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(new)
        os.replace(tmp, self.override)
        return {"path": self.override, "backup": backup}

    def write_updates(self, updates: dict[tuple[str, str], object], now: float | None = None) -> dict:
        """Like write_props for values outside the Car tab's form (e.g. ("ugv", "units_per_mps")): patch the
        text of the active file, check it by parsing it back, back up, write the override atomically."""
        if not self.writable:
            raise RuntimeError("car.yaml is read-only here: CAR_NAV_CONFIG_OVERRIDE is not set")
        with open(self.path(), "r", encoding="utf-8") as fh:
            text = fh.read()
        new = patch_yaml_text(text, updates)
        cfg = yaml.safe_load(new) or {}
        for (sec, key), v in updates.items():
            got = (cfg.get(sec) or {}).get(key)
            if got is None or abs(float(got) - float(v)) > 1e-9:
                raise ValueError(f"car.yaml could not be updated at {sec}.{key} (got {got!r}, wanted {v!r})")
        os.makedirs(os.path.dirname(self.override) or ".", exist_ok=True)
        backup = self._backup(now)
        tmp = self.override + ".tmp"
        with open(tmp, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(new)
        os.replace(tmp, self.override)
        return {"path": self.override, "backup": backup}

    def reset(self, now: float | None = None) -> str | None:
        """Remove the override (the shipped file is used again); returns the backup made, or None."""
        if not self.is_override():
            return None
        backup = self._backup(now)
        os.remove(self.override)
        return backup


class CarProfileStore:
    """Named sets of car properties: list / get / put (save as) / rename / delete."""

    def __init__(self, data_dir: str) -> None:
        self.dir = os.path.join(data_dir, "cars")
        os.makedirs(self.dir, exist_ok=True)

    def _path(self, name: str) -> str:
        return os.path.join(self.dir, safe_name(name) + ".json")

    def list(self) -> list[dict]:
        out = []
        applied = self.applied()
        for f in sorted(os.listdir(self.dir)):
            if f.endswith(".json"):
                try:
                    with open(os.path.join(self.dir, f), encoding="utf-8") as fh:
                        d = json.load(fh)
                except (OSError, ValueError):
                    continue
                n = d.get("name", f[:-5])
                out.append({"name": n, "note": d.get("note", ""), "updated": d.get("updated"),
                            "applied": n == applied})
        return out

    def get(self, name: str) -> dict | None:
        try:
            with open(self._path(name), encoding="utf-8") as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return None

    def exists(self, name: str) -> bool:
        return os.path.isfile(self._path(name))

    def put(self, name: str, props: dict, note: str = "", overwrite: bool = True) -> dict:
        name = safe_name(name)
        if not overwrite and self.exists(name):
            raise FileExistsError(f"a car profile named {name!r} exists")
        d = {"name": name, "note": str(note)[:500], "props": validate_props(props),
             "updated": time.strftime("%Y-%m-%d %H:%M:%S")}
        tmp = self._path(name) + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(d, fh, indent=1)
        os.replace(tmp, self._path(name))
        return d

    def rename(self, old: str, new: str) -> dict:
        d = self.get(old)
        if d is None:
            raise LookupError(f"no such car profile {old!r}")
        new = safe_name(new)
        if new != safe_name(old) and self.exists(new):
            raise FileExistsError(f"a car profile named {new!r} exists")
        d["name"] = new
        self.put(new, d["props"], d.get("note", ""))
        if new != safe_name(old):
            was_applied = self.applied() == safe_name(old)
            self.delete(old)
            if was_applied:
                self.set_applied(new)
        return self.get(new)

    def delete(self, name: str) -> bool:
        try:
            os.remove(self._path(name))
        except OSError:
            return False
        if self.applied() == safe_name(name):
            self.set_applied(None)
        return True

    def applied(self) -> str | None:
        try:
            with open(os.path.join(self.dir, "_applied"), encoding="utf-8") as fh:
                return fh.read().strip() or None
        except OSError:
            return None

    def set_applied(self, name: str | None) -> None:
        with open(os.path.join(self.dir, "_applied"), "w", encoding="utf-8") as fh:
            fh.write("" if name is None else safe_name(name))
