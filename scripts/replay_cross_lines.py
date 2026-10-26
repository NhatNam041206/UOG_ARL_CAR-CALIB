#!/usr/bin/env python3
"""Replay route_raw.avi through CrossLineCounter and report the tile count.

Examples:
    python scripts/replay_cross_lines.py routes/route-20261024T115526Z-218a9f1c
    python scripts/replay_cross_lines.py routes/route-* --video --out /tmp/cross

Writes per route into --out/<route_id>/: cross_lines.csv (one row per frame),
and with --video cross_lines.mp4 (overlay: look-ahead ROI, counter band,
detected lines, confirmed tracks, running count; dark-line mask on the right).
Prints one summary line per route.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision.cross_line_counter import CrossLineConfig, CrossLineCounter  # noqa: E402


def _find_video(route: Path) -> Path | None:
    for candidate in (route / "route_raw.avi", *route.glob("*/route_raw.avi")):
        if candidate.is_file():
            return candidate
    return None


def replay(route: Path, out_root: Path, config: CrossLineConfig, write_video: bool) -> dict:
    video = _find_video(route)
    if video is None:
        return {"route": route.name, "error": "no route_raw.avi"}
    route_id = video.parent.name
    out_dir = out_root / route_id
    out_dir.mkdir(parents=True, exist_ok=True)
    counter = CrossLineCounter(config)
    cap = cv2.VideoCapture(str(video))
    writer = None
    frames = 0
    count_frames: list[int] = []
    with (out_dir / "cross_lines.csv").open("w", newline="", encoding="utf-8") as fh:
        rows = csv.writer(fh)
        rows.writerow(["frame", "tile_count", "counted", "lines", "confirmed_tracks", "line_y", "line_tilt"])
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            result = counter.update(frame)
            if result.counted_this_frame:
                count_frames.append(frames)
            rows.writerow([
                frames,
                result.tile_count,
                int(result.counted_this_frame),
                len(result.lines),
                sum(1 for t in result.tracks if t["confirmed"]),
                ";".join(f"{l.y:.3f}" for l in result.lines),
                ";".join(f"{l.tilt_deg:.1f}" for l in result.lines),
            ])
            if write_video:
                vis = counter.draw(frame.copy(), result)
                cv2.putText(vis, f"{route_id[-8:]} f{frames}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                mask = np.zeros_like(frame)
                dark = counter._last_dark
                if dark is not None:
                    top = int(config.roi_top * frame.shape[0])
                    mask[top:top + dark.shape[0], :, :] = cv2.cvtColor(dark, cv2.COLOR_GRAY2BGR)
                vis = np.hstack([vis, mask])
                if writer is None:
                    writer = cv2.VideoWriter(
                        str(out_dir / "cross_lines.mp4"),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        10.0,
                        (vis.shape[1], vis.shape[0]),
                    )
                writer.write(vis)
            frames += 1
    cap.release()
    if writer is not None:
        writer.release()
    summary = {
        "route": route_id,
        "frames": frames,
        "tile_count": counter.tile_count,
        "count_frames": count_frames,
        "config": config.as_dict(),
    }
    (out_dir / "cross_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("routes", nargs="+", type=Path, help="route folders (containing route_raw.avi)")
    parser.add_argument("--out", type=Path, default=Path("cross_replay"), help="output folder")
    parser.add_argument("--video", action="store_true", help="also write an overlay video per route")
    parser.add_argument(
        "--set", action="append", default=[], metavar="KEY=VALUE",
        help="override a CrossLineConfig field, e.g. --set roi_top=0.3 (repeatable)",
    )
    args = parser.parse_args()
    config = CrossLineConfig.from_env()
    for item in args.set:
        key, _, value = item.partition("=")
        if not hasattr(config, key):
            parser.error(f"unknown config field: {key}")
        current = getattr(config, key)
        setattr(config, key, type(current)(value) if not isinstance(current, bool) else value.lower() in ("1", "true", "yes"))
    for route in args.routes:
        s = replay(route, args.out, config, args.video)
        if "error" in s:
            print(f"{s['route']}: {s['error']}")
        else:
            print(f"{s['route']}: frames={s['frames']} tiles={s['tile_count']} at frames {s['count_frames']}")


if __name__ == "__main__":
    main()
