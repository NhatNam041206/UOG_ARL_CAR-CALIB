#!/usr/bin/env python3
"""Estimate the camera focal length / HFOV from square floor tiles.

Lane (longitudinal) grout lines and transverse grout lines are orthogonal on
the floor, so their two vanishing points v1, v2 satisfy
(v1 - c) . (v2 - c) = -f^2 for principal point c (assumed at the image
centre, no lens distortion). Each usable frame gives one f; the median is
reported together with the HFOV to enter as CAMERA_HFOV_DEG.

Usage:
    python scripts/estimate_camera_hfov.py ROUTE_DIR/route_raw.avi [...] --env-file .env.pi5
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _transverse_vp(lines, geometry, line_length):
    """VP of the two longest transverse lines that are far apart vertically."""
    transverse = [
        line for line in lines
        if line[2] != line[0]
        and abs((line[3] - line[1]) / (line[2] - line[0])) < 0.25
        and line_length(line) > 120
    ]
    best = None
    for i, a in enumerate(transverse):
        for b in transverse[i + 1:]:
            if abs((a[1] + a[3]) / 2 - (b[1] + b[3]) / 2) < 80:
                continue
            point = geometry.calculate_vanishing_point(a, b)
            if point is None:
                continue
            support = line_length(a) + line_length(b)
            if best is None or support > best[0]:
                best = (support, point)
    return None if best is None else best[1]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("videos", nargs="+", type=Path, help="raw videos (route_raw.avi), no HUD overlay")
    parser.add_argument("--env-file", type=Path, help="env file loaded before the pipeline, e.g. .env.pi5")
    parser.add_argument("--step", type=int, default=3, help="use every Nth frame")
    parser.add_argument("--canny", type=int, nargs=2, default=(30, 90), metavar=("LOW", "HIGH"))
    parser.add_argument("--hough-threshold", type=int, default=40)
    parser.add_argument("--vp-horizon-y", type=float, default=-350.0)
    args = parser.parse_args(argv)
    if args.env_file is not None:
        from dotenv import load_dotenv

        load_dotenv(args.env_file, override=True)
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))

    import cv2
    import numpy as np

    from unified_calibration_components import GeometryCalculator, VisionProcessor, _line_length

    # Full-frame ROI: transverse tile lines in the lower half are needed too.
    vision = VisionProcessor(1.0)
    vision._canny_low, vision._canny_high = args.canny
    vision._hough_threshold = args.hough_threshold
    vision._vp_horizon_y = args.vp_horizon_y
    focals: list[float] = []
    for video in args.videos:
        cap = cv2.VideoCapture(str(video))
        index = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            index += 1
            if index % max(1, args.step):
                continue
            h, w = frame.shape[:2]
            lines, _ = vision.process_frame_debug(frame)
            pair = vision._apply_geometric_filter(lines, w, h)
            if pair is None:
                continue
            v1 = GeometryCalculator.calculate_vanishing_point(*pair)
            v2 = _transverse_vp(lines, GeometryCalculator, _line_length)
            if v1 is None or v2 is None:
                continue
            cx, cy = w / 2.0, h / 2.0
            f2 = -((v1[0] - cx) * (v2[0] - cx) + (v1[1] - cy) * (v2[1] - cy))
            if f2 > 0:
                focals.append(math.sqrt(f2))
        cap.release()

    if not focals:
        print("no usable frames (need both lane lines and two transverse tile lines)")
        return 1
    values = np.array(focals)
    focal = float(np.median(values))
    half_width = w / 2.0
    print(f"frames used : {len(values)}")
    print(f"focal px    : median {focal:.0f}  (p25 {np.percentile(values, 25):.0f}, p75 {np.percentile(values, 75):.0f})")
    print(f"HFOV deg    : {math.degrees(2 * math.atan(half_width / focal)):.1f}  -> CAMERA_HFOV_DEG")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
