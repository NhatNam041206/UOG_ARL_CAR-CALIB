"""Small map view drawn next to the overlay in the recorded route video.

The static part (runnable cells, planned path, anchors) is rasterised once per
start; each frame only crops a window around the car and draws the trail and
the car on top, so the cost stays well under a millisecond.
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from positioning.tile_position import TileMap

BG = (24, 20, 16)
CELL = (90, 84, 74)
PATH = (225, 125, 47)
ANCHOR = (216, 66, 143)
TRAIL = (0, 138, 255)
CAR_OK = (74, 222, 128)
CAR_BAD = (60, 60, 230)
TEXT = (236, 236, 236)


class MapPanel:
    def __init__(self, tile_map: TileMap, path: list[list[int]] | None, cell_px: int = 12) -> None:
        self.px = int(cell_px)
        rows = [r for r, _ in tile_map.cells]
        cols = [c for _, c in tile_map.cells]
        self.min_r, self.min_c = min(rows), min(cols)
        h = (max(rows) - self.min_r + 1) * self.px
        w = (max(cols) - self.min_c + 1) * self.px
        base = np.full((h, w, 3), BG, np.uint8)
        for r, c in tile_map.cells:
            y, x = self._xy(r, c)
            base[y + 1:y + self.px, x + 1:x + self.px] = CELL
        for r, c in path or []:
            y, x = self._xy(r, c)
            base[y + 1:y + self.px, x + 1:x + self.px] = PATH
        for a in tile_map.anchors.values():
            y, x = self._xy(*a.cell)
            cv2.circle(base, (x + self.px // 2, y + self.px // 2), max(3, self.px // 3), ANCHOR, -1)
        self.base = base

    def _xy(self, r: int, c: int) -> tuple[int, int]:
        return (r - self.min_r) * self.px, (c - self.min_c) * self.px

    def render(self, height: int, width: int, state: dict[str, Any]) -> np.ndarray:
        out = np.full((height, width, 3), BG, np.uint8)
        pos = state.get("position")
        if pos is not None:
            cy, cx = self._xy(pos["row"], pos["column"])
            cy += self.px // 2
            cx += self.px // 2
        else:
            cy, cx = self.base.shape[0] // 2, self.base.shape[1] // 2
        top = cy - height // 2
        left = cx - width // 2
        # copy the visible part of the static raster
        sy0, sx0 = max(0, top), max(0, left)
        sy1, sx1 = min(self.base.shape[0], top + height), min(self.base.shape[1], left + width)
        if sy1 > sy0 and sx1 > sx0:
            out[sy0 - top:sy1 - top, sx0 - left:sx1 - left] = self.base[sy0:sy1, sx0:sx1]
        half = self.px // 2
        for r, c in state.get("trail") or []:
            y, x = self._xy(r, c)
            y, x = y - top + half, x - left + half
            if 0 <= y < height and 0 <= x < width:
                cv2.circle(out, (x, y), max(2, self.px // 4), TRAIL, -1)
        if pos is not None:
            color = CAR_OK if pos.get("status") == "OK" else CAR_BAD
            cv2.circle(out, (cx - left, cy - top), max(5, int(self.px * 0.6)), color, -1)
            cv2.circle(out, (cx - left, cy - top), max(5, int(self.px * 0.6)), (0, 0, 0), 1)
        cv2.rectangle(out, (0, 0), (width - 1, 26), (40, 34, 28), -1)
        if pos is not None:
            label = f"MAP ({pos['row']},{pos['column']}) {pos.get('status', '')}  tiles {state.get('tiles', 0)}"
        else:
            label = "MAP not set"
        cv2.putText(out, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT, 1, cv2.LINE_AA)
        return out
