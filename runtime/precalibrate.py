"""Pre-run calibration checklist for the dashboard Pre-calibrate tab.

The car stands still, aligned with the floor tiles. Every frame the main loop
feeds the calibrator telemetry (and a tiny grey thumbnail for brightness);
status() takes medians over the last window and returns pass/fail checks with
a hint for each. capture() writes the measured VP into the live tune values.

Directions in hints follow the image: lane_offset_ratio > 0 means the lane
centre is right of the image centre, so the car must move right.
"""

from __future__ import annotations

import os
import threading
import time
from collections import deque
from statistics import median
from typing import Any

import cv2
import numpy as np


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except ValueError:
        return default


def _num(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if np.isfinite(out) else None


class PreCalibrator:
    def __init__(self) -> None:
        self.window_s = _env_float("PRECAL_WINDOW_S", 1.5)
        self.center_tol = _env_float("PRECAL_CENTER_TOL", 0.05)  # lane offset ratio
        self.vp_x_tol_px = _env_float("PRECAL_VP_X_TOL_PX", 12.0)
        self.vp_row_tol_px = _env_float("PRECAL_VP_ROW_TOL_PX", 40.0)
        self.pair_min_ratio = _env_float("PRECAL_PAIR_MIN_RATIO", 0.9)
        self.brightness_min = _env_float("PRECAL_BRIGHTNESS_MIN", 60.0)
        self.brightness_max = _env_float("PRECAL_BRIGHTNESS_MAX", 210.0)
        self.saturated_max = _env_float("PRECAL_SATURATED_MAX", 0.05)
        self.fps_min = _env_float("PRECAL_FPS_MIN", 10.0)
        self.tile_size_cm = _env_float("PRECAL_TILE_SIZE_CM", 60.0)
        self._lock = threading.Lock()
        self._samples: deque[dict[str, Any]] = deque()
        self._last_poll = 0.0

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def observe(
        self,
        frame: np.ndarray,
        telemetry: dict[str, Any],
        cross_lines: int,
        vp_horizon_y: float | None,
        now: float | None = None,
    ) -> None:
        now = time.monotonic() if now is None else now
        thumb = cv2.resize(frame, (80, 60), interpolation=cv2.INTER_AREA)
        gray = thumb if thumb.ndim == 2 else cv2.cvtColor(thumb, cv2.COLOR_BGR2GRAY)
        sample = {
            "t": now,
            "frame_width": _num(telemetry.get("frame_width")) or float(frame.shape[1]),
            "pair": bool(int(_num(telemetry.get("selected_pair_found")) or 0)),
            "vp_x": _num(telemetry.get("vp_x")),
            "vp_y": _num(telemetry.get("vp_y")),
            "vp_angle": _num(telemetry.get("vp_angle")),
            "vp_row": _num(vp_horizon_y),  # tuned nominal row, not the tracked one
            "offset": _num(telemetry.get("lane_offset_ratio")),
            "lane_width": _num(telemetry.get("lane_width_px")),
            "cross_lines": int(cross_lines),
            "brightness": float(gray.mean()),
            "saturated": float((gray >= 250).mean()),
        }
        with self._lock:
            self._samples.append(sample)
            while self._samples and now - self._samples[0]["t"] > self.window_s:
                self._samples.popleft()

    def overlay_active(self, now: float | None = None) -> bool:
        """True while the Pre-calibrate tab is polling (draw the guides)."""
        now = time.monotonic() if now is None else now
        return now - self._last_poll < 3.0

    def draw(self, frame: np.ndarray) -> np.ndarray:
        """Centre line and the current VP marker (in place)."""
        h, w = frame.shape[:2]
        cx = w // 2
        cv2.line(frame, (cx, 0), (cx, h - 1), (0, 255, 255), 1)
        tol = int(self.vp_x_tol_px)
        cv2.line(frame, (cx - tol, 0), (cx - tol, 12), (0, 255, 255), 1)
        cv2.line(frame, (cx + tol, 0), (cx + tol, 12), (0, 255, 255), 1)
        with self._lock:
            last = self._samples[-1] if self._samples else None
        if last is not None and last["vp_x"] is not None:
            vx = int(max(0, min(w - 1, last["vp_x"])))
            color = (0, 200, 0) if abs(last["vp_x"] - cx) <= self.vp_x_tol_px else (0, 0, 255)
            cv2.line(frame, (vx, 0), (vx, 24), color, 3)
            cv2.putText(frame, f"VP x={last['vp_x']:.0f}", (min(vx + 6, w - 90), 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return frame

    # ------------------------------------------------------------------ #
    # Dashboard API
    # ------------------------------------------------------------------ #
    def status(self, _query: dict[str, Any] | None = None) -> dict[str, Any]:
        self._last_poll = time.monotonic()
        with self._lock:
            samples = list(self._samples)
        if not samples:
            return {"ready": False, "frames": 0, "checks": [], "measured": {}}

        def med(key: str) -> float | None:
            values = [s[key] for s in samples if s[key] is not None]
            return median(values) if values else None

        n = len(samples)
        span = samples[-1]["t"] - samples[0]["t"]
        fps = (n - 1) / span if span > 0 else 0.0
        width = samples[-1]["frame_width"]
        pair_ratio = sum(s["pair"] for s in samples) / n
        m = {
            "frames": n,
            "fps": round(fps, 1),
            "pair_ratio": round(pair_ratio, 2),
            "vp_x": med("vp_x"),
            "vp_y": med("vp_y"),
            "vp_angle": med("vp_angle"),
            "vp_row": med("vp_row"),
            "lane_offset_ratio": med("offset"),
            "lane_width_px": med("lane_width"),
            "cross_lines_max": max(s["cross_lines"] for s in samples),
            "brightness": med("brightness"),
            "saturated": med("saturated"),
            "target_vp_x": width / 2.0,
        }
        checks = [
            self._check_light(m),
            self._check_fps(m),
            self._check_lane(m),
            self._check_center(m),
            self._check_vp_x(m),
            self._check_vp_row(m),
            self._check_cross(m),
        ]
        rounded = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items()}
        return {
            "ready": all(c["ok"] for c in checks),
            "frames": n,
            "checks": checks,
            "measured": rounded,
        }

    def capture(self, tune_manager: Any, body: dict[str, Any]) -> dict[str, Any]:
        """Write measured values into the live tune (Apply, not Save)."""
        st = self.status()
        m = st["measured"]
        what = set(body.get("what") or ["vp_center_deg", "vp_horizon_y"])
        values: dict[str, Any] = {}
        if "vp_center_deg" in what:
            if m.get("vp_angle") is None:
                raise ValueError("no VP measured: lane pair not visible")
            values["vp_center_deg"] = round(float(m["vp_angle"]), 1)
        if "vp_horizon_y" in what:
            if m.get("vp_y") is None:
                raise ValueError("no VP measured: lane pair not visible")
            values["vp_horizon_y"] = int(round(float(m["vp_y"])))
        if not values:
            raise ValueError("nothing to capture")
        tune_manager.apply(values)
        return {"applied": values, "precal": self.status()}

    # ------------------------------------------------------------------ #
    # Checks
    # ------------------------------------------------------------------ #
    @staticmethod
    def _result(key: str, label: str, ok: bool, value: str, hint: str) -> dict[str, Any]:
        return {"key": key, "label": label, "ok": bool(ok), "value": value, "hint": "" if ok else hint}

    def _check_light(self, m: dict[str, Any]) -> dict[str, Any]:
        b, sat = m["brightness"], m["saturated"]
        ok = self.brightness_min <= b <= self.brightness_max and sat <= self.saturated_max
        if b < self.brightness_min:
            hint = "Ảnh quá tối: bật đèn hoặc tăng exposure."
        elif b > self.brightness_max:
            hint = "Ảnh quá sáng: giảm exposure hoặc tránh nắng trực tiếp."
        else:
            hint = f"{sat * 100:.0f}% điểm ảnh bị cháy sáng (đèn phản chiếu): đổi chỗ hoặc góc."
        return self._result("light", "Ánh sáng", ok, f"mean {b:.0f}, cháy {sat * 100:.1f}%", hint)

    def _check_fps(self, m: dict[str, Any]) -> dict[str, Any]:
        fps = m["fps"]
        return self._result(
            "fps", "Tốc độ xử lý", fps >= self.fps_min, f"{fps:.1f} fps",
            f"Dưới {self.fps_min:.0f} fps: tắt object detection hoặc giảm tải CPU.",
        )

    def _check_lane(self, m: dict[str, Any]) -> dict[str, Any]:
        ratio = m["pair_ratio"]
        return self._result(
            "lane", "Thấy 2 vạch làn", ratio >= self.pair_min_ratio, f"{ratio * 100:.0f}% frame",
            "Không thấy đủ cặp vạch dọc: đặt xe giữa 2 vạch, kiểm tra Canny/Hough trong Tune.",
        )

    def _check_center(self, m: dict[str, Any]) -> dict[str, Any]:
        off = m["lane_offset_ratio"]
        if off is None:
            return self._result("center", "Xe ở giữa làn", False, "-", "Chưa thấy cặp vạch làn.")
        cm = off * self.tile_size_cm
        side = "phải" if off > 0 else "trái"
        return self._result(
            "center", "Xe ở giữa làn", abs(off) <= self.center_tol, f"lệch {off:+.3f} ({cm:+.1f} cm)",
            f"Dịch xe sang {side} khoảng {abs(cm):.0f} cm.",
        )

    def _check_vp_x(self, m: dict[str, Any]) -> dict[str, Any]:
        vx, target = m["vp_x"], m["target_vp_x"]
        if vx is None:
            return self._result("vp_x", f"VP ở giữa (x={target:.0f})", False, "-", "Chưa thấy VP.")
        dx = vx - target
        turn = "trái" if dx < 0 else "phải"
        return self._result(
            "vp_x", f"VP ở giữa (x={target:.0f})", abs(dx) <= self.vp_x_tol_px, f"x={vx:.0f} ({dx:+.0f} px)",
            f"Xe hoặc camera đang lệch hướng: xoay xe sang {turn} một chút. Nếu xe đã thẳng theo vạch gạch "
            f"thì lệch là do camera: chỉnh camera, hoặc bấm 'Lưu VP làm hướng thẳng'.",
        )

    def _check_vp_row(self, m: dict[str, Any]) -> dict[str, Any]:
        vy, row = m["vp_y"], m["vp_row"]
        if vy is None or row is None:
            return self._result("vp_row", "Pitch camera (VP row)", False, "-", "Chưa thấy VP.")
        return self._result(
            "vp_row", "Pitch camera (VP row)", abs(vy - row) <= self.vp_row_tol_px,
            f"VP y={vy:.0f}, tune {row:.0f}",
            "VP row khác giá trị trong Tune (camera bị nghiêng lên/xuống): bấm 'Lưu VP row'.",
        )

    def _check_cross(self, m: dict[str, Any]) -> dict[str, Any]:
        n = m["cross_lines_max"]
        return self._result(
            "cross", "Thấy vạch ngang", n >= 1, f"{n} vạch",
            "Bộ đếm ô không thấy vạch ngang: kiểm tra nhóm 'Cross lines' trong Tune (dark threshold, ROI).",
        )
