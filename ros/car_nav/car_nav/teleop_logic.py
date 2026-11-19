"""Manual-drive arbitration between the web page's joystick and Nav2 (no ROS imports).

teleop_mux sits in front of the collision monitor:

    Nav2 / PD follower -> /cmd_vel_nav ----+
                                           +--> TeleopMux --> /cmd_vel_mux -> collision monitor ->
    web page -> /cmd_vel_teleop -----------+                                   safety guard -> /cmd_vel

Rules (all unit-tested):

* The page streams commands at 10-20 Hz. A command older than `hold_s` (0.3 s) counts as none:
  the mux then sends zero (deadman). Releasing the joystick makes the page send zero; closing the
  page or losing the connection stops the stream, so the car stops within `hold_s`.
* While manual is active the Nav2 output is DROPPED (the two never fight); otherwise it passes
  through unchanged. Taking manual control is reported to the caller (`take` = True once) so it can
  cancel the active Nav2 goal; a new goal (`goal()`) ends manual control and ignores the page's
  commands until it has been silent for `release_s`, so the page's next command cannot grab the car
  back by accident.
* E-STOP overrides: while `estop` is set, the output is zero and manual commands are refused.
* Speed is clipped to `max_speed`; backward input only when `allow_reverse`, otherwise v < 0 -> 0
  (the turn is kept) and `reverse_ignored` is reported so the page can say so.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class TeleopConfig:
    hold_s: float = 0.3          # no command for this long = deadman: zero
    release_s: float = 0.5       # after a goal took the car back: ignore the page until it was silent this long
    max_speed_mps: float = 0.4
    max_yaw_rps: float = 1.5
    allow_reverse: bool = False
    zero_after_s: float = 0.6    # keep sending zero this long after manual ended (the downstream sees a clean stop)


def clip(x: float, lim: float) -> float:
    return max(-lim, min(lim, x))


@dataclass
class TeleopMux:
    cfg: TeleopConfig = field(default_factory=TeleopConfig)
    estop: bool = False
    _cmd: tuple[float, float] = (0.0, 0.0)
    _cmd_t: float = -1e9
    _manual: bool = False
    _locked: bool = False            # a goal ended manual control: ignore the page until it is silent
    _manual_end: float = -1e9
    reverse_ignored: bool = False
    reason: str = "off"              # why manual is not active: off | goal | estop | timeout | ...

    # ---------------------------------------------------------------- inputs
    def on_teleop(self, now: float, v: float, w: float) -> bool:
        """A command from the page. Returns True when this takes manual control (cancel the Nav2 goal)."""
        if self._locked:
            if now - self._cmd_t > self.cfg.release_s:
                self._locked = False         # the page was silent: it may take control again
            else:
                self._cmd_t = now            # still streaming: keep ignoring it
                return False
        self._cmd_t = now
        if self.estop:
            self.reason = "estop"
            return False
        c = self.cfg
        self.reverse_ignored = v < -1e-3 and not c.allow_reverse
        if v < 0.0 and not c.allow_reverse:
            v = 0.0
        self._cmd = (clip(v, c.max_speed_mps), clip(w, c.max_yaw_rps))
        took = not self._manual
        self._manual = True
        self.reason = "manual"
        return took

    def goal(self, now: float) -> bool:
        """A Nav2 goal was sent: manual control ends. Returns True when manual was active."""
        was = self.active(now)
        self._manual = False
        self._locked = True
        self._cmd_t = max(self._cmd_t, now)
        if was:
            self._manual_end = now           # a stop phase only when the car was driven by hand
        self._cmd = (0.0, 0.0)
        self.reason = "goal"
        return was

    def set_estop(self, now: float, on: bool) -> None:
        self.estop = on
        if on:
            self._manual = False
            self._cmd = (0.0, 0.0)
            self._manual_end = now
            self.reason = "estop"

    def release(self, now: float) -> None:
        """The page gave manual control back explicitly."""
        if self._manual:
            self._manual_end = now
        self._manual = False
        self._cmd = (0.0, 0.0)
        self.reason = "off"

    # --------------------------------------------------------------- outputs
    def active(self, now: float) -> bool:
        """Manual control is in force (a fresh command arrived within hold_s)."""
        return self._manual and not self.estop and now - self._cmd_t <= self.cfg.hold_s

    def command(self, now: float) -> tuple[float, float] | None:
        """What to publish for manual driving at this tick: (v, w), or None when manual is not active."""
        if self._manual and not self.estop and now - self._cmd_t > self.cfg.hold_s:
            self._manual = False             # deadman: the stream stopped
            self._cmd = (0.0, 0.0)
            self._manual_end = now
            self.reason = "timeout"
        if self._manual and not self.estop:
            return self._cmd
        if now - self._manual_end <= self.cfg.zero_after_s:
            return (0.0, 0.0)                # manual just ended: explicit zeros
        return None

    def nav_passes(self, now: float) -> bool:
        """The Nav2 output goes through (manual is not in force, and the stop phase after it is over)."""
        if self.estop:
            return True                      # the guard / E-STOP chain handles it downstream
        if self._manual and now - self._cmd_t <= self.cfg.hold_s:
            return False
        return now - self._manual_end > self.cfg.zero_after_s

    def state(self, now: float) -> dict:
        return {"manual": self.active(now), "reason": self.reason, "locked": self._locked,
                "estop": self.estop, "allow_reverse": self.cfg.allow_reverse,
                "reverse_ignored": self.reverse_ignored, "max_speed_mps": self.cfg.max_speed_mps,
                "max_yaw_rps": self.cfg.max_yaw_rps}
