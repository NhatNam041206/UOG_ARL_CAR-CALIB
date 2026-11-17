"""Timing budget helper (ros/car_nav/car_nav/timing.py)."""
from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.timing import TimingStats


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, s):
        self.lines.append(s)


def test_wrap_values_counts_and_window_log():
    log = _Log()
    t = TimingStats(log, period_s=0.0)
    f = t.wrap("cb", lambda x: x * 2)
    assert f(3) == 6
    t.value("age_s", 0.25)
    t.count("skipped", 2)
    joined = "\n".join(log.lines)
    assert "timing cb: n 1" in joined and " ms" in joined
    assert "timing age_s: n 1 p50 0.250" in joined
    assert "timing skipped: count 2" in joined


def test_disabled_is_transparent():
    log = _Log()
    t = TimingStats(log, period_s=0.0, enabled=False)
    fn = lambda: 1  # noqa: E731
    assert t.wrap("x", fn) is fn
    t.value("v", 1.0)
    assert log.lines == []
