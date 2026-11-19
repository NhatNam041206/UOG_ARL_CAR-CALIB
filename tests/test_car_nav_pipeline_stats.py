"""Pipeline rate / age bookkeeping for the Logs tab (ros/car_nav/car_nav/pipeline_stats.py)."""
from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.pipeline_stats import STAGES, Pipeline, Stage, status


def test_rate_and_age():
    s = Stage("/scan")
    for i in range(11):
        s.hit(i * 0.1)                                   # 10 Hz for 1 s
    assert abs(s.rate(1.0) - 10.0) < 0.01
    assert abs(s.age(1.5) - 0.5) < 1e-9


def test_rate_window_forgets_old_messages():
    s = Stage("/scan")
    for i in range(5):
        s.hit(i * 0.1)
    assert s.rate(100.0) == 0.0
    assert s.age(100.0) > 99


def test_never_seen():
    s = Stage("/x")
    assert s.rate(1.0) == 0.0 and s.age(1.0) is None


def test_pipeline_rows_and_status():
    p = Pipeline()
    p.hit("/scan_nav", 1.0, "n=360")
    p.hit("/scan_nav", 1.1, "n=361")
    p.hit("/not_a_stage", 1.0)                           # ignored
    rows = {r["topic"]: r for r in p.rows(1.2, {"/scan_nav": 1, "/cmd_vel": 1})}
    assert rows["/scan_nav"]["n"] == 2 and rows["/scan_nav"]["last"] == "n=361"
    assert status(rows["/scan_nav"]) == "ok"
    assert status(rows["/cmd_vel"]) == "silent"          # a publisher but nothing received
    assert status(rows["/scan"]) == "absent"
    stale = p.rows(10.0, lambda t: 1)
    assert status({r["topic"]: r for r in stale}["/scan_nav"]) == "stale"


def test_every_stage_of_the_chain_is_listed():
    topics = [t for _g, t, _k in STAGES]
    for t in ("/scan", "/scan_filtered", "/scan_nav", "/scan_dynamic", "/camera/depth/points", "/depth_obstacles",
              "/odom_rf2o", "/odom", "/amcl_pose", "/plan", "/cmd_vel_nav", "/cmd_vel_guarded", "/cmd_vel",
              "/car/wheels/cmd", "/car/wheels/applied", "/safety_guard/state"):
        assert t in topics
