"""Manual drive arbitration (ros/car_nav/car_nav/teleop_logic.py)."""
from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav.teleop_logic import TeleopConfig, TeleopMux


def mux(**kw):
    return TeleopMux(TeleopConfig(**kw))


def test_first_command_takes_control_once():
    m = mux()
    assert m.on_teleop(0.0, 0.2, 0.1) is True          # caller cancels the Nav2 goal
    assert m.on_teleop(0.05, 0.2, 0.1) is False
    assert m.command(0.06) == (0.2, 0.1)
    assert m.active(0.06)


def test_nav_output_is_dropped_while_manual_and_passes_otherwise():
    m = mux()
    assert m.nav_passes(0.0)
    m.on_teleop(1.0, 0.1, 0.0)
    assert not m.nav_passes(1.1)
    # the stream stops: deadman after hold_s, then a short stop phase, then Nav2 again
    assert m.command(1.5) == (0.0, 0.0)
    assert not m.nav_passes(1.6)
    assert m.command(2.5) is None and m.nav_passes(2.5)


def test_deadman_zero_when_stream_stops():
    m = mux(hold_s=0.3)
    m.on_teleop(0.0, 0.3, 0.0)
    assert m.command(0.25) == (0.3, 0.0)
    assert m.command(0.35) == (0.0, 0.0)               # no message for > 0.3 s
    assert not m.active(0.35)
    assert m.reason == "timeout"


def test_release_is_a_zero_command_and_stays_manual():
    m = mux()
    m.on_teleop(0.0, 0.3, 0.5)
    m.on_teleop(0.1, 0.0, 0.0)
    assert m.command(0.12) == (0.0, 0.0) and m.active(0.12)


def test_speed_and_turn_are_clipped():
    m = mux(max_speed_mps=0.2, max_yaw_rps=1.0)
    m.on_teleop(0.0, 5.0, -9.0)
    assert m.command(0.01) == (0.2, -1.0)


def test_reverse_ignored_unless_allowed():
    m = mux(allow_reverse=False)
    m.on_teleop(0.0, -0.3, 0.4)
    assert m.command(0.01) == (0.0, 0.4) and m.reverse_ignored
    m2 = mux(allow_reverse=True, max_speed_mps=0.4)
    m2.on_teleop(0.0, -0.3, 0.0)
    assert m2.command(0.01) == (-0.3, 0.0) and not m2.reverse_ignored


def test_estop_overrides_and_refuses_manual():
    m = mux()
    m.on_teleop(0.0, 0.3, 0.0)
    m.set_estop(0.1, True)
    assert not m.active(0.11)
    assert m.command(0.11) == (0.0, 0.0)
    assert m.on_teleop(0.2, 0.3, 0.0) is False
    assert m.command(5.0) is None                       # nothing from the page is passed
    m.set_estop(5.0, False)
    assert m.on_teleop(5.1, 0.3, 0.0) is True


def test_goal_ends_manual_and_page_is_ignored_until_silent():
    m = mux(release_s=0.5)
    m.on_teleop(0.0, 0.3, 0.0)
    assert m.goal(0.1) is True
    assert not m.active(0.12)
    assert m.on_teleop(0.2, 0.3, 0.0) is False          # the page is still streaming: ignored
    assert m.on_teleop(0.4, 0.3, 0.0) is False
    assert m.nav_passes(2.0)                             # Nav2 drives
    # silent for more than release_s, then the page may take control again
    assert m.on_teleop(1.2, 0.3, 0.0) is True
    assert m.active(1.21)


def test_goal_without_manual_does_not_block_nav():
    m = mux()
    assert m.goal(10.0) is False
    assert m.nav_passes(10.0)


def test_state_dict():
    m = mux(allow_reverse=False)
    m.on_teleop(0.0, -0.1, 0.0)
    s = m.state(0.01)
    assert s["manual"] and s["reverse_ignored"] and not s["allow_reverse"]


def test_collision_monitor_listens_to_the_mux_and_the_mux_is_launched():
    import yaml
    from tests._car_nav_paths import NAV2_YAML, ROOT
    cm = yaml.safe_load(open(NAV2_YAML, encoding="utf-8"))["collision_monitor"]["ros__parameters"]
    assert cm["cmd_vel_in_topic"] == "cmd_vel_mux" and cm["cmd_vel_out_topic"] == "cmd_vel_guarded"
    launch = (ROOT / "ros/car_nav/launch/nav_stack.launch.py").read_text(encoding="utf-8")
    assert 'executable="teleop_mux"' in launch
    assert "teleop_mux = car_nav.teleop_mux:main" in (ROOT / "ros/car_nav/setup.py").read_text(encoding="utf-8")
    assert '"/cmd_vel_mux"' in (ROOT / "ros/car_nav/car_nav/teleop_mux.py").read_text(encoding="utf-8")
