"""Recorded runs: store, zip, disk guard (ros/car_nav/car_nav/run_store.py)."""
import io
import zipfile

from tests import _car_nav_paths as P  # noqa: F401  (sets sys.path)
from car_nav import run_store as rs


def test_new_list_note_delete(tmp_path):
    s = rs.RunStore(str(tmp_path))
    m = s.new("lap 1", note="first", topics=["/scan_nav", "/tf", "bad topic!"])
    assert m["state"] == "recording" and m["topics"] == ["/scan_nav", "/tf"]
    bag = tmp_path / "runs" / m["id"] / "bag"
    bag.mkdir()
    (bag / "x.mcap").write_bytes(b"0" * 2_000_000)
    s.finish(m["id"], "stopped", 12.34)
    s.update(m["id"], note="slow lap")
    row = s.list()[0]
    assert row["state"] == "saved" and row["note"] == "slow lap" and row["duration_s"] == 12.3
    assert 1.9 < row["size_mb"] < 2.1
    assert s.delete(m["id"]) and s.list() == [] and not s.delete(m["id"])


def test_default_topics_cover_the_brief():
    for t in ("/scan_nav", "/odom", "/tf", "/tf_static", "/amcl_pose", "/plan", "/cmd_vel", "/car/wheels/cmd",
              "/car/wheels/applied", "/safety_guard/state", "/depth_obstacles", "/imu/data_raw", "/rosout"):
        assert t in rs.DEFAULT_TOPICS
    assert rs.clean_topics(None) == [] and rs.clean_topics(["/a", "/a", "b", "/c/d"]) == ["/a", "/c/d"]


def test_zip_contains_the_bag_folder(tmp_path):
    s = rs.RunStore(str(tmp_path))
    m = s.new("z")
    (tmp_path / "runs" / m["id"] / "bag").mkdir()
    (tmp_path / "runs" / m["id"] / "bag" / "metadata.yaml").write_text("a: 1")
    buf = io.BytesIO()
    assert s.zip_to(m["id"], buf)
    names = zipfile.ZipFile(io.BytesIO(buf.getvalue())).namelist()
    assert f"{m['id']}/bag/metadata.yaml" in names and f"{m['id']}/meta.json" in names
    assert not s.zip_to("nope", io.BytesIO())


def test_disk_guard(tmp_path):
    assert rs.disk_blocker(str(tmp_path), min_free_mb=0) is None
    assert "free" in rs.disk_blocker(str(tmp_path), min_free_mb=1e12)


def test_unfinished_runs_are_marked(tmp_path):
    s = rs.RunStore(str(tmp_path))
    m = s.new("crash")
    s.mark_unfinished()
    assert s.get(m["id"])["state"] == "interrupted"


def test_link_test(tmp_path):
    s = rs.RunStore(str(tmp_path))
    m = s.new("t")
    s.link_test(m["id"], "20260101-000000_a")
    s.link_test(m["id"], "20260101-000000_a")
    assert s.get(m["id"])["tests"] == ["20260101-000000_a"]


def _fake(sleep_s=30):
    import sys

    def cmd(bag, topics, storage):
        return [sys.executable, "-c", f"import time,os;os.makedirs({bag!r});open({bag!r}+'/x.db3','w').write('1');time.sleep({sleep_s})"]
    return cmd


def test_recorder_start_stop_and_guards(tmp_path):
    import time

    from car_nav.run_recorder import RunRecorder
    st = rs.RunStore(str(tmp_path))
    rec = RunRecorder(st, min_free_mb=0, storage="sqlite3", command=_fake(), stop_wait_s=3)
    m = rec.start("t1", "note")
    assert rec.recording and rec.status()["name"] == "t1"
    try:
        rec.start("again")
        raise AssertionError("must refuse a second recording")
    except RuntimeError:
        pass
    time.sleep(1.0)
    done = rec.stop("stopped")
    assert not rec.recording and done["state"] == "saved" and st.get(m["id"])["stop_reason"] == "stopped"
    assert rec.stop() is None


def test_recorder_refuses_a_full_disk_and_stops_when_it_fills(tmp_path):
    import time

    from car_nav.run_recorder import RunRecorder
    st = rs.RunStore(str(tmp_path))
    full = RunRecorder(st, min_free_mb=1e12, storage="sqlite3", command=_fake())
    try:
        full.start("x")
        raise AssertionError("must refuse")
    except RuntimeError as exc:
        assert "free" in str(exc)
    rec = RunRecorder(st, min_free_mb=0, storage="sqlite3", command=_fake(), stop_wait_s=3)
    rec.start("y")
    rec.min_free_mb = 1e12                                   # the disk "fills up"
    assert rec.poll() == "disk" and not rec.recording
    assert "disk" in st.list()[0]["stop_reason"]
    rec2 = RunRecorder(st, min_free_mb=0, storage="sqlite3", command=_fake(0.2), stop_wait_s=3)
    rec2.start("z")
    time.sleep(1.5)
    assert rec2.poll() == "died"                             # the recorder exited by itself
