"""Start / stop one `ros2 bag record` process for a run (no ROS imports; the process is the recorder).

SIGINT lets rosbag2 write its metadata and close the file; after `stop_wait_s` it is terminated.
`poll()` (called every second by ui_bridge while recording) stops the run when the disk is nearly full,
after MAX_RUN_S, or when the recorder died.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time

from .run_store import DEFAULT_TOPICS, MAX_RUN_S, MIN_FREE_MB, RunStore, clean_topics, disk_blocker, free_mb


def default_storage() -> str:
    """mcap when its plugin is installed (rosbag2_storage_mcap), else sqlite3."""
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("rosbag2_storage_mcap")
        return "mcap"
    except Exception:  # noqa: BLE001
        return "sqlite3"


def record_command(bag_dir: str, topics: list[str], storage: str) -> list[str]:
    return ["ros2", "bag", "record", "-o", bag_dir, "--storage", storage, *topics]


class RunRecorder:
    def __init__(self, store: RunStore, min_free_mb: float = MIN_FREE_MB, max_run_s: float = MAX_RUN_S,
                 storage: str | None = None, command=record_command, stop_wait_s: float = 12.0) -> None:
        self.store = store
        self.min_free_mb = min_free_mb
        self.max_run_s = max_run_s
        self.storage = storage or default_storage()
        self._command = command
        self.stop_wait_s = stop_wait_s
        self.proc: subprocess.Popen | None = None
        self.run: dict | None = None
        self._t0 = 0.0
        self._log = None

    @property
    def recording(self) -> bool:
        return self.proc is not None

    def start(self, name: str, note: str = "", topics=None) -> dict:
        if self.proc is not None:
            raise RuntimeError(f"already recording {self.run['name']!r}")
        why = disk_blocker(self.store.root, self.min_free_mb)
        if why:
            raise RuntimeError(why)
        topics = clean_topics(topics) or list(DEFAULT_TOPICS)
        meta = self.store.new(name, note, topics, self.storage)
        bag = os.path.join(self.store.root, meta["id"], "bag")
        self._log = open(os.path.join(self.store.root, meta["id"], "record.log"), "wb")
        try:
            self.proc = subprocess.Popen(self._command(bag, topics, self.storage), stdout=self._log,
                                         stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                         start_new_session=(os.name != "nt"))
        except OSError as exc:
            self._log.close()
            self.store.delete(meta["id"])
            raise RuntimeError(f"cannot start ros2 bag: {exc}") from exc
        self.run, self._t0 = meta, time.monotonic()
        return meta

    def stop(self, reason: str = "stopped") -> dict | None:
        proc, run = self.proc, self.run
        if proc is None or run is None:
            return None
        if proc.poll() is None:
            try:
                proc.send_signal(signal.SIGINT if os.name != "nt" else signal.SIGTERM)
                proc.wait(self.stop_wait_s)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(5)
                except subprocess.TimeoutExpired:
                    proc.kill()
            except OSError:
                pass
        if self._log is not None:
            self._log.close()
        self.proc, self.run, self._log = None, None, None
        return self.store.finish(run["id"], reason, time.monotonic() - self._t0)

    def poll(self) -> str | None:
        """Call about once a second. Returns the reason when it stopped the run."""
        if self.proc is None:
            return None
        if self.proc.poll() is not None:
            code = self.proc.returncode
            self.stop(f"the recorder exited ({code}); see record.log")
            return "died"
        if free_mb(self.store.root) < self.min_free_mb:
            self.stop(f"stopped: disk space below {self.min_free_mb:.0f} MB")
            return "disk"
        if time.monotonic() - self._t0 > self.max_run_s:
            self.stop(f"stopped: maximum length {self.max_run_s / 60:.0f} min")
            return "max_length"
        return None

    def status(self) -> dict:
        r = self.run
        return {"recording": r is not None, "id": r["id"] if r else None, "name": r["name"] if r else None,
                "elapsed_s": round(time.monotonic() - self._t0, 1) if r else None,
                "size_mb": round(self._size() / 1e6, 2) if r else None,
                "free_mb": round(free_mb(self.store.root)), "min_free_mb": self.min_free_mb,
                "storage": self.storage}

    def _size(self) -> int:
        from .run_store import dir_size
        return dir_size(os.path.join(self.store.root, self.run["id"])) if self.run else 0
