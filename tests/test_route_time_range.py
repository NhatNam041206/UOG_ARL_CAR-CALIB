from __future__ import annotations

import io
import json
import time
import urllib.error
import urllib.request
import zipfile
from datetime import datetime, timezone
from pathlib import Path

import pytest

import runtime.route_logging as route_logging
from runtime.pi5_http import Pi5HttpServer
from runtime.route_logging import (
    bundle_routes_zip,
    parse_utc_bound,
    route_ids_in_range,
    route_start_utc,
)

IDS = [
    "route-20261101T080000Z-aaaa0001",
    "route-20261102T120000Z-aaaa0002",
    "route-20261103T090000Z-aaaa0003",
]


@pytest.fixture
def route_root(tmp_path: Path, monkeypatch) -> Path:
    monkeypatch.setattr(route_logging, "ROUTE_LOG_ROOT", str(tmp_path))
    for rid in IDS:
        (tmp_path / rid).mkdir()
        with zipfile.ZipFile(tmp_path / f"{rid}.zip", "w") as z:
            z.writestr("route_summary.json", "{}")
    # A route still running (no zip yet) and a dir without a timestamped id.
    (tmp_path / "route-20261102T130000Z-nozip000").mkdir()
    (tmp_path / "route-legacy").mkdir()
    return tmp_path


def _utc(s: str) -> datetime:
    return parse_utc_bound(s)


def test_route_start_utc_parses_id() -> None:
    assert route_start_utc(IDS[0]) == datetime(2026, 11, 1, 8, 0, tzinfo=timezone.utc)
    assert route_start_utc("route-legacy") is None


def test_parse_utc_bound_handles_z_offsets_and_blank() -> None:
    assert parse_utc_bound("") is None
    assert parse_utc_bound("2026-11-02T12:00:00Z") == datetime(2026, 11, 2, 12, tzinfo=timezone.utc)
    assert parse_utc_bound("2026-11-02T19:00:00+07:00") == datetime(2026, 11, 2, 12, tzinfo=timezone.utc)
    assert parse_utc_bound("2026-11-02T12:00") == datetime(2026, 11, 2, 12, tzinfo=timezone.utc)
    with pytest.raises(ValueError):
        parse_utc_bound("yesterday")


def test_route_ids_in_range_is_inclusive_and_open_ended(route_root: Path) -> None:
    assert route_ids_in_range(_utc("2026-11-02T12:00:00Z"), _utc("2026-11-03T09:00:00Z"), require_zip=True) == [IDS[2], IDS[1]]
    assert route_ids_in_range(None, _utc("2026-11-02T00:00:00Z")) == [IDS[0]]
    assert route_ids_in_range(_utc("2026-11-02T00:00:00Z"), None) == [
        IDS[2], "route-20261102T130000Z-nozip000", IDS[1],
    ]
    assert "route-legacy" in route_ids_in_range(None, None)


def test_bundle_of_range_contains_only_matching_zips(route_root: Path, tmp_path: Path) -> None:
    ids = route_ids_in_range(_utc("2026-11-02T00:00:00Z"), None, require_zip=True)
    dest = tmp_path / "out.zip"
    assert bundle_routes_zip(ids, dest) == 2
    with zipfile.ZipFile(dest) as z:
        assert sorted(z.namelist()) == sorted(f"{r}.zip" for r in IDS[1:])


@pytest.fixture
def port(route_root: Path):
    server = Pi5HttpServer(host="127.0.0.1", port=0, token="")
    server.set_routes_zip_bundler(bundle_routes_zip)
    server.start()
    deadline = time.monotonic() + 2.0
    while server._server is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert server._server is not None
    yield server._server.server_address[1]
    server.stop()
    server._server.server_close()


def _get(port: int, path: str) -> tuple[int, bytes]:
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=2) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


def test_download_all_count_and_range(port: int) -> None:
    code, body = _get(port, "/routes/download_all?count=1&from=2026-11-02T00:00:00Z&to=2026-11-02T23:59:59Z")
    assert code == 200
    assert json.loads(body) == {"count": 1, "route_ids": [IDS[1]]}

    code, body = _get(port, "/routes/download_all?from=2026-11-02T00:00:00Z")
    assert code == 200
    with zipfile.ZipFile(io.BytesIO(body)) as z:
        assert sorted(z.namelist()) == sorted(f"{r}.zip" for r in IDS[1:])


def test_download_all_range_errors(port: int) -> None:
    assert _get(port, "/routes/download_all?from=nope")[0] == 400
    assert _get(port, "/routes/download_all?from=2026-11-03T00:00:00Z&to=2026-11-01T00:00:00Z")[0] == 400
    assert _get(port, "/routes/download_all?from=2027-01-01T00:00:00Z")[0] == 404
