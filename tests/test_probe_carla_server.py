from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import textwrap


ROOT = Path(__file__).parents[1]
PROBE = ROOT / "scripts/e2e/probe_carla_server.py"


FAKE_CARLA = """
import os


class _Map:
    name = os.environ.get("FAKE_CARLA_MAP", "Carla/Maps/Town01")


class _Timestamp:
    elapsed_seconds = 12.5


class _Snapshot:
    frame = 42
    timestamp = _Timestamp()


class _World:
    def get_map(self):
        return _Map()

    def get_snapshot(self):
        if os.environ.get("FAKE_CARLA_SNAPSHOT_FAIL") == "1":
            raise RuntimeError("snapshot RPC unavailable")
        return _Snapshot()


class Client:
    def __init__(self, host, port):
        self.host = host
        self.port = port

    def set_timeout(self, timeout):
        self.timeout = timeout

    def get_world(self):
        if os.environ.get("FAKE_CARLA_WORLD_FAIL") == "1":
            raise RuntimeError("world RPC unavailable")
        return _World()
"""


def _run_probe(
    tmp_path: Path, *extra: str, env_override: dict[str, str] | None = None
) -> tuple[subprocess.CompletedProcess[str], dict]:
    module = tmp_path / "carla.py"
    module.write_text(textwrap.dedent(FAKE_CARLA), encoding="utf-8")
    server_log = tmp_path / "carla.log"
    server_log.write_text("CARLA_READY\n", encoding="utf-8")
    output = tmp_path / "health.json"
    env = os.environ.copy()
    env["PYTHONPATH"] = str(tmp_path)
    env.update(env_override or {})
    completed = subprocess.run(
        [
            str(PROBE),
            "--host",
            "127.0.0.1",
            "--port",
            "2100",
            "--expected-map",
            "Town01",
            "--stage",
            "trial_preflight",
            "--generation-id",
            "town01_straight_attempt_001",
            "--server-log",
            str(server_log),
            "--output",
            str(output),
            *extra,
        ],
        env=env,
        text=True,
        capture_output=True,
        check=False,
        timeout=5,
    )
    return completed, json.loads(output.read_text(encoding="utf-8"))


def test_fresh_read_only_rpc_probe_passes_exact_map_and_snapshot(
    tmp_path: Path,
) -> None:
    completed, health = _run_probe(tmp_path)

    assert completed.returncode == 0, completed.stderr
    assert health["status"] == "PASS"
    assert health["active_map_basename"] == "Town01"
    assert health["snapshot_frame"] == 42
    assert health["rpc_sequence"] == [
        "get_world",
        "world.get_map",
        "world.get_snapshot",
    ]
    assert health["read_only"] is True


def test_probe_rejects_map_mismatch(tmp_path: Path) -> None:
    completed, health = _run_probe(
        tmp_path, env_override={"FAKE_CARLA_MAP": "Carla/Maps/Town02"}
    )

    assert completed.returncode == 1
    assert health["status"] == "FAIL"
    assert "map mismatch" in health["error"]


def test_probe_rejects_rpc_failure_even_when_python_client_exists(
    tmp_path: Path,
) -> None:
    completed, health = _run_probe(
        tmp_path, env_override={"FAKE_CARLA_SNAPSHOT_FAIL": "1"}
    )

    assert completed.returncode == 1
    assert health["status"] == "FAIL"
    assert "snapshot RPC unavailable" in health["error"]
