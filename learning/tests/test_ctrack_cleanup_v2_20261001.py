"""HH_261001 - Reject cleanup races and keep RGB and semantic acceptance separate."""

from types import SimpleNamespace

import pytest

from scripts.e2e import ctrack_sandbox_cleanup_20261001 as cleanup
from scripts.e2e import run_owned_ctrack_isolated_probe_20261001_v2 as owner


def test_cleanup_waits_for_group_and_stably_free_ports(monkeypatch):
    row = {"pid": 10, "pgid": 10, "state": "S", "start_ticks": 123}
    process_states = iter(([row], [row], [], [], []))
    calls, now, port_checks = [], [0.0], [0]
    def clock():
        now[0] += .1
        return now[0]
    def ports(_port):
        port_checks[0] += 1
        if port_checks[0] <= 2:
            raise OSError("address in use during namespace teardown")
    monkeypatch.setattr(cleanup.time, "monotonic", clock)
    monkeypatch.setattr(cleanup.time, "sleep", lambda _delay: None)
    monkeypatch.setattr(cleanup.os, "killpg", lambda pid, sig: calls.append((pid, sig)))
    launcher = SimpleNamespace(pid=10, poll=lambda: 0, wait=lambda timeout: 0)
    result = cleanup.drain_owned_sandbox(
        launcher, row, None, 2400, inspect_process=lambda pid: row,
        list_processes=lambda: next(process_states), assert_ports_free=ports)
    assert result["status"] == "PASS"
    assert len(result["samples"]) == 4
    assert result["samples"][1]["ports_released"] is False
    assert len(calls) == 1


def test_cleanup_rejects_reused_launcher_before_any_signal(monkeypatch):
    monkeypatch.setattr(cleanup.os, "killpg", lambda *args: pytest.fail("must not signal reused PID"))
    with pytest.raises(RuntimeError, match="generation or PGID"):
        cleanup.drain_owned_sandbox(
            SimpleNamespace(pid=10), {"start_ticks": 1}, None, 2400,
            inspect_process=lambda pid: {"start_ticks": 2, "pgid": 10},
            list_processes=lambda: [], assert_ports_free=lambda port: None)


def test_rgb_warning_is_reported_without_altering_pixels():
    pixels = bytes([255, 190, 60]) * (640 * 32) + bytes([80, 80, 80]) * (640 * (360 - 32))
    result = owner.rgb_quality(pixels)
    assert result["warning_detected"] is True
    assert result["yellow_top32_pixels"] == 640 * 32
    assert result["visual_review_required"] is True
