"""HH_261001 - Source guard still fails foreign imports; deliberate wrapper no longer self-rejects."""

import pytest

from scripts.e2e import capture_town12_j15824_spawn_lift_v2_20261001 as worker
from scripts.e2e import run_owned_town12_j15824_spawn_lift_v2_20261001 as owner


def test_source_validation_occurs_before_hooks_and_trusted_tuple_is_scoped(monkeypatch):
    calls, trusted = [], (object(), object())
    def original_api():
        calls.append("original-source-validation")
        return trusted
    def checked_run(args):
        calls.append("inside-instrumentation")
        assert worker.inherited.scout.source_api() is trusted
        return args
    monkeypatch.setattr(worker.inherited.scout, "source_api", original_api)
    monkeypatch.setattr(worker.inherited.collector, "run", checked_run)
    with worker.checked_api_before_hooks():
        assert calls == ["original-source-validation"]
        assert worker.inherited.scout.source_api is original_api
        assert worker.inherited.collector.run(7) == 7
        assert worker.inherited.scout.source_api is original_api
    assert worker.inherited.collector.run is checked_run
    assert calls == ["original-source-validation", "inside-instrumentation"]


def test_foreign_api_rejected_before_collector_run(monkeypatch):
    def bad_api():
        raise ValueError("foreign source binding")
    monkeypatch.setattr(worker.inherited.scout, "source_api", bad_api)
    monkeypatch.setattr(worker.inherited.collector, "run", lambda *a: pytest.fail("collector called"))
    with pytest.raises(ValueError, match="foreign source"):
        with worker.checked_api_before_hooks():
            pytest.fail("guard bypassed")


def test_failure_restores_both_bindings(monkeypatch):
    original = lambda: (object(), object())
    def failed(args):
        raise RuntimeError("failed spawn")
    monkeypatch.setattr(worker.inherited.scout, "source_api", original)
    monkeypatch.setattr(worker.inherited.collector, "run", failed)
    with pytest.raises(RuntimeError):
        with worker.checked_api_before_hooks():
            worker.inherited.collector.run(None)
    assert worker.inherited.scout.source_api is original
    assert worker.inherited.collector.run is failed


def test_owner_selects_new_worker_and_new_output_without_changing_prior_sources():
    previous = owner.inherited.OUTPUT, owner.inherited.WORKER, owner.inherited.SOURCE_FILES
    with owner.configured_owner():
        assert owner.inherited.OUTPUT == owner.OUTPUT and owner.inherited.WORKER == owner.WORKER
        assert set(previous[2]) <= set(owner.inherited.SOURCE_FILES)
    assert (owner.inherited.OUTPUT, owner.inherited.WORKER, owner.inherited.SOURCE_FILES) == previous
