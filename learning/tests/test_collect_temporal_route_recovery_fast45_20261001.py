"""HH_261001 - Check explicit new architecture activation and restoration, with no simulator launch."""

from contextlib import contextmanager
from types import SimpleNamespace

import pytest
import torch

from portable_e2e.temporal_model_v2 import TemporalModelConfigV2
from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
from scripts.e2e import collect_temporal_route_recovery_fast45_20261001 as new


@pytest.mark.parametrize("failure", [False, True])
def test_scoped_activation_preserves_original_sources_and_restores(tmp_path, monkeypatch, failure):
    model = RouteRecoveryTemporalModelV3(TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128)).eval()
    model._research_recovery_simulator_control_authorized = True
    model._research_recovery_owner_binding = dict(purpose="owned_carla_research_only", simulation_only=True,
        owner_plan_sha256="a"*64)
    args = SimpleNamespace(output=tmp_path/"episode", route_file=tmp_path/"route.json", research_source_sha256="a"*64)
    calls = []
    saved = new.previous.headroom.HeadroomOwnedCameraTemporalSimulationResearchPolicyV2
    original_validate = new.frozen.validate_capture
    monkeypatch.setattr(new.frozen.collector, "load_route", lambda path: {"town": "Town07"})
    monkeypatch.setattr(new.previous, "validate_capture", lambda *a: calls.append("validate"))
    @contextmanager
    def headroom(*a):
        assert new.previous.headroom.HeadroomOwnedCameraTemporalSimulationResearchPolicyV2 is new.RecoverySingleCopyPolicyV3
        calls.append("headroom"); yield
    @contextmanager
    def recorder(collector, path):
        assert path == tmp_path/"episode.partial"
        calls.append("recorder"); yield
    monkeypatch.setattr(new.previous.headroom, "activated", headroom)
    monkeypatch.setattr(new, "deferred_recording", recorder)
    try:
        with new.activated(model, args):
            assert new._active
            with pytest.raises(RuntimeError):
                with new.activated(model, args): pass
            if failure: raise OSError("capture failed")
    except OSError:
        assert failure
    assert calls == ["validate", "headroom", "recorder"]
    assert new.frozen.validate_capture is original_validate
    assert new.previous.headroom.HeadroomOwnedCameraTemporalSimulationResearchPolicyV2 is saved
    assert not new._active


@pytest.mark.parametrize("binding", [{}, {"purpose":"offline_cpu_numeric_validation_only"},
    {"purpose":"owned_carla_research_only","simulation_only":True,"owner_plan_sha256":"b"*64}])
def test_offline_or_other_owner_cannot_actuate(tmp_path,binding):
    model = RouteRecoveryTemporalModelV3(TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128)).eval()
    model._research_recovery_simulator_control_authorized = binding.get("purpose") == "owned_carla_research_only"
    model._research_recovery_owner_binding = binding
    with pytest.raises(ValueError,match="numeric-only"):
        with new.activated(model,SimpleNamespace(research_source_sha256="a"*64)): pass
    assert not new._active


@pytest.mark.parametrize("bad", ["not_model", "training", "double", "wrong_id"])
def test_wrong_architecture_or_runtime_ABI_fails_before_activation(tmp_path, bad):
    model = RouteRecoveryTemporalModelV3(TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128)).eval()
    if bad == "not_model": model = object()
    elif bad == "training": model.train()
    elif bad == "double": model.double()
    else: model.model_id = "not_v3"
    with pytest.raises(ValueError):
        with new.activated(model, SimpleNamespace(output=tmp_path/"episode")): pass
    assert not new._active
