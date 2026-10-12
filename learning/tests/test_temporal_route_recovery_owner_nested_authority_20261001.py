"""HH_261001 - Verify actual nested owner binding preserves numeric-only versus private-worker authority."""

from unittest import mock

import pytest

from scripts.e2e import research_temporal_route_recovery_fast45_town07_20261001 as owner
from portable_e2e import temporal_route_recovery_export_loader_research_20261001 as numeric


@pytest.mark.parametrize("failure", [False, True])
def test_real_outer_export_and_inherited_trial_scope_keep_private_binding_and_restore(failure):
    base = owner.previous.base
    original_worker, original_loader, original_live = base.worker, base.load_npz, base.live
    original_parent_loader = owner.previous.load_model
    description = dict(schema=owner.EXPORT_SCHEMA, optimizer_updates=3000, npz_sha256="a"*64,
        checkpoint_sha256="c"*64, declaration_sha256="d"*64)
    expected, calls, bindings = "f"*64, [], []
    def verify(run_id, sha):
        assert owner._owner_plan_sha is None
        calls.append("plan")
        return {"sha": sha}
    def authority(plan, sha):
        assert owner._owner_plan_sha is None and plan["sha"] == sha == expected
        calls.append("authority")
    def loaded(*args, **kwargs):
        bindings.append(dict(kwargs["research_owner_binding"]))
        return object()
    def frozen_worker(run_id, sha):
        assert base.worker is owner.private_worker and base.load_npz is owner.load_model
        assert base.live is owner.live and base.WEIGHTS == owner.WEIGHTS and base.METADATA == owner.METADATA
        assert owner._owner_plan_sha == sha == expected
        calls.append("frozen_worker")
        base.load_npz(base.WEIGHTS, base.METADATA, weights_sha256=base.WEIGHTS_SHA, metadata_sha256=base.METADATA_SHA)
        if failure: raise OSError("retain actual failure")
        return 7
    with mock.patch.object(owner, "read_json", return_value=description), \
            mock.patch.object(owner, "digest", return_value="a"*64), \
            mock.patch.object(base, "verify_plan", side_effect=verify), \
            mock.patch.object(base, "assert_owned_worker", side_effect=authority), \
            mock.patch.object(owner, "_frozen_worker", side_effect=frozen_worker), \
            mock.patch.object(numeric, "load_route_recovery_research_npz", side_effect=loaded):
        with owner.bound_fixed_recovery_exports():
            with owner.previous.scoped_trial(0):
                assert base.worker is owner.private_worker
                base.load_npz(base.WEIGHTS, base.METADATA, weights_sha256=base.WEIGHTS_SHA, metadata_sha256=base.METADATA_SHA)
                assert bindings[-1]["simulator_control_authorized"] is False
                if failure:
                    with pytest.raises(OSError): base.worker("v1", expected)
                else: assert base.worker("v1", expected) == 7
                assert owner._owner_plan_sha is None
                base.load_npz(base.WEIGHTS, base.METADATA, weights_sha256=base.WEIGHTS_SHA, metadata_sha256=base.METADATA_SHA)
                assert bindings[-1]["simulator_control_authorized"] is False
    assert calls == ["plan", "authority", "frozen_worker"]
    assert [row["simulator_control_authorized"] for row in bindings] == [False, True, False]
    assert bindings[1]["owner_plan_sha256"] == expected
    assert bindings[1]["purpose"] == "owned_carla_research_only" and bindings[1]["simulation_only"] is True
    assert all(row["vehicle_control_approved"] is False and row["model_promoted"] is False for row in bindings)
    assert base.worker is original_worker and base.load_npz is original_loader and base.live is original_live
    assert owner.previous.load_model is original_parent_loader
    assert owner._owner_plan_sha is None and not owner._active and owner._exports is None
