"""HH_261002 - Exercise the real owner preflight call path, including validator aliases before collector activation."""

import ast
from contextlib import contextmanager, ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

from scripts.e2e import collect_temporal_signed_recovery45_v3_20261002 as live
from scripts.e2e import research_temporal_signed_recovery45_v3_town07_20261002 as owner


def args(seed=2026102100):
    return SimpleNamespace(seed=seed,wheelbase_m=2.85,host="127.0.0.1",port=2440,allow_map_load=False,
        control_transport="acknowledged_batch",goal_stop_profile="disabled",agent_initialization="after_bootstrap",
        physics_hz=20,capture_hz=10,stationary_warmup_sec=1.2,max_duration_sec=45,target_speed_kmh=9,
        weather="ClearNoon",spawn_z_offset_m=.5,research_speed_cap_mps=2.5,research_max_cte_m=1.,
        research_source_sha256="a"*64,research_longitudinal_pi=True)


@pytest.mark.parametrize("seed",[2026102100,2026102101,2026102102])
def test_public_validator_before_activation_uses_actual_uint32_profile(seed):
    live.validate_capture(args(seed),dict(town="Town07",scenario="straight"))
    state=np.random.get_state()
    try:np.random.seed(seed)
    finally:np.random.set_state(state)


@pytest.mark.parametrize("field,value",[("seed",20261002100),("seed",True),("seed",0),("port",2000),
    ("max_duration_sec",44),("research_speed_cap_mps",2.6),("research_longitudinal_pi",False),
    ("agent_initialization","before_bootstrap"),("control_transport","direct"),("allow_map_load",True)])
def test_public_validator_retains_every_original_gate(field,value):
    arguments=args();setattr(arguments,field,value)
    with pytest.raises(Exception):live.validate_capture(arguments,dict(town="Town07",scenario="straight"))


def test_compiled_validator_original_ast_and_explicit_namespace_are_exact():
    original=live.previous.previous
    parsed=ast.parse(Path(original.__file__).read_bytes())
    function=next(n for n in parsed.body if isinstance(n,ast.FunctionDef) and n.name=="validate_capture")
    validator=live.explicit_original_validator()
    assert validator.__globals__["profile"] is live.profile
    assert validator.__code__.co_filename.endswith(":unchanged_validator_explicit_profile")
    assert "profile(args.seed)" in ast.unparse(function)
    assert validator.__globals__["frozen"] is original.frozen


@pytest.mark.parametrize("trial",[0,1,2])
def test_real_base_preflight_inside_all_owner_scopes_validates_prior_to_any_collector_activation(tmp_path,trial):
    """HH_261002 - Only external resources/loader are stubbed; the actual frozen preflight and alias resolution execute."""
    base,parent=owner.base,owner.parent
    old_validator=live.frozen.validate_capture
    model=SimpleNamespace(_research_temporal_checkpoint_sha256=base.CHECKPOINT_SHA)
    plan=dict(longitudinal_pi=True)
    output=tmp_path/"not-created"
    api=SimpleNamespace(stdout=str(base.EGG)+"/__init__.py\n"+str(base.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n1.8.0\n")
    route=dict(town="Town07",scenario="straight")
    # HH_261002 - Actual scoped owner export loading remains unchanged; avoid inference/model reads in this call-path unit proof.
    actual_v1=owner.previous.previous
    patches=(mock.patch.object(owner,"runtime_sources",return_value={}),
        mock.patch.object(owner.previous,"runtime_sources",return_value={}),
        mock.patch.object(actual_v1,"runtime_sources",return_value={}),
        mock.patch.object(base,"verify_plan",return_value=plan),
        mock.patch.object(base,"paths",return_value=(output,None)),
        mock.patch.object(base.lifecycle,"_assert_no_other_carla"),
        mock.patch.object(base,"ports_free"),mock.patch.object(base,"disk_check",return_value={}),
        mock.patch.object(base.subprocess,"run",return_value=api),
        mock.patch.object(base,"environment",return_value={}),
        mock.patch.object(live.collector,"load_route",return_value=route),
        mock.patch.object(base,"load_npz",return_value=model),
        mock.patch.object(base.torch,"set_num_threads"))
    with ExitStack() as stack:
        for patch in patches:stack.enter_context(patch)
        # HH_261002 - Mirror all frozen owner layers without reading/mutating actual checkpoint assets.
        @contextmanager
        def exports():
            changes=dict(batch_paths=actual_v1.batch_paths,fixed_contract=actual_v1.fixed_contract,
                runtime_sources=actual_v1.runtime_sources,live=actual_v1.live,MODULE=actual_v1.MODULE,
                SCHEMA=actual_v1.SCHEMA,diagnostic_report=actual_v1.diagnostic_report)
            saved={k:getattr(parent,k) for k in changes}
            try:
                for k,v in changes.items():setattr(parent,k,v)
                yield
            finally:
                for k,v in saved.items():setattr(parent,k,v)
        original_export_owner=actual_v1.previous
        with mock.patch.object(original_export_owner,"bound_fixed_recovery_exports",side_effect=exports), \
             owner.activated(),parent.scoped_trial(trial):
            assert base.live is live and live.frozen.validate_capture is live.validate_capture
            assert not live._active and not live.previous._active
            # HH_261002 - scoped_trial changes these stubs, so bind resource-only stubs after entering it.
            with mock.patch.object(base,"paths",return_value=(output,None)), \
                 mock.patch.object(base,"disk_check",return_value={}), \
                 mock.patch.object(base,"load_npz",return_value=model):
                actual_plan,result=base.preflight("v3","a"*64)
            assert actual_plan is plan and result["status"]=="PREFLIGHT_PASS_NO_SIMULATOR"
            assert result["learned_control_executed"] is False
    assert live.frozen.validate_capture is old_validator and not owner._active and not parent._active


@pytest.mark.parametrize("failure",[False,True])
def test_v3_collector_and_owner_restore_on_all_paths(failure):
    previous=live.previous; old_schema,old_validate=previous.SCHEMA,previous.validate_capture
    @contextmanager
    def delegate(model,arguments):
        assert previous.SCHEMA==live.SCHEMA and previous.validate_capture is live.validate_capture
        previous.validate_capture(arguments,dict(town="Town07",scenario="straight"))
        if failure:raise ValueError("original error preserved")
        yield
    with mock.patch.object(live,"verify_frozen"),mock.patch.object(previous,"activated",side_effect=delegate):
        try:
            with live.activated("fixed",args()):
                with pytest.raises(Exception,match="nest"):
                    with live.activated("fixed",args()):pass
        except ValueError:assert failure
    assert previous.SCHEMA==old_schema and previous.validate_capture is old_validate and not live._active


def test_new_source_closure_contains_all_previous_frozen_versions_and_new_wrapper_sources():
    live.verify_frozen()
    assert owner.digest(owner.previous.__file__)==owner.V2_SHA
    closure=owner.base.imported_source_closure((Path(owner.__file__),))
    assert all(Path(module.__file__).resolve() in closure for module in (owner,owner.previous,live,live.previous,live.previous.previous))
