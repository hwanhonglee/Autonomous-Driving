"""HH_261002 - Test the actual NumPy seed ABI and preserve original pre-spawn failures in a NEW scoped version."""

from contextlib import contextmanager
from pathlib import Path
import random
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

from scripts.e2e import collect_temporal_signed_recovery45_v2_20261002 as live
from scripts.e2e import research_temporal_signed_recovery45_v2_town07_20261002 as owner


def args(seed=2026102100):
    return SimpleNamespace(seed=seed,wheelbase_m=2.85,host="127.0.0.1",port=2440,allow_map_load=False,
        control_transport="acknowledged_batch",goal_stop_profile="disabled",agent_initialization="after_bootstrap",
        physics_hz=20,capture_hz=10,stationary_warmup_sec=1.2,max_duration_sec=45,target_speed_kmh=9,
        weather="ClearNoon",spawn_z_offset_m=.5,research_speed_cap_mps=2.5,research_max_cte_m=1.,
        research_source_sha256="a"*64,research_longitudinal_pi=True)


@pytest.mark.parametrize("seed",[2026102100,2026102101,2026102102])
def test_exact_profiles_pass_actual_frozen_numpy_reseeding_operation(seed):
    state=np.random.get_state()
    try:
        np.random.seed(seed)
        assert np.random.randint(10000)==np.random.RandomState(seed).randint(10000)
    finally:np.random.set_state(state)
    assert live.profile(seed)["seed"]==seed


@pytest.mark.parametrize("seed",[20261002100,20261002101,20261002102])
def test_original_python_random_acceptance_is_not_actual_numpy_seed_compatibility(seed):
    random.Random(seed)
    with pytest.raises(ValueError):np.random.seed(seed)
    with pytest.raises(Exception,match="uint32"):live.validate_seed(seed)


@pytest.mark.parametrize("seed",[True,-1,2**32,2**32+1,1.,None,"2026102100"])
def test_unsigned32_type_and_interval_reject_before_actor_sensor_or_capture(seed):
    with pytest.raises(Exception,match="uint32"):live.profile(seed)


@pytest.mark.parametrize("seed",[0,2**32-1])
def test_uint32_boundary_valid_but_unlisted_profiles_fail(seed):
    assert live.validate_seed(seed)==seed
    with pytest.raises(Exception,match="exact NEW"):live.profile(seed)


def test_v2_validator_retains_all_original_physical_gates_in_scoped_bindings():
    with mock.patch.object(live.previous,"profile",side_effect=live.profile):
        live.validate_capture(args(),dict(town="Town07",scenario="straight"))
        invalid=args();invalid.research_speed_cap_mps=2.6
        with pytest.raises(Exception,match="contract"):live.validate_capture(invalid,dict(town="Town07",scenario="straight"))


@pytest.mark.parametrize("after_spawn",[False,True])
def test_original_exception_not_masked_by_zero_spawn_finalizer_and_all_hooks_restore(tmp_path,after_spawn):
    agent=SimpleNamespace(BasicAgent=type("Agent",(),{}));manifest=dict(capture_contract={})
    center=dict(x=0.,y=0.,z=0.,yaw=0.,pitch=0.,roll=0.)
    def original(*positional,**keywords):
        if after_spawn:live.collector.apply_spawn_z_offset(center,.5)
        raise ValueError("original actual error: seeds must be uint32")
    scope=live.preserved_exception_spawn_scope()
    with mock.patch.object(live.collector,"collect_episode",side_effect=original), \
         mock.patch.object(live.collector,"apply_spawn_z_offset",side_effect=lambda pose,z:dict(pose,z=pose["z"]+z)):
        names=("collect_episode","apply_spawn_z_offset","_blank_brake_control","checked_agent_initialization_observation")
        saved={name:getattr(live.collector,name) for name in names};original_agent=agent.BasicAgent
        with pytest.raises(ValueError,match="original actual error"):
            with scope(args(),agent_module=agent):
                live.collector.collect_episode(None,None,None,tmp_path,None,None,manifest)
        assert all(getattr(live.collector,name) is value for name,value in saved.items())
        assert agent.BasicAgent is original_agent
    assert manifest["result"]["actual_spawn_perturbation_call_count"]==int(after_spawn)


def test_success_without_spawn_is_still_rejected_not_silently_accepted(tmp_path):
    agent=SimpleNamespace(BasicAgent=type("Agent",(),{}));manifest=dict(capture_contract={})
    with mock.patch.object(live.collector,"collect_episode",return_value="invalid apparent success"), \
         live.preserved_exception_spawn_scope()(args(),agent_module=agent),pytest.raises(Exception,match="exactly once"):
        live.collector.collect_episode(None,None,None,tmp_path,None,None,manifest)


@pytest.mark.parametrize("failure",[False,True])
def test_new_collector_scope_restores_frozen_aliases_on_success_and_error(failure):
    names=("PROFILES","SCHEMA","profile","validate_capture","startup_spawn_scope")
    saved={name:getattr(live.previous,name) for name in names}
    @contextmanager
    def parent(model,arguments):
        assert model=="fixed V3" and arguments.seed==2026102100
        assert live.previous.PROFILES==live.PROFILES and live.previous.SCHEMA==live.SCHEMA
        assert live.previous.profile is live.profile
        if failure:raise OSError("original delegated failure")
        yield
    with mock.patch.object(live,"verify_frozen"),mock.patch.object(live.previous,"activated",side_effect=parent):
        try:
            with live.activated("fixed V3",args()):
                with pytest.raises(Exception,match="nest"):
                    with live.activated("fixed V3",args()):pass
        except OSError:assert failure
    assert not live._active and all(getattr(live.previous,name) is value for name,value in saved.items())


@pytest.mark.parametrize("failure",[False,True])
def test_new_owner_scope_binds_exact_live_module_schema_and_worker_then_restores(failure):
    names=("live","MODULE","SCHEMA","fixed_contract","runtime_sources")
    saved={name:getattr(owner.previous,name) for name in names}
    @contextmanager
    def original_scope():
        assert owner.previous.live is live and owner.previous.MODULE==owner.MODULE and owner.previous.SCHEMA==owner.SCHEMA
        assert owner.previous.fixed_contract is owner.fixed_contract
        if failure:raise RuntimeError("preflight failure")
        yield
    with mock.patch.object(owner,"runtime_sources",return_value={}),mock.patch.object(owner.previous,"activated",side_effect=original_scope):
        try:
            with owner.activated():
                with pytest.raises(RuntimeError,match="nest"):
                    with owner.activated():pass
        except RuntimeError as error:
            assert failure and str(error)=="preflight failure"
    assert not owner._active and all(getattr(owner.previous,name) is value for name,value in saved.items())


def test_v2_contract_is_prospective_uint32_seed_revision_not_old_outcome_rewrite():
    with mock.patch.object(owner.previous,"trial_profile",return_value=live.PROFILES[1]), \
         mock.patch.object(owner,"_fixed",return_value=dict(schema=owner.SCHEMA)):
        plan=owner.fixed_contract("v2",True)
    assert plan["unsigned32_seed_prevalidated"] and plan["actual_seed_interval_inclusive"]==[0,2**32-1]
    assert plan["failed_v1_seeds_and_output_retained"] and plan["original_exception_not_masked_by_no_spawn_finalizer"]
    assert owner.previous.batch_paths("v2")[0]!=owner.previous.batch_paths("v1")[0]


def test_all_frozen_v1_bytes_remain_exact_and_new_closure_includes_both_wrappers():
    live.verify_frozen()
    assert owner.digest(Path(owner.previous.__file__))==owner.V1_SHA
    closure=owner.base.imported_source_closure((Path(owner.__file__),))
    assert Path(owner.__file__).resolve() in closure and Path(live.__file__).resolve() in closure
