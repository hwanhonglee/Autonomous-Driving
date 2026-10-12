"""HH_261002 - Final6000 permission, both parent pairs, real preflight aliases and unchanged signed45 guards."""

from contextlib import contextmanager, ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_route_joint_signed45_20261002 as live
from scripts.e2e import research_temporal_route_joint_signed45_town07_20261002 as owner


def pins():
    return {name + "_sha256": character * 64 for name, character in
        (("weights", "a"), ("metadata", "b"), ("checkpoint", "c"), ("declaration", "d"))}


def args(seed=2026102100):
    return SimpleNamespace(seed=seed, wheelbase_m=2.85, host="127.0.0.1", port=2440, allow_map_load=False,
        control_transport="acknowledged_batch", goal_stop_profile="disabled", agent_initialization="after_bootstrap",
        physics_hz=20, capture_hz=10, stationary_warmup_sec=1.2, max_duration_sec=45, target_speed_kmh=9,
        weather="ClearNoon", spawn_z_offset_m=.5, research_speed_cap_mps=2.5, research_max_cte_m=1.,
        research_source_sha256="e"*64, research_longitudinal_pi=True, route_file=Path("unused"))


def model():
    return SimpleNamespace(_research_temporal_export_metadata={"schema": live.EXPORT_SCHEMA},
        _research_recovery_numeric_loader_schema=live.LOADER_SCHEMA,
        _research_recovery_simulator_control_authorized=True,
        _research_recovery_owner_binding=dict(schema=live.OWNER_SCHEMA, fixed_endpoint_updates=6000,
            purpose="owned_carla_research_only", simulation_only=True, simulator_control_authorized=True,
            owner_plan_sha256="e"*64, vehicle_control_approved=False, model_promoted=False))


def fake_export_description():
    return dict(schema=owner.EXPORT_SCHEMA, npz_sha256="a"*64, checkpoint_sha256="c"*64, declaration_sha256="d"*64)


@pytest.mark.parametrize("trial", [0, 1, 2])
def test_corrected_baseline_profiles_and_all_sensor_control_guards_are_identical(trial):
    spec = live.PROFILES[trial]
    assert spec == live.previous.PROFILES[trial]
    live.validate_capture(args(spec["seed"]), dict(town="Town07", scenario="straight"))
    assert live.validate_capture is live.previous.validate_capture
    assert owner.digest(Path(live.previous.__file__)) == live.FROZEN_COLLECTOR_SHA
    assert owner.digest(Path(owner.corrected.__file__)) == owner.CORRECTED_OWNER_SHA


@pytest.mark.parametrize("field,value", [("seed", 20261002100), ("seed", True), ("port", 2000),
    ("max_duration_sec", 44), ("research_speed_cap_mps", 2.6), ("research_longitudinal_pi", False),
    ("agent_initialization", "before_bootstrap"), ("goal_stop_profile", "comfortable_v1")])
def test_new_collector_cannot_relax_the_corrected_baseline_contract(field, value):
    arguments = args(); setattr(arguments, field, value)
    with pytest.raises(Exception): live.validate_capture(arguments, dict(town="Town07", scenario="straight"))


@pytest.mark.parametrize("field,value", [("schema", "old_frozen97_owner"), ("fixed_endpoint_updates", 3000),
    ("purpose", "offline_cpu_numeric_validation_only"), ("simulation_only", False),
    ("simulator_control_authorized", False), ("owner_plan_sha256", "f"*64),
    ("vehicle_control_approved", True), ("model_promoted", True)])
def test_only_explicit_final6000_private_owner_can_actuate(field, value):
    actual = model(); actual._research_recovery_owner_binding[field] = value
    with mock.patch.object(live, "verify_scope"), pytest.raises(Exception):
        live.validate_model_binding(actual, args())


@pytest.mark.parametrize("failure", [False, True])
def test_new_collector_restores_schema_and_does_not_replace_path_speed_or_control(failure):
    saved = live.previous.SCHEMA; actual = model()
    @contextmanager
    def inherited(given, used):
        assert given is actual and used.seed == 2026102100
        assert live.previous.SCHEMA == live.SCHEMA
        if failure: raise OSError("unchanged inherited failure")
        yield
    with mock.patch.object(live, "verify_frozen"), mock.patch.object(live, "verify_scope"), \
         mock.patch.object(live.collector, "load_route", return_value=dict(town="Town07", scenario="straight")), \
         mock.patch.object(live.previous, "activated", side_effect=inherited):
        try:
            with live.activated(actual, args()):
                with pytest.raises(Exception, match="nest"):
                    with live.activated(actual, args()): pass
        except OSError: assert failure
    assert live.previous.SCHEMA == saved and not live._active


def test_fixed_contract_removes_false_branch_only_learning_claims_and_pins_child_CLI():
    original = dict(current_fixed_final3000_V3_weights_unchanged=True, original_backbone_weights_byte_unchanged=True,
        backbone_optimizer_updates=3200, route_recovery_branch_optimizer_updates=3000, parent_checkpoint_updates=3200,
        no_new_training_or_checkpoint_selection=True,
        worker_command=["python", "-m", owner.MODULE, "--", "--mode", "worker"])
    with mock.patch.object(owner, "_exports", pins()), mock.patch.object(owner, "_signed_fixed", return_value=original):
        result = owner.fixed_contract("v1", True)
    assert result["optimizer_updates"] == 6000 and result["known_total_optimizer_updates"] == 12200
    assert result["trainable_state_tensor_count"] == 54 and result["frozen_state_tensor_count"] == 47
    assert not result["original_backbone_weights_byte_unchanged"] and not result["semantic_outputs_preserved"]
    assert "current_fixed_final3000_V3_weights_unchanged" not in result and "backbone_optimizer_updates" not in result
    assert "no_new_training_or_checkpoint_selection" not in result
    assert result["joint6000_training_completed_before_physical_study"]
    assert result["no_training_during_physical_study"] and result["no_checkpoint_selection"]
    for name, sha in pins().items():
        option = "--expected-" + name.removesuffix("_sha256") + "-sha256"
        assert result["worker_command"][result["worker_command"].index(option)+1] == sha
    assert result["no_output_path_speed_or_intent_replacement"]


@pytest.mark.parametrize("trial", [0, 1, 2])
def test_real_fixed_contract_in_all_aliases_has_truthful6000_child_and_unchanged_signed45_setup(trial):
    with mock.patch.object(owner, "verify_exports", return_value=fake_export_description()), \
         mock.patch.object(owner, "runtime_sources", return_value={}), \
         owner.activated(pins()), owner.parent.scoped_trial(trial):
        result = owner.base.fixed_contract("v1", True)
    assert result["seed"] == live.PROFILES[trial]["seed"]
    assert result["optimizer_updates"] == 6000 and result["known_total_optimizer_updates"] == 12200
    assert result["checkpoint_selection"] == "prospectively fixed final6000 only; no observed-best or HOLDOUT selection"
    assert result["total_capture_seconds"] == 45 and result["controller_target_cap_mps"] == 2.40
    assert result["sensor_and_ACK_screen"]["native_state_count"] == 900
    assert result["sensor_and_ACK_screen"]["camera_bundle_count"] == 450
    assert result["precursor_motion_screen"]["net_learned_progress_minimum_m"] == 90.
    assert result["precursor_motion_screen"]["post_handoff_base_planar_speed_maximum_mps"] == 2.5
    assert result["capture_argv"][result["capture_argv"].index("--seed")+1] == str(live.PROFILES[trial]["seed"])
    assert result["worker_command"][result["worker_command"].index("-m")+1] == owner.MODULE
    assert result["worker_command"][result["worker_command"].index("--expected-checkpoint-sha256")+1] == "c"*64
    assert result["original_backbone_weights_byte_unchanged"] is False and result["semantic_outputs_preserved"] is False
    assert "current_fixed_final3000_V3_weights_unchanged" not in result
    assert "no_new_training_or_checkpoint_selection" not in result
    assert result["joint6000_training_completed_before_physical_study"]


def test_numeric_preflight_and_private_worker_load_bind_both_actual_parent_pairs_without_pickles():
    with mock.patch.object(owner, "_exports", pins()), mock.patch.object(owner, "load_route_joint_research_npz", return_value="numeric") as loader:
        assert owner.load_model(owner.WEIGHTS, owner.METADATA, weights_sha256="a"*64, metadata_sha256="b"*64) == "numeric"
        received = loader.call_args.kwargs
        assert received["parent_weights"] == owner.WARM_PARENT / "model_state.npz"
        assert received["backbone_parent_weights"] == owner.BACKBONE_PARENT / "model_state.npz"
        assert received["research_owner_binding"]["fixed_endpoint_updates"] == 6000
        assert not received["research_owner_binding"]["simulator_control_authorized"]
        with mock.patch.object(owner, "_owner_plan_sha", "e"*64):
            owner.load_model(owner.WEIGHTS, owner.METADATA, weights_sha256="a"*64, metadata_sha256="b"*64)
        assert loader.call_args.kwargs["research_owner_binding"]["simulator_control_authorized"]
        assert loader.call_args.kwargs["research_owner_binding"]["owner_plan_sha256"] == "e"*64
        with pytest.raises(RuntimeError):
            owner.load_model(owner.WEIGHTS, owner.METADATA, weights_sha256="f"*64, metadata_sha256="b"*64)


@pytest.mark.parametrize("failure", [False, True])
def test_owner_bindings_restore_every_original_module_even_on_exception(failure):
    modules = (owner.base, owner.parent, owner.signed, owner.recovery)
    watched = {module: dict(vars(module)) for module in modules}
    with mock.patch.object(owner, "verify_exports", return_value=fake_export_description()), \
         mock.patch.object(owner, "runtime_sources", return_value={}):
        try:
            with owner.activated(pins()):
                assert owner.base.WEIGHTS == owner.WEIGHTS and owner.parent.live is live
                assert owner.parent.load_model is owner.load_model and owner.base.worker is owner.private_worker
                assert owner.signed.live is live and owner.recovery.PARENT == owner.WARM_PARENT
                for path in (owner.WARM_PARENT / "model_state.npz", owner.WARM_PARENT / "metadata.json",
                             owner.BACKBONE_PARENT / "model_state.npz", owner.BACKBONE_PARENT / "metadata.json"):
                    assert path in owner.base.ASSETS
                with pytest.raises(RuntimeError, match="overlap"):
                    with owner.activated(pins()): pass
                if failure: raise OSError("before-CARLA failure retained")
        except OSError: assert failure
    assert all(all(getattr(module, name) is value for name, value in before.items()) for module, before in watched.items())
    assert not owner._active and owner._exports is None


@pytest.mark.parametrize("failure", [False, True])
def test_worker_control_permission_follows_actual_private_PID_socket_assertion_only(failure):
    calls = []
    def worker(*arguments):
        assert calls == ["plan", "actual_private_PID_socket"] and owner._owner_plan_sha == "e"*64
        if failure: raise OSError("actual child failure")
        return 0
    with mock.patch.object(owner, "_active", True), mock.patch.object(owner, "_exports", pins()), \
         mock.patch.object(owner.base, "verify_plan", side_effect=lambda *a: calls.append("plan") or {}), \
         mock.patch.object(owner.base, "assert_owned_worker", side_effect=lambda *a: calls.append("actual_private_PID_socket")), \
         mock.patch.object(owner, "_original_worker", side_effect=worker):
        try: assert owner.private_worker("v1", "e"*64) == 0
        except OSError: assert failure
    assert owner._owner_plan_sha is None


def test_private_assertion_failure_cannot_enable_a_numeric_only_model():
    with mock.patch.object(owner, "_active", True), mock.patch.object(owner, "_exports", pins()), \
         mock.patch.object(owner.base, "verify_plan", return_value={}), \
         mock.patch.object(owner.base, "assert_owned_worker", side_effect=RuntimeError("foreign process")), \
         mock.patch.object(owner, "_original_worker") as worker, pytest.raises(RuntimeError, match="foreign"):
        owner.private_worker("v1", "e"*64)
    assert owner._owner_plan_sha is None and not worker.called


@pytest.mark.parametrize("trial", [0, 1, 2])
def test_actual_frozen_base_preflight_with_new6000_aliases_and_corrected_uint32_validator(tmp_path, trial):
    base, parent = owner.base, owner.parent
    output = tmp_path / "not-created"
    plan = dict(longitudinal_pi=True)
    numeric_model = SimpleNamespace(_research_temporal_checkpoint_sha256="c"*64)
    api = SimpleNamespace(stdout=str(base.EGG)+"/__init__.py\n"+str(base.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n1.8.0\n")
    saved_validator = live.frozen.validate_capture
    patches = (mock.patch.object(owner, "verify_exports", return_value=fake_export_description()),
        mock.patch.object(owner, "runtime_sources", return_value={}), mock.patch.object(base, "verify_plan", return_value=plan),
        mock.patch.object(base.lifecycle, "_assert_no_other_carla"), mock.patch.object(base, "ports_free"),
        mock.patch.object(base.subprocess, "run", return_value=api), mock.patch.object(base, "environment", return_value={}),
        mock.patch.object(live.collector, "load_route", return_value=dict(town="Town07", scenario="straight")),
        mock.patch.object(base.torch, "set_num_threads"))
    with ExitStack() as stack:
        for patch in patches: stack.enter_context(patch)
        with owner.activated(pins()), parent.scoped_trial(trial):
            assert base.live is live and live.frozen.validate_capture is live.validate_capture
            capture = base.capture_argv("v1", True, "e"*64)
            assert capture[capture.index("--seed")+1] == str(live.PROFILES[trial]["seed"])
            assert capture[capture.index("--research-weights")+1] == str(owner.WEIGHTS)
            with mock.patch.object(base, "paths", return_value=(output, None)), \
                 mock.patch.object(base, "disk_check", return_value={}), \
                 mock.patch.object(base, "load_npz", return_value=numeric_model):
                _, result = base.preflight("v1", "e"*64)
            assert result["status"] == "PREFLIGHT_PASS_NO_SIMULATOR" and not result["learned_control_executed"]
            assert not live._active
    assert not owner._active and not parent._active and live.frozen.validate_capture is saved_validator


def test_new_namespace_and_source_closure_cannot_overwrite_baseline_run():
    assert owner.batch_paths("v1")[0] != owner.corrected.previous.previous.batch_paths("v1")[0]
    assert "temporal_route_joint_signed45_town07_v1" in str(owner.batch_paths("v1")[0])
    for value in ("v0", "../v1", "v1/x"):
        with pytest.raises(RuntimeError): owner.batch_paths(value)
    closure = owner.base.imported_source_closure((Path(owner.__file__),))
    for module in (owner, live, owner.corrected, live.previous): assert Path(module.__file__).resolve() in closure
    assert owner.ROOT / "learning/portable_e2e/temporal_route_joint_export_research_20261002.py" in closure


def test_diagnostic_replaces_every_old_frozen97_and_final3000_claim():
    with mock.patch.object(owner, "_signed_diagnostic", return_value=dict(original_backbone_weights_byte_unchanged=True)):
        result = owner.diagnostic_report(Path("unused"), dict(source_checkpoint_sha256="c"*64), {})
    assert not result["original_backbone_weights_byte_unchanged"]
    assert result["optimizer_updates"] == 6000 and result["trainable_state_tensor_count"] == 54 and result["frozen_state_tensor_count"] == 47
    assert not result["semantic_outputs_preserved"] and not result["full_MAIN9_completion_claimed"]
