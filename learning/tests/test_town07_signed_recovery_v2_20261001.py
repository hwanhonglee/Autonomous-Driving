"""HH_261001 - Test isolated smaller v2 actual-spawn cohort without launching a simulator."""

import copy
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import run_owned_town07_signed_recovery_v2_20261001 as v2


def test_new_full_factorial_and_holdout_seeds_never_reuse_v1():
    train = [row for row in v2.CASES if row["split"] == "train"]
    dev = [row for row in v2.CASES if row["split"] == "dev"]
    assert len(train) == 9 and len(dev) == 2
    assert {(row["signed_lateral_offset_ros_m"], row["signed_yaw_offset_ros_deg"]) for row in train} == {
        (lateral, yaw) for lateral in (-.20, 0., .20) for yaw in (-1.5, 0., 1.5)}
    seeds = {row["seed"] for row in v2.CASES}
    assert len(seeds) == 11 and not seeds & {row["seed"] for row in v2.task.CASES}
    assert not {row["seed"] for row in train} & {row["seed"] for row in dev}


def test_full_scope_restores_frozen_modules_on_exception():
    task_names = ("CASES", "SCHEMA", "fixed_contract")
    owner_names = ("MODULE", "SCHEMA", "CASE_IDS", "runtime_sources", "declare_batch", "ASSETS")
    old_task = {name: getattr(v2.task, name) for name in task_names}
    old_owner = {name: getattr(v2.previous, name) for name in owner_names}
    old_control, old_collect = v2.previous.worker_module.v2.CombinedControl, v2.previous.worker_module.base.collect_episode
    with pytest.raises(RuntimeError):
        with v2.activated():
            assert v2.task.CASES is v2.CASES and v2.previous.MODULE == v2.MODULE
            assert v2.previous.ASSETS[v2.CENTER_MANIFEST] == v2.CENTER_MANIFEST_SHA
            assert v2.previous.worker_module.v2.CombinedControl is v2.SmoothRecoveryLaunchControl
            with pytest.raises(RuntimeError):
                with v2.activated(): pass
            raise RuntimeError("test")
    assert all(getattr(v2.task, name) is value for name, value in old_task.items())
    assert all(getattr(v2.previous, name) is value for name, value in old_owner.items())
    assert v2._active is False
    assert v2.previous.worker_module.v2.CombinedControl is old_control
    assert v2.previous.worker_module.base.collect_episode is old_collect


def test_case_scope_restores_then_v2_restores():
    old = v2.previous.lifecycle.MODULE
    with v2.activated():
        with v2.previous.scoped_case("train_00"):
            assert v2.previous.lifecycle.MODULE == v2.MODULE
        assert v2.previous.lifecycle.MODULE == old
    assert v2.previous._case is None and not v2.previous._active


def test_selected_clock_future_native_body_and_control_contract_unchanged():
    old = v2.task.fixed_contract("v1", "train_04")
    with v2.activated():
        new = v2.task.fixed_contract("v2", "train_04")
        for key in ("selected_driving_relative_time_ns", "minimum_selected_span_ns", "required_future_support_ns",
            "required_history_camera_observations", "required_history_ns", "native_timestamp_rounding_tolerance_ns",
            "observed_history_aggregate_clock_tolerance_ns", "observed_selected_span_aggregate_clock_tolerance_ns",
            "native_actor_speed_emergency_mps", "rear_base_absolute_CTE_emergency_m",
            "episode_quota_bytes", "free_disk_floor_bytes", "closing_reserve_bytes", "required_checks"):
            assert new[key] == old[key]
        assert new["schema"] == v2.TASK_SCHEMA and new["seed"] == 2026100314
        assert new["controller"]["launch_throttle"] == .11
        assert new["controller"]["post_handoff_throttle_ramp_per_second"] == .025
        assert new["controller"]["native_acceleration_qa_limit_mps2"] == 3.
        assert all(new["controller"][key] == value for key, value in old["controller"].items() if key != "launch_throttle")
        assert new["previous_failed_grid_preserved"] and new["old_successful_subset_not_used_as_v2_training"]
        assert new["holdout_training_or_checkpoint_selection_allowed"] is False
        assert "training_data_approved" not in new


def test_worker_command_new_module_new_version_and_original_guarded_collector():
    with v2.activated():
        value = v2.previous.execution_contract("v2", "train_00")
        assert v2.MODULE in value["worker_command"] and "v2" in value["worker_command"]
        assert "town07_signed_teacher_recovery_v2/train_00" in value["owner_output"]
        argv = v2.previous.capture_argv("v2", "train_00", "a"*64)
        assert argv[argv.index("--seed")+1] == "2026100310"
        assert argv[argv.index("--stationary-tail-sec")+1] == "6.5"
        assert argv[argv.index("--max-duration-sec")+1] == "50"
        assert "--allow-map-load" not in argv


@pytest.mark.parametrize("run_id", ["v1", "v3", "../v2", "V2"])
def test_old_or_arbitrary_versions_rejected(run_id):
    with pytest.raises(RuntimeError): v2.main(["--mode", "contract", "--run-id", run_id])


def test_main_activates_v2_and_forwards_default_then_restores():
    def invoked(argv):
        assert argv == ["--mode", "contract", "--run-id", "v2"]
        assert v2.task.SCHEMA == v2.TASK_SCHEMA
        return 7
    with mock.patch.object(v2.previous, "main", invoked):
        assert v2.main(["--mode", "contract"]) == 7
    assert not v2._active


def test_import_only_source_closure_includes_wrapper_and_exact_frozen_dependencies():
    with v2.activated():
        pins = v2.runtime_sources()
        assert pins["tools/e2e/run_owned_town07_signed_recovery_v2_20261001.py"] == v2.previous.digest(Path(v2.__file__))
        assert all(pins[name] == sha for name, sha in v2.FROZEN_V1.items())
        assert "tools/e2e/carla_physics_substeps.py" in pins
        assert "tools/e2e/carla_moving_capture.py" in pins
        assert pins == v2.runtime_sources()


def test_frozen_source_mutation_is_rejected_before_scope():
    original = v2.previous.digest
    target = next(iter(v2.FROZEN_V1))
    def changed(path): return "0"*64 if Path(path) == v2.ROOT/target else original(path)
    with mock.patch.object(v2.previous, "digest", changed):
        with pytest.raises(RuntimeError):
            with v2.activated(): pass
    assert not v2._active


def test_preflight_requires_explicit_scope():
    with pytest.raises(RuntimeError): v2.initial_body_preflight()


def test_geometry_must_pass_before_any_declaration_write():
    with v2.activated():
        with mock.patch.object(v2, "initial_body_preflight", side_effect=ValueError("unsafe")), \
                mock.patch.object(v2, "_declare_batch") as declare:
            with pytest.raises(ValueError): v2.declare_batch("v2")
            declare.assert_not_called()


def test_all_eleven_real_asset_static_boxes_and_sweeps_are_clear():
    # HH_261001 - Offline CARLA.Map only: actual recorded bbox, no simulator/actor/sensor jobs and no TRAIN rows.
    with v2.activated(): result = v2.initial_body_preflight()
    assert result["status"] == "PASS" and len(result["cases"]) == 11
    assert result["CARLA_server_started"] is False and result["training_data_approved"] is False
    assert result["method"] == "OFFLINE_PREDICTED_STATIC_FOOTPRINT_ONLY_NOT_ACTUAL_CAPTURE"
    for row in result["cases"]:
        assert row["status"] == "PASS" and not row["synthetic_training_rows_created"]
        assert all(value["violation_count"] == 0 for value in row["census"].values())
        assert all(value["minimum_interior_clearance_m"] > 0 for value in row["census"].values())


def test_actual_spawn_signs_and_hazardous_v1_grid_are_not_silently_relabelled():
    prior = copy.deepcopy(v2.task.CASES)
    with v2.activated():
        new = v2.task.case("train_00")
        assert (new["signed_lateral_offset_ros_m"], new["signed_yaw_offset_ros_deg"]) == (-.20, -1.5)
        nominal = dict(x=0., y=0., z=0., yaw=0., pitch=0., roll=0.)
        _, rear = v2.task.perturb_actor_center(nominal, wheelbase_m=2.85,
            signed_lateral_offset_ros_m=new["signed_lateral_offset_ros_m"], signed_yaw_offset_ros_deg=new["signed_yaw_offset_ros_deg"])
        assert rear["y"] == .20 and rear["yaw"] == 1.5
    assert v2.task.CASES == prior and prior[0]["signed_lateral_offset_ros_m"] == -.35


def control(throttle=.20, brake=0., steer=.2):
    return SimpleNamespace(throttle=throttle, brake=brake, steer=steer, hand_brake=False, reverse=False,
        manual_gear_shift=False, gear=0)


def test_new_teacher_actual_speed_launch_handoff_and_native_time_slew():
    policy = v2.SmoothRecoveryLaunchControl()
    first = policy.apply(control(), speed=0., timestamp=10., frame=1, normal_proposal={"raw": "retained"})
    assert first.throttle == .11 and first.steer == 0.
    assert policy.last["measured_planar_speed_mps"] == 0. and not policy.last["handoff_latched"]
    handed = policy.apply(control(), speed=.5, timestamp=10.5, frame=11, normal_proposal={"raw": "retained"})
    assert handed.throttle == pytest.approx(.11125) and handed.steer == 0.
    assert policy.handoff_at == 10.5 and policy.last["normal_launch_controls_inherited_v2"] is False
    next_control = policy.apply(control(), speed=1., timestamp=11.5, frame=31, normal_proposal={"raw": "retained"})
    assert next_control.throttle == pytest.approx(.13625) and next_control.steer == .2
    assert policy.last["measured_planar_speed_mps"] == 1. and policy.last["raw_normal_planner_proposal"] == {"raw": "retained"}
    final = policy.apply(control(), speed=2.4, timestamp=20.5, frame=211, normal_proposal=None)
    assert final.throttle == .20 and policy.last["normal_throttle_cap"] == .20


@pytest.mark.parametrize("speed,timestamp", [(-.1, 1.), (float("nan"), 1.), (0., float("inf"))])
def test_normal_teacher_nonfinite_measured_inputs_fail_not_clipped(speed, timestamp):
    with pytest.raises(ValueError): v2.SmoothRecoveryLaunchControl().apply(control(), speed=speed,
        timestamp=timestamp, frame=1, normal_proposal=None)


def test_actual_handoff_timeout_backwards_clock_and_normal_brake_preserved():
    policy = v2.SmoothRecoveryLaunchControl()
    policy.apply(control(), speed=0., timestamp=10., frame=1, normal_proposal=None)
    with pytest.raises(ValueError): policy.apply(control(), speed=.49, timestamp=18., frame=161, normal_proposal=None)
    with pytest.raises(ValueError): v2.SmoothRecoveryLaunchControl().apply(control(brake=.1), speed=0., timestamp=1., frame=1, normal_proposal=None)
    policy = v2.SmoothRecoveryLaunchControl()
    policy.apply(control(), speed=0., timestamp=10., frame=1, normal_proposal=None)
    with pytest.raises(ValueError): policy.apply(control(), speed=0., timestamp=9., frame=2, normal_proposal=None)


def test_new_policy_updates_live_metadata_truthfully_before_ack():
    manifest = {"capture_contract": {"teacher_controller": {"normal_launch_controls_inherited_v2": True}}}
    with mock.patch.object(v2, "_capture_manifest", manifest):
        v2.SmoothRecoveryLaunchControl().apply(control(), speed=0., timestamp=1., frame=1, normal_proposal=None)
    metadata = manifest["capture_contract"]["teacher_controller"]
    assert metadata["normal_launch_controls_inherited_v2"] is False
    assert metadata["launch_throttle"] == .11 and metadata["post_handoff_throttle_ramp_per_second"] == .025
    assert metadata["native_acceleration_qa_limit_mps2"] == 3. and metadata["inherited_BasicAgent_emergency_unchanged"]
    assert metadata["measured_speed_clipped"] is False and metadata["measured_acceleration_clipped"] is False


def test_capture_failure_keeps_new_metadata_and_restores_private_manifest_and_callback():
    manifest = {"capture_contract": {}}
    def failed(*p, **kw):
        p[6]["capture_contract"]["teacher_controller"] = {"normal_launch_controls_inherited_v2": True}
        raise RuntimeError("capture retained")
    with mock.patch.object(v2.previous.worker_module.base, "collect_episode", failed):
        with v2.activated():
            with pytest.raises(RuntimeError):
                v2.previous.worker_module.base.collect_episode(None, None, None, None, [], [], manifest)
            assert v2._capture_manifest is None
            assert manifest["capture_contract"]["teacher_controller"]["normal_launch_controls_inherited_v2"] is False
        assert v2.previous.worker_module.base.collect_episode is failed
