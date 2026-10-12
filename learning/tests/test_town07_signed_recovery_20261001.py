"""HH_261001 - CPU-only actual-spawn, prospective recovery membership and isolated owner contracts."""

import errno
import ast
import hashlib
import math
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import town07_signed_recovery_task_20261001 as task
from scripts.e2e import collect_town07_signed_recovery_20261001 as worker
from scripts.e2e import run_owned_town07_signed_recovery_20261001 as owner


def observe(value, frame, phase, *, speed=2.4, cte=0., event=0, progress=None, output=0, free=10*1024**3):
    return value.observe(frame=frame, timestamp=frame*.05, phase=phase, progress=frame*.1 if progress is None else progress,
        base_stop_reason=None, event_count=event, observed_free_disk_bytes=free,
        native_actor_speed_mps=speed, rear_base_CTE_m=cte, observed_output_bytes=output)


def camera(frame, phase):
    return dict(frame=frame, timestamp=frame*.05, capture_phase=phase, camera_order=list(task.original.CAMERAS),
        source_timestamps={name: frame*.05 for name in task.original.CAMERAS}, images={name: "real.jpg" for name in task.original.CAMERAS})


def complete_task():
    value = task.MovingTask(task.fixed_contract("v1", "train_04"))
    states, cameras = [], []
    projector = worker.base.RouteProjector(dict(route=[dict(x=0., y=0., distance_m=0., vad_command=3),
        dict(x=1000., y=0., distance_m=1000., vad_command=3)]))
    for frame in range(1, 904):
        phase = "stationary_warmup" if frame <= 24 else "driving" if frame <= 773 else "stationary_tail"
        progress = task.measured_progress([dict(x=0., y=0.), dict(x=1000., y=0.)], frame*.1, 0.)
        cte = projector.project(frame*.1, 0., progress).cross_track_error_m
        observed, _ = observe(value, frame, phase, progress=progress, cte=cte)
        states.append(dict(frame=frame, timestamp=frame*.05, capture_phase=phase, moving_task=observed))
        if (frame-1)%2 == 0:
            bundle = camera(frame, phase)
            value.saved_camera(bundle)
            cameras.append(bundle)
    return value, states, cameras


def test_grid_is_signed_full_factorial_and_holdouts_are_reserved():
    train = [row for row in task.CASES if row["split"] == "train"]
    dev = [row for row in task.CASES if row["split"] == "dev"]
    assert len(train) == 9 and len(dev) == 2
    assert {(row["signed_lateral_offset_ros_m"], row["signed_yaw_offset_ros_deg"]) for row in train} == {
        (lateral, yaw) for lateral in (-.35, 0., .35) for yaw in (-3., 0., 3.)}
    assert not {row["seed"] for row in train} & {row["seed"] for row in dev}
    assert task.fixed_contract("v1", "holdout_00")["holdout_training_or_checkpoint_selection_allowed"] is False


@pytest.mark.parametrize("yaw", [0., 30., 90., -70.])
@pytest.mark.parametrize("lateral,delta", [(-.35, -3.), (0., 0.), (.35, 3.), (-.25, 2.)])
def test_actual_rear_pose_shift_rotation_then_actor_center(yaw, lateral, delta):
    center = dict(x=10., y=20., z=.3, yaw=yaw, pitch=0., roll=0.)
    changed, rear = task.perturb_actor_center(center, wheelbase_m=2.85,
        signed_lateral_offset_ros_m=lateral, signed_yaw_offset_ros_deg=delta)
    theta = math.radians(yaw)
    nominal_rear = (center["x"]-1.425*math.cos(theta), center["y"]-1.425*math.sin(theta))
    assert rear["x"]-nominal_rear[0] == pytest.approx(lateral*math.sin(theta))
    assert rear["y"]-nominal_rear[1] == pytest.approx(-lateral*math.cos(theta))
    assert rear["yaw"] == yaw-delta
    newtheta = math.radians(yaw-delta)
    assert changed["x"]-1.425*math.cos(newtheta) == pytest.approx(rear["x"])
    assert changed["y"]-1.425*math.sin(newtheta) == pytest.approx(rear["y"])
    assert center == dict(x=10., y=20., z=.3, yaw=yaw, pitch=0., roll=0.)


@pytest.mark.parametrize("lateral,yaw", [(.351, 0.), (0., 3.001), (math.nan, 0.), (0., math.inf)])
def test_outside_spawn_envelope_refused(lateral, yaw):
    with pytest.raises(ValueError): task.perturb_actor_center(dict(x=0., y=0., z=0., yaw=0., pitch=0., roll=0.),
        wheelbase_m=2.85, signed_lateral_offset_ros_m=lateral, signed_yaw_offset_ros_deg=yaw)


def test_fixed_membership_exact_history_actual_future_then_tail():
    value, states, cameras = complete_task()
    result = value.result(tail_states=130)
    assert result["task_complete"]
    assert len(states) == 903 and len(cameras) == 452 and len(value.selected) == 302
    assert value.first["frame"] == 43 and value.last["frame"] == 645
    assert value.first["timestamp_ns"]-value.driving_origin == 900_000_000
    assert value.last["timestamp_ns"]-value.driving_origin == 31_000_000_000
    assert value.support["timestamp_ns"] == value.last["timestamp_ns"]+6_400_000_000
    assert value.history_support["camera_observations"][0]["frame"] == 25
    assert value.history_support["preceding_native_guard"]["frame"] == 24
    assert not any(row["moving_task_selected"] for row in cameras if row["capture_phase"] != "driving")
    assert "training_data_approved" not in result
    assert result["training_admission_state"] == "PENDING_QA_NOT_AUTHORIZED"


def test_support_never_fabricated_from_short_capture():
    value = task.MovingTask(task.fixed_contract("v1", "train_00"))
    for frame in range(1, 41):
        phase = "stationary_warmup" if frame <= 24 else "driving"
        observe(value, frame, phase)
        if frame%2: value.saved_camera(camera(frame, phase))
    assert value.support is None and not value.result(tail_states=130)["task_complete"]


@pytest.mark.parametrize("kwargs,reason", [({"speed": 2.70001}, "recovery_native_actor_speed_limit"),
    ({"cte": 1.00001}, "recovery_rear_base_CTE_limit"), ({"cte": -1.00001}, "recovery_rear_base_CTE_limit"),
    ({"event": 1}, "recovery_collision_or_lane_event"), ({"free": task.FLOOR+task.RESERVE-1}, "recovery_disk_floor"),
    ({"output": task.QUOTA-task.RESERVE+1}, "recovery_episode_disk_quota")])
def test_actual_native_event_and_resource_failure_latches(kwargs, reason):
    value = task.MovingTask(task.fixed_contract("v1", "train_04"))
    _, observed_reason = observe(value, 1, "driving", **kwargs)
    assert observed_reason == reason
    observe(value, 2, "driving")
    assert value.failure == reason and not value.result(tail_states=130)["task_complete"]


def test_target_cap_is_teacher_request_not_observed_speed_clip():
    ramp = worker.TargetRamp()
    assert ramp.update(1) == 2.4
    values = [ramp.update(frame) for frame in range(2, 62)]
    assert max(values) == 2.4 and values[0] == pytest.approx(.075)
    with pytest.raises(ValueError): ramp.update(62+1)


def test_scoped_owner_restores_every_shared_binding_on_error():
    names = ("paths", "preflight", "diagnostic_report", "SCHEMA", "CHECKPOINT_SHA", "ASSETS", "MODULE")
    before = {name: getattr(owner.lifecycle, name) for name in names}
    with pytest.raises(RuntimeError):
        with owner.scoped_case("train_00"):
            assert owner.lifecycle.CHECKPOINT_SHA is None
            with pytest.raises(RuntimeError):
                with owner.scoped_case("train_01"): pass
            raise RuntimeError("test")
    assert all(getattr(owner.lifecycle, name) is value for name, value in before.items())
    assert owner._case is None and owner._active is False


def test_recovery_command_uses_actual_source_route_rig_and_case_plan():
    argv = owner.capture_argv("v1", "train_00", "a"*64)
    assert argv[argv.index("--moving-task-plan-sha256")+1] == "a"*64
    assert argv[argv.index("--target-speed-kmh")+1] == "9"
    assert argv[argv.index("--stationary-tail-sec")+1] == "6.5"
    assert str(task.ROUTE) in argv and "--allow-map-load" not in argv
    assert not any("weights" in value or "learned" in value for value in argv)


def test_all_case_disk_reserve_not_single_case_only():
    need = task.FLOOR+11*owner.ARM_RESERVE+owner.SHARED_RESERVE
    assert owner.disk_check(need)["prospective_cases"] == 11
    with pytest.raises(RuntimeError): owner.disk_check(need-1)


def test_source_closure_is_import_only_no_glob_and_frozen_original():
    pins = owner.runtime_sources()
    assert "tools/e2e/collect_town07_signed_recovery_20261001.py" in pins
    assert "tools/e2e/town07_signed_recovery_task_20261001.py" in pins
    assert "tools/e2e/carla_physics_substeps.py" in pins
    assert pins["tools/e2e/collect_temporal_v2_simulation_research_20261001.py"] == owner.lifecycle.FROZEN_GENERIC["tools/e2e/collect_temporal_v2_simulation_research_20261001.py"]


def test_case_zero_holdout_cannot_be_declared_as_train():
    contract = task.fixed_contract("v1", "holdout_00")
    assert contract["split"] == "dev"
    assert contract["holdout_training_or_checkpoint_selection_allowed"] is False


def test_body_postclosure_corruption_fails_before_pass():
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory)
        source = output/"live.py"; source.write_text("original")
        archive = output/"source"/"live.py"; archive.parent.mkdir(); archive.write_text("original")
        sha = hashlib.sha256(source.read_bytes()).hexdigest()
        with mock.patch.object(owner, "ROOT", output):
            plan = dict(source_sha256={"live.py": sha}, external_source_sha256={}, asset_sha256={})
            owner.closure_check(output, plan)
            archive.write_text("corrupt")
            with pytest.raises(RuntimeError): owner.closure_check(output, plan)


def test_provider_covers_every_original_moving_module_attribute():
    source = ast.parse(Path(worker.base.__file__).read_text())
    required = {node.attr for node in ast.walk(source) if isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name) and node.value.id == "moving"}
    provider = worker.runtime_provider(task.MovingTask)
    assert all(hasattr(provider, name) for name in required)
    assert provider.__file__ == task.__file__ and provider.reconstruct is task.reconstruct


def test_private_worker_cli_cannot_connect_without_owner():
    with mock.patch.object(worker.base, "run") as run:
        assert worker.main([]) == 1
    run.assert_not_called()


def test_teacher_pixels_are_original_sources_not_synthetic_augmentation():
    contract = task.fixed_contract("v1", "train_00")
    assert contract["raw_original_BGRA_camera_SHA_at_every_bundle"] is True
    assert contract["saved_original_JPEG_SHA_at_every_bundle"] is True
    assert contract["exact_BGRA_payload_archive_retained"] is False
    assert contract["original_JPEG_encoding_unchanged"] is True


def test_full_actual_camera_source_denominator_and_jpeg_corruption():
    with tempfile.TemporaryDirectory() as directory:
        episode = Path(directory)
        bundle = camera(1, "driving")
        bundle["original_BGRA_source_sha256"], bundle["saved_JPEG_sha256"] = {}, {}
        records = []
        for name in worker.base.MODEL_CAMERA_ORDER:
            relative = Path("images")/name/"00000001.jpg"
            path = episode/relative; path.parent.mkdir(parents=True); path.write_bytes(b"actual fixture JPEG")
            sha = hashlib.sha256(path.read_bytes()).hexdigest()
            bundle["images"][name] = str(relative)
            bundle["original_BGRA_source_sha256"][name] = "a"*64
            bundle["saved_JPEG_sha256"][name] = sha
            records.append(dict(frame=1, camera=name, timestamp=.05, encoding="original_carla_bgra8", original_BGRA_sha256="a"*64,
                saved_JPEG_sha256=sha, saved_JPEG_relative_path=str(relative), JPEG_encoder_unchanged=True, pose_or_image_post_transformed=False))
        journal = episode/"actual_camera_pixels.jsonl"
        journal.write_text("".join(json.dumps(row)+"\n" for row in records))
        assert owner.pixel_audit(episode, [bundle])["actual_camera_source_count"] == 6
        (episode/bundle["images"][worker.base.MODEL_CAMERA_ORDER[0]]).write_bytes(b"changed")
        with pytest.raises(RuntimeError): owner.pixel_audit(episode, [bundle])


def test_real_owned_command_parses_against_create_only_prospective_case_declaration():
    with tempfile.TemporaryDirectory() as directory:
        base = Path(directory)
        def paths(run_id, case_id=None):
            if case_id is None: return base, base/"batch_plan.json"
            task.case(case_id)
            return base/case_id, base/(case_id+"_plan.json")
        with mock.patch.object(task, "paths", paths):
            plan = owner.build_plan("v1", "train_04")
            _, path = paths("v1", "train_04")
            owner.write_new(path, plan)
            sha = owner.digest(path)
            parsed = worker.parse_args(owner.capture_argv("v1", "train_04", sha))
            assert parsed.recovery_case_id == "train_04" and parsed.target_speed_kmh == 9.
            assert task.configuration_from_args(parsed) == plan
            parsed.seed += 1
            with pytest.raises(ValueError): task.configuration_from_args(parsed)


def test_worker_scope_restores_provider_spawn_native_reader_and_encoder_on_exception():
    original_agent = type("FakeAgent", (), {})
    agents = SimpleNamespace(BasicAgent=original_agent)
    names = ("collect_episode", "AcknowledgedControlTransport", "moving_capture_module", "apply_spawn_z_offset", "acknowledged_snapshot_state", "_save_jpeg")
    before = {name: getattr(worker.base, name) for name in names}
    args = SimpleNamespace(wheelbase_m=2.85)
    with mock.patch.dict("sys.modules", {"agents": SimpleNamespace(), "agents.navigation": SimpleNamespace(basic_agent=agents),
        "agents.navigation.basic_agent": agents}), mock.patch.object(task, "configuration_from_args", return_value=task.fixed_contract("v1", "train_04")):
        with pytest.raises(RuntimeError):
            with worker.scope(args):
                assert worker.base.apply_spawn_z_offset is not before["apply_spawn_z_offset"]
                assert worker.base._save_jpeg is not before["_save_jpeg"]
                assert worker.base.moving_capture_module(args).canonical is task.canonical
                raise RuntimeError("test")
    assert all(getattr(worker.base, name) is value for name, value in before.items())
    assert agents.BasicAgent is original_agent


def test_actual_carla_float32_delta_accumulation_retains_last_nominal_anchor_without_snapping():
    value = task.MovingTask(task.fixed_contract("v1", "train_04"))
    native_delta = 0.05000000074505806
    last_camera_timestamp = None
    for frame in range(1, 904):
        phase = "stationary_warmup" if frame <= 24 else "driving" if frame <= 773 else "stationary_tail"
        timestamp = frame*native_delta
        value.observe(frame=frame, timestamp=timestamp, phase=phase, progress=frame*.1, base_stop_reason=None,
            event_count=0, observed_free_disk_bytes=10*1024**3, native_actor_speed_mps=2.4, rear_base_CTE_m=0., observed_output_bytes=0)
        if (frame-1)%2 == 0:
            bundle = camera(frame, phase)
            bundle["timestamp"] = timestamp
            bundle["source_timestamps"] = {name: timestamp for name in task.original.CAMERAS}
            value.saved_camera(bundle)
            if bundle["moving_task_selected"]: last_camera_timestamp = timestamp
    assert value.result(tail_states=130)["task_complete"]
    assert len(value.selected) == 302 and value.last["frame"] == 645
    assert value.last["timestamp_ns"] == round(last_camera_timestamp*1e9)
    assert value.last["timestamp_ns"]-value.first["timestamp_ns"] > 30_100_000_000
    assert value.history_support["camera_observations"][-1]["timestamp_ns"]-value.history_support["camera_observations"][0]["timestamp_ns"] > 900_000_000


def test_pure_reconstruction_reproduces_all_fixed_native_inputs_and_membership():
    value, states, cameras = complete_task()
    route = [dict(x=0., y=0., z=0., distance_m=0., vad_command=3), dict(x=1000., y=0., z=0., distance_m=1000., vad_command=3)]
    for row in states:
        row.update(x=row["frame"]*.1, y=0., world_velocity_carla=[2.4, 0., 0.])
        row["moving_task"].update(original_termination_reason=None, observed_event_count=0)
    reconstructed = task.reconstruct(task.fixed_contract("v1", "train_04"), states, cameras, route)
    assert reconstructed.selected == value.selected and reconstructed.support == value.support
    assert reconstructed.result(tail_states=130) == value.result(tail_states=130)
    states[70]["world_velocity_carla"] = [2.8, 0., 0.]
    with pytest.raises(ValueError): task.reconstruct(task.fixed_contract("v1", "train_04"), states, cameras, route)
