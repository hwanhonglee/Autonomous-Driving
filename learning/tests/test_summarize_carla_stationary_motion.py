"""HH_260906 - Offline synthetic stationary evidence tests; never connect to CARLA or admit data."""

import copy
import hashlib
import io
import json
import math
from pathlib import Path

import pytest
from PIL import Image

from scripts.e2e import summarize_carla_stationary_motion as audit


def digest(payload):
    return hashlib.sha256(payload).hexdigest()


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def jsonl(path, rows):
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


def control(arm="service"):
    return dict(throttle=0., brake=1., steer=0., gear=0, hand_brake=arm == "parking",
                reverse=False, manual_gear_shift=False)


def row(tick, *, yaw=0., pitch=0., position=(0., 0., 0.), velocity=(0., 0., 0.), angular=0.):
    """HH_260906 - Independently encode the legacy shifted reference for a raw CARLA observation."""
    angle, incline = math.radians(yaw), math.radians(pitch)
    c, s = math.cos(angle), math.sin(angle)
    rate = -math.radians(angular)
    return dict(tick=tick, frame=1000 + tick, phase="settle" if tick <= 70 else "hold",
        timestamp=tick * .05, timestamp_ns=tick * 50_000_000, measurement_complete=True,
        actor_snapshot_transform_carla=dict(x=position[0], y=position[1], z=position[2], yaw=yaw, pitch=pitch, roll=0.),
        world_velocity_carla=list(velocity), world_acceleration_carla=[0., 0., 0.],
        world_angular_velocity_carla_deg_s=[0., 0., angular], x=position[0] - 1.425 * math.cos(incline) * c,
        y=-position[1] + 1.425 * math.cos(incline) * s, z=position[2] - 1.425 * math.sin(incline),
        yaw=math.atan2(-s, c), vx=c * velocity[0] + s * velocity[1],
        vy=s * velocity[0] - c * velocity[1] - 1.425 * rate, ax=0., ay=0., yaw_rate=rate,
        requested_control=control(), reported_control=control(), collision=[],
        observed_aux_frame_before=1000 + tick, observed_aux_frame_after=1000 + tick,
        control_alignment=dict(status="PASS", frame_binding_pass=True, expected_receipt_sequence=tick,
            expected_command_before_frame=999 + tick, observed_frame=1000 + tick,
            control_read_frame_before=1000 + tick, control_read_frame_after=1000 + tick,
            mismatched_fields=[], physical_actuation_proven=False))


def receipt(tick):
    return dict(sequence=tick, mode="acknowledged_batch", actor_id=7, response_actor_id=7,
        reason="settle" if tick <= 70 else "hold", requested_control=control(), do_tick=False,
        status="ACKNOWLEDGED", server_accepted=True, physical_actuation_proven=False,
        before_frame=999 + tick, expected_before_frame=999 + tick, after_ack_frame=999 + tick)


def plan_fixture():
    sources = sorted(audit.WORKER_SOURCES) + [f"scripts/e2e/helper_{i}.py" for i in range(8)]
    return dict(schema="carla.stationary_motion_campaign.v1", campaign_id="synthetic-only",
        declared_at_utc="2026-09-14T00:00:00Z", route_path="synthetic-route.json", route_sha256=digest(b"route"),
        input_sha256={key: digest(value) for key, value in (("route_file", b"route"), ("mapping", b"mapping"), ("calibration", b"calibration"))},
        source_sha256={name: digest(name.encode()) for name in sources}, port=2100, quality="Epic", wall_timeout_sec=300,
        cases=[dict(id=key, arm=arm) for key, arm in (("01_service_A1", "service"), ("02_parking_B1", "parking"),
            ("03_parking_B2", "parking"), ("04_service_A2", "service"))], physics_hz=20, capture_hz=10,
        settle_ticks=70, hold_ticks=400, total_ticks=470, training_data_approved=False,
        vehicle_control_approved=False, no_automatic_retry=True)


def trial_fixture(root, *, complete=False):
    """HH_260906 - One synthetic full trial or one failed prefix; unrun ABBA arms stay explicitly unrun."""
    plan = plan_fixture()
    put(root / "plan.json", plan)
    folder = root / plan["cases"][0]["id"]
    worker = folder / ("motion_probe" if complete else "motion_probe.partial")
    worker.mkdir(parents=True)
    for name in plan["source_sha256"]:
        path = folder / "provenance" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(name.encode())
    route = str((Path(audit.__file__).resolve().parents[2] / plan["route_path"]).resolve())
    owner = dict(schema="portable_e2e.owned_expert_trial.v1", capture_mode="stationary-motion",
        source_sha256=plan["source_sha256"], source_bytes_archived=True, route_sha256=plan["route_sha256"],
        route_path=route, map="C_track_1_0_7", host="127.0.0.1", worker_path="scripts/e2e/probe_carla_stationary_motion.py",
        planned_at_utc="2026-09-14T00:00:01Z", quality="Epic", port=2100, collector_wall_timeout_sec=300,
        learned_model_control=False, vehicle_control_approved=False,
        collector_argv=[str(folder / "motion_probe"), route, "--host", "127.0.0.1", "--port", "2100", "--arm", "service"])
    put(folder / "owner_plan.json", owner)
    put(folder / "owner_result.json", dict(capture_mode="stationary-motion", learned_model_control=False,
        vehicle_control_approved=False, exit_code=0 if complete else 1, source_bytes_unchanged_and_archived=True,
        source_checks={key: True for key in plan["source_sha256"]}))
    native = [row(i) for i in range(1, 471)] if complete else [row(1), dict(tick=2, frame=1002, phase="settle", measurement_complete=False, requested_control=control())]
    observations = [dict(value, stage="OBSERVED_BEFORE_CAMERA", measurement_complete=False) for value in native if "timestamp" in value]
    frames = []
    encoded = io.BytesIO()
    Image.new("RGB", (640, 360), (31, 73, 117)).save(encoded, format="JPEG", quality=95)
    payload = encoded.getvalue()
    if complete:
        (worker / "images").mkdir()
        for tick in range(2, 471, 2):
            images = {}
            for camera in audit.CAMERAS:
                name = f"images/{tick}_{camera}.jpg"
                (worker / name).write_bytes(payload)
                images[camera] = dict(path=name, timestamp=tick * .05, sha256=digest(payload))
            frames.append(dict(tick=tick, frame=1000 + tick, timestamp=tick * .05, complete=True, images=images))
    for name, rows in (("native_states.jsonl", native), ("native_observations.jsonl", observations),
                       ("camera_frames.jsonl", frames), ("control_receipts.jsonl", [receipt(i) for i in range(1, len(native) + 1)])):
        jsonl(worker / name, rows)
    for name, value in (("route.json", b"route"), ("mapping.yaml", b"mapping"), ("calibration.yaml", b"calibration")):
        (worker / name).write_bytes(value)
    manifest = dict(schema="carla.stationary_motion_probe.v1", arm="service", status="COMPLETE" if complete else "FAIL",
        training_data_approved=False, vehicle_approved=False, scheduled_ticks=470, settle_ticks=70, hold_ticks=400,
        physics_hz=20, camera_hz=10, camera_bundle_observed_every_native_tick=True, requested_control=control(),
        source_sha256_before={key: plan["source_sha256"][key] for key in audit.WORKER_SOURCES},
        source_sha256_after={key: plan["source_sha256"][key] for key in audit.WORKER_SOURCES},
        input_sha256_before=plan["input_sha256"], input_sha256_after=plan["input_sha256"],
        world_settings=dict(synchronous_mode=True, no_rendering_mode=False, substepping=True,
            fixed_delta_seconds=.05, max_substep_delta_time=.01, max_substeps=10),
        artifact_sha256={path.name: digest(path.read_bytes()) for path in worker.iterdir() if path.is_file()},
        state_count=len(native), saved_camera_anchor_count=len(frames), partial_camera_anchor_count=0,
        town="C_track_1_0_7", route_sha256=plan["route_sha256"], wheelbase_m=2.85, cleanup_errors=[], collision_events=[],
        owned_actor_ids=list(range(7, 15)), destroy_rpc_confirmed_actor_ids=list(range(7, 15)))
    put(worker / "manifest.json", manifest)
    if complete:
        put(folder / "owner_started.json", dict(server_pid=123, server_pgid=123, route_sha256=plan["route_sha256"],
            map="C_track_1_0_7", port=2100, quality="Epic", learned_model_control=False))
        (folder / "server.log").write_bytes(b"synthetic log\n")
        for stage in ("ready", "after_capture", "stopped"):
            stopped = stage == "stopped"
            put(folder / "lifecycle" / f"{stage}.json", dict(schema_version=1, status="PASS", error=None,
                stage=stage, generation_id="expert_123", owner_pid=123, owner_pgid=123, host="127.0.0.1", port=2100,
                expected_map="C_track_1_0_7", read_only=True, mode="stopped" if stopped else "running",
                owner_process_state=None if stopped else "S", port_released=stopped, active_map_basename="C_track_1_0_7",
                snapshot_frame=1500 if stage == "after_capture" else 1000,
                snapshot_elapsed_seconds=24. if stage == "after_capture" else 0.,
                rpc_sequence=[] if stopped else ["get_world", "world.get_map", "world.get_snapshot"],
                server_log=dict(size_bytes=14, sha256=digest(b"synthetic log\n"))))
    return plan, worker, manifest


def test_handedness_and_distinct_virtual_reference():
    sample = audit.motion_sample(row(1, yaw=90., pitch=10., position=(1., 2., 3.), velocity=(2., 3., 0.), angular=10.))
    assert sample["actor"] == (1., -2.)
    assert sample["actor_velocity"] == (2., -3.)
    assert sample["base"][1] == pytest.approx(-2. + 1.425 * math.cos(math.radians(10)))
    assert sample["base_velocity"][0] == pytest.approx(2. + 1.425 * math.radians(10))
    assert sample["yaw"] == pytest.approx(-math.pi / 2)


def test_midpoint_integral_both_offsets_phase_crossing_and_zero_residual():
    samples = [audit.motion_sample(row(i, position=((i * .05) ** 2, 0., 0.), velocity=(2 * i * .05, 0., 0.))) for i in range(68, 73)]
    summary, intervals = audit.summarize_motion(samples)
    assert [len(value) for value in intervals.values()] == [4, 2, 1]
    assert summary["native_50ms"]["hold"]["phase_crossing_interval_count"] == 1
    assert sum(summary["native_50ms"][phase]["interval_count"] for phase in audit.PHASES) == 4
    for series in summary.values():
        assert series["all"]["references"]["actor"]["displacement_minus_velocity_integral_m"]["maximum"] < 1e-14
    points = [audit.motion_sample(row(i)) for i in range(1, 4)]
    points[1]["actor_velocity"] = (1., 0.)
    assert audit.interval_record(points)["references"]["actor"]["trapezoidal_velocity_integral_xy_m"] == pytest.approx([.05, 0.])


def test_yaw_wrap_and_empty_denominators():
    points = [audit.motion_sample(row(i + 1, yaw=yaw)) for i, yaw in enumerate((179., -179., 179.))]
    summary, _ = audit.summarize_motion(points)
    assert summary["native_50ms"]["all"]["yaw_total_variation_rad"] == pytest.approx(math.radians(4))
    assert audit.describe([]) == dict(count=0, maximum=None, rms=None)
    assert audit.describe([3., 4.])["rms"] == pytest.approx(math.sqrt(12.5))


def test_native_timestamp_representations_must_agree():
    native = row(1)
    native["timestamp_ns"] += 1
    with pytest.raises(ValueError, match="timestamp representations differ"):
        audit.motion_sample(native)


@pytest.mark.parametrize("value", [True, float("nan"), float("inf"), "0"])
def test_nonfinite_and_boolean_numbers_rejected(value):
    with pytest.raises(ValueError):
        audit.number(value)


@pytest.mark.parametrize("relative", ["../outside", "/tmp/file", "a/../file", "./file", "a//file"])
def test_unsafe_input_paths_rejected(tmp_path, relative):
    with pytest.raises(ValueError):
        audit.checked_path(tmp_path, relative)


def test_symlinks_duplicates_and_mutation_rejected(tmp_path):
    (tmp_path / "file").write_bytes(b"before")
    (tmp_path / "link").symlink_to(tmp_path / "file")
    with pytest.raises(ValueError): audit.checked_path(tmp_path, "link")
    with pytest.raises(ValueError): audit.strict_json('{"x":1,"x":2}')
    pins = {}
    audit.read_bytes(tmp_path, "file", pins)
    (tmp_path / "file").write_bytes(b"after")
    with pytest.raises(ValueError): audit.read_bytes(tmp_path, "file", pins)


@pytest.mark.parametrize("change", ["order", "count", "approval", "source", "settings"])
def test_fixed_plan_cannot_be_relaxed(change):
    plan = plan_fixture()
    if change == "order": plan["cases"].reverse()
    if change == "count": plan["total_ticks"] = 469
    if change == "approval": plan["training_data_approved"] = True
    if change == "source": plan["source_sha256"].pop(next(iter(plan["source_sha256"])))
    if change == "settings": plan["physics_hz"] = True
    with pytest.raises(ValueError): audit.validate_plan(plan)


def test_failed_incomplete_prefix_preserves_every_planned_arm_and_raw_count(tmp_path):
    root = tmp_path / "campaign"
    trial_fixture(root)
    result = audit.analyze(root, tmp_path / "summary", render=False)
    assert result["status"] == "PARTIAL_OR_FAILED_NOT_ADMITTED"
    assert result["planned_case_count"] == 4 and result["complete_case_count"] == 0
    assert result["native_state_count"] == 2 and result["camera_anchor_count"] == 0
    failed = result["cases"][0]
    assert failed["status"] == "FAIL" and failed["motion_sample_count"] == 1
    assert failed["motion_unavailable"][0]["tick"] == 2
    assert len(failed["protocol_issues"]) == 1
    assert [case["status"] for case in result["cases"]] == ["FAIL", "NOT_RUN", "NOT_RUN", "NOT_RUN"]
    assert result["scope"]["noise_tolerance_established"] is False


def test_early_failure_without_settings_keeps_zero_denominators(tmp_path):
    root = tmp_path / "campaign"
    plan, worker, manifest = trial_fixture(root)
    for name in ("native_states.jsonl", "native_observations.jsonl", "camera_frames.jsonl", "control_receipts.jsonl"):
        jsonl(worker / name, [])
        manifest["artifact_sha256"][name] = digest(b"")
    manifest.pop("world_settings")
    manifest["state_count"] = 0
    put(worker / "manifest.json", manifest)
    result, samples, cameras, _ = audit.validate_trial(root, plan["cases"][0], plan, {})
    assert result["status"] == "FAIL" and result["native_state_count"] == result["camera_anchor_count"] == 0
    assert samples == cameras == [] and result["protocol_issues"]


def test_complete_counts_receipts_images_and_actual_full_fov_render(tmp_path):
    root = tmp_path / "campaign"
    plan, _, _ = trial_fixture(root, complete=True)
    pins = {}
    result, samples, cameras, worker = audit.validate_trial(root, plan["cases"][0], plan, pins)
    assert (result["native_state_count"], result["camera_anchor_count"], result["decoded_image_count"]) == (470, 235, 1410)
    assert result["motion"]["native_50ms"]["hold"]["interval_count"] == 400
    assert result["motion"]["derived_100ms_offset0"]["all"]["interval_count"] == 234
    output = tmp_path / "render"
    output.mkdir()
    audit.render_trial(root, output, result, samples, cameras, worker, pins)
    for suffix in ("measured_traces", "six_camera_trace"):
        with Image.open(output / f"01_service_A1_{suffix}.png") as image:
            image.load()
            assert image.width >= 1500 and image.height >= 900


@pytest.mark.parametrize("change", ["mask", "receipt", "partial", "substeps", "lifecycle", "stale_lifecycle", "ego", "cleanup"])
def test_complete_evidence_cannot_hide_protocol_failure(tmp_path, change):
    root = tmp_path / "campaign"
    plan, worker, manifest = trial_fixture(root, complete=True)
    if change == "mask":
        frames = [json.loads(line) for line in (worker / "camera_frames.jsonl").read_text().splitlines()]
        frames[-1]["complete"] = 1
        jsonl(worker / "camera_frames.jsonl", frames)
    if change == "receipt":
        rows = [receipt(i) for i in range(1, 471)]
        rows[3]["after_ack_frame"] += 1
        jsonl(worker / "control_receipts.jsonl", rows)
    if change == "partial":
        rows = [row(i) for i in range(1, 471)]
        rows[-1]["measurement_complete"] = False
        jsonl(worker / "native_states.jsonl", rows)
    if change == "substeps": manifest["world_settings"]["max_substeps"] = 20
    if change == "lifecycle":
        path = worker.parent / "lifecycle/stopped.json"
        value = json.loads(path.read_text())
        value["port_released"] = False
        put(path, value)
    if change == "stale_lifecycle":
        path = worker.parent / "lifecycle/after_capture.json"
        value = json.loads(path.read_text())
        value.update(snapshot_frame=1000, snapshot_elapsed_seconds=0.)
        put(path, value)
    if change == "ego":
        manifest["owned_actor_ids"][0] = 99
        manifest["destroy_rpc_confirmed_actor_ids"][0] = 99
    if change == "cleanup": manifest["destroy_rpc_confirmed_actor_ids"].pop()
    manifest["artifact_sha256"] = {name: digest((worker / name).read_bytes()) for name in manifest["artifact_sha256"]}
    put(worker / "manifest.json", manifest)
    with pytest.raises(ValueError, match="ACK actor is not owned ego" if change == "ego" else None):
        audit.validate_trial(root, plan["cases"][0], plan, {})


def test_output_must_be_fresh_and_disjoint(tmp_path):
    root = tmp_path / "campaign"
    trial_fixture(root)
    for output in (root, root / "summary", tmp_path):
        with pytest.raises(ValueError): audit.analyze(root, output, render=False)
