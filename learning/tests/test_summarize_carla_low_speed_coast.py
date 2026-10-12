"""HH_260906 - Synthetic offline coast evidence; no CARLA process, trained model, or dataset writes."""

import copy
import hashlib
import json
import math
from pathlib import Path

import pytest
from PIL import Image

from scripts.e2e import summarize_carla_low_speed_coast as audit
from test_summarize_carla_stationary_motion import control, digest, jsonl, put, receipt, row, trial_fixture


def native_rows(launch=2, coast=900):
    rows = []
    for tick in range(1, 71 + launch + coast):
        phase = "settle" if tick <= 70 else "launch" if tick <= 70 + launch else "coast"
        local = tick if phase == "settle" else tick - 70 if phase == "launch" else tick - 70 - launch
        speed = .5 if phase == "launch" and local == launch else .2 if phase == "launch" else 0.
        value = row(tick, velocity=(speed, 0., 0.))
        commanded = dict(control(), throttle=.13 if phase == "launch" else 0., brake=1. if phase == "settle" else 0.)
        value.update(phase=phase, phase_tick=local, requested_control=commanded, reported_control=commanded.copy(),
                     planar_speed_mps=speed, net_planar_motion_m=0., route_cte_m=0., route_progress_m=0.)
        rows.append(value)
    return rows


def coast_fixture(root, complete=True, launch=2):
    """HH_260906 - Reuse only synthetic owner/JPEG fixture helpers; do not execute the old or new collectors."""
    plan, old_worker, old_manifest = trial_fixture(root, complete=complete)
    old_folder = old_worker.parent
    folder = root / audit.CASES[0]["id"]
    old_folder.rename(folder)
    old_worker = folder / old_worker.name
    worker = folder / ("coast_probe" if complete else "coast_probe.partial")
    old_worker.rename(worker)
    plan.update(schema="carla.low_speed_coast_campaign.v1", cases=copy.deepcopy(audit.CASES), map="C_track_1_0_7", **audit.FIXED)
    plan["source_sha256"][audit.WORKER] = digest(audit.WORKER.encode())
    path = folder / "provenance" / audit.WORKER
    path.write_bytes(audit.WORKER.encode())
    put(root / "plan.json", plan)
    owner = json.loads((folder / "owner_plan.json").read_text())
    owner.update(capture_mode="low-speed-coast", worker_path=audit.WORKER, source_sha256=plan["source_sha256"])
    owner["collector_argv"] = [str(folder / "coast_probe"), owner["route_path"], "--host", "127.0.0.1", "--port", "2100"]
    put(folder / "owner_plan.json", owner)
    owned_result = json.loads((folder / "owner_result.json").read_text())
    owned_result.update(capture_mode="low-speed-coast", source_checks={key: True for key in plan["source_sha256"]})
    put(folder / "owner_result.json", owned_result)
    native = native_rows(launch) if complete else []
    receipts = []
    for value in native:
        ack = receipt(value["tick"])
        ack.update(reason=value["phase"], requested_control=value["requested_control"])
        receipts.append(ack)
    observations = [dict(value, stage="OBSERVED_BEFORE_CAMERA", measurement_complete=False) for value in native]
    cameras = []
    if complete:
        existing = json.loads((worker / "camera_frames.jsonl").read_text().splitlines()[0])
        payload = (worker / next(iter(existing["images"].values()))["path"]).read_bytes()
        for tick in range(2, len(native) + 1, 2):
            images = {}
            for camera in audit.CAMERAS:
                name = f"images/{tick}_{camera}.jpg"
                (worker / name).write_bytes(payload)
                images[camera] = dict(path=name, timestamp=tick * .05, sha256=digest(payload))
            cameras.append(dict(tick=tick, frame=1000 + tick, timestamp=tick * .05, timestamp_ns=tick * 50_000_000, complete=True, images=images))
    for name, values in (("native_states.jsonl", native), ("native_observations.jsonl", observations),
                         ("control_receipts.jsonl", receipts), ("camera_frames.jsonl", cameras)):
        jsonl(worker / name, values)
    manifest = old_manifest
    manifest.update(schema="carla.low_speed_coast_probe.v1", maximum_native_ticks=1130, **{key: value for key, value in audit.FIXED.items()
        if key in ("settle_ticks", "max_launch_ticks", "coast_ticks", "launch_throttle", "target_speed_mps")},
        vehicle_control_approved=False, learned_model_control=False, data_admission_performed=False,
        operational_limits=dict(max_speed_mps=.75, minimum_body_vx_mps=-.1, max_net_motion_m=10., max_route_cte_m=1.,
            settled_max_speed_mps=.1, settled_max_net_motion_m=.5),
        state_count=len(native), phase_counts={phase: sum(value["phase"] == phase for value in native) for phase in audit.PHASES},
        saved_camera_anchor_count=len(cameras), controls_by_phase={phase: dict(control(), throttle=.13 if phase == "launch" else 0.,
            brake=1. if phase == "settle" else 0.) for phase in audit.PHASES},
        source_sha256_before={key: plan["source_sha256"][key] for key in audit.WORKER_SOURCES},
        source_sha256_after={key: plan["source_sha256"][key] for key in audit.WORKER_SOURCES},
        motion_origin_source="REQUESTED_SPAWN_TRANSFORM_NOT_PRETICK_ACTOR_API", requested_spawn_transform_carla=dict(x=0., y=0., z=0.),
        motion_origin_transform_carla=dict(x=0., y=0., z=0.), phase_transitions=[], completion_reason="full_900_tick_zero_pedal_coast_observed")
    if complete:
        for source_tick, before, after in ((70, "settle", "launch"), (70 + launch, "launch", "coast")):
            source = native[source_tick - 1]
            manifest["phase_transitions"].append(dict(from_phase=before, to_phase=after, source_tick=source_tick,
                source_phase_tick=source["phase_tick"], source_frame=source["frame"], source_timestamp=source["timestamp"],
                source_planar_speed_mps=source["planar_speed_mps"], effective_tick=source_tick + 1,
                next_requested_control=native[source_tick]["requested_control"], physical_transition_time_proven=False))
        manifest["launch_handoff"] = manifest["phase_transitions"][-1]
        settings = dict(manifest["world_settings"], deterministic_ragdolls=True, spectator_as_ego=True,
            max_culling_distance=0., tile_stream_distance=3000., actor_active_distance=2000.)
        for key in ("world_settings", "world_settings_before", "world_settings_requested", "world_settings_restored"):
            manifest[key] = settings.copy()
        weather = {name: 0. for name in ("cloudiness", "dust_storm", "fog_density", "fog_distance", "fog_falloff",
            "mie_scattering_scale", "precipitation", "precipitation_deposits", "rayleigh_scattering_scale", "scattering_intensity",
            "sun_altitude_angle", "sun_azimuth_angle", "wetness", "wind_intensity")}
        for key in ("weather_before", "weather_requested", "weather_actual", "weather_restored"): manifest[key] = weather.copy()
        manifest["weather"] = "ClearNoon"
        manifest["preflight"] = dict(status="PASS", foreign_dynamic_actors_absent=True, frame_before=1000, frame_after=1000,
            corridor_samples=[dict(status="PASS")], obstacle_rays=[dict(lateral_offset_m=lateral, height_above_road_m=height,
            status="CLEAR_SAMPLED_RAY", hits=[]) for lateral in (0., -.75, .75) for height in (.5, 1.25)])
        lifecycle = folder / "lifecycle/after_capture.json"
        value = json.loads(lifecycle.read_text())
        value.update(snapshot_frame=2300, snapshot_elapsed_seconds=70.)
        put(lifecycle, value)
    else:
        manifest.pop("world_settings")
        manifest["preflight"] = dict(status="FAIL", error="synthetic failed lane lookup")
    manifest["artifact_sha256"] = {name: digest((worker / name).read_bytes()) for name in audit.ARTIFACTS}
    put(worker / "manifest.json", manifest)
    return plan, worker, manifest


def test_phase_schedule_first_threshold_and_maximum_launch_inclusive():
    for launch in (1, 2, 160):
        counts, threshold = audit.check_phase_sequence(native_rows(launch), True)
        assert counts == dict(settle=70, launch=launch, coast=900) and threshold == 70 + launch
    rows = native_rows(3)
    rows[70]["world_velocity_carla"][0] = .5
    with pytest.raises(ValueError, match="first-threshold"):
        audit.check_phase_sequence(rows, True)


def test_continuous_dwell_requires_elapsed_two_seconds_and_reports_first_crossing_separately():
    points = [audit.sample(value) for value in native_rows(2, 50)]
    coast = points[72:]
    coast[2]["actor_velocity"] = (.2, 0.)
    observed = audit.coast_observation(points)
    assert observed["first_observed_speed_at_or_below_0.1_tick"] == 73
    assert observed["first_qualifying_low_speed_tick"] == 76
    assert observed["dwell_confirmed_tick"] == 116
    assert observed["dwell_confirmed_time_s"] - observed["first_qualifying_low_speed_time_s"] == 2.
    assert audit.coast_observation(points[:72 + 40])["status"] == "LOW_SPEED_DWELL_NOT_OBSERVED"
    points[115]["timestamp_ns"] -= 1
    assert audit.coast_observation(points)["dwell_confirmed_tick"] == 117


def test_interval_parity_phase_crossings_speed_bins_and_native_midpoint():
    points = [audit.sample(value) for value in native_rows(2, 5)]
    stats, intervals = audit.summarize_motion(points)
    assert len(intervals["native_50ms"]) == 76
    for series in stats.values():
        phases = series["phases"]
        assert sum(phases[phase]["interval_count"] for phase in audit.PHASES) == phases["all"]["interval_count"]
        assert phases["all"]["phase_crossing_interval_count"] == 2
        assert sum(group["interval_count"] for group in series["coast_max_native_speed_bins"].values()) == phases["coast"]["interval_count"]
    triple = points[70:73]
    record = audit.stationary.interval_record(triple)
    assert record["references"]["actor"]["trapezoidal_velocity_integral_xy_m"][0] == pytest.approx(.03)


def test_failed_zero_state_preflight_retained_with_second_notrun(tmp_path):
    root = tmp_path / "campaign"
    coast_fixture(root, False)
    result = audit.analyze(root, tmp_path / "summary", render=False)
    assert result["status"] == "PARTIAL_OR_FAILED_NOT_ADMITTED"
    assert [row["status"] for row in result["cases"]] == ["FAIL", "NOT_RUN"]
    assert result["native_state_count"] == result["camera_anchor_count"] == 0
    assert result["cases"][0]["preflight_status"] == "FAIL" and result["cases"][0]["protocol_issues"]


def test_failed_incomplete_native_row_is_counted_without_gap_bridging(tmp_path):
    root = tmp_path / "campaign"
    plan, worker, manifest = coast_fixture(root, False)
    native = native_rows()[:2]
    native[1] = {key: native[1][key] for key in ("tick", "frame", "phase", "phase_tick", "requested_control")}
    native[1]["measurement_complete"] = False
    jsonl(worker / "native_states.jsonl", native)
    jsonl(worker / "native_observations.jsonl", [dict(native[0], measurement_complete=False, stage="OBSERVED_BEFORE_CAMERA")])
    jsonl(worker / "control_receipts.jsonl", [receipt(1), receipt(2)])
    manifest.update(state_count=2, phase_counts=dict(settle=2, launch=0, coast=0))
    manifest["artifact_sha256"] = {name: digest((worker / name).read_bytes()) for name in audit.ARTIFACTS}
    put(worker / "manifest.json", manifest)
    result, samples, _, _ = audit.validate_trial(root, audit.CASES[0], plan, {})
    assert result["status"] == "FAIL" and result["native_state_count"] == 2
    assert len(samples) == result["motion_sample_count"] == 1 and result["motion_unavailable"][0]["tick"] == 2


def test_complete_odd_final_native_full_jpegs_and_predeclared_camera_panels(tmp_path):
    root = tmp_path / "campaign"
    plan, _, _ = coast_fixture(root, launch=1)
    result, samples, cameras, worker = audit.validate_trial(root, audit.CASES[0], plan, {})
    assert (result["native_state_count"], result["camera_anchor_count"], result["decoded_image_count"]) == (971, 485, 2910)
    assert result["camera_anchor_ticks"] == dict(coast_entry=72, scheduled_final=970)
    assert result["final_camera_native_offset_ticks"] == 1
    assert result["coast_observation"]["observed_elapsed_from_control_boundary_s"] == 45.
    output = tmp_path / "render"
    output.mkdir()
    audit.render_trial(root, output, result, samples, cameras, worker, {})
    for suffix in ("measured_traces", "coast_entry_six_camera_trace", "scheduled_final_six_camera_trace"):
        with Image.open(output / f"01_coast_repeat_1_{suffix}.png") as image:
            image.load()
            assert image.width >= 1500 and image.height >= 900


@pytest.mark.parametrize("change", ["ack", "boolean_ack", "camera_mask", "camera_time", "early_coast_end", "restore", "preflight", "origin", "source", "handoff", "retained_error", "retained_safety", "settle_speed", "settle_motion"])
def test_complete_cannot_hide_changed_evidence(tmp_path, change):
    root = tmp_path / "campaign"
    plan, worker, manifest = coast_fixture(root)
    if change in ("ack", "boolean_ack"):
        values = [json.loads(line) for line in (worker / "control_receipts.jsonl").read_text().splitlines()]
        if change == "ack": values[71]["after_ack_frame"] -= 1
        else: values[0]["sequence"] = True
        jsonl(worker / "control_receipts.jsonl", values)
    if change in ("camera_mask", "camera_time"):
        values = [json.loads(line) for line in (worker / "camera_frames.jsonl").read_text().splitlines()]
        if change == "camera_mask": values[-1]["complete"] = 1
        else: values[-1]["timestamp_ns"] += 1
        jsonl(worker / "camera_frames.jsonl", values)
    if change == "early_coast_end":
        values = native_rows(2, 899)
        jsonl(worker / "native_states.jsonl", values)
        manifest["state_count"] = len(values)
    if change == "restore": manifest["world_settings_restored"]["tile_stream_distance"] += 1.
    if change == "preflight": manifest["preflight"]["obstacle_rays"][0]["hits"] = [dict(label="Wall")]
    if change == "origin": manifest["motion_origin_transform_carla"]["x"] = 196.
    if change == "source": manifest["source_sha256_after"][audit.WORKER] = "0" * 64
    if change == "handoff": manifest["phase_transitions"][1]["source_frame"] -= 1
    # HH_260906 - Re-sign synthetic evidence to prove the semantic bounds, not stale file hashes, reject it.
    if change == "retained_error": manifest["error"] = "synthetic retained exception"
    if change == "retained_safety": manifest["safety_failure"] = "settled_speed_exceeded"
    if change in ("settle_speed", "settle_motion"):
        values = native_rows()
        changed = values[69]
        if change == "settle_speed":
            changed.update(vx=.2, planar_speed_mps=.2, world_velocity_carla=[.2, 0., 0.])
        else:
            changed["actor_snapshot_transform_carla"]["x"] = .6
            changed.update(x=changed["x"] + .6, net_planar_motion_m=.6)
        jsonl(worker / "native_states.jsonl", values)
        jsonl(worker / "native_observations.jsonl", [dict(value, stage="OBSERVED_BEFORE_CAMERA", measurement_complete=False) for value in values])
        manifest["phase_transitions"][0]["source_planar_speed_mps"] = changed["planar_speed_mps"]
    manifest["artifact_sha256"] = {name: digest((worker / name).read_bytes()) for name in audit.ARTIFACTS}
    put(worker / "manifest.json", manifest)
    with pytest.raises(ValueError): audit.validate_trial(root, audit.CASES[0], plan, {})


def test_nonfinite_speed_and_disjoint_fresh_outputs(tmp_path):
    value = native_rows(1, 1)[-1]
    value["world_velocity_carla"][0] = float("nan")
    with pytest.raises(ValueError): audit.sample(value)
    root = tmp_path / "campaign"
    coast_fixture(root, False)
    for output in (root, root / "summary", tmp_path):
        with pytest.raises(ValueError): audit.analyze(root, output, render=False)
