"""HH_260906 - Synthetic four-phase coast/brake evidence; never execute a CARLA worker or change old evidence."""

import copy
import json
import math

import pytest
from PIL import Image

from scripts.e2e import summarize_carla_coast_brake_transition as audit
from test_summarize_carla_low_speed_coast import coast_fixture, native_rows
from test_summarize_carla_stationary_motion import digest, jsonl, put, receipt


def brake_rows(launch=2, arm="coast"):
    rows = native_rows(launch)
    for row in rows:
        if row["phase"] == "coast" and row["phase_tick"] > 600:
            row.update(phase="tail", phase_tick=row["phase_tick"] - 600)
            row["requested_control"] = dict(row["requested_control"], brake=1. if arm == "service" else 0.)
            row["reported_control"] = row["requested_control"].copy()
    return rows


def brake_fixture(root, *, arm="coast", complete=True, launch=2):
    """HH_260906 - Only temporary synthetic fixtures are extended; the new validator reads its own unmodified protocol."""
    plan, old_worker, manifest = coast_fixture(root, complete=complete, launch=launch)
    case = audit.CASES[0 if arm == "coast" else 1]
    old_folder = old_worker.parent
    folder = root / case["id"]
    old_folder.rename(folder)
    old_worker = folder / old_worker.name
    worker = folder / ("brake_probe" if complete else "brake_probe.partial")
    old_worker.rename(worker)
    plan.update(schema="carla.coast_brake_transition_campaign.v1", cases=copy.deepcopy(audit.CASES), **audit.FIXED)
    plan["source_sha256"][audit.WORKER] = digest(audit.WORKER.encode())
    (folder / "provenance" / audit.WORKER).write_bytes(audit.WORKER.encode())
    put(root / "plan.json", plan)
    owner = json.loads((folder / "owner_plan.json").read_text())
    owner.update(capture_mode="coast-brake-transition", worker_path=audit.WORKER, source_sha256=plan["source_sha256"])
    owner["collector_argv"] = [str(folder / "brake_probe"), owner["route_path"], "--host", "127.0.0.1", "--port", "2100", "--arm", arm]
    put(folder / "owner_plan.json", owner)
    result = json.loads((folder / "owner_result.json").read_text())
    result.update(capture_mode="coast-brake-transition", source_checks={key: True for key in plan["source_sha256"]})
    put(folder / "owner_result.json", result)
    rows = brake_rows(launch, arm) if complete else []
    receipts = []
    for row in rows:
        value = receipt(row["tick"])
        value.update(reason=row["phase"], requested_control=row["requested_control"])
        receipts.append(value)
    for name, values in (("native_states.jsonl", rows), ("native_observations.jsonl", [dict(row,
        stage="OBSERVED_BEFORE_CAMERA", measurement_complete=False) for row in rows]), ("control_receipts.jsonl", receipts)):
        jsonl(worker / name, values)
    manifest.update(schema="carla.coast_brake_transition_probe.v1", arm=arm, coast_ticks=600, tail_ticks=300,
        tail_entry_max_speed_mps=.1, phase_counts={phase: sum(row["phase"] == phase for row in rows) for phase in audit.PHASES},
        source_sha256_before={key: plan["source_sha256"][key] for key in audit.WORKER_SOURCES},
        source_sha256_after={key: plan["source_sha256"][key] for key in audit.WORKER_SOURCES},
        completion_reason="full_600_tick_coast_and_300_tick_tail_observed")
    manifest["controls_by_phase"]["tail"] = dict(manifest["controls_by_phase"]["coast"], brake=1. if arm == "service" else 0.)
    if complete:
        source = rows[70 + launch + 600 - 1]
        transition = dict(from_phase="coast", to_phase="tail", source_tick=source["tick"], source_phase_tick=600,
            source_frame=source["frame"], source_timestamp=source["timestamp"], source_planar_speed_mps=source["planar_speed_mps"],
            effective_tick=source["tick"] + 1, next_requested_control=rows[source["tick"]]["requested_control"], physical_transition_time_proven=False)
        manifest["phase_transitions"].append(transition)
        manifest["tail_handoff"] = transition.copy()
        manifest["tail_entry_check"] = dict(source_tick=source["tick"], source_frame=source["frame"], source_timestamp=source["timestamp"],
            source_planar_speed_mps=source["planar_speed_mps"], maximum_speed_mps=.1, status="PASS", applies_to_both_arms=True)
    manifest["artifact_sha256"] = {name: digest((worker / name).read_bytes()) for name in audit.ARTIFACTS}
    put(worker / "manifest.json", manifest)
    return plan, worker, manifest, case


def test_exact_four_phase_schedule_and_boundary_interval_are_retained():
    for launch in (1, 2, 160):
        rows = brake_rows(launch)
        counts, threshold = audit.check_phase_sequence(rows, True)
        assert counts == dict(settle=70, launch=launch, coast=600, tail=300)
        assert threshold == 70 + launch
    points = [audit.sample(row) for row in brake_rows()]
    source = 70 + 2 + 600 - 1
    for i, point in enumerate(points[source + 1:], 1):
        point["actor"] = (min(i, 2) * .1, 0.)
    motion, _ = audit.summarize_motion(points)
    tail = motion["native_50ms"]["phases"]["tail"]
    assert tail["interval_count"] == 300 and tail["phase_crossing_interval_count"] == 1
    assert tail["references"]["actor"]["net_displacement_m"] == pytest.approx(.2)
    assert tail["sample_net_drift_m"]["actor"] == pytest.approx(.1)


def test_signed_velocity_parity_is_not_scalar_speed_integral():
    points = [audit.sample(row) for row in brake_rows()]
    for point in points:
        if point["phase"] == "tail":
            velocity = -.01 if point["tick"] % 2 == 0 else .02
            point.update(actor_velocity=(velocity, 0.), body_vx=velocity, actor=(point["tick"] * .00025, 0.))
    stats = audit.tail_statistics(points)
    assert stats["full_10s_window_available"] and stats["native_sample_count"] == 201
    assert stats["signed_body_vx"]["sign_change_count"] == 200
    assert stats["signed_body_vx"]["by_global_tick_parity"]["0"]["mean_mps"] == pytest.approx(-.01)
    actor = stats["references"]["actor"]
    assert actor["vector_velocity_integral_norm_m"] == pytest.approx(.05)
    assert actor["scalar_speed_integral_m"] == pytest.approx(.15)
    assert actor["aggregate_residual_norm_m"] < 1e-14
    assert stats["actor_forward_position_increments"]["negative_count"] == 0


def test_prefix_comparison_is_descriptive_and_excludes_tail_control():
    trials = []
    for case in audit.CASES:
        trials.append((dict(status="COMPLETE", case_id=case["id"]), [audit.sample(row) for row in brake_rows(arm=case["arm"])], [], None))
    assert audit.pre_tail_comparison(trials)["status"] == "OBSERVED_MATCH"
    trials[1][1][0]["comparison_fields"]["vx"] = .01
    result = audit.pre_tail_comparison(trials)
    assert result["status"] == "OBSERVED_DIFFERENCE" and result["is_execution_or_admission_gate"] is False


def test_zero_state_failure_retains_all_four_cases_and_no_tail_measurement(tmp_path):
    root = tmp_path / "campaign"
    brake_fixture(root, complete=False)
    result = audit.analyze(root, tmp_path / "summary", render=False)
    assert [case["status"] for case in result["cases"]] == ["FAIL", "NOT_RUN", "NOT_RUN", "NOT_RUN"]
    assert result["native_state_count"] == 0 and result["cases"][0]["tail_last10s"]["status"] == "TAIL_NOT_OBSERVED"


def test_service_arm_full_counts_boundary_duration_and_real_jpeg_panels(tmp_path):
    root = tmp_path / "campaign"
    plan, _, _, case = brake_fixture(root, arm="service", launch=1)
    result, samples, cameras, worker = audit.validate_trial(root, case, plan, {})
    assert (result["native_state_count"], result["camera_anchor_count"], result["decoded_image_count"]) == (971, 485, 2910)
    assert result["tail_from_command_boundary"]["observed_elapsed_s"] == 15.
    assert result["tail_from_command_boundary"]["observed_interval_count"] == 300
    assert result["camera_anchor_ticks"] == dict(tail_entry=672, scheduled_final=970)
    output = tmp_path / "render"
    output.mkdir()
    audit.render_trial(root, output, result, samples, cameras, worker, {})
    for suffix in ("measured_traces", "tail_entry_six_camera_trace", "scheduled_final_six_camera_trace"):
        with Image.open(output / f"{case['id']}_{suffix}.png") as image:
            image.load()
            assert image.width >= 1500 and image.height >= 900


@pytest.mark.parametrize("change", ["tail_brake", "entry_speed", "tail_short", "tail_handoff", "entry_receipt", "settle_speed", "error", "safety", "camera", "archive"])
def test_complete_protocol_cannot_be_weakened(tmp_path, change):
    root = tmp_path / "campaign"
    plan, worker, manifest, case = brake_fixture(root)
    rows = brake_rows()
    if change == "tail_brake": rows[712]["reported_control"]["brake"] = 1.
    if change in ("entry_speed", "settle_speed"):
        value = rows[70 + 2 + 600 - 1 if change == "entry_speed" else 69]
        value.update(vx=.2, planar_speed_mps=.2, world_velocity_carla=[.2, 0., 0.])
        manifest["phase_transitions"][2 if change == "entry_speed" else 0]["source_planar_speed_mps"] = .2
        if change == "entry_speed":
            manifest["tail_handoff"]["source_planar_speed_mps"] = .2
            manifest["tail_entry_check"]["source_planar_speed_mps"] = .2
    if change == "tail_short":
        rows.pop()
        manifest["state_count"] -= 1
        manifest["phase_counts"]["tail"] -= 1
    if change == "tail_handoff": manifest["phase_transitions"][2]["source_frame"] -= 1
    if change == "entry_receipt": manifest["tail_entry_check"]["applies_to_both_arms"] = False
    if change == "error": manifest["error"] = "retained failure"
    if change == "safety": manifest["safety_failure"] = "retained safety failure"
    if change == "camera":
        camera = [json.loads(line) for line in (worker / "camera_frames.jsonl").read_text().splitlines()]
        camera[-1]["complete"] = 1
        jsonl(worker / "camera_frames.jsonl", camera)
    if change == "archive": (worker.parent / "provenance" / audit.WORKER).write_bytes(b"changed")
    jsonl(worker / "native_states.jsonl", rows)
    jsonl(worker / "native_observations.jsonl", [dict(row, measurement_complete=False, stage="OBSERVED_BEFORE_CAMERA") for row in rows])
    manifest["artifact_sha256"] = {name: digest((worker / name).read_bytes()) for name in audit.ARTIFACTS}
    put(worker / "manifest.json", manifest)
    with pytest.raises(ValueError): audit.validate_trial(root, case, plan, {})


def test_old_three_phase_manifest_and_nested_output_are_rejected(tmp_path):
    root = tmp_path / "campaign"
    plan, worker, manifest, case = brake_fixture(root, complete=False)
    manifest["schema"] = "carla.low_speed_coast_probe.v1"
    put(worker / "manifest.json", manifest)
    with pytest.raises(ValueError): audit.validate_trial(root, case, plan, {})
    with pytest.raises(ValueError): audit.analyze(root, root / "summary", render=False)
