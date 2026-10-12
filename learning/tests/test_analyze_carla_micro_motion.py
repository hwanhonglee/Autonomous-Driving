"""HH_260906 - Verify the micro-motion sidecar on synthetic inputs without opening original evidence or changing gates."""

import copy
import json
import math
from pathlib import Path

import pytest

from scripts.e2e import analyze_carla_micro_motion as m


def sample(index, x=None, vx=1., phase="driving", yaw=0., vy=0., offset=-1.425):
    # HH_260906 - These fixtures are algebraic samples, never simulated vehicle evidence.
    x = index * .05 if x is None else x
    c, s = math.cos(yaw), math.sin(yaw)
    return {"frame": index, "timestamp_ns": 1_000_000_000 + index * 50_000_000, "phase": phase,
        "api": [x, 0., index * .01], "api_velocity": [vx, vy, .2],
        "rear": [x + offset * c, offset * s, index * .01], "legacy_world_velocity": [vx, vy]}


def native_row(index, phase):
    s = sample(index, phase=phase)
    return {"frame": s["frame"], "timestamp_ns": s["timestamp_ns"], "capture_phase": phase,
        "recorded_rear_ros_xyz_m": s["rear"], "actor_api_reference_ros_xyz_m": s["api"],
        "endpoint_actor_api_velocity_ros_mps": s["api_velocity"], "body_yaw_ros_rad": 0.,
        "original_recorded_planar_state": {"x": s["rear"][0], "y": 0., "z": s["rear"][2], "yaw": 0., "vx": 1., "vy": 0.},
        "source_actor_snapshot_transform_carla": {"x": s["api"][0], "y": 0., "z": s["api"][2], "yaw": 0.},
        "source_world_velocity_carla": [1., 0., .2]}


def future_row(frame=0, points=None):
    anchor = 1_000_000_000 + frame * 50_000_000
    points = [[(i + 1) * .1, 0.] for i in range(64)] if points is None else points
    previous, heading, steps = [0., 0.], 0., []
    for i, point in enumerate(points):
        delta = [point[k] - previous[k] for k in (0, 1)]; distance = math.hypot(*delta)
        value = None
        if distance > 1e-4:
            current = math.atan2(delta[1], delta[0]); value = abs(math.atan2(math.sin(current-heading), math.cos(current-heading))) / distance
            heading = current
        steps.append({"index": i, "target_timestamp_ns": anchor + (i+1)*100_000_000,
            "episode_relative_100ms_tick": frame // 2 + i + 1, "phase": "stationary_tail" if frame + (i+1)*2 >= 10 else "pre_tail", "dt_s": .1,
            "metrics": {"xy_curvature": [value, value is not None and value > .200001]}})
        previous = point
    return {"frame": frame, "anchor_timestamp_ns": anchor, "capture_phase": "stationary_warmup" if frame < 2 else "driving",
        "training_data_approved": False, "valid_future_points": 64, "disposition": "full_64_point_anchor",
        "diagnostic_future_xy_m": points, "diagnostic_body_yaw_delta_rad": [0.]*64,
        "diagnostic_target_timestamp_ns": [anchor+(i+1)*100_000_000 for i in range(64)],
        "valid_mask": [True]*64, "invalid_reasons": [None]*64,
        "discrete_bound_diagnostic": {"valid_points": 64, "invalid_points": 0, "steps": steps}}


def test_native_midpoint_integrals_and_phase_crossing():
    samples = [sample(0, x=0., vx=0., phase="stationary_warmup"), sample(1, x=.025, vx=1.), sample(2, x=.1, vx=2.)]
    row = m.interval_record(samples, "derived_10hz_offset0")
    api = row["references"]["actor_api_reference"]
    for rule, expected in (("left", .05), ("right", .15), ("trapezoid", .1)):
        assert api["integrals_xy_m"][rule] == pytest.approx([expected, 0.], abs=1e-15)
    assert api["residual_norm_m"]["trapezoid"] == pytest.approx(0.)
    assert row["crosses_phase"] is True and row["native_phases"] == ["stationary_warmup", "driving", "driving"]
    assert row["api_z_separate"]["delta_m"] == pytest.approx(.02)


def test_both_parities_cover_all_intermediate_samples():
    samples = [sample(i) for i in range(6)]
    got = {}
    for start in (0, 1):
        got[start] = [m.interval_record(samples[i:i+3], str(start)) for i in range(start, len(samples)-2, 2)]
    assert [(x["first_frame"], x["last_frame"]) for x in got[0]] == [(0, 2), (2, 4)]
    assert [(x["first_frame"], x["last_frame"]) for x in got[1]] == [(1, 3), (3, 5)]


def test_offset_identity_is_not_forced_equal_positions():
    samples = [sample(0, yaw=0.), sample(1, yaw=.2)]
    proof = m.interval_record(samples, "native_20hz")["reference_offset_identity"]
    assert math.hypot(*proof["rear_delta_minus_api_delta_xyz_m"][:2]) > .1
    assert proof["floating_residual_xyz_m"] == pytest.approx([0., 0., 0.], abs=1e-14)


def test_rear_velocity_rotation_is_planar_and_z_stays_separate():
    row = native_row(0, "driving")
    row["body_yaw_ros_rad"] = row["original_recorded_planar_state"]["yaw"] = math.pi/2
    row["source_actor_snapshot_transform_carla"]["yaw"] = -90.
    row["original_recorded_planar_state"].update(vx=2., vy=3.)
    parsed = m.native_sample(row)
    assert parsed["legacy_world_velocity"] == pytest.approx([-3., 2.])
    assert parsed["api_velocity"][2] == .2


def test_original_wrapped_yaw_uses_formula_not_raw_float_equality():
    row = native_row(0, "driving")
    row["source_actor_snapshot_transform_carla"]["yaw"] = -8.44379711151123
    raw = -math.radians(row["source_actor_snapshot_transform_carla"]["yaw"])
    row["body_yaw_ros_rad"] = raw
    row["original_recorded_planar_state"]["yaw"] = math.atan2(math.sin(raw), math.cos(raw))
    assert m.native_sample(row)["legacy_world_velocity"] == pytest.approx([math.cos(raw), math.sin(raw)])
    row["original_recorded_planar_state"]["yaw"] += 1e-8
    with pytest.raises(ValueError, match="yaw wrapping"):m.native_sample(row)


def test_target_native_bracket_keeps_phase_crossing_distinct_from_old_tail_class():
    samples = [sample(0, phase="stationary_warmup"), sample(1), sample(2, phase="stationary_tail")]
    stamps = [x["timestamp_ns"] for x in samples]
    result = m.target_bracket(samples, stamps, stamps[1]-1)
    assert result['phases']==['stationary_warmup','driving'] and result['frames']==[0,1]
    exact=m.target_bracket(samples,stamps,stamps[2])
    assert exact['phases']==['stationary_tail'] and exact['association']=='exact_native_timestamp'
    with pytest.raises(ValueError):m.target_bracket(samples,stamps,stamps[-1]+1)


@pytest.mark.parametrize("mutation", ["gap", "time", "backward"])
def test_native_bad_sampling_fails(mutation):
    samples = [sample(0), sample(1), sample(2)]
    if mutation == "gap": samples[1]["frame"] += 1
    elif mutation == "time": samples[1]["timestamp_ns"] += 2000
    else: samples[1]["timestamp_ns"] = samples[0]["timestamp_ns"]
    with pytest.raises(ValueError, match="gap|cadence"): m.interval_record(samples, "test")


def test_all_future_windows_have_exact_prefix_denominators():
    rows = list(m.future_direction_records(future_row()))
    assert len(rows) == 64
    for length, missing in (("1", 0), ("2", 1), ("5", 4)):
        assert sum(r["windows"][length]["status"] == "INSUFFICIENT_WITHIN_ANCHOR_PREFIX" for r in rows) == missing
    assert rows[0]["windows"]["1"]["delta_xy_m"] == [.1, 0.]
    assert rows[4]["windows"]["5"]["delta_xy_m"] == [.5, 0.]
    assert all(r["original_curvature_failed"] is False and r["coarse_interval_pass_claimed"] is False for r in rows)


def test_tiny_displacement_holds_original_heading_and_does_not_reset():
    points = [[0., .1], [0., .10005], [.1, .10005]] + [[.1+(i-2)*.1, .10005] for i in range(3,64)]
    rows = list(m.future_direction_records(future_row(points=points)))
    assert rows[1]["windows"]["1"]["status"] == "TINY_DISPLACEMENT_UNASSESSED"
    assert rows[1]["original_heading_state_before_rad"] == rows[1]["original_heading_state_after_rad"] == math.pi/2
    assert rows[1]["original_100ms_curvature_rad_per_m"] is None
    assert rows[2]["original_100ms_curvature_rad_per_m"] == pytest.approx((math.pi/2)/.1)


def test_exact_tiny_threshold_unassessed_and_no_coarse_pass():
    points = [[.0001, 0.]] + [[.0001+i*.1, 0.] for i in range(1,64)]
    row = list(m.future_direction_records(future_row(points=points)))[0]
    assert row["windows"]["1"]["heading_rad"] is None
    assert row["original_100ms_curvature_rad_per_m"] is None and row["original_curvature_failed"] is False
    assert row["comparisons"]["direction_200ms_vs_100ms_abs_rad"] is None


@pytest.mark.parametrize("mutation", ["bool_timestamp", "nan", "mask", "flag_bool", "verdict", "value", "length", "time", "denial", "count"])
def test_malformed_future_fails_without_repair(mutation):
    row = future_row()
    if mutation == "bool_timestamp": row["diagnostic_target_timestamp_ns"][0] = True
    elif mutation == "nan": row["diagnostic_future_xy_m"][0][0] = float("nan")
    elif mutation == "mask": row["valid_mask"][0] = 1
    elif mutation == "flag_bool": row["discrete_bound_diagnostic"]["steps"][0]["metrics"]["xy_curvature"][1] = 0
    elif mutation == "verdict": row["discrete_bound_diagnostic"]["steps"][0]["metrics"]["xy_curvature"][1] = True
    elif mutation == "value": row["discrete_bound_diagnostic"]["steps"][0]["metrics"]["xy_curvature"][0] = 1.
    elif mutation == "length": row["diagnostic_body_yaw_delta_rad"].pop()
    elif mutation == "time": row["diagnostic_target_timestamp_ns"][0] += 1
    elif mutation == "count": row["discrete_bound_diagnostic"]["valid_points"] = True
    else: row["training_data_approved"] = 0
    with pytest.raises(ValueError): list(m.future_direction_records(row))


@pytest.mark.parametrize("value", [True, False, float("nan"), float("inf"), "0", None])
def test_strict_finite_numbers(value):
    with pytest.raises(ValueError): m.number(value)


def test_descriptive_quantiles_are_linear_not_a_new_gate():
    assert m.describe([0., 1., 2.]) == {"count": 3, "rms": pytest.approx(math.sqrt(5/3)), "p50": 1., "p95": pytest.approx(1.9), "maximum": 2.}
    assert m.describe([])["rms"] is None


def test_duplicate_or_nonfinite_json_refused():
    for text in ('{"a":1,"a":2}', '{"a":NaN}'):
        with pytest.raises(ValueError): m.load_json(text)


@pytest.fixture
def evidence(tmp_path, monkeypatch):
    # HH_260906 - The integration corpus has eight synthetic cases with five full anchors each; only test-local pins/counts change.
    root = tmp_path / "inputs"; root.mkdir()
    monkeypatch.setattr(m, "NATIVE_COUNTS", (140,)*8); monkeypatch.setattr(m, "CAMERA_COUNTS", (70,)*8)
    monkeypatch.setattr(m, "FULL_COUNTS", (5,)*8); monkeypatch.setattr(m, "CURVATURE_ANCHORS", (0,)*8)
    cases, native, future = [], [], []
    for key in m.expected_cases():
        for i in range(140):
            phase = "stationary_warmup" if i < 2 else "driving" if i < 10 else "stationary_tail"
            native.append({"case_id": key, **native_row(i, phase)})
        for frame in range(0,140,2):
            row = future_row(frame) if frame < 10 else {"frame": frame, "anchor_timestamp_ns": 1_000_000_000+frame*50_000_000,
                "capture_phase": "stationary_tail", "training_data_approved": False, "valid_future_points": 0, "disposition": "tail_label_context_only"}
            future.append({"case_id": key, **row})
        cases.append({"case_id": key, "raw_native_state_count": 140,
            "raw_native_state_counts_by_phase": {"stationary_warmup": 2, "driving": 8, "stationary_tail": 130},
            "original_training_data_approved": False, "original_development_only": True, "training_data_approved": False,
            "measured_future": {"camera_anchor_count": 70, "available_64_point_anchor_count": 5, "bound_assessed_64_point_anchor_count": 5,
                "bound_unassessed_anchor_count": 0, "anchor_count_by_phase": {"stationary_warmup": 1, "driving": 4, "stationary_tail": 65},
                "all_available_prefixes": {"64": {"metrics": {"xy_curvature": {"horizon_eligible_violating_anchor_count": 0}}}}}})
    scope = {k: False for k in ("training_data_approved", "dataset_admission", "common10_dataset_written", "labels_modified",
        "training", "model_loaded", "model_inference", "test_payload_read", "automatic_winner_selection")}
    summary = {"schema": "carla_expert.turn_launch_raw_geometry_diagnostic.v1", "status": "DIAGNOSED_NOT_ADMITTED",
        "planned_case_count": 8, "finalized_case_count": 8, "all_planned_cases_retained": True, "scope": scope, "cases": cases}
    (root/'summary.json').write_text(json.dumps(summary))
    for name, values in [('native_snapshot_audit.jsonl',native),('future_geometry_audit.jsonl',future)]:
        (root/name).write_text(''.join(json.dumps(row)+'\n' for row in values))
    (root/'jpeg_audit.jsonl').write_text('{"synthetic_fixture":true}\n')
    (root/'SHA256SUMS').write_text('synthetic test-local checksum bytes\n')
    monkeypatch.setattr(m, 'PINS', {p.name:m.sha(p) for p in root.iterdir()})
    return root, tmp_path/'output'


def test_full_synthetic_integration_retains_every_denominator_and_source(evidence):
    root, output = evidence
    before = m.input_pins(root); result = m.analyze(root,output)
    assert m.input_pins(root)==before and result['status']=='MEASURED_NOT_ADMITTED'
    assert result['native_state_count']==1120 and result['camera_anchor_count']==560
    assert result['full_future_anchor_count']==40 and result['future_index_row_count']==2560
    assert sum(result['native_interval_counts'].values())==8*(139+69+69)
    assert all(v==0 for v in result['original_curvature_violating_anchors_by_case'].values())
    assert (output/'executed_source.py').read_bytes()==Path(m.__file__).read_bytes()
    assert len(list(m.rows(output/'future_directions.jsonl')))==2560
    for line in (output/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  '); assert m.sha(output/name)==digest


@pytest.mark.parametrize('kind',['existing','inside','symlink'])
def test_output_refusal_is_before_input_read(evidence,monkeypatch,kind):
    root,output=evidence
    if kind=='existing':output.mkdir()
    elif kind=='inside':output=root/'nested'
    else:output.symlink_to(root,target_is_directory=True)
    monkeypatch.setattr(m,'input_pins',lambda *_:pytest.fail('must refuse before reading inputs'))
    with pytest.raises(ValueError,match='fresh'):m.analyze(root,output)


def test_changed_input_sha_or_inventory_fails(evidence):
    root,_=evidence
    (root/'summary.json').write_text('{}')
    with pytest.raises(ValueError,match='SHA'):m.input_pins(root)
    (root/'unexpected').write_text('x')
    with pytest.raises(ValueError,match='inventory'):m.input_pins(root)


def test_output_refuses_physical_alias_of_symlinked_dataset_root(evidence,tmp_path,monkeypatch):
    root,_=evidence
    repo=tmp_path/'repo'; repo.mkdir()
    physical=tmp_path/'private_data'; physical.mkdir()
    (repo/'datasets').symlink_to(physical,target_is_directory=True)
    monkeypatch.setattr(m,'ROOT',repo)
    monkeypatch.setattr(m,'input_pins',lambda *_:pytest.fail('must refuse dataset alias before reading inputs'))
    with pytest.raises(ValueError,match='fresh'):m.analyze(root,physical/'new_sidecar')


def test_source_change_during_run_fails_and_retains_partial(evidence,monkeypatch):
    root,output=evidence; original=m.input_pins; calls=0
    def changed(path):
        nonlocal calls
        calls+=1
        if calls==2:raise ValueError('synthetic postcheck mutation')
        return original(path)
    monkeypatch.setattr(m,'input_pins',changed)
    with pytest.raises(ValueError,match='mutation'):m.analyze(root,output)
    assert (output/'future_directions.jsonl').is_file() and not (output/'SHA256SUMS').exists()
    status=json.loads((output/'execution_status.json').read_text())
    assert status['status']=='FAILED' and status['source_postcheck_pass'] is True


def test_cli_abbreviation_rejected():
    with pytest.raises(SystemExit):m.main(['--input','example'])
