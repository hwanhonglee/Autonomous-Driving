"""HH_260906 - CPU-only witness tests; no simulator, images, target export or model import."""

import ast
import copy
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

from scripts.e2e import audit_carla_curvature_conditioning as audit

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


def old_pure_functions():
    # HH_260906 - Execute only named original scalar functions, never their model-importing module.
    namespace = {"math": math, "ContractError": ValueError,
        "PHYSICAL_TIME_STEP_S": .1, "PHYSICAL_MAXIMUM_SPEED_MPS": 30. / 3.6,
        "PHYSICAL_MAXIMUM_ACCELERATION_MPS2": 2.9, "PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M": .2,
        "PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2": 2.8}
    source = ast.parse((_hh_source_path(audit.ROOT, "scripts/e2e/audit_portable_target_feasibility.py")).read_text())
    names = {"_require", "_number", "_integer", "_point", "_wrap", "audit_sample", "summarize_samples"}
    constants = {"ROUNDING_TOLERANCE", "HEADING_MINIMUM_STEP_M", "NEAR_GOAL_BUFFER_M", "METRICS"}
    nodes = [node for node in source.body if isinstance(node, ast.FunctionDef) and node.name in names or
        isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in constants for t in node.targets)]
    exec(compile(ast.Module(body=nodes, type_ignores=[]), "original_pure_target_math", "exec"), namespace)
    source = ast.parse((_hh_source_path(audit.ROOT, "scripts/e2e/prepare_carla_common10_dataset.py")).read_text())
    nodes = [ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)]
    nodes += [node for node in source.body if isinstance(node, ast.FunctionDef) and
        node.name in {"_interpolate_state", "_relative_xy_yaw"}]
    namespace["bisect_right"] = audit.bisect_right
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), "original_pure_adapter_math", "exec"), namespace)
    return namespace


def state(index, *, actor_x=None, actor_y=None, body_yaw=0.):
    x = 200. + .005 * min(index, 32) if actor_x is None else actor_x
    y = (0. if index < 20 else .00004) if actor_y is None else actor_y
    phase = "driving" if index < 6 else "stationary_tail"
    c, s = math.cos(-body_yaw), math.sin(-body_yaw)
    control = dict(throttle=0., brake=0., steer=0., hand_brake=False, reverse=False, gear=1, manual_gear_shift=False)
    return dict(frame=1000 + index, timestamp=index * .05, capture_phase=phase,
        x=x-1.425*c, y=-(y-1.425*s), z=0., yaw=math.atan2(-s, c), vx=.1, vy=0., ax=0., ay=0., yaw_rate=0.,
        actor_snapshot_transform_carla=dict(x=x, y=y, z=0., yaw=math.degrees(-body_yaw), pitch=0., roll=0.),
        world_velocity_carla=[.1, 0., 0.], world_acceleration_carla=[0., 0., 0.],
        world_angular_velocity_carla_deg_s=[0., 0., 0.], current_control=control.copy(), next_control=control.copy(),
        goal_stop=dict(pilot_state="coast_low" if index < 6 else "setup_or_tail"))


def originals(states):
    old = old_pure_functions()
    timeline = [(audit.stamp(row), row) for row in states]
    records, diagnostics, cameras = [], [], []
    for row in states[::2]:
        camera = {key: row[key] for key in ("frame", "timestamp", "capture_phase")}
        cameras.append(camera)
        anchor = audit.stamp(row)
        record = dict(frame=row["frame"], capture_phase=row["capture_phase"], anchor_timestamp_ns=anchor)
        if row["capture_phase"] == "stationary_tail":
            records.append(dict(**record, disposition="tail_label_context_only", valid_points=0))
            continue
        planning = dict(available=True, dt_s=.1, positions_base_xy_m=[], speed_mps=[], yaw_rad=[], valid=[True]*64,
            target_timestamp_ns=[anchor+(i+1)*100_000_000 for i in range(64)], invalid_reason=[None]*64)
        for timestamp in planning["target_timestamp_ns"]:
            future = old["_interpolate_state"](timeline, timestamp)
            xy, yaw = old["_relative_xy_yaw"](row, future)
            planning["positions_base_xy_m"].append(xy)
            planning["yaw_rad"].append(yaw)
            planning["speed_mps"].append(math.hypot(future["vx"], future["vy"]))
        sample = dict(sample_id=f"raw-frame:{row['frame']}", anchor_timestamp_ns=anchor,
            ego=dict(linear_velocity_base_mps=[row["vx"], row["vy"], 0.]),
            navigation=dict(goal_base_m=[100., 0.], route_anchor_arc_m=0.), labels=dict(planning=planning))
        diagnostic = old["audit_sample"](sample, route_length_m=100., episode_start_ns=timeline[0][0],
            episode_end_ns=timeline[-1][0], tail_start_ns=audit.stamp(states[6]))
        diagnostic.update(episode_id="single_raw_trial", capture_phase=row["capture_phase"])
        diagnostics.append(diagnostic)
        records.append(dict(**record, disposition="full_64_point_anchor", valid_points=64,
            unavailable_reason=None, valid_mask=[True]*64, invalid_reasons=[None]*64, diagnostic=diagnostic))
    published = dict(measured_future=dict(full_64_point_anchors=old["summarize_samples"](diagnostics, 64),
        camera_anchor_count=len(cameras), disposition_counts=dict(audit.Counter(r["disposition"] for r in records))))
    # HH_260906 - The retained evidence is JSON, so original scalar tuples are serialized as arrays.
    return cameras, json.loads(json.dumps(records)), published


@pytest.fixture
def numerical():
    states = [state(i) for i in range(134)]
    cameras, futures, published = originals(states)
    return states, cameras, futures, published


def test_replays_original_scalar_math_and_preserves_every_occurrence(numerical):
    before = copy.deepcopy(numerical)
    summary, witnesses = audit.analyze(*numerical, 2.85)
    assert numerical == before
    assert summary["curvature_replay_exact_value_and_flag_match"] is True
    assert summary["counts"]["full_64_point_anchor_count"] == 3
    original = numerical[-1]["measured_future"]["full_64_point_anchors"]["metrics"]["xy_curvature"]
    assert original["violating_step_count"] > 0
    assert summary["counts"]["original_violating_window_step_count"] == original["violating_step_count"]
    assert sum(len(w["window_occurrences"]) for w in witnesses) == original["violating_step_count"]
    assert len(witnesses) == original["unique_episode_relative_100ms_violation_tick_count"]
    assert all(w["original_failure_retained"] and all(o["original_failure_retained"] for o in w["window_occurrences"]) for w in witnesses)


def test_first_point_and_heading_persisted_through_unassessed_steps_are_distinct():
    xy = [.1 * math.cos(.03), .1 * math.sin(.03)]
    first, heading, index = audit.geometry(xy, [0., 0.], 0., None, 0)
    assert first["heading_reference"] == "anchor_body_heading_first_point"
    assert first["curvature_rad_per_m"] == pytest.approx(.3)
    tiny, held, held_index = audit.geometry([1e-6, 0.], [0., 0.], 0., None, 0)
    assert tiny["curvature_rad_per_m"] is None and held == 0. and held_index is None
    after, _, _ = audit.geometry(xy, [1e-6, 0.], held, held_index, 1)
    assert after["heading_reference"] == "anchor_body_heading_after_unassessed_steps"
    repeat, _, _ = audit.geometry([xy[0]*2, xy[1]*2], xy, heading, index, 1)
    assert repeat["heading_reference"] == "last_assessed_segment_heading" and repeat["curvature_rad_per_m"] == 0.


def test_minimum_step_is_strict_and_never_reported_as_curvature_pass():
    result, _, _ = audit.geometry([.0001, 0.], [0., 0.], .2, 2, 4)
    assert result["curvature_rad_per_m"] is None
    result, _, _ = audit.geometry([math.nextafter(.0001, 1.), 0.], [0., 0.], .2, 2, 4)
    assert result["curvature_rad_per_m"] > 1000


def test_actor_and_virtual_base_remain_separate_under_yaw_change():
    first = state(0, actor_x=200., actor_y=0.)
    second = state(2, actor_x=200.01, actor_y=0., body_yaw=.0003)
    result = audit.native_segment(first, second)
    assert result["actor_reference"]["body_displacement_forward_left_m"][1] == 0.
    assert abs(result["virtual_base"]["body_displacement_forward_left_m"][1]) > .0004
    assert result["virtual_base_minus_actor_displacement_ros_xy_m"][1] == pytest.approx(-1.425*math.sin(.0003))
    assert result["actor_reference"]["adjacent_segment_curvature_rad_per_m"] is None


def test_float32_spacing_and_two_quantum_step_are_measurements_not_classifications():
    assert audit.float32_resolution(-198.) == dict(absolute_float32_spacing_m=2**-16, exactly_float32_representable=True)
    first, second = state(0, actor_x=-198., actor_y=142.), state(2, actor_x=-198.-2**-15, actor_y=142.01)
    result = audit.native_segment(first, second)
    assert result["actor_reference"]["coordinate_step_in_endpoint_float32_spacings"][0] == -2.
    assert "is_artifact" not in result and "failure_removed" not in result


def test_interpolation_uses_exact_nanosecond_brackets_and_original_adapter(numerical):
    states = numerical[0]
    states[1]["timestamp"] += 1e-9
    stamps = [audit.stamp(r) for r in states]
    got, binding = audit.interpolate(states, stamps, 50_000_000)
    expected = old_pure_functions()["_interpolate_state"](list(zip(stamps, states)), 50_000_000)
    assert {k: got[k] for k in expected} == expected
    assert binding["mode"] == "linear_native_bracket" and binding["frames"] == [1000, 1001]


@pytest.mark.parametrize("mutation,error", [
    (lambda s,c,f,p: c[0].update(frame=1002), "frame-time-phase"),
    (lambda s,c,f,p: c[0].update(timestamp=.00001), "frame-time-phase"),
    (lambda s,c,f,p: f[0].update(anchor_timestamp_ns=1), "frame-time-phase"),
    (lambda s,c,f,p: f.pop(), "cadence count"),
    (lambda s,c,f,p: s[10].update(frame=9000), "frame/time gap"),
    (lambda s,c,f,p: s[10].update(timestamp=math.nan), "nonfinite"),
    (lambda s,c,f,p: s[10].update(x=s[10]["x"]+.01), "pinned conversion"),
    (lambda s,c,f,p: f[0]["diagnostic"]["steps"][0].update(target_timestamp_ns=100_000_001), "grid mismatch"),
    (lambda s,c,f,p: f[0]["diagnostic"]["steps"][0]["metrics"]["xy_curvature"].__setitem__(0, 10.), "independent raw replay"),
    (lambda s,c,f,p: f[0]["diagnostic"]["steps"][0]["metrics"]["xy_curvature"].__setitem__(1, True), "independent raw replay"),
    (lambda s,c,f,p: f[-1].update(disposition="full_64_point_anchor"), "tail context"),
    (lambda s,c,f,p: f[0]["valid_mask"].__setitem__(0, False), "complete non-tail"),
    (lambda s,c,f,p: p["measured_future"]["full_64_point_anchors"]["metrics"]["xy_curvature"].update(violating_step_count=0), "counts mismatch"),
])
def test_malformed_or_reclassified_evidence_is_rejected(numerical, mutation, error):
    mutation(*numerical)
    with pytest.raises(audit.EvidenceError, match=error):
        audit.analyze(*numerical, 2.85)


def encoded(value):
    return (json.dumps(value, allow_nan=False) + "\n").encode()


def reseal(evidence):
    payload = "".join(f"{audit.sha((evidence/name).read_bytes())}  {name}\n" for name in ("summary.json", "future_windows.jsonl"))
    (evidence/"SHA256SUMS").write_text(payload)
    return audit.sha(payload.encode())


@pytest.fixture
def files(tmp_path, numerical):
    trial, evidence = tmp_path/"raw", tmp_path/"sealed"
    trial.mkdir(); evidence.mkdir()
    states, cameras, futures, published = numerical
    for name in audit.RAW_PATHS:
        path = trial/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(encoded({}))
    (trial/"episode/states.jsonl").write_bytes(b"".join(encoded(r) for r in states))
    (trial/"episode/camera_frames.jsonl").write_bytes(b"".join(encoded(r) for r in cameras))
    (trial/"episode/manifest.json").write_bytes(encoded(dict(capture_contract=dict(goal_stop_profile=dict(profile_id="comfortable_v5_brake_hold")),
        result=dict(training_data_approved=False), coordinate_contract=dict(wheelbase_m=2.85))))
    published.update(schema="carla.expert_brake_hold_pilot_summary.v1", status="DIAGNOSED_NOT_ADMITTED",
        training_data_approved=False, dataset_export=False, original_input_bytes_unchanged=True,
        input_metadata_sha256={name: audit.sha((trial/name).read_bytes()) for name in audit.RAW_PATHS},
        source_sha256=dict(files={name: audit.sha((_hh_source_path(audit.ROOT, name)).read_bytes()) for name in audit.SOURCE_PATHS}))
    (evidence/"summary.json").write_bytes(encoded(published))
    (evidence/"future_windows.jsonl").write_bytes(b"".join(encoded(r) for r in futures))
    return trial, evidence, reseal(evidence), audit.sha(Path(audit.__file__).read_bytes())


def test_full_readonly_verification_then_fresh_diagnostic_only_outputs(files, tmp_path, monkeypatch):
    trial, evidence, seal, script = files
    before = {str(p): p.read_bytes() for root in (trial, evidence) for p in root.rglob("*") if p.is_file()}
    opened, original = [], audit.read
    def guard(path, *args):
        opened.append(str(path))
        assert not any(token in str(path) for token in ("images/", ".jpg", ".png", ".pt", "TEST", "test_payload"))
        return original(path, *args)
    monkeypatch.setattr(audit, "read", guard)
    summary, witnesses = audit.run(*files)
    assert before == {str(p): p.read_bytes() for root in (trial, evidence) for p in root.rglob("*") if p.is_file()}
    output = tmp_path/"diagnostic"
    audit.run(*files, output_dir=output)
    assert {p.name for p in output.iterdir()} == {"summary.json", "witnesses.jsonl", "SHA256SUMS"}
    assert summary["scope"]["training_data_approved"] is False and summary["scope"]["failures_removed"] is False
    assert len(audit.json_lines((output/"witnesses.jsonl").read_bytes())) == len(witnesses)
    for line in (output/"SHA256SUMS").read_text().splitlines():
        digest, name = line.split("  ")
        assert audit.sha((output/name).read_bytes()) == digest
    with pytest.raises(audit.EvidenceError, match="fresh output"):
        audit.run(*files, output_dir=output)
    assert all(Path(path).suffix in (".py", ".md", ".json", ".jsonl", "") for path in opened)


@pytest.mark.parametrize("kind", ["script", "seal", "raw", "future", "source"])
def test_hash_tampering_stops_before_output(files, tmp_path, kind):
    trial, evidence, seal, script = files
    if kind == "script": script = "0"*64
    elif kind == "seal": seal = "0"*64
    elif kind == "raw": (trial/"episode/states.jsonl").write_bytes(b"{}\n")
    elif kind == "future": (evidence/"future_windows.jsonl").write_bytes(b"{}\n")
    else:
        published = audit.loads((evidence/"summary.json").read_bytes())
        published["source_sha256"]["files"]["portable_e2e/model.py"] = "0"*64
        (evidence/"summary.json").write_bytes(encoded(published)); seal = reseal(evidence)
    output = tmp_path/"new"
    with pytest.raises(audit.EvidenceError): audit.run(trial, evidence, seal, script, output_dir=output)
    assert not output.exists()


@pytest.mark.parametrize("kind", ["denial", "extra_raw", "extra_source"])
def test_scope_and_input_closure_are_fail_closed_before_unknown_payload_open(files, tmp_path, monkeypatch, kind):
    trial, evidence, seal, script = files
    published = audit.loads((evidence/"summary.json").read_bytes())
    if kind == "denial": published["training_data_approved"] = True
    elif kind == "extra_raw": published["input_metadata_sha256"]["TEST/test_payload.jsonl"] = "0"*64
    else: published["source_sha256"]["files"]["TEST/test_payload.jsonl"] = "0"*64
    (evidence/"summary.json").write_bytes(encoded(published)); seal = reseal(evidence)
    original = audit.read
    def guard(path, *args):
        assert "TEST" not in str(path)
        return original(path, *args)
    monkeypatch.setattr(audit, "read", guard)
    with pytest.raises(audit.EvidenceError): audit.run(trial, evidence, seal, script)


def test_input_change_during_analysis_prevents_any_output(files, tmp_path, monkeypatch):
    original = audit.analyze
    def changed(*args):
        result = original(*args)
        (files[0]/"episode/states.jsonl").write_bytes(b"changed\n")
        return result
    monkeypatch.setattr(audit, "analyze", changed)
    with pytest.raises(audit.EvidenceError, match="SHA-256 mismatch"):
        audit.run(*files, output_dir=tmp_path/"output")
    assert not (tmp_path/"output").exists()


@pytest.mark.parametrize("kind", ["inside_raw", "inside_seal", "parent_traversal", "symlink"])
def test_output_cannot_be_nested_or_aliased_into_originals(files, tmp_path, kind):
    trial, evidence = files[:2]
    output = trial/"new" if kind == "inside_raw" else evidence/"new"
    if kind == "parent_traversal":
        (tmp_path/"other").mkdir()
        output = tmp_path/"other/../raw/new"
    if kind == "symlink":
        (tmp_path/"alias").symlink_to(trial, target_is_directory=True)
        output = tmp_path/"alias/new"
    with pytest.raises(audit.EvidenceError): audit.run(*files, output_dir=output)
    assert not (trial/"new").exists() and not (evidence/"new").exists()


def test_import_is_stdlib_only_and_cli_abbreviations_are_rejected():
    command = [sys.executable, "-B", "-c", "import sys,runpy; runpy.run_path('tools/workspace_python.py')['configure'](); from scripts.e2e import audit_carla_curvature_conditioning; assert not ({'torch','carla','PIL','numpy'} & sys.modules.keys())"]
    assert subprocess.run(command, cwd=audit.ROOT, capture_output=True, text=True).returncode == 0
    with pytest.raises(SystemExit): audit.main(["--trial", "x"])


def test_json_nonfinite_duplicates_and_invalid_raw_numbers_rejected():
    for text in ('{"x":1,"x":2}', '{"x":NaN}', '{"x":1e999}', '{"x":-1e999}'):
        with pytest.raises(audit.EvidenceError): audit.loads(text)
    for value in (True, math.inf, math.nan):
        with pytest.raises(audit.EvidenceError): audit.number(value)


def test_float32_extreme_has_no_finite_spacing_and_is_rejected():
    maximum = audit.struct.unpack("!f", audit.struct.pack("!I", 0x7f7fffff))[0]
    with pytest.raises(audit.EvidenceError, match="finite upper neighbor"):
        audit.float32_resolution(maximum)
    with pytest.raises(audit.EvidenceError, match="representable range"):
        audit.float32_resolution(1e100)
