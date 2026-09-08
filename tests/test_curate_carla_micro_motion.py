"""HH_260906 - Test publication boundaries and synthetic plot plumbing without simulator, training or raw-data changes."""

import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import curate_carla_micro_motion as module


def fixture_root(tmp_path):
    root=tmp_path/"input"; root.mkdir()
    source=Path(module.analysis.__file__).read_bytes()
    digest=hashlib.sha256(source).hexdigest()
    inputs={name:{"sha256":value} for name,value in module.analysis.PINS.items()}
    summary={"schema":module.analysis.SCHEMA,"status":"MEASURED_NOT_ADMITTED","source_sha256":digest,
        "input_and_source_postcheck_pass":True,"input_pins":inputs,
        "scope":{key:False for key in ("training_data_approved","dataset_admission","labels_modified",
            "model_loaded","training","live_simulator_access","thresholds_changed")}}
    (root/"summary.json").write_text(json.dumps(summary))
    (root/"execution_status.json").write_text(json.dumps({"status":"COMPLETE","source_sha256":digest,"input_and_source_postcheck_pass":True,"input_pins":inputs}))
    (root/"executed_source.py").write_bytes(source)
    for name in ("native_intervals.jsonl","future_directions.jsonl"):
        (root/name).write_text("{}\n")
    checksums(root)
    return root,module.analysis.sha(root/"summary.json")


def checksums(root):
    (root/"SHA256SUMS").write_text("".join(f"{module.analysis.sha(p)}  {p.name}\n"
        for p in sorted(root.iterdir()) if p.name!="SHA256SUMS"))


def test_checked_originals(tmp_path):
    root,digest=fixture_root(tmp_path)
    summary,pins=module.checked_inputs(root,digest,module.analysis.sha(root/"SHA256SUMS"))
    assert summary["scope"]["training_data_approved"] is False
    assert len(pins)==6


@pytest.mark.parametrize("change",["wrong_pin","mutated_stream","symlink","extra_file","unfinished","approved","wrong_source","postcheck_false","missing_original_pins","status_source"])
def test_rejects_incomplete_or_changed_inputs(tmp_path,change):
    root,digest=fixture_root(tmp_path)
    if change=="wrong_pin": digest="0"*64
    elif change=="mutated_stream": (root/"native_intervals.jsonl").write_text("changed\n")
    elif change=="symlink":
        p=root/"future_directions.jsonl"; p.unlink(); p.symlink_to(root/"native_intervals.jsonl")
    elif change=="extra_file": (root/"extra.txt").write_text("unexpected")
    elif change=="unfinished":
        p=root/"execution_status.json"; s=json.loads(p.read_text()); s["status"]="FAILED"; p.write_text(json.dumps(s)); checksums(root)
    elif change=="wrong_source": (root/"executed_source.py").write_text("different"); checksums(root)
    elif change=="status_source":
        p=root/"execution_status.json"; s=json.loads(p.read_text()); s["source_sha256"]="0"*64; p.write_text(json.dumps(s)); checksums(root)
    else:
        p=root/"summary.json"; s=json.loads(p.read_text())
        if change=="approved": s["scope"]["training_data_approved"]=True
        elif change=="missing_original_pins": s["input_pins"]={}
        else: s["input_and_source_postcheck_pass"]=False
        p.write_text(json.dumps(s)); checksums(root); digest=module.analysis.sha(p)
    with pytest.raises(ValueError): module.checked_inputs(root,digest,module.analysis.sha(root/"SHA256SUMS"))


@pytest.mark.parametrize("kind",["existing","inside_input","symlink"])
def test_output_must_be_new_outside_inputs(tmp_path,kind):
    root,digest=fixture_root(tmp_path)
    output=tmp_path/"out"
    if kind=="existing": output.mkdir()
    elif kind=="inside_input": output=root/"nested"
    else: output.symlink_to(tmp_path/"missing",target_is_directory=True)
    with pytest.raises(ValueError): module.publish(root,output,digest,module.analysis.sha(root/"SHA256SUMS"))


def test_draw_synthetic_fixture_only(tmp_path):
    # HH_260906 - These fabricated unit fixtures test chart rendering, never stand in for actual published driving evidence.
    pytest.importorskip("matplotlib")
    image=pytest.importorskip("PIL.Image")
    cases=module.analysis.expected_cases()
    compact={"native_residual_pooled":{},"direction_difference_pooled_radians":{}}
    for case in cases:
        for series in module.SERIES:
            for reference in module.REFERENCES:
                for rule in module.RULES:
                    compact["native_residual_pooled"]["/".join((case,series,reference,rule))]={"p95":.001}
    for count in (2,5):
        for bucket in module.BINS:
            for subset in ("all_assessed","original_failed"):
                compact["direction_difference_pooled_radians"]["/".join((str(count),bucket,subset))]={"count":4,"p95":.05}
    # HH_260906 - Exercise unassessed rendering instead of turning an empty subset into a zero-valued success.
    compact["direction_difference_pooled_radians"].pop("5/ge1/original_failed")
    traces={case:[{"first_timestamp_ns":i*50_000_000,"target_timestamp_ns":(i+1)*50_000_000,
        "native_phases":["stationary_tail" if i>12 else "driving"],
        "references":{ref:{"residual_norm_m":{"trapezoid":i*.00001}} for ref in module.REFERENCES}}
        for i in range(20)] for case in (cases[1],cases[6])}
    module.draw(compact,traces,tmp_path)
    files=list(tmp_path.glob("*.png")); assert len(files)==3
    for p in files:
        with image.open(p) as frame:
            assert frame.width>=1600 and frame.height>=700
            frame.verify()


def test_changed_stream_after_plot_is_rejected(tmp_path,monkeypatch):
    root,digest=fixture_root(tmp_path)
    monkeypatch.setattr(module,"collect_plot_values",lambda *_: ({},{}))
    monkeypatch.setattr(module,"draw",lambda *_:(root/"future_directions.jsonl").write_text("mutated\n"))
    output=tmp_path/"out"
    with pytest.raises(ValueError): module.publish(root,output,digest,module.analysis.sha(root/"SHA256SUMS"))
    assert not (output/"SHA256SUMS").exists()


def test_recomputed_stream_manifest_cannot_override_reviewed_digest(tmp_path):
    root,digest=fixture_root(tmp_path)
    reviewed=module.analysis.sha(root/"SHA256SUMS")
    (root/"future_directions.jsonl").write_text("replacement\n"); checksums(root)
    with pytest.raises(ValueError,match="checksum-manifest"):
        module.checked_inputs(root,digest,reviewed)


def test_duplicate_native_interval_cannot_replace_missing_row(tmp_path,monkeypatch):
    case=module.analysis.expected_cases()[0]
    row={"case_id":case,"series":"native_20hz","first_frame":1,"last_frame":2,
        "references":{ref:{"residual_norm_m":{rule:0. for rule in module.RULES}} for ref in module.REFERENCES}}
    monkeypatch.setattr(module.analysis,"rows",lambda *_:iter([row,row]))
    with pytest.raises(ValueError,match="duplicate native"):
        module.collect_plot_values(tmp_path,{"native_interval_counts":{}})


@pytest.mark.parametrize("kind",["duplicate","missing_index","timestamp"])
def test_bad_future_coverage_is_rejected_before_totals(tmp_path,monkeypatch,kind):
    case=module.analysis.expected_cases()[0]
    row={"case_id":case,"index":0,"anchor_frame":10,"anchor_timestamp_ns":1_000_000_000,
        "target_timestamp_ns":1_100_000_000,"xy_speed_bin_100ms":"ge1","original_curvature_failed":False,
        "windows":{str(n):{"status":"ASSESSED"} for n in (1,2,5)},
        "comparisons":{f"direction_{n}00ms_vs_100ms_abs_rad":0. for n in (2,5)}}
    other=dict(row)
    if kind=="missing_index": other["index"]=2
    if kind=="timestamp": other.update(index=1,target_timestamp_ns=1_200_000_001)
    monkeypatch.setattr(module.analysis,"rows",lambda path:iter([] if path.name=="native_intervals.jsonl" else [row,other]))
    with pytest.raises(ValueError,match="duplicate future|ordered 0..63|timestamp grid"):
        module.collect_plot_values(tmp_path,{"native_interval_counts":{}})
