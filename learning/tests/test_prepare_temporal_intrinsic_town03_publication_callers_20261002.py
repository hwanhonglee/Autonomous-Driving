"""HH_261002 - Synthetic metadata-only caller preparation; no actual archives, models, simulator, or render."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import prepare_temporal_intrinsic_town03_publication_callers_20261002 as tool


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def study(tmp_path, monkeypatch):
    root = tmp_path/"workspace"; root.mkdir()
    plans = root/"storage/experiments/plans/2026-10-02"; plans.mkdir(parents=True)
    runs = root/"original_pair"; output = plans/"prepared_callers_v1"
    pair_authority = plans/"completed_pair_authority.json"
    pair_sha = write_json(pair_authority, dict(schema=tool.paired.AUTHORITY_SCHEMA,
        both_fixed_endpoints_completed=True, arm_order=list(tool.paired.ARMS),
        model_promoted=False, vehicle_control_approved=False))
    source = root/"source.py"; source.write_text("# source fixture\n")
    font = root/"font.ttf"; font.write_bytes(b"font fixture")
    helper = root/"helper.py"; helper.write_text("# helper fixture\n")
    monkeypatch.setattr(tool, "ROOT", root); monkeypatch.setattr(tool, "OUTPUT", output)
    monkeypatch.setattr(tool, "PAIR_AUTHORITY", pair_authority)
    monkeypatch.setattr(tool, "PAIR_AUTHORITY_SHA", pair_sha)
    monkeypatch.setattr(tool, "__file__", str(helper))
    wanted = dict(schema=tool.SCHEMA, helper_sha256=tool.digest(helper), publisher_sha256=tool.PUBLISHER_SHA,
        publication_source_sha256={"source.py": tool.digest(source)}, font_sha256={str(font): tool.digest(font)},
        run_roots={arm: str(runs/arm) for arm in tool.paired.ARMS},
        publication_outputs={arm: str(root/"docs"/arm) for arm in tool.paired.ARMS},
        caller_output=str(output), completed_pair_authority_path=str(pair_authority), completed_pair_authority_sha256=pair_sha,
        PNG_index_policy="first_quarter_half_threequarters_exact_last", minimum_free_bytes=tool.MINIMUM_FREE_BYTES,
        actual_archive_read_authorized_by_import=False, renderer_or_model_execution=False,
        original_FAIL_verdicts_preserved=True, TRAIN_permission=False)
    monkeypatch.setattr(tool, "contract", lambda: copy.deepcopy(wanted))
    monkeypatch.setattr(tool.shutil, "disk_usage", lambda _: SimpleNamespace(free=tool.MINIMUM_FREE_BYTES))
    leading = {}
    for arm in tool.paired.ARMS:
        run = runs/arm
        owner = dict(worker_exit_code=2, cleanup_errors=[], source_hashes_match=True)
        plan = dict(schema=tool.paired.OWNER_SCHEMA, arm=arm, output=str(run),
            completed_pair_authority=str(pair_authority), completed_pair_authority_sha256=pair_sha)
        diagnostic = dict(status=tool.FAILED_STATUS, original_owner=owner, original_capture_status="failed",
            full_MAIN9_completion_claimed=False, full_RTE03_approved=False, training_data_approved=False,
            vehicle_control_approved=False, model_promoted=False, original_error="real failed turn")
        for name, value in zip(tool.LEADING_NAMES, (plan, owner, diagnostic)):
            leading[str(run/name)] = write_json(run/name, value)
        (run/"episode"/"tail").mkdir(parents=True)
        (run/"episode"/"tail"/"failed_camera.jpeg").write_bytes(b"synthetic actual tail bytes")
        (run/"episode"/"states.jsonl").write_text('{"failed_state":true}\n')
        (run/"source").mkdir(); (run/"source"/"archived.py").write_text("# immutable archived source\n")
    authority = dict(schema=tool.SCHEMA+".external_ROOT_authority", contract=copy.deepcopy(wanted),
        caller_preparation_authorized=True, publication_authorized=False, renderer_authorized=False,
        model_execution_authorized=False, TRAIN_authorized=False, original_outcome_file_sha256=leading)
    authority_path = plans/"root_authority.json"; authority_sha = write_json(authority_path, authority)
    return SimpleNamespace(root=root, runs=runs, output=output, source=source, helper=helper, font=font,
        pair=pair_authority, pair_sha=pair_sha, wanted=wanted, authority=authority,
        authority_path=authority_path, authority_sha=authority_sha)


def invoke(study):
    return tool.prepare(study.authority_path, study.authority_sha)


def rewrite_authority(study):
    study.authority_sha = write_json(study.authority_path, study.authority)


def change_original(study, arm, name, change, *, update_external_pin=False):
    path = study.runs/arm/name; value = json.loads(path.read_bytes()); change(value)
    actual = write_json(path, value)
    if update_external_pin:
        study.authority["original_outcome_file_sha256"][str(path)] = actual
        rewrite_authority(study)


def test_complete_archives_retained_without_publication(study):
    originals = {str(p): p.read_bytes() for p in study.runs.rglob("*") if p.is_file()}
    result = invoke(study)
    assert result["status"] == "CALLERS_PREPARED_ROOT_PUBLICATION_REVIEW_REQUIRED"
    assert not any(result[k] for k in ("original_raw_rewritten", "publication_executed", "renderer_executed", "model_executed", "TRAIN_permission"))
    assert set(p.name for p in study.output.iterdir()) == {"control_caller_pins.json", "intrinsic_curve_caller_pins.json", "summary.json"}
    for arm in tool.paired.ARMS:
        caller = json.loads((study.output/(arm+"_caller_pins.json")).read_bytes())
        assert caller["schema"] == tool.paired.CALLER_SCHEMA and caller["arm"] == arm and caller["attempts"] == 1
        assert caller["run_files_sha256"] == tool.inventory(study.runs/arm)
        assert "episode/tail/failed_camera.jpeg" in caller["run_files_sha256"]
        assert caller["actual_publication_authorized"] is False
        assert caller["completed_pair_authority_sha256"] == study.pair_sha
        assert caller["PNG_index_policy"] == "first_quarter_half_threequarters_exact_last"
        assert result["callers"][arm]["status"] == tool.FAILED_STATUS
        assert tool.digest(Path(result["callers"][arm]["path"])) == result["callers"][arm]["sha256"]
        assert not Path(caller["output"]).exists()
    assert all(Path(path).read_bytes() == raw for path, raw in originals.items())


@pytest.mark.parametrize("field,value", [("caller_preparation_authorized", False), ("caller_preparation_authorized", 1),
    ("publication_authorized", True), ("renderer_authorized", True), ("model_execution_authorized", True),
    ("TRAIN_authorized", True), ("schema", "unknown")])
def test_authority_denied_before_archive_access(study, monkeypatch, field, value):
    study.authority[field] = value; rewrite_authority(study)
    monkeypatch.setattr(tool, "inventory", lambda _: pytest.fail("unauthorized archive access"))
    with pytest.raises(RuntimeError): invoke(study)
    assert not study.output.exists()


@pytest.mark.parametrize("mutation", ["wrong_sha", "missing_pin", "extra_pin", "changed_contract"])
def test_root_pins_and_exact_contract_required(study, mutation):
    if mutation == "wrong_sha": study.authority_sha = "0"*64
    else:
        if mutation == "missing_pin": study.authority["original_outcome_file_sha256"].pop(next(iter(study.authority["original_outcome_file_sha256"])))
        elif mutation == "extra_pin": study.authority["original_outcome_file_sha256"][str(study.root/"other.json")] = "0"*64
        else: study.authority["contract"]["PNG_index_policy"] = "cherry_pick"
        rewrite_authority(study)
    with pytest.raises(RuntimeError): invoke(study)
    assert not study.output.exists()


@pytest.mark.parametrize("name,field,value", [("declaration.json", "arm", "intrinsic_curve"),
    ("declaration.json", "schema", "unknown"), ("declaration.json", "output", "/foreign/run"),
    ("declaration.json", "completed_pair_authority_sha256", "0"*64),
    ("diagnostic_report.json", "status", "PASS_KNOWN_DEV_RIGHT_TURN_PRECURSOR"),
    ("diagnostic_report.json", "training_data_approved", True),
    ("diagnostic_report.json", "vehicle_control_approved", 0),
    ("diagnostic_report.json", "original_owner", {})])
def test_authentic_original_failures_not_rewritten_or_reinterpreted(study, name, field, value):
    change_original(study, "control", name, lambda v: v.update({field: value}), update_external_pin=True)
    with pytest.raises(RuntimeError): invoke(study)
    assert not study.output.exists()


def test_original_outcome_byte_mutation_rejected(study):
    change_original(study, "control", "owner_result.json", lambda v: v.update(error="changed"))
    with pytest.raises(RuntimeError): invoke(study)
    assert not study.output.exists()


@pytest.mark.parametrize("target", ["source", "archive", "new_archive_file"])
def test_mutation_during_inventory_fails_before_output(study, monkeypatch, target):
    original = tool.inventory; called = False
    def mutating(run):
        nonlocal called
        result = original(run)
        if not called:
            called = True
            if target == "source": study.source.write_text("changed source")
            elif target == "archive": (Path(run)/"episode/states.jsonl").write_text("changed original")
            else: (Path(run)/"new_raw.json").write_text("{}")
        return result
    monkeypatch.setattr(tool, "inventory", mutating)
    with pytest.raises(RuntimeError): invoke(study)
    assert not study.output.exists()


@pytest.mark.parametrize("target", ["existing", "archive", "publication", "symlink", "symlink_ancestor"])
def test_create_only_outside_originals_and_publication(study, monkeypatch, target):
    if target == "existing": study.output.mkdir()
    elif target == "archive": monkeypatch.setattr(tool, "OUTPUT", study.runs/"control"/"new_metadata")
    elif target == "publication": monkeypatch.setattr(tool, "OUTPUT", Path(study.wanted["publication_outputs"]["control"])/"metadata")
    elif target == "symlink": study.output.symlink_to(study.source)
    else:
        link = study.output.parent/"linked"; link.symlink_to(study.root, target_is_directory=True)
        monkeypatch.setattr(tool, "OUTPUT", link/"new_metadata")
    with pytest.raises(RuntimeError): invoke(study)
    assert not (study.output/"summary.json").exists()


def test_archive_symlink_refused(study):
    (study.runs/"control"/"linked.jpeg").symlink_to(study.font)
    with pytest.raises(RuntimeError): invoke(study)
    assert not study.output.exists()


def test_floor_checked_before_any_output(study, monkeypatch):
    monkeypatch.setattr(tool.shutil, "disk_usage", lambda _: SimpleNamespace(free=tool.MINIMUM_FREE_BYTES-1))
    with pytest.raises(RuntimeError): invoke(study)
    assert not study.output.exists()


def test_postwrite_change_keeps_partial_fresh_output_not_authorized(study, monkeypatch):
    original = tool.recheck; called = 0
    def mutating(pins, tables):
        nonlocal called
        called += 1
        if called == 2: study.source.write_text("changed after caller write")
        return original(pins, tables)
    monkeypatch.setattr(tool, "recheck", mutating)
    with pytest.raises(RuntimeError): invoke(study)
    assert study.output.is_dir() and not (study.output/"summary.json").exists()
    assert all(json.loads(p.read_bytes())["actual_publication_authorized"] is False for p in study.output.glob("*_caller_pins.json"))


def test_print_contract_does_not_prepare_or_read_archives(study, monkeypatch, capsys):
    monkeypatch.setattr(tool, "prepare", lambda *_: pytest.fail("contract mode cannot prepare"))
    assert tool.main(["--print-contract"]) == 0
    assert json.loads(capsys.readouterr().out)["TRAIN_permission"] is False
    assert not study.output.exists()


def test_parsed_duplicate_keys_rejected(study):
    path = study.root/"ambiguous.json"; path.write_text('{"a":1,"a":2}')
    with pytest.raises(RuntimeError): tool.parsed(path, tool.digest(path), {})
