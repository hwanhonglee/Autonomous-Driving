"""HH_261008 - Scalar/tmp-tree status fixtures; no real process probe or job.

Fixtures deliberately monkeypatch the live generation and FD observer. Passing
them does not establish real RUNNING, training, feature approval or model parity.
"""
from __future__ import annotations

import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/maintenance/report_workspace_status_20261008_v1.py"


@pytest.fixture
def helper(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location("_hh_workspace_status_pure", SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "ROOT", tmp_path)
    monkeypatch.setattr(module, "HERE", tmp_path / module.TOOL_NAMES[0])
    monkeypatch.setattr(module, "LOCK", tmp_path / "lock")
    return module


def store(helper, path, value, raw=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    data = value if raw else helper.encoded(value)
    path.write_bytes(data)
    return dict(path=str(path), sha256=hashlib.sha256(data).hexdigest())


@pytest.fixture
def campaign(helper):
    root = helper.ROOT
    sources = {}
    for name in helper.TOOL_NAMES:
        sources[name] = store(helper, root / name, b"# SYNTHETIC_METADATA_SOURCE_ONLY\n", raw=True)["sha256"]
    index = store(helper, root / "index.md", b"SYNTHETIC curated 0/9, 0/30\n", raw=True)
    plan = store(helper, root / "plan.json", dict(schema="SYNTHETIC_LOCAL_PLAN"))
    authority = store(helper, root / "authority.json", dict(plan_path=plan["path"], plan_sha256=plan["sha256"]))
    result = store(helper, root / "result.json", dict(status="FAILED_RETAINED", cleanup_errors=[]))
    post = store(helper, root / "post.json", dict(status="SYNTHETIC_METADATA_CHECK_COMPLETE", model_promoted=False))
    entry = dict(name="SYNTHETIC_LATEST_FAILED_RUNTIME", mode="completed", declared_outcome="FAIL",
        evidence=dict(plan=plan, authority=authority, result=result, postcheck=post),
        checks=[dict(document="result", keys=["status"], expected="FAILED_RETAINED"),
            dict(document="postcheck", keys=["status"], expected="SYNTHETIC_METADATA_CHECK_COMPLETE")], live=None)
    manifest = dict(schema=helper.SCHEMA, provenance="HH_261008 - SYNTHETIC fixture, not a ROOT status authority.",
        tool_sha256=sources, curated_index=dict(reference=index, main_approved=0, sub_approved=0,
            scope="CURATED_INDEX_NOT_TEST_PASS_OR_AUTOMATIC_PROMOTION"), entries=[entry])
    def persist():
        return store(helper, root / "manifest.json", manifest)
    return SimpleNamespace(manifest=manifest, entry=entry, persist=persist)


def run_report(helper, campaign):
    reference = campaign.persist()
    return helper.report(reference["path"], reference["sha256"])


def test_failed_experiment_is_stored_completed_not_approved_or_running(helper, campaign):
    value = run_report(helper, campaign)
    assert value["entries"][0]["status"] == "STORED_COMPLETED_WITH_VERIFIED_METADATA"
    assert value["entries"][0]["outcome"] == "FAIL"
    assert value["entries"][0]["live_observation"]["status"] == "UNKNOWN_ACTIVE_OR_STOPPED"
    assert value["formal_main_approved"] == value["formal_sub_approved"] == 0
    assert value["formal_main_total"] == 9 and value["formal_sub_total"] == 30
    assert value["full_input_or_model_rehash"] is value["independent_experiment_repeat"] is False
    assert value["training_or_NN_or_ROS_started"] is value["remote_checked"] is value["files_written"] is False
    assert len(helper.encoded(value)) <= helper.OUTPUT_LIMIT


def test_prepared_is_never_claimed_running(helper, campaign):
    entry = campaign.entry
    entry.update(mode="prepared", declared_outcome="PENDING", checks=[], live=None)
    entry["evidence"] = {"plan": entry["evidence"]["plan"]}
    assert run_report(helper, campaign)["entries"][0]["status"] == "PREPARED_NOTRUNNING"


@pytest.mark.parametrize("fault", ["missing_result", "result_SHA", "post_status", "duplicate_names", "boolean_count", "wrong_count", "unknown_source", "bad_manifest_SHA"])
def test_manifest_and_stored_integrity_faults_deny_or_are_explicit_unknown(helper, campaign, fault):
    if fault == "missing_result":
        Path(campaign.entry["evidence"]["result"]["path"]).unlink()
    elif fault == "result_SHA":
        campaign.entry["evidence"]["result"]["sha256"] = "f" * 64
    elif fault == "post_status":
        campaign.entry["checks"][1]["expected"] = "UNPROVED_PASS"
    elif fault == "duplicate_names":
        campaign.manifest["entries"].append(deepcopy(campaign.entry))
    elif fault == "boolean_count":
        campaign.manifest["curated_index"]["main_approved"] = False
    elif fault == "wrong_count":
        campaign.manifest["curated_index"]["sub_approved"] = 1
    elif fault == "unknown_source":
        campaign.manifest["tool_sha256"]["unapproved.py"] = "a" * 64
    if fault in ("missing_result", "result_SHA", "post_status"):
        value = run_report(helper, campaign)
        assert value["entries"][0]["status"] == "UNKNOWN_STORED_EVIDENCE"
    else:
        reference = campaign.persist()
        with pytest.raises(ValueError):
            helper.report(reference["path"], "f" * 64 if fault == "bad_manifest_SHA" else reference["sha256"])


@pytest.mark.parametrize("raw", [b'{"a":1,"a":2}', b'{"a":NaN}', b'{"a":Infinity}', b'{"a":-Infinity}'])
def test_strict_JSON_denies_duplicates_and_nonfinite(helper, raw):
    with pytest.raises(ValueError):
        helper.decoded(raw)


def test_type_sensitive_metadata_assertions_do_not_rewrite_float_as_int(helper, campaign):
    result = store(helper, helper.ROOT / "result2.json", dict(gradient_clip_l2=5.0))
    campaign.entry["evidence"]["result"] = result
    campaign.entry["checks"][0] = dict(document="result", keys=["gradient_clip_l2"], expected=5)
    assert run_report(helper, campaign)["entries"][0]["status"] == "UNKNOWN_STORED_EVIDENCE"


@pytest.mark.parametrize("fault", ["symlink", "hardlink", "oversize", "foreign_root", "nonmetadata"])
def test_file_scope_and_storage_bounds(helper, tmp_path, fault):
    path = helper.ROOT / "data.json"
    reference = store(helper, path, dict(SYNTHETIC=True))
    if fault == "symlink":
        alias = helper.ROOT / "alias.json"; alias.symlink_to(path); reference["path"] = str(alias)
    elif fault == "hardlink":
        os.link(path, helper.ROOT / "alias.json")
    elif fault == "oversize":
        path.write_bytes(b"x" * (helper.INPUT_LIMIT + 1))
    elif fault == "foreign_root":
        reference["path"] = str(tmp_path.parent / "outside.json")
    else:
        renamed = path.with_suffix(".pt"); path.rename(renamed); reference["path"] = str(renamed)
    with pytest.raises((ValueError, FileNotFoundError)):
        helper.MetadataReader().read(reference)


def test_all32_inputs_counted_and_same_size_mutation_cannot_reuse_receipt(helper):
    reader = helper.MetadataReader()
    references = [store(helper, helper.ROOT / (str(n) + ".json"), dict(n=n)) for n in range(33)]
    for reference in references[:32]:
        reader.read(reference)
    with pytest.raises(ValueError, match="maximum32"):
        reader.read(references[32])
    path = Path(references[0]["path"]); raw = path.read_bytes(); path.write_bytes(raw.replace(b"0", b"1"))
    with pytest.raises(ValueError):
        reader.read(references[0])


def active_fixture(helper, campaign):
    entry = campaign.entry
    entry.update(mode="active", declared_outcome="PENDING", checks=[])
    entry["evidence"] = {key: entry["evidence"][key] for key in ("plan", "authority")}
    identity = dict(pid=12345, starttime=654321, ppid=123, pgid=12345, session=12345, uid=os.getuid(),
        argv=["SYNTHETIC_PRIVATE_PROCESS_NOT_EXECUTED"], boot_id="SYNTHETIC_CURRENT_BOOT")
    receipt = dict(identity=identity, plan_sha256=entry["evidence"]["plan"]["sha256"],
        authority_sha256=entry["evidence"]["authority"]["sha256"], lock_fd=9, lock_identity=dict(dev=1, ino=2))
    entry["live"] = dict(started=store(helper, helper.ROOT / "started.json", receipt))
    return identity, receipt


def test_RUNNING_requires_both_fresh_same_generation_and_FD_lease_observations(helper, campaign, monkeypatch):
    identity, receipt = active_fixture(helper, campaign); calls = []
    def process(pid):
        calls.append(("generation", pid)); return deepcopy(identity)
    def lease(actual, declared):
        calls.append(("lease", actual["pid"])); assert declared == receipt
        return dict(fd=9, dev=1, ino=2, observation="SYNTHETIC_ONLY")
    monkeypatch.setattr(helper, "process_identity", process); monkeypatch.setattr(helper, "lease_probe", lease)
    value = run_report(helper, campaign)["entries"][0]
    assert value["status"] == "RUNNING" and calls == [("generation",12345),("lease",12345),("generation",12345)]
    assert value["scope"] == "FRESH_POINT_IN_TIME_NOT_CONTINUOUS_LIVENESS"


def test_historical_started_without_boot_never_probes_or_claims_running(helper, campaign, monkeypatch):
    _, receipt = active_fixture(helper, campaign); receipt["identity"].pop("boot_id")
    campaign.entry["live"]["started"] = store(helper, helper.ROOT / "started.json", receipt)
    monkeypatch.setattr(helper, "process_identity", lambda _: pytest.fail("no boot: no real process observation"))
    assert run_report(helper, campaign)["entries"][0]["status"] == "UNKNOWN_ACTIVE_OR_STOPPED"


@pytest.mark.parametrize("fault", ["PID_reuse", "boot", "argv", "UID", "permission", "absent", "lease", "race", "boolean_generation", "authority"])
def test_active_generation_unknown_not_false_running(helper, campaign, monkeypatch, fault):
    identity, receipt = active_fixture(helper, campaign); calls = []
    if fault == "boolean_generation":
        receipt["identity"]["starttime"] = True
    elif fault == "authority":
        receipt["authority_sha256"] = "f" * 64
    elif fault == "UID":
        receipt["identity"]["uid"] = os.getuid() + 1
    campaign.entry["live"]["started"] = store(helper, helper.ROOT / "started.json", receipt)
    def process(_):
        calls.append(1)
        if fault == "permission": raise PermissionError("SYNTHETIC unavailable /proc")
        if fault == "absent": raise FileNotFoundError("SYNTHETIC old generation absent")
        value = deepcopy(identity)
        if fault == "PID_reuse" or fault == "race" and len(calls) == 2: value["starttime"] += 1
        if fault == "boot": value["boot_id"] = "SYNTHETIC_OTHER_BOOT"
        if fault == "argv": value["argv"] = ["SYNTHETIC_OTHER_PROCESS"]
        return value
    def lease(*_):
        if fault == "lease": raise ValueError("SYNTHETIC replaced lock/FD")
        return dict(SYNTHETIC=True)
    monkeypatch.setattr(helper, "process_identity", process); monkeypatch.setattr(helper, "lease_probe", lease)
    value = run_report(helper, campaign)["entries"][0]
    assert value["status"].startswith("UNKNOWN") and value["status"] != "RUNNING"
    if fault in ("boolean_generation", "authority", "UID"): assert calls == []


def test_template_is_unfilled_not_status_authority(helper):
    value = helper.manifest_template()
    assert value["schema"] == helper.SCHEMA and value["entries"][0]["live"] is None
    assert all(not helper.sha64(pin) for pin in value["tool_sha256"].values())
    assert value["curated_index"]["main_approved"] == value["curated_index"]["sub_approved"] == 0


def test_source_has_no_job_framework_remote_write_or_glob_latest_api():
    tree = ast.parse(SOURCE.read_bytes())
    imports = {node.names[0].name.split(".")[0] for node in tree.body if isinstance(node, ast.Import)}
    assert not imports & {"subprocess","torch","numpy","PIL","matplotlib","rclpy","carla","paramiko"}
    calls = {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)}
    assert not any(name.endswith((".glob", ".rglob", ".unlink", ".write_text", ".write_bytes", ".kill", ".killpg", ".flock")) for name in calls)
    assert "process_identity" in calls and "lease_probe" in calls and "os.readlink" in calls
    assert "main_approved" in SOURCE.read_text() and "CURATED_INDEX_NOT_TEST_PASS_OR_AUTOMATIC_PROMOTION" in SOURCE.read_text()
    old = ROOT / "tools/workspace.py"
    assert hashlib.sha256(old.read_bytes()).hexdigest() == "4064e9c28585141a36abe8de05360a4d937154523a95bb265ee8434bf77ced0f"
