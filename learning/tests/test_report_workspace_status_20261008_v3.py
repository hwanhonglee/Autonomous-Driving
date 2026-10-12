"""HH_261008 - V3 known-map schema joins; preserve tested V2 and all existing scalar guards.

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
SOURCE = ROOT / "tools/maintenance/report_workspace_status_20261008_v3.py"


@pytest.fixture
def helper(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location("_hh_workspace_status_v3_pure", SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "ROOT", tmp_path)
    monkeypatch.setattr(module, "HERE", tmp_path / module.TOOL_NAMES[0])
    monkeypatch.setattr(module, "LOCK", tmp_path / "lock")
    monkeypatch.setattr(module, "MAP_V3_OUTPUT", tmp_path / "storage/experiments/diagnostics/2026-10-08/map_paired_sign_regions_v3")
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
    preserved = {}
    for number, name in enumerate(helper.PRESERVED_SOURCE_PINS):
        preserved[name] = store(helper, root / name, ("SYNTHETIC_STOPPED_SOURCE_" + str(number)).encode(), raw=True)["sha256"]
    helper.PRESERVED_SOURCE_PINS = preserved
    manifest = dict(schema=helper.SCHEMA, provenance="HH_261008 - SYNTHETIC fixture, not a ROOT status authority.",
        tool_sha256=sources, preserved_source_sha256=preserved, curated_index=dict(reference=index, main_approved=0, sub_approved=0,
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


# HH_261008 - Exact disconnected grants and hidden float overflow are denied before any process observation.
@pytest.mark.parametrize("mode", ["completed", "active"])
@pytest.mark.parametrize("fault", ["plan_sha", "plan_path"])
def test_disconnected_exact_four_evidence_or_active_pair_DENIED(helper, campaign, monkeypatch, mode, fault):
    if mode == "active":
        active_fixture(helper, campaign)
    authority = dict(plan_path=campaign.entry["evidence"]["plan"]["path"],
        plan_sha256=campaign.entry["evidence"]["plan"]["sha256"])
    authority["plan_sha256" if fault == "plan_sha" else "plan_path"] = "f" * 64 if fault == "plan_sha" else "/SYNTHETIC_OTHER_PLAN.json"
    campaign.entry["evidence"]["authority"] = store(helper, helper.ROOT / "wrong_authority.json", authority)
    monkeypatch.setattr(helper, "process_identity", lambda _: pytest.fail("disconnected authority must not probe"))
    value = run_report(helper, campaign)["entries"][0]
    assert value["status"] == "UNKNOWN_STORED_EVIDENCE" and "authority/plan" in value["reason"]


@pytest.mark.parametrize("number", [b"1e309", b"-1e309"])
def test_overflow_anywhere_unchecked_nested_fields_is_denied(helper, campaign, number):
    raw = b'{"status":"FAILED_RETAINED","unchecked":{"deep":[' + number + b']}}'
    campaign.entry["evidence"]["result"] = store(helper, helper.ROOT / "overflow.json", raw, raw=True)
    assert run_report(helper, campaign)["entries"][0]["status"] == "UNKNOWN_STORED_EVIDENCE"
    with pytest.raises(ValueError, match="overflow/nonfinite"):
        helper.decoded(raw)


def test_ordinary_float_types_and_quoted_marker_are_preserved(helper):
    value = helper.decoded(b'{"typed_float":5.0,"typed_int":5,"marker":"+Infinity","nested":[1e-3]}')
    assert type(value["typed_float"]) is float and value["typed_float"] == 5.0
    assert type(value["typed_int"]) is int and value["typed_int"] == 5
    assert type(value["marker"]) is str and value["marker"] == "+Infinity"
    assert type(value["nested"][0]) is float and value["nested"][0] == .001


def test_deep_JSON_is_denied_without_partial_document(helper):
    raw = b'{"nested":' + b'[' * 2000 + b'0' + b']' * 2000 + b'}'
    with pytest.raises(ValueError, match="nesting"):
        helper.decoded(raw)


@pytest.mark.parametrize("expected", [[], {"unknown": True}])
def test_nonscalar_assertions_are_not_generic_object_admission(helper, campaign, expected):
    campaign.entry["checks"][0] = dict(document="result", keys=["cleanup_errors"], expected=expected)
    assert run_report(helper, campaign)["entries"][0]["status"] == "UNKNOWN_STORED_EVIDENCE"


def test_preserved_V1_and_finding_refs_count_toward_same_input_bound(helper, campaign):
    value = run_report(helper, campaign)
    assert value["preserved_source_sha256"] == helper.PRESERVED_SOURCE_PINS
    paths = {record["path"] for record in value["metadata_reads"]}
    assert all(str(helper.ROOT / name) in paths for name in helper.PRESERVED_SOURCE_PINS)
    assert helper.INPUT_COUNT_LIMIT == 32 and helper.INPUT_LIMIT == 4 * 1024**2
    assert helper.OUTPUT_LIMIT == 256 * 1024
    assert value["automatic_result_postcheck_semantic_join_claimed"] is False


def test_actual_V1_two_sources_and_ROOT_finding_unchanged_without_import():
    pins = {
        "tools/maintenance/report_workspace_status_20261008_v1.py": "cacbd9b8481b9eee52d5cd249eebe9292c5da00888c5340180c1d99c7a65beab",
        "learning/tests/test_report_workspace_status_20261008_v1.py": "1ffb0bb0a54f19e7b2a65682360951ecd7060070a6a4ecd3f1dde07cd96632a6",
        "storage/experiments/diagnostics/2026-10-08/workspace_status_v1_source_finding_v1.json": "1b64552ead0e7a7c06c03e155cee03122668bdfeff6076d690946a94bd775c06",
    }
    for name, pin in pins.items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == pin
    assert (ROOT / "tools/maintenance/report_workspace_status_20261008_v1.py").name not in SOURCE.name


# HH_261008 - These are symbolic JSON metadata, not actual map masks or a ROOT run grant.
@pytest.fixture
def map_campaign(helper, campaign):
    entry = campaign.entry
    candidate = dict(schema=helper.MAP_V3 + ".prepared_candidate",
        contract=dict(schema=helper.MAP_V3), output=str(helper.MAP_V3_OUTPUT),
        source_sha256={"SYNTHETIC_NOT_LOADED.py": "a" * 64})
    plan = store(helper, helper.ROOT / "prepared_candidate.json", candidate)
    review = dict(candidate_RAW_sha256=plan["sha256"], source_sha256=candidate["source_sha256"], CPU_affinity=[3])
    review_ref = store(helper, helper.ROOT / "review.json", review)
    authority = dict(schema=helper.MAP_V3 + ".ROOT_authority", provenance="HH_261008 - SYNTHETIC scalar fixture only.",
        candidate_sha256=plan["sha256"], review_sha256=review_ref["sha256"], output=str(helper.MAP_V3_OUTPUT),
        CPU_affinity=[3], stored_mask_region_diagnostic_authorized=True,
        training_authorized=False, model_inference_authorized=False)
    auth_ref = store(helper, helper.ROOT / "map_authority.json", authority)
    result = dict(schema=helper.MAP_V3, candidate_sha256=plan["sha256"], authority_sha256=auth_ref["sha256"],
        status="COMPLETE_DESCRIPTIVE_PAIRED_MASK_REGIONS_NOT_MODEL_APPROVAL")
    result_ref = store(helper, helper.MAP_V3_OUTPUT / "result.json", result)
    entry["evidence"] = dict(plan=plan, authority=auth_ref, review=review_ref,
        result=result_ref, postcheck=entry["evidence"]["postcheck"])
    entry["declared_outcome"] = "DIAGNOSTIC_ONLY"
    entry["checks"][0]["expected"] = result["status"]
    return SimpleNamespace(campaign=campaign, candidate=candidate, review=review, authority=authority, result=result)


def test_known_map_completed_RAW_candidate_review_output_join_not_feature_approval(helper, map_campaign):
    value = run_report(helper, map_campaign.campaign)
    entry = value["entries"][0]
    assert entry["status"] == "STORED_COMPLETED_WITH_VERIFIED_METADATA"
    assert entry["outcome"] == "DIAGNOSTIC_ONLY"
    assert entry["authority_join"] == "MAP_PAIRED_SIGN_REGIONS_V3_RAW_CANDIDATE_REVIEW_OUTPUT"
    assert value["formal_main_approved"] == value["formal_sub_approved"] == 0
    assert value["full_input_or_model_rehash"] is False
    assert value["historical_V2_ROOT_scalar_passes"] == 47
    assert value["historical_V2_actual_status_CLI_invocations"] == 0


def test_known_map_active_has_join_but_no_invented_running_identity(helper, map_campaign, monkeypatch):
    entry = map_campaign.campaign.entry
    entry.update(mode="active", declared_outcome="PENDING", checks=[])
    entry["evidence"] = {k: entry["evidence"][k] for k in ("plan", "authority", "review")}
    monkeypatch.setattr(helper, "process_identity", lambda _: pytest.fail("no invented boot-bound map owner"))
    value = run_report(helper, map_campaign.campaign)["entries"][0]
    assert value["status"] == "UNKNOWN_ACTIVE_OR_STOPPED"
    assert value["authority_join"] == "MAP_PAIRED_SIGN_REGIONS_V3_RAW_CANDIDATE_REVIEW_OUTPUT"


@pytest.mark.parametrize("fault", ["missing_review", "candidate_SHA", "review_SHA", "authority_output", "authority_extra",
    "authority_schema", "training", "inference", "CPU", "candidate_schema", "contract_schema", "candidate_output",
    "review_candidate", "review_sources", "review_CPU", "result_schema", "result_candidate", "result_authority", "result_output"])
def test_known_map_disconnected_or_unknown_schema_DENY_without_live_probe(helper, map_campaign, monkeypatch, fault):
    entry = map_campaign.campaign.entry
    evidence = entry["evidence"]
    if fault == "missing_review":
        evidence.pop("review")
    elif fault in ("candidate_SHA", "review_SHA", "authority_output", "authority_extra", "authority_schema", "training", "inference", "CPU"):
        authority = deepcopy(map_campaign.authority)
        key, value = {
            "candidate_SHA": ("candidate_sha256", "f" * 64),
            "review_SHA": ("review_sha256", "f" * 64),
            "authority_output": ("output", str(helper.ROOT / "wrong_output")),
            "authority_extra": ("unreviewed", True),
            "authority_schema": ("schema", helper.MAP_V3 + ".UNKNOWN_authority"),
            "training": ("training_authorized", True),
            "inference": ("model_inference_authorized", True),
            "CPU": ("CPU_affinity", [4]),
        }[fault]
        authority[key] = value
        evidence["authority"] = store(helper, helper.ROOT / "wrong_authority.json", authority)
    elif fault in ("candidate_schema", "contract_schema", "candidate_output"):
        candidate = deepcopy(map_campaign.candidate)
        if fault == "candidate_schema": candidate["schema"] = "UNKNOWN"
        elif fault == "contract_schema": candidate["contract"]["schema"] = "UNKNOWN"
        else: candidate["output"] = str(helper.ROOT / "wrong_output")
        evidence["plan"] = store(helper, helper.ROOT / "wrong_candidate.json", candidate)
    elif fault.startswith("review_"):
        review = deepcopy(map_campaign.review)
        key, value = {
            "review_candidate": ("candidate_RAW_sha256", "f" * 64),
            "review_sources": ("source_sha256", {"OTHER_SYNTHETIC.py": "b" * 64}),
            "review_CPU": ("CPU_affinity", [4]),
        }[fault]
        review[key] = value
        evidence["review"] = store(helper, helper.ROOT / "wrong_review.json", review)
        # HH_261008 - Refresh outer RAW seals so the intended inner review join is exercised.
        authority = deepcopy(map_campaign.authority)
        authority["review_sha256"] = evidence["review"]["sha256"]
        evidence["authority"] = store(helper, helper.ROOT / "resealed_authority.json", authority)
        result = deepcopy(map_campaign.result)
        result["authority_sha256"] = evidence["authority"]["sha256"]
        evidence["result"] = store(helper, helper.MAP_V3_OUTPUT / "resealed_result.json", result)
    else:
        result = deepcopy(map_campaign.result)
        if fault == "result_schema": result["schema"] = "UNKNOWN"
        elif fault == "result_candidate": result["candidate_sha256"] = "f" * 64
        elif fault == "result_authority": result["authority_sha256"] = "f" * 64
        result_path = helper.ROOT / "wrong_output/result.json" if fault == "result_output" else helper.MAP_V3_OUTPUT / "other_result.json"
        evidence["result"] = store(helper, result_path, result)
    monkeypatch.setattr(helper, "process_identity", lambda _: pytest.fail("disconnected metadata must not probe"))
    value = run_report(helper, map_campaign.campaign)["entries"][0]
    assert value["status"] == "UNKNOWN_STORED_EVIDENCE"
    if fault == "review_sources": assert "source metadata tables" in value["reason"]
    if fault == "review_CPU": assert "CPU-only diagnostic" in value["reason"]


def test_legacy_authority_does_not_accept_extra_map_review_role(helper, campaign):
    campaign.entry["evidence"]["review"] = store(helper, helper.ROOT / "review.json", dict(SYNTHETIC=True))
    assert run_report(helper, campaign)["entries"][0]["status"] == "UNKNOWN_STORED_EVIDENCE"


def test_V2_running_observers_finite_parser_and_bounds_remain_exact_AST():
    old = ast.parse((ROOT / "tools/maintenance/report_workspace_status_20261008_v2.py").read_text())
    new = ast.parse(SOURCE.read_text())
    functions = lambda tree: {n.name: n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
    before, after = functions(old), functions(new)
    for name in before:
        if name not in {"entry_status", "report", "manifest_template"}:
            assert ast.dump(before[name], include_attributes=False) == ast.dump(after[name], include_attributes=False)
    assert "authority_join" in after
    assert "MAP_PAIRED_SIGN_REGIONS_V3_RAW_CANDIDATE_REVIEW_OUTPUT" in SOURCE.read_text()


def test_preserved_V2_two_sources_and_V1_findings_are_exact_without_import():
    pins = {
        "tools/maintenance/report_workspace_status_20261008_v2.py": "de48b0940a588c975d0f550c64bffc5370bf272793225f53277cfa2b4d597dd9",
        "learning/tests/test_report_workspace_status_20261008_v2.py": "6404dcc9abb63b934c465a72eab528115fa5e1a214e001e7aa6b34d325b935ca",
    }
    for name, pin in pins.items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == pin
