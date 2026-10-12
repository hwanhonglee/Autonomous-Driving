"""HH_261004 - Verify prospective V2 media policy and pure full-denominator publishing.

Synthetic scalar/tempfile fixtures are never driving evidence.  No actual
experiment, camera payload, decoder, model, simulator or remote host is used.
"""

import copy
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from scripts.e2e import publish_temporal_route_joint_town03_straight_full209_v2_20261004 as m


def bundle(native_count):
    # HH_261004 - Unknown actual denominators include odd final native frames and complete failed tails.
    states = [dict(frame=100+index, timestamp=5.+index*.05) for index in range(native_count)]
    return dict(states=states, cameras=[dict(frame=row["frame"], timestamp=row["timestamp"]) for row in states[::2]])


def selection():
    return dict(gallery=str(m.OUTPUT.relative_to(m.ROOT)),
        GIF_camera_indices="every5th actual camera bundle starting0 plus exact last; unique sorted",
        GIF_playback="all selected timestamp intervals plus original final0.1s camera cadence",
        PNG_camera_indices="floor((actual_camera_count-1)*num/4), num=0,1,2,3,4; unique sorted",
        complete_actual_interval_including_failures_required=True, outcome_or_failure_dependent_frame_selection=False,
        real_six_cameras_required=True, vehicle_centered=True,
        original_full_route_and_actual_travel_and_prior_learned_ACK_trajectory_required=True,
        final_native_state_and_final_camera_timestamp_distinction_retained=True,
        actual_Autoware_or_RViz_screenshot_claim=False, illustrative_or_generated_image_allowed=False)


def plan_values():
    return dict(schema=m.OWNER_SCHEMA, output=str(m.RUN), declaration_path=str(m.PLAN_PATH), run_id="v2",
        town="Town03", scenario="straight", dataset_split="val", capture_raw_ABI_schema=m.CAPTURE_SCHEMA,
        attempts=1, automatic_retry=False, total_capture_seconds=110,
        maximum_native_state_count=2200, maximum_camera_bundle_count=1100,
        original_navigation_sha256=m.ROUTE_SHA, original_catalog_goal_tolerance_m=2.5,
        prospective_visual_publication=selection(), source_sha256={},
        known_DEV_site_reused=True, independent_validation=False,
        full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False,
        learned_STOP_claim=False)


class FakePins:
    # HH_261004 - Only schema/reference checks are synthetic; the sealer tests below hash real temporary file bytes.
    def __init__(self, documents):
        self.documents, self.files = documents, []
    def file(self, path, sha):
        self.files.append((Path(path), sha))
    def json(self, path):
        return self.documents[Path(path)]


def policy_values():
    plan = plan_values()
    policy = m.policy_document(plan, "a"*64)
    caller = dict(prospective_publication_policy=dict(path=str(m.POLICY_PATH), sha256="b"*64),
        run_files_sha256={"declaration.json": "a"*64})
    return caller, FakePins({m.POLICY_PATH: policy, m.PLAN_PATH: plan}), plan, policy


def test_fresh_publisher_has_separate_identity_and_authentic_pure_source_closure():
    assert m.OWNER_SCHEMA.endswith(".v2") and m.CAPTURE_SCHEMA.endswith(".v1")
    assert m.RUN.name.endswith("full209_v2")
    assert m.OUTPUT.name == "12_town03_full_route209_v2"
    assert "full209_v2" in str(m.POLICY_PATH.parent)
    assert "torch" not in sys.modules and "carla" not in sys.modules
    sources = m.publication_sources()
    assert Path(m.__file__).resolve() in sources
    assert not any("research_temporal_route_joint_town03_straight_full209" in str(path) for path in sources)


def test_all_prospective_unknown_camera_denominators_preserve_fixed_complete_selection():
    for count in range(1, 1101):
        indices = m.camera_indices(count)
        assert indices == sorted(set(range(0, count, 5)) | {count-1})
        assert indices[0] == 0 and indices[-1] == count-1 and len(indices) == len(set(indices))
        assert m.png_indices(count) == {name: (count-1)*numerator//4 for name, numerator in m.PNG_NUMERATORS.items()}


@pytest.mark.parametrize("bad", [0, 1101, True, -1, 1.5, "918", None])
def test_invalid_camera_denominators_cannot_select_a_prefix(bad):
    with pytest.raises(RuntimeError): m.camera_indices(bad)
    with pytest.raises(RuntimeError): m.png_indices(bad)


@pytest.mark.parametrize("native_count", [1, 2, 25, 26, 35, 36, 1755, 1836, 2199, 2200])
def test_odd_even_final_native_and_camera_hold_are_disclosed_not_fabricated(native_count):
    value = bundle(native_count)
    indices = m.camera_indices(len(value["cameras"]))
    before = copy.deepcopy(value)
    durations = m.playback(value, indices)
    assert value == before
    assert durations[-1] == 100 and sum(durations) == len(value["cameras"])*100
    assert sum(durations)-native_count*50 == (50 if native_count % 2 else 0)
    assert value["cameras"][-1]["frame"] == value["states"][-1]["frame"] - (0 if native_count % 2 else 1)


def test_failed_maximum_interval_never_changes_sampling_to_success_only():
    failed, passed = bundle(2200), bundle(2200)
    failed["diagnostic"] = dict(status="FAIL_KNOWN_DEV_FULL209_RESEARCH", learned_driving_screen_passed=False,
        original_capture_error="complete failed budget retained", catalog_goal_qualification_error="no genuine goal")
    passed["diagnostic"] = dict(status="PASS_KNOWN_DEV_FULL209_RESEARCH", learned_driving_screen_passed=True)
    indices = m.camera_indices(1100)
    assert m.playback(failed, indices) == m.playback(passed, indices)
    text = m.markdown(failed, m.png_indices(1100))
    assert "FAIL_KNOWN_DEV_FULL209_RESEARCH" in text and "complete failed budget retained" in text
    assert "no genuine goal" in text and "메인0/9·하위0/30" in text
    assert "새 카메라·차량 상태·제어 관측이 아닙니다" in text
    assert "Autoware/RViz" in text and "실패" in text


@pytest.mark.parametrize("fault", ["crop", "duplicate", "camera_stamp", "nonfinite"])
def test_playback_refuses_selection_or_timestamp_relabeling(fault):
    value = bundle(2200)
    indices = m.camera_indices(1100)
    if fault == "crop": indices = [0, 50, 1099]
    elif fault == "duplicate": indices.insert(1, 0)
    elif fault == "camera_stamp": value["cameras"][5]["timestamp"] += .01
    else: value["cameras"][5]["timestamp"] = float("nan")
    with pytest.raises(RuntimeError): m.playback(value, indices)


def test_policy_binds_fresh_owner_and_original_raw_ABI_to_declaration():
    caller, pins, plan, policy = policy_values()
    assert m.verify_policy(caller, pins, plan) is policy
    assert policy["owner_schema"] == m.OWNER_SCHEMA and policy["capture_raw_ABI_schema"] == m.CAPTURE_SCHEMA
    assert policy["actual_counts_unknown_before_capture"] is True
    assert policy["native_camera_counts_selected_from_actual_archive_later"] is True
    assert pins.files == [(m.POLICY_PATH, "b"*64), (m.PLAN_PATH, "a"*64)]


@pytest.mark.parametrize("name", ["real_six_cameras_required", "vehicle_centered",
    "original_full_route_and_actual_travel_and_prior_learned_ACK_trajectory_required",
    "final_native_state_and_final_camera_timestamp_distinction_retained"])
def test_prospective_document_requires_real_centered_route_and_native_camera_disclosures(name):
    plan = plan_values()
    plan["prospective_visual_publication"][name] = False
    with pytest.raises(RuntimeError): m.policy_document(plan, "a"*64)


@pytest.mark.parametrize("name", ["full_RTE01_approved", "full_MAIN9_completion_claimed", "training_data_approved",
    "vehicle_control_approved", "model_promoted", "learned_STOP_claim"])
def test_prospective_document_cannot_approve_a_future_episode_or_learned_stop(name):
    plan = plan_values()
    plan[name] = True
    with pytest.raises(RuntimeError): m.policy_document(plan, "a"*64)


@pytest.mark.parametrize("fault", ["owner", "raw_policy", "raw_plan", "late", "observed", "retry", "gallery",
    "GIF", "PNG", "hold", "crop", "adapt", "UI", "generated", "declaration", "approval", "independent"])
def test_policy_never_admits_cross_revision_or_outcome_adaptive_media(fault):
    caller, pins, plan, policy = policy_values()
    if fault == "owner": policy["owner_schema"] = m.OWNER_SCHEMA.replace(".v2", ".v1")
    elif fault == "raw_policy": policy["capture_raw_ABI_schema"] = m.CAPTURE_SCHEMA.replace(".v1", ".v2")
    elif fault == "raw_plan": plan["capture_raw_ABI_schema"] = "arbitrary_raw_ABI"
    elif fault == "late": policy["created_before_actual_episode"] = False
    elif fault == "observed": policy["actual_episode_observed"] = True
    elif fault == "retry": policy["automatic_retry"] = True
    elif fault == "gallery": policy["media_selection"]["gallery"] += "_other"
    elif fault == "GIF": policy["media_selection"]["GIF_camera_indices"] = "best scenes"
    elif fault == "PNG": policy["media_selection"]["PNG_camera_indices"] = "successful segment"
    elif fault == "hold": policy["media_selection"]["GIF_playback"] = "new future camera timestamp"
    elif fault == "crop": policy["media_selection"]["complete_actual_interval_including_failures_required"] = False
    elif fault == "adapt": policy["media_selection"]["outcome_or_failure_dependent_frame_selection"] = True
    elif fault == "UI": policy["media_selection"]["actual_Autoware_or_RViz_screenshot_claim"] = True
    elif fault == "generated": policy["media_selection"]["illustrative_or_generated_image_allowed"] = True
    elif fault == "declaration": caller["run_files_sha256"]["declaration.json"] = "c"*64
    elif fault == "approval": policy["full_RTE01_approved"] = True
    else: policy["independent_validation"] = True
    with pytest.raises(RuntimeError): m.verify_policy(caller, pins, plan)


@pytest.fixture
def prospective_files(tmp_path, monkeypatch):
    # HH_261004 - Patch only this new module's fixed paths into a test-owned private root.
    task_root = tmp_path / "synthetic_cpu_workspace"
    task_root.mkdir()
    run = task_root / "storage/experiments/validation/synthetic/full209_v2"
    output = task_root / "docs/assets/synthetic/12_town03_full_route209_v2"
    plan = run.with_name(run.name+"_plan.json")
    policy = task_root / "storage/experiments/plans/synthetic/full209_v2/ROOT_prospective_publication_policy.json"
    for name, value in dict(ROOT=task_root, RUN=run, OUTPUT=output, PLAN_PATH=plan, POLICY_PATH=policy).items():
        monkeypatch.setattr(m, name, value)
    source = task_root / "tools/synthetic_frozen_source.py"
    source.parent.mkdir(parents=True)
    source.write_bytes(b"# HH_261004 - Explicit synthetic CPU source, not an actual experiment.\n")
    value = plan_values()
    value["source_sha256"] = {str(source.relative_to(task_root)): hashlib.sha256(source.read_bytes()).hexdigest()}
    plan.parent.mkdir(parents=True)
    plan.write_bytes(json.dumps(value, sort_keys=True).encode())
    return dict(root=task_root, run=run, output=output, plan=plan, policy=policy, source=source,
        plan_value=value, sha256=hashlib.sha256(plan.read_bytes()).hexdigest())


def test_sealer_creates_exact_pinned_policy_before_any_actual_episode(prospective_files):
    files = prospective_files
    result = m.seal_policy(files["sha256"])
    assert result["path"] == str(files["policy"])
    assert result["sha256"] == hashlib.sha256(files["policy"].read_bytes()).hexdigest()
    assert result["created"] is True
    policy = json.loads(files["policy"].read_bytes())
    assert policy["created_before_actual_episode"] is True and policy["actual_episode_observed"] is False
    assert policy["run_declaration"] == dict(path=str(files["plan"]), sha256=files["sha256"])
    assert policy["media_selection"] == files["plan_value"]["prospective_visual_publication"]
    assert policy["owner_schema"] == m.OWNER_SCHEMA and policy["capture_raw_ABI_schema"] == m.CAPTURE_SCHEMA
    assert policy["created_UTC"] and policy["provenance"].startswith("HH_261004 - ")
    assert policy["owner_source_sha256"] == files["plan_value"]["source_sha256"]
    assert policy["root_sealer_sha256"] == hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest()
    assert not files["run"].exists() and not files["output"].exists()
    before = files["policy"].read_bytes()
    with pytest.raises(RuntimeError): m.seal_policy(files["sha256"])
    assert files["policy"].read_bytes() == before


@pytest.mark.parametrize("fault", ["existing_policy", "started", "episode", "partial", "collector_log", "output",
    "wrong_sha", "invalid_sha", "changed_source", "wrong_owner", "wrong_raw", "adaptive"])
def test_sealer_refuses_started_existing_or_mismatched_inputs_without_overwrite(prospective_files, fault):
    files = prospective_files
    expected = files["sha256"]
    if fault == "existing_policy":
        files["policy"].parent.mkdir(parents=True)
        files["policy"].write_bytes(b"untouched existing policy")
    elif fault in ("started", "episode", "partial", "collector_log"):
        files["run"].mkdir()
        if fault == "started": (files["run"] / "owner_started.json").write_bytes(b"{}")
        elif fault == "collector_log": (files["run"] / "collector.log").write_bytes(b"actual collection cannot be resealed")
        else: (files["run"] / ("episode" if fault == "episode" else "episode.partial")).mkdir()
    elif fault == "output": files["output"].mkdir(parents=True)
    elif fault == "wrong_sha": expected = "f"*64
    elif fault == "invalid_sha": expected = "not_a_SHA256"
    elif fault == "changed_source": files["source"].write_bytes(b"changed synthetic source")
    else:
        value = files["plan_value"]
        if fault == "wrong_owner": value["schema"] = "legacy_owner"
        elif fault == "wrong_raw": value["capture_raw_ABI_schema"] = "made_up_raw_schema"
        else: value["prospective_visual_publication"]["outcome_or_failure_dependent_frame_selection"] = True
        files["plan"].write_bytes(json.dumps(value, sort_keys=True).encode())
        expected = hashlib.sha256(files["plan"].read_bytes()).hexdigest()
    with pytest.raises(RuntimeError): m.seal_policy(expected)
    if fault == "existing_policy": assert files["policy"].read_bytes() == b"untouched existing policy"
    else: assert not files["policy"].exists()


@pytest.mark.parametrize("fault", ["absolute_name", "parent_traversal", "malformed_sha", "boolean_sha", "uppercase_sha"])
def test_sealer_refuses_unsafe_source_names_and_nonexact_SHA_pins(prospective_files, fault):
    files = prospective_files
    plan = files["plan_value"]
    relative, digest = next(iter(plan["source_sha256"].items()))
    if fault == "absolute_name": plan["source_sha256"] = {str(files["source"]): digest}
    elif fault == "parent_traversal": plan["source_sha256"] = {"../synthetic_escape.py": digest}
    elif fault == "malformed_sha": plan["source_sha256"] = {relative: "not_a_SHA256"}
    elif fault == "boolean_sha": plan["source_sha256"] = {relative: True}
    else: plan["source_sha256"] = {relative: digest.upper()}
    files["plan"].write_bytes(json.dumps(plan, sort_keys=True).encode())
    expected = hashlib.sha256(files["plan"].read_bytes()).hexdigest()
    with pytest.raises(RuntimeError): m.seal_policy(expected)
    assert not files["policy"].exists()


@pytest.mark.parametrize("target", ["plan", "policy", "policy_parent", "output", "run", "source"])
def test_sealer_refuses_symlink_targets_and_parent_aliases(prospective_files, target):
    files = prospective_files
    path = files["policy"].parent if target == "policy_parent" else files[target]
    destination = files["root"] / ("unrelated_target_"+target)
    if target in ("policy_parent", "output", "run"):
        destination.mkdir()
    else:
        destination.write_bytes(files["plan"].read_bytes() if target == "plan" else b"unrelated bytes")
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists(): path.unlink()
    path.symlink_to(destination, target_is_directory=destination.is_dir())
    with pytest.raises(RuntimeError): m.seal_policy(files["sha256"])
    assert path.is_symlink()
    assert not files["policy"].exists() or target == "policy"


def test_validate_only_never_creates_gallery_or_decodes_records(tmp_path, monkeypatch):
    value = bundle(35)
    monkeypatch.setattr(m, "OUTPUT", tmp_path/"uncreated_gallery")
    calls = []
    monkeypatch.setattr(m, "load_archive", lambda *args, **kwargs: (calls.append(kwargs) or ({}, value, object())))
    result = m.publish(SimpleNamespace(caller_pins=str(tmp_path/"caller.json"), caller_pins_sha256="a"*64, validate_only=True))
    assert result["validated"] is True and result["rendered"] is result["output_created"] is False
    assert calls == [dict(decode=False)] and not m.OUTPUT.exists()


def test_cross_run_archive_is_denied_before_camera_decode(tmp_path):
    path = tmp_path / "caller.json"
    payload = json.dumps(dict(schema=m.CALLER_SCHEMA, attempts=1, run_root=str(tmp_path), output=str(m.OUTPUT))).encode()
    path.write_bytes(payload)
    with pytest.raises(RuntimeError): m.load_archive(path, hashlib.sha256(payload).hexdigest())
