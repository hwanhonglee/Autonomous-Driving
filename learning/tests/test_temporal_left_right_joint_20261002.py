"""HH_261002 - Synthetic B5/RIGHT697/77-screen tests; no actual data, weights, GPU or remote jobs."""
import ast
import copy
from collections import Counter
import hashlib
import importlib.util
import inspect
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest
import torch

from scripts.e2e import train_temporal_left_right_joint_20261002 as t

# HH_261002 - Load only frozen synthetic fixture generators; no frozen source or runtime mutation.
_spec = importlib.util.spec_from_file_location("hh_synthetic_left_joint_cases", Path(__file__).with_name("test_train_temporal_left_joint_20261002.py"))
helper = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(helper)


@pytest.fixture(scope="module")
def order(): return t.fixed_batches(helper.groups(), helper.strata())


def test_full20k_b5_preserves_original_four_columns_full697_no_dev(order):
    assert len(order) == 20000 and sum(map(len, order)) == 100000
    assert [r[:4] for r in order] == t.baseline.fixed_batches(helper.groups(), helper.strata())
    assert order == t.fixed_batches(helper.groups(), helper.strata())
    assert all(len(set(r)) == 5 and 6179 <= r[4] < 6876 for r in order)
    counts = Counter(r[4] - 6179 for r in order)
    assert set(counts) == set(range(697)) and set(counts.values()) == {28, 29}
    assert sum(v == 29 for v in counts.values()) == 484
    for offset in range(0, 697 * 28, 697): assert {r[4] - 6179 for r in order[offset:offset + 697]} == set(range(697))
    left = Counter(r[3] - 5832 for r in order)
    assert set(left.values()) == {57, 58} and sum(v == 58 for v in left.values()) == 221
    queue = t.replay.CycleQueue(range(697), t.SEED + 900)
    assert [r[4] - 6179 for r in order] == [queue.next() for _ in range(20000)]


def test_original_three_protection_queues_exact_signed_focus_and_legacy_maps(order):
    g, s = helper.groups(), helper.strata(); by_index = {i: k for k, v in g["focus"].items() for i in v}
    focus = [r[0] for step, r in enumerate(order) if step % 2 == 0]
    assert Counter(by_index[i] for i in focus) == {k: 1250 for k in g["focus"]}
    assert {r[0] for step, r in enumerate(order) if step % 2} == set(range(2718))
    assert {r[1] - 2718 for r in order} == set(range(327))
    assert {r[2] - 3045 for r in order} == set(range(2487))
    curved = {i for v in s["curved"].values() for i in v}
    assert sum(r[2] - 3045 in curved for r in order) == 10000
    assert [sum(r[2] - 3045 in v for r in order) for v in s["curved"].values()] == [5000, 5000]
    assert sorted(sum(r[2] - 3045 in v for r in order) for v in s["straight"].values()) == [3333, 3333, 3334]


@pytest.mark.parametrize("bad", ["emptyfocus", "sevenbins", "eightprofiles", "lost302", "emptycurve", "emptystraight"])
def test_no_sampler_eligibility_weakening(bad):
    helper.t = t
    try: helper.test_no_empty_or_fabricated_recovery_legacy_strata(bad)
    finally: helper.t = t.baseline


def test_same101_54_47_abi_and_good6000_parent_only():
    # HH_261002 - Random CPU architecture construction is not actual model inference or a learned-weight read.
    model = helper.RouteRecoveryTemporalModelV3(); names = t.original.freeze_inputs(model)
    assert len(model.state_dict()) == 101 and len(names) == 54 and len(set(model.state_dict()) - set(names)) == 47
    assert sum(p.numel() for p in model.parameters() if p.requires_grad) == 2277827
    assert t.pair.PARENT == t.baseline.pair.PARENT
    assert t.pair.PARENT["weights_sha256"] == "b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625"
    assert t.PARENT_HISTORY_SHA == "3c1fe1e4f9e18017a2db31b5371001873dece4a2b8e78bb3cdb21f9aad5ebac3"
    assert t.original.MAX_UPDATES == 6000 and t.baseline.MAX_UPDATES == t.MAX_UPDATES == 20000
    assert t.main.__globals__ is vars(t) and t.main.__globals__["run"] is t.run
    assert t.baseline.main.__globals__["run"] is t.baseline.run


@pytest.mark.parametrize("group,peak,final", [("shared", 1e-5, 1e-6), ("heads", 3e-5, 3e-6)])
def test_every_lr_exact_frozen_left20k(group, peak, final):
    assert t.learning_rate_config() == t.baseline.learning_rate_config()
    assert [t.learning_rate(i, group) for i in range(1, 20001)] == [t.baseline.learning_rate(i, group) for i in range(1, 20001)]
    assert t.learning_rate(300, group) == peak and t.learning_rate(20000, group) == final


def test_mean_turn_only_preserves_protected_loss_coefficients():
    leaves = [torch.tensor(float(i + 1), requires_grad=True) for i in range(6)]
    batch = dict(marker=torch.tensor([0, 1, 2, 3, 4]), sample_id=["rec", "low", "legacy", "LEFT", "RIGHT"]); calls = []
    def joint(model, decoder, b):
        i = int(b["marker"][0]); calls.append(i); return None, None, dict(total=leaves[{0: 0, 3: 1, 1: 2, 4: 3}[i]])
    def raw(model, b): assert int(b["marker"][0]) == 2; return {"marker": 2}
    def old(output, b): assert output["marker"] == 2; return dict(raw_curvature=leaves[4], raw_preferred_speed=leaves[5])
    with mock.patch.object(t.joint, "joint_forward", side_effect=joint), mock.patch.object(t.raw_parent, "forward", side_effect=raw), \
            mock.patch.object(t.original, "temporal_trajectory_loss_v2", side_effect=old): loss, terms, legacy, values = t.domains(None, None, batch)
    assert calls == [0, 3, 1, 4]
    assert torch.equal(loss, t.original.exact_objective(leaves[0], .5 * (leaves[1] + leaves[3]), leaves[2], leaves[4], leaves[5]))
    loss.backward(); assert [float(v.grad) for v in leaves] == [.25, .125, .25, .125, .0625, .25]
    assert values[0] is terms[0]["total"] and values[1] is terms[2]["total"] and values[3] is terms[1]["total"] and values[4] is terms[3]["total"]


def right_receipt():
    r = helper.receipt(); r.pop("rolling_LEFT_only")
    r.update(schema=t.RIGHT_CONVERSION_SCHEMA, status="QUALIFIED_RESEARCH_TRAIN_RIGHT697_COMMON10", train_examples=697,
        true_future_points=44608, full_five_camera_history_examples=689, full_ten_ego_history_examples=688,
        dataset_manifest_sha256=t.RIGHT_MANIFEST_SHA, validation_report_sha256=t.RIGHT_VALIDATION_SHA,
        qualification_receipt_sha256=t.RIGHT_QUALIFICATION_SHA, train_loader_fingerprint_sha256=t.RIGHT_LOADER_SHA,
        temporal_dataset_fingerprint_sha256=t.RIGHT_TEMPORAL_SHA, **t.RIGHT_SCOPE)
    r["per_episode_native_qualification"][0].update(selected_examples=697, episode_id="RIGHTnative", manifest="episodes/RIGHTnative/episode.json",
        qualification_receipt_sha256=t.RIGHT_QUALIFICATION_SHA, native_episode="/actual/RIGHT80/pilot_00/episode")
    return r


def test_closed_actual_right80_697_complete44608_and689688_history():
    r = right_receipt(); assert t.verify_RIGHT_conversion(r) is r["per_episode_native_qualification"][0]
    assert t.right_scope_clear(r) and t.RIGHT_BASENAME.endswith("right80-train-20261002-v1")


@pytest.mark.parametrize("key,value", [("schema", "carla.failed_right70.v1"), ("train_examples", 696), ("train_examples", 697.),
    ("val_examples", 1), ("test_examples", 1), ("true_future_points", 44544), ("full_five_camera_history_examples", 697),
    ("full_ten_ego_history_examples", 689), ("formal_MAIN9_approved", 1), ("research_training_authorized", False),
    ("raw_native_denials_overridden", True), ("original_pending_metadata_unchanged", False), ("labels_clipped", True),
    ("labels_or_pixels_transformed", True), ("synthetic_history_created", True), ("future_labels_used_as_model_input", True),
    ("STOP_or_stationary_tail_supervision", True), ("rolling_RIGHT_only", 1), ("failed_RIGHT70_attempts_not_requalified", False),
    ("whole_nominal_clearance_is_not_actual_training_approval", False), ("full_generated_navigation_route_retained", False),
    ("capture_container_is_declared_not_measured_geometry", False), ("fixed_driving_relative_anchor_offsets", [80, 1400]),
    ("fixed_anchor_driving_seconds", [4., 70.]), ("new_independent_site_or_sealed_TEST_claimed", True), ("model_promoted", True),
    ("vehicle_control_approved", True), ("dataset_manifest_sha256", "a" * 64), ("validation_report_sha256", "a" * 64),
    ("qualification_receipt_sha256", "a" * 64), ("train_loader_fingerprint_sha256", "a" * 64),
    ("temporal_dataset_fingerprint_sha256", "a" * 64), ("external_ROOT_authority_sha256", None)])
def test_right_membership_denials_labels_actual_fingerprint_pins_strict(key, value):
    r = right_receipt(); r[key] = value
    with pytest.raises(ValueError): t.verify_RIGHT_conversion(r)


@pytest.mark.parametrize("key,value", [("split", "test"), ("selected_examples", 696), ("selected_examples", 697.),
    ("case_id", "failed70"), ("qualification_receipt_sha256", "a" * 64), ("source_state_sha256", "bad")])
def test_one_native_right_training_identity_strict(key, value):
    r = right_receipt(); r["per_episode_native_qualification"][0][key] = value
    with pytest.raises(ValueError): t.verify_RIGHT_conversion(r)


@pytest.fixture
def RIGHTfixture(tmp_path, monkeypatch):
    # HH_261002 - Synthetic receipt files only; monkeypatch hard pins solely for this temporary fixture.
    root = tmp_path / "dataset/prepared" / t.RIGHT_BASENAME; episode = root / "episodes/RIGHTnative"; episode.mkdir(parents=True)
    monkeypatch.setattr(t.original, "PERSONAL", tmp_path); r = right_receipt(); case = r["per_episode_native_qualification"][0]
    def write(path, value): path.write_text(json.dumps(value)); return t.digest(path)
    r["dataset_manifest_sha256"] = write(root / "dataset.json", dict(episodes=[dict(manifest=case["manifest"])]))
    r["validation_report_sha256"] = write(root / "validation_report.json", dict(status="synthetic"))
    r["signed_current_state_audit_sha256"] = write(root / "signed_current_state_audit.jsonl", dict(diagnostic=True))
    write(episode / "episode.json", dict(split="train", episode_id="RIGHTnative", source_provenance=dict(adapter_id=t.RIGHT_ADAPTER)))
    case["native_manifest_sha256"] = write(episode / "native_manifest.json", dict(native="PENDING_QA_NOT_AUTHORIZED"))
    case["source_route_sha256"] = write(episode / "native_route.json", dict(route="actual whole navigation"))
    q = dict(schema=t.RIGHT_QUALIFICATION_SCHEMA, status="QUALIFIED_RESEARCH_TRAIN_RIGHT697", research_training_authorized=True,
        raw_native_denials_overridden=False, external_ROOT_authority_sha256=r["external_ROOT_authority_sha256"], cases=[copy.deepcopy(case)], **t.RIGHT_SCOPE)
    r["qualification_receipt_sha256"] = write(episode / "moving_qualification.json", q); case["qualification_receipt_sha256"] = r["qualification_receipt_sha256"]
    pin = write(root / "moving_conversion_qualification.json", r)
    for attr, key in (("RIGHT_CONVERSION_SHA", None), ("RIGHT_MANIFEST_SHA", "dataset_manifest_sha256"),
            ("RIGHT_VALIDATION_SHA", "validation_report_sha256"), ("RIGHT_QUALIFICATION_SHA", "qualification_receipt_sha256")):
        monkeypatch.setattr(t, attr, pin if key is None else r[key])
    dataset = helper.fake_dataset(697, "RIGHTnative")
    loader = dict(validated_fingerprint_sha256=r["train_loader_fingerprint_sha256"], temporal_fingerprint_sha256=r["temporal_dataset_fingerprint_sha256"])
    monkeypatch.setattr(t.raw_parent, "load_view", lambda *args: (dataset, loader))
    monkeypatch.setattr(t.joint, "cohort_statistics", lambda d: dict(causal_navigation_support_m=dict(minimum=54.97)))
    return SimpleNamespace(args=SimpleNamespace(right_train=root, right_conversion_sha256=pin), root=root, episode=episode, dataset=dataset, r=r)


def test_right_loader_full697_train_no_cuda_native_denial_retained(RIGHTfixture):
    f = RIGHTfixture; dataset, receipt, pins = t.load_RIGHT(f.args)
    assert dataset is f.dataset and len(dataset) == 697 and len(pins) == 7 and receipt["conversion_sha256"] == f.args.right_conversion_sha256
    assert t.read(f.episode / "native_manifest.json") == dict(native="PENDING_QA_NOT_AUTHORIZED")
    assert not torch.cuda.is_initialized()


@pytest.mark.parametrize("mutation", ["current_speed_high", "current_zero", "current_nan", "zero_future", "nan_future", "missing_point",
    "lost_anchor", "copiednative", "episodeVAL", "foreignadapter", "wrongbasename", "foreign_conversion"])
def test_right_load_never_clips_fills_or_admits_failed70(RIGHTfixture, mutation):
    f = RIGHTfixture
    if mutation == "current_speed_high": f.dataset.examples[0].features[1] = 2.6
    elif mutation == "current_zero": f.dataset.examples[0].features[1] = 0.
    elif mutation == "current_nan": f.dataset.examples[0].features[1] = float("nan")
    elif mutation == "zero_future": f.dataset.examples[0].target_speed_mps[0] = 0.
    elif mutation == "nan_future": f.dataset.examples[0].target_speed_mps[0] = float("nan")
    elif mutation == "missing_point": f.dataset.examples[0].valid_point_count = 63
    elif mutation == "lost_anchor": f.dataset.examples.pop()
    elif mutation == "copiednative": (f.episode / "native_manifest.json").write_text("tampered")
    elif mutation in ("episodeVAL", "foreignadapter"):
        v = t.read(f.episode / "episode.json")
        if mutation == "episodeVAL": v["split"] = "val"
        else: v["source_provenance"]["adapter_id"] = "carla_actual_failed_right70_to_common10"
        (f.episode / "episode.json").write_text(json.dumps(v))
    elif mutation == "foreign_conversion": f.args.right_conversion_sha256 = "f" * 64
    else:
        replacement = f.root.with_name(f.root.name.replace("right80", "right70")); f.root.rename(replacement); f.args.right_train = replacement
    with pytest.raises(ValueError): t.load_RIGHT(f.args)


def test_new_contract31_all_frozen_sources_and_coupled_research_scope():
    c = t.contract("a" * 64); old = t.baseline.source_snapshot()
    assert len(c["sources"]) == 31 and len(old) == 30 and all(c["sources"][k] == v for k, v in old.items())
    assert c["sources"]["tools/e2e/train_temporal_left_joint_20261002.py"] == t.BASELINE_SHA
    assert c["effective_batch_size"] == 5 and c["examples_seen"] == 100000
    assert c["original_TRAIN_anchor_count"] == 6876 and c["eligible_TRAIN_anchor_count"] == 6576 and c["training_episode_count"] == 20
    assert c["new_LEFT_count"] == 347 and c["new_RIGHT_count"] == 697 and c["known_final_total_optimizer_updates"] == 32200
    assert sum(c["exposures"][k] for k in ("recovery", "old_low", "legacy", "LEFT", "RIGHT")) == 100000
    assert c["retained_original_diagnostic_screens"] == 74 and c["total_diagnostic_screen_count"] == 77 and c["original_failed_screen_count_retained"] == 14
    assert c["RIGHT_TRAIN_screens"] == dict(ADE_m=.1, FDE_m=.25, speed_MAE_mps=.05)
    assert all(c[k] is False for k in ("new_RIGHT_independent_validation", "new_head", "intrinsic_loss_added", "legacy_decoded", "runtime_architecture_changed",
        "runtime_caps_changed", "model_promoted", "vehicle_control_approved", "inference_loader_authorized"))
    assert c["official_main_features_approved"] == c["official_subfeatures_approved"] == 0
    assert t.digest(t.baseline.__file__) == t.BASELINE_SHA


def test_contract_never_reads_actual_train_or_models(tmp_path, monkeypatch, capsys):
    for name in ("load_train", "verify_parent"): monkeypatch.setattr(t.pair, name, lambda *a: pytest.fail("actual payload forbidden"))
    for name in ("load_LEFT", "load_RIGHT"): monkeypatch.setattr(t, name, lambda *a: pytest.fail("actual payload forbidden"))
    monkeypatch.setattr(t.gpu_revision, "gpu_preflight", lambda: pytest.fail("GPU forbidden"))
    assert t.main(["--mode", "contract", "--left-conversion-sha256", "a" * 64]) == 0
    assert json.loads(capsys.readouterr().out)["examples_seen"] == 100000 and not list(tmp_path.iterdir()) and not torch.cuda.is_initialized()


def authority(proposed):
    return dict(schema=t.AUTH_SCHEMA, authorized=True, contract=proposed, contract_sha256=t.json_sha(proposed),
        actual_parent_history_sha256=t.PARENT_HISTORY_SHA, GPU_physical_index=0, model_promoted=False, vehicle_control_approved=False)


def test_authority_hash_and_parse_same_first_buffer_not_replacement(tmp_path, monkeypatch):
    monkeypatch.setattr(t.original, "PERSONAL", tmp_path); proposed = dict(batch_size=5, sources=dict(new="a" * 64)); v = authority(proposed)
    path = tmp_path / "ROOT.json"; path.write_text(json.dumps(v)); payload = path.read_bytes(); calls = []
    def raced(this): calls.append(this); path.write_text('{"authorized":false}'); return payload
    with mock.patch.object(Path, "read_bytes", raced): assert t.validate_authorization(path, hashlib.sha256(payload).hexdigest(), proposed) == v
    assert len(calls) == 1


@pytest.mark.parametrize("key,value", [("authorized", False), ("GPU_physical_index", False), ("GPU_physical_index", 1),
    ("actual_parent_history_sha256", "c" * 64), ("model_promoted", True), ("vehicle_control_approved", True), ("contract_sha256", "b" * 64), ("schema", t.baseline.AUTH_SCHEMA)])
def test_external_root_authority_strict(tmp_path, monkeypatch, key, value):
    monkeypatch.setattr(t.original, "PERSONAL", tmp_path); proposed = dict(batch_size=5, source="a" * 64)
    v = dict(authority(proposed), **{key: value}); path = tmp_path / "ROOT.json"; path.write_text(json.dumps(v))
    with pytest.raises(ValueError): t.validate_authorization(path, t.digest(path), proposed)


def test_authority_typed_numbers_duplicate_wrong_sha_symlink_denied(tmp_path, monkeypatch):
    monkeypatch.setattr(t.original, "PERSONAL", tmp_path); proposed = dict(updates=20000, GPU=0); path = tmp_path / "ROOT.json"
    for c in (dict(updates=20000., GPU=0), dict(updates=20000, GPU=False)):
        path.write_text(json.dumps(dict(authority(proposed), contract=c)))
        with pytest.raises(ValueError): t.validate_authorization(path, t.digest(path), proposed)
    path.write_text('{"authorized":true,"authorized":false}')
    with pytest.raises(ValueError): t.pinned_read(path, t.digest(path))
    path.write_text('{}')
    with pytest.raises(ValueError): t.pinned_read(path, "a" * 64)
    alias = tmp_path / "alias.json"; alias.symlink_to(path)
    with pytest.raises(ValueError): t.pinned_read(alias, t.digest(path))


def screens_fixture(monkeypatch):
    checks = [dict(name=f"old/{i:02d}", actual=.2 if i < 14 else .01, threshold=.1, passed=i >= 14) for i in range(71)]
    monkeypatch.setattr(t.original, "endpoint_screens", lambda *a: dict(checks=copy.deepcopy(checks), fixed_endpoint_updates=6000))
    def metrics(n, label): return dict(full_fixed_denominator=n, rows=[dict(token=f"{label}/{i}") for i in range(n)],
        mean_ade_6p4s_m=.08, mean_fde_6p4s_m=.2, mean_instantaneous_speed_mae_mps=.03)
    before = dict(LEFT=metrics(347, "LEFT"), RIGHT=metrics(697, "RIGHT")); after = copy.deepcopy(before)
    return before, after, dict(endpoint_diagnostic_screens=t.baseline.endpoint_screens(before, after))


def test_full77_old74_numeric_thresholds_and_all14_previous_failures(monkeypatch):
    before, after, ref = screens_fixture(monkeypatch); report = t.endpoint_screens(before, after, ref)
    assert len(report["checks"]) == 77 and report["diagnostic_screen_count"] == 77
    assert report["checks"][:74] == report["retained_previous74_results"] == ref["endpoint_diagnostic_screens"]["checks"]
    assert len(report["original_failed14_results"]) == len(report["original_failed14_current_results"]) == 14
    assert sum(r["passed"] for r in report["checks"]) == 63 and not report["all_diagnostic_screens_passed"]
    assert [r["threshold"] for r in report["checks"][-3:]] == [.1, .25, .05]
    assert report["fixed_endpoint_updates"] == 20000 and not report["new_RIGHT_independent_validation"]


@pytest.mark.parametrize("mutation", ["right_count", "right_lost", "right_duplicate", "right_reorder", "left_lost", "left_duplicate",
    "old_threshold", "lost_failure", "old_missing", "old_duplicate"])
def test_no_rebasing_dropping14_or_cropping_turn_denominators(monkeypatch, mutation):
    before, after, ref = screens_fixture(monkeypatch)
    if mutation == "right_count": after["RIGHT"]["full_fixed_denominator"] = 696
    elif mutation in ("right_lost", "left_lost"):
        key = "RIGHT" if mutation == "right_lost" else "LEFT"; before[key]["rows"].pop(); after[key]["rows"].pop()
    elif mutation in ("right_duplicate", "left_duplicate"):
        key = "RIGHT" if mutation == "right_duplicate" else "LEFT"
        for v in (before, after): v[key]["rows"][-1]["token"] = v[key]["rows"][0]["token"]
    elif mutation == "right_reorder": after["RIGHT"]["rows"].reverse()
    else:
        rows = ref["endpoint_diagnostic_screens"]["checks"]
        if mutation == "old_threshold": rows[0]["threshold"] = .3
        elif mutation == "lost_failure": rows[0]["passed"] = True
        elif mutation == "old_missing": rows.pop()
        else: rows[-1]["name"] = rows[0]["name"]
    with pytest.raises(ValueError): t.endpoint_screens(before, after, ref)


@pytest.mark.parametrize("key,value", [("mean_ade_6p4s_m", .10001), ("mean_fde_6p4s_m", .25001), ("mean_instantaneous_speed_mae_mps", .05001),
    ("mean_ade_6p4s_m", float("nan")), ("mean_ade_6p4s_m", True), ("mean_ade_6p4s_m", False)])
def test_right_failure_full77_not_checkpoint_selection(monkeypatch, key, value):
    before, after, ref = screens_fixture(monkeypatch); after["RIGHT"][key] = value; result = t.endpoint_screens(before, after, ref)
    assert len(result["checks"]) == 77 and len(result["original_failed14_results"]) == 14
    assert not next(r for r in result["checks"] if r["name"] == "RIGHT_TRAIN_resubstitution/" + key)["passed"]


def test_deferred_payload_boundary_source_no_actual_job():
    source = inspect.getsource(t.run)
    assert source.index("validate_authorization(") < source.index("pair.load_train(")
    assert source.index("verify_final(args.output, result, parent") < source.index("pair.load_evaluation_only(")
    assert source.index("pinned_read(ORIGINAL_SCREENS_PATH") > source.index("verify_final(args.output, result, parent")
    assert source.count("pair.load_evaluation_only(") == 2 and "load_RIGHT(args)[1] == right_receipt" in source
    assert "parent_order = original.fixed_batches(groups, strata)" in source and "pair.verify_parent(args, parent_order, (training, low, legacy))" in source
    assert "with raw_parent.gpu0_lease()" in source and "gpu_revision.gpu_preflight()" in source
    assert "pair.verify_training_images((*datasets, holdout, known))" in source
    assert not any(word in source for word in ("Popen", "ssh", "sudo", "pip install"))


def test_inverse_ast_frozen_strict_guard_preservation_and_new_b5_accounting():
    for name in ("left_scope_clear", "verify_LEFT_conversion", "load_LEFT", "save_checkpoint", "train", "verify_final"):
        assert isinstance(t.revised_function_ast(name), ast.FunctionDef) and t.digest(t.baseline.__file__) == t.BASELINE_SHA
    save, train, final = [ast.unparse(t.revised_function_ast(n)) for n in ("save_checkpoint", "train", "verify_final")]
    assert "RIGHT_examples_seen=step" in save and "examples_seen=5 * step" in save
    assert "RIGHT=joint.loss_numbers(terms[3])" in train and "examples_seen=5 * completed" in train
    assert "AdamW(groups, weight_decay=0.0001)" in train
    assert "allow_pickle=False" in final and "'RIGHT_examples_seen'" in final and "100000" in final and "5 * (i + 1)" in final
    source = inspect.getsource(t.baseline.preflight_gradient)
    assert "optimizer.step(" not in source and "AdamW(" not in source
    assert all(word in source for word in ("torch.set_rng_state", "torch.cuda.set_rng_state", "random.setstate", "np.random.set_state"))


def test_failure_receipt_fresh_namespace_preserves_original_runner(tmp_path, monkeypatch):
    monkeypatch.setattr(t, "_created_output", None); before = t.baseline.run
    def fail(argv): t._created_output = tmp_path; raise RuntimeError("synthetic all5 guard")
    monkeypatch.setattr(t, "run", fail)
    with pytest.raises(RuntimeError): t.main([])
    v = t.read(tmp_path / "failure_receipt.json")
    assert v["schema"] == t.SCHEMA and v["partial_outputs_retained"] and v["GPU_lease_context_exited"]
    assert not v["model_promoted"] and not v["foreign_processes_killed"] and t._created_output is None and t.baseline.run is before


@pytest.fixture
def numeric_endpoint(tmp_path, monkeypatch):
    # HH_261002 - Tiny scalar101 numeric ABI, synthetic20k journal only; no actual model arrays or pickle load.
    helper.t = t
    try: f = helper.numeric_endpoint.__wrapped__(tmp_path, monkeypatch)
    finally: helper.t = t.baseline
    d = t.read(f.output / "declaration.json"); d["fixed_sample_ids"] = [r + ["RIGHT/0"] for r in d["fixed_sample_ids"]]
    indices = [0, 2718, 3045, 5832, 6179]; d["fixed_schedule_sha256"] = t.json_sha([indices] * 20000)
    (f.output / "declaration.json").write_text(json.dumps(d))
    rows = [json.loads(line) for line in (f.output / "history.jsonl").read_text().splitlines()]
    for r in rows: r.update(examples_seen=5 * r["optimizer_updates"], sample_ids=d["fixed_sample_ids"][r["optimizer_updates"] - 1], indices=indices)
    (f.output / "history.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    f.meta.update(examples_seen=100000, RIGHT_examples_seen=20000, history_sha256=t.digest(f.output / "history.jsonl"),
        declaration_sha256=t.digest(f.output / "declaration.json"), fixed_sample_ids_sha256=t.json_sha(d["fixed_sample_ids"]))
    f.result["examples_seen"] = 100000; repin_endpoint(f); return f


def repin_endpoint(f):
    f.description.write_text(json.dumps(f.meta)); f.result["checkpoints"][-1]["metadata_sha256"] = t.digest(f.description)


def test_actual20k100k_all101_frozen47_bytes_no_pickle(numeric_endpoint, monkeypatch):
    monkeypatch.setattr(torch, "load", lambda *a, **k: pytest.fail("pickle forbidden")); f = numeric_endpoint
    v = t.verify_final(f.output, f.result, f.parent, f.sources)
    assert v["optimizer_updates"] == 20000 and v["frozen47_byte_identical"] and v["all20000_actual_journal_rows_verified"]


@pytest.mark.parametrize("key,value", [("examples_seen", 80000), ("RIGHT_examples_seen", 0), ("LEFT_examples_seen", 0),
    ("known_total_optimizer_updates", 32201), ("runtime_loader_implemented", True), ("parent_npz_sha256", "a" * 64), ("source_sha256", {}), ("model_promoted", True)])
def test_final_truthful_new_census_same_good_parent(numeric_endpoint, key, value):
    f = numeric_endpoint; f.meta[key] = value; repin_endpoint(f)
    with pytest.raises(ValueError): t.verify_final(f.output, f.result, f.parent, f.sources)


@pytest.mark.parametrize("mutation", ["NaN", "float64", "shape", "missing", "frozenchanged"])
def test_complete101_numeric_finite_shape_dtype_frozen47(numeric_endpoint, mutation):
    f = numeric_endpoint; name = next(k for k in f.arrays if not k.startswith(t.original.TRAINABLE_PREFIXES))
    if mutation == "NaN": f.arrays[name][0] = np.nan
    elif mutation == "float64": f.arrays[name] = f.arrays[name].astype(np.float64)
    elif mutation == "shape": f.arrays[name] = np.repeat(f.arrays[name], 2)
    elif mutation == "missing": f.arrays.pop(name)
    else: f.arrays[name][0] += .25
    with f.npz.open("wb") as stream: np.savez(stream, **f.arrays)
    f.meta["npz_sha256"] = f.result["checkpoints"][-1]["npz_sha256"] = t.digest(f.npz); repin_endpoint(f)
    with pytest.raises(ValueError): t.verify_final(f.output, f.result, f.parent, f.sources)


@pytest.mark.parametrize("mutation", ["missing_row", "B4_accounting", "lost_RIGHT", "LEFT_as_RIGHT", "RIGHT_index", "LR", "batch_sha"])
def test_all20k_b5_actual_journal_and_right_order(numeric_endpoint, mutation):
    f = numeric_endpoint; path = f.output / "history.jsonl"; rows = [json.loads(line) for line in path.read_text().splitlines()]
    if mutation == "missing_row": rows.pop()
    elif mutation == "B4_accounting": rows[0]["examples_seen"] = 4
    elif mutation == "lost_RIGHT": rows[0]["sample_ids"].pop()
    elif mutation == "LEFT_as_RIGHT": rows[0]["sample_ids"][4] = "LEFT/0"
    elif mutation == "RIGHT_index": rows[0]["indices"][4] = 5832
    elif mutation == "LR": rows[0]["learning_rates"]["heads"] = .1
    else: rows[0]["batch_sha256"] = "unknown"
    path.write_text("\n".join(json.dumps(r) for r in rows) + "\n"); f.meta["history_sha256"] = t.digest(path); repin_endpoint(f)
    with pytest.raises(ValueError, match="all actual20k"): t.verify_final(f.output, f.result, f.parent, f.sources)


@pytest.mark.parametrize("mutation", ["incomplete", "lost_checkpoint", "corrupted_pt", "old80000"])
def test_fit_boolean_cannot_open_evaluation_without_actual_endpoint(numeric_endpoint, mutation):
    f = numeric_endpoint
    if mutation == "incomplete": f.result["optimizer_updates"] = 19999
    elif mutation == "lost_checkpoint": f.result["checkpoints"].pop(0)
    elif mutation == "corrupted_pt": (f.output / "checkpoints/step_020000.pt").write_bytes(b"changed")
    else: f.result["examples_seen"] = 80000
    with pytest.raises(ValueError): t.verify_final(f.output, f.result, f.parent, f.sources)


def test_bad_root_authority_stops_before_any_dataset_read(tmp_path, monkeypatch):
    w = tmp_path / "portable_e2e"; (w / "venvs/py312").mkdir(parents=True)
    monkeypatch.setattr(t.original, "WORKSPACE", w); monkeypatch.setattr(t.original, "PERSONAL", tmp_path)
    monkeypatch.setattr(t.sys, "prefix", str(w / "venvs/py312")); monkeypatch.setattr(t.pair, "load_train", lambda *a: pytest.fail("TRAIN before authority"))
    p = t.contract("a" * 64); path = tmp_path / "ROOT.json"; path.write_text(json.dumps(dict(authority(p), authorized=False)))
    argv = ["--mode", "fit", "--left-conversion-sha256", "a" * 64, "--authorization", str(path), "--authorization-sha256", t.digest(path), "--output", str(w / "runs/training" / t.STUDY_ID)]
    for name in ("parent-run", "warm-parent-weights", "warm-parent-metadata", "backbone-parent-weights", "backbone-parent-metadata",
            "recovery-train", "recovery-conversion", "recovery-holdout", "holdout-conversion", "left-train", "right-train"):
        argv.extend(["--" + name, str(tmp_path / name)])
    with pytest.raises(ValueError, match="authority"): t.main(argv)
    assert not (w / "runs").exists() and not torch.cuda.is_initialized()


def test_source31_canonical_copy_query_no_legacy_alias_train(tmp_path):
    for name in t.source_snapshot():
        path = tmp_path / name; path.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(t.ROOT / name, path)
    result = subprocess.run([sys.executable, "-B", str(tmp_path / "tools/workspace_python.py"), "-m", "scripts.e2e.train_temporal_left_right_joint_20261002",
        "--", "--mode", "contract", "--left-conversion-sha256", "a" * 64], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    c = json.loads(result.stdout); assert len(c["sources"]) == 31 and c["examples_seen"] == 100000 and c["fixed_new_updates"] == 20000
    assert not (tmp_path / "scripts").exists() and not (tmp_path / "portable_e2e").exists()
