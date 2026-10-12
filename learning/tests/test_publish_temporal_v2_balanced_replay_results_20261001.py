"""HH_261001 - Verify honest fixed-endpoint publication without Torch, plots, or simulator execution."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

ENTRY = Path(__file__).resolve().parents[2] / "tools/e2e/publish_temporal_v2_balanced_replay_results_20261001.py"
SPEC = importlib.util.spec_from_file_location("hh_balanced_result_publisher", ENTRY)
publisher = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(publisher)


def raw_row(index, episode, map_id, stratum, curvature=.01, speed=.02):
    return dict(index=index, token=episode + ":" + str(index), episode_id=episode, map=map_id,
        stratum=stratum, raw_curvature=curvature, raw_preferred_speed=speed, total=.25 * curvature + speed,
        intent=0., stop_distance=0., initial_acceleration=0., temporal_xy=0., temporal_speed=0.)


def raw_report(rows, split):
    def metrics(chosen):
        return dict(count=len(chosen), **{c: sum(r[c] for r in chosen) / len(chosen) for c in publisher.RAW_COMPONENTS})
    return dict(split=split, independent_validation=False, legacy_decoded=False,
        full_fixed_eligible_denominator=len(rows), rows=rows,
        groups={key: metrics(rows if key == "all_eligible" else [r for r in rows if r["stratum"] == key])
                for key in ("all_eligible", "curved", "straight")},
        per_episode={ep: metrics([r for r in rows if r["episode_id"] == ep]) for ep in {r["episode_id"] for r in rows}})


def v4_report():
    rows = [dict(index=i, token="v4:" + str(i), ade_6p4s_m=.104, fde_6p4s_m=.257,
                 instantaneous_speed_mae_mps=.024) for i in range(327)]
    return dict(split="train", independent_validation=False, full_fixed_denominator=327, rows=rows,
                mean_ade_6p4s_m=.104, mean_fde_6p4s_m=.257, mean_instantaneous_speed_mae_mps=.024,
                maximum_ade_6p4s_m=.104)


@pytest.fixture
def evidence():
    flags = dict(training_only=True, simulation_only=True, one_episode_screening=False, research_screening=True,
        mixed_raw_joint_training=True, main_training=False, model_promoted=False, vehicle_control_approved=False,
        independent_validation=False, stop_intent_trained=False, labels_clipped=False, labels_extrapolated=False,
        legacy_decoded=False, decoder_integrated=True, raw_spatial_only=False, training_episode_count=9,
        runtime_intent="explicit_cruise_GO_not_head")
    sources = {key: "d" * 64 for key in publisher.SOURCE_NAMES}
    sources["tools/e2e/train_temporal_v2_balanced_replay_20261001.py"] = publisher.TRAINER_SHA
    declaration_sha = "a" * 64
    cohorts = dict(legacy=dict(sample_count=2787, episode_count=8, split="train"),
        v4=dict(split="train", statistics=dict(anchor_count=327)), known_dev=dict(sample_count=665, split="val"))
    legacy = []
    for ep, (count, map_id) in enumerate(zip([266, 328, 328, 310, 328, 271, 328, 328],
            ["C_track_1_0_7"] * 3 + ["Town01"] * 2 + ["Town07"] * 3)):
        for position in range(count):
            stratum = "curved" if (ep == 0 and position < 260) or (ep == 3 and position < 68) else "straight"
            legacy.append(raw_row(len(legacy), "legacy_ep" + str(ep), map_id, stratum))
    declared = {kind: {map_id: [r["index"] for r in legacy if r["stratum"] == kind and r["map"] == map_id]
                      for map_id in {r["map"] for r in legacy if r["stratum"] == kind}}
                for kind in ("curved", "straight")}
    excluded = set(range(38)) | set(range(40, 66)) | {100, 101}
    dev = [raw_row(i, "known_right" if i < 337 else "known_straight", "Town03", "curved" if i < 150 else "straight",
                   speed=(.2 if i in publisher.PROBES else .5) if i < 337 else .02)
           for i in range(665) if i not in excluded]
    old599, new599 = raw_report(dev, "val"), raw_report(copy.deepcopy(dev), "val")
    for report in (old599, new599): report.update(available_known_val_count=665, excluded_count=66)
    for row in new599["rows"]:
        if row["episode_id"] == "known_straight":
            row["raw_preferred_speed"] = .06; row["total"] = .25 * row["raw_curvature"] + .06
    new599.update(raw_report(new599["rows"], "val"))
    def probes(report):
        by_index = {r["index"]: r for r in report["rows"]}
        rows = [dict(index=i, token=by_index[i]["token"], **{c: by_index[i][c] for c in publisher.RAW_COMPONENTS})
                for i in publisher.PROBES]
        return dict(independent_validation=False, temporal_trajectory_evaluated=False, rows=rows,
                    mean_total=sum(r["total"] for r in rows) / len(rows))
    before8, after8 = probes(old599), probes(new599)
    low = v4_report()
    history = []
    curved = [r["index"] for r in legacy if r["stratum"] == "curved"]
    straight = [r["index"] for r in legacy if r["stratum"] == "straight"]
    for i in range(3000):
        indices = [(2 * i) % 327, (2 * i + 1) % 327, 327 + curved[i % 328], 327 + straight[i % 2159]]
        history.append(dict(step=i + 1, optimizer_updates=i + 1, examples_seen=(i + 1) * 4,
            v4_examples_seen=(i + 1) * 2, legacy_examples_seen=(i + 1) * 2, indices=indices,
            sample_ids=[low["rows"][indices[0]]["token"], low["rows"][indices[1]]["token"],
                        legacy[indices[2] - 327]["token"], legacy[indices[3] - 327]["token"]],
            total_loss=.5, v4_joint=dict(total=.6), legacy_raw=dict(total=.4),
            domain_parameter_gradients={k: dict(all_present_parameter_gradients_finite=True) for k in ("v4", "legacy")}))
    schedule_sha = hashlib.sha256(json.dumps([r["indices"] for r in history], separators=(",", ":")).encode()).hexdigest()
    declaration = dict(flags, schema=publisher.RUN_SCHEMA, source_sha256=sources, cohorts=cohorts,
        parent_checkpoint_sha256=publisher.PARENT_SHA, requested_optimizer_updates=3000, maximum_optimizer_updates=3000,
        start_optimizer_updates=0, resume_checkpoint_sha256=None, effective_batch_size=4, seed=2026100103,
        batch_domains=publisher.BATCH_DOMAINS, training_dataset_fingerprint_sha256="f" * 64,
        known_dev_is_independent=False, legacy_strata=dict(indices=declared), sampler_remaining_schedule_sha256=schedule_sha)
    checkpoints = [dict(optimizer_updates=i, sha256="c" * 64, export_npz_sha256="e" * 64,
                        export_metadata_sha256="b" * 64) for i in range(500, 3001, 500)]
    summary = dict(flags, schema=publisher.RUN_SCHEMA, status="BOUNDED_BALANCED_REPLAY_COMPLETE_NOT_PROMOTED",
        optimizer_updates=3000, examples_seen=12000, v4_examples_seen=6000, legacy_examples_seen=6000,
        final_checkpoint_sha256="c" * 64, source_hashes_match=True, wall_budget_stop=False,
        optimizer_state_retained=True, checkpoints=checkpoints, elapsed_owned_gpu_phase_wall_seconds=939.,
        fit_phase_including_baseline_seconds=685., optimizer_loop_wall_seconds=437., postfit_endpoint_evaluation_seconds=254.)
    metadata = dict(flags, schema=publisher.EXPORT_SCHEMA, parent_checkpoint_sha256=publisher.PARENT_SHA,
        parent_source_optimizer_updates=200, known_total_optimizer_updates=3200, declaration_sha256=declaration_sha,
        optimizer_updates=3000, examples_seen=12000, v4_examples_seen=6000, legacy_examples_seen=6000,
        maximum_optimizer_updates=3000, batch_domains=publisher.BATCH_DOMAINS, source_sha256=sources,
        checkpoint_sha256="c" * 64, npz_sha256="e" * 64, training_dataset_fingerprint_sha256="f" * 64,
        inference_loader_authorized=False, single_episode_export_compatibility_claimed=False, state_tensor_shapes={"weight": [1]})
    checks = publisher.endpoint_checks(old599, new599, after8, low)
    endpoint = dict(schema=publisher.ENDPOINT_SCHEMA, fixed_endpoint_updates=3000, checks=checks,
        all_diagnostic_screens_passed=False, feature_approved=False, vehicle_control_approved=False,
        model_promoted=False, independent_validation=False)
    documents = {"declaration.json": declaration, "summary.json": summary,
        "started.json": dict(gpu=dict(physical_index=0, uuid="GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5", idle_before_torch=True)),
        "exports/step_003000/metadata.json": metadata, "endpoint_diagnostic_screens.json": endpoint,
        "known_dev_raw_before.json": before8, "known_dev_raw_final.json": after8,
        "known_moving_dev599_raw_before.json": old599, "known_moving_dev599_raw_final.json": new599,
        "legacy_train_raw_before.json": raw_report(legacy, "train"),
        "legacy_train_raw_final.json": raw_report(copy.deepcopy(legacy), "train"),
        "v4_train_joint_before.json": copy.deepcopy(low), "v4_train_joint_final.json": copy.deepcopy(low)}
    return documents, history, declaration_sha


def test_full_report_retains_all_ten_three_failures_and_no_promotion(evidence):
    report = publisher.build_report(*evidence)
    assert report["diagnostic_pass_count"] == 7
    assert len(report["checks"]) == 10 and report["all_diagnostic_screens_passed"] is False
    assert [c["name"] for c in report["checks"] if not c["passed"]] == [
        "V4_TRAIN_ADE", "V4_TRAIN_FDE", "known_DEV599/known_straight/raw_preferred_speed"]
    assert report["official_function_approvals"] == 0 and report["official_function_total"] == 30
    assert report["lineage"]["known_total_updates"] == 3200
    assert report["data"]["actual_exposures"] == 12000
    assert report["actual_journal"]["distinct_eligible_anchors"] == 2814
    assert len(report["raw_comparisons"]) == (3 + 2) * 3 + (9 + 2) * 3
    assert report["model_promoted"] is report["simulator_control_authorized"] is False


@pytest.mark.parametrize("field,value", [("one_episode_screening", True), ("training_episode_count", 1),
    ("model_promoted", True), ("vehicle_control_approved", True), ("inference_loader_authorized", True),
    ("optimizer_updates", 2500), ("examples_seen", 11996), ("parent_checkpoint_sha256", "0" * 64),
    ("known_total_optimizer_updates", 3000), ("legacy_decoded", True), ("labels_clipped", True)])
def test_rejects_falsified_export_lineage_or_scope(evidence, field, value):
    documents, history, sha = evidence
    documents["exports/step_003000/metadata.json"][field] = value
    with pytest.raises(ValueError): publisher.build_report(documents, history, sha)


@pytest.mark.parametrize("mutation", ["drop", "duplicate", "different_final_token", "faked_average", "split"])
def test_full_raw_denominator_cannot_be_cherry_picked(evidence, mutation):
    documents, history, sha = evidence
    report = documents["known_moving_dev599_raw_final.json"]
    if mutation == "drop": report["rows"].pop()
    elif mutation == "duplicate": report["rows"][1] = copy.deepcopy(report["rows"][0])
    elif mutation == "different_final_token": report["rows"][0]["token"] = "replaced"
    elif mutation == "faked_average": report["groups"]["all_eligible"]["raw_preferred_speed"] = .0
    elif mutation == "split": report["split"] = "test"
    with pytest.raises(ValueError): publisher.build_report(documents, history, sha)


def test_gate_equality_is_inclusive_but_near_miss_not_rounded_to_pass(evidence):
    documents, _, _ = evidence
    before, after = documents["known_moving_dev599_raw_before.json"], documents["known_moving_dev599_raw_final.json"]
    low = documents["v4_train_joint_final.json"]
    low["mean_ade_6p4s_m"], low["mean_fde_6p4s_m"] = .1, .2500000001
    checks = publisher.endpoint_checks(before, after, documents["known_dev_raw_final.json"], low)
    assert checks[0]["passed"] is True and checks[1]["passed"] is False


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1., True, "0.1"])
def test_gate_requires_actual_finite_numeric(value):
    with pytest.raises(ValueError): publisher.number(value)


@pytest.mark.parametrize("mutation", ["drop_update", "wrong_order", "fake_count", "duplicate_low", "wrong_stratum",
                                     "fake_token", "gradient_nonfinite", "wrong_objective", "wrong_schedule"])
def test_actual_update_journal_is_bound_to_true_domains(evidence, mutation):
    documents, rows, sha = evidence
    if mutation == "drop_update": rows.pop()
    elif mutation == "wrong_order": rows[0]["step"] = 2
    elif mutation == "fake_count": rows[-1]["examples_seen"] = 12001
    elif mutation == "duplicate_low": rows[0]["indices"][1] = rows[0]["indices"][0]
    elif mutation == "wrong_stratum": rows[0]["indices"][2] = rows[0]["indices"][3]
    elif mutation == "fake_token": rows[0]["sample_ids"][0] = "TEST:forbidden"
    elif mutation == "gradient_nonfinite": rows[0]["domain_parameter_gradients"]["legacy"]["all_present_parameter_gradients_finite"] = False
    elif mutation == "wrong_objective": rows[0]["total_loss"] = .7
    elif mutation == "wrong_schedule": documents["declaration.json"]["sampler_remaining_schedule_sha256"] = "0" * 64
    with pytest.raises(ValueError): publisher.build_report(documents, rows, sha)


def test_never_accept_rewritten_gate_receipt(evidence):
    documents, history, sha = evidence
    documents["endpoint_diagnostic_screens.json"]["checks"][0]["passed"] = True
    with pytest.raises(ValueError, match="endpoint receipt"): publisher.build_report(documents, history, sha)


def test_numeric_not_run_never_loads_torch_or_claims_pass(evidence, monkeypatch):
    metadata = evidence[0]["exports/step_003000/metadata.json"]
    monkeypatch.setattr(publisher, "load_numeric_model", lambda *args: pytest.fail("numeric loader called without authorization"))
    receipt = publisher.numeric_receipt(Path("unused"), metadata, "b" * 64, None, Path("unused"), False)
    assert receipt["status"] == "NOT_RUN" and receipt["full_numeric_state_validated"] is False
    assert receipt["inference_loader_authorized"] is receipt["simulator_control_authorized"] is False


def test_explicit_numeric_receipt_retains_false_authorization_and_endpoint(evidence, monkeypatch):
    metadata = evidence[0]["exports/step_003000/metadata.json"]
    model = SimpleNamespace(_research_temporal_export_metadata=copy.deepcopy(metadata),
        _research_balanced_simulator_control_authorized=False,
        _research_temporal_checkpoint_sha256=metadata["checkpoint_sha256"], training=False,
        _research_balanced_numeric_loader_schema="reviewed-numeric-schema",
        state_dict=lambda: {"weight": object()}, parameters=lambda: [SimpleNamespace(requires_grad=False)])
    monkeypatch.setattr(publisher, "load_numeric_model", lambda *args: model)
    receipt = publisher.numeric_receipt(Path("unused"), metadata, "b" * 64, "root-offline-owner", Path("unused"), True)
    assert receipt["status"] == "PASS_NUMERIC_CPU_ONLY_NOT_CONTROL" and receipt["tensor_count"] == 1
    assert metadata["inference_loader_authorized"] is False and metadata["one_episode_screening"] is False
    assert receipt["simulator_control_authorized"] is receipt["end_to_end_10Hz_validated"] is False


def test_numeric_loader_cannot_relabel_original_metadata(evidence, monkeypatch):
    metadata = evidence[0]["exports/step_003000/metadata.json"]
    model = SimpleNamespace(_research_temporal_export_metadata=dict(metadata, inference_loader_authorized=True),
        _research_balanced_simulator_control_authorized=False,
        _research_temporal_checkpoint_sha256=metadata["checkpoint_sha256"])
    monkeypatch.setattr(publisher, "load_numeric_model", lambda *args: model)
    with pytest.raises(ValueError, match="altered metadata"):
        publisher.numeric_receipt(Path("unused"), metadata, "b" * 64, "owner", Path("unused"), True)


def test_read_json_rejects_duplicates_and_nonfinite(tmp_path):
    duplicate, nonfinite = tmp_path / "duplicate.json", tmp_path / "nonfinite.json"
    duplicate.write_text('{"passed":false,"passed":true}')
    nonfinite.write_text('{"score":NaN}')
    with pytest.raises(ValueError): publisher.read_json(duplicate)
    with pytest.raises(ValueError): publisher.read_json(nonfinite)


def test_create_only_writer_and_symlink_guard(tmp_path):
    target = tmp_path / "receipt.json"
    publisher.write_new(target, dict(model_promoted=False))
    with pytest.raises(FileExistsError): publisher.write_new(target, dict(model_promoted=True))
    link = tmp_path / "link.json"; link.symlink_to(target)
    with pytest.raises(ValueError): publisher.regular(link)


def test_report_rejects_existing_directory_before_loading_models(tmp_path, monkeypatch):
    output = tmp_path / "existing"; output.mkdir()
    monkeypatch.setattr(publisher, "load_numeric_model", lambda *args: pytest.fail("numeric load before fresh output guard"))
    with pytest.raises(ValueError, match="fresh create-only"):
        publisher.publish(tmp_path, output, summary_sha256="a" * 64, declaration_sha256="b" * 64,
                          numeric_validation=True, owner_id="owner", root=tmp_path)


def test_readable_report_discloses_three_failures_training_scope_and_official_zero(evidence):
    report = publisher.build_report(*evidence)
    text = publisher.markdown(report, dict(status="NOT_RUN"), Path("immutable-run"))
    assert "7개 통과 / 3개 미달" in text and "0/30" in text
    assert text.count("FAIL") == 3 and "Town03 직진 raw-speed" in text
    assert "599" in text and "665" in text and "독립 DEV는 없습니다" in text
    assert "지도 모델은 이번 주행 모델 학습에 포함되지" in text
