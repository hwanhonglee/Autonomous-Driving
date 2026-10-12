"""HH_261001 - Test honest full48 fixed-endpoint publication using synthetic evidence, without GPU or CARLA."""
import copy
from collections import Counter
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import publish_temporal_route_recovery_moving_v4_results_20261001 as publisher


def scope():
    return dict(training_only=True, simulation_only=True, main_training=False, model_promoted=False,
        vehicle_control_approved=False, inference_loader_authorized=False, stop_intent_trained=False,
        labels_clipped=False, labels_extrapolated=False, backbone_frozen=True, independent_validation=False,
        legacy_decoded=False, checkpoint_selection_by_holdout=False, parent_all_diagnostic_screens_passed=False,
        parent_diagnostic_pass_count=7, runtime_intent="explicit_cruise_GO_not_head", training_episode_count=18,
        recovery_training_episode_count=9, replay_training_episode_count=9, holdout_episode_count=2,
        new_signed_recovery_cohort_moving_only=True, startup_recovery_fitted=False,
        raw_startup_retained_outside_new_signed_recovery_training=True, startup_or_route_goal_capability_claimed=False,
        recovery_data_revision="prospective_actual_signed_recovery_moving_v4",
        startup_recovery_scope="NEW signed-recovery cohort only; previous center/cruise and old TRAIN replay unchanged",
        fixed_recovery_driving_relative_window_seconds=[4.0, 34.1], fixed_recovery_native_frame_offsets=[80, 682],
        earliest_full_raw_history_driving_relative_seconds=3.1, true_future_seconds=6.4, fixed_recovery_anchors_per_episode=302)


def joint_report(prefix, episodes, count, split, *, final=False):
    rows = []
    for i in range(episodes*count):
        rows.append(dict(index=i, token=prefix+":"+str(i), episode_id=prefix+"_ep"+str(i//count),
            ade_6p4s_m=.09 if final else .2, fde_6p4s_m=.22 if final else .4,
            instantaneous_speed_mae_mps=.03, raw_spatial_preferred_speed_mae_mps=.04,
            true_near_curvature_inv_m=.02, predicted_near_curvature_inv_m=.018 if final else 0.,
            learned_near_branch_logit_mean=.09 if final else 0.))
    report = dict(split=split, independent_validation=False, full_fixed_denominator=len(rows), rows=rows,
        mean_ade_6p4s_m=rows[0]["ade_6p4s_m"], maximum_ade_6p4s_m=rows[0]["ade_6p4s_m"],
        mean_fde_6p4s_m=rows[0]["fde_6p4s_m"], mean_instantaneous_speed_mae_mps=.03,
        role="TRAIN_resubstitution" if split == "train" else "never_fit_actual_signed_recovery_HOLDOUT",
        per_episode={prefix+"_ep"+str(ep): dict(count=count, **{key: rows[0][key] for key in publisher.JOINT_COMPONENTS})
            for ep in range(episodes)})
    if count != 327: report["actual_signed_current_state_counts"] = {"(1, 0)": len(rows)}
    return report


def raw_row(index, episode, map_id, stratum):
    return dict(index=index, token=episode+":"+str(index), episode_id=episode, map=map_id, stratum=stratum,
        raw_curvature=.01, raw_preferred_speed=.02, total=.0225, raw_spatial_preferred_speed_mae_mps=.05,
        intent=0., stop_distance=0., initial_acceleration=0., temporal_xy=0., temporal_speed=0.)


def raw_report(rows, split):
    def metrics(selected):
        return dict(count=len(selected), **{key: sum(row[key] for row in selected)/len(selected) for key in publisher.RAW_COMPONENTS})
    return dict(split=split, independent_validation=False, legacy_decoded=False, full_fixed_eligible_denominator=len(rows), rows=rows,
        groups={key: metrics(rows if key == "all_eligible" else [row for row in rows if row["stratum"] == key])
            for key in ("all_eligible", "curved", "straight")},
        per_episode={ep: metrics([row for row in rows if row["episode_id"] == ep]) for ep in sorted({r["episode_id"] for r in rows})},
        speed_metric_notice="raw_preferred_speed is original SmoothL1 loss, NOT m/s; raw_spatial_preferred_speed_mae_mps is physical MAE. No legacy decoded MAE exists.")


def response(before, after):
    rows = []
    for old, new in zip(before["rows"], after["rows"]):
        wanted = new["true_near_curvature_inv_m"]-old["predicted_near_curvature_inv_m"]
        change = new["predicted_near_curvature_inv_m"]-old["predicted_near_curvature_inv_m"]
        rows.append(dict(token=new["token"], expert_minus_parent_near_curvature=wanted,
            actual_learned_near_curvature_change=change, learned_branch_logit=new["learned_near_branch_logit_mean"],
            signed_response_eligible=abs(wanted)>=.001, signed_response_agrees=wanted*change>0))
    return dict(research_diagnostic_only=True, no_inference_oracle=True, feature_approved=False,
        groups={"(1, 0)": dict(all_anchor_count=len(rows), signed_response_denominator=len(rows), signed_response_agree_count=len(rows), rows=rows)})


@pytest.fixture
def evidence():
    # HH_261001 - These fabricated arrays/journals test contracts only, never model performance or actual capture admission.
    before = dict(recovery_train=joint_report("recovery", 9, 302, "train"),
        recovery_holdout=joint_report("holdout", 2, 302, "val"), v4=joint_report("v4", 1, 327, "train"))
    after = dict(recovery_train=joint_report("recovery", 9, 302, "train", final=True),
        recovery_holdout=joint_report("holdout", 2, 302, "val", final=True), v4=joint_report("v4", 1, 327, "train", final=True))
    legacy = []
    for ep, (count, map_id) in enumerate(zip([266, 328, 328, 310, 328, 271, 328, 328],
            ["C_track_1_0_7"]*3+["Town01"]*2+["Town07"]*3)):
        for position in range(count):
            kind = "curved" if (ep == 0 and position < 260) or (ep == 3 and position < 68) else "straight"
            legacy.append(raw_row(len(legacy), "legacy"+str(ep), map_id, kind))
    known = [raw_row(i, "known_curve" if i < 271 else "known_straight", "Town03", "curved" if i < 271 else "straight") for i in range(599)]
    for values in (before, after):
        values["legacy"] = raw_report(copy.deepcopy(legacy), "train")
        values["known599"] = dict(raw_report(copy.deepcopy(known), "val"), available_known_val_count=665, excluded_count=66,
            role="previously_exposed_known_moving_DEV599_raw_diagnostic")
    # HH_261001 - One near miss is retained, with all302 anchors/aggregates consistently updated rather than only editing a gate.
    failed_ep = "recovery_ep0"
    for row in after["recovery_train"]["rows"]:
        if row["episode_id"] == failed_ep: row["fde_6p4s_m"] = .2500000001
    after["recovery_train"]["per_episode"][failed_ep]["fde_6p4s_m"] = .2500000001
    after["recovery_train"]["mean_fde_6p4s_m"] = sum(r["fde_6p4s_m"] for r in after["recovery_train"]["rows"])/2718
    domains = {kind: {map_id: [r for r in legacy if r["stratum"] == kind and r["map"] == map_id]
        for map_id in sorted({r["map"] for r in legacy if r["stratum"] == kind})} for kind in ("curved", "straight")}
    history, map_count, unique = [], Counter(), {key: set() for key in ("recovery", "v4", "legacy")}
    collisions = 0
    for i in range(3000):
        kind = "curved" if i % 2 == 0 else "straight"
        maps = sorted(domains[kind]); map_id = maps[map_count[kind] % len(maps)]
        old_rows = domains[kind][map_id]; old = old_rows[(map_count[kind]//len(maps)) % len(old_rows)]; map_count[kind] += 1
        indices = [i % 2718, (i % 9)*302 + (i//9)%302, 2718+i%327, 3045+old["index"]]
        tokens = [before["recovery_train"]["rows"][index]["token"] for index in indices[:2]]
        tokens += [before["v4"]["rows"][i%327]["token"], old["token"]]
        unique["recovery"].update(tokens[:2]); unique["v4"].add(tokens[2]); unique["legacy"].add(tokens[3]); collisions += indices[0] == indices[1]
        history.append(dict(optimizer_updates=i+1, examples_seen=4*(i+1), indices=indices, sample_ids=tokens,
            loss=.75*.2+.0625*.01, recovery0=dict(total=.2), recovery1=dict(total=.2), v4=dict(total=.2), legacy_raw_curvature=.01,
            per_domain_branch_gradients={key: dict(all_present_branch_gradients_finite=True, gradient_l2_norm=.1,
                unused_branch_tensors=0) for key in ("recovery0", "recovery1", "v4", "legacy_raw_curvature")},
            learning_rate=publisher.learning_rate(i+1), preclip_branch_gradient_norm=.2, elapsed_owned_seconds=i*.05))
    declaration_sha = "a"*64
    sources = {key: "d"*64 for key in publisher.SOURCE_NAMES}; sources.update(publisher.FROZEN_SOURCES)
    shared = dict(scope(), parent_checkpoint_sha256=publisher.PARENT_CHECKPOINT_SHA, parent_npz_sha256=publisher.PARENT_NPZ_SHA,
        parent_metadata_sha256=publisher.PARENT_METADATA_SHA, source_sha256=sources, learning_rate=copy.deepcopy(publisher.RATE),
        batch_domains=publisher.BATCH_DOMAINS, trainable_state_names=sorted(publisher.BRANCH_SHAPES),
        model_config=dict(test_backbone="fixture_not_actual_model"), recovery_config=dict(test_branch="fixture_not_actual_model"),
        training_dataset_fingerprint_sha256="f"*64, holdout_dataset_fingerprint_sha256="e"*64, backbone_state_sha256="b"*64)
    declaration = dict(shared, schema=publisher.RUN_SCHEMA, requested_optimizer_updates=3000, maximum_optimizer_updates=3000,
        effective_batch_size=4, seed=2026100104, holdout_fitted=False, holdout_selected_checkpoint=False,
        cohorts=dict(recovery_train=dict(split="train", sample_count=2718), recovery_holdout=dict(split="val", sample_count=604),
            replay=dict(v4=dict(statistics=dict(anchor_count=327)), legacy=dict(sample_count=2787, split="train"),
                known_dev=dict(sample_count=665, split="val"))), actual_dataset_available_train_anchor_count=5832,
        actual_eligible_train_pool_anchor_count=5532, maximum_fit_including_baseline_seconds=2700, postfit_checks_separately_timed=True,
        fixed_schedule_sha256=hashlib.sha256(json.dumps([r["indices"] for r in history], separators=(",", ":")).encode()).hexdigest(),
        declared_unique_source_exposure_counts={key: len(values) for key, values in unique.items()},
        declared_focus_full_same_anchor_batch_count=collisions,
        actual_profile_full302_counts={"profile"+str(i): 302 for i in range(9)}, actual_noncentered_grid_counts={"(1, 0)": 2718})
    metadata = dict(shared, schema=publisher.EXPORT_SCHEMA, architecture_schema=publisher.ARCHITECTURE_SCHEMA, model_id=publisher.MODEL_ID,
        optimizer_updates=3000, maximum_optimizer_updates=3000, additional_optimizer_updates=3000, effective_batch_size=4,
        examples_seen=12000, recovery_examples_seen=6000, v4_examples_seen=3000, legacy_examples_seen=3000,
        parent_optimizer_updates=3000, parent_known_total_optimizer_updates=3200, known_total_optimizer_updates=6200,
        declaration_sha256=declaration_sha, checkpoint_sha256="c"*64, npz_sha256="e"*64, state_tensor_shapes={"fixture": [1]})
    endpoint = dict(fixed_endpoint_updates=3000, diagnostic_screen_count=48, checks=publisher.endpoint_checks(before, after),
        all_diagnostic_screens_passed=False, independent_validation=False, feature_approved=False, vehicle_control_approved=False,
        model_promoted=False, absolute_speed_tracking_claimed=False, frozen_parent_7of10_not_relabelled=True)
    checkpoints = [dict(optimizer_updates=step, sha256="c"*64, export_npz_sha256="e"*64,
        export_metadata_sha256="b"*64, optimizer_state_retained=True) for step in range(500, 3001, 500)]
    summary = dict(scope(), schema=publisher.RUN_SCHEMA, status="FIXED3000_BRANCH_COMPLETE_NOT_PROMOTED",
        optimizer_updates=3000, examples_seen=12000, recovery_examples_seen=6000, v4_examples_seen=3000, legacy_examples_seen=3000,
        wall_budget_stop=False, backbone_97_tensors_byte_identical=True, source_hashes_match=True, checkpoints=checkpoints,
        endpoint_diagnostic_screens=endpoint, fit_including_baseline_seconds=1600., postfit_endpoint_seconds=400., elapsed_owned_seconds=2000.)
    documents = {"declaration.json": declaration, "summary.json": summary, "exports/step_003000/metadata.json": metadata,
        "endpoint_diagnostic_screens.json": endpoint, "started.json": dict(gpu=dict(physical_index=0,
            uuid="GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5", idle_before_torch=True),
            frozen_backbone_sha256="b"*64, trainable_branch_parameter_count=1409),
        "actual_signed_branch_response_train.json": response(before["recovery_train"], after["recovery_train"]),
        "actual_signed_branch_response_holdout.json": response(before["recovery_holdout"], after["recovery_holdout"])}
    for stage, values in (("before", before), ("final", after)):
        documents.update({key+"_"+stage+".json": value for key, value in values.items()})
    return documents, history, declaration_sha


def test_full48_report_keeps_near_miss_and_official_zero(evidence):
    report = publisher.build_report(*evidence)
    assert report["diagnostic_gate_count"] == 48 and report["diagnostic_pass_count"] == 47
    assert [r["name"] for r in report["checks"] if not r["passed"]] == ["recovery_train/recovery_ep0/FDE"]
    assert report["official_function_approvals"] == 0 and report["official_function_total"] == 30
    assert report["official_main_function_approvals"] == 0 and report["official_main_function_total"] == 9
    assert report["lineage"]["known_total_updates"] == 6200
    assert report["actual_journal"]["actual_anchor_exposures"] == 12000
    assert report["actual_journal"]["recovery_exposures"] == 6000
    assert report["actual_journal"]["v4_exposures"] == report["actual_journal"]["legacy_exposures"] == 3000
    assert report["data"]["TRAIN_episode_count"] == 18 and report["data"]["HOLDOUT_anchors"] == 604
    assert report["startup_recovery_fitted"] is report["physical_3x45_trials_included"] is False
    assert len(report["joint_comparisons"]) == 12 and len(report["raw_comparisons"]) == 64


@pytest.mark.parametrize("field,value", [("schema", "old"), ("startup_recovery_fitted", True), ("model_promoted", True),
    ("inference_loader_authorized", True), ("labels_clipped", True), ("legacy_decoded", True),
    ("training_episode_count", 9), ("holdout_episode_count", 0), ("examples_seen", 11996),
    ("optimizer_updates", 2500), ("known_total_optimizer_updates", 6000), ("parent_npz_sha256", "0"*64),
    ("fixed_recovery_driving_relative_window_seconds", [.9, 31.0]), ("source_sha256", {})])
def test_rejects_falsified_lineage_membership_or_promotion(evidence, field, value):
    documents, rows, sha = evidence; documents["exports/step_003000/metadata.json"][field] = value
    with pytest.raises(ValueError): publisher.build_report(documents, rows, sha)


@pytest.mark.parametrize("mutation", ["missing", "duplicate", "foreign", "average", "episode_count", "role"])
def test_all604_holdout_anchors_are_not_selected_or_relabelled(evidence, mutation):
    documents, rows, sha = evidence; report = documents["recovery_holdout_final.json"]
    if mutation == "missing": report["rows"].pop()
    elif mutation == "duplicate": report["rows"][1] = copy.deepcopy(report["rows"][0])
    elif mutation == "foreign": report["rows"][0]["token"] = "replacement"
    elif mutation == "average": report["mean_ade_6p4s_m"] = 0.
    elif mutation == "episode_count": report["per_episode"].popitem()
    elif mutation == "role": report["role"] = "unseen_map_generalization"
    with pytest.raises(ValueError): publisher.build_report(documents, rows, sha)


@pytest.mark.parametrize("mutation", ["missing_update", "order", "exposure", "holdout", "legacy_geometry", "objective",
    "gradient", "learning_rate", "schedule", "collision_count", "fake_unique_count"])
def test_actual_journal_binds3000_fixed_train_only_updates(evidence, mutation):
    documents, rows, sha = evidence
    if mutation == "missing_update": rows.pop()
    elif mutation == "order": rows[0]["optimizer_updates"] = 2
    elif mutation == "exposure": rows[-1]["examples_seen"] = 12001
    elif mutation == "holdout": rows[0]["sample_ids"][0] = documents["recovery_holdout_before.json"]["rows"][0]["token"]
    elif mutation == "legacy_geometry": rows[0]["indices"][3] = rows[1]["indices"][3]
    elif mutation == "objective": rows[0]["loss"] = 1.
    elif mutation == "gradient": rows[0]["per_domain_branch_gradients"]["recovery0"]["all_present_branch_gradients_finite"] = False
    elif mutation == "learning_rate": rows[0]["learning_rate"] += 1e-9
    elif mutation == "schedule": documents["declaration.json"]["fixed_schedule_sha256"] = "0"*64
    elif mutation == "collision_count": documents["declaration.json"]["declared_focus_full_same_anchor_batch_count"] += 1
    elif mutation == "fake_unique_count": documents["declaration.json"]["declared_unique_source_exposure_counts"]["legacy"] += 1
    with pytest.raises(ValueError): publisher.build_report(documents, rows, sha)


def test_all48_acceptance_flags_and_original_parent7of10_are_retained(evidence):
    documents, rows, sha = evidence
    documents["endpoint_diagnostic_screens.json"]["checks"][1]["passed"] = True
    with pytest.raises(ValueError, match="original48"): publisher.build_report(documents, rows, sha)


@pytest.mark.parametrize("mutation", ["raw_as_mps", "physical_average", "omitted_response", "altered_response", "pretend_startup"])
def test_signed_response_and_raw_speed_units_cannot_be_forged(evidence, mutation):
    documents, rows, sha = evidence
    if mutation == "raw_as_mps": documents["legacy_final.json"]["speed_metric_notice"] = "all speed numbers are m/s"
    elif mutation == "physical_average": documents["legacy_final.json"]["groups"]["all_eligible"]["raw_spatial_preferred_speed_mae_mps"] = 0.
    elif mutation == "omitted_response": documents["actual_signed_branch_response_train.json"]["groups"]["(1, 0)"]["rows"].pop()
    elif mutation == "altered_response": documents["actual_signed_branch_response_train.json"]["groups"]["(1, 0)"]["rows"][0]["actual_learned_near_curvature_change"] = 1.
    elif mutation == "pretend_startup": documents["summary.json"]["startup_recovery_fitted"] = True
    with pytest.raises(ValueError): publisher.build_report(documents, rows, sha)


def test_readable_report_has_full_failures_training_scope_and_separate_physics(evidence):
    report = publisher.build_report(*evidence)
    text = publisher.markdown(report, dict(status="ARCHIVES_CHECKED"), Path("immutable-run"))
    assert "47개 통과 / 1개 미달" in text and "0/9" in text and "0/30" in text and text.count("FAIL") == 1
    assert "4.0~34.1초" in text and "18개 TRAIN" in text and "3×45초" in text
    assert "초기 정지·출발 회복은" in text and "지도 모델은 이번" in text
    assert "독립 검증으로 부르지" in text and "97개 tensor" in text
    comparisons = publisher.comparison_markdown(report)
    assert comparisons.count("recovery_ep") == 9 and comparisons.count("holdout_ep") == 2
    assert "SmoothL1" in comparisons and "raw_spatial_preferred_speed_mae_mps" in comparisons


def test_optional_model_loader_is_never_called_without_explicit_cpu_authorization(evidence, monkeypatch):
    metadata = evidence[0]["exports/step_003000/metadata.json"]
    monkeypatch.setattr(publisher, "load_numeric_model", lambda *args: pytest.fail("unauthorized Torch model load"))
    receipt = publisher.numeric_receipt(Path("unused"), metadata, "b"*64, None, Path("unused"), Path("unused"), Path("unused"), False, {})
    assert receipt["reviewed_model_loader_run"] is False and receipt["simulator_control_authorized"] is False
    assert receipt["status"] == "PASS_ARCHIVES_ONLY_REVIEWED_MODEL_LOADER_NOT_RUN"


def test_reviewed_model_loader_receipt_has101_and_preserves_false_authorization(evidence, monkeypatch):
    metadata = evidence[0]["exports/step_003000/metadata.json"]
    keys = {"fixture"+str(i): [1] for i in range(101)}; metadata["state_tensor_shapes"] = keys
    model = SimpleNamespace(_research_temporal_export_metadata=copy.deepcopy(metadata),
        _research_recovery_simulator_control_authorized=False, _research_temporal_checkpoint_sha256=metadata["checkpoint_sha256"],
        _research_recovery_numeric_loader_schema="reviewed", training=False,
        state_dict=lambda: {key: object() for key in keys}, parameters=lambda: [SimpleNamespace(requires_grad=False)])
    monkeypatch.setattr(publisher, "load_numeric_model", lambda *args: model)
    receipt = publisher.numeric_receipt(Path("unused"), metadata, "b"*64, "offline-owner", Path("unused"), Path("unused"), Path("unused"), True, {})
    assert receipt["tensor_count"] == 101 and receipt["status"] == "PASS_FULL101_NUMERIC_CPU_ONLY_NOT_CONTROL"
    assert metadata["inference_loader_authorized"] is receipt["simulator_control_authorized"] is False


@pytest.mark.parametrize("mutation", ["float64", "nan", "shape", "extra", "object", "wrong_sha"])
def test_strict_numeric_archive_uses_finitefloat32_completekeys_nopickle(tmp_path, mutation):
    import numpy as np
    arrays = {"weight": np.array([1.], dtype=np.float32)}
    if mutation == "float64": arrays["weight"] = arrays["weight"].astype(np.float64)
    elif mutation == "nan": arrays["weight"][0] = np.nan
    elif mutation == "shape": arrays["weight"] = np.zeros((2,), dtype=np.float32)
    elif mutation == "extra": arrays["unused"] = arrays["weight"]
    elif mutation == "object": arrays["weight"] = np.array([dict(not_allowed=True)], dtype=object)
    path = tmp_path/"state.npz"
    with path.open("xb") as stream: np.savez(stream, **arrays)
    sha = "0"*64 if mutation == "wrong_sha" else publisher.digest(path)
    with pytest.raises(ValueError): publisher.bounded_npz(path, sha, {"weight": [1]}, {"weight": "float32"})


def test_numeric_archive_parent_bytes_include_signedzero(tmp_path):
    import numpy as np
    path = tmp_path/"state.npz"
    with path.open("xb") as stream: np.savez(stream, weight=np.array([-0.], dtype=np.float32))
    arrays = publisher.bounded_npz(path, publisher.digest(path), {"weight": [1]}, {"weight": "float32"})
    assert publisher.backbone_fingerprint(arrays) != publisher.backbone_fingerprint({"weight": np.array([0.], dtype=np.float32)})


@pytest.fixture
def archive_evidence(tmp_path, evidence, monkeypatch):
    # HH_261001 - Tiny97+4 synthetic tensors exercise every retained file boundary, not an actual trained model.
    import numpy as np
    run, root = tmp_path/"run", tmp_path/"source-root"; run.mkdir(); root.mkdir()
    documents = evidence[0]; declaration = documents["declaration.json"]; summary = documents["summary.json"]
    parent_arrays = {"original.%02d" % i: np.array([float(i)], dtype=np.float32) for i in range(97)}
    parent_dir = tmp_path/"frozen-parent-export"; parent_dir.mkdir()
    parent_weights, parent_metadata = parent_dir/"parent.npz", parent_dir/"parent.json"
    with parent_weights.open("xb") as stream: np.savez(stream, **parent_arrays)
    parent = dict(schema=publisher.prior.EXPORT_SCHEMA, checkpoint_sha256=publisher.PARENT_CHECKPOINT_SHA,
        declaration_sha256=publisher.PARENT_DECLARATION_SHA, known_total_optimizer_updates=3200, optimizer_updates=3000,
        model_promoted=False, vehicle_control_approved=False, model_config=declaration["model_config"],
        state_tensor_shapes={key: list(value.shape) for key, value in parent_arrays.items()},
        state_tensor_dtypes={key: "float32" for key in parent_arrays})
    publisher.write_new(parent_metadata, parent)
    monkeypatch.setattr(publisher, "PARENT_NPZ_SHA", publisher.digest(parent_weights))
    monkeypatch.setattr(publisher, "PARENT_METADATA_SHA", publisher.digest(parent_metadata))
    for name in declaration["source_sha256"]:
        for base in (root, run/"source"):
            path = base/name; path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(b"synthetic immutable source")
        declaration["source_sha256"][name] = publisher.digest(root/name)
    helper = root/publisher.PRIOR_FILE; helper.parent.mkdir(parents=True, exist_ok=True); helper.write_bytes(b"synthetic audit helper")
    monkeypatch.setattr(publisher, "PRIOR_SHA", publisher.digest(helper))
    publisher.write_new(run/"declaration.json", declaration); declaration_sha = publisher.digest(run/"declaration.json")
    backbone_sha = publisher.backbone_fingerprint(parent_arrays)
    final = None
    for step in range(500, 3001, 500):
        folder = run/"exports"/("step_%06d" % step); folder.mkdir(parents=True)
        checkpoint = run/"checkpoints"/("step_%06d.pt" % step); checkpoint.parent.mkdir(exist_ok=True)
        checkpoint.write_bytes(b"opaque synthetic checkpoint never deserialized"+str(step).encode())
        arrays = dict(parent_arrays, **{key: np.zeros(shape, dtype=np.float32) for key, shape in publisher.BRANCH_SHAPES.items()})
        with (folder/"model_state.npz").open("xb") as stream: np.savez(stream, **arrays)
        middle = dict(scope(), schema=publisher.EXPORT_SCHEMA, learning_rate=copy.deepcopy(publisher.RATE),
            model_config=declaration["model_config"], source_sha256=declaration["source_sha256"], declaration_sha256=declaration_sha,
            parent_checkpoint_sha256=publisher.PARENT_CHECKPOINT_SHA, backbone_state_sha256=backbone_sha,
            checkpoint_sha256=publisher.digest(checkpoint), npz_sha256=publisher.digest(folder/"model_state.npz"),
            optimizer_updates=step, additional_optimizer_updates=step, examples_seen=4*step,
            recovery_examples_seen=2*step, v4_examples_seen=step, legacy_examples_seen=step, known_total_optimizer_updates=3200+step,
            state_tensor_shapes={key: list(value.shape) for key, value in arrays.items()}, state_tensor_dtypes={key: "float32" for key in arrays})
        publisher.write_new(folder/"metadata.json", middle)
        summary["checkpoints"][step//500-1].update(sha256=middle["checkpoint_sha256"], export_npz_sha256=middle["npz_sha256"],
            export_metadata_sha256=publisher.digest(folder/"metadata.json"))
        if step == 3000: final = middle
    documents["exports/step_003000/metadata.json"] = final
    publisher.write_new(run/"summary.json", summary)
    kwargs = dict(summary_sha256=publisher.digest(run/"summary.json"), declaration_sha256=declaration_sha,
        checkpoint_sha256=final["checkpoint_sha256"], weights_sha256=final["npz_sha256"],
        metadata_sha256=publisher.digest(run/"exports/step_003000/metadata.json"), parent_weights=parent_weights, parent_metadata=parent_metadata)
    return run, root, documents, kwargs


def test_all_six_exports_and97_parent_bytes_are_verified_without_torch_pickle(archive_evidence):
    run, root, documents, kwargs = archive_evidence
    receipt = publisher.verify_evidence_files(run, root, documents, **kwargs)
    assert receipt["all_six_exports_finite_and_parent97_byte_identical"] is True
    assert receipt["final_tensor_count"] == 101 and receipt["no_Torch_pickle"] is True
    assert [row["step"] for row in receipt["fixed_exports"]] == list(range(500, 3001, 500))


@pytest.mark.parametrize("mutation", ["parent_byte", "checkpoint_byte", "intermediate_metadata_byte", "archived_source_byte", "external_final_sha"])
def test_actual_endpoint_and_retained_intermediates_cannot_be_changed(archive_evidence, mutation):
    run, root, documents, kwargs = archive_evidence
    if mutation == "parent_byte": kwargs["parent_weights"].write_bytes(b"changed parent")
    elif mutation == "checkpoint_byte": (run/"checkpoints/step_000500.pt").write_bytes(b"changed checkpoint")
    elif mutation == "intermediate_metadata_byte": (run/"exports/step_001500/metadata.json").write_bytes(b"changed metadata")
    elif mutation == "archived_source_byte": (run/"source"/next(iter(publisher.SOURCE_NAMES))).write_bytes(b"changed source")
    elif mutation == "external_final_sha": kwargs["weights_sha256"] = "0"*64
    with pytest.raises(ValueError): publisher.verify_evidence_files(run, root, documents, **kwargs)


@pytest.mark.parametrize("inside", ["run", "parent_export"])
def test_fresh_report_cannot_mutate_an_immutable_input_directory(archive_evidence, inside):
    run, root, _, kwargs = archive_evidence
    output = (run if inside == "run" else kwargs["parent_weights"].parent)/"fresh-report"
    with pytest.raises(ValueError, match="outside immutable"):
        publisher.publish(run, output, root=root, **kwargs)
    assert not output.exists()


@pytest.mark.parametrize("field", ["run", "root", "output", "parent_weights"])
def test_nested_symlink_ancestors_are_rejected_before_publication(archive_evidence, tmp_path, field):
    run, root, _, kwargs = archive_evidence
    docs = tmp_path/"docs"; docs.mkdir(); output = docs/"new-report"
    if field == "run":
        alias = tmp_path/"run-alias"; alias.symlink_to(run, target_is_directory=True); run = alias
    elif field == "root":
        alias = tmp_path/"root-alias"; alias.symlink_to(root, target_is_directory=True); root = alias
    elif field == "output":
        alias = tmp_path/"docs-alias"; alias.symlink_to(docs, target_is_directory=True); output = alias/"new-report"
    else:
        alias = tmp_path/"assets-alias"; alias.symlink_to(kwargs["parent_weights"].parent, target_is_directory=True)
        kwargs["parent_weights"] = alias/kwargs["parent_weights"].name
    with pytest.raises(ValueError, match="canonical path without symlink"):
        publisher.publish(run, output, root=root, **kwargs)
    assert not (docs/"new-report").exists()


@pytest.mark.parametrize("changed", ["history.jsonl", "recovery_train_before.json", "recovery_holdout_final.json", "source", "parent_export"])
def test_first_read_pins_are_rechecked_after_numeric_validation_before_first_write(archive_evidence, tmp_path, monkeypatch, changed):
    run, root, documents, kwargs = archive_evidence
    for name in publisher.DOCUMENTS:
        path = run/name
        if not path.exists(): publisher.write_new(path, documents[name])
    (run/"history.jsonl").write_text('{"test_only":true}\n')
    docs = root/"docs"; docs.mkdir(); output = docs/"new-report"
    monkeypatch.setattr(publisher, "build_report", lambda *args: dict(diagnostic_pass_count=47))
    def numeric_then_mutate(*args):
        if changed == "source": (root/next(iter(publisher.SOURCE_NAMES))).write_bytes(b"source changed after numeric validation")
        elif changed == "parent_export": kwargs["parent_weights"].write_bytes(b"parent changed after numeric validation")
        else: (run/changed).write_bytes(b"parsed evidence changed after numeric validation")
        return dict(status="TEST_NUMERIC_RECEIPT_ONLY")
    monkeypatch.setattr(publisher, "numeric_receipt", numeric_then_mutate)
    with pytest.raises(ValueError, match="pinned evidence/source/parent/export changed"):
        publisher.publish(run, output, root=root, **kwargs)
    assert not output.exists() and (run/"history.jsonl").exists()


def test_create_only_output_guard_precedes_numeric_loads(tmp_path, monkeypatch):
    output = tmp_path/"existing"; output.mkdir()
    monkeypatch.setattr(publisher, "load_numeric_model", lambda *args: pytest.fail("model load before fresh-output guard"))
    with pytest.raises(ValueError, match="fresh create-only"):
        publisher.publish(tmp_path, output, summary_sha256="a"*64, declaration_sha256="b"*64, checkpoint_sha256="c"*64,
            weights_sha256="d"*64, metadata_sha256="e"*64, parent_weights=tmp_path/"p.npz", parent_metadata=tmp_path/"p.json", root=tmp_path)


def test_create_only_writer_never_overwrites_original_report(tmp_path):
    path = tmp_path/"report.json"; publisher.write_new(path, dict(model_promoted=False))
    with pytest.raises(FileExistsError): publisher.write_new(path, dict(model_promoted=True))
