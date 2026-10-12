"""HH_261001 - Prove fixed TRAIN-only recovery sampling, frozen backbone and honest diagnostic criteria."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import train_temporal_route_recovery_v3_20261001 as trainer


def fixture_groups():
    full = {profile: list(range(i * 302, (i + 1) * 302)) for i, profile in enumerate(sorted(trainer.TRAIN_PROFILES))}
    focus = {(-1, -1): full[(-.2, -1.5)][:30], (-1, 0): full[(-.2, 0.)][:40],
             (1, 0): full[(.2, 0.)][:40], (1, 1): full[(.2, 1.5)][:30]}
    old = dict(curved={"C_track_1_0_7": list(range(260)), "Town01": list(range(260, 328))},
        straight={"C_track_1_0_7": list(range(328, 990)), "Town01": list(range(990, 1560)), "Town07": list(range(1560, 2487))})
    return dict(full=full, focus=focus), old


def test_fixed_effective_b4_domains_allprofiles_no_holdout_no_drop_full_cycle():
    groups, old = fixture_groups(); rows = trainer.fixed_batches(groups, old)
    assert len(rows) == 3000 and all(len(row) == 4 for row in rows)
    focus = {index for values in groups["focus"].values() for index in values}
    assert all(row[0] in focus and 0 <= row[1] < 2718 for row in rows)
    assert {row[1] for row in rows} == set(range(2718))
    assert {row[2] - 2718 for row in rows} == set(range(327))
    for step, row in enumerate(rows):
        kind = "curved" if step % 2 == 0 else "straight"
        assert row[3] - 3045 in {index for values in old[kind].values() for index in values}
    assert len([row for i, row in enumerate(rows) if i % 2 == 0]) == 1500
    assert max(max(row) for row in rows) < 2718 + 327 + 2787
    assert rows == trainer.fixed_batches(groups, old)


def test_noncentered_focus_round_robin_does_not_fake_missing_strata_or_skip_full_tails():
    groups, old = fixture_groups(); rows = trainer.fixed_batches(groups, old)
    assert {row[0] for row in rows} == {i for values in groups["focus"].values() for i in values}
    assert len({row[3] for row in rows}) < 2487  # HH_261001 - 1500 straight exposures are not all2159 anchors.
    empty = dict(groups, focus={})
    with pytest.raises(ValueError, match="nonempty"): trainer.fixed_batches(empty, old)


@pytest.mark.parametrize("value,expected", [(-.05, -1), (.05, 1), (.04999, 0), (-.04999, 0)])
def test_actual_signed_threshold_is_inclusive(value, expected):
    assert trainer.signed_bin(value, .05) == expected


@pytest.mark.parametrize("value", [True, float("nan"), float("inf"), "0.1"])
def test_actual_signed_state_not_initialcase_or_nonfinite(value):
    with pytest.raises(ValueError): trainer.signed_bin(value, .05)


def test_actual_profile_groups_preserve302_and_reject_old_unsafe_v1_profile():
    examples, audits = [], {}
    for profile_number, (offset, yaw) in enumerate(sorted(trainer.TRAIN_PROFILES)):
        for index in range(302):
            token = "episode%d:%d" % (profile_number, index)
            examples.append(SimpleNamespace(token=token, episode_id="episode%d" % profile_number))
            audits[token] = dict(token=token, episode_id="episode%d" % profile_number,
                declared_signed_lateral_offset_ros_m=offset, declared_signed_yaw_offset_ros_deg=yaw,
                actual_measured_rear_base_signed_CTE_m=offset if index < 30 else 0.,
                actual_yaw_route_heading_error_rad=0.)
    result = trainer.recovery_groups(SimpleNamespace(examples=examples), audits)
    assert len(result["full"]) == 9 and set(result["focus"]) == {(-1, 0), (1, 0)}
    changed = copy.deepcopy(audits); changed[examples[0].token]["declared_signed_lateral_offset_ros_m"] = -.35
    with pytest.raises(ValueError): trainer.recovery_groups(SimpleNamespace(examples=examples), changed)
    for value in audits.values(): value[trainer.SIGNED_CTE_KEY] = 0.
    with pytest.raises(ValueError, match="genuine noncentered"): trainer.recovery_groups(SimpleNamespace(examples=examples), audits)


def test_exact_objective_gradient_denominator_matches_four_anchors_not_domain_size_bias():
    values = [torch.tensor(value, requires_grad=True) for value in (1., 2., 3., 4.)]
    loss = trainer.exact_objective(*values); loss.backward()
    assert float(loss) == 1.75
    assert [float(value.grad) for value in values] == [.25, .25, .25, .0625]


def test_freeze_all_original_parameters_only_four_branch_names_trainable():
    values = [("speed_head.weight", torch.nn.Parameter(torch.ones(1))),
              *[(name, torch.nn.Parameter(torch.ones(1))) for name in trainer.BRANCH_KEYS]]
    model = SimpleNamespace(named_parameters=lambda: values)
    trainer.freeze_backbone(model)
    assert {name for name, parameter in model.named_parameters() if parameter.requires_grad} == trainer.BRANCH_KEYS


def test_source_snapshot_is_recursive_and_pins_all17_plus_new_architecture_loader():
    sources = trainer.source_snapshot()
    assert len(sources) > 17
    assert sources["tools/e2e/train_temporal_route_recovery_v3_20261001.py"] == trainer.digest(trainer.ROOT / "tools/e2e/train_temporal_route_recovery_v3_20261001.py")
    assert "learning/portable_e2e/temporal_route_recovery_export_loader_research_20261001.py" in sources
    assert "learning/portable_e2e/temporal_cpu_finite_validation_v2_20261001.py" in sources
    assert not any("collect_" in key or "simulation_research" in key for key in sources)


def test_exact_recursive_source_only_fresh_clone_imports_without_the_original_workspace(tmp_path):
    # HH_261001 - A minimal source-only staging must import lazy GPU ownership helpers without launching them.
    sources = trainer.source_snapshot()
    for name in sources:
        target = tmp_path / name; target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(trainer.ROOT / name, target)
    result = subprocess.run([sys.executable, "-B", str(tmp_path / "tools/workspace_python.py"), "-m",
        "scripts.e2e.train_temporal_route_recovery_v3_20261001", "--", "--help"],
        cwd=tmp_path, check=True, capture_output=True, text=True, timeout=30)
    assert "--recovery-conversion-sha256" in result.stdout and "--validate-only" in result.stdout


def test_new_scope_truthfully_counts18not9_and_retains_parent_failure():
    fields = trainer.scope_fields()
    assert fields["training_episode_count"] == 18
    assert fields["recovery_training_episode_count"] == fields["replay_training_episode_count"] == 9
    assert fields["holdout_episode_count"] == 2
    assert fields["parent_all_diagnostic_screens_passed"] is False and fields["parent_diagnostic_pass_count"] == 7
    assert fields["inference_loader_authorized"] is fields["model_promoted"] is False


def test_observed_branch_response_groups_compare_true_labels_only_offline():
    before = dict(rows=[dict(token="a", predicted_near_curvature_inv_m=.02)])
    after = dict(rows=[dict(token="a", true_near_curvature_inv_m=-.01,
                           predicted_near_curvature_inv_m=-.005, learned_near_branch_logit_mean=-.1)])
    audits = dict(a=dict(actual_measured_rear_base_signed_CTE_m=.2, actual_yaw_route_heading_error_rad=0.))
    result = trainer.branch_sign_response(before, after, audits)
    assert result["groups"]["(1, 0)"]["signed_response_agree_count"] == 1
    assert result["no_inference_oracle"] is True and result["feature_approved"] is False


def conversion(split="train"):
    profiles = sorted(trainer.TRAIN_PROFILES if split == "train" else trainer.HOLDOUT_PROFILES)
    count = 302 * len(profiles)
    return dict(full_cohort_fields(), schema=trainer.CONVERSION_SCHEMA,
        status="QUALIFIED_RESEARCH_TRAIN_COMMON10" if split == "train" else "QUALIFIED_HOLDOUT_COMMON10_EVALUATION_ONLY",
        episode_count=len(profiles), train_examples=count if split == "train" else 0,
        val_examples=count if split == "val" else 0, test_examples=0, true_future_points=count*64,
        raw_native_denials_overridden=False, vehicle_control_approved=False, model_promoted=False,
        model_or_vehicle_release_approved=False, exact_native_ego_navigation_and_target_reconstruction=True,
        research_training_authorized=split == "train", research_holdout_evaluation_authorized=split == "val",
        holdout_training_or_checkpoint_selection_allowed=False, original_pending_metadata_unchanged=True,
        synthetic_history_created=False, labels_clipped=False, labels_or_pixels_transformed=False,
        future_labels_used_as_model_input=False, actual_hypot_current_speed_no_clipping=True,
        causal_navigation_support_not_future_distance=True,
        per_episode_native_qualification=[dict(case_id="case%d" % i, episode_id="ep%d" % i,
            selected_examples=302, split=split, signed_lateral_offset_ros_m=p[0], signed_yaw_offset_ros_deg=p[1])
            for i, p in enumerate(profiles)])


@pytest.mark.parametrize("split,count,episodes", [("train", 2718, 9), ("val", 604, 2)])
def test_whole_actual_v3_conversion_without_holdout_fit(split, count, episodes):
    actual_count, actual_episodes, cases = trainer.verify_conversion(conversion(split), split)
    assert (actual_count, actual_episodes, len(cases)) == (count, episodes, episodes)


@pytest.mark.parametrize("key,value", [("schema", "carla.actual_signed_recovery_common10_conversion.20261001.v1"),
    ("schema", "carla.actual_signed_recovery_common10_conversion.20261001.v2"),
    ("research_training_authorized", False), ("raw_native_denials_overridden", True), ("episode_count", 8),
    ("episode_count", 9.), ("true_future_points", 2718*63), ("labels_clipped", True),
    ("test_examples", 1), ("future_labels_used_as_model_input", True)])
def test_conversion_rejects_failed_v1_v2_denial_rewrites_or_missing_trueanchors(key, value):
    document = conversion(); document[key] = value
    with pytest.raises(ValueError): trainer.verify_conversion(document, "train")


def signed_fixture():
    example = SimpleNamespace(token="ep:7", episode_id="ep", anchor_timestamp_ns=123456789)
    case = dict(case_id="train_00", native_manifest_sha256="a"*64, signed_lateral_offset_ros_m=-.2, signed_yaw_offset_ros_deg=-1.5)
    audit = dict(token=example.token, sample_id=example.token, episode_id="ep", case_id="train_00", timestamp_ns=example.anchor_timestamp_ns,
        source_native_manifest_sha256="a"*64, diagnostic_only_not_model_input=True,
        declared_signed_lateral_offset_ros_m=-.2, declared_signed_yaw_offset_ros_deg=-1.5,
        actual_measured_rear_base_signed_CTE_m=-.18, actual_measured_rear_base_CTE_m=.18,
        actual_yaw_route_heading_error_rad=-.02)
    return example, case, audit


def test_signed_audit_binds_actual_negative_cte_instead_of_absolute_magnitude():
    example, case, audit = signed_fixture()
    audits = trainer.verify_signed_audits([audit], [example], {"ep": case})
    assert trainer.signed_bin(audits[example.token][trainer.SIGNED_CTE_KEY], .05) == -1


@pytest.mark.parametrize("key,value", [("timestamp_ns", 0), ("token", "holdout:0"), ("source_native_manifest_sha256", "b"*64),
    ("diagnostic_only_not_model_input", False), ("actual_measured_rear_base_CTE_m", -.18),
    ("declared_signed_yaw_offset_ros_deg", -3.), ("actual_measured_rear_base_signed_CTE_m", True)])
def test_signed_current_state_rejects_rewritten_foreign_or_unsigned_evidence(key, value):
    example, case, audit = signed_fixture(); audit[key] = value
    with pytest.raises(ValueError): trainer.verify_signed_audits([audit], [example], {"ep": case})


def test_every_domain_gradient_checked_without_autograd_on_frozen_backbone():
    frozen = torch.nn.Parameter(torch.tensor(2.), requires_grad=False)
    learned = torch.nn.Parameter(torch.tensor(1.))
    model = SimpleNamespace(parameters=lambda: iter((frozen, learned)))
    result = trainer.finite_branch_gradient(learned.square() * frozen, model)
    assert result["gradient_l2_norm"] == 4. and frozen.grad is None


def endpoint_fixture():
    result = {}
    for name, episodes, total in (("recovery_train", 9, 2718), ("recovery_holdout", 2, 604), ("known599", 2, 599), ("legacy", 8, 2487)):
        rows = [dict(index=i, token=name+":"+str(i)) for i in range(total)]
        if name.startswith("recovery_"):
            result[name] = dict(full_fixed_denominator=total, rows=rows,
                per_episode={str(e): dict(ade_6p4s_m=.08, fde_6p4s_m=.2, instantaneous_speed_mae_mps=.04) for e in range(episodes)})
        else:
            result[name] = dict(full_fixed_eligible_denominator=total, rows=rows,
                groups=dict(all_eligible=dict(raw_curvature=.02)),
                per_episode={str(e): dict(raw_curvature=.02) for e in range(episodes)})
    result["v4"] = dict(full_fixed_denominator=327, rows=[dict(token="V4:"+str(i)) for i in range(327)],
        mean_ade_6p4s_m=.1043, mean_fde_6p4s_m=.2575, mean_instantaneous_speed_mae_mps=.024)
    return result


def test_exact48_fixed_endpoint_screens_keep_all11cases_and_parent_frozen_speed_bias():
    before = endpoint_fixture(); after = copy.deepcopy(before)
    result = trainer.endpoint_screens(before, after)
    assert result["diagnostic_screen_count"] == 48 and result["all_diagnostic_screens_passed"] is True
    assert result["absolute_speed_tracking_claimed"] is False and result["model_promoted"] is False
    after["recovery_holdout"]["per_episode"]["0"]["ade_6p4s_m"] = .1001
    result = trainer.endpoint_screens(before, after)
    assert sum(not r["passed"] for r in result["checks"]) == 1


def test_missing_holdout_or_fixed_token_change_cannot_reduce_denominator_after_results():
    before = endpoint_fixture(); after = copy.deepcopy(before)
    after["recovery_holdout"]["rows"][0]["token"] = "different"
    with pytest.raises(ValueError, match="denominator"): trainer.endpoint_screens(before, after)


def qualification():
    return dict(full_cohort_fields(), schema=trainer.QUALIFICATION_SCHEMA, run_id="v3", group="train", status="QUALIFIED_RESEARCH_TRAIN",
        research_training_authorized=True, research_holdout_evaluation_authorized=False,
        holdout_training_or_checkpoint_selection_allowed=False, raw_native_denials_overridden=False,
        original_pending_metadata_unchanged=True, window_filtering_performed=False, thresholds_relaxed=False,
        labels_or_pixels_transformed=False, synthetic_history_created=False, training_run_performed=False,
        vehicle_control_approved=False, model_promoted=False, model_or_vehicle_release_approved=False,
        episode_count=9, selected_camera_anchors=2718,
        cases=[dict(all_fixed_anchor_necessary_checks_clear=True, selected_camera_anchors=302) for _ in range(9)])


def test_copied_qualification_cannot_convert_protected_native_failures_into_approval():
    receipt = qualification(); trainer.verify_qualification(receipt, "train", 2718, 9)
    receipt["cases"][4]["all_fixed_anchor_necessary_checks_clear"] = False
    with pytest.raises(ValueError): trainer.verify_qualification(receipt, "train", 2718, 9)


@pytest.mark.parametrize("key,value", [("schema", "old-v1"), ("run_id", "v1"), ("run_id", "v2"), ("thresholds_relaxed", True),
    ("window_filtering_performed", True), ("research_holdout_evaluation_authorized", True), ("selected_camera_anchors", 2717)])
def test_original_receipt_scope_whole_group_and_flags_are_immutable(key, value):
    receipt = qualification(); receipt[key] = value
    with pytest.raises(ValueError): trainer.verify_qualification(receipt, "train", 2718, 9)


def full_cohort_fields():
    rows = [dict(case_id="train_%02d" % i, dataset_split_intent="train", seed=2026100510+i,
        signed_lateral_offset_ros_m=profile[0], signed_yaw_offset_ros_deg=profile[1])
        for i, profile in enumerate(sorted(trainer.TRAIN_PROFILES))]
    rows += [dict(case_id="holdout_00", dataset_split_intent="dev", seed=2026100610,
        signed_lateral_offset_ros_m=-.15, signed_yaw_offset_ros_deg=1.),
        dict(case_id="holdout_01", dataset_split_intent="dev", seed=2026100611,
        signed_lateral_offset_ros_m=.15, signed_yaw_offset_ros_deg=-1.)]
    for row in rows:
        row.update(native_episode="/synthetic/"+row["case_id"], selected_camera_anchors=302,
            selected_span_ns=30_100_000_000, all_fixed_anchor_necessary_checks_clear=True,
            **{k: "a"*64 for k in ("case_plan_sha256", "native_manifest_sha256", "native_state_sha256", "native_camera_sha256", "source_route_sha256")})
    return dict(all_eleven_fixed_cohort_cases_qualified=True, full_cohort_case_count=11,
        full_cohort_fixed_selected_anchors=3322, full_cohort_case_provenance=rows)


@pytest.mark.parametrize("change", ["missing_holdout", "wrong_seed", "reused_native", "failed_case", "wrong_split"])
def test_full_cohort_requires_all11_qualified_actual_sources_not_a_successful_subset(change):
    proof = full_cohort_fields(); trainer.verify_full_cohort(proof)
    row = proof["full_cohort_case_provenance"][-1]
    if change == "missing_holdout": proof["full_cohort_case_provenance"].pop()
    if change == "wrong_seed": row["seed"] = 2026100411
    if change == "reused_native": row["native_episode"] = proof["full_cohort_case_provenance"][0]["native_episode"]
    if change == "failed_case": row["all_fixed_anchor_necessary_checks_clear"] = False
    if change == "wrong_split": row["dataset_split_intent"] = "train"
    with pytest.raises(ValueError): trainer.verify_full_cohort(proof)


def test_fresh_zero_output_branch_uses_new_fixed_lr_not_pretrained_joint_lr():
    rates = [trainer.branch_learning_rate(i) for i in range(1, 3001)]
    assert rates[0] == pytest.approx(3e-6) and rates[99] == 3e-4 and rates[-1] == 3e-5
    assert all(3e-5 <= rate <= 3e-4 for rate in rates[99:])
    assert sum(rates) == pytest.approx(.493515)
    assert trainer.learning_rate_config()["inherited_joint_schedule_reused"] is False
    with pytest.raises(ValueError): trainer.branch_learning_rate(True)
