"""HH_261001 - Prove NEW rolling-only admission preserves frozen learning and never fits failed startup data."""
import copy
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from scripts.e2e import train_temporal_route_recovery_moving_v4_20261001 as revision
from scripts.e2e import train_temporal_route_recovery_v3_20261001 as previous


def full_proof():
    rows = [dict(case_id="train_%02d" % i, dataset_split_intent="train", seed=2026100710+i,
        signed_lateral_offset_ros_m=profile[0], signed_yaw_offset_ros_deg=profile[1])
        for i, profile in enumerate(sorted(previous.TRAIN_PROFILES))]
    rows += [dict(case_id="holdout_00", dataset_split_intent="dev", seed=2026100810,
        signed_lateral_offset_ros_m=-.15, signed_yaw_offset_ros_deg=1.),
        dict(case_id="holdout_01", dataset_split_intent="dev", seed=2026100811,
        signed_lateral_offset_ros_m=.15, signed_yaw_offset_ros_deg=-1.)]
    for row in rows:
        row.update(native_episode="/synthetic-v4/"+row["case_id"], selected_camera_anchors=302,
            selected_span_ns=30_100_000_000, all_fixed_anchor_necessary_checks_clear=True,
            **{key: "a"*64 for key in ("case_plan_sha256", "native_manifest_sha256", "native_state_sha256", "native_camera_sha256", "source_route_sha256")})
    return dict(all_eleven_fixed_cohort_cases_qualified=True, full_cohort_case_count=11,
        full_cohort_fixed_selected_anchors=3322, full_cohort_case_provenance=rows,
        rolling_recovery_only=True, startup_anchors_used_for_training=False, raw_startup_retained_as_context_only=True,
        fixed_driving_relative_anchor_offsets=[80, 682], fixed_anchor_driving_seconds=[4.0, 34.1])


def conversion(split="train"):
    profiles = sorted(previous.TRAIN_PROFILES if split == "train" else previous.HOLDOUT_PROFILES)
    count = 302 * len(profiles)
    return dict(full_proof(), schema=revision.CONVERSION_SCHEMA,
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


def qualification():
    return dict(full_proof(), schema=revision.QUALIFICATION_SCHEMA, run_id="v4", group="train", status="QUALIFIED_RESEARCH_TRAIN",
        research_training_authorized=True, research_holdout_evaluation_authorized=False,
        holdout_training_or_checkpoint_selection_allowed=False, raw_native_denials_overridden=False,
        original_pending_metadata_unchanged=True, window_filtering_performed=False, thresholds_relaxed=False,
        labels_or_pixels_transformed=False, synthetic_history_created=False, training_run_performed=False,
        vehicle_control_approved=False, model_promoted=False, model_or_vehicle_release_approved=False,
        episode_count=9, selected_camera_anchors=2718,
        cases=[dict(all_fixed_anchor_necessary_checks_clear=True, selected_camera_anchors=302) for _ in range(9)])


def test_frozen_original_source_bytes_and23plusone_recursive_closure():
    revision.verify_frozen_sources()
    sources = revision.source_snapshot()
    assert len(sources) == 24 and all(sources[k] == v for k, v in revision.FROZEN_V3_SOURCES.items())
    assert "tools/e2e/train_temporal_route_recovery_moving_v4_20261001.py" in sources
    assert not any("collect_" in name or "qualify_" in name for name in sources)


def test_restored_scope_and_export_truthful_moving_only_no_learning_changes():
    keys = ("SCHEMA", "CONVERSION_SCHEMA", "QUALIFICATION_SCHEMA", "TRAIN_BASENAME", "HOLDOUT_BASENAME",
            "source_snapshot", "verify_full_cohort", "verify_conversion", "verify_qualification", "scope_fields")
    before = {key: getattr(previous, key) for key in keys}
    rates = [previous.branch_learning_rate(i) for i in range(1, 3001)]
    objective = previous.exact_objective
    assert not hasattr(previous.replay, "load_all")
    with revision.activated():
        assert previous.TRAIN_BASENAME.endswith("-v4") and previous.SCHEMA == revision.SCHEMA
        assert previous.EXPORT_SCHEMA == "portable_e2e.temporal_route_recovery_export_research.20261001.v3"
        fields = previous.scope_fields()
        assert fields["startup_recovery_fitted"] is False and fields["new_signed_recovery_cohort_moving_only"] is True
        assert fields["fixed_recovery_native_frame_offsets"] == [80, 682]
        assert fields["model_promoted"] is fields["inference_loader_authorized"] is False
        assert previous.exact_objective is objective and rates == [previous.branch_learning_rate(i) for i in range(1, 3001)]
        assert previous.replay.load_all is previous.replay.load_cohorts
    assert all(getattr(previous, key) is value or getattr(previous, key) == value for key, value in before.items())
    assert not hasattr(previous.replay, "load_all")


def test_all_globals_restored_after_exception_and_nested_scope_rejected():
    before = previous.scope_fields
    with pytest.raises(RuntimeError, match="sentinel"):
        with revision.activated():
            with pytest.raises(ValueError, match="cannot nest"):
                with revision.activated(): pass
            raise RuntimeError("sentinel")
    assert previous.scope_fields is before and revision._active is False
    assert not hasattr(previous.replay, "load_all")


def test_prospective_v2_actual_main_reaches_the_existing_frozen_cohort_api_without_cuda(tmp_path, monkeypatch):
    # HH_261001 - Exercise the real main call site before any data fitting, rather than testing only --help/import.
    class ReachedFrozenCohortAPI(RuntimeError): pass
    called = []
    def exact_existing_api(*args):
        called.append(args)
        raise ReachedFrozenCohortAPI("actual main reached replay.load_cohorts")
    monkeypatch.setattr(previous.replay, "load_cohorts", exact_existing_api)
    monkeypatch.setattr(previous.replay, "storage_reserve", lambda *args: dict(test_only_no_write=True))
    monkeypatch.setattr(previous.torch, "set_num_threads", lambda *args: None)
    monkeypatch.setattr(previous, "WORKSPACE", tmp_path)
    args = []
    for name in ("recovery-train", "recovery-holdout", "recovery-conversion", "holdout-conversion", "parent-weights", "parent-metadata"):
        args += ["--"+name, str(tmp_path/name)]
    args += ["--output", str(tmp_path/"runs/training/fresh-only"), "--recovery-conversion-sha256", "a"*64,
             "--holdout-conversion-sha256", "b"*64, "--validate-only"]
    assert revision.SCHEMA.endswith(".v2") and not hasattr(previous.replay, "load_all")
    with pytest.raises(ReachedFrozenCohortAPI, match="actual main reached"):
        revision.main(args)
    assert called == [(previous.replay.LOW_PATH, previous.replay.LEGACY_PATH, previous.replay.DEV_PATH)]
    assert not hasattr(previous.replay, "load_all") and revision._active is False
    assert not (tmp_path/"runs/training/fresh-only").exists()
    revision.verify_frozen_sources()


def test_unknown_existing_compatibility_alias_fails_closed_without_overwrite(monkeypatch):
    sentinel = lambda *args: None
    monkeypatch.setattr(previous.replay, "load_all", sentinel, raising=False)
    with pytest.raises(ValueError, match="unknown existing"):
        with revision.activated(): pass
    assert previous.replay.load_all is sentinel and revision._active is False


def test_exact_known_existing_alias_is_restored_after_failure(monkeypatch):
    existing = previous.replay.load_cohorts
    monkeypatch.setattr(previous.replay, "load_all", existing, raising=False)
    with pytest.raises(RuntimeError, match="sentinel"):
        with revision.activated(): raise RuntimeError("sentinel")
    assert previous.replay.load_all is existing and revision._active is False


@pytest.mark.parametrize("split,count,episodes", [("train", 2718, 9), ("val", 604, 2)])
def test_only_full11_v4_membership_accepted_without_training(split, count, episodes):
    with revision.activated():
        n, e, records = previous.verify_conversion(conversion(split), split)
        assert (n, e, len(records)) == (count, episodes, episodes)
        if split == "train": previous.verify_qualification(qualification(), "train", 2718, 9)


@pytest.mark.parametrize("mutation", ["v3schema", "v3seed", "startup", "wrong_offsets", "wrong_seconds", "missing_holdout", "failed_case", "relaxed"])
def test_failed_v3_or_posthoc_membership_changes_are_not_authorized(mutation):
    document = conversion()
    if mutation == "v3schema": document["schema"] = "carla.actual_signed_recovery_common10_conversion.20261001.v3"
    if mutation == "v3seed": document["full_cohort_case_provenance"][0]["seed"] = 2026100510
    if mutation == "startup": document["startup_anchors_used_for_training"] = True
    if mutation == "wrong_offsets": document["fixed_driving_relative_anchor_offsets"] = [18, 620]
    if mutation == "wrong_seconds": document["fixed_anchor_driving_seconds"] = [4, 34.1]
    if mutation == "missing_holdout": document["full_cohort_case_provenance"].pop()
    if mutation == "failed_case": document["full_cohort_case_provenance"][3]["all_fixed_anchor_necessary_checks_clear"] = False
    if mutation == "relaxed": document["raw_native_denials_overridden"] = True
    with revision.activated(), pytest.raises(ValueError): previous.verify_conversion(document, "train")


def test_numeric_data_documents_are_never_rewritten_for_old_verifiers():
    document = conversion(); before = copy.deepcopy(document)
    with revision.activated(): previous.verify_conversion(document, "train")
    assert document == before and document["schema"] == revision.CONVERSION_SCHEMA
    assert previous.CONVERSION_SCHEMA.endswith(".v3")


def test_new24_source_only_fresh_clone_help_import_never_loads_data_or_gpu(tmp_path):
    for name in revision.source_snapshot():
        target = tmp_path/name; target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(revision.ROOT/name, target)
    result = subprocess.run([sys.executable, "-B", str(tmp_path/"tools/workspace_python.py"), "-m",
        "scripts.e2e.train_temporal_route_recovery_moving_v4_20261001", "--", "--help"],
        cwd=tmp_path, check=True, capture_output=True, text=True, timeout=30)
    assert "--validate-only" in result.stdout and "--recovery-train" in result.stdout


def test_admission_cannot_be_called_outside_explicit_new_revision_scope():
    with pytest.raises(ValueError, match="explicit restored"): revision.verify_conversion(conversion(), "train")
