"""HH_261001 - Admit only prospectively fixed rolling V4 recovery without modifying frozen V3 learning.

This explicit NEW data-admission revision keeps the V3 neural architecture,
numeric export, frozen97/learned4 tensors, objective, B4 schedule, fixed3000
endpoint, learning rate and48 screens unchanged. The NEW signed-recovery
cohort starts at4.0s and retains302 anchors through34.1s, with actual history
from3.1s and6.4s true future. Raw startup remains outside this NEW cohort;
this is not evidence of learning or approving startup recovery.
"""
from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path

# HH_261001 - Spawn workers bootstrap only the canonical source namespace; no worker owns CUDA.
if __name__ == "__mp_main__":
    import importlib.util
    _entry = Path(__file__).resolve().parents[1] / "workspace_python.py"
    _spec = importlib.util.spec_from_file_location("hh_moving_recovery_worker_workspace", _entry)
    _module = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_module)
    _module.configure(Path(__file__).resolve().parents[2])

from scripts.e2e import train_temporal_route_recovery_v3_20261001 as previous

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.temporal_route_recovery_moving_training.20261001.v2"
CONVERSION_SCHEMA = "carla.actual_signed_recovery_common10_conversion.20261001.v4"
QUALIFICATION_SCHEMA = "carla.actual_signed_recovery_common10_qualification.20261001.v4"
TRAIN_BASENAME = "town07-signed-recovery-train-20261001-v4"
HOLDOUT_BASENAME = "town07-signed-recovery-holdout-20261001-v4"
FROZEN_V3_SOURCES = {
    "tools/e2e/train_temporal_route_recovery_v3_20261001.py": "6e1909f5574cf1449f56c4e0379507369f9a0f8841d05160a8e2cb861985452b",
    "learning/portable_e2e/temporal_route_recovery_v3_20261001.py": "41494aeb6175c7e8bb054b0c099b121e1897144681a6e3591919bc0740925a25",
    "learning/portable_e2e/temporal_route_recovery_export_loader_research_20261001.py": "d9f1cb77adec368a48145ed522eef8508a40906c1d327b68d9f8fe03443a7670",
}
_source_snapshot = previous.source_snapshot
_verify_conversion = previous.verify_conversion
_scope_fields = previous.scope_fields
_active = False
_missing_alias = object()
require, digest = previous.require, previous.digest


def verify_frozen_sources():
    require(all(digest(ROOT / name) == sha for name, sha in FROZEN_V3_SOURCES.items()),
            "frozen V3 trainer/model/loader must remain byte-identical")


def source_snapshot():
    # HH_261001 - Pin only the exact imported24-file closure, never unrelated live modules or a directory glob.
    verify_frozen_sources()
    original = _source_snapshot()
    seeds = [Path(__file__).resolve(), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"]
    paths = set(previous.replay.imported_source_closure(ROOT, seeds))
    paths.add(ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json")
    sources = {str(path.relative_to(ROOT)): digest(path) for path in sorted(paths)}
    require(all(sources.get(name) == sha for name, sha in original.items())
            and all(sources.get(name) == sha for name, sha in FROZEN_V3_SOURCES.items()),
            "all original23 imported learning sources and original17 training pins must remain unchanged")
    return sources


def moving_scope_fields():
    # HH_261001 - This declaration describes only NEW signed-recovery membership; unchanged older center/cruise replay still participates.
    return dict(_scope_fields(), recovery_data_revision="prospective_actual_signed_recovery_moving_v4",
        new_signed_recovery_cohort_moving_only=True, startup_recovery_fitted=False,
        startup_recovery_scope="NEW signed-recovery cohort only; previous center/cruise and old TRAIN replay unchanged",
        raw_startup_retained_outside_new_signed_recovery_training=True,
        fixed_recovery_driving_relative_window_seconds=[4.0, 34.1],
        fixed_recovery_native_frame_offsets=[80, 682],
        earliest_full_raw_history_driving_relative_seconds=3.1,
        true_future_seconds=6.4, fixed_recovery_anchors_per_episode=302,
        startup_or_route_goal_capability_claimed=False)


def verify_moving_membership(document):
    # HH_261001 - Fixed rolling membership is prospectively declared, not selected after inspecting model losses or failed windows.
    require(document.get("rolling_recovery_only") is True and document.get("startup_anchors_used_for_training") is False
            and document.get("raw_startup_retained_as_context_only") is True,
            "V4 admits only new rolling recovery while preserving raw startup outside training")
    offsets, seconds = document.get("fixed_driving_relative_anchor_offsets"), document.get("fixed_anchor_driving_seconds")
    require(isinstance(offsets, list) and all(type(value) is int for value in offsets) and offsets == [80, 682]
            and isinstance(seconds, list) and all(type(value) is float for value in seconds) and seconds == [4.0, 34.1],
            "exact prospective4.0..34.1s/native80..682 rolling anchor membership required")


def verify_full_cohort(document):
    # HH_261001 - Fresh V4 seeds, not renamed failed V3 cases, are required for every exact11-case receipt.
    require(document.get("all_eleven_fixed_cohort_cases_qualified") is True
            and type(document.get("full_cohort_case_count")) is int and document["full_cohort_case_count"] == 11
            and type(document.get("full_cohort_fixed_selected_anchors")) is int and document["full_cohort_fixed_selected_anchors"] == 3322,
            "all11 actual V4 cases and all3322 prospective fixed windows must qualify")
    rows = document.get("full_cohort_case_provenance")
    expected = {"train_%02d" % i: ("train", 2026100710+i, *profile) for i, profile in enumerate(sorted(previous.TRAIN_PROFILES))}
    expected.update(holdout_00=("dev", 2026100810, -.15, 1.), holdout_01=("dev", 2026100811, .15, -1.))
    require(isinstance(rows, list) and len(rows) == 11 and {r["case_id"] for r in rows} == set(expected)
            and len({r.get("native_episode") for r in rows}) == 11
            and all(isinstance(r.get("native_episode"), str) and r["native_episode"] for r in rows),
            "all11 distinct prospective V4 native case identities required")
    for row in rows:
        require((row.get("dataset_split_intent"), row.get("seed"), row.get("signed_lateral_offset_ros_m"), row.get("signed_yaw_offset_ros_deg"))
                    == expected[row["case_id"]]
                and type(row.get("seed")) is int and type(row.get("selected_camera_anchors")) is int and row["selected_camera_anchors"] == 302
                and type(row.get("selected_span_ns")) is int and row["selected_span_ns"] >= 30_000_000_000
                and row.get("all_fixed_anchor_necessary_checks_clear") is True,
                "actual V4 profile/seed/split/span/whole-window proof differs")
        require(all(isinstance(row.get(k), str) and len(row[k]) == 64 and all(c in "0123456789abcdef" for c in row[k]) for k in
                    ("case_plan_sha256", "native_manifest_sha256", "native_state_sha256", "native_camera_sha256", "source_route_sha256")),
                "all11 exact V4 native provenance hash pins required")
    return {r["case_id"]: r for r in rows}


def verify_conversion(document, split):
    # HH_261001 - The inherited verifier reads the NEW scoped schema and full-cohort verifier; no receipt field is rewritten.
    require(_active, "V4 admission requires the explicit restored trainer scope")
    result = _verify_conversion(document, split)
    verify_moving_membership(document)
    return result


def verify_qualification(qualification, split, count, episodes):
    # HH_261001 - Preserve every frozen guard, replacing only the honestly declared data version/run-id.
    flags = dict(research_training_authorized=split == "train", research_holdout_evaluation_authorized=split == "val",
        holdout_training_or_checkpoint_selection_allowed=False, raw_native_denials_overridden=False,
        original_pending_metadata_unchanged=True, window_filtering_performed=False, thresholds_relaxed=False,
        labels_or_pixels_transformed=False, synthetic_history_created=False, training_run_performed=False,
        vehicle_control_approved=False, model_promoted=False, model_or_vehicle_release_approved=False)
    require(qualification.get("schema") == QUALIFICATION_SCHEMA and qualification.get("run_id") == "v4"
            and qualification.get("group") == ("train" if split == "train" else "holdout")
            and qualification.get("status") == ("QUALIFIED_RESEARCH_TRAIN" if split == "train" else "QUALIFIED_RESERVED_HOLDOUT_EVALUATION_ONLY")
            and all(qualification.get(k) is value for k, value in flags.items())
            and type(qualification.get("episode_count")) is int and qualification["episode_count"] == episodes
            and type(qualification.get("selected_camera_anchors")) is int and qualification["selected_camera_anchors"] == count
            and len(qualification.get("cases", [])) == episodes
            and all(row.get("all_fixed_anchor_necessary_checks_clear") is True and row.get("selected_camera_anchors") == 302
                    for row in qualification["cases"]), "authenticated whole actual V4 recovery qualification scope differs")
    verify_full_cohort(qualification)
    verify_moving_membership(qualification)


@contextmanager
def activated():
    """HH_261001 - Restore original globals after success or exception; no source bytes, receipt contents or model output edits."""
    global _active
    require(not _active, "V4 trainer admission activation cannot nest")
    verify_frozen_sources()
    # HH_261001 - Prospective pre-fit API revision calls the frozen load_cohorts function without editing any original23 source.
    original_alias = getattr(previous.replay, "load_all", _missing_alias)
    require(original_alias is _missing_alias or original_alias is previous.replay.load_cohorts,
            "unknown existing replay.load_all alias cannot be replaced")
    changes = dict(SCHEMA=SCHEMA, CONVERSION_SCHEMA=CONVERSION_SCHEMA, QUALIFICATION_SCHEMA=QUALIFICATION_SCHEMA,
        TRAIN_BASENAME=TRAIN_BASENAME, HOLDOUT_BASENAME=HOLDOUT_BASENAME, source_snapshot=source_snapshot,
        verify_full_cohort=verify_full_cohort, verify_conversion=verify_conversion,
        verify_qualification=verify_qualification, scope_fields=moving_scope_fields)
    original = {name: getattr(previous, name) for name in changes}
    _active = True
    try:
        previous.replay.load_all = previous.replay.load_cohorts
        for name, value in changes.items(): setattr(previous, name, value)
        yield
    finally:
        for name, value in original.items(): setattr(previous, name, value)
        if original_alias is _missing_alias: delattr(previous.replay, "load_all")
        else: previous.replay.load_all = original_alias
        _active = False


def main(argv=None):
    with activated(): return previous.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
