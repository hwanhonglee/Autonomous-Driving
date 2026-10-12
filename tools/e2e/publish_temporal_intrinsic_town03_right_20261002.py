"""HH_261002 - Publish complete actual fixed-pair turn archives without inference, rerouting, or promotion."""
from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
import json
from pathlib import Path

from PIL import ImageDraw
from . import publish_temporal_route_joint_town03_right_20261002 as previous

ROOT = previous.ROOT
SCHEMA = "portable_e2e.actual_intrinsic_pair_town03_publication.20261002.v1"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
OWNER_SCHEMA = "portable_e2e.owned_intrinsic_pair_town03_known_dev_right.20261002.v1"
AUTHORITY_SCHEMA = "portable_e2e.temporal_intrinsic_curve_completed_pair_authority.20261002.v1"
PAIR_ROOT = ROOT/"storage/experiments/validation/2026-10-02/temporal_intrinsic_pair_town03_known_dev_right_v1"
OUTPUT = ROOT/"docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/17_intrinsic_curve_pair_actual_town03_right"
ARMS = ("control", "intrinsic_curve")
FROZEN_SHA = "ab179034c9bd3e73f0776245695e0edcaacb9e316d2c580102786862a73bf7d8"
_load, _render, _markdown, _sources = previous.load_archive, previous.render, previous.markdown, previous.publication_sources
_publish = previous.publish
_active, _loaded = False, None


def output_for(run):
    run = previous.canonical(run)
    previous.require(run.parent == PAIR_ROOT and run.name in ARMS, "exact fixed-pair arm archive required")
    return previous.canonical(OUTPUT/run.name)


def publication_sources():
    return previous.core.audit.source_closure(Path(__file__), ROOT/"tools/workspace_python.py", ROOT/"tools/canonical_workspace.py")


def archive_function():
    # HH_261002 - Change ONLY the obsolete basename-derived owner schema; inverse AST identity retains all archive guards.
    source = ROOT/"tools/e2e/publish_temporal_route_joint_town03_right_20261002.py"
    previous.require(previous.prior.digest(source) == FROZEN_SHA, "exact frozen complete-archive reader required")
    original = next(n for n in ast.parse(source.read_bytes()).body if isinstance(n, ast.FunctionDef) and n.name == "load_archive")
    changed = copy.deepcopy(original); count = 0
    for node in ast.walk(changed):
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "owner_schema":
            previous.require(ast.unparse(node.value) == "OWNER_SCHEMA[:-1] + run.name[-1]", "exact old schema anchor required")
            node.value = ast.Name(id="OWNER_SCHEMA", ctx=ast.Load()); count += 1
    previous.require(count == 1, "one owner-schema-only adapter required")
    restored = copy.deepcopy(changed)
    for node in ast.walk(restored):
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "owner_schema":
            node.value = ast.parse("OWNER_SCHEMA[:-1] + run.name[-1]", mode="eval").body
    previous.require(ast.dump(restored) == ast.dump(original), "another archive or observed-control check changed")
    namespace = dict(vars(previous))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])), __file__, "exec"), namespace)
    return namespace["load_archive"]


def verify_pair_and_recording(caller, bundle, pins):
    run = bundle["episode"].parent; plan = pins.json(run/"declaration.json")
    previous.require(caller.get("arm") == plan.get("arm") == run.name in ARMS
        and plan.get("paired_arm_order") == list(ARMS) and plan.get("both_pair_endpoints_completed_before_declaration") is True
        and plan.get("known_total_optimizer_updates") == 18200,
        "exact declared arm and completed6000 continuation lineage required")
    authority_path = previous.canonical(plan["completed_pair_authority"])
    expected = previous.sha(caller["completed_pair_authority_sha256"])
    previous.require(plan["completed_pair_authority_sha256"] == expected and pins.values.get(str(authority_path)) == expected,
        "root caller must bind the original completed-pair authority asset")
    authority = pins.json(authority_path); endpoint = authority["endpoints"][run.name]
    previous.require(authority.get("schema") == AUTHORITY_SCHEMA and authority.get("mode") == "RESEARCH_ONLY"
        and authority.get("both_fixed_endpoints_completed") is True and authority.get("arm_order") == list(ARMS)
        and authority.get("model_promoted") is False and authority.get("vehicle_control_approved") is False
        and plan["source_checkpoint_sha256"] == endpoint["checkpoint_sha256"]
        and plan["new_fixed_endpoint_export_sha256"] == dict(weights=endpoint["weights_sha256"], metadata=endpoint["metadata_sha256"]),
        "actual selected endpoint must match the externally sealed pair, not a prior parent")
    green = plan.get("owned_green_signal_fixture", {})
    expected_green = dict(original_own_world_traffic_actors="ALL", state="Green", frozen=True, ignore_traffic_lights=False,
        all_original_hazard_detectors_and_emergency_returns_unchanged=True, known_v1_hazard_subtype_proven=False,
        closed_course_basic_turn_only_not_traffic_rule_performance=True)
    previous.require(plan.get("actual_manual_first_gear_bootstrap_and_engagement_required") is True
        and all(type(green.get(k)) is type(v) and green[k] == v for k,v in expected_green.items()),
        "controlled-green basic-turn disclosure and all original hazards required")
    rows = pins.rows(bundle["episode"]/"recording_after_ack.jsonl")
    proof = pins.json(bundle["episode"]/"actual_recording_binding.json")
    providers = ("_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
        "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg")
    # HH_261002 - A real pre-send exception may preserve its final pending sources in the outer failure flush AFTER this witness.
    pending_failure = bool(rows) and bundle["diagnostic"]["original_capture_status"] == "failed" \
        and proof.get("pending_source_camera_count") == 6 and proof.get("actual_recording_bundle_count") == len(rows)-1 \
        and rows[-1].get("acknowledged_before_recording") is False and rows[-1].get("ACK_receipt_sequence") is None
    complete_recorder = proof.get("pending_source_camera_count") == 0 and proof.get("actual_recording_bundle_count") == len(rows)
    previous.require([r["frame"] for r in rows] == [c["frame"] for c in bundle["cameras"]]
        and proof.get("schema") == "portable_e2e.actual_intrinsic_pair_town03_known_dev_right.20261002.v1.actual_recording_binding"
        and proof.get("all_copied_mutable_providers_current") is True and proof.get("enqueue_and_ACK_share_exact_recorder") is True
        and tuple(proof.get("copied_provider_names", ())) == providers and proof.get("every_recording_event_preserved") is True
        and all(type(proof.get(k)) is int for k in ("actual_recording_bundle_count", "pending_source_camera_count", "actual_after_ACK_receipt_count"))
        and (complete_recorder or pending_failure),
        "full actual camera recorder provider proof required")
    receipts = {r["sequence"]: r for r in bundle["receipts"]}; acknowledged = 0
    for row, camera in zip(rows, bundle["cameras"]):
        expected_files = {str(previous.prior.safe_image(bundle["episode"], p)) for p in camera["images"].values()}
        previous.require(row.get("schema") == "portable_e2e.exact_camera_recording_after_ACK.20261001.v1"
            and row.get("enqueued_camera_count") == row.get("written_camera_count") == 6
            and set(row["written_files"]) == expected_files and len(row["written_files"]) == 6
            and row.get("all_queued_images_written") is True and row.get("failure") is None
            and row.get("no_source_image_dropped") is True and row.get("source_timestamp_changed") is False
            and row.get("source_pixels_changed") is False, "every original six-camera source must be preserved")
        if row.get("acknowledged_before_recording") is True:
            receipt = receipts.get(row["ACK_receipt_sequence"], {})
            previous.require(receipt.get("server_accepted") is True and receipt.get("status") == "ACKNOWLEDGED"
                and receipt.get("before_frame") == receipt.get("after_ack_frame") == row["frame"], "recording ACK must be actual same-frame receipt")
            acknowledged += 1
        else:
            previous.require(row.get("acknowledged_before_recording") is False and row.get("ACK_receipt_sequence") is None,
                "pre-ACK failure preservation cannot invent an accepted receipt")
    previous.require(proof.get("actual_after_ACK_receipt_count") == acknowledged, "measured recorder count differs")
    bundle.update(paired_arm=run.name, selected_endpoint=endpoint,
        actual_recording_census=dict(bundle_count=len(rows), after_ACK_count=acknowledged,
            all_recording_after_ACK=bool(rows) and acknowledged == len(rows),
            failed_pending_final_bundle_flushed_without_ACK=pending_failure, full_tick_10Hz_wall_rate_claimed=False))
    pins.recheck()


def load_archive(path, sha, *, decode=False):
    global _loaded
    caller, bundle, pins = archive_function()(path, sha, decode=decode)
    verify_pair_and_recording(caller, bundle, pins)
    _loaded = caller, bundle, pins
    return caller, bundle, pins


def render(bundle, index):
    out = _render(bundle, index); draw = ImageDraw.Draw(out)
    draw.rectangle((0, 0, 1699, 91), fill=previous.visual.BG)
    verdict, _ = previous.outcome_text(bundle)
    draw.text((24, 18), "Actual Town03 | " + bundle["paired_arm"] + " | " + verdict,
        fill=previous.visual.WHITE, font=previous.visual.font(24, True))
    draw.text((24, 63), "Parent12200 +6000 / NPZ " + bundle["selected_endpoint"]["weights_sha256"][:12]
        + " / CPU4 PI2.4 preview0.5 / complete recorded interval",
        fill=previous.visual.MUTED, font=previous.visual.font(18))
    return out


def markdown(bundle, pngs):
    # HH_261002 - Label the actual arm and measured recorder separately; do not equate recording order to whole sensor latency.
    return "# 동일 조건 모델 비교 — " + bundle["paired_arm"] + "\n\n" + _markdown(bundle, pngs) \
        + "\n실제 이미지 저장 확인: `" + str(bundle["actual_recording_census"]) \
        + "`. ACK 후 저장 계측은 센서 노출부터의 전체 10Hz 승인이 아닙니다.\n"


def publish(args):
    # HH_261002 - Append new measured evidence as a separate create-only companion; never rewrite frozen publisher output.
    result = _publish(args)
    if not args.validate_only:
        previous.require(_loaded is not None, "actual pinned bundle must precede measured companion")
        caller, bundle, pins = _loaded; pins.recheck()
        evidence = dict(schema=SCHEMA+".pair_and_recording_evidence", caller_sha256=args.caller_pins_sha256,
            arm=bundle["paired_arm"], endpoint=bundle["selected_endpoint"],
            completed_pair_authority_sha256=caller["completed_pair_authority_sha256"],
            actual_recording_census=bundle["actual_recording_census"],
            base_publication_provenance_sha256=previous.prior.digest(bundle["output"]/"publication_provenance.json"),
            full_tick_10Hz_wall_rate_claimed=False, model_promoted=False, vehicle_control_approved=False,
            official_main_features_approved=0, official_subfeatures_approved=0)
        with (bundle["output"]/"pair_and_recording_evidence.json").open("x") as stream:
            json.dump(evidence, stream, indent=2, allow_nan=False)
        pins.recheck()
    return result


@contextmanager
def activated():
    global _active, _loaded
    changes = dict(SCHEMA=SCHEMA, CALLER_SCHEMA=CALLER_SCHEMA, OWNER_SCHEMA=OWNER_SCHEMA,
        output_for=output_for, publication_sources=publication_sources, load_archive=load_archive, render=render, markdown=markdown,
        publish=publish)
    saved = {k:getattr(previous,k) for k in changes}
    previous.require(not _active and saved["load_archive"] is _load and saved["render"] is _render
        and saved["markdown"] is _markdown and saved["publication_sources"] is _sources and saved["publish"] is _publish
        and _loaded is None,
        "non-nested known publication providers required")
    _active = True
    try:
        for k,v in changes.items(): setattr(previous,k,v)
        yield
    finally:
        for k,v in saved.items(): setattr(previous,k,v)
        _active, _loaded = False, None


def main(argv=None):
    with activated(): return previous.main(argv)


if __name__ == "__main__": raise SystemExit(main())
