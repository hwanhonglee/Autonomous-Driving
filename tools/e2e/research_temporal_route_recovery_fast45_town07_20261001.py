"""HH_261001 - Prospectively test the fixed V3 recovery endpoint in three owned, unchanged-safety 45-second trials."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import re

from PIL import Image

from portable_e2e.temporal_route_recovery_v3_20261001 import MODEL_ID, EXPORT_SCHEMA
from . import research_temporal_v2_headroom_fast45_town07_20261001 as previous
from . import collect_temporal_route_recovery_fast45_20261001 as live


ROOT = previous.ROOT
RUN = ROOT/"storage/experiments/training/2026-10-01/temporal_route_recovery_v3_v1"
WEIGHTS, METADATA = RUN/"exports/step_003000/model_state.npz", RUN/"exports/step_003000/metadata.json"
PARENT = ROOT/"storage/experiments/training/2026-10-01/temporal_v2_balanced_replay_v1/exports/step_003000"
MODULE = "scripts.e2e.research_temporal_route_recovery_fast45_town07_20261001"
SCHEMA = "portable_e2e.temporal_route_recovery_fast45_three_trials.20261001.v1"
require, digest, read_json = previous.require, previous.digest, previous.read_json
_fixed, _diagnostic, _runtime = previous.fixed_contract, previous.diagnostic_report, previous.runtime_sources
_active = False
_exports = None
_owner_plan_sha = None
_frozen_worker = previous.base.worker


def batch_paths(run_id):
    require(isinstance(run_id, str) and re.fullmatch(r"v[1-9][0-9]*", run_id), "explicit safe recovery batch version required")
    output = ROOT/("storage/experiments/validation/2026-10-01/temporal_route_recovery_fast45_town07_"+run_id)
    return output, output.with_name(output.name+"_plan.json")


def fixed_contract(run_id, longitudinal_pi):
    require(_exports is not None, "completed fixed recovery export required before a physical declaration")
    result = _fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, model_architecture_id=MODEL_ID,
        parent_checkpoint_updates=3200, parent_raw_updates=200, parent_mixed_updates=3000,
        optimizer_updates=3000, route_recovery_branch_optimizer_updates=3000,
        known_total_optimizer_updates=6200, backbone_optimizer_updates=3200,
        one_episode_specialized_research=False, training_episode_count=18,
        actual_signed_recovery_training_episode_count=9, replay_training_episode_count=9,
        reserved_recovery_holdout_episode_count=2,
        independent_map_validation=False, heldout_initialization_seeds_not_independent_map=True,
        original_NN_decoder_selected_trajectory_math_unchanged=False,
        original_backbone_weights_byte_unchanged=True, learned_route_logit_residual=True,
        original_decoder_gate_selected_tracker_math_unchanged_except_disclosed_PI_headroom=True,
        runtime_route_as_output_replacement=False, oracle_future_inputs=False,
        CPU_finite_guard_revision="same dual full validation; strict V3 route and permanent-hook guards",
        single_copy_normalized_history=True, complete_source_and_normalized_input_SHA_preserved=True,
        actual_JPEG_recording_after_ACK_before_next_tick=True,
        JPEG_recording_latency_removed_from_control_before_actual_ACK=True,
        camera_bundle_delivery_start_boundary_unchanged=True,
        all2700_JPEG_required_before_driving_screen=True,
        parent_numeric_export_sha256=dict(weights=digest(PARENT/"model_state.npz"), metadata=digest(PARENT/"metadata.json")),
        new_fixed_endpoint_export_sha256=dict(weights=_exports["weights_sha256"], metadata=_exports["metadata_sha256"]))
    return result


def runtime_sources():
    pins = _runtime()
    sources = previous.base.imported_source_closure((Path(__file__).resolve(),))
    pins.update({str(path.relative_to(ROOT)): digest(path) for path in sorted(sources)})
    return pins


def load_model(weights, metadata, *, weights_sha256, metadata_sha256):
    from portable_e2e.temporal_route_recovery_export_loader_research_20261001 import load_route_recovery_research_npz
    require(_exports is not None and Path(weights) == WEIGHTS and Path(metadata) == METADATA
        and weights_sha256 == _exports["weights_sha256"] and metadata_sha256 == _exports["metadata_sha256"],
        "declared fixed final3000 V3 export required")
    description = read_json(metadata)
    actuation = _owner_plan_sha is not None
    binding = dict(schema="portable_e2e.temporal_route_recovery_owner_binding.20261001.v1",
        mode="RESEARCH_ONLY", purpose="owned_carla_research_only" if actuation else "offline_cpu_numeric_validation_only", owner_id=MODULE,
        weights_sha256=weights_sha256, metadata_sha256=metadata_sha256,
        checkpoint_sha256=description["checkpoint_sha256"], declaration_sha256=description["declaration_sha256"],
        fixed_endpoint_updates=3000, simulator_control_authorized=actuation, vehicle_control_approved=False,
        model_promoted=False, port_triplet=[2440,2441,2442], maximum_speed_mps=2.5,
        hard_policy_deadline_seconds=.1, inherited_private_owner_required=True, simulation_only=True,
        owner_plan_sha256=_owner_plan_sha)
    return load_route_recovery_research_npz(weights, metadata, weights_sha256=weights_sha256,
        metadata_sha256=metadata_sha256, parent_weights=PARENT/"model_state.npz",
        parent_metadata=PARENT/"metadata.json", research_owner_binding=binding)


def private_worker(run_id, expected):
    """HH_261001 - Numeric preflight cannot authorize actuation; require the actual private parent/PID/socket first."""
    global _owner_plan_sha
    require(_active and _exports is not None and _owner_plan_sha is None, "exclusive bound recovery worker required")
    plan = previous.base.verify_plan(run_id,expected)
    previous.base.assert_owned_worker(plan,expected)
    _owner_plan_sha = expected
    try:
        return _frozen_worker(run_id,expected)
    finally:
        _owner_plan_sha = None


def recording_report(output):
    episode = output/"episode.partial"
    if not episode.is_dir(): episode = output/"episode"
    camera = previous.jsonl(episode/"camera_frames.jsonl")
    recordings = previous.jsonl(episode/"recording_after_ack.jsonl")
    timings = previous.jsonl(episode/"wall_timing.jsonl")
    pins = {str(episode/name):digest(episode/name) for name in ("camera_frames.jsonl", "recording_after_ack.jsonl", "wall_timing.jsonl")}
    require(len(camera) == len(recordings) == 450, "all450 camera bundles and post-ACK recordings required")
    by_frame = {row["frame"]:row for row in recordings}
    by_sequence = {row["ACK_sequence"]:row for row in timings}
    require(len(by_frame) == 450 and len(by_sequence) == len(timings), "duplicate camera or ACK recording identity")
    image_count = 0
    for row in camera:
        recorded = by_frame[row["frame"]]
        require(recorded["enqueued_camera_count"] == recorded["written_camera_count"] == 6
            and recorded["all_queued_images_written"] is True and recorded["failure"] is None
            and recorded["acknowledged_before_recording"] is True,
            "incomplete or failed post-ACK camera evidence")
        sequence = recorded["ACK_receipt_sequence"]
        require(sequence in by_sequence, "recording lacks original timed RPC receipt")
        ack = by_sequence[sequence]
        require(ack["ACK_server_accepted"] is True and ack["command_frame"] == row["frame"]
            and ack["ACK_finished_monotonic_ns"] <= recorded["recording_started_monotonic_ns"]
            <= recorded["recording_finished_monotonic_ns"], "actual ACK must precede unchanged recording")
        # HH_261001 - The original recorder names partial paths before atomic successful rename; preserve both literal receipts and hashes.
        relatives = [Path(row["images"][name]) for name in live.frozen.collector.MODEL_CAMERA_ORDER]
        require(all(not path.is_absolute() and ".." not in path.parts and len(path.parts) == 3
            and path.parts[0] == "images" and path.parts[1] == name and path.suffix == ".jpg"
            for name,path in zip(live.frozen.collector.MODEL_CAMERA_ORDER,relatives)), "recording relative image path escaped camera layout")
        expected = [episode/path for path in relatives]
        partial_literals = [str(output/"episode.partial"/path) for path in relatives]
        final_literals = [str(output/"episode"/path) for path in relatives]
        require(recorded["written_files"] in (partial_literals,final_literals),
                "recording literal paths must retain exact relative order under declared owned partial/final aliases")
        for path in expected:
            require(path.absolute() == path.resolve() and not any(parent.is_symlink() for parent in (path,*path.parents)),
                    "recorded actual JPEG must remain canonical inside the owned episode")
            pins[str(path)] = digest(path)
            with Image.open(path) as picture:
                require(picture.format == "JPEG" and picture.size == (640,360), "actual JPEG ABI differs")
                picture.load()
            image_count += 1
    require(all(digest(name) == sha for name,sha in pins.items()), "camera/ACK/recording sources changed during complete audit")
    return dict(status="PASS", bundle_count=450, image_count=image_count,
        actual_ACK_precedes_each_complete_six_camera_recording=True, source_sha256=pins,
        partial_to_final_rename_alias_checked_without_rewriting_receipts=True,
        measurement_start_boundary_moved=False, sensor_callback_or_exposure_latency_proven=False)


def diagnostic_report(output, plan, owner):
    result = _diagnostic(output, plan, owner)
    recording = dict(status="NOT_PROVEN")
    if result.get("ownership_passed") and result.get("bounded_capture_complete"):
        try:
            recording = recording_report(output)
            previous.verify_derived_source_closure(output,plan)
        except Exception as error:
            recording = dict(status="FAIL", error=type(error).__name__+": "+str(error))
    result.update(schema=SCHEMA+".diagnostic_report", model_architecture_id=MODEL_ID,
        original_NN_decoder_selected_trajectory_math_unchanged=False,
        original_backbone_weights_byte_unchanged=True, learned_route_logit_residual=True,
        runtime_route_as_output_replacement=False, complete_post_ACK_recording=recording)
    # HH_261001 - Preserve an incomplete parent diagnostic as failure instead of raising a second missing-field exception.
    result["learned_driving_screen_passed"] = bool(result.get("learned_driving_screen_passed",False)) and recording["status"] == "PASS"
    result["predeclared_observed_bundle_to_ACK_wall_latency_screen_passed"] = bool(result.get("predeclared_observed_bundle_to_ACK_wall_latency_screen_passed",False)) and recording["status"] == "PASS"
    result["status"] = "PASS_BASIC_DRIVING_PRECURSOR" if result["learned_driving_screen_passed"] else "FAIL_BASIC_DRIVING_PRECURSOR"
    return result


@contextmanager
def bound_fixed_recovery_exports():
    """HH_261001 - Bind only private versioned worker globals; all frozen modules are restored after every exit."""
    global _active, _exports
    require(not _active, "recovery export owner scopes cannot nest")
    description = read_json(METADATA)
    require(description["schema"] == EXPORT_SCHEMA and description["optimizer_updates"] == 3000,
        "completed fixed V3 final3000 metadata required")
    weights_sha, metadata_sha = digest(WEIGHTS), digest(METADATA)
    require(description["npz_sha256"] == weights_sha, "V3 weights differ from completed metadata")
    assets = {path:sha for path,sha in previous.base.ASSETS.items()
        if path not in (previous.base.WEIGHTS, previous.base.METADATA)}
    assets.update({WEIGHTS:weights_sha, METADATA:metadata_sha,
        PARENT/"model_state.npz":digest(PARENT/"model_state.npz"), PARENT/"metadata.json":digest(PARENT/"metadata.json")})
    base_changes = dict(WEIGHTS=WEIGHTS, METADATA=METADATA, WEIGHTS_SHA=weights_sha, METADATA_SHA=metadata_sha,
        CHECKPOINT_SHA=description["checkpoint_sha256"], ASSETS=assets, worker=private_worker)
    parent_changes = dict(batch_paths=batch_paths, fixed_contract=fixed_contract, runtime_sources=runtime_sources,
        load_model=load_model, live=live, MODULE=MODULE, SCHEMA=SCHEMA, diagnostic_report=diagnostic_report,
        CHECKPOINT_SHA=description["checkpoint_sha256"], ASSETS=assets)
    saved_base = {name:getattr(previous.base,name) for name in base_changes}
    saved_parent = {name:getattr(previous,name) for name in parent_changes}
    _active = True; _exports = dict(weights_sha256=weights_sha, metadata_sha256=metadata_sha)
    try:
        for name,value in base_changes.items(): setattr(previous.base,name,value)
        for name,value in parent_changes.items(): setattr(previous,name,value)
        yield
    finally:
        for name,value in saved_parent.items(): setattr(previous,name,value)
        for name,value in saved_base.items(): setattr(previous.base,name,value)
        _active = False; _exports = None


def main(argv=None):
    with bound_fixed_recovery_exports():
        return previous.main(argv)


if __name__ == "__main__": raise SystemExit(main())
