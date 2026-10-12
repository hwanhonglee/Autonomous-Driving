"""HH_261002 - Seal CPU1/2/4 evidence under identical JPEG-derived causal inputs and unchanged live guards.

This is an OFFLINE computational probe. JPEG-derived BGRA is not the original
lossless CARLA pixel stream. Each fixed anchor has its own two real adjacent
past-frame prefills and zero initial PI integral, so deadline rejection cannot
change the next arm's input/control state. All rejected proposals and cold
prefills remain recorded; no actuator, teacher fallback, or deadline waiver is
provided. Telemetry wrappers call the exact frozen functions once and restore
them on every exit; their overhead remains inside the measured policy clock.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import functools
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from . import audit_temporal_route_joint_signed45_batch_20261002 as evidence


ROOT = evidence.ROOT
SCHEMA = "portable_e2e.temporal_route_joint_cpu_thread_evidence.20261002.v1"
MODULE = "scripts.e2e.profile_temporal_route_joint_cpu_threads_20261002"
THREAD_ARMS = (1, 2, 4)
MODEL_CAMERA_ORDER = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
OBSERVATION_FILES = ("manifest.json", "states.jsonl", "camera_frames.jsonl", "route.json", "learned_summary.json")
ASSET_LIMIT_BYTES = 512 * 1024**2
DISK_FLOOR_BYTES = 5 * 1024**3
_timing_active = False


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "regular non-symlink evidence required")
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            result.update(block)
    return result.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(), object_pairs_hook=unique_object)


def unique_object(pairs):
    result = {}
    for name, value in pairs:
        require(name not in result, "duplicate JSON member")
        result[name] = value
    return result


def write_new(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def source_path(value):
    path = Path(value)
    require(path.is_absolute() and path.is_file() and not path.is_symlink()
            and path.resolve(strict=True) == path and path.is_relative_to(ROOT),
            "exact canonical regular source inside this workspace required")
    return path


def camera_path(episode, value):
    relative = Path(value)
    require(isinstance(value, str) and value and not relative.is_absolute() and ".." not in relative.parts,
            "camera member must remain relative to its owned episode")
    path = source_path(episode / relative)
    require(path.is_relative_to(episode), "camera member escaped owned episode")
    return path


def fixed_ordinals(start, count):
    require(type(start) is int and type(count) is int and 11 <= start < 450
            and 1 <= count <= 128 and start + count <= 450,
            "predeclared integer camera start>=11 and count1..128 inside full450 required")
    return tuple(range(start, start + count))


def recorded_controller(summary):
    # HH_261002 - Live physics measured wheelbase/steer differ from the nominal rear-reference coordinate convention.
    fields = {"wheelbase_m","maximum_steer_angle_rad","speed_cap_mps","minimum_lookahead_m",
              "lookahead_time_s","speed_preview_s","throttle_gain","brake_gain","maximum_throttle"}
    value = summary.get("controller")
    require(isinstance(value,dict) and set(value)==fields
            and all(type(v) in (int,float) and math.isfinite(v) and v>0 for v in value.values())
            and value["speed_cap_mps"]==2.5 and value["maximum_steer_angle_rad"]<math.pi/2
            and value["maximum_throttle"]<=1 and value["speed_preview_s"]<=6.4,
            "all nine original actual controller fields and unchanged2.5 envelope required")
    return dict(value)


def observation_rows(episode):
    result = {}
    for name in ("states.jsonl", "camera_frames.jsonl"):
        values = [json.loads(line, object_pairs_hook=unique_object) for line in (episode / name).read_text().splitlines()]
        expected = 900 if name == "states.jsonl" else 450
        require(len(values) == expected and all(type(r.get("frame")) is int for r in values)
                and len({r["frame"] for r in values}) == expected, "all900 native/all450 camera rows required")
        require(all(values[i+1]["frame"] - values[i]["frame"] == (1 if expected == 900 else 2)
                    for i in range(expected-1)), "actual native/camera frame chain differs")
        require(all(type(r.get("timestamp")) in (int,float) and math.isfinite(r["timestamp"]) for r in values)
                and all(abs(round((values[i+1]["timestamp"]-values[i]["timestamp"])*1e9)
                            - (50_000_000 if expected == 900 else 100_000_000)) <= 1_000
                        for i in range(expected-1)), "actual native/camera timestamp cadence differs")
        result[name] = values
    by_frame = {row["frame"]: row for row in result["states.jsonl"]}
    image_members = set()
    for row in result["camera_frames.jsonl"]:
        require(row.get("camera_order") == list(MODEL_CAMERA_ORDER)
                and set(row.get("images", {})) == set(MODEL_CAMERA_ORDER)
                and set(row.get("source_timestamps", {})) == set(MODEL_CAMERA_ORDER)
                and row["frame"] in by_frame, "exact producer camera order and observed native frame required")
        require(len(set(row["images"].values())) == 6 and not image_members.intersection(row["images"].values()),
                "source JPEG members must be distinct for each actual camera/frame")
        image_members.update(row["images"].values())
        stamp = round(by_frame[row["frame"]]["timestamp"] * 1e9)
        require(all(round(row["source_timestamps"][name] * 1e9) == stamp for name in MODEL_CAMERA_ORDER),
                "actual camera exposure/state timestamps differ")
    return result["states.jsonl"], result["camera_frames.jsonl"]


def validate_input_spec(spec):
    require(isinstance(spec, dict) and spec.get("schema") == SCHEMA + ".input_pins", "exact offline input-pin schema required")
    anchors = fixed_ordinals(spec.get("start_camera_ordinal"), spec.get("measured_frames"))
    endpoint = evidence.endpoint_pins(spec.get("final_endpoint"))
    episode = Path(spec.get("episode", ""))
    require(episode.is_absolute() and episode.is_dir() and not episode.is_symlink()
            and episode.resolve(strict=True) == episode
            and episode.is_relative_to(ROOT / "storage/experiments/validation/2026-10-02")
            and episode.name in ("episode", "episode.partial"), "completed owned Oct2 episode only")
    require(set(spec.get("observation_sha256", {})) == set(OBSERVATION_FILES), "all five raw observation identities required")
    pins = {}
    for name, sha in spec["observation_sha256"].items():
        path = source_path(episode / name)
        require(evidence.previous.previous.sha(sha) and digest(path) == sha, "caller raw observation identity differs")
        pins[str(path)] = sha
    assets = {name: source_path(spec[name]) for name in ("weights", "metadata", "training_declaration",
        "parent_weights", "parent_metadata", "backbone_parent_weights", "backbone_parent_metadata")}
    identities = dict(weights=endpoint["weights_sha256"], metadata=endpoint["metadata_sha256"],
        training_declaration=endpoint["declaration_sha256"], parent_weights=evidence.PARENT_NPZ_SHA,
        parent_metadata=evidence.PARENT_METADATA_SHA, backbone_parent_weights=evidence.BACKBONE_NPZ_SHA,
        backbone_parent_metadata=evidence.BACKBONE_METADATA_SHA)
    for name, path in assets.items():
        require(digest(path) == identities[name], "exact final6000 or parent numeric asset differs")
        pins[str(path)] = identities[name]
    metadata = read_json(assets["metadata"])
    evidence.explicit_numeric_protocol()(metadata)
    require(metadata["checkpoint_sha256"] == endpoint["checkpoint_sha256"]
            and metadata["npz_sha256"] == endpoint["weights_sha256"]
            and metadata["declaration_sha256"] == endpoint["declaration_sha256"], "fixed6000 four-pin identity differs")
    states, cameras = observation_rows(episode)
    # HH_261002 - Same actual ten-frame history at every anchor and both adjacent prefill anchors, never duplicated fake history.
    observed_ordinals = tuple(range(anchors[0] - 11, anchors[-1] + 1))
    for index in observed_ordinals:
        for name in MODEL_CAMERA_ORDER:
            path = camera_path(episode, cameras[index]["images"][name])
            pins[str(path)] = digest(path)
    summary = read_json(episode / "learned_summary.json")
    controller = recorded_controller(summary)
    total = sum(Path(name).stat().st_size for name in pins)
    require(total <= ASSET_LIMIT_BYTES, "offline selected evidence exceeds declared512MiB cap")
    return dict(endpoint=endpoint, episode=episode, assets=assets, metadata=metadata, anchors=anchors,
        observed_ordinals=observed_ordinals, pins=pins, asset_bytes=total, controller=controller)


def assert_pins(pins):
    require(all(digest(name) == sha for name, sha in pins.items()), "declared source/input bytes changed")


def archive_pins(output, pins):
    archived = {}
    for name, sha in pins.items():
        source = source_path(name)
        target = output / "source" / source.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        with source.open("rb") as before, target.open("xb") as after:
            shutil.copyfileobj(before, after)
        require(digest(target) == sha, "archive does not match predeclared exact source bytes")
        archived[str(target)] = sha
    return archived


def declare(output, inputs, inputs_sha):
    require(digest(inputs) == inputs_sha, "caller input declaration changed")
    spec = read_json(inputs); data = validate_input_spec(spec)
    require(output.is_absolute() and output.resolve() == output and output.is_relative_to(ROOT / "storage/experiments/validation/2026-10-02")
            and not output.exists() and not output.is_symlink(), "fresh canonical Oct2 offline output required")
    require(shutil.disk_usage(ROOT).free >= DISK_FLOOR_BYTES + ASSET_LIMIT_BYTES, "offline archive storage reserve unavailable")
    sources = evidence.source_closure(Path(__file__), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py")
    sources.add(ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json")
    # HH_261002 - Exact immutable training-source pins supplement runtime imports; no directory-wide mutable-code glob.
    for name, sha in data["metadata"]["source_sha256"].items():
        path = source_path(ROOT / name)
        require(digest(path) == sha, "declared training source differs before the offline study")
        sources.add(path)
    pins = dict(data["pins"])
    pins[str(source_path(inputs))] = inputs_sha
    for path in sources:
        pins[str(path)] = digest(path)
    require(all(pins.get(str(ROOT / name)) == sha for name, sha in data["metadata"]["source_sha256"].items()),
            "every exact26 training source must be present in imported-only benchmark closure")
    output.mkdir(parents=True, exist_ok=False)
    archived = archive_pins(output, pins)
    plan = dict(schema=SCHEMA, declared_at_utc=datetime.now(timezone.utc).isoformat(), input_spec=spec,
        input_spec_sha256=inputs_sha, source_and_asset_sha256=pins, archived_source_and_asset_sha256=archived,
        thread_arms=list(THREAD_ARMS), interop_threads=1, fixed_camera_ordinals=list(data["anchors"]),
        fixed_prefill_ordinals_per_anchor=[[index-2,index-1] for index in data["anchors"]],
        actual_native_and_camera_history=True, original_lossless_BGRA_pixels_replayed=False,
        reconstructed_BGRA_from_actual_JPEG=True, identical_normalized_tensor_SHA_required_across_thread_arms=True,
        numeric_only_fixed_final6000=True, all47_frozen_tensors_checked_by_original_loader=True,
        hard_policy_deadline_seconds=.1, deadline_relaxed=False, controller_target_cap_mps=2.4,
        exact_recorded_actor_controller_config=data["controller"],
        controller_geometry_source="pinned_actual_learned_summary_actor_physics_config_not_nominal_rear_reference",
        native_actor_physics_geometry_independently_remeasured_by_this_benchmark=False,
        original_NN_decoder_gate_PI_control_guard_math=True, source_pixel_state_or_weight_guards_removed=False,
        anchor_windows_independently_reset=True, PI_initial_integral_per_anchor=0.,
        warm_cache_requires_two_real_adjacent_past_prefills=True, all_cold_prefills_and_rejections_retained=True,
        telemetry_wrapper_overhead_inside_measured_policy_deadline=True,
        conditional_cache_reset_is_offline_probe_not_live_latch_recovery=True,
        simulation_or_vehicle_run=False, actual_control_applied=False, teacher_fallback_after_handoff=False,
        camera_callback_or_ACK_measured=False, exposure_to_ACK_latency_proven=False,
        ten_hz_end_to_end_proven=False, MAIN9_approval_count=0, full_feature_approval=False,
        observed_best_checkpoint_or_frame_selection=False, no_runtime_thread_preset_promoted_by_this_benchmark=True)
    write_new(output / "declaration.json", plan)
    assert_pins(pins); assert_pins(archived)
    return dict(declaration=str(output / "declaration.json"), declaration_sha256=digest(output / "declaration.json"),
        thread_arms=list(THREAD_ARMS), measured_frames_per_arm=len(data["anchors"]), model_executed=False)


@contextmanager
def measured_calls(bindings, records):
    """HH_261002 - Instrument exact calls once without changing values, validation, exceptions, cache or deadline."""
    global _timing_active
    require(not _timing_active, "serial nonnested offline stage instrumentation required")
    saved = []
    _timing_active = True
    try:
        for target, name, label in bindings:
            original = getattr(target, name)
            @functools.wraps(original)
            def measured(*args, _original=original, _label=label, **kwargs):
                started = time.perf_counter_ns()
                try:
                    return _original(*args, **kwargs)
                finally:
                    records.append(dict(stage=_label, seconds=(time.perf_counter_ns()-started)/1e9))
            saved.append((target, name, original))
            setattr(target, name, measured)
        yield
    finally:
        for target, name, original in reversed(saved):
            setattr(target, name, original)
        _timing_active = False


def stage_summary(records):
    result = {}
    for row in records:
        value = result.setdefault(row["stage"], dict(calls=0, inclusive_seconds=0.))
        value["calls"] += 1; value["inclusive_seconds"] += row["seconds"]
    return result


def stats(values):
    values = sorted(values)
    require(all(type(v) in (int,float) and math.isfinite(v) and v >= 0 for v in values), "finite nonnegative times required")
    if not values:
        return dict(count=0, mean=None, p95=None, maximum=None)
    place = .95 * (len(values)-1); lo = int(place); hi = math.ceil(place)
    return dict(count=len(values), mean=sum(values)/len(values),
        p95=values[lo]+(values[hi]-values[lo])*(place-lo), maximum=values[-1])


def equivalent(first, second, *, atol=1e-6, rtol=1e-6):
    # HH_261002 - Numeric comparison is not a driving pass; absent NN/decoder/control evidence never passes vacuously.
    if first is None or second is None:
        return False
    if type(first) is bool or type(second) is bool:
        return type(first) is type(second) and first == second
    if isinstance(first, dict) or isinstance(second, dict):
        return isinstance(first, dict) and isinstance(second, dict) and set(first) == set(second) \
            and all(equivalent(first[k], second[k], atol=atol, rtol=rtol) for k in first)
    if isinstance(first, (list,tuple)) or isinstance(second, (list,tuple)):
        return isinstance(first, (list,tuple)) and isinstance(second, (list,tuple)) and len(first) == len(second) \
            and all(equivalent(a,b,atol=atol,rtol=rtol) for a,b in zip(first,second))
    if type(first) in (int,float) and type(second) in (int,float):
        return math.isfinite(first) and math.isfinite(second) and abs(first-second) <= atol+rtol*abs(first)
    return type(first) is type(second) and first == second


def probe_control(decision):
    # HH_261002 - Retain pre-deadline computed normal control as a proposal, never substitute it for a rejected brake.
    audit = decision.get("longitudinal_pi_research")
    return audit.get("proposed_normal_tracker_control") if isinstance(audit,dict) else None


def compare_arms(arms):
    require([r["threads"] for r in arms] == list(THREAD_ARMS), "fixed complete CPU1/2/4 arm order required")
    reference = arms[0]["rows"]
    results = []
    for arm in arms:
        require(len(arm["rows"]) == len(reference) and reference, "all fixed anchors must remain in each arm")
        matched = []
        for first, current in zip(reference, arm["rows"]):
            require(first["camera_ordinal"] == current["camera_ordinal"], "thread arm anchor order changed")
            a, b = first["decision"], current["decision"]
            first_hashes = a.get("owned_camera_history", {}).get("normalized_input_tensor_sha256")
            current_hashes = b.get("owned_camera_history", {}).get("normalized_input_tensor_sha256")
            same_input = bool(first_hashes) and first_hashes == current_hashes
            match = dict(camera_ordinal=first["camera_ordinal"], same_exact_normalized_input_SHA=same_input,
                raw_output_equivalent=equivalent(a.get("raw_output"), b.get("raw_output")),
                decoded_output_equivalent=equivalent(a.get("decoded_output"), b.get("decoded_output")),
                selected_trajectory_equivalent=equivalent(a.get("selected"), b.get("selected")),
                proposed_normal_PI_control_equivalent=equivalent(probe_control(a), probe_control(b)),
                actual_fail_closed_decision_equivalence_claimed=False)
            matched.append(match)
        results.append(dict(threads=arm["threads"], anchor_count=len(matched), rows=matched,
            all_exact_normalized_inputs_equal=all(r["same_exact_normalized_input_SHA"] for r in matched),
            all_raw_decoded_selected_proposed_control_equivalent=all(all(r[name] for name in
                ("raw_output_equivalent","decoded_output_equivalent","selected_trajectory_equivalent","proposed_normal_PI_control_equivalent")) for r in matched),
            absolute_tolerance=1e-6, relative_tolerance=1e-6))
    return results


def reconstructed_frames(plan):
    """HH_261002 - Reuse frozen observed ego/calibration/navigation preprocessing; only JPEG decoding is approximate."""
    import numpy as np
    from PIL import Image
    from portable_e2e.contract import _canonical_route_in_base, load_contract
    from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs
    from portable_e2e.temporal_simulation_research_v2_20261001 import ObservedTemporalFrameV2
    from portable_e2e.temporal_owned_camera_history_v2_20261001 import own_observed_camera_bytes
    from .collect_ah_simulation_research import state_features
    from .prepare_carla_common10_dataset import _optical_transform

    spec = plan["input_spec"]; data = validate_input_spec(spec); episode = data["episode"]
    states, cameras = observation_rows(episode); by_frame = {r["frame"]:r for r in states}
    manifest = read_json(episode / "manifest.json")
    calibration = []
    require(len(manifest["cameras"]) == 6, "actual six camera calibration required")
    for index, camera in enumerate(manifest["cameras"]):
        require(camera["name"] == MODEL_CAMERA_ORDER[index] and camera["model_index"] == index
                and camera["width"] == 640 and camera["height"] == 360, "actual producer/calibration ABI differs")
        k = camera["intrinsic"]
        calibration.append((k["fx"]/640,k["fy"]/360,k["cx"]/640,k["cy"]/360,*_optical_transform(camera["ros_extrinsic"])[:12]))
    calibration = tuple(calibration); contract = load_contract()
    route = tuple((float(r["x"]),float(r["y"])) for r in read_json(episode / "route.json")["route"])
    identity = dict(episode_id="OFFLINE_JPEG_CPU_THREADS_NOT_LIVE", split="test",
        rig_id=hashlib.sha256(json.dumps(calibration,allow_nan=False).encode()).hexdigest(),
        source_sha256=hashlib.sha256(json.dumps(plan["source_and_asset_sha256"],sort_keys=True).encode()).hexdigest())
    frames = {}
    for index in data["observed_ordinals"]:
        camera = cameras[index]; state = by_frame[camera["frame"]]; stamp = round(state["timestamp"]*1e9)
        yaw = float(state["yaw"]); quaternion = (0.,0.,math.sin(yaw/2),math.cos(yaw/2))
        position = tuple(float(state[name]) for name in ("x","y","z"))
        local,_,_ = _canonical_route_in_base(route,position_m=position,orientation_xyzw=quaternion,
            contract=contract,context="offline identical-JPEG CPU-thread evidence; no future target input")
        payloads = []
        for name in MODEL_CAMERA_ORDER:
            path = camera_path(episode,camera["images"][name])
            with Image.open(path) as image:
                require(image.size == (640,360), "exact source JPEG resolution required")
                rgb = np.asarray(image.convert("RGB"),dtype=np.uint8)
                bgra = np.empty((360,640,4),dtype=np.uint8); bgra[:,:,:3] = rgb[:,:,::-1]; bgra[:,:,3] = 255
            payloads.append(RuntimeCameraFrame(stamp,bgra,"bgra8"))
        history = tuple(state_features(by_frame[cameras[i]["frame"]]) for i in range(max(0,index-9),index+1))
        missing = 10-len(history)
        inputs = RuntimeInputs(tuple(payloads),calibration,((0.,)*13,)*missing+history,
            (False,)*missing+(True,)*len(history),local)
        frames[index] = own_observed_camera_bytes(ObservedTemporalFrameV2(stamp,inputs,position,quaternion,stamp,**identity))
    return frames, identity, data["controller"]


def worker(output, expected_plan_sha, threads):
    require(type(threads) is int and threads in THREAD_ARMS and os.environ.get("CUDA_VISIBLE_DEVICES") == "",
            "explicit CPU-only fixed thread worker required")
    plan_path = output / "declaration.json"
    require(digest(plan_path) == expected_plan_sha, "sealed benchmark declaration changed")
    plan = read_json(plan_path); require(plan["schema"] == SCHEMA and plan["thread_arms"] == list(THREAD_ARMS), "unknown benchmark plan")
    assert_pins(plan["source_and_asset_sha256"]); assert_pins(plan["archived_source_and_asset_sha256"])
    import torch
    torch.set_num_threads(threads); torch.set_num_interop_threads(1)
    require(torch.get_num_threads() == threads and torch.get_num_interop_threads() == 1, "actual Torch thread setting differs")
    from portable_e2e.temporal_route_joint_export_research_20261002 import load_route_joint_research_npz, OWNER_SCHEMA
    from portable_e2e.simulation_research import ControllerConfig
    from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
    from portable_e2e.temporal_cached_encoder_v2 import CachedTemporalModelV2
    from portable_e2e import temporal_cached_encoder_v2 as cached
    from portable_e2e import temporal_single_copy_history_research_20261001 as builder
    from portable_e2e import temporal_simulation_research_v2_20261001 as policy_module
    from portable_e2e.path_time_decoder_v2 import PathTimeDecoderV2
    from portable_e2e.longitudinal_pi_headroom_research_20261001 import LongitudinalPIHeadroomResearch
    from .collect_temporal_route_recovery_fast45_20261001 import RecoverySingleCopyPolicyV3

    spec=plan["input_spec"]; endpoint=spec["final_endpoint"]
    owner=dict(schema=OWNER_SCHEMA,mode="RESEARCH_ONLY",owner_id="HH_261002-sealed-CPU-thread-offline-only",
        purpose="offline_cpu_numeric_validation_only",simulator_control_authorized=False,
        vehicle_control_approved=False,model_promoted=False,fixed_endpoint_updates=6000,**endpoint)
    model=load_route_joint_research_npz(spec["weights"],spec["metadata"],weights_sha256=endpoint["weights_sha256"],
        metadata_sha256=endpoint["metadata_sha256"],parent_weights=spec["parent_weights"],parent_metadata=spec["parent_metadata"],
        backbone_parent_weights=spec["backbone_parent_weights"],backbone_parent_metadata=spec["backbone_parent_metadata"],
        research_owner_binding=owner)
    reconstruct_started=time.perf_counter();frames,identity,controller=reconstructed_frames(plan)
    reconstruction_seconds=time.perf_counter()-reconstruct_started
    config=ControllerConfig(**controller);config.validate()
    state_before=model._state_identity()
    stages=[]
    bindings=[(builder,"build_runtime_tensors","current_six_camera_preprocess_and_runtime_validation"),
        (RouteRecoveryTemporalModelV3,"_validate_inputs","full_model_input_validation_each_call"),
        (CachedTemporalModelV2,"_state_identity","whole_model_state_and_topology_identity"),
        (cached,"_tensor_sha256","cached_encoder_weight_calibration_and_normalized_pixel_SHA"),
        (PathTimeDecoderV2,"forward","unchanged_path_time_decoder"),
        (policy_module,"validate_and_select_trajectory","unchanged_candidate_geometry_gate"),
        (policy_module,"trajectory_control","unchanged_trajectory_tracker"),
        (LongitudinalPIHeadroomResearch,"update","unchanged_2p40_headroom_PI")]
    rows=[];prefills=[];path=output/f"threads_{threads}_rows.jsonl"
    require(not path.exists(), "fresh per-arm all-attempt journal required")
    with path.open("x") as journal,measured_calls(bindings,stages):
        for index in plan["fixed_camera_ordinals"]:
            policy=RecoverySingleCopyPolicyV3(model,config,**identity,deadline_s=.1,longitudinal_pi=True)
            require(policy.pi.integral_error == 0., "each independent warmed probe must start from zero PI state")
            for prefill_index in (index-2,index-1):
                frame=frames[prefill_index];stages.clear();started=time.perf_counter_ns()
                record=policy.prefill(frame,past_frames=tuple(frames[j] for j in range(prefill_index-9,prefill_index)),
                    now_ns=frame.timestamp_ns,ego_stamp_ns=frame.timestamp_ns,route_stamp_ns=frame.timestamp_ns)
                prefills.append(dict(anchor_camera_ordinal=index,prefill_camera_ordinal=prefill_index,
                    full_call_seconds=(time.perf_counter_ns()-started)/1e9,stages=stage_summary(stages),record=record))
            frame=frames[index];stages.clear();started=time.perf_counter_ns()
            decision=policy.step(frame,past_frames=tuple(frames[j] for j in range(index-9,index)),
                now_ns=frame.timestamp_ns,ego_stamp_ns=frame.timestamp_ns,route_stamp_ns=frame.timestamp_ns)
            row=dict(camera_ordinal=index,frame_timestamp_ns=frame.timestamp_ns,
                full_call_seconds=(time.perf_counter_ns()-started)/1e9,stages=stage_summary(stages),decision=decision,
                cold_cache_expected=False,actual_actuator_ACK=False,original_guard_deadline_preserved=True)
            rows.append(row);journal.write(json.dumps(row,sort_keys=True,allow_nan=False)+"\n");journal.flush()
    require(model._state_identity() == state_before, "weights/module topology changed during offline thread arm")
    assert_pins(plan["source_and_asset_sha256"]);assert_pins(plan["archived_source_and_asset_sha256"])
    result=dict(schema=SCHEMA+".arm",threads=threads,interop_threads=torch.get_num_interop_threads(),
        torch_version=torch.__version__,declaration_sha256=expected_plan_sha,rows=rows,cold_and_adjacent_prefills=prefills,
        JPEG_to_owned_BGRA_history_reconstruction_seconds=reconstruction_seconds,
        identical_original_model_guards_retained=True,source_and_archives_unchanged=True,weights_and_topology_unchanged=True,
        normalized_input_SHA_every_attempt=True,pre_deadline_normal_control_is_not_applied=True,
        cold_prefill_deadline_waived_for_actuation=False,independent_zero_PI_anchor_windows=True,
        full_call_seconds=stats([r["full_call_seconds"] for r in rows]),
        policy_total_seconds=stats([r["decision"]["total_seconds"] for r in rows]),
        raw_nn_seconds=stats([r["decision"]["raw_nn_seconds"] for r in rows if r["decision"]["raw_nn_seconds"] is not None]),
        proposal_accepted_count=sum(r["decision"]["learned_control_proposed"] for r in rows),
        deadline_rejected_count=sum(r["decision"]["emergency_reason"] == "temporal inference/control deadline exceeded" for r in rows),
        all_rejections=[dict(camera_ordinal=r["camera_ordinal"],reason=r["decision"]["emergency_reason"])
            for r in rows if not r["decision"]["learned_control_proposed"]],
        inclusive_stage_durations_overlap=True,actual_live_or_vehicle_control=False,ten_hz_end_to_end_proven=False)
    write_new(output/f"threads_{threads}.json",result)
    return result


def worker_command(output, plan_sha, threads):
    # HH_261002 - The canonical workspace parser needs its explicit remainder separator before worker options.
    require(threads in THREAD_ARMS and type(threads) is int, "fixed subprocess thread arm required")
    return [sys.executable,"-B",str(ROOT/"tools/workspace_python.py"),"-m",MODULE,"--","--worker","--output",str(output),
            "--declaration-sha256",plan_sha,"--threads",str(threads)]


def execute(output, expected_plan_sha):
    plan_path=output/"declaration.json";require(digest(plan_path)==expected_plan_sha,"caller must pin exact preexecution declaration")
    plan=read_json(plan_path);require(plan.get("schema")==SCHEMA and plan.get("thread_arms")==list(THREAD_ARMS),"exact fixed thread plan required")
    require(not (output/"summary.json").exists(),"do not retry/overwrite a completed benchmark")
    arms=[];attempts=[]
    for threads in THREAD_ARMS:
        assert_pins(plan["source_and_asset_sha256"]);assert_pins(plan["archived_source_and_asset_sha256"])
        environment=dict(os.environ,CUDA_VISIBLE_DEVICES="",OMP_NUM_THREADS=str(threads),MKL_NUM_THREADS=str(threads),OPENBLAS_NUM_THREADS=str(threads))
        command=worker_command(output,expected_plan_sha,threads)
        with (output/f"threads_{threads}_stdout.log").open("x") as stdout,(output/f"threads_{threads}_stderr.log").open("x") as stderr:
            child=subprocess.run(command,cwd=ROOT,env=environment,stdout=stdout,stderr=stderr,timeout=900,check=False)
        path=output/f"threads_{threads}.json"
        attempts.append(dict(threads=threads,returncode=child.returncode,command=command,
            stdout_sha256=digest(output/f"threads_{threads}_stdout.log"),stderr_sha256=digest(output/f"threads_{threads}_stderr.log"),
            result_sha256=digest(path) if path.is_file() else None))
        if child.returncode == 0 and path.is_file():arms.append(read_json(path))
    assert_pins(plan["source_and_asset_sha256"]);assert_pins(plan["archived_source_and_asset_sha256"])
    complete=len(arms)==3;comparison=compare_arms(arms) if complete else []
    summary=dict(schema=SCHEMA+".summary",declaration_sha256=expected_plan_sha,attempts=attempts,
        every_predeclared_arm_completed=complete,comparisons=comparison,
        thread_arm_timings=[dict(threads=r["threads"],accepted=r["proposal_accepted_count"],deadline_rejections=r["deadline_rejected_count"],
            policy_total_seconds=r["policy_total_seconds"],full_call_seconds=r["full_call_seconds"],raw_nn_seconds=r["raw_nn_seconds"]) for r in arms],
        all_exact_normalized_inputs_equal=complete and all(r["all_exact_normalized_inputs_equal"] for r in comparison),
        all_raw_decoded_selected_proposed_control_equivalent=complete and all(r["all_raw_decoded_selected_proposed_control_equivalent"] for r in comparison),
        all_source_and_archive_pins_unchanged=True,actual_physical_latency_or_driving_pass=False,
        JPEG_reconstruction_not_original_lossless_BGRA=True,camera_callback_or_ACK_measured=False,
        deadline_relaxed=False,ten_hz_end_to_end_proven=False,MAIN9_approval_count=0,
        runtime_thread_configuration_promoted=False,all_failures_retained=True)
    write_new(output/"summary.json",summary)
    return summary


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    mode=parser.add_mutually_exclusive_group(required=True)
    for name in ("declare","execute","worker"):mode.add_argument("--"+name,action="store_true")
    parser.add_argument("--output",type=Path,required=True);parser.add_argument("--inputs",type=Path)
    parser.add_argument("--inputs-sha256");parser.add_argument("--declaration-sha256");parser.add_argument("--threads",type=int)
    args=parser.parse_args(argv)
    if args.declare:
        require(args.inputs is not None and args.inputs_sha256 is not None and args.declaration_sha256 is None and args.threads is None,
                "declare only with externally pinned inputs; no model/thread execution")
        result=declare(args.output,args.inputs,args.inputs_sha256)
    elif args.execute:
        require(args.declaration_sha256 is not None and args.inputs is None and args.inputs_sha256 is None and args.threads is None,
                "execute only exact sealed declaration and all fixed thread arms")
        result=execute(args.output,args.declaration_sha256)
    else:
        require(args.declaration_sha256 is not None and args.threads in THREAD_ARMS and args.inputs is None and args.inputs_sha256 is None,
                "explicit declared offline subprocess arm required")
        result=worker(args.output,args.declaration_sha256,args.threads)
    print(json.dumps(result,sort_keys=True,allow_nan=False));return 0


if __name__=="__main__":raise SystemExit(main())
