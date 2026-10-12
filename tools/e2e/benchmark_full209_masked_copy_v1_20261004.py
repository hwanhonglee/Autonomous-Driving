"""HH_261004 - Prepare full cached-forward parity on all original V2 JPEG-derived causal contexts.

This source-only offline probe keeps 23 original cold/warm prefills and all 865
fresh NN contexts. The 901 recorded camera bundles and all 7,366 archive files
are pinned, including the failed tail. JPEG reconstruction is lossy: neither
original BGRA/tensor equality nor original actual raw-NN equality is claimed.
Thirty fixed first-to-last AB/BA pairs time the complete forward_cached calls,
including the first cold call and every original model guard. No simulator,
decoder, control application, optimizer, deadline waiver or promotion is used.

After serialized CPU authorization, prospective declaration then execution:
CUDA_VISIBLE_DEVICES='' /usr/bin/python3 tools/workspace_python.py -m \
 scripts.e2e.benchmark_full209_masked_copy_v1_20261004 -- --mode declare
CUDA_VISIBLE_DEVICES='' /usr/bin/python3 tools/workspace_python.py -m \
 scripts.e2e.benchmark_full209_masked_copy_v1_20261004 -- --mode execute \
 --expected-declaration-sha256 <printed declaration SHA256>
"""

from __future__ import annotations

import argparse
from collections import deque
from contextlib import contextmanager
from dataclasses import asdict
import fcntl
import hashlib
from io import BytesIO
import json
import math
import os
from pathlib import Path
import platform
import sys
import time

from . import benchmark_full209_parallel_JPEG_v1_20261004 as recorded
from .benchmark_scandir_episode_quota_v1_20261004 import digest, read_json, require, safe_path, write_new_json


ROOT, RUN, EPISODE, CALLER = recorded.ROOT, recorded.RUN, recorded.EPISODE, recorded.CALLER
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_full209_masked_copy_v1_20261004.py"
CANDIDATE = ROOT / "learning/portable_e2e/temporal_all_valid_masked_copy_research_v1_20261004.py"
CANDIDATE_TEST = ROOT / "learning/tests/test_temporal_all_valid_masked_copy_research_v1_20261004.py"
CANDIDATE_SHA = "3cffa23b9a63965739d38cd47f980a799a6748cbf374e61ab35b3ceb91d66d5b"
CANDIDATE_TEST_SHA = "7dba2dbb77494101827b4b3399050ea1d05c24f7f15ccd96fc49dd2b0ea67d3a"
SCHEMA = "portable_e2e.full209_masked_copy_JPEG_counterfactual.20261004.v1"
PLAN = ROOT / "storage/experiments/plans/2026-10-04/full209_masked_copy_JPEG_counterfactual_v1/declaration.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-04/full209_masked_copy_JPEG_counterfactual_v1"
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
PREFILLS, FRESH_NN, CONTEXTS, PAIRS = 23, 865, 888, 30
CPU_THREADS, INTEROP_THREADS = 4, 1
TRAIN = ROOT / "storage/experiments/training"
ASSETS = {
    "weights": TRAIN / "2026-10-02/temporal_route_joint_v1/exports/step_006000/model_state.npz",
    "metadata": TRAIN / "2026-10-02/temporal_route_joint_v1/exports/step_006000/metadata.json",
    "training_declaration": TRAIN / "2026-10-02/temporal_route_joint_v1/declaration.json",
    "parent_weights": TRAIN / "2026-10-01/temporal_route_recovery_v3_v1/exports/step_003000/model_state.npz",
    "parent_metadata": TRAIN / "2026-10-01/temporal_route_recovery_v3_v1/exports/step_003000/metadata.json",
    "backbone_parent_weights": TRAIN / "2026-10-01/temporal_v2_balanced_replay_v1/exports/step_003000/model_state.npz",
    "backbone_parent_metadata": TRAIN / "2026-10-01/temporal_v2_balanced_replay_v1/exports/step_003000/metadata.json",
}
ENDPOINT = dict(weights_sha256="b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625",
    metadata_sha256="a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
    checkpoint_sha256="43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88",
    declaration_sha256="0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5")


def jsonl(path):
    path = safe_path(path)
    require(path.stat().st_size <= 48 * 1024**2, "bounded complete recorded JSONL required")
    # HH_261004 - Reuse the original strict metadata parser for each raw record without importing model code.
    def unique(items):
        result = {}
        for name, value in items:
            require(name not in result, "duplicate original JSONL member")
            result[name] = value
        return result
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line, object_pairs_hook=unique,
            parse_constant=lambda value: require(False, "nonfinite original JSONL")) for line in stream]


def event_catalog(cameras, prefills, learned):
    """HH_261004 - Preserve every real prefill/fresh NN call; held commands cannot become new NN contexts."""
    by_frame = {row["frame"]: index for index, row in enumerate(cameras)}
    fresh = [row for row in learned if row.get("held_between_camera_frames") is False]
    require(len(prefills) == PREFILLS and len(fresh) == FRESH_NN, "complete23-prefill/865-fresh-NN denominator required")
    events = []
    for kind, rows in (("original_prefill", prefills), ("original_fresh_NN", fresh)):
        for row in rows:
            frame = row["native_frame"] if kind == "original_prefill" else row["inference_frame"]
            require(frame in by_frame and (kind == "original_prefill" or
                row["command_frame"] == frame and bool(row.get("raw_output"))), "authentic fresh frame/raw output required")
            index = by_frame[frame]
            audit = row.get("owned_camera_history", {})
            expected_stamp = round(cameras[index]["timestamp"] * 1e9)
            sources = audit.get("source_camera_frames", ())
            require(index >= 9 and audit.get("frame_timestamp_ns") == expected_stamp and len(sources) == 5,
                "complete recorded causal image and ten-row ego history required")
            expected = [round(cameras[index + offset]["timestamp"] * 1e9) for offset in (-8, -6, -4, -2, 0)]
            require([item["timestamp_ns"] if item else None for item in sources] == expected,
                "all five original source timestamps must match actual causal camera slots")
            hashes = audit.get("normalized_input_tensor_sha256", {})
            require(set(hashes) == {"images", "calibration", "ego_history", "ego_history_mask", "route_xy",
                "route_mask", "time_valid", "relative_pose", "time_offsets_s"}, "all nine original input SHA identities required")
            events.append(dict(camera_ordinal=index, native_frame=frame, kind=kind,
                anchor_timestamp_ns=expected_stamp, original_input_sha256=hashes,
                original_BGRA_source_camera_frames=sources))
    events.sort(key=lambda row: row["camera_ordinal"])
    require(len(events) == CONTEXTS and len({row["camera_ordinal"] for row in events}) == CONTEXTS,
        "every original model context exactly once required")
    require([row["camera_ordinal"] for row in events] == list(range(12, 900)),
        "complete original context chain12..899 with both ends required")
    return events


def selected_indices(count=CONTEXTS):
    return recorded.selected_indices(count)


def contract():
    return dict(all_original_files=7366, all_camera_bundles=901, all_original_JPEGs=5406,
        original_prefills=PREFILLS, original_fresh_NN=FRESH_NN, complete_recorded_contexts=CONTEXTS,
        all_camera_ordinals_retained=True, model_context_camera_ordinals=list(range(12, 900)),
        unexecuted_warmup_ordinals=list(range(12)), final_camera_without_original_NN_ordinal=900,
        context_selection="all actual model calls; unused warmup/tail camera rows never fabricated as original NN calls",
        representative_context_indices=selected_indices(), pairs=PAIRS, timed_calls=2 * PAIRS,
        pair_order="AB for even selected pair index, BA for odd; all intervening original contexts processed serially",
        timer_scope="complete unchanged forward_cached including both finite scans, state/pixel/provenance hashes, CNN cache, recovery head hook and all neural outputs",
        timer_excludes="JPEG reconstruction/original input builder, external input/state equality checks, model construction/load, journals",
        existing_shared_runtime_lock=str(LOCK),
        first_selected_pair_is_original_cold_prefill=True, no_warmup_or_failed_tail_cropping=True,
        no_rejected_trial_dropping_or_retry=True, no_deadline_filter_or_waiver=True,
        CPU_threads=CPU_THREADS, interop_threads=INTEROP_THREADS, fixed_endpoint=ENDPOINT,
        raw_control_candidate_tensor_bytes_equal_required=True, all_input_and_learned_state_unchanged_required=True,
        every_nonimage_input_SHA_equal_to_original_record_required=True, every_original_source_timestamp_equal_required=True,
        cache_work_and_cached_feature_bytes_equal_required=True, source_hashing_warms_OS_page_cache_disclosed=True,
        reconstructed_lossy_JPEG=True, original_lossless_BGRA_equality_claimed=False,
        original_actual_raw_NN_equality_claimed=False, decoder_or_control_executed=False,
        actual_sensor_latency_gain_proven=False, ten_hz_pipeline_proven=False, actual_runtime_activated=False,
        training_data_approved=False, model_promoted=False, main_functions_approved=0, subfunctions_approved=0)


def source_snapshot():
    # HH_261004 - Inherit sealed original Python/preprocessing sources; keep the candidate and its review test immutable.
    original = read_json(RUN / "declaration.json")
    pins = {name: sha for name, sha in original["source_sha256"].items()
            if name.endswith(".py") or name == "learning/portable_e2e/config/common_10hz_v1.contract.json"}
    require(all(digest(ROOT / name) == sha for name, sha in pins.items()), "original frozen source closure changed")
    require(digest(CANDIDATE) == CANDIDATE_SHA and digest(CANDIDATE_TEST) == CANDIDATE_TEST_SHA,
        "frozen source-only masked-copy candidate/review test changed")
    for path in (HERE, TEST, CANDIDATE, CANDIDATE_TEST, Path(recorded.__file__),
            ROOT / "tools/e2e/benchmark_scandir_episode_quota_v1_20261004.py"):
        pins[str(path.relative_to(ROOT))] = digest(path)
    return pins


def asset_snapshot():
    original = read_json(RUN / "declaration.json")["asset_sha256"]
    pins = {str(path): digest(path) for path in ASSETS.values()}
    require(all(original.get(name) == sha for name, sha in pins.items()), "exact original GOOD6000/parent numeric assets required")
    require(pins[str(ASSETS["weights"])] == ENDPOINT["weights_sha256"]
        and pins[str(ASSETS["metadata"])] == ENDPOINT["metadata_sha256"]
        and pins[str(ASSETS["training_declaration"])] == ENDPOINT["declaration_sha256"], "fixed final6000 endpoint changed")
    return pins


def environment():
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == "" and sys.executable == "/usr/bin/python3"
        and sys.version_info[:2] == (3, 10), "original Python3.10 and explicitly disabled GPU required")
    return dict(interpreter=sys.executable, python=platform.python_version(), CPU_threads=CPU_THREADS,
        interop_threads=INTEROP_THREADS, GPU_used=False,
        environment={name: os.environ.get(name) for name in
            ("CUDA_VISIBLE_DEVICES", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")})


@contextmanager
def serialized_runtime():
    """HH_261004 - Reuse the existing nonblocking workspace lease before any archive scan or model import."""
    with safe_path(LOCK).open("rb") as lease:
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("existing CARLA/runtime owner must finish before offline CPU benchmark") from error
        try:
            yield
        finally:
            fcntl.flock(lease, fcntl.LOCK_UN)


def append_journal(name, value):
    path = safe_path(OUTPUT / name, missing_tail=True)
    with path.open("a" if path.exists() else "x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, sort_keys=True, allow_nan=False) + "\n")


@contextmanager
def retain_attempt(parity, timings):
    try:
        yield
    except BaseException as error:
        write_new_json(OUTPUT / "FAILED_attempt.json", dict(schema=SCHEMA + ".failed_attempt",
            provenance="HH_261004 - Retain partial/mismatched contexts and every attempted cold/slow/failed timed call; never retry this output.",
            status="FAILED", planned_contexts=CONTEXTS, planned_timed_calls=2 * PAIRS,
            completed_parity_rows=parity, attempted_timing_rows=timings,
            complete_and_partial_context_journal="all_context_attempts.jsonl",
            exception_type=type(error).__name__, exception=str(error),
            original_actual_FAIL_changed=False, model_promoted=False, main_functions_approved=0, subfunctions_approved=0))
        raise


def reconstructed_frames(cameras, states):
    """HH_261004 - Keep at most ten real frames and reuse frozen route/ego/calibration preprocessing."""
    import numpy as np
    from PIL import Image
    from portable_e2e.contract import _canonical_route_in_base, load_contract
    from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs
    from portable_e2e.temporal_simulation_research_v2_20261001 import ObservedTemporalFrameV2
    from portable_e2e.temporal_owned_camera_history_v2_20261001 import own_observed_camera_bytes
    from .collect_ah_simulation_research import state_features
    from .prepare_carla_common10_dataset import _optical_transform

    require(len(states) == 1801 and len({row["frame"] for row in states}) == 1801,
        "all1801 original native states required")
    by_frame = {row["frame"]: row for row in states}
    manifest = read_json(EPISODE / "manifest.json")
    calibration = []
    require(len(manifest["cameras"]) == 6, "original six-camera calibration required")
    for index, camera in enumerate(manifest["cameras"]):
        require(camera["name"] == recorded.ROLES[index] and camera["model_index"] == index
            and camera["width"] == 640 and camera["height"] == 360, "original camera calibration ABI required")
        k = camera["intrinsic"]
        calibration.append((k["fx"] / 640, k["fy"] / 360, k["cx"] / 640, k["cy"] / 360,
            *_optical_transform(camera["ros_extrinsic"])[:12]))
    calibration = tuple(calibration)
    route = tuple((float(row["x"]), float(row["y"])) for row in read_json(EPISODE / "route.json")["route"])
    identity = dict(episode_id="OFFLINE_FULL209_JPEG_MASKED_COPY_NOT_LIVE", split="val",
        rig_id=hashlib.sha256(json.dumps(calibration, allow_nan=False).encode()).hexdigest(),
        source_sha256=recorded.CALLER_SHA)
    loaded_contract = load_contract()
    original_file_pins = read_json(CALLER)["run_files_sha256"]
    history, frames = deque(maxlen=10), deque(maxlen=10)
    for index, camera in enumerate(cameras):
        require(camera["frame"] in by_frame, "camera must bind a recorded native state")
        state = by_frame[camera["frame"]]
        stamp = round(float(state["timestamp"]) * 1e9)
        require(stamp == round(camera["timestamp"] * 1e9) and all(
            round(camera["source_timestamps"][role] * 1e9) == stamp for role in recorded.ROLES),
            "all six actual source/native timestamps must agree")
        yaw = float(state["yaw"])
        quaternion = (0., 0., math.sin(yaw / 2), math.cos(yaw / 2))
        position = tuple(float(state[name]) for name in ("x", "y", "z"))
        local, _, _ = _canonical_route_in_base(route, position_m=position, orientation_xyzw=quaternion,
            contract=loaded_contract, context="offline original causal full209 navigation; no future target input")
        payloads = []
        for role in recorded.ROLES:
            path = safe_path(EPISODE / camera["images"][role])
            payload = path.read_bytes()
            require(hashlib.sha256(payload).hexdigest() == original_file_pins[str(path.relative_to(RUN))],
                "each consumed original JPEG byte must match the actual whole-run seal")
            with Image.open(BytesIO(payload)) as image:
                require(image.size == (640, 360), "original640x360 JPEG required")
                rgb = np.asarray(image.convert("RGB"), dtype=np.uint8)
                bgra = np.empty((360, 640, 4), dtype=np.uint8)
                bgra[:, :, :3], bgra[:, :, 3] = rgb[:, :, ::-1], 255
            payloads.append(RuntimeCameraFrame(stamp, bgra, "bgra8"))
        history.append(state_features(state))
        missing = 10 - len(history)
        inputs = RuntimeInputs(tuple(payloads), calibration, ((0.,) * 13,) * missing + tuple(history),
            (False,) * missing + (True,) * len(history), local)
        frames.append(own_observed_camera_bytes(ObservedTemporalFrameV2(stamp, inputs, position, quaternion, stamp, **identity)))
        yield index, frames[-1], tuple(frames)[:-1], identity


def tensor_hashes(values):
    from portable_e2e.temporal_owned_camera_history_v2_20261001 import normalized_tensor_hashes
    return normalized_tensor_hashes(values)


def compare_raw(first, second):
    """HH_261004 - Require complete raw output tensors and exact bytes, never rounded serialized output tolerance."""
    import torch
    expected = {"curvature_knots", "preferred_speed_knots", "intent_logits", "stop_distance_raw", "acceleration_prediction"}
    require(set(first) == set(second) == expected, "complete original five raw neural outputs required")
    require(all(type(first[name]) is torch.Tensor and type(second[name]) is torch.Tensor
        and first[name].dtype == second[name].dtype and first[name].shape == second[name].shape
        and torch.equal(first[name], second[name]) for name in expected)
        and tensor_hashes(first) == tensor_hashes(second), "every raw output byte must equal on identical JPEG-derived tensors")
    return tensor_hashes(first)


def cache_signature(model):
    # HH_261004 - Cache values may evolve normally but the original/candidate work and every retained feature must agree.
    return dict(work=asdict(model.cache_stats), entries=[dict(timestamp_ns=frame.timestamp_ns,
        camera_sha256=list(frame.camera_sha256), pixel_sha256=entry.pixel_sha256,
        feature_sha256=tensor_hashes({"features": entry.features})["features"])
        for frame, entry in sorted(model.image_encoder.entries.items(), key=lambda item: item[0].timestamp_ns)])


def call(model, inputs, window, *, arm, context_index, timed_pair, timings):
    """HH_261004 - Keep only full forward_cached inside the timer; preserve unsuccessful attempted calls too."""
    started = time.perf_counter_ns() if timed_pair is not None else None
    error = None
    try:
        return model.forward_cached(inputs, window=window)
    except BaseException as caught:
        error = caught
        raise
    finally:
        if started is not None:
            elapsed = time.perf_counter_ns() - started
            value = dict(pair=timed_pair, arm=arm, context_index=context_index, elapsed_ns=elapsed,
                elapsed_ms=elapsed / 1e6, status="COMPLETED" if error is None else "REJECTED",
                exception_type=type(error).__name__ if error is not None else None,
                exception=str(error) if error is not None else None,
                original_first_cold_context=context_index == 0, no_deadline_filter=True)
            timings.append(value)
            append_journal("all_timing_rows.jsonl", value)


def models():
    import torch
    from portable_e2e.temporal_route_joint_export_research_20261002 import load_route_joint_research_npz, OWNER_SCHEMA
    from portable_e2e.temporal_all_valid_masked_copy_research_v1_20261004 import AllValidMaskedCopyRouteRecoveryTemporalModelV3
    owner = dict(schema=OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id="HH_261004-full209-lossy-masked-copy-offline-only",
        purpose="offline_cpu_numeric_validation_only", simulator_control_authorized=False,
        vehicle_control_approved=False, model_promoted=False, fixed_endpoint_updates=6000, **ENDPOINT)
    control = load_route_joint_research_npz(ASSETS["weights"], ASSETS["metadata"],
        weights_sha256=ENDPOINT["weights_sha256"], metadata_sha256=ENDPOINT["metadata_sha256"],
        parent_weights=ASSETS["parent_weights"], parent_metadata=ASSETS["parent_metadata"],
        backbone_parent_weights=ASSETS["backbone_parent_weights"], backbone_parent_metadata=ASSETS["backbone_parent_metadata"],
        research_owner_binding=owner)
    candidate = AllValidMaskedCopyRouteRecoveryTemporalModelV3(control.config, control.recovery_config)
    candidate.load_state_dict({name: value.detach().clone() for name, value in control.state_dict().items()}, strict=True)
    candidate.requires_grad_(False).eval()
    for name in ("_research_temporal_checkpoint_sha256", "_research_temporal_export_npz_sha256", "_research_temporal_metadata_sha256"):
        setattr(candidate, name, getattr(control, name))
    require(all(torch.equal(value, candidate.state_dict()[name]) for name, value in control.state_dict().items()),
        "all101 strict original learned state tensors must equal")
    return {"A": control, "B": candidate}


def execute(events, cameras, parity, timings):
    import torch
    from portable_e2e.simulation_research import ControllerConfig
    from portable_e2e.temporal_single_copy_history_research_20261001 import SingleCopyOwnedHeadroomPolicyV2
    torch.set_num_threads(CPU_THREADS)
    torch.set_num_interop_threads(INTEROP_THREADS)
    require(torch.get_num_threads() == CPU_THREADS and torch.get_num_interop_threads() == INTEROP_THREADS,
        "original CPU4/interop1 actual readback required")
    arms = models()
    controller = ControllerConfig(**read_json(EPISODE / "learned_summary.json")["controller"])
    builder = None
    baseline = {arm: model._state_identity() for arm, model in arms.items()}
    by_ordinal = {row["camera_ordinal"]: (index, row) for index, row in enumerate(events)}
    selected = {index: pair for pair, index in enumerate(selected_indices())}
    reconstructed_count = 0
    with torch.no_grad():
        for ordinal, frame, past, identity in reconstructed_frames(cameras, jsonl(EPISODE / "states.jsonl")):
            reconstructed_count += 1
            if ordinal not in by_ordinal:
                continue
            index, event = by_ordinal[ordinal]
            append_journal("all_context_attempts.jsonl", dict(context_index=index, **event, status="STARTED"))
            if builder is None:
                builder = SingleCopyOwnedHeadroomPolicyV2(arms["A"], controller, longitudinal_pi=True, **identity)
                # HH_261004 - Builder initialization owns a normal cache reset; compare each arm's immutable learned state afterward.
                arms["B"].reset_cache("new_live_stream_requires_prefill")
            tensors, window = builder.build_inputs(frame, past_frames=past, now_ns=frame.timestamp_ns,
                ego_stamp_ns=frame.timestamp_ns, route_stamp_ns=frame.timestamp_ns)
            before = tensor_hashes(tensors)
            require(all(before[name] == sha for name, sha in event["original_input_sha256"].items() if name != "images"),
                "JPEG-derived context nonimage tensor differs from actual recorded causal context")
            require(bool(tensors["time_valid"].all().item()), "complete original all-valid model context required")
            pair = selected.get(index)
            order = ("A", "B") if (pair if pair is not None else index) % 2 == 0 else ("B", "A")
            raw = {}
            for arm in order:
                raw[arm] = call(arms[arm], tensors, window, arm=arm, context_index=index, timed_pair=pair, timings=timings)
                append_journal("all_context_attempts.jsonl", dict(context_index=index, arm=arm,
                    status="FORWARD_RETURNED", raw_output_sha256=tensor_hashes(raw[arm])))
                require(before == tensor_hashes(tensors), "original/candidate changed an input tensor")
                require(arms[arm]._state_identity() == baseline[arm] and not arms[arm]._recovery_active
                    and arms[arm]._recovery_features is None and arms[arm].image_encoder.request is None,
                    "every learned state/topology and original serial recovery/cache cleanup must remain unchanged")
            row = dict(context_index=index, **event, input_sha256=before, order=list(order),
                raw_output_sha256=compare_raw(raw["A"], raw["B"]), all_input_and_learned_state_unchanged=True,
                raw_control_candidate_tensor_bytes_equal=True, nonimage_inputs_equal_original_record=True,
                original_actual_raw_NN_equality_claimed=False, cache_control=cache_signature(arms["A"]),
                cache_candidate=cache_signature(arms["B"]))
            parity.append(row)
            append_journal("all_parity_rows.jsonl", row)
            require(row["cache_control"] == row["cache_candidate"], "original/candidate cache work or retained feature bytes differ")
    require(reconstructed_count == 901 and len(parity) == CONTEXTS and len(timings) == 2 * PAIRS,
        "all901 cameras/all888 contexts/all60 inclusive timing calls required")
    return dict(reconstructed_camera_count=reconstructed_count, torch_version=torch.__version__,
        torch_threads=torch.get_num_threads(), torch_interop_threads=torch.get_num_interop_threads(),
        every_learned_state_unchanged=all(model._state_identity() == baseline[arm] for arm, model in arms.items()))


def run(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("declare", "execute"), required=True)
    parser.add_argument("--expected-declaration-sha256")
    args = parser.parse_args(argv)
    recorded.fresh_output(OUTPUT)
    before = dict(sources=source_snapshot(), inputs=recorded.input_snapshot(), assets=asset_snapshot(),
        environment=environment(), contract=contract())
    require(before["inputs"]["all_original_file_count"] == 7366, "all7366 sealed original files required")
    cameras = recorded.camera_catalog()
    events = event_catalog(cameras, jsonl(EPISODE / "temporal_prefill.jsonl"), jsonl(EPISODE / "learned_control.jsonl"))
    # HH_261004 - Pin every complete event with one canonical digest; retain full per-context provenance in the output journal.
    before["complete_event_catalog_sha256"] = hashlib.sha256(json.dumps(events, sort_keys=True,
        allow_nan=False, separators=(",", ":")).encode()).hexdigest()
    if args.mode == "declare":
        require(args.expected_declaration_sha256 is None and "torch" not in sys.modules and "carla" not in sys.modules,
            "declaration cannot execute/import model or simulator")
        recorded.fresh_output(PLAN)
        write_new_json(PLAN, dict(schema=SCHEMA + ".prospective",
            provenance="HH_261004 - Seal all original cold/prefill/fresh contexts and fixed30 inclusive AB/BA pairs before any model execution.", **before))
        print(json.dumps(dict(status="DECLARED_NOT_MEASURED", declaration_sha256=digest(PLAN))))
        return 0
    require(args.expected_declaration_sha256 == digest(PLAN), "exact prospective benchmark declaration SHA required")
    plan = read_json(PLAN)
    require(plan["schema"] == SCHEMA + ".prospective" and all(plan[name] == value for name, value in before.items()),
        "declared sources/full inputs/assets/contexts/environment/selection changed")
    OUTPUT.mkdir(parents=True, exist_ok=False)
    parity, timings = [], []
    with retain_attempt(parity, timings):
        execution = execute(events, cameras, parity, timings)
        after = dict(sources=source_snapshot(), inputs=recorded.input_snapshot(), assets=asset_snapshot(),
            environment=environment(), contract=contract())
        after_events = event_catalog(recorded.camera_catalog(), jsonl(EPISODE / "temporal_prefill.jsonl"),
            jsonl(EPISODE / "learned_control.jsonl"))
        after["complete_event_catalog_sha256"] = hashlib.sha256(json.dumps(after_events, sort_keys=True,
            allow_nan=False, separators=(",", ":")).encode()).hexdigest()
        require(before == after and "carla" not in sys.modules, "whole original/source/asset/context identity must remain unchanged")
        report = dict(schema=SCHEMA, provenance="HH_261004 - Report only exact candidate/control parity on lossy reconstructed contexts and inclusive complete-forward timing; no live gain or approval.",
            declaration_sha256=args.expected_declaration_sha256, before=before, after=after, execution=execution,
            all_parity_rows=parity, all_timing_rows=timings,
            control=recorded.stats([row["elapsed_ms"] for row in timings if row["arm"] == "A"]),
            candidate=recorded.stats([row["elapsed_ms"] for row in timings if row["arm"] == "B"]),
            source_and_input_and_asset_unchanged=True, actual_sensor_latency_gain_proven=False,
            original_actual_raw_NN_equality_claimed=False, model_promoted=False, main_functions_approved=0, subfunctions_approved=0)
        write_new_json(OUTPUT / "report.json", report)
        summary = {name: report[name] for name in ("schema", "execution", "control", "candidate",
            "source_and_input_and_asset_unchanged", "actual_sensor_latency_gain_proven", "original_actual_raw_NN_equality_claimed",
            "model_promoted", "main_functions_approved", "subfunctions_approved")}
        summary.update(complete_recorded_contexts=len(parity), attempted_timed_calls=len(timings), report_sha256=digest(OUTPUT / "report.json"))
        write_new_json(OUTPUT / "summary.json", summary)
        print(json.dumps(summary, sort_keys=True))
        return 0


def main(argv=None):
    with serialized_runtime():
        return run(argv)


if __name__ == "__main__":
    raise SystemExit(main())
