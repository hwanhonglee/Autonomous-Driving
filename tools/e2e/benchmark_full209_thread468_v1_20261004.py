"""HH_261004 - Prepare independent CPU4/6/8 cached-forward screens on every frozen V2 JPEG context.

Each arm is a fresh serial process with the same explicit masked-copy V1 model
and strict original GOOD6000 loader. Establish the original V1 CPU4/interop1
baseline BEFORE model imports/construction; the loader does not configure threads.
Verify that baseline after loading, then set the target intra-op arm once.
Keep all 23 prefills, 865 fresh NN contexts and 901 camera
bundles; thirty fixed first-to-last forward_cached samples include cold start.
Archive every raw head and bounded input/cache hashes, compare all raw tensors
at the original atol=rtol=1e-6, and retain all rejected/failed prefixes.

The unchanged ordinary GO decoder is a separate numerical screen only where
the actual nonimage navigation support is >=18m. Terminal proof, original BGRA,
full safety engine/control, live latency and model adoption remain unverified.
No full CNN/image tensor archive is written; all values are hashed, every raw
head is retained, and inputs are reproducible from the pinned original archive.

After source review, not by this source-preparation task:
CUDA_VISIBLE_DEVICES='' /usr/bin/python3 tools/workspace_python.py -m \
 scripts.e2e.benchmark_full209_thread468_v1_20261004 -- --mode declare
CUDA_VISIBLE_DEVICES='' /usr/bin/python3 tools/workspace_python.py -m \
 scripts.e2e.benchmark_full209_thread468_v1_20261004 -- --mode execute \
 --expected-declaration-sha256 <printed prospective declaration SHA256>
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from . import benchmark_full209_masked_copy_v1_20261004 as previous


ROOT, RUN, EPISODE = previous.ROOT, previous.RUN, previous.EPISODE
require, digest, read_json, safe_path, write_new_json = previous.require, previous.digest, previous.read_json, previous.safe_path, previous.write_new_json
SCHEMA = "portable_e2e.full209_masked_copy_thread468_JPEG_counterfactual.20261004.v1"
MODULE = "scripts.e2e.benchmark_full209_thread468_v1_20261004"
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_full209_thread468_v1_20261004.py"
PLAN = ROOT / "storage/experiments/plans/2026-10-04/full209_thread468_JPEG_counterfactual_v1/declaration.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-04/full209_thread468_JPEG_counterfactual_v1"
V1_SOURCE_SHA = "0b46a719b30e110a75dd1e47ce5576c4129a73d1a2bd59dbbfc982a8c888c1c9"
V1_TEST_SHA = "6ad167690c08613b8ccfc384c59aa211aeb5368fa4e758d9d880285793d659f2"
THREAD_ARMS, CONTEXTS, PAIRS = (4, 6, 8), 888, 30
RAW_SHAPES = {"curvature_knots": (1, 33), "preferred_speed_knots": (1, 33),
    "intent_logits": (1, 3), "stop_distance_raw": (1, 1), "acceleration_prediction": (1, 1)}
RAW_ATOL, RAW_RTOL = 1e-6, 1e-6
XY_LIMIT_M, SPEED_LIMIT_MPS, ORDINARY_SUPPORT_M = .05, .1, 18.
MAX_ARM_BYTES, MAX_TOTAL_BYTES = 12 * 1024**2, 48 * 1024**2


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def contract():
    return dict(thread_arms=list(THREAD_ARMS), separate_fresh_serial_process_per_arm=True,
        explicit_model="AllValidMaskedCopyRouteRecoveryTemporalModelV3 frozen V1 candidate",
        model_change_across_thread_arms=False, final_endpoint=previous.ENDPOINT,
        all_original_files=7366, all_camera_bundles=901, all_original_contexts=CONTEXTS,
        original_prefills=23, original_fresh_NN=865, context_camera_ordinals=list(range(12, 900)),
        original_V2_fresh_ordinary=802, original_V2_fresh_terminal=63,
        selected_context_indices=previous.selected_indices(), timed_calls_per_arm=PAIRS,
        first_selected_context_cold_cache=True, intermediate_or_failure_tail_cropping=False,
        timer_scope="complete original forward_cached, including all finite/state/pixel/provenance/cache/recovery-hook guards",
        timer_excludes="strict loader, reconstruction/builder, external state/input/raw comparison, hashing, decoder and archive writes",
        preload_intraop_threads=4, preload_interop_threads=1,
        strict_GOOD6000_loader_configures_threads=False,
        threads_set_once_after_strict_GOOD6000_loader=True, postloader_required_intraop_threads=4,
        interop_threads=1, per_call_threadpool_retuning=False, no_deadline_filter_or_waiver=True,
        raw_heads=dict(RAW_SHAPES), raw_absolute_tolerance=RAW_ATOL, raw_relative_tolerance=RAW_RTOL,
        cross_thread_bitwise_equality_claimed=False, all_raw_heads_finite_and_strict_allclose_required=True,
        ordinary_decoder_support_minimum_m=ORDINARY_SUPPORT_M,
        ordinary_decoder_XY_pointwise_Euclidean_limit_m=XY_LIMIT_M,
        ordinary_decoder_XY_chord_speed_absolute_limit_mps=SPEED_LIMIT_MPS,
        ordinary_GO_decoder_screen_includes_prefill_counterfactuals_separately=True,
        terminal_contexts_retained_but_terminal_decoder_proof_unverified=True,
        complete_safety_engine_and_control_parity_verified=False,
        actual_V4_reference_ordinary817_terminal66_is_not_V2_context_denominator=True,
        bounded_archive="all888 raw head values, complete input and retained feature SHA, state/cache/decoder statuses; no full CNN/image value export",
        original_nonimage_inputs_reconstructible_from_sealed_V2=True,
        every_image_and_retained_cache_feature_byte_hashed=True,
        maximum_archive_bytes_per_arm=MAX_ARM_BYTES, maximum_total_archive_bytes=MAX_TOTAL_BYTES,
        whole_source_hashing_and_first_arm_OS_cache_warming_disclosed=True,
        lossy_JPEG_reconstruction=True, original_actual_BGRA_or_raw_NN_equality_claimed=False,
        actual_sensor_latency_gain_proven=False, actual_runtime_activated=False,
        model_promoted=False, main_functions_approved=0, subfunctions_approved=0)


def source_snapshot():
    require(digest(Path(previous.__file__)) == V1_SOURCE_SHA and digest(previous.TEST) == V1_TEST_SHA,
        "frozen original masked-copy V1 source/review test changed")
    pins = previous.source_snapshot()
    pins.update({str(path.relative_to(ROOT)): digest(path) for path in (HERE, TEST)})
    return pins


def snapshot():
    value = dict(sources=source_snapshot(), inputs=previous.recorded.input_snapshot(),
        assets=previous.asset_snapshot(), environment=previous.environment(), contract=contract())
    require(value["inputs"]["all_original_file_count"] == 7366, "all7366 original V2 files required")
    cameras = previous.recorded.camera_catalog()
    prefills, learned = previous.jsonl(EPISODE / "temporal_prefill.jsonl"), previous.jsonl(EPISODE / "learned_control.jsonl")
    events = original_decoder_provenance(previous.event_catalog(cameras, prefills, learned), learned)
    value["complete_original_event_catalog_sha256"] = canonical_sha(events)
    value["environment"]["inherited_CPU_affinity"] = sorted(os.sched_getaffinity(0))
    require(len(value["environment"]["inherited_CPU_affinity"]) >= max(THREAD_ARMS),
        "declared eight-thread arm requires at least eight inherited logical CPUs")
    return value, cameras, events


def original_decoder_provenance(events, learned):
    """HH_261004 - Derive ordinary/terminal roles from original V2 receipts, never from V4 or future labels."""
    fresh = {row["inference_frame"]: row for row in learned if row.get("held_between_camera_frames") is False}
    result = []
    for event in events:
        if event["kind"] == "original_prefill":
            result.append(dict(event, original_decoder_role="PREFILL_NO_ACTUAL_DECODER",
                original_support_logged=False, original_terminal_witness_logged=False))
            continue
        original = fresh[event["native_frame"]]
        receipt = original.get("navigation_terminal_research")
        supported = original.get("causal_navigation_support_m")
        require(isinstance(receipt, dict) and type(supported) in (int, float) and math.isfinite(supported)
            and receipt.get("frame_timestamp_ns") == event["anchor_timestamp_ns"]
            and receipt.get("inherited_tensor_support_m") == supported, "original V2 decoder support/frame receipt required")
        ordinary = receipt.get("branch") == "ordinary_exact_delegate"
        terminal = receipt.get("branch") == "authenticated_navigation_terminal_V2"
        require(ordinary and supported >= ORDINARY_SUPPORT_M or terminal and supported < ORDINARY_SUPPORT_M,
            "original V2 ordinary/terminal provenance disagrees with actual support")
        if terminal:
            require(isinstance(receipt.get("witness_decision"), dict)
                and receipt["witness_decision"].get("status") == "VERIFIED",
                "retain the original authentic verified terminal witness identity")
        result.append(dict(event, original_decoder_role="ACTUAL_ORDINARY" if ordinary else "ACTUAL_TERMINAL",
            original_support_logged=True, original_causal_support_m=float(supported),
            original_terminal_witness_logged=terminal, original_decoder_receipt_sha256=canonical_sha(receipt),
            replay_terminal_proof_verified=False))
    require(sum(row["original_decoder_role"] == "PREFILL_NO_ACTUAL_DECODER" for row in result) == 23
        and sum(row["original_decoder_role"] == "ACTUAL_ORDINARY" for row in result) == 802
        and sum(row["original_decoder_role"] == "ACTUAL_TERMINAL" for row in result) == 63,
        "complete original V2 census23-prefill/802-ordinary/63-terminal required")
    return result


def bounded_append(root, name, value):
    """HH_261004 - Stop with a retained failed prefix if the small declared evidence cap is exceeded."""
    path = safe_path(root / name, missing_tail=True)
    encoded = (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()
    require(len(encoded) <= 32 * 1024, "bounded per-context record required")
    inventory = previous.recorded.tree_members(root)
    limit = MAX_TOTAL_BYTES if root == OUTPUT else MAX_ARM_BYTES
    require(sum(inventory["files"].values()) + len(encoded) <= limit - 64 * 1024,
        "declared archive cap/summary reserve exhausted; no evidence dropping or retry")
    with path.open("ab" if path.exists() else "xb") as stream:
        stream.write(encoded)


def initialize_worker_runtime(torch):
    """HH_261004 - Restore V1's explicit CPU4/interop1 setup before model imports; the numeric loader does not set threads."""
    torch.set_num_threads(4)
    torch.set_num_interop_threads(1)
    require(torch.get_num_threads() == 4 and torch.get_num_interop_threads() == 1,
        "explicit pre-loader CPU4/interop1 baseline did not take effect")


def set_threads_after_load(torch, threads):
    """HH_261004 - Enforce one post-loader thread change; retain original inter-op configuration."""
    require(type(threads) is int and threads in THREAD_ARMS, "fixed CPU4/6/8 arm required")
    require(torch.get_num_threads() == 4 and torch.get_num_interop_threads() == 1,
        "strict original loader must retain the explicitly established CPU4/interop1 baseline")
    torch.set_num_threads(threads)
    require(torch.get_num_threads() == threads and torch.get_num_interop_threads() == 1,
        "post-loader thread arm did not take effect")


def raw_values(raw):
    import torch
    require(set(raw) == set(RAW_SHAPES), "complete five original raw heads required")
    for name, value in raw.items():
        require(type(value) is torch.Tensor and value.dtype == torch.float32 and value.device.type == "cpu"
            and tuple(value.shape) == RAW_SHAPES[name] and bool(torch.isfinite(value).all().item()),
            "strict finite original raw head ABI required: " + name)
    return {name: value.detach().cpu().tolist() for name, value in raw.items()}


def raw_comparison(reference, current):
    """HH_261004 - Use the original cached-output tolerances on every head; retain every maximum error."""
    import torch
    from portable_e2e.temporal_cached_encoder_v2 import CACHED_OUTPUT_ATOL, CACHED_OUTPUT_RTOL
    require((CACHED_OUTPUT_ATOL, CACHED_OUTPUT_RTOL) == (RAW_ATOL, RAW_RTOL),
        "original frozen numerical tolerance changed")
    require(set(reference) == set(current) == set(RAW_SHAPES), "complete five archived raw heads required")
    result = {}
    for name, shape in RAW_SHAPES.items():
        first, second = torch.tensor(reference[name], dtype=torch.float32), torch.tensor(current[name], dtype=torch.float32)
        require(tuple(first.shape) == tuple(second.shape) == shape and bool(torch.isfinite(first).all().item())
            and bool(torch.isfinite(second).all().item()), "finite complete archived raw ABI required")
        result[name] = dict(allclose=bool(torch.allclose(first, second, atol=CACHED_OUTPUT_ATOL, rtol=CACHED_OUTPUT_RTOL)),
            maximum_absolute_error=float((first - second).abs().max()),
            bitwise_equal_diagnostic=previous.tensor_hashes({name: first}) == previous.tensor_hashes({name: second}))
    return result


def ordinary_decoder_screen(raw, tensors):
    """HH_261004 - Decode only the authentic ordinary geometry; never synthesize terminal witnesses or support."""
    import torch
    from portable_e2e.path_time_decoder_v2 import GO, PathTimeDecoderV2
    from portable_e2e.temporal_simulation_research_v2_20261001 import causal_route_support, measured_current_speed
    value = dict(current_base_link_planar_speed_mps=None, causal_navigation_support_m=None,
        full_safety_engine_parity_verified=False, original_BGRA_proof_fabricated=False)
    try:
        current = measured_current_speed(tensors)
        value["current_base_link_planar_speed_mps"] = float(current[0])
        supported = causal_route_support(tensors["route_xy"], tensors["route_mask"])
        value["causal_navigation_support_m"] = float(supported[0])
        if float(supported[0]) < ORDINARY_SUPPORT_M:
            return dict(value, status="TERMINAL_PROOF_UNVERIFIED", ordinary_decoder_executed=False)
        decoded = PathTimeDecoderV2()(raw["curvature_knots"], raw["preferred_speed_knots"], current,
            torch.full_like(current, GO, dtype=torch.long), torch.zeros_like(current), supported)
        require(tuple(decoded["xy"].shape) == (1, 1, 64, 2) and tuple(decoded["speed"].shape) == (1, 1, 64)
            and bool(torch.isfinite(decoded["xy"]).all().item()) and bool(torch.isfinite(decoded["speed"]).all().item()),
            "finite complete ordinary decoder XY/chord-speed ABI required")
        return dict(value, status="ORDINARY_GO_COUNTERFACTUAL_DECODED", ordinary_decoder_executed=True,
            xy=decoded["xy"].detach().cpu().tolist(), chord_speed=decoded["speed"].detach().cpu().tolist())
    except Exception as error:
        # HH_261004 - Rejections retain the context and raw output; they prevent a decoder-pass claim, not its denominator.
        return dict(value, status="ORDINARY_DECODER_REJECTED", ordinary_decoder_executed=True,
            exception_type=type(error).__name__, exception=str(error))


def decoder_comparison(reference, current):
    import torch
    require(reference["status"] == current["status"] or
        "ORDINARY_DECODER_REJECTED" in (reference["status"], current["status"]), "decoder geometry scope changed across arms")
    require(reference["current_base_link_planar_speed_mps"] == current["current_base_link_planar_speed_mps"]
        and reference["causal_navigation_support_m"] == current["causal_navigation_support_m"],
        "identical actual nonimage decoder context required")
    if reference["status"] != "ORDINARY_GO_COUNTERFACTUAL_DECODED" or current["status"] != reference["status"]:
        return dict(status="UNVERIFIED_OR_REJECTED", ordinary_numeric_screen_passed=False,
            full_safety_engine_parity_verified=False)
    first, second = torch.tensor(reference["xy"], dtype=torch.float32), torch.tensor(current["xy"], dtype=torch.float32)
    fs, ss = torch.tensor(reference["chord_speed"], dtype=torch.float32), torch.tensor(current["chord_speed"], dtype=torch.float32)
    require(tuple(first.shape) == tuple(second.shape) == (1, 1, 64, 2)
        and tuple(fs.shape) == tuple(ss.shape) == (1, 1, 64) and all(bool(torch.isfinite(x).all().item()) for x in (first, second, fs, ss)),
        "complete finite archived ordinary decoder geometry required")
    xy_error, speed_error = float(torch.norm(first - second, dim=-1).max()), float((fs - ss).abs().max())
    return dict(status="ORDINARY_GO_COUNTERFACTUAL_ONLY", maximum_pointwise_XY_error_m=xy_error,
        maximum_chord_speed_error_mps=speed_error,
        ordinary_numeric_screen_passed=xy_error <= XY_LIMIT_M and speed_error <= SPEED_LIMIT_MPS,
        full_safety_engine_parity_verified=False)


def compact_cache(model):
    full = previous.cache_signature(model)
    return dict(work=full["work"], entries=[{name: row[name] for name in
        ("timestamp_ns", "pixel_sha256", "feature_sha256")} for row in full["entries"]])


def timed_forward(model, tensors, window, threads, index, timings, output):
    selected = {ordinal: pair for pair, ordinal in enumerate(previous.selected_indices())}
    started = time.perf_counter_ns() if index in selected else None
    error = None
    try:
        return model.forward_cached(tensors, window=window)
    except BaseException as caught:
        error = caught
        raise
    finally:
        if started is not None:
            elapsed = time.perf_counter_ns() - started
            row = dict(threads=threads, pair=selected[index], context_index=index, elapsed_ns=elapsed,
                elapsed_ms=elapsed / 1e6, original_first_cold_context=index == 0,
                status="COMPLETED" if error is None else "REJECTED", exception_type=type(error).__name__ if error else None,
                exception=str(error) if error else None, deadline_filter_used=False)
            timings.append(row)
            bounded_append(output, "all30_timing_rows.jsonl", row)


def worker(threads, expected_sha, lease_fd):
    # HH_261004 - The child retains the exact inherited lease; unrelated runtime owners cannot overlap this process.
    require(type(lease_fd) is int and lease_fd >= 3
        and Path(os.readlink(f"/proc/self/fd/{lease_fd}")) == previous.LOCK, "exact caller-held shared lease descriptor required")
    fcntl.flock(lease_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    require(digest(PLAN) == expected_sha, "prospective thread declaration SHA changed")
    before, cameras, events = snapshot()
    plan = read_json(PLAN)
    require(plan["schema"] == SCHEMA + ".prospective" and all(plan[name] == value for name, value in before.items()),
        "thread worker sources/assets/input contexts/environment changed")
    output = OUTPUT / f"CPU{threads}"
    previous.recorded.fresh_output(output)
    output.mkdir(parents=True, exist_ok=False)
    timings, completed, reconstructed = [], 0, 0
    try:
        import torch
        initialize_worker_runtime(torch)
        from portable_e2e.simulation_research import ControllerConfig
        from portable_e2e.temporal_single_copy_history_research_20261001 import SingleCopyOwnedHeadroomPolicyV2
        from portable_e2e.temporal_all_valid_masked_copy_research_v1_20261004 import AllValidMaskedCopyRouteRecoveryTemporalModelV3
        loaded = previous.models()
        model = loaded["B"]
        require(type(model) is AllValidMaskedCopyRouteRecoveryTemporalModelV3, "explicit frozen masked-copy V1 model required")
        del loaded
        set_threads_after_load(torch, threads)
        baseline = model._state_identity()
        controller = ControllerConfig(**read_json(EPISODE / "learned_summary.json")["controller"])
        by_ordinal = {row["camera_ordinal"]: (index, row) for index, row in enumerate(events)}
        builder = None
        with torch.no_grad():
            for ordinal, frame, past, identity in previous.reconstructed_frames(cameras, previous.jsonl(EPISODE / "states.jsonl")):
                reconstructed += 1
                if ordinal not in by_ordinal:
                    continue
                index, event = by_ordinal[ordinal]
                bounded_append(output, "context_attempts.jsonl", dict(context_index=index, camera_ordinal=ordinal, status="STARTED"))
                if builder is None:
                    builder = SingleCopyOwnedHeadroomPolicyV2(model, controller, longitudinal_pi=True, **identity)
                require(torch.get_num_threads() == threads and torch.get_num_interop_threads() == 1,
                    "threadpool changed after one-time post-loader configuration")
                tensors, window = builder.build_inputs(frame, past_frames=past, now_ns=frame.timestamp_ns,
                    ego_stamp_ns=frame.timestamp_ns, route_stamp_ns=frame.timestamp_ns)
                input_sha = previous.tensor_hashes(tensors)
                require(all(input_sha[name] == sha for name, sha in event["original_input_sha256"].items() if name != "images")
                    and bool(tensors["time_valid"].all().item()), "every actual nonimage context and all-valid causal slot must match")
                raw = timed_forward(model, tensors, window, threads, index, timings, output)
                archive_raw = raw_values(raw)
                require(input_sha == previous.tensor_hashes(tensors) and model._state_identity() == baseline
                    and not model._recovery_active and model._recovery_features is None and model.image_encoder.request is None,
                    "input, learned state/topology or serial recovery/cache cleanup changed")
                cache = compact_cache(model)
                row = dict(context_index=index, camera_ordinal=ordinal, native_frame=event["native_frame"],
                    kind=event["kind"], anchor_timestamp_ns=event["anchor_timestamp_ns"], threads=threads,
                    original_decoder_provenance={name: value for name, value in event.items() if name.startswith("original_decoder")
                        or name in ("original_causal_support_m", "original_terminal_witness_logged", "replay_terminal_proof_verified")},
                    status="COMPLETED", input_sha256=input_sha, cache_window_sha256=canonical_sha(asdict(window)),
                    raw_output=archive_raw, raw_output_sha256=previous.tensor_hashes(raw), cache=cache,
                    actual_nonimage_input_SHA_equal=True, learned_state_and_inputs_unchanged=True,
                    ordinary_decoder=ordinary_decoder_screen(raw, tensors),
                    original_actual_BGRA_or_raw_NN_equality_claimed=False)
                if event["kind"] == "original_fresh_NN":
                    require(row["ordinary_decoder"]["causal_navigation_support_m"] == event["original_causal_support_m"],
                        "pure decoder geometry must equal the original V2 support receipt")
                require(input_sha == previous.tensor_hashes(tensors) and model._state_identity() == baseline,
                    "pure decoder changed an original input or learned model state")
                bounded_append(output, "all888_context_rows.jsonl", row)
                completed += 1
        require(reconstructed == 901 and completed == CONTEXTS and len(timings) == PAIRS
            and model._state_identity() == baseline and torch.get_num_threads() == threads, "complete unchanged thread-arm denominator required")
        after, _, _ = snapshot()
        require(before == after and "carla" not in sys.modules, "all original/source/asset identities must remain unchanged")
        summary = dict(schema=SCHEMA + ".arm", threads=threads, completed_contexts=completed,
            reconstructed_camera_bundles=reconstructed, timed_calls=len(timings), timing=previous.recorded.stats([row["elapsed_ms"] for row in timings]),
            all_original_input_and_state_guards_unchanged=True, source_and_input_and_assets_unchanged=True,
            torch_version=torch.__version__, interop_threads=torch.get_num_interop_threads(),
            model_class=type(model).__name__, same_explicit_masked_copy_V1=True,
            all_context_rows_sha256=digest(output / "all888_context_rows.jsonl"),
            all_timing_rows_sha256=digest(output / "all30_timing_rows.jsonl"),
            full_safety_engine_parity_verified=False, model_promoted=False)
        write_new_json(output / "summary.json", summary)
        return 0
    except BaseException as error:
        write_new_json(output / "FAILED_attempt.json", dict(schema=SCHEMA + ".arm.failed", threads=threads,
            planned_contexts=CONTEXTS, completed_contexts=completed, planned_timed_calls=PAIRS,
            every_completed_or_partial_journal_retained=True, attempted_timing_rows=timings,
            exception_type=type(error).__name__, exception=str(error), no_retry=True, model_promoted=False))
        raise


def compare_arms(arms):
    require([arm["threads"] for arm in arms] == list(THREAD_ARMS), "fixed complete CPU4/6/8 order required")
    reference = arms[0]["rows"]
    require(len(reference) == CONTEXTS, "all888 CPU4 reference contexts required")
    results = []
    for arm in arms:
        require(len(arm["rows"]) == CONTEXTS, "all888 candidate contexts required")
        for first, current in zip(reference, arm["rows"]):
            require(first["context_index"] == current["context_index"]
                and first["camera_ordinal"] == current["camera_ordinal"] and first["kind"] == current["kind"]
                and first["input_sha256"] == current["input_sha256"]
                and first["cache_window_sha256"] == current["cache_window_sha256"]
                and first["original_decoder_provenance"] == current["original_decoder_provenance"],
                "identical complete causal inputs and original V2 decoder provenance required across thread arms")
            require(first["cache"]["work"] == current["cache"]["work"]
                and [(row["timestamp_ns"], row["pixel_sha256"]) for row in first["cache"]["entries"]]
                == [(row["timestamp_ns"], row["pixel_sha256"]) for row in current["cache"]["entries"]],
                "original causal cache work/source identity changed across arms")
            raw = raw_comparison(first["raw_output"], current["raw_output"])
            decoded = decoder_comparison(first["ordinary_decoder"], current["ordinary_decoder"])
            results.append(dict(threads=arm["threads"], context_index=current["context_index"], camera_ordinal=current["camera_ordinal"],
                kind=current["kind"], raw_heads=raw, all_five_raw_heads_strict_allclose=all(row["allclose"] for row in raw.values()),
                original_decoder_role=current["original_decoder_provenance"]["original_decoder_role"],
                ordinary_decoder_rejected=first["ordinary_decoder"]["status"] == "ORDINARY_DECODER_REJECTED"
                    or current["ordinary_decoder"]["status"] == "ORDINARY_DECODER_REJECTED",
                ordinary_decoder=decoded, feature_hash_difference_count=sum(a["feature_sha256"] != b["feature_sha256"] for a, b in
                    zip(first["cache"]["entries"], current["cache"]["entries"])),
                cross_thread_cached_feature_bitwise_equality_claimed=False, full_safety_engine_parity_verified=False))
    return results


def run_parent(args, lease):
    previous.recorded.fresh_output(OUTPUT)
    before, _, _ = snapshot()
    if args.mode == "declare":
        require(args.expected_declaration_sha256 is None and "torch" not in sys.modules and "carla" not in sys.modules,
            "source/metadata declaration cannot run model or simulator")
        previous.recorded.fresh_output(PLAN)
        write_new_json(PLAN, dict(schema=SCHEMA + ".prospective", provenance="HH_261004 - Fix independent thread arms, all original contexts, numerical tolerance and small evidence caps before any NN execution.", **before))
        print(json.dumps(dict(status="DECLARED_NOT_MEASURED", declaration_sha256=digest(PLAN))))
        return 0
    require(args.expected_declaration_sha256 == digest(PLAN), "exact prospective thread declaration SHA required")
    plan = read_json(PLAN)
    require(plan["schema"] == SCHEMA + ".prospective" and all(plan[name] == value for name, value in before.items()),
        "prospective inputs/sources/selection/environment changed")
    OUTPUT.mkdir(parents=True, exist_ok=False)
    arms, codes = [], []
    try:
        for threads in THREAD_ARMS:
            command = [sys.executable, str(ROOT / "tools/workspace_python.py"), "-m", MODULE, "--", "--mode", "worker",
                "--threads", str(threads), "--expected-declaration-sha256", args.expected_declaration_sha256,
                "--runtime-lease-fd", str(lease.fileno())]
            with (OUTPUT / f"CPU{threads}.stdout.log").open("xb") as stdout, (OUTPUT / f"CPU{threads}.stderr.log").open("xb") as stderr:
                result = subprocess.run(command, cwd=ROOT, env=dict(os.environ), pass_fds=(lease.fileno(),), stdout=stdout, stderr=stderr, check=False)
            codes.append(dict(threads=threads, exit_code=result.returncode))
            write_new_json(OUTPUT / f"CPU{threads}.process_receipt.json", codes[-1])
            require(result.returncode == 0, "original thread worker rejected; prefix retained and no retry")
            arm_root = OUTPUT / f"CPU{threads}"
            summary = read_json(arm_root / "summary.json")
            require(summary["threads"] == threads and digest(arm_root / "all888_context_rows.jsonl") == summary["all_context_rows_sha256"],
                "complete original worker archive changed")
            arms.append(dict(threads=threads, summary=summary, rows=previous.jsonl(arm_root / "all888_context_rows.jsonl")))
        comparison = compare_arms(arms)
        for row in comparison:
            bounded_append(OUTPUT, "all2664_comparison_rows.jsonl", row)
        after, _, _ = snapshot()
        require(before == after and sum(previous.recorded.tree_members(OUTPUT)["files"].values()) <= MAX_TOTAL_BYTES,
            "unchanged original pins and complete bounded archive required")
        summary = dict(schema=SCHEMA, provenance="HH_261004 - Report every numerical head/ordinary geometry screen and all timing samples; terminal/full-engine/live adoption remains unverified.",
            arm_summaries=[arm["summary"] for arm in arms], complete_contexts_per_arm=CONTEXTS,
            all_five_raw_heads_strict_allclose=all(row["all_five_raw_heads_strict_allclose"] for row in comparison),
            raw_absolute_tolerance=RAW_ATOL, raw_relative_tolerance=RAW_RTOL,
            ordinary_decoded_comparison_count=sum(row["ordinary_decoder"]["status"] == "ORDINARY_GO_COUNTERFACTUAL_ONLY" for row in comparison),
            ordinary_rejected_or_terminal_unverified_count=sum(row["ordinary_decoder"]["status"] == "UNVERIFIED_OR_REJECTED" for row in comparison),
            ordinary_decoder_rejection_count=sum(row["ordinary_decoder_rejected"] for row in comparison),
            original_V2_roles_per_arm=dict(prefill_no_actual_decoder=23, actual_ordinary=802, actual_terminal=63),
            all_available_ordinary_XY_speed_screens_passed=all(row["ordinary_decoder"]["ordinary_numeric_screen_passed"] for row in comparison
                if row["ordinary_decoder"]["status"] == "ORDINARY_GO_COUNTERFACTUAL_ONLY"),
            comparison_rows_sha256=digest(OUTPUT / "all2664_comparison_rows.jsonl"),
            full_safety_engine_parity_verified=False, terminal_proof_verified=False,
            source_input_assets_unchanged=True, original_actual_BGRA_or_raw_NN_equality_claimed=False,
            actual_sensor_latency_gain_proven=False, model_promoted=False, main_functions_approved=0, subfunctions_approved=0)
        write_new_json(OUTPUT / "summary.json", summary)
        print(json.dumps(summary, sort_keys=True))
        return 0 if summary["all_five_raw_heads_strict_allclose"] and summary["all_available_ordinary_XY_speed_screens_passed"] \
            and summary["ordinary_decoder_rejection_count"] == 0 and summary["ordinary_decoded_comparison_count"] > 0 else 2
    except BaseException as error:
        write_new_json(OUTPUT / "FAILED_attempt.json", dict(schema=SCHEMA + ".failed", process_receipts=codes,
            complete_prefix_arm_count=len(arms), exception_type=type(error).__name__, exception=str(error),
            every_original_worker_prefix_retained=True, no_retry=True, model_promoted=False))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("declare", "execute", "worker"), required=True)
    parser.add_argument("--expected-declaration-sha256")
    parser.add_argument("--threads", type=int, choices=THREAD_ARMS)
    parser.add_argument("--runtime-lease-fd", type=int)
    args = parser.parse_args(argv)
    if args.mode == "worker":
        require(args.threads in THREAD_ARMS and args.runtime_lease_fd is not None, "internal fixed thread/lease worker arguments required")
        return worker(args.threads, args.expected_declaration_sha256, args.runtime_lease_fd)
    require(args.threads is None and args.runtime_lease_fd is None, "thread worker arguments cannot alter parent declaration")
    with safe_path(previous.LOCK).open("rb") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            return run_parent(args, lease)
        finally:
            fcntl.flock(lease, fcntl.LOCK_UN)


if __name__ == "__main__":
    raise SystemExit(main())
