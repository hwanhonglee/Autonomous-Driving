"""HH_261001 - Compare cached/full raw V2 on one real causal stream and the completed checkpoint."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import torch
from torch.utils.data.dataloader import default_collate

from portable_e2e.dataset import load_training_examples
from portable_e2e.temporal_dataset_v2 import (TIME_OFFSETS_NS, TemporalCommon10TorchDataset,
    load_common10_temporal_metadata)
from portable_e2e.temporal_cached_encoder_v2 import (CameraFrameIdentityV2, TemporalCacheWindowV2,
    CachedTemporalModelV2, CACHED_OUTPUT_ATOL, CACHED_OUTPUT_RTOL)
from portable_e2e.temporal_model_v2 import TemporalModelV2, TemporalModelConfigV2
from scripts.e2e.profile_temporal_v2_local_20261001 import CHECKPOINT_SHA, FIELDS, sha, statistics


ROOT = Path(__file__).resolve().parents[2]


def window(dataset, index):
    example = dataset.examples[index]
    frames = []
    for offset in TIME_OFFSETS_NS:
        past = dataset._past_index(example, offset)
        frames.append(None if past is None else CameraFrameIdentityV2(
            dataset.examples[past].anchor_timestamp_ns, dataset.examples[past].camera_sha256))
    metadata = dataset.raw_metadata[example.token]
    return TemporalCacheWindowV2(example.episode_id, dataset.split, example.rig_id,
                                  metadata.sample_jsonl_sha256, example.anchor_timestamp_ns, tuple(frames))


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.is_symlink():
        raise ValueError("fresh stream profile required")
    if not args.output.resolve().is_relative_to(ROOT / "storage/experiments/training"):
        raise ValueError("local diagnostic output only")
    if sha(args.checkpoint) != CHECKPOINT_SHA:
        raise ValueError("fixed completed checkpoint differs")
    torch.set_num_threads(1)
    loaded = load_training_examples(args.data, split="train", mode="planning", check_image_hashes=True)
    metadata = load_common10_temporal_metadata(args.data, split="train", expected_tokens=[row.token for row in loaded.examples])
    dataset = TemporalCommon10TorchDataset(loaded.examples, raw_metadata=metadata, split="train", verify_image_sha256=True)
    first = next(i for i, row in enumerate(dataset.examples) if row.features[1] > .05 and bool(dataset[i]["time_valid"].all()))
    indices = list(range(first, first+40))
    # HH_261001 - Predeclare the first40 consecutive observed moving-stream anchors, not favorable outputs.
    initial = dataset.examples[first]
    for previous, current in zip(indices, indices[1:]):
        left, right = dataset.examples[previous], dataset.examples[current]
        if (right.episode_id != initial.episode_id or right.sequence_index != left.sequence_index+1
                or abs(right.anchor_timestamp_ns-left.anchor_timestamp_ns-100_000_000) > 2):
            raise ValueError("fixed40 anchors must form one recorded10Hz stream")
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    for name, digest in checkpoint["source_sha256"].items():
        if sha(ROOT / name) != digest:
            raise ValueError("checkpoint source changed: "+name)
    configuration = TemporalModelConfigV2.from_mapping(checkpoint["model_config"])
    full, cached = TemporalModelV2(configuration), CachedTemporalModelV2(configuration)
    full.load_state_dict(checkpoint["model_state_dict"], strict=True)
    cached.load_state_dict(checkpoint["model_state_dict"], strict=True)
    full.eval(); cached.eval()
    sources = dict(checkpoint["source_sha256"])
    for name in ("learning/portable_e2e/temporal_cached_encoder_v2.py",
                 "tools/e2e/profile_temporal_v2_local_20261001.py",
                 "tools/e2e/profile_temporal_v2_cached_stream_20261001.py"):
        sources[name] = sha(ROOT / name)
    ledger = []
    with torch.no_grad():
        warm = default_collate([dataset[first]])
        inputs = {name: warm[name] for name in FIELDS}
        for _ in range(10):
            full(**inputs)
        for index in indices:
            started = time.monotonic()
            batch = default_collate([dataset[index]])
            decoded_seconds = time.monotonic()-started
            inputs = {name: batch[name] for name in FIELDS}
            identity = window(dataset, index)
            started = time.monotonic()
            baseline = full(**inputs)
            full_seconds = time.monotonic()-started
            started = time.monotonic()
            reused = cached.forward_cached(inputs, window=identity)
            cached_seconds = time.monotonic()-started
            deltas = {name: float((baseline[name]-reused[name]).abs().max()) for name in baseline}
            if not all(torch.allclose(baseline[name], reused[name], atol=CACHED_OUTPUT_ATOL,
                                      rtol=CACHED_OUTPUT_RTOL) for name in baseline):
                raise ValueError("cached/full learned-checkpoint outputs disagree")
            ledger.append(dict(token=dataset.examples[index].token,
                anchor_timestamp_ns=identity.anchor_timestamp_ns, output_deltas=deltas,
                input_hash_decode_collate_seconds=decoded_seconds, full_raw_seconds=full_seconds,
                cached_raw_seconds=cached_seconds, cache=asdict(cached.cache_stats)))
    if not all(sha(ROOT / name) == expected for name, expected in sources.items()):
        raise ValueError("source changed during stream profile")
    steady = ledger[2:]
    result = dict(schema="portable_e2e.temporal_v2_cached_actual_stream.20261001.v1",
        status="CACHED_FULL_RAW_EQUIVALENCE_PASS_NOT_RUNTIME_APPROVAL", checkpoint_sha256=CHECKPOINT_SHA,
        source_sha256=sources, dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        device="cpu", torch_version=torch.__version__, cpu_threads=1,
        source_stream_anchors=40, warmup_full_calls=10,
        equivalence_atol=CACHED_OUTPUT_ATOL, equivalence_rtol=CACHED_OUTPUT_RTOL,
        maximum_raw_output_absolute_delta=max(max(row["output_deltas"].values()) for row in ledger),
        cold_or_first_parity_cnn_images=[row["cache"]["cnn_images"] for row in ledger[:2]],
        steady_cnn_images=[row["cache"]["cnn_images"] for row in steady],
        full_raw_same_steady_anchors=statistics([row["full_raw_seconds"] for row in steady]),
        cached_raw_steady=statistics([row["cached_raw_seconds"] for row in steady]),
        decode_collate_plus_cached_steady=statistics([row["input_hash_decode_collate_seconds"]+row["cached_raw_seconds"] for row in steady]),
        decoded_image_cache=False, sensor_capture_included=False, temporal_decoder_included=False,
        control_ACK_included=False, autoware_included=False, vehicle_control_approved=False,
        ten_hz_end_to_end_proven=False, ledger=ledger)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    concise = {key: value for key, value in result.items() if key not in ("ledger", "source_sha256")}
    for key in ("full_raw_same_steady_anchors", "cached_raw_steady", "decode_collate_plus_cached_steady"):
        concise[key] = {name: value for name, value in result[key].items() if name != "individual_ms"}
    print(json.dumps(concise, indent=2))


if __name__ == "__main__":
    main()
