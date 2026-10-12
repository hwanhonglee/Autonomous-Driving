"""HH_261001 - Measure both caches on real serialized inputs without skipping file-byte verification."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import torch
from torch.utils.data.dataloader import default_collate

from portable_e2e.dataset import load_training_examples
from portable_e2e.temporal_dataset_v2 import TemporalCommon10TorchDataset, load_common10_temporal_metadata
from portable_e2e.temporal_decoded_cache_v2 import DecodedCacheTemporalDatasetV2
from portable_e2e.temporal_cached_encoder_v2 import CachedTemporalModelV2, CACHED_OUTPUT_ATOL, CACHED_OUTPUT_RTOL
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2, TemporalModelV2
from scripts.e2e.profile_temporal_v2_local_20261001 import CHECKPOINT_SHA, FIELDS, sha, statistics
from scripts.e2e.profile_temporal_v2_cached_stream_20261001 import window


ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.is_symlink():
        raise ValueError("fresh combined profile required")
    if not args.output.resolve().is_relative_to(ROOT / "storage/experiments/training"):
        raise ValueError("local diagnostic result only")
    if sha(args.checkpoint) != CHECKPOINT_SHA:
        raise ValueError("fixed raw200 checkpoint differs")
    torch.set_num_threads(1)
    loaded = load_training_examples(args.data, split="train", mode="planning", check_image_hashes=True)
    metadata = load_common10_temporal_metadata(args.data, split="train", expected_tokens=[row.token for row in loaded.examples])
    original = TemporalCommon10TorchDataset(loaded.examples, raw_metadata=metadata, split="train", verify_image_sha256=True)
    optimized = DecodedCacheTemporalDatasetV2(loaded.examples, raw_metadata=metadata, split="train",
                                            mode="evaluation", verify_image_sha256=True)
    if original.fingerprint_sha256 != optimized.fingerprint_sha256:
        raise ValueError("pixel cache changed the scientific dataset fingerprint")
    first = next(i for i, row in enumerate(original.examples) if row.features[1] > .05 and bool(original[i]["time_valid"].all()))
    indices = list(range(first, first+40))
    initial = original.examples[first]
    for previous, current in zip(indices, indices[1:]):
        left, right = original.examples[previous], original.examples[current]
        if (right.episode_id != initial.episode_id or right.sequence_index != left.sequence_index+1
                or abs(right.anchor_timestamp_ns-left.anchor_timestamp_ns-100_000_000)>2):
            raise ValueError("first40 anchors are not a consecutive10Hz stream")
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    for name, digest in checkpoint["source_sha256"].items():
        if sha(ROOT/name) != digest:
            raise ValueError("original checkpoint source changed: "+name)
    config = TemporalModelConfigV2.from_mapping(checkpoint["model_config"])
    full, cached = TemporalModelV2(config), CachedTemporalModelV2(config)
    full.load_state_dict(checkpoint["model_state_dict"], strict=True)
    cached.load_state_dict(checkpoint["model_state_dict"], strict=True)
    full.eval(); cached.eval()
    source_names = ("learning/portable_e2e/temporal_cached_encoder_v2.py", "learning/portable_e2e/temporal_decoded_cache_v2.py",
        "tools/e2e/profile_temporal_v2_local_20261001.py", "tools/e2e/profile_temporal_v2_cached_stream_20261001.py",
        "tools/e2e/profile_temporal_v2_decoded_stream_20261001.py")
    sources = {**checkpoint["source_sha256"], **{name:sha(ROOT/name) for name in source_names}}
    ledger = []
    with torch.no_grad():
        prepared = default_collate([original[first]])
        for _ in range(10):
            full(**{name:prepared[name] for name in FIELDS})
        for index in indices:
            baseline_batch = default_collate([original[index]])
            baseline = full(**{name:baseline_batch[name] for name in FIELDS})
            started = time.monotonic()
            batch = default_collate([optimized[index]])
            decoded_seconds = time.monotonic()-started
            inputs = {name:batch[name] for name in FIELDS}
            tick = time.monotonic()
            reused = cached.forward_cached(inputs, window=window(optimized,index))
            raw_seconds = time.monotonic()-tick
            elapsed = time.monotonic()-started
            # HH_261001 - Dataset pixel/cache outputs remain exact; CNN batch-size changes use declared tolerance.
            if not all(torch.equal(batch[name], baseline_batch[name]) for name in FIELDS):
                raise ValueError("decoded cache altered model inputs")
            if not all(torch.allclose(baseline[name],reused[name],atol=CACHED_OUTPUT_ATOL,rtol=CACHED_OUTPUT_RTOL) for name in baseline):
                raise ValueError("combined cache changed raw outputs")
            ledger.append(dict(token=optimized.examples[index].token,
                maximum_output_delta=max(float((baseline[name]-reused[name]).abs().max()) for name in baseline),
                input_seconds=decoded_seconds, raw_seconds=raw_seconds, total_seconds=elapsed,
                feature_cache=asdict(cached.cache_stats), decoded_cache=asdict(optimized.cache_stats)))
    if not all(sha(ROOT/name)==digest for name,digest in sources.items()):
        raise ValueError("source changed during combined profile")
    steady=ledger[2:]
    result=dict(schema="portable_e2e.temporal_v2_combined_actual_stream.20261001.v1",
        status="DECODED_FEATURE_CACHE_EQUIVALENCE_PASS_NOT_RUNTIME_APPROVAL", checkpoint_sha256=CHECKPOINT_SHA,
        source_sha256=sources, dataset_fingerprint_sha256=original.fingerprint_sha256,
        device="cpu",cpu_threads=1,torch_version=torch.__version__,actual_stream_anchors=40,
        all_model_input_tensors_exactly_equal=True, maximum_raw_output_delta=max(row["maximum_output_delta"] for row in ledger),
        equivalence_atol=CACHED_OUTPUT_ATOL,equivalence_rtol=CACHED_OUTPUT_RTOL,
        steady_verified_input=statistics([row["input_seconds"] for row in steady]),
        steady_cached_raw=statistics([row["raw_seconds"] for row in steady]),
        steady_verified_input_plus_cached_raw=statistics([row["total_seconds"] for row in steady]),
        file_bytes_verified_on_every_access=True,warm_filesystem_cache=True,
        sensor_capture_included=False,temporal_decoder_included=False,control_ACK_included=False,
        autoware_included=False,ten_hz_end_to_end_proven=False,vehicle_control_approved=False,ledger=ledger)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result,stream,indent=2,allow_nan=False);stream.write("\n")
    concise={key:value for key,value in result.items() if key not in ("source_sha256","ledger")}
    for key in ("steady_verified_input","steady_cached_raw","steady_verified_input_plus_cached_raw"):
        concise[key]={name:value for name,value in result[key].items() if name!="individual_ms"}
    print(json.dumps(concise,indent=2))


if __name__ == "__main__":
    main()
