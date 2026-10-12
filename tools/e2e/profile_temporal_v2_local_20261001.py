"""HH_261001 - Measure the new checkpoint on the local GPU without simulator or vehicle commands."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

import torch
from torch.utils.data.dataloader import default_collate

from portable_e2e.dataset import load_training_examples
from portable_e2e.temporal_dataset_v2 import TemporalCommon10TorchDataset, load_common10_temporal_metadata
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2, TemporalModelV2


ROOT = Path(__file__).resolve().parents[2]
CHECKPOINT_SHA = "6c8bfceee5b3a2a0e332666b225769af236499baac9414294c18ccc6e9833ece"
FIELDS = ("images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask",
          "time_valid", "relative_pose", "time_offsets_s")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def statistics(seconds):
    # HH_261001 - Keep all repetitions and report nearest-rank p95, not only the fastest kernel.
    return dict(count=len(seconds), mean_ms=1000*sum(seconds)/len(seconds),
                p95_ms=1000*sorted(seconds)[math.ceil(.95*len(seconds))-1],
                maximum_ms=1000*max(seconds), individual_ms=[1000*value for value in seconds])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), required=True)
    args = parser.parse_args(argv)
    if args.output.exists() or args.output.is_symlink():
        raise ValueError("new profile result required")
    if not args.output.resolve().is_relative_to(ROOT / "storage/experiments/training"):
        raise ValueError("local training diagnostics output required")
    if sha(args.checkpoint) != CHECKPOINT_SHA:
        raise ValueError("only the observed raw200 checkpoint may be profiled")
    if args.device == "cuda":
        # HH_261001 - Refuse to compete with another compute job; leave display workloads untouched.
        consumers = subprocess.check_output(["nvidia-smi", "--query-compute-apps=pid", "--format=csv,noheader,nounits"],
                                           text=True, timeout=15).strip()
        if consumers:
            raise ValueError("local GPU has other compute consumers; do not interrupt them")
        if os.environ.get("CUDA_VISIBLE_DEVICES") != "0":
            raise ValueError("profile must explicitly select local GPU0")
    torch.set_num_threads(1)
    loaded = load_training_examples(args.data, split="train", mode="planning", check_image_hashes=True)
    metadata = load_common10_temporal_metadata(args.data, split="train", expected_tokens=[row.token for row in loaded.examples])
    dataset = TemporalCommon10TorchDataset(loaded.examples, raw_metadata=metadata, split="train", verify_image_sha256=True)
    index = next(i for i, row in enumerate(dataset.examples) if row.features[1] > .05 and bool(dataset[i]["time_valid"].all()))
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    for name, digest in checkpoint["source_sha256"].items():
        if sha(ROOT / name) != digest:
            raise ValueError("checkpoint source differs: " + name)
    if args.device == "cuda" and (not torch.cuda.is_available() or torch.cuda.device_count() != 1):
        raise ValueError("exactly one local CUDA device required")
    device = torch.device("cuda:0" if args.device == "cuda" else "cpu")
    model = TemporalModelV2(TemporalModelConfigV2.from_mapping(checkpoint["model_config"])).to(device)
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    model.eval()
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True

    def synchronize():
        if args.device == "cuda":
            torch.cuda.synchronize()

    def inputs():
        batch = default_collate([dataset[index]])
        return {name: batch[name].to(device) for name in FIELDS}

    resident = inputs()
    with torch.no_grad():
        for _ in range(10):
            model(**resident)
        synchronize()
        resident_times, decode_transfer_times = [], []
        for _ in range(30):
            started = time.monotonic()
            output = model(**resident)
            synchronize()
            resident_times.append(time.monotonic()-started)
        for _ in range(30):
            started = time.monotonic()
            output = model(**inputs())
            synchronize()
            decode_transfer_times.append(time.monotonic()-started)
    if not all(bool(torch.isfinite(value).all()) for value in output.values()):
        raise ValueError("nonfinite raw inference")
    result = dict(schema="portable_e2e.temporal_v2_local_raw_latency.20261001.v1",
        status="RAW_INFERENCE_PROFILE_COMPLETE_NOT_RUNTIME_APPROVAL", device=args.device,
        gpu=torch.cuda.get_device_name(0) if args.device == "cuda" else None,
        cpu_threads=torch.get_num_threads(), compiled_cuda=torch.version.cuda,
        torch_version=torch.__version__, checkpoint_sha256=CHECKPOINT_SHA,
        source_hashes_match=True, profile_source_sha256=sha(Path(__file__)),
        dataset_fingerprint_sha256=dataset.fingerprint_sha256, token=dataset.examples[index].token,
        input_shape=list(resident["images"].shape), image_history_recomputed=True, online_image_cache=False,
        warmup_calls=10, raw_device_resident=statistics(resident_times),
        repeated_same_anchor_jpeg_hash_decode_collate_transfer_raw_forward=statistics(decode_transfer_times),
        warm_filesystem_cache=True, sensor_capture_included=False, temporal_decoder_included=False,
        control_and_ACK_included=False, autoware_included=False, vehicle_control_approved=False,
        ten_hz_end_to_end_proven=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({name: value for name, value in result.items() if name not in (
        "raw_device_resident", "repeated_same_anchor_jpeg_hash_decode_collate_transfer_raw_forward")}, indent=2))
    for name in ("raw_device_resident", "repeated_same_anchor_jpeg_hash_decode_collate_transfer_raw_forward"):
        print(name, json.dumps({key: value for key, value in result[name].items() if key != "individual_ms"}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
