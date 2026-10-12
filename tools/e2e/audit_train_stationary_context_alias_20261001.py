"""HH_261001 - Compare original TRAIN warmup/start inputs against the frozen AH context cache; never alter labels."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from portable_e2e.dataset import FEATURE_NAMES, load_training_examples
from portable_e2e.model import ModelConfig
from portable_e2e.torch_dataset import Common10TorchDataset


ROOT = Path(__file__).resolve().parents[2]
DATASET = ROOT / "storage/datasets/prepared/carla-common10-trainval-research-20260914-v1"
PAIR = ROOT / "storage/experiments/training/2026-10-01/ah_selector_phase_ab_20261001_v1"
AUDIT = ROOT / "docs/assets/validation/2026-10-01/ah_town07_straight_research_v2/train_start_stop_audit.json"
CACHE_SHA = "763dcbb76046317e6cbc9254eec0ee4232bbf01cae4b8407a9f9cf761093e856"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def differences(a, b):
    result = {}
    for name in ("images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask"):
        x, y = a[name].numpy(), b[name].numpy()
        delta = np.abs(x.astype(np.float64) - y.astype(np.float64))
        result[name] = dict(exact_equal=bool(np.array_equal(x, y)), max_abs=float(delta.max()),
                            mean_abs=float(delta.mean()), nonidentical_fraction=float(np.mean(x != y)))
    x, y = a["images"].numpy(), b["images"].numpy()
    result["per_camera_normalized_image_mae"] = np.mean(np.abs(x-y), axis=(1, 2, 3)).tolist()
    result["ego_feature_max_abs"] = dict(zip(FEATURE_NAMES,
        np.max(np.abs(a["ego_history"].numpy()-b["ego_history"].numpy()), axis=0).tolist()))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("fresh read-only audit output required")
    torch.set_num_threads(1)
    cache_path = PAIR / "frozen_train_cache.npz"
    receipt, declaration, source_audit = read(PAIR/"cache_receipt.json"), read(PAIR/"declaration.json"), read(AUDIT)
    if sha(cache_path) != CACHE_SHA or receipt["npz_sha256"] != CACHE_SHA:
        raise ValueError("frozen AH TRAIN cache differs")
    if source_audit["dataset_manifest_sha256"] != declaration["training_manifest_sha256"]:
        raise ValueError("cache/index TRAIN manifest identities differ")
    offsets, offset = {}, 0
    for row in source_audit["episodes"]:
        offsets[row["episode_id"]] = offset
        offset += row["counts"]["all"]
    if offset != 2787:
        raise ValueError("full cache index coverage differs")
    cache = np.load(cache_path, allow_pickle=False)
    loaded = load_training_examples(DATASET, split="train", check_image_hashes=False)
    config = ModelConfig.from_mapping(read(ROOT / "learning/portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json"))
    dataset = Common10TorchDataset(loaded.examples, config, split="train", verify_image_sha256=True)
    manifest = read(DATASET / "dataset.json")
    metadata = {}
    for ref in manifest["episodes"]:
        path = DATASET / ref["manifest"]
        episode = read(path)
        if episode["split"] != "train":
            continue
        config_path = path.parent / episode["source_provenance"]["collection_config_file"]
        if sha(config_path) != episode["source_provenance"]["collection_config_sha256"]:
            raise ValueError("original collection phase metadata differs")
        phases = read(config_path)["native_result"]["capture_phases"]
        metadata[episode["episode_id"]] = dict(capture_phases=phases, source_config_sha256=sha(config_path),
                                               config_path=str(config_path.relative_to(ROOT)))
    tensors, summaries = {}, []
    for index, example in enumerate(dataset.examples):
        if example.sequence_index > 40:
            continue
        item = dataset[index]
        cache_index = offsets[example.episode_id] + example.sequence_index
        for cached, field in (("target_xy", "target_xy"), ("target_speed", "target_speed_mps"), ("valid", "target_valid")):
            if not np.array_equal(cache[cached][cache_index], item[field].numpy()):
                raise ValueError("local TRAIN example and exact cached target index differ")
        if cache["current"][cache_index] != np.float32(example.features[1]):
            raise ValueError("cached current state association differs")
        begin = metadata[example.episode_id]["capture_phases"]["driving"]["first_timestamp"]
        phase = "stationary_warmup" if example.anchor_timestamp_ns / 1e9 < begin-.001 else "driving"
        vectors = cache["fused"][cache_index].astype(np.float64)
        item["fused"] = vectors
        tensors[(example.episode_id, example.sequence_index)] = item
        summaries.append(dict(episode_id=example.episode_id, sequence_index=example.sequence_index, token=example.token,
            capture_phase=phase, anchor_timestamp_ns=example.anchor_timestamp_ns, current_features=dict(zip(FEATURE_NAMES, example.features)),
            history_valid_count=int(item["ego_history_mask"].sum()), first5_target_speed=item["target_speed_mps"][:5].tolist(),
            original_ah_selected=int(cache["logits"][cache_index].argmax()), original_ah_oracle=int(cache["costs"][cache_index].argmin()),
            camera_sha256=list(example.camera_sha256), cache_index=cache_index))
    pairs, nearest = [], []
    for episode_id in metadata:
        warmup = [row for row in summaries if row["episode_id"] == episode_id and row["capture_phase"] == "stationary_warmup"]
        queries = [row for row in summaries if row["episode_id"] == episode_id and row["sequence_index"] in (35, 37)]
        for query in queries:
            q = tensors[(episode_id, query["sequence_index"])]
            group = []
            for row in warmup:
                w = tensors[(episode_id, row["sequence_index"])]
                cosine = float(np.dot(w["fused"], q["fused"]) / (np.linalg.norm(w["fused"])*np.linalg.norm(q["fused"])))
                pair = dict(episode_id=episode_id, warmup_sequence=row["sequence_index"], driving_sequence=query["sequence_index"],
                    fused_cosine_similarity=cosine, fused_l2_distance=float(np.linalg.norm(w["fused"]-q["fused"])),
                    input_differences=differences(w, q), warmup_first5_target_speed=row["first5_target_speed"],
                    driving_first5_target_speed=query["first5_target_speed"],
                    immediate_target_contrast=row["first5_target_speed"][2] <= .05 and query["first5_target_speed"][2] > .1,
                    warmup_original_oracle=row["original_ah_oracle"], driving_original_oracle=query["original_ah_oracle"])
                group.append(pair)
            pairs.extend(group)
            closest = min(group, key=lambda row: row["fused_l2_distance"])
            full_history = [row for row in group if row["warmup_sequence"] >= 9]
            nearest.append(dict(episode_id=episode_id, driving_sequence=query["sequence_index"], closest_by_fused_l2=closest,
                full_history_minimum_cosine=min(row["fused_cosine_similarity"] for row in full_history),
                full_history_maximum_cosine=max(row["fused_cosine_similarity"] for row in full_history),
                exact_all_input_matches=sum(all(row["input_differences"][name]["exact_equal"] for name in
                    ("images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask")) for row in group)))
    report = dict(schema="portable_e2e.train_stationary_context_alias_audit.v1", neural_forward_passes=0, optimizer_steps=0,
        cached_model_checkpoint_sha256=receipt["source_checkpoint_sha256"], cache_npz_sha256=CACHE_SHA,
        index_manifest_sha256=declaration["training_manifest_sha256"], mapping_verified_against_original_targets=len(summaries),
        source_collection_metadata=metadata, model_input_fields=["images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask"],
        ego_feature_names=list(FEATURE_NAMES), capture_phase_is_model_input=False, absolute_time_is_model_input=False,
        throttle_brake_or_go_trigger_is_model_input=False, warmup_anchors=sum(row["capture_phase"] == "stationary_warmup" for row in summaries),
        anchors=summaries, comparisons=pairs, nearest_warmup_to_driving=nearest,
        exact_input_alias_not_assumed=True, feature_nearness_not_proof_of_nonlinear_nonseparability=True,
        dev_metrics_computed=False, dataset_contract_validation_may_read_nontrain_records=True,
        labels_modified=False, training_rows_discarded=0, training_data_approved=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps(dict(output=str(args.output), cached_anchors=len(summaries), warmup=report["warmup_anchors"],
                         pairs=len(pairs), neural_forward_passes=0)))


if __name__ == "__main__":
    main()
