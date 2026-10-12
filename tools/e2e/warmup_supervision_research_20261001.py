"""HH_261001 - Provenance-only warmup anchor weights; no target/history rewrites."""

import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from scripts.e2e import run_ah_initial_acceleration_ab_20261001 as previous
from scripts.e2e import selector_phase_research_20261001 as phase


AUDIT_RELATIVE = "docs/assets/validation/2026-10-01/selector_phase_ab/train_stationary_context_alias.json"
AUDIT_SHA = "1d099dfaf043feb28640e86cd5e9057753148d59eed1d84072534056043abc55"
CACHE_SHA = "763dcbb76046317e6cbc9254eec0ee4232bbf01cae4b8407a9f9cf761093e856"
MANIFEST_SHA = "8c71781052d0a2f355dee35dacae21f8e630f197ab7f6d6fd1fe14ac0d442957"
ACTIVE_WEIGHT = 11996 / 11549
SKIP_ONE_BASED = (1693,)


def digest_json(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def pinned(path, expected):
    previous.base.require(previous.base.digest(path) == expected, "provenance bytes differ: " + str(path))
    return previous.base.read_json(path)


def ns(timestamp):
    return int(round(float(timestamp) * 1_000_000_000))


def classify_anchor(timestamp_ns, phases):
    """HH_261001 - Half-open source-owned phase boundaries, never inferred from speed or labels."""
    driving, warmup, tail = (phases.get(name, {}) for name in ("driving", "stationary_warmup", "stationary_tail"))
    if driving.get("first_timestamp") is None or driving.get("last_timestamp") is None:
        raise ValueError("explicit driving timestamps required")
    begin, end = ns(driving["first_timestamp"]), ns(driving["last_timestamp"])
    if begin > end:
        raise ValueError("nonmonotonic driving interval")
    has_warmup = int(warmup.get("state_count", 0)) > 0
    if has_warmup:
        if warmup.get("control_source") != "full_service_brake":
            raise ValueError("warmup owner is not the declared external full-service brake")
        start, last = ns(warmup["first_timestamp"]), ns(warmup["last_timestamp"])
        if not start <= last < begin:
            raise ValueError("warmup interval overlaps driving")
        if start <= timestamp_ns < begin:
            return "warmup"
    if timestamp_ns < begin:
        raise ValueError("unexplained pre-driving anchor")
    if timestamp_ns <= end:
        return "driving"
    if int(tail.get("state_count", 0)) > 0 and ns(tail["first_timestamp"]) <= timestamp_ns <= ns(tail["last_timestamp"]):
        return "tail"
    raise ValueError("anchor outside declared acquisition phases")


def make_provenance(dataset_path, examples, root, cache_path):
    manifest = pinned(dataset_path / "dataset.json", MANIFEST_SHA)
    alias = pinned(root / AUDIT_RELATIVE, AUDIT_SHA)
    expected_warmup = {(r["episode_id"], r["token"], r["sequence_index"], r["anchor_timestamp_ns"])
                       for r in alias["anchors"] if r["capture_phase"] == "stationary_warmup"}
    records, sources = {}, {}
    for reference in manifest["episodes"]:
        path = dataset_path / reference["manifest"]
        episode = pinned(path, reference["sha256"])
        if episode["split"] != "train":
            continue
        provenance = episode["source_provenance"]
        config_path = path.parent / provenance["collection_config_file"]
        config = pinned(config_path, provenance["collection_config_sha256"])
        phases = config["native_result"]["capture_phases"]
        source_warmup = phases.get("stationary_warmup", {})
        if int(source_warmup.get("state_count", 0)) > 0:
            schedule = config["native_capture_contract"]["capture_phase_schedule"]
            if (schedule["order"] != ["stationary_warmup", "driving", "stationary_tail"]
                    or source_warmup["state_count"] != schedule["stationary_warmup"]["scheduled_ticks"]
                    or source_warmup["control_source"] != schedule["stationary_warmup"]["control_source"]):
                raise ValueError("source warmup schedule and observed ownership differ")
        sample_path = path.parent / episode["sample_jsonl"]
        previous.base.require(previous.base.digest(sample_path) == episode["sample_jsonl_sha256"], "annotation bytes differ")
        sources[episode["episode_id"]] = dict(episode_manifest_sha256=reference["sha256"],
            collection_config_sha256=provenance["collection_config_sha256"],
            sample_jsonl_sha256=episode["sample_jsonl_sha256"], capture_phases=phases)
        for line in sample_path.read_text().splitlines():
            sample = json.loads(line)
            key = (episode["episode_id"], sample["sample_id"], sample["sequence_index"], sample["anchor_timestamp_ns"])
            if key in records:
                raise ValueError("ambiguous original anchor association")
            kind = classify_anchor(key[-1], phases)
            tail = phases.get("stationary_tail", {})
            future_tail = (kind == "driving" and tail.get("first_timestamp") is not None
                           and key[-1] + 6_400_000_000 >= ns(tail["first_timestamp"]))
            records[key] = dict(episode_id=key[0], token=key[1], sequence_index=key[2], anchor_timestamp_ns=key[3],
                capture_phase=kind, warmup_off=kind == "warmup", future_crosses_imposed_tail=bool(future_tail),
                collection_config_sha256=provenance["collection_config_sha256"])
    rows = []
    for index, example in enumerate(examples):
        key = (example.episode_id, example.token, example.sequence_index, example.anchor_timestamp_ns)
        if key not in records:
            raise ValueError("loaded TRAIN identity absent from pinned provenance")
        row = dict(records.pop(key), index=index)
        if row["warmup_off"] != (key in expected_warmup):
            raise ValueError("independent audited warmup identities differ")
        rows.append(row)
    if records or len(rows) != 2787 or sum(row["warmup_off"] for row in rows) != 105:
        raise ValueError("exact TRAIN/provenance coverage differs")
    previous.base.require(previous.base.digest(cache_path) == CACHE_SHA, "protected original AH TRAIN cache differs")
    with np.load(cache_path, allow_pickle=False) as cache:
        for i, example in enumerate(examples):
            target = np.array([0. if v is None else v for v in example.target_speed_mps], dtype=np.float32)
            valid = np.array([v is not None for v in example.targets_xy], dtype=bool)
            if (not np.array_equal(target, cache["target_speed"][i]) or not np.array_equal(valid, cache["valid"][i])
                    or np.float32(example.features[1]) != cache["current"][i]):
                raise ValueError("protected cache association differs from loaded TRAIN")
        phases = phase.phase_ids(torch.from_numpy(cache["current"]), torch.from_numpy(cache["target_speed"]),
            torch.from_numpy(cache["valid"]), torch.from_numpy(cache["costs"]).argmin(1))
    off = torch.tensor([row["warmup_off"] for row in rows], dtype=torch.bool)
    if (int((phases == 0).sum()) != 200 or bool(off[phases == 0].any())
            or torch.bincount(phases[off], minlength=4).tolist() != [0, 0, 105, 0]):
        raise ValueError("original STOP/full-hold or warmup stratum protection differs")
    batches = phase.schedule(len(rows))
    skipped = [i for i, batch in enumerate(batches, 1) if all(bool(off[j]) for j in batch)]
    exposures = sum(int(off[j]) for batch in batches for j in batch)
    if skipped != list(SKIP_ONE_BASED) or exposures != 451 or tuple(batches[1692]) != (617, 619, 322, 317):
        raise ValueError("predeclared shared schedule/skip differs")
    proof = dict(rows=rows, sources=sources, warmup_mask_sha256=digest_json(off.tolist()),
        exposure_order_sha256=digest_json(batches), schedule=batches, skipped_schedule_steps_one_based=skipped,
        phase_ids=phases.tolist(), warmup_count=105, active_count=2682,
        tail_anchor_count=sum(row["capture_phase"] == "tail" for row in rows),
        driving_future_crosses_tail_count=sum(row["future_crosses_imposed_tail"] for row in rows),
        active_weight=ACTIVE_WEIGHT, original_stop_or_full_hold_count=200)
    return off, batches, proof


def weighted_original_loss(xy, speed, logits, batch, weights):
    """HH_261001 - Apply one scalar per unchanged native loss; no masked target or oracle redefinition."""
    if weights.shape != (len(xy),) or not bool(torch.isfinite(weights).all()) or bool((weights < 0).any()):
        raise ValueError("finite nonnegative anchor weights required")
    losses, nears, oracles = [], [], []
    for i in range(len(xy)):
        s = slice(i, i+1)
        loss = previous.base.trajectory_loss(xy[s], speed[s], logits[s], batch["target_xy"][s],
            batch["target_speed_mps"][s], batch["target_valid"][s], previous.base.LOSS_CONFIG,
            target_yaw=batch["target_yaw_rad"][s])
        near = previous.near_horizon_loss(xy[s], speed[s], batch["target_xy"][s], batch["target_speed_mps"][s],
            batch["target_valid"][s], loss["oracle_candidate_index"])
        losses.append(loss["loss"]); nears.append(near); oracles.append(loss["oracle_candidate_index"])
    original, near = torch.stack(losses), torch.stack(nears)
    total = ((original + .5 * near) * weights.detach()).mean()
    return total, (original * weights).mean(), (near * weights).mean(), torch.cat(oracles)
