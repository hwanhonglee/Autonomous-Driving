"""HH_261001 - Screen true-label balanced replay without changing frozen V2 scientific modules.

Exactly 3000 new updates mix two V4 joint examples, one legacy curved and one
legacy straight raw-only example. This nine-episode research fit is not main
training, independent validation, STOP training, or permission to control a car.
"""
from __future__ import annotations

import argparse
import ast
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import random
import shutil
import time

# HH_261001 - Spawn workers reconstruct only canonical import paths, never a CUDA context.
if __name__ == "__mp_main__":
    import importlib.util
    _entry = Path(__file__).resolve().parents[1] / "workspace_python.py"
    _spec = importlib.util.spec_from_file_location("hh_balanced_worker_workspace", _entry)
    _module = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_module)
    _module.configure(Path(__file__).resolve().parents[2])

import numpy as np
import torch
from torch.utils.data import ConcatDataset, DataLoader
from torch.utils.data.dataloader import default_collate

from portable_e2e.deterministic_linear_interpolation_v2 import KERNEL_ID, DeterministicPathTimeDecoderV2
from portable_e2e.path_time_decoder_v2 import GO
from portable_e2e.temporal_dataset_v2 import spatial_expert_labels
from portable_e2e.temporal_losses_v2 import temporal_trajectory_loss_v2
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2, TemporalModelV2, parameter_count
from scripts.e2e import train_temporal_v2_raw_bootstrap_20261001 as raw_parent
from scripts.e2e import train_temporal_v2_low_speed_joint_20261001 as joint

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE, PERSONAL, GPU_UUID = raw_parent.WORKSPACE, raw_parent.PERSONAL, raw_parent.GPU_UUID
SCHEMA = "portable_e2e.temporal_v2_balanced_replay_screening.20261001.v1"
EXPORT_SCHEMA = "portable_e2e.temporal_v2_balanced_replay_export.20261001.v1"
PREPARED = PERSONAL / "dataset/prepared"
LOW_PATH = PREPARED / "town07-low-speed-cruise-v4-research-train-20261001-v1"
LEGACY_PATH = PREPARED / "moving-recovery-k1-20260922-v1"
DEV_PATH = PREPARED / "moving-data-ab-seed20260906-20260921-v1/B_town07_augmented_train"
LOW_MANIFEST_SHA = "de521e9f2d6809efd564d2a0287c2d6c82ee52fee050e4d3c59f11d218f9bfc8"
LOW_CONVERSION_SHA = "185cbe0c9b9d39dace6326a1a73f875e54f63328fc508553d7990c4179f32751"
LOW_LOADER_FP = "abb0d03ebf541a280a5947fe529d3ae0e0187a8d3443b131c0a10f92a80c807b"
LOW_TEMPORAL_FP = "37b61b86c8c64cbcf09b025cc8c6c6da50a2509a2ebf4493fce1397f38e8ea6b"
LEGACY_LOADER_FP = "4810b0167b082cdace8df766994f3ec28dac21508e8f7a260b9ccf79bead1d29"
LEGACY_TEMPORAL_FP = "74e6ef72f2f04aee30002846647d60d939cc51c2835ac6a1640d4e394414699e"
DEV_LOADER_FP = "caccf80f68d8aa6cdcac2e04d6cd11fdc147be574affdf204fa1683c7621d016"
DEV_TEMPORAL_FP = "8619c4682ab62c620db3845c7443b9c7a8f98a3161bfe9bdd544b0ff87c4e62e"
PARENT_SHA = joint.PARENT_SHA
MAX_UPDATES, BATCH_SIZE, CHECKPOINT_INTERVAL = 3000, 4, 500
SEED, MAX_WALL_SECONDS, CURVED_THRESHOLD = 2026100103, 45 * 60, .01
MINIMUM_FREE_BYTES, CHECKPOINT_WRITE_RESERVE_BYTES = 5 * 1024**3, 512 * 1024**2
FROZEN_SOURCES = {
    "tools/e2e/train_temporal_v2_raw_bootstrap_20261001.py": "85b24d472112364e8c3a468bbe7849c10f4a802a3f7c2ea391c1049ddda5648e",
    "tools/e2e/train_temporal_v2_low_speed_joint_20261001.py": "f047739b639586c5998d8d117bd6d0330940ee28c9c5eadad34ce0a57340178d",
    "learning/portable_e2e/temporal_model_v2.py": "93f709c6c229e705899fcf1778f9dd253ea867e88a6f237ad3f0fb8efdbbf435",
    "learning/portable_e2e/temporal_losses_v2.py": "53d4d61cea15c136ec2c65ffbabc849d16499596f995d9ae3bf55ba175074fe7",
    "learning/portable_e2e/path_time_decoder_v2.py": "d80e370d52c73591df8a593a0483e225cb4cf2595e8389a91400f68f94eea7e7",
    "learning/portable_e2e/deterministic_linear_interpolation_v2.py": "778cda87276ee9330ce1a41c6a60f7f8793f4b61cf0c779eefc84c6e4cd006e6",
}
require, digest, write_new = raw_parent.require, raw_parent.digest, raw_parent.write_new


def imported_source_closure(root, seeds):
    """HH_261001 - Recursively pin local AST imports, including lazy imports and package initializers.

    Dynamic canonical bootstrap imports are explicit seeds. Third-party packages
    belong to the existing venv and are not copied or installed by this helper.
    """
    root = Path(root).resolve(strict=True)
    def module_path(name):
        parts = name.split(".")
        if parts[0] == "portable_e2e": base = root / "learning" / Path(*parts)
        elif parts[0] == "scripts": base = root / "tools" / Path(*parts[1:])
        else: return None
        return base.with_suffix(".py") if base.with_suffix(".py").is_file() else base / "__init__.py" if (base / "__init__.py").is_file() else None
    def package_paths(name):
        return [path for i in range(1, len(name.split("."))+1)
                if (path := module_path(".".join(name.split(".")[:i]))) is not None]
    found, pending = set(), list(seeds)
    while pending:
        path = Path(pending.pop()).resolve(strict=True)
        require(path.is_relative_to(root) and not path.is_symlink(), "source closure escaped canonical root")
        if path in found: continue
        found.add(path)
        relative = path.relative_to(root)
        if relative.parts[0] == "learning": logical = ".".join(relative.with_suffix("").parts[1:])
        elif relative.parts[0] == "tools": logical = ".".join(("scripts", *relative.with_suffix("").parts[1:]))
        else: continue
        package = logical if path.name == "__init__.py" else logical.rpartition(".")[0]
        if path.name == "__init__.py": package = package.removesuffix(".__init__")
        for node in ast.walk(ast.parse(path.read_bytes(), filename=str(path))):
            names = []
            if isinstance(node, ast.Import): names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                prefix = package.split(".")[:len(package.split("."))-node.level+1] if node.level else []
                name = ".".join((*prefix, *node.module.split("."))) if node.module else ".".join(prefix)
                names = [name] + [name+"."+a.name for a in node.names if a.name != "*"]
            for name in names:
                pending.extend(package_paths(name))
    return sorted(found)


def source_snapshot():
    # HH_261001 - Never glob unused live-runtime files; the minimal-clone test proves this recursive closure imports.
    seeds = [Path(__file__).resolve(), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"]
    paths = set(imported_source_closure(ROOT, seeds))
    paths.add(ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json")
    result = {str(p.relative_to(ROOT)): digest(p) for p in sorted(paths)}
    require(all(result[name] == sha for name, sha in FROZEN_SOURCES.items()), "frozen scientific dependency differs")
    return result


def storage_reserve(path):
    # HH_261001 - Retain a 5 GiB floor plus a predeclared 512 MiB write reserve; never delete to make room.
    location = Path(path).resolve(strict=True)
    require(location.is_dir() and not Path(path).is_symlink(), "regular canonical output mount required")
    usage = shutil.disk_usage(location)
    require(usage.free >= MINIMUM_FREE_BYTES + CHECKPOINT_WRITE_RESERVE_BYTES,
            "output mount lacks predeclared free floor/checkpoint reserve; no CUDA/output/write permitted")
    return dict(mount_probe_path=str(location), free_bytes=usage.free, minimum_free_floor_bytes=MINIMUM_FREE_BYTES,
                checkpoint_export_write_reserve_bytes=CHECKPOINT_WRITE_RESERVE_BYTES, deletions_performed=False)


def verify_sources(output, sources):
    # HH_261001 - The live implementation AND the archived scientific record must retain every declared byte pin.
    require(all(digest(ROOT / name) == sha and digest(output / "source" / name) == sha
                for name, sha in sources.items()), "live or archived source changed during owned fit")


def verify_train_disjoint(low, legacy, known):
    require(low.split == legacy.split == "train" and known.split == "val", "only canonical TRAIN and known VAL views")
    all_train = list(low.examples) + list(legacy.examples)
    require(len({r.token for r in all_train}) == len(all_train), "duplicate TRAIN token/repeated view")
    low_eps, old_eps, dev_eps = ({r.episode_id for r in d.examples} for d in (low, legacy, known))
    require(not (low_eps & old_eps) and not ((low_eps | old_eps) & dev_eps), "TRAIN/repeated-view/VAL episode overlap")
    require(not ({r.token for r in all_train} & {r.token for r in known.examples}), "TRAIN/known DEV token overlap")
    require(low_eps and old_eps and dev_eps, "empty declared episode cohort")


def legacy_strata(dataset):
    # HH_261001 - Group from unchanged valid expert geometry, never model outputs or sparse navigation commands.
    eligible, curved, straight, excluded = [], {}, {}, []
    for index, row in enumerate(dataset.examples):
        labels = spatial_expert_labels(row)
        if row.features[1] <= .05 or int(labels["spatial_valid"].sum()) < 2:
            excluded.append(index)
            continue
        eligible.append(index)
        valid = labels["spatial_curvature_valid"]
        maximum = float(labels["spatial_curvature"][valid].abs().max()) if bool(valid.any()) else 0.
        destination = curved if maximum >= CURVED_THRESHOLD else straight
        destination.setdefault(row.map_id, []).append(index)
    require(eligible and curved and straight, "both actual geometry replay strata are required")
    return dict(eligible=eligible, curved=curved, straight=straight, excluded=excluded)


def stratum_receipt(dataset, strata):
    return dict(eligible_count=len(strata["eligible"]), excluded_count=len(strata["excluded"]),
        excluded_indices=strata["excluded"], eligibility="observed vx>.05 and at least two recorded spatial knots",
        curved_rule="max(abs(valid expert curvature)) >= .01 inverse meters",
        curved_by_map={k: len(v) for k, v in strata["curved"].items()},
        straight_by_map={k: len(v) for k, v in strata["straight"].items()},
        indices={key: strata[key] for key in ("curved", "straight")},
        current_speed_range_mps=[min(math.hypot(r.features[1], r.features[2]) for r in dataset.examples),
                                 max(math.hypot(r.features[1], r.features[2]) for r in dataset.examples)],
        legacy_decoded=False, labels_clipped=False, labels_extrapolated=False)


class CycleQueue:
    """HH_261001 - Visit every element once before reshuffling, including non-divisible tails."""
    def __init__(self, values, seed):
        require(values and len(set(values)) == len(values), "nonempty unique sampling queue required")
        self.values, self.rng, self.order, self.cursor = list(values), random.Random(seed), [], 0

    def next(self):
        if self.cursor == len(self.order):
            self.order = list(self.values); self.rng.shuffle(self.order); self.cursor = 0
        value = self.order[self.cursor]; self.cursor += 1
        return value


def balanced_batches(low_count, legacy_count, strata, *, updates=MAX_UPDATES, start_update=0):
    require(type(updates) is int and 0 <= start_update <= updates <= MAX_UPDATES, "bounded sampler interval required")
    require(low_count >= 2 and legacy_count > 0, "nonempty raw and joint TRAIN required")
    indices = [i for key in ("curved", "straight") for values in strata[key].values() for i in values]
    require(len(indices) == len(set(indices)) and all(type(i) is int and 0 <= i < legacy_count for i in indices),
            "replay strata must be disjoint in-range original TRAIN indices")
    require(set(indices) == set(strata["eligible"]) and not (set(indices) & set(strata["excluded"])),
            "no eligible omissions or excluded/non-TRAIN replay")
    low = CycleQueue(range(low_count), SEED)
    queues = {}
    for kind_id, kind in enumerate(("curved", "straight")):
        require(strata[kind] and all(strata[kind].values()), "empty map stratum")
        queues[kind] = {name: CycleQueue(values, SEED + 100*(kind_id+1) + pos)
                        for pos, (name, values) in enumerate(sorted(strata[kind].items()))}
    for update in range(updates):
        choices = [low.next(), low.next()]
        for kind in ("curved", "straight"):
            maps = sorted(queues[kind]); name = maps[update % len(maps)]
            choices.append(low_count + queues[kind][name].next())
        if update >= start_update:
            yield choices


def slice_batch(batch, begin, end):
    return {name: value[begin:end] for name, value in batch.items()}


def mean_terms(records):
    return {name: torch.stack([record[name] for record in records]).mean()
            for name in records[0] if isinstance(records[0][name], torch.Tensor)}


def balanced_objective(raw, decoder, batch):
    """HH_261001 - Normalize each anchor's true masks FIRST, then the 2-example domain means.

    The decoder sees only the first two low-speed anchors. Legacy actual v0 and
    labels may exceed 2.5 m/s and are never passed to or clipped for this decoder.
    """
    require(raw["curvature_knots"].shape == (4, 33), "exact two/one/one batch ABI required")
    for name in ("intent_valid", "stop_distance_valid", "initial_acceleration_valid"):
        if name in batch:
            require(not bool(batch[name].any()), "verified new semantic labels require a separately declared experiment")
    low = slice_batch(batch, 0, 2)
    v0, support = joint.measured_current_speed(low), joint.causal_route_support(low["route_xy"], low["route_mask"])
    decoded = decoder(raw["curvature_knots"][:2], raw["preferred_speed_knots"][:2], v0,
                      torch.full_like(v0, GO, dtype=torch.long), torch.zeros_like(v0), support)
    low_records, old_records = [], []
    for index in range(4):
        item = slice_batch(batch, index, index+1)
        outputs = slice_batch(raw, index, index+1)
        if index < 2:
            losses = temporal_trajectory_loss_v2(outputs, item, decoded=slice_batch(decoded, index, index+1))
            prior = joint.preview_speed_regularizer(outputs["preferred_speed_knots"], item["spatial_speed_valid"],
                                                    decoder.config.go_active_distance_m)
            low_records.append(dict(losses, preview_speed_regularizer=prior,
                                    total=losses["total"] + .1 * prior))
        else:
            old_records.append(temporal_trajectory_loss_v2(outputs, item))
    low_terms, old_terms = mean_terms(low_records), mean_terms(old_records)
    total = .5 * low_terms["total"] + .5 * old_terms["total"]
    require(bool(torch.isfinite(total)), "nonfinite balanced objective")
    return decoded, low_terms, old_terms, total


def gradient_receipt(loss, model):
    # HH_261001 - Independently audit each domain's parameter gradients before the weighted optimizer update.
    values = torch.autograd.grad(loss, tuple(model.parameters()), retain_graph=True, allow_unused=True)
    norms = [v.detach().square().sum() for v in values if v is not None]
    require(norms and all(bool(torch.isfinite(v).all()) for v in values if v is not None), "nonfinite domain gradient")
    norm = torch.stack(norms).sum().sqrt()
    require(bool(torch.isfinite(norm)), "nonfinite domain gradient norm")
    return dict(all_present_parameter_gradients_finite=True, gradient_l2_norm=float(norm),
                unused_parameter_tensors=sum(v is None for v in values))


def validate_batch_identity(batch, order, low, legacy):
    expected = [low.examples[i].token if i < len(low) else legacy.examples[i-len(low)].token for i in order]
    require(list(batch["sample_id"]) == expected, "loader order/sample identity differs from declared TRAIN sampler")


def raw_stratum_evaluation(model, dataset, strata, device):
    # HH_261001 - These full legacy TRAIN metrics are resubstitution diagnostics, not independent validation.
    model.eval(); records = []
    index_kind = {i: kind for kind in ("curved", "straight") for v in strata[kind].values() for i in v}
    with torch.no_grad():
        for offset in range(0, len(strata["eligible"]), 4):
            chosen = strata["eligible"][offset:offset+4]
            batch = raw_parent.move(default_collate([dataset[i] for i in chosen]), device)
            raw = raw_parent.forward(model, batch)
            for pos, index in enumerate(chosen):
                terms = temporal_trajectory_loss_v2(slice_batch(raw, pos, pos+1), slice_batch(batch, pos, pos+1))
                records.append(dict(index=index, token=dataset.examples[index].token,
                    episode_id=dataset.examples[index].episode_id, map=dataset.examples[index].map_id,
                    stratum=index_kind[index], **joint.loss_numbers(terms)))
    groups = {}
    for name in ("all_eligible", "curved", "straight"):
        rows = records if name == "all_eligible" else [r for r in records if r["stratum"] == name]
        groups[name] = dict(count=len(rows), **{k: sum(r[k] for r in rows)/len(rows)
                           for k in ("total", "raw_curvature", "raw_preferred_speed")})
    per_episode = {}
    for episode in sorted({r["episode_id"] for r in records}):
        rows = [r for r in records if r["episode_id"] == episode]
        per_episode[episode] = dict(count=len(rows), **{k: sum(r[k] for r in rows)/len(rows)
            for k in ("total", "raw_curvature", "raw_preferred_speed")})
    return dict(split=dataset.split, independent_validation=False, legacy_decoded=False,
                full_fixed_eligible_denominator=len(records), groups=groups, per_episode=per_episode, rows=records)


def evaluate_known_full(model, dataset, device):
    # HH_261001 - The 599 moving anchors are exposed known DEV; neither all665 nor independent/sealed TEST.
    strata = legacy_strata(dataset)
    require(dataset.split == "val" and len(strata["eligible"]) == 599
            and len({r.episode_id for r in dataset.examples}) == 2, "fixed known moving DEV599 denominator differs")
    result = raw_stratum_evaluation(model, dataset, strata, device)
    return dict(result, role="previously_exposed_known_moving_DEV599_raw_diagnostic",
                available_known_val_count=665, excluded_count=66, independent_validation=False)


def endpoint_screens(before8, after8, before599, after599, after_v4):
    # HH_261001 - Predeclared diagnostic screens cannot authorize driving or certify an independent capability.
    checks = []
    def record(name, actual, threshold, evidence):
        checks.append(dict(name=name, actual=actual, threshold=threshold,
                           passed=bool(math.isfinite(actual) and actual <= threshold), evidence_scope=evidence))
    record("V4_TRAIN_ADE", after_v4["mean_ade_6p4s_m"], .10, "TRAIN_resubstitution327")
    record("V4_TRAIN_FDE", after_v4["mean_fde_6p4s_m"], .25, "TRAIN_resubstitution327")
    record("V4_TRAIN_instantaneous_speed_MAE", after_v4["mean_instantaneous_speed_mae_mps"], .03, "TRAIN_resubstitution327")
    record("known_DEV8_raw_total", after8["mean_total"], .303911, "previously_exposed8_probes")
    require(before599["full_fixed_eligible_denominator"] == after599["full_fixed_eligible_denominator"] == 599
            and set(before599["per_episode"]) == set(after599["per_episode"]), "known DEV endpoint membership differs")
    for label, before, after in [("aggregate", before599["groups"]["all_eligible"], after599["groups"]["all_eligible"]),
            *[(ep, before599["per_episode"][ep], after599["per_episode"][ep]) for ep in sorted(before599["per_episode"])]]:
        require(before["count"] == after["count"], "known DEV episode denominator differs")
        for component in ("raw_curvature", "raw_preferred_speed"):
            record("known_DEV599/"+label+"/"+component, after[component], before[component]*1.10+1e-4,
                   "previously_exposed599_raw_diagnostic_not_independent")
    return dict(schema="portable_e2e.balanced_replay_endpoint_screen.20261001.v1", fixed_endpoint_updates=3000,
        all_diagnostic_screens_passed=all(r["passed"] for r in checks), checks=checks,
        before_known8_mean_total=before8["mean_total"], after_known8_mean_total=after8["mean_total"],
        feature_approved=False, vehicle_control_approved=False, model_promoted=False, independent_validation=False)


def load_cohorts(low_path, legacy_path, dev_path):
    for actual, expected in ((low_path, LOW_PATH), (legacy_path, LEGACY_PATH), (dev_path, DEV_PATH)):
        require(actual.resolve(strict=True) == expected.resolve(strict=True), "only exact existing canonical personal view is legal")
    low, low_receipt = joint.load_train(low_path, LOW_MANIFEST_SHA, LOW_CONVERSION_SHA)
    legacy, old_receipt = raw_parent.load_view(legacy_path, "train", raw_parent.TRAIN_MANIFEST_SHA)
    known, dev_receipt = raw_parent.load_view(dev_path, "val", raw_parent.DEV_MANIFEST_SHA)
    for receipt, loader, temporal in ((low_receipt, LOW_LOADER_FP, LOW_TEMPORAL_FP),
            (old_receipt, LEGACY_LOADER_FP, LEGACY_TEMPORAL_FP), (dev_receipt, DEV_LOADER_FP, DEV_TEMPORAL_FP)):
        require(receipt.get("validated_fingerprint_sha256", receipt.get("validation_fingerprint_sha256")) == loader
                and receipt["temporal_fingerprint_sha256"] == temporal, "exact existing loader/temporal fingerprint differs")
    verify_train_disjoint(low, legacy, known)
    require(len(low) == 327 and len(legacy) == 2787 and len(known) == 665,
            "fixed TRAIN and known DEV denominators differ")
    strata = legacy_strata(legacy)
    require(stratum_receipt(legacy, strata)["curved_by_map"] == {"C_track_1_0_7": 260, "Town01": 68}
            and stratum_receipt(legacy, strata)["straight_by_map"] == {"C_track_1_0_7": 662, "Town01": 570, "Town07": 927}
            and len(strata["eligible"]) == 2487 and len(strata["excluded"]) == 300,
            "fixed true-geometry replay memberships differ")
    require(len({r.episode_id for r in legacy.examples}) == 8 and len({r.episode_id for r in low.examples}) == 1,
            "mixed research must retain exactly nine TRAIN episodes")
    return low, legacy, known, strata, dict(v4=low_receipt, legacy=old_receipt, known_dev=dev_receipt)


def scope_fields():
    return dict(training_only=True, simulation_only=True, one_episode_screening=False,
        research_screening=True, mixed_raw_joint_training=True, training_episode_count=9,
        main_training=False, model_promoted=False, vehicle_control_approved=False,
        independent_validation=False, stop_intent_trained=False, runtime_intent="explicit_cruise_GO_not_head",
        raw_spatial_only=False, decoder_integrated=True, legacy_decoded=False, labels_clipped=False, labels_extrapolated=False)


def training_identity(receipts, strata):
    value = dict(receipts=receipts, legacy_strata=strata, batch=[2, 1, 1], seed=SEED,
                 objective=".5 per-anchor V4 joint mean + .5 per-anchor legacy raw mean")
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def verify_parent_checkpoint(path, sources):
    # HH_261001 - Preserve the complete pinned historical receipt but validate only the actually imported parent closure.
    require(digest(path) == PARENT_SHA, "exact raw200 parent checkpoint differs")
    parent = torch.load(path, map_location="cpu")
    require(parent.get("schema") == "portable_e2e.temporal_raw_bootstrap.20261001.v1"
            and parent.get("optimizer_updates") == 200 and parent.get("raw_spatial_only") is True
            and parent.get("stop_intent_trained") is False and parent.get("decoder_integrated") is False
            and parent.get("vehicle_control_approved") is False
            and parent.get("training_dataset_fingerprint_sha256") == LEGACY_TEMPORAL_FP,
            "exact raw200 research lineage differs")
    historical = parent.get("source_sha256")
    require(isinstance(historical, dict) and historical, "raw200 historical source receipt required")
    # HH_261001 - Newly introduced decoder/kernel/trainer files are explicitly pinned new revisions, not raw200 parents.
    inherited = {name for name in sources if name in historical}
    require("learning/portable_e2e/temporal_model_v2.py" in inherited
            and "learning/portable_e2e/temporal_losses_v2.py" in inherited
            and all(sources[name] == historical[name] for name in inherited), "imported raw200 parent source changed")
    TemporalModelConfigV2.from_mapping(parent["model_config"])
    model = TemporalModelV2(TemporalModelConfigV2.from_mapping(parent["model_config"]))
    model.load_state_dict(parent["model_state_dict"], strict=True)
    require(all(bool(torch.isfinite(v).all()) for v in model.state_dict().values()), "nonfinite raw200 parent state")
    return parent


def verify_resume(path, expected_sha, sources, identity):
    require(digest(path) == expected_sha, "externally pinned resume checkpoint differs")
    state = torch.load(path, map_location="cpu")
    step = state.get("optimizer_updates")
    require(state.get("schema") == SCHEMA and type(step) is int and 0 < step < MAX_UPDATES
            and state.get("examples_seen") == step*4 and state.get("v4_examples_seen") == step*2
            and state.get("legacy_examples_seen") == step*2
            and state.get("parent_checkpoint_sha256") == PARENT_SHA
            and state.get("parent_source_optimizer_updates") == 200
            and state.get("known_total_optimizer_updates") == 200+step
            and state.get("maximum_optimizer_updates") == MAX_UPDATES
            and state.get("batch_domains") == ["v4_joint", "v4_joint", "legacy_curved_raw", "legacy_straight_raw"]
            and state.get("training_dataset_fingerprint_sha256") == identity
            and state.get("source_sha256") == sources and isinstance(state.get("optimizer_state_dict"), dict)
            and all(state.get(k) == v for k, v in scope_fields().items()), "exact mixed lineage/exposure/resume scope differs")
    return state


def save_checkpoint(output, step, model, optimizer, sources, declaration_sha, identity):
    storage_reserve(output)
    state = dict(schema=SCHEMA, model_config=model.config.to_dict(),
        model_state_dict={k: v.detach().cpu() for k, v in model.state_dict().items()},
        optimizer_state_dict=optimizer.state_dict(), optimizer_updates=step, examples_seen=step*4,
        v4_examples_seen=step*2, legacy_examples_seen=step*2,
        parent_checkpoint_sha256=PARENT_SHA, parent_source_optimizer_updates=200,
        known_total_optimizer_updates=200+step, source_sha256=sources, declaration_sha256=declaration_sha,
        training_dataset_fingerprint_sha256=identity, maximum_optimizer_updates=MAX_UPDATES,
        batch_domains=["v4_joint", "v4_joint", "legacy_curved_raw", "legacy_straight_raw"], **scope_fields())
    path = output / "checkpoints" / f"step_{step:06d}.pt"; path.parent.mkdir(exist_ok=True)
    with path.open("xb") as stream:
        torch.save(state, stream)
    folder = output / "exports" / f"step_{step:06d}"; folder.mkdir(parents=True, exist_ok=False)
    arrays = {k: v.numpy() for k, v in state["model_state_dict"].items()}
    require(all(v.dtype == np.float32 and np.isfinite(v).all() for v in arrays.values()), "finite float32 numeric export required")
    npz = folder / "model_state.npz"
    with npz.open("xb") as stream:
        np.savez(stream, **arrays)
    with np.load(npz, allow_pickle=False) as checked:
        require(set(checked.files) == set(arrays) and all(np.array_equal(checked[k], v) for k, v in arrays.items()),
                "data-only float32 export roundtrip differs")
    metadata = {k: v for k, v in state.items() if k not in ("model_state_dict", "optimizer_state_dict")}
    metadata.update(schema=EXPORT_SCHEMA, checkpoint_sha256=digest(path), npz_sha256=digest(npz),
        state_tensor_shapes={k: list(v.shape) for k, v in arrays.items()},
        state_tensor_dtypes={k: str(v.dtype) for k, v in arrays.items()},
        numerical_revision=dict(kernel_id=KERNEL_ID, strict_deterministic_algorithms=True,
            tf32_matrix_multiplication=False, source_sha256={k: sources[k] for k in FROZEN_SOURCES},
            frozen_decoder_sha256=FROZEN_SOURCES["learning/portable_e2e/path_time_decoder_v2.py"],
            finite_precision_bitwise_equivalence_claimed=False),
        inference_loader_authorized=False, single_episode_export_compatibility_claimed=False)
    write_new(folder / "metadata.json", metadata)
    return dict(path=str(path), sha256=digest(path), optimizer_updates=step, optimizer_state_retained=True,
        export_npz=str(npz), export_npz_sha256=metadata["npz_sha256"], export_metadata=str(folder / "metadata.json"),
        export_metadata_sha256=digest(folder / "metadata.json"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--train", type=Path, default=LOW_PATH)
    parser.add_argument("--legacy-train", type=Path, default=LEGACY_PATH)
    parser.add_argument("--known-dev", type=Path, default=DEV_PATH)
    parser.add_argument("--parent-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--loader-workers", type=int, choices=(0, 1, 2), default=2)
    parser.add_argument("--resume-checkpoint", type=Path)
    parser.add_argument("--resume-checkpoint-sha256")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    require(not args.output.exists() and not args.output.is_symlink()
            and args.output.resolve().is_relative_to((WORKSPACE / "runs/training").resolve()), "create-only personal training output required")
    require((args.resume_checkpoint is None) == (args.resume_checkpoint_sha256 is None), "resume path and digest must be paired")
    initial_storage = storage_reserve(args.output.parent)
    torch.set_num_threads(4)
    sources = source_snapshot()
    low, legacy, known, strata, receipts = load_cohorts(args.train, args.legacy_train, args.known_dev)
    parent = verify_parent_checkpoint(args.parent_checkpoint, sources)
    identity = training_identity(receipts, strata)
    state = verify_resume(args.resume_checkpoint, args.resume_checkpoint_sha256, sources, identity) if args.resume_checkpoint else parent
    start = state["optimizer_updates"] if args.resume_checkpoint else 0
    order = list(balanced_batches(len(low), len(legacy), strata, start_update=start))
    probes = raw_parent.fixed_probe(known, raw_parent.eligible_indices(known))
    require(probes == [266, 273, 279, 286, 38, 257, 472, 664], "exact historical known DEV probes differ")
    declaration = dict(schema=SCHEMA, cohorts=receipts, legacy_strata=stratum_receipt(legacy, strata),
        known_dev_probe_indices=probes, known_dev_is_independent=False, source_sha256=sources,
        parent_checkpoint_sha256=PARENT_SHA, resume_checkpoint_sha256=args.resume_checkpoint_sha256,
        parent_historical_source_sha256=parent["source_sha256"],
        parent_dependency_policy="all actually imported inherited files byte-identical; unused historical modules archived by parent, not imported by this fit",
        start_optimizer_updates=start, requested_optimizer_updates=MAX_UPDATES, maximum_optimizer_updates=MAX_UPDATES,
        maximum_owned_training_wall_seconds=MAX_WALL_SECONDS, effective_batch_size=4, seed=SEED,
        initial_storage_reserve=initial_storage,
        storage_policy="5GiB free floor plus512MiB write reserve checked before CUDA/output and every checkpoint/export; no deletion/overwrite",
        sampler="each update2 V4 cycle +1 true-curved alternating map +1 true-straight round-robin map; no drop-last",
        batch_domains=["v4_joint", "v4_joint", "legacy_curved_raw", "legacy_straight_raw"],
        sampler_remaining_schedule_sha256=hashlib.sha256(json.dumps(order, separators=(",", ":")).encode()).hexdigest(),
        objective=".5 mean(two per-anchor V4 joint losses) + .5 mean(two per-anchor legacy raw losses)",
        loss_denominators="each anchor: valid curvature knots/speed knots; decoded XY mean coordinates then valid horizon .4/.3/.3; invalid active preview prior its own count; each domain mean2; weighted .5/.5",
        endpoint_screens=dict(v4_train_ADE_max_m=.10, v4_train_FDE_max_m=.25,
            v4_train_instantaneous_speed_MAE_max_mps=.03, known_dev8_raw_total_max=.303911,
            known_moving_dev599_each_raw_component="aggregate and each episode <=raw200_parent*1.10+1e-4",
            fixed_endpoint_updates=3000, checkpoint_selection_by_dev=False, feature_approval=False),
        owned_wall_policy="45min boundary includes GPU initialization, raw200 before-evaluations and optimizer/checkpoint phase; completed fixed-endpoint final evaluations are separately timed post-fit checks, no further optimizer updates",
        preview_regularization_weight=.1, preview_regularization_domain="V4 only, unchanged detached PREDICTED boundary prior",
        old_current_speed_never_decoded=True, expert_future_never_model_input=True,
        learning_rate=dict(peak=joint.INITIAL_LR, final=joint.FINAL_LR, warmup_updates=joint.WARMUP_UPDATES,
                           cosine_horizon_updates=MAX_UPDATES, restart_on_resume=False),
        optimizer="fresh AdamW at parent warm start; exact optimizer restoration only on declared resume",
        weight_decay=1e-4, gradient_clip=5., per_domain_parameter_gradients_audited=True,
        loader_workers=args.loader_workers, endpoint_selection="fixed3000; no selection by known DEV",
        training_dataset_fingerprint_sha256=identity, **scope_fields())
    if args.validate_only:
        print(json.dumps(dict(status="VALIDATED_NO_GPU_OR_OUTPUT", **declaration), indent=2, allow_nan=False)); return 0
    with raw_parent.gpu0_lease():
        wall_started = time.monotonic()
        storage_reserve(args.output.parent)
        gpu = raw_parent.gpu_preflight()
        require(os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8", "deterministic cuBLAS environment required")
        random.seed(SEED); torch.manual_seed(SEED); torch.cuda.manual_seed_all(SEED)
        torch.use_deterministic_algorithms(True)
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.benchmark = False; torch.backends.cudnn.deterministic = True
        if hasattr(torch.backends.cuda, "enable_flash_sdp"):
            torch.backends.cuda.enable_flash_sdp(False); torch.backends.cuda.enable_mem_efficient_sdp(False)
        args.output.mkdir(parents=True, exist_ok=False)
        write_new(args.output / "declaration.json", declaration); declaration_sha = digest(args.output / "declaration.json")
        joint.archive_sources(args.output, sources)
        device = torch.device("cuda:0")
        model = TemporalModelV2(TemporalModelConfigV2.from_mapping(state["model_config"])).to(device)
        model.load_state_dict(state["model_state_dict"], strict=True)
        decoder = DeterministicPathTimeDecoderV2().to(device)
        optimizer = torch.optim.AdamW(model.parameters(), lr=joint.INITIAL_LR, weight_decay=1e-4)
        if start:
            optimizer.load_state_dict(state["optimizer_state_dict"])
        write_new(args.output / "started.json", dict(gpu=gpu, pid=os.getpid(),
            started_at_utc=datetime.now(timezone.utc).isoformat(), parameters=parameter_count(model)))
        # HH_261001 - All before/final checks retain their actual TRAIN or previously-exposed known DEV names.
        model.load_state_dict(parent["model_state_dict"], strict=True)
        before8 = raw_parent.evaluate(model, known, probes, device)
        before599 = evaluate_known_full(model, known, device)
        write_new(args.output / "known_dev_raw_before.json", dict(before8, reference="exact_raw200_parent_even_on_resume"))
        write_new(args.output / "known_moving_dev599_raw_before.json", dict(before599, reference="exact_raw200_parent_even_on_resume"))
        write_new(args.output / "v4_train_joint_before.json", joint.evaluate_train(model, decoder, low, device))
        write_new(args.output / "legacy_train_raw_before.json", raw_stratum_evaluation(model, legacy, strata, device))
        model.load_state_dict(state["model_state_dict"], strict=True)
        kwargs = dict(dataset=ConcatDataset([low, legacy]), batch_sampler=order, num_workers=args.loader_workers, pin_memory=True)
        if args.loader_workers:
            kwargs.update(prefetch_factor=2, persistent_workers=True, multiprocessing_context="spawn")
        loader = DataLoader(**kwargs)
        started = time.monotonic(); completed, checkpoints, times, wall_stop = start, [], [], False
        with (args.output / "history.jsonl").open("x") as journal:
            for offset, batch in enumerate(loader):
                if time.monotonic()-wall_started >= MAX_WALL_SECONDS:
                    wall_stop = True; break
                step = start+offset+1; tick = time.monotonic()
                validate_batch_identity(batch, order[offset], low, legacy)
                batch = raw_parent.move(batch, device)
                model.train(); optimizer.zero_grad(set_to_none=True)
                rate = joint.learning_rate(step)
                for group in optimizer.param_groups:
                    group["lr"] = rate
                raw = raw_parent.forward(model, batch)
                _, low_terms, old_terms, loss = balanced_objective(raw, decoder, batch)
                gradients = dict(v4=gradient_receipt(low_terms["total"], model), legacy=gradient_receipt(old_terms["total"], model))
                loss.backward(); norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.)
                require(bool(torch.isfinite(norm)), "nonfinite weighted parameter gradients")
                optimizer.step(); torch.cuda.synchronize()
                require(all(bool(torch.isfinite(v).all()) for v in model.state_dict().values()), "nonfinite updated state")
                completed = step; elapsed = time.monotonic()-tick; times.append(elapsed)
                row = dict(step=step, optimizer_updates=step, examples_seen=step*4,
                    v4_examples_seen=step*2, legacy_examples_seen=step*2, indices=order[offset],
                    sample_ids=list(batch["sample_id"]), total_loss=float(loss.detach()),
                    v4_joint=joint.loss_numbers(low_terms), legacy_raw=joint.loss_numbers(old_terms),
                    domain_parameter_gradients=gradients, preclip_gradient_norm=float(norm), learning_rate=rate,
                    step_compute_wall_seconds=elapsed, elapsed_training_wall_seconds=time.monotonic()-started)
                journal.write(json.dumps(row, allow_nan=False)+"\n"); journal.flush()
                if step == start+1 or step % 50 == 0:
                    print(json.dumps(row, allow_nan=False), flush=True)
                if step % CHECKPOINT_INTERVAL == 0:
                    checkpoints.append(save_checkpoint(args.output, step, model, optimizer, sources, declaration_sha, identity))
                    print(json.dumps(dict(status="MIXED_CHECKPOINT_RETAINED_NOT_PROMOTED", **checkpoints[-1])), flush=True)
        if completed > start and completed % CHECKPOINT_INTERVAL:
            checkpoints.append(save_checkpoint(args.output, completed, model, optimizer, sources, declaration_sha, identity))
        fit_phase_seconds = time.monotonic()-wall_started
        optimizer_loop_seconds = time.monotonic()-started
        if completed == MAX_UPDATES:
            postfit_started = time.monotonic()
            after8 = raw_parent.evaluate(model, known, probes, device)
            after599 = evaluate_known_full(model, known, device)
            after_v4 = joint.evaluate_train(model, decoder, low, device)
            write_new(args.output / "known_dev_raw_final.json", after8)
            write_new(args.output / "known_moving_dev599_raw_final.json", after599)
            write_new(args.output / "v4_train_joint_final.json", after_v4)
            write_new(args.output / "legacy_train_raw_final.json", raw_stratum_evaluation(model, legacy, strata, device))
            write_new(args.output / "endpoint_diagnostic_screens.json", endpoint_screens(before8, after8, before599, after599, after_v4))
            postfit_seconds = time.monotonic()-postfit_started
        else:
            postfit_seconds = 0.
        verify_sources(args.output, sources)
        summary = dict(schema=SCHEMA, status="BOUNDED_BALANCED_REPLAY_COMPLETE_NOT_PROMOTED" if completed == MAX_UPDATES
            else "OWNED_WALL_BUDGET_STOP_NOT_PROMOTED", optimizer_updates=completed,
            additional_optimizer_updates=completed-start, examples_seen=completed*4,
            v4_examples_seen=completed*2, legacy_examples_seen=completed*2,
            v4_equivalent_anchor_exposure_cycles=completed*2/327,
            legacy_geometry_balanced_exposure_not_uniform_cycles=True, wall_budget_stop=wall_stop,
            elapsed_training_wall_seconds=time.monotonic()-started,
            elapsed_owned_gpu_phase_wall_seconds=time.monotonic()-wall_started,
            fit_phase_including_baseline_seconds=fit_phase_seconds, optimizer_loop_wall_seconds=optimizer_loop_seconds,
            postfit_endpoint_evaluation_seconds=postfit_seconds,
            mean_step_compute_seconds=sum(times)/len(times) if times else None,
            peak_GPU_allocated_bytes=torch.cuda.max_memory_allocated(), source_hashes_match=True,
            final_checkpoint_sha256=checkpoints[-1]["sha256"] if checkpoints else None,
            optimizer_state_retained=bool(checkpoints), checkpoints=checkpoints, **scope_fields())
        write_new(args.output / "summary.json", summary); print(json.dumps(summary, allow_nan=False), flush=True)
    return 0 if completed == MAX_UPDATES else 2


if __name__ == "__main__":
    raise SystemExit(main())
