"""HH_261001 - Train true low-speed futures with frozen V2 math in one bounded research cohort.

This is an explicitly scoped, one-episode TRAIN screening, not generalization,
full-route approval, STOP/GO learning, or the planned 30-episode main training.
Only the physical remote GPU 0 and the existing personal Python venv are legal.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import random
import time

# HH_261001 - Spawned CPU data-loader workers must reconstruct the canonical namespace before unpickling dependencies.
if __name__ == "__mp_main__":
    import importlib.util
    _workspace_entry = Path(__file__).resolve().parents[1] / "workspace_python.py"
    _workspace_spec = importlib.util.spec_from_file_location("hh_joint_worker_workspace", _workspace_entry)
    _workspace_module = importlib.util.module_from_spec(_workspace_spec)
    _workspace_spec.loader.exec_module(_workspace_module)
    _workspace_module.configure(Path(__file__).resolve().parents[2])

import numpy as np
import torch
from torch.nn import functional as F
from torch.utils.data import DataLoader
from torch.utils.data.dataloader import default_collate

from portable_e2e.contract import ContractError, _safe_file
from portable_e2e.dataset import load_training_examples
from portable_e2e.path_time_decoder_v2 import GO, PathTimeDecoderV2
from portable_e2e.temporal_dataset_v2 import (
    TemporalCommon10TorchDataset, load_common10_temporal_metadata, spatial_expert_labels)
from portable_e2e.temporal_losses_v2 import temporal_trajectory_loss_v2
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2, TemporalModelV2, parameter_count
from scripts.e2e import train_temporal_v2_raw_bootstrap_20261001 as previous


ROOT = Path(__file__).resolve().parents[2]
WORKSPACE, PERSONAL, GPU_UUID = previous.WORKSPACE, previous.PERSONAL, previous.GPU_UUID
FIELDS = previous.FIELDS
PARENT_SHA = "6c8bfceee5b3a2a0e332666b225769af236499baac9414294c18ccc6e9833ece"
SCHEMA = "portable_e2e.temporal_v2_low_speed_joint.20261001.v1"
CONVERSION_SCHEMA = "carla.town07_low_speed_v4_research_train_conversion_result.20261001.v1"
MAXIMUM_UPDATES, BATCH_SIZE, CHECKPOINT_INTERVAL = 3000, 4, 500
SEED, WARMUP_UPDATES, INITIAL_LR, FINAL_LR = 2026100102, 100, 3e-5, 3e-6
TAIL_REGULARIZATION_WEIGHT = .1
MAX_WALL_SECONDS = 45 * 60
require, digest, write_new = previous.require, previous.digest, previous.write_new


def read(path):
    with Path(path).open() as stream:
        value = json.load(stream)
    require(isinstance(value, dict), "JSON object required")
    return value


def source_snapshot():
    # HH_261001 - New training orchestration archives dependencies without changing frozen model/loss/decoder bytes.
    files = {*ROOT.glob("learning/portable_e2e/*.py"),
             *ROOT.glob("learning/portable_e2e/config/*.json"), Path(__file__),
             Path(previous.__file__).resolve(), ROOT / "tools/workspace_python.py",
             ROOT / "tools/canonical_workspace.py"}
    return {str(path.relative_to(ROOT)): digest(path) for path in sorted(files)}


def causal_route_support(route_xy, route_mask):
    """HH_261001 - Conservative navigation arc coverage, never recorded expert-future distance.

    This supports the research decoder's path domain; it is NOT a claim that a
    navigation polyline establishes free space or proves a learned path safe.
    Prefix-only validity is required. Distance from the ego to the first route
    point is deliberately not added to usable forward route coverage.
    """
    if (route_xy.ndim != 3 or route_xy.shape[-1] != 2
            or route_mask.shape != route_xy.shape[:2] or route_mask.dtype != torch.bool):
        raise ContractError("causal navigation shape/mask differs")
    if not bool(torch.isfinite(route_xy).all()):
        raise ContractError("causal navigation must be finite")
    if bool((route_mask[:, 1:] & ~route_mask[:, :-1]).any()):
        raise ContractError("causal navigation validity must be a prefix")
    if bool((route_mask.sum(-1) < 2).any()):
        raise ContractError("causal navigation needs two actual points")
    distances = torch.linalg.norm(route_xy[:, 1:] - route_xy[:, :-1], dim=-1)
    valid_segments = route_mask[:, 1:] & route_mask[:, :-1]
    return (distances * valid_segments.to(distances.dtype)).sum(-1).clamp(max=64.)


def measured_current_speed(batch):
    # HH_261001 - Preserve the exact measured planar speed at the same base-link reference as true labels.
    history, valid = batch["ego_history"], batch["ego_history_mask"]
    if history.ndim != 3 or history.shape[-1] != 13 or not bool(valid[:, -1].all()):
        raise ContractError("current measured ego state is missing")
    value = torch.linalg.norm(history[:, -1, 1:3], dim=-1)
    if not bool(torch.isfinite(value).all()) or bool(((value < 0) | (value > 2.5)).any()):
        raise ContractError("actual current speed exceeds the unchanged low-speed envelope; no clipping")
    return value


def preview_speed_regularizer(raw_speed, speed_valid, active_distance_m=18.):
    """HH_261001 - Penalize unsupported raw preview discontinuities without extrapolating expert labels.

    The two typically unlabelled preview knots at 16 and 18 m receive a soft
    prior toward the LAST PREDICTED supervised speed knot, detached at that
    boundary. Expert speeds are not extended, masks stay unchanged, decoder
    outputs are never replaced, and runtime uses the exact raw model outputs.
    This is a declared training-only smooth-preview bias for the cruise task.
    """
    if (raw_speed.ndim != 2 or raw_speed.shape[1] != 33
            or speed_valid.shape != raw_speed.shape or speed_valid.dtype != torch.bool):
        raise ContractError("preview speed/mask ABI differs")
    if bool((speed_valid.sum(-1) == 0).any()) or bool((speed_valid[:, 1:] & ~speed_valid[:, :-1]).any()):
        raise ContractError("observed spatial speed labels must be a nonempty prefix")
    index = torch.arange(33, device=raw_speed.device)[None].expand_as(raw_speed)
    last = torch.where(speed_valid, index, torch.zeros_like(index)).max(-1).values
    reference = raw_speed.gather(1, last[:, None]).detach().expand_as(raw_speed)
    regularized = (~speed_valid) & (index.to(raw_speed.dtype) * 2 <= active_distance_m)
    differences = F.smooth_l1_loss(raw_speed, reference, reduction="none")
    return (differences * regularized.to(raw_speed.dtype)).sum() / regularized.sum().clamp_min(1)


def joint_forward(model, decoder, batch):
    raw = model(**{name: batch[name] for name in FIELDS}, history_mode="temporal")
    current = measured_current_speed(batch)
    support = causal_route_support(batch["route_xy"], batch["route_mask"])
    # HH_261001 - GO is the declared cruise experiment condition, NOT a prediction from an untrained intent head.
    intent = torch.full_like(current, GO, dtype=torch.long)
    decoded = decoder(raw["curvature_knots"], raw["preferred_speed_knots"], current,
                      intent, torch.zeros_like(current), support)
    base = temporal_trajectory_loss_v2(raw, batch, decoded=decoded)
    prior = preview_speed_regularizer(raw["preferred_speed_knots"], batch["spatial_speed_valid"],
                                      decoder.config.go_active_distance_m)
    loss = base["total"] + TAIL_REGULARIZATION_WEIGHT * prior
    require(bool(torch.isfinite(loss)), "joint objective must be finite")
    return raw, decoded, dict(base, total=loss, preview_speed_regularizer=prior)


def deterministic_batches(count, updates, seed=SEED, start_update=0):
    """HH_261001 - Visit every declared TRAIN anchor each cycle; retain leftovers across cycles.

    No model-based sample selection, drop-last omission, or frame trimming is
    allowed. Regenerating all previous batches makes resumption deterministic.
    """
    require(count >= BATCH_SIZE and 0 <= start_update <= updates <= MAXIMUM_UPDATES,
            "bounded exact-batch sampling required")
    generator = random.Random(seed)
    order, cursor = [], 0
    for update in range(updates):
        batch = []
        while len(batch) < BATCH_SIZE:
            if cursor == len(order):
                order = list(range(count)); generator.shuffle(order); cursor = 0
            take = min(BATCH_SIZE-len(batch), len(order)-cursor)
            batch.extend(order[cursor:cursor+take]); cursor += take
        if update >= start_update:
            yield batch


def learning_rate(update):
    require(1 <= update <= MAXIMUM_UPDATES, "learning-rate update outside declared horizon")
    if update <= WARMUP_UPDATES:
        return INITIAL_LR * update / WARMUP_UPDATES
    progress = (update-WARMUP_UPDATES)/(MAXIMUM_UPDATES-WARMUP_UPDATES)
    return FINAL_LR + .5*(INITIAL_LR-FINAL_LR)*(1+math.cos(math.pi*progress))


def cohort_statistics(dataset):
    """HH_261001 - Inventory EVERY admitted anchor before output creation or a GPU reservation."""
    current, speeds, chord_errors, arcs, support, curvatures = [], [], [], [], [], []
    for row in dataset.examples:
        require(row.valid_point_count == 64 and len(row.target_speed_mps) == 64,
                "every admitted anchor needs all unchanged 64 true future labels")
        v0 = math.hypot(row.features[1], row.features[2])
        require(0 < v0 <= 2.5, "TRAIN current speed outside cruise envelope; never clipped")
        future_speed = [float(v) for v in row.target_speed_mps]
        require(all(math.isfinite(v) and 0 < v <= 2.5 for v in future_speed),
                "TRAIN future speed outside cruise envelope; never clipped")
        previous_xy, arc = (0., 0.), 0.
        for xy, actual in zip(row.targets_xy, future_speed):
            distance = math.dist(previous_xy, xy); arc += distance; previous_xy = xy
            chord_errors.append(abs(distance/.1-actual))
        xy, mask = dataset._route(row)
        nav_support = float(causal_route_support(xy[None], mask[None]))
        require(nav_support >= PathTimeDecoderV2().config.go_active_distance_m,
                "causal navigation lacks the unchanged GO preview; future labels are not support")
        spatial = spatial_expert_labels(row)
        require(int(spatial["spatial_speed_valid"].sum()) >= 2, "not enough observed raw spatial labels")
        curvatures.extend(spatial["spatial_curvature"][spatial["spatial_curvature_valid"]].tolist())
        current.append(v0); speeds.extend(future_speed); arcs.append(arc); support.append(nav_support)
    def range_summary(values):
        return dict(minimum=min(values), mean=sum(values)/len(values), maximum=max(values))
    return dict(anchor_count=len(current), full_true_target_points=len(speeds),
                current_speed_mps=range_summary(current), true_instantaneous_speed_mps=range_summary(speeds),
                observed_future_arc_m=range_summary(arcs), causal_navigation_support_m=range_summary(support),
                absolute_chord_vs_instantaneous_speed_difference_mps=range_summary(chord_errors),
                observed_spatial_curvature_inv_m=range_summary(curvatures),
                labels_clipped=False, future_distance_used_as_decoder_support=False,
                future_labels_used_as_model_input=False)


def load_train(path, manifest_sha, conversion_sha):
    root = path.resolve(strict=True)
    require(root.is_relative_to(PERSONAL.resolve()), "new TRAIN must stay within personal hwanhong")
    require(digest(root / "dataset.json") == manifest_sha, "externally pinned TRAIN manifest differs")
    conversion_path = root / "moving_conversion_qualification.json"
    require(digest(conversion_path) == conversion_sha, "externally pinned conversion result differs")
    conversion = read(conversion_path)
    require(conversion.get("schema") == CONVERSION_SCHEMA
            and conversion.get("status") == "QUALIFIED_RESEARCH_TRAIN_COMMON10"
            and conversion.get("research_training_authorized") is True
            and conversion.get("vehicle_control_approved") is False
            and conversion.get("train_examples") == 327 and conversion.get("episode_count") == 1
            and conversion.get("val_examples") == conversion.get("test_examples") == 0
            and conversion.get("dataset_manifest_sha256") == manifest_sha
            and conversion.get("exact_native_ego_navigation_and_target_reconstruction") is True
            and conversion.get("raw_native_denials_overridden") is False,
            "only the separately qualified 327-anchor research TRAIN cohort is legal")
    manifest = read(root / "dataset.json")
    require(len(manifest["episodes"]) == 1, "conversion must retain exactly one original episode")
    episode_path = _safe_file(root, manifest["episodes"][0]["manifest"], "admitted TRAIN episode")
    episode_root = episode_path.parent
    receipt_path = _safe_file(episode_root, "moving_qualification.json", "copied native qualification receipt")
    native_path = _safe_file(episode_root, "native_manifest.json", "copied original native manifest")
    native_route = _safe_file(episode_root, "native_route.json", "copied original native route")
    require(digest(receipt_path) == conversion["qualification_receipt_sha256"],
            "copied original qualification receipt differs")
    receipt = read(receipt_path)
    require(receipt.get("schema") == "carla.town07_low_speed_v4_research_train_receipt.20261001.v1"
            and receipt.get("status") == "QUALIFIED_FOR_RESEARCH_TRAIN_CONVERSION"
            and receipt.get("research_training_authorized") is True
            and receipt.get("raw_native_denials_overridden") is False
            and receipt.get("vehicle_control_approved") is False
            and digest(native_path) == receipt["native_manifest_sha256"]
            and digest(native_route) == receipt["route_identity"]["route_sha256"],
            "qualified native evidence identity or truthful research authority differs")
    loaded = load_training_examples(root, split="train", mode="planning", check_image_hashes=True)
    require(len(loaded.examples) == 327 and len({r.episode_id for r in loaded.examples}) == 1,
            "exact one-episode TRAIN denominator differs")
    require(loaded.fingerprint_sha256 == conversion["train_loader_fingerprint_sha256"],
            "conversion loader fingerprint differs")
    metadata = load_common10_temporal_metadata(root, split="train", expected_tokens=[r.token for r in loaded.examples])
    dataset = TemporalCommon10TorchDataset(loaded.examples, raw_metadata=metadata,
                                          split="train", verify_image_sha256=True)
    require(dataset.fingerprint_sha256 == conversion["temporal_dataset_fingerprint_sha256"],
            "converted temporal dataset fingerprint differs")
    return dataset, dict(path=str(root), split="train", dataset_manifest_sha256=manifest_sha,
        conversion_result_sha256=conversion_sha, validation_fingerprint_sha256=loaded.fingerprint_sha256,
        temporal_fingerprint_sha256=dataset.fingerprint_sha256,
        conversion_qualification_receipt_sha256=conversion["qualification_receipt_sha256"],
        statistics=cohort_statistics(dataset))


def verify_checkpoint(path, expected, *, resume=False):
    require(digest(path) == expected, "externally pinned source checkpoint differs")
    checkpoint = torch.load(path, map_location="cpu")
    require(isinstance(checkpoint, dict), "source checkpoint is not a dictionary")
    for name, value in checkpoint["source_sha256"].items():
        require(digest(ROOT / name) == value, "checkpoint dependency changed: " + name)
    require(checkpoint.get("vehicle_control_approved") is False, "research checkpoint scope differs")
    if resume:
        require(checkpoint.get("schema") == SCHEMA and checkpoint.get("decoder_integrated") is True
                and type(checkpoint.get("optimizer_updates")) is int
                and 0 < checkpoint["optimizer_updates"] < MAXIMUM_UPDATES
                and checkpoint.get("examples_seen") == checkpoint["optimizer_updates"] * BATCH_SIZE
                and checkpoint.get("parent_checkpoint_sha256") == PARENT_SHA
                and checkpoint.get("stop_intent_trained") is False
                and checkpoint.get("runtime_intent") == "explicit_cruise_GO_not_head"
                and checkpoint.get("training_only") is True
                and checkpoint.get("one_episode_screening") is True
                and checkpoint.get("main_training") is False
                and isinstance(checkpoint.get("optimizer_state_dict"), dict),
                "only a complete versioned research checkpoint with exact exposure/lineage may resume")
    else:
        require(expected == PARENT_SHA and checkpoint.get("optimizer_updates") == 200
                and checkpoint.get("raw_spatial_only") is True and checkpoint.get("stop_intent_trained") is False,
                "fresh screening must warm-start from the exact observed raw200 diagnostic")
    return checkpoint


def loss_numbers(losses):
    return {name: float(value.detach()) for name, value in losses.items() if isinstance(value, torch.Tensor)}


def evaluate_train(model, decoder, dataset, device):
    # HH_261001 - Full one-episode TRAIN reconstruction, NEVER renamed DEV or independent validation.
    model.eval()
    records = []
    with torch.no_grad():
        for offset in range(0, len(dataset), BATCH_SIZE):
            indices = list(range(offset, min(offset+BATCH_SIZE, len(dataset))))
            batch = previous.move(default_collate([dataset[i] for i in indices]), device)
            _, decoded, losses = joint_forward(model, decoder, batch)
            xy_error = torch.linalg.norm(decoded["xy"][:, 0]-batch["target_xy"], dim=-1)
            speed_error = (decoded["instantaneous_speed"][:, 1:]-batch["target_speed_mps"]).abs()
            for row, index in enumerate(indices):
                records.append(dict(index=index, token=dataset.examples[index].token,
                    ade_6p4s_m=float(xy_error[row].mean()), fde_6p4s_m=float(xy_error[row, -1]),
                    instantaneous_speed_mae_mps=float(speed_error[row].mean()),
                    loss_is_batch_aggregate=loss_numbers(losses)))
    return dict(split="train", independent_validation=False, full_fixed_denominator=len(records), rows=records,
        mean_ade_6p4s_m=sum(r["ade_6p4s_m"] for r in records)/len(records),
        maximum_ade_6p4s_m=max(r["ade_6p4s_m"] for r in records),
        mean_fde_6p4s_m=sum(r["fde_6p4s_m"] for r in records)/len(records),
        mean_instantaneous_speed_mae_mps=sum(r["instantaneous_speed_mae_mps"] for r in records)/len(records))


def archive_sources(output, sources):
    for name, expected in sources.items():
        require(digest(ROOT / name) == expected, "source changed before archive")
        destination = output / "source" / name; destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("xb") as stream:
            stream.write((ROOT / name).read_bytes())


def save_checkpoint(output, step, model, optimizer, train, sources, declaration_sha, seen):
    path = output / "checkpoints" / f"step_{step:06d}.pt"
    path.parent.mkdir(exist_ok=True)
    with path.open("xb") as stream:
        torch.save(dict(schema=SCHEMA, model_config=model.config.to_dict(),
            model_state_dict={key: value.detach().cpu() for key, value in model.state_dict().items()},
            optimizer_state_dict=optimizer.state_dict(), optimizer_updates=step, examples_seen=seen,
            parent_checkpoint_sha256=PARENT_SHA, parent_source_optimizer_updates=200,
            known_total_optimizer_updates=200+step,
            source_sha256=sources, declaration_sha256=declaration_sha,
            training_dataset_fingerprint_sha256=train.fingerprint_sha256,
            training_only=True, one_episode_screening=True, decoder_integrated=True,
            raw_spatial_only=False, stop_intent_trained=False, runtime_intent="explicit_cruise_GO_not_head",
            preview_regularization="training_only_detached_last_predicted_valid_speed_to_invalid_active_preview_knots",
            vehicle_control_approved=False, model_promoted=False, main_training=False), stream)
    checkpoint_sha = digest(path)
    # HH_261001 - Export only numeric arrays for the old local Torch runtime; no pickle-dependent checkpoint loading.
    folder = output / "exports" / f"step_{step:06d}"
    folder.mkdir(parents=True, exist_ok=False)
    arrays = {key: value.detach().cpu().numpy() for key, value in model.state_dict().items()}
    require(all(value.dtype == np.float32 and np.isfinite(value).all() for value in arrays.values()),
            "local export requires finite float32 state tensors")
    npz = folder / "model_state.npz"
    with npz.open("xb") as stream:
        np.savez(stream, **arrays)
    with np.load(npz, allow_pickle=False) as archived:
        require(set(archived.files) == set(arrays)
                and all(np.array_equal(archived[name], value) for name, value in arrays.items()),
                "data-only numeric export roundtrip differs")
    metadata = dict(schema="portable_e2e.temporal_v2_low_speed_joint_export.20261001.v1",
        model_config=model.config.to_dict(), checkpoint_sha256=checkpoint_sha, npz_sha256=digest(npz),
        state_tensor_shapes={name: list(value.shape) for name, value in arrays.items()},
        state_tensor_dtypes={name: str(value.dtype) for name, value in arrays.items()}, source_sha256=sources,
        optimizer_updates=step, parent_checkpoint_sha256=PARENT_SHA,
        parent_source_optimizer_updates=200, known_total_optimizer_updates=200+step,
        training_dataset_fingerprint_sha256=train.fingerprint_sha256,
        raw_spatial_only=False, decoder_integrated=True, stop_intent_trained=False,
        runtime_intent="explicit_cruise_GO_not_head", simulation_only=True,
        preview_regularization="training_only_detached_last_predicted_valid_speed_to_invalid_active_preview_knots",
        model_promoted=False, vehicle_control_approved=False, independent_validation=False,
        one_episode_screening=True, main_training=False)
    write_new(folder / "metadata.json", metadata)
    return dict(path=str(path), sha256=checkpoint_sha, optimizer_updates=step, optimizer_state_retained=True,
                export_npz=str(npz), export_npz_sha256=metadata["npz_sha256"],
                export_metadata=str(folder / "metadata.json"), export_metadata_sha256=digest(folder / "metadata.json"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--train", type=Path, required=True)
    parser.add_argument("--train-manifest-sha256", required=True)
    parser.add_argument("--conversion-result-sha256", required=True)
    parser.add_argument("--known-dev", type=Path, required=True)
    parser.add_argument("--parent-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--updates", type=int, default=MAXIMUM_UPDATES)
    parser.add_argument("--loader-workers", type=int, default=2)
    parser.add_argument("--resume-checkpoint", type=Path)
    parser.add_argument("--resume-checkpoint-sha256")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    require(args.updates in range(500, MAXIMUM_UPDATES+1, 500), "bounded joint screening ends on a 500-update interval")
    require(args.loader_workers in (0, 1, 2), "at most two personal data-loader workers")
    require(not args.output.exists() and not args.output.is_symlink(), "create-only new training output required")
    require(args.output.resolve().is_relative_to((WORKSPACE / "runs/training").resolve()),
            "output must stay within personal runs/training")
    require((args.resume_checkpoint is None) == (args.resume_checkpoint_sha256 is None),
            "resume path and external digest must be paired")
    torch.set_num_threads(4)
    train, train_receipt = load_train(args.train, args.train_manifest_sha256, args.conversion_result_sha256)
    known, known_receipt = previous.load_view(args.known_dev, "val", previous.DEV_MANIFEST_SHA)
    require(not ({row.episode_id for row in train.examples} & {row.episode_id for row in known.examples}),
            "TRAIN/previously-used DEV episode overlap")
    probes = previous.fixed_probe(known, previous.eligible_indices(known))
    require(len(probes) == 8, "exact historical eight-probe raw regression scope required")
    parent = verify_checkpoint(args.parent_checkpoint, PARENT_SHA)
    state, start_update = parent, 0
    if args.resume_checkpoint is not None:
        state = verify_checkpoint(args.resume_checkpoint, args.resume_checkpoint_sha256, resume=True)
        require(state["training_dataset_fingerprint_sha256"] == train.fingerprint_sha256,
                "resume TRAIN fingerprint differs")
        start_update = state["optimizer_updates"]
        require(start_update < args.updates, "resume interval must precede requested final interval")
    sources = source_snapshot()
    declaration = dict(schema=SCHEMA, train=train_receipt, known_dev=known_receipt,
        known_dev_probe_indices=probes, known_dev_is_independent=False,
        parent_checkpoint_sha256=PARENT_SHA, resume_checkpoint_sha256=args.resume_checkpoint_sha256,
        start_optimizer_updates=start_update, maximum_optimizer_updates=MAXIMUM_UPDATES,
        requested_optimizer_updates=args.updates, effective_batch_size=BATCH_SIZE, seed=SEED,
        learning_rate=dict(peak=INITIAL_LR, final=FINAL_LR, warmup_updates=WARMUP_UPDATES,
                           cosine_horizon_updates=MAXIMUM_UPDATES, restart_on_resume=False),
        optimizer="AdamW", weight_decay=1e-4, preclip_gradient_norm_limit=5.,
        loader_workers=args.loader_workers, source_sha256=sources,
        objective="unchanged raw+decoded temporal loss plus .1 predicted-preview speed smoothness prior",
        maximum_owned_training_wall_seconds=MAX_WALL_SECONDS,
        preview_regularization_weight=TAIL_REGULARIZATION_WEIGHT,
        decoder_support="causal navigation valid polyline arc capped64; NOT expert future distance or free-space assurance",
        intent="explicit GO task condition; intent and stop heads remain masked and untrained",
        labels_clipped=False, labels_extrapolated=False, raw_model_output_replaced=False,
        anchors_omitted=False, one_episode_screening=True, main_training=False,
        independent_validation=False, vehicle_control_approved=False, model_promoted=False)
    if args.validate_only:
        print(json.dumps(dict(status="VALIDATED_NO_GPU_OR_OUTPUT", **declaration), indent=2, allow_nan=False))
        return 0
    with previous.gpu0_lease():
        gpu = previous.gpu_preflight()
        wall_started = time.monotonic()
        require(previous.os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8", "deterministic cuBLAS environment required")
        random.seed(SEED); torch.manual_seed(SEED); torch.cuda.manual_seed_all(SEED)
        torch.use_deterministic_algorithms(True)
        torch.backends.cudnn.benchmark = False; torch.backends.cudnn.deterministic = True
        if hasattr(torch.backends.cuda, "enable_flash_sdp"):
            torch.backends.cuda.enable_flash_sdp(False); torch.backends.cuda.enable_mem_efficient_sdp(False)
        args.output.mkdir(parents=True, exist_ok=False)
        write_new(args.output / "declaration.json", declaration)
        declaration_sha = digest(args.output / "declaration.json")
        archive_sources(args.output, sources)
        device = torch.device("cuda:0")
        model = TemporalModelV2(TemporalModelConfigV2.from_mapping(state["model_config"])).to(device)
        model.load_state_dict(state["model_state_dict"], strict=True)
        decoder = PathTimeDecoderV2().to(device)
        optimizer = torch.optim.AdamW(model.parameters(), lr=INITIAL_LR, weight_decay=1e-4)
        if start_update:
            optimizer.load_state_dict(state["optimizer_state_dict"])
        write_new(args.output / "started.json", dict(gpu=gpu, pid=previous.os.getpid(),
            started_at_utc=datetime.now(timezone.utc).isoformat(), parameters=parameter_count(model)))
        write_new(args.output / "known_dev_raw_before.json", previous.evaluate(model, known, probes, device))
        write_new(args.output / "train_joint_before.json", evaluate_train(model, decoder, train, device))
        batch_order = list(deterministic_batches(len(train), args.updates, start_update=start_update))
        loader_kwargs = dict(dataset=train, batch_sampler=batch_order, num_workers=args.loader_workers,
                             pin_memory=True)
        if args.loader_workers:
            # HH_261001 - Spawn workers so their CPU-only image loading does not inherit an initialized CUDA context.
            loader_kwargs.update(prefetch_factor=2, persistent_workers=True, multiprocessing_context="spawn")
        loader = DataLoader(**loader_kwargs)
        seen, unique, times, checkpoints = start_update*BATCH_SIZE, set(), [], []
        started = time.monotonic()
        completed_updates, wall_budget_stop = start_update, False
        with (args.output / "history.jsonl").open("x") as journal:
            for offset, batch in enumerate(loader):
                # HH_261001 - Stop only this owned fit at the declared 45-minute phase boundary, retaining resumable optimizer state.
                if time.monotonic()-wall_started >= MAX_WALL_SECONDS:
                    wall_budget_stop = True
                    break
                step = start_update+offset+1; tick = time.monotonic()
                chosen = batch_order[offset]; unique.update(chosen); seen += len(chosen)
                batch = previous.move(batch, device)
                model.train(); optimizer.zero_grad(set_to_none=True)
                rate = learning_rate(step)
                for group in optimizer.param_groups:
                    group["lr"] = rate
                _, _, losses = joint_forward(model, decoder, batch)
                losses["total"].backward()
                norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.)
                require(bool(torch.isfinite(norm)), "nonfinite gradients; no optimizer update allowed")
                optimizer.step(); torch.cuda.synchronize()
                completed_updates = step
                require(all(bool(torch.isfinite(value).all()) for value in model.state_dict().values()),
                        "nonfinite updated model state")
                elapsed = time.monotonic()-tick; times.append(elapsed)
                row = dict(step=step, optimizer_updates=step, examples_seen=seen, indices=chosen,
                    loss=loss_numbers(losses), preclip_gradient_norm=float(norm.detach()), learning_rate=rate,
                    step_compute_wall_seconds=elapsed, elapsed_training_wall_seconds=time.monotonic()-started)
                journal.write(json.dumps(row, allow_nan=False)+"\n"); journal.flush()
                if step == start_update+1 or step % 50 == 0:
                    print(json.dumps(row), flush=True)
                if step % CHECKPOINT_INTERVAL == 0:
                    checkpoint = save_checkpoint(args.output, step, model, optimizer, train, sources, declaration_sha, seen)
                    checkpoints.append(checkpoint)
                    write_new(args.output / f"train_joint_step_{step:06d}.json", evaluate_train(model, decoder, train, device))
                    write_new(args.output / f"known_dev_raw_step_{step:06d}.json", previous.evaluate(model, known, probes, device))
                    print(json.dumps(dict(status="CHECKPOINT_RETAINED_NOT_PROMOTED", **checkpoint)), flush=True)
        if wall_budget_stop:
            if completed_updates > start_update and completed_updates % CHECKPOINT_INTERVAL:
                checkpoints.append(save_checkpoint(args.output, completed_updates, model, optimizer, train,
                                                   sources, declaration_sha, seen))
            write_new(args.output / "wall_budget_stop.json", dict(status="OWNED_WALL_BUDGET_STOP_NOT_PROMOTED",
                maximum_wall_seconds=MAX_WALL_SECONDS, elapsed_wall_seconds=time.monotonic()-wall_started,
                completed_optimizer_updates=completed_updates, optimizer_state_retained=bool(checkpoints),
                other_processes_stopped=False))
        require(all(digest(ROOT / name) == expected for name, expected in sources.items()), "source changed during training")
        summary = dict(schema=SCHEMA, status=("OWNED_WALL_BUDGET_STOP_NOT_PROMOTED" if wall_budget_stop
                      else "BOUNDED_JOINT_TRAIN_SCREENING_COMPLETE_NOT_PROMOTED"),
            optimizer_updates=completed_updates, additional_optimizer_updates=completed_updates-start_update,
            examples_seen=seen, this_segment_unique_training_anchors=len(unique),
            unique_training_episodes=1, equivalent_anchor_exposure_cycles=seen/len(train),
            elapsed_training_wall_seconds=time.monotonic()-started,
            mean_step_compute_seconds=sum(times)/len(times) if times else None,
            nearest_rank_p95_step_compute_seconds=sorted(times)[math.ceil(.95*len(times))-1] if times else None,
            peak_GPU_allocated_bytes=torch.cuda.max_memory_allocated(), checkpoints=checkpoints,
            final_checkpoint_sha256=checkpoints[-1]["sha256"] if checkpoints else None,
            optimizer_state_retained=bool(checkpoints), wall_budget_stop=wall_budget_stop,
            source_hashes_match=True, labels_clipped=False, labels_extrapolated=False,
            decoder_integrated=True, stop_intent_trained=False, one_episode_screening=True,
            main_training=False, independent_validation=False, vehicle_control_approved=False, model_promoted=False)
        write_new(args.output / "summary.json", summary)
        print(json.dumps(summary), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
