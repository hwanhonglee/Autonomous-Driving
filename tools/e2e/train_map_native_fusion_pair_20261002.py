"""HH_261002 - Prepare a same-parent, same-original-CE architecture-only pair.

Only an external ROOT authority permits private physical-GPU0 execution.  The
known350 diagnostic opens after BOTH verified numeric final20k endpoints.  This
module does not run CARLA, install providers, load pickle or deploy a model.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gc
import hashlib
import importlib
import json
from pathlib import Path
import random
import subprocess
import sys
import time

import numpy as np

sys.dont_write_bytecode = True
try:
    from . import map_native_fusion_contract_20261002 as protocol
except ImportError:
    import map_native_fusion_contract_20261002 as protocol

require, data, mining = protocol.require, protocol.data, protocol.mining
STEPS, BATCH, ARMS = protocol.STEPS, protocol.BATCH, protocol.ARMS


def providers():
    # HH_261002 - Lazy imports keep contract/schedule queries free of model construction and CUDA initialization.
    import torch
    try:
        from portable_e2e import map_native_fusion_v1_20261002 as fusion
        from portable_e2e import map_context_recovery as context
        from portable_e2e import map_perception as perception
    except ImportError:
        import map_native_fusion_v1_20261002 as fusion
        import map_context_recovery as context
        import map_perception as perception
    try:
        from . import train_map_photometric_pilot as guard
    except ImportError:
        import train_map_photometric_pilot as guard
    return torch, fusion, context, perception, guard


def factory(arm):
    require(arm in protocol.PARAMETERS, "unknown model architecture arm")
    _, fusion, context, _, _ = providers()
    return fusion.MapNativeFusion() if arm == "candidate" else context.MapContextRecovery()


def initialize(arm, parent_path, parent_sha):
    model = factory(arm)
    torch, _, _, _, guard = providers()
    require(sum(p.numel() for p in model.parameters()) == protocol.PARAMETERS[arm], "honest arm parameter ABI required")
    if arm == "candidate":
        state = data.read_npz(parent_path, parent_sha)
        model.load_current_parent({k: torch.from_numpy(v.copy()) for k, v in state.items()})
        shared = factory("parent")
        shared.load_state_dict({k: v for k, v in model.state_dict().items() if not k.startswith("native_fusion.")}, strict=True)
        require(guard.state_digest(shared) == protocol.previous.INITIAL_STATE_SHA, "candidate shared parent state differs")
    else:
        protocol.previous.previous.load_weights(model, parent_path, parent_sha)
        require(guard.state_digest(model) == protocol.previous.INITIAL_STATE_SHA, "exact current228 initial state required")
    return model


class CropBatches:
    """HH_261002 - Spawned CPU-only readers use the same frozen crop function and full declared TRAIN package."""
    def __init__(self, arrays, schedule, source):
        self.arrays, self.schedule, self.source = arrays, schedule, source

    def __len__(self):
        return len(self.schedule["control_indices"])

    def __getstate__(self):
        value = dict(self.__dict__)
        value["arrays"] = None  # HH_261002 - Do not pickle multi-GiB arrays or inherit a CUDA context into workers.
        return value

    def __getitem__(self, step):
        require(type(step) is int and 0 <= step < len(self), "exact ordered batch index required")
        if self.arrays is None:
            import torch
            require(not torch.cuda.is_initialized(), "spawned reader must never initialize CUDA")
            require(protocol.source_snapshot(protocol.SOURCE) == self.source, "worker source bytes differ")
            _, self.arrays, _, _ = protocol.train_package()
        ids, boxes, bits = (self.schedule[k][step] for k in
                           ("control_indices", "shared_crops", "shared_FALSE_view_bits"))
        rgb, target, actual = protocol.previous.previous.batch_arrays(self.arrays, ids, boxes, bits, "control")
        require(np.array_equal(actual, bits), "TRUE RGB is not a training input")
        return step, rgb, target, actual


def identity_collate(value):
    return value


def loader(dataset, workers):
    import torch
    require(type(workers) is int and workers in (0, 2), "exact declared CPU worker count required")
    options = dict(batch_size=None, num_workers=workers, collate_fn=identity_collate, shuffle=False,
                   pin_memory=False)
    if workers:
        options.update(multiprocessing_context="spawn", persistent_workers=False, prefetch_factor=2)
    return torch.utils.data.DataLoader(dataset, **options)


def close_iterator(iterator):
    # HH_261002 - Join only this owned DataLoader's spawned children before endpoint verification/evaluation.
    shutdown = getattr(iterator, "_shutdown_workers", None)
    if shutdown is not None:
        shutdown()


def loader_parity(dataset):
    results = {}
    for workers in (0, 2):
        iterator = iter(loader(dataset, workers))
        try:
            rows = [next(iterator) for _ in range(2)]
            require([v[0] for v in rows] == [0, 1], "first two actual TRAIN batch order differs")
            results[workers] = [dict(step=v[0], RGB_sha256=mining.array_sha(v[1]),
                target_sha256=mining.array_sha(v[2]), FALSE_bits_sha256=mining.array_sha(v[3])) for v in rows]
        finally:
            close_iterator(iterator)
    require(results[0] == results[2], "loader0/2 actual normalized preprocessing bytes differ")
    return dict(loader0=results[0], loader2=results[2], byte_equal=True, actual_TRAIN_only=True)


def tensors(rgb, target, device):
    import torch
    require(rgb.shape == (BATCH, 360, 640, 3) and target.shape == (BATCH, 360, 640)
        and rgb.dtype == target.dtype == np.uint8 and int(target.max()) < 7, "full native B8 TRAIN input ABI differs")
    return (torch.tensor(rgb.transpose(0, 3, 1, 2).copy(), dtype=torch.float32, device=device) / 255,
            torch.tensor(target.astype(np.int64), device=device))


def all_gradients_finite(model, arm):
    import torch
    values = list(model.parameters())
    return (sum(v.numel() for v in values) == protocol.PARAMETERS[arm]
            and all(v.requires_grad and v.grad is not None and bool(torch.isfinite(v.grad).all()) for v in values))


def strict_preflights(first, parent_path, parent_sha):
    torch, _, _, perception, guard = providers()
    python_state, numpy_state = random.getstate(), np.random.get_state()
    cpu_state, cuda_state = torch.get_rng_state(), torch.cuda.get_rng_state_all()
    result, reference = {}, None
    x, y = tensors(first[1], first[2], "cuda:0")
    weights = torch.tensor(mining.CLASS_WEIGHTS, dtype=torch.float32, device="cuda:0")
    try:
        for arm in ARMS:
            guard.seed_everything(protocol.SEED)
            model = initialize(arm, parent_path, parent_sha).to("cuda:0").train()
            before = guard.state_digest(model)
            logits = model(x)
            require(logits.shape == (BATCH, 7, 360, 640) and bool(torch.isfinite(logits).all()), "finite actual TRAIN logits required")
            if reference is None:
                reference = logits.detach().clone()
            else:
                require(torch.equal(reference, logits.detach()), "zero-branch initial actual TRAIN logits are not bit-equal")
            loss = perception.semantic_loss(logits, y, weights)
            require(bool(torch.isfinite(loss)), "finite unchanged original CE required")
            loss.backward()
            require(all_gradients_finite(model, arm), "every trainable arm parameter gradient must be PRESENT and finite")
            norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.)
            require(bool(torch.isfinite(norm)) and guard.state_digest(model) == before, "preflight norm/state changed without optimizer")
            result[arm] = dict(parameter_count=protocol.PARAMETERS[arm], initial_state_sha256=before,
                shared_parent_state_sha256=protocol.previous.INITIAL_STATE_SHA,
                initial_TRAIN_logits_sha256=mining.array_sha(logits.detach().cpu().numpy()),
                all_gradients_present_finite=True, gradient_norm=float(norm), optimizer_updates=0,
                actual_original_CE=float(loss.detach()))
            del model, logits, loss
        require(result["control"]["initial_TRAIN_logits_sha256"] == result["candidate"]["initial_TRAIN_logits_sha256"], "initial logit proof differs")
        return dict(arms=result, both_preflights_before_ANY_update=True, global_optimizer_updates=0)
    finally:
        random.setstate(python_state); np.random.set_state(numpy_state)
        torch.set_rng_state(cpu_state); torch.cuda.set_rng_state_all(cuda_state)
        del x, y, reference
        torch.cuda.empty_cache()


def gpu_idle_before_cuda():
    # HH_261002 - Retain strict zero-driver-residual policy; no implicit fallback to another GPU or busy-job termination.
    rows = subprocess.check_output(["nvidia-smi", "-i", "0", "--query-gpu=index,uuid,memory.used,utilization.gpu",
        "--format=csv,noheader,nounits"], text=True, timeout=20).strip().splitlines()
    require(len(rows) == 1 and [v.strip() for v in rows[0].split(",")] == ["0", mining.GPU_UUID, "0", "0"], "strict GPU0 idle identity differs")
    apps = subprocess.check_output(["nvidia-smi", "-i", "0", "--query-compute-apps=pid,gpu_uuid,used_memory",
        "--format=csv,noheader,nounits"], text=True, timeout=20).strip()
    require(not apps, "physical GPU0 has a compute process")


def exposure_row(step, schedule, rgb, target, actual):
    ids, boxes, bits = (schedule[k][step] for k in ("control_indices", "shared_crops", "shared_FALSE_view_bits"))
    require(np.array_equal(actual, bits), "actual FALSE view bits differ")
    return dict(step=step + 1, physical_slot_indices=ids.tolist(), shared_crop_boxes=boxes.tolist(),
        actual_paired_TRUE_bits=actual.tolist(), RGB_sha256=mining.array_sha(rgb), target_sha256=mining.array_sha(target))


def verify_journals(directory, schedule):
    exposure_path, history_path = directory / "exposures.jsonl", directory / "history.jsonl"
    rows = [json.loads(v) for v in data.payload(exposure_path)[0].splitlines()]
    history = [json.loads(v) for v in data.payload(history_path)[0].splitlines()]
    require(len(rows) == len(history) == STEPS, "ALL20000 actual update/exposure rows required")
    ids_sha, crop_sha, bit_sha, input_sha = (hashlib.sha256() for _ in range(4))
    for i, (row, update) in enumerate(zip(rows, history)):
        ids, boxes, bits = (schedule[k][i] for k in ("control_indices", "shared_crops", "shared_FALSE_view_bits"))
        require(type(row.get("step")) is int and row["step"] == i + 1
            and row.get("physical_slot_indices") == ids.tolist() and row.get("shared_crop_boxes") == boxes.tolist()
            and row.get("actual_paired_TRUE_bits") == bits.tolist()
            and protocol.sha(row.get("RGB_sha256")) and protocol.sha(row.get("target_sha256")), "actual full input chronology differs")
        require(type(update.get("step")) is int and update["step"] == i + 1
            and update.get("optimizer_updates") == i + 1 and type(update["optimizer_updates"]) is int
            and all(type(update.get(k)) in (int, float) and np.isfinite(update[k]) for k in ("original_CE", "gradient_norm", "wall_seconds"))
            and all(update[k] >= 0 for k in ("original_CE", "gradient_norm", "wall_seconds"))
            and update.get("all_gradients_present_finite") is True, "actual finite update chronology differs")
        ids_sha.update(ids.tobytes()); crop_sha.update(boxes.tobytes()); bit_sha.update(bits.tobytes())
        input_sha.update(bytes.fromhex(row["RGB_sha256"])); input_sha.update(bytes.fromhex(row["target_sha256"]))
    return dict(steps=STEPS, examples_seen=STEPS * BATCH, sample_sequence_sha256=ids_sha.hexdigest(),
        crop_sequence_sha256=crop_sha.hexdigest(), view_sequence_sha256=bit_sha.hexdigest(),
        actual_input_sequence_sha256=input_sha.hexdigest(), full_exposure_journal_sha256=data.digest(exposure_path),
        full_update_journal_sha256=data.digest(history_path))


def numeric_endpoint(directory, arm, info):
    # HH_261002 - The map architecture/state ABI is explicit per arm; never relabel it as a driving101 endpoint.
    torch, _, _, _, guard = providers()
    metadata = data.read_json(directory / "training.json", info["metadata_sha256"])
    require(metadata == {k: v for k, v in info.items() if k != "metadata_sha256"}
        and metadata["schema"] == protocol.EXPORT_SCHEMA and metadata["arm"] == arm
        and type(metadata["steps"]) is int and metadata["steps"] == STEPS
        and metadata["examples_seen"] == STEPS * BATCH and metadata["total_lineage_updates"] == 30000
        and metadata["parameter_count"] == protocol.PARAMETERS[arm]
        and metadata["initial_model_weights_sha256"] == mining.INITIAL_MODEL_SHA
        and metadata["model_promoted"] is False and metadata["vehicle_control_approved"] is False,
        "honest exact final20k map metadata required")
    model = factory(arm)
    expected_abi = {k: dict(shape=list(v.shape), dtype="float32") for k, v in model.state_dict().items()}
    require(metadata.get("state_tensor_abi") == expected_abi and metadata.get("fixed_endpoint_only") is True
        and metadata.get("driving101_export") is False and metadata.get("initial_completed_updates") == 10000
        and type(metadata["initial_completed_updates"]) is int
        and metadata.get("optimizer_initial_state_entries") == 0 and type(metadata["optimizer_initial_state_entries"]) is int
        and metadata.get("shared_parent_completed_updates_after_fit") == 30000
        and metadata.get("new_branch_completed_updates") == (STEPS if arm == "candidate" else 0),
        "declared per-arm tensor/lineage/optimizer ABI differs")
    protocol.previous.previous.load_weights(model, directory / "weights.npz", info["weights_sha256"])
    require(guard.state_digest(model) == metadata["final_state_sha256"], "final numeric tensor state differs")
    return model


def verify_both_endpoints(arms, schedule):
    require(type(arms) is dict and list(arms) == list(ARMS), "BOTH predeclared ordered endpoints required before evaluation")
    evidence = {}
    for arm in ARMS:
        directory = protocol.OUTPUT / arm
        actual = verify_journals(directory, schedule)
        require(all(arms[arm].get(k) == v for k, v in actual.items()), "actual final journal proof differs")
        numeric_endpoint(directory, arm, arms[arm])
        evidence[arm] = actual
    require(evidence["control"]["actual_input_sequence_sha256"] == evidence["candidate"]["actual_input_sequence_sha256"]
        and evidence["control"]["full_exposure_journal_sha256"] == evidence["candidate"]["full_exposure_journal_sha256"],
        "both arm ALL20000 actual RGB/target/crop exposures must be bit-identical")
    return evidence


def evaluate(arms, schedule, guard):
    import torch
    # HH_261002 - This non-boolean boundary actually rereads BOTH numeric payloads and every fixed journal BEFORE labels open.
    boundary = verify_both_endpoints(arms, schedule)
    previous = protocol.previous.previous
    data.read_json(previous.EVAL_DATA / "manifest.json", previous.EVAL_MANIFEST_SHA)
    output, deadline = {}, time.monotonic() + protocol.EVALUATION_BUDGET_SECONDS
    for split, (count, expected_sha) in previous.EVAL_FILES.items():
        values = data.read_npz(previous.EVAL_DATA / (split + ".npz"), expected_sha)
        require(set(values) == {"rgb", "target"}, "known diagnostic keys differ")
        rgb, target = values["rgb"], values["target"]
        require(rgb.shape == (count, 360, 640, 3) and target.shape == (count, 360, 640)
            and rgb.dtype == target.dtype == np.uint8 and int(target.max()) < 7, "all known350 native fullFOV required")
        output[split] = {}
        for arm in ("parent", *ARMS):
            model = (initialize("parent", protocol.OUTPUT / "initial_model_weights.npz", mining.INITIAL_MODEL_SHA)
                     if arm == "parent" else numeric_endpoint(protocol.OUTPUT / arm, arm, arms[arm]))
            model.to("cuda:0").eval()
            predictions, matrix = np.empty_like(target), np.zeros((7, 7), dtype=np.int64)
            with torch.no_grad():
                for begin in range(0, count, BATCH):
                    require(time.monotonic() < deadline, "bounded postfit full350 evaluation exhausted")
                    x = torch.tensor(rgb[begin:begin+BATCH].transpose(0, 3, 1, 2).copy(), dtype=torch.float32, device="cuda:0") / 255
                    logits = model(x)
                    require(bool(torch.isfinite(logits).all()), "known diagnostic logits nonfinite")
                    predicted = logits.argmax(1).cpu().numpy().astype(np.uint8)
                    predictions[begin:begin+len(predicted)] = predicted
                    matrix += np.bincount((target[begin:begin+len(predicted)].astype(np.int64)*7+predicted).ravel(), minlength=49).reshape(7, 7)
            directory = protocol.OUTPUT / ("parent_diagnostic" if arm == "parent" else arm)
            mining.reserve(protocol.OUTPUT)
            directory.mkdir(exist_ok=True)
            with (directory / (split + "_predictions.npz")).open("xb") as stream:
                np.savez_compressed(stream, prediction=predictions)
            metrics = protocol.previous.metrics(matrix)
            metrics.update(prediction_file_sha256=data.digest(directory / (split + "_predictions.npz")),
                parameter_count=protocol.PARAMETERS[arm], full_FOV=True, evaluated_frames=count,
                count=count, source_sha256=expected_sha,
                weights_sha256=mining.INITIAL_MODEL_SHA if arm == "parent" else arms[arm]["weights_sha256"],
                known_exposed_diagnostic_not_independent=True)
            output[split][arm] = metrics
            del model, logits, x
            torch.cuda.empty_cache()
        data.payload(previous.EVAL_DATA / (split + ".npz"), expected_sha)
    data.write_new(protocol.OUTPUT / "known_evaluation.json", output)
    verify_both_endpoints(arms, schedule)
    return output, boundary


def archive_inputs(source, authorization, manifest, schedule, proof, guard):
    for name in source: guard.copy_new(protocol.SOURCE / name, protocol.OUTPUT / "source" / name)
    guard.copy_new(authorization, protocol.OUTPUT / "authorization.json")
    guard.copy_new(mining.INITIAL_MODEL_PATH, protocol.OUTPUT / "initial_model_weights.npz")
    for name in mining.INITIAL_RECEIPT_SHA:
        guard.copy_new(mining.INITIAL_RUN / name, protocol.OUTPUT / "initial_lineage" / name)
    for name in protocol.previous.MINING_SHA:
        guard.copy_new(protocol.previous.MINING_OUTPUT / name, protocol.OUTPUT / "mining" / name)
    for name in ("manifest.json", "study_train_registry.json", "paired_samples.json"):
        guard.copy_new(protocol.DATA / name, protocol.OUTPUT / "inputs" / name)
    with (protocol.OUTPUT / "equal_global_schedule.npz").open("xb") as stream:
        np.savez_compressed(stream, **schedule)
    data.write_new(protocol.OUTPUT / "equal_global_schedule_proof.json", proof)


def archived_postcheck(source, manifest, authorization, authorization_sha, runtime, schedule, proof):
    protocol.postcheck(source, manifest, authorization, authorization_sha, runtime)
    for name, expected in source.items(): data.payload(protocol.OUTPUT / "source" / name, expected)
    data.payload(protocol.OUTPUT / "authorization.json", authorization_sha)
    data.payload(protocol.OUTPUT / "initial_model_weights.npz", mining.INITIAL_MODEL_SHA)
    for name, expected in mining.INITIAL_RECEIPT_SHA.items(): data.payload(protocol.OUTPUT / "initial_lineage" / name, expected)
    for name, expected in protocol.previous.MINING_SHA.items(): data.payload(protocol.OUTPUT / "mining" / name, expected)
    for name, expected in (("manifest.json", mining.DATA_MANIFEST_SHA), ("study_train_registry.json", mining.DATA_REGISTRY_SHA),
                           ("paired_samples.json", manifest["paired_samples_sha256"])):
        data.payload(protocol.OUTPUT / "inputs" / name, expected); data.payload(protocol.DATA / name, expected)
    registry = data.read_json(protocol.DATA / "study_train_registry.json", mining.DATA_REGISTRY_SHA)
    for name, expected in registry["copied_provenance_files"].items(): data.payload(protocol.DATA / name, expected)
    saved = data.read_npz(protocol.OUTPUT / "equal_global_schedule.npz", data.digest(protocol.OUTPUT / "equal_global_schedule.npz"))
    require(protocol.schedule_proof(saved) == proof
        and data.read_json(protocol.OUTPUT / "equal_global_schedule_proof.json") == proof,
        "archived full schedule differs")


def run(args):
    protocol.environment_contract(); protocol.verify_imports()
    source = protocol.source_snapshot(protocol.SOURCE)
    authority = Path(args.authorization)
    raw, authority_sha = data.payload(authority, args.authorization_sha256)
    authorization = json.loads(raw)
    schedule = protocol.schedule(); proof = protocol.schedule_proof(schedule)
    protocol.validate_authorization(authorization, source, proof, args.loader_workers)
    runtime = protocol.screens.runtime_identity()
    require(runtime == authorization["runtime_identity"], "actual current runtime differs from external authority")
    torch, fusion, context, perception, guard = providers()
    for module in (fusion, context, perception, guard):
        require(Path(module.__file__).resolve() == protocol.SOURCE / Path(module.__file__).name, "provider escaped sealed flat package")
    require(not torch.cuda.is_initialized(), "CUDA initialized before authenticated lease")
    manifest, arrays, actual_schedule, actual_proof = protocol.train_package()
    require(actual_proof == proof, "TRAIN load schedule declaration differs")
    dataset = CropBatches(arrays, actual_schedule, source)
    receipt = dict(**protocol.prepared_contract(args.loader_workers), source_sha256=source,
        authorization_sha256=authority_sha, equal_global_schedule=proof, current_runtime=runtime)
    if args.validate_only:
        print(json.dumps(dict(status="TRAIN_CPU_PREFLIGHT_PASS_NO_CUDA_NO_OUTPUT_NO_DEV", **receipt), allow_nan=False), flush=True)
        return
    torch.set_num_threads(16); torch.set_num_interop_threads(1)
    torch.use_deterministic_algorithms(True); torch.backends.cudnn.benchmark = False; torch.backends.cudnn.deterministic = True
    torch.backends.cuda.matmul.allow_tf32 = False; torch.backends.cudnn.allow_tf32 = False
    arms, counts, stage = {}, {k: 0 for k in ARMS}, "exclusive_preflight"
    with guard.gpu0_lease():
        gpu_idle_before_cuda(); gpu = guard.preflight_gpu()
        mining.reserve(protocol.OUTPUT); protocol.OUTPUT.mkdir()
        archive_inputs(source, authority, manifest, schedule, proof, guard)
        data.write_new(protocol.OUTPUT / "started.json", dict(receipt, gpu=gpu,
            created_at_utc=datetime.now(timezone.utc).isoformat(), training_authorized=True,
            global_optimizer_updates=0, no_DEV_or_HOLDOUT_opened=True))
        try:
            stage = "loader0_loader2_TRAIN_parity"
            parity = loader_parity(dataset)
            data.write_new(protocol.OUTPUT / "loader_parity.json", parity)
            stage = "both_strict_CUDA_preflights_before_any_update"
            preflight = strict_preflights(dataset[0], protocol.OUTPUT / "initial_model_weights.npz", mining.INITIAL_MODEL_SHA)
            data.write_new(protocol.OUTPUT / "strict_CUDA_preflight.json", preflight)
            deadline = time.monotonic() + protocol.PAIR_BUDGET_SECONDS
            weights = torch.tensor(mining.CLASS_WEIGHTS, dtype=torch.float32, device="cuda:0")
            for arm in ARMS:
                stage = arm + "_fixed20000_fit"
                guard.seed_everything(protocol.SEED)
                model = initialize(arm, protocol.OUTPUT / "initial_model_weights.npz", mining.INITIAL_MODEL_SHA).to("cuda:0").train()
                initial_sha = guard.state_digest(model)
                require(initial_sha == preflight["arms"][arm]["initial_state_sha256"], "preflight and training initial state differ")
                # HH_261002 - Re-seed after unequal constructor draw counts; deterministic model has no dropout/running statistics.
                guard.seed_everything(protocol.SEED)
                optimizer = torch.optim.AdamW(model.parameters(), lr=3e-5, weight_decay=1e-4)
                require(not optimizer.state, "fresh optimizer state required")
                directory = protocol.OUTPUT / arm; directory.mkdir()
                start = time.monotonic(); iterator = iter(loader(dataset, args.loader_workers))
                try:
                    with (directory / "history.jsonl").open("x") as history, (directory / "exposures.jsonl").open("x") as exposure:
                        for step in range(STEPS):
                            require(time.monotonic() < deadline, "bounded fixed paired-fit wall budget exhausted")
                            actual_step, rgb, target, bits = next(iterator)
                            require(actual_step == step, "actual DataLoader skipped/reordered a declared batch")
                            x, y = tensors(rgb, target, "cuda:0")
                            optimizer.zero_grad(set_to_none=True)
                            logits = model(x)
                            require(logits.shape == (BATCH, 7, 360, 640) and bool(torch.isfinite(logits).all()), "finite full TRAIN logits required")
                            loss = perception.semantic_loss(logits, y, weights)
                            require(bool(torch.isfinite(loss)), "unchanged original CE nonfinite")
                            loss.backward(); require(all_gradients_finite(model, arm), "missing/nonfinite trainable gradient")
                            norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.)
                            require(bool(torch.isfinite(norm)), "nonfinite aggregate gradient norm")
                            optimizer.step(); counts[arm] += 1
                            row = dict(step=step+1, optimizer_updates=counts[arm], original_CE=float(loss.detach()),
                                gradient_norm=float(norm), all_gradients_present_finite=True, wall_seconds=time.monotonic()-start)
                            history.write(json.dumps(row, allow_nan=False) + "\n")
                            exposure.write(json.dumps(exposure_row(step, schedule, rgb, target, bits), allow_nan=False) + "\n")
                            if (step+1) % 100 == 0:
                                history.flush(); exposure.flush(); mining.reserve(protocol.OUTPUT)
                                print(json.dumps(dict(arm=arm, **row)), flush=True)
                finally:
                    close_iterator(iterator)
                torch.cuda.synchronize(); require(counts[arm] == STEPS, "actual fixed optimizer count differs")
                state = {k: v.detach().cpu().numpy().copy() for k, v in model.state_dict().items()}
                require(all(v.dtype == np.float32 and bool(np.isfinite(v).all()) for v in state.values()), "finite numeric final state required")
                mining.reserve(protocol.OUTPUT)
                with (directory / "weights.npz").open("xb") as stream: np.savez_compressed(stream, **state)
                info = dict(schema=protocol.EXPORT_SCHEMA, arm=arm, **verify_journals(directory, schedule),
                    parameter_count=protocol.PARAMETERS[arm], model_architecture="MapNativeFusion" if arm == "candidate" else "MapContextRecovery",
                    initial_model_weights_sha256=mining.INITIAL_MODEL_SHA, initial_completed_updates=10000,
                    total_lineage_updates=30000, initial_state_sha256=initial_sha, final_state_sha256=guard.state_digest(model),
                    shared_parent_completed_updates_after_fit=30000, new_branch_completed_updates=STEPS if arm == "candidate" else 0,
                    weights_sha256=data.digest(directory / "weights.npz"), state_tensor_abi={k: dict(shape=list(v.shape), dtype=str(v.dtype)) for k, v in state.items()},
                    source_sha256=source, authorization_sha256=authority_sha, optimizer_initial_state_entries=0,
                    training_wall_seconds=time.monotonic()-start, fixed_endpoint_only=True,
                    model_promoted=False, vehicle_control_approved=False, driving101_export=False)
                data.write_new(directory / "training.json", info)
                arms[arm] = dict(info, metadata_sha256=data.digest(directory / "training.json"))
                del model, optimizer, state, x, y, logits, loss, iterator; gc.collect(); torch.cuda.empty_cache()
            stage = "BOTH_actual_numeric_and_full_journal_verification_before_known350"
            boundary = verify_both_endpoints(arms, schedule)
            data.write_new(protocol.OUTPUT / "fit_complete.json", dict(status="BOTH_FIXED_FINAL20000_NUMERIC_ENDPOINTS_VERIFIED_BEFORE_KNOWN_EVALUATION",
                arms=arms, actual_full_journal_evidence=boundary, known_payload_opened=False))
            stage = "postfit_known350_diagnostic_only"
            evaluation, boundary = evaluate(arms, schedule, guard)
            archived_postcheck(source, manifest, authority, authority_sha, runtime, schedule, proof)
            result = dict(receipt, status="FIXED20K_ARCHITECTURE_PAIR_DIAGNOSTIC_NOT_PROMOTED", arms_result=arms,
                known_evaluation=evaluation, exploratory_screen_result=protocol.original_27_screen(evaluation),
                actual_boundary=boundary, source_and_inputs_unchanged=True, training_authorized=True)
            data.write_new(protocol.OUTPUT / "summary.json", result)
            print(json.dumps(dict(status=result["status"], output=str(protocol.OUTPUT),
                passed_checks=sum(result["exploratory_screen_result"]["checks"].values()), check_count=27)), flush=True)
        except Exception as error:
            data.write_new(protocol.OUTPUT / "failed.json", dict(status="FAILED_RETAINED_NO_PROMOTION", stage=stage,
                error_type=type(error).__name__, error=str(error), actual_optimizer_updates=dict(counts),
                completed_arms=list(arms), all_existing_files_retained=True, model_promoted=False))
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("contract", "runtime-contract", "schedule-contract", "fit"), default="contract")
    parser.add_argument("--loader-workers", type=int, choices=(0, 2), default=0)
    parser.add_argument("--authorization"); parser.add_argument("--authorization-sha256")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    if args.mode != "fit":
        require(not args.validate_only and args.authorization is None and args.authorization_sha256 is None,
                "query modes cannot accept fit authority or validation flags")
        result = (protocol.prepared_contract(args.loader_workers) if args.mode == "contract" else
                  protocol.screens.runtime_identity() if args.mode == "runtime-contract" else protocol.schedule_proof(protocol.schedule()))
        print(json.dumps(result, indent=2, allow_nan=False)); return 0
    require(protocol.sha(args.authorization_sha256) and args.authorization is not None, "external exact ROOT fit authority required")
    run(args); return 0


if __name__ == "__main__": raise SystemExit(main())
