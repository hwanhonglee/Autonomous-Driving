"""HH_261002 - Synthetic architecture/protocol tests, never actual TRAIN or CUDA evidence."""
import ast
import copy
import hashlib
import inspect
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning")); sys.path.insert(0, str(ROOT / "tools/e2e"))
from portable_e2e.map_context_recovery import MapContextRecovery
from portable_e2e.map_native_fusion_v1_20261002 import MapNativeFusion
from portable_e2e import map_perception
import map_native_fusion_contract_20261002 as contract
import train_map_native_fusion_pair_20261002 as study

torch.set_num_threads(1)


@pytest.fixture(scope="module")
def full_schedule():
    values = contract.schedule()
    return values, contract.schedule_proof(values)


def source_map():
    result = dict(contract.FROZEN_SOURCE_SHA)
    result.update({k: "a" * 64 for k in contract.NEW_SOURCES})
    return result


def runtime():
    names = ("python_executable", "numpy", "numpy_native", "PIL.Image", "PIL._imaging", "torch", "torch._C", "torch.lib/libtorch.so")
    return dict(python_version="synthetic", python_implementation="CPython", python_prefix="/synthetic/private/venv",
        numpy="synthetic", pillow="synthetic", torch="synthetic", torch_compiled_CUDA="synthetic",
        provider_files={k: dict(path="/synthetic/provider/" + str(i), bytes=1, sha256="b"*64) for i, k in enumerate(names)})


def authorization(proof, workers=0):
    return dict(schema=contract.AUTHORIZATION_SCHEMA, contract=contract.prepared_contract(workers), source_sha256=source_map(),
        protocol_sha256=contract.document_sha(contract.prepared_contract(workers)), equal_global_schedule_sha256=contract.document_sha(proof),
        runtime_identity=runtime(), reviewer="synthetic-not-real-authority", research_fit_authorized=True,
        native_denials_preserved=True, DEV_or_HOLDOUT_used_for_fit=False, automatic_model_promotion=False, vehicle_control_approved=False)


def test_parameter_tensor_abi_and_zero_projection():
    model = MapNativeFusion()
    assert sum(v.numel() for v in model.parameters()) == 397175
    assert sum(v.numel() for v in model.native_fusion.parameters()) == 151344
    assert len(model.state_dict()) == 70
    assert len(MapContextRecovery().state_dict()) == 52
    assert not model.native_fusion[-1].weight.count_nonzero()
    assert not model.native_fusion[-1].bias.count_nonzero()
    assert [m.dilation for m in model.native_fusion if isinstance(m, torch.nn.Conv2d)] == [(1,1), (1,1), (2,2), (4,4)]
    assert not any(isinstance(m, (torch.nn.Upsample, torch.nn.BatchNorm2d, torch.nn.Dropout)) for m in model.modules())


@pytest.mark.parametrize("shape", [(1,3,20,28), (2,3,21,31), (1,3,32,33)])
def test_zero_residual_shared_parent_bit_exact_logits_and_finite_all_present_gradients(shape):
    torch.manual_seed(71)
    parent, model = MapContextRecovery(), MapNativeFusion()
    saved = {k: v.clone() for k, v in parent.state_dict().items()}
    model.load_current_parent(saved)
    assert all(torch.equal(saved[k], model.state_dict()[k]) for k in saved)
    x = torch.randn(shape)
    actual, expected = model(x), parent(x)
    assert torch.equal(actual, expected)
    target = torch.arange(shape[0] * shape[2] * shape[3]).reshape(shape[0], shape[2], shape[3]) % 7
    loss = map_perception.semantic_loss(actual, target, torch.tensor(contract.mining.CLASS_WEIGHTS))
    loss.backward()
    assert study.all_gradients_finite(model, "candidate")
    assert all(p.grad is not None and bool(torch.isfinite(p.grad).all()) for p in model.parameters())
    # HH_261002 - Zero upstream gradients at the zero-output initialization are not missing-gradient failures or learning proof.
    assert not model.native_fusion[0].weight.grad.count_nonzero()
    assert model.native_fusion[-1].weight.grad.count_nonzero()
    assert torch.nn.utils.clip_grad_norm_(model.parameters(), 5.).isfinite()


@pytest.mark.parametrize("kind", ["missing", "extra", "shape", "double", "nan", "inf", "nonzero_projection"])
def test_parent_load_rejects_unknown_state(kind):
    model, parent = MapNativeFusion(), MapContextRecovery()
    state = {k: v.clone() for k, v in parent.state_dict().items()}
    key = next(iter(state))
    if kind == "missing": state.pop(key)
    elif kind == "extra": state["unknown"] = torch.zeros(1)
    elif kind == "shape": state[key] = state[key].reshape(-1)
    elif kind == "double": state[key] = state[key].double()
    elif kind in ("nan", "inf"): state[key].reshape(-1)[0] = float(kind)
    else: model.native_fusion[-1].bias.data[0] = 1.
    with pytest.raises(ValueError): model.load_current_parent(state)


def test_missing_nonfinite_frozen_gradient_is_denied():
    model = MapNativeFusion()
    for p in model.parameters(): p.grad = torch.zeros_like(p)
    assert study.all_gradients_finite(model, "candidate")
    p = next(model.parameters()); p.grad = None
    assert not study.all_gradients_finite(model, "candidate")
    p.grad = torch.full_like(p, float("nan"))
    assert not study.all_gradients_finite(model, "candidate")
    assert not study.all_gradients_finite(model, "control")


def test_frozen_parent_forward_statements_remain_ordered_and_original_CE_called_once():
    assert "native + residual" in inspect.getsource(MapNativeFusion.forward)
    run = ast.parse(inspect.getsource(study.run))
    ce = [n for n in ast.walk(run) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "semantic_loss"]
    assert len(ce) == 1
    assert "auxiliary" not in ast.unparse(ce[0])
    text = inspect.getsource(study)
    assert "torch.load(" not in text and "interpolate(" not in text
    assert "both_strict_CUDA_preflights_before_any_update" in text
    assert "step()" in text and "zero_grad(set_to_none=True)" in text


def test_full20k_shared_schedule_frozen_global_prefix_crop_law_and_complete_cycles(full_schedule):
    values, proof = full_schedule
    assert proof["total_exposures_per_arm"] == 160000
    assert proof["full_cycles"] == 132 and proof["last_partial_cycle"] == 412
    assert len(proof["sample_exposures"]) == 1209
    assert set(proof["sample_exposures"]) <= {132, 133}
    assert sum(proof["sample_exposures"]) == 160000
    assert np.array_equal(values["control_indices"], values["candidate_indices"])
    assert not values["shared_FALSE_view_bits"].any()
    old_ids = contract.mining.full_cycle(np.arange(1209, dtype=np.int64), 32000, contract.SEED).reshape(4000,8)
    _, old_crops = contract.data.frozen.schedule(contract.SEED)
    assert np.array_equal(values["control_indices"][:4000], old_ids)
    assert np.array_equal(values["shared_crops"][:4000], old_crops[:4000])
    assert contract.data.frozen.STEPS == 10000 and contract.data.frozen.BATCH == 8


@pytest.mark.parametrize("change", ["index", "crop", "view", "dtype", "shape", "missing", "extra"])
def test_schedule_unknown_or_modified_tensor_is_denied(full_schedule, change):
    values = {k: v.copy() for k, v in full_schedule[0].items()}
    if change == "index": values["candidate_indices"][0,0] = (values["candidate_indices"][0,0]+1)%1209
    elif change == "crop": values["shared_crops"][0,0,0] += 1
    elif change == "view": values["shared_FALSE_view_bits"][0,0] = 1
    elif change == "dtype": values["control_indices"] = values["control_indices"].astype(np.int32)
    elif change == "shape": values["control_indices"] = values["control_indices"][:-1]
    elif change == "missing": values.pop("shared_crops")
    else: values["extra"] = np.zeros(1)
    with pytest.raises(ValueError): contract.schedule_proof(values)


def test_contract_one_architecture_factor_honest_counts_and_no_adoption():
    value = contract.prepared_contract(2)
    assert value["parameter_count_by_arm"] == {"parent":245831, "control":245831, "candidate":397175}
    assert value["additional_updates_per_arm"] == 20000 and value["total_lineage_updates_per_arm"] == 30000
    assert value["candidate_new_branch_initial_completed_updates"] == 0
    assert value["candidate_new_branch_final_completed_updates"] == 20000
    assert value["examples_per_arm"] == 160000 and value["physical_TRAIN_slots"] == 1209
    assert value["loader_workers"] == 2 and value["auxiliary_loss"] is False
    assert value["new_head"] is False and value["new_inputs"] is False
    assert value["all_existing_27_screens_unchanged"] and value["known350_previously_exposed_not_independent"]
    assert value["class_weights"] == contract.mining.CLASS_WEIGHTS
    assert value["initial_model_weights_sha256"].startswith("2284511d")
    assert value["main_capabilities_approved"] == value["subfunctions_approved"] == 0
    assert not any(value[k] for k in ("training_authorized", "model_promoted", "vehicle_control_approved", "inference_latency_measured", "hundred_ms_or_live_mapping_approved"))


@pytest.mark.parametrize("workers", [True, False, 1, 3, -1, "2", 2.])
def test_worker_count_is_typed_exact(workers):
    with pytest.raises(ValueError): contract.prepared_contract(workers)


def test_source_thirteen_flat_bytes_and_no_frozen_edits(tmp_path):
    for name in contract.SOURCE_NAMES:
        src = ROOT / ("learning/portable_e2e" if name in {"map_perception.py", "map_context_recovery.py", "map_sign_margin_loss_v1_20261002.py", "map_native_fusion_v1_20261002.py"} else "tools/e2e") / name
        shutil.copyfile(src, tmp_path / name)
    source = contract.source_snapshot(tmp_path)
    assert len(source) == 13
    assert all(source[k] == v for k,v in contract.FROZEN_SOURCE_SHA.items())
    (tmp_path / "map_perception.py").write_text("synthetic mutated source")
    with pytest.raises(ValueError): contract.source_snapshot(tmp_path)


def test_exact_external_authority_and_runtime_required(full_schedule):
    proof = full_schedule[1]
    value = authorization(proof, 2)
    contract.validate_authorization(value, source_map(), proof, 2)


@pytest.mark.parametrize("key,value", [("schema", "old"), ("protocol_sha256", "b"*64), ("equal_global_schedule_sha256", "b"*64),
    ("research_fit_authorized", False), ("research_fit_authorized", 1), ("native_denials_preserved", False),
    ("DEV_or_HOLDOUT_used_for_fit", True), ("automatic_model_promotion", True), ("vehicle_control_approved", True),
    ("reviewer", ""), ("runtime_identity", None)])
def test_authority_negative(full_schedule, key, value):
    actual = authorization(full_schedule[1]); actual[key] = value
    with pytest.raises(ValueError): contract.validate_authorization(actual, source_map(), full_schedule[1], 0)


def test_authority_wrong_worker_or_float_representation_denied(full_schedule):
    value = authorization(full_schedule[1], 0)
    with pytest.raises(ValueError): contract.validate_authorization(value, source_map(), full_schedule[1], 2)
    value["contract"]["gradient_norm_cap"] = 5
    with pytest.raises(ValueError): contract.validate_authorization(value, source_map(), full_schedule[1], 0)


class SyntheticCropBatches(study.CropBatches):
    def __getstate__(self):
        # HH_261002 - Test fixture ONLY: keep tiny generated pixels; production workers always authenticate actual TRAIN on spawn.
        return self.__dict__.copy()


def tiny_pixels():
    rgb = np.zeros((2,360,640,3), dtype=np.uint8)
    rgb[1, :40, :60, 0] = 171
    target = np.zeros((2,360,640), dtype=np.uint8); target[1, :40, :60] = 5
    ids = np.tile(np.arange(8) % 2, (3,1)).astype(np.int64)
    boxes = np.tile(np.asarray([0,0,360,640], dtype=np.int16), (3,8,1))
    bits = np.zeros((3,8), dtype=np.uint8)
    return (rgb, target, rgb[:0], rgb[:0], target[:0]), dict(control_indices=ids, candidate_indices=ids.copy(), shared_crops=boxes, shared_FALSE_view_bits=bits)


def test_real_DataLoader_spawn0_2_synthetic_pixels_byte_order_parity():
    arrays, values = tiny_pixels()
    dataset = SyntheticCropBatches(arrays, values, {})
    result = study.loader_parity(dataset)
    assert result["byte_equal"] and result["loader0"] == result["loader2"]
    assert [v["step"] for v in result["loader0"]] == [0,1]


def test_spawn_production_state_never_pickles_actual_arrays():
    arrays, values = tiny_pixels()
    dataset = study.CropBatches(arrays, values, {})
    assert dataset.__getstate__()["arrays"] is None
    with pytest.raises(ValueError): dataset[-1]
    with pytest.raises(ValueError): dataset[True]


def toy_journals(tmp_path, monkeypatch):
    monkeypatch.setattr(study, "STEPS", 3)
    monkeypatch.setattr(study, "BATCH", 8)
    _, values = tiny_pixels()
    rows, history = [], []
    for i in range(3):
        row = dict(step=i+1, physical_slot_indices=values["control_indices"][i].tolist(), shared_crop_boxes=values["shared_crops"][i].tolist(),
            actual_paired_TRUE_bits=values["shared_FALSE_view_bits"][i].tolist(), RGB_sha256="a"*64, target_sha256="b"*64)
        rows.append(row)
        history.append(dict(step=i+1, optimizer_updates=i+1, original_CE=.5, gradient_norm=.2, wall_seconds=i+.1, all_gradients_present_finite=True))
    def save():
        (tmp_path / "exposures.jsonl").write_text("\n".join(json.dumps(v) for v in rows)+"\n")
        (tmp_path / "history.jsonl").write_text("\n".join(json.dumps(v) for v in history)+"\n")
    save()
    return values, rows, history, save


def test_actual_journal_gate_not_boolean(tmp_path, monkeypatch):
    values, _, _, _ = toy_journals(tmp_path, monkeypatch)
    proof = study.verify_journals(tmp_path, values)
    assert proof["steps"] == 3 and proof["examples_seen"] == 24
    assert proof["full_exposure_journal_sha256"] == hashlib.sha256((tmp_path/"exposures.jsonl").read_bytes()).hexdigest()


@pytest.mark.parametrize("kind", ["missing", "extra", "order", "step_bool", "ids", "crop", "TRUE", "input_sha", "updates", "nan", "grad_missing", "negative_norm"])
def test_full_actual_journal_rejects_partial_or_modified(tmp_path, monkeypatch, kind):
    values, rows, history, save = toy_journals(tmp_path, monkeypatch)
    if kind == "missing": rows.pop()
    elif kind == "extra": history.append(copy.deepcopy(history[-1]))
    elif kind == "order": rows.reverse()
    elif kind == "step_bool": rows[0]["step"] = True
    elif kind == "ids": rows[0]["physical_slot_indices"][0] = 9
    elif kind == "crop": rows[0]["shared_crop_boxes"][0][0] = 1
    elif kind == "TRUE": rows[0]["actual_paired_TRUE_bits"][0] = 1
    elif kind == "input_sha": rows[0]["RGB_sha256"] = "z"*64
    elif kind == "updates": history[1]["optimizer_updates"] = 1
    elif kind == "nan": history[1]["original_CE"] = float("nan")
    elif kind == "negative_norm": history[1]["gradient_norm"] = -.1
    else: history[1]["all_gradients_present_finite"] = False
    save()
    with pytest.raises(ValueError): study.verify_journals(tmp_path, values)


def test_real_model_factory_and_arm_aware_numeric_endpoint_initialization(tmp_path, monkeypatch):
    _, _, _, _, guard = study.providers()
    parent = MapContextRecovery()
    with (tmp_path / "synthetic.npz").open("wb") as stream:
        np.savez_compressed(stream, **{k:v.detach().numpy() for k,v in parent.state_dict().items()})
    monkeypatch.setattr(contract.previous, "INITIAL_STATE_SHA", guard.state_digest(parent))
    digest = contract.data.digest(tmp_path / "synthetic.npz")
    control = study.initialize("control", tmp_path / "synthetic.npz", digest)
    candidate = study.initialize("candidate", tmp_path / "synthetic.npz", digest)
    assert len(control.state_dict()) == 52 and len(candidate.state_dict()) == 70
    x = torch.randn(1,3,20,28)
    assert torch.equal(control(x), candidate(x))
    with pytest.raises(ValueError): study.factory("unknown")


def endpoint_fixture(directory, arm, monkeypatch):
    monkeypatch.setattr(study, "STEPS", 3)
    _, _, _, _, guard = study.providers()
    model = study.factory(arm)
    state = {k: v.detach().numpy().copy() for k,v in model.state_dict().items()}
    with (directory / "weights.npz").open("wb") as stream: np.savez_compressed(stream, **state)
    metadata = dict(schema=contract.EXPORT_SCHEMA, arm=arm, steps=3, examples_seen=24, total_lineage_updates=30000,
        parameter_count=contract.PARAMETERS[arm], initial_model_weights_sha256=contract.mining.INITIAL_MODEL_SHA,
        initial_completed_updates=10000, optimizer_initial_state_entries=0, fixed_endpoint_only=True,
        driving101_export=False, model_promoted=False, vehicle_control_approved=False,
        shared_parent_completed_updates_after_fit=30000, new_branch_completed_updates=3 if arm == "candidate" else 0,
        state_tensor_abi={k:dict(shape=list(v.shape), dtype=str(v.dtype)) for k,v in state.items()},
        weights_sha256=contract.data.digest(directory / "weights.npz"), final_state_sha256=guard.state_digest(model))
    def save():
        (directory / "training.json").write_text(json.dumps(metadata))
        return dict(metadata, metadata_sha256=contract.data.digest(directory / "training.json"))
    return metadata, save


@pytest.mark.parametrize("arm", ["control", "candidate"])
def test_full_arm_aware_numeric_export_state_and_honest_branch_lineage(tmp_path, monkeypatch, arm):
    metadata, save = endpoint_fixture(tmp_path, arm, monkeypatch)
    info = save()
    model = study.numeric_endpoint(tmp_path, arm, info)
    assert len(model.state_dict()) == (52 if arm == "control" else 70)
    assert metadata["new_branch_completed_updates"] == (0 if arm == "control" else 3)


@pytest.mark.parametrize("field,value", [("steps", True), ("steps", 4000), ("examples_seen", 32000),
    ("total_lineage_updates", 14000), ("parameter_count", 245831), ("schema", "old101"),
    ("model_promoted", True), ("vehicle_control_approved", True), ("fixed_endpoint_only", False),
    ("driving101_export", True), ("initial_completed_updates", True), ("optimizer_initial_state_entries", False),
    ("state_tensor_abi", {}), ("final_state_sha256", "b"*64), ("new_branch_completed_updates", 30000)])
def test_numeric_export_wrong_architecture_lineage_fixed_metadata_denied(tmp_path, monkeypatch, field, value):
    metadata, save = endpoint_fixture(tmp_path, "candidate", monkeypatch)
    metadata[field] = value
    with pytest.raises(ValueError): study.numeric_endpoint(tmp_path, "candidate", save())


def test_full_endpoint_source_has_every_shape_and_model_state_key():
    assert len(study.factory("control").state_dict()) == 52
    assert len(study.factory("candidate").state_dict()) == 70
    source = inspect.getsource(study.numeric_endpoint)
    assert "model.state_dict()" in source and "expected_abi" in source
    assert "guard.state_digest(model)" in source


def test_evaluation_boundary_must_verify_numeric_and_journals_before_any_known_data(monkeypatch):
    opened = []
    def reject(*args): raise ValueError("actual endpoint gate failure")
    monkeypatch.setattr(study, "verify_both_endpoints", reject)
    monkeypatch.setattr(contract.data, "read_npz", lambda *args: opened.append(args))
    monkeypatch.setattr(contract.data, "read_json", lambda *args: opened.append(args))
    with pytest.raises(ValueError, match="actual endpoint gate"): study.evaluate({}, {}, None)
    assert opened == []


def test_original27_threshold_predicate_is_direct_delegate():
    original = contract.screens.diagnostic_screen
    assert original is not contract.original_27_screen
    source = inspect.getsource(contract.original_27_screen)
    assert "screens.diagnostic_screen(evaluation)" in source and "27" in source
    assert contract.data.digest(Path(contract.screens.__file__)) == contract.FROZEN_SOURCE_SHA[Path(contract.screens.__file__).name]


def test_fit_authority_before_payload_runtime_before_CUDA_and_both_preflights_before_any_update_AST():
    source = inspect.getsource(study.run)
    assert source.index("protocol.validate_authorization") < source.index("protocol.train_package")
    assert source.index("runtime == authorization") < source.index("protocol.train_package")
    assert source.index("strict_preflights") < source.index("for arm in ARMS") < source.index("optimizer.step()")
    assert source.index("verify_both_endpoints") < source.index("evaluate(arms")
    assert source.index("gpu0_lease()") < source.index("gpu_idle_before_cuda()") < source.index("guard.preflight_gpu()")


@pytest.mark.parametrize("rows,apps", [(["0, wrong, 0, 0"], ""), (["0, " + contract.mining.GPU_UUID + ", 1, 0"], ""),
    (["0, " + contract.mining.GPU_UUID + ", 0, 1"], ""), (["0, " + contract.mining.GPU_UUID + ", 0, 0"], "123, gpu, 0")])
def test_GPU_idle_typed_identity_no_residual_or_process_fallback(monkeypatch, rows, apps):
    responses = iter(["\n".join(rows), apps])
    monkeypatch.setattr(study.subprocess, "check_output", lambda *a,**k: next(responses))
    with pytest.raises(ValueError): study.gpu_idle_before_cuda()


def test_GPU_idle_accepts_only_exact_zero_empty_apps(monkeypatch):
    responses = iter(["0, " + contract.mining.GPU_UUID + ", 0, 0\n", ""])
    monkeypatch.setattr(study.subprocess, "check_output", lambda *a,**k: next(responses))
    study.gpu_idle_before_cuda()


def test_public_actual_workspace_contract_parse_no_authority_no_model_payload():
    result = subprocess.run([sys.executable, str(ROOT/"tools/workspace_python.py"), "-m",
        "scripts.e2e.train_map_native_fusion_pair_20261002", "--", "--mode", "contract", "--loader-workers", "2"],
        text=True, capture_output=True, timeout=15)
    assert result.returncode == 0, result.stderr
    value = json.loads(result.stdout)
    assert value["loader_workers"] == 2 and value["source_count"] == 13 and value["training_authorized"] is False


@pytest.mark.parametrize("args", [["--mode","fit"], ["--mode","contract","--validate-only"], ["--loader-workers","1"],
    ["--mode","schedule-contract","--authorization","/synthetic/not_authorized"]])
def test_public_no_accidental_fit_or_query_authorization(args):
    with pytest.raises((ValueError, SystemExit)): study.main(args)
