"""HH_261002 - Fresh GPU0 residual-only revision; the frozen thirteen-source fusion pair stays unchanged.

Preparation is not execution. ROOT must separately seal the new fourteen-source
authority and fresh paths; both fixed20k arms retain the original CE and screens.
"""
from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
import os
from pathlib import Path
from types import FunctionType, SimpleNamespace

try:
    from . import train_map_native_fusion_pair_20261002 as original
except ImportError:
    import train_map_native_fusion_pair_20261002 as original

previous, require, data = original.protocol, original.require, original.data
NAME = "map_native_fusion_pair_20261002_v2"
SCHEMA = "portable_e2e.map_native_fusion_pair.20261002.v2"
SOURCE = previous.WORKSPACE / "source_worktrees" / NAME
OUTPUT = previous.WORKSPACE / "runs/diagnostics" / NAME
GPU_REVISION = dict(schema="portable_e2e.map_fusion_GPU0_driver_residual_preflight.20261002.v2",
    physical_index=0, uuid=original.mining.GPU_UUID, maximum_driver_residual_MiB=16,
    required_utilization_percent=0, required_GPU0_compute_process_count=0,
    physical_queries_before_torch_initialization=True, original_gpu0_lease_unchanged=True,
    model_CE_data_schedule_or_screens_changed=False)
FROZEN_SOURCE_SHA = {**previous.FROZEN_SOURCE_SHA,
    "map_native_fusion_v1_20261002.py": "88d494daf864d3cb4d28bb1226c4d8e997257960a67f2138f91eac1b8325592f",
    "map_native_fusion_contract_20261002.py": "acc0b5b821302187d7a35c5e9d4208db0a4c103f0a50a21afd85e4a1161b4dc5",
    "train_map_native_fusion_pair_20261002.py": "3918b5a529f08e17cfd294d24a21fbd545ee0adccc0e1ac3adaf79c60eb3f841"}
SOURCE_NAMES = set(FROZEN_SOURCE_SHA) | {Path(__file__).name}
_protocol, _providers = original.protocol, original.providers
_idle, _batches = original.gpu_idle_before_cuda, original.CropBatches
_active = False


def verify_frozen():
    require(len(FROZEN_SOURCE_SHA) == 13 and len(SOURCE_NAMES) == 14, "thirteen original sources plus one fresh wrapper required")
    for name, expected in FROZEN_SOURCE_SHA.items():
        path = Path(__file__).resolve().parent / name
        if not path.is_file(): path = Path(__file__).resolve().parents[2] / "learning/portable_e2e" / name
        require(data.digest(path) == expected, "frozen fusion source changed: " + name)


def source_snapshot(root):
    data.canonical(root)
    result = {name: data.digest(root / name) for name in sorted(SOURCE_NAMES)}
    require(len(result) == 14 and all(result[name] == pin for name, pin in FROZEN_SOURCE_SHA.items()),
        "all thirteen frozen source bytes and new wrapper required")
    return result


def count_revision_ast():
    # HH_261002 - Authorization changes ONLY its source-count literal; inverse AST proves every original authority predicate.
    tree = ast.parse(Path(previous.__file__).read_bytes())
    old = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "validate_authorization")
    node = copy.deepcopy(old)
    hits = [n for n in ast.walk(node) if isinstance(n, ast.Constant) and type(n.value) is int and n.value == 13]
    require(len(hits) == 1, "one exact authorization source-count literal required")
    hits[0].value = 14
    inverse = copy.deepcopy(node)
    next(n for n in ast.walk(inverse) if isinstance(n, ast.Constant) and type(n.value) is int and n.value == 14).value = 13
    require(ast.dump(inverse) == ast.dump(old), "authority revision exceeded the source-count change")
    return node


def protocol_namespace():
    namespace = dict(vars(previous), __file__=__file__, NAME=NAME, SCHEMA=SCHEMA,
        EXPORT_SCHEMA=SCHEMA + ".numeric_map_endpoint", AUTHORIZATION_SCHEMA=SCHEMA + ".authorization",
        SCHEDULE_SCHEMA=SCHEMA + ".equal_global_schedule", SOURCE=SOURCE, OUTPUT=OUTPUT,
        FROZEN_SOURCE_SHA=FROZEN_SOURCE_SHA, SOURCE_NAMES=SOURCE_NAMES)
    for name, value in vars(previous).items():
        if isinstance(value, FunctionType) and value.__globals__ is vars(previous):
            namespace[name] = FunctionType(value.__code__, namespace, name, value.__defaults__, value.__closure__)
    old_contract = namespace["prepared_contract"]
    def prepared_contract(loader_workers=0):
        return dict(old_contract(loader_workers), source_count=14, gpu_preflight_revision=dict(GPU_REVISION),
            GPU_idle_precondition="exact_UUID_memory0_to16MiB_util0_and_no_compute_process_before_CUDA",
            frozen_original_study_id=previous.NAME, frozen_original_schema=previous.SCHEMA,
            fresh_revision_is_not_an_original_V1_execution=True)
    namespace.update(prepared_contract=prepared_contract, source_snapshot=source_snapshot)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[count_revision_ast()], type_ignores=[])), __file__, "exec"), namespace)
    return SimpleNamespace(**namespace)


protocol = protocol_namespace()


def physical_idle(torch):
    # HH_261002 - BOTH guards independently query the same physical GPU before any CUDA initialization; no process is ignored.
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == original.mining.GPU_UUID and not torch.cuda.is_initialized(),
        "exact visible physical GPU0 and uninitialized CUDA required")
    rows = original.subprocess.check_output(["nvidia-smi", "-i", "0",
        "--query-gpu=index,uuid,memory.used,utilization.gpu", "--format=csv,noheader,nounits"], text=True, timeout=20).strip().splitlines()
    apps = original.subprocess.check_output(["nvidia-smi", "-i", "0",
        "--query-compute-apps=pid,gpu_uuid,used_memory", "--format=csv,noheader,nounits"], text=True, timeout=20)
    fields = [v.strip() for v in rows[0].split(",")] if len(rows) == 1 else []
    require(len(fields) == 4 and fields[:2] == ["0", original.mining.GPU_UUID]
        and fields[2].isdecimal() and fields[3].isdecimal() and 0 <= int(fields[2]) <= 16
        and int(fields[3]) == 0 and not apps.strip(),
        "GPU0 needs zero compute processes/utilization and only 0..16MiB driver residue")
    return int(fields[2])


def gpu_idle_before_cuda():
    physical_idle(_providers()[0])


def preflight_gpu():
    torch, _, _, _, _ = _providers()
    residual = physical_idle(torch)
    require(torch.cuda.is_available() and torch.cuda.device_count() == 1, "exactly one approved CUDA device required")
    props = torch.cuda.get_device_properties(torch.device("cuda:0"))
    require(str(getattr(props, "uuid", "")) in (original.mining.GPU_UUID, original.mining.GPU_UUID.removeprefix("GPU-")),
        "Torch physical GPU0 UUID differs")
    return dict(physical_index=0, uuid=original.mining.GPU_UUID, memory_used_mib_before_torch=residual,
        utilization_percent_before_torch=0, GPU0_compute_process_count=0, name=props.name,
        gpu_preflight_revision=dict(GPU_REVISION))


def providers():
    torch, fusion, context, perception, guard = _providers()
    require(guard.GPU0_UUID == original.mining.GPU_UUID and Path(guard.__file__).name == "train_map_photometric_pilot.py",
        "exact frozen GPU provider required")
    # HH_261002 - Private guard namespace preserves the original lease object and all mathematical helpers without mutating its module.
    return torch, fusion, context, perception, SimpleNamespace(**dict(vars(guard), preflight_gpu=preflight_gpu))


class CropBatches(_batches):
    # HH_261002 - Spawned CPU readers retain EXACT dataset bytecode but resolve the fresh14-source namespace, never the frozen V1 path.
    __getitem__ = FunctionType(_batches.__getitem__.__code__, dict(_batches.__getitem__.__globals__, protocol=protocol))


@contextmanager
def activated():
    global _active
    require(not _active and original.protocol is _protocol and original.providers is _providers
        and original.gpu_idle_before_cuda is _idle and original.CropBatches is _batches,
        "unknown or nested fusion/GPU/source namespace")
    verify_frozen(); _active = True
    try:
        original.protocol, original.providers = protocol, providers
        original.gpu_idle_before_cuda, original.CropBatches = gpu_idle_before_cuda, CropBatches
        yield
    finally:
        original.protocol, original.providers = _protocol, _providers
        original.gpu_idle_before_cuda, original.CropBatches = _idle, _batches
        _active = False


def main(argv=None):
    with activated(): return original.main(argv)


if __name__ == "__main__": raise SystemExit(main())
