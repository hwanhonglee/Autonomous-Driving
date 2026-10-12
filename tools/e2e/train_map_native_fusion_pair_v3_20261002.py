"""HH_261002 - Fresh identity-only replay after V2's pipe failure; both arms start from the same original parent.

This is not a resume. The frozen V2 fit, GPU guards, data, CE, schedule and 27
screens are unchanged. A separate ROOT authority and durable launcher are needed.
"""
from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
from pathlib import Path
from types import FunctionType, SimpleNamespace

try:
    from . import train_map_native_fusion_pair_v2_20261002 as prior
except ImportError:
    import train_map_native_fusion_pair_v2_20261002 as prior

original, previous, require, data = prior.original, prior.previous, prior.require, prior.data
NAME = "map_native_fusion_pair_20261002_v3"
SCHEMA = "portable_e2e.map_native_fusion_pair.20261002.v3"
SOURCE = previous.WORKSPACE / "source_worktrees" / NAME
OUTPUT = previous.WORKSPACE / "runs/diagnostics" / NAME
FROZEN_SOURCE_SHA = {**prior.FROZEN_SOURCE_SHA,
    "train_map_native_fusion_pair_v2_20261002.py": "7f124ef3bc67104c483f62c16af1daad12f9db4b6829634827ca5ae59e530edd"}
SOURCE_NAMES = set(FROZEN_SOURCE_SHA) | {Path(__file__).name}
_active = False


def verify_frozen():
    prior.verify_frozen()
    require(len(FROZEN_SOURCE_SHA) == 14 and len(SOURCE_NAMES) == 15,
        "fourteen frozen sources and exactly one fresh identity wrapper required")
    require(data.digest(Path(prior.__file__).resolve()) == FROZEN_SOURCE_SHA[Path(prior.__file__).name],
        "frozen V2 infrastructure changed")


def source_snapshot(root):
    data.canonical(root)
    result = {name: data.digest(root / name) for name in sorted(SOURCE_NAMES)}
    require(len(result) == 15 and all(result[name] == pin for name, pin in FROZEN_SOURCE_SHA.items()),
        "all fourteen frozen source bytes and fresh wrapper required")
    return result


def count_revision_ast():
    # HH_261002 - Inverting the sole source-count literal proves every original authorization predicate stays intact.
    tree = ast.parse(Path(previous.__file__).read_bytes())
    old = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "validate_authorization")
    node = copy.deepcopy(old)
    hits = [n for n in ast.walk(node) if isinstance(n, ast.Constant) and type(n.value) is int and n.value == 13]
    require(len(hits) == 1, "one exact original source-count literal required")
    hits[0].value = 15
    inverse = copy.deepcopy(node)
    next(n for n in ast.walk(inverse) if isinstance(n, ast.Constant) and type(n.value) is int and n.value == 15).value = 13
    require(ast.dump(inverse) == ast.dump(old), "identity revision changed an authority predicate")
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
        return dict(old_contract(loader_workers), source_count=15, gpu_preflight_revision=dict(prior.GPU_REVISION),
            GPU_idle_precondition="exact_UUID_memory0_to16MiB_util0_and_no_compute_process_before_CUDA",
            frozen_original_study_id=previous.NAME, frozen_original_schema=previous.SCHEMA,
            fresh_revision_is_not_an_original_V1_execution=True,
            interrupted_V2_study_preserved=prior.NAME, interrupted_optimizer_or_endpoint_reused=False,
            fresh_both_arms_fixed20000_not_resume=True, infrastructure_revision="detached_regular_file_logs_only")
    namespace.update(prepared_contract=prepared_contract, source_snapshot=source_snapshot)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[count_revision_ast()], type_ignores=[])), __file__, "exec"), namespace)
    return SimpleNamespace(**namespace)


protocol = protocol_namespace()


class CropBatches(prior.CropBatches):
    # HH_261002 - Spawn readers preserve exact crop bytecode while resolving only the fresh fifteen-source identity.
    __getitem__ = FunctionType(prior.CropBatches.__getitem__.__code__,
        dict(prior.CropBatches.__getitem__.__globals__, protocol=protocol))


@contextmanager
def activated():
    global _active
    require(not _active and not prior._active and original.protocol is prior._protocol
        and original.providers is prior._providers and original.gpu_idle_before_cuda is prior._idle
        and original.CropBatches is prior._batches, "unknown or nested V3 fusion namespace")
    verify_frozen(); _active = True
    try:
        original.protocol, original.providers = protocol, prior.providers
        original.gpu_idle_before_cuda, original.CropBatches = prior.gpu_idle_before_cuda, CropBatches
        yield
    finally:
        original.protocol, original.providers = prior._protocol, prior._providers
        original.gpu_idle_before_cuda, original.CropBatches = prior._idle, prior._batches
        _active = False


def main(argv=None):
    with activated(): return original.main(argv)


if __name__ == "__main__": raise SystemExit(main())
