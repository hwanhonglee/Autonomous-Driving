"""HH_261001 - Execute only pinned unchanged CPU export definitions, without simulator imports."""

import argparse
import ast
from contextlib import contextmanager
import hashlib
import inspect
import json
import os
from pathlib import Path
import re
import sys
from types import SimpleNamespace
from typing import Any, Mapping

ROOT = Path(__file__).resolve().parents[2]
PLAN_REL = 'docs/assets/validation/2026-10-01/selector_phase_ab/W2_CPU_EXPORT_SHIM_ADDENDUM.md'
PLAN_SHA = 'c04d79f32aa1c703d2907e32974a5bd5573a2a93f380250474b5615d71970e95'
REPLAY_REL = 'tools/e2e/research_warmup_duration_w2_replay_20261001.py'
BASE_REL = 'tools/e2e/run_moving_multiroute_recovery_k.py'
W1_REL = 'tools/e2e/collect_w1_town07_long_20261001.py'
SOURCE_PINS = {
    REPLAY_REL: '07dd532b9d193b39ab7b0987110c4ced024e5d07f041ae6b4be3758aa6a54528',
    BASE_REL: '9044dd3fc421e5738ef58b2fdae189402db76db60b2d305d82cc897bdfc84944',
    W1_REL: '53da4cc6786ee6f9f68b43fad5c1d68bfac60e0e66a50624245902de5fc0b6a3',
    'learning/portable_e2e/simulation_research.py': '4a66e2d3b522272822ddb43c43edeed1ebc6a2ee6c20bd0cda6ce2b3ea6c9252',
    PLAN_REL: PLAN_SHA,
}
FUNCTIONS = ('require', 'digest', '_stream_digest', 'json_digest', 'read_json', 'write_new',
             'exact_sha', 'validate_first', 'validate_payload', 'identity', 'export')
CONSTANTS = ('ROOT', 'PLAN', 'PLAN_SHA', 'TRAIN_PLAN_SHA', 'TRAIN_SOURCE_SHA', 'W2_SCHEMA',
             'W1_SCHEMA', 'BINDING_SCHEMA', 'W2_ARM', 'PARENT_SHA', 'W1_STATE', 'MASK_SHA',
             'ORDER_SHA', 'DATASET_SHA', 'FIRST_FIT', 'FINAL_FIT')
W1_CHECKPOINT = 'b9ee946ada6824e7d79f51bc1693463dc506db3faaa9038ddaa05816cf7ecf76'
NEW_FILES = ('tools/e2e/export_warmup_duration_w2_cpu_20261001.py',
             'learning/tests/test_warmup_duration_w2_cpu_export_20261001.py', PLAN_REL)


def pinned(path, expected):
    if path.is_symlink() or not path.is_file():
        raise ValueError('regular pinned export source required: ' + str(path))
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected:
        raise ValueError('pinned export source differs: ' + str(path))
    return raw


def select_nodes(raw, *, functions=(), constants=()):
    """HH_261001 - Select named original AST nodes verbatim; never run original imports or top-level code."""
    selected, seen_functions, seen_constants = [], set(), set()
    for node in ast.parse(raw).body:
        if isinstance(node, ast.FunctionDef) and node.name in functions:
            if node.name in seen_functions:
                raise ValueError('duplicate allowlisted function')
            seen_functions.add(node.name)
            selected.append(node)
        elif isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name in constants:
                if name in seen_constants:
                    raise ValueError('duplicate allowlisted constant')
                seen_constants.add(name)
                selected.append(node)
    if seen_functions != set(functions) or seen_constants != set(constants):
        raise ValueError('exact CPU export allowlist is incomplete')
    return selected


def make_core(root=ROOT):
    raw = {name: pinned(root / name, expected) for name, expected in SOURCE_PINS.items()}
    w1_nodes = select_nodes(raw[W1_REL], constants=('CHECKPOINT',))
    if ast.literal_eval(w1_nodes[0].value) != W1_CHECKPOINT:
        raise ValueError('actual pinned W1 checkpoint literal differs')
    # HH_261001 - These real CPU model/safe-reader imports have no CARLA collector dependency.
    import torch
    from portable_e2e import simulation_research as research
    if Path(research.__file__).resolve() != (root / 'learning/portable_e2e/simulation_research.py').resolve():
        raise ValueError('CPU model loader resolved outside pinned source')
    namespace = dict(__name__='pinned_w2_cpu_export_core', __file__=str(root / REPLAY_REL),
        contextmanager=contextmanager, hashlib=hashlib, json=json, Path=Path, re=re,
        torch=torch, research=research, Mapping=Mapping, Tensor=torch.Tensor, Any=Any,
        w1=SimpleNamespace(CHECKPOINT=ast.literal_eval(w1_nodes[0].value)))
    base_nodes = select_nodes(raw[BASE_REL], functions=('compact', 'tensor_digest'))
    export_nodes = select_nodes(raw[REPLAY_REL], functions=FUNCTIONS, constants=CONSTANTS)
    module = ast.Module(body=base_nodes + export_nodes, type_ignores=[])
    exec(compile(module, str(root / REPLAY_REL), 'exec'), namespace)
    return namespace


def receipt_sources():
    result = dict(SOURCE_PINS)
    for name in NEW_FILES:
        path = ROOT / name
        if path.is_symlink() or not path.is_file():
            raise ValueError('shim source or test missing: ' + name)
        result[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--mode', choices=('contract', 'preflight', 'export'), required=True)
    for name in ('training-output', 'parent-checkpoint', 'weights', 'binding', 'shim-receipt'):
        parser.add_argument('--' + name, type=Path)
    parser.add_argument('--checkpoint-sha256')
    args = parser.parse_args(argv)
    core = make_core()
    sources = receipt_sources()
    forbidden = [name for name in sys.modules if name in ('yaml', 'carla', 'agents')
                 or name.startswith('scripts.e2e.collect_')]
    if forbidden:
        raise ValueError('CPU export unexpectedly imported simulator dependencies: ' + ','.join(forbidden))
    proof = dict(source_sha256=sources, selected_export_functions=list(FUNCTIONS),
        selected_export_constants=list(CONSTANTS), selected_tensor_functions=['compact', 'tensor_digest'],
        original_imports_executed=False, actual_W1_checkpoint_literal=W1_CHECKPOINT,
        simulator_dependencies_imported=False, unchanged_export_function=True)
    if args.mode != 'contract':
        core['require'](os.environ.get('CUDA_VISIBLE_DEVICES') == '', 'explicit empty CUDA visibility required for CPU export')
        core['require']('weights_only' in inspect.signature(core['torch'].load).parameters,
                        'existing modern Torch weights_only support required; no fallback')
    if args.mode == 'export':
        core['require'](all(getattr(args, name) is not None for name in
            ('training_output', 'parent_checkpoint', 'weights', 'binding', 'shim_receipt', 'checkpoint_sha256')),
            'all exact export inputs and separate shim receipt required')
        core['require'](not args.shim_receipt.exists() and not args.shim_receipt.is_symlink(), 'fresh shim receipt required')
        core['require'](args.shim_receipt.resolve() not in (args.binding.resolve(), args.weights.resolve(),
            args.weights.with_suffix('.json').resolve()), 'shim evidence and original outputs must be separate')
        result = core['export'](args)
        core['require'](receipt_sources() == sources, 'source changed during CPU export')
        args.shim_receipt.parent.mkdir(parents=True, exist_ok=True)
        core['write_new'](args.shim_receipt, dict(schema='portable_e2e.w2_pure_cpu_export.v1',
            status='EXPORTED_WITH_UNCHANGED_PINNED_VALIDATION', export=result, **proof,
            optimizer_updates=0, training_data_approved=False, vehicle_control_approved=False))
        result.update(shim_receipt=str(args.shim_receipt.resolve()), shim_receipt_sha256=core['digest'](args.shim_receipt))
    else:
        result = dict(status='CPU_EXPORT_IMPORT_AND_SOURCE_PASS_NO_CHECKPOINT' if args.mode == 'contract'
                      else 'CPU_EXPORT_SECURE_LOADER_PREFLIGHT_PASS_NO_CHECKPOINT', **proof)
    print(json.dumps(result, sort_keys=True, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
