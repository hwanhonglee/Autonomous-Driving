"""HH_261001 - Exercise exact pinned AST selection with simulator dependency imports forbidden."""

import ast
import os
import subprocess
import sys
from types import SimpleNamespace
import pytest
import torch
from scripts.e2e import export_warmup_duration_w2_cpu_20261001 as shim


def test_selected_export_nodes_are_original_not_rewritten():
    raw=shim.pinned(shim.ROOT/shim.REPLAY_REL,shim.SOURCE_PINS[shim.REPLAY_REL])
    selected=shim.select_nodes(raw,functions=shim.FUNCTIONS,constants=shim.CONSTANTS)
    originals={ast.dump(node,include_attributes=True) for node in ast.parse(raw).body}
    assert len(selected)==len(shim.FUNCTIONS)+len(shim.CONSTANTS)
    assert all(ast.dump(node,include_attributes=True) in originals for node in selected)
    assert not any(isinstance(node,(ast.Import,ast.ImportFrom,ast.If,ast.Expr)) for node in selected)


def test_allowlist_rejects_missing_or_duplicate():
    with pytest.raises(ValueError):shim.select_nodes(b'def other(): pass',functions=('export',))
    with pytest.raises(ValueError):shim.select_nodes(b'def export(): pass\ndef export(): pass',functions=('export',))
    with pytest.raises(ValueError):shim.select_nodes(b'X=1\nX=2',constants=('X',))


def test_unlisted_top_level_side_effect_never_selected():
    raw=b'import absent_package\nraise RuntimeError("must not run")\nX=3\ndef export(): return X\n'
    nodes=shim.select_nodes(raw,functions=('export',),constants=('X',))
    namespace={};exec(compile(ast.Module(body=nodes,type_ignores=[]),'test','exec'),namespace)
    assert namespace['export']()==3


def test_pins_fail_before_compilation(tmp_path):
    path=tmp_path/'source.py';path.write_bytes(b'changed')
    with pytest.raises(ValueError):shim.pinned(path,'0'*64)


def test_exact_tensor_digest_and_endpoint_counts():
    core=shim.make_core()
    from scripts.e2e.run_moving_multiroute_recovery_k import tensor_digest
    values={'a':torch.arange(6,dtype=torch.float32).reshape(2,3),'b':torch.zeros(1)}
    assert core['tensor_digest'](values)==tensor_digest(values)
    assert core['FINAL_FIT']['global_step']==11996 and core['FIRST_FIT']['global_step']==2999
    assert core['w1'].CHECKPOINT==shim.W1_CHECKPOINT
    with pytest.raises(ValueError):core['validate_payload']({})


def test_identity_uses_real_research_module_and_restores():
    core=shim.make_core();research=core['research']
    old=research.AH_CHECKPOINT_SHA256,research.CHECKPOINT_SCHEMA
    with pytest.raises(RuntimeError):
        with core['identity']('W2','c'*64):
            assert research.AH_CHECKPOINT_SHA256=='c'*64
            assert research.CHECKPOINT_SCHEMA==core['W2_SCHEMA']
            raise RuntimeError('injected')
    assert (research.AH_CHECKPOINT_SHA256,research.CHECKPOINT_SCHEMA)==old


def test_fresh_cpu_process_with_yaml_and_collectors_forbidden():
    code="""
import importlib.abc, sys, runpy
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('yaml','carla','agents') or fullname.startswith('scripts.e2e.collect_'):
            raise AssertionError('forbidden import: '+fullname)
sys.meta_path.insert(0,Block())
runpy.run_path('tools/workspace_python.py')['configure']()
from scripts.e2e import export_warmup_duration_w2_cpu_20261001 as shim
shim.main(['--mode','contract'])
"""
    env=dict(os.environ,CUDA_VISIBLE_DEVICES='',PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2')
    result=subprocess.run([sys.executable,'-B','-c',code],cwd=shim.ROOT,env=env,
        capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stderr
    assert 'CPU_EXPORT_IMPORT_AND_SOURCE_PASS_NO_CHECKPOINT' in result.stdout
