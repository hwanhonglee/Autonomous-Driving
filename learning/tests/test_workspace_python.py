"""HH_260906 - Test canonical namespace loading in fresh processes without changing module bytes."""
from pathlib import Path
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def run(code):
    return subprocess.run([sys.executable, '-I', '-B', '-c', code], capture_output=True, text=True,
                          env=dict(os.environ, CUDA_VISIBLE_DEVICES=''))


def bootstrap():
    return ("import importlib.util, pathlib, sys; "
            f"s=importlib.util.spec_from_file_location('hh_runner', {str(ROOT / 'tools/workspace_python.py')!r}); "
            "m=importlib.util.module_from_spec(s); s.loader.exec_module(m); ")


def test_canonical_imports_preserve_historical_source_hash():
    result = run(bootstrap() + "m.configure(); import scripts.e2e.audit_carla_turn_launch_matrix as a; "
                 "import portable_e2e.contract as c; import hashlib; "
                 f"assert pathlib.Path(a.__file__).parent == pathlib.Path({str(ROOT / 'tools/e2e')!r}); "
                 f"assert pathlib.Path(c.__file__).parent == pathlib.Path({str(ROOT / 'learning/portable_e2e')!r}); "
                 "assert hashlib.sha256(pathlib.Path(a.__file__).read_bytes()).hexdigest() == "
                 "'2f1013cdaa8b2d94329d2dce6ca5a903c772dcd5d4145e35b88c2d3422859eab'")
    assert result.returncode == 0, result.stderr


def test_foreign_namespace_is_not_silently_replaced():
    result = run(bootstrap() + "import types; x=types.ModuleType('scripts'); x.__path__=['/foreign']; "
                 "sys.modules['scripts']=x; m.configure()")
    assert result.returncode != 0
    assert 'already loaded' in result.stderr


def test_configuration_is_idempotent():
    result = run(bootstrap() + "m.configure(); before=list(sys.path); m.configure(); assert sys.path==before; import scripts; "
                 f"assert scripts.__path__ == [{str(ROOT / 'tools')!r}]")
    assert result.returncode == 0, result.stderr
