"""HH_260906 - Permit only declared location fields, never source/binary hash drift."""
import hashlib
import importlib.util
import json
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('runtime_receipt_relocation',ROOT/'tools/rebase_runtime_provenance.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_only_location_fields_change():
    old={'path':'build/lib.so','source_paths':['src/a.cpp'],'sha256':'a'*64,'stage':'build',
         'runtime_resolution':{'library':'install/lib.so'}}
    current=module.relocate(old)
    assert current=={'path':'autonomy/build/lib.so','source_paths':['autonomy/src/a.cpp'],
                     'sha256':'a'*64,'stage':'build','runtime_resolution':{'library':'autonomy/install/lib.so'}}
    assert current!=dict(current,sha256='b'*64)
    assert old['path']=='build/lib.so'


def test_aggregate_digest_is_verified_before_comparison():
    data={'path':'build/lib.so','sha256':'a'*64}
    digest=hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    assert module.payload_without_digest(dict(data,source_runtime_contract_sha256=digest))==data
    with pytest.raises(ValueError,match='aggregate digest'):
        module.payload_without_digest(dict(data,sha256='b'*64,source_runtime_contract_sha256=digest))


def test_external_absolute_location_is_not_reinterpreted():
    data={'path':'/different/workspace/install/lib.so'}
    assert module.relocate(data)==data
