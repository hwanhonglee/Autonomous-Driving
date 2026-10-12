"""HH_261007 - Test prospective descriptive masks using stdlib/scalar fixtures only.

No NumPy/SciPy import, genuine mask entry, model, training, simulator or ROS is used.
"""
import ast
import hashlib
import io
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import zipfile

import pytest
from scripts.e2e import diagnose_map_sign_components_20261007_v1 as check


def header(shape=(1, 360, 640), dtype='|u1', fortran=False, version=b'\x01\x00'):
    payload = repr(dict(descr=dtype, fortran_order=fortran, shape=shape)).encode() + b'\n'
    width = 2 if version == b'\x01\x00' else 4
    return b'\x93NUMPY' + version + len(payload).to_bytes(width, 'little') + payload


@pytest.mark.parametrize('version', [b'\x01\x00', b'\x02\x00', b'\x03\x00'])
def test_fixed_native_uint8_header(version):
    stream = io.BytesIO(header(version=version) + b'pixels')
    check.npy_header(stream, (1, 360, 640))
    assert stream.read() == b'pixels'


@pytest.mark.parametrize('payload', [b'', b'notnpy', b'\x93NUMPY\x04\x00',
    b'\x93NUMPY\x01\x00\x01', b'\x93NUMPY\x01\x00' + (16385).to_bytes(2, 'little'),
    header(dtype='<f4'), header(fortran=True), header(shape=(360, 640)), header(shape=(True, 360, 640)),
    b'\x93NUMPY\x01\x00\x05\x00{}'])
def test_header_fail_closed(payload):
    with pytest.raises((ValueError, SyntaxError, UnicodeDecodeError)):
        check.npy_header(io.BytesIO(payload), (1, 360, 640))


class FakeArray:
    def __init__(self, pixels): self.pixels = pixels
    def reshape(self, shape): self.shape = shape; return self
    def max(self): return max(self.pixels)


def fake_numpy():
    return SimpleNamespace(uint8='uint8', frombuffer=lambda pixels, dtype: FakeArray(pixels))


def zipped(path, members):
    with zipfile.ZipFile(path, 'w', compression=zipfile.ZIP_DEFLATED) as stream:
        for key, value in members:
            stream.writestr(key, value)


def test_target_only_entry_never_opens_rgb(tmp_path, monkeypatch):
    pixels = bytes([5]) * (360 * 640); path = tmp_path / 'cohort.npz'
    zipped(path, [('rgb.npy', b'not even valid NPY'), ('target.npy', header() + pixels)])
    old = zipfile.ZipFile.open; opened = []
    def observe(self, name, *args, **kwargs):
        opened.append(name); return old(self, name, *args, **kwargs)
    monkeypatch.setattr(zipfile.ZipFile, 'open', observe)
    value, sha = check.load_mask(path, 'target.npy', 1, fake_numpy())
    assert value.shape == (1, 360, 640) and sha == hashlib.sha256(pixels).hexdigest()
    assert opened == ['target.npy']


@pytest.mark.parametrize('variant', ['extra', 'duplicate', 'badclass', 'short', 'tail', 'wrongshape'])
def test_prediction_entry_denials(tmp_path, variant):
    pixels = bytes([7 if variant == 'badclass' else 0]) * (360 * 640)
    payload = header(shape=(2,360,640) if variant == 'wrongshape' else (1,360,640)) + pixels
    if variant == 'short': payload = payload[:-1]
    if variant == 'tail': payload += b'!'
    members = [('prediction.npy', payload)]
    if variant == 'extra': members.append(('rgb.npy', b'forbidden'))
    if variant == 'duplicate': members.append(('prediction.npy', payload))
    path = tmp_path / 'prediction.npz'; zipped(path, members)
    with pytest.raises(ValueError): check.load_mask(path, 'prediction.npy', 1, fake_numpy())


@pytest.mark.parametrize('area,index', [(1,0),(2,1),(4,1),(5,2),(16,2),(17,3),(64,3),(65,4),(256,4),(257,5),(230400,5)])
def test_prospective_fixed_bins(area, index):
    assert check.bin_index(area) == index


@pytest.mark.parametrize('area', [0, -1, 230401, True, 1.0])
def test_bin_abi_denied(area):
    with pytest.raises(ValueError): check.bin_index(area)


def test_component_full_partition_and_deterministic_digest():
    rows = [(1,0,1,True), (4,3,1,False), (17,17,0,True), (300,0,300,False)]
    bins, sha = check.summarize_components(rows)
    assert bins[0] == [1,1,0,1,1,1,1,0,1]
    assert bins[1] == [1,4,3,1,0,0,0,0,0]
    assert bins[3] == [1,17,17,0,0,1,17,17,0]
    assert bins[5] == [1,300,0,300,1,0,0,0,0]
    assert sum(r[1] for r in bins) == 322 and sum(r[2]+r[3] for r in bins) == 322
    assert sha == hashlib.sha256(b''.join(struct.pack('<qqqq',a,tp,e,int(b)) for a,tp,e,b in rows)).hexdigest()
    assert check.summarize_components([]) == ([[0]*9 for _ in check.BINS], hashlib.sha256(b'').hexdigest())


@pytest.mark.parametrize('row', [(1,1,1,False),(1,-1,2,False),(0,0,0,False),(1,0,1,1),(1.,0,1,False)])
def test_component_corrupt_partition_denied(row):
    with pytest.raises(ValueError): check.summarize_components([row])


def test_confusion_all_seven_pixels_and_absent_fp():
    matrix = [[0]*7 for _ in range(7)]
    matrix[0][0]=2; matrix[0][5]=3; matrix[5][0]=4; matrix[5][5]=1; matrix[0][6]=2
    value = check.metrics(matrix)
    assert value['target_pixels']['traffic_sign'] == 5
    assert value['predicted_pixels']['traffic_sign'] == 4
    assert value['recall']['traffic_sign'] == .2 and value['precision']['traffic_sign'] == .25
    assert value['iou']['traffic_sign'] == 1/8 and value['other_to_sign_false_positive_pixels'] == 3
    assert value['iou']['pole'] is None and value['false_positive_pixels']['pole'] == 2
    assert value['pixel_accuracy'] == 3/12


@pytest.mark.parametrize('matrix', [[], [[0]*7]*7, [[-1]*7]*7, [[True]*7]*7, [[0]*6]*7])
def test_confusion_denominator_or_abi_denial(matrix):
    with pytest.raises(ValueError): check.metrics(matrix)


def test_all1050_fixed_order_failed_tail_is_not_dropped():
    rows = check.ledger(); assert len(rows) == 1050
    assert rows[0] == dict(cohort='dev_town01',arm='parent',ordinal=0,status='UNEXECUTED')
    assert rows[117]['ordinal'] == 117 and rows[118]['arm'] == 'control'
    assert rows[354]['cohort'] == 'holdout_town02_p02' and rows[-1]['ordinal'] == 231
    rows[0]['status']='FAIL'
    assert sum(r['status']=='FAIL' for r in rows) == 1 and sum(r['status']=='UNEXECUTED' for r in rows) == 1049


def fake_evaluation():
    row = dict(precision={'traffic_sign':.5}, recall={'traffic_sign':.5},
        iou={k:.5 for k in check.LABELS}, other_to_sign_false_positive_pixels=100)
    return {cohort:{arm:json.loads(json.dumps(row)) for arm in check.ARMS} for cohort,_ in check.COHORTS}


def test_genuine_ast_27_screens_keep_original_failures():
    evaluation = fake_evaluation()
    evaluation['dev_town01']['candidate']['precision']['traffic_sign']=.6
    evaluation['dev_town01']['candidate']['recall']['traffic_sign']=.47
    evaluation['dev_town01']['candidate']['iou']['traffic_sign']=.6
    evaluation['dev_town01']['candidate']['other_to_sign_false_positive_pixels']=80
    screen = check.original_screens(evaluation)
    assert len(screen['checks']) == 27 and len(screen['original_twenty_checks']) == 20
    assert sum(screen['checks'].values()) == 23 and screen['passed_exploratory_only'] is False
    assert {k for k,v in screen['checks'].items() if not v} == {
        'Town01_sign_recall_vs_parent','Town01_sign_recall_vs_control',
        'ADDED_Town01_sign_recall_strict_gain_vs_parent','ADDED_Town01_sign_recall_strict_gain_vs_control'}
    assert screen['model_promoted'] is screen['vehicle_control_approved'] is screen['independent_validation'] is False


def test_json_and_output_bounds_fail_before_write(tmp_path, monkeypatch):
    monkeypatch.setattr(check,'OUTPUT',tmp_path); monkeypatch.setattr(check,'LIMIT',32)
    with pytest.raises(ValueError): check.write_new(tmp_path/'oversized.json',{'large':'x'*40})
    assert not (tmp_path/'oversized.json').exists()
    with pytest.raises(ValueError): check.encoded({'nan':float('nan')})
    duplicate=tmp_path/'duplicate.json'; duplicate.write_text('{"a":1,"a":2}')
    with pytest.raises(ValueError): check.read_json(duplicate)


def test_raw_bytes_archive_preserves_external_sha(tmp_path, monkeypatch):
    monkeypatch.setattr(check,'OUTPUT',tmp_path)
    payload=b'{ "ns": 1234567890123456789 }\n'
    check.write_bytes(tmp_path/'ROOT_authority.json',payload)
    assert (tmp_path/'ROOT_authority.json').read_bytes() == payload
    with pytest.raises(FileExistsError): check.write_bytes(tmp_path/'ROOT_authority.json',payload)


def test_default_actual_admission_denied_without_root_sha():
    with pytest.raises(ValueError, match='external ROOT'): check.admission(None)


def authority_fixture(monkeypatch):
    declared = {'runtime_packages': {'numpy':'1.21.5','scipy':'1.8.0'}}
    authority = dict(schema=check.SCHEMA+'.ROOT_authority', contract=declared,
        stored_mask_CPU_diagnostic_authorized=True, output=str(check.OUTPUT), CPU_affinity=[3])
    monkeypatch.setattr(check, 'digest', lambda path: 'a'*64)
    monkeypatch.setattr(check, 'read_json', lambda path: authority)
    monkeypatch.setattr(check, 'contract', lambda: declared)
    monkeypatch.setattr(check.os, 'sched_getaffinity', lambda pid: {3,4})
    monkeypatch.setattr(check.importlib.metadata, 'version', lambda package: declared['runtime_packages'][package])
    for name,value in {'CUDA_VISIBLE_DEVICES':'','PYTHONNOUSERSITE':'1','PYTHONDONTWRITEBYTECODE':'1',
        'OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1'}.items():
        monkeypatch.setenv(name,value)
    return authority


def test_exact_fixed_authority_fake_only(monkeypatch):
    authority = authority_fixture(monkeypatch)
    assert check.admission('a'*64) == (authority,[3])


@pytest.mark.parametrize('fault', ['rawsha','schema','scope','output','contract','multicpu','boolcpu','unavailablecpu','environment','version'])
def test_root_admission_faults_fail_before_numeric_import(monkeypatch, fault):
    authority = authority_fixture(monkeypatch); sha='a'*64
    if fault=='rawsha': sha='b'*64
    elif fault=='schema': authority['schema']='other'
    elif fault=='scope': authority['stored_mask_CPU_diagnostic_authorized']=False
    elif fault=='output': authority['output']='/elsewhere'
    elif fault=='contract': monkeypatch.setattr(check,'contract',lambda:{'changed':True})
    elif fault=='multicpu': authority['CPU_affinity']=[3,4]
    elif fault=='boolcpu': authority['CPU_affinity']=[True]
    elif fault=='unavailablecpu': authority['CPU_affinity']=[99]
    elif fault=='environment': monkeypatch.setenv('CUDA_VISIBLE_DEVICES','0')
    elif fault=='version': monkeypatch.setattr(check.importlib.metadata,'version',lambda package:'different')
    with pytest.raises(ValueError): check.admission(sha)
    assert 'numpy' not in sys.modules and 'scipy' not in sys.modules


@pytest.mark.parametrize('fault', ['deadline','RSS','disk'])
def test_numeric_resource_faults_preserved_as_errors(monkeypatch, fault):
    monkeypatch.setattr(check.time,'monotonic',lambda:check.DEADLINE+1 if fault=='deadline' else 1)
    monkeypatch.setattr(check.resource,'getrusage',lambda who:SimpleNamespace(ru_maxrss=check.RSS//1024+1 if fault=='RSS' else 1))
    monkeypatch.setattr(check.os,'statvfs',lambda path:SimpleNamespace(f_bavail=0 if fault=='disk' else check.FLOOR+check.RESERVE,f_frsize=1))
    with pytest.raises(ValueError): check.guard(0)


def test_source_changed_and_checkpoint_deny(monkeypatch):
    monkeypatch.setattr(check,'digest',lambda path:'a'*64)
    with pytest.raises(ValueError,match='seal changed'): check.verify_pins({'/fake':'b'*64})
    called=[]
    check.verify_pins({'/fake':'a'*64},lambda:called.append('before_hash'))
    assert called==['before_hash']


def test_no_output_directory_or_symlink_omission(tmp_path, monkeypatch):
    monkeypatch.setattr(check,'OUTPUT',tmp_path)
    (tmp_path/'unexpected').mkdir()
    with pytest.raises(ValueError): check.write_bytes(tmp_path/'result.json',b'{}')
    assert not (tmp_path/'result.json').exists()


def test_source_only_import_and_numeric_seams():
    tree=ast.parse(Path(check.__file__).read_text())
    top=[n for n in tree.body if isinstance(n,(ast.Import,ast.ImportFrom))]
    roots={n.module.split('.')[0] if isinstance(n,ast.ImportFrom) else alias.name.split('.')[0] for n in top for alias in (n.names if isinstance(n,ast.Import) else [None])}
    assert not roots & {'numpy','scipy','torch','carla','rclpy','PIL'}
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='components')
    assert 'ndimage.label(mask, structure=np.ones((3, 3), dtype=np.uint8))' in ast.unparse(function)
    run=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
    text=ast.unparse(run)
    assert text.index('admission(authority_sha)') < text.index('import numpy')
    assert text.index('flock') < text.index('OUTPUT.mkdir') < text.index('import numpy')
    assert "'target.npy'" in text and "'prediction.npy'" in text and 'load_model' not in text
    assert 'SIGALRM' in text and 'original_screens(evaluation)' in text
    assert 'numpy' not in sys.modules and 'scipy' not in sys.modules and 'torch' not in sys.modules
