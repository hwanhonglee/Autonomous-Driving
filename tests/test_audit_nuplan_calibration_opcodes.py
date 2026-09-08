"""HH_260906 - Test untrusted opcode inventory and a synthetic one-member follow-up without opening nuPlan data."""
import argparse
import ast
import base64
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import pickle
import signal
import sqlite3
import struct
from types import SimpleNamespace
import zipfile

import pytest

from scripts.e2e import audit_nuplan_calibration_opcodes as audit


@pytest.mark.parametrize('protocol',range(6))
def test_primitive_pickles_remain_inventory_only_without_numeric_extraction(protocol,monkeypatch):
    raw=pickle.dumps([[1.,2.,3.],[0.,1.,0.]],protocol=protocol)
    monkeypatch.setattr(pickle,'loads',lambda *_:pytest.fail('no unpickling'))
    monkeypatch.setattr(pickle,'load',lambda *_:pytest.fail('no unpickling'))
    result=audit.inspect_pickle_opcodes(raw)
    assert result['parse_status']=='PARSED_ENCODING_ONLY'
    assert result['numeric_calibration_extracted'] is result['constructors_executed'] is False
    assert result['semantic_status']=='UNRESOLVED_INVENTORY_ONLY'
    assert all(0<=r['offset']<r['end_offset']<=len(raw) for r in result['opcodes'])
    assert all(r['encoded_sha256']==audit.sha(raw[r['offset']:r['end_offset']]) for r in result['opcodes'])


@pytest.mark.parametrize('raw',[
    b'cunknown_package\nconstructor\n)R.',
    b'cos\nsystem\n(S\'touch /tmp/never-executed\'\ntR.',
    b'\x80\x04\x8c\x02os\x8c\x06system\x93)R.',
    b'\x80\x02\x82\x01.',
    b'Punknown_external_reference\n.',
    b'\x80\x05\x97\x98.',
])
def test_unknown_constructor_reduce_persistent_or_buffer_semantics_are_never_executed(raw,monkeypatch):
    monkeypatch.setattr(pickle,'loads',lambda *_:pytest.fail('no unpickling'))
    result=audit.inspect_pickle_opcodes(raw)
    assert result['parse_status']=='PARSED_ENCODING_ONLY'
    assert result['dynamic_opcode_count']>0 and result['constructors_executed'] is False
    assert all(r['execution']=='NEVER_EXECUTED_UNRESOLVED' for r in result['opcodes'] if r['name'] in audit.DYNAMIC)
    assert 'touch /tmp/never-executed' not in json.dumps(result)


def test_global_descriptors_are_lexical_not_resolved_callables():
    result=audit.inspect_pickle_opcodes(b'cnumpy.core.multiarray\n_reconstruct\n)R.')
    declared=result['opcodes'][0]['global_declaration']
    assert declared['module']=='numpy.core.multiarray' and declared['symbol']=='_reconstruct'
    reduce=next(r for r in result['opcodes'] if r['name']=='REDUCE')
    assert reduce['nearby_global_declarations_not_resolved_callables']==[declared]
    assert result['stack_semantics_validated'] is False


def test_cycles_and_shared_references_are_unresolved_not_recursively_walked():
    cycle=[];cycle.append(cycle)
    for value in (cycle,[cycle,cycle]):
        result=audit.inspect_pickle_opcodes(pickle.dumps(value,protocol=4))
        assert result['parse_status']=='PARSED_ENCODING_ONLY' and result['backreference_count']>0
        assert result['potential_alias_or_cycle_unresolved'] is True and result['memo_cycles_proved_absent'] is False


def test_dup_can_alias_a_container_without_memo_get_and_remains_unresolved():
    result=audit.inspect_pickle_opcodes(b']2a.')
    assert result['parse_status']=='PARSED_ENCODING_ONLY' and result['backreference_count']==0
    assert result['stack_duplication_count']==1 and result['potential_alias_or_cycle_unresolved'] is True


def test_parser_consumption_is_not_misreported_as_completed_inventory():
    raw=b'\x80\x04B'+struct.pack('<I',4)+b'ab'
    result=audit.inspect_pickle_opcodes(raw)
    assert result['parse_status']=='REJECTED' and result['inventoried_bytes']==2
    assert result['parser_consumed_bytes']==len(raw) and result['unread_bytes']==0
    assert result['uninventoried_bytes']==len(raw)-2


@pytest.mark.parametrize('raw',[
    b'N.extra',b'N.N.',b'\xff.',b'\x80\x06N.',b'N\x80\x02.',
    b'\x80\x04\x95'+struct.pack('<Q',999999999999)+b'N.',
    b'\x80\x04\x95'+struct.pack('<Q',1)+b'G'+struct.pack('>d',1.)+b'.',
    b'\x80\x04\x95'+struct.pack('<Q',10)+b'\x95'+struct.pack('<Q',0)+b'.',
    b'\x80\x04\x8d'+struct.pack('<Q',2**63)+b'x.',
    b'\x80\x04B'+struct.pack('<I',2**32-1)+b'x.',
    b'\x80\x02\x8b'+struct.pack('<I',2**32-1)+b'.',
    b'I'+b'9'*1000+b'\n.',
    b'\x80\x02\x8b'+struct.pack('<I',100)+b'\x7f'*100+b'.',
    b'\x80\x02r'+struct.pack('<I',1000000)+b'.',
    b'\x80\x02j'+struct.pack('<I',1000000)+b'.',
    b'N'*4097+b'.',b'\x80\x04N',
])
def test_malformed_or_oversized_encodings_retain_bounded_rejected_inventory(raw):
    result=audit.inspect_pickle_opcodes(raw)
    assert result['parse_status']=='REJECTED' and result['semantic_status']=='UNRESOLVED_INVENTORY_ONLY'
    assert len(result['opcodes'])<=audit.MAX_OPCODES and len(result['error'])<=240
    json.dumps(result,allow_nan=False)


@pytest.mark.parametrize('raw',[b'',b'x'*65537,bytearray(b'N.'),'N.'])
def test_blob_size_and_type_limit_applies_before_parsing(raw):
    with pytest.raises(ValueError):audit.inspect_pickle_opcodes(raw)


@pytest.mark.parametrize('value',[float('inf'),float('-inf'),float('nan')])
def test_nonfinite_float_is_described_without_nonfinite_json_or_numeric_calibration(value):
    r=audit.inspect_pickle_opcodes(b'G'+struct.pack('>d',value)+b'.')
    assert r['opcodes'][0]['argument']=={'kind':'float','finite':False,'bounded_value':None}
    json.dumps(r,allow_nan=False)


def test_valid_multi_frame_boundaries_and_exact_final_stop():
    raw=b'\x80\x04\x95'+struct.pack('<Q',1)+b'N'+b'\x95'+struct.pack('<Q',1)+b'.'
    r=audit.inspect_pickle_opcodes(raw)
    assert r['parse_status']=='PARSED_ENCODING_ONLY' and len(r['frames'])==2


def prior(blobs,archive_identity):
    return dict(status='METADATA_INSPECTED_NOT_READY',member_sha256=audit.MEMBER_SHA,archive_identity_unchanged=True,
        database_payloads_opened=1,archive_post_identity=archive_identity,archive_previous_content_sha256='c'*64,
        selection_plan_sha256='d'*64,metadata={'calibration_opaque_fields':[
            {'channel':c,'fields':{f:{'storage_type':'blob','size_bytes':len(blobs[c+'/'+f]),
                'sha256':audit.sha(blobs[c+'/'+f]),'deserialized':False} for f in audit.FIELDS}} for c in audit.CHANNELS]})


class FrozenTime(datetime):
    @classmethod
    def now(cls,tz=None):return cls(2026,9,8,20,0,tzinfo=timezone.utc)


@pytest.fixture
def scenario(tmp_path,monkeypatch):
    monkeypatch.setattr(audit,'datetime',FrozenTime)
    personal=tmp_path/'personal';archive_root=personal/'dataset/raw/nuplan/v1.1-mini/research-only-pending-terms-review'
    archive_root.mkdir(parents=True);archive_path=archive_root/'nuplan-v1.1_mini.zip'
    monkeypatch.setattr(audit.sys,'prefix',str(personal/'portable_e2e/venvs/py312'))
    monkeypatch.setenv('CUDA_VISIBLE_DEVICES','');monkeypatch.setenv('PYTHONNOUSERSITE','1')
    dbraw=b'Synthetic memory database bytes, never a real dataset.'
    with zipfile.ZipFile(archive_path,'w',compression=zipfile.ZIP_DEFLATED) as z:
        z.writestr(audit.MEMBER,dbraw);z.writestr('must-not-open/camera.jpg',b'no')
    with zipfile.ZipFile(archive_path) as z:info=z.getinfo(audit.MEMBER)
    monkeypatch.setattr(audit,'MEMBER_BYTES',len(dbraw));monkeypatch.setattr(audit,'COMPRESSED_BYTES',info.compress_size)
    monkeypatch.setattr(audit,'MEMBER_SHA',audit.sha(dbraw));monkeypatch.setattr(audit,'CRC32',f'{info.CRC:08x}')
    blobs={c+'/'+f:pickle.dumps([1.,2.,3.],protocol=4) for c in audit.CHANNELS for f in audit.FIELDS}
    p=prior(blobs,audit.regular(archive_path));prior_path=personal/'portable_e2e/runs/diagnostics/prior/report.json'
    prior_path.parent.mkdir(parents=True);raw=json.dumps(p).encode();prior_path.write_bytes(raw);monkeypatch.setattr(audit,'PRIOR_SHA',audit.sha(raw))
    worker=tmp_path/'worker.py';worker.write_bytes(b'# synthetic source fixture\n');monkeypatch.setattr(audit,'__file__',str(worker))
    plan=audit.build_plan(p,declared_at_utc='2026-09-08T19:59:00Z',source_sha256=audit.sha(worker.read_bytes()))
    args=argparse.Namespace(personal_root=str(personal),archive_root=str(archive_root),prior_report=str(prior_path),
        plan_base64=base64.b64encode(json.dumps(plan).encode()).decode(),output_dir=str(personal/'portable_e2e/runs/diagnostics/followup'))
    calls=[]
    def memory(value,pins):
        calls.append('camera-only in-memory metadata');assert value==dbraw and pins==audit.calibration_pins(p)
        return blobs.copy()
    monkeypatch.setattr(audit,'camera_blobs_from_memory',memory)
    return SimpleNamespace(args=args,plan=plan,prior=p,blobs=blobs,archive=archive_path,prior_path=prior_path,worker=worker,calls=calls)


def test_new_followup_reads_only_one_synthetic_member_retains32_and_never_changes_original(scenario,monkeypatch):
    old=zipfile.ZipFile.open;opened=[]
    def opening(self,name,*a,**kw):
        opened.append(name.filename if isinstance(name,zipfile.ZipInfo) else name);return old(self,name,*a,**kw)
    monkeypatch.setattr(zipfile.ZipFile,'open',opening)
    before=hashlib.sha256(scenario.archive.read_bytes()).hexdigest()
    assert audit.run(scenario.args)==0
    root=Path(scenario.args.output_dir);r=json.loads((root/'report.json').read_text())
    assert opened==[audit.MEMBER] and scenario.calls==['camera-only in-memory metadata']
    assert r['status']=='OPCODE_INVENTORY_COMPLETE_NOT_DECODED' and len(r['blob_reports'])==32
    assert r['numeric_calibration_extracted'] is False and r['opaque_calibration_blob_copies_written']==32
    assert r['camera_payloads_read']==r['map_payloads_read']==r['lidar_payloads_read']==0
    assert hashlib.sha256(scenario.archive.read_bytes()).hexdigest()==before
    lines=(root/'SHA256SUMS').read_text().splitlines();assert len(lines)==35
    for line in lines:
        value,name=line.split('  ');assert name!='SHA256SUMS' and audit.sha((root/name).read_bytes())==value
    with pytest.raises(ValueError,match='fresh'):audit.run(scenario.args)


@pytest.mark.parametrize('key,value',[('database_payloads_allowed',2),('camera_payloads_allowed',1),
    ('numeric_calibration_extraction',True),('constructors_allowed',True),('maximum_blob_bytes',65537),
    ('retain_opaque_calibration_blobs_private',False),('authorization_scope','old authorization')])
def test_prospective_scope_changes_fail_before_member_read(scenario,key,value):
    plan=copy.deepcopy(scenario.plan);plan[key]=value
    scenario.args.plan_base64=base64.b64encode(json.dumps(plan).encode()).decode()
    with pytest.raises(ValueError,match='scope'):audit.run(scenario.args)
    assert not scenario.calls and not Path(scenario.args.output_dir).exists()


@pytest.mark.parametrize('changed',['prior','archive','worker'])
def test_changed_pinned_input_fails_before_selected_payload(scenario,changed):
    path={'prior':scenario.prior_path,'archive':scenario.archive,'worker':scenario.worker}[changed]
    path.write_bytes(path.read_bytes()+b' ')
    with pytest.raises(ValueError):audit.run(scenario.args)
    assert not scenario.calls


@pytest.mark.parametrize('fault',['source','prior','retained_blob','retained_symlink','interrupt'])
def test_failure_and_post_mutation_preserve_partial_evidence(scenario,monkeypatch,fault):
    original=audit.inspect_pickle_opcodes;calls=[]
    def inspect(raw):
        calls.append(1)
        if len(calls)==2:
            if fault=='source':scenario.worker.write_bytes(b'changed source')
            elif fault=='prior':scenario.prior_path.unlink()
            elif fault=='retained_blob':next(Path(scenario.args.output_dir).rglob('*.bin')).write_bytes(b'changed blob')
            elif fault=='retained_symlink':
                path=next(Path(scenario.args.output_dir).rglob('*.bin'));path.unlink();path.symlink_to(scenario.worker)
            else:raise KeyboardInterrupt('retained synthetic stop')
        return original(raw)
    monkeypatch.setattr(audit,'inspect_pickle_opcodes',inspect)
    assert audit.run(scenario.args)==2
    r=json.loads((Path(scenario.args.output_dir)/'report.json').read_text())
    assert r['status']=='FAILED_METADATA_FOLLOWUP'
    assert r['opaque_calibration_blob_copies_written']==(1 if fault=='interrupt' else 32)
    assert Path(scenario.args.output_dir,'SHA256SUMS').is_file()


@pytest.mark.parametrize('scope',['inside_prior','parent_of_prior','traversal'])
def test_new_output_cannot_mutate_original_evidence_namespace(scenario,scope):
    if scope=='inside_prior':scenario.args.output_dir=str(scenario.prior_path.parent/'new')
    elif scope=='parent_of_prior':scenario.args.output_dir=str(scenario.prior_path.parent.parent)
    else:scenario.args.output_dir=str(scenario.prior_path.parent.parent/'new/../../escape')
    with pytest.raises(ValueError):audit.run(scenario.args)
    assert not scenario.calls


@pytest.mark.parametrize('variable,value',[('CUDA_VISIBLE_DEVICES','0'),('PYTHONNOUSERSITE','0')])
def test_explicit_cpu_and_no_user_environment_required_before_member_access(scenario,monkeypatch,variable,value):
    monkeypatch.setenv(variable,value)
    with pytest.raises(ValueError,match='venv'):audit.run(scenario.args)
    assert not scenario.calls


def test_only_existing_personal_venv_allowed(scenario,monkeypatch):
    monkeypatch.setattr(audit.sys,'prefix','/usr')
    with pytest.raises(ValueError,match='venv'):audit.run(scenario.args)
    assert not scenario.calls


def test_no_pickle_vm_numpy_orm_import_or_eval_exec_calls_in_worker():
    tree=ast.parse(Path(audit.__file__).read_text())
    for node in ast.walk(tree):
        if isinstance(node,ast.Import):assert all(n.name.split('.')[0] not in ('pickle','numpy','sqlalchemy') for n in node.names)
        if isinstance(node,ast.ImportFrom):assert (node.module or '').split('.')[0] not in ('pickle','numpy','sqlalchemy')
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Name):assert node.func.id not in ('eval','exec','compile','__import__')
    assert 'pickletools' in audit.__dict__


@pytest.mark.parametrize('query',["ATTACH DATABASE ':memory:' AS x",'CREATE TABLE x(y)',
    'SELECT * FROM image','SELECT randomblob(100)','PRAGMA query_only=OFF','SELECT load_extension(\'x\')'])
def test_camera_authorizer_rejects_other_payload_queries_and_execution(query):
    connection=sqlite3.connect(':memory:');connection.execute('CREATE TABLE camera(channel,translation,rotation,intrinsic,distortion)')
    connection.execute('CREATE TABLE image(x)');connection.set_authorizer(audit.camera_authorizer)
    with pytest.raises(sqlite3.DatabaseError):connection.execute(query).fetchall()
    assert connection.execute('SELECT channel,translation,rotation,intrinsic,distortion FROM camera').fetchall()==[]
    connection.close()


@pytest.mark.parametrize('fault',[None,'blob_sha','null_blob','missing_camera','virtual_table','wrong_member_sha'])
def test_camera_only_memory_reader_checks32_fields_without_pickle_execution(monkeypatch,fault):
    # HH_260906 - The connection is a synthetic memory fixture; deserialize is a spy, not an actual DB payload read.
    raw=bytearray(512);raw[:16]=b'SQLite format 3\0';raw[18:20]=b'\x01\x01';raw=bytes(raw)
    monkeypatch.setattr(audit,'MEMBER_BYTES',len(raw));monkeypatch.setattr(audit,'MEMBER_SHA',audit.sha(raw))
    connection=sqlite3.connect(':memory:')
    if fault=='virtual_table':connection.execute('CREATE VIRTUAL TABLE camera USING fts5(channel,translation,rotation,intrinsic,distortion)')
    else:connection.execute('CREATE TABLE camera(channel,translation,rotation,intrinsic,distortion)')
    blobs={c+'/'+f:b'F1.0\n.' for c in audit.CHANNELS for f in audit.FIELDS}
    for channel in audit.CHANNELS:
        connection.execute('INSERT INTO camera VALUES(?,?,?,?,?)',(channel,*(blobs[channel+'/'+f] for f in audit.FIELDS)))
    if fault=='null_blob':connection.execute("UPDATE camera SET rotation=NULL WHERE channel='CAM_B0'")
    if fault=='missing_camera':connection.execute("DELETE FROM camera WHERE channel='CAM_B0'")
    connection.commit()
    pins={key:dict(size_bytes=len(value),sha256=audit.sha(value)) for key,value in blobs.items()}
    if fault=='blob_sha':pins['CAM_B0/rotation']['sha256']='f'*64
    seen=[]
    class ConnectionProxy:
        def deserialize(self,payload):seen.append(payload)
        def __getattr__(self,name):return getattr(connection,name)
    monkeypatch.setattr(audit.sqlite3,'Connection',ConnectionProxy)
    monkeypatch.setattr(audit.sqlite3,'connect',lambda name:ConnectionProxy() if name==':memory:' else pytest.fail('disk connection forbidden'))
    monkeypatch.setattr(pickle,'loads',lambda *_:pytest.fail('no unpickling'))
    if fault=='wrong_member_sha':
        with pytest.raises(ValueError,match='SHA'):audit.camera_blobs_from_memory(raw+b'x',pins)
        assert seen==[];connection.close()
    elif fault is None:
        assert audit.camera_blobs_from_memory(raw,pins)==blobs and seen==[raw]
    else:
        with pytest.raises((ValueError,sqlite3.DatabaseError)):audit.camera_blobs_from_memory(raw,pins)


def test_original32_blob_pins_reject_duplicate_camera_and_missing_field(scenario):
    for fault in ('duplicate_camera','missing_field','size'):
        data=copy.deepcopy(scenario.prior)
        if fault=='duplicate_camera':data['metadata']['calibration_opaque_fields'][1]=data['metadata']['calibration_opaque_fields'][0]
        elif fault=='missing_field':data['metadata']['calibration_opaque_fields'][0]['fields'].pop('rotation')
        else:data['metadata']['calibration_opaque_fields'][0]['fields']['rotation']['size_bytes']=65537
        with pytest.raises(ValueError):audit.calibration_pins(data)


def test_finalization_ignores_repeated_signals_then_restores_handlers():
    watched=(signal.SIGALRM,signal.SIGTERM,signal.SIGINT);before={s:signal.getsignal(s) for s in watched}
    with audit.guarded_finalization():
        for sig in watched:signal.raise_signal(sig)
    assert all(signal.getsignal(s)==value for s,value in before.items())


def test_cli_abbreviations_fail_before_resource_changes():
    with pytest.raises(SystemExit):audit.main(['--personal','none'])
