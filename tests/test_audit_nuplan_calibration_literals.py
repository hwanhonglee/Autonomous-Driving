"""HH_260906 - Exercise exact literal grammars with synthetic values; never decode real nuPlan calibration during tests."""

import argparse
import ast
import copy
from datetime import datetime,timezone
import math
from pathlib import Path
import pickle
import signal
import struct

import pytest

from scripts.e2e import audit_nuplan_calibration_literals as literal
from test_curate_nuplan_calibration_opcodes import evidence,reseal  # noqa: F401


VALUES={'translation':[1.,-2.,3.], 'rotation':[1.,0.,0.,0.],
    'intrinsic':[1000.,0.,960.,0.,1001.,540.,0.,0.,1.], 'distortion':[0.,.1,-.2,0.,0.]}
OFFSETS={'translation':[158,177,196], 'rotation':[155,174,193,212],
    'intrinsic':[75,84,93,106,115,124,137,146,155], 'distortion':[15,24,33,42,51]}
SIZES={'translation':211,'rotation':227,'intrinsic':166,'distortion':61}


def synthetic(field,values=None):
    raw=bytearray(literal.layout(field)[0])
    for offset,value in zip(OFFSETS[field],VALUES[field] if values is None else values):
        raw[offset:offset+8]=struct.pack('<d' if field in ('translation','rotation') else '>d',value)
    return bytes(raw)


@pytest.mark.parametrize('field',literal.audit.FIELDS)
def test_fixed_offsets_sizes_ieee_encodings_and_no_constructor_execution(field,monkeypatch):
    raw=synthetic(field)
    monkeypatch.setattr(pickle,'loads',lambda *_:pytest.fail('no VM'))
    monkeypatch.setattr(pickle,'load',lambda *_:pytest.fail('no VM'))
    result=literal.decode_literals(raw,field)
    assert result['values']==VALUES[field] and result['finite'] and result['template_matched']
    assert [s['offset'] for s in result['numeric_slots']]==OFFSETS[field]
    assert len(raw)==SIZES[field] and struct.unpack('<Q',raw[3:11])[0]==len(raw)-11
    parsed=literal.audit.inspect_pickle_opcodes(raw)
    assert parsed['parse_status']=='PARSED_ENCODING_ONLY'
    assert parsed['declared_protocol']==5 and parsed['opcodes'][-1]['name']=='STOP'


@pytest.mark.parametrize('field',literal.audit.FIELDS)
def test_every_nonnumeric_byte_is_mandatory_and_every_numeric_bit_pattern_retained(field):
    raw=synthetic(field);slots={i for start in OFFSETS[field] for i in range(start,start+8)}
    for index in range(len(raw)):
        if index in slots:continue
        changed=bytearray(raw);changed[index]^=1
        with pytest.raises(ValueError):literal.decode_literals(bytes(changed),field)
    for value in (0.,-0.,math.ldexp(1.,-1074),1e300,-1e300):
        values=[value]*len(OFFSETS[field]);result=literal.decode_literals(synthetic(field,values),field)
        assert result['values']==values and result['finite']
        assert math.copysign(1.,result['values'][0])==math.copysign(1.,value)


@pytest.mark.parametrize('field',literal.audit.FIELDS)
@pytest.mark.parametrize('value',[float('nan'),float('inf'),float('-inf')])
def test_nonfinite_values_are_reported_as_failures_not_repaired_or_json_nan(field,value):
    values=list(VALUES[field]);values[0]=value
    result=literal.decode_literals(synthetic(field,values),field)
    assert result['finite'] is False and result['values'][0] is None
    assert result['numeric_slots'][0]['nonfinite_kind'] in ('NaN','positive_infinity','negative_infinity')
    literal.inventory.encoded(result)


@pytest.mark.parametrize('payload',[
    b']2a.',b'cos\nsystem\n)R.',b'\x80\x05\x97.',b'N.',b'',b'X'*65537,
])
def test_unknown_vm_programs_cycles_globals_and_unbounded_payloads_rejected(payload):
    with pytest.raises(ValueError):literal.decode_literals(payload,'distortion')


@pytest.mark.parametrize('field',['translation','rotation'])
def test_only_fixed_symbol_dtype_references_and_little_endian_are_accepted(field):
    raw=synthetic(field)
    assert raw.count(b'h\x06h\x0c')==len(VALUES[field])-1
    for old,new in ((b'h\x06h\x0c',b'h\x03h\x0c'),(b'\x8c\x01<',b'\x8c\x01>'),(b'\x8c\x02f8',b'\x8c\x02f4')):
        with pytest.raises(ValueError):literal.decode_literals(raw.replace(old,new,1),field)


def decoded(values=VALUES):
    return {field:literal.decode_literals(synthetic(field,values[field]),field) for field in literal.audit.FIELDS}


def test_sanity_only_keeps_values_unchanged_and_does_not_create_transform():
    fields=decoded();before=copy.deepcopy(fields)
    result=literal.camera_checks(fields)
    assert result['sanity_checks_all_pass'] and result['quaternion_wxyz_norm']==1.
    assert fields==before and 'transform' not in result


@pytest.mark.parametrize('field,index,value,failed_key',[
    ('rotation',0,1.000002,'quaternion_norm_within_tolerance'),
    ('intrinsic',8,.999999999999,'intrinsic_last_row_001'),
    ('intrinsic',3,.000001,'intrinsic_lower_left_zero'),
    ('intrinsic',0,0.,'focal_lengths_positive'),('intrinsic',4,-1.,'focal_lengths_positive'),
    ('intrinsic',2,1920.,'principal_point_within_metadata_dimensions'),
    ('intrinsic',5,-1.,'principal_point_within_metadata_dimensions'),
])
def test_sanity_violations_not_fixed_or_promoted(field,index,value,failed_key):
    values=copy.deepcopy(VALUES);values[field][index]=value;fields=decoded(values)
    result=literal.camera_checks(fields)
    assert result[failed_key] is False and not result['sanity_checks_all_pass'] and fields[field]['values'][index]==value


def test_quaternion_tolerance_finite_extreme_and_missing_fields():
    values=copy.deepcopy(VALUES);values['rotation']=[1.0000005,0,0,0]
    assert literal.camera_checks(decoded(values))['quaternion_norm_within_tolerance']
    values['rotation']=[1.7e308]*4
    result=literal.camera_checks(decoded(values))
    assert result['quaternion_wxyz_norm'] is None and result['quaternion_norm_within_tolerance'] is False
    assert literal.camera_checks({})['sanity_checks_all_pass'] is False
    assert literal.camera_checks(decoded(),640,360)['metadata_dimensions_match'] is False


class FrozenTime(datetime):
    @classmethod
    def now(cls,tz=None):return cls(2026,9,8,20,0,tzinfo=timezone.utc)


@pytest.fixture
def local_case(evidence,tmp_path,monkeypatch):
    audit=literal.audit;root=evidence.root
    monkeypatch.setenv('CUDA_VISIBLE_DEVICES','');monkeypatch.setenv('PYTHONNOUSERSITE','1')
    monkeypatch.setattr(literal,'datetime',FrozenTime)
    prior=audit.json_load(evidence.prior_path.read_bytes())
    rows=[]
    for c in prior['metadata']['calibration_opaque_fields']:
        c.update(width=1920,height=1080)
        for field in audit.FIELDS:
            raw=synthetic(field);name='opaque_blobs/'+c['channel']+'/'+field+'.bin';(root/name).write_bytes(raw)
            c['fields'][field].update(size_bytes=len(raw),sha256=audit.sha(raw))
            rows.append(dict(channel=c['channel'],field=field,retained_private_blob=name,**audit.inspect_pickle_opcodes(raw)))
    evidence.prior_path.write_bytes(literal.inventory.encoded(prior));monkeypatch.setattr(audit,'PRIOR_SHA',audit.sha(evidence.prior_path.read_bytes()))
    opcode_plan=audit.build_plan(prior,declared_at_utc='2026-09-08T19:18:00Z',source_sha256=literal.inventory.SOURCE_SHA)
    (root/'followup_plan.json').write_bytes(literal.inventory.encoded(opcode_plan));monkeypatch.setattr(literal.inventory,'PLAN_SHA',audit.sha((root/'followup_plan.json').read_bytes()))
    evidence.report.update(blob_reports=rows,previous_report_sha256=audit.PRIOR_SHA,followup_plan_sha256=literal.inventory.PLAN_SHA)
    reseal(evidence);monkeypatch.setattr(literal,'OPCODE_REPORT_SHA',evidence.report_sha);monkeypatch.setattr(literal,'OPCODE_MANIFEST_SHA',evidence.checksums_sha)
    references=tmp_path/'reference';reference_pins={}
    for name in literal.OFFICIAL:
        p=references/name;p.parent.mkdir(parents=True,exist_ok=True);raw=b'# synthetic reference, never imported\n';p.write_bytes(raw)
        reference_pins[name]=dict(sha256=audit.sha(raw),size_bytes=len(raw))
    monkeypatch.setattr(literal,'OFFICIAL',reference_pins)
    plan=literal.build_plan(prior,opcode_plan,declared_at_utc='2026-09-08T19:59:00Z',source_sha256=audit.sha(Path(literal.__file__).read_bytes()))
    plan_path=tmp_path/'literal_plan.json';plan_path.write_bytes(literal.inventory.encoded(plan))
    args=argparse.Namespace(input_root=root,prior_report=evidence.prior_path,reference_root=references,plan=plan_path,output_dir=tmp_path/'literal_output')
    return args,plan,evidence


def test_full_synthetic32_report_has_no_approval_or_vm_and_preserves_original_inputs(local_case):
    args,plan,e=local_case;before={p:p.read_bytes() for p in e.root.rglob('*') if p.is_file()}
    assert literal.run(args)==0
    r=literal.audit.json_load((args.output_dir/'report.json').read_bytes())
    assert r['status']=='LITERAL_DIAGNOSTIC_COMPLETE_NOT_TRUSTED' and r['reported_field_count']==32
    assert r['sanity_pass_camera_count']==8 and r['rejected_template_count']==r['nonfinite_field_count']==0
    assert all(r[k] is False for k in literal.DENIALS)
    assert r['database_reads']==r['zip_reads']==r['image_reads']==r['model_inference']==0
    assert all(p.read_bytes()==raw for p,raw in before.items()) and not list(args.output_dir.rglob('*.bin'))
    manifest=literal.inventory.manifest_entries((args.output_dir/'SHA256SUMS').read_bytes())
    assert len(manifest)==5 and all(literal.audit.sha((args.output_dir/n).read_bytes())==s for n,s in manifest.items())
    with pytest.raises(ValueError):literal.run(args)


@pytest.mark.parametrize('field,value',[('database_reads',1),('quaternion_norm_absolute_tolerance',.1),('calibration_trusted',True),('source_sha256','0'*64)])
def test_plan_changes_fail_before_any_output_or_numeric_read(local_case,field,value,monkeypatch):
    args,plan,_=local_case;plan[field]=value;args.plan.write_bytes(literal.inventory.encoded(plan))
    monkeypatch.setattr(literal,'decode_literals',lambda *_:pytest.fail('preflight must reject'))
    with pytest.raises(ValueError):literal.run(args)
    assert not args.output_dir.exists()


def test_interpretation_cannot_be_backdated_before_original_opcode_completion(local_case,monkeypatch):
    args,plan,_=local_case;plan['declared_at_utc']='2026-09-08T19:18:30Z';args.plan.write_bytes(literal.inventory.encoded(plan))
    monkeypatch.setattr(literal,'decode_literals',lambda *_:pytest.fail('chronology must reject'))
    with pytest.raises(ValueError,match='after the original'):literal.run(args)
    assert not args.output_dir.exists()


@pytest.mark.parametrize('target',['reference','blob','prior'])
def test_tampered_inputs_fail_before_output(local_case,target):
    args,_,_=local_case
    path=next(args.reference_root.rglob('*.py')) if target=='reference' else args.input_root/'opaque_blobs/CAM_B0/rotation.bin' if target=='blob' else args.prior_report
    path.write_bytes(path.read_bytes()+b'!')
    with pytest.raises(ValueError):literal.run(args)
    assert not args.output_dir.exists()


def test_all32_rows_retained_when_one_template_fails_or_quaternion_sanity_fails(local_case,monkeypatch):
    args,_,_=local_case;original=literal.decode_literals;calls=[]
    def synthetic_failure(raw,field):
        calls.append(field)
        if len(calls)==1:raise ValueError('synthetic unknown template')
        result=original(raw,field)
        if field=='rotation':result['values']=[2.,0.,0.,0.]
        return result
    monkeypatch.setattr(literal,'decode_literals',synthetic_failure)
    assert literal.run(args)==0
    r=literal.audit.json_load((args.output_dir/'report.json').read_bytes())
    assert len(calls)==r['reported_field_count']==32 and r['rejected_template_count']==1 and r['sanity_pass_camera_count']==0
    assert r['calibration_trusted'] is False


def test_interrupt_and_postwrite_input_mutation_preserve_failed_report(local_case,monkeypatch):
    args,_,_=local_case;original=literal.decode_literals;calls=[]
    def stop(raw,field):
        calls.append(field)
        if len(calls)==4:
            p=args.input_root/'opaque_blobs/CAM_B0/translation.bin';p.write_bytes(b'changed')
            raise KeyboardInterrupt('synthetic stop')
        return original(raw,field)
    monkeypatch.setattr(literal,'decode_literals',stop)
    assert literal.run(args)==2
    r=literal.audit.json_load((args.output_dir/'report.json').read_bytes())
    assert r['status']=='FAILED_LITERAL_DIAGNOSTIC' and r['reported_field_count']==3 and len(r['missing_fields'])==29
    assert r['postcheck_errors'] and not r['inputs_unchanged_before_after']


@pytest.mark.parametrize('kind',['reference','source_symlink','unexpected_file'])
def test_modified_output_source_or_reference_cannot_receive_success_report(local_case,monkeypatch,kind,tmp_path):
    args,_,_=local_case;original=literal.decode_literals;done=[]
    def change(raw,field):
        if not done:
            done.append(True)
            if kind=='reference':next((args.output_dir/'reference_source').rglob('*.py')).write_bytes(b'changed')
            elif kind=='source_symlink':
                p=args.output_dir/'source_executed.py';outside=tmp_path/'outside.py';outside.write_bytes(p.read_bytes());p.unlink();p.symlink_to(outside)
            else:(args.output_dir/'unplanned.bin').write_bytes(b'not approved')
        return original(raw,field)
    monkeypatch.setattr(literal,'decode_literals',change)
    assert literal.run(args)==2
    r=literal.audit.json_load((args.output_dir/'report.json').read_bytes())
    assert r['status']=='FAILED_LITERAL_DIAGNOSTIC' and r['reported_field_count']==32 and r['postcheck_errors']
    entries=literal.inventory.manifest_entries((args.output_dir/'SHA256SUMS').read_bytes())
    assert 'report.json' in entries and 'unplanned.bin' not in entries


@pytest.mark.parametrize('kind',['gpu_visible','usersite','input_child','reference_child','dataset_alias'])
def test_cpu_and_fresh_output_scope_guards(local_case,monkeypatch,kind,tmp_path):
    args,_,_=local_case
    if kind=='gpu_visible':monkeypatch.setenv('CUDA_VISIBLE_DEVICES','0')
    if kind=='usersite':monkeypatch.delenv('PYTHONNOUSERSITE')
    if kind=='input_child':args.output_dir=args.input_root/'new'
    if kind=='reference_child':args.output_dir=args.reference_root/'new'
    if kind=='dataset_alias':
        repo=tmp_path/'repo';data=tmp_path/'dataset';repo.mkdir();data.mkdir();(repo/'datasets').symlink_to(data,target_is_directory=True)
        monkeypatch.setattr(literal.inventory,'REPO',repo);args.output_dir=data/'new'
    with pytest.raises(ValueError):literal.run(args)
    assert not args.output_dir.exists()


def test_no_pickle_vm_numpy_orm_or_database_import_in_numeric_source():
    source=ast.parse(Path(literal.__file__).read_text())
    forbidden={'pickle','numpy','sqlite3','zipfile','sqlalchemy','urllib','subprocess'}
    for node in ast.walk(source):
        if isinstance(node,ast.Import):assert all(n.name.split('.')[0] not in forbidden for n in node.names)
        if isinstance(node,ast.ImportFrom):assert node.module.split('.')[0] not in forbidden
        if isinstance(node,ast.Call):
            if isinstance(node.func,ast.Name):assert node.func.id not in ('eval','exec','compile','__import__')
            if isinstance(node.func,ast.Attribute):assert node.func.attr not in ('load','loads','Unpickler','deserialize')


def test_parser_no_abbreviations_and_original_signal_handlers_restored(monkeypatch):
    with pytest.raises(SystemExit):literal.main(['--input-r','x'])
    before={s:signal.getsignal(s) for s in (signal.SIGALRM,signal.SIGINT,signal.SIGTERM)}
    monkeypatch.setattr(literal,'run',lambda _:0)
    assert literal.main(sum(([flag,'x'] for flag in ('--input-root','--prior-report','--reference-root','--plan','--output-dir')),[]))==0
    assert {s:signal.getsignal(s) for s in before}==before
