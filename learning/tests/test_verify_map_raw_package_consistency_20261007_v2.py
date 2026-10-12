"""HH_261007 - Pure scalar/AST/fake-stream tests only; no NumPy/PIL/models/payloads/ROS or affinity syscalls."""
# HH_261007 - Fresh V2 proves SHA-addressed copy deduplication only; frozen V1 storage denial/source/test stay unchanged.
from __future__ import annotations

import ast
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import zipfile

import pytest

ROOT=Path(__file__).resolve().parents[2]
TOOL=ROOT/'tools/e2e/verify_map_raw_package_consistency_20261007_v2.py'


@pytest.fixture
def tool():
    spec=importlib.util.spec_from_file_location('_hh_map_pixel_pure',TOOL)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def header(shape,*,dtype='|u1',order=False,version=b'\x01\x00',body=None):
    text=body if body is not None else repr(dict(descr=dtype,fortran_order=order,shape=shape))
    raw=text.encode('latin1');raw+=b' '*(117-len(raw))+b'\n'
    return b'\x93NUMPY'+version+len(raw).to_bytes(2,'little')+raw


def archive(members):
    buffer=io.BytesIO()
    with zipfile.ZipFile(buffer,'w',zipfile.ZIP_DEFLATED) as z:
        for name,data in members:z.writestr(name,data)
    return buffer.getvalue()


def fake_catalog(tool):
    return [dict(cohort=c,ordinal=n,native_frame=1000+n,timestamp_s=10.+n/10) for c,count in tool.COHORTS for n in range(count)]


def test_source_imports_stdlib_only_no_numeric_autorun(tool):
    tree=ast.parse(TOOL.read_bytes())
    imports={n.names[0].name.split('.')[0] if isinstance(n,ast.Import) else (n.module or '').split('.')[0]
        for n in tree.body if isinstance(n,(ast.Import,ast.ImportFrom))}
    assert not imports&{'numpy','PIL','torch','carla','rclpy','portable_e2e'}
    assert not any(isinstance(n,ast.Call) and ast.unparse(n.func) in ('prepare','execute','catalog') for n in tree.body)
    with pytest.raises(ValueError):tool.authorization(None,None)


def test_fixed_original_resources_scope_and_no_adoption(tool):
    v=tool.fixed()
    assert v['logical_CPU_affinity']==[3] and v['threads']==1
    assert v['VmHWM_cap_bytes']==512*1024**2 and v['floor_bytes']==2*1024**3 and v['reserve_bytes']==64*1024**2
    assert v['output_cap_bytes']==2*1024**2 and v['deadline_seconds']==900
    assert v['combined_PLAN_and_OUTPUT_file_byte_budget'] and v['closing_bytes']==512*1024
    assert v['deadline_enforcement']=='sampled_nonpreemptive_including_closing'
    assert v['noNN'] and v['no_TRAIN_TEST_pixels'] and v['predictions_only_metadata_full_SHA']
    assert v['OS_memory_containment_claimed'] is v['model_improvement'] is False
    assert len(tool.PREDICTION_PINS)==6 and len(tool.HEADER_PINS)==4


@pytest.mark.parametrize('shape',[(2,1,1,3),(2,1,1),(360,640,4)])
def test_header_strict_valid_uint8_and_exact_bytes(tool,shape):
    raw=header(shape);assert len(raw)==128
    assert tool.npy_header(io.BytesIO(raw),shape)==raw


@pytest.mark.parametrize('fault',['magic','version','short_length','empty','oversize','truncated','dtype','order','shape','bool_dimension','duplicate','extra','list_shape'])
def test_header_malformed_dynamic_or_padded_contract_denied(tool,fault):
    shape=(2,1,1);raw=header(shape)
    if fault=='magic':raw=b'X'+raw[1:]
    elif fault=='version':raw=header(shape,version=b'\x02\x00')
    elif fault=='short_length':raw=raw[:9]
    elif fault=='empty':raw=raw[:8]+bytes(2)
    elif fault=='oversize':raw=raw[:8]+(4097).to_bytes(2,'little')
    elif fault=='truncated':raw=raw[:-1]
    elif fault=='dtype':raw=header(shape,dtype='<f4')
    elif fault=='order':raw=header(shape,order=True)
    elif fault=='shape':raw=header((3,1,1))
    elif fault=='bool_dimension':raw=header((True,1,1))
    elif fault=='duplicate':raw=header(shape,body="{'descr':'|u1','descr':'|u1','shape':(2,1,1),'fortran_order':False}")
    elif fault=='extra':raw=header(shape,body="{'descr':'|u1','shape':(2,1,1),'fortran_order':False,'other':1}")
    else:raw=header(shape,body="{'descr':'|u1','shape':[2,1,1],'fortran_order':False}")
    with pytest.raises((ValueError,SyntaxError)):tool.npy_header(io.BytesIO(raw),shape)


def test_zip_two_sequential_streams_no_padding_and_CRC(tool):
    from contextlib import ExitStack
    raw=archive([('rgb.npy',header((2,1,1,3))+b'abcdef'),('target.npy',header((2,1,1))+b'12')])
    with ExitStack() as stack:
        z=stack.enter_context(zipfile.ZipFile(io.BytesIO(raw)));tool.zip_census(z,('rgb.npy','target.npy'))
        rgb=tool.entry_stream(stack,z,'rgb.npy',(2,1,1,3));target=tool.entry_stream(stack,z,'target.npy',(2,1,1))
        assert tool.read_exact(rgb,3)==b'abc' and tool.read_exact(target,1)==b'1'
        assert tool.read_exact(rgb,3)==b'def' and tool.read_exact(target,1)==b'2'
        tool.finish_stream(rgb);tool.finish_stream(target)


@pytest.mark.parametrize('names',[['rgb.npy'],['rgb.npy','target.npy','extra.npy'],['rgb.npy','rgb.npy']])
def test_zip_members_missing_extra_duplicate_denied(tool,names):
    with pytest.warns(UserWarning) if names.count('rgb.npy')>1 else __import__('contextlib').nullcontext():
        raw=archive([(name,header((1,))+b'0') for name in names])
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        with pytest.raises(ValueError):tool.zip_census(z,('rgb.npy','target.npy'))


@pytest.mark.parametrize('body',[b'abc',b'abcde'])
def test_entry_declared_length_disallows_truncation_padding(tool,body):
    from contextlib import ExitStack
    with ExitStack() as stack:
        z=stack.enter_context(zipfile.ZipFile(io.BytesIO(archive([('semantic.npy',header((4,))+body)]))))
        with pytest.raises(ValueError):tool.entry_stream(stack,z,'semantic.npy',(4,))


def test_read_exact_and_EOF_fail_instead_of_padding(tool):
    with pytest.raises(ValueError):tool.read_exact(io.BytesIO(b'ab'),3)
    with pytest.raises(ValueError):tool.read_exact(io.BytesIO(b'a'),0)
    with pytest.raises(ValueError):tool.read_exact(io.BytesIO(b'a'),921601)
    with pytest.raises(ValueError):tool.finish_stream(io.BytesIO(b'X'))


def test_crc_corruption_is_not_an_accepted_frame(tool):
    data=bytearray(archive([('target.npy',header((2,))+b'ab')]))
    central=data.index(b'PK\x01\x02');data[central+16]^=1
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        with pytest.raises(zipfile.BadZipFile):z.read('target.npy')


def test_raw_three_members_streams_semantic_only_depth_pose_bodies_unread(tool):
    from contextlib import ExitStack
    members=[('depth.npy',header((1,),dtype='<f4')+bytes(4)),('semantic.npy',header((1,1,4))+b'BGRA'),
        ('world_from_camera.npy',header((4,4),dtype='<f8')+bytes(128))]
    class SemanticOnly(zipfile.ZipFile):
        def open(self,name,*args,**kwargs):
            assert name=='semantic.npy';return super().open(name,*args,**kwargs)
    with ExitStack() as stack:
        z=stack.enter_context(SemanticOnly(io.BytesIO(archive(members))))
        tool.zip_census(z,('depth.npy','semantic.npy','world_from_camera.npy'))
        stream=tool.entry_stream(stack,z,'semantic.npy',(1,1,4));assert tool.read_exact(stream,4)==b'BGRA';tool.finish_stream(stream)
    with zipfile.ZipFile(io.BytesIO(archive(members+[('extra.npy',b'x')]))) as z:
        with pytest.raises(ValueError):tool.zip_census(z,('depth.npy','semantic.npy','world_from_camera.npy'))


def test_header_only_actual_raw_semantic_sample_literal_and_CRC_failure(tool):
    from contextlib import ExitStack
    raw=bytes.fromhex('934e554d5059010076007b276465736372273a20277c7531272c2027666f727472616e5f6f72646572273a2046616c73652c20277368617065273a20283336302c203634302c2034292c207d2020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020200a')
    assert len(raw)==128 and hashlib.sha256(raw).hexdigest()==tool.RAW_SEMANTIC_HEADER_PIN
    assert tool.npy_header(io.BytesIO(raw),(360,640,4))==raw
    members=[('depth.npy',header((1,),dtype='<f4')+bytes(4)),('semantic.npy',raw+bytes(921600)),('world_from_camera.npy',header((4,4),dtype='<f8')+bytes(128))]
    data=bytearray(archive(members));central=data.index(b'PK\x01\x02',data.index(b'PK\x01\x02')+4);data[central+16]^=1
    with ExitStack() as stack:
        z=stack.enter_context(zipfile.ZipFile(io.BytesIO(data)));tool.zip_census(z,('depth.npy','semantic.npy','world_from_camera.npy'))
        with pytest.raises(zipfile.BadZipFile):
            stream=tool.entry_stream(stack,z,'semantic.npy',(360,640,4),tool.RAW_SEMANTIC_HEADER_PIN)
            tool.read_exact(stream,921600);tool.finish_stream(stream)


def test_genuine_target_AST_and_seven_labels_using_symbolic_backend_only(tool):
    class Tags:
        def __eq__(self,value):return [v==value for v in (0,1,2,24,7,8,6,255)]
    class Target:
        def __init__(self):self.values=[0]*8
        def __setitem__(self,mask,value):
            for n,selected in enumerate(mask):
                if selected:self.values[n]=value
    uint8=object()
    fake_np=SimpleNamespace(uint8=uint8,zeros=lambda shape,dtype:Target())
    class SymbolicSemantic:
        shape=(360,640,4);dtype=uint8
        def __getitem__(self,key):assert key==(Ellipsis,2);return Tags()
    namespace=tool.private_functions(tool.SEMANTIC,('require','targets'),dict(np=fake_np),('LABELS','RAW_LABELS'))
    assert namespace['LABELS']==tool.LABELS
    assert namespace['targets'](SymbolicSemantic()).values==[0,1,2,3,4,5,6,0]
    assert not {'evaluate','torch','load_models'}&namespace.keys()
    original=ast.parse(tool.SEMANTIC.read_bytes());target=next(n for n in original.body if isinstance(n,ast.FunctionDef) and n.name=='targets')
    assert target.name==namespace['targets'].__name__ and namespace['targets'].__code__.co_firstlineno==target.lineno


def test_original_private_source_change_denied_before_AST_exec(tool,tmp_path,monkeypatch):
    path=tmp_path/'fake.py';path.write_text('def targets(x): return x\n')
    monkeypatch.setattr(tool,'SOURCE_PINS',{path:'0'*64})
    with pytest.raises(ValueError):tool.private_functions(path,('targets',),{})


@pytest.mark.parametrize('text',[
    '', 'Pid: 5\nTgid: 5\nVmRSS: 1 kB\n',
    'Pid: 5\nTgid: 5\nVmRSS: 1 kB\nVmHWM: 2 MB\n',
    'Pid: 5\nTgid: 5\nVmRSS: 3 kB\nVmHWM: 2 kB\n',
    'Pid: 5\nTgid: 5\nVmRSS: 1 kB\nVmHWM: 2 kB\nVmHWM: 2 kB\n'])
def test_genuine_process_memory_parser_missing_duplicate_units_chronology(tool,text):
    import re
    parser=tool.private_functions(tool.MEMORY,('require','parse_process_memory'),dict(re=re))['parse_process_memory']
    with pytest.raises(ValueError):parser(text,5)


def test_budget_uses_observedVmHWM_not_inherited_ru_max_and_retains_violation(tool,monkeypatch):
    parser=lambda *_:None
    monkeypatch.setattr(tool,'private_functions',lambda *a,**k:{'parse_process_memory':parser})
    monkeypatch.setattr(tool,'time',SimpleNamespace(monotonic=lambda:1.0))
    observed={'VmRSS_bytes':10*1024,'VmHWM_bytes':20*1024,'inherited_ru_maxrss_bytes_not_gate':4*1024**3}
    monkeypatch.setattr(tool,'memory_read',lambda *_:dict(observed))
    monkeypatch.setattr(tool.os,'statvfs',lambda _:SimpleNamespace(f_bavail=tool.FLOOR+tool.RESERVE,f_frsize=1))
    budget=tool.Budget();budget.guard();assert budget.last_memory['inherited_ru_maxrss_bytes_not_gate']>tool.RSS
    observed['VmHWM_bytes']=tool.RSS+1
    with pytest.raises(ValueError,match='VmHWM'):budget.guard()
    assert budget.last_memory['VmHWM_bytes']==tool.RSS+1


def test_whole350_failed_metrics_and_unexecuted_never_padded_or_dropped(tool):
    rows=tool.frame_ledger(fake_catalog(tool));rows[0]['status']='PASS';rows[1]['status']='FAILED'
    metrics={'RGB_mismatched_bytes':3,'target_mismatched_pixels':2};rows[1]['metrics']=metrics
    memory={'VmHWM_bytes':tool.RSS+1};error=ValueError('observed cap exceeded after real metrics')
    failure=tool.retained_failure(rows,1,error,'post_frame',memory)
    assert len(rows)==350 and rows[1]['metrics'] is metrics and failure['resources'] is memory
    assert sum(r['status']=='PASS' for r in rows)==sum(r['status']=='FAILED' for r in rows)==1
    assert sum(r['status']=='UNEXECUTED' for r in rows)==348 and all(r['metrics'] is None for r in rows[2:])
    with pytest.raises(ValueError):tool.retained_failure(rows,2,error,'fake_started',memory)


@pytest.mark.parametrize('fault',['missing','duplicate','swap','cohorts','boolordinal'])
def test_ledger_full_native_order_malformed_denied(tool,fault):
    rows=fake_catalog(tool)
    if fault=='missing':rows.pop()
    elif fault=='duplicate':rows[1]=dict(rows[0])
    elif fault=='swap':rows[0],rows[1]=rows[1],rows[0]
    elif fault=='cohorts':rows=rows[118:]+rows[:118]
    else:rows[0]['ordinal']=False
    with pytest.raises(ValueError):tool.frame_ledger(rows)


def test_closing_checks_entire_table_and_archives_after_first_failure(tool,tmp_path,monkeypatch):
    monkeypatch.setattr(tool,'ROOT',tmp_path);monkeypatch.setattr(tool,'PLAN_DIR',tmp_path/'plan')
    monkeypatch.setattr(tool,'PLAN',tmp_path/'plan.json');monkeypatch.setattr(tool,'AUTHORITY',tmp_path/'auth.json')
    paths=[tmp_path/name for name in ('a.py','b.npz','plan.json','auth.json')];observed=[]
    def fake_digest(p,*_):observed.append(str(p));return 'f'*64
    monkeypatch.setattr(tool,'digest',fake_digest)
    plan={'source_sha256':{str(paths[0]):'a'*64},'input_sha256':{str(paths[1]):'b'*64}}
    errors=tool.closing_seals(plan,'c'*64,'d'*64)
    assert len(errors)==5 and len(observed)==5
    assert str(tmp_path/'plan/sources_by_SHA256'/('a'*64+'.py')) in errors


def test_output_oversize_and_existing_file_do_not_overwrite(tool,tmp_path,monkeypatch):
    monkeypatch.setattr(tool,'LIMIT',20);monkeypatch.setattr(tool,'CLOSING',0)
    roots=(tmp_path/'plan',tmp_path/'output')
    for root in roots:root.mkdir()
    target=roots[1]/'result.json'
    with pytest.raises(ValueError):tool.write_new(roots[1],target,{'too_large':'x'*30},closing=True,storage_roots=roots)
    assert not target.exists()
    tool.write_new(roots[1],target,{'x':1},closing=True,storage_roots=roots);before=target.read_bytes()
    with pytest.raises(FileExistsError):tool.write_new(roots[1],target,{'x':2},closing=True,storage_roots=roots)
    assert target.read_bytes()==before


def test_combined_canonical_roots_no_separate_budget_or_alias(tool,tmp_path,monkeypatch):
    roots=(tmp_path/'plan',tmp_path/'output')
    for root in roots:root.mkdir()
    monkeypatch.setattr(tool,'LIMIT',20);monkeypatch.setattr(tool,'CLOSING',4)
    tool.write_bytes(roots[0],roots[0]/'candidate',b'123456',storage_roots=roots)
    tool.write_bytes(roots[1],roots[1]/'ROOT_copy',b'abcdef',storage_roots=roots)
    assert tool.storage_used(roots)==12
    with pytest.raises(ValueError):tool.write_bytes(roots[1],roots[1]/'extra',b'abcde',storage_roots=roots)
    tool.write_bytes(roots[1],roots[1]/'result',bytes(8),closing=True,storage_roots=roots)
    assert tool.storage_used(roots)==20
    with pytest.raises(ValueError):tool.write_bytes(tmp_path,tmp_path/'not_our_roots',b'x',storage_roots=roots)
    with pytest.raises(ValueError):tool.storage_used((roots[0],roots[0]/'nested'))
    alias=roots[0]/'alias';alias.symlink_to(roots[1]/'result')
    with pytest.raises(ValueError):tool.storage_used(roots)


def test_storage_proof_counts_two_ROOT_copies_and_complete_closing_before_output(tool,tmp_path,monkeypatch):
    planroot=tmp_path/'plan';planroot.mkdir();output=tmp_path/'output'
    monkeypatch.setattr(tool,'PLAN_DIR',planroot);monkeypatch.setattr(tool,'OUTPUT',output)
    monkeypatch.setattr(tool,'PLAN',planroot/'ROOT_plan.json');monkeypatch.setattr(tool,'AUTHORITY',planroot/'ROOT_authority.json')
    monkeypatch.setattr(tool,'CLOSING',10)
    caps={'source_archive_bytes':3,'ROOT_plan_copy_max_bytes':1024,'ROOT_authority_copy_max_bytes':1024,'closing_reserved_bytes':10,'closing_complete_envelope_max_bytes':9}
    plan={'storage_protocol':caps};monkeypatch.setattr(tool,'storage_protocol',lambda _:dict(caps))
    bound=3+len(tool.encoded(plan))+2*1024+2*1024+10
    monkeypatch.setattr(tool,'LIMIT',bound-1)
    with pytest.raises(ValueError):tool.storage_admission(plan,prospective=True)
    monkeypatch.setattr(tool,'LIMIT',bound);assert tool.storage_admission(plan,prospective=True)==bound
    tool.PLAN.write_bytes(b'plan');tool.AUTHORITY.write_bytes(b'auth')
    monkeypatch.setattr(tool,'LIMIT',26);assert tool.storage_admission(plan)==18 and not output.exists()
    (planroot/'ROOT_review.json').write_bytes(b'x')
    with pytest.raises(ValueError):tool.storage_admission(plan)
    (planroot/'ROOT_review.json').unlink();tool.PLAN.write_bytes(bytes(1025));monkeypatch.setattr(tool,'LIMIT',10000)
    with pytest.raises(ValueError,match='copy cap'):tool.storage_admission(plan)


def test_closing_bound_populates_all350_rows_and_all_seal_errors(tool,tmp_path,monkeypatch):
    monkeypatch.setattr(tool,'ROOT',tmp_path);monkeypatch.setattr(tool,'PLAN_DIR',tmp_path/'plan')
    monkeypatch.setattr(tool,'OUTPUT',tmp_path/'out');monkeypatch.setattr(tool,'PLAN',tmp_path/'plan/ROOT_plan.json');monkeypatch.setattr(tool,'AUTHORITY',tmp_path/'plan/ROOT_authority.json')
    plan={'catalog':fake_catalog(tool),'source_sha256':{str(tmp_path/f'source_{n}.py'):f'{n%21:064x}' for n in range(36)},
        'input_sha256':{str(tmp_path/f'input_{n}.npz'):'b'*64 for n in range(717)}}
    original=tool.final_result;observed={}
    def record(*args):
        observed['rows']=args[1];observed['errors']=args[3];return original(*args)
    monkeypatch.setattr(tool,'final_result',record)
    size=tool.closing_size_proof(plan)
    assert size<=512*1024 and len(observed['rows'])==350 and all(r['metrics'] is not None and r['resources'] for r in observed['rows'])
    assert len(observed['errors'])==776 and len(observed['rows'][-1]['metrics']['target_class_pixels'])==256
    assert all(len(r['metrics']['target_class_pixels'])==7 for r in observed['rows'][:-1])
    assert all('status' not in r for r in plan['catalog'])
    monkeypatch.setattr(tool,'META_LIMIT',size-1)
    with pytest.raises(ValueError,match='maximum closing'):tool.closing_size_proof(plan)


def test_indexed_closing_errors_preserve_manifest_unknowns_and_raw_exception_identity(tool,tmp_path,monkeypatch):
    monkeypatch.setattr(tool,'ROOT',tmp_path);monkeypatch.setattr(tool,'PLAN_DIR',tmp_path/'plan')
    plan={'catalog':fake_catalog(tool),'source_sha256':{str(tmp_path/'a.py'):'a'*64},'input_sha256':{str(tmp_path/'b.npz'):'b'*64}}
    raw=ValueError('large\n'+('x'*4096));error=tool.bounded_error(raw)
    assert error['truncated'] and error['raw_bytes']==len(repr(raw).encode()) and error['raw_SHA256']==hashlib.sha256(repr(raw).encode()).hexdigest()
    witness=tool.seal_exception(raw)
    assert witness==[hashlib.sha256(repr(raw).encode()).hexdigest(),len(repr(raw).encode()),repr(raw).encode()[:16].hex()]
    errors={str(tmp_path/'a.py'):dict(actual=None,error=witness)}
    result=tool.final_result(plan,tool.frame_ledger(plan['catalog']),None,errors,'c'*64,'d'*64,False,True,None,1.)
    assert result['closing_seal_errors']==[[0,None,witness]] and result['source_count']==result['input_count']==1
    assert 'source_sha256' not in result and 'input_sha256' not in result
    assert result['full_source_input_maps_in_exact_archived_ROOT_plan'].endswith('/ROOT_plan.json')


def test_actual_numeric_imports_after_seals_no_array_load_floatcasts_or_depth_pose(tool):
    tree=ast.parse(TOOL.read_bytes());execute=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
    text=ast.unparse(execute);assert text.index('verify_tables(plan, budget)')<text.index('import numpy as np')<text.index('from PIL import Image')
    assert text.index('storage_admission(plan)')<text.index('OUTPUT.mkdir')<text.index('import numpy as np')
    assert 'np.load' not in text and 'float32' not in text and '.astype(' not in text
    assert "entry_stream(sensor_stack, sensor, 'semantic.npy'" in text
    assert "entry_stream(sensor_stack, sensor, 'depth.npy'" not in text and "entry_stream(sensor_stack, sensor, 'world_from_camera.npy'" not in text
    assert 'model(' not in text and 'forward(' not in text
    assert text.index("row['metrics'] =")<text.index("stage = 'post_frame'")
    assert 'retained_failure(rows, current, error, stage, budget.last_memory)' in text


def test_no_evolving_README_scientific_input_or_whole_prediction_decoder(tool):
    assert not any(p.suffix=='.md' for p in tool.META_PINS)
    text=TOOL.read_text();assert 'mapping_review/README' not in text
    assert "PREDICTION_PINS" in text and 'prediction.npy' not in text


def test_many_path_source_seals_keep_unique_copy_and_every_current_original_path(tool,tmp_path,monkeypatch):
    monkeypatch.setattr(tool,'PLAN_DIR',tmp_path/'plan')
    paths=[tmp_path/name for name in ('current.py','original.py','different.py')]
    contents=[b'original source\n',b'original source\n',b'different source\n']
    for p,raw in zip(paths,contents):p.write_bytes(raw)
    pins={str(p):hashlib.sha256(raw).hexdigest() for p,raw in zip(paths,contents)};plan={'source_sha256':pins,'input_sha256':{}}
    archives=tool.source_archives(plan)
    assert len(pins)==3 and len(archives)==2 and tool.source_archive_bytes(plan)==len(contents[0])+len(contents[2])
    assert all(Path(p).parent==tmp_path/'plan/sources_by_SHA256' and Path(p).name==h+'.py' for p,h in archives.items())
    manifest=tool.seal_manifest(plan,'a'*64,'b'*64)
    assert all(manifest[str(p)]==pins[str(p)] for p in paths) and len(manifest)==7
    paths[1].write_bytes(b'replaced larger original\n')
    with pytest.raises(ValueError,match='different byte size'):tool.source_archive_bytes(plan)


def test_original_math_resources_and_order_are_exact_V1_AST_except_source_copy_seams(tool):
    original=ROOT/'tools/e2e/verify_map_raw_package_consistency_20261007_v1.py'
    original_test=ROOT/'learning/tests/test_verify_map_raw_package_consistency_20261007_v1.py'
    assert hashlib.sha256(original.read_bytes()).hexdigest()=='416d40cc14a4258cd9f7f49aedfe861caea2b3729f465704db23c21cecfdfbf8'
    assert hashlib.sha256(original_test.read_bytes()).hexdigest()=='11985dc5e49ced45e9b61668a0194a2c9c34fe573f178d1d81edc96a8e33601e'
    before={n.name:n for n in ast.parse(original.read_bytes()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
    after={n.name:n for n in ast.parse(TOOL.read_bytes()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
    changed={'verify_tables','seal_manifest','storage_protocol','prepare'}
    assert set(after)-set(before)=={'source_archives','source_archive_bytes'} and set(before)<=set(after)
    for name in set(before)-changed:
        candidate=after[name]
        if name=='final_result':
            class UndoArchiveLabel(ast.NodeTransformer):
                def visit_Constant(self,node):
                    if isinstance(node.value,str):node.value=node.value.replace('then_unique_SHA_source_archives;','then_source_archives;')
                    return node
            candidate=UndoArchiveLabel().visit(ast.parse(ast.unparse(candidate)).body[0])
        assert ast.dump(before[name],include_attributes=False)==ast.dump(candidate,include_attributes=False),name
    assert tool.NAME=='map_raw_package_consistency_v2' and tool.SCHEMA.endswith('.v2')
    assert tool.TEST.name=='test_verify_map_raw_package_consistency_20261007_v2.py'


def test_closing_every_duplicate_original_path_and_unique_archive_is_checked(tool,tmp_path,monkeypatch):
    monkeypatch.setattr(tool,'PLAN_DIR',tmp_path/'plan');monkeypatch.setattr(tool,'PLAN',tmp_path/'plan/ROOT_plan.json')
    monkeypatch.setattr(tool,'AUTHORITY',tmp_path/'plan/ROOT_authority.json')
    paths=[str(tmp_path/name) for name in ('current.py','original.py','other.py')]
    plan={'source_sha256':dict(zip(paths,['a'*64,'a'*64,'b'*64])),'input_sha256':{}}
    expected=tool.seal_manifest(plan,'c'*64,'d'*64);observed=[]
    def fake_digest(p,*args):
        p=str(p);observed.append(p);return 'f'*64 if p==paths[1] else expected[p]
    monkeypatch.setattr(tool,'digest',fake_digest)
    errors=tool.closing_seals(plan,'c'*64,'d'*64)
    assert list(errors)==[paths[1]] and len(observed)==7 and set(observed)==set(expected)
    assert len(tool.source_archives(plan))==2 and all(p in observed for p in paths)


@pytest.mark.parametrize('changed_original',[False,True])
def test_genuine_prepare_fake_sources_copies_once_never_overwrites_duplicate(tool,tmp_path,monkeypatch,changed_original):
    from contextlib import contextmanager
    monkeypatch.setattr(tool,'ROOT',tmp_path);monkeypatch.setattr(tool,'PLAN_DIR',tmp_path/'plan');monkeypatch.setattr(tool,'OUTPUT',tmp_path/'output')
    paths=[tmp_path/name for name in ('current.py','original.py','other.py')];contents=[b'same source\n',b'same source\n',b'other source\n']
    for p,raw in zip(paths,contents):p.write_bytes(raw)
    pins={str(p):hashlib.sha256(raw).hexdigest() for p,raw in zip(paths,contents)}
    @contextmanager
    def fake_lease():yield (1,2)
    monkeypatch.setattr(tool,'lease',fake_lease);monkeypatch.setattr(tool,'Budget',lambda:SimpleNamespace(guard=lambda:None))
    monkeypatch.setattr(tool,'catalog',lambda:fake_catalog(tool));monkeypatch.setattr(tool,'sources',lambda:dict(pins))
    monkeypatch.setattr(tool,'input_pins',lambda _:{});monkeypatch.setattr(tool,'environment',lambda:{})
    caps={'source_archive_bytes':len(contents[0])+len(contents[2]),'ROOT_plan_copy_max_bytes':65536,
        'ROOT_authority_copy_max_bytes':2048,'closing_complete_envelope_max_bytes':1,'closing_reserved_bytes':tool.CLOSING}
    monkeypatch.setattr(tool,'storage_protocol',lambda _:dict(caps))
    writes=[];original_write=tool.write_bytes
    def record(root,path,payload,**kwargs):writes.append(Path(path));return original_write(root,path,payload,**kwargs)
    monkeypatch.setattr(tool,'write_bytes',record)
    if changed_original:paths[1].write_bytes(b'diff source\n')
    if changed_original:
        with pytest.raises(ValueError,match='source changed before archive'):tool.prepare()
        assert not (tool.PLAN_DIR/'candidate_plan.json').exists()
        assert (tool.PLAN_DIR/'sources_by_SHA256'/(pins[str(paths[0])]+'.py')).read_bytes()==contents[0]
    else:
        result=tool.prepare();plan=json.loads((tool.PLAN_DIR/'candidate_plan.json').read_bytes())
        assert result['status']=='CANDIDATE_ONLY_NOT_ROOT_AUTHORITY' and plan['source_sha256']==pins
        assert len(list((tool.PLAN_DIR/'sources_by_SHA256').iterdir()))==2
        assert len([p for p in writes if p.parent.name=='sources_by_SHA256'])==2
    assert not tool.OUTPUT.exists() and all(p.read_bytes()==raw for p,raw in zip(paths[:1]+paths[2:],contents[:1]+contents[2:]))
