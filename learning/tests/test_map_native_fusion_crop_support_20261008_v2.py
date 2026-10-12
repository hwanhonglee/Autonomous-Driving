"""HH_261008 - Pure scalar/AST/fake-stream tests; never decode actual labels or import numeric/model providers.

Fixtures represent whole-frame GT pixel support, not connected components or
physical sign objects. Synthetic journals are explicitly smaller helper cases,
never evidence for the production20k/320k denominator.
"""
from __future__ import annotations

# HH_261008 - Fresh V2 adds fake-native full/extra/missing/failed progression tests without editing the frozen V1 suite.

import ast
from contextlib import nullcontext
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import zipfile

import pytest

ROOT=Path(__file__).resolve().parents[2]
TOOL=ROOT/'tools/e2e/audit_map_native_fusion_crop_support_20261008_v2.py'


@pytest.fixture
def tool():
    spec=importlib.util.spec_from_file_location('_hh_native_crop_pure_v2',TOOL)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def batch(step=1,ids=None,boxes=None,bits=None):
    ids=[0,1] if ids is None else ids
    return dict(step=step,physical_slot_indices=ids,
        shared_crop_boxes=boxes or [[0,0,360,640] for _ in ids],
        actual_paired_TRUE_bits=[0]*len(ids) if bits is None else bits,
        RGB_sha256='a'*64,target_sha256=hashlib.sha256(str(step).encode()).hexdigest())


def journal(tmp_path,rows,*,name='journal.jsonl',newline=True):
    path=tmp_path/name
    path.write_bytes(b'\n'.join(json.dumps(row).encode() for row in rows)+(b'\n' if newline else b''))
    return path


def header(shape,*,dtype='|u1',order=False,version=b'\x01\x00',body=None):
    text=body if body is not None else repr(dict(descr=dtype,fortran_order=order,shape=shape))
    raw=text.encode('latin1');raw+=b' '*(117-len(raw))+b'\n'
    return b'\x93NUMPY'+version+len(raw).to_bytes(2,'little')+raw


def archive(members,compression=zipfile.ZIP_DEFLATED):
    value=io.BytesIO()
    with zipfile.ZipFile(value,'w',compression) as z:
        for name,raw in members:z.writestr(name,raw)
    return value.getvalue()


def fake_plan(tool):
    return dict(source_sha256={'/fake/source.py':'a'*64},input_sha256={'/fake/input.json':'b'*64},
        environment=dict(origins={},selected_provider_sha256={},historical_environment_equivalence_claimed=False),
        evidence=dict(original_runtime=dict(pillow='original_fixture',numpy='original_fixture')),
        prepare_process_generation=dict(PID=1,PPID=2,process_group=3,session=4,start_ticks=5,
            boot_id='test',executable='/fake/exe',cmdline_sha256='c'*64))


def test_top_level_is_stdlib_only_no_autorun(tool):
    tree=ast.parse(TOOL.read_bytes())
    names={n.names[0].name.split('.')[0] if isinstance(n,ast.Import) else (n.module or '').split('.')[0]
        for n in tree.body if isinstance(n,(ast.Import,ast.ImportFrom))}
    assert not names&{'numpy','PIL','torch','carla','rclpy','portable_e2e'}
    assert not any(isinstance(n,ast.Call) and ast.unparse(n.func) in ('prepare','execute','evidence','sources') for n in tree.body)
    with pytest.raises(ValueError):tool.authorization(None,None)


def test_source_contains_no_full_package_loader_or_RGB_member_open(tool):
    tree=ast.parse(TOOL.read_bytes())
    forbidden={'np.load','numpy.load','read_npz','load_package','train_package','providers'}
    assert not any(isinstance(n,ast.Call) and ast.unparse(n.func) in forbidden for n in ast.walk(tree))
    numeric_imports=[n for n in ast.walk(tree) if isinstance(n,ast.Import) and any(a.name=='numpy' for a in n.names)]
    assert len(numeric_imports)==1
    opens=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and ast.unparse(n.func)=='archive.open']
    assert len(opens)==1 and ast.literal_eval(opens[0].args[0])=='target.npy'
    execute=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
    assert 'authorization(plan_sha, grant_sha)' in ast.unparse(execute)
    assert ast.unparse(execute).index('verify_tables(plan, budget)')<ast.unparse(execute).index('import numpy')
    assert ast.unparse(execute).index('storage_admission(plan)')<ast.unparse(execute).index('import numpy')


def test_production_scope_has_both_denominators_and_original_resource_caps(tool):
    v=tool.fixed()
    assert v['original_steps_per_arm']==20000 and v['batch']==8
    assert v['original_exposures_per_arm']==160000 and v['original_exposures_both_arms']==320000
    assert v['computed_shared_exposures']==160000 and v['physical_TRAIN_slots']==1209
    assert v['targets_only'] and not v['RGB_TRUE_weights_DEV_HOLDOUT_TEST_CTrack_bodies_opened']
    assert v['observed_VmRSS_VmHWM_cap_bytes']==512*1024**2 and not v['inherited_ru_maxrss_is_gate']
    assert v['output_cap_bytes']==2*1024**2 and v['closing_bytes']==512*1024
    assert v['deadline_seconds']==900 and v['deadline_enforcement']=='sampled_nonpreemptive_including_closing'
    assert v['floor_bytes']==2*1024**3 and v['reserve_bytes']==64*1024**2 and v['logical_CPU_affinity']==[3]
    assert not v['historical_environment_equivalence_claimed'] and not v['model_promoted']
    assert v['NN']==v['training_steps']==v['main_approved']==v['sub_approved']==0


@pytest.mark.parametrize('bad',[True,1.0,'1',0,-1,20001])
def test_journal_step_is_strict_positive_declared_integer(tool,bad):
    row=batch();row['step']=bad
    with pytest.raises(ValueError):tool.validate_batch(row,0,total=3,batch=2)


@pytest.mark.parametrize('bad',[True,0.0,'0',-1,3])
def test_indices_never_coerce_bool_float_or_out_of_pool(tool,bad):
    row=batch(ids=[bad,1])
    with pytest.raises(ValueError):tool.validate_batch(row,0,total=3,batch=2)


@pytest.mark.parametrize('bad',[1,True,0.0,'0',-1])
def test_TRUE_or_coerced_view_bit_is_denied(tool,bad):
    with pytest.raises(ValueError):tool.validate_batch(batch(bits=[bad,0]),0,total=3,batch=2)


@pytest.mark.parametrize('box',[[0,0,0,640],[-1,0,360,640],[0,-1,360,640],[0,0,361,640],
    [0,0,360,641],[1,0,360,640],[0,1,360,640],[0,0,360],[False,0,360,640],[0.,0,360,640]])
def test_original_crop_bounds_and_types_are_not_relaxed(tool,box):
    with pytest.raises(ValueError):tool.validate_batch(batch(boxes=[box,[0,0,360,640]]),0,total=3,batch=2)


@pytest.mark.parametrize('fault',['extra','missing','short_ids','short_boxes','short_bits','bad_target_sha','bad_RGB_sha'])
def test_whole_actual_B8_schema_and_hash_witnesses_required(tool,fault):
    row=batch()
    if fault=='extra':row['new']=0
    elif fault=='missing':del row['target_sha256']
    elif fault.startswith('short_'):row[{'short_ids':'physical_slot_indices','short_boxes':'shared_crop_boxes','short_bits':'actual_paired_TRUE_bits'}[fault]].pop()
    else:row['target_sha256' if fault=='bad_target_sha' else 'RGB_sha256']='z'*64
    with pytest.raises(ValueError):tool.validate_batch(row,0,total=3,batch=2)


def test_synthetic_complete_journal_exact_sequence_and_hash_chain(tool,tmp_path):
    rows=[batch(1,ids=[0,1]),batch(2,ids=[2,0],boxes=[[1,2,300,500],[0,0,360,640]])]
    path=journal(tmp_path,rows);result=tool.journal_summary(path,steps=2,total=3,batch=2)
    assert result['slot_exposures']==[2,1,1] and result['exposures']==4 and result['whole']==3 and result['partial']==1
    assert result['sample_sequence_sha256']==hashlib.sha256(struct.pack('<4q',0,1,2,0)).hexdigest()
    expected=hashlib.sha256()
    for i,row in enumerate(rows):expected.update(struct.pack('<Q',i+1)+bytes.fromhex(row['target_sha256']))
    assert result['expected_target_chain_sha256']==expected.hexdigest()


@pytest.mark.parametrize('rows',[[batch(1)],[batch(1),batch(2),batch(3)],[batch(1),batch(3)],[batch(2),batch(1)]])
def test_missing_extra_or_reordered_actual_tail_fails(tool,tmp_path,rows):
    with pytest.raises(ValueError):tool.journal_summary(journal(tmp_path,rows),steps=2,total=3,batch=2)


@pytest.mark.parametrize('raw',[b'{"step":1,"step":1}\n',b'{"step":NaN}\n',b'{}',b' '*8193+b'\n'])
def test_journal_duplicate_nonfinite_unterminated_or_oversize_row_denied(tool,tmp_path,raw):
    path=tmp_path/'bad.jsonl';path.write_bytes(raw)
    with pytest.raises(ValueError):list(tool.json_lines(path))


def test_history_is_metadata_only_but_all_updates_are_required(tool,tmp_path):
    rows=[dict(step=i,optimizer_updates=i,original_CE=.1,gradient_norm=.2,wall_seconds=float(i),all_gradients_present_finite=True) for i in (1,2)]
    assert tool.history_summary(journal(tmp_path,rows),steps=2)==2
    rows[1]['optimizer_updates']=1
    with pytest.raises(ValueError):tool.history_summary(journal(tmp_path,rows,name='bad.jsonl'),steps=2)


@pytest.mark.parametrize('key,value',[('original_CE',float('nan')),('gradient_norm',-.1),('wall_seconds',True),('all_gradients_present_finite',False)])
def test_original_update_invalid_scalar_never_qualifies_completion(tool,tmp_path,key,value):
    row=dict(step=1,optimizer_updates=1,original_CE=.1,gradient_norm=.2,wall_seconds=1.,all_gradients_present_finite=True);row[key]=value
    with pytest.raises(ValueError):tool.history_summary(journal(tmp_path,[row]),steps=1)


def test_source_group_boundaries_are_TRAIN_native_FALSE_not_named_HOLDOUT(tool):
    assert [tool.source_group(i) for i in (0,496,497,728,729,968,969,1208)]==[
        tool.GROUPS[0],tool.GROUPS[0],tool.GROUPS[1],tool.GROUPS[1],tool.GROUPS[2],tool.GROUPS[2],tool.GROUPS[3],tool.GROUPS[3]]
    for bad in (True,1.,-1,1209):
        with pytest.raises(ValueError):tool.source_group(bad)


def test_GT_size_bins_are_whole_frame_support_not_components_or_objects(tool):
    values=[0,1,16,17,64,65,256,257,230400]
    assert [tool.frame_mask_support_bin(n) for n in values]==['zero','1_to16','1_to16','17_to64','17_to64','65_to256','65_to256','257_plus','257_plus']
    assert 'frame_mask_support_bins' in tool.empty_statistics()
    assert 'NOT components or physical objects' in tool.fixed()['source_size_unit']


def test_exact_original_label_AST_is_extracted_not_a_full_loader(tool):
    old,node=tool.crop_ast();proof=tool.crop_proof()
    assert not any(isinstance(n,ast.Name) and n.id in ('rgb','image') for n in ast.walk(node))
    assert ast.dump(node.body[0],include_attributes=False)==ast.dump(old.body[2],include_attributes=False)
    assert ast.dump(node.body[1],include_attributes=False)==ast.dump(old.body[3],include_attributes=False)
    assert ast.dump(node.body[3],include_attributes=False)==ast.dump(old.body[5],include_attributes=False)
    assert ast.dump(node.body[4],include_attributes=False)==ast.dump(old.body[7],include_attributes=False)
    assert ast.dump(node.body[-1].value,include_attributes=False)==ast.dump(old.body[-1].value.elts[1],include_attributes=False)
    assert proof['source_sha256']==tool.CROP_PIN and proof['historical_label_bytes_required_for_every_B8']


def test_symbolic_original_label_branch_uses_NEAREST_and_no_RGB(tool):
    calls=[]
    class Target:
        def __getitem__(self,slices):calls.append(('slice',[(s.start,s.stop) for s in slices]));return self
        def copy(self):calls.append(('copy',));return self
        def resize(self,size,method):calls.append(('resize',size,method));return self
    target=Target()
    image=SimpleNamespace(NEAREST=7,fromarray=lambda x:x)
    np=SimpleNamespace(asarray=lambda x:x)
    crop=tool.compile_target_crop(np,image)
    assert crop(target,[0,0,360,640]) is target and calls==[('copy',)]
    calls.clear();assert crop(target,[10,20,200,300]) is target
    assert calls==[('slice',[(10,210),(20,320)]),('resize',(640,360),7),('copy',)]


@pytest.mark.parametrize('fault',['magic','version','length','truncated','dtype','order','shape','bool','duplicate','extra','list'])
def test_NPY_unknown_dtype_shape_order_or_malformed_header_denied(tool,fault):
    shape=(1,360,640);raw=header(shape)
    if fault=='magic':raw=b'X'+raw[1:]
    elif fault=='version':raw=header(shape,version=b'\x02\x00')
    elif fault=='length':raw=raw[:8]+(4097).to_bytes(2,'little')
    elif fault=='truncated':raw=raw[:-1]
    elif fault=='dtype':raw=header(shape,dtype='<f4')
    elif fault=='order':raw=header(shape,order=True)
    elif fault=='shape':raw=header((2,360,640))
    elif fault=='bool':raw=header((True,360,640))
    elif fault=='duplicate':raw=header(shape,body="{'descr':'|u1','descr':'|u1','shape':(1,360,640),'fortran_order':False}")
    elif fault=='extra':raw=header(shape,body="{'descr':'|u1','shape':(1,360,640),'fortran_order':False,'x':0}")
    else:raw=header(shape,body="{'descr':'|u1','shape':[1,360,640],'fortran_order':False}")
    with pytest.raises((ValueError,SyntaxError)):tool.npy_header(io.BytesIO(raw),shape)


@pytest.mark.parametrize('names',[['target.npy'],['rgb.npy','target.npy','../extra.npy'],['rgb.npy','rgb.npy'],['rgb.npy','../target.npy']])
def test_ZIP_member_missing_duplicate_extra_or_traversal_denied(tool,names):
    with pytest.warns(UserWarning) if names.count('rgb.npy')>1 else nullcontext():raw=archive([(n,b'x') for n in names])
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        with pytest.raises(ValueError):tool.zip_census(z,('rgb.npy','target.npy'))


def test_stored_uncompressed_member_is_not_silently_admitted(tool):
    with zipfile.ZipFile(io.BytesIO(archive([('rgb.npy',b'x'),('target.npy',b'y')],zipfile.ZIP_STORED))) as z:
        with pytest.raises(ValueError):tool.zip_census(z,('rgb.npy','target.npy'))


def test_target_only_member_reads_and_fill_preallocated_without_RGB_decode(tool):
    class TargetOnly(zipfile.ZipFile):
        def open(self,name,*args,**options):assert name=='target.npy';return super().open(name,*args,**options)
    raw=archive([('rgb.npy',b'RGB forbidden body'),('target.npy',header((1,360,640))+bytes(360*640))])
    with TargetOnly(io.BytesIO(raw)) as z:
        tool.zip_census(z,('rgb.npy','target.npy'));stream,h=tool.target_entry(z,1)
        with stream:
            out=bytearray(360*640);assert tool.fill_exact(stream,out)==len(out)
        assert len(h)==128 and out==bytearray(360*640)


@pytest.mark.parametrize('length',[360*640-1,360*640+1])
def test_declared_target_padding_or_truncation_fails_before_array(tool,length):
    raw=archive([('rgb.npy',b'x'),('target.npy',header((1,360,640))+bytes(length))])
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        with pytest.raises(ValueError):tool.target_entry(z,1)


@pytest.mark.parametrize('body',[b'ab',b'abcd'])
def test_readinto_short_or_extra_stream_never_pads_labels(tool,body):
    with pytest.raises(ValueError):tool.fill_exact(io.BytesIO(body),bytearray(3))


def test_zero_progress_readinto_is_denied(tool):
    class Bad(io.BytesIO):
        def readinto(self,destination):return 0
    with pytest.raises(ValueError):tool.fill_exact(Bad(b'abc'),bytearray(3))


def test_target_CRC_corruption_is_retained_not_relabelled(tool):
    raw=bytearray(archive([('rgb.npy',b'x'),('target.npy',header((1,360,640))+bytes(360*640))]))
    first=raw.index(b'PK\x01\x02');second=raw.index(b'PK\x01\x02',first+4);raw[second+16]^=1
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        with pytest.raises(zipfile.BadZipFile):
            stream,_=tool.target_entry(z,1)
            with stream:tool.fill_exact(stream,bytearray(360*640))


def test_support_counts_distinguish_crop_cut_from_resize_loss_not_object_count(tool):
    cut=tool.exposure_counts(10,0,[230400,0,0,0,0,0,0],False)
    resize=tool.exposure_counts(10,2,[230400,0,0,0,0,0,0],False)
    keep=tool.exposure_counts(10,10,[230390,0,0,0,0,10,0],True)
    col={key:i for i,key in enumerate(tool.COUNTER_COLUMNS)}
    assert cut[col['raw_crop_cut_all_sign']]==1 and cut[col['resize_zero_from_positive_crop']]==0
    assert resize[col['raw_crop_cut_all_sign']]==0 and resize[col['resize_zero_from_positive_crop']]==1
    assert resize[col['raw_crop_cut_some_sign']]==1 and keep[col['whole_crop']]==1
    assert keep[col['resized_sign_pixels_sum']]==10
    assert len(cut)==18 and sum(keep[11:])==230400


def test_scalar_batch_commit_is_transactional_and_preserves_repeated_slots(tool):
    old=tool.empty_statistics();before=copy.deepcopy(old);signs=[0]*1209;signs[0]=10
    row=tool.exposure_counts(10,10,[230390,0,0,0,0,10,0],True)
    new=tool.commit_counts(old,[0]*8,[row]*8,signs)
    assert old==before and new['shared_once_overall'][0]==8 and new['slot_statistics'][0][0]==8
    assert new['slot_statistics'][0][10]==80
    with pytest.raises(ValueError):tool.commit_counts(old,[0]*8,[row]*7,signs)
    assert old==before


@pytest.mark.parametrize('full,source,hist,whole',[(True,0,[230400,0,0,0,0,0,0],False),
    (1,2,[230400,0,0,0,0,0,0],False),(1,0,[230399,0,0,0,0,0,0],False),
    (1,0,[230400.,0,0,0,0,0,0],False),(1,0,[230400,0,0,0,0,0,0],0)])
def test_support_strict_source_and_seven_class_pixel_denominators(tool,full,source,hist,whole):
    with pytest.raises(ValueError):tool.exposure_counts(full,source,hist,whole)


@pytest.mark.parametrize('passed,failed',[(0,0),(0,1),(123,1),(20000,0)])
def test_failures_keep_both_whole_original_denominators(tool,passed,failed):
    d=tool.denominator(passed,failed)
    assert d['original_total_exposures']==320000
    assert d['original_PASS_exposures']+d['original_FAILED_batch_exposures']+d['original_UNEXECUTED_exposures']==320000
    assert all(a['expected_batches']==20000 and a['PASS_batches']+a['FAIL_batches']+a['UNEXECUTED_batches']==20000 for a in d['by_arm'].values())


@pytest.mark.parametrize('passed,failed',[(True,0),(1,True),(-1,0),(20000,1),(1,2)])
def test_false_or_shrunken_completion_denominator_denied(tool,passed,failed):
    with pytest.raises(ValueError):tool.denominator(passed,failed)


def test_completed_counts_alone_never_override_failure_or_seals(tool):
    p=fake_plan(tool);stats=tool.empty_statistics()
    for failed,failure,seals in [(0,{'stage':'closing'},{}),(0,None,{'/fake/input.json':{'actual':'0'*64}}),(1,{'stage':'batch'}, {})]:
        passed=20000 if not failed else 19999
        r=tool.result_value(p,'c'*64,'d'*64,passed,failed,failure,stats,seals,{},p['prepare_process_generation'],1.,'e'*64,'00',None)
        assert r['status']=='FAILED_WITH_FULL_ORIGINAL_DENOMINATOR' and not r['label_byte_equivalence_proven']
        assert not r['model_improved'] and r['training_steps']==r['NN']==r['main_approved']==r['sub_approved']==0


def test_failure_tail_retains_actual_batch_hash_and_partial_examined_slots(tool):
    p=fake_plan(tool);failure=dict(stage='historical_B8_hash',failed_batch=dict(ordinal=7,
        expected_target_sha256='a'*64,actual_target_sha256='b'*64,examined_exposures=8,computed_support=[[1]*18]*8))
    r=tool.result_value(p,'c'*64,'d'*64,7,1,failure,tool.empty_statistics(),{}, {},p['prepare_process_generation'],1.,'e'*64,'00',None)
    assert r['failure_tail'] is failure and r['by_arm']['control']['UNEXECUTED_batches']==19992
    assert r['failure_tail']['failed_batch']['actual_target_sha256']!='a'*64


def test_external_authority_never_grants_training_or_evaluation_payload(tool):
    value=tool.authority_value(fake_plan(tool),'c'*64)
    assert value['TRAIN_target_support_diagnostic_only_authorized'] and value['every_original_B8_target_hash_required']
    for key in ('training_authorized','NN_authorized','ROS_authorized','GPU_authorized','DEV_HOLDOUT_TEST_authorized','model_promoted'):assert value[key] is False


def test_process_generation_parser_uses_real_start_ticks_not_PID_alone(tool):
    fields=['S','2','3','4']+['0']*15+['987654']+['0']*3
    result=tool.parse_generation('12 (name with ) spaces) '+' '.join(fields),12)
    assert result==dict(PID=12,PPID=2,process_group=3,session=4,start_ticks=987654)
    with pytest.raises(ValueError):tool.parse_generation('13 (x) '+' '.join(fields),12)
    with pytest.raises(ValueError):tool.parse_generation('12 (x) '+' '.join(fields),True)


def test_combined_storage_counts_plan_and_output_and_reserves_closing(tool,tmp_path,monkeypatch):
    plan=tmp_path/'plan';out=tmp_path/'out';plan.mkdir();out.mkdir()
    (plan/'existing').write_bytes(b'x'*10);(out/'existing').write_bytes(b'y'*20)
    monkeypatch.setattr(tool,'LIMIT',40);monkeypatch.setattr(tool,'CLOSING',5)
    assert tool.storage_used((plan,out))==30
    with pytest.raises(ValueError):tool.write_bytes(plan,plan/'denied',b'a'*6,roots=(plan,out))
    tool.write_bytes(out,out/'close',b'a'*10,closing=True,roots=(plan,out))
    assert tool.storage_used((plan,out))==40
    with pytest.raises(FileExistsError):tool.write_bytes(out,out/'close',b'',closing=True,roots=(plan,out))


def test_storage_symlink_and_existing_source_write_are_denied(tool,tmp_path):
    p=tmp_path/'plan';o=tmp_path/'out';p.mkdir();o.mkdir();(p/'alias').symlink_to(o,target_is_directory=True)
    with pytest.raises(ValueError):tool.storage_used((p,o))
    (p/'alias').unlink();outside=tmp_path/'original';outside.write_bytes(b'original')
    with pytest.raises(ValueError):tool.write_bytes(p,outside,b'changed',roots=(p,o))
    assert outside.read_bytes()==b'original'


def test_closing_size_proof_populates_full1209_not_empty_success(tool):
    assert 100000<tool.closing_size_proof(fake_plan(tool))<=512*1024


def test_closing_seals_visit_all_files_even_after_first_bad(tool,monkeypatch):
    p=fake_plan(tool);visited=[]
    def bad(path):visited.append(str(path));raise ValueError('fixture changed')
    monkeypatch.setattr(tool,'digest',bad)
    result=tool.closing_seals(p,'c'*64,'d'*64)
    assert set(visited)==set(tool.seal_manifest(p,'c'*64,'d'*64))==set(result)


def test_observed_memory_parser_rejects_inherited_ru_as_gate(tool):
    parse=tool.memory_parser()
    value=parse('Pid:\t7\nTgid:\t7\nVmRSS:\t1024 kB\nVmHWM:\t2048 kB\n',7)
    assert value['VmRSS_bytes']==1024**2 and value['VmHWM_bytes']==2*1024**2
    for raw in ('Pid: 7\nTgid: 8\nVmRSS: 1 kB\nVmHWM: 2 kB\n',
        'Pid: 7\nTgid: 7\nVmRSS: 3 kB\nVmHWM: 2 kB\n','Pid: 7\nTgid: 7\nVmRSS: 1 MB\nVmHWM: 2 kB\n'):
        with pytest.raises(ValueError):parse(raw,7)


def test_fresh_V2_seals_both_frozen_V1_sources_and_keeps_original_caps(tool):
    assert tool.SCHEMA.endswith('.v2') and tool.NAME.endswith('_v2')
    assert {p.name:h for p,h in tool.OLD_V1_PINS.items()}=={
        'audit_map_native_fusion_crop_support_20261008_v1.py':'f0f8b404cae1280ed040f6706214f64f9e0dcf0d309b4f8ce3f52b35b8aa9966',
        'test_map_native_fusion_crop_support_20261008_v1.py':'eb344b8934d30b02783170cc5bde28a3665a7f3d74602820a14acb6e98d24adc'}
    assert tool.STEPS==20000 and tool.BATCH==8 and tool.TOTAL==1209
    assert tool.RSS==512*1024**2 and tool.LIMIT==2*1024**2 and tool.DEADLINE==900


def test_json_lines_preserves_actual_raw_row_SHA_not_reencoded_dict_SHA(tool,tmp_path):
    raw=b'{  "index" : 0, "value":5.0 }\n';path=tmp_path/'raw.jsonl';path.write_bytes(raw)
    pairs=list(tool.json_lines(path,with_raw_sha=True));assert pairs==[({'index':0,'value':5.0},hashlib.sha256(raw).hexdigest())]
    assert list(tool.json_lines(path))==[{'index':0,'value':5.0}]
    assert pairs[0][1]!=hashlib.sha256(tool.encoded(pairs[0][0])).hexdigest()


def fake_native(tool,tmp_path,monkeypatch,*,mutate=None,rows_count=None,deny_guard_at=None):
    # HH_261008 - One shared synthetic zero buffer with fake bincount, not1209 decoded real arrays or object-size labels.
    class FakeTarget(bytes):
        def ravel(self):return self
    target=FakeTarget(bytes(tool.PIXELS));target_sha=hashlib.sha256(target).hexdigest()
    histogram=[tool.PIXELS,0,0,0,0,0,0]
    rows=[dict(index=slot,source_group=tool.source_group(slot),split='TRAIN',appearance='FALSE_OR_ORIGINAL',
        full_FOV=True,target_sha256=target_sha,target_class_pixels=list(histogram),sign_target_pixels=0) for slot in range(tool.TOTAL)]
    if mutate is not None:mutate(rows)
    if rows_count is not None:rows=rows[:rows_count]
    folder=tmp_path/'run';(folder/'mining').mkdir(parents=True)
    path=folder/'mining/records.jsonl';path.write_bytes(b''.join(tool.encoded(row) for row in rows))
    monkeypatch.setattr(tool,'RUN',folder)
    class FakeBudget:
        def __init__(self):self.calls=0
        def guard(self):
            self.calls+=1
            if self.calls==deny_guard_at:raise ValueError('synthetic pre-slot resource denial')
    np=SimpleNamespace(bincount=lambda value,minlength:SimpleNamespace(tolist=lambda:list(histogram)))
    targets=([target]*729,[target]*480)
    return np,targets,FakeBudget(),tool.native_progress(),path,rows


def assert_native_denominator(tool,progress):
    failed=int(progress['failed_slot'] is not None)
    assert progress['completed_slots']+failed+progress['UNEXECUTED_slots']==tool.TOTAL
    assert 0<=progress['UNEXECUTED_slots']<=tool.TOTAL
    assert len(progress['sign_pixels_by_slot'])==progress['completed_slots']


def test_fake_full_native1209_progression_has_no_failure_or_missing_objects_claim(tool,tmp_path,monkeypatch):
    np,targets,budget,progress,path,rows=fake_native(tool,tmp_path,monkeypatch)
    signs,pixels,chain=tool.native_support(np,targets,budget,progress)
    assert_native_denominator(tool,progress)
    assert progress['completed_slots']==1209 and progress['UNEXECUTED_slots']==0
    assert progress['status']=='COMPLETE_NATIVE1209_TARGETS_ONLY'
    assert progress['failed_slot'] is progress['failed_row_witness'] is progress['extra_row_witness'] is progress['missing_tail_witness'] is None
    assert signs==[0]*1209 and pixels==[1209*tool.PIXELS,0,0,0,0,0,0] and progress['target_hash_chain_sha256']==chain
    assert 'NOT components or physical objects' in tool.fixed()['source_size_unit']


def test_fake_extra_row_after1209_has_separate_actual_SHA_and_never_negative_UNEX(tool,tmp_path,monkeypatch):
    def extra(rows):
        row=dict(rows[-1]);row['index']=1209;rows.append(row)
    np,targets,budget,progress,path,rows=fake_native(tool,tmp_path,monkeypatch,mutate=extra)
    with pytest.raises(ValueError,match='extra original TRAIN'):tool.native_support(np,targets,budget,progress)
    assert_native_denominator(tool,progress)
    assert progress['completed_slots']==1209 and progress['failed_slot'] is None and progress['UNEXECUTED_slots']==0
    witness=progress['extra_row_witness']
    assert witness['observed_ordinal']==witness['declared_index']==1209
    assert witness['raw_jsonl_row_sha256']==hashlib.sha256(tool.encoded(rows[-1])).hexdigest()
    assert progress['status']=='EXTRA_NATIVE_METADATA_ROW_RETAINED' and budget.calls==1209


@pytest.mark.parametrize('count',[0,1,37,1208])
def test_fake_missing_native_tail_retains_full1209_unexecuted_count(tool,tmp_path,monkeypatch,count):
    np,targets,budget,progress,path,rows=fake_native(tool,tmp_path,monkeypatch,rows_count=count)
    with pytest.raises(ValueError,match='tail missing'):tool.native_support(np,targets,budget,progress)
    assert_native_denominator(tool,progress)
    assert progress['completed_slots']==count and progress['failed_slot'] is None and progress['UNEXECUTED_slots']==1209-count
    assert progress['missing_tail_witness']==dict(first_missing_ordinal=count,UNEXECUTED_slots=1209-count,observed_end_of_jsonl=True)


@pytest.mark.parametrize('ordinal',[0,17,1208])
def test_fake_resource_denial_before_slot_hash_has_failed_witness_and_complete_tail(tool,tmp_path,monkeypatch,ordinal):
    np,targets,budget,progress,path,rows=fake_native(tool,tmp_path,monkeypatch,deny_guard_at=ordinal+1)
    with pytest.raises(ValueError,match='pre-slot resource'):tool.native_support(np,targets,budget,progress)
    assert_native_denominator(tool,progress)
    assert progress['completed_slots']==ordinal and progress['failed_slot']==ordinal and progress['UNEXECUTED_slots']==1208-ordinal
    assert progress['failed_row_witness']['raw_jsonl_row_sha256']==hashlib.sha256(tool.encoded(rows[ordinal])).hexdigest()
    assert 'actual_target_sha256' not in progress['failed_row_witness']


@pytest.mark.parametrize('fault',['non_dict','bool_index','float_index','reordered','wrong_native_hash','wrong_class_support'])
def test_fake_mid_native_schema_type_order_or_hash_failures_are_honest(tool,tmp_path,monkeypatch,fault):
    ordinal=23
    def mutate(rows):
        if fault=='non_dict':rows[ordinal]=['not','a','header']
        elif fault=='bool_index':rows[ordinal]['index']=True
        elif fault=='float_index':rows[ordinal]['index']=23.
        elif fault=='reordered':rows[ordinal]['index']=22
        elif fault=='wrong_native_hash':rows[ordinal]['target_sha256']='a'*64
        else:rows[ordinal]['target_class_pixels']=[tool.PIXELS-1,1,0,0,0,0,0]
    np,targets,budget,progress,path,rows=fake_native(tool,tmp_path,monkeypatch,mutate=mutate)
    with pytest.raises(ValueError):tool.native_support(np,targets,budget,progress)
    assert_native_denominator(tool,progress)
    assert progress['completed_slots']==23 and progress['failed_slot']==23 and progress['UNEXECUTED_slots']==1185
    assert progress['failed_row_witness']['raw_jsonl_row_sha256']==hashlib.sha256(tool.encoded(rows[23])).hexdigest()
    if fault=='wrong_native_hash':assert progress['failed_row_witness']['actual_target_sha256']!=progress['failed_row_witness']['expected_target_sha256']


def test_pessimistic_closing_proof_explicitly_includes_inherited_ru_and_extra_witness(tool):
    tree=ast.parse(TOOL.read_bytes());proof=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='closing_size_proof')
    keys={key.arg for node in ast.walk(proof) if isinstance(node,ast.Call) for key in node.keywords}
    assert {'inherited_ru_maxrss_bytes_NOT_GATE','extra_row_witness','missing_tail_witness','failed_row_witness'}<=keys
    assert tool.closing_size_proof(fake_plan(tool))<=512*1024
