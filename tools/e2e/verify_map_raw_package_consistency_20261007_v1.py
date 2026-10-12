"""HH_261007 - Source-prepared whole350 raw-to-package pixel consistency, not new perception or sensor-time proof.

Only an externally sealed ROOT plan/grant can open one-frame RGB/semantic pixels.
No TRAIN/TEST, depth/pose arrays, model imports, NN, ROS, GPU or raw-image writes.
Observed VmHWM is fail-closed protection, not OS-enforced memory containment.
The 900s limit is sampled/nonpreemptive; closing seals still run and retain an overrun.
"""
from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager, ExitStack
import fcntl
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import resource
import stat
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
TEST = ROOT / 'learning/tests/test_verify_map_raw_package_consistency_20261007_v1.py'
NAME = 'map_raw_package_consistency_v1'
SCHEMA = 'portable_e2e.map_raw_package_consistency.20261007.v1'
PLAN_DIR = ROOT / 'storage/experiments/plans/2026-10-07' / NAME
PLAN, AUTHORITY = PLAN_DIR / 'ROOT_plan.json', PLAN_DIR / 'ROOT_authority.json'
OUTPUT = ROOT / 'storage/experiments/diagnostics/2026-10-07' / NAME
LOCK = ROOT / 'storage/assets/locks/autoware_e2e_runtime.lock'
RUN = ROOT / 'storage/experiments/training/2026-10-02/map_native_fusion_pair_v3'
DATA = ROOT / 'storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1'
PARENT = ROOT / 'storage/datasets/prepared/map_expanded_20260926_ai_v2/manifest.json'
DEV_RAW = ROOT / 'storage/experiments/validation/2026-09-22/mapping_sensor_probe/moving_turn_clear_v2/mapping'
HOLD_RAW = ROOT / 'storage/experiments/training/2026-09-27/map_town02_p02_sign_train_scout_epic_v1/mapping'
QA = ROOT / 'docs/assets/validation/2026-09-27/map_town02_p02_train_scout/summary.json'
SEMANTIC = ROOT / 'tools/e2e/evaluate_ctrack_fixed_maps_20261001.py'
MEMORY = ROOT / 'tools/e2e/diagnose_map_sign_components_20261007_v2.py'
LABELS = ('other', 'road', 'sidewalk', 'lane_marking', 'traffic_light_fixture', 'traffic_sign', 'pole')
COHORTS = (('dev_town01', 118), ('holdout_town02_p02', 232))
FLOOR, RESERVE, LIMIT, RSS, DEADLINE = 2*1024**3, 64*1024**2, 2*1024**2, 512*1024**2, 900
META_LIMIT, CLOSING, HEADER_LIMIT = 512*1024, 512*1024, 4096
RAW_SEMANTIC_HEADER_PIN = 'fd9416df153cfa09e50e92701d4fe1d50039b44b247aeeb85f685ad5b2422a97'
# HH_261007 - Header-only first-frame samples of both declared raw cohorts showed these three DEFLATED members and this128B semantic header; not an all350 pixel proof.
ENV = {'CUDA_VISIBLE_DEVICES':'', 'PYTHONNOUSERSITE':'1', 'PYTHONDONTWRITEBYTECODE':'1',
       'OMP_NUM_THREADS':'1', 'MKL_NUM_THREADS':'1', 'OPENBLAS_NUM_THREADS':'1'}
META_PINS = {
    RUN/'authorization.json':'2b5a000cb2e3f94ced7cbbeb1e64fc07a1fe71b5e11c0d72b11f1f51ffe89ef8',
    RUN/'inputs/manifest.json':'614840471e938a0e9ecd169368d6240e96c01fdf76c74b56f82863c9d49d2b1b',
    RUN/'inputs/study_train_registry.json':'ca892a6b6e34602b2c983fdec83972a3693bfd763baafb4abfac2f1a488bad8d',
    RUN/'known_evaluation.json':'89f5b512e048c8af004ff73b5f1e8836af8cce3c18fdf41a5f1f995caa940a10',
    DATA/'manifest.json':'f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834',
    PARENT:'e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c',
    QA:'d2dd43837eeef50c9ab71106b6e67a926802fada111d735de2b859c51d1a9d49',
    DEV_RAW/'records.json':'f47deeba0bdf2aa717fe3939f0bc8087548da2d7931ff71bfd45c2191c294236',
    HOLD_RAW/'records.json':'2196f54897ecce0ca2cf1326927019231316df0f363e3362d82e42a9d85df375',
}
SOURCE_PINS = {SEMANTIC:'fa210fbce9aefa99f4ffc13efbeae42e2a4fe9c7281d144e71f298a44700ed38',
               MEMORY:'cb72db2bc043166259db66a3276d8e3edfa51ce390dc0b93428ac61316198ef8'}
PACKAGE_PINS = {'dev_town01':'417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3',
                'holdout_town02_p02':'858f92ccd874c154576d3cf6a85132508801a22671baf6d0c07abe3936eb16cb'}
HEADER_PINS = {
    ('dev_town01','rgb.npy'):'ef72e39b2bdde6cd318d42b06c289f8a5568b202bdfef7b7fa79c56b5763534a',
    ('dev_town01','target.npy'):'47b37f62afbb7c8be94c464262a88e107c7329a01fffff660de04ef3d02bd506',
    ('holdout_town02_p02','rgb.npy'):'f27c93e41edcdb3e9ebd34e002034ee559d22ddd229b09a26aa5f56bd24f9542',
    ('holdout_town02_p02','target.npy'):'4246be3c8a9515e2daaa4c26706ad11d040814077fa7c91746ee890abba99b6b'}
PREDICTION_PINS = {
    ('dev_town01','parent_diagnostic'):'c335375371ef30b07fb5042fbfbed902bb0758c0d15b307ccf683fdc3d79472a',
    ('dev_town01','control'):'f05096b04655762543f82e5ed7390de917beaa88f4c100ba83bc39aed3c50988',
    ('dev_town01','candidate'):'0f9f6a3f2db8aea63b5778e2c2e57444dd141c75149ebe5c626cd1310ebf94f1',
    ('holdout_town02_p02','parent_diagnostic'):'8d8f47c4ec709acd19f0f49d0ecd214b068725125d34f5e8f8309597f1ad3b4b',
    ('holdout_town02_p02','control'):'e31b9f4ec787870a855b6b2ca285bfbaa6c04ac431f0691331ab620bf17929fb',
    ('holdout_town02_p02','candidate'):'ff55003d3f71aa8c83e62cfaff1673a028549b49170d12d9e68cd415f3973ee5'}


def require(value, message):
    if not value: raise ValueError(message)


def canonical(path):
    path=Path(path); require(path==path.resolve(strict=True) and path.is_file(), 'canonical regular source/input required')
    return path


def fingerprint(path):
    s=canonical(path).stat(); return s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns


def digest(path, guard=lambda:None):
    path=canonical(path); before=fingerprint(path); h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024**2),b''):guard();h.update(chunk)
    require(before==fingerprint(path),'input changed during SHA read');return h.hexdigest()


def read_json(path, pin=None):
    path=canonical(path);require(path.stat().st_size<=META_LIMIT,'metadata bound exceeded')
    raw=path.read_bytes();require(pin is None or hashlib.sha256(raw).hexdigest()==pin,'metadata SHA differs')
    def pairs(items):
        value={}
        for k,v in items:require(k not in value,'duplicate JSON field');value[k]=v
        return value
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=lambda x:require(False,'nonfinite JSON'))


def encoded(value):
    return (json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()


def storage_used(storage_roots=None):
    # HH_261007 - One complete file-byte budget covers BOTH fixed plan and actual roots, including ROOT-written files.
    roots=tuple(map(Path,storage_roots if storage_roots is not None else (PLAN_DIR,OUTPUT)))
    require(len(roots)==2 and roots[0]!=roots[1] and not any(a.is_relative_to(b) for a,b in (roots,roots[::-1])),'two disjoint storage roots required')
    used=0
    for root in roots:
        require(root.is_absolute() and root==root.resolve(strict=False),'storage root alias denied')
        if not root.exists():continue
        require(root.is_dir(),'storage root must be a directory')
        for folder,dirs,files in os.walk(root,followlinks=False):
            for name in dirs+files:
                path=Path(folder)/name;mode=path.lstat().st_mode
                require(path==path.resolve(strict=True) and (stat.S_ISDIR(mode) or stat.S_ISREG(mode)),'storage alias/type denied')
                if stat.S_ISREG(mode):used+=path.stat().st_size
    return used


def write_bytes(root,path,payload,*,closing=False,storage_roots=None):
    root=Path(root);path=Path(path)
    roots=tuple(map(Path,storage_roots if storage_roots is not None else (PLAN_DIR,OUTPUT)))
    require(root in roots and root==root.resolve(strict=True) and path.parent==path.parent.resolve(strict=True) and path.is_relative_to(root) and len(payload)<=META_LIMIT,'bounded own metadata write only')
    require(storage_used(roots)+len(payload)<=LIMIT-(0 if closing else CLOSING),'combined plan/output quota exceeded')
    with path.open('xb') as stream:stream.write(payload)


def write_new(root,path,value,*,closing=False,storage_roots=None):
    write_bytes(root,path,encoded(value),closing=closing,storage_roots=storage_roots)


def bounded_error(error):
    raw=repr(error).encode('utf8',errors='backslashreplace')
    text=raw.decode()
    return text if len(encoded(text))<=128 else dict(prefix_hex=raw[:32].hex(),raw_bytes=len(raw),raw_SHA256=hashlib.sha256(raw).hexdigest(),truncated=True)


def seal_exception(error):
    raw=repr(error).encode('utf8',errors='backslashreplace')
    return [hashlib.sha256(raw).hexdigest(),len(raw),raw[:16].hex()]


def private_functions(path,names,namespace,literals=()):
    # HH_261007 - Genuine frozen scalar/semantic AST only; never import a trainer/ROS/model module.
    raw=canonical(path).read_bytes();require(hashlib.sha256(raw).hexdigest()==SOURCE_PINS[path],'original private source SHA differs')
    tree=ast.parse(raw);nodes=[x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name in names]
    require(len(nodes)==len(names) and {x.name for x in nodes}==set(names),'unique exact frozen function seams required')
    result=dict(namespace,__file__=str(path),__name__='_hh_map_pixel_private')
    for name in literals:
        selected=[x for x in tree.body if isinstance(x,ast.Assign) and isinstance(x.targets[0],ast.Name) and x.targets[0].id==name]
        require(len(selected)==1,'exact original literal seam required');result[name]=ast.literal_eval(selected[0].value)
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),result);return result


def memory_read(parser=None):
    parser=parser or private_functions(MEMORY,('require','parse_process_memory'),dict(re=re))['parse_process_memory']
    with Path('/proc/self/status').open('r',encoding='ascii') as stream:text=stream.read(16385)
    value=parser(text,os.getpid());value['inherited_ru_maxrss_bytes_not_gate']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024
    require(all(type(v) is int and 0<v<=2**64-1 for v in value.values()),'bounded Linux self-memory integer required')
    return value


class Budget:
    def __init__(self):
        self.started=time.monotonic();self.last_memory=None
        self.parser=private_functions(MEMORY,('require','parse_process_memory'),dict(re=re))['parse_process_memory']
    def guard(self):
        self.last_memory=memory_read(self.parser)
        require(time.monotonic()-self.started<DEADLINE,'sampled whole900s time limit exceeded')
        require(self.last_memory['VmHWM_bytes']<=RSS,'observed self512MiB VmHWM exceeded')
        s=os.statvfs(OUTPUT.parent);require(s.f_bavail*s.f_frsize>=FLOOR+RESERVE,'2GiB floor plus64MiB reserve unavailable')


@contextmanager
def lease():
    with canonical(LOCK).open('r') as stream:
        fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:yield fingerprint(LOCK)[:2]
        finally:fcntl.flock(stream,fcntl.LOCK_UN)


def environment():
    require(sys.flags.optimize==0 and sys.flags.dont_write_bytecode and all(os.environ.get(k)==v for k,v in ENV.items()),'CPU-only clean single-thread environment required')
    require(3 in os.sched_getaffinity(0),'declared logical CPU3 unavailable')
    modules={name:importlib.util.find_spec(name) for name in ('numpy','PIL')}
    require(all(s is not None and s.origin for s in modules.values()),'existing numeric/image providers absent')
    origins={k:str(Path(v.origin).resolve(strict=True)) for k,v in modules.items()}
    selected={origins['numpy'],origins['PIL'],str(Path(origins['PIL']).parent/'Image.py')}
    return dict(executable=str(Path(sys.executable).resolve(strict=True)),versions={k:importlib.metadata.version(k) for k in ('numpy','Pillow')},
        origins=origins,selected_provider_python_SHA={p:digest(p) for p in sorted(selected)},declared_affinity=[3],
        transitive_package_native_binary_closure_claimed=False)


def sources():
    auth=read_json(RUN/'authorization.json',META_PINS[RUN/'authorization.json']);require(len(auth['source_sha256'])==15,'all15 original sources required')
    table=dict(SOURCE_PINS)
    for name,pin in auth['source_sha256'].items():
        current=ROOT/'tools/e2e'/name
        if not current.is_file():current=ROOT/'learning/portable_e2e'/name
        table[current]=pin;table[RUN/'source'/name]=pin
    for p in (HERE,TEST,ROOT/'tools/workspace_python.py',ROOT/'tools/canonical_workspace.py'):table[p]=digest(p)
    require(all(digest(p)==h for p,h in table.items()),'original current/archive source closure changed')
    return {str(p):h for p,h in table.items()}


def catalog():
    docs={p:read_json(p,h) for p,h in META_PINS.items()};parent=docs[PARENT];data=docs[DATA/'manifest.json'];qa=docs[QA]
    require(parent['labels']==data['labels']==docs[RUN/'inputs/manifest.json']['labels']==list(LABELS),'exact7class order required')
    require(docs[RUN/'inputs/study_train_registry.json']['ctrack_train_eligible'] is False and docs[RUN/'authorization.json']['DEV_or_HOLDOUT_used_for_fit'] is False,'TRAIN/heldout boundary differs')
    rows=[];dev=parent['splits']['dev'];original={x['stem']:x for x in docs[DEV_RAW/'records.json']}
    require(len(dev['source_records'])==dev['count']==118 and len(original)==240,'wholeDEV metadata count')
    for n,r in enumerate(dev['source_records']):
        require(r==original[r['stem']],'nativeDEV record differs')
        pair={suffix:str((DEV_RAW/(r['stem']+suffix)).relative_to(ROOT)) for suffix in ('.png','.npz')}
        require(all(p in dev['source_hashes'] for p in pair.values()),'nativeDEV pair absent')
        rows.append(dict(cohort='dev_town01',ordinal=n,native_frame=r['frame'],timestamp_s=r['timestamp_s'],stem=r['stem'],
            RGB_path=str(DEV_RAW/(r['stem']+'.png')),RGB_sha256=dev['source_hashes'][pair['.png']],
            semantic_path=str(DEV_RAW/(r['stem']+'.npz')),semantic_sha256=dev['source_hashes'][pair['.npz']]))
    kept=[r for r in qa['frames'] if r['retained_0p25m']];pairs=data['provenance']['p02_holdout_raw_pairs'];original={r['stem']:r for r in docs[HOLD_RAW/'records.json']}
    require([r['stem'] for r in kept]==qa['retained_stems'] and len(kept)==len(pairs)==232 and len(original)==240,'whole heldout native order')
    for n,(r,p) in enumerate(zip(kept,pairs)):
        require(p==dict(stem=r['stem'],rgb_sha256=r['rgb_sha256'],semantic_npz_sha256=r['semantic_npz_sha256']) and original[r['stem']]['frame']==r['frame'] and original[r['stem']]['timestamp_s']==r['timestamp_s'],'native heldout identity differs')
        rows.append(dict(cohort='holdout_town02_p02',ordinal=n,native_frame=r['frame'],timestamp_s=r['timestamp_s'],stem=r['stem'],RGB_path=str(HOLD_RAW/(r['stem']+'.png')),RGB_sha256=p['rgb_sha256'],semantic_path=str(HOLD_RAW/(r['stem']+'.npz')),semantic_sha256=p['semantic_npz_sha256']))
    for c,n in COHORTS:
        selected=[r for r in rows if r['cohort']==c];require([r['ordinal'] for r in selected]==list(range(n)) and len({r['native_frame'] for r in selected})==n,'all350 ordinal/frame denominator required')
        require(all(a['native_frame']<b['native_frame'] and a['timestamp_s']<b['timestamp_s'] for a,b in zip(selected,selected[1:])),'causal native metadata differs')
        require(data['files'][c]['count']==n and data['files'][c]['sha256']==PACKAGE_PINS[c] and sum(data['files'][c]['class_pixels'])==n*360*640,'fixed target support/shape boundary')
        values=docs[RUN/'known_evaluation.json'][c];require(set(values)=={'parent','control','candidate'},'all3 stored references required')
        for arm,v in values.items():
            directory='parent_diagnostic' if arm=='parent' else arm
            require(v['count']==v['evaluated_frames']==n and v['full_FOV'] is True and v['source_sha256']==PACKAGE_PINS[c]
                and v['prediction_file_sha256']==PREDICTION_PINS[c,directory] and v['target_pixels']==dict(zip(LABELS,data['files'][c]['class_pixels'])),'all6 original prediction metadata links')
    require(len(rows)==350,'whole350 required');return rows


def input_pins(rows):
    pins={str(p):h for p,h in META_PINS.items()}
    pins.update({str(DATA/(c+'.npz')):h for c,h in PACKAGE_PINS.items()})
    pins.update({str(RUN/arm/(c+'_predictions.npz')):h for (c,arm),h in PREDICTION_PINS.items()})
    for r in rows:
        for kind in ('RGB','semantic'):pins[r[kind+'_path']]=r[kind+'_sha256']
    return pins


def npy_header(stream,shape):
    prefix=stream.read(8);require(prefix==b'\x93NUMPY\x01\x00','exact NPYv1 header required')
    size=stream.read(2);require(len(size)==2,'truncated header length');length=int.from_bytes(size,'little')
    require(0<length<=HEADER_LIMIT,'header bound');raw=stream.read(length);require(len(raw)==length,'truncated header')
    node=ast.parse(raw.decode('latin1'),mode='eval').body
    require(isinstance(node,ast.Dict) and len(node.keys)==3 and all(isinstance(k,ast.Constant) and type(k.value) is str for k in node.keys) and len({k.value for k in node.keys})==3,'duplicate/malformed NPY header field')
    value=ast.literal_eval(node);require(type(value) is dict and set(value)=={'descr','fortran_order','shape'} and value['descr']=='|u1' and value['fortran_order'] is False and type(value['shape']) is tuple and all(type(n) is int for n in value['shape']) and value['shape']==tuple(shape),'exact C-order uint8 shape required')
    return prefix+size+raw


def zip_census(archive,names):
    actual=archive.namelist();require(len(actual)==len(set(actual))==len(names) and set(actual)==set(names),'duplicate/missing/extra ZIP members denied')
    require(all(not r.flag_bits&1 and r.compress_type==zipfile.ZIP_DEFLATED for r in archive.infolist()),'only sealed unencrypted deflated ZIP required')


def entry_stream(stack,archive,name,shape,header_pin=None):
    expected=math.prod(shape);entry=archive.getinfo(name)
    require(expected+10<entry.file_size<=expected+HEADER_LIMIT+10,'entry declared body bound')
    stream=stack.enter_context(archive.open(name));header=npy_header(stream,shape)
    require(entry.file_size==len(header)+expected,'no body padding/truncation')
    require(header_pin is None or len(header)==128 and hashlib.sha256(header).hexdigest()==header_pin,'fixed packaged128B header differs')
    return stream


def read_exact(stream,count):
    require(type(count) is int and 0<count<=360*640*4,'one uint8 frame maximum required')
    value=stream.read(count);require(len(value)==count,'truncated frame body');return value


def finish_stream(stream):
    require(stream.read(1)==b'','extra body byte denied')  # HH_261007 - Reading EOF also completes ZipExtFile CRC verification.


def fixed():
    return dict(schema=SCHEMA,cohorts=[[c,n] for c,n in COHORTS],expected=350,logical_CPU_affinity=[3],threads=1,
        VmHWM_cap_bytes=RSS,OS_memory_containment_claimed=False,floor_bytes=FLOOR,reserve_bytes=RESERVE,output_cap_bytes=LIMIT,
        closing_bytes=CLOSING,combined_PLAN_and_OUTPUT_file_byte_budget=True,deadline_seconds=DEADLINE,deadline_enforcement='sampled_nonpreemptive_including_closing',RGB_frame_bytes=691200,target_frame_bytes=230400,semantic_frame_bytes=921600,
        no_full_array_load=True,no_float_RGB_conversion=True,exact_pixel_equality=True,threshold_relaxation=False,
        no_TRAIN_TEST_pixels=True,no_depth_pose_arrays=True,predictions_only_metadata_full_SHA=True,noNN=True,noROS=True,noGPU=True,
        sensor_header_identity_independently_verified=False,object_label_truth_proven=False,model_improvement=False,main_approved=0,sub_approved=0)


def verify_tables(plan,budget):
    require(plan['fixed']==fixed() and plan['source_sha256']==sources() and plan['catalog']==catalog() and plan['input_sha256']==input_pins(plan['catalog']),'exact source/scalar/input closure differs')
    for p,h in plan['input_sha256'].items():require(digest(p,budget.guard)==h,'complete scientific input SHA differs')
    for p,h in plan['source_sha256'].items():require(digest(PLAN_DIR/'sources'/Path(p).relative_to(ROOT))==h,'archive/current source differs')
    require(plan['environment']==environment(),'existing package/executable/environment changed')


def seal_manifest(plan,plan_sha,grant_sha):
    values={**plan['source_sha256'],**plan['input_sha256'],str(PLAN):plan_sha,str(AUTHORITY):grant_sha}
    values.update({str(PLAN_DIR/'sources'/Path(p).relative_to(ROOT)):h for p,h in plan['source_sha256'].items()})
    return values


def closing_seals(plan,plan_sha,grant_sha):
    # HH_261007 - Never short-circuit the complete closing input/source census after a first mismatch.
    values=seal_manifest(plan,plan_sha,grant_sha)
    errors={}
    for p,h in values.items():
        try:
            actual=digest(p)
            if actual!=h:errors[p]=dict(expected=h,actual=actual)
        except BaseException as error:errors[p]=dict(expected=h,actual=None,error=seal_exception(error))
    return errors


def frame_ledger(catalog_rows):
    require(len(catalog_rows)==350,'whole350 ledger required')
    require([(r['cohort'],r['ordinal']) for r in catalog_rows]==[(c,n) for c,count in COHORTS for n in range(count)]
        and all(type(r['ordinal']) is type(r['native_frame']) is int and r['native_frame']>0 and type(r['timestamp_s']) in (int,float) and math.isfinite(r['timestamp_s']) for r in catalog_rows),'ledger may not filter/reorder/pad anchors')
    return [dict(cohort=r['cohort'],ordinal=r['ordinal'],native_frame=r['native_frame'],timestamp_s=r['timestamp_s'],status='UNEXECUTED',metrics=None,error=None) for r in catalog_rows]


def retained_failure(rows,current,error,stage,memory):
    value=dict(stage=stage,error=bounded_error(error),current_frame=current,resources=memory)
    if current is not None:
        require(type(current) is int and 0<=current<len(rows) and rows[current]['status']=='FAILED','failure must bind an actually started frame')
        rows[current]['error']=bounded_error(error);rows[current]['resources']=memory
    return value


def final_result(plan,rows,failure,seal_errors,plan_sha,grant_sha,source_after,input_after,memory,elapsed):
    manifest=seal_manifest(plan,plan_sha,grant_sha);indices={p:n for n,p in enumerate(manifest)}
    errors=[[indices[p],e['actual'],e.get('error')] for p,e in seal_errors.items()]
    passed=sum(r['status']=='PASS' for r in rows);failed=sum(r['status']=='FAILED' for r in rows)
    return dict(schema=SCHEMA+'.result',status='COMPLETE_PIXEL_CONSISTENCY_ONLY' if failure is None and passed==350 and source_after and input_after else 'FAILED_WITH_FULL_DENOMINATOR',
        expected_frames=350,completed_frames=passed,failed_frames=failed,UNEXECUTED=350-passed-failed,rows=rows,failure_tail=failure,
        ROOT_plan_sha256=plan_sha,ROOT_authority_sha256=grant_sha,full_source_input_maps_in_exact_archived_ROOT_plan=str(OUTPUT/PLAN.name),
        source_count=len(plan['source_sha256']),input_count=len(plan['input_sha256']),closing_seal_manifest_order='source_then_input_then_ROOT_plan_then_ROOT_authority_then_source_archives; insertion order from archived ROOT_plan',
        source_after_unchanged=source_after,input_after_unchanged=input_after,observed_closing_memory=memory,elapsed_seconds=elapsed,
        closing_seal_errors=errors,closing_seal_error_format=['manifest_index','actual_SHA256_or_null','exception_[repr_SHA256,raw_bytes,prefix_hex16B]_or_null'],ROOT_grants_and_archived_sources_after_unchanged=not seal_errors,
        NN=0,training_steps=0,ROS=False,GPU=False,TRAIN_TEST_pixels_read=False,depth_pose_arrays_read=False,predictions_decoded=False,
        original_pixel_criteria_unchanged=True,object_truth_or_independent_sensor_header_alignment_proven=False,model_improved=False,main_approved=0,sub_approved=0,
        resource_bounds_sampled_not_OS_containment=True,partial_package_CRC_verified_only_when_EOF_reached=True)


def closing_size_proof(plan):
    # HH_261007 - Serialize a pessimistic FULL350 populated ledger and ALL-seals error envelope, not a best-case empty ledger.
    rows=frame_ledger(plan['catalog']);big=2**64-1;memory=dict(PID=big,VmRSS_bytes=big,VmHWM_bytes=big,inherited_ru_maxrss_bytes_not_gate=big)
    error=dict(prefix_hex='f'*64,raw_bytes=big,raw_SHA256='f'*64,truncated=True)
    metrics=dict(RGB_mismatched_bytes=691200,target_mismatched_pixels=230400,packaged_RGB_sha256='f'*64,raw_RGB_pixels_sha256='f'*64,
        packaged_target_sha256='f'*64,reconstructed_target_sha256='f'*64,target_class_pixels=[230400]*7,packaged_target_max_label=255)
    for row in rows:row.update(metrics=dict(metrics),resources=dict(memory),elapsed_seconds=-1.7976931348623157e308)
    rows[-1].update(status='FAILED',error=error);rows[-1]['metrics']['target_class_pixels']=[230400]*256
    failure=dict(stage='x'*64,resources=dict(memory,error=error),deadline_seconds=DEADLINE,previous_failure=dict(stage='x'*64,error_count=big,
        previous_failure=dict(stage='x'*64,error=error,current_frame=349,resources=memory)))
    errors={p:dict(actual=None,error=['f'*64,big,'f'*32]) for p in seal_manifest(plan,'f'*64,'f'*64)}
    size=len(encoded(final_result(plan,rows,failure,errors,'f'*64,'f'*64,False,False,dict(memory,error=error),-1.7976931348623157e308)))
    require(size<=META_LIMIT,'prospective full350 maximum closing JSON exceeds512KiB');return size


def authority_value(plan,plan_sha):
    return dict(schema=SCHEMA+'.ROOT_authority',ROOT_plan_path=str(PLAN),ROOT_plan_sha256=plan_sha,source_sha256=plan['source_sha256'],
        one_frame_known350_pixel_diagnostic_authorized=True,training_authorized=False,TEST_authorized=False,NN_authorized=False,ROS_authorized=False,GPU_authorized=False,model_promoted=False)


def storage_protocol(plan):
    base={k:plan[k] for k in ('schema','fixed','catalog','source_sha256','input_sha256','environment','lock_identity')}
    caps=dict(source_archive_bytes=sum(canonical(p).stat().st_size for p in plan['source_sha256']),
        ROOT_plan_copy_max_bytes=len(encoded(base))+64*1024,ROOT_authority_copy_max_bytes=len(encoded(authority_value(plan,'f'*64)))+4096,
        closing_complete_envelope_max_bytes=closing_size_proof(plan),closing_reserved_bytes=CLOSING)
    require(max(caps['ROOT_plan_copy_max_bytes'],caps['ROOT_authority_copy_max_bytes'])<=META_LIMIT,'prospective ROOT metadata copy cap exceeds512KiB')
    return caps


def storage_admission(plan,*,prospective=False):
    caps=storage_protocol(plan);require(plan['storage_protocol']==caps,'prospective storage/source-size proof changed')
    require(len(encoded(plan))<=caps['ROOT_plan_copy_max_bytes'],'complete candidate exceeds prospective ROOT plan copy cap')
    if prospective:
        required=caps['source_archive_bytes']+len(encoded(plan))+2*caps['ROOT_plan_copy_max_bytes']+2*caps['ROOT_authority_copy_max_bytes']+CLOSING
    else:
        require(PLAN.stat().st_size<=caps['ROOT_plan_copy_max_bytes'] and AUTHORITY.stat().st_size<=caps['ROOT_authority_copy_max_bytes'],'ROOT raw metadata exceeds declared copy cap')
        required=PLAN.stat().st_size+AUTHORITY.stat().st_size+CLOSING
    require(storage_used()+required<=LIMIT,'combined candidate/archives/ROOT copies/full closing reserve cannot fit2MiB')
    return required


def prepare():
    require(not PLAN_DIR.exists() and not OUTPUT.exists(),'fresh namespace only; no retry/overwrite')
    with lease() as lock_id:
        budget=Budget();budget.guard();rows=catalog();src=sources();pins=input_pins(rows);env=environment()
        for p,h in pins.items():require(digest(p,budget.guard)==h,'all prospective inputs sealed before candidate')
        plan=dict(schema=SCHEMA,fixed=fixed(),catalog=rows,source_sha256=src,input_sha256=pins,environment=env,lock_identity=list(lock_id))
        plan['storage_protocol']=storage_protocol(plan);storage_admission(plan,prospective=True)
        PLAN_DIR.mkdir(parents=True,exist_ok=False)
        for p,h in src.items():
            destination=PLAN_DIR/'sources'/Path(p).relative_to(ROOT);destination.parent.mkdir(parents=True,exist_ok=True)
            raw=canonical(p).read_bytes();require(hashlib.sha256(raw).hexdigest()==h,'source changed before archive')
            write_bytes(PLAN_DIR,destination,raw)
        write_new(PLAN_DIR,PLAN_DIR/'candidate_plan.json',plan)
        return dict(status='CANDIDATE_ONLY_NOT_ROOT_AUTHORITY',path=str(PLAN_DIR/'candidate_plan.json'),sha256=digest(PLAN_DIR/'candidate_plan.json'),NN=0,pixels_decoded=0)


def authorization(plan_sha,grant_sha):
    require(type(plan_sha) is type(grant_sha) is str and all(re.fullmatch('[0-9a-f]{64}',x) for x in (plan_sha,grant_sha)),'external raw SHA arguments required')
    plan=read_json(PLAN,plan_sha);grant=read_json(AUTHORITY,grant_sha)
    expected=authority_value(plan,plan_sha)
    require(grant==expected and plan['schema']==SCHEMA and plan['fixed']==fixed() and len(plan['catalog'])==350,'exact fixed ROOT scope required; no selfapproval bool')
    return plan


def execute(plan_sha,grant_sha):
    plan=authorization(plan_sha,grant_sha);require(not OUTPUT.exists(),'fresh actual output only; no retry')
    with lease() as lock_id:
        require(list(lock_id)==plan['lock_identity'],'runtime lock object changed');budget=Budget()
        storage_admission(plan)  # HH_261007 - Exact current parent files + both future raw ROOT copies +512KiB closing fit BEFORE output/import/pixels.
        OUTPUT.mkdir(parents=True,exist_ok=False)
        rows=frame_ledger(plan['catalog'])
        failure=None;stage='admission';current=None;source_after=False;input_after=False
        try:
            for path,pin in ((PLAN,plan_sha),(AUTHORITY,grant_sha)):
                payload=canonical(path).read_bytes();require(hashlib.sha256(payload).hexdigest()==pin,'exact ROOT archive bytes changed')
                write_bytes(OUTPUT,OUTPUT/path.name,payload)
            budget.guard();verify_tables(plan,budget);os.sched_setaffinity(0,{3});require(os.sched_getaffinity(0)=={3},'actual single CPU3 affinity differs')
            # HH_261007 - Numeric/image imports and genuine target math are reachable only after external authority+full seals+lease/budgets.
            import numpy as np
            from PIL import Image
            require(str(Path(np.__file__).resolve(strict=True))==plan['environment']['origins']['numpy'] and str(Path(sys.modules['PIL'].__file__).resolve(strict=True))==plan['environment']['origins']['PIL'],'actual numeric/image import origin differs')
            targets=private_functions(SEMANTIC,('require','targets'),dict(np=np),('LABELS','RAW_LABELS'))['targets'];offset=0
            for cohort,count in COHORTS:
                stage='package_stream';path=DATA/(cohort+'.npz');before=fingerprint(path)
                with ExitStack() as stack:
                    package=stack.enter_context(zipfile.ZipFile(path));zip_census(package,('rgb.npy','target.npy'))
                    rgb_stream=entry_stream(stack,package,'rgb.npy',(count,360,640,3),HEADER_PINS[cohort,'rgb.npy'])
                    target_stream=entry_stream(stack,package,'target.npy',(count,360,640),HEADER_PINS[cohort,'target.npy'])
                    for n in range(count):
                        current=offset+n;row=rows[current];item=plan['catalog'][current];row['status']='FAILED';began=time.monotonic();stage='frame_start';budget.guard()
                        require(item['cohort']==cohort and item['ordinal']==n,'no reordered/padded frame')
                        for kind in ('RGB','semantic'):require(digest(item[kind+'_path'],budget.guard)==item[kind+'_sha256'],'fresh exact raw frame SHA differs')
                        stage='package_frame';rgb_bytes=read_exact(rgb_stream,691200);target_bytes=read_exact(target_stream,230400)
                        stage='raw_RGB';raw_rgb=Path(item['RGB_path']);before_rgb=fingerprint(raw_rgb)
                        with Image.open(raw_rgb) as image:
                            require(image.mode=='RGB' and image.size==(640,360),'native RGB mode/size differs');actual_rgb=np.asarray(image,dtype=np.uint8)
                            require(actual_rgb.shape==(360,640,3),'one native RGB shape');actual_rgb_bytes=actual_rgb.tobytes()
                        require(before_rgb==fingerprint(raw_rgb),'RGB changed during decode')
                        stage='raw_semantic';raw_semantic=Path(item['semantic_path']);before_semantic=fingerprint(raw_semantic)
                        with ExitStack() as sensor_stack:
                            sensor=sensor_stack.enter_context(zipfile.ZipFile(raw_semantic));zip_census(sensor,('depth.npy','semantic.npy','world_from_camera.npy'))
                            semantic_stream=entry_stream(sensor_stack,sensor,'semantic.npy',(360,640,4),RAW_SEMANTIC_HEADER_PIN);semantic_bytes=read_exact(semantic_stream,921600);finish_stream(semantic_stream)
                        require(before_semantic==fingerprint(raw_semantic),'semantic archive changed during decode')
                        actual_target=targets(np.frombuffer(semantic_bytes,dtype=np.uint8).reshape(360,640,4));actual_target_bytes=actual_target.tobytes()
                        packaged_target=np.frombuffer(target_bytes,dtype=np.uint8).reshape(360,640)
                        row['metrics']=dict(RGB_mismatched_bytes=int(np.count_nonzero(np.frombuffer(rgb_bytes,dtype=np.uint8).reshape(360,640,3)!=actual_rgb)),target_mismatched_pixels=int(np.count_nonzero(packaged_target!=actual_target)),
                            packaged_RGB_sha256=hashlib.sha256(rgb_bytes).hexdigest(),raw_RGB_pixels_sha256=hashlib.sha256(actual_rgb_bytes).hexdigest(),
                            packaged_target_sha256=hashlib.sha256(target_bytes).hexdigest(),reconstructed_target_sha256=hashlib.sha256(actual_target_bytes).hexdigest(),
                            target_class_pixels=np.bincount(packaged_target.ravel(),minlength=7).tolist(),packaged_target_max_label=int(packaged_target.max()))
                        require(row['metrics']['packaged_target_max_label']<7,'packaged class outside frozen7labels')
                        stage='post_frame';budget.guard()
                        for kind in ('RGB','semantic'):require(digest(item[kind+'_path'],budget.guard)==item[kind+'_sha256'],'raw frame changed after comparison')
                        row['resources']=budget.last_memory;row['elapsed_seconds']=time.monotonic()-began
                        require(row['metrics']['RGB_mismatched_bytes']==row['metrics']['target_mismatched_pixels']==0,'strict raw/package pixel equality failed')
                        row['status']='PASS';current=None
                        del rgb_bytes,target_bytes,actual_rgb,actual_rgb_bytes,semantic_bytes,actual_target,actual_target_bytes,packaged_target
                    finish_stream(rgb_stream);finish_stream(target_stream)
                require(before==fingerprint(path),'package changed during stream');offset+=count
            stage='closing';verify_tables(plan,budget);source_after=input_after=True
        except BaseException as error:
            failure=retained_failure(rows,current,error,stage,budget.last_memory)
        finally:
            # HH_261007 - Retain existing raw metrics and resource failure tails, even when the same RSS/deadline guard now denies more work.
            seal_errors=closing_seals(plan,plan_sha,grant_sha)
            source_after=not any(p in seal_errors for p in plan['source_sha256']);input_after=not any(p in seal_errors for p in plan['input_sha256'])
            if seal_errors:failure=dict(stage='closing_seals',error_count=len(seal_errors),previous_failure=failure)
            try:closing_memory=memory_read(budget.parser)
            except BaseException as error:closing_memory=dict(error=bounded_error(error))
            if 'error' in closing_memory or closing_memory['VmHWM_bytes']>RSS or time.monotonic()-budget.started>=DEADLINE:
                failure=dict(stage='closing_resource',resources=closing_memory,deadline_seconds=DEADLINE,previous_failure=failure)
            passed=sum(r['status']=='PASS' for r in rows);failed=sum(r['status']=='FAILED' for r in rows)
            result=final_result(plan,rows,failure,seal_errors,plan_sha,grant_sha,source_after,input_after,closing_memory,time.monotonic()-budget.started)
            write_new(OUTPUT,OUTPUT/'result.json',result,closing=True)
        return dict(status=result['status'],PASS=passed,FAIL=failed,UNEXECUTED=result['UNEXECUTED'],path=str(OUTPUT/'result.json'))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);p.add_argument('--mode',choices=('prepare','execute'),required=True)
    p.add_argument('--plan-sha256');p.add_argument('--authority-sha256');a=p.parse_args(argv)
    value=prepare() if a.mode=='prepare' else execute(a.plan_sha256,a.authority_sha256)
    print(encoded(value).decode(),end='');return 0 if value['status'] in ('CANDIDATE_ONLY_NOT_ROOT_AUTHORITY','COMPLETE_PIXEL_CONSISTENCY_ONLY') else 2


if __name__=='__main__':raise SystemExit(main())
