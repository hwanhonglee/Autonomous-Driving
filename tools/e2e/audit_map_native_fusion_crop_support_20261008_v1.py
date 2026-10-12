"""HH_261008 - Reconstruct sealed V3 TRAIN label support without training or inference.

Both original 20k/B8 journals are identical: compute 160k label crops once, but
retain both original 160k denominators. Every actual B8 target hash must match.
Only the two approved TRAIN target.npy members are decoded, never RGB/TRUE,
weights, predictions, DEV/HOLDOUT/TEST or denied CTrack captures. Current local
Pillow is separately sealed; equality of historical environments is not claimed.
The 512MiB RSS and 900s deadline are observed/sampled, not OS containment.
"""
from __future__ import annotations

import argparse
import ast
import copy
from contextlib import contextmanager
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
import struct
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
TEST = ROOT / 'learning/tests/test_map_native_fusion_crop_support_20261008_v1.py'
NAME = 'map_native_fusion_crop_support_v1'
SCHEMA = 'portable_e2e.map_native_fusion_crop_support.20261008.v1'
PLAN_DIR = ROOT / 'storage/experiments/plans/2026-10-08' / NAME
PLAN, AUTHORITY = PLAN_DIR / 'ROOT_plan.json', PLAN_DIR / 'ROOT_authority.json'
OUTPUT = ROOT / 'storage/experiments/diagnostics/2026-10-08' / NAME
LOCK = ROOT / 'storage/assets/locks/autoware_e2e_runtime.lock'
RUN = ROOT / 'storage/experiments/training/2026-10-02/map_native_fusion_pair_v3'
DATA = ROOT / 'storage/experiments/training/2026-10-02/map_paired_balanced_20261002_v1'
CROP = ROOT / 'tools/e2e/map_appearance_matched_contract_20261001.py'
MEMORY = ROOT / 'tools/e2e/diagnose_map_sign_components_20261007_v2.py'
CROP_PIN = '9fb7b057a09d955a8952699f3a5868a8f6a6d04cdf78ea66ada7eea742c53b96'
MEMORY_PIN = 'cb72db2bc043166259db66a3276d8e3edfa51ce390dc0b93428ac61316198ef8'
LABELS = ('other', 'road', 'sidewalk', 'lane_marking', 'traffic_light_fixture', 'traffic_sign', 'pole')
STEPS, BATCH, TOTAL, ORIGINAL, PIXELS, SEED = 20000, 8, 1209, 729, 360*640, 2026100203
ARMS = ('control', 'candidate')
GROUPS = ('Town01_original497', 'Town02_original232', 'Town01_p02_FALSE240', 'Town01_p04_FALSE240')
FRAME_MASK_SUPPORT_BINS = ('zero', '1_to16', '17_to64', '65_to256', '257_plus')
FLOOR, RESERVE, LIMIT, RSS, DEADLINE = 2*1024**3, 64*1024**2, 2*1024**2, 512*1024**2, 900
META_LIMIT, CLOSING, ROOT_REVIEW_RESERVE, LINE_LIMIT, HEADER_LIMIT = 512*1024, 512*1024, 64*1024, 8192, 4096
ENV = {'CUDA_VISIBLE_DEVICES':'', 'PYTHONNOUSERSITE':'1', 'PYTHONDONTWRITEBYTECODE':'1',
       'OMP_NUM_THREADS':'1', 'MKL_NUM_THREADS':'1', 'OPENBLAS_NUM_THREADS':'1'}
META_PINS = {
    RUN/'authorization.json':'2b5a000cb2e3f94ced7cbbeb1e64fc07a1fe71b5e11c0d72b11f1f51ffe89ef8',
    RUN/'summary.json':'57c658b0919d131350cf8a9adbffffdd3e0f2d7f22a9a3e2b184b41b24dca7b1',
    RUN/'equal_global_schedule_proof.json':'c058dd0d79a7578a7d61709975cfefe3f9aada937a049bfe78cae731e1c2db35',
    RUN/'fit_complete.json':'8b4694f5def04f12315c7b308f739273f55e128bccd3cdaa351a2aabc506a453',
    RUN/'loader_parity.json':'0ae8df93015fad6191ce52a181b7a29b031244ea9a6972c0af752af4fb46497b',
    RUN/'control/training.json':'14cebf2ebca5cf3db558b9266f29abf6fbbd88f36f42e13ffbf55c332152d92b',
    RUN/'candidate/training.json':'6a053701e287e96f780cb12e85c25e3c9410fb933bdaf4bedf33fddd95000348',
    RUN/'inputs/manifest.json':'614840471e938a0e9ecd169368d6240e96c01fdf76c74b56f82863c9d49d2b1b',
    RUN/'inputs/study_train_registry.json':'ca892a6b6e34602b2c983fdec83972a3693bfd763baafb4abfac2f1a488bad8d',
    RUN/'inputs/paired_samples.json':'899d82128ae9597beca6e22fbad4abaaff40c7e1fbc502418f946f71c72b9d6b',
}
EXPOSURE_PIN = '1fa5bfba0e979f049b04a9001e3fd6f52100bd77a45a7ce1ef63d1881d7b6225'
RECORD_PIN = '7f6df41b42c057ec0d25b7cedf4aaa277391eefb07b3a7815eb2590868fd8fcc'
PACKAGE_PINS = {'original_train729.npz':'d2d7b8b306fcba9672a229b4b1222b208e8d17fe500caf43755687612025e8f3',
                'paired_train480.npz':'47c23a3f6fe67aa40ea7f5bb0c770fd6037463ed090f2b9390a89317c4dedee3'}
OTHER_PINS = {
    RUN/'equal_global_schedule.npz':'82dfa28e2166e1fd7f59a5428c89df282e8103d179bed99f481762f628259e55',
    RUN/'mining/records.jsonl':RECORD_PIN,
    RUN/'control/exposures.jsonl':EXPOSURE_PIN, RUN/'candidate/exposures.jsonl':EXPOSURE_PIN,
    RUN/'control/history.jsonl':'af6286c5ee76a11a3bf5dff740de6a76d7b64a6bcfaf3fbd3b5b2ded921b5561',
    RUN/'candidate/history.jsonl':'3e70f694791f856c5ed29ba0da45795f87d1574bee89a18d761d307f741494f3',
    **{DATA/name:pin for name,pin in PACKAGE_PINS.items()},
}
COUNTER_COLUMNS = ('exposures', 'whole_crop', 'partial_crop', 'positive_full_frame',
    'zero_after_from_positive_full', 'raw_crop_cut_all_sign', 'resize_zero_from_positive_crop',
    'raw_crop_cut_some_sign', 'full_frame_sign_pixels_sum', 'source_crop_sign_pixels_sum',
    'resized_sign_pixels_sum', *('resized_'+label+'_pixels' for label in LABELS))


def require(value, message):
    if not value: raise ValueError(message)


def sha(value):
    return type(value) is str and re.fullmatch('[0-9a-f]{64}', value) is not None


def canonical(path, *, exists=True, directory=False):
    path=Path(path)
    require(path.is_absolute() and '..' not in path.parts and path==path.resolve(strict=exists)
        and not any(p.is_symlink() for p in (path,*path.parents)), 'canonical unaliased path required')
    if exists:require(path.is_dir() if directory else path.is_file(), 'regular expected path type required')
    return path


def fingerprint(path):
    s=canonical(path).stat();return [s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns]


def digest(path, guard=lambda:None):
    path=canonical(path);before=fingerprint(path);state=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024**2),b''):guard();state.update(chunk)
    require(before==fingerprint(path),'sealed file changed during SHA read');return state.hexdigest()


def strict_json(raw):
    def pairs(items):
        result={}
        for key,value in items:require(key not in result,'duplicate JSON key');result[key]=value
        return result
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=lambda x:require(False,'nonfinite JSON constant'))


def read_json(path, pin=None):
    path=canonical(path);require(path.stat().st_size<=META_LIMIT,'metadata JSON exceeds512KiB')
    before=fingerprint(path);raw=path.read_bytes();require(before==fingerprint(path),'metadata changed during read')
    require(pin is None or hashlib.sha256(raw).hexdigest()==pin,'metadata SHA differs');return strict_json(raw)


def encoded(value):
    return (json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()


def bounded_error(error):
    raw=repr(error).encode('utf8',errors='backslashreplace')
    return dict(repr_SHA256=hashlib.sha256(raw).hexdigest(),bytes=len(raw),prefix_hex=raw[:32].hex())


def storage_used(roots=None):
    roots=tuple(map(Path,roots or (PLAN_DIR,OUTPUT)));require(len(roots)==2 and roots[0]!=roots[1]
        and not any(a.is_relative_to(b) for a,b in (roots,roots[::-1])),'two disjoint storage roots required')
    total=0
    for root in roots:
        canonical(root,exists=False)
        if not root.exists():continue
        canonical(root,directory=True)
        for folder,dirs,files in os.walk(root,followlinks=False):
            for name in dirs+files:
                path=Path(folder)/name;mode=path.lstat().st_mode
                require(path==path.resolve(strict=True) and (stat.S_ISREG(mode) or stat.S_ISDIR(mode)),'storage link/type denied')
                if stat.S_ISREG(mode):total+=path.stat().st_size
    return total


def write_bytes(root,path,payload,*,closing=False,roots=None):
    roots=tuple(map(Path,roots or (PLAN_DIR,OUTPUT)));root=canonical(root,directory=True);path=Path(path)
    require(root in roots and path.is_relative_to(root) and path.parent==path.parent.resolve(strict=True)
        and not path.is_symlink() and len(payload)<=META_LIMIT,'bounded own metadata write required')
    require(storage_used(roots)+len(payload)<=LIMIT-(0 if closing else CLOSING),'combined plan/output2MiB budget denied')
    with path.open('xb') as stream:stream.write(payload)


def write_new(root,path,value,**options):
    write_bytes(root,path,encoded(value),**options)


def memory_parser():
    require(digest(MEMORY)==MEMORY_PIN,'frozen memory parser changed')
    tree=ast.parse(MEMORY.read_bytes());nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef)
        and n.name in ('require','parse_process_memory')]
    require(len(nodes)==2,'unique original current-process memory seams required')
    namespace=dict(re=re,__name__='_hh_crop_memory_private')
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(MEMORY),'exec'),namespace)
    return namespace['parse_process_memory']


def process_memory(parser):
    with Path('/proc/self/status').open('r',encoding='ascii') as stream:text=stream.read(16385)
    value=parser(text,os.getpid())
    value['inherited_ru_maxrss_bytes_NOT_GATE']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024
    return value


def parse_generation(raw,pid):
    require(type(raw) is str and type(pid) is int and pid>0 and raw.startswith(str(pid)+' ('),'exact /proc PID required')
    right=raw.rfind(')');require(right>=0,'malformed process stat')
    fields=raw[right+2:].split();require(len(fields)>=20 and all(re.fullmatch('[0-9]+',fields[i]) for i in (1,2,3,19)),'typed PID-generation fields required')
    return dict(PID=pid,PPID=int(fields[1]),process_group=int(fields[2]),session=int(fields[3]),start_ticks=int(fields[19]))


def process_generation():
    pid=os.getpid();value=parse_generation(Path('/proc/self/stat').read_text(),pid)
    value.update(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        executable=str(Path('/proc/self/exe').resolve(strict=True)),
        cmdline_sha256=hashlib.sha256(Path('/proc/self/cmdline').read_bytes()).hexdigest())
    require(value['PPID']==os.getppid() and value['process_group']==os.getpgid(0) and value['session']==os.getsid(0),'live self generation differs')
    return value


class Budget:
    def __init__(self):
        self.started=time.monotonic();self.parser=memory_parser();self.last=None;self.maximum_HWM=0;self.strict_affinity=False
    def guard(self):
        self.last=process_memory(self.parser);self.maximum_HWM=max(self.maximum_HWM,self.last['VmHWM_bytes'])
        require(self.maximum_HWM<=RSS,'observed self512MiB VmHWM exceeded')
        require(time.monotonic()-self.started<DEADLINE,'sampled whole900s deadline exceeded')
        if self.strict_affinity:require(os.sched_getaffinity(0)=={3},'actual single CPU3 affinity changed')
        s=os.statvfs(OUTPUT.parent);require(s.f_bavail*s.f_frsize>=FLOOR+RESERVE,'2GiB floor plus64MiB reserve unavailable')


@contextmanager
def lease():
    with canonical(LOCK).open('r') as stream:
        fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:yield fingerprint(LOCK)[:2]
        finally:fcntl.flock(stream,fcntl.LOCK_UN)


def environment(*,unloaded=True):
    require(Path.cwd()==ROOT and os.geteuid()==ROOT.stat().st_uid and sys.flags.optimize==0 and sys.flags.dont_write_bytecode
        and all(os.environ.get(k)==v for k,v in ENV.items()) and not any(os.environ.get(k)
        for k in ('PYTHONPATH','PYTHONHOME','PYTHONUSERBASE','PYTHONSTARTUP')),'clean canonical CPU-only environment required')
    denied=('torch','tensorflow','carla','rclpy','rospy','portable_e2e')+ (('numpy','PIL') if unloaded else ())
    require(not any(k==n or k.startswith(n+'.') for k in sys.modules for n in denied),'unexpected model/simulator/numeric import before grant')
    require(3 in os.sched_getaffinity(0),'declared logical CPU3 unavailable')
    specs={key:importlib.util.find_spec(key) for key in ('numpy','PIL')}
    require(all(v and v.origin for v in specs.values()),'existing local NumPy/Pillow providers unavailable')
    origins={key:str(canonical(v.origin)) for key,v in specs.items()};npdir=Path(origins['numpy']).parent;pildir=Path(origins['PIL']).parent
    native=list(npdir.glob('core/_multiarray_umath*.so'))+list(npdir.glob('_core/_multiarray_umath*.so'))
    imaging=list(pildir.glob('_imaging.*.so'));require(len(native)==len(imaging)==1,'one selected native provider required')
    providers={**origins,'PIL.Image':str(pildir/'Image.py'),'PIL._imaging':str(imaging[0]),
        'numpy_native':str(native[0]),'python_executable':str(Path(sys.executable).resolve(strict=True))}
    return dict(executable=providers['python_executable'],origins=origins,
        versions={k:importlib.metadata.version(k) for k in ('numpy','Pillow')},
        selected_provider_sha256={p:digest(p) for p in providers.values()},
        complete_transitive_native_closure_claimed=False,historical_environment_equivalence_claimed=False,CPU_affinity=[3])


def source_group(index):
    require(type(index) is int and 0<=index<TOTAL,'physical TRAIN slot outside1209')
    return GROUPS[0] if index<497 else GROUPS[1] if index<729 else GROUPS[2] if index<969 else GROUPS[3]


def frame_mask_support_bin(sign_pixels):
    require(type(sign_pixels) is int and 0<=sign_pixels<=PIXELS,'whole-frame GT sign pixel support required')
    return FRAME_MASK_SUPPORT_BINS[0] if sign_pixels==0 else FRAME_MASK_SUPPORT_BINS[1] if sign_pixels<=16 else FRAME_MASK_SUPPORT_BINS[2] if sign_pixels<=64 else FRAME_MASK_SUPPORT_BINS[3] if sign_pixels<=256 else FRAME_MASK_SUPPORT_BINS[4]


def validate_batch(row,ordinal,*,total=TOTAL,batch=BATCH):
    keys={'step','physical_slot_indices','shared_crop_boxes','actual_paired_TRUE_bits','RGB_sha256','target_sha256'}
    require(type(row) is dict and set(row)==keys and type(row['step']) is int and row['step']==ordinal+1,'exact original exposure row/chronology required')
    ids,boxes,bits=(row[k] for k in ('physical_slot_indices','shared_crop_boxes','actual_paired_TRUE_bits'))
    require(all(type(v) is list and len(v)==batch for v in (ids,boxes,bits)),'one complete B8 exposure required')
    require(all(type(v) is int and 0<=v<total for v in ids) and all(type(v) is int and v==0 for v in bits),'exact physical indices/FALSE-only integers required')
    for box in boxes:
        require(type(box) is list and len(box)==4 and all(type(v) is int for v in box),'four integer crop coordinates required')
        top,left,height,width=box
        require(top>=0 and left>=0 and height>0 and width>0 and top+height<=360 and left+width<=640,'original calibrated crop bounds required')
    require(sha(row['RGB_sha256']) and sha(row['target_sha256']),'actual historical RGB/target hash witnesses required')
    return row


def json_lines(path,*,limit=LINE_LIMIT):
    path=canonical(path);before=fingerprint(path)
    with path.open('rb') as stream:
        while True:
            line=stream.readline(limit+1)
            if not line:break
            require(len(line)<=limit and line.endswith(b'\n'),'bounded complete JSONL row required')
            yield strict_json(line)
    require(before==fingerprint(path),'journal changed during full chronology read')


def journal_summary(path,*,steps=STEPS,total=TOTAL,batch=BATCH,guard=lambda:None):
    ids_hash,crops_hash,bits_hash,target_chain=(hashlib.sha256() for _ in range(4))
    counts=[0]*total;full=partial=rows=0
    for ordinal,row in enumerate(json_lines(path)):
        require(ordinal<steps,'extra exposure row denied');guard();validate_batch(row,ordinal,total=total,batch=batch)
        ids_hash.update(struct.pack('<'+str(batch)+'q',*row['physical_slot_indices']))
        crops_hash.update(struct.pack('<'+str(batch*4)+'h',*[v for box in row['shared_crop_boxes'] for v in box]))
        bits_hash.update(bytes(row['actual_paired_TRUE_bits']))
        target_chain.update(struct.pack('<Q',ordinal+1)+bytes.fromhex(row['target_sha256']))
        for index,box in zip(row['physical_slot_indices'],row['shared_crop_boxes']):
            counts[index]+=1;full+=int(box==[0,0,360,640]);partial+=int(box!=[0,0,360,640])
        rows+=1
    require(rows==steps,'missing exposure tail denied')
    return dict(steps=rows,exposures=rows*batch,slot_exposures=counts,whole=full,partial=partial,
        sample_sequence_sha256=ids_hash.hexdigest(),crop_sequence_sha256=crops_hash.hexdigest(),
        view_sequence_sha256=bits_hash.hexdigest(),expected_target_chain_sha256=target_chain.hexdigest())


def history_summary(path,*,steps=STEPS,guard=lambda:None):
    count=0
    for ordinal,row in enumerate(json_lines(path)):
        guard();require(ordinal<steps and type(row) is dict and type(row.get('step')) is int and row['step']==ordinal+1
            and type(row.get('optimizer_updates')) is int and row['optimizer_updates']==ordinal+1
            and row.get('all_gradients_present_finite') is True,'complete original optimizer chronology required')
        require(all(type(row.get(k)) in (int,float) and math.isfinite(row[k]) and row[k]>=0
            for k in ('original_CE','gradient_norm','wall_seconds')),'original finite update metadata required')
        count+=1
    require(count==steps,'missing original optimizer tail denied');return count


def sources():
    auth=read_json(RUN/'authorization.json',META_PINS[RUN/'authorization.json']);pins=auth.get('source_sha256')
    require(type(pins) is dict and len(pins)==15 and all(Path(n).name==n and sha(h) for n,h in pins.items()),'all15 original frozen source identities required')
    table={str(MEMORY):MEMORY_PIN}
    for name,pin in pins.items():
        path=ROOT/'tools/e2e'/name
        if not path.is_file():path=ROOT/'learning/portable_e2e'/name
        table[str(path)]=pin;table[str(RUN/'source'/name)]=pin
    for p in (HERE,TEST,ROOT/'tools/workspace_python.py',ROOT/'tools/canonical_workspace.py'):table[str(p)]=digest(p)
    require(table[str(CROP)]==CROP_PIN and all(digest(p)==h for p,h in table.items()),'original/new current/archive source closure differs')
    return table


def input_pins():
    return {str(p):h for p,h in {**META_PINS,**OTHER_PINS}.items()}


def crop_ast():
    # HH_261008 - Keep the exact original target expressions; omit only the independent RGB branch and RGB-only ABI clause.
    require(digest(CROP)==CROP_PIN,'original crop provider changed')
    nodes=[n for n in ast.parse(CROP.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=='apply_crop']
    require(len(nodes)==1 and len(nodes[0].body)==9,'unique exact nine-statement original crop required');old=nodes[0]
    require(isinstance(old.body[4],ast.If) and len(old.body[4].body)==1 and isinstance(old.body[4].body[0],ast.Return)
        and isinstance(old.body[4].body[0].value,ast.Tuple) and len(old.body[4].body[0].value.elts)==2
        and isinstance(old.body[8],ast.Return) and isinstance(old.body[8].value,ast.Tuple) and len(old.body[8].value.elts)==2,'original paired return seams differ')
    whole=copy.deepcopy(old.body[4]);whole.body[0].value=copy.deepcopy(old.body[4].body[0].value.elts[1])
    final=ast.Return(value=copy.deepcopy(old.body[8].value.elts[1]))
    args=ast.arguments(posonlyargs=[],args=[ast.arg(arg='target'),ast.arg(arg='box')],vararg=None,kwonlyargs=[],kw_defaults=[],kwarg=None,defaults=[])
    node=ast.FunctionDef(name='target_crop',args=args,body=[copy.deepcopy(old.body[i]) for i in (2,3)]+[whole,copy.deepcopy(old.body[5]),copy.deepcopy(old.body[7]),final],decorator_list=[])
    require(not any(isinstance(n,ast.Name) and n.id in ('rgb','image') for n in ast.walk(node)),'label-only AST may not touch RGB')
    require(ast.unparse(old.body[7].targets[0])=='label' and ast.unparse(old.body[5].targets[0])=='sampling'
        and ast.unparse(old.body[7].value.args[-1])=='sampling.NEAREST','exact original label NEAREST branch required')
    return old,ast.fix_missing_locations(node)


def crop_proof():
    old,node=crop_ast();return dict(source_sha256=CROP_PIN,
        original_apply_crop_AST_sha256=hashlib.sha256(ast.dump(old,include_attributes=False).encode()).hexdigest(),
        exact_original_label_branch_AST_sha256=hashlib.sha256(ast.dump(node,include_attributes=False).encode()).hexdigest(),
        RGB_branch_excluded=True,historical_label_bytes_required_for_every_B8=True)


def compile_target_crop(np,Image):
    _,node=crop_ast();space=dict(np=np,Image=Image,require=require,__name__='_hh_crop_label_private')
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(CROP),'exec'),space);return space['target_crop']


def npy_header(stream,shape):
    prefix=stream.read(8);require(prefix==b'\x93NUMPY\x01\x00','exact NPYv1 uint8 header required')
    size=stream.read(2);require(len(size)==2,'truncated NPY header length');length=int.from_bytes(size,'little')
    require(0<length<=HEADER_LIMIT,'NPY header bound');raw=stream.read(length);require(len(raw)==length,'truncated NPY header')
    node=ast.parse(raw.decode('latin1'),mode='eval').body
    require(isinstance(node,ast.Dict) and len(node.keys)==3 and all(isinstance(k,ast.Constant) and type(k.value) is str for k in node.keys)
        and len({k.value for k in node.keys})==3,'duplicate/malformed header key denied')
    value=ast.literal_eval(node);require(type(value) is dict and set(value)=={'descr','fortran_order','shape'}
        and value['descr']=='|u1' and value['fortran_order'] is False and type(value['shape']) is tuple
        and all(type(n) is int and n>0 for n in value['shape']) and value['shape']==tuple(shape),'exact C-order uint8 shape required')
    return prefix+size+raw


def zip_census(archive,names):
    actual=archive.namelist();require(len(actual)==len(set(actual))==len(names) and set(actual)==set(names),'exact ZIP members required; aliases/extra/traversal denied')
    require(all(not r.flag_bits&1 and r.compress_type==zipfile.ZIP_DEFLATED and r.compress_size>0
        and not r.is_dir() for r in archive.infolist()),'sealed unencrypted regular DEFLATED ZIP members required')
    return [dict(name=r.filename,size=r.file_size,compressed=r.compress_size,CRC=r.CRC,flags=r.flag_bits,compression=r.compress_type) for r in archive.infolist()]


def target_entry(archive,count):
    # HH_261008 - This sole member-open site never opens any RGB, TRUE, weight or prediction body.
    stream=archive.open('target.npy')
    try:
        header=npy_header(stream,(count,360,640));entry=archive.getinfo('target.npy')
        require(len(header)==128 and entry.file_size==len(header)+count*PIXELS,'no target truncation/padding allowed')
        return stream,header
    except BaseException:
        stream.close();raise


def package_metadata(path,count,names):
    before=fingerprint(path)
    with zipfile.ZipFile(canonical(path)) as archive:
        members=zip_census(archive,names)
        stream,header=target_entry(archive,count)
        with stream:pass  # HH_261008 - Only the128B header is inspected during prepare; body/CRC not claimed before EOF.
    require(before==fingerprint(path),'package changed during header inspection')
    return dict(members=members,target_header_sha256=hashlib.sha256(header).hexdigest(),target_shape=[count,360,640],
        target_body_bytes=count*PIXELS,body_decoded=False,whole_member_CRC_verified=False)


def fill_exact(stream,destination,guard=lambda:None):
    # HH_261008 - Preallocated target-only storage; bounded readinto avoids a second full-target byte buffer.
    view=memoryview(destination).cast('B');offset=0
    while offset<len(view):
        guard();end=min(offset+1024**2,len(view));amount=stream.readinto(view[offset:end])
        require(type(amount) is int and 0<amount<=end-offset,'truncated/nonprogress target readinto denied');offset+=amount
    require(stream.read(1)==b'','extra target byte denied; EOF completes ZIP CRC');return offset


def load_targets(np,metadata,budget):
    values=[]
    for name,count,names in (('original_train729.npz',729,('rgb.npy','target.npy')),
        ('paired_train480.npz',480,('rgb_false.npy','rgb_true.npy','target.npy'))):
        path=DATA/name;before=fingerprint(path);budget.guard()
        with zipfile.ZipFile(canonical(path)) as archive:
            require(zip_census(archive,names)==metadata[name]['members'],'declared archive metadata changed')
            stream,header=target_entry(archive,count)
            require(hashlib.sha256(header).hexdigest()==metadata[name]['target_header_sha256'],'sealed target header changed')
            target=np.empty((count,360,640),dtype=np.uint8);budget.guard()
            with stream:require(fill_exact(stream,target,budget.guard)==count*PIXELS,'whole target byte denominator differs')
        require(before==fingerprint(path),'target package changed during decode')
        require(target.shape==(count,360,640) and target.dtype==np.uint8 and int(target.max())<7,'unknown label or decoded target ABI denied')
        budget.guard();values.append(target)
    return tuple(values)


def evidence(guard=lambda:None):
    docs={p:read_json(p,h) for p,h in META_PINS.items()};auth=docs[RUN/'authorization.json'];summary=docs[RUN/'summary.json']
    manifest=docs[RUN/'inputs/manifest.json'];registry=docs[RUN/'inputs/study_train_registry.json'];proof=docs[RUN/'equal_global_schedule_proof.json']
    require(summary['status']=='FIXED20K_ARCHITECTURE_PAIR_DIAGNOSTIC_NOT_PROMOTED' and summary['source_and_inputs_unchanged'] is True,'completed immutable original V3 pair required')
    require(auth['source_sha256']==summary['source_sha256'] and len(auth['source_sha256'])==15 and auth['research_fit_authorized'] is True
        and auth['DEV_or_HOLDOUT_used_for_fit'] is False and auth['automatic_model_promotion'] is False,'original V3 authority/source/split boundary differs')
    require(manifest['labels']==auth['contract']['labels']==list(LABELS),'exact original seven-class order required')
    for item in (manifest,registry):
        require(item['ctrack_train_eligible'] is False and item['dev_or_test_used_for_fit'] is False
            and item['native_diagnostic_denials_preserved'] is True and item['vehicle_control_approved'] is False,'original protected TRAIN boundary differs')
    require(registry['research_fit_authorized'] is True and registry['broad_registry_modified'] is False
        and manifest['counts']['logical_slots_per_arm']==TOTAL and manifest['counts']['original']==ORIGINAL
        and manifest['counts']['physical_pairs']==480 and registry['data_files']==manifest['files'],'approved original1209 package required')
    require(set(registry['source_captures'])=={'p02','p04'} and all(r['raw_source_dataset_admitted'] is False
        and r['raw_source_train_eligible'] is False for r in registry['source_captures'].values()),'native raw-capture denial must remain false')
    require(manifest['study_registry_sha256']==META_PINS[RUN/'inputs/study_train_registry.json']
        and manifest['paired_samples_sha256']==META_PINS[RUN/'inputs/paired_samples.json'],'registry/paired provenance pointer differs')
    for name,pin in PACKAGE_PINS.items():require(manifest['files'][name]['sha256']==pin and canonical(DATA/name).stat().st_size==manifest['files'][name]['bytes'],'exact registered TRAIN package size/hash required')
    require(proof['steps']==STEPS and proof['batch']==BATCH and proof['seed']==SEED and proof['total_exposures_per_arm']==STEPS*BATCH
        and proof['CTRL_and_candidate_exact_same_input_schedule'] is True and proof['native_denials_preserved'] is True
        and summary['equal_global_schedule']==proof,'fixed20k paired schedule proof required')
    require(auth['equal_global_schedule_sha256']==hashlib.sha256(json.dumps(proof,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest(),'original schedule authority differs')
    require(auth['contract']['FALSE_views_only'] is True and auth['contract']['examples_per_arm']==STEPS*BATCH
        and auth['contract']['source_group_counts']==dict(Town01_original=497,Town02_original=232,Town01_p02_FALSE=240,Town01_p04_FALSE=240),'no view/scope expansion required')
    parity=docs[RUN/'loader_parity.json'];require(parity['byte_equal'] is True and parity['actual_TRAIN_only'] is True and parity['loader0']==parity['loader2'],'original CPU loader parity required')
    shared=None
    for arm in ARMS:
        path=RUN/arm/'exposures.jsonl';require(digest(path,guard)==EXPOSURE_PIN,'both whole journals must be exactly byte-identical')
        actual=journal_summary(path,guard=guard);info=docs[RUN/arm/'training.json'];boundary=docs[RUN/'fit_complete.json']['actual_full_journal_evidence'][arm]
        require(actual['slot_exposures']==proof['sample_exposures'] and actual['steps']==info['steps']==boundary['steps']==STEPS
            and info['examples_seen']==boundary['examples_seen']==STEPS*BATCH and info['full_exposure_journal_sha256']==boundary['full_exposure_journal_sha256']==EXPOSURE_PIN,'all actual original exposure rows required')
        for key,proofkey in (('sample_sequence_sha256','control_indices'),('crop_sequence_sha256','shared_crops'),('view_sequence_sha256','shared_FALSE_view_bits')):
            require(actual[key]==info[key]==boundary[key]==proof['tensor_sha256'][proofkey],'actual sequence differs from fixed proof')
        require(proof['tensor_sha256']['control_indices']==proof['tensor_sha256']['candidate_indices'],'matched candidate schedule differs')
        require(history_summary(RUN/arm/'history.jsonl',guard=guard)==STEPS,'original update denominator differs')
        require(shared is None or shared==actual,'two actual journal chronologies differ');shared=actual
    require(sum(shared['slot_exposures'])==STEPS*BATCH and min(shared['slot_exposures'])==132 and max(shared['slot_exposures'])==133,'complete132cycles plus412 exposure census required')
    archives={name:package_metadata(DATA/name,count,names) for name,count,names in (
        ('original_train729.npz',729,('rgb.npy','target.npy')),('paired_train480.npz',480,('rgb_false.npy','rgb_true.npy','target.npy')))}
    return dict(journal=shared,archive_metadata=archives,source_class_pixels=manifest['common_class_pixels'],
        original_runtime=auth['runtime_identity'],crop_AST=crop_proof(),known_evaluation_payloads_opened=False)


def fixed():
    return dict(schema=SCHEMA,arms=list(ARMS),original_steps_per_arm=STEPS,batch=BATCH,
        original_exposures_per_arm=STEPS*BATCH,original_exposures_both_arms=2*STEPS*BATCH,
        computed_shared_exposures=STEPS*BATCH,physical_TRAIN_slots=TOTAL,FALSE_only=True,
        targets_only=True,RGB_TRUE_weights_DEV_HOLDOUT_TEST_CTrack_bodies_opened=False,
        every_actual_B8_target_SHA_required=True,original_apply_crop_target_AST_unchanged=True,
        historical_environment_equivalence_claimed=False,logical_CPU_affinity=[3],threads=1,
        observed_VmRSS_VmHWM_cap_bytes=RSS,inherited_ru_maxrss_is_gate=False,OS_memory_containment_claimed=False,
        deadline_seconds=DEADLINE,deadline_enforcement='sampled_nonpreemptive_including_closing',
        floor_bytes=FLOOR,reserve_bytes=RESERVE,output_cap_bytes=LIMIT,closing_bytes=CLOSING,
        combined_PLAN_and_OUTPUT_file_byte_budget=True,NN=0,training_steps=0,ROS=False,GPU=False,
        source_size_unit='whole-frame GT sign pixels; NOT components or physical objects',
        crop_reroll=False,relabeling=False,model_promoted=False,main_approved=0,sub_approved=0)


def source_archives(plan):
    return {str(PLAN_DIR/'sources_by_SHA256'/(pin+'.py')):pin for pin in set(plan['source_sha256'].values())}


def authority_value(plan,plan_sha):
    return dict(schema=SCHEMA+'.ROOT_authority',ROOT_plan_path=str(PLAN),ROOT_plan_sha256=plan_sha,
        source_sha256=plan['source_sha256'],TRAIN_target_support_diagnostic_only_authorized=True,
        compute_once_only_after_both_full_journals_identical=True,every_original_B8_target_hash_required=True,
        training_authorized=False,NN_authorized=False,ROS_authorized=False,GPU_authorized=False,
        DEV_HOLDOUT_TEST_authorized=False,model_promoted=False)


def storage_admission(plan,*,prospective=False):
    closing_size_proof(plan)
    grant=encoded(authority_value(plan,'f'*64));raw=encoded(plan)
    if prospective:
        sizes={pin:canonical(path).stat().st_size for path,pin in plan['source_sha256'].items()}
        needed=sum(sizes.values())+3*len(raw)+2*len(grant)+ROOT_REVIEW_RESERVE+CLOSING
        require(storage_used()==0,'fresh plan/output storage required')
    else:needed=storage_used()+canonical(PLAN).stat().st_size+canonical(AUTHORITY).stat().st_size+CLOSING
    require(needed<=LIMIT,'prospective complete plan/output2MiB including closing denied');return needed


def verify_tables(plan,budget=None,*,unloaded=True):
    guard=budget.guard if budget else lambda:None
    require(environment(unloaded=unloaded)==plan['environment'] and crop_proof()==plan['evidence']['crop_AST'],'current clean providers/original crop AST changed')
    for table in (plan['source_sha256'],plan['input_sha256'],source_archives(plan)):
        for p,pin in table.items():require(digest(p,guard)==pin,'whole sealed source/input/archive differs')
    require({str(p) for p in (PLAN_DIR/'sources_by_SHA256').iterdir()}==set(source_archives(plan)),'unique source-copy census differs')


def prepare():
    require(not PLAN_DIR.exists() and not OUTPUT.exists(),'fresh immutable plan/output only')
    with lease() as lock_id:
        budget=Budget();env=environment();budget.guard();src=sources();pins=input_pins()
        for p,h in pins.items():require(digest(p,budget.guard)==h,'all prospective TRAIN metadata/packages must be sealed before import')
        observed=evidence(budget.guard);require(env==environment(),'providers changed during metadata prepare')
        plan=dict(schema=SCHEMA,fixed=fixed(),source_sha256=src,input_sha256=pins,
            evidence=observed,environment=env,lock_identity=lock_id,prepare_process_generation=process_generation())
        storage_admission(plan,prospective=True);PLAN_DIR.mkdir(parents=True,exist_ok=False)
        archive=PLAN_DIR/'sources_by_SHA256';archive.mkdir(exist_ok=False)
        for p,h in src.items():
            raw=canonical(p).read_bytes();require(hashlib.sha256(raw).hexdigest()==h,'source changed before copy')
            destination=archive/(h+'.py')
            if destination.exists():require(digest(destination)==h,'unique source copy differs')
            else:write_bytes(PLAN_DIR,destination,raw)
        write_new(PLAN_DIR,PLAN_DIR/'candidate_plan.json',plan)
        return dict(status='CANDIDATE_ONLY_NOT_ROOT_AUTHORITY',path=str(PLAN_DIR/'candidate_plan.json'),
            sha256=digest(PLAN_DIR/'candidate_plan.json'),target_pixels_decoded=0,NN=0,training_steps=0)


def authorization(plan_sha,grant_sha):
    require(sha(plan_sha) and sha(grant_sha),'external raw ROOT plan and authority SHA required')
    plan=read_json(PLAN,plan_sha);grant=read_json(AUTHORITY,grant_sha)
    require(plan['schema']==SCHEMA and plan['fixed']==fixed() and grant==authority_value(plan,plan_sha),'exact external fixed diagnostic authority required')
    require(plan['input_sha256']==input_pins() and plan['source_sha256']==sources(),'ROOT sealed original sources/inputs differ')
    require(plan==read_json(PLAN_DIR/'candidate_plan.json') and digest(PLAN_DIR/'candidate_plan.json')==plan_sha,'ROOT plan must retain exact prepared bytes')
    return plan


def empty_counts():
    return [0]*len(COUNTER_COLUMNS)


def exposure_counts(full,source_sign,histogram,whole):
    require(type(full) is type(source_sign) is int and 0<=source_sign<=full<=PIXELS and type(whole) is bool
        and type(histogram) in (list,tuple) and len(histogram)==7 and all(type(n) is int and n>=0 for n in histogram)
        and sum(histogram)==PIXELS,'actual source/resized support denominator differs')
    sign=histogram[5]
    return [1,int(whole),int(not whole),int(full>0),int(full>0 and sign==0),int(full>0 and source_sign==0),
        int(source_sign>0 and sign==0),int(0<source_sign<full),full,source_sign,sign,*histogram]


def add_counts(target,values):
    require(len(target)==len(values)==len(COUNTER_COLUMNS),'counter ABI differs')
    for i,value in enumerate(values):target[i]+=value


def native_support(np,targets,budget,progress):
    pixels=[0]*7;signs=[];chain=hashlib.sha256();records=json_lines(RUN/'mining/records.jsonl');count=0
    for slot,row in enumerate(records):
        progress['failed_slot']=slot;progress['UNEXECUTED_slots']=TOTAL-count-1
        budget.guard();require(slot<TOTAL and row.get('index')==slot and type(row.get('index')) is int
            and row.get('source_group')==source_group(slot) and row.get('split')=='TRAIN'
            and row.get('appearance')=='FALSE_OR_ORIGINAL' and row.get('full_FOV') is True,'all1209 original TRAIN identities required')
        target=targets[0][slot] if slot<ORIGINAL else targets[1][slot-ORIGINAL]
        actual=hashlib.sha256(memoryview(target)).hexdigest();hist=np.bincount(target.ravel(),minlength=7).tolist()
        require(sha(row.get('target_sha256')) and actual==row['target_sha256'] and len(hist)==7 and hist==row['target_class_pixels']
            and hist[5]==row['sign_target_pixels'],'native1209 target bytes/support differs at slot '+str(slot))
        chain.update(struct.pack('<Q',slot)+bytes.fromhex(actual));signs.append(hist[5]);pixels=[a+b for a,b in zip(pixels,hist)];count+=1
        progress.update(sign_pixels_by_slot=list(signs),class_pixels=list(pixels),target_hash_chain_sha256=chain.hexdigest(),
            completed_slots=count,failed_slot=None,UNEXECUTED_slots=TOTAL-count)
    require(count==TOTAL,'native TRAIN target tail missing');return signs,pixels,chain.hexdigest()


def commit_counts(stats,slots,values,signs):
    # HH_261008 - Build a small transactional scalar update; a failed batch cannot leak partial counters into the PASS-prefix totals.
    require(len(slots)==len(values)==BATCH,'one whole scalar B8 commit required')
    result=dict(stats,shared_once_overall=list(stats['shared_once_overall']),
        source_groups={k:list(v) for k,v in stats['source_groups'].items()},
        frame_mask_support_bins={k:list(v) for k,v in stats['frame_mask_support_bins'].items()},
        slot_statistics=list(stats['slot_statistics']))
    for slot in set(slots):result['slot_statistics'][slot]=list(stats['slot_statistics'][slot])
    for slot,row in zip(slots,values):
        for destination in (result['shared_once_overall'],result['source_groups'][source_group(slot)],
            result['frame_mask_support_bins'][frame_mask_support_bin(signs[slot])],result['slot_statistics'][slot]):add_counts(destination,row)
    return result


def denominator(passed,failed):
    require(type(passed) is type(failed) is int and 0<=passed<=STEPS and failed in (0,1) and passed+failed<=STEPS,'full20k failure denominator required')
    per=dict(expected_batches=STEPS,PASS_batches=passed,FAIL_batches=failed,UNEXECUTED_batches=STEPS-passed-failed,
        expected_exposures=STEPS*BATCH,PASS_exposures=passed*BATCH,FAIL_batch_exposures=failed*BATCH,
        UNEXECUTED_exposures=(STEPS-passed-failed)*BATCH)
    return dict(by_arm={arm:dict(per) for arm in ARMS},original_total_exposures=2*STEPS*BATCH,
        original_PASS_exposures=2*passed*BATCH,original_FAILED_batch_exposures=2*failed*BATCH,
        original_UNEXECUTED_exposures=2*(STEPS-passed-failed)*BATCH,shared_batches_hashed=passed)


def seal_manifest(plan,plan_sha,grant_sha):
    return {**plan['source_sha256'],**plan['input_sha256'],str(PLAN):plan_sha,str(AUTHORITY):grant_sha,
        str(PLAN_DIR/'candidate_plan.json'):plan_sha,str(OUTPUT/PLAN.name):plan_sha,str(OUTPUT/AUTHORITY.name):grant_sha,
        **source_archives(plan)}


def closing_seals(plan,plan_sha,grant_sha):
    errors={}
    for path,pin in seal_manifest(plan,plan_sha,grant_sha).items():
        try:
            actual=digest(path)
            if actual!=pin:errors[path]=dict(expected=pin,actual=actual)
        except BaseException as error:errors[path]=dict(expected=pin,actual=None,error=bounded_error(error))
    return errors


def result_value(plan,plan_sha,grant_sha,passed,failed,failure,stats,seals,memory,generation,elapsed,chain,bitmap,native):
    complete=(failure is None and passed==STEPS and failed==0 and not seals and native is not None
        and native['completed_slots']==TOTAL and native['failed_slot'] is None
        and chain==plan['evidence']['journal']['expected_target_chain_sha256'] and bitmap=='ff'*(STEPS//8))
    return dict(schema=SCHEMA+'.result',status='COMPLETE_ACTUAL_TRAIN_LABEL_SUPPORT_ONLY' if complete else 'FAILED_WITH_FULL_ORIGINAL_DENOMINATOR',
        **denominator(passed,failed),failure_tail=failure,statistics=stats,native_target_support=native,
        completed_B8_target_chain_sha256=chain,verified_batch_bitmap_hex=bitmap,
        bitmap_definition='little bit order ordinal0..19999;1 only after whole actual B8 target hash and sampled post-batch guards pass',
        ROOT_plan_sha256=plan_sha,ROOT_authority_sha256=grant_sha,exact_original_maps_in_archived_ROOT_plan=str(OUTPUT/PLAN.name),
        source_count=len(plan['source_sha256']),input_count=len(plan['input_sha256']),closing_seal_errors=seals,
        whole_source_input_grant_archive_unchanged=not seals,observed_closing_memory=memory,
        current_process_generation=generation,elapsed_seconds_measured_until_result_write=elapsed,
        final_metadata_publication_not_hard_preempted_or_post_write_RSS_measured=True,
        local_provider_identity=plan['environment'],historical_provider_identity=plan['evidence']['original_runtime'],
        historical_environment_equivalence_claimed=False,label_byte_equivalence_proven=complete,
        physical_original_slots=729,physical_paired_FALSE_slots=480,new_images_or_sites=0,
        NN=0,training_steps=0,ROS=False,GPU=False,RGB_TRUE_weights_predictions_DEV_HOLDOUT_TEST_CTrack_bodies_opened=False,
        source_size_unit='whole-frame GT sign pixels; NOT components or physical objects',
        sampler_or_model_causality_proven=False,model_improved=False,model_promoted=False,main_approved=0,sub_approved=0,
        resource_bounds_sampled_not_OS_containment=True,original_27_criteria_unchanged=True)


def empty_statistics():
    return dict(columns=list(COUNTER_COLUMNS),shared_once_overall=empty_counts(),source_groups={g:empty_counts() for g in GROUPS},
        frame_mask_support_bins={b:empty_counts() for b in FRAME_MASK_SUPPORT_BINS},slot_statistics=[empty_counts() for _ in range(TOTAL)],
        statistics_apply_identically_to_both_original_arms_only_after_every_hash_matches=True,
        partial_statistics_cover_PASS_batches_only=True)


def closing_size_proof(plan):
    # HH_261008 - Bound a full1209 populated result plus every closing-seal error before opening any target body.
    maximum=STEPS*BATCH*PIXELS;stats=empty_statistics()
    stats['shared_once_overall']=[maximum]*len(COUNTER_COLUMNS)
    for table in ('source_groups','frame_mask_support_bins'):
        stats[table]={key:[maximum]*len(COUNTER_COLUMNS) for key in stats[table]}
    stats['slot_statistics']=[[maximum]*len(COUNTER_COLUMNS) for _ in range(TOTAL)]
    errors={path:dict(expected='f'*64,actual=None,error=dict(repr_SHA256='f'*64,bytes=2**64-1,prefix_hex='f'*64))
        for path in seal_manifest(plan,'f'*64,'f'*64)}
    failure=dict(stage='x'*64,failed_batch=dict(ordinal=STEPS-1,examined_exposures=BATCH,
        physical_slot_indices=[TOTAL-1]*BATCH,shared_crop_boxes=[[360,640,360,640]]*BATCH,
        actual_target_sha256='f'*64,expected_target_sha256='f'*64,computed_support=[[maximum]*len(COUNTER_COLUMNS)]*BATCH),
        error=dict(repr_SHA256='f'*64,bytes=2**64-1,prefix_hex='f'*64),
        closing_resource_error=dict(repr_SHA256='f'*64,bytes=2**64-1,prefix_hex='f'*64))
    native=dict(sign_pixels_by_slot=[PIXELS]*TOTAL,class_pixels=[maximum]*7,target_hash_chain_sha256='f'*64,
        completed_slots=TOTAL,failed_slot=None,UNEXECUTED_slots=0)
    value=result_value(plan,'f'*64,'f'*64,STEPS-1,1,failure,stats,errors,dict(PID=2**64-1,VmRSS_bytes=2**64-1,
        VmHWM_bytes=2**64-1,maximum_observed_VmHWM_bytes=2**64-1),plan['prepare_process_generation'],
        -1.7976931348623157e308,'f'*64,'f'*(STEPS//4),native)
    size=len(encoded(value));require(size<=META_LIMIT,'pessimistic whole1209 closing metadata exceeds512KiB');return size


def execute(plan_sha,grant_sha):
    plan=authorization(plan_sha,grant_sha);require(not OUTPUT.exists(),'fresh actual result only; never resume/retry')
    with lease() as lock_id:
        require(lock_id==plan['lock_identity'],'owned runtime lock inode changed')
        generation=process_generation();budget=Budget();storage_admission(plan);OUTPUT.mkdir(parents=True,exist_ok=False)
        passed=failed=0;failure=None;current=None;stage='admission';stats=empty_statistics();native=None
        chain=hashlib.sha256();bitmap=bytearray((STEPS+7)//8);targets=None
        try:
            for path,pin in ((PLAN,plan_sha),(AUTHORITY,grant_sha)):
                raw=canonical(path).read_bytes();require(hashlib.sha256(raw).hexdigest()==pin,'ROOT bytes changed before archive')
                write_bytes(OUTPUT,OUTPUT/path.name,raw)
            budget.guard();verify_tables(plan,budget);require(evidence(budget.guard)==plan['evidence'],'original whole chronology/provider metadata changed')
            os.sched_setaffinity(0,{3});require(os.sched_getaffinity(0)=={3},'actual single CPU3 affinity differs')
            budget.strict_affinity=True;budget.guard()
            # HH_261008 - Only external authority, full seals, lease and budgets can reach numeric/image imports.
            import numpy as np
            from PIL import Image
            require(str(Path(np.__file__).resolve(strict=True))==plan['environment']['origins']['numpy']
                and str(Path(sys.modules['PIL'].__file__).resolve(strict=True))==plan['environment']['origins']['PIL'],'actual imported provider differs')
            crop=compile_target_crop(np,Image);stage='approved_TRAIN_target_members';targets=load_targets(np,plan['evidence']['archive_metadata'],budget)
            stage='native1209_target_support';native=dict(sign_pixels_by_slot=[],class_pixels=[0]*7,target_hash_chain_sha256=None,
                completed_slots=0,failed_slot=None,UNEXECUTED_slots=TOTAL)
            signs,pixels,native_chain=native_support(np,targets,budget,native)
            require(pixels==plan['evidence']['source_class_pixels'],'all native TRAIN class support differs')
            for ordinal,row in enumerate(json_lines(RUN/'control/exposures.jsonl')):
                require(ordinal<STEPS,'extra actual exposure row');validate_batch(row,ordinal);stage='batch_start';budget.guard()
                current=dict(ordinal=ordinal,physical_slot_indices=row['physical_slot_indices'],shared_crop_boxes=row['shared_crop_boxes'],
                    expected_target_sha256=row['target_sha256'],actual_target_sha256=None,examined_exposures=0,computed_support=[])
                batch_hash=hashlib.sha256()
                for slot,box in zip(row['physical_slot_indices'],row['shared_crop_boxes']):
                    stage='exact_original_label_crop';budget.guard();target=targets[0][slot] if slot<ORIGINAL else targets[1][slot-ORIGINAL]
                    resized=crop(target,box);require(resized.shape==(360,640) and resized.dtype==np.uint8 and int(resized.max())<7,'original crop target ABI/labels differ')
                    batch_hash.update(memoryview(resized));hist=np.bincount(resized.ravel(),minlength=7).tolist()
                    top,left,height,width=box;source_sign=int(np.count_nonzero(target[top:top+height,left:left+width]==5))
                    current['computed_support'].append(exposure_counts(signs[slot],source_sign,hist,box==[0,0,360,640]))
                    current['examined_exposures']+=1;del resized,hist;budget.guard()
                stage='historical_B8_hash';current['actual_target_sha256']=batch_hash.hexdigest()
                require(current['actual_target_sha256']==row['target_sha256'],'actual historical B8 label bytes differ; no crop reroll/substitution allowed')
                stage='post_batch';budget.guard();require(process_generation()==generation,'current process generation changed')
                stage='transactional_counter_commit';next_stats=commit_counts(stats,row['physical_slot_indices'],current['computed_support'],signs)
                next_chain=chain.copy();next_chain.update(struct.pack('<Q',ordinal+1)+batch_hash.digest())
                next_bitmap=bytearray(bitmap);next_bitmap[ordinal//8]|=1<<(ordinal%8)
                stats,chain,bitmap,passed,current=next_stats,next_chain,next_bitmap,passed+1,None
                if passed%1000==0:print(encoded(dict(stage='verified_label_batches',PASS_batches=passed,expected_batches=STEPS,
                    shared_exposures=passed*BATCH,original_exposures_both_arms=2*passed*BATCH)).decode(),end='',flush=True)
            require(passed==STEPS and chain.hexdigest()==plan['evidence']['journal']['expected_target_chain_sha256'],'whole20k actual B8 target chain/tail required')
            require(stats['shared_once_overall'][0]==STEPS*BATCH and [r[0] for r in stats['slot_statistics']]==plan['evidence']['journal']['slot_exposures']
                and stats['shared_once_overall'][1]==plan['evidence']['journal']['whole'] and stats['shared_once_overall'][2]==plan['evidence']['journal']['partial'],'whole160k support/exposure census differs')
            require(sum(stats['shared_once_overall'][11:])==STEPS*BATCH*PIXELS,'whole seven-label resized denominator differs')
            stage='closing';verify_tables(plan,budget,unloaded=False)
        except BaseException as error:
            failed=int(current is not None);failure=dict(stage=stage,error=bounded_error(error),failed_batch=current,resources=budget.last)
        finally:
            targets=None  # HH_261008 - Drop only owned in-memory targets; no source/raw/output deletion.
            seals=closing_seals(plan,plan_sha,grant_sha)
            if seals:
                if failure is None:failure=dict(stage='closing_seals',failed_batch=None)
                failure['closing_seal_error_count']=len(seals)
            try:
                closing_memory=process_memory(budget.parser);budget.maximum_HWM=max(budget.maximum_HWM,closing_memory['VmHWM_bytes'])
                closing_memory['maximum_observed_VmHWM_bytes']=budget.maximum_HWM
                require(budget.maximum_HWM<=RSS and time.monotonic()-budget.started<DEADLINE,'closing sampled RSS/deadline exceeded')
                require(process_generation()==generation,'closing current PID generation changed')
                require(environment(unloaded=False)==plan['environment'],'closing selected provider bytes changed')
            except BaseException as error:
                closing_memory=locals().get('closing_memory',dict(last_observed=budget.last))
                if failure is None:failure=dict(stage='closing_resources',failed_batch=None)
                failure['closing_resource_error']=bounded_error(error)
            result=result_value(plan,plan_sha,grant_sha,passed,failed,failure,stats,seals,closing_memory,generation,
                time.monotonic()-budget.started,chain.hexdigest(),bitmap.hex(),native)
            write_new(OUTPUT,OUTPUT/'result.json',result,closing=True)
        return dict(status=result['status'],PASS_batches_per_arm=passed,FAIL_batches_per_arm=failed,
            UNEXECUTED_batches_per_arm=STEPS-passed-failed,path=str(OUTPUT/'result.json'))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--mode',choices=('prepare','execute'),required=True)
    parser.add_argument('--plan-sha256');parser.add_argument('--authority-sha256');args=parser.parse_args(argv)
    value=prepare() if args.mode=='prepare' else execute(args.plan_sha256,args.authority_sha256)
    print(encoded(value).decode(),end='');return 0 if value['status'] in ('CANDIDATE_ONLY_NOT_ROOT_AUTHORITY','COMPLETE_ACTUAL_TRAIN_LABEL_SUPPORT_ONLY') else 2


if __name__=='__main__':raise SystemExit(main())
