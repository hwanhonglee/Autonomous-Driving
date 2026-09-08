#!/usr/bin/env python3
"""HH_260906 - Publish exact real-fit aggregates and one measured cost plot without exposing private trace/checkpoint bytes."""

from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

ROOT=Path(__file__).resolve().parents[3]
BASE=ROOT/'artifacts/training/2026-09-09'
INPUT=BASE/'training_profile_v1'
PARENT=ROOT/'docs/assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/02_overnight_data_and_learning/03_training_cost'
OUTPUT=PARENT/'actual_training_profile'
PINS={'report.json':'3a3e0baad18c66fdd1b1053c01b571eb487206a235eb875c6ac00ffd6586d151',
 'key_averages.json':'8dfe62d9ba98a7725f5e6dc55557fef2d17107a472b9029d10cdd2d163cc8abf',
 'profile_analysis.json':'a19444e11a2417913b424d342c3488c380e303a4437133448efa03ccfdd0d1ae',
 'transport_verification.json':'ad4e1afe0fed76c4782d464bf143c1c963a49d4849c310d1ecc9286efe2c1397',
 'completion_observation.json':'ed2234f5890bfc648824c8a924adcc835b317ed878d6c6a414c96d0c52dfbb8f',
 'LOCAL_SHA256SUMS':'1ced297ecb912e6d479d2c8377d14f3c8080db956a125b83d40896a50df73cce'}
COPIES=('report.json','key_averages.json','profile_analysis.json','transport_verification.json','completion_observation.json',
 'plan.json','A_unprofiled/metrics.jsonl','B_cpu_cuda_profiled/metrics.jsonl','A_unprofiled/run.json','B_cpu_cuda_profiled/run.json')

def sha(raw):return hashlib.sha256(raw).hexdigest()
def regular(p):
 assert p.is_file() and not any(v.is_symlink() for v in (p,*p.parents)), 'unsafe input'
 return p
def read(p):return regular(p).read_bytes()
def snapshot_parent():
 return {str(p.relative_to(PARENT)):sha(read(p)) for p in PARENT.rglob('*') if p.is_file() and not p.is_relative_to(OUTPUT)}
def safe_text(raw):
 text=raw.decode()
 # HH_260906 - Group home/root alternatives so the detector does not mistake its own literal regex for a disclosed path.
 assert not re.search(r'/(?:home/[A-Za-z0-9_.-]+|root)/|(?<![\w.])(?:\d{1,3}\.){3}\d{1,3}(?![\w.])|BEGIN [A-Z ]*PRIVATE KEY',text), 'private account/path/credential'
 return text
def write(p,raw):
 p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('xb') as stream:stream.write(raw)
def js(p,value):write(p,(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n').encode())
def main():
 assert not OUTPUT.exists(), 'new category only'
 old=snapshot_parent(); source=read(Path(__file__))
 for name,pin in PINS.items():assert sha(read(INPUT/name))==pin,name
 local={}
 for line in read(INPUT/'LOCAL_SHA256SUMS').decode().splitlines():
  pin,name=line.split('  ',1)
  assert name not in local and '..' not in Path(name).parts and not Path(name).is_absolute()
  assert sha(read(INPUT/name))==pin,name
  local[name]=pin
 actual={str(p.relative_to(INPUT)) for p in INPUT.rglob('*') if p.is_file() and p.name!='LOCAL_SHA256SUMS'}
 assert actual==set(local), 'local complete inventory changed'
 report=json.loads(read(INPUT/'report.json')); analysis=json.loads(read(INPUT/'profile_analysis.json'))
 assert report['status']=='PROFILE_COMPLETE_EXACT_PARITY_NOT_PROMOTED'
 assert report['checkpoint_parity']['exact'] and report['metrics_parity']['exact']
 assert report['checkpoint_parity']['ignored_top_level_fields']==['created_at_utc']
 assert analysis['metrics']['retained_rows']==32 and analysis['metrics']['exact_history_bytes_equal']
 assert all(a['state']['global_step']==16 and a['state']['samples_seen']==64 for a in report['arms'])
 assert read(INPUT/'A_unprofiled/metrics.jsonl')==read(INPUT/'B_cpu_cuda_profiled/metrics.jsonl')
 assert report['approval']==dict(data_admission=False,model_promotion=False,production_source_changes=False,
  test_neural_network_use=False,vehicle_control_approved=False)
 OUTPUT.mkdir()
 records={}
 def copy(src,dest):
  raw=read(src);safe_text(raw);write(OUTPUT/dest,raw)
  records[dest]={'raw_source_path':str(src.relative_to(ROOT)),'raw_source_sha256':sha(raw),
   'public_sha256':sha(raw),'bytes':len(raw),'transformation':'none; exact bytes; account/IP scan clear'}
 for name in COPIES:copy(INPUT/name,'raw/'+name)
 copy(INPUT/'SHA256SUMS','remote_manifest_original.sha256')
 copy(INPUT/'source/independent_profiler.py','source/executed_profiler.py')
 copy(BASE/'training_profile_v1_public_README.md','README.md')
 copy(Path(__file__),'source/publish_training_profile_evidence.py')
 rows=json.loads(read(INPUT/'key_averages.json'))['rows']
 cpu=[r for r in rows if r['device_type']=='DeviceType.CPU']
 by_name={r['key']:r for r in cpu};assert len(by_name)==len(cpu)
 names=('enumerate(DataLoader)#_SingleProcessDataLoaderIter.__next__','cudaLaunchKernel','aten::fill_',
  'aten::empty','aten::mul','aten::copy_')
 labels=['CPU batch preparation (DataLoader)','Host CUDA kernel launch','Tensor fill','Tensor allocation',
  'Elementwise multiplication','Tensor copy','All other CPU-self categories']
 values=[by_name[n]['self_cpu_time_total_us']/1000 for n in names]
 values.append(sum(r['self_cpu_time_total_us'] for r in cpu)/1000-sum(values))
 assert all(v>=0 for v in values)
 fig,ax=plt.subplots(figsize=(16,9),dpi=100)
 fig.subplots_adjust(left=.31,right=.93,top=.80,bottom=.23)
 bars=ax.barh(labels,values,color=['#326ca8','#a85129',*(['#6c9aaf']*4),'#808080'])
 ax.invert_yaxis();ax.set_xlim(0,max(values)*1.17)
 ax.bar_label(bars,labels=[f'{v:,.3f} ms' for v in values],padding=7,fontsize=11)
 ax.grid(axis='x',alpha=.2);ax.set_axisbelow(True)
 ax.set_xlabel('Attributed CPU self time across the recorded 16-step context (milliseconds)',fontsize=11)
 fig.suptitle('Actual unchanged training: CPU preparation and launch costs',fontsize=19,y=.94)
 fig.text(.5,.865,'B only: 16 fresh optimizer updates, batch 4, original physical model, num_workers=0',ha='center',fontsize=12)
 fig.text(.5,.12,'Not GPU utilization, not whole-wall time, and not an A/B speed improvement.',ha='center',fontsize=13)
 fig.text(.5,.075,'CPU inclusive operators and CUDA events may overlap. All original safety checks remain enabled.',ha='center',fontsize=11)
 image=io.BytesIO();fig.savefig(image,format='png');plt.close(fig)
 png=image.getvalue()
 with Image.open(io.BytesIO(png)) as im:im.load();assert im.size==(1600,900)
 write(OUTPUT/'01_measured_cpu_self_cost.png',png)
 provenance={'schema':'portable_e2e.actual_training_profile_publication.v1',
  'comment':'HH_260906 - Separate original source, profiler instrumentation and this read-only publication.',
  'published_at_utc':datetime.now(timezone.utc).isoformat(),'publisher_sha256':sha(source),
  'training_source_commit':report['source_commit'],'executed_profiler_sha256':report['source_sha256']['independent_profiler.py'],
  'raw_input_pins':PINS,'copied_files':records,'redacted_file_count':0,
  'redaction_notice':'Selected raw text inputs contain no account-specific absolute paths; they were copied exactly. No derivative wrappers or hidden redactions were applied.',
  'plot':{'path':'01_measured_cpu_self_cost.png','sha256':sha(png),'pixels':[1600,900],
   'input_sha256':PINS['key_averages.json'],'ordered_labels':labels,'ordered_CPU_self_milliseconds':values,
   'method':'Six explicitly named CPU-self rows plus the residual sum of all other CPU-self rows. No CUDA device-time addition or utilization calculation.',
   'matplotlib_version':matplotlib.__version__},
  'diagnostic_fit_count':2,'optimizer_steps_per_fit':16,'sample_exposures_per_fit':64,
  'full_1540_step_campaign_fits_added':0,'published_trace_or_checkpoints':False,
  'remote_omissions':json.loads(read(INPUT/'transport_verification.json'))['omitted_from_mirror'],
  'parent_original_file_hashes':old,'parent_original_files_unchanged':True,'approval':report['approval']}
 assert snapshot_parent()==old, 'parent evidence changed'
 for name,pin in local.items():assert sha(read(INPUT/name))==pin,name
 assert read(Path(__file__))==source
 js(OUTPUT/'provenance.json',provenance)
 for p in OUTPUT.rglob('*'):
  if p.is_file() and p.suffix in ('.json','.jsonl','.md','.py'):
   safe_text(read(p))
   if p.suffix=='.json':json.loads(read(p))
 sums=''.join(sha(read(p))+'  '+str(p.relative_to(OUTPUT))+'\n' for p in sorted(OUTPUT.rglob('*')) if p.is_file())
 write(OUTPUT/'SHA256SUMS',sums.encode())
 assert snapshot_parent()==old
 print(json.dumps({'status':'PUBLISHED_EXACT_PROFILE_EVIDENCE_NOT_PROMOTED','files':len(list(OUTPUT.rglob('*'))),
  'parent_original_files_unchanged':len(old),'provenance_sha256':sha(read(OUTPUT/'provenance.json'))}))
if __name__=='__main__':main()
