#!/usr/bin/env python3
"""HH_260906 - Recount saved TRAIN oracle costs without importing a model or reading original datasets."""
import argparse
from collections import Counter, defaultdict
from copy import deepcopy
from datetime import datetime
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[3]
ART=ROOT/'artifacts/training/2026-09-09'
RAW=ART/'stopmix_training_oracles_v1'
OUTPUT=ROOT/'docs/assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/02_overnight_data_and_learning/09_train_oracle_diagnosis'
PINS={'report.json':'cc97748804dcb433973335f7b6c1ea3fb1d5118ba6247306a388aa88d42970fc',
    'SHA256SUMS':'d0a678b3a7dfebe09754de92dda6f5f5c4d7d3794a4588811a80b9d6353886b0',
    'plan.json':'a2f4ac813a99155026ee23cb9f1002bdcdb0d810a0b901238e65c1c5e7e097a4',
    'transport_verification.json':'5e33ec6c7b64e05b6ba17c4d09558c8afbbe73ccc5929b5bde2cc9ec0d91e582'}
EXTRA={'stopmix_training_oracles_plan_preparation_v1.json':'3ae3c79556c085ab624c2a616ab541b615846e545e49f9e7b9f052e024ac06c0',
    'stopmix_training_oracles_plan_v1.json':'5c4a5571be978ca6fbe638002fadca304f4f007775f54c78b2bc62ca7be535a4',
    'stopmix_training_oracles_plan_v2.json':PINS['plan.json'],
    'stopmix_training_oracles_v1_launch.json':'f45efde6ca00efe5368369695c799694be42fcd697fcd3803088ae38654a5db0',
    'stopmix_training_oracles_v1_ssh_completion.json':'3b1afdf184e11097416f880a617d38c53d7dc6d6b25ba47a1195d787fda10cbd'}
WORKER_SHA='35d3d0a464fb43c9789f6e24a35e453736c99fdceb36e3292d085c5ceb960d37'
SOURCE_COMMIT='b478f02e42b94bf04bffec5c8170e05edc33b0f8'
SEEDS=('20260903','20260904','20260905')
ENDPOINTS=('parent1540','continued2870')
GROUPS=('unavailable_masks','stationary_hold','moving_to_stop','continuing_motion','other_motion')
MEANS=('selected_ade_m','composite_oracle_ade_m','oracle_ade_m','selected_composite_regret','second_minus_first_composite_cost')
IDENTITY=('sample_id','episode_id','sequence_index','anchor_timestamp_ns','model_input_sha256','target_sha256',
          'raw_current_vx_mps','target_motion_group','capture_phase','camera_sha256','original_target_valid','raw_target_speed_mps')
DENIALS=('model_training','optimizer_created','validation_neural_inference','test_neural_inference',
    'test_used_for_selection','model_promotion','training_data_approved','vehicle_control_approved',
    'source_or_label_changes','near_tie_tolerance_used')


def require(value,message):
    if not value:raise ValueError(message)


def sha(raw):return hashlib.sha256(raw).hexdigest()


def regular(path):
    require(path.is_file() and not any(p.is_symlink() for p in (path,*path.parents)),'regular nonsymlink file required')
    return path


def digest(path):return sha(regular(path).read_bytes())


def decode(raw):
    def pairs(items):
        result={}
        for k,v in items:require(k not in result,'duplicate JSON key');result[k]=v
        return result
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=lambda _:(_ for _ in()).throw(ValueError('nonfinite JSON')))


def encoded(value):return (json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n').encode()


def safe_name(name):
    require(type(name)is str and name and not Path(name).is_absolute() and '..' not in Path(name).parts
            and Path(name).as_posix()==name,'unsafe relative evidence path')
    return name


def same(a,b,message):
    require(type(a)is type(b),message)
    if isinstance(a,list):
        require(len(a)==len(b),message)
        for x,y in zip(a,b):same(x,y,message)
    elif isinstance(a,dict):
        require(set(a)==set(b),message)
        for key in a:same(a[key],b[key],message)
    else:require(a==b,message)


def finite(value):return type(value)in(int,float) and math.isfinite(value)


def hexsha(value):return type(value)is str and re.fullmatch('[0-9a-f]{64}',value)is not None


def original_inventory():
    payload={p.relative_to(RAW).as_posix():regular(p).read_bytes() for p in RAW.rglob('*') if p.is_file()or p.is_symlink()}
    require(len(payload)==27,'exact26 originals plus transport required')
    for name,pin in PINS.items():require(sha(payload[name])==pin,'pinned oracle evidence differs')
    manifest={}
    for line in payload['SHA256SUMS'].decode().splitlines():
        pin,name=line.split('  ');safe_name(name)
        require(name not in manifest and name not in ('SHA256SUMS','transport_verification.json'),'invalid original manifest')
        require(sha(payload[name])==pin,'original payload checksum failed');manifest[name]=pin
    require(len(manifest)==25 and set(manifest)==set(payload)-{'SHA256SUMS','transport_verification.json'},'original25 payload denominator differs')
    transport=decode(payload['transport_verification.json'])
    require(transport['status']=='VERIFIED_SIX_TRAIN_PASSES_NOT_PROMOTED' and transport['omitted_files']==[]
        and transport['ordered_sample_rows']==6882 and transport['all_original_sha_payloads_pass']==25,'transport completion differs')
    for flag in ('remote_pre_post_bytes_unchanged','source_and_parent_checkpoint_bytes_unchanged',
                 'cooperative_gpu0_lease_free','all_six_input_target_identities_equal'):require(transport[flag]is True,'transport proof missing')
    for flag in ('checkpoint_tensors_loaded_by_collector','checkpoint_tensors_transferred','model_promotion','data_admission'):
        require(transport[flag]is False,'transport scope changed')
    expected={n:dict(sha256=sha(v),size_bytes=len(v)) for n,v in payload.items()if n!='transport_verification.json'}
    require(transport['regular_file_inventory']==expected,'transport bytes/inventory differ')
    require(sum(v['size_bytes']for v in expected.values())==28306546,'original26 byte total differs')
    extras={name:regular(ART/name).read_bytes()for name in EXTRA}
    require(all(sha(extras[n])==pin for n,pin in EXTRA.items()),'preparation/execution receipt changed')
    return payload,extras


def motion_group(vx,speed,mask):
    require(finite(vx)and len(speed)==len(mask)==64 and all(type(m)is bool for m in mask),'full64 original masks required')
    require(all(finite(v)if m else v is None for v,m in zip(speed,mask)),'finite valid target or masked null required')
    if not all(mask):return 'unavailable_masks'
    if abs(vx)<=.1 and all(v<=.1 for v in speed):return 'stationary_hold'
    if vx>.1 and all(v<=.1 for v in speed[-10:]):return 'moving_to_stop'
    if all(v>=.5 for v in speed):return 'continuing_motion'
    return 'other_motion'


def check_row(row,index,phase_metadata):
    same(row['index'],index,'ordered unique row index required')
    for name in ('sequence_index','anchor_timestamp_ns'):
        require(type(row[name])is int and row[name]>=0,'integer sample identity required')
    for name in ('model_input_sha256','target_sha256','source_manifest_sha256'):require(hexsha(row[name]),'input identity SHA required')
    require(len(row['camera_sha256'])==6 and all(hexsha(v)for v in row['camera_sha256']),'six original image hashes required')
    require(finite(row['raw_current_vx_mps'])and finite(row['model_tensor_current_vx_mps']),'finite original current velocity required')
    require(row['candidate_cost_dtype']=='torch.float32' and row['native_oracle_and_regression_exact']is True,'native source-bound oracle comparison required')
    arrays=[row[k]for k in ('candidate_composite_costs','candidate_logits','candidate_ade_m','candidate_probabilities')]
    require(all(len(a)==12 and all(finite(x)for x in a)for a in arrays),'all12 finite costs/logits/ADE/probabilities required')
    costs,logits,ade,prob=arrays
    require(all(v>=0 for v in ade)and all(0<=p<=1 for p in prob)and abs(sum(prob)-1)<1e-6,'invalid ADE/probability values')
    minimum=min(costs);best=costs.index(minimum);selected=max(range(12),key=lambda k:logits[k]);ties=[i for i,c in enumerate(costs)if c==minimum]
    expected=dict(composite_oracle_index=best,exact_minimum_indices=ties,exact_minimum_count=len(ties),
        minimum_composite_cost=minimum,second_minus_first_composite_cost=sorted(costs)[1]-minimum,
        selected_candidate_index=selected,selected_in_exact_minimum=selected in ties,
        selected_composite_regret=costs[selected]-minimum,ade_oracle_index=ade.index(min(ade)),
        selected_ade_m=ade[selected],composite_oracle_ade_m=ade[best],oracle_ade_m=min(ade),
        raw_current_vx_nonpositive=row['raw_current_vx_mps']<=0)
    for k,v in expected.items():same(row[k],v,'stored oracle arithmetic mismatch: '+k)
    group=motion_group(row['raw_current_vx_mps'],row['raw_target_speed_mps'],row['original_target_valid'])
    same(row['target_motion_group'],group,'future-derived group mismatch')
    phases=[p for p,(a,b)in phase_metadata[row['episode_id']].items()if a<=row['anchor_timestamp_ns']<=b]
    require(len(phases)<=1,'overlapping original phase metadata')
    same(row['capture_phase'],phases[0]if phases else'unmatched','native phase identity mismatch')
    return row


def aggregate(rows):
    n=len(rows)
    return dict(sample_count=n,selected_in_exact_minimum_count=sum(r['selected_in_exact_minimum']for r in rows),
        multiple_exact_minimum_count=sum(r['exact_minimum_count']>1 for r in rows),
        raw_vx_nonpositive_count=sum(r['raw_current_vx_mps']<=0 for r in rows),
        selected_composite_oracle_index_agreement_count=sum(r['selected_candidate_index']==r['composite_oracle_index']for r in rows),
        composite_ade_oracle_index_agreement_count=sum(r['composite_oracle_index']==r['ade_oracle_index']for r in rows),
        selected_histogram={str(i):sum(r['selected_candidate_index']==i for r in rows)for i in range(12)},
        composite_oracle_histogram={str(i):sum(r['composite_oracle_index']==i for r in rows)for i in range(12)},
        means={k:sum(r[k]for r in rows)/n if n else None for k in MEANS})


def group_summary(rows):
    result={'all_samples':aggregate(rows)}
    for key in ('episode_id','target_motion_group','capture_phase'):
        categories=GROUPS if key=='target_motion_group'else sorted({r[key]for r in rows})
        result[key]={v:aggregate([r for r in rows if r[key]==v])for v in categories}
    result['raw_nonpositive']=aggregate([r for r in rows if r['raw_current_vx_mps']<=0])
    return result


def input_ambiguities(rows):
    groups=defaultdict(list)
    for row in rows:groups[row['model_input_sha256']].append(row)
    pairs=[];duplicate_groups=[]
    for key,items in groups.items():
        if len(items)>1:duplicate_groups.append([r['index']for r in items])
        for i,a in enumerate(items):
            for b in items[i+1:]:
                same(a['candidate_logits'],b['candidate_logits'],'same saved model input has different logits')
                # HH_260906 - Tensor hashes and saved camera hashes are witnesses, not a new decoding or sensor-cause proof.
                same(a['camera_sha256'],b['camera_sha256'],'same-input image-byte witness differs')
                if a['target_sha256']==b['target_sha256']or set(a['exact_minimum_indices'])&set(b['exact_minimum_indices']):continue
                pairs.append(dict(indices=[a['index'],b['index']],model_input_sha256=key,camera_sha256=a['camera_sha256'],
                    target_sha256=[a['target_sha256'],b['target_sha256']],sample_ids=[a['sample_id'],b['sample_id']],
                    episode_sequence_indices=[a['sequence_index'],b['sequence_index']],
                    episode_ids=[a['episode_id'],b['episode_id']],oracle_sets=[a['exact_minimum_indices'],b['exact_minimum_indices']],
                    candidate_logits=a['candidate_logits'],anchor_timestamp_ns=[a['anchor_timestamp_ns'],b['anchor_timestamp_ns']]))
    return dict(duplicate_input_groups=duplicate_groups,disjoint_teacher_pairs=pairs,pair_count=len(pairs),
        affected_row_count=len({i for p in pairs for i in p['indices']}),
        interpretation='Identical saved six-input tensor hashes and logits with different target hashes/disjoint minimum sets. This does not prove incorrect labels, traffic-light state, a sole error cause, or future predictability.')


def teacher_changes(a,b):
    require(len(a)==len(b),'paired teacher row counts differ')
    require(all(all(x[k]==y[k]for k in IDENTITY)for x,y in zip(a,b)),'same ordered input/target identity required')
    changes=[i for i,(x,y)in enumerate(zip(a,b))if x['composite_oracle_index']!=y['composite_oracle_index']]
    disjoint=[i for i,(x,y)in enumerate(zip(a,b))if not set(x['exact_minimum_indices'])&set(y['exact_minimum_indices'])]
    return dict(sample_count=len(a),oracle_index_changed_count=len(changes),oracle_index_changed_indices=changes,
        exact_minimum_sets_disjoint_count=len(disjoint),exact_minimum_sets_disjoint_indices=disjoint,
        by_episode={ep:dict(sample_count=sum(r['episode_id']==ep for r in a),changed_count=sum(a[i]['episode_id']==ep for i in changes))for ep in sorted({r['episode_id']for r in a})})


def authenticate(payload,extras):
    plan,report=decode(payload['plan.json']),decode(payload['report.json'])
    require(plan['schema']=='portable_e2e.stopmix_training_oracle_plan.v1' and report['schema']=='portable_e2e.stopmix_training_oracle_probe.v1','schema mismatch')
    require(report['status']=='SIX_TRAIN_PASSES_COMPLETE_NOT_PROMOTED' and report['completed_pass_count']==6,'complete six-pass report required')
    require(report['postcheck_errors']==[] and all(report[k]is True for k in ('deadline_met','source_and_original_inputs_unchanged','all_six_ordered_model_inputs_and_targets_identical')),'recorded completion proof failed')
    require(all(plan[k]is False and report[k]is False for k in DENIALS),'scope denial changed')
    require(report['plan_sha256']==PINS['plan.json'] and plan['source_commit']==SOURCE_COMMIT
        and plan['worker_source_sha256']==WORKER_SHA and plan['evaluation_split']=='train','executed source/split differs')
    require((plan['expected_pass_count'],plan['expected_samples_per_pass'],plan['expected_training_episodes'],plan['candidate_count'],plan['future_points'])==(6,1147,3,12,64),'fixed denominator differs')
    expected=dict(plan['source_sha256'],**{'independent_oracle_probe.py':WORKER_SHA,'frozen_helper.py':plan['frozen_helper_sha256']})
    require(len(expected)==17 and report['source_sha256']==expected,'all17 source identities required')
    for name,pin in expected.items():safe_name(name);require(sha(payload['source/'+name])==pin,'archived source differs')
    require(len(report['original_input_sha256'])==29 and all(hexsha(v)for v in report['original_input_sha256'].values()),'29 parent input witnesses required')
    require(all(report['original_input_sha256'][n]==v for n,v in plan['original_receipt_sha256'].items()),'original parent receipts differ')
    projection=report['loss_projection'];require(projection==plan['loss_projection'],'native loss projection differs')
    loss=payload['source/portable_e2e/losses.py'];require(sha(loss)==projection['original_loss_sha256'],'loss bytes differ')
    prefix=b''.join(loss.splitlines(keepends=True)[projection['prefix_first_line']-1:projection['prefix_last_line']])
    require(sha(prefix)==projection['prefix_source_sha256'],'native cost prefix bytes differ')
    preparation=decode(extras['stopmix_training_oracles_plan_preparation_v1.json']);old=decode(extras['stopmix_training_oracles_plan_v1.json'])
    require(preparation['unexecuted_preparation']['gpu_execution_started']is False and preparation['prospective_execution_plan']['sha256']==PINS['plan.json'],'unexecuted plan relabeled')
    normalized_old=deepcopy(old);normalized_plan=deepcopy(plan)
    for data in (normalized_old,normalized_plan):
        data.pop('declared_at_utc')
        for k in ('original_function_ast_sha256','derived_ast_sha256'):data['loss_projection'].pop(k)
    require(normalized_old==normalized_plan,'plan changes exceed declared interpreter AST/time difference')
    launch=decode(extras['stopmix_training_oracles_v1_launch.json']);finished=decode(extras['stopmix_training_oracles_v1_ssh_completion.json'])
    for receipt in (launch,finished):require(receipt['plan_sha256']==PINS['plan.json']and receipt['source_sha256']==WORKER_SHA,'launch/completion pin mismatch')
    require(finished['exit_code']==0 and finished['error_type']is None,'execution did not finish cleanly')
    time=lambda s:datetime.fromisoformat(s.replace('Z','+00:00'))
    require(time(plan['declared_at_utc'])<=time(launch['started_at_utc'])<=time(report['started_at_utc'])
        <=time(report['completed_at_utc'])<=time(finished['completed_at_utc'])<=time(plan['finish_before_utc']),'execution chronology mismatch')
    order=[(s,e)for s in SEEDS for e in ENDPOINTS];require([(p['seed'],p['endpoint'])for p in report['passes']]==order,'all6 fixed pass order required')
    passes=[];all_rows=[]
    for item,(seed,endpoint)in zip(report['passes'],order):
        name=f'seed_{seed}/{endpoint}/samples.jsonl';raw=payload[name]
        require(item['status']=='PASS_COMPLETE_NOT_PROMOTED'and item['sample_count']==1147 and item['samples_file']==name
            and item['samples_sha256']==sha(raw)and raw.endswith(b'\n'),'complete per-pass row binding differs')
        require(item['checkpoint_sha256']==plan['checkpoint_sha256'][seed][endpoint]==item['checkpoint_validation']['checkpoint_sha256'],'checkpoint binding differs')
        rows=[check_row(decode(line),i,report['capture_phase_metadata'])for i,line in enumerate(raw.splitlines())]
        require(len(rows)==1147 and len({r['sample_id']for r in rows})==1147 and len({r['episode_id']for r in rows})==3,'1147 unique TRAIN rows/3 episodes required')
        summary=group_summary(rows);require(summary==item['summary'],'saved group/count/mean summary differs')
        ties=[r for r in rows if r['exact_minimum_count']>1]
        passes.append(dict(seed=seed,endpoint=endpoint,samples_sha256=sha(raw),checkpoint_sha256=item['checkpoint_sha256'],
            summary=summary,ambiguity=input_ambiguities(rows),
            exact_tie_observation=dict(count=len(ties),all_stop6_to11=all(r['exact_minimum_indices']==list(range(6,12))for r in ties),
                all_selected6=all(r['selected_candidate_index']==6 for r in ties),all_zero_regret=all(r['selected_composite_regret']==0 for r in ties))))
        all_rows.append(rows)
    require(all(all(all(a[k]==b[k]for k in IDENTITY)for a,b in zip(all_rows[0],rows))for rows in all_rows[1:]),'six input identities changed')
    # HH_260906 - Bind every fixed README finding to the full observed denominator rather than a selected example.
    for item in passes:
        require({k:v['sample_count']for k,v in item['summary']['target_motion_group'].items()}==dict(zip(GROUPS,(0,118,202,611,216))),
                'fixed full-group publication finding differs')
        require(item['summary']['capture_phase']['stationary_warmup']['sample_count']==105,'warmup overlap finding differs')
        require(item['ambiguity']['pair_count']==11 and item['ambiguity']['affected_row_count']==22,'all11 ambiguity pairs required')
        require(all(item['exact_tie_observation'][k]is True for k in ('all_stop6_to11','all_selected6','all_zero_regret')),'exact-tie publication finding differs')
    require(len(report['teacher_changes'])==3,'all3 teacher comparisons required')
    changes=[]
    for i,seed in enumerate(SEEDS):
        change=teacher_changes(all_rows[2*i],all_rows[2*i+1]);record=report['teacher_changes'][i]
        require(record['seed']==seed and all(record[k]==v for k,v in change.items()),'saved teacher changes differ')
        changes.append(dict(seed=seed,**change))
    return dict(schema='portable_e2e.independent_saved_train_oracle_recount.v1',status='SAVED_ROWS_VERIFIED_NOT_PROMOTED',
        samples_per_pass=1147,pass_count=6,total_rows=6882,total_cost_values=6882*12,source_commit=SOURCE_COMMIT,
        original_report_sha256=PINS['report.json'],executed_plan_sha256=PINS['plan.json'],passes=passes,teacher_changes=changes,
        native_loss_forward_reexecuted=False,original_dataset_or_image_files_read=False,checkpoint_tensors_loaded=False,
        original_input_proof_scope='29 original parent file SHA witnesses authenticated by the completed remote transport record; no re-read during this saved-result audit.',
        native_oracle_proof_scope='All saved native_oracle_and_regression_exact flags, archived exact loss-prefix/source bytes and executed report bound; only saved scalar arithmetic is independently recomputed here.',
        interpreter_notice='The executed Python3.12 AST hashes are retained; no replacement with local Python3.10 AST hashes. Earlier v1 was unexecuted preparation.',
        **{k:False for k in DENIALS})


def redact(value):
    if isinstance(value,str):return re.sub(r'/(?:home)/[^/\s\"\']+',lambda _:'${USER_HOME}',value)
    if isinstance(value,list):return [redact(v)for v in value]
    if isinstance(value,dict):return {redact(k):redact(v)for k,v in value.items()}
    return value


def new_bytes(path,raw):
    require(not path.exists()and not any(p.is_symlink()for p in (path,*path.parents)),'create-only regular output required')
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('xb')as stream:stream.write(raw)


def privacy_links(root):
    links=0
    for path in root.rglob('*'):
        if not path.is_file()or path.suffix=='.png':continue
        text=path.read_text();require(not re.search(r'/(?:home/[^/\s]+|tmp|root)/',text),'private absolute path')
        for candidate in re.findall(r'(?<![\d.])(?:\d{1,3}\.){3}\d{1,3}(?![\d.])',text):
            try:ipaddress.IPv4Address(candidate)
            except ipaddress.AddressValueError:continue
            raise ValueError('private IPv4')
        if path.suffix=='.md':
            for target in re.findall(r'\]\(([^)]+)\)',text):
                require(not target.startswith(('http:','https:','file:'))and(path.parent/target).resolve().is_file(),'broken/local-only link');links+=1
    return links


def chart(summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,5.6),dpi=150,layout='constrained')
    x=list(range(6));labels=[p['seed'][-2:]+' / '+p['endpoint'].removeprefix('parent').removeprefix('continued')for p in summary['passes']]
    means=[p['summary']['all_samples']['means']for p in summary['passes']]
    axes[0].bar([i-.18 for i in x],[r['selected_ade_m']for r in means],.36,label='Selected ADE')
    axes[0].bar([i+.18 for i in x],[r['composite_oracle_ade_m']for r in means],.36,label='Composite-oracle ADE')
    axes[0].set_ylabel('Mean 6.4 s ADE (m)');axes[0].legend()
    axes[1].bar(x,[r['selected_composite_regret']for r in means],color='#a85f22');axes[1].set_ylabel('Mean selected composite regret (loss units)')
    for ax in axes:ax.set_xticks(x,labels,rotation=35);ax.set_xlabel('Seed suffix / checkpoint update');ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('Saved TRAIN diagnosis: 1,147 rows per checkpoint; no new fit or promotion')
    from io import BytesIO
    stream=BytesIO();fig.savefig(stream,format='png');plt.close(fig)
    return stream.getvalue(),dict(matplotlib_version=matplotlib.__version__,matplotlib_init_sha256=digest(Path(matplotlib.__file__)),
        fixed_pass_order=True,native_tensor_reexecution=False)


def readme(summary):
    text=['# TRAIN oracle 진단 — 저장된 6개 체크포인트 결과', '',
        '<!-- HH_260906 - Separate stored-cost diagnosis from new fitting, causal claims, and dataset admission. -->','',
        '기존 STOPMIX 3seed의 1540/2870 update 체크포인트를 TRAIN 1,147행씩 진단했습니다. 이 게시 검증은 저장된 **6,882행·82,584개 cost**만 다시 계산했으며 모델·optimizer·원본 데이터·이미지를 실행하거나 읽지 않았습니다. 모델 승격·새 데이터 승인도 없습니다.','',
        '| seed | update | selected ADE m | composite oracle ADE m | mean regret (loss units) | exact tie rows |',
        '| --- | ---: | ---: | ---: | ---: | ---: |']
    for p in summary['passes']:
        m=p['summary']['all_samples']['means'];text.append(f"| {p['seed']} | {p['endpoint'].removeprefix('parent').removeprefix('continued')} | {m['selected_ade_m']:.6f} | {m['composite_oracle_ade_m']:.6f} | {m['selected_composite_regret']:.6f} | {p['exact_tie_observation']['count']} |")
    text+=['','![6개 저장 결과의 ADE와 regret](train_oracle_costs.png)','',
        '각 1,147행의 future-derived 분류는 stationary hold118 / moving-to-stop202 / continuing611 / other216 / unavailable0입니다. Warmup105는 별도 phase 분류로 위 그룹과 겹치므로 더하지 않습니다. 이는 미래 정답을 보고 나눈 진단 그룹이며 모델의 causal 입력이나 실제 신호등 상태가 아닙니다.','',
        '동률 판정은 저장된 float32 cost의 정확한 equality만 사용했습니다. 최소값 동률은 모두 STOP 후보6–11이며 이미 후보6이 선택돼 regret0입니다. 따라서 이 동률만으로 현재 선택 오차를 설명하지 않습니다.','',
        '같은 모델 입력 hash를 가진 Town01 11쌍(22행)은 원본6장 JPEG SHA와 저장된 logits도 같지만 target hash와 최소-cost 후보 집합이 다릅니다. 해당 관측 입력만 받는 결정적 선택기는 두 개의 서로 겹치지 않는 정답 집합을 동시에 만족시킬 수 없습니다. **라벨 오류·신호등 상태·유일한 성능 저하 원인·미래 예측 불가능성 일반론의 증거는 아닙니다.** 원본을 제거하거나 정답을 대체하지 않았습니다.','',
        'Teacher index는 같은 lineage의 두 checkpoint 사이에 seed별 '+', '.join(str(c['oracle_index_changed_count'])for c in summary['teacher_changes'])+'행에서 바뀌었습니다. 후보 생성기와 selector가 함께 바뀌므로 index 변경 자체를 학습 불안정이나 잘못된 정답으로 해석하지 않습니다. 전체 episode/group와 모든 쌍 witness는 독립 요약에 있습니다.','',
        '[독립 전수 재계산](independent_summary.json) · [실제 실행 report](raw/report.json) · [실제 v2 계획](raw/plan.json) · [원격 전후 전송 검증](raw/transport_verification.json)','',
        '[계획 준비 경위](preparation/stopmix_training_oracles_plan_preparation_v1.json) · [미실행 v1](preparation/stopmix_training_oracles_plan_v1.json) · [실행 receipt](preparation/stopmix_training_oracles_v1_launch.json) · [종료 receipt](preparation/stopmix_training_oracles_v1_ssh_completion.json)','',
        '로컬 Python3.10 준비본 v1은 실행하지 않았습니다. Python3.12의 AST type_params 필드 차이 때문에 별도 v2를 실행 전에 선언했습니다. loss 원문·prefix bytes는 동일하며 AST hash를 사후 정규화하거나 v1이 실행됐다고 바꾸지 않았습니다.','',
        '## 6개 전체 저장 행','']
    for p in summary['passes']:text.append(f"- [{p['seed']} / {p['endpoint']} 전체1,147행](raw/seed_{p['seed']}/{p['endpoint']}/samples.jsonl)")
    text+=['','기존 full-corpus 무결성 검사는 test metadata/JSON/JPEG를 읽을 수 있지만 실제 진단 NN forward는 TRAIN만입니다. native oracle/regression의 일치는 원래 실행의 source-bound 증거이며, 이 도구는 native loss나 checkpoint를 재실행하지 않습니다. checkpoint는 원격 원본 SHA만 연결합니다.','',
        '[원본/공개 SHA·소스 provenance](publication_manifest.json) · [공개 파일 체크섬](SHA256SUMS) · [앞선 10epoch 결과](../08_stopmix_ten_epoch_continuation/results/README.md)','',
        'raw/SHA256SUMS는 원본 private 파일용 참조로 이름을 바꿔 보존했습니다. 공개 JSON은 account 경로만 가린 metadata view이며 raw SHA를 포함합니다. samples JSONL은 원본 바이트 그대로입니다. 공개 source는 감사를 위한 보관본이며 실행 요청이 아닙니다.','']
    return '\n'.join(text)


def publish():
    require(not OUTPUT.exists()and not any(p.is_symlink()for p in (OUTPUT,*OUTPUT.parents)),'fresh public category required')
    payload,extras=original_inventory();summary=authenticate(payload,extras);own=regular(Path(__file__)).read_bytes();own_sha=sha(own);entries={}
    sources=[('raw/'+n,n,v)for n,v in payload.items()]+[('preparation/'+n,n,v)for n,v in extras.items()]
    for relative,name,raw in sources:
        if name=='SHA256SUMS':relative='raw/original_private_SHA256SUMS.txt'
        if name.endswith('.json'):
            view=redact(decode(raw));require(not {'raw_source_sha256','publication_notice'}&set(view),'reserved publication keys')
            view.update(raw_source_sha256=sha(raw),publication_notice='Metadata view; only account paths redacted. No model/data approval.');public=encoded(view);kind='metadata_view'
        elif name.endswith('.jsonl'):public=raw;kind='original_bytes'
        else:public=redact(raw.decode()).encode();kind='original_bytes'if public==raw else'redacted_source_reference'
        new_bytes(OUTPUT/relative,public);entries[relative]=dict(raw_source_sha256=sha(raw),public_sha256=sha(public),kind=kind)
    new_bytes(OUTPUT/'independent_summary.json',encoded(summary));png,render=chart(summary);new_bytes(OUTPUT/'train_oracle_costs.png',png)
    new_bytes(OUTPUT/'README.md',readme(summary).encode());new_bytes(OUTPUT/'publication_source.py',own)
    for name in ('independent_summary.json','train_oracle_costs.png','README.md','publication_source.py'):
        entries[name]=dict(public_sha256=digest(OUTPUT/name),kind='generated_publication_evidence')
    after_payload,after_extras=original_inventory()
    require(after_payload==payload and after_extras==extras and digest(Path(__file__))==own_sha,'source/input changed during publication')
    provenance=dict(schema='portable_e2e.train_oracle_publication.v1',publisher_sha256=own_sha,public_files=entries,
        original_sha256={n:sha(v)for n,v in payload.items()},preparation_sha256=EXTRA,renderer=render,
        all6_saved_rows_retained=True,raw_jsonl_bytes_unchanged=True,original_report_sha256=PINS['report.json'],**{k:False for k in DENIALS})
    new_bytes(OUTPUT/'publication_manifest.json',encoded(provenance))
    files=sorted(p for p in OUTPUT.rglob('*')if p.is_file())
    new_bytes(OUTPUT/'SHA256SUMS',''.join(digest(p)+'  '+p.relative_to(OUTPUT).as_posix()+'\n'for p in files).encode())
    links=privacy_links(OUTPUT)
    for line in (OUTPUT/'SHA256SUMS').read_text().splitlines():pin,name=line.split('  ');require(digest(OUTPUT/name)==pin,'public SHA mismatch')
    require(original_inventory()==(payload,extras)and digest(Path(__file__))==own_sha,'final source/input postcheck failed')
    print(json.dumps(dict(status=summary['status'],files=len(files)+1,links=links,rows=6882,publication_manifest_sha256=digest(OUTPUT/'publication_manifest.json'))))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);parser.add_argument('--verify-only',action='store_true');args=parser.parse_args()
    if args.verify_only:
        own_sha=digest(Path(__file__))
        payload,extras=original_inventory();summary=authenticate(payload,extras)
        require(original_inventory()==(payload,extras)and digest(Path(__file__))==own_sha,'post-audit source/input changed')
        print(json.dumps(dict(status=summary['status'],passes=len(summary['passes']),rows=summary['total_rows'],costs=summary['total_cost_values'],
            teacher_changes=[r['oracle_index_changed_count']for r in summary['teacher_changes']],pairs_per_pass=[r['ambiguity']['pair_count']for r in summary['passes']])))
    else:publish()
