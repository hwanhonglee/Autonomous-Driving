#!/usr/bin/env python3
"""HH_260906 - Verify and publish four completed diagnostic fits without loading checkpoints or changing training settings."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
from types import ModuleType

from scripts.e2e import curate_nuplan_calibration_opcodes as safe

REPORT_SHA='6630327dd6167cdde1f1c54a7eb9768cf135a8d7aebc71336894283091c69a25'
MANIFEST_SHA='769563f88adb372f898724c23cd2830750c356e56b14bf0066338e5875f93a9b'
PLAN_SHA='77415490527f6b7323b30f7a77c63b19d129861506b99beffb06102057ba743e'
WORKER_SHA='4b228baa3ad550dcd3ed602904cc43183c0b3c23b67f6078e83c5d4407086501'
HELPER_SHA='a995722f1a60f192a059433afb44e6297937d97166877c5cc4de23220d15b436'
TRANSPORT_SHA='5a2b14ae2d63a51f677f3ea282aa2e4c61f03b654b594f8d1ae6bc919f611d72'
SOURCE_COMMIT='b478f02e42b94bf04bffec5c8170e05edc33b0f8'
ARMS=(('A1_workers0',0),('B1_workers2',2),('B2_workers2',2),('A2_workers0',0))
TRAIN_TIME='train_call_wall_seconds_including_final_sync'
PROCESS_TIME='process_wall_seconds_including_python_startup_and_integrity_checks'
sha,require,read_json,encoded=safe.audit.sha,safe.audit.require,safe.audit.json_load,safe.encoded


def load_owned_code(path,expected,pins):
    # HH_260906 - Only exact reviewed stdlib source is compiled in memory; main, fitting, GPU, process and checkpoint functions are never called.
    raw=safe.checked(path,pins,expected);module=ModuleType('_owned_abba_publication_contract')
    module.__file__=str(path);exec(compile(raw,str(path),'exec'),module.__dict__)
    return module


def finite_positive(value):
    return type(value) in (int,float) and math.isfinite(value) and value>0


def bind(root):
    root=Path(root).absolute();pins={}
    read=lambda name,expected=None:read_json(safe.checked(root/name,pins,expected))
    report=read('report.json',REPORT_SHA);plan=read('plan.json',PLAN_SHA)
    transport=read('transport_verification.json',TRANSPORT_SHA)
    manifest=safe.manifest_entries(safe.checked(root/'SHA256SUMS',pins,MANIFEST_SHA))
    missing={arm+'/fit/checkpoints/latest.pt' for arm,_ in ARMS}
    require(set(manifest)==set(report['files'])|{'report.json'} and len(manifest)==43,'original 43-payload manifest differs')
    require(safe.tree_files(root)==(set(manifest)-missing)|{'SHA256SUMS','transport_verification.json'},'local original inventory differs')
    for name,expected in manifest.items():
        if name in missing:
            require(not (root/name).exists(),'checkpoint-free publication input expected')
        else:safe.checked(root/name,pins,expected)
    for name,item in report['files'].items():
        require(item['sha256']==manifest[name] and type(item['bytes']) is int and item['bytes']>=0,'original output declaration differs')
        if name not in missing:require((root/name).stat().st_size==item['bytes'],'original byte count differs')
    expected_transport={n:dict(sha256=manifest[n],size_bytes=report['files'][n]['bytes']) for n in report['files']}
    expected_transport.update({n:dict(sha256=pins[root/n],size_bytes=(root/n).stat().st_size) for n in ('report.json','SHA256SUMS')})
    require(transport.get('regular_file_inventory')==expected_transport and transport.get('omitted_checkpoints')==sorted(missing)
        and transport.get('status')=='VERIFIED_ABBA_EXACT_PARITY_NOT_PROMOTED'
        and transport.get('all_original_sha_payloads_pass')==43,'transport/omitted checkpoint receipt differs')
    for key in ('remote_pre_post_bytes_unchanged','source_pre_post_bytes_unchanged','metrics_four_byte_identical','cooperative_gpu0_lease_free'):
        require(transport.get(key) is True,'transport completion proof missing: '+key)
    require(transport.get('checkpoint_tensors_loaded_by_collector') is False and transport.get('data_admission') is False
        and transport.get('model_promotion') is False,'collector scope differs')
    worker=load_owned_code(root/'source/independent_worker.py',WORKER_SHA,pins)
    helper=load_owned_code(root/'source/frozen_helper.py',HELPER_SHA,pins)
    worker.validate_plan(plan,helper)
    require(worker.SOURCE_COMMIT==SOURCE_COMMIT==plan['source_commit'] and tuple(worker.ARMS)==ARMS,'reviewed source/arm order differs')
    sources=dict(plan['source_sha256'],**{'independent_worker.py':WORKER_SHA,'frozen_helper.py':HELPER_SHA})
    require(report.get('source_sha256')==sources and len(sources)==13,'all thirteen archived sources required')
    for name,value in sources.items():safe.checked(root/'source'/name,pins,value)
    require(report.get('schema')==worker.SCHEMA and report.get('status')=='ABBA_COMPLETE_EXACT_PARITY_NOT_PROMOTED'
        and report.get('plan_sha256')==PLAN_SHA and report.get('approval')==worker.DENIALS
        and report.get('deadline_met') is True and report.get('postcheck_errors')==[] and report.get('unperformed_arms')==[],'complete nonpromotion report required')
    started,finished=helper.timestamp(report['started_at_utc']),helper.timestamp(report['completed_at_utc'])
    require(helper.timestamp(plan['declared_at_utc'])<=started<=finished<helper.timestamp(plan['finish_before_utc']),'execution chronology differs')
    require([(r.get('arm'),r.get('num_workers')) for r in report['arms']]==list(ARMS),'four ABBA fits in fixed order required')
    require(report.get('metrics_parity')==[dict(reference=ARMS[0][0],candidate=arm,exact=True,first_difference=None,retained_rows_per_arm=287) for arm,_ in ARMS[1:]],'remote metric parity receipt differs')
    checkpoint_rows=report.get('checkpoint_parity',[])
    require(len(checkpoint_rows)==4 and [r['arm'] for r in checkpoint_rows]==[a for a,_ in ARMS],'four remote checkpoint parity rows required')
    baseline=None;measurements=[];previous=started
    for item,(arm,workers),checkpoint in zip(report['arms'],ARMS,checkpoint_rows):
        require(item.get('status')=='FIT_COMPLETE_NOT_PROMOTED' and type(item.get('exit_code')) is int and item['exit_code']==0
            and item.get('cleanup',{}).get('group_absent') is True,'fit process not cleanly complete')
        require(item['cleanup']['pid']==item['cleanup']['process_group']==item['pid'],'owned cleanup identity differs')
        began,ended=helper.timestamp(item['started_at_utc']),helper.timestamp(item['completed_at_utc'])
        require(previous<=began<=ended<=finished,'fit chronology/order differs');previous=ended
        fit=read(arm+'/fit_report.json',item['fit_report_sha256']);train=read(arm+'/fit/run.json')
        state=dict(global_step=287,epoch=1,next_batch_index=0,samples_seen=1147,domain_samples_seen={'carla':1147})
        require(fit.get('schema')==worker.SCHEMA and fit.get('status')=='FIT_COMPLETE_NOT_PROMOTED'
            and fit.get('arm')==arm and fit.get('num_workers')==workers and fit.get('state')==state
            and fit.get('source_sha256')==sources and fit.get('approval')==worker.DENIALS and fit.get('postcheck_errors')==[]
            and fit.get('pid')==item['pid']==fit.get('process_group')
            and all(r.get('still_alive') is False for r in fit.get('child_cleanup',[])),'fit source/state/cleanup differs')
        require(began<=helper.timestamp(fit['started_at_utc'])<=helper.timestamp(fit['completed_at_utc'])<=ended,'fit report timing interval differs')
        require(finite_positive(item.get(TRAIN_TIME)) and finite_positive(item.get(PROCESS_TIME))
            and item[TRAIN_TIME]==fit.get(TRAIN_TIME) and item[TRAIN_TIME]<=item[PROCESS_TIME],'finite labelled timing fields differ')
        require(train.get('status')=='TRAINING_TARGET_REACHED' and train.get('state')==state and train.get('dataset_size')==1147
            and train.get('training_split')=='train' and train.get('dataset_fingerprint_sha256')==helper.TRAIN_SHA
            and train.get('corpus_fingerprint_sha256')==helper.CORPUS_SHA and train.get('loss_config')==helper.LOSS_CONFIG
            and train.get('train_config')==worker.train_config(helper,workers) and train.get('model_parameter_count')==954590,'one-epoch trainer contract differs')
        require(len(train.get('training_episode_ids',[]))==len(set(train.get('training_episode_ids',[])))==3,'three train episodes required')
        require(train.get('model_config')==read('source/'+helper.MODEL_CONFIG),'fixed physical-v1 model config differs')
        metrics_path=root/arm/'fit/metrics.jsonl';rows=worker.read_metrics(metrics_path,helper)
        require(train.get('last_metrics')==rows[-1],'final metric receipt differs')
        raw=safe.checked(metrics_path,pins)
        if baseline is None:baseline=raw
        require(raw==baseline,'all four full metric streams must be byte-identical')
        directory=root/arm/'worker_proofs'
        # HH_260906 - File-only transport need not materialize the two empty zero-worker directories; nonzero-worker witnesses remain mandatory.
        proofs=[] if workers==0 and not directory.exists() else worker.validate_worker_proofs(directory,workers,item['pid'],item['pid'],WORKER_SHA,helper)
        require(fit.get('worker_proofs')==proofs,'CPU spawn witness mismatch')
        require(checkpoint==dict(arm=arm,checkpoint_sha256=manifest[arm+'/fit/checkpoints/latest.pt'],exact=True,
            first_difference=None,ignored_fields=['created_at_utc','train_config.num_workers'],weights_only=True),'remote secure checkpoint parity differs')
        measurements.append(dict(arm=arm,num_workers=workers,train_call_seconds=item[TRAIN_TIME],process_seconds=item[PROCESS_TIME],
            metric_rows=len(rows),sample_exposures=rows[-1]['samples_seen'],checkpoint_sha256=checkpoint['checkpoint_sha256']))
    safe.checked(Path(__file__).absolute(),pins)
    safe.checked(Path(safe.__file__).absolute(),pins)
    safe.checked(Path(safe.audit.__file__).absolute(),pins)
    return report,plan,transport,measurements,pins,manifest


def statistics(measurements):
    result={}
    for key in ('train_call_seconds','process_seconds'):
        a=[r[key] for r in measurements if r['num_workers']==0];b=[r[key] for r in measurements if r['num_workers']==2]
        require(len(a)==len(b)==2 and all(finite_positive(v) for v in a+b),'two measured repetitions per condition required')
        am,bm=sum(a)/2,sum(b)/2
        result[key]=dict(workers0_mean=am,workers2_mean=bm,observed_reduction_percent=100*(1-bm/am),workers0_observed=a,workers2_observed=b)
    return result


def redact(value):
    value=safe.sanitize(value)
    if isinstance(value,str):return re.sub(r'(?:GPU-)?[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}','${GPU0_UUID}',value)
    if isinstance(value,list):return [redact(v) for v in value]
    if isinstance(value,dict):return {redact(k):redact(v) for k,v in value.items()}
    return value


def render(measurements,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure,axes=plt.subplots(1,2,figsize=(16,7))
    for axis,key,title in zip(axes,('train_call_seconds','process_seconds'),('Training call incl. final GPU sync','Whole fit process incl. startup + integrity checks')):
        values=[r[key] for r in measurements];colors=['#3276ad' if r['num_workers']==0 else '#dc8a35' for r in measurements]
        axis.bar(range(4),values,color=colors,width=.64)
        for i,value in enumerate(values):axis.text(i,value+.7,f'{value:.3f} s',ha='center',fontsize=12)
        axis.set_xticks(range(4),[r['arm'].replace('_','\n') for r in measurements]);axis.set_ylim(0,max(values)*1.19)
        axis.set_ylabel('Measured elapsed seconds');axis.set_title(title,fontsize=12);axis.grid(axis='y',alpha=.2)
    figure.suptitle('FIXED ABBA INPUT-LOADING DIAGNOSTIC — four fresh one-epoch fits',fontsize=15)
    figure.text(.5,.065,'Each fit: 287 updates / 1,147 train samples; blue = 0 workers, orange = 2 CPU workers.',ha='center',fontsize=11)
    figure.text(.5,.026,'Only two fixed-order repetitions per condition. Not full-campaign/inference speed evidence; production unchanged.',ha='center',fontsize=10)
    figure.tight_layout(rect=(0,.105,1,.94))
    with (output/'01_measured_abba_timing.png').open('xb') as stream:figure.savefig(stream,format='png',dpi=120)
    plt.close(figure)
    return matplotlib.__version__


def readme(measurements,stats):
    lines=['# DataLoader 0명/2명 · 고정 ABBA 실제 계측','',
        '<!-- HH_260906 - Four one-epoch diagnostic fits are not four full training campaigns or an adopted performance change. -->','',
        '같은 physical-v1 모델·TRAIN 1,147개·seed·loss·학습 순서를 유지하고 DataLoader CPU worker 수만 0→2→2→0으로 비교했습니다. 각자 새 프로세스·새 초기화에서 287 update(1 epoch)를 실행한 진단 4회이며, 장기 학습 캠페인 4개가 아닙니다.','',
        '| 순서 | CPU workers | 학습 함수 시간 s | 전체 fit 프로세스 시간 s |','|---|---:|---:|---:|']
    for r in measurements:lines.append(f"| {r['arm']} | {r['num_workers']} | {r['train_call_seconds']:.6f} | {r['process_seconds']:.6f} |")
    t,p=stats['train_call_seconds'],stats['process_seconds']
    lines+=['',f"이 진단에서 관찰한 평균: 학습 함수 {t['workers0_mean']:.6f}→{t['workers2_mean']:.6f}s ({t['observed_reduction_percent']:.2f}% 감소), 전체 fit 프로세스 {p['workers0_mean']:.6f}→{p['workers2_mean']:.6f}s ({p['observed_reduction_percent']:.2f}% 감소). 두 조건 각각 2회뿐인 고정 순서 반복입니다. 일반 서버 성능·장기 학습 시간·추론 FPS 개선을 증명하지 않습니다.",'',
        '학습 함수 시간에는 worker 생성, 데이터 읽기·전처리, forward/backward/update, checkpoint 저장과 최종 GPU 동기화가 포함됩니다. 전체 fit 프로세스는 여기에 Python/CUDA 시작과 각 fit 전후 corpus 무결성 검사 등을 포함합니다. 전체 ABBA 감독 프로세스의 시간과는 다르며, 디스크/page cache·spawn 비용을 제거한 순수 연산 benchmark가 아닙니다.','',
        '4개 metrics.jsonl은 각 287행, 합계 1,148행으로 원본 바이트가 모두 같습니다. 노출은 1,147×4=4,588회이며 고유 데이터가 늘어난 것은 아닙니다. 원격 실행은 checkpoint를 안전한 CPU loader로 비교하여 `created_at_utc`, `train_config.num_workers` 두 필드만 제외하고 일치했다고 기록했습니다. 공개 helper는 checkpoint tensor를 다운로드하거나 다시 읽지 않았고, 원격 검사 기록과 SHA를 대조했습니다.','',
        'TRAIN만 신경망에 사용했습니다. 기존 전체 corpus 무결성 검사는 TEST metadata/JPEG를 읽을 수 있지만 VAL/TEST 추론·loss·모델 선택은 하지 않았습니다. 실제 GPU 이용 기록과 worker의 CUDA 비노출 시작 증거를 보존하되 장치 UUID·개인 경로는 공개 view에서 치환합니다. 데이터 승인·모델 승격·프로덕션 worker 설정 변경은 없습니다.','',
        '- [실측 시간 그림](01_measured_abba_timing.png)','- [재계산한 시간·분모](timing_summary.json)',
        '- [원격 완료 보고서 view](metadata/report.json)','- [사전 고정 계획 view](metadata/plan.json)',
        '- [수집·원격 SHA 검증 receipt view](metadata/transport_verification.json)',
        '- [원본→공개 SHA 및 제외 항목](publication_manifest.json)','',
        '각 arm 폴더의 전체 metrics.jsonl은 원본 바이트입니다. fit/run report와 source_views는 명시적인 metadata/source text view이며 실행용 원본이 아닙니다. 원본 44파일 중 checkpoint 4개는 원격에 유지했고, 로컬 수집 40파일의 비어 있는 lock/log 8개는 중복 공개하지 않습니다. 원본은 삭제하지 않았습니다. 재현에는 원본 private 자료와 고정 실행 소스가 필요하며 git clone만으로 데이터·실행 승인이 생기지는 않습니다.','']
    return '\n'.join(lines).encode()


def publish(root,output):
    root,output=Path(root).absolute(),Path(output).absolute()
    require(not output.exists() and not output.is_symlink() and not any(p.is_symlink() for p in output.parents)
        and not output.resolve().is_relative_to(root.resolve()) and not root.resolve().is_relative_to(output.resolve())
        and not output.resolve().is_relative_to((safe.REPO/'datasets').resolve()),'fresh separate nondataset output required')
    report,plan,transport,measurements,pins,raw_manifest=bind(root);stats=statistics(measurements)
    payloads={'README.md':readme(measurements,stats),'timing_summary.json':encoded(dict(measurements=measurements,statistics=stats,
        source_commit=SOURCE_COMMIT,worker_sha256=WORKER_SHA,report_sha256=REPORT_SHA,plan_sha256=PLAN_SHA,
        diagnostic_fit_count=4,full_training_campaign_count=0,total_updates=1148,total_sample_exposures=4588,
        unique_train_samples=1147,checkpoint_tensors_locally_loaded=False,production_changed=False)),
        'original_SHA256SUMS.txt':safe.checked(root/'SHA256SUMS',pins,MANIFEST_SHA)}
    mappings=[]
    for name in sorted(safe.tree_files(root)):
        raw=safe.checked(root/name,pins)
        if name.endswith('.jsonl'):target=name;payloads[target]=raw;representation='original_bytes'
        elif name.endswith('.json') or name.startswith('source/'):
            target=('source_views/'+name.removeprefix('source/')+'.json') if name.startswith('source/') else 'metadata/'+name
            record=read_json(raw) if name.endswith('.json') else raw.decode()
            payloads[target]=encoded(dict(publication_notice='Redacted metadata/source-text view, not executable replay or an approved model.',raw_source_sha256=sha(raw),record=redact(record)))
            representation='redacted_metadata_view'
        else:
            require(name=='SHA256SUMS' or len(raw)==0,'unreviewed nonmetadata publication input')
            target='original_SHA256SUMS.txt' if name=='SHA256SUMS' else None;representation='original_bytes' if target else 'empty_lock_or_log_retained_private'
        mappings.append(dict(private_relative_path=name,raw_sha256=sha(raw),public_path=target,
            public_sha256=sha(payloads[target]) if target else None,representation=representation))
    for arm,_ in ARMS:mappings.append(dict(private_relative_path=arm+'/fit/checkpoints/latest.pt',raw_sha256=raw_manifest[arm+'/fit/checkpoints/latest.pt'],
        public_path=None,public_sha256=None,representation='checkpoint_not_downloaded_or_published'))
    for value in payloads.values():safe.privacy(value)
    output.mkdir(parents=True,exist_ok=False)
    for name,value in payloads.items():safe.audit.write_new(output/name,value)
    version=render(measurements,output)
    for path,value in list(pins.items()):safe.checked(path,pins,value)
    require(safe.tree_files(root)=={row['private_relative_path'] for row in mappings if row['representation']!='checkpoint_not_downloaded_or_published'},'input inventory changed while publishing')
    manifest=dict(schema='portable_e2e.dataloader_abba_publication.v1',status='PUBLISHED_DIAGNOSTIC_NOT_ADOPTED',published_at_utc=safe.audit.utc(),
        source_commit=SOURCE_COMMIT,worker_sha256=WORKER_SHA,helper_sha256=HELPER_SHA,plan_sha256=PLAN_SHA,
        original_report_sha256=REPORT_SHA,original_manifest_sha256=MANIFEST_SHA,transport_receipt_sha256=TRANSPORT_SHA,
        publication_source_sha256=pins[Path(__file__).absolute()],matplotlib_version=version,all_inputs_unchanged_before_after=True,
        helper_source_sha256={Path(p).name:v for p,v in pins.items() if p in (Path(safe.__file__).absolute(),Path(safe.audit.__file__).absolute())},
        local_original_files=40,additional_transport_receipts=1,original_remote_files=44,checkpoint_tensors_loaded=False,
        all_four_metric_streams_independently_byte_equal=True,checkpoint_exact_parity_scope='Original remote secure comparison receipt; not locally replayed.',
        data_admission=False,model_promotion=False,production_changed=False,source_views=mappings,
        files=[dict(path=p.relative_to(output).as_posix(),sha256=sha(p.read_bytes()),size_bytes=p.stat().st_size) for p in sorted(output.rglob('*')) if p.is_file()])
    safe.privacy(encoded(manifest));safe.audit.write_new(output/'publication_manifest.json',encoded(manifest))
    names=sorted(p for p in output.rglob('*') if p.is_file())
    safe.audit.write_new(output/'SHA256SUMS',''.join(sha(p.read_bytes())+'  '+p.relative_to(output).as_posix()+'\n' for p in names).encode())
    return manifest


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--input-root',required=True,type=Path);parser.add_argument('--output-dir',required=True,type=Path)
    args=parser.parse_args(argv);result=publish(args.input_root,args.output_dir)
    print(json.dumps({'status':result['status'],'original_remote_files':result['original_remote_files']}));return 0


if __name__=='__main__':raise SystemExit(main())
