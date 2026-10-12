"""HH_261007 - Plot every actual V9 row, retaining deferred audit timing and the failed50th original result.

Charts are diagnostic renderings of archived numeric observations, never RViz
screenshots or a driving approval. Unexecuted rows keep empty timing fields.
"""
import argparse
import csv
import fcntl
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / 'storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v9'
OUT = ROOT / 'storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v9_v2'
LOCK = ROOT / 'storage/assets/locks/autoware_e2e_runtime.lock'
INPUTS = {'subscriber_anchors.jsonl':'0e8fa5197819c1bbcd20ea75aa9ee5b314886d5b4ee450e8be27c7864d64269e',
    'direct_anchors.jsonl':'b9afe4d3c2bc4627b9716d6b4419aa33c45703971d4b339f9a64736df03aa2ce',
    'subscriber_output_calls.jsonl':'e031aa0817a0675df80e419c6b870d7f5879ecb539e2bb02ed72620deb143f4e',
    'publisher_received_outputs.jsonl':'5f1062b6ec76e48ade20e52f63d11c46b3b0bec807d24d40d75fa0b4b99de666'}


def need(value, message):
    if not value: raise ValueError(message)


def sha(path):
    path=Path(path);need(path.resolve(strict=True)==path and path.is_file(),'canonical source/input required')
    before=path.stat();h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024**2),b''):h.update(chunk)
    after=path.stat()
    need((before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)==
        (after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns),'input generation changed')
    return h.hexdigest()


def metadata(raw):
    def unique(pairs):
        result={}
        for k,v in pairs:need(k not in result,'duplicate JSON key');result[k]=v
        return result
    return json.loads(raw,object_pairs_hook=unique,parse_constant=lambda v:need(False,'nonfinite JSON'))


def milliseconds(value):
    need(type(value) in (float,int) and math.isfinite(value) and value>=0,'measured finite nonnegative duration required')
    return value*1000


def delta(start,end):
    need(type(start) is int and type(end) is int and 0<start<=end,'true monotonic integer interval required')
    return (end-start)/1e6


def original_and_deferred_diagnostics(row):
    # HH_261007 - A failed ACK may contain only deferred fields; recover the same-anchor genuine policy timer without dropping its failure.
    diagnostics=row.get('policy_diagnostics',{})
    need(type(diagnostics) is dict,'actual diagnostic dictionary required')
    if 'total_seconds' not in diagnostics:
        prior=row.get('actual_policy_result_before_transport_failure')
        need(type(prior) is dict and prior.get('anchor_ns')==row['timestamp_ns']
            and type(prior.get('diagnostics')) is dict and 'total_seconds' in prior['diagnostics'],
            'same-anchor actual pre-failure policy required')
        diagnostics={**prior['diagnostics'],**diagnostics}
    timing=diagnostics.get('owned_recorded_deferred_audit_timing')
    need(type(timing) is dict and timing.get('ACK_completion_includes_audit_cost') is True,'deferred audit must remain in ACK timing')
    return diagnostics


def summarize(direct,subscriber,calls,received):
    need(len(direct)==918 and [r['ordinal'] for r in direct]==list(range(918)),'whole918 direct journal required')
    need(len(subscriber)==50 and [r['ordinal'] for r in subscriber]==list(range(50)),'whole actually recorded V9 prefix50 required')
    need(subscriber[-1]['status']=='FAILED_RETAINED','original failed50th row required')
    table=[];stages=[]
    for r in direct:
        table.append(dict(role='direct',ordinal=r['ordinal'],status=r['status'],
            policy_ms=milliseconds(r['policy_diagnostics']['total_seconds']),
            policy_plus_audit_ms=milliseconds(r['policy_plus_audit_seconds'])))
    for ordinal in range(918):
        if ordinal>=len(subscriber):
            table.append(dict(role='ROS',ordinal=ordinal,status='UNEXECUTED'));continue
        r=subscriber[ordinal];s=r['stages']
        begin=s['publisher_bundle_started_ns'];ready=s['policy_started_ns'];end=s['policy_finished_ns']
        ack=r['actual_ACK_publish_completed_ns']
        policy=original_and_deferred_diagnostics(r)
        need(type(policy) is dict,'actual policy diagnostics or preserved failed result required')
        original=milliseconds(policy['total_seconds']);wrapped=delta(ready,end)
        need(wrapped>=original,'original policy timer must fit actual wrapper interval')
        item=dict(role='ROS',ordinal=ordinal,status=r['status'],input_and_admission_ms=delta(begin,ready),
            policy_ms=original,policy_plus_audit_ms=wrapped,attachment_and_audit_residual_ms=wrapped-original,
            after_policy_to_ACK_ms=delta(end,ack),full_ACK_ms=delta(begin,ack),
            audit_flush_ms=milliseconds(policy['owned_recorded_deferred_audit_timing']['audit_flush_seconds']))
        table.append(item);stages.append(item)
    need(len(calls)==72 and len(received)==71,'all real emitted and denied publication calls required')
    denied=[r for r in calls if r['actually_published'] is False]
    need(len(denied)==1 and denied[0]['ordinal']==49 and denied[0]['topic'].endswith('/shadow_path'), 'failed partial path retained')
    failed_path=delta(denied[0]['publisher_bundle_started_ns'],denied[0]['actual_publish_entered_ns'])
    need(failed_path>100,'unchanged100ms failure required')
    return table,stages,dict(direct=918,ROS_nonfailure=49,ROS_failed=1,ROS_UNEXECUTED=868,
        actual_published_calls=71,actual_received_outputs=71,full_trajectory_path_pairs=35,
        partial_trajectory_without_path=1,failed_path_entry_ms=failed_path,
        audit_flush_duration_separately_measured=True,residual_includes_original_result_attachment=True,
        whole_ACK_including_audit_measured=True)


def run(expected_sha):
    need(sha(Path(__file__).resolve())==expected_sha,'external source SHA required')
    for key,value in {'CUDA_VISIBLE_DEVICES':'','PYTHONNOUSERSITE':'1','OMP_NUM_THREADS':'1',
        'MKL_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}.items():
        need(os.environ.get(key)==value,'clean CPU-only report environment required')
    need(importlib.metadata.version('matplotlib')=='3.5.1' and importlib.metadata.version('numpy')=='1.21.5','existing plot environment required')
    need(3 in os.sched_getaffinity(0),'declared CPU3 unavailable')
    with LOCK.open('r') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        need(not OUT.exists(),'fresh report only')
        need(os.statvfs(ROOT).f_bavail*os.statvfs(ROOT).f_frsize>=2*1024**3+64*1024**2,'shared CPU floor/reserve required')
        for name,pin in INPUTS.items():need(sha(RUN/name)==pin,'frozen timing journal changed')
        os.sched_setaffinity(0,{3})
        rows={name:[metadata(line) for line in (RUN/name).read_bytes().splitlines()] for name in INPUTS}
        table,stages,counts=summarize(rows['direct_anchors.jsonl'],rows['subscriber_anchors.jsonl'],
            rows['subscriber_output_calls.jsonl'],rows['publisher_received_outputs.jsonl'])
        # HH_261007 - Keep Matplotlib's font cache inside the fresh diagnostic output, not the user's global cache or tmp.
        OUT.mkdir(parents=True,exist_ok=False)
        os.environ['MPLCONFIGDIR']=str(OUT/'matplotlib_cache')
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        columns=['role','ordinal','status','input_and_admission_ms','policy_ms','policy_plus_audit_ms',
            'attachment_and_audit_residual_ms','after_policy_to_ACK_ms','full_ACK_ms','audit_flush_ms']
        with (OUT/'complete_timing.csv').open('x',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader();writer.writerows(table)
        fig,ax=plt.subplots(figsize=(13,14))
        left=[0.]*50
        for key,label,color in [('input_and_admission_ms','Inputs + admission','#5878a4'),
            ('policy_ms','Original policy timer','#71b77c'),
            ('attachment_and_audit_residual_ms','Result attachment + audit (not hash only)','#f1bd62'),
            ('after_policy_to_ACK_ms','After policy to actual ACK completion','#b688b6')]:
            values=[r[key] for r in stages];ax.barh(range(50),values,left=left,label=label,color=color)
            left=[a+b for a,b in zip(left,values)]
        ax.axvline(100,color='red',linestyle='--',label='Unchanged100ms reference')
        ax.scatter([counts['failed_path_entry_ms']],[49],marker='x',color='red',s=90,label='Denied path entry; trajectory already emitted')
        ax.set_yticks(range(50),[f"{r['ordinal']}: {r['status']}" for r in stages]);ax.tick_params(axis='y',labelsize=6);ax.invert_yaxis()
        ax.set_xlabel('Actual milliseconds from publisher bundle start; purple includes deferred audit');ax.set_title('V9 recorded ROS: all50 observed rows; remaining868 UNEXECUTED')
        ax.legend(loc='upper right',fontsize=7);fig.tight_layout();fig.savefig(OUT/'ROS_prefix_all50.png',dpi=130);plt.close(fig)
        fig,ax=plt.subplots(figsize=(12,4))
        for field,label in [('policy_ms','Original policy timer'),('policy_plus_audit_ms','Policy + audit observation')]:
            ax.plot(range(918),[r[field] for r in table[:918]],linewidth=.6,label=label)
        ax.axhline(100,color='red',linestyle='--',label='100ms reference; not sensor/actuator timing')
        ax.set_xlabel('Every recorded anchor ordinal0..917');ax.set_ylabel('Actual milliseconds')
        ax.set_title('V9 direct: complete918 observations, no driving or DDS claim');ax.legend(fontsize=8)
        fig.tight_layout();fig.savefig(OUT/'direct_full918.png',dpi=130);plt.close(fig)
        for name,pin in INPUTS.items():need(sha(RUN/name)==pin,'closing journal changed')
        need(sha(Path(__file__).resolve())==expected_sha,'closing report source changed')
        need(not any(k=='torch' or k=='rclpy' or k=='carla' for k in sys.modules),'no model/ROS/CARLA imports allowed')
        report=dict(provenance='HH_261007 - Diagnostic numeric plots preserve all918+918 rows, blank unexecuted timings and partial publication; not RViz screenshots.',
            schema='portable_e2e.recorded_ROS_latency_report.20261007.v2',counts=counts,source_sha256=expected_sha,
            input_sha256=INPUTS,charts_are_recorded_numeric_renderings=True,actual_RViz_screenshot=False,
            training_steps=0,NN_inferences=0,weights_changed=False,model_performance_improved=False,
            fixed_GPU_floor_changed=False,formal_main_approved=0,formal_sub_approved=0)
        with (OUT/'result.json').open('x') as stream:json.dump(report,stream,sort_keys=True,indent=2,allow_nan=False);stream.write('\n')
        need(sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())<=2*1024**2,'complete report output cap exceeded')
        print(json.dumps(counts))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--expected-source-sha256',required=True)
    run(parser.parse_args().expected_source_sha256)
