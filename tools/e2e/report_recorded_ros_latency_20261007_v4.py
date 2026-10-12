"""HH_261007 - Plot every actual V11 row, retaining deferred audit timing and the failed43rd original result.

Charts are diagnostic renderings of archived numeric observations, never RViz
screenshots or a driving approval. Unexecuted rows keep empty timing fields.
"""
import argparse
import csv
import gzip
import fcntl
import hashlib
import importlib.metadata
import json
import math
import os
import statistics
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / 'storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v11'
OUT = ROOT / 'storage/experiments/diagnostics/2026-10-07/recorded_ros_latency_v11_v4'
LOCK = ROOT / 'storage/assets/locks/autoware_e2e_runtime.lock'
INPUTS = {'subscriber_anchors.jsonl':'d8247fa9df84b07604f3f543b6a6a1c8469794edf58a33b411ffb93cb006d3d9',
    'direct_anchors.jsonl':'320533e2d4fce42dcfa47f13b21cd5941de32a0721759732733e0af72d050072',
    'subscriber_output_calls.jsonl':'f8f3c50468b5ba3f552e49228efb29d33442f78beccfec7b4aaf30d50dd876cd',
    'publisher_received_outputs.jsonl':'14eaa5d366a3a995d2bbaebdc41b067e3fd2e7e5e38da2af4b704d76600d294a'}


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
    need(len(subscriber)==43 and [r['ordinal'] for r in subscriber]==list(range(43)),'whole actually recorded V11 prefix43 required')
    need(subscriber[-1]['status']=='FAILED_RETAINED','original failed43rd row required')
    need(sum(r['status']=='REJECTED_ROS_END_TO_END_DEADLINE' for r in subscriber)==2,'actual prepublication deadline rejection must remain')
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
    need(len(calls)==53 and len(received)==52,'all real emitted and denied publication calls required')
    denied=[r for r in calls if r['actually_published'] is False]
    need(len(denied)==1 and denied[0]['ordinal']==42 and denied[0]['topic'].endswith('/shadow_trajectory'), 'denied first trajectory retained')
    failed_output=delta(denied[0]['publisher_bundle_started_ns'],denied[0]['actual_publish_entered_ns'])
    need(failed_output>100,'unchanged100ms failure required')
    return table,stages,dict(direct=918,ROS_nonfailure=42,ROS_failed=1,ROS_UNEXECUTED=875,
        actual_published_calls=52,actual_received_outputs=52,full_trajectory_path_pairs=26,
        partial_trajectory_without_path=0,failed_trajectory_entry_ms=failed_output,
        prepublication_deadline_rejected=2,
        audit_flush_duration_separately_measured=True,residual_includes_original_result_attachment=True,
        whole_ACK_including_audit_measured=True)


def local_stage_summary(subscriber):
    """HH_261007 - Preserve all measured messages, including UNKNOWN anchors; local stage sums are not wall or network delay."""
    expected_native=("native_C_take_and_matched_endpoint_lookup","native_take_tuple_validation","native_execute_tuple_validation")
    expected_receive=("peer","provenance_graph","publisher_gid","native_witness","CDR","CDR_hash","callback")
    stage_rows=[];groups=[];unknown=0
    for row in subscriber:
        profile=row["owned_received_input_stage_timing"];messages=profile["messages"]
        need(profile["observed_count"]==len(messages)==19 and profile["padded_missing_messages"] is False,"all actually observed19 input rows required")
        need(profile["native_stage_names"]==list(expected_native) and profile["receive_stage_names"]==list(expected_receive),"fixed actual local stage schemas required")
        need(len({m["topic"] for m in messages})==19,"unique actual observed topics required")
        group={}
        for message in messages:
            if message["anchor_binding"]=="UNKNOWN":
                unknown+=1;need(message["ordinal"] is None,"unknown input anchor may not be invented")
            else:need(message["ordinal"]==row["ordinal"],"bound actual input ordinal differs")
            for kind,names in (("native_stages",expected_native),("receive_stages",expected_receive)):
                measured=message[kind];need(len(measured)==len(names),"all ten actual complete local stages required")
                for index,stage in enumerate(measured):
                    need(stage[0]==index and len(stage)==4 and all(type(v) is int for v in stage),"actual successful integer local stage required")
                    elapsed=delta(stage[1],stage[2]);need(stage[3]==stage[2]-stage[1],"actual duration must equal endpoints")
                    name=names[index];group[name]=group.get(name,0)+elapsed
                    stage_rows.append(dict(receipt_ordinal=row["ordinal"],topic=message["topic"],anchor_binding=message["anchor_binding"],
                        bound_ordinal=message["ordinal"],stage=name,begun_ns=stage[1],finished_ns=stage[2],duration_ns=stage[3],duration_ms=elapsed))
        groups.append(group)
    need(len(stage_rows)==8170 and unknown==19,"complete43 by19 by10 plus actual unknown failure group required")
    return stage_rows,dict(groups=43,messages=817,complete_local_stages=8170,UNKNOWN_anchor_messages=19,
        mean_summed_per_observed_group_ms={name:statistics.mean(g.get(name,0) for g in groups) for name in (*expected_native,*expected_receive)},
        does_not_equal_wall_latency=True,pure_network_latency_claimed=False)


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
        local_rows,local_counts=local_stage_summary(rows['subscriber_anchors.jsonl'])
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
        # HH_261007 - Losslessly compress all8170 CSV rows inside the unchanged2MiB report cap; no row truncation.
        with gzip.open(OUT/'actual_local_stages.csv.gz','xt',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(local_rows[0]));writer.writeheader();writer.writerows(local_rows)
        fig,ax=plt.subplots(figsize=(13,14))
        left=[0.]*43
        for key,label,color in [('input_and_admission_ms','Inputs + admission','#5878a4'),
            ('policy_ms','Original policy timer','#71b77c'),
            ('attachment_and_audit_residual_ms','Result attachment + audit (not hash only)','#f1bd62'),
            ('after_policy_to_ACK_ms','After policy to actual ACK completion','#b688b6')]:
            values=[r[key] for r in stages];ax.barh(range(43),values,left=left,label=label,color=color)
            left=[a+b for a,b in zip(left,values)]
        ax.axvline(100,color='red',linestyle='--',label='Unchanged100ms reference')
        ax.scatter([counts['failed_trajectory_entry_ms']],[42],marker='x',color='red',s=90,label='Denied trajectory entry; no output for this anchor')
        ax.set_yticks(range(43),[f"{r['ordinal']}: {r['status']}" for r in stages]);ax.tick_params(axis='y',labelsize=6);ax.invert_yaxis()
        ax.set_xlabel('Actual milliseconds from publisher bundle start; purple includes deferred audit');ax.set_title('V11 recorded ROS: all43 observed rows; remaining875 UNEXECUTED')
        ax.legend(loc='upper right',fontsize=7);fig.tight_layout();fig.savefig(OUT/'ROS_prefix_all43.png',dpi=130);plt.close(fig)
        fig,ax=plt.subplots(figsize=(12,4))
        for field,label in [('policy_ms','Original policy timer'),('policy_plus_audit_ms','Policy + audit observation')]:
            ax.plot(range(918),[r[field] for r in table[:918]],linewidth=.6,label=label)
        ax.axhline(100,color='red',linestyle='--',label='100ms reference; not sensor/actuator timing')
        ax.set_xlabel('Every recorded anchor ordinal0..917');ax.set_ylabel('Actual milliseconds')
        ax.set_title('V11 direct: complete918 observations, no driving or DDS claim');ax.legend(fontsize=8)
        fig.tight_layout();fig.savefig(OUT/'direct_full918.png',dpi=130);plt.close(fig)
        for name,pin in INPUTS.items():need(sha(RUN/name)==pin,'closing journal changed')
        need(sha(Path(__file__).resolve())==expected_sha,'closing report source changed')
        need(not any(k=='torch' or k=='rclpy' or k=='carla' for k in sys.modules),'no model/ROS/CARLA imports allowed')
        report=dict(provenance='HH_261007 - Diagnostic numeric plots preserve all918+918 rows, blank unexecuted timings and partial publication; not RViz screenshots.',
            schema='portable_e2e.recorded_ROS_latency_report.20261007.v4',counts=counts,local_stage_counts=local_counts,source_sha256=expected_sha,
            input_sha256=INPUTS,charts_are_recorded_numeric_renderings=True,actual_RViz_screenshot=False,
            training_steps=0,NN_inferences=0,weights_changed=False,model_performance_improved=False,
            fixed_GPU_floor_changed=False,formal_main_approved=0,formal_sub_approved=0)
        with (OUT/'result.json').open('x') as stream:json.dump(report,stream,sort_keys=True,indent=2,allow_nan=False);stream.write('\n')
        need(sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())<=2*1024**2,'complete report output cap exceeded')
        print(json.dumps(counts))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--expected-source-sha256',required=True)
    run(parser.parse_args().expected_source_sha256)
