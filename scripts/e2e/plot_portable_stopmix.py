#!/usr/bin/env python3
"""HH_260906 - Plot all six measured DRIVE/STOP development runs without synthesizing performance or promoting a model."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from portable_e2e.contract import _loads_json
from scripts.e2e import render_portable_no_accel_learning as legacy
from scripts.e2e import summarize_portable_stopmix as campaign

ROOT, ARMS, SEEDS = campaign.ROOT, tuple(campaign.ARMS), tuple(campaign.base.SEEDS)
TRAIN_METRICS = ('regression_loss', 'selected_ade_m')
VAL_METRICS = legacy.VAL_METRICS
require, finite, read = legacy.require, legacy.finite, legacy.read


def history(payload):
    rows, counted_bins = legacy.history(payload)
    for row in rows:
        require('candidate_regret_loss' not in row, 'STOPMIX original loss cannot include an auxiliary term')
        for metric in TRAIN_METRICS: finite(row.get(metric))
    bins = []
    for counted in counted_bins:
        group = rows[counted['first_step']-1:counted['last_step']]
        bins.append({k:v for k,v in counted.items() if k not in legacy.TRAIN_METRICS} | {
            metric: math.fsum(r[metric]*r['batch_domain_sample_counts']['carla'] for r in group)/counted['sample_exposures']
            for metric in TRAIN_METRICS})
    return rows, bins


def validate_summary(report):
    require(report.get('schema') == campaign.SCHEMA and report.get('status') in ('COMPLETE_NORMAL_STAGES','COMPLETE_NOT_PROMOTED')
        and report.get('normal_stage_completion') == 'COMPLETE' and report.get('completed_stage_count') == 18
        and all(report.get(k) is False for k in ('automatic_promotion','vehicle_control_approved','test_evaluated',
            'test_used_for_training_or_selection','training_data_approved_by_this_report')), 'complete research-only normal stages required')
    require([p.get('seed') for p in report.get('pairs',[])] == list(SEEDS), 'all three paired seeds required')
    require((report['status']=='COMPLETE_NOT_PROMOTED') == (report.get('behavior_completion')=='COMPLETE'), 'normal/behavior completion conflated')
    for pair in report['pairs']:
        require(pair.get('candidate_screen') in ('PASS','FAIL') and pair.get('absolute_quality') in ('PASS','FAIL'), 'seed verdict missing')
        for label,arm in zip(('baseline','candidate'),ARMS):
            run=pair[label]
            require(run.get('arm')==arm and run.get('candidate_count')==campaign.COUNTS[arm]
                and run.get('model_parameter_count')==campaign.PARAMETERS[arm]
                and run.get('training_dataset_size')==1147 and run.get('validation_sample_count')==337,'arm capacity/denominator differs')
            for metric in VAL_METRICS: finite(run['metrics'][metric])
            value=run['geometry']['selected_pass_count']
            require(type(value) is int and 0<=value<=337 and run['geometry']['sample_count']==337,'invalid selected geometry count')


def draw(output,report,measurements):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors=('#2468a2','#bf721a','#28875b')
    fig,axes=plt.subplots(2,1,figsize=(16,10))
    for index,item in enumerate(measurements):
        candidate=item['arm']==ARMS[1]; bins=item['bins']
        label=f"{item['seed']}: {'B DRIVE+STOP K12' if candidate else 'A DRIVE K6'}"
        for axis,metric in zip(axes,TRAIN_METRICS):
            axis.plot([(b['first_step']+b['last_step'])/2 for b in bins],[b[metric] for b in bins],
                color=colors[index//2],linestyle='--' if candidate else '-',marker='.',label=label)
    for axis,metric in zip(axes,TRAIN_METRICS):
        axis.set(xlabel='Optimizer step',ylabel='Sample-weighted '+metric,xlim=(1,1540))
        axis.axvspan(1500.5,1540.5,color='gray',alpha=.12);axis.grid(alpha=.2)
    fig.legend(*axes[0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.95),ncol=3,fontsize=9)
    fig.suptitle('All 6 fresh fits | 1,540 steps / 6,155 exposures each | measured training only',fontsize=15)
    fig.text(.5,.055,'100-step sample-weighted bins; shaded last bin = 40 steps / 160 exposures. No intermediate validation.',ha='center',fontsize=10)
    fig.text(.5,.027,'Same loss coefficients, different best-of-K and capacity: A 954,590 / B 1,056,362 parameters. Not a capacity-matched effect.',ha='center',fontsize=10)
    fig.subplots_adjust(top=.84,bottom=.14,left=.085,right=.975,hspace=.30)
    fig.savefig(output/'01_measured_learning.png',dpi=120);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(16,10))
    for axis,metric in zip(axes.flat,(*VAL_METRICS,'selected_geometry_pass_count')):
        peak=0.
        for index,pair in enumerate(report['pairs']):
            for offset,label,color in ((-.18,'baseline','#2468a2'),(.18,'candidate','#bf721a')):
                run=pair[label];value=run['geometry']['selected_pass_count'] if metric=='selected_geometry_pass_count' else run['metrics'][metric]
                axis.bar(index+offset,value,.34,color=color,label='A DRIVE K6' if label=='baseline' else 'B DRIVE+STOP K12')
                axis.text(index+offset,value,f'{value}/337' if metric=='selected_geometry_pass_count' else f'{value:.3f}',ha='center',va='bottom',fontsize=9)
                peak=max(peak,value)
        axis.set_xticks(range(3),[f"{p['seed']}\nRelative {p['candidate_screen']} / Absolute {p['absolute_quality']}" for p in report['pairs']],fontsize=8)
        axis.set(ylabel=metric,ylim=(0,max(.01,peak)*1.20));axis.grid(axis='y',alpha=.2)
    handles,labels=axes.flat[0].get_legend_handles_labels();fig.legend(handles[:2],labels[:2],loc='upper center',bbox_to_anchor=(.5,.94),ncol=2)
    fig.suptitle('Final checkpoint val337 | all three seeds | original runtime v8 thresholds',fontsize=15)
    fig.text(.5,.058,f"Relative: {report['candidate_screen']} | Absolute: {report['absolute_quality']} | Normal stages 18/18 | Behavior {report['completed_behavior_count']}/6",ha='center')
    fig.text(.5,.028,'No automatic promotion, data admission, learned closed-loop or real-vehicle approval. Full candidate failures remain in JSON.',ha='center',fontsize=10)
    fig.subplots_adjust(top=.84,bottom=.15,left=.08,right=.975,hspace=.3,wspace=.23)
    fig.savefig(output/'02_paired_validation.png',dpi=120);plt.close(fig)


def render(root,summary_path,output,behavior_root=None):
    root,summary_path,output=(Path(p).absolute() for p in (root,summary_path,output))
    forbidden=[root.resolve(),summary_path.parent.resolve(),(ROOT/'datasets').resolve()]
    if behavior_root is not None: forbidden.append(Path(behavior_root).resolve())
    require(not output.exists() and all(not p.is_symlink() for p in (output,*output.parents))
        and not any(output.resolve().is_relative_to(p) for p in forbidden),'fresh output must be outside all inputs/datasets')
    payload=read(summary_path);report=_loads_json(payload.decode(),'STOPMIX summary')
    actual=campaign.summarize_campaign(root,expected_source_commit=report.get('source_commit'),
        expected_plan_sha256=report.get('plan_sha256'),behavior_root=behavior_root)
    require(campaign.canonical(actual)==campaign.canonical(report),'supplied summary differs from freshly verified evidence')
    validate_summary(report)
    pins={}
    for entry in report['input_manifest']:
        name=entry['path'];relative=Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts and name not in pins,'unsafe/duplicate campaign path')
        require(campaign.checked_hash(root/name)==entry['sha256'],'campaign input SHA differs');pins[name]=entry['sha256']
    measurements=[]
    for pair in report['pairs']:
        for label,arm in zip(('baseline','candidate'),ARMS):
            prefix=f"seed_{pair['seed']}/{arm}";name=prefix+'/training/metrics.jsonl';raw=read(root/name)
            rows,bins=history(raw);pins[name]=campaign.base._sha(raw)
            training=campaign.read(root/prefix/'training/run.json');evaluation=campaign.read(root/prefix/'evaluation/metrics.json')
            require(campaign.canonical(rows[-1])==campaign.canonical(training.get('last_metrics')),'history final row differs from run.json')
            require(all(evaluation['metrics'].get(k)==pair[label]['metrics'][k] for k in VAL_METRICS),'plotted validation metric differs')
            measurements.append(dict(seed=pair['seed'],arm=arm,bins=bins,optimizer_steps=1540,sample_exposures=6155))
    modules=(campaign,campaign.base,campaign.common,campaign.common.shared,campaign.expansion,legacy)
    paths={Path(__file__).resolve(),*(Path(m.__file__).resolve() for m in modules)}
    sources={p.relative_to(ROOT).as_posix():campaign.sha_file(p) for p in paths}
    output.mkdir(parents=True,exist_ok=False);draw(output,report,measurements)
    require(read(summary_path)==payload and all(campaign.checked_hash(root/n)==v for n,v in pins.items()),'plot inputs changed during rendering')
    if behavior_root is not None:
        require(all(campaign.checked_hash(Path(behavior_root)/r['path'])==r['sha256'] for r in report['behavior_input_manifest']),'behavior input changed during rendering')
    require(all(campaign.sha_file(ROOT/n)==v for n,v in sources.items()),'plot source changed during rendering')
    proof=dict(schema='portable_e2e.stopmix_plots.v1',summary_sha256=campaign.base._sha(payload),input_layout='single_campaign_root_v1',
        inputs=[dict(path=n,sha256=v) for n,v in sorted(pins.items())],behavior_inputs=report['behavior_input_manifest'],
        source_sha256=sources,training_bins=measurements,plotted_training_metrics=list(TRAIN_METRICS),
        normal_stage_completion=report['normal_stage_completion'],behavior_completion=report['behavior_completion'],
        capacity_matched=False,model_loaded=False,model_training=False,test_evaluated=False,training_data_approved=False,
        note='HH_260906 - All batches and failed seeds retained; K and parameter capacity differ. No intermediate validation or closed-loop proof.')
    with (output/'plot_inputs.json').open('x') as stream:stream.write(json.dumps(proof,indent=2,allow_nan=False)+'\n')
    paths=sorted(output.iterdir())
    with (output/'SHA256SUMS').open('x') as stream:
        for path in paths:stream.write(f'{campaign.sha_file(path)}  {path.name}\n')
    return proof


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('campaign_root',type=Path);parser.add_argument('--summary',type=Path,required=True)
    parser.add_argument('--behavior-root',type=Path);parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args(argv);render(args.campaign_root,args.summary,args.output_dir,args.behavior_root);return 0


if __name__=='__main__':
    raise SystemExit(main())
