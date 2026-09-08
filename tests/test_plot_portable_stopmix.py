"""HH_260906 - Render synthetic numeric fixtures only; never treat test graphs as actual campaign evidence."""

from copy import deepcopy
import json

import pytest

from scripts.e2e import plot_portable_stopmix as module
from test_render_portable_no_accel_learning import rows as base_rows, encoded


def rows():
    values=base_rows()
    for row in values: row['regression_loss']=2.0 if row['batch_domain_sample_counts']['carla']==3 else 1.0
    return values


def test_all1540_batches_weighted_and_final40_not_dropped():
    values,bins=module.history(encoded(rows()))
    assert len(values)==1540 and len(bins)==16 and sum(b['sample_exposures'] for b in bins)==6155
    assert bins[-1]['batch_count']==40 and bins[-1]['sample_exposures']==160 and bins[-1]['partial_display_bin']
    assert bins[2]['regression_loss']==pytest.approx((396+6)/399)
    assert all('loss' not in b for b in bins)


@pytest.mark.parametrize('fault',['missing','duplicate','epoch','batch','samples','nan','negative','bool','auxiliary'])
def test_bad_histories_fail_closed(fault):
    data=rows()
    if fault=='missing':data.pop()
    elif fault=='duplicate':data[12]=deepcopy(data[11])
    elif fault=='epoch':data[287]['epoch']=0
    elif fault=='batch':data[286]['batch_domain_sample_counts']['carla']=4
    elif fault=='samples':data[9]['samples_seen']+=1
    elif fault=='nan':data[0]['regression_loss']=float('nan')
    elif fault=='negative':data[0]['regression_loss']=-.1
    elif fault=='bool':data[0]['regression_loss']=True
    else:data[0]['candidate_regret_loss']=0.
    with pytest.raises((module.campaign.ContractError,ValueError)):module.history(encoded(data))


@pytest.fixture
def inputs(tmp_path,monkeypatch):
    root=tmp_path/'campaign';root.mkdir();summary=tmp_path/'summary/summary.json';summary.parent.mkdir();output=tmp_path/'plots'
    report=dict(schema=module.campaign.SCHEMA,status='COMPLETE_NORMAL_STAGES',normal_stage_completion='COMPLETE',behavior_completion='INCOMPLETE',
        automatic_promotion=False,vehicle_control_approved=False,training_data_approved_by_this_report=False,
        test_evaluated=False,test_used_for_training_or_selection=False,completed_stage_count=18,completed_behavior_count=0,
        candidate_screen='FAIL',absolute_quality='FAIL',pairs=[],input_manifest=[],behavior_input_manifest=[],
        source_commit='b478f02e42b94bf04bffec5c8170e05edc33b0f8',plan_sha256='f06c68ce47aaee6941a3ca42559460ac5837b53f669bf195fa15b5d5e80a05d9')
    for seed in module.SEEDS:
        pair=dict(seed=seed,candidate_screen='FAIL',absolute_quality='FAIL')
        for label,arm in zip(('baseline','candidate'),module.ARMS):
            prefix=f'seed_{seed}/{arm}';item=root/prefix;(item/'training').mkdir(parents=True);(item/'evaluation').mkdir()
            values=rows();(item/'training/metrics.jsonl').write_bytes(encoded(values))
            training={'last_metrics':values[-1]};metrics={'selected_ade_m':4.,'selected_fde_m':10.,'selected_speed_mae_mps':.8}
            evaluation={'metrics':dict(metrics,loss=3.,regression_loss=2.)}
            pair[label]=dict(arm=arm,candidate_count=module.campaign.COUNTS[arm],model_parameter_count=module.campaign.PARAMETERS[arm],
                training_dataset_size=1147,validation_sample_count=337,metrics=dict(metrics,selection_regret_ade_m=.5),
                geometry={'selected_pass_count':336,'sample_count':337})
            for name,value in (('training/run.json',training),('evaluation/metrics.json',evaluation)):
                path=item/name;path.write_text(json.dumps(value));report['input_manifest'].append(dict(path=prefix+'/'+name,sha256=module.campaign.sha_file(path)))
        report['pairs'].append(pair)
    summary.write_text(json.dumps(report))
    monkeypatch.setattr(module.campaign,'summarize_campaign',lambda *_,**__:deepcopy(report))
    return root,summary,output,report


def test_two_numeric_graphs_preserve_normal_vs_behavior_and_capacity(inputs):
    pytest.importorskip('matplotlib');image=pytest.importorskip('PIL.Image')
    root,summary,output,report=inputs;proof=module.render(root,summary,output)
    assert proof['capacity_matched'] is proof['model_loaded'] is proof['model_training'] is False
    assert proof['normal_stage_completion']=='COMPLETE' and proof['behavior_completion']=='INCOMPLETE'
    assert len(proof['training_bins'])==6 and len(proof['inputs'])==18
    assert proof['summary_sha256']==module.campaign.sha_file(summary)
    assert proof['plotted_training_metrics']==['regression_loss','selected_ade_m']
    for name in ('01_measured_learning.png','02_paired_validation.png'):
        with image.open(output/name) as frame:assert frame.size==(1920,1200);frame.verify()
    for line in (output/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ');assert name!='SHA256SUMS' and module.campaign.sha_file(output/name)==digest


@pytest.mark.parametrize('fault',['incomplete','stale','conflated_behavior','wrong_k','duplicate_manifest','final_row','raw_val',
    'mutated_history','mutated_summary','existing','inside_campaign','inside_summary','symlink'])
def test_fail_closed_outputs(inputs,monkeypatch,fault):
    root,summary,output,report=inputs;monkeypatch.setattr(module,'draw',lambda *_:None)
    if fault=='incomplete':report['status']='INCOMPLETE';summary.write_text(json.dumps(report))
    elif fault=='stale':report['candidate_screen']='PASS'
    elif fault=='conflated_behavior':report['status']='COMPLETE_NOT_PROMOTED';summary.write_text(json.dumps(report))
    elif fault=='wrong_k':report['pairs'][0]['candidate']['candidate_count']=6;summary.write_text(json.dumps(report))
    elif fault=='duplicate_manifest':report['input_manifest'].append(report['input_manifest'][0]);summary.write_text(json.dumps(report))
    elif fault in ('final_row','raw_val'):
        suffix='training/run.json' if fault=='final_row' else 'evaluation/metrics.json'
        entry=next(e for e in report['input_manifest'] if e['path'].endswith(suffix));path=root/entry['path'];value=json.loads(path.read_text())
        if fault=='final_row':value['last_metrics']['regression_loss']+=1
        else:value['metrics']['selected_ade_m']+=1
        path.write_text(json.dumps(value));entry['sha256']=module.campaign.sha_file(path);summary.write_text(json.dumps(report))
    elif fault=='mutated_history':
        path=root/f'seed_{module.SEEDS[0]}/{module.ARMS[0]}/training/metrics.jsonl';monkeypatch.setattr(module,'draw',lambda *_:path.write_text('changed\n'))
    elif fault=='mutated_summary':monkeypatch.setattr(module,'draw',lambda *_:summary.write_text('changed\n'))
    elif fault=='existing':output.mkdir()
    elif fault=='inside_campaign':output=root/'plots'
    elif fault=='inside_summary':output=summary.parent/'plots'
    else:output.symlink_to(output.parent/'absent',target_is_directory=True)
    with pytest.raises((module.campaign.ContractError,ValueError)):module.render(root,summary,output)
    assert not (output/'SHA256SUMS').exists()


def test_physical_dataset_alias_forbidden(inputs,tmp_path,monkeypatch):
    root,summary,_,_=inputs;repo=tmp_path/'repo';repo.mkdir();physical=tmp_path/'physical';physical.mkdir()
    (repo/'datasets').symlink_to(physical,target_is_directory=True);monkeypatch.setattr(module,'ROOT',repo)
    with pytest.raises(module.campaign.ContractError,match='outside'):module.render(root,summary,physical/'plots')
