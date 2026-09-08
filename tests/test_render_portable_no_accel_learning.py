"""HH_260906 - Test synthetic plot plumbing and rejection paths; these fixtures are never real training evidence."""

import copy
import json
from pathlib import Path

import pytest

from scripts.e2e import render_portable_no_accel_learning as module


def rows():
    seen, result = 0, []
    for step in range(1,1541):
        count=3 if step%287==0 else 4; seen+=count
        result.append({'global_step':step,'epoch':(step-1)//287,'samples_seen':seen,
            'batch_domain_sample_counts':{'carla':count},'domain_samples_seen':{'carla':seen},
            'loss':2. if count==3 else 1.,'selected_ade_m':step/100.})
    return result


def encoded(value):
    return ('\n'.join(json.dumps(row) for row in value)+'\n').encode()


def test_all_rows_sample_weighted_and_last40_not_dropped():
    values,bins=module.history(encoded(rows()))
    assert len(values)==1540 and len(bins)==16
    assert bins[-1]['first_step']==1501 and bins[-1]['last_step']==1540
    assert bins[-1]['batch_count']==40 and bins[-1]['sample_exposures']==160
    assert bins[-1]['partial_display_bin'] is True
    assert sum(b['sample_exposures'] for b in bins)==6155
    assert bins[2]['sample_exposures']==399
    assert bins[2]['loss']==pytest.approx((99*4+2*3)/399)
    assert bins[-1]['selected_ade_m']==pytest.approx(sum(i/100 for i in range(1501,1541))/40)


@pytest.mark.parametrize('fault',['missing','duplicate','order','epoch','batch','seen','domain','bool','nan','negative','overflow'])
def test_bad_history_rejected(fault):
    data=rows()
    if fault=='missing':data.pop()
    elif fault=='duplicate':data[100]=dict(data[99])
    elif fault=='order':data[5],data[6]=data[6],data[5]
    elif fault=='epoch':data[287]['epoch']=0
    elif fault=='batch':data[286]['batch_domain_sample_counts']['carla']=4
    elif fault=='seen':data[286]['samples_seen']+=1
    elif fault=='domain':data[0]['domain_samples_seen']={'real':4}
    elif fault=='bool':data[0]['global_step']=True
    elif fault=='nan':data[0]['loss']=float('nan')
    elif fault=='negative':data[0]['selected_ade_m']=-1
    else:data[0]['loss']=float('inf')
    with pytest.raises((module.ContractError,ValueError)):
        module.history(encoded(data))


@pytest.fixture
def inputs(tmp_path,monkeypatch):
    root=tmp_path/'campaign';root.mkdir();summary=tmp_path/'summary'/'summary.json';summary.parent.mkdir();out=tmp_path/'plots'
    report={'schema':module.campaign.SCHEMA,'status':'COMPLETE_NOT_PROMOTED','automatic_promotion':False,
        'vehicle_control_approved':False,'test_evaluated':False,'test_used_for_training_or_selection':False,
        'candidate_screen':'FAIL','absolute_quality':'FAIL','stages':[{'status':'COMPLETE'}]*18,'pairs':[],'input_manifest':[]}
    values=rows(); payload=encoded(values)
    for seed in module.SEEDS:
        pair={'seed':seed}
        for label,arm in zip(('baseline','candidate'),module.ARMS):
            relative=f'seed_{seed}/{arm}';run=root/relative
            (run/'training').mkdir(parents=True);(run/'evaluation').mkdir()
            metrics={'selected_ade_m':4.,'selected_fde_m':10.,'selected_speed_mae_mps':.8}
            pair[label]={'arm':arm,'training_dataset_size':1147,'validation_sample_count':337,
                'metrics':{**metrics,'selection_regret_ade_m':.5},'geometry':{'selected_pass_count':336,'sample_count':337}}
            (run/'training/metrics.jsonl').write_bytes(payload)
            # HH_260906 - Actual evaluation has extra losses while the strict summary has derived regret; only plotted keys match.
            for name,value in [('training/run.json',{'last_metrics':values[-1]}),('evaluation/metrics.json',{'metrics':{**metrics,'loss':3.,'regression_loss':2.}})]:
                path=run/name;path.write_text(json.dumps(value))
                report['input_manifest'].append({'path':relative+'/'+name,'sha256':module.campaign.sha_file(path)})
        report['pairs'].append(pair)
    summary.write_text(json.dumps(report));monkeypatch.setattr(module.campaign,'summarize_campaign',lambda *_:copy.deepcopy(report))
    return root,summary,out,report


def test_mock_verified_summary_render_only_real_numeric_fixture(inputs):
    pytest.importorskip('matplotlib');image=pytest.importorskip('PIL.Image')
    root,summary,out,report=inputs
    raw=json.loads((root/f'seed_{module.SEEDS[0]}/{module.ARMS[0]}/evaluation/metrics.json').read_text())
    assert 'loss' in raw['metrics'] and 'loss' not in report['pairs'][0]['baseline']['metrics']
    assert 'selection_regret_ade_m' not in raw['metrics'] and 'selection_regret_ade_m' in report['pairs'][0]['baseline']['metrics']
    proof=module.render(root,summary,out)
    assert proof['summary_sha256']==module.campaign.sha_file(summary)
    assert len(proof['training_bins'])==6 and len(proof['inputs'])==18
    assert proof['model_loaded'] is proof['model_training'] is proof['training_data_approved'] is False
    assert all(p['bins'][-1]['batch_count']==40 for p in proof['training_bins'])
    for name in ('01_training_curves.png','02_paired_validation.png'):
        with image.open(out/name) as frame:
            assert frame.width==1920;frame.verify()
    for line in (out/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ');assert module.campaign.sha_file(out/name)==digest


@pytest.mark.parametrize('fault',['incomplete','stale_summary','duplicate_manifest','last_metric','raw_eval','mutated_input','existing','inside','symlink'])
def test_publication_rejects_unsafe_or_unbound_data(inputs,monkeypatch,fault):
    root,summary,out,report=inputs
    monkeypatch.setattr(module,'draw',lambda *_:None)
    if fault=='incomplete':report['status']='INCOMPLETE';summary.write_text(json.dumps(report))
    elif fault=='stale_summary':report['candidate_screen']='PASS'
    elif fault=='duplicate_manifest':report['input_manifest'].append(report['input_manifest'][0]);summary.write_text(json.dumps(report))
    elif fault in ('last_metric','raw_eval'):
        suffix='training/run.json' if fault=='last_metric' else 'evaluation/metrics.json'
        entry=next(e for e in report['input_manifest'] if e['path'].endswith(suffix));path=root/entry['path']
        data=json.loads(path.read_text())
        if fault=='last_metric':data['last_metrics']['loss']+=1
        else:data['metrics']['selected_ade_m']+=1
        path.write_text(json.dumps(data));entry['sha256']=module.campaign.sha_file(path);summary.write_text(json.dumps(report))
    elif fault=='mutated_input':
        path=root/f'seed_{module.SEEDS[0]}/{module.ARMS[0]}/training/metrics.jsonl'
        monkeypatch.setattr(module,'draw',lambda *_:path.write_text('changed\n'))
    elif fault=='existing':out.mkdir()
    elif fault=='inside':out=root/'plots'
    else:out.symlink_to(out.parent/'absent',target_is_directory=True)
    with pytest.raises((module.ContractError,ValueError)):
        module.render(root,summary,out)
    assert not (out/'SHA256SUMS').exists()


def test_dataset_physical_alias_rejected(inputs,tmp_path,monkeypatch):
    root,summary,_,_=inputs;repo=tmp_path/'repo';repo.mkdir();dataset=tmp_path/'physical';dataset.mkdir()
    (repo/'datasets').symlink_to(dataset,target_is_directory=True);monkeypatch.setattr(module,'ROOT',repo)
    with pytest.raises(module.ContractError,match='outside'):
        module.render(root,summary,dataset/'plots')
