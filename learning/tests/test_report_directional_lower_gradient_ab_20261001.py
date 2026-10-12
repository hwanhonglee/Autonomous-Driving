"""HH_261001 - Reject partial/mixed accounting without reading the active directional fit."""

from copy import deepcopy
import json
from pathlib import Path
import pytest
from scripts.e2e import report_directional_lower_gradient_ab_20261001 as report


def trace(step=1000):
    return dict(scheduled_step=step,optimizer_updates=step-int(step>=1693),indices=[1,2,3,4],
        backward_calls=64,lower_saturated=10,lower_inward=3,lower_outward=2,lower_zero=5,
        minimum_preclamp_speed=-.2)


def history():
    rows=[]
    for step in sorted(list(range(100,3001,100))+[1693]):
        row=dict(scheduled_step=step,optimizer_updates=step-int(step>=1693))
        if step==1693:row.update(skipped=True,no_forward_or_optimizer_call=True,indices=[617,619,322,317])
        else:row.update(total_loss=1.,base_loss=1.,near_loss=1.,gradient_norm=1.,elapsed_s=1.)
        rows.append(row)
    return rows


def binding():
    complete={arm:dict(checkpoint_sha256=str(i+1)*64,
        final_model_state_sha256=report.original.W1_STATE if i==0 else 'b'*64) for i,arm in enumerate(report.ARMS)}
    endpoints={arm:dict(checkpoint=arm+'/training/final_checkpoint.pt',checkpoint_sha256=complete[arm]['checkpoint_sha256'],
        model_state_sha256=complete[arm]['final_model_state_sha256'],weights='exports/'+arm+'.npz',
        metadata='exports/'+arm+'.json') for arm in report.ARMS}
    value=dict(schema='portable_e2e.directional_gradient_export.v1',status='BOTH_FIXED_ENDPOINTS_EXPORTED_NOT_PROMOTED',
        training_schema=report.SCHEMA,training_source_closure_sha256=report.CLOSURE_SHA,
        training_plan_sha256=report.TRAIN_PLAN,fixed_fit_each_arm=deepcopy(report.FIT),
        parent_checkpoint_sha256=report.original.PARENT_SHA,current_and_archived_training_sources_verified_before_and_after=True,
        original_inference_model=True,trained_surrogate_required_at_runtime=False,endpoint_selection_used_metric_values=False,
        vehicle_control_approved=False,model_promoted=False,simulator_launched=False,optimizer_updates=0,
        export_source_sha256=dict(report.EXPORT_PINS),endpoints=endpoints)
    return value,complete


def test_exact_history_and_native_trace_partition():
    assert report.validate_history(history())['actual_final_updates']==2999
    assert report.validate_trace(trace(),1000,[1,2,3,4])['inward_fraction_of_lower']==.3


@pytest.mark.parametrize('field,value',[('backward_calls',63),('lower_saturated',11),('lower_inward',-1),
    ('lower_outward',True),('minimum_preclamp_speed',float('nan')),('indices',[4,3,2,1]),('optimizer_updates',999)])
def test_wrong_trace_accounting_rejected(field,value):
    row=trace();row[field]=value
    with pytest.raises(ValueError):report.validate_trace(row,1000,[1,2,3,4])


def test_unregistered_trace_and_partial_history_rejected():
    with pytest.raises(ValueError):report.validate_trace(trace(200),200,[1,2,3,4])
    with pytest.raises(ValueError):report.validate_history(history()[:-1])
    rows=history();rows[-1]['optimizer_updates']=3000
    with pytest.raises(ValueError):report.validate_history(rows)


def test_complete_dual_export_not_selected_single_arm():
    value,complete=binding();report.validate_export_binding(value,complete)
    value['endpoints'].pop(report.ARMS[0])
    with pytest.raises(ValueError):report.validate_export_binding(value,complete)


@pytest.mark.parametrize('field,value',[('current_and_archived_training_sources_verified_before_and_after',False),
    ('trained_surrogate_required_at_runtime',True),('endpoint_selection_used_metric_values',True),
    ('training_schema','wrong'),('fixed_fit_each_arm',{'global_step':11996})])
def test_scope_recipe_and_both2999updates_required(field,value):
    obj,complete=binding();obj[field]=value
    with pytest.raises(ValueError):report.validate_export_binding(obj,complete)


def test_relabel_preserves_actual_values_and_integer_candidate_keys():
    value={'W1':{'counts':{0:3},'token':'literal_W1_name'},'W2_minus_W1':2.5}
    assert report.relabel(value)=={'A':{'counts':{0:3},'token':'literal_W1_name'},'B_minus_A':2.5}
    assert value['W1']['token']=='literal_W1_name'


def test_existing_frozen279_self_comparison_uses_exact_original_groups():
    root=report.ROOT/'storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1'
    declaration=json.loads((root/'declaration.json').read_text())
    proof=json.loads((root/'provenance_receipt.json').read_text())
    rows=json.loads((root/report.original.W1/'train_probe_after.json').read_text())
    models,changes=report.original.compare_probes(rows,deepcopy(rows),declaration,proof)
    renamed=report.relabel(models)
    assert renamed['A']['groups']==renamed['B']['groups']
    assert renamed['A']['groups']['protected_stop_hold200']['count']==200
    assert len(report.relabel(changes))==279


def test_unsafe_evidence_path_rejected(tmp_path):
    with pytest.raises(ValueError):report.evidence_relative(tmp_path,'../checkpoint.pt')
    with pytest.raises(ValueError):report.evidence_relative(tmp_path,'/checkpoint.pt')


def test_pinned_helpers_and_protocol_unchanged():
    assert all(report.sha(path)==expected for path,expected in report.HELPER_PINS.items())
    assert all(report.sha(report.ROOT/name)==expected for name,expected in report.EXPORT_PINS.items())
