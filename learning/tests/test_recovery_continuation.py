"""HH_260906 - Reject incomplete, tampered or mismatched AF continuation parents."""
import hashlib
import importlib.util
import json
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('recovery_continuation',ROOT/'tools/e2e/run_recovery_ae.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def fixture_parent(tmp_path):
    checkpoint=tmp_path/'K1_current_plus_successful4/training/final_checkpoint.pt'
    checkpoint.parent.mkdir(parents=True)
    checkpoint.write_bytes(b'test-checkpoint')
    sha=hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    (tmp_path/'baseline_plan.json').write_text('{}')
    plan_sha=hashlib.sha256(b'{}').hexdigest()
    summary={'status':'COMPLETE_NOT_PROMOTED','training':{'steps':15000,'checkpoint_sha256':sha},
             'evaluation':{'checkpoint_sha256':sha}}
    (tmp_path/'summary.json').write_text(json.dumps(summary))
    (tmp_path/'declaration.json').write_text(json.dumps({'objective':'rank_only','preservation_weight':0}))
    return checkpoint,sha,plan_sha


def test_valid_parent(tmp_path):
    checkpoint,sha,plan=fixture_parent(tmp_path)
    assert module.continuation_reference(tmp_path,sha,plan)[0]==checkpoint


@pytest.mark.parametrize('damage',['checkpoint','plan','status','steps','objective','summary_hash','evaluation_hash'])
def test_refuse_invalid_parent(tmp_path,damage):
    checkpoint,sha,plan=fixture_parent(tmp_path)
    if damage=='checkpoint':checkpoint.write_bytes(b'changed')
    elif damage=='plan':(tmp_path/'baseline_plan.json').write_text('{"changed":true}')
    elif damage=='objective':
        (tmp_path/'declaration.json').write_text('{"objective":"rank_retention","preservation_weight":0.5}')
    else:
        path=tmp_path/'summary.json';summary=json.loads(path.read_text())
        if damage=='status':summary['status']='RUNNING'
        elif damage=='steps':summary['training']['steps']=250
        elif damage=='summary_hash':summary['training']['checkpoint_sha256']='0'*64
        else:summary['evaluation']['checkpoint_sha256']='0'*64
        path.write_text(json.dumps(summary))
    with pytest.raises(ValueError):module.continuation_reference(tmp_path,sha,plan)
