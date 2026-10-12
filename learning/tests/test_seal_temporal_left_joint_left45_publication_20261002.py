"""HH_261002 - Synthetic closed-archive sealing only; no actual captures, weights, remote jobs or simulator."""
import json
from pathlib import Path
import pytest
from scripts.e2e import seal_temporal_left_joint_left45_publication_20261002 as s


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))
    return s.pub.recorded.digest(path)


def fixture(tmp_path, monkeypatch):
    root=tmp_path.resolve(); pair=root/'pair'; output=root/'plans/caller.json'
    monkeypatch.setattr(s,'ROOT',root); monkeypatch.setattr(s,'OUTPUT',output)
    monkeypatch.setattr(s.pub,'PAIR',pair)
    # HH_261002 - Limit the synthetic process descriptor without affecting actual runtime modules.
    monkeypatch.setattr(s,'sys',type('Synthetic',(),{'modules':{}}))
    monkeypatch.setattr(s.pub,'public_contract',lambda:{'BOTH':True})
    done=root/'plans/completed.json'; pins={'completion':put(done,{'complete':True})}
    cases={}
    for arm in s.pub.ARMS:
        cases[arm]=put(pair/(arm+'_plan.json'),{'arm':arm})
        pins[arm+'_plan']=cases[arm]
        owner={'cleanup_errors':[],'source_hashes_match':True,'external_source_hashes_match':True,
            'worker_exit_code':1 if arm=='parent6000' else 0}
        pins[arm+'_owner']=put(pair/arm/'owner_result.json',owner)
        pins[arm+'_diagnostic']=put(pair/arm/'diagnostic_report.json',{'original_owner':owner,
            'status':'FAIL_RETAINED','model_promoted':False,'vehicle_control_approved':False})
        put(pair/arm/'episode.partial/raw.json',{'raw':'full_failure_retained'})
    pins['batch']=put(pair/'batch_plan.json',{'arm_order':list(s.pub.ARMS),'physical_case_count':2,
        'completed_run_authority_sha256':pins['completion'],'case_sha256':cases})
    a={'schema':s.SCHEMA,'purpose':'BOTH_closed_archives_publication_only','model_promoted':False,
        'vehicle_control_approved':False,'pair':str(pair),'output':str(output),
        'source_tool_sha256':s.pub.recorded.digest(s.__file__),
        'completed_run_authority':str(done),'document_sha256':pins}
    path=root/'plans/authority.json'; sha=put(path,a)
    return path,sha,a


def test_synthetic_BOTH_full_archives_and_original_failures_retained(tmp_path,monkeypatch):
    path,sha,_=fixture(tmp_path,monkeypatch);result=s.seal(path,sha)
    assert set(result['runs'])==set(s.pub.ARMS) and result['publication_authorized']
    assert all('episode.partial/raw.json' in r for r in result['runs'].values())
    assert result['case_plan_sha256'].keys()==result['runs'].keys()
    assert not s.OUTPUT.exists()


@pytest.mark.parametrize('fault',['scope','source','missing','extra','pair','output','sha','promote','vehicle'])
def test_ROOT_authority_first_read_denials(tmp_path,monkeypatch,fault):
    path,sha,a=fixture(tmp_path,monkeypatch)
    if fault=='scope':a['purpose']='execute_vehicle'
    elif fault=='source':a['source_tool_sha256']='a'*64
    elif fault=='missing':a['document_sha256'].pop('LEFT20000_owner')
    elif fault=='extra':a['document_sha256']['other']='a'*64
    elif fault=='pair':a['pair']='/foreign'
    elif fault=='output':a['output']='/foreign'
    elif fault=='promote':a['model_promoted']=True
    elif fault=='vehicle':a['vehicle_control_approved']=True
    if fault=='sha':sha='a'*64
    else:sha=put(path,a)
    with pytest.raises((ValueError,RuntimeError)):s.seal(path,sha)
    assert not s.OUTPUT.exists()


@pytest.mark.parametrize('fault',['cleanup','source','external','outcome','changed','symlink'])
def test_closed_archive_evidence_denials(tmp_path,monkeypatch,fault):
    path,sha,a=fixture(tmp_path,monkeypatch);arm='parent6000';o=s.pub.PAIR/arm/'owner_result.json'
    value=json.loads(o.read_bytes())
    if fault=='changed':o.write_text('{}')
    elif fault=='symlink':(s.pub.PAIR/arm/'foreign').symlink_to(o)
    else:
        if fault=='cleanup':value['cleanup_errors']=['failed']
        elif fault=='source':value['source_hashes_match']=False
        elif fault=='external':value['external_source_hashes_match']=False
        else:value['worker_exit_code']=2
        a['document_sha256'][arm+'_owner']=put(o,value);sha=put(path,a)
    with pytest.raises((ValueError,RuntimeError)):s.seal(path,sha)


def test_create_only_cli_and_second_attempt_never_overwrites(tmp_path,monkeypatch,capsys):
    path,sha,_=fixture(tmp_path,monkeypatch)
    assert s.main(['--authority',str(path),'--authority-sha256',sha])==0
    before=s.OUTPUT.read_bytes();result=json.loads(capsys.readouterr().out)
    assert result['status']=='BOTH_CLOSED_ARCHIVES_SEALED_NO_RENDER_NO_APPROVAL'
    with pytest.raises((ValueError,RuntimeError)):s.main(['--authority',str(path),'--authority-sha256',sha])
    assert s.OUTPUT.read_bytes()==before


def test_duplicate_document_keys_rejected(tmp_path,monkeypatch):
    root=tmp_path.resolve();monkeypatch.setattr(s,'ROOT',root)
    path=root/'bad.json';path.write_text('{"a":1,"a":2}')
    with pytest.raises((ValueError,RuntimeError)):s.document(path,s.pub.recorded.digest(path))
