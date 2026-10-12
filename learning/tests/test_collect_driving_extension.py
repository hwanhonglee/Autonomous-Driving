"""HH_260906 - Publish only the new driving run without implying new mapping training."""
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('driving_publication',ROOT/'tools/e2e/collect_dual_learning_results.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_driving_only_publication(tmp_path,monkeypatch):
    monkeypatch.setattr(module,'__file__',str(tmp_path/'tools/e2e/collect.py'))
    output=tmp_path/'docs/assets/validation/result'
    monkeypatch.setattr(sys,'argv',['collect','--output',str(output),'--data',str(tmp_path),
                                   '--driving-job','full_recovery_ag_test','--driving-only'])
    requested=[]
    def ssh(command):
        requested.append(command)
        return 'READY'
    def fetch(remote,local):
        requested.append(remote)
        metrics={'metrics':{'selected_ade_m':1.}}
        local.write_text(json.dumps({'status':'COMPLETE_NOT_PROMOTED',
            'evaluation':{'subgroups':{'town03_right_turn_dev':metrics,'town03_straight_dev':metrics}},
            'versus_original_k1':{'pass_count':8},'versus_parent_af':{'pass_count':7},
            'strict_gate':{'metric_pass_count':6}}))
    monkeypatch.setattr(module,'ssh',ssh)
    monkeypatch.setattr(module,'fetch',fetch)
    module.main()
    assert not (output/'mapping').exists()
    assert all('map_perception' not in command for command in requested)
    assert json.loads((output/'complete.json').read_text())['status']=='DRIVING_RESULT_PUBLISHED_NOT_PROMOTED'
    assert '지도 학습은 이번 실행에 포함되지 않았습니다' in (output/'README.md').read_text()
