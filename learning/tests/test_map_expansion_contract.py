"""HH_260906 - Prevent expanded TRAIN experiments from rewriting parent data or DEV."""
import copy
import pytest
from scripts.e2e.map_expansion_contract import validate_expansion


def fixtures():
    parent={'splits':{'train':{'count':1,'source_records':[{'frame':1}],'source_hashes':{'a':'sha'}},'dev':{'sha256':'dev'}}}
    new=copy.deepcopy(parent);new['parent_manifest_sha256']='parent'
    new['splits']['train'].update(count=2,source_records=[{'frame':1},{'frame':2}],source_hashes={'a':'sha','b':'new'})
    new['expansion']=dict(added_frames=1,minimum_dev_distance_m=100,owner_exit_code=0,collision_events=0,
        source_bytes_unchanged_and_archived=True,physical_capture=True,dev_used_for_training=False)
    return new,parent


def test_additive_train_accepted():
    new,parent=fixtures();validate_expansion(new,parent,'parent')


@pytest.mark.parametrize('change',['dev','old_record','old_hash','distance','collision','physical'])
def test_invalid_expansion_rejected(change):
    new,parent=fixtures()
    if change=='dev':new['splits']['dev']['sha256']='changed'
    elif change=='old_record':new['splits']['train']['source_records'][0]['frame']=7
    elif change=='old_hash':new['splits']['train']['source_hashes']['a']='changed'
    elif change=='distance':new['expansion']['minimum_dev_distance_m']=79
    elif change=='collision':new['expansion']['collision_events']=1
    elif change=='physical':new['expansion']['physical_capture']=False
    with pytest.raises(ValueError):validate_expansion(new,parent,'parent')
