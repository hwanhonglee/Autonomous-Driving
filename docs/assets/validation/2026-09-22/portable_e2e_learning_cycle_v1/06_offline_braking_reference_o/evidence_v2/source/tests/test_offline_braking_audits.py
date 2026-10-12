"""HH_260906 - Test read-only target and braking diagnostics on tiny synthetic fixtures."""
import hashlib
import json
import pytest
from scripts.e2e.audit_existing_train_targets import audit
from scripts.e2e.analyze_m_braking_events import analyze


def write(path,value):
    """HH_260906 - Create isolated test evidence only under pytest's temporary directory."""
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fixture_dataset(root,split='train'):
    """HH_260906 - Build one stationary valid future and one speed-inconsistent future."""
    identity='carla_town01_fixture'
    rows=[]
    for i,speed in enumerate((0.,20.)):
        rows.append(dict(sample_id=str(i),ego=dict(linear_velocity_base_mps=[0,0,0]),
            labels=dict(planning=dict(valid=[True]*64,speed_mps=[speed]*64,
                        yaw_rad=[0.]*64,positions_base_xy_m=[[0.,0.]]*64))))
    sample=root/'episode/samples.jsonl';sample.parent.mkdir(parents=True)
    sample.write_text('\n'.join(map(json.dumps,rows)))
    sha=write(root/'episode/episode.json',dict(split=split,sample_jsonl='samples.jsonl',sample_count=2,
                                            sample_jsonl_sha256=hashlib.sha256(sample.read_bytes()).hexdigest()))
    write(root/'dataset.json',dict(episodes=[dict(episode_id=identity,manifest='episode/episode.json',sha256=sha)]))
    return identity


def test_target_pass_and_fail_are_separate(tmp_path):
    """HH_260906 - Report failure without filtering or changing source samples."""
    identity=fixture_dataset(tmp_path);before=(tmp_path/'episode/samples.jsonl').read_bytes()
    report=audit(tmp_path,[identity]);episode=report['episodes'][0]
    assert episode['sample_count']==2 and episode['passing']==1
    assert not report['labels_modified'] and not report['filtering_applied']
    assert (tmp_path/'episode/samples.jsonl').read_bytes()==before


def test_dev_split_rejected(tmp_path):
    """HH_260906 - A TRAIN map name does not override an episode's actual DEV split."""
    identity=fixture_dataset(tmp_path,split='val')
    with pytest.raises(ValueError,match='not TRAIN'):audit(tmp_path,[identity])


def test_protected_map_rejected_before_episode_read(tmp_path):
    """HH_260906 - Held-out episode references do not authorize payload reads."""
    fixture_dataset(tmp_path)
    with pytest.raises(ValueError,match='allowlist'):audit(tmp_path,['carla_town04_missing'])


def test_corrupt_samples_rejected(tmp_path):
    """HH_260906 - Authenticate sample bytes before evaluating labels."""
    identity=fixture_dataset(tmp_path);(tmp_path/'episode/samples.jsonl').write_text('changed')
    with pytest.raises(ValueError,match='checksum'):audit(tmp_path,[identity])


def test_braking_regions_and_unobserved_signal(tmp_path):
    """HH_260906 - Separate tail braking without pretending traffic-light state is recorded."""
    episode=tmp_path/'episode';episode.mkdir()
    write(episode/'manifest.json',dict(result=dict(route_length_m=50)))
    rows=[]
    for i,v in enumerate((4.,3.,2.)):
        rows.append(dict(frame=i,timestamp=i*.05,vx=v,vy=0,route_progress_m=49,
                         capture_phase='stationary_tail',current_control=dict(brake=1),next_control=dict(brake=1)))
    (episode/'states.jsonl').write_text('\n'.join(map(json.dumps,rows)))
    (episode/'camera_frames.jsonl').write_text('\n'.join(map(json.dumps,[dict(frame=0),dict(frame=2)])))
    r=analyze(tmp_path)
    assert r['cadences']['native20hz']['violation_count']==2
    assert r['cadences']['camera10hz']['violation_count']==1
    row=r['cadences']['camera10hz']['violation_intervals'][0]
    assert row['region']=='tail_transition_or_hold'
    assert row['traffic_signal_cause']=='unknown_not_recorded'
