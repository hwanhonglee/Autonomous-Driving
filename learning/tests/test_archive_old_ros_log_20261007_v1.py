"""HH_261007 - Synthetic temporary-file backup regressions; no real log, compression, ROS, GPU or deletion."""
import ast
import hashlib
import inspect
import json
import os
from pathlib import Path
import time
import zlib

import pytest

from scripts.maintenance import archive_old_ros_log_20261007_v1 as m


def fixture_file(tmp_path):
    path=tmp_path/'synthetic.log';path.write_bytes(b'synthetic-not-a-ROS-log\n'*17)
    return path,m.generation(path.stat())


def test_no_delete_or_external_launch_on_import():
    assert m.SOURCE==Path('/home/a/.ros/log/component_container_mt_4904_1744181413078.log')
    assert m.OUTPUT==Path('/home/a/thesis_4pc/90_move_records_and_cleanup/ros_log_archive_20261007_v1')
    assert m.ARCHIVE_CAP < m.OUTPUT_CAP==512*1024**2
    assert m.WALL_LIMIT_S==300
    source=inspect.getsource(m)
    tree=ast.parse(source)
    calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
    assert not any(name in calls for name in ('unlink','remove','rmtree','rename','replace'))
    assert 'deletion_implemented=False' in source


def test_whole_pread_hash_preserves_file_offset_and_bytes(tmp_path):
    path,expected=fixture_file(tmp_path);raw=path.read_bytes()
    fd=m.opened_source(path,expected,uid=os.getuid())
    try:
        os.lseek(fd,5,os.SEEK_SET)
        value=m.hash_fd(fd,time.monotonic()+2,byte_limit=len(raw))
        assert value==dict(sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw))
        assert os.lseek(fd,0,os.SEEK_CUR)==5
        m.verify_source(path,fd,expected)
    finally:os.close(fd)
    assert path.read_bytes()==raw


@pytest.mark.parametrize('fault',['symlink','hardlink','wrong_generation','wrong_uid','fifo'])
def test_open_rejects_unapproved_file_kinds_and_generations(tmp_path,fault):
    path,expected=fixture_file(tmp_path)
    if fault=='symlink':
        link=tmp_path/'alias';link.symlink_to(path);path=link
    elif fault=='hardlink':os.link(path,tmp_path/'other')
    elif fault=='wrong_generation':expected={**expected,'inode':expected['inode']+1}
    elif fault=='fifo':
        path=tmp_path/'pipe';os.mkfifo(path);expected=m.generation(path.stat())
    uid=os.getuid()+1 if fault=='wrong_uid' else os.getuid()
    with pytest.raises(ValueError):m.opened_source(path,expected,uid=uid)


def test_source_byte_mutation_after_open_is_denied(tmp_path):
    path,expected=fixture_file(tmp_path);fd=m.opened_source(path,expected,uid=os.getuid())
    try:
        path.write_bytes(b'changed')
        with pytest.raises(ValueError,match='generation'):m.verify_source(path,fd,expected)
    finally:os.close(fd)


@pytest.mark.parametrize('kind',['deadline','byte_limit'])
def test_whole_hash_cannot_skip_timeout_or_extra_bytes(tmp_path,kind):
    path,_=fixture_file(tmp_path);fd=os.open(path,os.O_RDONLY)
    try:
        with pytest.raises(ValueError):m.hash_fd(fd,time.monotonic()+(-1 if kind=='deadline' else 2),byte_limit=1 if kind=='byte_limit' else None)
    finally:os.close(fd)


def test_hash_budget_callback_is_enforced(tmp_path):
    path,_=fixture_file(tmp_path);fd=os.open(path,os.O_RDONLY)
    def deny():raise ValueError('quota blocked')
    try:
        with pytest.raises(ValueError,match='quota'):m.hash_fd(fd,time.monotonic()+2,check=deny)
    finally:os.close(fd)


def test_create_only_receipt_keeps_existing_failure(tmp_path):
    path=tmp_path/'result.json';m.new_json(path,{'failed':True});raw=path.read_bytes()
    with pytest.raises(FileExistsError):m.new_json(path,{'failed':False})
    assert path.read_bytes()==raw


def test_json_size_cap_precedes_file_creation(tmp_path):
    path=tmp_path/'too_large.json'
    with pytest.raises(ValueError,match='bounded'):m.new_json(path,{'payload':'x'*m.JSON_CAP})
    assert not path.exists()


@pytest.mark.parametrize('fault',['bad_pin','duplicate','nonfinite','hardlink'])
def test_root_json_hash_shape_and_single_link_admission(tmp_path,monkeypatch,fault):
    monkeypatch.setattr(m,'UID',os.getuid())
    path=tmp_path/'grant.json'
    raw=(b'{"a":1,"a":2}' if fault=='duplicate' else b'{"a":NaN}' if fault=='nonfinite' else b'{"a":1}')
    path.write_bytes(raw)
    if fault=='hardlink':os.link(path,tmp_path/'second')
    with pytest.raises(ValueError):m.read_pinned_json(path,'0'*64 if fault=='bad_pin' else m.sha(raw))


def test_exact_backup_only_authority_requires_user_approval(tmp_path,monkeypatch):
    plan={'source_sha256':{'fake':'1'*64}}
    monkeypatch.setattr(m,'prospective_plan',lambda:plan)
    auth=dict(schema=m.SCHEMA+'.ROOT_archive_authority',authorized=True,mode='backup_only',
        ROOT_source_execution_review_completed=True,explicit_user_approved_one_log_full_backup=True,
        source=str(m.SOURCE),output=str(m.OUTPUT),plan_sha256='2'*64,source_sha256=plan['source_sha256'],
        no_original_or_other_file_deletion=True,keep_failed_partial_outputs=True,
        no_installation_training_GPU_ROS_CARLA_or_remote_write=True)
    m.verify_authority(plan,auth,'2'*64)
    for wrong in ({**auth,'explicit_user_approved_one_log_full_backup':False},
        {**auth,'mode':'remove'},{**auth,'authorized':1},{**auth,'other_key':True},
        {**auth,'plan_sha256':'3'*64}):
        with pytest.raises(ValueError):m.verify_authority(plan,wrong,'2'*64)


def test_changed_plan_sources_deny_authority(monkeypatch):
    monkeypatch.setattr(m,'prospective_plan',lambda:{'source_sha256':{'exact':'1'*64}})
    with pytest.raises(ValueError,match='exact fresh plan'):m.verify_authority({'source_sha256':{}},{},'2'*64)


def test_plan_default_never_invokes_archive(monkeypatch,capsys):
    monkeypatch.setattr(m,'prospective_plan',lambda:{'status':'PREPARED_ONLY'})
    monkeypatch.setattr(m,'execute',lambda *_:pytest.fail('actual backup must not run'))
    assert m.main([])==0
    assert json.loads(capsys.readouterr().out)=={'status':'PREPARED_ONLY'}


def test_delete_mode_is_not_implemented():
    with pytest.raises(SystemExit):m.main(['--mode','remove'])


def test_unowned_child_is_not_signalled(monkeypatch):
    class Child:
        pid=99999
        def poll(self):return None
    monkeypatch.setattr(m,'process_generation',lambda pid:{'pid':pid,'start_ticks':2})
    monkeypatch.setattr(m.os,'killpg',lambda *_:pytest.fail('unowned process must never be killed'))
    with pytest.raises(ValueError,match='identity'):m.stop_owned(Child(),{'pid':99999,'start_ticks':1})


def fixture_fake_archive(tmp_path,monkeypatch,fault=None):
    # HH_261007 - Mock external compression entirely; exercise full backup admission and retained result lifecycle on tiny fixtures.
    source,expected=fixture_file(tmp_path)
    output=tmp_path/'fresh_archive';plan_path=tmp_path/'ROOT_plan.json'
    auth_path=tmp_path/'ROOT_archive_authority.json';lease=tmp_path/'lease';lease.write_bytes(b'')
    for name,value in dict(SOURCE=source,EXPECTED_GENERATION=expected,OUTPUT=output,
        PLAN=plan_path,AUTHORITY=auth_path,LOCK=lease,UID=os.getuid()).items():
        monkeypatch.setattr(m,name,value)
    plan={'source_sha256':{'synthetic_only':'1'*64}}
    monkeypatch.setattr(m,'prospective_plan',lambda:plan)
    plan_raw=m.encode(plan);plan_path.write_bytes(plan_raw);plan_sha=m.sha(plan_raw)
    auth=dict(schema=m.SCHEMA+'.ROOT_archive_authority',authorized=True,mode='backup_only',
        ROOT_source_execution_review_completed=True,explicit_user_approved_one_log_full_backup=True,
        source=str(source),output=str(output),plan_sha256=plan_sha,source_sha256=plan['source_sha256'],
        no_original_or_other_file_deletion=True,keep_failed_partial_outputs=True,
        no_installation_training_GPU_ROS_CARLA_or_remote_write=True)
    if fault=='authority':auth['explicit_user_approved_one_log_full_backup']=False
    auth_raw=m.encode(auth);auth_path.write_bytes(auth_raw)
    monkeypatch.setattr(m,'budget',lambda output=None:1)
    monkeypatch.setattr(m,'source_fd_census',lambda expected:dict(hits=[],unknown=[{'reason':'FAKE_UNKNOWN'}],complete_nonuse_proven=False,removal_authority=False))
    def fake_zstd(fd,out,deadline,*,decompress=False,expected_size=None):
        data=os.pread(fd,expected['size']+1,0)
        if not decompress:
            (out/m.ARCHIVE_NAME).write_bytes(zlib.compress(data))
            if fault=='compression':raise ValueError('synthetic failed compressor')
            if fault=='source_change':source.write_bytes(b'writer changed source')
            return {'returncode':0,'synthetic_mock_not_actual_zstd':True}
        decoded=zlib.decompress(data)
        return dict(returncode=0,synthetic_mock_not_actual_zstd=True,
            decompressed_sha256='0'*64 if fault=='roundtrip_sha' else m.sha(decoded),
            decompressed_bytes=len(decoded)-1 if fault=='roundtrip_count' else len(decoded))
    monkeypatch.setattr(m,'run_zstd',fake_zstd)
    return source,output,plan_sha,m.sha(auth_raw)


def test_full_fake_backup_verifies_roundtrip_without_removing_source(tmp_path,monkeypatch):
    source,output,plan_sha,auth_sha=fixture_fake_archive(tmp_path,monkeypatch)
    original=source.read_bytes();before=m.generation(source.stat())
    result=m.execute(plan_sha,auth_sha)
    assert result['status']=='VERIFIED_BACKUP_ORIGINAL_RETAINED'
    assert result['recoverable_full_backup_verified'] is True
    assert result['source_deleted'] is result['deletion_implemented'] is False
    assert source.read_bytes()==original and m.generation(source.stat())==before
    assert result['source_fd_audit_after']['complete_nonuse_proven'] is False
    assert zlib.decompress((output/m.ARCHIVE_NAME).read_bytes())==original
    assert json.loads((output/'result.json').read_bytes())==result


@pytest.mark.parametrize('fault',['compression','roundtrip_sha','roundtrip_count','source_change'])
def test_every_fake_archive_failure_keeps_source_and_partial_evidence(tmp_path,monkeypatch,fault):
    source,output,plan_sha,auth_sha=fixture_fake_archive(tmp_path,monkeypatch,fault)
    with pytest.raises(ValueError):m.execute(plan_sha,auth_sha)
    result=json.loads((output/'result.json').read_bytes())
    assert source.exists() and (output/m.ARCHIVE_NAME).exists()
    assert result['status']=='FAILED_RETAINED' and result['recoverable_full_backup_verified'] is False
    assert result['source_deleted'] is result['deletion_implemented'] is False
    assert 'error' in result


def test_unapproved_fake_archive_has_no_output_directory(tmp_path,monkeypatch):
    source,output,plan_sha,auth_sha=fixture_fake_archive(tmp_path,monkeypatch,'authority')
    with pytest.raises(ValueError,match='user-approved'):m.execute(plan_sha,auth_sha)
    assert source.exists() and not output.exists()
