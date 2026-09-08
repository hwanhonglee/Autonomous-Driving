"""HH_260906 - Validate create-only opcode publication using synthetic BLOBs, never a nuPlan database."""

import copy
from pathlib import Path
import pickle
from types import SimpleNamespace

import pytest

from scripts.e2e import curate_nuplan_calibration_opcodes as curate


@pytest.fixture
def evidence(tmp_path, monkeypatch):
    audit = curate.audit
    root = tmp_path/'raw'; root.mkdir()
    prior_path = tmp_path/'old/report.json'; prior_path.parent.mkdir()
    blob = b'cnumpy.core.multiarray\n_reconstruct\n)R.'
    fields = {f: dict(storage_type='blob', size_bytes=len(blob), sha256=audit.sha(blob), deserialized=False) for f in audit.FIELDS}
    prior = dict(status='METADATA_INSPECTED_NOT_READY', member_sha256=audit.MEMBER_SHA, archive_identity_unchanged=True,
        database_payloads_opened=1, archive_post_identity={'size_bytes':999, 'mtime_ns':1}, archive_previous_content_sha256='a'*64,
        selection_plan_sha256='b'*64, metadata={'calibration_opaque_fields':[dict(channel=c, fields=fields) for c in audit.CHANNELS]})
    prior_path.write_bytes(curate.encoded(prior)); monkeypatch.setattr(audit, 'PRIOR_SHA', audit.sha(prior_path.read_bytes()))
    plan = audit.build_plan(prior, declared_at_utc='2026-09-08T19:18:00Z', source_sha256=curate.SOURCE_SHA)
    (root/'followup_plan.json').write_bytes(curate.encoded(plan)); monkeypatch.setattr(curate, 'PLAN_SHA', audit.sha((root/'followup_plan.json').read_bytes()))
    (root/'source_executed.py').write_bytes(Path(audit.__file__).read_bytes())
    rows = []
    for channel in audit.CHANNELS:
        for field in audit.FIELDS:
            name = 'opaque_blobs/'+channel+'/'+field+'.bin'
            path = root/name; path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(blob)
            rows.append(dict(channel=channel, field=field, retained_private_blob=name, **audit.inspect_pickle_opcodes(blob)))
    report = dict(schema=audit.SCHEMA, status='OPCODE_INVENTORY_COMPLETE_NOT_DECODED', source_sha256=curate.SOURCE_SHA,
        followup_plan_sha256=curate.PLAN_SHA, previous_report_sha256=audit.PRIOR_SHA, original_selection_plan_sha256=prior['selection_plan_sha256'],
        member_sha256=audit.MEMBER_SHA, database_payloads_opened=1, opaque_calibration_blob_copies_written=32,
        camera_payloads_read=0, lidar_payloads_read=0, map_payloads_read=0, database_written_to_disk=False,
        numeric_calibration_extracted=False, pickle_vm_or_constructors_executed=False, primitive_interpreter_implemented=False,
        archive_whole_content_rehashed_this_run=False, database_decompressed_in_memory=True, member_crc_verified_to_eof=True,
        finish_deadline_met=True, missing_calibration_fields=[], postcheck_errors=[], archive_pre_identity=plan['archive_identity'],
        previous_archive_content_sha256=plan['previous_archive_content_sha256'], started_at_utc='2026-09-08T19:19:00Z',
        completed_at_utc='2026-09-08T19:19:01Z', parse_rejection_count=0, blob_reports=rows, **audit.DENIALS)
    record = SimpleNamespace(root=root, prior_path=prior_path, output=tmp_path/'public', plan=plan, report=report)
    reseal(record)
    return record


def reseal(e):
    (e.root/'report.json').write_bytes(curate.encoded(e.report))
    names = sorted(path.relative_to(e.root).as_posix() for path in e.root.rglob('*') if path.is_file() and path.name != 'SHA256SUMS')
    (e.root/'SHA256SUMS').write_text(''.join(curate.audit.sha((e.root/name).read_bytes())+'  '+name+'\n' for name in names))
    e.report_sha = curate.audit.sha((e.root/'report.json').read_bytes())
    e.checksums_sha = curate.audit.sha((e.root/'SHA256SUMS').read_bytes())


def publish(e):
    return curate.publish(e.root, e.prior_path, e.output, report_sha=e.report_sha, checksums_sha=e.checksums_sha)


def hashes(root):
    return {p.relative_to(root).as_posix():curate.audit.sha(p.read_bytes()) for p in root.rglob('*') if p.is_file()}


def test_complete_static_publication_excludes_all_32_binary_blobs_and_does_not_unpickle(evidence, monkeypatch):
    before = hashes(evidence.root)
    monkeypatch.setattr(pickle, 'loads', lambda *_:pytest.fail('no VM'))
    monkeypatch.setattr(pickle, 'load', lambda *_:pytest.fail('no VM'))
    result = publish(evidence)
    assert result['status']=='PUBLISHED_STATIC_INVENTORY_NOT_CALIBRATION'
    assert result['private_blob_count']==32 and result['public_blob_count']==0
    assert result['all_32_static_inventories_recomputed'] and not result['database_reopened_by_publisher']
    assert hashes(evidence.root)==before and not list(evidence.output.rglob('*.bin'))
    inventory=curate.manifest_entries((evidence.output/'SHA256SUMS').read_bytes())
    assert len(inventory)==7 and set(hashes(evidence.output))==set(inventory)|{'SHA256SUMS'}
    for name, digest in inventory.items(): assert curate.audit.sha((evidence.output/name).read_bytes())==digest
    assert (evidence.output/'execution_source.py').read_bytes()==(evidence.root/'source_executed.py').read_bytes()
    assert (evidence.output/'original_SHA256SUMS.txt').read_bytes()==(evidence.root/'SHA256SUMS').read_bytes()
    view=curate.audit.json_load((evidence.output/'report.json').read_bytes())
    assert view['record']==evidence.report and view['raw_source_sha256']==evidence.report_sha
    stats=curate.audit.json_load((evidence.output/'opcode_summary.json').read_bytes())
    assert stats['blob_count']==32 and stats['fields'][0]['dynamic_opcode_count']==16
    assert not stats['trusted_intrinsics_or_extrinsics']


def test_existing_output_rejected_without_modifying_any_file(evidence):
    publish(evidence); before=hashes(evidence.output)
    with pytest.raises(ValueError, match='fresh'): publish(evidence)
    assert hashes(evidence.output)==before


@pytest.mark.parametrize('field,value',[
    ('status','FAILED_METADATA_FOLLOWUP'),('camera_payloads_read',1),('database_payloads_opened',2),
    ('numeric_calibration_extracted',True),('training_data_approved',True),('pickle_vm_or_constructors_executed',True),
    ('postcheck_errors',[{'error':'changed'}]),('missing_calibration_fields',['CAM_F0/rotation']),
    ('parse_rejection_count',1),('parse_rejection_count',False),('source_sha256','0'*64),('followup_plan_sha256','0'*64),
    ('completed_at_utc','2026-09-09T02:00:00Z'),('started_at_utc','2026-09-08T19:17:00Z'),
    ('finish_deadline_met',False),('archive_whole_content_rehashed_this_run',True),('opaque_calibration_blob_copies_written',31),
])
def test_resealed_report_scope_counts_source_or_chronology_rejected_before_public_write(evidence,field,value):
    evidence.report[field]=value; reseal(evidence)
    with pytest.raises(ValueError):publish(evidence)
    assert not evidence.output.exists()


@pytest.mark.parametrize('kind',['duplicate','missing','opcode','semantics','blob','extra','source','plan','manifest'])
def test_resealed_source_inventory_or_blob_tampering_rejected(evidence,kind):
    if kind=='duplicate':evidence.report['blob_reports'][-1]=copy.deepcopy(evidence.report['blob_reports'][0])
    if kind=='missing':evidence.report['blob_reports'].pop()
    if kind=='opcode':evidence.report['blob_reports'][0]['opcodes'][0]['name']='NONE'
    if kind=='semantics':evidence.report['blob_reports'][0]['constructors_executed']=True
    if kind=='blob':(evidence.root/'opaque_blobs/CAM_B0/translation.bin').write_bytes(b'N.')
    if kind=='extra':(evidence.root/'unexpected.bin').write_bytes(b'N.')
    if kind=='source':(evidence.root/'source_executed.py').write_bytes(b'# changed')
    if kind=='plan':(evidence.root/'followup_plan.json').write_bytes(b'{}')
    reseal(evidence)
    if kind=='manifest':
        p=evidence.root/'SHA256SUMS';p.write_bytes(p.read_bytes()+b'0'*64+b'  ../outside\n'); evidence.checksums_sha=curate.audit.sha(p.read_bytes())
    with pytest.raises(ValueError):publish(evidence)
    assert not evidence.output.exists()


@pytest.mark.parametrize('kind',['blob','directory','output','input_root'])
def test_symlink_inputs_and_outputs_rejected(evidence,kind,tmp_path):
    if kind=='blob':
        p=evidence.root/'opaque_blobs/CAM_B0/translation.bin';raw=p.read_bytes();p.unlink();outside=tmp_path/'outside';outside.write_bytes(raw);p.symlink_to(outside)
    if kind=='directory':(evidence.root/'link').symlink_to(tmp_path,target_is_directory=True)
    if kind=='output':evidence.output.symlink_to(tmp_path/'absent',target_is_directory=True)
    if kind=='input_root':
        link=tmp_path/'linked_input';link.symlink_to(evidence.root,target_is_directory=True);evidence.root=link
    with pytest.raises(ValueError):publish(evidence)


@pytest.mark.parametrize('where',['input_child','prior_child','prior_ancestor','dataset'])
def test_separate_output_and_no_dataset_boundary(evidence,where,monkeypatch,tmp_path):
    if where=='input_child':evidence.output=evidence.root/'new'
    if where=='prior_child':evidence.output=evidence.prior_path.parent/'new'
    if where=='prior_ancestor':evidence.output=evidence.prior_path.parent.parent
    if where=='dataset':
        repo=tmp_path/'repository';data=tmp_path/'actual_datasets';repo.mkdir();data.mkdir();(repo/'datasets').symlink_to(data,target_is_directory=True)
        monkeypatch.setattr(curate,'REPO',repo);evidence.output=data/'new'
    with pytest.raises(ValueError):publish(evidence)


def test_mutation_after_writing_keeps_partial_output_without_success_manifest(evidence,monkeypatch):
    original=curate.audit.write_new
    def changing(path,raw):
        original(path,raw)
        if path.name=='README.md':(evidence.root/'opaque_blobs/CAM_B0/translation.bin').write_bytes(b'N.')
    monkeypatch.setattr(curate.audit,'write_new',changing)
    with pytest.raises(ValueError):publish(evidence)
    assert evidence.output.exists() and (evidence.output/'README.md').exists()
    assert not (evidence.output/'publication_manifest.json').exists() and not (evidence.output/'SHA256SUMS').exists()


def test_account_paths_are_redacted_only_in_public_derivative(evidence):
    evidence.report['diagnostic_note']='/home/testaccount/personal/owner/portable_e2e/runs/private'
    reseal(evidence);before=(evidence.root/'report.json').read_bytes();publish(evidence)
    public=(evidence.output/'report.json').read_text()
    assert '/home/' not in public and '${PORTABLE_E2E_ROOT}/runs/private' in public
    assert (evidence.root/'report.json').read_bytes()==before


@pytest.mark.parametrize('text',['/home/private/path','/root/file','220.90.5.14','GPU-12345678-abcd-abcd-abcd-123456789abc'])
def test_privacy_checker_rejects_machine_details(text):
    with pytest.raises(ValueError):curate.privacy(text.encode())


def test_cli_disables_argument_abbreviations():
    with pytest.raises(SystemExit) as error:curate.main(['--input-r','anything'])
    assert error.value.code==2
