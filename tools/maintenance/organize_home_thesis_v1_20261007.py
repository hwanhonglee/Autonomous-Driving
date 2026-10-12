"""HH_261007 - Move only declared document/download groups with hashes, no overwrite, and a reversible journal."""
from __future__ import annotations

import argparse
import ctypes
import errno
import hashlib
import json
import os
from pathlib import Path
import stat
import time

HOME_ROOT = Path('/home/a')
DESTINATION = HOME_ROOT / 'thesis_4pc'
RECORDS = DESTINATION / '90_move_records_and_cleanup/20261007_documents_downloads_v1'


def declared_moves():
    # HH_261007 - Leave XDG roots, AnyDesk and the path-dependent Unreal project in place.
    moves = []
    def add(root, name, category):
        moves.append({'source': str(HOME_ROOT / root / name),
                      'destination': str(DESTINATION / category / name)})
    add('Downloads', 'Autoware_Performance_Analysis_rideflux_20260928.pdf', '01_papers_and_reports')
    for name in (
        'CODEX_PC4_FIRST_MESSAGE_V2_ALIAS_TRACE_KO.txt',
        'CODEX_PC4_FOLLOWUP_AFTER_ARCHIVE_AUDIT_KO.md',
        'CODEX_PC4_MASTER_PROMPT_V2_ALIAS_TRACE_KO.md',
        'CODEX_PC4_PLANNING_SIMULATOR_FIRST_MESSAGE_KO.txt',
        'CODEX_PC4_PLANNING_SIMULATOR_FOLLOWUP_KO.md',
        'CODEX_VILS_SURROGATE_TO_REAL_DEPLOYMENT_KO.md',
        'CODEX_VILS_SURROGATE_TO_REAL_FIRST_MESSAGE_KO.txt',
    ):
        add('Downloads', name, '02_pc_test_documents/pc4_requests')
    for name in ('VILS_TEST_ARCHIVE_AUDIT.md', 'VILS_BAG_TOPIC_INVENTORY.csv',
                 'VILS_OBJECT_REPLAY_SEQUENCE_AUDIT.csv'):
        add('Downloads', name, '02_pc_test_documents/archived_data_audit')
    for name in ('VILS_Test_Virtual_Avoid.zip', 'VILS_Test_Virtual_GFP.zip', 'VILS_Test_Virtual_TLR.zip'):
        add('Downloads', name, '03_raw_test_data/vils_2025_archives')
    add('Downloads', 'chrony_CHRONY_R01_a_20260815T091533Z_PC4', '03_raw_test_data/clock_sync')
    for name in ('C_track_1_0_7.fbx', 'C_track_1_0_7.fbm', 'C_track_1_0_7.xodr'):
        add('Downloads', name, '05_maps_and_simulator_assets/c_track_source/downloaded')
    for name in ('C_track_1_0_7.umap', 'C_track_1_0_7.xodr'):
        add('Documents', name, '05_maps_and_simulator_assets/c_track_source/document_copies')
    add('Downloads', 'Driving Set', '05_maps_and_simulator_assets/lanelet_and_pointcloud')
    add('Downloads', 'pointcloud_map (CARLA_C_track_v1.0.7_ref_C_track_v1.0.0).zip',
        '05_maps_and_simulator_assets/map_archives')
    for name in ('carla-0.9.15-cp310-cp310-linux_x86_64.whl', 'carla-0.9.15-py3.10-linux-x86_64.egg'):
        add('Documents', name, '06_installers_and_tools/carla_python_0_9_15')
    for name in ('collect_chrony_4pc.sh', 'txt.txt'):
        add('Downloads', name, '06_installers_and_tools/clock_measurement')
    return moves


def metadata(value):
    return dict(device=value.st_dev, inode=value.st_ino, mode=value.st_mode,
                uid=value.st_uid, gid=value.st_gid, nlink=value.st_nlink,
                bytes=value.st_size, mtime_ns=value.st_mtime_ns, ctime_ns=value.st_ctime_ns)


def snapshot(path):
    # HH_261007 - Reject symbolic/special/shared objects and detect a source changed during hashing.
    path = Path(path)
    if path.resolve() != path:
        raise RuntimeError(f'symbolic or noncanonical source: {path}')
    rows = []
    def visit(current):
        first = current.lstat()
        if first.st_uid != os.getuid():
            raise RuntimeError(f'foreign ownership: {current}')
        row = dict(relative=str(current.relative_to(path)), **metadata(first))
        if stat.S_ISDIR(first.st_mode):
            row['kind'] = 'directory'
            rows.append(row)
            for child in sorted(current.iterdir()):
                visit(child)
        elif stat.S_ISREG(first.st_mode) and first.st_nlink == 1:
            row['kind'] = 'file'
            descriptor = os.open(current, os.O_RDONLY | os.O_NOFOLLOW)
            try:
                if metadata(os.fstat(descriptor)) != metadata(first):
                    raise RuntimeError(f'changed before read: {current}')
                digest = hashlib.sha256()
                while block := os.read(descriptor, 1024 * 1024):
                    digest.update(block)
                if metadata(os.fstat(descriptor)) != metadata(first):
                    raise RuntimeError(f'changed during read: {current}')
                row['sha256'] = digest.hexdigest()
            finally:
                os.close(descriptor)
            rows.append(row)
        else:
            raise RuntimeError(f'link, special object or shared file: {current}')
        if metadata(current.lstat()) != metadata(first):
            raise RuntimeError(f'changed after read: {current}')
    visit(path)
    return rows


def equivalent_after_rename(before, after):
    # HH_261007 - Rename changes only the moved root's ctime; retain every content/identity field.
    def normalize(rows):
        return [{key: value for key, value in row.items()
                 if not (row['relative'] == '.' and key == 'ctime_ns')} for row in rows]
    return normalize(before) == normalize(after)


def rename_no_replace(source, destination):
    # HH_261007 - Linux atomic NOREPLACE prevents a concurrent file from being overwritten.
    library = ctypes.CDLL(None, use_errno=True)
    operation = library.renameat2
    operation.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    operation.restype = ctypes.c_int
    result = operation(-100, os.fsencode(source), -100, os.fsencode(destination), 1)
    if result:
        code = ctypes.get_errno()
        if code == errno.EEXIST:
            raise FileExistsError(str(destination))
        raise OSError(code, os.strerror(code), str(source))


def validate_layout(moves):
    sources = [Path(row['source']) for row in moves]
    destinations = [Path(row['destination']) for row in moves]
    if len(set(sources)) != len(sources) or len(set(destinations)) != len(destinations):
        raise RuntimeError('duplicate move')
    for source, destination in zip(sources, destinations):
        if source.parent not in (HOME_ROOT / 'Documents', HOME_ROOT / 'Downloads'):
            raise RuntimeError('source outside declared document/download roots')
        if not destination.is_relative_to(DESTINATION) or destination == DESTINATION:
            raise RuntimeError('destination outside categorized folder')
        if source.resolve() != source or destination.resolve() != destination:
            raise RuntimeError('symlink or noncanonical parent')
        if source.name != destination.name:
            raise RuntimeError('original object name must be preserved')
        if destination.exists() or destination.is_symlink():
            raise FileExistsError(str(destination))
        if source.lstat().st_dev != HOME_ROOT.stat().st_dev:
            raise RuntimeError('cross-filesystem move forbidden')
        if not (stat.S_ISREG(source.lstat().st_mode) or stat.S_ISDIR(source.lstat().st_mode)):
            raise RuntimeError('source must be a regular file or directory')
    if DESTINATION.exists() and DESTINATION.stat().st_dev != HOME_ROOT.stat().st_dev:
        raise RuntimeError('destination is another filesystem')


def active_path_check(moves):
    # HH_261007 - Record unreadable processes honestly; never terminate or attach to them.
    roots = [row['source'] for row in moves]
    hits, unreadable = [], []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            owner = proc.stat().st_uid
            links = [proc / 'cwd', proc / 'exe'] + list((proc / 'fd').iterdir())
            for link in links:
                try:
                    target = os.readlink(link).removesuffix(' (deleted)')
                except FileNotFoundError:
                    continue
                if any(target == root or target.startswith(root + '/') for root in roots):
                    hits.append(dict(pid=int(proc.name), uid=owner, reference=str(link), path=target))
        except FileNotFoundError:
            continue
        except PermissionError:
            unreadable.append(int(proc.name))
    result = dict(matching_references=hits, unreadable_pids=unreadable,
                  unreadable_processes_are_not_nonuse_proof=True)
    if hits:
        raise RuntimeError(f'declared source has an active path reference: {hits}')
    return result


def write_json_new(path, value):
    payload = (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()
    with Path(path).open('xb') as output:
        output.write(payload)
        output.flush()
        os.fsync(output.fileno())
    return hashlib.sha256(payload).hexdigest()


def read_pinned_plan(expected):
    payload = (RECORDS / 'plan.json').read_bytes()
    if hashlib.sha256(payload).hexdigest() != expected:
        raise RuntimeError('plan SHA mismatch')
    plan = json.loads(payload)
    if [{key: row[key] for key in ('source', 'destination')} for row in plan['moves']] != declared_moves():
        raise RuntimeError('plan differs from the exact declared move set')
    if plan['source_sha256'] != hashlib.sha256(Path(__file__).read_bytes()).hexdigest():
        raise RuntimeError('organizer source changed')
    return plan


def inspect():
    moves = declared_moves()
    validate_layout(moves)
    usage = active_path_check(moves)
    if RECORDS.exists():
        raise FileExistsError('fresh inspection output required')
    capacity = os.statvfs(HOME_ROOT)
    if capacity.f_bavail * capacity.f_frsize < 64 * 1024**2:
        raise RuntimeError('bounded metadata reserve unavailable')
    for row in moves:
        row['before'] = snapshot(row['source'])
    protected_documents = {str(path): snapshot(path) for path in (
        HOME_ROOT / 'Documents/AnyDesk', HOME_ROOT / 'Documents/Unreal Projects') if path.exists()}
    usage_after = active_path_check(moves)
    DESTINATION.mkdir(mode=0o700, exist_ok=True)
    RECORDS.mkdir(parents=True, mode=0o700)
    plan = dict(provenance='HH_261007 - Authorized reversible home organization; no deletion or runtime execution.',
                observed_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), moves=moves,
                protected_document_snapshots=protected_documents,
                process_observation_before=usage, process_observation_after=usage_after,
                protected=['/home/a/Documents/AnyDesk', '/home/a/Documents/Unreal Projects',
                           '/home/a/autoware_e2e', '/home/a/VILS_Test',
                           '/home/a/carla-autoware-universe', '/home/a/UnrealEngine_4.26',
                           '/home/a/carla', '/home/a/Autonomous-Driving-vils-docs'],
                permanently_deleted=0, original_content_modified=0, runtime_jobs_started=0)
    digest = write_json_new(RECORDS / 'plan.json', plan)
    return dict(plan_sha256=digest, top_level_objects=len(moves),
                files=sum(row['kind'] == 'file' for move in moves for row in move['before']))


def execute(expected):
    plan = read_pinned_plan(expected)
    validate_layout(plan['moves'])
    usage = active_path_check(plan['moves'])
    for row in plan['moves']:
        if snapshot(row['source']) != row['before']:
            raise RuntimeError(f'source changed since inspection: {row["source"]}')
    # HH_261007 - The pre-existing plan and fsynced per-object journal survive a partial interruption.
    with (RECORDS / 'move_journal.jsonl').open('x') as journal:
        for row in plan['moves']:
            source, destination = Path(row['source']), Path(row['destination'])
            destination.parent.mkdir(parents=True, exist_ok=True)
            if destination.resolve() != destination or destination.parent.stat().st_dev != source.stat().st_dev:
                raise RuntimeError('destination parent changed')
            if snapshot(source) != row['before']:
                raise RuntimeError('source changed immediately before move')
            active_path_check([row])
            journal.write(json.dumps(dict(phase='intent', source=str(source), destination=str(destination))) + '\n')
            journal.flush(); os.fsync(journal.fileno())
            rename_no_replace(source, destination)
            after = snapshot(destination)
            if not equivalent_after_rename(row['before'], after):
                raise RuntimeError(f'post-move preservation mismatch: {destination}')
            journal.write(json.dumps(dict(phase='verified', source=str(source), destination=str(destination))) + '\n')
            journal.flush(); os.fsync(journal.fileno())
    receipt = dict(plan_sha256=expected, completed_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                   moved_top_level_objects=len(plan['moves']),
                   preserved_files=sum(row['kind'] == 'file' for move in plan['moves'] for row in move['before']),
                   logical_bytes=sum(row['bytes'] for move in plan['moves'] for row in move['before'] if row['kind'] == 'file'),
                   process_observation=usage, content_changes=0, deletion=0, copied_large_files=0,
                   legacy_symlinks_created=0, runtime_jobs_started=0)
    write_json_new(RECORDS / 'completed.json', receipt)
    return receipt


def verify(expected):
    plan = read_pinned_plan(expected)
    for row in plan['moves']:
        if os.path.lexists(row['source']):
            raise RuntimeError(f'old source reappeared: {row["source"]}')
        if not equivalent_after_rename(row['before'], snapshot(row['destination'])):
            raise RuntimeError(f'current destination differs: {row["destination"]}')
    for path, before in plan['protected_document_snapshots'].items():
        if snapshot(path) != before:
            raise RuntimeError(f'protected project changed: {path}')
    return dict(verified=True, verified_top_level_objects=len(plan['moves']),
                original_bytes_modes_mtime_ownership_inodes_preserved=True,
                protected_document_roots_unchanged=len(plan['protected_document_snapshots']))


def rollback(expected):
    # HH_261007 - Undo only intact, fully verified moves; refuse new originals or user-edited destinations.
    plan = read_pinned_plan(expected)
    completed = json.loads((RECORDS / 'completed.json').read_text())
    if completed['plan_sha256'] != expected:
        raise RuntimeError('completion receipt differs from the pinned plan')
    verify(expected)
    reverse_moves = [dict(source=row['destination'], destination=row['source']) for row in plan['moves']]
    active_path_check(reverse_moves)
    for row in reverse_moves:
        source, destination = Path(row['source']), Path(row['destination'])
        if destination.parent.resolve() != destination.parent or destination.parent.stat().st_dev != source.stat().st_dev:
            raise RuntimeError('old parent is symbolic or another filesystem')
    with (RECORDS / 'rollback_journal.jsonl').open('x') as journal:
        for row in reversed(plan['moves']):
            source, destination = Path(row['destination']), Path(row['source'])
            if not equivalent_after_rename(row['before'], snapshot(source)):
                raise RuntimeError('destination changed before rollback')
            active_path_check([dict(source=str(source))])
            journal.write(json.dumps(dict(phase='intent', source=str(source), destination=str(destination))) + '\n')
            journal.flush(); os.fsync(journal.fileno())
            rename_no_replace(source, destination)
            if not equivalent_after_rename(row['before'], snapshot(destination)):
                raise RuntimeError('rollback verification differs')
            journal.write(json.dumps(dict(phase='verified', source=str(source), destination=str(destination))) + '\n')
            journal.flush(); os.fsync(journal.fileno())
    receipt = dict(plan_sha256=expected, restored_top_level_objects=len(plan['moves']), deletion=0,
                   completed_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
    write_json_new(RECORDS / 'rollback_completed.json', receipt)
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--inspect', action='store_true')
    group.add_argument('--execute-plan-sha256')
    group.add_argument('--verify-plan-sha256')
    group.add_argument('--rollback-plan-sha256')
    args = parser.parse_args(argv)
    result = inspect() if args.inspect else (execute(args.execute_plan_sha256)
             if args.execute_plan_sha256 else (verify(args.verify_plan_sha256)
             if args.verify_plan_sha256 else rollback(args.rollback_plan_sha256)))
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
