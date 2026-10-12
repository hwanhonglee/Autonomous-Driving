"""HH_261007 - Prepare one exact ROS-log backup with full streamed roundtrip verification.

Import and default plan inspection do not hash the large log, compress, install,
launch ROS/CARLA, or remove anything. Actual backup needs a freshly sealed ROOT
plan and explicit-user-approved backup authority. There is NO deletion code.
Failed or incomplete outputs remain in the fresh private attempt directory.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import resource
import selectors
import signal
import stat
import subprocess
import time


ROOT = Path(__file__).resolve().parents[2]
SOURCE = Path('/home/a/.ros/log/component_container_mt_4904_1744181413078.log')
OUTPUT = Path('/home/a/thesis_4pc/90_move_records_and_cleanup/ros_log_archive_20261007_v1')
PLAN = ROOT / 'storage/experiments/plans/2026-10-07/ros_log_archive_v1/ROOT_plan.json'
AUTHORITY = PLAN.with_name('ROOT_archive_authority.json')
LOCK = ROOT / 'storage/assets/locks/autoware_e2e_runtime.lock'
HERE = Path(__file__).resolve()
TEST = ROOT / 'learning/tests/test_archive_old_ros_log_20261007_v1.py'
ZSTD = Path('/usr/bin/zstd')
ZSTD_SHA256 = 'd304445daa7e6429293dc02035063b7993fb6a489ee90d8851bff497952836dc'
SCHEMA = 'portable_e2e.exact_one_old_ROS_log_archive.20261007.v1'
UID = 1000
EXPECTED_GENERATION = dict(device=66309, inode=35823911, size=6502342656,
    blocks=12699904, uid=1000, gid=1000, permissions=0o664, nlink=1,
    mtime_ns=1744220271850359659, ctime_ns=1744220271850359659)
OUTPUT_CAP = 512 * 1024**2
CLOSING_RESERVE = 64 * 1024**2
FREE_FLOOR = 128 * 1024**2
ARCHIVE_CAP = OUTPUT_CAP - 4 * 1024**2
JSON_CAP = 1024**2
ERROR_CAP = 1024**2
CHUNK = 1024**2
WALL_LIMIT_S = 300
ARCHIVE_NAME = SOURCE.name + '.zst'


def require(value, message):
    if not value:
        raise ValueError(message)


def encode(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def canonical(path):
    path = Path(path)
    require(path.is_absolute() and '..' not in path.parts and path == path.resolve(strict=True)
        and not any(p.is_symlink() for p in (path, *path.parents)), 'canonical existing non-symlink path required')
    return path


def generation(entry):
    return dict(device=entry.st_dev, inode=entry.st_ino, size=entry.st_size,
        blocks=entry.st_blocks, uid=entry.st_uid, gid=entry.st_gid,
        permissions=stat.S_IMODE(entry.st_mode), nlink=entry.st_nlink,
        mtime_ns=entry.st_mtime_ns, ctime_ns=entry.st_ctime_ns)


def opened_source(path, expected, uid=UID):
    # HH_261007 - O_NOFOLLOW and exact fd/path identity prevent substitution before any large-byte read.
    path = canonical(path)
    initial = path.lstat()
    require(stat.S_ISREG(initial.st_mode) and initial.st_uid == uid and initial.st_nlink == 1,
        'owned single-link regular source required')
    require(generation(initial) == expected, 'source generation differs from exact prospective plan')
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        verify_source(path, fd, expected)
    except BaseException:
        os.close(fd)
        raise
    return fd


def verify_source(path, fd, expected):
    canonical(path)
    require(generation(os.fstat(fd)) == expected and generation(Path(path).lstat()) == expected,
        'source fd/path generation changed; keep original and all failed backup files')


def hash_fd(fd, deadline, *, byte_limit=None, check=lambda: None):
    # HH_261007 - pread avoids altering the shared file offset used by the compression child.
    digest = hashlib.sha256(); count = 0
    while True:
        require(time.monotonic() < deadline, 'whole-stream hashing deadline reached')
        check()
        raw = os.pread(fd, CHUNK, count)
        if not raw:
            return dict(sha256=digest.hexdigest(), bytes=count)
        count += len(raw)
        require(byte_limit is None or count <= byte_limit, 'stream exceeds declared complete source byte count')
        digest.update(raw)


def small_sha(path):
    path = canonical(path); initial = path.lstat()
    require(stat.S_ISREG(initial.st_mode) and 0 < initial.st_size <= JSON_CAP * 8,
        'bounded regular code/binary/proof required')
    raw = path.read_bytes()
    require(generation(path.lstat()) == generation(initial), 'small sealed input changed while reading')
    return sha(raw)


def prospective_plan():
    # HH_261007 - Read metadata and small source/binary bytes only; never hash the 6.5GB source in plan mode.
    require(os.getuid() == UID and canonical(SOURCE) == SOURCE, 'fixed local owner/source required')
    entry = SOURCE.lstat()
    require(stat.S_ISREG(entry.st_mode) and generation(entry) == EXPECTED_GENERATION,
        'exact old log metadata changed; a new reviewed revision is required')
    parent = canonical(OUTPUT.parent)
    require(parent.stat().st_uid == UID and not OUTPUT.exists() and not OUTPUT.is_symlink(),
        'fresh owned home archive output required; never reuse an attempt')
    require(small_sha(ZSTD) == ZSTD_SHA256, 'reviewed existing zstd binary changed; do not install a replacement')
    return dict(schema=SCHEMA+'.ROOT_plan', source=str(SOURCE), source_generation=EXPECTED_GENERATION,
        output=str(OUTPUT), archive_name=ARCHIVE_NAME, executable=str(ZSTD), executable_sha256=ZSTD_SHA256,
        source_sha256={str(HERE.relative_to(ROOT)):small_sha(HERE), str(TEST.relative_to(ROOT)):small_sha(TEST)},
        runtime_lock=str(LOCK), output_cap_bytes=OUTPUT_CAP, archive_cap_bytes=ARCHIVE_CAP,
        closing_reserve_bytes=CLOSING_RESERVE, free_floor_bytes=FREE_FLOOR,
        wall_limit_seconds=WALL_LIMIT_S, compressed_threads=1, compression_level=3,
        full_source_sha_before_and_after=True, whole_stream_roundtrip_sha_and_count_required=True,
        source_deleted=False, deletion_implemented=False, training_GPU_ROS_CARLA_authorized=False)


def read_pinned_json(path, expected_sha):
    require(type(expected_sha) is str and len(expected_sha) == 64
        and all(c in '0123456789abcdef' for c in expected_sha), 'external raw JSON SHA256 required')
    path = canonical(path); initial = path.lstat()
    require(stat.S_ISREG(initial.st_mode) and initial.st_uid == UID and initial.st_nlink == 1
        and 0 < initial.st_size <= JSON_CAP, 'bounded owned single-link ROOT JSON required')
    raw = path.read_bytes()
    require(sha(raw) == expected_sha and generation(path.lstat()) == generation(initial), 'ROOT JSON bytes/generation changed')
    def unique(pairs):
        value = {}
        for key, child in pairs:
            require(key not in value, 'duplicate ROOT JSON key denied'); value[key] = child
        return value
    value = json.loads(raw, object_pairs_hook=unique,
        parse_constant=lambda value: require(False, 'nonfinite ROOT JSON denied'))
    require(encode(value) == encode(value), 'finite JSON required')
    return value, raw


def verify_authority(plan, authority, plan_sha):
    require(encode(plan) == encode(prospective_plan()), 'exact fresh plan, source generation and helper/tests required')
    expected = dict(schema=SCHEMA+'.ROOT_archive_authority', authorized=True,
        mode='backup_only', ROOT_source_execution_review_completed=True,
        explicit_user_approved_one_log_full_backup=True, source=str(SOURCE), output=str(OUTPUT),
        plan_sha256=plan_sha, source_sha256=plan['source_sha256'],
        no_original_or_other_file_deletion=True, keep_failed_partial_outputs=True,
        no_installation_training_GPU_ROS_CARLA_or_remote_write=True)
    require(encode(authority) == encode(expected), 'exact sealed user-approved backup-only ROOT authority required')


@contextmanager
def runtime_lease():
    with canonical(LOCK).open('rb') as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def budget(output=None):
    current = os.statvfs(OUTPUT.parent)
    free = current.f_bavail * current.f_frsize
    require(free >= FREE_FLOOR + CLOSING_RESERVE + (OUTPUT_CAP if output is None else 0),
        'backup-only fixed floor and reserves unavailable; no existing GPU floor is changed')
    if output is not None:
        require(canonical(output) == output and output.stat().st_uid == UID, 'exact owned archive output required')
        total = 0
        for item in output.iterdir():
            entry = item.lstat()
            require(stat.S_ISREG(entry.st_mode) and entry.st_uid == UID and entry.st_nlink == 1,
                'unknown archive entry/type/owner/link denied')
            total += entry.st_size
        require(total <= OUTPUT_CAP, 'whole output cap reached; preserve the failed partial attempt')
    return free


def new_bytes(path, raw):
    # HH_261007 - All attempt metadata, including original ROOT bytes, use create-only durable streams.
    require(type(raw) is bytes, 'exact original metadata bytes required')
    require(len(raw) <= JSON_CAP, 'bounded closing receipt required')
    with path.open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())


def new_json(path, value):
    new_bytes(path,encode(value))


def process_generation(pid):
    try:
        fields = Path('/proc/%d/stat'%pid).read_text().rsplit(')',1)[1].split()
        return dict(pid=pid, ppid=int(fields[1]), pgid=int(fields[2]), sid=int(fields[3]), start_ticks=int(fields[19]))
    except (FileNotFoundError, ProcessLookupError):
        return None


def stop_owned(process, identity):
    # HH_261007 - Only our exact still-unreaped child session may be signalled; never stop another workload.
    if process.poll() is not None:
        return
    require(identity is not None and process_generation(process.pid) == identity
        and identity['ppid'] == os.getpid() and identity['pgid'] == identity['sid'] == process.pid,
        'child identity changed; unowned process is not signalled')
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        require(process_generation(process.pid) == identity, 'child generation changed before owned kill')
        os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=2)


def child_limits():
    resource.setrlimit(resource.RLIMIT_FSIZE, (ARCHIVE_CAP, ARCHIVE_CAP))
    resource.setrlimit(resource.RLIMIT_CORE, (0,0))
    resource.setrlimit(resource.RLIMIT_CPU, (240,245))


def run_zstd(source_fd, output, deadline, *, decompress=False, expected_size=None):
    # HH_261007 - Stream complete bytes once with bounded stderr, output and child lifetime; never restore a second 6.5GB copy.
    archive = output / ARCHIVE_NAME
    require(small_sha(ZSTD)==ZSTD_SHA256, 'existing sealed zstd changed before its exact owned child')
    stderr_name = 'decompression.stderr' if decompress else 'compression.stderr'
    archive_stream = None
    if decompress:
        argv = [str(ZSTD), '-q', '-d', '-c']
        stdout = subprocess.PIPE
    else:
        argv = [str(ZSTD), '-q', '-T1', '-3', '-c']
        archive_stream = archive.open('xb'); stdout = archive_stream
    error_stream = (output / stderr_name).open('xb')
    process = None; identity = None; digest = hashlib.sha256(); size = errors = 0
    try:
        os.lseek(source_fd, 0, os.SEEK_SET)
        process = subprocess.Popen(argv, stdin=source_fd, stdout=stdout, stderr=subprocess.PIPE,
            env={'PATH':'/usr/bin:/bin','LC_ALL':'C'}, start_new_session=True,
            close_fds=True, preexec_fn=child_limits)
        identity = process_generation(process.pid)
        require(identity and identity['ppid']==os.getpid() and identity['pgid']==identity['sid']==process.pid,
            'exact owned zstd session required')
        with selectors.DefaultSelector() as selector:
            selector.register(process.stderr, selectors.EVENT_READ, 'error')
            if decompress:
                selector.register(process.stdout, selectors.EVENT_READ, 'data')
            while selector.get_map():
                require(time.monotonic() < deadline, 'whole archive wall deadline reached')
                budget(output)
                for key, _ in selector.select(timeout=.05):
                    raw = os.read(key.fileobj.fileno(), CHUNK)
                    if not raw:
                        selector.unregister(key.fileobj); continue
                    if key.data == 'error':
                        errors += len(raw); error_stream.write(raw); error_stream.flush()
                        require(errors <= ERROR_CAP, 'stderr cap reached; full recorded prefix retained')
                    else:
                        size += len(raw)
                        require(type(expected_size) is int and size <= expected_size, 'decompressed bytes exceed original whole source')
                        digest.update(raw)
        code = process.wait(timeout=max(.01,deadline-time.monotonic()))
        require(code == 0, 'zstd failed, returncode=%d; keep original and failed outputs'%code)
        if archive_stream:
            archive_stream.flush(); os.fsync(archive_stream.fileno())
        return dict(argv=argv, process_identity=identity, returncode=code,
            stderr_bytes=errors, decompressed_sha256=digest.hexdigest() if decompress else None,
            decompressed_bytes=size if decompress else None)
    finally:
        if process is not None:
            stop_owned(process, identity)
            for stream in (process.stdout,process.stderr):
                if hasattr(stream,'close'): stream.close()
        if archive_stream is not None: archive_stream.close()
        error_stream.flush(); os.fsync(error_stream.fileno()); error_stream.close()


def source_fd_census(expected):
    # HH_261007 - Permission/generation gaps remain UNKNOWN and cannot later authorize removal.
    hits, unknown = [], []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit() or int(path.name)==os.getpid(): continue
        try:
            before = process_generation(int(path.name))
            if before is None: continue
            for entry in (path/'fd').iterdir():
                try:
                    value = entry.stat()
                    if (value.st_dev,value.st_ino)==(expected['device'],expected['inode']):
                        hits.append(dict(pid=int(path.name),fd=int(entry.name),generation=before))
                except (FileNotFoundError,ProcessLookupError): continue
                except PermissionError: unknown.append(dict(pid=int(path.name),reason='FD_STAT_PERMISSION'))
            if process_generation(int(path.name)) != before:
                unknown.append(dict(pid=int(path.name),reason='PROCESS_GENERATION_CHANGED'))
        except (FileNotFoundError,ProcessLookupError): continue
        except PermissionError: unknown.append(dict(pid=int(path.name),reason='FD_DIRECTORY_PERMISSION'))
    return dict(hits=hits, unknown=unknown, complete_nonuse_proven=not hits and not unknown,
        removal_authority=False)


def execute(plan_sha, authority_sha):
    require(os.getuid() == UID, 'only fixed local owner may archive this one log')
    plan, plan_raw = read_pinned_json(PLAN, plan_sha)
    authority, authority_raw = read_pinned_json(AUTHORITY, authority_sha)
    verify_authority(plan,authority,plan_sha)
    with runtime_lease():
        verify_authority(plan,authority,plan_sha); budget()
        fd = opened_source(SOURCE,EXPECTED_GENERATION)
        deadline = time.monotonic()+WALL_LIMIT_S
        result = dict(schema=SCHEMA+'.result', status='FAILED_RETAINED', source=str(SOURCE),
            source_generation=EXPECTED_GENERATION, output=str(OUTPUT), source_deleted=False,
            deletion_implemented=False, recoverable_full_backup_verified=False,
            ROOT_plan_sha256=plan_sha, ROOT_authority_sha256=authority_sha,
            source_atime_not_restored_because_no_source_metadata_mutation=True)
        made_output = False
        try:
            OUTPUT.mkdir(mode=0o700); made_output=True
            new_bytes(OUTPUT/'ROOT_plan.json',plan_raw)
            new_bytes(OUTPUT/'ROOT_archive_authority.json',authority_raw)
            new_json(OUTPUT/'original_metadata.json',dict(path=str(SOURCE),generation=EXPECTED_GENERATION,
                observed_atime_ns=os.fstat(fd).st_atime_ns, restoration_requires_a_separate_reviewed_action=True))
            result['source_fd_audit_before'] = source_fd_census(EXPECTED_GENERATION)
            result['whole_source_before'] = hash_fd(fd,deadline,byte_limit=EXPECTED_GENERATION['size'],check=lambda:budget(OUTPUT))
            require(result['whole_source_before']['bytes']==EXPECTED_GENERATION['size'], 'entire original source was not hashed')
            verify_source(SOURCE,fd,EXPECTED_GENERATION)
            result['compression'] = run_zstd(fd,OUTPUT,deadline)
            verify_source(SOURCE,fd,EXPECTED_GENERATION)
            archive_fd = os.open(OUTPUT/ARCHIVE_NAME,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
            try:
                archived_generation = generation(os.fstat(archive_fd))
                require(0 < archived_generation['size'] <= ARCHIVE_CAP
                    and archived_generation['uid']==UID and archived_generation['nlink']==1,
                    'bounded owned single-link complete archive required')
                result['archive'] = dict(path=str(OUTPUT/ARCHIVE_NAME),generation=archived_generation,
                    **hash_fd(archive_fd,deadline,byte_limit=ARCHIVE_CAP,check=lambda:budget(OUTPUT)))
                result['roundtrip'] = run_zstd(archive_fd,OUTPUT,deadline,decompress=True,expected_size=EXPECTED_GENERATION['size'])
                require(generation(os.fstat(archive_fd))==archived_generation
                    and generation((OUTPUT/ARCHIVE_NAME).lstat())==archived_generation,'archive fd/path changed during whole roundtrip')
                require(result['roundtrip']['decompressed_bytes']==result['whole_source_before']['bytes']
                    and result['roundtrip']['decompressed_sha256']==result['whole_source_before']['sha256'],
                    'whole decompressed count/SHA256 differs from complete original source')
            finally:
                os.close(archive_fd)
            result['whole_source_after'] = hash_fd(fd,deadline,byte_limit=EXPECTED_GENERATION['size'],check=lambda:budget(OUTPUT))
            verify_source(SOURCE,fd,EXPECTED_GENERATION)
            require(result['whole_source_before']==result['whole_source_after'],'original bytes changed while backing up')
            result['source_fd_audit_after'] = source_fd_census(EXPECTED_GENERATION)
            result.update(status='VERIFIED_BACKUP_ORIGINAL_RETAINED',recoverable_full_backup_verified=True)
            return result
        except BaseException as error:
            result['error'] = repr(error)
            raise
        finally:
            os.close(fd)
            if made_output:
                new_json(OUTPUT/'result.json',result)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--mode',choices=('plan','archive'),default='plan')
    parser.add_argument('--plan-sha256');parser.add_argument('--authority-sha256')
    args=parser.parse_args(argv)
    if args.mode=='plan':
        require(args.plan_sha256 is None and args.authority_sha256 is None,'plan metadata mode takes no execution grant')
        print(encode(prospective_plan()).decode());return 0
    result=execute(args.plan_sha256,args.authority_sha256)
    print(encode(result).decode());return 0


if __name__=='__main__':
    raise SystemExit(main())
