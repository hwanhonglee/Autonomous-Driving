#!/usr/bin/env python3
"""HH_260906 - Inventory untrusted calibration pickle opcodes without executing constructors or decoding numeric calibration."""

from __future__ import annotations

import argparse
import base64
from collections import Counter
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import pickletools
import re
import resource
import signal
import sqlite3
import stat
import sys
import time
import zipfile

SCHEMA = 'nuplan.calibration_opcode_inventory.v1'
PLAN_SCHEMA = 'nuplan.calibration_opcode_followup_plan.v1'
PRIOR_SHA = '0b9a2a3de73c5e96af869307873a235c8279b4eaeaf6f328950973c3c97ef83e'
MEMBER = 'data/cache/mini/2021.06.03.12.02.06_veh-35_00233_00609.db'
MEMBER_SHA = 'deb9a23c2aee75228f112932df30f34b9214c4cb7c050cee8191ac9d4552390c'
MEMBER_BYTES, COMPRESSED_BYTES, CRC32 = 173629440, 100266146, '28c49074'
CHANNELS = ('CAM_B0', 'CAM_F0', 'CAM_L0', 'CAM_L1', 'CAM_L2', 'CAM_R0', 'CAM_R1', 'CAM_R2')
FIELDS = ('translation', 'rotation', 'intrinsic', 'distortion')
MAX_BLOB_BYTES, MAX_OPCODES, MAX_INTEGER_BITS, MAX_METADATA_BYTES = 65536, 4096, 64, 1048576
DYNAMIC = frozenset(('GLOBAL', 'STACK_GLOBAL', 'REDUCE', 'BUILD', 'INST', 'OBJ', 'NEWOBJ', 'NEWOBJ_EX',
    'EXT1', 'EXT2', 'EXT4', 'PERSID', 'BINPERSID', 'NEXT_BUFFER', 'READONLY_BUFFER'))
BACKREF = frozenset(('GET', 'BINGET', 'LONG_BINGET'))
MEMO_WRITE = frozenset(('PUT', 'BINPUT', 'LONG_BINPUT', 'MEMOIZE'))
DENIALS = dict(terms_consent_provided=False, training_data_approved=False, conversion_allowed=False,
    training_allowed=False, dataset_export_allowed=False, numeric_calibration_extraction=False,
    pickle_execution_allowed=False, constructors_allowed=False)


def require(value, message):
    if not value: raise ValueError(message)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def utc():
    return datetime.now(timezone.utc).isoformat()


def regular(path):
    path = Path(path).absolute()
    require(path.is_file() and not any(p.is_symlink() for p in (path, *path.parents)), 'regular nonsymlink input required')
    info = path.stat(); require(stat.S_ISREG(info.st_mode), 'special input forbidden')
    return dict(size_bytes=info.st_size, mtime_ns=info.st_mtime_ns, ctime_ns=info.st_ctime_ns, device=info.st_dev, inode=info.st_ino)


def read_small(path, expected=None):
    before = regular(path); require(before['size_bytes'] <= MAX_METADATA_BYTES, 'oversized metadata')
    raw = Path(path).read_bytes()
    require(regular(path) == before and (expected is None or sha(raw) == expected), 'metadata identity mismatch')
    return raw


def json_load(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'duplicate JSON key'); result[key] = value
        return result
    return json.loads(raw, object_pairs_hook=pairs,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite metadata')))


def write_new(path, raw):
    require(not path.exists() and not path.is_symlink() and not any(p.is_symlink() for p in path.parents), 'existing or unsafe output')
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('xb') as stream: stream.write(raw)


def write_json(path, value):
    write_new(path, (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n').encode())


def argument_descriptor(value):
    # HH_260906 - Do not publish arbitrary pickle strings or huge scalar representations as trusted calibration values.
    if value is None: return None
    if type(value) is bool: return dict(kind='boolean', value=value)
    if type(value) is int:
        bits = value.bit_length()
        return dict(kind='integer', bits=bits, sign=-1 if value < 0 else 1,
            bounded_value=value if bits <= MAX_INTEGER_BITS else None)
    if type(value) is float:
        return dict(kind='float', finite=math.isfinite(value), bounded_value=value if math.isfinite(value) else None)
    if isinstance(value, bytes): return dict(kind='bytes', size_bytes=len(value), sha256=sha(value))
    if isinstance(value, str):
        raw = value.encode('utf-8', errors='surrogatepass')
        return dict(kind='string', codepoints=len(value), utf8_bytes=len(raw), sha256=sha(raw),
            identifier_hint=value if len(value) <= 128 and re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.]*', value) else None)
    return dict(kind='unsupported_argument_type', type_name=type(value).__name__)


def inspect_pickle_opcodes(raw):
    # HH_260906 - pickletools parses encodings only; no pickle.load/loads/Unpickler, globals lookup or VM execution occurs.
    require(type(raw) is bytes and 0 < len(raw) <= MAX_BLOB_BYTES, 'bounded nonempty BLOB required')
    result = dict(blob_sha256=sha(raw), size_bytes=len(raw), parse_status='REJECTED', semantic_status='UNRESOLVED_INVENTORY_ONLY',
        numeric_calibration_extracted=False, constructors_executed=False, stack_semantics_validated=False,
        memo_cycles_proved_absent=False, opcodes=[], frames=[], dynamic_opcode_count=0, backreference_count=0, stack_duplication_count=0,
        descriptor_scope='Lexical hints only: nearby literals/global declarations are not proved REDUCE operands or resolved symbols.')
    stream = io.BytesIO(raw); frame_end = None; protocol = None; literals = []; globals_seen = []
    try:
        for opcode, argument, offset in pickletools.genops(stream):
            end = stream.tell(); name = opcode.name
            require(len(result['opcodes']) < MAX_OPCODES, 'opcode-count limit exceeded')
            require(0 <= offset < end <= len(raw), 'opcode byte range invalid')
            if frame_end is not None and offset == frame_end: frame_end = None
            require(frame_end is None or end <= frame_end, 'opcode crosses FRAME boundary')
            if name == 'PROTO':
                require(offset == 0 and protocol is None and type(argument) is int and 0 <= argument <= 5, 'unsupported or misplaced protocol')
                protocol = argument
            elif opcode.proto >= 2:
                require(protocol is not None and opcode.proto <= protocol, 'opcode requires declared protocol')
            entry = dict(index=len(result['opcodes']), offset=offset, end_offset=end, name=name,
                minimum_protocol=opcode.proto, encoded_sha256=sha(raw[offset:end]), argument=argument_descriptor(argument))
            result['opcodes'].append(entry)
            if name == 'FRAME':
                require(frame_end is None and type(argument) is int and 0 <= argument <= len(raw) - end, 'invalid or nested FRAME length')
                frame_end = end + argument
                result['frames'].append(dict(opcode_offset=offset, payload_offset=end, end_offset=frame_end, declared_bytes=argument))
            if type(argument) is int:
                require(argument.bit_length() <= MAX_INTEGER_BITS, 'integer argument exceeds bounded inventory contract')
            if name in MEMO_WRITE | BACKREF and name != 'MEMOIZE':
                require(type(argument) is int and 0 <= argument <= MAX_OPCODES, 'memo index exceeds bounded inventory contract')
            if isinstance(argument, str) and entry['argument'].get('identifier_hint'):
                literals.append(dict(offset=offset, identifier=entry['argument']['identifier_hint']))
                literals = literals[-4:]
            if name in ('GLOBAL', 'INST'):
                parts = argument.split(' ') if isinstance(argument, str) else []
                declaration = dict(offset=offset, opcode=name, module=None, symbol=None)
                if len(parts) == 2 and all(len(p) <= 128 and re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.]*', p) for p in parts):
                    declaration.update(module=parts[0], symbol=parts[1])
                entry['global_declaration'] = declaration; globals_seen = (globals_seen + [declaration])[-4:]
            if name in DYNAMIC:
                result['dynamic_opcode_count'] += 1
                entry['execution'] = 'NEVER_EXECUTED_UNRESOLVED'
                entry['nearby_identifier_hints_not_resolved_operands'] = list(literals)
                entry['nearby_global_declarations_not_resolved_callables'] = list(globals_seen)
            if name in BACKREF: result['backreference_count'] += 1
            if name == 'DUP': result['stack_duplication_count'] += 1
            if name == 'STOP':
                require(end == len(raw) and (frame_end is None or frame_end == end), 'trailing bytes or incomplete FRAME after STOP')
                result['parse_status'] = 'PARSED_ENCODING_ONLY'
        require(result['parse_status'] == 'PARSED_ENCODING_ONLY', 'missing final STOP')
    except Exception as error:
        result.update(parse_status='REJECTED', error_type=type(error).__name__, error=str(error)[:240])
    result['declared_protocol'] = protocol
    result['opcode_counts'] = dict(sorted(Counter(row['name'] for row in result['opcodes']).items()))
    # HH_260906 - GET is not the only alias source: DUP can form a cycle, and unknown constructors remain unresolved too.
    result['potential_alias_or_cycle_unresolved'] = True
    result['inventoried_bytes'] = result['opcodes'][-1]['end_offset'] if result['opcodes'] else 0
    result['parser_consumed_bytes'] = stream.tell()
    result['uninventoried_bytes'] = len(raw)-result['inventoried_bytes']
    result['unread_bytes'] = max(0,len(raw)-stream.tell())
    return result


def calibration_pins(prior):
    require(prior.get('status') == 'METADATA_INSPECTED_NOT_READY' and prior.get('member_sha256') == MEMBER_SHA
        and prior.get('archive_identity_unchanged') is True and prior.get('database_payloads_opened') == 1, 'prior DB proof differs')
    cameras = prior['metadata']['calibration_opaque_fields']
    require(len(cameras) == 8 and {r['channel'] for r in cameras} == set(CHANNELS), 'eight prior camera records required')
    result = {}
    for camera in cameras:
        require(set(camera['fields']) == set(FIELDS), 'four prior fields required')
        for name, record in camera['fields'].items():
            require(record['storage_type'] == 'blob' and type(record['size_bytes']) is int and 0 < record['size_bytes'] <= MAX_BLOB_BYTES
                and re.fullmatch('[0-9a-f]{64}', record['sha256']) and record['deserialized'] is False, 'invalid opaque field proof')
            result[camera['channel'] + '/' + name] = dict(size_bytes=record['size_bytes'], sha256=record['sha256'])
    require(len(result) == 32, '32 original BLOB identities required')
    return dict(sorted(result.items()))


def build_plan(prior, *, declared_at_utc, source_sha256):
    # HH_260906 - This is a new one-member read authorization; the previous opaque-only selection plan is never extended or rewritten.
    return dict(schema=PLAN_SCHEMA, declared_at_utc=declared_at_utc, source_sha256=source_sha256,
        authorization_scope='NEW_SEPARATE_METADATA_FOLLOWUP_NOT_ORIGINAL_PLAN', previous_report_sha256=PRIOR_SHA,
        original_selection_plan_sha256=prior['selection_plan_sha256'], member=MEMBER, member_sha256=MEMBER_SHA,
        uncompressed_bytes=MEMBER_BYTES, compressed_bytes=COMPRESSED_BYTES, crc32=CRC32, compression_method=8,
        archive_name='nuplan-v1.1_mini.zip', archive_identity=prior['archive_post_identity'],
        previous_archive_content_sha256=prior['archive_previous_content_sha256'], calibration_blob_pins=calibration_pins(prior),
        database_payloads_allowed=1, camera_payloads_allowed=0, lidar_payloads_allowed=0, map_payloads_allowed=0,
        maximum_blob_bytes=MAX_BLOB_BYTES, maximum_opcodes_per_blob=MAX_OPCODES, maximum_integer_bits=MAX_INTEGER_BITS,
        maximum_address_space_bytes=2*1024**3, maximum_cpu_seconds=60, cooperative_wall_timeout_seconds=85,
        external_wall_timeout_seconds=90, finish_before_utc='2026-09-09T01:00:00Z',
        retain_opaque_calibration_blobs_private=True, maximum_retained_blob_bytes=32*MAX_BLOB_BYTES,
        database_written_to_disk=False, numeric_mode='OPCODE_INVENTORY_ONLY_NO_PRIMITIVE_INTERPRETER', **DENIALS)


def validate_plan(plan, prior, source_sha):
    expected = build_plan(prior, declared_at_utc=plan.get('declared_at_utc'), source_sha256=source_sha)
    require(json.dumps(plan, sort_keys=True) == json.dumps(expected, sort_keys=True), 'unreviewed follow-up scope or source')
    require(re.fullmatch('[0-9a-f]{64}', source_sha or ''), 'source hash required')
    declared = datetime.fromisoformat(plan['declared_at_utc'].replace('Z', '+00:00'))
    deadline = datetime.fromisoformat(plan['finish_before_utc'].replace('Z', '+00:00')); now = datetime.now(timezone.utc)
    require(declared.tzinfo is not None and declared.utcoffset().total_seconds() == 0 and declared <= now
        and (deadline-now).total_seconds() >= 120, 'declaration or completion budget invalid')


def select_member(infos, plan):
    entries = [entry for entry in infos if entry.filename == MEMBER]
    require(len(entries) == 1, 'selected DB member is missing or duplicated')
    entry = entries[0]
    require(entry.file_size == plan['uncompressed_bytes'] and entry.compress_size == plan['compressed_bytes']
        and entry.compress_type == 8 and f'{entry.CRC:08x}' == CRC32 and not entry.is_dir()
        and not entry.flag_bits & 1 and not stat.S_ISLNK(entry.external_attr >> 16), 'DB central-directory identity mismatch')
    return entry


def camera_authorizer(action, first, second, database, trigger):
    # HH_260906 - Even though all SQLite bytes are hash-pinned, this follow-up permits camera metadata and schema reads only.
    if trigger is not None: return sqlite3.SQLITE_DENY
    if action == sqlite3.SQLITE_SELECT: return sqlite3.SQLITE_OK
    if action == sqlite3.SQLITE_READ and database == 'main' and (
        first in ('sqlite_master', 'sqlite_schema') or first == 'camera' and second in ('channel', *FIELDS)):
        return sqlite3.SQLITE_OK
    return sqlite3.SQLITE_DENY


def camera_blobs_from_memory(raw, pins):
    require(len(raw) == MEMBER_BYTES and sha(raw) == MEMBER_SHA, 'full selected database SHA mismatch')
    require(raw[:16] == b'SQLite format 3\0' and raw[18:20] == b'\x01\x01', 'unsupported SQLite format/WAL header')
    require(hasattr(sqlite3.Connection, 'deserialize'), 'SQLite in-memory loading unavailable; no disk fallback')
    connection = sqlite3.connect(':memory:')
    try:
        connection.deserialize(raw); connection.enable_load_extension(False)
        if hasattr(connection, 'setconfig') and hasattr(sqlite3, 'SQLITE_DBCONFIG_DEFENSIVE'):
            connection.setconfig(sqlite3.SQLITE_DBCONFIG_DEFENSIVE, True)
        for key, value in (('temp_store','MEMORY'), ('trusted_schema','OFF'), ('query_only','ON')):
            connection.execute('PRAGMA '+key+'='+value)
        require(connection.execute('PRAGMA temp_store').fetchone()[0] == 2 and
            connection.execute('PRAGMA trusted_schema').fetchone()[0] == 0 and
            connection.execute('PRAGMA query_only').fetchone()[0] == 1, 'SQLite safeguards unavailable')
        connection.set_authorizer(camera_authorizer)
        deadline = time.monotonic()+55
        connection.set_progress_handler(lambda: int(time.monotonic() >= deadline), 1000)
        schema = connection.execute("SELECT name,type,sql FROM sqlite_schema WHERE name='camera'").fetchmany(2)
        require(len(schema) == 1 and schema[0][1] == 'table' and isinstance(schema[0][2],str), 'camera must be an ordinary table')
        sql = re.sub(r'/\*.*?\*/|--[^\n]*', ' ', schema[0][2], flags=re.DOTALL)
        require(not re.search(r'\bCREATE\s+VIRTUAL\s+TABLE\b',sql,re.IGNORECASE), 'virtual camera table forbidden')
        rows = connection.execute('SELECT channel,translation,rotation,intrinsic,distortion FROM camera ORDER BY channel').fetchmany(9)
        require(len(rows) == 8 and [r[0] for r in rows] == list(CHANNELS), 'eight unique camera channels required')
        blobs = {row[0]+'/'+name:blob for row in rows for name,blob in zip(FIELDS,row[1:])}
        require(set(blobs) == set(pins), '32 BLOB field names differ')
        for name,blob in blobs.items():
            require(type(blob) is bytes and 0 < len(blob) <= MAX_BLOB_BYTES and
                dict(size_bytes=len(blob),sha256=sha(blob)) == pins[name], 'original BLOB SHA/size mismatch')
        return blobs
    finally: connection.close()


@contextmanager
def guarded_finalization():
    signals = (signal.SIGALRM,signal.SIGINT,signal.SIGTERM)
    previous = {sig:signal.getsignal(sig) for sig in signals};signal.alarm(0)
    try:
        for sig in signals:signal.signal(sig,signal.SIG_IGN)
        yield
    finally:
        for sig,handler in previous.items():signal.signal(sig,handler)


def run(args):
    plan_raw = base64.b64decode(args.plan_base64, validate=True)
    require(len(plan_raw) <= MAX_METADATA_BYTES, 'plan too large')
    plan = json_load(plan_raw); prior_path = Path(args.prior_report).absolute()
    prior_raw = read_small(prior_path,PRIOR_SHA);prior = json_load(prior_raw)
    source = globals().get('EXECUTED_SOURCE_BYTES');source_path = None
    if source is None:
        source_path = Path(__file__).absolute();source = read_small(source_path)
    require(type(source) is bytes,'source bytes required');source_sha = sha(source)
    validate_plan(plan,prior,source_sha)
    personal = Path(args.personal_root).absolute();archive_root = Path(args.archive_root).absolute();output = Path(args.output_dir).absolute()
    require(personal == personal.resolve() and archive_root == archive_root.resolve() and prior_path == prior_path.resolve()
        and output == output.resolve(), 'canonical nonsymlink scope paths required')
    require(Path(sys.prefix).absolute() == personal/'portable_e2e/venvs/py312' and os.environ.get('CUDA_VISIBLE_DEVICES') == ''
        and os.environ.get('PYTHONNOUSERSITE') == '1', 'existing personal venv and explicit CPU-only environment required')
    require(archive_root == personal/'dataset/raw/nuplan/v1.1-mini/research-only-pending-terms-review','unexpected archive scope')
    require(prior_path.is_relative_to(personal/'portable_e2e/runs/diagnostics'),'prior report must remain in personal diagnostics')
    require(output.is_relative_to(personal/'portable_e2e/runs/diagnostics') and not output.exists()
        and not any(p.is_symlink() for p in (output,*output.parents)) and not output.is_relative_to(prior_path.parent)
        and not prior_path.parent.is_relative_to(output),'fresh separate personal diagnostic output required')
    archive_path = archive_root/plan['archive_name'];before = regular(archive_path)
    require(before == plan['archive_identity'],'archive stat changed before follow-up')
    output.mkdir(parents=True,exist_ok=False)
    result = dict(schema=SCHEMA,status='RUNNING',started_at_utc=utc(),source_sha256=source_sha,
        python_version=sys.version.split()[0],sqlite_version=sqlite3.sqlite_version,
        opcode_parser='stdlib pickletools.genops; no pickle virtual machine or semantic interpreter',
        followup_plan_sha256=sha(plan_raw),previous_report_sha256=PRIOR_SHA,original_selection_plan_sha256=prior['selection_plan_sha256'],
        archive_pre_identity=before,archive_whole_content_rehashed_this_run=False,previous_archive_content_sha256=plan['previous_archive_content_sha256'],
        database_payloads_opened=0,database_written_to_disk=False,camera_payloads_read=0,lidar_payloads_read=0,map_payloads_read=0,
        numeric_calibration_extracted=False,pickle_vm_or_constructors_executed=False,primitive_interpreter_implemented=False,
        opaque_calibration_blob_copies_written=0,blob_reports=[],**DENIALS)
    try:
        write_new(output/'followup_plan.json',plan_raw);write_new(output/'source_executed.py',source)
        with zipfile.ZipFile(archive_path) as archive:
            member = select_member(archive.infolist(),plan);result['database_payloads_opened']=1
            with archive.open(member) as stream:raw = stream.read(MEMBER_BYTES+1)
        require(len(raw)==MEMBER_BYTES and sha(raw)==MEMBER_SHA,'selected full DB payload SHA mismatch')
        result.update(member_sha256=sha(raw),database_decompressed_in_memory=True,member_crc_verified_to_eof=True)
        blobs = camera_blobs_from_memory(raw,plan['calibration_blob_pins']);del raw
        for name,blob in sorted(blobs.items()):
            inspected = inspect_pickle_opcodes(blob);channel,field=name.split('/')
            relative = 'opaque_blobs/'+channel+'/'+field+'.bin';write_new(output/relative,blob)
            require(sha((output/relative).read_bytes())==plan['calibration_blob_pins'][name]['sha256'],'retained BLOB bytes differ')
            result['opaque_calibration_blob_copies_written']+=1
            result['blob_reports'].append(dict(channel=channel,field=field,retained_private_blob=relative,**inspected))
        require(len(result['blob_reports'])==32,'all32 calibration fields must be reported')
        result['status']='OPCODE_INVENTORY_COMPLETE_NOT_DECODED'
        result['parse_rejection_count']=sum(r['parse_status']=='REJECTED' for r in result['blob_reports'])
        if result['parse_rejection_count']:result['status']='OPCODE_INVENTORY_COMPLETE_WITH_PARSE_REJECTIONS'
    except BaseException as error:
        result.update(status='FAILED_METADATA_FOLLOWUP',error_type=type(error).__name__,error=str(error)[:240])
    finally:
        with guarded_finalization():
            errors=[]
            for name,operation in (
                ('archive_identity',lambda:regular(archive_path)==before),
                ('prior_report_bytes',lambda:read_small(prior_path,PRIOR_SHA)==prior_raw),
                ('source_copy',lambda:sha((output/'source_executed.py').read_bytes())==source_sha),
                ('source_identity',lambda:read_small(source_path)==source if source_path is not None else globals().get('EXECUTED_SOURCE_BYTES')==source),
                ('retained_blob_copies',lambda:all(sha(read_small(output/r['retained_private_blob']))==
                    plan['calibration_blob_pins'][r['channel']+'/'+r['field']]['sha256'] for r in result['blob_reports'])),
                ('plan_copy',lambda:(output/'followup_plan.json').read_bytes()==plan_raw)):
                try:require(operation(),name+' changed')
                except Exception as error:errors.append(dict(operation=name,error_type=type(error).__name__,error=str(error)[:240]))
            inventory={}
            for path in sorted(output.rglob('*')):
                if path.is_dir() and not path.is_symlink():continue
                try:
                    regular(path)
                    raw=read_small(path);inventory[str(path.relative_to(output))]=sha(raw)
                except Exception as error:errors.append(dict(operation='output_inventory',path=str(path.relative_to(output)),error_type=type(error).__name__,error=str(error)[:240]))
            result.update(postcheck_errors=errors,completed_at_utc=utc(),peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                missing_calibration_fields=sorted(set(plan['calibration_blob_pins'])-{r['channel']+'/'+r['field'] for r in result['blob_reports']}))
            result['finish_deadline_met'] = datetime.now(timezone.utc) < datetime.fromisoformat(plan['finish_before_utc'].replace('Z','+00:00'))
            if errors or not result['finish_deadline_met']:result['status']='FAILED_METADATA_FOLLOWUP'
            write_json(output/'report.json',result)
            inventory['report.json']=sha((output/'report.json').read_bytes())
            write_new(output/'SHA256SUMS',''.join(value+'  '+name+'\n' for name,value in sorted(inventory.items())).encode())
    print(json.dumps({k:result[k] for k in ('status','database_payloads_opened','opaque_calibration_blob_copies_written','numeric_calibration_extracted')}))
    return 2 if result['status']=='FAILED_METADATA_FOLLOWUP' else 0


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ('personal-root','archive-root','prior-report','output-dir','plan-base64'):parser.add_argument('--'+name,required=True)
    args=parser.parse_args(argv)
    resource.setrlimit(resource.RLIMIT_AS,(2*1024**3,2*1024**3));resource.setrlimit(resource.RLIMIT_CPU,(60,60))
    def stop(signum,_frame):raise TimeoutError('bounded metadata follow-up signal '+str(signum))
    previous={sig:signal.getsignal(sig) for sig in (signal.SIGALRM,signal.SIGTERM,signal.SIGINT)}
    try:
        for sig in previous:signal.signal(sig,stop)
        signal.alarm(85);return run(args)
    finally:
        signal.alarm(0)
        for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':raise SystemExit(main())
