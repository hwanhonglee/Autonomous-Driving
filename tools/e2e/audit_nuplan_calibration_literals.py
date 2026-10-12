#!/usr/bin/env python3
"""HH_260906 - Read fixed calibration numeric slots only after exact serialization-template proof, without a pickle VM."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import struct

from scripts.e2e import curate_nuplan_calibration_opcodes as inventory

audit = inventory.audit
SCHEMA = 'nuplan.calibration_literal_diagnostic.v1'
PLAN_SCHEMA = 'nuplan.calibration_literal_followup_plan.v1'
OPCODE_REPORT_SHA = 'f8ae13c4d042baa59714c84b2b395f45ee2cf078aab419bff0a97f44eaa76f27'
OPCODE_MANIFEST_SHA = 'ef4cb23f5ad91b099921a0d03889ad14c928c2ad7770813fffed1c46746b5d37'
CURATOR_SHA = '9502b315a77732d4c2d1f5c3ee2e1c36297edb1cf6804c7628876c1cef43fda3'
OFFICIAL_COMMIT = 'e9241677997dd86bfc0bcd44817ab04fe631405b'
OFFICIAL = {
    'nuplan/database/common/data_types.py': {'sha256':'8151180b93c99dda0ea1d9ec5d39644342783efc2a42fd1b3df47e5a9b8f381d','size_bytes':4970},
    'nuplan/database/nuplan_db_orm/camera.py': {'sha256':'f18eb1bc36d6b3c0743f177b8fb529e708956640e3444580376554e063efe0b8','size_bytes':3280},
}
Q_NORM_TOLERANCE = 1e-6
DENIALS = dict(calibration_trusted=False, projection_verified=False, tf_applied=False, image_correspondence_verified=False,
    physical_rig_verified=False, training_data_approved=False, dataset_exported=False, terms_consent_provided=False,
    pickle_vm_executed=False, constructors_executed=False, numpy_or_orm_executed=False, values_repaired=False)


def text_literal(value):
    raw=value.encode('ascii');audit.require(len(raw)<256,'bounded static identifier')
    return b'\x8c'+bytes([len(raw)])+raw+b'\x94'


def layout(field):
    # HH_260906 - These four complete byte templates are not a stack machine; no input opcode chooses an operation or a callable.
    audit.require(field in audit.FIELDS,'unknown calibration field')
    body=bytearray();slots=[]
    def fixed(raw):body.extend(raw)
    def numeric(encoding):
        fixed(b'C\x08' if encoding=='<d' else b'G')
        slots.append((11+len(body),encoding));fixed(b'\0'*8)
    if field in ('translation','rotation','intrinsic'):
        name={'translation':'Translation','rotation':'Rotation','intrinsic':'CameraIntrinsic'}[field]
        fixed(text_literal('nuplan.database.common.data_types')+text_literal(name)+b'\x93\x94)\x81\x94(')
    if field in ('translation','rotation'):
        fixed(text_literal('numpy.core.multiarray')+text_literal('scalar')+b'\x93\x94')
        fixed(text_literal('numpy')+text_literal('dtype')+b'\x93\x94'+text_literal('f8')+b'\x89\x88\x87\x94R\x94(')
        fixed(b'K\x03'+text_literal('<')+b'NNNJ\xff\xff\xff\xffJ\xff\xff\xff\xffK\x00t\x94b')
        numeric('<d');fixed(b'\x94\x86\x94R\x94')
        for _ in range(2 if field=='translation' else 3):
            # HH_260906 - Only the exact scalar-symbol memo6 and built little-endian f8 dtype memo12 may recur; no container reference is accepted.
            fixed(b'h\x06h\x0c');numeric('<d');fixed(b'\x94\x86\x94R\x94')
        fixed(b'e.')
    elif field=='intrinsic':
        for _ in range(3):
            fixed(b']\x94(')
            for _ in range(3):numeric('>d')
            fixed(b'e')
        fixed(b'e.')
    else:
        fixed(b']\x94(')
        for _ in range(5):numeric('>d')
        fixed(b'e.')
    return b'\x80\x05\x95'+struct.pack('<Q',len(body))+bytes(body),tuple(slots)


def decode_literals(raw,field):
    template,slots=layout(field)
    audit.require(type(raw) is bytes and len(raw)==len(template),'exact fixed serialization length required')
    masked=bytearray(raw)
    for offset,_ in slots:masked[offset:offset+8]=b'\0'*8
    audit.require(bytes(masked)==template,'unreviewed opcode, reference, dtype, endian, frame or trailing byte')
    # HH_260906 - Read IEEE-754 only after every nonnumeric byte matched; NaN/Inf remain explicit failures, never JSON NaN or repaired values.
    values=[];proof=[]
    for offset,encoding in slots:
        value=struct.unpack(encoding,raw[offset:offset+8])[0];finite=math.isfinite(value)
        values.append(value if finite else None)
        proof.append(dict(offset=offset,size_bytes=8,encoding=encoding,raw_slot_sha256=audit.sha(raw[offset:offset+8]),finite=finite,
            nonfinite_kind=None if finite else 'NaN' if math.isnan(value) else 'negative_infinity' if value<0 else 'positive_infinity'))
    return dict(template_sha256=audit.sha(template),template_bytes=len(template),template_matched=True,
        finite=all(item['finite'] for item in proof),values=values,numeric_slots=proof,
        representation={'translation':'xyz','rotation':'wxyz','intrinsic':'row_major_3x3','distortion':'five_coefficients_order_not_independently_validated'}[field])


def camera_checks(fields,width=1920,height=1080):
    # HH_260906 - Sanity tests never normalize quaternions, change intrinsics, infer projection, or approve the sensor installation.
    complete=set(fields)==set(audit.FIELDS) and all(row.get('template_matched') is True for row in fields.values())
    finite=complete and all(row.get('finite') is True for row in fields.values())
    result=dict(all_templates_matched=complete,all_values_finite=finite,metadata_dimensions_match=(width,height)==(1920,1080),
        quaternion_wxyz_norm=None,quaternion_norm_error=None,quaternion_norm_within_tolerance=False,
        intrinsic_shape_3x3=False,intrinsic_last_row_001=False,intrinsic_lower_left_zero=False,focal_lengths_positive=False,
        principal_point_within_metadata_dimensions=False,sanity_checks_all_pass=False)
    if not finite:return result
    t,q,k,d=(fields[f]['values'] for f in audit.FIELDS)
    audit.require(len(t)==3 and len(q)==4 and len(k)==9 and len(d)==5,'internal exact slot count differs')
    norm=math.hypot(*q);error=abs(norm-1)
    result.update(quaternion_wxyz_norm=norm if math.isfinite(norm) else None,quaternion_norm_error=error if math.isfinite(error) else None,
        quaternion_norm_within_tolerance=math.isfinite(error) and error<=Q_NORM_TOLERANCE,
        intrinsic_shape_3x3=True,intrinsic_last_row_001=k[6:]==[0.,0.,1.],intrinsic_lower_left_zero=k[3]==0.,
        focal_lengths_positive=k[0]>0 and k[4]>0,
        principal_point_within_metadata_dimensions=0<=k[2]<width and 0<=k[5]<height)
    result['sanity_checks_all_pass']=all(result[key] is True for key in (
        'all_templates_matched','all_values_finite','metadata_dimensions_match','quaternion_norm_within_tolerance',
        'intrinsic_shape_3x3','intrinsic_last_row_001','intrinsic_lower_left_zero','focal_lengths_positive','principal_point_within_metadata_dimensions'))
    return result


def official_pins():
    return {name:dict(info,url='https://raw.githubusercontent.com/motional/nuplan-devkit/'+OFFICIAL_COMMIT+'/'+name) for name,info in OFFICIAL.items()}


def build_plan(prior,opcode_plan,*,declared_at_utc,source_sha256):
    return dict(schema=PLAN_SCHEMA,declared_at_utc=declared_at_utc,source_sha256=source_sha256,
        authorization_scope='NEW_LOCAL_RETAINED_BLOB_INTERPRETATION_NOT_OPCODE_PLAN_EXTENSION',
        original_opcode_plan_sha256=inventory.PLAN_SHA,original_opcode_report_sha256=OPCODE_REPORT_SHA,
        original_opcode_manifest_sha256=OPCODE_MANIFEST_SHA,original_opcode_worker_sha256=inventory.SOURCE_SHA,
        original_report_sha256=audit.PRIOR_SHA,original_db_member_sha256=audit.MEMBER_SHA,
        opcode_publication_helper_sha256=CURATOR_SHA,calibration_blob_pins=audit.calibration_pins(prior),
        original_opcode_plan_declared_at_utc=opcode_plan['declared_at_utc'],official_source_commit=OFFICIAL_COMMIT,
        official_source_pins=official_pins(),template_sha256={f:audit.sha(layout(f)[0]) for f in audit.FIELDS},
        expected_private_input_files=36,expected_fields=32,expected_cameras=8,expected_metadata_width=1920,expected_metadata_height=1080,
        quaternion_norm_absolute_tolerance=Q_NORM_TOLERANCE,intrinsic_last_row='exact [0,0,1]',
        intrinsic_lower_left='exact zero',focal_lengths='strictly positive',principal_point='0<=cx<1920 and 0<=cy<1080',
        maximum_wall_seconds=30,external_wall_timeout_seconds=45,finish_before_utc='2026-09-09T01:00:00Z',
        database_reads=0,zip_reads=0,image_reads=0,model_inference=0,raw_blobs_exported=False,**DENIALS)


def validate_plan(plan,prior,opcode_plan,source_sha):
    expected=build_plan(prior,opcode_plan,declared_at_utc=plan.get('declared_at_utc'),source_sha256=source_sha)
    audit.require(inventory.encoded(plan)==inventory.encoded(expected),'new local literal plan or source differs')
    times=[datetime.fromisoformat(value.replace('Z','+00:00')) for value in (plan['declared_at_utc'],plan['finish_before_utc'])]
    now=datetime.now(timezone.utc)
    audit.require(all(t.tzinfo is not None and t.utcoffset().total_seconds()==0 for t in times)
        and times[0]<=now and (times[1]-now).total_seconds()>=60,'prospective declaration or cleanup budget invalid')


def run(args):
    audit.require(os.environ.get('CUDA_VISIBLE_DEVICES')=='' and os.environ.get('PYTHONNOUSERSITE')=='1','explicit CPU and no user-site environment required')
    root,prior_path,references,plan_path,output=(Path(getattr(args,name)).absolute() for name in ('input_root','prior_report','reference_root','plan','output_dir'))
    audit.require(not output.exists() and not output.is_symlink() and not any(p.is_symlink() for p in output.parents)
        and all(not output.resolve().is_relative_to(p.resolve()) and not p.resolve().is_relative_to(output.resolve()) for p in (root,prior_path.parent,references))
        and not output.resolve().is_relative_to((inventory.REPO/'datasets').resolve()),'fresh separate nondataset diagnostic output required')
    pins={}
    checked=lambda path,expected=None:inventory.checked(path,pins,expected)
    source=checked(Path(__file__).absolute());source_sha=audit.sha(source)
    checked(Path(inventory.__file__).absolute(),CURATOR_SHA)
    prior=audit.json_load(checked(prior_path,audit.PRIOR_SHA))
    opcode_plan=audit.json_load(checked(root/'followup_plan.json',inventory.PLAN_SHA))
    plan_raw=checked(plan_path);plan=audit.json_load(plan_raw)
    validate_plan(plan,prior,opcode_plan,source_sha)
    for name,info in OFFICIAL.items():
        raw=checked(references/name,info['sha256']);audit.require(len(raw)==info['size_bytes'],'official reference byte length differs')
    # HH_260906 - This reuses the previously reviewed static parser over retained local BLOBs only; it never opens their original SQLite/ZIP.
    opcode_report,_,bound,_=inventory.bind_inputs(root,prior_path,OPCODE_REPORT_SHA,OPCODE_MANIFEST_SHA)
    pins.update(bound)
    audit.require(datetime.fromisoformat(plan['declared_at_utc'].replace('Z','+00:00'))>=
        datetime.fromisoformat(opcode_report['completed_at_utc'].replace('Z','+00:00')),
        'new local interpretation must be declared after the original opcode observation completed')
    dimensions={r['channel']:(r['width'],r['height']) for r in prior['metadata']['calibration_opaque_fields']}
    output.mkdir(parents=True,exist_ok=False)
    copied={'plan.json':audit.sha(plan_raw),'source_executed.py':source_sha,
        **{'reference_source/'+name:info['sha256'] for name,info in OFFICIAL.items()}}
    report=dict(schema=SCHEMA,status='RUNNING',started_at_utc=audit.utc(),source_sha256=source_sha,plan_sha256=audit.sha(plan_raw),
        original_opcode_report_sha256=OPCODE_REPORT_SHA,original_opcode_manifest_sha256=OPCODE_MANIFEST_SHA,
        original_opcode_status=opcode_report['status'],official_source_commit=OFFICIAL_COMMIT,official_source_pins=official_pins(),
        database_reads=0,zip_reads=0,image_reads=0,model_inference=0,fields=[],cameras=[],**DENIALS)
    try:
        audit.write_new(output/'plan.json',plan_raw);audit.write_new(output/'source_executed.py',source)
        for name in OFFICIAL:audit.write_new(output/'reference_source'/name,checked(references/name))
        for channel in audit.CHANNELS:
            fields={}
            for field in audit.FIELDS:
                key=channel+'/'+field;pin=plan['calibration_blob_pins'][key]
                raw=checked(root/'opaque_blobs'/channel/(field+'.bin'),pin['sha256'])
                row=dict(channel=channel,field=field,source_blob_sha256=pin['sha256'],source_blob_bytes=len(raw))
                try:row.update(status='LITERAL_SLOTS_READ_NOT_TRUSTED',**decode_literals(raw,field))
                except ValueError as error:row.update(status='REJECTED_TEMPLATE',template_matched=False,error=str(error))
                fields[field]=row;report['fields'].append(row)
            width,height=dimensions[channel]
            report['cameras'].append(dict(channel=channel,metadata_width=width,metadata_height=height,
                image_dimensions_verified_by_decode=False,checks=camera_checks(fields,width,height)))
        report['status']='LITERAL_DIAGNOSTIC_COMPLETE_NOT_TRUSTED'
    except BaseException as error:
        report.update(status='FAILED_LITERAL_DIAGNOSTIC',error_type=type(error).__name__,error=str(error)[:240])
    finally:
        with audit.guarded_finalization():
            errors=[]
            for path,digest in pins.items():
                try:checked(path,digest)
                except Exception as error:errors.append(dict(input_sha256=digest,error_type=type(error).__name__))
            try:
                audit.require(inventory.tree_files(root)==set(inventory.manifest_entries((root/'SHA256SUMS').read_bytes()))|{'SHA256SUMS'},'input tree changed')
                audit.require(inventory.tree_files(output)==set(copied),'output source/reference inventory changed')
            except Exception as error:errors.append(dict(check='inventory_and_source_copies',error_type=type(error).__name__))
            payloads={}
            for name,digest in copied.items():
                try:payloads[name]=audit.sha(audit.read_small(output/name,digest))
                except Exception as error:errors.append(dict(check='copied_reference_or_source',path=name,error_type=type(error).__name__))
            report.update(completed_at_utc=audit.utc(),postcheck_errors=errors,inputs_unchanged_before_after=not errors,
                verified_input_sha256=sorted(set(pins.values())),copied_source_sha256=payloads,
                reported_field_count=len(report['fields']),expected_field_count=32,reported_camera_count=len(report['cameras']),
                rejected_template_count=sum(r['status']=='REJECTED_TEMPLATE' for r in report['fields']),
                nonfinite_field_count=sum(r.get('finite') is False for r in report['fields']),
                sanity_pass_camera_count=sum(r['checks']['sanity_checks_all_pass'] for r in report['cameras']),
                missing_fields=sorted(set(plan['calibration_blob_pins'])-{r['channel']+'/'+r['field'] for r in report['fields']}))
            report['finish_deadline_met']=datetime.now(timezone.utc)<datetime.fromisoformat(plan['finish_before_utc'].replace('Z','+00:00'))
            if errors or not report['finish_deadline_met']:report['status']='FAILED_LITERAL_DIAGNOSTIC'
            audit.write_json(output/'report.json',report)
            payloads['report.json']=audit.sha(audit.read_small(output/'report.json'))
            audit.write_new(output/'SHA256SUMS',''.join(digest+'  '+name+'\n' for name,digest in sorted(payloads.items())).encode())
    return 0 if report['status']=='LITERAL_DIAGNOSTIC_COMPLETE_NOT_TRUSTED' else 2


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ('input-root','prior-report','reference-root','plan','output-dir'):parser.add_argument('--'+name,required=True)
    args=parser.parse_args(argv)
    def stop(signum,_frame):raise TimeoutError('local literal diagnostic signal '+str(signum))
    previous={sig:signal.getsignal(sig) for sig in (signal.SIGALRM,signal.SIGINT,signal.SIGTERM)}
    try:
        for sig in previous:signal.signal(sig,stop)
        signal.alarm(30);return run(args)
    finally:
        signal.alarm(0)
        for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':raise SystemExit(main())
