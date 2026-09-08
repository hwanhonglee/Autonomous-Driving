#!/usr/bin/env python3
"""HH_260906 - Publish completed, untrusted calibration literal metadata without reading DBs or opaque BLOBs."""
import argparse
from datetime import datetime
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[3]
RAW=ROOT/'artifacts/training/2026-09-09/nuplan_calibration_literals_v1'
OUTPUT=ROOT/'docs/assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/02_overnight_data_and_learning/05_real_data_readiness/calibration_literal_diagnostic'
PINS={'report.json':'0afe48afd7370ac06e4534f3ca6adc90768f014546b2bdbf54ec1de230a456db',
    'plan.json':'a3ba0de3faf4acbeedff0ab7a3c2a0ecd770f8e01adc8160f1b371cae84e87e2',
    'source_executed.py':'5709f4ec5e5fe61baef728b27df95839bf832927e27f62de02acc32a76f2078f',
    'SHA256SUMS':'693bfb5e1bcf171c17bc789eeeaab97359ea247ea99aaf211aba6b78b9124480'}
OFFICIAL_COMMIT='e9241677997dd86bfc0bcd44817ab04fe631405b'
REFERENCES={'nuplan/database/common/data_types.py':'8151180b93c99dda0ea1d9ec5d39644342783efc2a42fd1b3df47e5a9b8f381d',
    'nuplan/database/nuplan_db_orm/camera.py':'f18eb1bc36d6b3c0743f177b8fb529e708956640e3444580376554e063efe0b8'}
CHANNELS=('CAM_B0','CAM_F0','CAM_L0','CAM_L1','CAM_L2','CAM_R0','CAM_R1','CAM_R2')
SLOTS={'translation':3,'rotation':4,'intrinsic':9,'distortion':5}
DENIALS=('calibration_trusted','projection_verified','tf_applied','image_correspondence_verified','physical_rig_verified',
    'training_data_approved','dataset_exported','terms_consent_provided','pickle_vm_executed','constructors_executed',
    'numpy_or_orm_executed','values_repaired')
NOTICE='Public literal-diagnostic metadata view; raw_source_sha256 binds its unchanged private original. Values are not trusted calibration, applied TF, verified projection, a selected camera rig, or admitted training data.'


def require(value,message):
    if not value:raise ValueError(message)


def sha(raw):return hashlib.sha256(raw).hexdigest()


def digest(path):
    require(path.is_file() and not any(p.is_symlink() for p in (path,*path.parents)),'regular nonsymlink evidence required')
    return sha(path.read_bytes())


def decode(raw):
    def pairs(items):
        result={}
        for key,value in items:
            require(key not in result,'duplicate JSON key');result[key]=value
        return result
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=lambda _:(_ for _ in ()).throw(ValueError('nonfinite JSON')))


def encoded(value):return (json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n').encode()


def redact(value):
    if isinstance(value,str):
        value=re.sub(r'/(?:home)/[^/\s\"\']+',lambda _:'${USER_HOME}',value)
        return re.sub(r'/(?:tmp|root)(?=/|$)',lambda _:'${PRIVATE_ROOT}',value)
    if isinstance(value,list):return [redact(v)for v in value]
    if isinstance(value,dict):return {redact(k):redact(v)for k,v in value.items()}
    return value


def new_bytes(path,raw):
    require(not path.exists() and not any(p.is_symlink()for p in (path,*path.parents)),'create-only output required')
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('xb')as stream:stream.write(raw)


def verify_files():
    inventory={**PINS,**{'reference_source/'+name:value for name,value in REFERENCES.items()}}
    found={p.relative_to(RAW).as_posix():digest(p)for p in RAW.rglob('*')if p.is_file()or p.is_symlink()}
    require(found==inventory and len(found)==6,'exact completed six-file evidence differs')
    manifest={}
    for line in (RAW/'SHA256SUMS').read_text().splitlines():
        value,name=line.split('  ')
        require(name not in manifest and name in inventory and name!='SHA256SUMS','invalid original manifest path')
        require(value==inventory[name],'original manifest SHA mismatch');manifest[name]=value
    require(set(manifest)==set(inventory)-{'SHA256SUMS'},'five original payload checks required')
    return inventory


def validate_report(plan,report):
    require(plan.get('schema')=='nuplan.calibration_literal_followup_plan.v1'
        and plan.get('authorization_scope')=='NEW_LOCAL_RETAINED_BLOB_INTERPRETATION_NOT_OPCODE_PLAN_EXTENSION', 'new separate authorization required')
    require(report.get('schema')=='nuplan.calibration_literal_diagnostic.v1'
        and report.get('status')=='LITERAL_DIAGNOSTIC_COMPLETE_NOT_TRUSTED','completed untrusted literal diagnosis required')
    for data in (plan,report):
        require(all(data.get(name)is False for name in DENIALS),'trust, execution or admission flag changed')
        require(all(type(data.get(name))is int and data[name]==0 for name in ('database_reads','zip_reads','image_reads','model_inference')),'new payload/NN reads prohibited')
    require(plan.get('raw_blobs_exported')is False,'no opaque BLOB export')
    time=lambda s:datetime.fromisoformat(s.replace('Z','+00:00'))
    require(time(plan['original_opcode_plan_declared_at_utc'])<time(plan['declared_at_utc'])<=time(report['started_at_utc'])
        <=time(report['completed_at_utc'])<=time(plan['finish_before_utc']),'separate plan chronology differs')
    require(report.get('plan_sha256')==PINS['plan.json'] and plan.get('source_sha256')==PINS['source_executed.py']
        and report['copied_source_sha256']['source_executed.py']==PINS['source_executed.py'],'plan/worker binding differs')
    require(report.get('reported_camera_count')==8 and report.get('reported_field_count')==32
        and report.get('sanity_pass_camera_count')==8 and report.get('inputs_unchanged_before_after')is True
        and report.get('finish_deadline_met')is True and report.get('postcheck_errors')==[] and report.get('missing_fields')==[], 'full bounded diagnostic proof differs')
    require(plan.get('official_source_commit')==report.get('official_source_commit')==OFFICIAL_COMMIT,'official source version differs')
    for name,value in REFERENCES.items():
        expected='https://raw.githubusercontent.com/motional/nuplan-devkit/'+OFFICIAL_COMMIT+'/'+name
        require(plan['official_source_pins'][name]==report['official_source_pins'][name]
            and report['official_source_pins'][name]['sha256']==value and report['official_source_pins'][name]['url']==expected,'official reference binding differs')
    require([r['channel']for r in report['cameras']]==list(CHANNELS),'eight cameras in fixed order required')
    fields={(r['channel'],r['field']):r for r in report['fields']}
    require(len(report['fields'])==len(fields)==32 and set(fields)=={(c,f)for c in CHANNELS for f in SLOTS},'all32 fields required')
    table=[]
    for camera in report['cameras']:
        channel=camera['channel'];values={}
        require(camera['image_dimensions_verified_by_decode']is False and (camera['metadata_width'],camera['metadata_height'])==(1920,1080),'metadata dimensions are not image decode proof')
        for field,size in SLOTS.items():
            row=fields[channel,field];v=row['values'];pin=plan['calibration_blob_pins'][channel+'/'+field]
            require(row['status']=='LITERAL_SLOTS_READ_NOT_TRUSTED' and row['finite']is True and row['template_matched']is True
                and len(v)==size and all(type(n)in(int,float)and math.isfinite(n)for n in v),'untrusted finite literal slots differ')
            require(row['source_blob_sha256']==pin['sha256'] and row['source_blob_bytes']==pin['size_bytes']
                and row['template_sha256']==plan['template_sha256'][field],'original BLOB/template identities differ')
            values[field]=v
        k,d,q=values['intrinsic'],values['distortion'],values['rotation'];norm=math.hypot(*q)
        require(k[6:]==[0.,0.,1.] and k[3]==0. and k[0]>0 and k[4]>0 and 0<=k[2]<1920 and 0<=k[5]<1080
            and abs(norm-1)<=plan['quaternion_norm_absolute_tolerance'],'numeric sanity does not pass')
        require(camera['checks']['sanity_checks_all_pass']is True and camera['checks']['quaternion_wxyz_norm']==norm,'reported sanity/norm differs from literal recomputation')
        table.append(dict(channel=channel,metadata_width=1920,metadata_height=1080,fx=k[0],fy=k[4],cx=k[2],cy=k[5],
            quaternion_norm=norm,quaternion_norm_error=abs(norm-1),intrinsic_row_major=k,distortion_literals=d,
            numeric_sanity_only=True,calibration_trusted=False,projection_verified=False,tf_applied=False))
    return table


def readme(plan,report,table):
    text=['# nuPlan 보정값 literal 진단 — 숫자 형식만 확인', '',
        '<!-- HH_260906 - Publish numeric sanity without claiming a verified coordinate basis, projection, rig, or training admission. -->', '',
        '8개 카메라·32개 필드의 저장된 숫자 슬롯을 검사했고, 유한값·K 구조·주점 범위·quaternion norm 검사만 8/8 통과했습니다. **보정값 신뢰·TF 적용·실제 센서 배치·이미지 투영·학습 사용은 모두 미승인입니다.**', '',
        '이번은 2026-09-09 04:41:10 KST(2026-09-08 19:41:10 UTC)에 별도로 선언한 로컬 진단입니다. 앞선 opcode-only 계획의 권한을 소급 확대하지 않았습니다. 기존에 보존된32개 BLOB에 대해 전체 비숫자 템플릿을 맞춘 뒤 지정된 IEEE-754 슬롯만 읽었습니다. pickle VM·생성자·NumPy·ORM은 실행하지 않았고 DB·ZIP·이미지를 다시 읽지 않았습니다.', '',
        '| 카메라 | metadata 해상도 | fx / fy | cx / cy | quaternion norm 오차 | 숫자 sanity |',
        '| --- | --- | ---: | ---: | ---: | --- |']
    for row in table:text.append(f"| {row['channel']} | 1920×1080 | {row['fx']:g} / {row['fy']:g} | {row['cx']:g} / {row['cy']:g} | {row['quaternion_norm_error']:.3g} | PASS · 미신뢰 |")
    same_k=all(r['intrinsic_row_major']==table[0]['intrinsic_row_major']for r in table)
    same_d=all(r['distortion_literals']==table[0]['distortion_literals']for r in table)
    text += ['',f"8개에 공통인 K literal: `{table[0]['intrinsic_row_major']}` (모두 동일: {same_k}).",
        f"공통 distortion literal: `{table[0]['distortion_literals']}` (모두 동일: {same_d}).", '',
        '이 값이 같은 것은 저장 값의 관측일 뿐, 실차가 동일한 광학계라는 증거가 아닙니다. distortion 계수 순서·모델, optical 축과 변환 방향, 카메라와 차량 좌표계 관계, 실제 해상도·왜곡·투영 정확성은 별도 검증해야 합니다. quaternion을 보정하거나 값·시각을 바꾸지 않았습니다.', '',
        '[별도 계획](plan.json) · [전체 진단과32개 필드](report.json) · [8개 숫자 요약](camera_sanity.json) · [실행 source](execution_source.py) · [원본/공개SHA](publication_manifest.json) · [공개 체크섬](SHA256SUMS)', '',
        '[이전 opcode 진단](../calibration_opcode_inventory/README.md) · [실제 데이터 준비 상태](../README.md)', '',
        '## 참고 소스와 남은 제한', '']
    for name,row in plan['official_source_pins'].items():text += [f"- [{name}]({row['url']}) — commit `{OFFICIAL_COMMIT}`, SHA256 `{row['sha256']}`."]
    text += ['', '공식 참고 코드 두 파일은 검증에 사용한 private 사본으로만 보존했습니다. 여기에는 링크·commit·SHA만 싣고 원문 코드를 재배포하지 않습니다. 원본32개 BLOB·DB·이미지는 공개하지 않습니다. 원본SHA 파일은 비공개6개 원본용 참조이며, 이 폴더는 자체SHA256SUMS로 검증하세요.', '',
        '다음에는 이미지 대응과 투영 검증, 8개 중 실제 사용할6개 rig 정의, native 시간 정렬, 변환 방향·축 검증, 약관 검토가 필요합니다. 이번 진단은 해당 검증이나 데이터 변환·학습·모델 추론을 수행한 것이 아닙니다.', '']
    return '\n'.join(text)


def privacy_links():
    allowed={row for row in ['https://raw.githubusercontent.com/motional/nuplan-devkit/'+OFFICIAL_COMMIT+'/'+name for name in REFERENCES]}
    for path in OUTPUT.rglob('*'):
        if not path.is_file():continue
        text=path.read_text();require(not re.search(r'/(?:home/[^/\s]+|tmp|root)/',text),'private account path')
        for candidate in re.findall(r'(?<![\d.])(?:\d{1,3}\.){3}\d{1,3}(?![\d.])',text):
            try:ipaddress.IPv4Address(candidate)
            except ipaddress.AddressValueError:continue
            raise ValueError('private IPv4')
        if path.suffix=='.md':
            for target in re.findall(r'\]\(([^)]+)\)',text):
                require(target in allowed or (not target.startswith(('http:','https:','file:'))and(path.parent/target).resolve().is_file()),'broken or unreviewed link')


def publish():
    require(not OUTPUT.exists()and not any(p.is_symlink()for p in (OUTPUT,*OUTPUT.parents)),'fresh category only')
    inventory=verify_files();plan=decode((RAW/'plan.json').read_bytes());report=decode((RAW/'report.json').read_bytes())
    table=validate_report(plan,report);self_sha=digest(Path(__file__).resolve());entries={}
    for name,target in (('plan.json','plan.json'),('report.json','report.json'),('source_executed.py','execution_source.py'),('SHA256SUMS','original_private_SHA256SUMS.txt')):
        raw=(RAW/name).read_bytes();require(sha(raw)==inventory[name],'copy input changed')
        if name.endswith('.json'):
            view=redact(decode(raw));require(not {'publication_notice','raw_source_sha256'}&set(view),'reserved view keys')
            view.update(publication_notice=NOTICE,raw_source_sha256=inventory[name]);payload=encoded(view);kind='metadata_view'
        else:payload=redact(raw.decode()).encode();kind='original_bytes'if payload==raw else'redacted_source_reference'
        new_bytes(OUTPUT/target,payload);entries[target]=dict(raw_private_source=str((RAW/name).relative_to(ROOT)),raw_source_sha256=inventory[name],public_sha256=digest(OUTPUT/target),kind=kind)
    new_bytes(OUTPUT/'camera_sanity.json',encoded(dict(schema='nuplan.untrusted_camera_literal_table.v1',raw_report_sha256=PINS['report.json'],rows=table,**{k:False for k in DENIALS})))
    new_bytes(OUTPUT/'README.md',readme(plan,report,table).encode())
    new_bytes(OUTPUT/'publication_source.py',Path(__file__).read_bytes())
    for name in ('camera_sanity.json','README.md','publication_source.py'):entries[name]=dict(public_sha256=digest(OUTPUT/name),kind='generated_publication_evidence')
    require(verify_files()==inventory and digest(Path(__file__).resolve())==self_sha,'source or private originals changed')
    provenance=dict(schema='nuplan.calibration_literal_publication.v1',private_input_sha256=inventory,public_files=entries,publisher_sha256=self_sha,
        executed_diagnostic_sha256=PINS['source_executed.py'],original_plan_sha256=PINS['plan.json'],official_references=plan['official_source_pins'],
        third_party_reference_code_copied=False,raw_blobs_copied=False,database_or_image_reads=False,model_inference=False,
        metadata_notice=NOTICE,**{k:False for k in DENIALS})
    new_bytes(OUTPUT/'publication_manifest.json',encoded(provenance))
    files=sorted(p for p in OUTPUT.rglob('*')if p.is_file())
    new_bytes(OUTPUT/'SHA256SUMS',''.join(digest(p)+'  '+p.relative_to(OUTPUT).as_posix()+'\n'for p in files).encode())
    privacy_links()
    for line in (OUTPUT/'SHA256SUMS').read_text().splitlines():value,name=line.split('  ');require(digest(OUTPUT/name)==value,'public checksum failed')
    print(json.dumps(dict(status='PUBLISHED_NOT_TRUSTED',files=len(files)+1,cameras=8,fields=32,publication_manifest_sha256=digest(OUTPUT/'publication_manifest.json'))))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);parser.add_argument('--verify-only',action='store_true');args=parser.parse_args()
    if args.verify_only:
        pins=verify_files();rows=validate_report(decode((RAW/'plan.json').read_bytes()),decode((RAW/'report.json').read_bytes()))
        print(json.dumps(dict(status='VERIFIED_NO_WRITES',private_files=len(pins),cameras=len(rows))))
    else:publish()
