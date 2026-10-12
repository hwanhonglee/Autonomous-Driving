"""HH_260906 - Archive and verify a fixed cache allowlist before recoverable cleanup."""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

ROOT=Path(__file__).resolve().parents[1]
TARGETS=('.pytest_cache','portable_e2e/__pycache__','tests/__pycache__','scripts/e2e/__pycache__')
EMPTY='artifacts/validation/2026-09-22/map_perception_pilot_v1_data'


def inspect(root=ROOT):
    files={};present=[]
    for relative in TARGETS:
        target=root/relative
        if not target.exists():continue
        if target.is_symlink() or target.resolve()!=target.absolute():raise ValueError('symlink target refused')
        if not target.is_dir():raise ValueError('cache target is not a directory')
        tracked=subprocess.check_output(['git','ls-files','--',relative],cwd=root,text=True)
        if tracked.strip():raise ValueError('tracked cache files refused')
        present.append(relative)
        for p in target.rglob('*'):
            if p.is_symlink():raise ValueError('symlink inside cache refused')
            if p.is_file():files[p.relative_to(root).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
    return present,files


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--apply',action='store_true');a=parser.parse_args()
    targets,files=inspect()
    report=dict(targets=targets,file_count=len(files),bytes=sum((ROOT/f).stat().st_size for f in files),
                preserved=['training data','source code','checkpoints','raw experiments','build/install/log','remote','system tmp'])
    if not a.apply:print(json.dumps(report,indent=2));return
    record=ROOT/'docs/workflows/05_maintenance/cleanup_record.json'
    if record.exists():raise ValueError('existing cleanup record; do not overwrite history')
    lock_path=ROOT/'storage/assets/locks/autoware_e2e_runtime.lock'
    with lock_path.open('a') as lock:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        targets,files=inspect()
        archive_dir=ROOT/'.workspace_archive'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        archive_dir.mkdir(parents=True,exist_ok=False)
        archive=archive_dir/'generated_caches.zip'
        with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as z:
            for name in files:z.write(ROOT/name,name)
        with zipfile.ZipFile(archive) as z:
            if z.testzip() is not None:raise ValueError('archive CRC failure')
            for name,digest in files.items():
                if hashlib.sha256(z.read(name)).hexdigest()!=digest:raise ValueError('archive hash mismatch')
        if inspect()!=(targets,files):raise ValueError('cache contents changed; cleanup aborted')
        report.update(archive=str(archive.relative_to(ROOT)),archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                      archive_bytes=archive.stat().st_size,source_hashes=files,removed=[],removed_empty=[])
        # HH_260906 - Persist recovery evidence before removing only these verified exact directories.
        record.write_text(json.dumps(report,indent=2))
        for relative in targets:
            if relative not in TARGETS:raise ValueError('outside cleanup allowlist')
            shutil.rmtree(ROOT/relative)
            report['removed'].append(relative)
            record.write_text(json.dumps(report,indent=2))
        empty=ROOT/EMPTY
        if empty.is_dir() and not empty.is_symlink() and not any(empty.iterdir()):
            empty.rmdir();report['removed_empty'].append(EMPTY)
        report['completed_at_utc']=datetime.now(timezone.utc).isoformat()
        record.write_text(json.dumps(report,indent=2))
        text=['# 작업공간 정리 완료','',
            '<!-- HH_260906 - Report reversible cache cleanup without claiming old evidence is disposable. -->','',
            f"- 원래 캐시 {report['bytes']/1024**2:.2f} MiB, 복구 ZIP {report['archive_bytes']/1024**2:.2f} MiB.",
            f"- 복구 ZIP: `{report['archive']}`",'- 원본 해시·삭제 범위: [cleanup_record.json](cleanup_record.json)',
            '- 학습·소스·원시 실험·build/install/log·원격·시스템 /tmp는 변경하지 않음.',
            '- 캐시는 Python/pytest 실행 시 다시 생성될 수 있음.','', '## 제거한 캐시','']
        text += [f'- `{p}`' for p in report['removed']]
        text += ['', '## 제거한 빈 실패 폴더','']+[f'- `{p}` (비어 있었음)' for p in report['removed_empty']]
        (record.parent/'cleanup_summary.md').write_text('\n'.join(text)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='source_hashes'},indent=2))


if __name__=='__main__':main()
