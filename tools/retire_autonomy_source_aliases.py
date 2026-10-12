"""HH_260906 - Rebase explicit source paths and retire five aliases with rollback checks."""
import argparse
import fcntl
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
TARGETS={name:'autonomy/'+name for name in ('src','autoware_e2e_vad_launch','cmake','include','patches')}
SKIP={'migrate_workspace_layout.py','rebase_layout_references.py','retire_low_risk_aliases.py',Path(__file__).name}


def rewrite_text(text):
    # HH_260906 - Only explicit workspace-root expressions are mechanical; manifest bytes stay frozen.
    names='|'.join(TARGETS)
    # HH_260906 - Build the root-name alternation separately to avoid quoting ambiguity.
    pattern=r"(\b(?:ROOT|REPO_ROOT|PROJECT_ROOT|WORKSPACE|REPOSITORY_ROOT|workspace)\s*/\s*(?:\(\s*)?)(['\"])("+names+r")(/[^'\"]*)?\2"
    text=re.sub(pattern,lambda m:m[1]+m[2]+TARGETS[m[3]]+(m[4] or '')+m[2],text)
    for old,new in TARGETS.items():
        for var in ('root','repo_root','AUTOWARE_ROOT','AUTOWARE_E2E_ROOT','_autoware_e2e_root'):
            for prefix in ('${'+var+'}/','$'+var+'/'):
                text=text.replace(prefix+old+'/',prefix+new+'/')
        text=text.replace('/home/a/autoware_e2e/'+old+'/', '/home/a/autoware_e2e/'+new+'/')
    text=text.replace('--base-paths src autoware_e2e_vad_launch',
                      '--base-paths autonomy/src autonomy/autoware_e2e_vad_launch')
    text=text.replace('--base-paths autoware_e2e_vad_launch', '--base-paths autonomy/autoware_e2e_vad_launch')
    return text


def rewrite(archive):
    archive.mkdir(parents=True,exist_ok=False)
    changes=[]
    for folder in ('tools','learning/tests','autonomy/autoware_e2e_vad_launch'):
        for path in (ROOT/folder).rglob('*'):
            if not path.is_file() or path.is_symlink() or path.suffix not in ('.py','.sh') or path.name in SKIP:
                continue
            old=path.read_text();new=rewrite_text(old)
            if new!=old:
                relative=path.relative_to(ROOT);backup=archive/'files'/relative
                backup.parent.mkdir(parents=True,exist_ok=True);backup.write_bytes(path.read_bytes())
                path.write_text(new);changes.append(str(relative))
    links=[]
    for folder in ('autonomy/build','autonomy/install','autonomy/layout_check','autonomy/canonical_rebuild'):
        for directory,dirs,files in os.walk(ROOT/folder,followlinks=False):
            for name in dirs+files:
                path=Path(directory)/name
                if not path.is_symlink():continue
                before=os.readlink(path)
                lexical=Path(os.path.abspath(path.parent/before))
                try:relative=lexical.relative_to(ROOT)
                except ValueError:continue
                if not relative.parts or relative.parts[0] not in TARGETS:continue
                target=ROOT/TARGETS[relative.parts[0]]/Path(*relative.parts[1:])
                if path.resolve()!=target.resolve():raise ValueError('link destination changed: '+str(path))
                after=os.path.relpath(target,path.parent)
                links.append(dict(path=str(path.relative_to(ROOT)),before=before,after=after))
    (archive/'changes.json').write_text(json.dumps(dict(files=changes,symlinks=links),indent=2))
    for item in links:
        path=ROOT/item['path'];path.unlink();path.symlink_to(item['after'])
    print(json.dumps(dict(rewritten_files=len(changes),rebased_symlinks=len(links))),flush=True)


def retire(root,checks,record):
    # HH_260906 - Validate exact aliases before removal; never unlink a real directory or its contents.
    if record.exists() or record.is_symlink():
        raise ValueError('fresh retirement record required')
    for old,new in TARGETS.items():
        path=root/old
        if not path.is_symlink() or os.readlink(path)!=new or not (root/new).is_dir() or (root/new).is_symlink():
            raise ValueError('unexpected alias: '+old)
    removed=[]
    try:
        for old in TARGETS:
            (root/old).unlink();removed.append(old)
        checks()
        record.write_text(json.dumps(dict(removed=TARGETS,data_files_deleted=0,
            verified_at_utc=datetime.now(timezone.utc).isoformat()),indent=2))
    except BaseException:
        for old in removed:
            if not os.path.lexists(root/old):(root/old).symlink_to(TARGETS[old],target_is_directory=True)
        raise


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--rewrite',action='store_true')
    parser.add_argument('--remove',action='store_true');a=parser.parse_args()
    if a.rewrite:rewrite(ROOT/'.workspace_archive/retire_autonomy_sources_20260922')
    if a.remove:
        lock=(ROOT/'storage/assets/locks/autoware_e2e_runtime.lock').open('a')
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        def checks():
            env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',CUDA_VISIBLE_DEVICES='',
                     AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD=str(lock.fileno()))
            tests=[str(p.relative_to(ROOT)) for p in (ROOT/'learning/tests').glob('test_*.py')
                   if any(s in p.name for s in ('patch','source_timestamp','camera_qos','runtime_timing','compat',
                       'build_launch','workspace','autoware_vad_town_matrix','validate_60kph','retire_autonomy'))]
            subprocess.run(['python3','-m','pytest','-q','-p','no:cacheprovider',*tests],cwd=ROOT,env=env,
                           check=True,pass_fds=(lock.fileno(),))
            subprocess.run(['env','-i','HOME=/home/a','USER=a','PATH=/usr/local/bin:/usr/bin:/bin',
                'LANG=C.UTF-8',f'AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD={lock.fileno()}',
                'bash','tools/e2e/check_canonical_runtime_load.sh'],cwd=ROOT,check=True,pass_fds=(lock.fileno(),))
        retire(ROOT,checks,ROOT/'docs/workflows/05_maintenance/autonomy_source_retirement_20260922.json')
        print('FIVE_SOURCE_ALIASES_REMOVED_TARGETS_PRESERVED',flush=True)


if __name__=='__main__':main()
