"""HH_260906 - Remove explicitly verified aliases only, restoring them if the canonical checks fail."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
TARGETS={'config':'learning/config','tests':'learning/tests','workflows':'docs/workflows'}


def rewrite():
    archive=ROOT/'.workspace_archive/retire_navigation_20260922'
    archive.mkdir(exist_ok=False)
    changes=[]
    for folder in ('tools','learning/tests','autonomy/autoware_e2e_vad_launch/test'):
        for path in (ROOT/folder).rglob('*.py'):
            if path.name in ('migrate_workspace_layout.py','rebase_layout_references.py',Path(__file__).name):
                continue
            old=path.read_text();text=old
            # HH_260906 - Restrict replacement to explicit repository-root Path expressions.
            pattern=r"(\b(?:ROOT|REPO_ROOT|PROJECT_ROOT|WORKSPACE|REPOSITORY_ROOT)\s*/\s*)(['\"])(config|tests|workflows)(/[^'\"]*)?\2"
            def replace(match):
                return match[1]+match[2]+TARGETS[match[3]]+(match[4] or '')+match[2]
            text=re.sub(pattern,replace,text)
            if text!=old:
                relative=path.relative_to(ROOT);backup=archive/relative
                backup.parent.mkdir(parents=True,exist_ok=True);backup.write_bytes(path.read_bytes())
                path.write_text(text);changes.append(str(relative))
    (archive/'rewritten_files.json').write_text(json.dumps(changes,indent=2))
    print('PATHS_UPDATED',len(changes))


def main():
    p=argparse.ArgumentParser();p.add_argument('--rewrite',action='store_true')
    p.add_argument('--remove',action='store_true');a=p.parse_args()
    if a.rewrite:rewrite()
    if a.remove:
        # HH_260906 - Validate every target before unlinking any of the three aliases.
        for old,new in TARGETS.items():
            alias=ROOT/old
            if not alias.is_symlink() or os.readlink(alias)!=new:
                raise ValueError('unexpected alias: '+old)
            if not (ROOT/new).is_dir():raise ValueError('missing target')
        removed=[]
        try:
            for old,new in TARGETS.items():
                (ROOT/old).unlink();removed.append(old)
            cmd=['python3','-m','pytest','-q','-p','no:cacheprovider',
                'learning/tests/test_workspace_navigation.py',
                'learning/tests/test_run_owned_carla_moving_dev_v2.py',
                'learning/tests/test_autoware_vad_town_matrix.py',
                'learning/tests/test_run_portable_training_campaign.py',
                'learning/tests/test_summarize_portable_stopmix.py',
                'learning/tests/test_summarize_portable_data_expansion.py',
                'learning/tests/test_summarize_portable_selector_weight.py',
                'learning/tests/test_summarize_portable_training_campaign.py',
                'learning/tests/test_summarize_portable_no_accel_development.py',
                'learning/tests/test_summarize_moving_turn_replay_recovery.py',
                'learning/tests/test_verify_stopmix_duration_results.py']
            subprocess.run(cmd,cwd=ROOT,check=True,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',CUDA_VISIBLE_DEVICES=''))
        except BaseException:
            for old in removed:
                if not (ROOT/old).exists():(ROOT/old).symlink_to(TARGETS[old],target_is_directory=True)
            raise
        print('ALIASES_REMOVED',','.join(removed),'TARGET_DATA_UNCHANGED')


if __name__=='__main__':main()
