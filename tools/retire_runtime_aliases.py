"""HH_260906 - Retire three runtime aliases under a shared lock, restoring them on any check failure."""
import fcntl
import importlib.util
import os
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def main():
    spec=importlib.util.spec_from_file_location('source_retirement',ROOT/'tools/retire_autonomy_source_aliases.py')
    previous=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous)
    previous.TARGETS={name:'autonomy/'+name for name in ('build','install','log')}
    record=ROOT/'docs/workflows/05_maintenance/runtime_alias_retirement_20260922.json'
    with (ROOT/'storage/assets/locks/autoware_e2e_runtime.lock').open('a') as lock:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',CUDA_VISIBLE_DEVICES='',
                 AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD=str(lock.fileno()),PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
        tests=[str(p.relative_to(ROOT)) for p in (ROOT/'learning/tests').glob('test_*.py')
               if any(s in p.name for s in ('patch','source_timestamp','camera_qos','runtime_timing','compat',
                   'build_launch','workspace','autoware_vad_town_matrix','validate_60kph','retire_autonomy','bootstrap'))]
        def checks():
            subprocess.run(['bash','-c','source tools/e2e/env.sh; exec python3 -m pytest -q -p no:cacheprovider "$@"','runtime-tests',*tests],
                           cwd=ROOT,env=env,check=True,pass_fds=(lock.fileno(),))
            subprocess.run(['bash','tools/e2e/check_active_runtime_layout.sh'],cwd=ROOT,env=env,check=True,pass_fds=(lock.fileno(),))
            subprocess.run(['bash','-c','source tools/e2e/env.sh; exec python3 -m pytest -q -p no:cacheprovider autonomy/autoware_e2e_vad_launch/test'],
                           cwd=ROOT,env=env,check=True,pass_fds=(lock.fileno(),))
        previous.retire(ROOT,checks,record)
    print('THREE_RUNTIME_ALIASES_REMOVED_TARGETS_PRESERVED',flush=True)


if __name__=='__main__':main()
