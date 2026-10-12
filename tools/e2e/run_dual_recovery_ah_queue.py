"""HH_260906 - Personal GPU0 sequential queue; preserve failures and never retry or promote."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from datetime import datetime,timezone


def main():
    workspace=Path('/home/pro6000/personal/hwanhong/portable_e2e')
    source=workspace/'source_worktrees/dual_recovery_20260926_ah_v1'
    frozen=workspace/'source_worktrees/moving_recovery_k_20260922_v1'
    runs=workspace/'runs/diagnostics'
    receipt=runs/'dual_recovery_20260926_ah_v1.queue.json'
    if Path(sys.prefix)!=workspace/'venvs/py312' or receipt.exists():
        raise ValueError('personal venv and fresh queue receipt required')
    env=dict(os.environ)
    for name in ('PYTHONPATH','PYTHONHOME','PYTHONSTARTUP','PYTHONUSERBASE'):
        env.pop(name,None)
    env.update(CUDA_VISIBLE_DEVICES='GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5',
        CUBLAS_WORKSPACE_CONFIG=':4096:8',PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',
        OMP_NUM_THREADS='16',MKL_NUM_THREADS='16')
    commands={
        'mapping':[sys.executable,str(source/'train_map_native_ah.py'),'--frozen-repo',str(frozen),
            '--data',str(workspace.parent/'dataset/map_native_20260926_ah_v1'),
            '--parent',str(runs/'map_perception_pilot_20260922_v2'),
            '--output',str(runs/'map_native_20260926_ah_v1')],
        'driving':[sys.executable,str(source/'run_driving_near_horizon_ah.py'),
            '--baseline',str(runs/'moving_recovery_k_20260922_v1'),
            '--output',str(runs/'driving_near_20260926_ah_v1')]}
    # HH_260906 - Validate both datasets/source seals on CPU before either optimization starts.
    for role,command in commands.items():
        subprocess.run(command+['--validate-only'],cwd=frozen,env=env,check=True)
    state=dict(started_at_utc=datetime.now(timezone.utc).isoformat(),pid=os.getpid(),
        physical_gpu=0,automatic_retry=False,automatic_promotion=False,commands=commands,jobs={})
    with receipt.open('x') as f:json.dump(state,f,indent=2)
    for role,command in commands.items():
        logpath=runs/f'{role}_20260926_ah_v1.launch.log'
        with logpath.open('x') as log:
            result=subprocess.run(command,cwd=frozen,env=env,stdout=log,stderr=subprocess.STDOUT)
        # HH_260906 - Each owned job has a separate immutable exit receipt, including failures.
        result=dict(returncode=result.returncode,completed_at_utc=datetime.now(timezone.utc).isoformat(),log=str(logpath))
        with (runs/f'{role}_20260926_ah_v1.exit.json').open('x') as f:json.dump(result,f,indent=2)
        state['jobs'][role]=result
        print(role+' EXIT '+str(result['returncode']),flush=True)
        time.sleep(5)
    with (runs/'dual_recovery_20260926_ah_v1.complete.json').open('x') as f:json.dump(state,f,indent=2)


if __name__=='__main__':main()
