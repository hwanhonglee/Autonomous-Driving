"""HH_260906 - Run one additive mapping TRAIN experiment inside the personal GPU0 environment."""
import json
import argparse
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone


def main():
    # HH_260906 - Each additive data revision has its own frozen source, dataset, parent and immutable receipts.
    parser=argparse.ArgumentParser()
    parser.add_argument('--revision',choices=('v1','v2'),default='v1')
    revision=parser.parse_args().revision
    workspace=Path('/home/pro6000/personal/hwanhong/portable_e2e')
    source=workspace/f'source_worktrees/map_expanded_20260926_ai_{revision}'
    frozen=workspace/'source_worktrees/moving_recovery_k_20260922_v1'
    runs=workspace/'runs/diagnostics'
    output=runs/f'map_expanded_20260926_ai_{revision}'
    logpath=runs/f'mapping_20260926_ai_{revision}.launch.log'
    receipt=runs/f'mapping_20260926_ai_{revision}.exit.json'
    if Path(sys.prefix)!=workspace/'venvs/py312' or any(p.exists() for p in (output,logpath,receipt)):
        raise ValueError('personal venv and fresh output/log/receipt required')
    env=dict(os.environ)
    for key in ('PYTHONPATH','PYTHONHOME','PYTHONSTARTUP','PYTHONUSERBASE'):
        env.pop(key,None)
    env.update(CUDA_VISIBLE_DEVICES='GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5',
        CUBLAS_WORKSPACE_CONFIG=':4096:8',PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',
        OMP_NUM_THREADS='16',MKL_NUM_THREADS='16')
    command=[sys.executable,str(source/'train_map_native_ah.py'),'--frozen-repo',str(frozen),
        '--data',str(workspace.parent/f'dataset/map_expanded_20260926_ai_{revision}'),
        '--parent',str(runs/('map_native_20260926_ah_v1' if revision=='v1' else 'map_expanded_20260926_ai_v1')),
        '--output',str(output),'--expanded-train']
    # HH_260906 - CPU validation must pass before acquiring the shared GPU0 lease and optimizing.
    subprocess.run(command+['--validate-only'],cwd=frozen,env=env,check=True)
    with logpath.open('x') as log:
        result=subprocess.run(command,cwd=frozen,env=env,stdout=log,stderr=subprocess.STDOUT)
    with receipt.open('x') as stream:
        json.dump(dict(returncode=result.returncode,completed_at_utc=datetime.now(timezone.utc).isoformat(),
            command=command,log=str(logpath),automatic_retry=False,automatic_promotion=False),stream,indent=2)
    raise SystemExit(result.returncode)


if __name__=='__main__':main()
