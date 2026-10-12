"""HH_260906 - Launch only the reviewed six-head residual study in personal GPU0 diagnostics."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

ROOT = Path('${USER_HOME}/autoware_e2e')
LOCAL = ROOT / 'artifacts/training/2026-09-09'
W = '${USER_HOME}/personal/hwanhong/portable_e2e'
DEST = W + '/benchmarks/frozen_stopmix_residual_v1'
OUT = W + '/runs/diagnostics/frozen_stopmix_residual_v1'
FILES = {
    ROOT / 'scripts/e2e/run_frozen_stopmix_residual.py': 'c1572d06d8718ce9c22f9899ef0c2a2e02d35652d16d95164959fd41bc66e9e6',
    ROOT / 'portable_e2e/frozen_stopmix_residual.py': 'd4bf42fc9f4db930f31ef7b8ccd79900249687642f47bf525dedb240207550b7',
    ROOT / 'scripts/e2e/run_frozen_stopmix_selector.py': '469bf474d63dbaa0c7556d77af981e12eee42df8f63318fc9f522a0b38e4bb8a',
    ROOT / 'portable_e2e/frozen_stopmix_selector.py': '4716c22d29b8e306b86f8835bf2a5ec2bf956bbb6859c6b6f001a261865b8de3',
    ROOT / 'scripts/e2e/diagnose_stopmix_training_oracles.py': '35d3d0a464fb43c9789f6e24a35e453736c99fdceb36e3292d085c5ceb960d37',
    LOCAL / 'frozen_stopmix_residual_plan_v1.json': '5bdb3e49d39db3788c6db4ed1b717510d7f53abd0532f3ca66fb01ee006d98b9',
}
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', 'pro6000-training']


def now():
    return datetime.now(timezone.utc).isoformat()


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def main():
    assert all(path.is_file() and not any(p.is_symlink() for p in (path, *path.parents))
        and hashlib.sha256(path.read_bytes()).hexdigest() == pin for path, pin in FILES.items())
    assert not (LOCAL / 'frozen_stopmix_residual_v1_launch.json').exists()
    assert (datetime(2026, 9, 9, 1, tzinfo=timezone.utc) - datetime.now(timezone.utc)).total_seconds() > 2280
    preflight = '''import json,pathlib,subprocess
w=pathlib.Path(%r); r=w/'autoware_e2e'; dest=pathlib.Path(%r); out=pathlib.Path(%r)
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=r,text=True).strip()=='b478f02e42b94bf04bffec5c8170e05edc33b0f8'
assert not subprocess.check_output(['git','status','--porcelain','--untracked-files=all'],cwd=r,text=True).strip()
assert not out.exists() and not any(p.is_symlink() for p in (dest,*dest.parents,out,*out.parents))
dest.mkdir(exist_ok=False)
print(json.dumps({'source_commit':'b478f02e42b94bf04bffec5c8170e05edc33b0f8','destination_created':str(dest),'repository_changed':False}))
''' % (W, DEST, OUT)
    pre = subprocess.check_output(SSH + [shlex.join([W + '/venvs/py312/bin/python', '-c', preflight])], text=True, timeout=30)
    subprocess.run(['scp', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
        *[str(path) for path in FILES], 'pro6000-training:' + DEST + '/'], check=True, timeout=60)
    pins = {path.name: value for path, value in FILES.items()}
    bootstrap = '''import hashlib,pathlib,runpy,sys
dest=pathlib.Path(sys.argv[1]); pins=%r
for name,pin in pins.items():
    path=dest/name
    assert path.is_file() and not any(p.is_symlink() for p in (path,*path.parents))
    assert hashlib.sha256(path.read_bytes()).hexdigest()==pin
worker=dest/'run_frozen_stopmix_residual.py'
sys.path[0]=str(pathlib.Path.cwd())
sys.argv=[str(worker),'--plan',str(dest/'frozen_stopmix_residual_plan_v1.json'),'--core',str(dest/'frozen_stopmix_residual.py'),'--frozen-worker',str(dest/'run_frozen_stopmix_selector.py'),'--frozen-core',str(dest/'frozen_stopmix_selector.py'),'--oracle-helper',str(dest/'diagnose_stopmix_training_oracles.py'),'--output-dir',sys.argv[2]]
runpy.run_path(str(worker),run_name='__main__')
''' % pins
    args = ['env', '-u', 'PYTHONPATH', '-u', 'PYTHONHOME', '-u', 'PYTHONSTARTUP', '-u', 'PYTHONUSERBASE',
        'CUDA_VISIBLE_DEVICES=${GPU0_UUID}', 'CUBLAS_WORKSPACE_CONFIG=:4096:8',
        'PYTHONNOUSERSITE=1', 'PYTHONDONTWRITEBYTECODE=1', 'PYTHONUNBUFFERED=1', 'OMP_NUM_THREADS=16', 'MKL_NUM_THREADS=16',
        'timeout', '--signal=TERM', '--kill-after=5s', '2100s', W + '/venvs/py312/bin/python', '-c', bootstrap, DEST, OUT]
    command = 'cd ' + shlex.quote(W + '/autoware_e2e') + ' && ' + shlex.join(args)
    receipt = dict(started_at_utc=now(), transferred_file_sha256=pins, remote_output=OUT,
        command=command, preflight=json.loads(pre), launcher_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        comment='HH_260906 - Six matched residual scorer-only fits reuse authenticated original caches; no generator forward, new admission, model promotion or vehicle control.')
    save(LOCAL / 'frozen_stopmix_residual_v1_launch.json', receipt)
    print(json.dumps({'started_at_utc': receipt['started_at_utc'], 'remote_output': OUT}), flush=True)
    error, exit_code = None, None
    try:
        with (LOCAL / 'frozen_stopmix_residual_v1_console.log').open('x') as log:
            result = subprocess.run(SSH + [command], stdout=log, stderr=subprocess.STDOUT, timeout=2130)
            exit_code = result.returncode
    except BaseException as caught:
        error = type(caught).__name__
        raise
    finally:
        completion = dict(completed_at_utc=now(), exit_code=exit_code, error_type=error, transferred_file_sha256=pins,
            local_source_bytes_unchanged=all(hashlib.sha256(path.read_bytes()).hexdigest() == pin for path, pin in FILES.items()),
            console_sha256=hashlib.sha256((LOCAL / 'frozen_stopmix_residual_v1_console.log').read_bytes()).hexdigest())
        save(LOCAL / 'frozen_stopmix_residual_v1_ssh_completion.json', completion)
        print(json.dumps(completion), flush=True)


if __name__ == '__main__':
    main()
