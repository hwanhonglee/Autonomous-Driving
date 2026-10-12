#!/usr/bin/env python3
"""HH_260906 - Retire only the data alias after a full no-regression and runtime check."""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT/'storage/experiments/validation/2026-09-23/legacy_retirement/remaining_failures.json'
PROTECTED = ROOT/'storage/experiments/validation/2026-09-23/legacy_retirement/protected_sources.json'


def evaluate_suite(text, returncode, baseline, *, minimum_passed=8300):
    # HH_260906 - Retain pre-existing failures explicitly; never call an incomplete suite green.
    failed = {line[7:].split(' - ', 1)[0] for line in text.splitlines() if line.startswith('FAILED ')}
    errors = [line for line in text.splitlines() if line.startswith('ERROR ')]
    passed_matches = re.findall(r'\b(\d+) passed\b', text)
    skipped_matches = re.findall(r'\b(\d+) skipped\b', text)
    passed = int(passed_matches[-1]) if passed_matches else 0
    skipped = int(skipped_matches[-1]) if skipped_matches else 0
    if (returncode not in (0, 1) or errors or passed < minimum_passed or skipped > 8
            or failed - set(baseline) or (returncode == 1 and not failed)
            or (returncode == 0 and failed)):
        raise RuntimeError('research suite regressed or did not complete; see pytest.log')
    return dict(status='NO_NEW_FAILURES' if failed else 'PASS', passed=passed,
                preexisting_failures=sorted(failed), skipped=skipped, errors=0)


def snapshot():
    result = {}
    for directory in ('tools', 'learning', 'autonomy/autoware_e2e_vad_launch'):
        for path in (ROOT/directory).rglob('*'):
            if path.is_file() and not path.is_symlink() and path.suffix in ('.py', '.sh', '.yaml', '.xml', '.json'):
                result[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, required=True)
    args = parser.parse_args()
    record = ROOT/'docs/workflows/05_maintenance/asset_alias_retirement_20260924.json'
    alias = ROOT/'data'
    target = ROOT/'storage/assets'
    if record.exists() or record.is_symlink():
        raise ValueError('fresh retirement record required')
    with (target/'locks/autoware_e2e_runtime.lock').open('a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        if not alias.is_symlink() or os.readlink(alias) != 'storage/assets' or target.is_symlink():
            raise ValueError('expected exact data alias and a real asset directory')
        args.log_dir.mkdir(parents=True, exist_ok=False)
        before = snapshot()
        protected = json.loads(PROTECTED.read_text())['expected_before']
        for name, checksum in protected.items():
            if before['tools/e2e/'+name] != checksum:
                raise ValueError('protected source changed: '+name)
        baseline_bytes = BASELINE.read_bytes()
        baseline = json.loads(baseline_bytes)['failed_tests']
        inode = target.stat().st_ino
        env = dict(os.environ, CUDA_VISIBLE_DEVICES='', PYTHONDONTWRITEBYTECODE='1',
                   PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
                   AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD=str(lock.fileno()))
        def run(name, command, *, check=True):
            print('CHECK', name, flush=True)
            with (args.log_dir/name).open('w') as stream:
                result = subprocess.run(['bash', '-c', command], cwd=ROOT, env=env,
                    stdout=stream, stderr=subprocess.STDOUT, pass_fds=(lock.fileno(),), check=False)
            if check and result.returncode:
                raise RuntimeError('failed check: '+name)
            return result.returncode
        alias.unlink()
        try:
            code = run('pytest.log', 'source tools/e2e/env.sh && python3 tools/workspace_python.py '
                       '-m pytest -- -q -p no:cacheprovider learning/tests', check=False)
            suite = evaluate_suite((args.log_dir/'pytest.log').read_text(), code, baseline)
            run('launch.log', 'source tools/e2e/env.sh && python3 -m pytest -q -p no:cacheprovider '
                'autonomy/autoware_e2e_vad_launch/test')
            run('runtime.log', 'bash tools/e2e/check_active_runtime_layout.sh')
            run('document_links.log', 'python3 tools/check_workspace_links.py')
            # HH_260906 - Re-read real map payloads; a zero-exit planner with every map blocked is not success.
            map_output = args.log_dir/'map_admission'
            run('map_admission.log', 'source tools/e2e/env.sh && python3 tools/workspace_python.py '
                'tools/e2e/autoware_vad_town_matrix.py prepare --output-root '+shlex.quote(str(map_output)))
            plan = json.loads((map_output/'matrix_plan.json').read_text())
            expected = {'town01', 'town02_opt', 'town03', 'town04', 'town05_opt', 'town06',
                        'town07', 'town10hd_opt', 'c_track_1_0_7'}
            if {item['map_id'] for item in plan['maps'] if item['runnable']} != expected:
                raise RuntimeError('real map admission changed; see map_admission.log')
            if before != snapshot() or BASELINE.read_bytes() != baseline_bytes:
                raise RuntimeError('source/configuration or baseline changed during verification')
            if target.stat().st_ino != inode or os.path.lexists(alias):
                raise RuntimeError('asset target identity or removed alias changed')
            record.write_text(json.dumps(dict(removed={'data':'storage/assets'}, data_files_deleted=0,
                source_pins_relaxed=False, target_inode=inode, research_suite=suite,
                remaining_aliases=[name for name in ('scripts','portable_e2e','artifacts','datasets') if (ROOT/name).is_symlink()],
                verified_at_utc=datetime.now(timezone.utc).isoformat(), log_dir=str(args.log_dir),
                source_snapshot_sha256=hashlib.sha256(json.dumps(before,sort_keys=True).encode()).hexdigest(),
                baseline_sha256=hashlib.sha256(baseline_bytes).hexdigest()), indent=2)+'\n')
            (args.log_dir/'source_snapshot.json').write_text(json.dumps(before,indent=2)+'\n')
        except BaseException:
            if not os.path.lexists(alias):
                alias.symlink_to('storage/assets', target_is_directory=True)
            raise
        print('DATA_ALIAS_REMOVED_ORIGINAL_ASSETS_PRESERVED', flush=True)


if __name__ == '__main__':
    main()
