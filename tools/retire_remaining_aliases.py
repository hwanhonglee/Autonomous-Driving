#!/usr/bin/env python3
"""HH_260906 - Remove the four exact root aliases only after canonical no-regression verification."""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
ALIASES = {'scripts':'tools', 'portable_e2e':'learning/portable_e2e',
           'datasets':'storage/datasets', 'artifacts':'storage/experiments'}
spec = importlib.util.spec_from_file_location('hh_retirement_checks', ROOT/'tools/retire_asset_alias.py')
checks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checks)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, required=True)
    args = parser.parse_args()
    record = ROOT/'docs/workflows/05_maintenance/final_alias_retirement_20260925.json'
    if os.path.lexists(record):
        raise ValueError('refusing to replace an existing retirement record')
    with (ROOT/'storage/assets/locks/autoware_e2e_runtime.lock').open('a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        targets = {}
        for name, destination in ALIASES.items():
            alias, target = ROOT/name, ROOT/destination
            if not alias.is_symlink() or os.readlink(alias) != destination:
                raise ValueError('unexpected alias: '+name)
            if target.is_symlink() or not target.is_dir():
                raise ValueError('invalid canonical directory: '+destination)
            targets[name] = {'device':target.stat().st_dev, 'inode':target.stat().st_ino}
        args.log_dir.mkdir(parents=True, exist_ok=False)
        before = checks.snapshot()
        protected = json.loads(checks.PROTECTED.read_text())['expected_before']
        for name, digest in protected.items():
            if before['tools/e2e/'+name] != digest:
                raise ValueError('protected source changed: '+name)
        baseline_bytes = checks.BASELINE.read_bytes()
        baseline = json.loads(baseline_bytes)['failed_tests']
        env = dict(os.environ, CUDA_VISIBLE_DEVICES='', PYTHONDONTWRITEBYTECODE='1',
                   PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
                   AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD=str(lock.fileno()))

        def run(name, command, check=True):
            print('CHECK', name, flush=True)
            with (args.log_dir/name).open('x') as output:
                result = subprocess.run(['bash', '-c', command], cwd=ROOT, env=env,
                    stdout=output, stderr=subprocess.STDOUT, pass_fds=(lock.fileno(),))
            if check and result.returncode:
                raise RuntimeError('failed check: '+name)
            return result.returncode

        removed = []
        try:
            for name in ALIASES:
                (ROOT/name).unlink()
                removed.append(name)
            code = run('pytest.log', 'source tools/e2e/env.sh && python3 tools/workspace_python.py '
                       '-m pytest -- -ra -q -p no:cacheprovider learning/tests', check=False)
            suite = checks.evaluate_suite((args.log_dir/'pytest.log').read_text(), code, baseline)
            run('launch.log', 'source tools/e2e/env.sh && python3 tools/workspace_python.py -m pytest -- -q -p no:cacheprovider '
                'autonomy/autoware_e2e_vad_launch/test')
            run('runtime.log', 'bash tools/e2e/check_active_runtime_layout.sh')
            run('document_links.log', 'python3 tools/check_workspace_links.py')
            map_output = args.log_dir/'map_admission'
            run('map_admission.log', 'source tools/e2e/env.sh && python3 tools/workspace_python.py '
                'tools/e2e/autoware_vad_town_matrix.py prepare --output-root '+shlex.quote(str(map_output)))
            plan = json.loads((map_output/'matrix_plan.json').read_text())
            runnable = {item['map_id'] for item in plan['maps'] if item['runnable']}
            expected = {'town01','town02_opt','town03','town04','town05_opt','town06',
                        'town07','town10hd_opt','c_track_1_0_7'}
            if runnable != expected:
                raise RuntimeError('real map admission changed')
            run('layout.json', 'python3 tools/audit_legacy_layout.py')
            layout = json.loads((args.log_dir/'layout.json').read_text())
            if not layout['legacy_removal_complete']:
                raise RuntimeError('legacy paths still present')
            if before != checks.snapshot() or checks.BASELINE.read_bytes() != baseline_bytes:
                raise RuntimeError('source/configuration or baseline changed during verification')
            for name, destination in ALIASES.items():
                target = ROOT/destination
                if (os.path.lexists(ROOT/name) or
                        {'device':target.stat().st_dev, 'inode':target.stat().st_ino} != targets[name]):
                    raise RuntimeError('canonical directory identity changed: '+name)
            receipt = dict(removed=ALIASES, legacy_alias_count=0, data_files_deleted=0,
                source_pins_relaxed=False, target_identity=targets, research_suite=suite,
                canonical_runtime_checked=True, training_or_driving_executed=False,
                runnable_maps=sorted(runnable), protected_source_sha256=protected,
                execution_adapter_sha256={name:before[name] for name in
                    ('tools/workspace_python.py','tools/canonical_workspace.py')},
                verified_at_utc=datetime.now(timezone.utc).isoformat(), log_dir=str(args.log_dir),
                baseline_sha256=hashlib.sha256(baseline_bytes).hexdigest())
            with record.open('x') as stream:
                json.dump(receipt, stream, indent=2)
                stream.write('\n')
            (args.log_dir/'source_snapshot.json').write_text(json.dumps(before,indent=2)+'\n')
        except BaseException:
            conflicts = []
            for name in removed:
                if os.path.lexists(ROOT/name):
                    conflicts.append(name)
                else:
                    (ROOT/name).symlink_to(ALIASES[name], target_is_directory=True)
            if conflicts:
                print('ROLLBACK_CONFLICTS', conflicts, flush=True)
            raise
        print('ALL_ROOT_COMPATIBILITY_LINKS_REMOVED_DATA_PRESERVED', flush=True)


if __name__ == '__main__':
    main()
