"""HH_260906 - Count temporary root aliases honestly; never equate hidden entries with removal."""
import json
import importlib.util
from pathlib import Path
# HH_260906 - CARLA already exports a tools package; do not shadow or depend on that namespace.
_spec = importlib.util.spec_from_file_location('hh_layout_migration', Path(__file__).with_name('migrate_workspace_layout.py'))
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
MOVES = _module.MOVES

ROOT = Path(__file__).resolve().parents[1]


def audit(root=ROOT):
    aliases, errors = [], []
    for old, new in MOVES.items():
        target, legacy = root / new, root / old
        if not target.is_dir() or target.is_symlink():
            errors.append('canonical directory missing or linked: ' + new)
        if legacy.is_symlink():
            aliases.append(old)
            if legacy.resolve() != target.resolve():
                errors.append('legacy points to wrong target: ' + old)
        elif legacy.exists():
            errors.append('unexpected real legacy path: ' + old)
    shortcuts = [str(p.relative_to(root)) for p in (root / 'docs/workflows').rglob('*') if p.is_symlink()]
    return dict(legacy_alias_count=len(aliases), legacy_aliases=aliases,
                workflow_shortcut_count=len(shortcuts), workflow_shortcuts=shortcuts,
                final_target_alias_count=0, path_errors=errors,
                legacy_removal_complete=not aliases and not errors and not shortcuts,
                canonical_only_runtime_verified=False)


if __name__ == '__main__':
    result = audit()
    print(json.dumps(result, indent=2))
    raise SystemExit(bool(result['path_errors']))
