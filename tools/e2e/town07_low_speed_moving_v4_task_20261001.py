"""HH_261001 - Rebind the unchanged moving-task contract to one fresh V4 capture."""

from contextlib import contextmanager
from pathlib import Path

from scripts.e2e import town07_low_speed_moving_task_20261001 as original


ROOT = original.ROOT
RUN = ROOT / 'storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v4'
PLAN = ROOT / 'storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_plan_v4.json'
SLOT = 'T07-LOW-CRUISE-20261001-V4'
SOURCE_PATHS = tuple(dict.fromkeys((*original.SOURCE_PATHS,
    'tools/e2e/town07_low_speed_moving_v4_task_20261001.py',
    'tools/e2e/town07_moving_external_sources_v3_20261001.py',
    'tools/e2e/town07_moving_external_sources_v4_20261001.py',
    'tools/e2e/collect_town07_low_speed_moving_v4_20261001.py',
    'tools/e2e/run_owned_town07_low_speed_moving_v4_20261001.py',
    'tools/e2e/run_owned_ah_town07_research_20261001.py',
    'tools/workspace_python.py', 'tools/canonical_workspace.py')))
PATCHED = dict(RUN=RUN, PLAN=PLAN, SLOT=SLOT, SOURCE_PATHS=SOURCE_PATHS)


@contextmanager
def activated():
    """HH_261001 - Use the original task code with only fresh path/identity globals."""
    saved = {name: getattr(original, name) for name in PATCHED}
    try:
        for name, value in PATCHED.items():
            setattr(original, name, value)
        yield
    finally:
        for name, value in saved.items():
            setattr(original, name, value)


def build_plan(mapping, calibration):
    if PLAN.exists() or RUN.exists():
        raise ValueError('create-only V4 task plan and output required')
    with activated():
        plan = original.build_plan(mapping, calibration)
    if plan['output'] != str(RUN / 'episode') or plan['cohort_slot_id'] != SLOT:
        raise ValueError('V4 task identity differs')
    return plan


def verify_plan(expected_sha):
    if not isinstance(expected_sha, str) or len(expected_sha) != 64 or original.sha(PLAN) != expected_sha:
        raise ValueError('caller-pinned V4 task plan required')
    plan = original.read(PLAN)
    if plan['output'] != str(RUN / 'episode') or plan['cohort_slot_id'] != SLOT:
        raise ValueError('V4 task path or cohort differs')
    if set(plan['source_sha256']) != set(SOURCE_PATHS):
        raise ValueError('V4 capture source closure differs')
    for relative, digest in plan['source_sha256'].items():
        path = ROOT / relative
        if path.is_symlink() or not path.is_file() or original.sha(path) != digest:
            raise ValueError('V4 capture source changed: ' + relative)
    return plan
