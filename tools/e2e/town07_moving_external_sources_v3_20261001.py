"""HH_261001 - Prospective fixed CARLA agents source closure; no policy or admission change."""

from datetime import datetime, timezone
import hashlib
import importlib
import inspect
import json
from pathlib import Path
import shutil
import sys

from scripts.e2e import town07_low_speed_moving_task_20261001 as task

PLAN_SHA = 'b8116231db1a5a548b1e890f21d403feb50bcef9403b49cbdcba5e3d055dfee1'
ADDENDUM = task.PLAN.parent / 'town07_low_speed_moving_v3_external_sources_addendum.json'
AGENTS_ROOT = Path('/home/a/carla-autoware-universe/CARLA_0.9.15/PythonAPI/carla/agents')
EMPTY = 'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855'
PINS = {'__init__.py': EMPTY, 'navigation/__init__.py': EMPTY, 'tools/__init__.py': EMPTY,
    'navigation/basic_agent.py': '7eb47c0ac30f36e2e71266e918c27fc02c8ff7b74d8f75259c313022e50be2ff',
    'navigation/local_planner.py': '7a5db9112b0d107038765abfb87b2d7a418b98e4d2c0aa92a4aade53c95b319f',
    'navigation/global_route_planner.py': '9b164a2de723d91d55cd1203543747580c1c6c509ed0294a6de17b001f7db8d0',
    'navigation/controller.py': 'f95f427a5ad5ba50a96f6971df72a5f3ec078c8457b0ba2a12558396e8b5c402',
    'tools/misc.py': '7e453c513c742a1ff3c8f3604fe55adafac2c682753093262e7f402e9ec1b72b'}
SOURCES = ('tools/e2e/town07_moving_external_sources_v3_20261001.py',
           'tools/e2e/collect_town07_low_speed_moving_v3_20261001.py',
           'tools/e2e/run_owned_town07_low_speed_moving_v3_20261001.py')
sha, read, require = task.sha, task.read, task.require


def write_json(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def safe_path(root, relative):
    item = Path(relative)
    require(not item.is_absolute() and '..' not in item.parts and relative in PINS, 'outside fixed agents source allowlist')
    candidate = root / item
    require(root.resolve() == root and candidate.resolve() == candidate and candidate.is_file(),
            'source symlink, redirection or missing file prohibited')
    return candidate


def source_snapshot(root=AGENTS_ROOT):
    rows = {}
    for relative, expected in PINS.items():
        path = safe_path(root, relative)
        require(sha(path) == expected, 'fixed CARLA agents source changed: ' + relative)
        rows[relative] = dict(source_file=str(path), sha256=expected, size_bytes=path.stat().st_size)
    return rows


def imported_snapshot():
    """HH_261001 - Bind actual import paths and inherited classes before/after temporary subclass scope."""
    require(AGENTS_ROOT.parent.resolve() == AGENTS_ROOT.parent, 'CARLA Python root is redirected')
    if str(AGENTS_ROOT.parent) not in sys.path:
        sys.path.insert(0, str(AGENTS_ROOT.parent))
    modules = {}
    for relative in PINS:
        suffix = relative.removesuffix('.py').replace('/', '.')
        suffix = suffix.removesuffix('.__init__')
        name = 'agents' if suffix == '__init__' else 'agents.' + suffix
        module = importlib.import_module(name)
        require(Path(module.__file__).absolute() == safe_path(AGENTS_ROOT, relative), 'actual imported agents module is shadowed')
        modules[name] = module
    expected = {'agents.navigation.basic_agent': 'BasicAgent', 'agents.navigation.local_planner': 'LocalPlanner',
                'agents.navigation.global_route_planner': 'GlobalRoutePlanner', 'agents.navigation.controller': 'VehiclePIDController'}
    classes = {}
    for name, attribute in expected.items():
        value = getattr(modules[name], attribute)
        require(value.__module__ == name and Path(inspect.getsourcefile(value)).resolve() == Path(modules[name].__file__),
                'inherited implementation class was replaced')
        classes[name + '.' + attribute] = str(Path(inspect.getsourcefile(value)))
    basic, local, global_route, controller = (modules[name] for name in expected)
    require(basic.LocalPlanner is local.LocalPlanner and basic.GlobalRoutePlanner is global_route.GlobalRoutePlanner
            and local.VehiclePIDController is controller.VehiclePIDController, 'actual transitive class aliases changed')
    return dict(schema='carla.fixed_agents_import_snapshot.v1', observed_at_utc=datetime.now(timezone.utc).isoformat(),
        sources=source_snapshot(), imported_module_paths={key: str(Path(value.__file__)) for key, value in modules.items()},
        inherited_classes=classes, status='PASS', scope='Eight fixed CARLA agents Python files, not a full Python/OS/binary attestation.')


def build_addendum():
    require(sha(task.PLAN) == PLAN_SHA and not task.RUN.exists(), 'unchanged unexecuted V2 task required')
    return dict(schema='carla.town07_moving_external_source_addendum.20261001.v3',
        declared_at_utc=datetime.now(timezone.utc).isoformat(), fixed_task_plan=str(task.PLAN), fixed_task_plan_sha256=PLAN_SHA,
        output=str(task.RUN), external_root=str(AGENTS_ROOT), external_sources=source_snapshot(),
        source_sha256={name: sha(task.ROOT / name) for name in SOURCES},
        capture_policy_changed=False, route_or_window_changed=False, numerical_gates_changed=False,
        resource_thresholds_changed=False, old_sources_and_declaration_preserved=True,
        first_capture_entrypoint='scripts.e2e.run_owned_town07_low_speed_moving_v3_20261001',
        task_admission_status='PENDING_QA', training_data_approved=False,
        plan_interpretation='Supplement unchanged V2 resource/task contract with prospective inherited agents source closure; V2-only launch is superseded before any capture.')


def verify_addendum(expected_sha):
    require(isinstance(expected_sha, str) and len(expected_sha) == 64 and sha(ADDENDUM) == expected_sha,
            'caller-pinned prospective V3 addendum required')
    plan = read(ADDENDUM)
    require(plan['schema'] == 'carla.town07_moving_external_source_addendum.20261001.v3'
            and plan['fixed_task_plan_sha256'] == sha(task.PLAN) == PLAN_SHA
            and plan['output'] == str(task.RUN) and plan['external_root'] == str(AGENTS_ROOT), 'V3 addendum identity differs')
    require(plan['external_sources'] == source_snapshot() and set(plan['source_sha256']) == set(SOURCES), 'V3 fixed source closure differs')
    for name, digest in plan['source_sha256'].items():
        require(sha(task.ROOT / name) == digest, 'V3 source changed: ' + name)
    return plan


def archive_sources(run, snapshot):
    """HH_261001 - Create-only 69 kB archive, before any server process is started."""
    destination = run / 'external_agents_source'
    require(not destination.exists() and run.resolve() == run, 'fresh canonical external archive required')
    require(snapshot == source_snapshot(), 'pre-archive source snapshot changed')
    for relative, record in snapshot.items():
        source = safe_path(AGENTS_ROOT, relative)
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        with source.open('rb') as src, target.open('xb') as dst:
            shutil.copyfileobj(src, dst)
        require(sha(target) == record['sha256'] == sha(source), 'archive/source changed during copy')


def verify_archive(run, snapshot):
    expected = set(snapshot)
    actual = {path.relative_to(run / 'external_agents_source').as_posix()
              for path in (run / 'external_agents_source').rglob('*') if path.is_file() or path.is_symlink()}
    require(actual == expected, 'external archive membership differs')
    for relative, record in snapshot.items():
        target = run / 'external_agents_source' / relative
        require(target.resolve() == target and sha(target) == record['sha256'], 'external archive bytes changed')
    require(snapshot == source_snapshot(), 'external source changed since declared snapshot')


def postcheck(run):
    owner_plan = read(run / 'owner_plan.json')
    plan = verify_addendum(owner_plan['external_sources_v3_addendum_sha256'])
    verify_archive(run, plan['external_sources'])
    before, after = (read(run / name) for name in ('external_agents_worker_before.json', 'external_agents_worker_after.json'))
    for item in (before, after):
        require(item['status'] == 'PASS' and item['addendum_sha256'] == sha(ADDENDUM)
                and item['import_snapshot']['sources'] == plan['external_sources'], 'worker source evidence differs')
        require(datetime.fromisoformat(plan['declared_at_utc']) <= datetime.fromisoformat(item['import_snapshot']['observed_at_utc']),
                'external-source declaration is not prospective')
    require(before['import_snapshot']['imported_module_paths'] == after['import_snapshot']['imported_module_paths']
            and before['import_snapshot']['inherited_classes'] == after['import_snapshot']['inherited_classes'],
            'worker actual imported classes/paths changed')
    return dict(status='PASS', schema='carla.town07_moving_external_sources_postcheck.v3',
        addendum_sha256=sha(ADDENDUM), source_files=len(PINS), source_bytes=sum(row['size_bytes'] for row in plan['external_sources'].values()),
        archive_and_pre_post_import_hashes_match=True, training_data_approved=False,
        frozen_v1_auditor_external_gap_resolved_for_this_v3_capture=True,
        notice='Source identity only, not numerical/route success or admission; V1 numerical audit remains separately required.')
