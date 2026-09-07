"""HH_260906 - Verify optional eleven-source ownership using only the existing harmless fake-process harness."""

import hashlib
import json

import pytest

from test_run_owned_carla_expert_trial import harness, _run


def enable_fixture_option(harness):
    path = harness['root'] / 'scripts/e2e/collect_carla_vad_expert.py'
    source = path.read_text()
    source = source.replace(" return p.parse_args(argv)", " p.add_argument('--wall-timing',action='store_true')\n return p.parse_args(argv)")
    path.write_text(source)
    helper = path.with_name('carla_wall_timing.py')
    helper.write_text('# HH_260906 - Fixture-only optional timing source.\n')
    return helper


def test_explicit_timing_archives_exactly_eleven_sources_and_postchecks_helper(harness):
    helper = enable_fixture_option(harness)
    result = _run(harness, '--', '--wall-timing')
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness['output'] / 'owner_plan.json').read_text())
    assert len(plan['source_sha256']) == 11
    assert plan['wall_timing_enabled'] is True and plan['wall_timing_source_bytes_archived'] is True
    assert plan['wall_timing_schema'] == 'carla.expert_wall_timing.v1'
    name = 'scripts/e2e/carla_wall_timing.py'
    assert plan['source_sha256'][name] == hashlib.sha256(helper.read_bytes()).hexdigest()
    assert (harness['output'] / 'provenance' / name).read_bytes() == helper.read_bytes()
    owner = json.loads((harness['output'] / 'owner_result.json').read_text())
    assert owner['source_checks'][name] is True and len(owner['source_checks']) == 11


def test_disabled_timing_preserves_ten_sources_and_no_new_owner_metadata(harness):
    enable_fixture_option(harness)
    result = _run(harness)
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness['output'] / 'owner_plan.json').read_text())
    assert len(plan['source_sha256']) == 10
    assert not any(key.startswith('wall_timing') for key in plan)
    assert 'scripts/e2e/carla_wall_timing.py' not in plan['source_sha256']


@pytest.mark.parametrize('mode', ['actuation-response', 'stationary-camera'])
def test_timing_option_cannot_add_hidden_dependency_to_other_owned_workers(harness, mode):
    enable_fixture_option(harness)
    result = _run(harness, '--capture-mode', mode, '--', '--wall-timing')
    assert result.returncode != 0 and not harness['events'].exists() and not harness['output'].exists()


def test_missing_timing_helper_cannot_start_a_server(harness):
    helper = enable_fixture_option(harness)
    helper.unlink()
    result = _run(harness, '--', '--wall-timing')
    assert result.returncode != 0 and not harness['events'].exists()


def test_changed_optional_helper_fails_postcheck_without_modifying_archived_bytes(harness):
    helper = enable_fixture_option(harness)
    original = helper.read_bytes()
    worker = helper.with_name('collect_carla_vad_expert.py')
    source = worker.read_text().replace(" a=parse_args()", " a=parse_args()\n with open('scripts/e2e/carla_wall_timing.py','a') as f: f.write('# HH_260906 - Synthetic mutation.\\n')")
    worker.write_text(source)
    result = _run(harness, '--', '--wall-timing')
    assert result.returncode != 0
    owner = json.loads((harness['output'] / 'owner_result.json').read_text())
    assert owner['source_checks']['scripts/e2e/carla_wall_timing.py'] is False
    assert (harness['output'] / 'provenance/scripts/e2e/carla_wall_timing.py').read_bytes() == original
