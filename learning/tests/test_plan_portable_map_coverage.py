"""HH_260906 - Keep next-campaign planning non-executable and independent of legacy split labels."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from scripts.e2e import plan_portable_map_coverage as planner


@pytest.fixture
def sources():
    maps, assets = [], []
    for name, (canonical, load, status, profile) in planner.inventory.MAP_CONTRACT.items():
        maps.append(dict(id=name, canonical_name=canonical, status=status, server_profile=profile,
                         split=planner.inventory.SPLIT_CONTRACT[name]))
        assets.append(dict(id=name, assets_complete=status != 'unavailable'))
    profiles = [dict(id=name, executable_exists=True, executable_is_runnable=True,
                     python_api_exists=True, python_api_sha256_matches=True)
                for name in planner.inventory.PROFILE_CONTRACT]
    return dict(maps=maps), dict(maps=assets, profiles=profiles)


def fake_counts(root, map_id, canonical, ledger):
    assert map_id not in planner.PROTECTED_MAPS
    return dict(straight=8, left=0 if map_id in ('town06', 'town10hd_opt') else 8, right=8)


def test_all_cells_denied_even_with_assets(sources, monkeypatch):
    monkeypatch.setattr(planner, 'catalog_counts', fake_counts)
    result = planner.build_plan(*sources, Path('/unused'))
    assert result['map_count'] == 19 and result['scenario_cell_count'] == 95
    assert result['execution_commands'] == []
    assert result['all_maps_validated'] is False
    assert result['world_rpc_performed'] is False
    assert all(not cell['execution_allowed'] and not cell['training_data_approved']
               for row in result['maps'] for cell in row['cells'])
    assert sum(row['learned_closed_loop_passes'] for row in result['maps']) == 0
    cfg = result['constraints']
    assert (cfg['camera_hz'], cfg['training_physical_gpu']) == (10, 0)
    assert not cfg['installations_allowed'] and not cfg['automatic_model_promotion']


def test_locked_and_dev_overrule_legacy_train_labels(sources, monkeypatch):
    monkeypatch.setattr(planner, 'catalog_counts', fake_counts)
    rows = {row['map_id']: row for row in planner.build_plan(*sources, Path('/unused'))['maps']}
    assert rows['town04']['active_role'] == 'LOCKED_HOLDOUT'
    assert rows['town04']['manifest_split'] == 'train'
    assert rows['town04']['split_policy_conflict']
    assert rows['town04']['historical_catalog_counts'] is None
    assert rows['town03']['active_role'] == 'ACTIVE_DEVELOPMENT_MAP'
    assert rows['town03']['split_policy_conflict']
    assert rows['town15']['active_role'] == 'LOCKED_HOLDOUT'


def test_missing_ready_assets_not_implicitly_available(sources, monkeypatch):
    def check(root, map_id, canonical, ledger):
        assert map_id != 'woraksan_1_0_3'
        return fake_counts(root, map_id, canonical, ledger)
    monkeypatch.setattr(planner, 'catalog_counts', check)
    next(row for row in sources[1]['maps'] if row['id'] == 'woraksan_1_0_3')['assets_complete'] = False
    rows = planner.build_plan(*sources, Path('/unused'))['maps']
    row = next(row for row in rows if row['map_id'] == 'woraksan_1_0_3')
    assert row['declared_status'] == 'ready'
    assert row['historical_catalog_counts'] is None
    assert 'REQUIRED_MAP_ASSETS_NOT_CONFIRMED' in row['cells'][0]['blockers']


def test_turn_directions_and_intent_gaps_remain_explicit(sources, monkeypatch):
    monkeypatch.setattr(planner, 'catalog_counts', fake_counts)
    row = next(row for row in planner.build_plan(*sources, Path('/unused'))['maps'] if row['map_id'] == 'town06')
    cells = {cell['scenario']: cell for cell in row['cells']}
    assert 'NO_CATALOG_ROUTE_FOR_DIRECTION' in cells['left']['blockers']
    assert 'NO_CATALOG_ROUTE_FOR_DIRECTION' not in cells['right']['blockers']
    assert 'STOP_RESTART_INTENT_PROTOCOL_NOT_APPROVED' in cells['restart']['blockers']


@pytest.mark.parametrize('name', sorted(planner.PROTECTED_MAPS))
def test_holdout_rejected_before_path_access(name, monkeypatch):
    monkeypatch.setattr(planner, 'checked_path', lambda value: pytest.fail('touched protected payload path'))
    with pytest.raises(ValueError, match='protected'):
        planner.catalog_counts(Path('/unused'), name, 'unused', {})


@pytest.mark.parametrize('kind', ['map_duplicate', 'map_missing', 'asset_duplicate', 'asset_missing', 'asset_not_bool'])
def test_invalid_inventory_rejected(sources, kind, monkeypatch):
    monkeypatch.setattr(planner, 'catalog_counts', fake_counts)
    first, second = deepcopy(sources)
    if kind == 'map_duplicate': first['maps'].append(first['maps'][0])
    if kind == 'map_missing': first['maps'].pop()
    if kind == 'asset_duplicate': second['maps'].append(second['maps'][0])
    if kind == 'asset_missing': second['maps'].pop()
    if kind == 'asset_not_bool': second['maps'][0]['assets_complete'] = 1
    with pytest.raises(ValueError):
        planner.build_plan(first, second, Path('/unused'))


@pytest.mark.parametrize('value', ['{"x":1,"x":2}', '{"x":NaN}', '{"x":Infinity}', '{"x":[1e999]}'])
def test_ambiguous_or_nonfinite_json_rejected(value):
    with pytest.raises(ValueError): planner.strict_json(value)


def make_catalogs(root):
    for kind, scenario in [('straight', 'straight'), ('turn', 'right')]:
        path = root / 'town06' / 'catalog' / kind / 'route_catalog.json'
        path.parent.mkdir(parents=True)
        path.write_text(json.dumps(dict(status='complete', map_id='town06', canonical_name='Town06',
            routes=[dict(id='town06_'+scenario, status='ready', scenario=scenario, path='/never/read/route.json')])) )


def test_catalog_metadata_counts_without_route_payloads(tmp_path):
    make_catalogs(tmp_path)
    ledger = {}
    counts = planner.catalog_counts(tmp_path, 'town06', 'Town06', ledger)
    assert counts == dict(straight=1, left=0, right=1)
    assert len(ledger) == 2 and all(len(value) == 64 for value in ledger.values())


@pytest.mark.parametrize('change', ['wrong_map', 'wrong_scenario', 'duplicate_id', 'incomplete', 'non_ready'])
def test_bad_catalog_rejected(tmp_path, change):
    make_catalogs(tmp_path)
    path = tmp_path / 'town06/catalog/turn/route_catalog.json'
    document = json.loads(path.read_text())
    if change == 'wrong_map': document['map_id'] = 'town04'
    if change == 'wrong_scenario': document['routes'][0]['scenario'] = 'straight'
    if change == 'duplicate_id': document['routes'][0]['id'] = 'town06_straight'
    if change == 'incomplete': document['status'] = 'partial'
    if change == 'non_ready': document['routes'][0]['status'] = 'rejected'
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError): planner.catalog_counts(tmp_path, 'town06', 'Town06', {})


def test_symlink_and_traversal_rejected(tmp_path):
    link = tmp_path / 'link'
    link.symlink_to(tmp_path, target_is_directory=True)
    for path in (link / 'file', tmp_path / '..' / 'elsewhere'):
        with pytest.raises(ValueError): planner.checked_path(path)


def test_existing_output_cannot_be_overwritten(tmp_path):
    path = tmp_path / 'report.json'
    path.write_text('previous evidence')
    with pytest.raises(ValueError, match='fresh'):
        planner.main(['--catalog-maps-root', str(tmp_path / 'catalogs'), '--output-json', str(path)])
    assert path.read_text() == 'previous evidence'


@pytest.mark.parametrize('field', ['executable_exists', 'executable_is_runnable', 'python_api_exists', 'python_api_sha256_matches'])
def test_missing_runtime_profile_blocks_catalog_reads(sources, monkeypatch, field):
    next(row for row in sources[1]['profiles'] if row['id'] == 'packaged_0915')[field] = False
    monkeypatch.setattr(planner, 'catalog_counts', lambda *args: pytest.fail('unavailable runtime catalog read'))
    result = planner.build_plan(*sources, Path('/unused'))
    town07 = next(row for row in result['maps'] if row['map_id'] == 'town07')
    assert 'PACKAGED_EXECUTABLE_OR_API_NOT_CONFIRMED' in town07['cells'][0]['blockers']


def test_main_unpacks_real_manifest_loader_tuple(tmp_path, sources, monkeypatch):
    # HH_260906 - Exercise the real loader ABI while replacing all asset scans and route reads.
    real_load_manifest = planner.inventory.load_manifest
    loaded = {}

    def load_manifest(path):
        result = real_load_manifest(path)
        assert isinstance(result, tuple) and len(result) == 2
        loaded['manifest'] = result[0]
        return result

    def static_inventory(manifest):
        assert manifest is loaded['manifest']
        assert isinstance(manifest, dict)
        return sources[1]

    monkeypatch.setattr(planner.inventory, 'load_manifest', load_manifest)
    monkeypatch.setattr(planner.inventory, 'collect_static_inventory', static_inventory)
    monkeypatch.setattr(planner.inventory, 'collect_live_inventory',
                        lambda *args, **kwargs: pytest.fail('planner attempted live CARLA inventory'))
    monkeypatch.setattr(planner, 'catalog_counts', fake_counts)
    output = tmp_path / 'fresh' / 'plan.json'
    assert planner.main(['--catalog-maps-root', str(tmp_path / 'catalogs'),
                         '--output-json', str(output)]) == 0
    report = json.loads(output.read_text())
    assert report['status'] == 'PLAN_ONLY_NOT_EXECUTABLE'
    assert report['map_count'] == 19 and report['scenario_cell_count'] == 95
    assert report['execution_commands'] == []
    assert all(cell['status'] == 'BLOCKED' for row in report['maps'] for cell in row['cells'])
