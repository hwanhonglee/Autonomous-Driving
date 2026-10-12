"""HH_260906 - Keep recorded paths readable without filesystem compatibility links."""
from pathlib import Path
import pytest
from portable_e2e.workspace_paths import recorded_path, source_path, workspace_path


def test_removed_alias_maps_only_within_exact_root(tmp_path):
    root=tmp_path/'workspace'
    target=root/'storage/experiments/run.json'
    target.parent.mkdir(parents=True)
    target.write_text('{}')
    assert not (root/'artifacts').exists()
    assert recorded_path(root,root/'artifacts/run.json')==target
    assert recorded_path(root,target)==target
    assert recorded_path(root,root)==root
    external=tmp_path/'another/artifacts/run.json'
    assert recorded_path(root,external)==external
    assert source_path(root,'portable_e2e/model.py')==root/'learning/portable_e2e/model.py'


@pytest.mark.parametrize('value',['../x','relative', '/tmp/../outside'])
def test_reject_traversal_and_ambiguous_records(tmp_path,value):
    with pytest.raises(ValueError):recorded_path(tmp_path,Path(value))


def test_current_layout_and_frozen_layout_are_not_conflated(tmp_path):
    # HH_260906 - Remap current local I/O while preserving real legacy layouts in isolated replay repositories.
    current, frozen = tmp_path/'current', tmp_path/'frozen'
    (current/'learning/portable_e2e').mkdir(parents=True)
    (current/'tools/e2e').mkdir(parents=True)
    frozen.mkdir()
    logical = 'scripts/e2e/replay.py'
    assert workspace_path(current, logical) == current/'tools/e2e/replay.py'
    assert workspace_path(frozen, logical) == frozen/logical
