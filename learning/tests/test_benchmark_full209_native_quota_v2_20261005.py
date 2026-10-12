"""HH_261005 - Verify parent creation and exact frozen delegates without actual files/model/capture execution."""

from pathlib import Path
import pytest

from scripts.e2e import benchmark_full209_native_quota_v2_20261005 as fresh


def test_parent_created_but_leaf_is_genuine_delegate(tmp_path, monkeypatch):
    output = tmp_path / "new-date" / "output"
    monkeypatch.setattr(fresh, "OUTPUT", output)
    seen = []
    def delegate(plan):
        assert output.parent.is_dir() and not output.exists()
        seen.append(plan)
        return "genuine unchanged result"
    monkeypatch.setattr(fresh, "_execute", delegate)
    assert fresh.execute({"seal": "unchanged"}) == "genuine unchanged result"
    assert seen == [{"seal": "unchanged"}]


@pytest.mark.parametrize("raises", [False, True])
def test_all_globals_restored(raises):
    names = ("SCHEMA", "PLAN_DIR", "PLAN", "LIBRARY", "OUTPUT", "HERE", "TEST", "sources", "execute")
    originals = {name:getattr(fresh.previous, name) for name in names}
    try:
        with fresh.activated():
            assert fresh.previous.OUTPUT == fresh.OUTPUT and fresh.previous.execute is fresh.execute
            if raises:
                raise RuntimeError("fixture failure")
    except RuntimeError:
        assert raises
    assert all(getattr(fresh.previous, name) is value for name, value in originals.items())


def test_source_closure_retains_failed_v1():
    with fresh.activated():
        result = fresh.sources()
    assert result[str(Path(fresh.previous.__file__))] == fresh.V1_SHA
    assert str(fresh.HERE) in result and str(fresh.TEST) in result
