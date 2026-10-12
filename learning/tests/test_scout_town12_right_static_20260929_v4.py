"""HH_260929 - Check the prepared Town12 cache and completed-junction ledger."""

import hashlib
import json

import pytest

from scripts.e2e import scout_town12_right_static_20260929 as base
from scripts.e2e import scout_town12_right_static_20260929_v4 as v4


def test_section_bounds_cache_equals_original_on_pinned_town12():
    xml = base.SOURCE_XODR.read_bytes()
    expected = v4.ORIGINAL_SECTION_BOUNDS(xml)
    cache = v4.SectionBoundsCache(hashlib.sha256(xml).hexdigest())
    first = cache(xml)
    assert first == expected
    first.clear()
    assert cache(xml) == expected
    assert cache.parse_count == 1 and cache.requests == 2
    with pytest.raises(base.ScoutError, match="differs from pinned Town12 XODR"):
        cache(xml + b" ")


def test_progress_ledger_records_only_completed_junctions(tmp_path):
    path = tmp_path / "progress.jsonl"
    ledger = v4.ProgressLedger(path, 2)
    ledger.record(5, {"endpoint_pairs": 1}, [{"route_sha256": "a" * 64}])
    ledger.record(9, {"endpoint_pairs": 3}, [{"route_sha256": "a" * 64}])
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert [row["completed_junctions"] for row in rows] == [1, 2]
    assert [row["last_completed_junction_id"] for row in rows] == [5, 9]
    assert all(row["partial_not_qualifying_evidence"] for row in rows)
    assert rows[-1]["cumulative_counts"] == {"endpoint_pairs": 3}
    with pytest.raises(base.ScoutError, match="progress exceeds junction count"):
        ledger.record(10, {"endpoint_pairs": 3}, [])
