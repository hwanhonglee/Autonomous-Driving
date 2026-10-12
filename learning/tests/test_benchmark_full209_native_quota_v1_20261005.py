"""HH_261005 - Verify complete paired counts/order and fail-tail retention without reading any actual episode."""

from pathlib import Path
import pytest

from scripts.e2e import benchmark_full209_native_quota_v1_20261005 as bench


def test_all60_fixed_order_full_counts(tmp_path, monkeypatch):
    expected = dict(entries=5, directories_including_root=2, regular_files=3, regular_file_bytes=7,
                    special_nodes_ignored_like_original=1)
    def call(path, receipt):
        receipt.update(expected, complete_walk=True, cached_or_incremental_byte_count_used=False)
        return 7
    monkeypatch.setattr(bench.original, "scandir_directory_bytes", call)
    class Candidate:
        directory_bytes = staticmethod(call)
    journal = []
    rows = bench.paired_calls(Candidate(), tmp_path, expected, journal.append)
    assert len(rows) == len(journal) == 60
    assert [row["arm"] for row in rows[:4]] == ["old", "native", "native", "old"]
    assert all(row["status"] == "PASS" for row in rows)
    assert all(row["counters"]["regular_file_bytes"] == 7 for row in rows)


def test_first_failure_is_journaled_not_filtered(tmp_path, monkeypatch):
    expected = {key: 0 for key in bench.FIELDS}
    def fail(path, receipt):
        raise PermissionError("unreadable prefix")
    monkeypatch.setattr(bench.original, "scandir_directory_bytes", fail)
    journal = []
    with pytest.raises(PermissionError):
        bench.paired_calls(None, tmp_path, expected, journal.append)
    assert len(journal) == 1 and journal[0]["status"] == "FAIL" and journal[0]["pair"] == 0


def test_wrong_count_is_a_failure(tmp_path, monkeypatch):
    expected = {key: 0 for key in bench.FIELDS}
    def wrong(path, receipt):
        receipt.update(expected, regular_files=1, complete_walk=True, cached_or_incremental_byte_count_used=False)
        return 0
    monkeypatch.setattr(bench.original, "scandir_directory_bytes", wrong)
    journal = []
    with pytest.raises(RuntimeError, match="counters"):
        bench.paired_calls(None, tmp_path, expected, journal.append)
    assert journal[0]["status"] == "FAIL"


def test_no_call_dropped_from_statistics():
    value = bench.stats([1., 2., 3., 4., 100.])
    assert value["count"] == 5 and value["maximum_ms"] == 100. and value["mean_ms"] == 22.
