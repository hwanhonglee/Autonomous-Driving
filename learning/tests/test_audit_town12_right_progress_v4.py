"""HH_260929 - Read-only checks for completed Town12 V4 ledger rows."""

import hashlib
import json

import pytest

from scripts.e2e import audit_town12_right_progress_v4 as audit


def _counts(physical=0, nominal_fail=0):
    counts = {key: 0 for key in audit.COUNT_KEYS}
    counts.update({
        "endpoint_pairs": physical,
        "length_at_least_146m": physical,
        "traverses_connector": physical,
        "single_right_no_other_maneuver": physical,
        "right_block_on_selected_connector": physical,
        "turn_inside_30_120m": physical,
        "physical_turn_pass": physical,
        "nominal_fail": nominal_fail,
    })
    return counts


def _row(completed, junction_id, counts):
    return {
        "schema": "portable_e2e.town12_right_static_progress.v4",
        "completed_junctions": completed,
        "total_junctions": audit.TOTAL_JUNCTIONS,
        "last_completed_junction_id": junction_id,
        "cumulative_counts": counts,
        "cumulative_physical_route_hashes_sha256": "a" * 64,
        "partial_not_qualifying_evidence": True,
    }


def _write(path, rows):
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


def test_auditor_accepts_monotone_partial_ledger(tmp_path):
    path = tmp_path / "progress.jsonl"
    _write(path, [_row(1, 2, _counts(1, 1)), _row(2, 7, _counts(2, 2))])
    result = audit.audit(path)
    assert result["status"] == "PARTIAL_OPERATIONS_ONLY"
    assert result["completed_junctions"] == 2
    assert result["counts"]["physical_turn_pass"] == 2
    assert result["ledger_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.parametrize("rows,match", [
    ([_row(1, 2, _counts(1, 1)), _row(2, 2, _counts(2, 2))], "junction IDs"),
    ([_row(1, 2, _counts(2, 2)), _row(2, 7, _counts(1, 1))], "decreased"),
    ([_row(1, 2, _counts(1, 0))], "physical/body count mismatch"),
    ([_row(2, 2, _counts(1, 1))], "progress header changed"),
])
def test_auditor_rejects_inconsistent_rows(tmp_path, rows, match):
    path = tmp_path / "progress.jsonl"
    _write(path, rows)
    with pytest.raises(audit.LedgerError, match=match):
        audit.audit(path)


def test_auditor_refuses_final_report_for_partial_ledger(tmp_path):
    path = tmp_path / "progress.jsonl"
    _write(path, [_row(1, 2, _counts())])
    report = tmp_path / "report.json"
    report.write_text("{}")
    with pytest.raises(audit.LedgerError, match="requires all registered junctions"):
        audit.audit(path, report)


def test_auditor_crosschecks_complete_report_and_route_digest(tmp_path):
    path = tmp_path / "progress.jsonl"
    digest = audit.route_hash_digest([])
    rows = [_row(index, index + 5, _counts())
            for index in range(1, audit.TOTAL_JUNCTIONS + 1)]
    for row in rows:
        row["cumulative_physical_route_hashes_sha256"] = digest
    _write(path, rows)
    report = tmp_path / "report.json"
    report.write_text(json.dumps({
        "schema": "portable_e2e.town12_right_static_scout.v4",
        "progress_completed_junctions": audit.TOTAL_JUNCTIONS,
        "progress_ledger_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "counts": _counts(),
        "physical_pass_routes": [],
    }))
    assert audit.audit(path, report)["status"] == "COMPLETE_REPORT_CONSISTENT"
    changed = json.loads(report.read_text())
    changed["progress_ledger_sha256"] = "0" * 64
    report.write_text(json.dumps(changed))
    with pytest.raises(audit.LedgerError, match="disagrees"):
        audit.audit(path, report)
