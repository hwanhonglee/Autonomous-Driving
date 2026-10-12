"""HH_261001 - Historical exception is receipt-specific; original source stays frozen."""
from scripts.e2e import audit_ctrack_isolated_moving_20261001 as old
from scripts.e2e import audit_ctrack_isolated_moving_20261001_v2 as new


def test_forensic_exception_requires_both_exact_receipts():
    hashes = {"owner_result":new.FORENSIC_OWNER_SHA,"summary":new.FORENSIC_SUMMARY_SHA}
    assert new.forensic_warning_failure(hashes)
    assert not new.forensic_warning_failure({"owner_result":new.FORENSIC_OWNER_SHA})
    assert not new.forensic_warning_failure({**hashes,"summary":"0"*64})


def test_other_qa_thresholds_remain_identical():
    assert vars(old.Policy()) == vars(new.Policy())
    assert old.ROUTES == new.ROUTES
    assert old.LABEL_TABLE == new.LABEL_TABLE


def test_forensic_result_never_claims_owner_pass():
    result = new._finish({"fail_reasons":[],"forensic_raw_recovery":True})
    assert result["status"] == "QA_CONDITIONAL_FORENSIC_RAW"
    result = new._finish({"fail_reasons":[{"code":"OTHER"}],"forensic_raw_recovery":True})
    assert result["status"] == "FAIL" and not result["raw_semantic_diagnostic_eligible"]
