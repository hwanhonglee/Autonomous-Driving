"""HH_261002 - Exact-path endpoint selection leaves every duplicate live/archive asset authentication in place."""

from contextlib import contextmanager
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import audit_temporal_route_joint_thread4_signed45_batch_v2_20261002 as audit


def test_generated_reader_adds_exact_final_metadata_path_without_changing_any_old_hash_loop():
    reader=audit.exact_metadata_reader()
    assert reader.__name__=="load_trial"
    assert str(audit.EXACT_METADATA) in reader.__code__.co_consts
    assert reader.__globals__["verify_contract"] is audit.previous.verify_contract
    assert reader.__globals__["extra_trial_evidence"] is audit.previous.extra_trial_evidence


@pytest.mark.parametrize("failure",[False,True])
def test_scope_preserves_frozen_source_and_original_reader_on_any_exit(failure):
    saved=audit.previous.transformed_trial_reader
    with mock.patch.object(audit.previous,"digest",return_value=audit.V1_SHA):
        try:
            with audit.activated():
                assert audit.previous.transformed_trial_reader is audit.exact_metadata_reader
                with pytest.raises(Exception):
                    with audit.activated():pass
                if failure:raise OSError("original failure retained")
        except OSError:assert failure
    assert audit.previous.transformed_trial_reader is saved and not audit._active


def test_unknown_frozen_v1_source_rejected():
    with mock.patch.object(audit.previous,"digest",return_value="a"*64),pytest.raises(Exception):
        with audit.activated():pass


def test_CLI_restores_original_report_builder_on_error():
    saved=audit.previous.build_report
    with mock.patch.object(audit.previous,"main",side_effect=OSError("retained")),pytest.raises(OSError):
        audit.main([])
    assert audit.previous.build_report is saved
