"""HH_261002 - Check exact reader identity and restoration of the versioned publication adapter."""

import pytest

from scripts.e2e import publish_temporal_route_joint_signed45_v2_20261002 as module


def test_exact_reader_and_restore():
    audit = module.previous.audit
    original = module.previous.load_archive
    assert not hasattr(audit, "measured_rear_errors")
    with module.activated():
        assert audit.measured_rear_errors is audit.previous.measured_rear_errors
        assert module.previous.load_archive is module.load_archive
    assert not hasattr(audit, "measured_rear_errors")
    assert module.previous.load_archive is original


def test_exception_and_nested_scope_restore():
    audit = module.previous.audit
    original = module.previous.load_archive
    with pytest.raises(RuntimeError, match="sentinel"):
        with module.activated():
            with pytest.raises(RuntimeError, match="cannot nest"):
                with module.activated():
                    pass
            raise RuntimeError("sentinel")
    assert not module._active
    assert not hasattr(audit, "measured_rear_errors")
    assert module.previous.load_archive is original
