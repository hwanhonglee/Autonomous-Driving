"""HH_261001 - Verify bounded natural release and frozen PI v2 restoration without CARLA."""

import errno
import pytest
from scripts.e2e import research_town07_w1_pi_v2_20261001 as v2


def test_immediate_real_probe_required():
    calls = []
    value = v2.wait_for_original_bind_checks(2440, calls.append)
    assert calls == [2440]
    assert value["original_all_tcp_udp_bind_check"] is True
    assert value["sockets_forced_or_reused"] is False


def test_transient_bind_is_not_bypassed():
    ticks, calls = [0.], []
    def probe(port):
        calls.append(port)
        if len(calls) < 3:
            raise OSError(errno.EADDRINUSE, "synthetic delayed shutdown")
    value = v2.wait_for_original_bind_checks(2440, probe,
        pause=lambda dt: ticks.__setitem__(0, ticks[0]+dt), clock=lambda: ticks[0])
    assert len(calls) == 3
    assert [row["passed"] for row in value["attempts"]] == [False, False, True]


def test_foreign_port_stays_rejected():
    ticks = [0.]
    def occupied(port):
        raise OSError(errno.EADDRINUSE, "foreign owner")
    with pytest.raises(OSError):
        v2.wait_for_original_bind_checks(2440, occupied, timeout_s=.2,
            pause=lambda dt: ticks.__setitem__(0, ticks[0]+dt), clock=lambda: ticks[0])


def test_other_bind_error_never_retried():
    def denied(port):
        raise OSError(errno.EACCES, "denied")
    with pytest.raises(OSError):
        v2.wait_for_original_bind_checks(2440, denied, pause=lambda _: pytest.fail("retried"))


def test_new_identity_and_exception_restoration():
    old_root, old_plan, old_module = v2.prior.RUN_ROOT, v2.prior.PLAN, v2.prior.runner.MODULE
    old_probe, old_fixed = v2.owner.ports_free, v2.prior.base.fixed_contract
    with pytest.raises(RuntimeError):
        with v2.activated():
            assert v2.prior.base.fixed_contract()["schema"] == v2.SCHEMA
            assert v2.prior.base.fixed_contract()["original_port_bind_check_natural_release_wait_s"] == 10.
            assert v2.prior.runner.MODULE == v2.MODULE
            assert v2.prior.base.output(1) == v2.RUN_ROOT / "B1_pi_only"
            raise RuntimeError("synthetic")
    assert v2.owner.ports_free is old_probe
    assert v2.prior.base.fixed_contract is old_fixed
    assert (v2.prior.RUN_ROOT, v2.prior.PLAN, v2.prior.runner.MODULE) == (old_root, old_plan, old_module)
