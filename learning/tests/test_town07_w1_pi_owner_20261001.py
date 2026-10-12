"""HH_261001 - Test prospective PI owner identity and restoration without starting CARLA."""

import pytest
from scripts.e2e import research_town07_w1_pi_20261001 as pi


def test_distinct_attempts():
    rows = pi.trial_specs()
    assert [row["arm"] for row in rows] == ["A_original", "B_pi_only"]
    assert len({row["output"] for row in rows}) == 2
    assert all(pi.RUN_ROOT.as_posix() in row["output"] for row in rows)


def test_unchanged_model_and_safety_restored():
    old_plan, old_module, old_control = pi.base.PLAN, pi.runner.MODULE, pi.runner.control_ab
    old_fixed = pi.base.fixed_contract
    original = old_fixed()
    with pi.activated():
        value = pi.base.fixed_contract()
        assert value["schema"] == pi.SCHEMA
        assert value["checkpoint_sha256"] == original["checkpoint_sha256"]
        assert value["raw_argmax_and_runtime_gates_unchanged"] is True
        assert value["original_emergency_authority_unchanged"] is True
        assert value["original_stop_command_unchanged"] is True
        assert value["integral_added"] is True
        assert value["feedforward_added"] is False
        assert value["total_capture_seconds"] == 20
        assert value["full_feature_approval"] is False
        assert value["prospective_motion_screen"]["net_progress_minimum_m"] == 30.
        assert pi.runner.MODULE == pi.MODULE
        assert pi.runner.control_ab is pi.control
    assert pi.base.PLAN == old_plan
    assert pi.base.fixed_contract is old_fixed
    assert pi.runner.MODULE == old_module
    assert pi.runner.control_ab is old_control


def test_restore_on_error():
    previous = pi.base.NEW_SOURCES
    with pytest.raises(RuntimeError):
        with pi.activated():
            raise RuntimeError("synthetic failure")
    assert pi.base.NEW_SOURCES == previous


def test_fixed_capture_paths():
    with pi.activated():
        argv = pi.runner.capture_argv(1)
        assert argv[0] == str(pi.RUN_ROOT / "B1_pi_only/episode")
        assert argv[argv.index("--max-duration-sec") + 1] == "20"
        assert argv[argv.index("--port") + 1] == "2440"
