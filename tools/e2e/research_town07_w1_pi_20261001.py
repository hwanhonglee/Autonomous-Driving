"""HH_261001 - Compare an unchanged learned planner with original versus PI-only tracking."""

from contextlib import contextmanager
from pathlib import Path

from scripts.e2e import research_town07_w1_moving_brake_ab_v2_20261001 as runner
from scripts.e2e import town07_moving_brake_ab_contract_v2_20261001 as base
from scripts.e2e import town07_w1_pi_control_20261001 as control


ROOT = base.ROOT
RUN_ROOT = ROOT / "storage/experiments/validation/2026-10-01/town07_w1_pi_controller_v1"
PLAN = RUN_ROOT.with_name("town07_w1_pi_controller_plan_v1.json")
SCHEMA = "portable_e2e.town07_w1_pi_controller.20261001.v1"
MODULE = "scripts.e2e.research_town07_w1_pi_20261001"
EXTRA_SOURCES = (
    "learning/portable_e2e/longitudinal_pi_research_20261001.py",
    "tools/e2e/town07_w1_pi_control_20261001.py",
    "tools/e2e/research_town07_w1_pi_20261001.py",
    "learning/tests/test_longitudinal_pi_research_20261001.py",
    "learning/tests/test_town07_w1_pi_control_20261001.py",
    "learning/tests/test_town07_w1_pi_owner_20261001.py",
)


def trial_specs():
    """HH_261001 - Declare both attempts before either outcome; retain failures without retries."""
    return tuple(dict(index=index, trial_id=name, arm=arm, output=str(RUN_ROOT / name))
        for index, (name, arm) in enumerate((
            ("A1_original", "A_original"), ("B1_pi_only", "B_pi_only"))))


@contextmanager
def activated():
    """HH_261001 - Scope the new controller/identity; never rewrite frozen lifecycle sources."""
    original_fixed = base.fixed_contract
    original_sources = base.NEW_SOURCES

    def fixed():
        value = original_fixed()
        value.update(intervention="B changes only normal-moving longitudinal tracking to PI; model/steer/stop/emergency unchanged",
            integral_added=True, feedforward_added=False, throttle_floor_added=False,
            proportional_gain=.7, integral_gain=.2, integral_limit=1., control_timestep_s=.1,
            controller_only_comparison=True,
            inference_deadline_s=.1,
            minimum_learning_updates=0,
            interpretation="Two bounded twenty-second TRAIN-route diagnostics, not RTE-01 or full-route approval.",
            prospective_motion_screen=dict(
                denominator="both predeclared attempts, including failures",
                interval="all learned-controlled observations after the disclosed expert startup, before cleanup",
                net_progress_minimum_m=30., unintended_stationary_dwell_limit_s=1.,
                stationary_threshold_mps=.05, cte_p95_maximum_m=.35, cte_absolute_maximum_m=.75,
                collisions_maximum=0, lane_invasions_maximum=0,
                body_or_swept_lane_clearance_below_point_one_m_maximum=0,
                learned_rejected_commands_maximum=0,
                independent_route=False, full_feature_approval=False))
        return value

    changes = dict(RUN_ROOT=RUN_ROOT, PLAN=PLAN, SCHEMA=SCHEMA,
        ARMS=("A_original", "B_pi_only"), trial_specs=trial_specs,
        NEW_SOURCES=tuple(dict.fromkeys((*original_sources, *EXTRA_SOURCES))), fixed_contract=fixed)
    saved_base = {name: getattr(base, name) for name in changes}
    saved_runner = runner.MODULE, runner.control_ab
    try:
        for name, value in changes.items():
            setattr(base, name, value)
        runner.MODULE, runner.control_ab = MODULE, control
        yield
    finally:
        runner.MODULE, runner.control_ab = saved_runner
        for name, value in saved_base.items():
            setattr(base, name, value)


def main(argv=None):
    with activated():
        return runner.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
