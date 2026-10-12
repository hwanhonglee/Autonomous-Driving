"""HH_261001 - One prospective target adjustment cannot relax the measured entry band or change pulse safety logic."""

import inspect
from types import SimpleNamespace

from scripts.e2e import calibrate_town12_j15824_brake_pulses_v4_20261001 as worker
from scripts.e2e import run_owned_town12_j15824_brake_calibration_v4_20261001 as owner


def test_entire_level_loop_diff_is_only_target_and_matching_verification_text():
    expected = inspect.getsource(worker.inherited.run_level).replace("5.58", "5.688").replace(
        "targets differ from 1.55 m/s", "targets differ from 1.58 m/s")
    assert inspect.getsource(worker.run_level) == expected
    assert worker.PulsePolicy is worker.inherited.PulsePolicy
    assert worker.scalar_guards is worker.inherited.scalar_guards


def test_metadata_preserves_every_bound_and_the_launch_policy():
    old, new = worker.ORIGINAL_PROTOCOL(), worker.protocol()
    changed = {key for key in old.keys() | new.keys() if old.get(key) != new.get(key)}
    assert changed == {"policy_id", "original_pid_target_speed_mps", "original_pid_target_speed_kmh",
                       "revision", "reason", "single_factor_revision_from_v3", "entry_gate_relaxed"}
    assert new["original_pid_target_speed_kmh"] == 5.688 and new["entry_band_mps"] == [1.45, 1.55]
    assert new["consecutive_entry_band_ticks"] == 10 and new["maximum_launch_seconds"] == 20.
    assert not new["training_data_admitted"] and not new["smoother_launch_claimed"]


def test_out_of_band_native_samples_cannot_satisfy_the_inherited_entry_gate():
    policy = worker.PulsePolicy(0.)
    for index in range(20):
        speed = 1.43 if index % 2 else 1.51
        control = SimpleNamespace(throttle=.2, brake=0., steer=0., gear=1,
            hand_brake=False, reverse=False, manual_gear_shift=False)
        policy.apply(control, speed=speed, timestamp=index * .05, frame=index)
    assert policy.entry is None and policy.pulse_ticks == 0


def test_new_worker_and_owner_scopes_restore_unchanged_prior_versions():
    previous = worker.inherited.run_level, worker.inherited.protocol
    with worker.configured_worker():
        assert worker.inherited.run_level is worker.run_level
        assert worker.inherited.protocol()["original_pid_target_speed_kmh"] == 5.688
    assert (worker.inherited.run_level, worker.inherited.protocol) == previous
    old = owner.inherited.OUTPUT, owner.inherited.WORKER, owner.inherited.SOURCE_FILES, owner.inherited.worker
    with owner.configured_owner():
        assert owner.inherited.OUTPUT == owner.OUTPUT and owner.inherited.WORKER == owner.WORKER
        assert all((owner.ROOT / name).is_file() for name in owner.SOURCE_FILES)
    assert (owner.inherited.OUTPUT, owner.inherited.WORKER, owner.inherited.SOURCE_FILES, owner.inherited.worker) == old
