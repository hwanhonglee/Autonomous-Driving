"""HH_261001 - Freeze the explicit combined correction without changing strict geometric/control gates."""

import json

from scripts.e2e import capture_town12_j15824_expert9_v2stop_20261001 as worker
from scripts.e2e import run_owned_town12_j15824_expert9_v2stop_20261001 as owner


def test_exact_three_argument_changes_and_restored_sources(tmp_path):
    lift = worker.inherited.inherited
    before = lift.collector_arguments(tmp_path, tmp_path / "route.json")
    old = lift.ORIGINAL_ARGUMENTS, lift.base.TARGET_SPEED_KMH
    with worker.configured_variant():
        after = lift.collector_arguments(tmp_path, tmp_path / "route.json")
        args = lift.collector.parse_args(after)
        assert args.target_speed_kmh == 9 and args.max_duration_sec == 180
        assert args.goal_stop_profile == "comfortable_v2" and args.spawn_z_offset_m == .5
        assert args.observe_traffic_lights and not args.allow_map_load
        assert args.physics_hz == 20 and args.capture_hz == 10
        assert args.control_transport == "acknowledged_batch"
        assert sum(a != b for a, b in zip(before, after)) == 3
        stop = lift.collector.configuration_from_args(args)
        assert stop.desired_deceleration_mps2 == 1 and stop.normal_brake_cap == .15 and stop.normal_throttle_cap == .2
        assert lift.base.TARGET_SPEED_KMH == 9
    assert (lift.ORIGINAL_ARGUMENTS, lift.base.TARGET_SPEED_KMH) == old


def test_final_frozen_plan_labels_are_nine_not_inherited_twenty(tmp_path):
    with owner.configured_owner() as actual_owner:
        target = tmp_path / "owner_plan.json"
        actual_owner._write_new_json(target, {"source_sha256": {}})
        plan = json.loads(target.read_text())
        assert plan["signal_expert_trial"]["target_speed_kmh"] == 9
        argv = plan["signal_expert_trial"]["collector_arguments"]
        assert argv[argv.index("--target-speed-kmh") + 1] == "9"
        assert argv[argv.index("--goal-stop-profile") + 1] == "comfortable_v2"
        assert plan["signal_expert_trial"]["combined_protocol_revision"]["single_factor_causal_ab"] is False
        paths = owner.inherited.inherited.inherited.scout_owner.diagnostic.source_paths()
        assert "tools/e2e/capture_town12_j15824_expert9_v2stop_20261001.py" in paths


def test_new_output_and_worker_do_not_reuse_failed_trial():
    with owner.configured_owner():
        assert owner.inherited.inherited.OUTPUT == owner.OUTPUT
        assert owner.inherited.inherited.WORKER == owner.WORKER
        assert owner.OUTPUT != owner.FAILED_TWENTY.parent
