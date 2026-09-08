"""HH_260906 - Verify numerical substep boundaries using fake worlds, never a simulator or GPU."""

from copy import deepcopy
import hashlib
import inspect
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import carla_physics_substeps as substeps
from scripts.e2e import collect_carla_vad_expert as collector
from test_run_owned_carla_expert_trial import harness, _run  # noqa: F401

ROOT = Path(__file__).resolve().parents[1]
ROUTE = ROOT / "datasets/raw/carla/common10_v1/2026-09-04/30kph/c_track_1_0_7/turn/route_catalog_v5/routes/c_track_1_0_7/left/c_track_1_0_7_left_s0000_p01.json"


def args(profile="reference_10ms"):
    return SimpleNamespace(physics_substep_profile=profile, goal_stop_profile="turn_launch_013_v1",
                           control_transport="acknowledged_batch", agent_initialization="after_bootstrap",
                           physics_hz=20.0, capture_hz=10.0)


def settings(**changes):
    values = dict(synchronous_mode=False, no_rendering_mode=False, substepping=True,
                  deterministic_ragdolls=False, spectator_as_ego=True, fixed_delta_seconds=None,
                  max_substep_delta_time=0.01, max_substeps=10, max_culling_distance=0.0,
                  tile_stream_distance=3000.0, actor_active_distance=2000.0)
    return SimpleNamespace(**dict(values, **changes))


class FakeWorld:
    def __init__(self, original=None, drift=None):
        self.current = original or settings()
        self.applied = []
        self.drift = drift or {}

    def get_settings(self):
        return deepcopy(self.current)

    def apply_settings(self, value):
        self.applied.append(deepcopy(value))
        self.current = deepcopy(value)
        for name, changed in self.drift.items():
            setattr(self.current, name, changed)


def flags(profile="reference_10ms", output="unused", route=ROUTE):
    return [str(output), str(route), "--physics-substep-profile", profile,
            "--goal-stop-profile", "turn_launch_013_v1", "--control-transport", "acknowledged_batch",
            "--agent-initialization", "after_bootstrap", "--target-speed-kmh", "14.4",
            "--goal-tolerance-m", "1", "--stationary-warmup-sec", "3.5", "--stationary-tail-sec", "6.5"]


@pytest.mark.parametrize("profile,delta", substeps.PROFILES.items())
def test_exact_settings_before_requested_after_and_only_numerical_difference(profile, delta):
    world = FakeWorld()
    original = world.get_settings()
    original_copy = deepcopy(vars(original))
    record = substeps.new_record(profile)
    substeps.configure_world(world, original, args(profile), record)
    assert len(world.applied) == 1 and vars(original) == original_copy
    assert record["status"] == "PASS" and len(record["after"]) == 11
    assert record["after"] == record["requested"]
    assert record["after"] == dict(record["before"], synchronous_mode=True,
                                   fixed_delta_seconds=.05, max_substep_delta_time=delta)
    assert record["development_only"] and not record["training_data_approved"]
    assert record["after"]["max_substeps"] == 10
    assert .05 <= delta * 10


@pytest.mark.parametrize("name,value", [("substepping", False), ("max_substep_delta_time", .005),
    ("max_substeps", 9), ("max_substeps", 17), ("max_substeps", True),
    ("max_substep_delta_time", float("nan")), ("actor_active_distance", float("inf"))])
def test_bad_original_settings_fail_without_apply(name, value):
    world = FakeWorld(settings(**{name: value}))
    record = substeps.new_record("fine_5ms")
    with pytest.raises(ValueError):
        substeps.configure_world(world, world.get_settings(), args("fine_5ms"), record)
    assert record["status"] == "FAIL" and record["after"] is None and not world.applied


@pytest.mark.parametrize("name,value", [("max_substep_delta_time", .01), ("fixed_delta_seconds", .1),
    ("no_rendering_mode", True), ("max_substeps", 11), ("actor_active_distance", 1500.0)])
def test_readback_any_setting_drift_fails_retained(name, value):
    world = FakeWorld(drift={name: value})
    record = substeps.new_record("fine_5ms")
    with pytest.raises(ValueError, match="readback"):
        substeps.configure_world(world, world.get_settings(), args("fine_5ms"), record)
    assert record["status"] == "FAIL" and record["after"][name] == value and len(world.applied) == 1


def test_world_change_between_original_and_request_fails_before_apply():
    world = FakeWorld()
    original = world.get_settings()
    world.current.no_rendering_mode = True
    record = substeps.new_record("reference_10ms")
    with pytest.raises(ValueError, match="changed before"):
        substeps.configure_world(world, original, args(), record)
    assert not world.applied


def test_apply_error_retains_requested_and_failure():
    world = FakeWorld()
    def fail(_value):
        raise RuntimeError("simulated apply error")
    world.apply_settings = fail
    record = substeps.new_record("reference_10ms")
    with pytest.raises(RuntimeError, match="simulated"):
        substeps.configure_world(world, world.get_settings(), args(), record)
    assert record["requested"] is not None and record["after"] is None and record["status"] == "FAIL"


@pytest.mark.parametrize("key,value", [("goal_stop_profile", "turn_launch_014_v1"),
    ("control_transport", "legacy_async"), ("agent_initialization", "before_bootstrap"),
    ("physics_hz", 40), ("capture_hz", 5), ("physics_substep_profile", "unknown")])
def test_wrong_argument_scope_rejected(key, value):
    value_args = args()
    setattr(value_args, key, value)
    with pytest.raises(ValueError):
        substeps.validate_arguments(value_args)


def test_inherited_does_not_import_helper_or_add_required_route(monkeypatch):
    # HH_260906 - The legacy CLI still accepts a route path without opening it during argument parsing.
    def forbidden(_args):
        raise AssertionError("inherited profile must not call the helper")
    monkeypatch.setattr(substeps, "validate_arguments", forbidden)
    parsed = collector.parse_args(["unused", "not-a-real-route"])
    assert parsed.physics_substep_profile == "inherited"
    assert collector.physics_substep_module(parsed) is None


@pytest.mark.parametrize("profile", substeps.PROFILES)
def test_cli_accepts_only_pinned_profile_route_before_world(profile):
    if not ROUTE.exists():
        pytest.skip("private exact C-track route is not installed")
    assert collector.parse_args(flags(profile)).physics_substep_profile == profile


def test_cli_rejects_changed_route_before_any_capture(tmp_path):
    route = tmp_path / "other-route.json"
    route.write_text(json.dumps({"town": "Town07", "route": []}))
    with pytest.raises((SystemExit, collector.CollectionError, ValueError)):
        collector.parse_args(flags(route=route))
    assert not (tmp_path / "unused").exists()


def test_abbreviated_optin_is_not_accepted():
    argv = flags()
    argv[2] = "--physics-substep-prof"
    with pytest.raises(SystemExit):
        collector.parse_args(argv)


def test_optin_failed_collection_retains_not_reached_and_helper_hash(tmp_path, monkeypatch):
    if not ROUTE.exists():
        pytest.skip("private exact C-track route is not installed")
    output = tmp_path / "episode"
    def failed(*_args, **_kwargs):
        raise collector.CollectionError("mocked connection failure")
    monkeypatch.setattr(collector, "collect_episode", failed)
    with pytest.raises(collector.CollectionError, match="mocked"):
        collector.run(collector.parse_args(flags(output=output)))
    manifest = json.loads(Path(str(output) + ".partial/manifest.json").read_text())
    assert manifest["status"] == "failed"
    assert manifest["capture_contract"]["physics_substeps"] == substeps.new_record("reference_10ms")
    assert manifest["provenance"]["physics_substeps_helper_sha256"] == hashlib.sha256(Path(substeps.__file__).read_bytes()).hexdigest()
    assert manifest["result"]["training_data_approved"] is False


def test_actor_order_and_legacy_branch_preserved():
    source = inspect.getsource(collector.collect_episode)
    assert source.index("substeps.configure_world(") < source.index("world.set_weather(") < source.index("world.try_spawn_actor(")
    assert "settings.fixed_delta_seconds = 1.0 / args.physics_hz" in source
    assert "world.apply_settings(original_settings)" in source


@pytest.mark.parametrize("profile", ["inherited", "reference_10ms", "fine_5ms"])
@pytest.mark.parametrize("equals", [False, True])
def test_owned_wrapper_archives_helper_only_for_optin(harness, profile, equals):
    # HH_260906 - Exercise the real wrapper with fixture-owned fake children and no CARLA process.
    worker = harness["root"] / "scripts/e2e/collect_carla_vad_expert.py"
    source = worker.read_text().replace(" return p.parse_args(argv)",
        " p.add_argument('--physics-substep-profile', choices=('inherited','reference_10ms','fine_5ms'), default='inherited')\n return p.parse_args(argv)")
    worker.write_text(source)
    helper = worker.with_name("carla_physics_substeps.py")
    helper.write_text("# HH_260906 - Fixture-only numerical helper bytes.\n")
    option = [f"--physics-substep-profile={profile}"] if equals else ["--physics-substep-profile", profile]
    result = _run(harness, "--", *option)
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    name = "scripts/e2e/carla_physics_substeps.py"
    assert (name in plan["source_sha256"]) == (profile != "inherited")
    assert ("physics_substeps_source_bytes_archived" in plan) == (profile != "inherited")
    if profile != "inherited":
        assert plan["physics_substep_profile"] == profile
        assert (harness["output"] / "provenance" / name).read_bytes() == helper.read_bytes()
        final = json.loads((harness["output"] / "owner_result.json").read_text())
        assert final["source_checks"][name]
