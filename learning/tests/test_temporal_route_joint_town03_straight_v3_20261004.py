"""HH_261004 - Prove the fresh V3 fixture-schema admission on CPU without CARLA or GPU."""

from contextlib import contextmanager
import json
import os
from pathlib import Path
import socket
import sys
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_route_joint_town03_straight_v3_20261004 as live
from scripts.e2e import research_temporal_route_joint_town03_straight_v3_20261004 as owner


def pins():
    return {name + "_sha256": str(index) * 64
            for index, name in enumerate(("weights", "metadata", "checkpoint", "declaration"), 1)}


def good6000_pins():
    return dict(
        weights_sha256="b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625",
        metadata_sha256="a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
        checkpoint_sha256="43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88",
        declaration_sha256="0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5")


@contextmanager
def owner_scope(values=None):
    threads = owner.previous.previous.runtime_owner.previous.previous.threads
    with mock.patch.object(owner.numeric, "verify_exports"), \
         mock.patch.object(threads, "evidence_assets", return_value={}):
        with owner.activated(pins() if values is None else values):
            yield


def arguments():
    with owner_scope():
        return live.frozen.parse_args(owner.base.capture_argv("v3", True, "a" * 64))


def fake_model_and_patches():
    """HH_261004 - Enter the genuine collector activation only; no inference or simulator API is called."""
    v2 = live.previous
    right = v2.previous.previous
    recovery = right.signed1.previous
    fast45 = recovery.previous
    model = SimpleNamespace(model_id=recovery.MODEL_ID, modules=lambda: [], state_dict=lambda: {},
        _research_speed_preview_calibration=live.PREVIEW_SCHEMA,
        _research_cpu_thread_numerical_revision=live.THREAD_SCHEMA,
        _research_recovery_simulator_control_authorized=True,
        _research_recovery_owner_binding=dict(purpose="owned_carla_research_only", simulation_only=True,
                                               owner_plan_sha256="a" * 64))
    basic = SimpleNamespace(BasicAgent=type("FakeBasicAgent", (), {}))
    patches = (
        mock.patch.object(right.previous.previous.previous, "validate_model_binding"),
        mock.patch.object(recovery, "RouteRecoveryTemporalModelV3", SimpleNamespace),
        mock.patch.object(fast45, "CPUFiniteCachedTemporalModelV2", SimpleNamespace),
        mock.patch.object(right.previous.previous.torch, "get_num_threads", return_value=4),
        mock.patch.object(right.previous.previous.torch, "get_num_interop_threads", return_value=1),
        mock.patch.dict(sys.modules, {"agents": SimpleNamespace(),
            "agents.navigation": SimpleNamespace(basic_agent=basic),
            "agents.navigation.basic_agent": basic}),
    )
    return model, patches


@contextmanager
def entered(patches):
    with patches[0], patches[1], patches[2], patches[3], patches[4], patches[5]:
        yield


class FakeLight:
    def __init__(self):
        self.id, self.is_alive, self.type_id, self.attributes = 17, True, "traffic.traffic_light", {}
        self.state = SimpleNamespace(name="Red")
        self.frozen = False
        self.times = {"green": 11.0, "yellow": 3.0, "red": 13.0}
        self.elapsed = 2.0
        self.transform = SimpleNamespace(
            location=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            rotation=SimpleNamespace(roll=0.0, pitch=0.0, yaw=90.0))

    def get_state(self): return self.state
    def set_state(self, value): self.state = value
    def is_frozen(self): return self.frozen
    def freeze(self, value): self.frozen = bool(value)
    def get_green_time(self): return self.times["green"]
    def get_yellow_time(self): return self.times["yellow"]
    def get_red_time(self): return self.times["red"]
    def set_green_time(self, value): self.times["green"] = value
    def set_yellow_time(self, value): self.times["yellow"] = value
    def set_red_time(self, value): self.times["red"] = value
    def get_transform(self): return self.transform
    def get_elapsed_time(self): return self.elapsed


class FakeActors:
    def __init__(self, light): self.light = light

    def filter(self, pattern):
        return [self.light] if pattern == "*traffic_light*" else []


class FakeWorld:
    def __init__(self):
        self.light = FakeLight()
        self.actors = FakeActors(self.light)
        self.frame, self.elapsed = 100, 5.0
        self.settings = SimpleNamespace(synchronous_mode=True, fixed_delta_seconds=0.05,
            no_rendering_mode=False, substepping=True, max_substep_delta_time=0.01, max_substeps=10)

    def get_map(self): return SimpleNamespace(name="Carla/Maps/Town03")
    def get_actors(self): return self.actors
    def get_settings(self): return self.settings
    def apply_settings(self, value): self.settings = value
    def get_snapshot(self):
        return SimpleNamespace(frame=self.frame, timestamp=SimpleNamespace(elapsed_seconds=self.elapsed))
    def tick(self):
        self.frame += 1
        self.elapsed += 0.05
        return self.frame


def write_actual_owner_guard(tmp_path, schema):
    """HH_261004 - Build actual /proc and listening-socket evidence accepted by the unmocked owner guard."""
    run = tmp_path / "actual_owner"
    run.mkdir()
    plan = dict(schema=schema, town="Town03", scenario="straight", run_id="v3",
        original_navigation_sha256=live.ROUTE_SHA, output=str(run))
    declaration = run / "declaration.json"
    declaration.write_text(json.dumps(plan, sort_keys=True) + "\n")
    expected = owner.digest(declaration)
    current = owner.base.process_record(os.getpid())
    parent = owner.base.process_record(os.getppid())
    assert current is not None and parent is not None
    started = dict(declaration_sha256=expected,
        owner=dict(pid=parent["pid"], start_ticks=parent["start_ticks"]),
        server=dict(pid=current["pid"], start_ticks=current["start_ticks"]),
        socket_owner=dict(pid=current["pid"], start_ticks=current["start_ticks"], pgid=current["pid"]))
    (run / "owner_started.json").write_text(json.dumps(started) + "\n")
    return SimpleNamespace(output=run / "episode", research_source_sha256=expected), expected


def test_v3_contract_changes_only_fixture_schema_admission_and_preserves_both_failures():
    with owner_scope():
        contract = owner.fixed_contract("v3", True)
    assert contract["schema"] == live.OWNER_SCHEMA and contract["run_id"] == "v3"
    assert contract["output"].endswith("town03_known_dev_straight_v3")
    assert contract["failed_v1_archived_reference"]["overwritten"] is False
    assert contract["failed_v2_archived_reference"]["original_capture_error"] == owner.FIXTURE_ERROR
    assert contract["failed_v2_archived_reference"]["native_state_count"] == 0
    assert contract["failed_v2_archived_reference"]["camera_bundle_count"] == 0
    assert contract["failed_v2_archived_reference"]["ACK_receipt_count"] == 0
    change = contract["fixture_owner_schema_admission"]
    assert change["declared_owner_schema"] == live.OWNER_SCHEMA
    assert change["actual_fixture_module_global_required"] is True
    assert change["model_weights_changed"] is change["controller_changed"] is False
    assert change["route_or_map_changed"] is change["camera_or_ACK_changed"] is False
    assert contract["actual_deferred_JPEG_after_ACK_required"] is True
    assert contract["authenticated_start_corridor"]["upstream_lane"] == [67, 0, 2]
    assert contract["total_capture_seconds"] == 45 and contract["maximum_native_state_count"] == 900
    assert contract["maximum_camera_bundle_count"] == 450 and contract["maximum_driving_native_ticks"] == 876
    assert contract["model_speed_cap_mps"] == 2.5 and contract["native_actor_planar_speed_maximum_mps"] == 2.7
    assert contract["policy_deadline_seconds"] == 0.1
    assert contract["controller_target_cap_mps"] == 2.4 and contract["speed_preview_s"] == 0.5
    assert contract["body_clearance_screen"]["lane_inset_m"] == 0.1
    assert contract["worker_command"][contract["worker_command"].index("-m") + 1] == owner.MODULE
    assert not contract["full_RTE01_approved"] and not contract["model_promoted"]


def test_actual_green_fixture_guard_install_restore_uses_v3_schema_and_real_owner_evidence(tmp_path):
    """HH_261004 - Exercise the real guard/class with /proc+socket evidence, synthetic CARLA-shaped objects only."""
    args = arguments()
    model, patches = fake_model_and_patches()
    fixture_module = live.fixture
    before = (live.previous.SCHEMA, live.previous.OWNER_SCHEMA,
              live.straight.SCHEMA, live.straight.OWNER_SCHEMA,
              fixture_module.SCHEMA, fixture_module.OWNER_SCHEMA)
    with owner_scope(), entered(patches), live.activated(model, args):
        assert live._actual_fixture_owner_schema() == live.OWNER_SCHEMA
        assert live.previous.SCHEMA == live.straight.SCHEMA == fixture_module.SCHEMA == live.SCHEMA
        assert live.previous.OWNER_SCHEMA == live.straight.OWNER_SCHEMA \
            == fixture_module.OWNER_SCHEMA == live.OWNER_SCHEMA
        assert fixture_module.SCHEMA == live.SCHEMA and fixture_module.OWNER_SCHEMA == live.OWNER_SCHEMA
        fixture_args, expected = write_actual_owner_guard(tmp_path, live.OWNER_SCHEMA)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(("127.0.0.1", 0)); sock.listen(1)
        port = sock.getsockname()[1]
        saved_plan_sha = fixture_module.numeric._owner_plan_sha
        fixture_module.numeric._owner_plan_sha = expected
        try:
            assert owner.base.listening_socket_owned(os.getpid(), port)
            with mock.patch.object(owner.base, "PORT", port):
                fixture_module.assert_owned_capture(fixture_args)
                world, manifest = FakeWorld(), {}
                original_state = world.light.state
                actual = fixture_module.GreenSignalFixture(
                    world, fixture_args, manifest, SimpleNamespace(name="Green"))
                actual.install()
                assert world.light.state.name == "Green" and world.light.is_frozen() is True
                assert actual.record["state"] == "GREEN_FROZEN_INSTALLED"
                assert actual.record["schema"] == live.SCHEMA + ".owned_green_signal_fixture"
                assert actual.check() == world.frame
                actual.restore()
                assert actual.restored is True and actual.record["state"] == "RESTORED"
                assert world.light.state is original_state and world.light.is_frozen() is False
                assert actual.record["pre_vehicle_fixture_tick"]["outside_capture"] is True
                assert actual.record["post_vehicle_restoration_tick"]["outside_capture"] is True
        finally:
            fixture_module.numeric._owner_plan_sha = saved_plan_sha
            sock.close()
    assert before == (live.previous.SCHEMA, live.previous.OWNER_SCHEMA,
                      live.straight.SCHEMA, live.straight.OWNER_SCHEMA,
                      fixture_module.SCHEMA, fixture_module.OWNER_SCHEMA)
    assert live._active is False and live.previous._active is False and live.straight._active is False


@pytest.mark.parametrize("fault", ["declaration", "schema", "pid"])
def test_actual_green_fixture_guard_rejects_wrong_identity_before_actor_mutation(tmp_path, fault):
    """HH_261004 - Reject three real owner-boundary failures before the fixture touches an actor or clock."""
    args = arguments()
    model, patches = fake_model_and_patches()
    fixture_module = live.fixture
    with owner_scope(), entered(patches), live.activated(model, args):
        fixture_args, expected = write_actual_owner_guard(
            tmp_path, owner.V2_OWNER_SCHEMA if fault == "schema" else live.OWNER_SCHEMA)
        if fault == "declaration":
            fixture_args.research_source_sha256 = "f" * 64
            admitted_sha = fixture_args.research_source_sha256
            expected_error = "declaration bytes differ"
        else:
            admitted_sha = expected
            expected_error = "restricted to the declared original" if fault == "schema" \
                else "worker lacks exact owning parent"
        if fault == "pid":
            started_path = Path(fixture_args.output).parent / "owner_started.json"
            started = json.loads(started_path.read_text())
            current = owner.base.process_record(os.getpid())
            started["owner"] = dict(pid=current["pid"], start_ticks=current["start_ticks"])
            started_path.write_text(json.dumps(started) + "\n")
        saved_plan_sha = fixture_module.numeric._owner_plan_sha
        fixture_module.numeric._owner_plan_sha = admitted_sha
        world, manifest = FakeWorld(), {}
        try:
            with pytest.raises(Exception, match=expected_error):
                fixture_module.GreenSignalFixture(
                    world, fixture_args, manifest, SimpleNamespace(name="Green"))
        finally:
            fixture_module.numeric._owner_plan_sha = saved_plan_sha
        assert world.frame == 100 and world.elapsed == 5.0
        assert world.light.state.name == "Red" and world.light.is_frozen() is False
        assert manifest == {}


@pytest.mark.parametrize("failure", [False, True])
def test_full_v3_schema_scope_restores_every_alias(failure):
    args = arguments()
    model, patches = fake_model_and_patches()
    fixture_module = live.fixture
    before = (live.previous.SCHEMA, live.previous.OWNER_SCHEMA,
              live.straight.SCHEMA, live.straight.OWNER_SCHEMA,
              fixture_module.SCHEMA, fixture_module.OWNER_SCHEMA)
    with owner_scope(), entered(patches):
        def enter():
            with live.activated(model, args):
                assert live._actual_fixture_owner_schema() == live.OWNER_SCHEMA
                if failure:
                    raise RuntimeError("scope failure")
        if failure:
            with pytest.raises(RuntimeError, match="scope failure"):
                enter()
        else:
            enter()
    assert before == (live.previous.SCHEMA, live.previous.OWNER_SCHEMA,
                      live.straight.SCHEMA, live.straight.OWNER_SCHEMA,
                      fixture_module.SCHEMA, fixture_module.OWNER_SCHEMA)


def test_v2_failure_archive_and_all_declared_root_sources_are_exact():
    diagnostic, result, manifest = owner._verify_v2_failure()
    assert diagnostic["actual_native_state_count"] == 0
    assert diagnostic["actual_camera_bundle_count"] == 0
    assert diagnostic["actual_ACK_receipt_count"] == 0
    assert result["worker_exit_code"] == 1 and result["cleanup_errors"] == []
    assert manifest["error"] == owner.FIXTURE_ERROR and manifest["cleanup"]["completed"] is True


def test_runtime_source_closure_includes_new_owner_and_collector():
    with owner_scope(good6000_pins()):
        sources = owner.runtime_sources()
    for module in (live, owner, owner.previous.start_geometry):
        path = Path(module.__file__).resolve()
        assert sources[str(path.relative_to(owner.ROOT))] == owner.digest(path)


def test_real_plan_build_uses_v3_worker_and_current_assets_without_CUDA_or_output():
    output, declaration = owner.paths("v3")
    assert not output.exists() and not declaration.exists()
    with owner.activated(good6000_pins()):
        plan = owner.base.build_plan("v3", True)
    assert plan["schema"] == live.OWNER_SCHEMA and plan["run_id"] == "v3"
    assert plan["worker_command"][plan["worker_command"].index("-m") + 1] == owner.MODULE
    assert plan["source_sha256"]["tools/e2e/collect_temporal_route_joint_town03_straight_v3_20261004.py"] \
        == owner.digest(Path(live.__file__))
    assert plan["source_sha256"]["tools/e2e/research_temporal_route_joint_town03_straight_v3_20261004.py"] \
        == owner.digest(Path(owner.__file__))
    assert plan["failed_v2_archived_reference"]["failure_before_any_frame"] is True
    assert plan["actual_deferred_JPEG_after_ACK_required"] is True
    assert not output.exists() and not declaration.exists()


def test_diagnostic_never_turns_v2_archive_into_approval(tmp_path, monkeypatch):
    inherited = dict(learned_driving_screen_passed=True,
                     straight_motion_body_policy_wall100ms_passed=True)
    monkeypatch.setattr(owner, "_parent_diagnostic", lambda *args: dict(inherited))
    result = owner.diagnostic_report(tmp_path, {}, {})
    assert result["archived_v2_fixture_failure_preserved"] is True
    assert result["learned_driving_screen_passed"] is True
    assert result["status"] == "PASS_KNOWN_DEV_STRAIGHT_PRECURSOR"
    assert result["full_RTE01_approved"] is False
    assert result["full_MAIN9_completion_claimed"] is False
    assert result["training_data_approved"] is False
    assert result["vehicle_control_approved"] is False
    assert result["model_promoted"] is False


def test_paths_are_fresh_create_only_and_other_versions_fail():
    output, declaration = owner.paths("v3")
    assert output.name.endswith("known_dev_straight_v3")
    assert declaration.name.endswith("known_dev_straight_v3_plan.json")
    assert output != owner.V2_OUTPUT and not output.exists() and not declaration.exists()
    for value in ("v1", "v2", "../v3", True):
        with pytest.raises(Exception):
            owner.paths(value)
