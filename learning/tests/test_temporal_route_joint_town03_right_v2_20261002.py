"""HH_261002 - Pure synthetic signal/namespace/owner tests; no CARLA, model, datasets, or actuation."""
from contextlib import contextmanager, ExitStack
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_route_joint_town03_right_v2_20261002 as live
from scripts.e2e import research_temporal_route_joint_town03_right_v2_20261002 as owner


def pins(): return {k+"_sha256": str(i)*64 for i, k in enumerate(("weights", "metadata", "checkpoint", "declaration"), 1)}


@contextmanager
def owner_scope():
    with mock.patch.object(owner.previous.numeric, "verify_exports"), mock.patch.object(owner.previous.threads, "evidence_assets", return_value={}):
        with owner.activated(pins()): yield


def capture_args():
    with owner_scope(): return live.frozen.parse_args(owner.capture_argv("v2", True, "a"*64))


class State:
    def __init__(self, name): self.name = name


GREEN, RED, YELLOW = State("Green"), State("Red"), State("Yellow")


class Light:
    def __init__(self, identity, state=RED, frozen=False):
        self.id, self.is_alive, self.state, self.frozen = identity, True, state, frozen
        self.times = [8., 3., 9.]; self.calls = []
    def get_state(self): return self.state
    def set_state(self, value): self.calls.append(("state", value.name)); self.state = value
    def is_frozen(self): return self.frozen
    def freeze(self, value): self.calls.append(("freeze", value)); self.frozen = value
    def get_green_time(self): return self.times[0]
    def get_yellow_time(self): return self.times[1]
    def get_red_time(self): return self.times[2]
    def set_green_time(self, value): self.times[0] = value
    def set_yellow_time(self, value): self.times[1] = value
    def set_red_time(self, value): self.times[2] = value
    def get_elapsed_time(self): return 2.
    def get_transform(self):
        return SimpleNamespace(location=SimpleNamespace(x=1., y=2., z=3.), rotation=SimpleNamespace(yaw=0., pitch=0., roll=0.))


class Actors:
    def __init__(self, lights): self.lights = lights
    def filter(self, pattern): return self.lights if pattern == "*traffic_light*" else []


class World:
    def __init__(self):
        self.lights = [Light(2, YELLOW, False), Light(1, RED, False)]; self.frame = 50; self.town = "Town03"
        self.settings = SimpleNamespace(synchronous_mode=True, fixed_delta_seconds=.05, no_rendering_mode=False,
            substepping=True, max_substep_delta_time=.01, max_substeps=5)
        self.freeze_calls = []
        for light in self.lights:
            light.freeze = lambda value: self.global_freeze(value)
    def global_freeze(self, value):
        self.freeze_calls.append(value)
        for light in self.lights: light.frozen = value
    def get_settings(self): return SimpleNamespace(**vars(self.settings))
    def apply_settings(self, value): self.settings = SimpleNamespace(**vars(value))
    def tick(self): self.frame += 1; return self.frame
    def get_actors(self): return Actors(self.lights)
    def get_map(self): return SimpleNamespace(name="/Game/Maps/"+self.town)
    def get_snapshot(self): return SimpleNamespace(frame=self.frame, timestamp=SimpleNamespace(elapsed_seconds=self.frame*.05))


def fixture(world=None, manifest=None):
    world = World() if world is None else world
    with mock.patch.object(live, "assert_owned_capture"):
        return live.GreenSignalFixture(world, SimpleNamespace(), {} if manifest is None else manifest, GREEN)


def test_exact_v1_sources_and_new_public_parser():
    live.verify_frozen()
    assert owner.previous.digest(Path(owner.previous.__file__)) == owner.V1_SHA
    args = capture_args(); live.validate_capture(args, live.collector.load_route(args.route_file))
    assert args.observe_traffic_lights and args.max_duration_sec == 110 and args.seed == 2026102500


@pytest.mark.parametrize("name,value", [("observe_traffic_lights", False), ("observe_traffic_lights", 1), ("port", 2000),
    ("max_duration_sec", 111), ("stationary_tail_sec", 1), ("research_speed_cap_mps", 2.6), ("seed", 2026102501)])
def test_no_original_threshold_seed_or_signal_witness_relaxation(name, value):
    args = capture_args(); setattr(args, name, value)
    with pytest.raises(Exception): live.validate_capture(args, live.collector.load_route(args.route_file))


def test_all_original_signals_installed_and_exact_state_freeze_times_restored():
    f = fixture(); original = [(a.get_state(), a.is_frozen(), list(a.times)) for a in f.actors]
    with mock.patch.object(live, "assert_owned_capture"):
        f.install(); assert all(a.get_state() is GREEN and a.is_frozen() for a in f.actors)
        assert f.check() == 51; f.restore()
    assert [(a.get_state(), a.is_frozen(), a.times) for a in f.actors] == original
    assert f.record["original_state_freeze_and_duration_restored"] and f.record["elapsed_signal_phase_not_rewound"]
    assert f.world.freeze_calls == [True, False] and f.world.frame == 52


@pytest.mark.parametrize("kind", ["town", "empty", "duplicate", "dead", "boolean", "nonfinite", "unknownstate"])
def test_fixture_census_fail_closed_before_any_mutation(kind):
    world = World()
    if kind == "town": world.town = "Town07"
    if kind == "empty": world.lights = []
    if kind == "duplicate": world.lights[1].id = 2
    if kind == "dead": world.lights[1].is_alive = False
    if kind == "boolean": world.lights[1].id = True
    if kind == "nonfinite": world.lights[1].times[1] = float("nan")
    if kind == "unknownstate": world.lights[1].state = State("Mystery")
    with pytest.raises(Exception): fixture(world)
    assert all(not a.calls for a in world.lights)


@pytest.mark.parametrize("kind", ["red", "unfrozen", "inventory", "frame"])
def test_actual_fixture_readback_rejects_tampering(kind):
    f = fixture()
    with mock.patch.object(live, "assert_owned_capture"):
        f.install()
        if kind == "red": f.actors[0].state = RED
        if kind == "unfrozen": f.actors[0].frozen = False
        if kind == "inventory": f.world.lights.append(Light(3))
        if kind == "frame": f.world.frame -= 1
        with pytest.raises(Exception): f.check()


def test_private_actuation_owner_required_before_any_mutation():
    with pytest.raises(Exception): live.GreenSignalFixture(World(), SimpleNamespace(research_source_sha256="a"*64), {}, GREEN)


def test_exact_owner_declaration_and_generation_guard(tmp_path):
    args = SimpleNamespace(research_source_sha256="a"*64, output=tmp_path/"episode")
    plan = dict(schema=live.OWNER_SCHEMA, town="Town03", original_navigation_sha256=live.ROUTE_SHA)
    with mock.patch.object(live.numeric, "_owner_plan_sha", "a"*64), mock.patch.object(live.numeric, "_exports", pins()), \
        mock.patch.object(live.numeric.base, "digest", return_value="a"*64), mock.patch.object(live.numeric.base, "read_json", return_value=plan), \
        mock.patch.object(live.numeric.base, "assert_owned_worker") as guard:
        live.assert_owned_capture(args); guard.assert_called_once_with(plan, "a"*64)
        plan["town"] = "Town07"
        with pytest.raises(Exception): live.assert_owned_capture(args)


def agent_fixture(tmp_path, affected=False):
    f = fixture(); f.check = lambda: 50
    vehicle_result = (False, None, -1); light_result = (affected, f.actors[0] if affected else None)
    calls = []
    control = SimpleNamespace(throttle=.11, steer=0., brake=0., reverse=False, hand_brake=False, gear=1, manual_gear_shift=True)
    class Original:
        _ignore_traffic_lights = _ignore_vehicles = False
        def _vehicle_obstacle_detected(self, *args, **kwargs): calls.append("vehicle"); return vehicle_result
        def _affected_by_traffic_light(self, *args, **kwargs): calls.append("traffic_light"); return light_result
        def run_step(self):
            a = self._vehicle_obstacle_detected(); b = self._affected_by_traffic_light()
            if a[0] or b[0]: control.throttle = 0.; control.brake = .5
            return control
    Agent = live.hazard_agent(Original, lambda: f, tmp_path)
    return Agent(), calls, vehicle_result, light_result, control


@pytest.mark.parametrize("affected", [False, True])
def test_original_detectors_called_once_same_tuple_and_hazard_control_not_changed(tmp_path, affected):
    agent, calls, vr, lr, control = agent_fixture(tmp_path, affected)
    assert agent.run_step() is control and calls == ["vehicle", "traffic_light"]
    assert control.brake == (.5 if affected else 0.) and control.throttle == (0. if affected else .11)
    agent._hazards = []; assert agent._vehicle_obstacle_detected() is vr and agent._affected_by_traffic_light() is lr
    witness = json.loads((tmp_path/"original_BasicAgent_hazard_decisions.jsonl").read_text())
    assert witness["frame"] == 50 and witness["final_BasicAgent_output_before_learned_mux"]["brake"] == control.brake
    assert witness["final_learned_mux_ACK_is_separately_recorded"] and not witness["original_decisions"][0]["affected"]


def test_ignored_hazard_authority_and_duplicate_calls_reject(tmp_path):
    agent, _, _, _, _ = agent_fixture(tmp_path); agent._ignore_traffic_lights = True
    with pytest.raises(Exception): agent.run_step()
    assert not (tmp_path/"original_BasicAgent_hazard_decisions.jsonl").exists()


@pytest.mark.parametrize("fails", [False, True])
def test_real_copied_namespace_callbacks_late_bound_and_restored_before_capture(tmp_path, fails):
    partial = tmp_path/"episode.partial"; partial.mkdir(); manifest = {}
    original_blank, original_check, original_spawn = object(), object(), object()
    scoped_blank, scoped_spawn = object(), object()
    manual_check = live.previous.signed1.manual.checked_first_gear_initialization_observation
    namespace = dict(_blank_brake_control=original_blank, checked_agent_initialization_observation=original_check,
                     apply_spawn_z_offset=original_spawn, _preflight_exclusive=lambda w: None,
                     verify_live_opendrive_sha256=lambda m, sha: (dict(sha=sha), b"same"))
    exec("def actual(*args, **kwargs):\n    assert _blank_brake_control is EXPECT_BLANK\n    assert checked_agent_initialization_observation is EXPECT_CHECK\n    assert apply_spawn_z_offset is EXPECT_SPAWN\n    if FAILS: raise RuntimeError('original failure')\n    return 'same result'", namespace)
    namespace.update(EXPECT_BLANK=scoped_blank, EXPECT_CHECK=manual_check, EXPECT_SPAWN=scoped_spawn, FAILS=fails)
    function = namespace["actual"]
    def factory():
        def wrapped(*args, **kwargs): return function(*args, **kwargs)
        return wrapped
    basic = SimpleNamespace(BasicAgent=type("Original", (), {})); original_agent = basic.BasicAgent
    modules = {"agents": SimpleNamespace(), "agents.navigation": SimpleNamespace(basic_agent=basic)}
    with mock.patch.object(live, "_original_factory", factory), mock.patch.object(live.previous, "run_event_capture", side_effect=lambda f,*a,**kw:f(*a,**kw)), \
        mock.patch.object(live.collector, "_blank_brake_control", scoped_blank), mock.patch.object(live.collector, "apply_spawn_z_offset", scoped_spawn), \
        mock.patch.object(live.collector, "checked_agent_initialization_observation", manual_check), mock.patch.dict("sys.modules", modules):
        capture = live.capture_factory()
        if fails:
            with pytest.raises(RuntimeError, match="original failure"): capture(None, None, None, partial, None, None, manifest)
        else: assert capture(None, None, None, partial, None, None, manifest) == "same result"
    assert namespace["_blank_brake_control"] is original_blank and namespace["checked_agent_initialization_observation"] is original_check
    assert namespace["apply_spawn_z_offset"] is original_spawn and basic.BasicAgent is original_agent


def test_new_owner_fixed_contract_restore_and_real_build_plan_preflight():
    before = (owner.base.MODULE, owner.base.live, owner.numeric.MODULE, owner.previous.paths, live.previous.SCHEMA)
    imported = SimpleNamespace(stdout=str(owner.base.EGG)+"/carla.so\n"+str(owner.previous.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n1.8.0\n")
    with owner_scope(), ExitStack() as stack:
        contract = owner.fixed_contract("v2", True)
        assert contract["schema"] == live.OWNER_SCHEMA and contract["total_capture_seconds"] == 110
        assert contract["original_launch_wall_limit_seconds"] == 8. and contract["speed_preview_s"] == .5
        assert contract["model_speed_cap_mps"] == 2.5 and contract["policy_deadline_seconds"] == .1
        assert contract["owned_green_signal_fixture"]["ignore_traffic_lights"] is False
        assert contract["actual_manual_first_gear_bootstrap_and_engagement_required"]
        assert "--observe-traffic-lights" in contract["capture_argv"] and contract["worker_command"][4] == owner.MODULE
        stack.enter_context(mock.patch.object(owner.base, "digest", return_value="a"*64))
        stack.enter_context(mock.patch.object(owner.base, "runtime_sources", return_value={"code":"a"*64}))
        stack.enter_context(mock.patch.object(owner.base, "external_sources", return_value={"API":"a"*64}))
        stack.enter_context(mock.patch.object(owner.base, "ASSETS", {p:"a"*64 for p in owner.base.ASSETS}))
        plan = owner.base.build_plan("v2", True)
        args = live.frozen.parse_args(owner.capture_argv("v2", True, "a"*64)); live.frozen.validate_capture(args, live.collector.load_route(args.route_file))
        stack.enter_context(mock.patch.object(owner.base, "verify_plan", return_value=plan))
        stack.enter_context(mock.patch.object(owner.base.lifecycle, "_assert_no_other_carla"))
        stack.enter_context(mock.patch.object(owner.base, "ports_free"))
        stack.enter_context(mock.patch.object(owner.base, "disk_check", return_value={}))
        stack.enter_context(mock.patch.object(owner.base.subprocess, "run", return_value=imported))
        stack.enter_context(mock.patch.object(owner.base, "load_npz", return_value=SimpleNamespace(_research_temporal_checkpoint_sha256=pins()["checkpoint_sha256"])))
        _, result = owner.base.preflight("v2", "a"*64)
        assert result["status"] == "PREFLIGHT_PASS_NO_SIMULATOR"
    assert before == (owner.base.MODULE, owner.base.live, owner.numeric.MODULE, owner.previous.paths, live.previous.SCHEMA)


def test_original_failure_unchanged_no_new_thresholds():
    assert hashlib.sha256(Path(live.previous.__file__).read_bytes()).hexdigest() == live.V1_SHA
    assert owner.previous.digest(Path(owner.previous.__file__)) == owner.V1_SHA
    with owner_scope():
        c = owner.fixed_contract("v2", True)
    assert c["precursor_motion_screen"]["net_learned_progress_minimum_m"] == 90
    assert c["additive_native_speed_rate_guard_mps2"]["maximum"] == 3.
    assert c["additive_native_speed_rate_guard_mps2"]["minimum"] == -6.
    assert c["event"]["current_unclamped_route_arc_minimum_m"] == previous_exit()+10


def previous_exit(): return live.EXIT


def test_world_global_freeze_rejects_heterogeneous_prior_flags_without_mutating():
    world = World(); world.lights[0].frozen = True
    with pytest.raises(Exception, match="heterogeneous"): fixture(world)
    assert world.freeze_calls == [] and all(not a.calls for a in world.lights)


class CachedWorld(World):
    def __init__(self):
        super().__init__(); self.pending = {}; self.pending_frozen = None
        for light in self.lights:
            light.set_state = lambda value, actor=light: self.pending.setdefault(actor.id, {}).update(state=value)
            light.freeze = lambda value: self.freeze_pending(value)
            for i, name in enumerate(("green", "yellow", "red")):
                setattr(light, "set_"+name+"_time", lambda value, actor=light, index=i:
                    self.pending.setdefault(actor.id, {}).update({index:value}))
    def freeze_pending(self, value): self.freeze_calls.append(value); self.pending_frozen = value
    def tick(self):
        assert self.settings.synchronous_mode and self.settings.fixed_delta_seconds == .05
        for light in self.lights:
            row = self.pending.get(light.id, {})
            if "state" in row: light.state = row["state"]
            for i in range(3):
                if i in row: light.times[i] = row[i]
            if self.pending_frozen is not None: light.frozen = self.pending_frozen
        self.pending = {}; self.pending_frozen = None
        return super().tick()


def test_cached_signal_setters_require_real_fresh_tick_and_restore_original_async_physics():
    world = CachedWorld(); f = fixture(world)
    with mock.patch.object(live, "assert_owned_capture"):
        f.install()
        assert all(a.state is GREEN and a.frozen for a in world.lights) and world.frame == 51
        # HH_261002 - The original collector restores async physics before the separately owned suffix cache tick.
        world.settings.synchronous_mode = False; world.settings.fixed_delta_seconds = None
        original = vars(world.get_settings()).copy()
        f.restore()
    assert vars(world.get_settings()) == original and world.frame == 52
    assert [(a.state.name, a.frozen) for a in world.lights] == [("Yellow", False), ("Red", False)]
    assert world.freeze_calls == [True, False]
    assert f.record["pre_vehicle_fixture_tick"]["counted_as_NN_or_capture_native"] is False
    assert f.record["post_vehicle_restoration_tick"]["outside_capture"] is True


def test_partial_install_error_retains_saved_census_and_can_restore_all():
    world = CachedWorld(); f = fixture(world); original_set = f.actors[1].set_state
    def fail(value):
        f.actors[1].set_state = original_set
        raise RuntimeError("original RPC failure")
    f.actors[1].set_state = fail
    with mock.patch.object(live, "assert_owned_capture"):
        with pytest.raises(RuntimeError, match="original RPC failure"): f.install()
        f.restore()
    assert f.restored and [(a.state.name, a.frozen) for a in world.lights] == [("Yellow", False), ("Red", False)]


def test_original_event_AST_plus_exactly_one_pre_vehicle_sync_fixture_hook():
    with mock.patch.object(live.previous, "_observer", SimpleNamespace(completed=False)):
        capture = live.fixture_event_factory()
    import inspect
    function = inspect.getclosurevars(capture).nonlocals["function"]
    assert "install_green_fixture" in function.__code__.co_names
    assert function.__globals__["install_green_fixture"] is live.install_green_fixture


def test_fixture_tick_refuses_async_or_nonempty_world():
    f = fixture(); f.world.settings.synchronous_mode = False
    with mock.patch.object(live, "assert_owned_capture"), pytest.raises(Exception, match="synchronous20Hz"):
        f.install()


def test_REAL_full_v2_activation_chain_and_final_restore_without_actual_capture():
    parent = live.previous; recovery = parent.signed1.previous; fast45 = recovery.previous
    basic = SimpleNamespace(BasicAgent=type("FakeBasicAgent", (), {}))
    fake = SimpleNamespace(model_id=recovery.MODEL_ID, modules=lambda:[], state_dict=lambda:{},
        _research_speed_preview_calibration=live.PREVIEW_SCHEMA, _research_cpu_thread_numerical_revision=live.THREAD_SCHEMA,
        _research_recovery_simulator_control_authorized=True,
        _research_recovery_owner_binding=dict(purpose="owned_carla_research_only", simulation_only=True, owner_plan_sha256="a"*64))
    before = (parent.cloned_collect_episode, parent.SCHEMA, parent.collector.collect_episode, parent.collector._blank_brake_control,
              parent.collector.checked_agent_initialization_observation, basic.BasicAgent)
    with mock.patch.object(parent.previous.previous.previous, "validate_model_binding"), \
        mock.patch.object(recovery, "RouteRecoveryTemporalModelV3", SimpleNamespace), \
        mock.patch.object(fast45, "CPUFiniteCachedTemporalModelV2", SimpleNamespace), \
        mock.patch.object(parent.previous.previous.torch, "get_num_threads", return_value=4), \
        mock.patch.object(parent.previous.previous.torch, "get_num_interop_threads", return_value=1), \
        mock.patch.dict("sys.modules", {"agents":SimpleNamespace(), "agents.navigation":SimpleNamespace(basic_agent=basic),
            "agents.navigation.basic_agent":basic}):
        with live.activated(fake, capture_args()):
            assert parent.generic.TemporalLiveResearchSessionV2.__name__ == "KnownDevRightObservedSession"
            assert parent.collector.checked_agent_initialization_observation is parent.signed1.manual.checked_first_gear_initialization_observation
            assert parent.cloned_collect_episode is live.capture_factory and parent.SCHEMA == live.SCHEMA
            assert callable(parent.collector.collect_episode)
    assert before == (parent.cloned_collect_episode, parent.SCHEMA, parent.collector.collect_episode, parent.collector._blank_brake_control,
                      parent.collector.checked_agent_initialization_observation, basic.BasicAgent)
    f = fixture()
    with mock.patch.object(live, "assert_owned_capture"), mock.patch.object(live.collector, "_preflight_exclusive", side_effect=RuntimeError("foreign vehicle")), \
        pytest.raises(RuntimeError, match="foreign vehicle"):
        f.install()
