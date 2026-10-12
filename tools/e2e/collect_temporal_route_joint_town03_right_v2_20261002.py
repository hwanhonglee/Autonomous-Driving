"""HH_261002 - New owned Green-signal basic-turn fixture; preserve v1 and original hazard decisions."""
from __future__ import annotations

from contextlib import contextmanager
import ast
import copy
import hashlib
import inspect
import json
import math
from pathlib import Path
import sys

from . import collect_temporal_route_joint_town03_right_20261002 as previous
from . import research_temporal_route_joint_signed45_town07_20261002 as numeric

frozen, collector = previous.frozen, previous.collector
ROOT, SEED = frozen.ROOT, previous.SEED
SCHEMA = "portable_e2e.actual_route_joint_preview05_town03_known_dev_right.20261002.v2"
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_right.20261002.v2"
PREVIEW_SCHEMA, THREAD_SCHEMA = previous.PREVIEW_SCHEMA, previous.THREAD_SCHEMA
ROUTE_SHA, ROUTE_LENGTH = previous.ROUTE_SHA, previous.ROUTE_LENGTH
ENTRY, EXIT, EXPECTED_TURN_DEG = previous.ENTRY, previous.EXIT, previous.EXPECTED_TURN_DEG
V1_SHA = "f5eff0f8da70533b94748ef10718d55fed63af9974ec1e2a080708ec4a46bf97"
XODR_SHA = "abf177bcd9b66cfefb7ac4919f6456815f1384f4229f1343c8a6a90bbb7713c9"
_active = False
_parent_factory = previous.cloned_collect_episode


def verify_frozen():
    frozen._require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == V1_SHA,
                    "failed v1 collector must remain byte exact")
    previous.verify_frozen()


def validate_capture(args, route):
    previous.validate_capture(args, route)
    frozen._require(getattr(args, "observe_traffic_lights", False) is True,
                    "new v2 actual signal witness must be explicitly enabled")


def assert_owned_capture(args):
    # HH_261002 - Recheck the existing exact parent/server/socket PID generations before fixture mutations/restoration.
    expected = args.research_source_sha256
    frozen._require(numeric._owner_plan_sha == expected and numeric._exports is not None,
                    "fixture requires the private actuating owner, never offline preflight")
    declaration = Path(args.output).parent / "declaration.json"
    frozen._require(numeric.base.digest(declaration) == expected, "owned fixture declaration bytes differ")
    plan = numeric.base.read_json(declaration)
    frozen._require(plan.get("schema") == OWNER_SCHEMA and plan.get("town") == "Town03"
                    and plan.get("original_navigation_sha256") == ROUTE_SHA,
                    "fixture is restricted to the declared original Town03 owner")
    numeric.base.assert_owned_worker(plan, expected)


def finite(value):
    frozen._require(type(value) in (int, float) and math.isfinite(value), "finite nonboolean fixture value required")
    return float(value)


def light_state(light):
    state = light.get_state(); name = getattr(state, "name", None)
    if not isinstance(name, str): name = str(state).rsplit(".", 1)[-1]
    frozen._require(name in {"Red", "Yellow", "Green", "Off", "Unknown"}, "unknown traffic enum")
    return state, name


def actor_id(actor):
    value = actor.id
    frozen._require(type(value) is int and value > 0 and actor.is_alive is True,
                    "live exact positive actor identity required")
    return value


class GreenSignalFixture:
    """HH_261002 - Change only original owned traffic actors, never disable or replace a hazard detector."""
    def __init__(self, world, args, manifest, green):
        assert_owned_capture(args)
        frozen._require(world.get_map().name.rsplit("/", 1)[-1] == "Town03", "wrong fixture world")
        collector._preflight_no_walkers(world)
        self.world, self.args, self.manifest, self.green = world, args, manifest, green
        self.actors = tuple(sorted(world.get_actors().filter("*traffic_light*"), key=actor_id))
        frozen._require(self.actors and len({actor_id(a) for a in self.actors}) == len(self.actors),
                        "complete nonempty unique original light inventory required")
        self.saved = []
        self.record = dict(schema=SCHEMA+".owned_green_signal_fixture", state="PREPARED", original_actor_ids=[],
            configured_before_vehicle_spawn=True, ignore_traffic_lights=False, detector_returns_modified=False,
            v1_hazard_light_vs_vehicle_cause_proven=False, traffic_rules_performance_claimed=False,
            original_hazard_braking_preserved=True, elapsed_signal_phase_not_rewound=True,
            actual_native_plus3_minus6_guards_unchanged=True)
        manifest.setdefault("capture_contract", {})["owned_green_signal_fixture"] = self.record
        # HH_261002 - Census all originals before the first mutation so partial installation can also restore every touched actor.
        for light in self.actors:
            state, name = light_state(light); frozen_value = light.is_frozen()
            frozen._require(type(frozen_value) is bool, "actual frozen-state bool required")
            times = tuple(finite(getattr(light, "get_"+name+"_time")()) for name in ("green", "yellow", "red"))
            frozen._require(all(t >= 0 for t in times), "nonnegative original signal durations required")
            transform = collector._transform_dict(light.get_transform())
            frozen._require(all(math.isfinite(v) for v in transform.values()), "finite signal pose required")
            self.saved.append((light, state, frozen_value, times))
            self.record["original_actor_ids"].append(dict(actor_id=actor_id(light), state=name,
                frozen=frozen_value, green_yellow_red_seconds=list(times), elapsed_seconds=finite(light.get_elapsed_time()),
                actor_transform_carla=transform))
        frozen._require(len({row[2] for row in self.saved}) == 1,
                        "CARLA freeze is world-global: heterogeneous original frozen flags cannot be restored")
        self.original_frozen = self.saved[0][2]
        self.frame = None; self.readback_count = 0; self.restored = False

    def fresh_tick(self, role):
        # HH_261002 - SetState/Freeze getters use cached snapshots; one owned empty-world20Hz tick is explicitly outside capture.
        assert_owned_capture(self.args); collector._preflight_exclusive(self.world); collector._preflight_no_walkers(self.world)
        settings = self.world.get_settings()
        frozen._require(settings.synchronous_mode is True and abs(float(settings.fixed_delta_seconds)-.05) <= 1.e-12,
                        "fixture ticks require explicit synchronous20Hz, never asynchronous tick assumptions")
        before = self.world.get_snapshot(); frame = self.world.tick(); after = self.world.get_snapshot()
        frozen._require(type(frame) is int and frame == before.frame+1 == after.frame,
                        "fixture fresh snapshot is not one exact owned native interval")
        frozen._require(abs(finite(after.timestamp.elapsed_seconds)-finite(before.timestamp.elapsed_seconds)-.05) <= 1.e-6,
                        "fixture-only fresh native timestamp is not20Hz")
        self.record[role] = dict(before_frame=before.frame, after_frame=frame,
            before_timestamp=finite(before.timestamp.elapsed_seconds), after_timestamp=finite(after.timestamp.elapsed_seconds),
            outside_capture=True, vehicle_or_sensor_existed=False, counted_as_NN_or_capture_native=False)

    def install(self):
        assert_owned_capture(self.args)
        self.actors[0].freeze(True)
        for light in self.actors: light.set_state(self.green)
        self.fresh_tick("pre_vehicle_fixture_tick")
        self.check(); self.record["state"] = "GREEN_FROZEN_INSTALLED"

    def check(self):
        ids = tuple(sorted(actor_id(a) for a in self.world.get_actors().filter("*traffic_light*")))
        frozen._require(ids == tuple(actor_id(a) for a in self.actors), "fixture actor inventory/generation changed")
        frame = self.world.get_snapshot().frame
        frozen._require(type(frame) is int and (self.frame is None or frame >= self.frame), "fixture frame regressed")
        for light in self.actors:
            frozen._require(light_state(light)[1] == "Green" and light.is_frozen() is True,
                            "actual original signal is not Green/frozen")
            finite(light.get_elapsed_time())
        frozen._require(self.world.get_snapshot().frame == frame, "signal readback crossed physics frames")
        self.frame = frame; self.readback_count += 1
        self.record.update(last_readback_frame=frame, readback_count=self.readback_count)
        return frame

    def restore(self):
        assert_owned_capture(self.args)
        frozen._require(tuple(sorted(actor_id(a) for a in self.world.get_actors().filter("*traffic_light*")))
                        == tuple(actor_id(a) for a in self.actors), "refuse restoration into a changed actor inventory")
        errors = []; original_settings = self.world.get_settings()
        # HH_261002 - The frozen collector has already restored its original settings and destroyed its actors.
        # Temporarily select20Hz only for the separate restoration snapshot, then restore exact original settings in finally.
        try:
            collector._preflight_exclusive(self.world); collector._preflight_no_walkers(self.world)
            settings = self.world.get_settings(); settings.synchronous_mode=True; settings.fixed_delta_seconds=.05
            self.world.apply_settings(settings)
            for light, state, frozen_value, times in self.saved:
                for name, value in zip(("green", "yellow", "red"), times): getattr(light, "set_"+name+"_time")(value)
                light.set_state(state)
            self.actors[0].freeze(self.original_frozen)
            self.fresh_tick("post_vehicle_restoration_tick")
            for light, state, frozen_value, times in self.saved:
                frozen._require(light.get_state() == state and light.is_frozen() is frozen_value
                    and tuple(finite(getattr(light, "get_"+n+"_time")()) for n in ("green", "yellow", "red")) == times,
                    "original signal state/freeze/durations not restored")
        except Exception as error: errors.append(type(error).__name__+": "+str(error))
        finally:
            self.world.apply_settings(original_settings)
            frozen._require(collector._settings_dict(self.world.get_settings()) == collector._settings_dict(original_settings),
                            "original physics settings not restored after the fixture-only tick")
        self.restored = not errors; self.record.update(state="RESTORED" if self.restored else "RESTORE_FAILED",
            original_state_freeze_and_duration_restored=self.restored, restore_errors=errors)
        frozen._require(self.restored, "signal fixture restoration failed")


def hazard_agent(original, fixture_provider, partial):
    # HH_261002 - Observe each original detector exactly once, in BasicAgent's original order; return the same tuple object.
    class HazardReasonStartup(original):
        def _vehicle_obstacle_detected(self, *args, **kwargs):
            result = super()._vehicle_obstacle_detected(*args, **kwargs)
            frozen._require(type(result) is tuple and len(result) == 3 and type(result[0]) is bool,
                            "original vehicle detector ABI differs")
            self._hazards.append(dict(kind="vehicle", affected=result[0], actor_id=actor_id(result[1]) if result[1] is not None else None,
                                      distance_m=finite(result[2])))
            return result
        def _affected_by_traffic_light(self, *args, **kwargs):
            result = super()._affected_by_traffic_light(*args, **kwargs)
            frozen._require(type(result) is tuple and len(result) == 2 and type(result[0]) is bool,
                            "original light detector ABI differs")
            self._hazards.append(dict(kind="traffic_light", affected=result[0],
                actor_id=actor_id(result[1]) if result[1] is not None else None,
                actual_state=light_state(result[1])[1] if result[1] is not None else None))
            return result
        def run_step(self, *args, **kwargs):
            fixture = fixture_provider()
            frozen._require(fixture is not None, "verified fixture must precede every agent decision")
            frame = fixture.check(); self._hazards = []
            result = super().run_step(*args, **kwargs)
            frozen._require([r["kind"] for r in self._hazards] == ["vehicle", "traffic_light"]
                and fixture.world.get_snapshot().frame == frame, "hazard decisions missing/duplicated or crossed frame")
            frozen._require(self._ignore_traffic_lights is False and self._ignore_vehicles is False,
                            "original hazard authority was disabled")
            with (partial/"original_BasicAgent_hazard_decisions.jsonl").open("a") as stream:
                stream.write(json.dumps(dict(schema=SCHEMA+".original_hazard_decisions", frame=frame,
                    original_decisions=self._hazards, final_BasicAgent_output_before_learned_mux=collector.control_dict(result),
                    final_learned_mux_ACK_is_separately_recorded=True, detector_results_returned_byte_unmodified=True,
                    original_hazard_braking_unchanged=True), allow_nan=False)+"\n")
            return result
    return HazardReasonStartup


def install_green_fixture(world, args, partial, manifest):
    raise collector.CollectionError("actual capture must bind its owned pre-vehicle fixture callback")


def install_fixture_AST(changed):
    original = copy.deepcopy(changed); count = 0
    class Insert(ast.NodeTransformer):
        def visit_Assign(self, node):
            nonlocal count
            if ast.unparse(node) == "blueprint_library = world.get_blueprint_library()":
                count += 1
                return [ast.parse("install_green_fixture(world, args, partial, manifest)").body[0], node]
            return self.generic_visit(node)
    result = Insert().visit(changed)
    frozen._require(count == 1, "exact sync-configured pre-vehicle blueprint seam required")
    class Undo(ast.NodeTransformer):
        def visit_Expr(self, node):
            if ast.unparse(node) == "install_green_fixture(world, args, partial, manifest)": return None
            return self.generic_visit(node)
    frozen._require(ast.dump(Undo().visit(copy.deepcopy(result))) == ast.dump(original),
                    "another v1 event/capture AST statement changed")
    return result


def fixture_event_factory():
    # HH_261002 - Reuse v1's full AST/inverse proof; add one new inverse-proven pre-vehicle hook only after that proof.
    verify_frozen()
    tree = ast.parse(Path(previous.__file__).read_bytes())
    node = copy.deepcopy(next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "cloned_collect_episode"))
    found = 0
    for index, statement in enumerate(node.body):
        if isinstance(statement, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "namespace" for t in statement.targets):
            node.body.insert(index, ast.parse("changed = install_fixture_AST(changed)").body[0])
            statement.value.keywords.append(ast.keyword(arg="install_green_fixture", value=ast.Name(id="install_green_fixture", ctx=ast.Load())))
            found += 1; break
    frozen._require(found == 1, "exact v1 factory namespace anchor required")
    namespace = dict(vars(previous), install_fixture_AST=install_fixture_AST, install_green_fixture=install_green_fixture)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])),
        str(Path(__file__))+":v1event_plus_one_sync_fixture_hook", "exec"), namespace)
    return namespace["cloned_collect_episode"]()


_original_factory = fixture_event_factory


def capture_factory():
    # HH_261002 - Retain the entire v1 event AST, late-binding only the three actual startup callbacks missed by v1's copied globals.
    original = _original_factory()
    function = inspect.getclosurevars(original).nonlocals["function"]
    def capture(*args, **kwargs):
        capture_args, partial, manifest = args[0], args[3], args[6]
        from agents.navigation import basic_agent as agent_module
        original_agent = agent_module.BasicAgent
        namespace = function.__globals__
        names = ("_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
                 "install_green_fixture")
        saved = {name: namespace.get(name) for name in names}; holder = dict(fixture=None)
        frozen._require(collector.checked_agent_initialization_observation is previous.signed1.manual.checked_first_gear_initialization_observation,
                        "actual strict manual1 checker must already be scoped before capture")
        for name in names[:3]: namespace[name] = getattr(collector, name)
        def install(world, args, output, manifest):
            record = manifest.get("capture_contract", {}).get("live_xodr_preflight", {})
            frozen._require(record.get("actual_sha256") == XODR_SHA and output == partial,
                            "fixture requires the unchanged verified pre-actor XODR identity")
            import carla
            fixture = GreenSignalFixture(world, capture_args, manifest, carla.TrafficLightState.Green)
            holder["fixture"] = fixture
            fixture.install()
        namespace["install_green_fixture"] = install
        # HH_261002 - The frozen function imports BasicAgent at entry, so install the wrapper before that import, not after map verification.
        agent_module.BasicAgent = hazard_agent(original_agent, lambda: holder["fixture"], partial)
        try: return previous.run_event_capture(function, *args, **kwargs)
        finally:
            original_error = sys.exc_info()[0] is not None
            fixture = holder["fixture"]
            try:
                if fixture is not None: fixture.restore()
            except Exception as error:
                manifest.setdefault("cleanup_errors", []).append("signal fixture: "+type(error).__name__+": "+str(error))
                if not original_error: raise
            finally:
                agent_module.BasicAgent = original_agent
                for name, value in saved.items(): namespace[name] = value
                if fixture is not None: frozen._write_new(partial/"owned_green_signal_fixture.json", fixture.record)
    return capture


@contextmanager
def activated(model, args):
    global _active
    frozen._require(not _active and not previous._active, "new v2 Town03 collector cannot nest")
    verify_frozen(); validate_capture(args, collector.load_route(args.route_file))
    old_factory, old_schema = previous.cloned_collect_episode, previous.SCHEMA
    _active = True; previous.cloned_collect_episode = capture_factory; previous.SCHEMA = SCHEMA
    try:
        with previous.activated(model, args): yield
    finally:
        previous.cloned_collect_episode, previous.SCHEMA = old_factory, old_schema; _active = False
