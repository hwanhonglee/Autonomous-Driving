"""HH_261004 - Bind one fixed45 Town03 known-DEV straight capture to the proven learned runtime hooks."""

from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
import hashlib
import json
import math
from pathlib import Path

from . import collect_temporal_route_joint_town03_right_v3_20261002 as runtime


fixture = runtime.previous
parent = fixture.previous
frozen, collector, ROOT = runtime.frozen, runtime.collector, runtime.ROOT
SCHEMA = "portable_e2e.actual_route_joint_preview05_town03_known_dev_straight_fixed45.20261004.v1"
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_straight_fixed45.20261004.v1"
SEED = 2026102102
ROUTE_SHA = "8fc442ed6cc6f6e40a4f346c2cfbda26105bd5a53c0992f7c6eb78964838cf06"
ROUTE_LENGTH = 209.370221122845
ROUTE_POINTS = 211
PREVIEW_SCHEMA, THREAD_SCHEMA = runtime.PREVIEW_SCHEMA, runtime.THREAD_SCHEMA
# HH_261004 - These compatibility constants are never straight-event thresholds; the owner removes the inherited turn contract.
ENTRY = EXIT = EXPECTED_TURN_DEG = 0.0
RUNTIME_SHA = "da212cdaf265ae7e94555e9cea80bfa8da49677def14657460fa694c9fddfdfc"
_active = False
_original_known_dev_session = parent.known_dev_session


def verify_frozen():
    """HH_261004 - Require the reviewed runtime-binding implementation without changing its frozen bytes."""
    frozen._require(hashlib.sha256(Path(runtime.__file__).read_bytes()).hexdigest() == RUNTIME_SHA,
                    "reviewed actual camera/ACK runtime binding changed")
    runtime.verify_frozen()


def profile(seed):
    frozen._require(type(seed) is int and seed == SEED and 0 <= seed < 2**32,
                    "exact matched centered uint32 seed required")
    return dict(case_id="town03_known_dev_straight_centered_fixed45", seed=seed,
                signed_lateral_offset_ros_m=0.0, signed_yaw_offset_ros_deg=0.0)


def validate_capture(args, route):
    """HH_261004 - Reject every physical/configuration change except the declared Town03 route and map."""
    profile(args.seed)
    frozen._require(args.host == "127.0.0.1" and args.port == 2440 and not args.allow_map_load
        and args.control_transport == "acknowledged_batch" and args.goal_stop_profile == "disabled"
        and args.agent_initialization == "after_bootstrap" and args.physics_hz == 20 and args.capture_hz == 10
        and args.stationary_warmup_sec == 1.2 and args.stationary_tail_sec == 0
        and args.max_duration_sec == 45 and args.target_speed_kmh == 9 and args.weather == "ClearNoon"
        and args.spawn_z_offset_m == 0.5 and args.wheelbase_m == 2.85
        and args.research_speed_cap_mps == 2.5 and args.research_max_cte_m == 1.0
        and args.research_longitudinal_pi is True and getattr(args, "observe_traffic_lights", False) is True
        and route.get("town") == "Town03" and route.get("scenario") == "straight"
        and len(route.get("route", ())) == ROUTE_POINTS and route.get("route_length_m") == ROUTE_LENGTH
        and isinstance(args.research_source_sha256, str) and len(args.research_source_sha256) == 64
        and all(c in "0123456789abcdef" for c in args.research_source_sha256),
        "fixed known-DEV Town03 straight45 contract differs")
    route_path = Path(args.route_file)
    frozen._require(route_path.is_file() and not route_path.is_symlink()
        and hashlib.sha256(route_path.read_bytes()).hexdigest() == ROUTE_SHA,
        "exact canonical Town03 p07 straight route bytes required")


class StraightIntervalObserver:
    """HH_261004 - Record the whole prior-ACK-controlled interval; never stop early on a favorable metric."""

    def __init__(self, route):
        self.route = route
        self.projector = collector.RouteProjector(route)
        self.project = parent.raw_project_function()
        self.session = None
        self.previous = None
        self.progress = 0.0
        self.records = []
        self.completed = False
        self.ACK_by_sequence = {}
        self.ACK_cursor = 0

    def observe(self, state, *, learned_effect, outgoing_safe):
        frozen._require(type(learned_effect) is bool and type(outgoing_safe) is bool,
                        "explicit prior learned ACK and outgoing safety predicates required")
        frame, stamp = state["frame"], state["timestamp"]
        values = [state[k] for k in ("x", "y", "yaw")] + [stamp]
        frozen._require(type(frame) is int and all(type(v) in (int, float) and math.isfinite(v) for v in values),
                        "finite actual native rear pose required")
        if self.previous is not None:
            frozen._require(frame == self.previous["frame"] + 1
                and abs(stamp - self.previous["timestamp"] - 0.05) <= 1e-6,
                "every consecutive native driving observation required")
        projection = self.project(self.projector, state["x"], state["y"], self.progress)
        self.progress = projection.progress_m
        points = self.route["route"]
        a, b = points[projection.segment_index], points[projection.segment_index + 1]
        tangent = math.atan2(b["y"] - a["y"], b["x"] - a["x"])
        yaw_error = math.atan2(math.sin(state["yaw"] - tangent), math.cos(state["yaw"] - tangent))
        row = dict(frame=frame, timestamp=stamp, actual_rear_progress_m=self.progress,
            actual_rear_CTE_m=projection.cross_track_error_m, actual_yaw_error_deg=math.degrees(yaw_error),
            current_prior_ACK_is_learned_effect=learned_effect,
            outgoing_original_safety_branch_clear=outgoing_safe,
            collision=bool(state.get("collision")), lane_invasion=bool(state.get("lane_invasion")),
            fixed45_interval_continues=True)
        self.records.append(row)
        self.previous = dict(state)
        return False

    def summary(self):
        complete = len(self.records) == 876
        return dict(schema=SCHEMA + ".fixed45_interval", completed=complete,
            observation_count=len(self.records), last_observation=self.records[-1] if self.records else None,
            full_fixed_interval_observed=complete,
            fixed45_interval_not_result_selected=True, no_early_success_break=True,
            split="val", known_DEV_site_reused=True, independent_validation=False,
            training_data_approved=False, model_promoted=False, vehicle_control_approved=False)


# HH_261004 - Preserve the public compatibility name while changing its semantics explicitly to a straight interval.
RightTurnObserver = StraightIntervalObserver


def observed_after_tick(states):
    frozen._require(parent._observer is not None and parent._observer.session is not None and states,
                    "owner-scoped actual straight session required")
    session, state = parent._observer.session, states[-1]
    sequence = state.get("control_transport", {}).get("expected_receipt_sequence")
    for row in session.wall_timing_records[parent._observer.ACK_cursor:]:
        number = row["ACK_sequence"]
        frozen._require(type(number) is int and number not in parent._observer.ACK_by_sequence,
                        "unique actual ACK sequence required")
        parent._observer.ACK_by_sequence[number] = row
    parent._observer.ACK_cursor = len(session.wall_timing_records)
    matching = parent._observer.ACK_by_sequence.get(sequence, {})
    learned = matching.get("ACK_server_accepted") is True and matching.get("learned_control_proposed") is True \
        and matching.get("control_source") == "learned_trajectory_tracker" \
        and state.get("control_transport", {}).get("status") == "PASS"
    parent._observer.observe(state, learned_effect=learned, outgoing_safe=session.latched_emergency is None)
    with (session.output / "known_dev_straight_observations.jsonl").open("a") as stream:
        stream.write(json.dumps(parent._observer.records[-1], allow_nan=False) + "\n")
    return False


def run_straight_capture(function, *args, **kwargs):
    """HH_261004 - Declare and retain the fixed interval before the original collector can write any result."""
    manifest, partial = args[6], args[3]
    manifest.setdefault("capture_contract", {})["known_DEV_straight_fixed45_interval"] = dict(
        schema=SCHEMA, dataset_split="val", known_DEV_site_reused=True, independent_validation=False,
        original_navigation_sha256=ROUTE_SHA, original_navigation_points=ROUTE_POINTS,
        fixed_total_seconds=45, fixed_native_state_count=900, fixed_camera_bundle_count=450,
        centered_lateral_offset_ros_m=0.0, centered_yaw_offset_ros_deg=0.0,
        current_prior_learned_ACK_required=True,
        observation_after_original_native_camera_control_ACK_and_recording=True,
        no_early_metric_or_catalog_goal_success=True, training_data_approved=False)
    try:
        return function(*args, **kwargs)
    finally:
        frozen._require(parent._observer is not None, "actual straight observer missing at capture close")
        frozen._write_new(partial / "known_dev_straight_summary.json", parent._observer.summary())


def cloned_collect_episode():
    """HH_261004 - Add only full-interval observation/result provenance and the reviewed owned signal fixture."""
    verify_frozen()
    tree = ast.parse(Path(collector.__file__).read_bytes())
    original = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "collect_episode")
    changed = copy.deepcopy(original)
    counts = dict(tick=0, result=0)

    class Insert(ast.NodeTransformer):
        def visit_Assign(self, node):
            node = self.generic_visit(node)
            text = ast.unparse(node)
            if text == "(goal_status, basic_agent_done, stop_reason) = tick_and_record('driving')":
                counts["tick"] += 1
                return [node, ast.parse("if stop_reason is None:\n    observed_after_tick(state_records)").body[0]]
            if any(ast.unparse(target) == "manifest['result']" for target in node.targets):
                counts["result"] += 1
                receipt = ast.parse("manifest['result']['known_dev_straight_fixed45_interval'] = _observer.summary()").body[0]
                return [node, receipt]
            return node

    changed = Insert().visit(changed)
    frozen._require(counts == dict(tick=1, result=1), "exact original tick/result AST anchors required")
    changed = fixture.install_fixture_AST(changed)
    restored = copy.deepcopy(changed)

    class Undo(ast.NodeTransformer):
        def visit_If(self, node):
            if ast.unparse(node.test) == "stop_reason is None" and len(node.body) == 1 \
                    and ast.unparse(node.body[0]) == "observed_after_tick(state_records)":
                return None
            return self.generic_visit(node)

        def visit_Assign(self, node):
            if ast.unparse(node.targets[0]) == "manifest['result']['known_dev_straight_fixed45_interval']":
                return None
            return self.generic_visit(node)

        def visit_Expr(self, node):
            if ast.unparse(node) == "install_green_fixture(world, args, partial, manifest)":
                return None
            return self.generic_visit(node)

    frozen._require(ast.dump(Undo().visit(restored)) == ast.dump(original),
                    "another original capture statement changed")
    namespace = dict(vars(collector), observed_after_tick=observed_after_tick,
                     _observer=parent._observer, install_green_fixture=fixture.install_green_fixture)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
                 str(Path(__file__)) + ":fixed45_straight_observation_and_fixture", "exec"), namespace)
    function = namespace["collect_episode"]

    def capture(*args, **kwargs):
        return run_straight_capture(function, *args, **kwargs)

    return capture


def known_dev_session():
    """HH_261004 - Keep the reviewed VAL MRO and give its fresh class an honest straight identity."""
    session, thread = _original_known_dev_session()
    session.__name__ = "KnownDevStraightObservedSession"
    session.__qualname__ = session.__name__
    return session, thread


@contextmanager
def activated(model, args):
    """HH_261004 - Scope the straight route around the proven fixture and actual late-bound camera/ACK hooks."""
    global _active
    frozen._require(not _active and not runtime._active and not fixture._active and not parent._active,
                    "Town03 straight activation cannot nest")
    verify_frozen()
    validate_capture(args, collector.load_route(args.route_file))
    changes = {
        parent: dict(profile=profile, validate_capture=validate_capture, RightTurnObserver=StraightIntervalObserver,
            observed_after_tick=observed_after_tick, run_event_capture=run_straight_capture,
            known_dev_session=known_dev_session, SCHEMA=SCHEMA, SEED=SEED, ROUTE_SHA=ROUTE_SHA,
            ROUTE_LENGTH=ROUTE_LENGTH, ENTRY=ENTRY, EXIT=EXIT, EXPECTED_TURN_DEG=EXPECTED_TURN_DEG),
        fixture: dict(validate_capture=validate_capture, SCHEMA=SCHEMA, OWNER_SCHEMA=OWNER_SCHEMA,
            SEED=SEED, ROUTE_SHA=ROUTE_SHA, ROUTE_LENGTH=ROUTE_LENGTH,
            ENTRY=ENTRY, EXIT=EXIT, EXPECTED_TURN_DEG=EXPECTED_TURN_DEG,
            _original_factory=cloned_collect_episode),
        runtime: dict(validate_capture=validate_capture, SCHEMA=SCHEMA, OWNER_SCHEMA=OWNER_SCHEMA,
            SEED=SEED, ROUTE_SHA=ROUTE_SHA, ROUTE_LENGTH=ROUTE_LENGTH,
            ENTRY=ENTRY, EXIT=EXIT, EXPECTED_TURN_DEG=EXPECTED_TURN_DEG,
            RightTurnObserver=StraightIntervalObserver),
    }
    saved = {module: {name: getattr(module, name) for name in values} for module, values in changes.items()}
    _active = True
    try:
        for module, values in changes.items():
            for name, value in values.items():
                setattr(module, name, value)
        with runtime.activated(model, args):
            yield
    finally:
        for module, values in reversed(tuple(saved.items())):
            for name, value in values.items():
                setattr(module, name, value)
        _active = False
