"""HH_261002 - Bind the actual observed-camera and learned-ACK hooks before the first Town03 driving command."""
from __future__ import annotations

import ast
import copy
import hashlib
import inspect
from contextlib import contextmanager
from pathlib import Path

from . import collect_temporal_route_joint_town03_right_v2_20261002 as previous

frozen, collector, ROOT = previous.frozen, previous.collector, previous.ROOT
SCHEMA = "portable_e2e.actual_route_joint_preview05_town03_known_dev_right.20261002.v3"
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_right.20261002.v3"
SEED, ROUTE_SHA, ROUTE_LENGTH = previous.SEED, previous.ROUTE_SHA, previous.ROUTE_LENGTH
ENTRY, EXIT, EXPECTED_TURN_DEG = previous.ENTRY, previous.EXIT, previous.EXPECTED_TURN_DEG
PREVIEW_SCHEMA, THREAD_SCHEMA = previous.PREVIEW_SCHEMA, previous.THREAD_SCHEMA
RightTurnObserver = previous.previous.RightTurnObserver
V2_SHA = "3473cbb37d771137a01459d7d634a4ce372e551059043904dd0c538bce0ffa04"
_active = False
_v2_factory = previous.capture_factory
_original_transport = collector.AcknowledgedControlTransport
_original_bundle = collector.exact_camera_bundle
_original_validate = previous.validate_capture


def verify_frozen():
    frozen._require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == V2_SHA,
                    "teacher-only failed v2 must remain byte exact")
    previous.verify_frozen()


def validate_capture(args, route):
    _original_validate(args, route)


def runtime_factory():
    # HH_261002 - Extend only the frozen v2 late-bound callback tuple/loop; inverse AST equality retains all other code.
    verify_frozen()
    tree = ast.parse(Path(previous.__file__).read_bytes())
    original = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "capture_factory")
    changed = copy.deepcopy(original); counts = dict(names=0, loop=0)
    for node in ast.walk(changed):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                and node.targets[0].id == "names":
            frozen._require(ast.literal_eval(node.value) == ("_blank_brake_control",
                "checked_agent_initialization_observation", "apply_spawn_z_offset", "install_green_fixture"),
                "exact frozen three-startup callback tuple required")
            node.value.elts[3:3] = [ast.Constant("AcknowledgedControlTransport"), ast.Constant("exact_camera_bundle")]
            counts["names"] += 1
        if isinstance(node, ast.For) and ast.unparse(node.iter) == "names[:3]":
            node.iter.slice.upper = ast.Constant(5); counts["loop"] += 1
    frozen._require(counts == dict(names=1, loop=1), "exact two runtime-binding AST anchors required")
    restored = copy.deepcopy(changed)
    for node in ast.walk(restored):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                and node.targets[0].id == "names": del node.value.elts[3:5]
        if isinstance(node, ast.For) and ast.unparse(node.iter) == "names[:5]": node.iter.slice.upper = ast.Constant(3)
    frozen._require(ast.dump(restored) == ast.dump(original), "another capture/fixture/hazard statement changed")
    namespace = dict(vars(previous))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
                 str(Path(__file__))+":five_actual_capture_callbacks", "exec"), namespace)
    return namespace["capture_factory"]()


def capture_factory():
    capture = runtime_factory()
    # HH_261002 - Real generic activation must already have installed both callbacks, not merely constructed an unused session.
    def guarded(*args, **kwargs):
        transport, bundle = collector.AcknowledgedControlTransport, collector.exact_camera_bundle
        frozen._require(transport is not _original_transport and bundle is not _original_bundle
            and any(cls.__name__ == "ResearchTransport" for cls in transport.__mro__),
            "actual learned transport and observed bundle hooks were not installed")
        manifest, partial = args[6], args[3]
        proof = dict(schema=SCHEMA+".actual_runtime_binding", actual_scoped_callbacks_bound=True,
            observed_bundle_calls=0, original_bundle_return_unmodified=True,
            first_driving_guard_passed=False, missing_wiring_aborts_before_first_driving_command=True,
            learned_commands_or_handoff_not_claimed_by_binding_alone=True)
        manifest.setdefault("capture_contract", {})["actual_runtime_binding"] = proof
        def observed(*positional, **options):
            result = bundle(*positional, **options)
            session = previous.previous._observer.session
            frame = positional[1]
            frozen._require(session is not None and session.bundle_frame == frame
                and session.observed_frames and session.observed_frames[-1].timestamp_ns
                == int(round(float(session.states[-1]["timestamp"])*1.e9)),
                "actual six-camera bundle did not reach the active causal learned session")
            proof["observed_bundle_calls"] += 1
            return result
        class CheckedResearchTransport(transport):
            def send(self, control, reason, expected_before_frame=None):
                if reason == "initial_drive_control":
                    session = previous.previous._observer.session
                    frozen._require(session is not None and proof["observed_bundle_calls"] == 12
                        and len(session.states) == 24 and all(s["capture_phase"] == "stationary_warmup" for s in session.states)
                        and len(session.history) == len(session.observed_frames) == 10
                        and session.bundle_frame == session.states[-1]["frame"]-1,
                        "all twelve actual warmup bundles must reach the policy before first driving command")
                    proof.update(first_driving_guard_passed=True, warmup_native_count=24,
                        warmup_observed_bundle_count=12, real_history_count=len(session.history))
                receipt = super().send(control, reason, expected_before_frame)
                if reason in ("initial_drive_control", "next_driving_control"):
                    session = previous.previous._observer.session
                    row = session.wall_timing_records[-1] if session.wall_timing_records else {}
                    frozen._require(row.get("ACK_sequence") == receipt["sequence"]
                        and row.get("ACK_server_accepted") is True and row.get("command_frame") == session.states[-1]["frame"]
                        and row.get("control_source") in ("expert_startup", "learned_trajectory_tracker", "emergency_brake"),
                        "actual policy proposal and acknowledged transport receipt must precede the next physics tick")
                    if reason == "initial_drive_control":
                        frozen._require(session.policy is not None and session.counts["expert_startup_commands"] >= 1,
                                        "actual initial-drive startup proposal was not executed")
                        proof.update(initial_drive_proposal_ACK_bound=True, initial_drive_ACK_sequence=receipt["sequence"],
                            initial_drive_prefill_count=session.counts["prefill_count"],
                            initial_drive_previous_camera_not_relabelled_fresh=True)
                    elif not proof.get("first_fresh_driving_proposal_checked") and session.bundle_frame == session.states[-1]["frame"]:
                        frozen._require(session.counts["prefill_count"] >= 1 or session.latched_emergency is not None,
                                        "first fresh driving proposal reached neither genuine prefill nor recorded emergency")
                        proof.update(first_fresh_driving_proposal_checked=True,
                            first_fresh_driving_prefill_count=session.counts["prefill_count"],
                            first_fresh_driving_original_emergency=session.latched_emergency)
                return receipt
        collector.AcknowledgedControlTransport, collector.exact_camera_bundle = CheckedResearchTransport, observed
        try: return capture(*args, **kwargs)
        finally:
            collector.AcknowledgedControlTransport, collector.exact_camera_bundle = transport, bundle
            frozen._write_new(partial/"actual_runtime_binding.json", proof)
    return guarded


@contextmanager
def activated(model, args):
    global _active
    frozen._require(not _active and not previous._active, "new v3 Town03 activation cannot nest")
    verify_frozen(); validate_capture(args, collector.load_route(args.route_file))
    saved = dict(capture_factory=previous.capture_factory, SCHEMA=previous.SCHEMA, OWNER_SCHEMA=previous.OWNER_SCHEMA)
    _active = True
    try:
        previous.capture_factory, previous.SCHEMA, previous.OWNER_SCHEMA = capture_factory, SCHEMA, OWNER_SCHEMA
        with previous.activated(model, args): yield
    finally:
        for name, value in saved.items(): setattr(previous, name, value)
        _active = False
