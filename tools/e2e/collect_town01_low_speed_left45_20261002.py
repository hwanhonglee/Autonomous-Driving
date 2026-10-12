"""HH_261002 - Actual original Town01 LEFT pixels/poses, first-gear teacher and explicitly separate diagnostic coast context."""
from __future__ import annotations

import ast
import copy
from contextlib import contextmanager, ExitStack
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from scripts.e2e import collect_town07_signed_recovery_20261001 as parent
from scripts.e2e import town01_low_speed_left45_task_20261002 as task
from scripts.e2e import run_owned_town07_signed_recovery_v3_20261001 as manual
from scripts.e2e import town01_train_left45_green_fixture_20261002 as signals

base = parent.base
SCHEMA = task.SCHEMA
PARENT_SHA = "8260f04a704394f1406cbfc5956fac678766ff20932b0621e35f4162f4883c3e"
BASE_SHA = "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08"
MANUAL_SHA = "01f6157bd61e32fe525166b4698fa70f8b82156843ecc1bb93edb538fd706a9a"
_active = False
_session = None


def verify_frozen():
    for provider, digest in ((parent, PARENT_SHA), (base, BASE_SHA), (manual, MANUAL_SHA)):
        task.require(task.sha(Path(provider.__file__)) == digest, "immutable teacher/capture/manual provider changed")
    task.verify_parent(); signals.verify_frozen(); manual.verify_frozen_dependencies()


def assert_owned_capture(args):
    # HH_261002 - The callback used before/after fixture ticks checks the declared private worker generation and socket owner.
    from scripts.e2e import research_temporal_v2_owned_camera_town07_20261001 as lifecycle
    plan = task.configuration_from_args(args)
    lifecycle.assert_owned_worker(dict(plan, output=plan["owner_output"]), args.moving_task_plan_sha256)


class Launch(manual.FirstGearRecoveryLaunchControl):
    def apply(self, control, **kwargs):
        result = super().apply(control, **kwargs)
        self.last.update(schema=SCHEMA+".actual_normal_teacher_control")
        if _session is not None:
            _session["manifest"]["capture_contract"]["teacher_controller"].update(
                normal_launch_controls_inherited_v2=False, normal_launch_throttle=.11,
                post_handoff_throttle_ramp_per_second=.025, gear=1, manual_gear_shift=True,
                final_BasicAgent_hazard_throttle_brake_steer_untouched=True,
                initialization_gear_contract=task.fixed_contract("v1", "pilot_00")["initialization_gear_contract"])
        return result


def definition(provider, name):
    verify_frozen()
    nodes = [n for n in ast.parse(Path(provider.__file__).read_bytes()).body if isinstance(n, ast.FunctionDef) and n.name == name]
    task.require(len(nodes) == 1, "exact immutable function required: "+name)
    return copy.deepcopy(nodes[0])


def teacher_namespace():
    # HH_261002 - Reuse the whole original teacher source in a private namespace; only provider identities and the declared manual law differ.
    namespace = dict(parent.__dict__, task=task, v2=SimpleNamespace(CombinedControl=Launch), __file__=__file__)
    nodes = [definition(parent, name) for name in ("moving_provider", "runtime_provider", "parsing_scope", "parse_args", "scope")]
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), __file__, "exec"), namespace)
    return namespace


_teacher = teacher_namespace()
parse_args = _teacher["parse_args"]


def manual_full_brake(carla):
    control = _session["original_blank"](carla)
    control.gear, control.manual_gear_shift = 1, True
    return control


def phase_blank(carla):
    states = [] if _session is None else _session["states"]
    current = states[-1] if states else {}
    return phase_control(carla, current.get("capture_phase"), current.get("moving_task"))


def phase_control(carla, phase, moving_record):
    control = manual_full_brake(carla)
    if moving_record is not None and moving_record.get("termination_reason") in (None, "recovery_future_support_complete") \
            and (phase == "stationary_tail" or moving_record.get("future_support_complete") is True):
        control.throttle, control.brake, control.steer = 0., 0., 0.
    return control


def capture_ast():
    node = definition(base, "collect_episode")
    seams = []
    for block in ast.walk(node):
        if not isinstance(block, ast.Try): continue
        for index, stmt in enumerate(block.body):
            if isinstance(stmt, ast.Assign) and len(stmt.targets)==1 and isinstance(stmt.targets[0], ast.Name) \
                    and stmt.targets[0].id == "blueprint_library": seams.append((block, index))
    task.require(len(seams)==1, "unique after-sync/pre-vehicle fixture seam required")
    block, index = seams[0]
    block.body.insert(index, ast.parse("install_green_fixture(world, args, manifest, partial, carla.TrafficLightState.Green)").body[0])
    changes = 0
    for item in ast.walk(node):
        if isinstance(item, ast.Assign) and len(item.targets)==1 and isinstance(item.targets[0], ast.Name) \
                and item.targets[0].id == "next_control" and isinstance(item.value, ast.IfExp):
            call = item.value.orelse
            task.require(isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == "_blank_brake_control", "original phase blank seam changed")
            item.value.orelse = ast.parse("phase_control(carla, phase, moving_record)", mode="eval").body
            changes += 1
    task.require(changes == 1, "one current-phase diagnostic control override required")
    # HH_261002 - A final service brake is accepted only after the fixed coast observations end, before actor destruction; no extra captured tick.
    task.require(bool(block.finalbody), "original capture cleanup is missing")
    block.finalbody.insert(0, ast.parse('if acknowledged is not None and sys.exc_info()[0] is None:\n    send_control(manual_full_brake(carla), "left45_post_capture_cleanup_full_brake", state_records[-1]["frame"] if state_records else None)').body[0])
    return node


@contextmanager
def scope(args):
    global _active, _session
    task.require(not _active, "LEFT45 teacher scopes cannot nest")
    verify_frozen(); assert_owned_capture(args)
    names = ("collect_episode", "acknowledged_snapshot_state", "_blank_brake_control", "checked_agent_initialization_observation", "AcknowledgedControlTransport")
    saved = {name: getattr(base, name) for name in names}
    stack = ExitStack()
    _active = True
    _session = dict(manifest=None, states=[], fixture=None, partial=None, original_blank=saved["_blank_brake_control"])

    def install_green_fixture(world, parsed, manifest, partial, green):
        task.require(_session["fixture"] is None, "fixture must be installed once before sensors")
        fixture = stack.enter_context(signals.owned_fixture(world, parsed, manifest, green, assert_owned_capture))
        _session.update(fixture=fixture, partial=partial)
        fixture.install()
        manifest["capture_contract"]["owned_green_signal_fixture"] = fixture.record

    def snapshot(*positional, **kwargs):
        state, raw = saved["acknowledged_snapshot_state"](*positional, **kwargs)
        if _session["fixture"] is not None: _session["fixture"].check()
        return state, raw

    class Transport(saved["AcknowledgedControlTransport"]):
        def send(self, control, reason, expected_before_frame=None):
            if reason in ("exception_cleanup_abort", "initial_governor_failure_abort", "alignment_failure_abort"):
                control.throttle, control.brake, control.steer, control.gear, control.manual_gear_shift = 0., 1., 0., 1, True
            return super().send(control, reason, expected_before_frame)

    def collect(*positional, **kwargs):
        manifest = positional[6]
        _session.update(manifest=manifest, states=positional[4])
        manifest.update(task.pending_scope(), control_source="Disclosed_BasicAgent_original_LEFT_teacher_not_NN")
        manifest["capture_contract"]["left45_protocol"] = task.fixed_contract("v1", "pilot_00")
        namespace = dict(base.__dict__, install_green_fixture=install_green_fixture, manual_full_brake=manual_full_brake,
            phase_control=phase_control)
        node = capture_ast()
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), __file__, "exec"), namespace)
        try: return namespace["collect_episode"](*positional, **kwargs)
        finally:
            stack.close()
            fixture = _session["fixture"]
            if fixture is not None:
                with (_session["partial"]/"owned_green_signal_fixture.json").open("x") as stream:
                    json.dump(fixture.record, stream, indent=2, allow_nan=False)

    base.collect_episode, base.acknowledged_snapshot_state = collect, snapshot
    base._blank_brake_control, base.checked_agent_initialization_observation = phase_blank, manual.checked_first_gear_initialization_observation
    base.AcknowledgedControlTransport = Transport
    try:
        with _teacher["scope"](args): yield
    finally:
        for name, value in saved.items(): setattr(base, name, value)
        _active, _session = False, None
