#!/usr/bin/env python3
"""HH_261004 - Measure prepared CPU terminal overhead on retained real raw NN fields.

Endpoint poses are explicitly COUNTERFACTUAL numerical fixtures on authentic
route geometry.  Saved NN fields were not inferred at these endpoint poses.
This does not train, run a model, own CARLA, actuate, or prove end-to-end100ms.
The prospective declaration is create-only and fsynced before any timed cell.
"""

from __future__ import annotations

import argparse
import ast
from collections import Counter
import copy
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import stat
import time
from types import SimpleNamespace

import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning"))

from portable_e2e.contract import ContractError, _canonical_route_in_base
from portable_e2e.longitudinal_pi_headroom_research_20261001 import LongitudinalPIHeadroomResearch
from portable_e2e.navigation_terminal_retiming_v1_20261004 import NavigationTerminalRetimingV1
from portable_e2e.path_time_decoder_v2 import GO
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory
from portable_e2e.simulation_research import ControllerConfig, trajectory_control
from portable_e2e import terminal_witness_v1_20261002 as proof


SCHEMA = "portable_e2e.navigation_terminal_cpu_counterfactual_benchmark.20261004.v1"
SUPPORTS_M = (17.9, 10., 5., 2.5)
EXPECTED_FRESH_COUNT = 415
RUN_REL = "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v3"
LEARNED_REL = RUN_REL + "/episode.partial/learned_control.jsonl"
STATES_REL = RUN_REL + "/episode.partial/states.jsonl"
ROUTE_REL = RUN_REL + "/source/storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town03/catalog/straight/routes/town03/straight/town03_straight_s0000_p07.json"
PINNED_SOURCE_SHA256 = {
    LEARNED_REL: "a34a3bca78e52437064cd3b0c9c19b13deb9929b1ff5b3a690cad54c7700da89",
    STATES_REL: "a6bfc083c5cd61a7421fbf798f65cde9354870fa6942ad767497e222f990968c",
    ROUTE_REL: "8fc442ed6cc6f6e40a4f346c2cfbda26105bd5a53c0992f7c6eb78964838cf06",
    "learning/portable_e2e/path_time_decoder_v2.py": "d80e370d52c73591df8a593a0483e225cb4cf2595e8389a91400f68f94eea7e7",
    "learning/portable_e2e/path_time_decoder_terminal_v1_20261002.py": "df9beb1d7f71c6757907f7b6e7792ba3d9b7e2679d75c165b952cf6f6895943a",
    "learning/portable_e2e/terminal_witness_v1_20261002.py": "52704ec0214053f7942a2657a76bf18fed2cbbbf81d729c3d512d66484672ca1",
    "learning/portable_e2e/navigation_terminal_retiming_v1_20261004.py": "99c90510a3bd53440d676cf0dedd8b1f3e7ea31bea1551784ef957cc77986690",
    "learning/tests/test_navigation_terminal_retiming_v1_20261004.py": "735b5c484dd3a15702259c15ca5aa541ae0cfbb82ee131102aa8d6e6fb3a4682",
    "learning/portable_e2e/contract.py": "e2e3a2669fa94aa98cb7e50f20ffa7db433b99eead9a12c654392e4b293a5c09",
    "learning/portable_e2e/runtime_contract.py": "38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3",
    "learning/portable_e2e/simulation_research.py": "4a66e2d3b522272822ddb43c43edeed1ebc6a2ee6c20bd0cda6ce2b3ea6c9252",
    "learning/portable_e2e/longitudinal_pi_research_20261001.py": "bc15f390398c6b1c2e678b5c90896d4ac4612ea349451b4cdb530d3f0946665b",
    "learning/portable_e2e/longitudinal_pi_headroom_research_20261001.py": "fd984bdd6473e4e350a62712451be02768e6a730ad8b6dd033c79ca3af9a1ec5",
    "tools/e2e/collect_temporal_route_joint_preview05_signed45_20261002.py": "e0b3f5aeec01ce035545ccc92dd26cba274463024486928a3ef72de099892085",
    "tools/e2e/collect_temporal_route_joint_preview08_signed45_20261002.py": "90a3788fa4be92136cb73b2f3413b4ddbbdd2c0a89f55f5998256e1abbba1498",
}
CHECKPOINT_SHA = "43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88"
NPZ_SHA = "b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625"
CANONICAL_NAVIGATION = dict(navigation=dict(maximum_anchor_distance_m=5., route_resample_spacing_m=1., route_lookahead_m=120.))
MAX_SOURCE_BYTES = 128 * 1024 * 1024
PREVIEW_SCHEMA = "portable_e2e.causal_selected_speed_preview05_calibration.20261002.v1"


def _require(condition, message):
    if not condition:
        raise ContractError(message)


def _sha(payload):
    return hashlib.sha256(payload).hexdigest()


def _encoded(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return payload, _sha(payload)


def read_canonical_regular(path):
    """HH_261004 - Bound reads and reject aliases/symlinks, nonregular sources and path/open inode drift."""
    path = Path(path)
    _require(path.is_absolute() and path.resolve(strict=True) == path, "source path must be canonical with no symlink")
    before = path.lstat()
    _require(stat.S_ISREG(before.st_mode) and 0 <= before.st_size <= MAX_SOURCE_BYTES, "source must be a bounded regular file")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(descriptor, "rb") as stream:
        opened = os.fstat(stream.fileno())
        _require(stat.S_ISREG(opened.st_mode) and (opened.st_dev, opened.st_ino) == (before.st_dev, before.st_ino),
                 "source inode changed before open")
        payload = stream.read(MAX_SOURCE_BYTES + 1)
    _require(len(payload) <= MAX_SOURCE_BYTES and path.resolve(strict=True) == path
             and (path.lstat().st_dev, path.lstat().st_ino) == (before.st_dev, before.st_ino), "source read bound or identity changed")
    return payload


def source_closure():
    """HH_261004 - Pin authentic inputs and frozen/new numerical code before and after measurement."""
    actual = {name: _sha(read_canonical_regular(ROOT / name)) for name in PINNED_SOURCE_SHA256}
    _require(actual == PINNED_SOURCE_SHA256, "source closure differs; no alternate raw source or threshold allowed")
    for name in ("tools/e2e/benchmark_navigation_terminal_retiming_v1_20261004.py",
                 "learning/tests/test_benchmark_navigation_terminal_retiming_v1_20261004.py"):
        actual[name] = _sha(read_canonical_regular(ROOT / name))
    return actual


def _lines(path):
    return [(json.loads(line), _sha(line), index) for index, line in enumerate(read_canonical_regular(path).splitlines(keepends=True))]


class PreparedPreview05Tracker:
    """HH_261004 - Run exact frozen preview-selection/tracker statements in a private CPU-only receipt scope.

Only .8->.5 substitutions already present in frozen preview05 are admitted and
inverse-proven.  STOP/non-GO keeps the WHOLE original .3s tracker.  No collector
is imported or activated; the exclusive namespace is numerical preparation,
not simulator ownership, sensor freshness, or actuation authority.
"""

    def __init__(self):
        name = "tools/e2e/collect_temporal_route_joint_preview08_signed45_20261002.py"
        payload = read_canonical_regular(ROOT / name)
        _require(_sha(payload) == PINNED_SOURCE_SHA256[name], "frozen preview08 source differs")
        preview05 = "tools/e2e/collect_temporal_route_joint_preview05_signed45_20261002.py"
        _require(_sha(read_canonical_regular(ROOT / preview05)) == PINNED_SOURCE_SHA256[preview05], "frozen preview05 source differs")
        source = ast.parse(payload)
        definitions = []
        self.ast_proof = {}
        for function, expected in (("preview_selection", 4), ("trajectory_control", 0)):
            original = next(node for node in source.body if isinstance(node, ast.FunctionDef) and node.name == function)
            changed = copy.deepcopy(original)
            count = [0]

            class Narrow(ast.NodeTransformer):
                def visit_Constant(self, node):
                    if type(node.value) is float and node.value == .8:
                        node.value = .5
                        count[0] += 1
                    elif isinstance(node.value, str):
                        node.value = node.value.replace("0.8", "0.5")
                    return node

            changed = Narrow().visit(changed)
            _require(count[0] == expected, "unexpected frozen preview-only scalar substitutions")
            restored = copy.deepcopy(changed)

            class Restore(ast.NodeTransformer):
                def visit_Constant(self, node):
                    if type(node.value) is float and node.value == .5:
                        node.value = .8
                    elif isinstance(node.value, str):
                        node.value = node.value.replace("0.5", "0.8")
                    return node

            restored = Restore().visit(restored)
            _require(ast.dump(restored) == ast.dump(original), "non-preview frozen tracker statement changed")
            self.ast_proof[function] = dict(preview_only_scalar_substitutions=count[0], inverse_original_AST_equal=True)
            definitions.append(changed)
        self.namespace = dict(math=math, replace=replace, frozen=SimpleNamespace(_require=_require),
            PREVIEW_SCHEMA=PREVIEW_SCHEMA, _trajectory_control=trajectory_control, _active=False, _proposal_session=None)
        exec(compile(ast.fix_missing_locations(ast.Module(body=definitions, type_ignores=[])),
                     str(ROOT / preview05) + ":PRIVATE_CPU_NUMERICAL_SCOPE_ONLY", "exec"), self.namespace)

    def control(self, selected, current, config):
        _require(not self.namespace["_active"] and self.namespace["_proposal_session"] is None, "private numerical preview calls cannot overlap")
        session = SimpleNamespace(_preview_tracker_receipt=None)
        self.namespace.update(_active=True, _proposal_session=session)
        try:
            result = self.namespace["trajectory_control"](selected, current, config)
            return result, session._preview_tracker_receipt
        finally:
            self.namespace.update(_active=False, _proposal_session=None)


def census_rows(rows):
    # HH_261004 - Eligibility is solely the disclosed fresh-camera flag; do not select only accepted numerical outputs.
    eligible = [(record, digest, line) for record, digest, line in rows if record.get("held_between_camera_frames") is False]
    classes = Counter("fresh_NN" if row.get("held_between_camera_frames") is False else
                      "held_NN" if row.get("held_between_camera_frames") is True else "other_startup" for row, _, _ in rows)
    return eligible, dict(classes)


def route_geometry(route):
    points = []
    for sample in route["route"]:
        point = (float(sample["x"]), float(sample["y"]))
        _require(all(math.isfinite(v) for v in point), "nonfinite source route")
        if not points or math.dist(points[-1], point) > 1.e-9:
            points.append(point)
    _require(len(points) >= 2, "authentic route has no nonzero segment")
    arcs = [0.]
    for first, second in zip(points, points[1:]):
        arcs.append(arcs[-1] + math.dist(first, second))
    return dict(points=tuple(points), arcs=tuple(arcs), total=arcs[-1], map_id=route["town"], route_id="town03_straight_s0000_p07")


def _sample(points, arcs, requested):
    for index, (start, end) in enumerate(zip(arcs, arcs[1:])):
        if requested <= end:
            ratio = (requested - start) / (end - start)
            a, b = points[index], points[index + 1]
            return (a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1])), index
    return points[-1], len(points) - 2


def load_cases():
    rows = _lines(ROOT / LEARNED_REL)
    fresh, census = census_rows(rows)
    _require(len(fresh) == EXPECTED_FRESH_COUNT and len(rows) == 877, "original415 fresh/877 full census differs")
    state_rows = _lines(ROOT / STATES_REL)
    states = {row["frame"]: (row, digest) for row, digest, _ in state_rows}
    _require(len(states) == len(state_rows) == 900, "original900 state denominator/unique frame differs")
    cases = []
    for source_index, (record, digest, line) in enumerate(fresh):
        _require(record["source_checkpoint_sha256"] == CHECKPOINT_SHA and record["source_npz_sha256"] == NPZ_SHA,
                 "original NN weights source differs")
        _require(record["training_data_approved"] is False and record["vehicle_control_approved"] is False,
                 "counterfactual benchmark grants neither TRAIN nor vehicle approval")
        _require(record["controller"]["speed_preview_s"] == .5, "original causal preview0.5 differs")
        _require(record["longitudinal_pi_research"]["controller_target_cap_mps"] == 2.4, "actual PI headroom differs")
        frame = record["inference_frame"]
        _require(frame in states, "fresh NN lacks original exact-frame observed state")
        observed_state, state_sha = states[frame]
        stamp = record["owned_camera_history"]["frame_timestamp_ns"]
        _require(abs(round(observed_state["timestamp"] * 1.e9) - stamp) <= 1, "source state and camera stamps differ")
        original_camera = next((c for c in record["owned_camera_history"]["source_camera_frames"] if c["timestamp_ns"] == stamp), None)
        _require(original_camera is not None and len(original_camera["camera_sha256"]) == 6, "original current6 camera provenance missing")
        cases.append(dict(source_index=source_index, source_line_index=line, source_line_sha256=digest,
            actual_state_sha256=state_sha, actual_state=observed_state, stamp_ns=stamp,
            original_camera_sha256=original_camera["camera_sha256"], record=record))
    return cases, census, route_geometry(json.loads(read_canonical_regular(ROOT / ROUTE_REL)))


def counterfactual_payloads(case, support, geometry):
    """HH_261004 - Authentic map polyline/stamps plus clearly separate counterfactual pose provenance."""
    points, arcs = geometry["points"], geometry["arcs"]
    anchor = geometry["total"] - support
    _require(anchor >= 0, "requested counterfactual endpoint lies outside authentic mission")
    xy, index = _sample(points, arcs, anchor)
    a, b = points[index], points[index + 1]
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    quaternion = (0., 0., math.sin(yaw / 2), math.cos(yaw / 2))
    position = (xy[0], xy[1], float(case["actual_state"]["z"]))
    provided, goal, _ = _canonical_route_in_base(points, position_m=position, orientation_xyzw=quaternion,
        contract=CANONICAL_NAVIGATION, context="COUNTERFACTUAL endpoint numerical fixture")
    provided_arc = sum(math.dist(a, b) for a, b in zip(provided, provided[1:]))
    mission = dict(schema=proof.MISSION_SCHEMA, coordinates=dict(proof.COORDINATES),
        mission_id="COUNTERFACTUAL-independent-endpoint-" + str(support), map_id=geometry["map_id"], route_id=geometry["route_id"],
        source_route_artifact_sha256=PINNED_SOURCE_SHA256[ROUTE_REL], source_available_timestamp_ns=0,
        complete_mission_route=True, polyline_map_xy_m=points, goal_map_xy_m=points[-1], stop_reference_offset_m=0.)
    pose_provenance = dict(counterfactual_not_observed=True, source_actual_state_sha256=case["actual_state_sha256"],
        source_frame=case["record"]["inference_frame"], fixture_support_m=support, position_map_m=position, orientation_xyzw=quaternion)
    _, pose_sha = _encoded(pose_provenance)
    stamp = case["stamp_ns"]
    obs = dict(schema=proof.OBSERVATION_SCHEMA, coordinates=dict(proof.COORDINATES), mission_id=mission["mission_id"],
        map_id=mission["map_id"], route_id=mission["route_id"], source_route_artifact_sha256=mission["source_route_artifact_sha256"],
        sequence_index=0, anchor_timestamp_ns=stamp, ego_timestamp_ns=stamp, pose_timestamp_ns=stamp, navigation_timestamp_ns=stamp,
        camera_timestamp_ns=[stamp] * 6, camera_source_sha256=case["original_camera_sha256"],
        ego_source_sha256=case["record"]["owned_camera_history"]["normalized_input_tensor_sha256"]["ego_history"], pose_source_sha256=pose_sha,
        position_map_m=position, orientation_xyzw=quaternion, provided_route_xy_base_m=provided,
        provided_route_support_m=provided_arc, route_anchor_arc_m=anchor, remaining_mission_arc_m=support, goal_base_m=goal)
    mp, ms = _encoded(mission)
    op, os_ = _encoded(obs)
    return mp, ms, op, os_, pose_provenance


def run_cell(case, support, geometry, decoder, *, phase, tracker=None):
    tracker = tracker or PreparedPreview05Tracker()
    # HH_261004 - Timer covers geometry/payload/witness/tensor creation, new decoder, original gate, tracker and actual PI; not NN/sensors/ACK.
    started = time.perf_counter()
    row = dict(schema=SCHEMA, phase=phase, source_index=case["source_index"], source_line_sha256=case["source_line_sha256"],
        actual_state_sha256=case["actual_state_sha256"], source_frame=case["record"]["inference_frame"], source_stamp_ns=case["stamp_ns"],
        declared_counterfactual_support_m=support, counterfactual_not_actual_endpoint_drive=True,
        old_raw_NN_seconds_separate=case["record"]["raw_nn_seconds"], old_policy_total_seconds_separate=case["record"]["total_seconds"],
        accepted=False, rejection_stage=None, rejection_code=None, rejection_reason=None, stage_seconds={},
        vehicle_control_approved=False, training_data_approved=False, learned_STOP_claim=False, end_to_end100ms_proven=False)
    stage = "counterfactual_geometry_and_payload"
    try:
        mp, ms, op, os_, provenance = counterfactual_payloads(case, support, geometry)
        mark = time.perf_counter()
        row["stage_seconds"][stage] = mark - started
        stage = "complete_mission_witness"
        decision = proof.evaluate_terminal_witness(mission_payload=mp, expected_mission_sha256=ms,
            expected_route_artifact_sha256=PINNED_SOURCE_SHA256[ROUTE_REL], observation_payload=op,
            expected_observation_sha256=os_, now_ns=case["stamp_ns"], requested_mode=proof.VERIFIED_TERMINAL)
        row.update(witness_status=decision.status, witness_reason_code=decision.reason_code, witness_reason=decision.reason)
        _require(decision.status == "VERIFIED", decision.reason_code + ": " + decision.reason)
        row.update(mission_sha256=ms, observation_sha256=os_, counterfactual_pose_provenance=provenance,
            actual_provided_arc_m=decision.witness.provided_route_arc_m, actual_remaining_arc_m=decision.witness.remaining_mission_arc_m)
        next_mark = time.perf_counter()
        row["stage_seconds"][stage] = next_mark - mark
        mark, stage = next_mark, "physical_tensorization_and_navigation_decoder"
        record = case["record"]
        raw = record["raw_output"]
        curve = torch.tensor([raw["curvature_knots"]], dtype=torch.float32)
        preferred = torch.tensor([raw["preferred_speed_knots"]], dtype=torch.float32)
        current = torch.tensor([record["measured_current_speed_mps"]], dtype=torch.float32)
        actual_support = min(decision.witness.provided_route_arc_m, decision.witness.remaining_mission_arc_m, 64.)
        with torch.no_grad():
            result = decoder(curve, preferred, current, torch.tensor([GO]), torch.zeros(1),
                torch.tensor([actual_support], dtype=torch.float64), terminal_witness=decision,
                terminal_anchor_timestamp_ns=case["stamp_ns"], terminal_now_ns=case["stamp_ns"],
                terminal_observation_sha256=os_, terminal_mission_sha256=ms, terminal_route_artifact_sha256=PINNED_SOURCE_SHA256[ROUTE_REL])
        row["decoder_output"] = {name: result[name].tolist() for name in ("xy", "speed", "instantaneous_speed", "arc_distance",
            "terminal_arrival_time_s", "terminal_stop_complete_within_horizon", "navigation_requested_next_speed_raw",
            "navigation_requested_next_speed_bounded", "navigation_initial_acceleration_request", "navigation_stop_distance_request",
            "navigation_first_speed_lateral_cap", "navigation_first_speed_endpoint_cap", "terminal_spatial_node_count")}
        next_mark = time.perf_counter()
        row["stage_seconds"][stage] = next_mark - mark
        mark, stage = next_mark, "original_runtime_trajectory_gate"
        selected = validate_and_select_trajectory(result["xy"][0].tolist(), result["speed"][0].tolist(), [0.],
            RuntimeGateConfig(candidate_count=1), current_speed_mps=current.item())
        next_mark = time.perf_counter()
        row["stage_seconds"][stage] = next_mark - mark
        mark, stage = next_mark, "actual_preview05_tracker_and_headroom_PI"
        controller = ControllerConfig(**record["controller"])
        control, preview_receipt = tracker.control(selected, current.item(), controller)
        original = dict(control)
        pi = LongitudinalPIHeadroomResearch()
        integral_before = record["longitudinal_pi_research"]["integral_before"]
        _require(type(integral_before) in (int, float) and math.isfinite(integral_before)
                 and abs(integral_before) <= pi.config.maximum_integral_error, "invalid original PI integral state")
        pi.integral_error = integral_before
        request, audit = pi.update(control["desired_speed_mps"], current.item())
        if request is not None:
            control.update(request)
        row.update(controller=asdict(controller), original_tracker_control=original, counterfactual_control_request=control,
            preview05_tracker_receipt=preview_receipt,
            counterfactual_PI_audit=audit, original_PI_before_restored=True, PI_updates_in_this_independent_fixture=1,
            accepted=True)
        row["stage_seconds"][stage] = time.perf_counter() - next_mark
    except (ContractError, ValueError, TypeError, KeyError, IndexError, FloatingPointError, RuntimeError) as error:
        code = getattr(error, "code", row.get("witness_reason_code") if stage == "complete_mission_witness" else type(error).__name__)
        row.update(rejection_stage=stage, rejection_code=code, rejection_reason=str(error))
    row["measured_overhead_seconds"] = time.perf_counter() - started
    return row


def _summary(values):
    values = sorted(float(v) for v in values)
    if not values:
        return dict(count=0, maximum=None, p50=None, p95=None)
    def quantile(fraction):
        position = (len(values) - 1) * fraction
        lower = math.floor(position)
        return values[lower] + (values[min(lower + 1, len(values) - 1)] - values[lower]) * (position - lower)
    return dict(count=len(values), maximum=values[-1], p50=quantile(.5), p95=quantile(.95))


def write_json_create(path, document):
    # HH_261004 - The declaration/results cannot replace any frozen or previously generated file.
    payload = json.dumps(document, sort_keys=True, indent=2, allow_nan=False).encode() + b"\n"
    with path.open("xb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    return _sha(payload)


def configure_cpu():
    torch.set_num_threads(4)
    torch.set_num_interop_threads(1)
    _require(torch.get_num_threads() == 4 and torch.get_num_interop_threads() == 1, "required CPU4/interop1 not applied")
    return dict(intraop_threads=4, interop_threads=1, cpu_affinity=sorted(os.sched_getaffinity(0)), pid=os.getpid(),
                torch_version=torch.__version__, cuda_work_performed=False)


def validate_output_path(output):
    # HH_261004 - Reject the original requested symlink/alias instead of silently writing through its resolved target.
    output = Path(output).absolute()
    _require(output == output.resolve(), "requested output path must be canonical with no symlink or alias")
    diagnostic_root = ROOT / "storage/experiments/diagnostics"
    _require(diagnostic_root.resolve() == diagnostic_root and output.is_relative_to(diagnostic_root), "fresh canonical diagnostics output location required")
    _require(not output.exists(), "output must be a new create-only directory")
    return output


def benchmark(output):
    execution = configure_cpu()
    closure = source_closure()
    cases, census, geometry = load_cases()
    output = validate_output_path(output)
    output.mkdir(parents=False, exist_ok=False)
    declaration = dict(schema=SCHEMA, status="PROSPECTIVE_BEFORE_ANY_TIMED_CELL", provenance_comment="HH_261004 - Freeze whole CPU numerical overhead census before measurement.",
        source_sha256=closure, source_row_census=census, execution=execution, supports_m=SUPPORTS_M,
        fresh_source_count=len(cases), declared_measured_cell_count=len(cases) * len(SUPPORTS_M),
        source_order=[dict(source_index=c["source_index"], source_line_index=c["source_line_index"], source_frame=c["record"]["inference_frame"],
                           source_line_sha256=c["source_line_sha256"]) for c in cases],
        measured_order="source-major ascending original fresh line order; each17.9,10,5,2.5m; every cell retained including rejection",
        warmup_policy=dict(cell_count=4, source_index=0, supports_m=SUPPORTS_M, passes=1, failures_retained=True, excluded_from_measured_denominator=True),
        counterfactual_pose_only=True, saved_NN_not_inferred_at_counterfactual_pose=True, independent_sequence0_witness_per_cell=True,
        witness_clock_domain="replayed original simulation stamps; now_ns=replayed anchor; not a claim of live wall-clock source freshness",
        original_per_frame_PI_integral_before_restored=True, configured_preview_s=.5,
        original_whole_tracker03_preserved_when_STOP_or_non_GO=True, actual_PI_controller_target_cap_mps=2.4,
        measured_scope="geometry/payload/hashes/witness/tensorization/navigation decoder/original gate/preview05 tracker/PI and diagnostic extraction",
        excluded_scope="source file loading; model/image preprocessing/NN; source camera exposure/callback; CARLA/control ACK/actuation; disk writes",
        model_weights_changed=False, raw_semantic_heads_used=False, training_data_approved=False,
        old_NN_timing_reported_separately_never_added=True, end_to_end100ms_proven=False, vehicle_control_approved=False,
        main_functions_approved=0, subfunctions_approved=0)
    declaration_sha = write_json_create(output / "prospective_declaration.json", declaration)
    decoder = NavigationTerminalRetimingV1()
    tracker = PreparedPreview05Tracker()
    measured = []
    warmups = []
    with (output / "all_warmup_and_measured_cells.jsonl").open("xb") as stream:
        for phase, inputs in (("warmup", [(cases[0], s) for s in SUPPORTS_M]),
                              ("measured", [(case, s) for case in cases for s in SUPPORTS_M])):
            for case, support in inputs:
                row = run_cell(case, support, geometry, decoder, phase=phase, tracker=tracker)
                (warmups if phase == "warmup" else measured).append(row)
                stream.write(_encoded(row)[0] + b"\n")
            stream.flush()
            os.fsync(stream.fileno())
    after = source_closure()
    _require(after == closure, "source closure changed during benchmark; retained cells cannot approve deployment")
    _require(len(measured) == len(cases) * 4 == 1660, "complete1660 measured denominator required")
    report = dict(schema=SCHEMA, status="COMPLETED_CPU_COUNTERFACTUAL_NUMERICAL_OVERHEAD_ONLY", prospective_declaration_sha256=declaration_sha,
        execution=execution, source_sha256_unchanged=True, measured_count=len(measured), accepted_count=sum(r["accepted"] for r in measured),
        rejected_count=sum(not r["accepted"] for r in measured), warmup_count=len(warmups), warmup_rejected_count=sum(not r["accepted"] for r in warmups),
        frozen_preview_tracker_AST_proof=tracker.ast_proof,
        preview_fallback_census=dict(Counter(r["preview05_tracker_receipt"]["fallback_reason"] or "GO_positive_effective05"
            for r in measured if r["accepted"])),
        rejection_census=dict(Counter(r["rejection_stage"] + ":" + r["rejection_code"] for r in measured if not r["accepted"])),
        measured_all_cell_overhead_seconds=_summary(r["measured_overhead_seconds"] for r in measured),
        measured_accepted_only_overhead_seconds=_summary(r["measured_overhead_seconds"] for r in measured if r["accepted"]),
        per_support={str(s): dict(count=sum(r["declared_counterfactual_support_m"] == s for r in measured),
            accepted_count=sum(r["declared_counterfactual_support_m"] == s and r["accepted"] for r in measured),
            overhead_seconds=_summary(r["measured_overhead_seconds"] for r in measured if r["declared_counterfactual_support_m"] == s)) for s in SUPPORTS_M},
        original_raw_NN_seconds_separate=_summary(c["record"]["raw_nn_seconds"] for c in cases),
        original_policy_total_seconds_separate=_summary(c["record"]["total_seconds"] for c in cases),
        old_and_new_timings_summed=False, actual_endpoint_driving_proven=False, sensor_to_plan100ms_proven=False,
        gates_unchanged=True, model_or_weights_changed=False, learned_STOP_claim=False, training_data_approved=False,
        vehicle_control_approved=False, main_functions_approved=0, subfunctions_approved=0,
        results_sha256=_sha(read_canonical_regular(output / "all_warmup_and_measured_cells.jsonl")))
    write_json_create(output / "summary.json", report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="new directory below storage/experiments/diagnostics; parent must exist")
    args = parser.parse_args()
    report = benchmark(args.output)
    print(json.dumps({key: report[key] for key in ("status", "measured_count", "accepted_count", "rejected_count",
        "measured_all_cell_overhead_seconds", "per_support", "original_raw_NN_seconds_separate", "sensor_to_plan100ms_proven")}, indent=2))


if __name__ == "__main__":
    main()
