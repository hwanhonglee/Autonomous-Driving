"""HH_261002 - Prepared publication of one real LEFT teacher; no raw reading until an external ROOT caller is pinned.

Reuse the frozen recorded-pixel/ego-centred V6 renderer and its clock/ACK/pixel
checks. LEFT has exactly one body per actual native row, not V6's extra bootstrap
body. The full existing LEFT347 qualification is rederived, never granted here.
"""
from __future__ import annotations

import argparse
import ast
import copy
import json
from pathlib import Path
import shutil
from types import FunctionType

from PIL import Image, ImageDraw

from scripts.e2e import publish_town07_natural_goal_stop_teacher_v6_20261002 as previous
from scripts.e2e import qualify_town01_left45_common10_v3_20261002 as admission

ROOT = previous.ROOT
SCHEMA = "carla.actual_town01_left45_teacher_v2_publication.20261002.v1"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
OUTPUT = ROOT / "docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/21_actual_low_speed_left_teacher_v2"
PARENT_SHA = "29ccf539330aa04ee89c3042431b44ea35f26ea145e8d99eaa8b4ced82d8eb55"
ADMISSION_SHA = "ff804fd3cbcee035c351e0dc97c57c83143adc65f9ca006338d86bac5cab3325"
COUNTS = dict(native=1055, camera=528, ACK=1060, JPEG=3168)
require, digest, canonical, rows, visual = previous.require, previous.digest, previous.canonical, previous.rows, previous.visual
CASE, CASE_PLAN = admission.task.paths("v2", "pilot_00")


def verify_frozen():
    require(digest(previous.__file__) == PARENT_SHA and digest(admission.__file__) == ADMISSION_SHA
        and digest(visual.__file__) == previous.HELPER_SHA, "frozen renderer/admission/helper bytes changed")


def public_contract():
    verify_frozen()
    qualified = admission.public_contract()
    require(len(qualified["source_sha256"]) == 99, "entire original99 qualification providers required")
    sources = dict(qualified["source_sha256"])
    for provider in (previous, visual):
        sources[str(Path(provider.__file__).resolve().relative_to(ROOT))] = digest(provider.__file__)
    sources[str(Path(__file__).resolve().relative_to(ROOT))] = digest(__file__)
    return dict(schema=CALLER_SCHEMA, source_sha256=dict(sorted(sources.items())),
        qualification_source_count=99, episode=str(CASE / "episode"), counts=dict(COUNTS),
        artifact_paths=[str(p) for p in admission.qa.artifact_paths()],
        qualification=str(admission.RECEIPT), QA_summary=str(admission.QA_SUMMARY), QA_windows=str(admission.QA_WINDOWS),
        output=str(OUTPUT), teacher_not_NN=True, manual_first_gear=True, rolling_and_diagnostic_coast_only=True,
        STOP_HOLD_GO_semantics_or_model_approval=False, GIF_rule="all528_every5th_plus_exact_last_with_actual_clock_gaps",
        PNG_rule="fixed_saved_camera_indices0_132_264_395_527", body_rows=1055, swept_pairs=1054,
        font_sha256={str(previous.FONT_ROOT / name): digest(previous.FONT_ROOT / name)
            for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf")})


def records_validator():
    # HH_261002 - Only these TWO geometry-membership clauses differ. No fabricated bootstrap record or clock/ACK relaxation.
    verify_frozen()
    original = next(n for n in ast.parse(Path(previous.__file__).read_bytes()).body
                    if isinstance(n, ast.FunctionDef) and n.name == "validate_records")
    pairs = (
        ('len(geometry) == len(states) + 1 and set(by_frame) < set(geometry)',
         'len(geometry) == len(states) and set(by_frame) == set(geometry)'),
        ('set(geometry)-set(by_frame) == {states[0]["frame"]-1}', 'set(geometry)-set(by_frame) == set()'))
    changed = copy.deepcopy(original)
    counts = []
    for old, new in pairs:
        old_node, new_node = ast.parse(old, mode="eval").body, ast.parse(new, mode="eval").body
        hits = [n for n in ast.walk(changed) if isinstance(n, ast.Call) and n.args and ast.dump(n.args[0]) == ast.dump(old_node)]
        require(len(hits) == 1 and isinstance(hits[0].func, ast.Name) and hits[0].func.id == "require", "exact original body membership guard required")
        hits[0].args[0] = new_node; counts.append((old_node, new_node))
    inverse = copy.deepcopy(changed)
    for old_node, new_node in counts:
        hits = [n for n in ast.walk(inverse) if isinstance(n, ast.Call) and n.args and ast.dump(n.args[0]) == ast.dump(new_node)]
        require(len(hits) == 1, "exact inverse body guard required"); hits[0].args[0] = old_node
    require(ast.dump(inverse) == ast.dump(original), "another original clock/camera/ACK/control guard changed")
    namespace = dict(vars(previous))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])), __file__, "exec"), namespace)
    return namespace["validate_records"]


def validate_records(states, cameras, receipts, geometry):
    return records_validator()(states, cameras, receipts, geometry, {k: COUNTS[k] for k in ("native", "camera", "ACK")})


def fixed_png_indices(bundle):
    count = len(bundle["cameras"])
    require(count == 528, "complete fixed actual camera interval required")
    # HH_261002 - Prospective time positions, not sign/turn/success-selected frames or fabricated phase events.
    return dict(zip(("01_first.png", "02_quarter.png", "03_middle.png", "04_three_quarter.png", "05_last.png"),
                    (0, 132, 264, 395, 527)))


def outcome_annotation(bundle):
    qa = bundle["future"]
    return ("Owner: " + bundle["diagnostic"]["status"] + " | true64: "
            + ("CHECKS_CLEAR" if qa["necessary_checks_clear"] else "FAIL_RETAINED")
            + f" {qa['failed_fixed_window_count']}/{qa['fixed_anchor_count']} windows"
            + " | existing research TRAIN qualification; NO model approval")


def load_archive(caller_path, caller_sha):
    pins = {}; previous.add_pin(pins, caller_path, previous.valid_sha(caller_sha))
    caller = admission.adapter._loads_json(Path(caller_path).read_text(), str(caller_path))
    wanted = public_contract()
    require(type(caller) is dict and set(caller) == {"contract", "artifact_sha256", "authority_path", "authority_sha256", "publication_authorized"}
        and caller["contract"] == wanted and caller["publication_authorized"] is True,
        "exact externally pinned LEFT publication/source/count contract required BEFORE native payloads")
    artifacts = caller["artifact_sha256"]
    required = {*wanted["artifact_paths"], wanted["qualification"], wanted["QA_summary"], wanted["QA_windows"]}
    require(type(artifacts) is dict and set(artifacts) == required, "full16 raw plus qualification/paired347 QA receipts required")
    for name, sha in wanted["source_sha256"].items(): previous.add_pin(pins, ROOT / name, previous.valid_sha(sha))
    for name, sha in wanted["font_sha256"].items(): previous.add_pin(pins, name, previous.valid_sha(sha))
    # HH_261002 - Bind every stored diagnostic before rederiving ALL347 windows; external ROOT admission grants no model capability.
    for name, sha in artifacts.items(): previous.add_pin(pins, name, previous.valid_sha(sha))
    authority_path = canonical(caller["authority_path"]); authority_sha = previous.valid_sha(caller["authority_sha256"])
    previous.add_pin(pins, authority_path, authority_sha)
    qualified = admission.evidence(authority_path, authority_sha)
    require(admission.document(admission.RECEIPT) == qualified
        and len(qualified["source_sha256"]) == 99 and qualified["selected_camera_anchors"] == 347
        and qualified["true_future_points"] == 22208 and qualified["research_training_authorized"] is True
        and qualified["model_promoted"] is False and qualified["vehicle_control_approved"] is False,
        "existing entire qualification must rederive exactly; never create or widen approval")
    for name, sha in qualified["input_sha256"].items(): previous.add_pin(pins, name, previous.valid_sha(sha))
    episode = canonical(wanted["episode"])
    plan, manifest, diagnostic = [previous.document(path) for path in
        (CASE_PLAN, episode / "manifest.json", CASE / "diagnostic_report.json")]
    require(plan["schema"] == admission.task.SCHEMA and plan["run_id"] == "v2" and plan["case_id"] == "pilot_00"
        and manifest["capture_contract"]["left45_protocol"] == admission.task.fixed_contract("v2", "pilot_00")
        and manifest["control_source"] == "Disclosed_BasicAgent_original_LEFT_teacher_not_NN",
        "actual original LEFT/manual1/rolling teacher contract required")
    for section in ("source_sha256", "external_source_sha256", "asset_sha256"):
        for name, sha in plan[section].items():
            path = canonical(ROOT / name if section == "source_sha256" else name)
            require(path.is_relative_to(ROOT) or path.is_relative_to(previous.CARLA_ROOT), "declared exact source root differs")
            archive = CASE / ("source" if path.is_relative_to(ROOT) else "source_external") / path.relative_to(ROOT if path.is_relative_to(ROOT) else previous.CARLA_ROOT)
            previous.add_pin(pins, path, previous.valid_sha(sha)); previous.add_pin(pins, archive, sha)
    states, cameras, receipts = [rows(episode / name) for name in ("states.jsonl", "camera_frames.jsonl", "control_receipts.jsonl")]
    body_rows = rows(CASE / "derived_body/all_native_bodies_and_sweeps.jsonl.gz")
    geometry = {r["frame"]: r for r in body_rows}
    require(len(geometry) == len(body_rows) == 1055, "all1055 distinct actual body frames required")
    by_frame = validate_records(states, cameras, receipts, geometry)
    previous.validate_pixels(cameras, rows(episode / "actual_camera_pixels.jsonl"))
    paths = []
    for camera in cameras:
        for name in visual.CAMERAS:
            relative = Path(camera["images"][name])
            require(not relative.is_absolute() and ".." not in relative.parts and relative.parts[:1] == ("images",), "safe actual saved camera path required")
            path = canonical(episode / relative); paths.append(path)
            previous.add_pin(pins, path, previous.valid_sha(camera["saved_JPEG_sha256"][name]))
            with Image.open(path) as picture:
                require(picture.format == "JPEG" and picture.size == (640, 360), "original full-FOV six-camera JPEG required")
                picture.load()
    require(len(paths) == len(set(paths)) == 3168, "all3168 distinct actual JPEGs required")
    future = previous.document(admission.QA_SUMMARY)
    require(future["fixed_anchor_count"] == 347 and future["true_future_point_count"] == 22208
        and future["failed_fixed_window_count"] == 0 and future["necessary_checks_clear"] is True,
        "preserve the existing all347 real future QA outcome")
    previous.recheck(pins); verify_frozen()
    return dict(episode=episode, states=states, cameras=cameras, receipts=receipts, geometry=geometry, by_frame=by_frame,
        route=previous.document(episode / "route.json")["route"], diagnostic=diagnostic, future=future,
        qualification=qualified, pins=pins)


def render(bundle, index):
    # HH_261002 - Exact recorded pixels and reviewed map-panel math; only truthful LEFT labels replace the terminal-teacher labels.
    namespace = dict(vars(previous), outcome_annotation=outcome_annotation)
    picture = FunctionType(previous.render.__code__, namespace, "render")(bundle, index)
    draw = ImageDraw.Draw(picture)
    draw.rectangle((0, 0, 1699, 93), fill=visual.BG)
    draw.text((24, 18), "Town01 V2 ACTUAL LEFT TEACHER | " + outcome_annotation(bundle), fill=visual.AMBER, font=visual.font(20, True))
    draw.text((24, 63), "BasicAgent / manual1 / rolling LEFT + diagnostic coast | NO neural control | NOT Autoware/RViz", fill=visual.MUTED, font=visual.font(18))
    draw.rectangle((0, 752, 1699, 859), fill=visual.BG)
    draw.text((24, 770), "Pedals above are actually observed native control; prior ACK is identity, not a new measured effect.", fill=visual.AMBER, font=visual.font(18))
    draw.text((24, 809), "Whole52.7s warmup/rolling/future/coast context | NOT GO/HOLD/STOP supervision or model-driving success", fill=visual.MUTED, font=visual.font(17))
    return picture


def publisher_function():
    verify_frozen()
    original = next(n for n in ast.parse(Path(previous.__file__).read_bytes()).body
                    if isinstance(n, ast.FunctionDef) and n.name == "publish")
    changed = copy.deepcopy(original)
    translations = {
        "# 실제 목적지 정차 교사 주행 v6\n\n이 화면은 실제 기록된 6카메라·측정 차량 위치·경로를 보여줍니다. BasicAgent 교사 제어이며 모델 자율주행이나 Autoware/RViz 화면이 아닙니다.\n\n":
        "# 실제 저속 좌회전 교사 v2 — 전체 기록\n\n실제 6카메라·측정 차량 위치·경로입니다. BasicAgent/manual1 교사이며 모델 자율주행이나 Autoware/RViz 화면이 아닙니다. rolling 좌회전과 진단 coast 기록이며 목적지 GO/HOLD/STOP 학습이 아닙니다.\n\n",
        "`. 실패·대기 상태와 전체 출발·정지 유지·후속 기록을 보존하며 학습 승인으로 해석하지 않습니다.\n\n":
        "`. 전체347 구간·실제 미래22,208점의 기존 연구 TRAIN 자격은 외부 봉인된 별도 영수증을 재검증했습니다. 이 화면 자체는 모델·차량·MAIN9/sub30 승인이나 새 학습 승인이 아닙니다. 전체 출발·rolling·coast 기록을 보존합니다.\n\n"}
    counts = dict.fromkeys(translations, 0)
    for node in ast.walk(changed):
        if isinstance(node, ast.Constant) and type(node.value) is str and node.value in translations:
            counts[node.value] += 1; node.value = translations[node.value]
    require(all(v == 1 for v in counts.values()), "exact two human-readable publication labels required")
    inverse = copy.deepcopy(changed)
    for node in ast.walk(inverse):
        if isinstance(node, ast.Constant) and type(node.value) is str and node.value in translations.values():
            node.value = next(old for old, new in translations.items() if new == node.value)
    require(ast.dump(inverse) == ast.dump(original), "another original GIF/PNG/provenance/write statement changed")
    namespace = dict(vars(previous), OUTPUT=OUTPUT, SCHEMA=SCHEMA, load_archive=load_archive, render=render,
        outcome_annotation=outcome_annotation,
        png_indices=fixed_png_indices, missing_png_phases=lambda bundle: {}, __file__=__file__)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])), __file__, "exec"), namespace)
    return namespace["publish"]


def publish(output, caller_path, caller_sha):
    require(canonical(output) == OUTPUT and not OUTPUT.exists(), "fresh fixed LEFT teacher category required")
    require(shutil.disk_usage(ROOT).free >= 5 * 1024**3 + 64 * 1024**2, "5GiB floor +64MiB publication reserve required")
    return publisher_function()(output, caller_path, caller_sha)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--print-contract", action="store_true")
    parser.add_argument("--caller-pins", type=Path); parser.add_argument("--caller-pins-sha256")
    parser.add_argument("--output", type=Path, default=OUTPUT); args = parser.parse_args(argv)
    if args.print_contract:
        require(args.caller_pins is None and args.caller_pins_sha256 is None, "contract-only opens no actual raw data")
        print(json.dumps(public_contract(), indent=2, allow_nan=False)); return 0
    require(args.caller_pins is not None and args.caller_pins_sha256 is not None, "external ROOT publication caller+SHA required")
    print(json.dumps(publish(args.output, args.caller_pins, args.caller_pins_sha256), allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
