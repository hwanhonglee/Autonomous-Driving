"""HH_261002 - Necessary true64 QA of one fixed actual LEFT347 pilot; no admission, missing-label repair or failed-window removal."""
from __future__ import annotations

import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path

from scripts.e2e import run_owned_town01_low_speed_left45_v2_20261002 as owner
from scripts.e2e import qualify_town07_signed_recovery_common10_20261001 as frozen
from scripts.e2e import audit_town07_natural_goal_future64_brakefree_v5_20261002 as strict

ROOT, task = owner.ROOT, owner.task
SCHEMA = "carla.actual_town01_left45_true_future64_diagnostic.20261002.v2"
OUTPUT = ROOT/"storage/experiments/diagnostics/2026-10-02/town01_low_speed_left45_future64_v2"
FROZEN = {"tools/e2e/town01_low_speed_left45_task_20261002.py":"9406fc63f77b61225eb2aa16ccaeeb5219a4b29da4582e82dfb4aefc769869b2",
    "tools/e2e/collect_town01_low_speed_left45_20261002.py":"e87b64c9f1df8348561c5a5ee3625e8bc6a90fde22e490b88dc45647dae36cb3",
    "tools/e2e/run_owned_town01_low_speed_left45_20261002.py":"87afc96b1da9e2ac71bf11e3b6a04583aaaea09c43a0a1e9d1af33f196c40eba",
    "tools/e2e/town01_low_speed_left45_task_v2_20261002.py":"f43d2035fe76a39eab6f5c61f54334b6b95d913b08b77e42b29ea8961ff6cca0",
    "tools/e2e/collect_town01_low_speed_left45_v2_20261002.py":"678998314c5cd367873dbe87871e9a521f18d4a2e34eccaee2458fe23703f812",
    "tools/e2e/run_owned_town01_low_speed_left45_v2_20261002.py":"dd4960ee7ce52f51eda64b7b6dfaa6b0c363e622aad7df46195dc734030cfab0",
    "tools/e2e/audit_town01_left45_closing_control_20261002.py":"2d567108d361b6b12f2c8538935cdc819d06b043dee364dbe8cc296e663fd313",
    "tools/e2e/audit_carla_acknowledged_control.py":"9d6fccb6874a5cdfc85e8bf5d9229435e4ba6fe6850eb63a9e83488f39bcc5bd",
    "tools/e2e/qualify_town07_signed_recovery_common10_20261001.py":"3280532abb005f80c4aa5bd518eafc19f6dc29eab5df62d50083b41e6ffb27bc",
    "tools/e2e/audit_town07_natural_goal_future64_brakefree_v5_20261002.py":"e96450e85d0f0afc3044041d7a78a13f9f21a7a58475f456be111018ffbf308f"}
require = frozen.require


def canonical(path):
    value=Path(path).absolute()
    require(value.is_relative_to(ROOT) and value==value.resolve() and not any(p.is_symlink() for p in (value,*value.parents)),
        "canonical workspace path with no symlink ancestors required")
    return value


def payload(path,expected,pins):
    path=canonical(path); data=path.read_bytes(); sha=hashlib.sha256(data).hexdigest()
    require(isinstance(expected,str) and len(expected)==64 and sha==expected,"external/first-read byte SHA differs: "+str(path))
    require(str(path) not in pins or pins[str(path)]==sha,"conflicting first-read source pin")
    pins[str(path)]=sha; return data


def source_pins():
    owner.runtime_sources()
    from scripts.e2e import audit_temporal_route_recovery_fast45_batch_20261001 as closure
    pins={str(path.relative_to(ROOT)):owner.digest(path) for path in closure.source_closure(Path(__file__).resolve())}
    require(all(pins.get(name)==sha for name,sha in FROZEN.items()),"frozen actual/numerical provider bytes changed")
    return dict(sorted(pins.items()))


def artifact_paths():
    case,plan=task.paths("v2","pilot_00"); episode=case/"episode"
    paths=[plan,case/"declaration.json",case/"owner_result.json",case/"diagnostic_report.json",
        case/"derived_body/summary.json",case/"derived_body/all_native_bodies_and_sweeps.jsonl.gz"]
    paths += [episode/name for name in ("manifest.json","route.json","states.jsonl","camera_frames.jsonl",
        "control_receipts.jsonl","actual_teacher_control.jsonl","actual_camera_pixels.jsonl","active_map_opendrive.xodr",
        "moving_task_plan.json","owned_green_signal_fixture.json")]
    return tuple(paths)


def strict_future(future):
    # HH_261002 - Reuse strict speed-independent defined-geometry math; original XY/yaw/speed/legacy metrics remain byte-value unchanged.
    valid=future.get("valid_mask",[False]*64)
    rows=[dict(index=i,valid=valid[i],xy=future.get("diagnostic_future_xy_m",[[None,None]]*64)[i],
        yaw_rad=future.get("diagnostic_body_yaw_delta_rad",[None]*64)[i],
        speed_mps=future.get("diagnostic_endpoint_planar_speed_mps",[None]*64)[i]) for i in range(64)]
    velocity=future.get("current_recorded_velocity_base_mps")
    require(velocity is not None,"actual anchor v0 required, never a speed clamp")
    return strict.defined_geometry(rows,initial_speed_mps=math.hypot(*velocity[:2]))


def analyze(selected,states,cameras,anchors,geometry,futures):
    windows,defects=frozen.window_checks(selected,cameras,anchors,states,geometry,futures)
    census={name:Counter() for name in ("xy_curvature","xy_lateral_acceleration","yaw_label_heading_envelope")}
    for window in windows:
        future=futures[window["camera_index"]]; witnesses=strict_future(future)
        failed=False
        for witness in witnesses:
            for name,(value,bad) in witness["metrics"].items():
                census[name]["unassessed" if value is None else "violations" if bad else "assessed_nonviolations"]+=1
                failed=failed or bad
        if failed: window["failure_reasons"].append("defined_geometry_exceeded_regardless_low_speed")
        window.update(necessary_checks_clear=not window["failure_reasons"],true_future_original_diagnostic=future,
            strict_defined_geometry_witness=witnesses,training_data_approved=False)
    return windows,defects,{name:dict(count) for name,count in census.items()}


def inspect(caller_path,caller_sha):
    pins={}; caller=json.loads(payload(caller_path,caller_sha,pins)); sources=source_pins()
    require(caller.get("schema")==SCHEMA+".caller_pins" and caller.get("source_sha256")==sources
        and caller.get("output")==str(OUTPUT),"exact externally sealed LEFT diagnostic identity required")
    artifacts=caller.get("artifact_sha256")
    require(type(artifacts) is dict and set(artifacts)=={str(path) for path in artifact_paths()},"all16 exact caller-pinned artifacts required")
    data={name:payload(name,sha,pins) for name,sha in artifacts.items()}
    for name,sha in sources.items(): payload(ROOT/name,sha,pins)
    case,plan_path=task.paths("v2","pilot_00"); episode=case/"episode"
    plan=json.loads(data[str(plan_path)]); plan_sha=artifacts[str(plan_path)]
    require(data[str(case/"declaration.json")]==data[str(plan_path)]==data[str(episode/"moving_task_plan.json")],"original declarations differ")
    owner.verify_case("v2","pilot_00",plan_sha); owner.closure_check(case,plan)
    for name,sha in plan["source_sha256"].items(): payload(ROOT/name,sha,pins);payload(case/"source"/name,sha,pins)
    # HH_261002 - External API/asset closures are independently authenticated by unchanged owned reader before and after this audit.
    manifest=json.loads(data[str(episode/"manifest.json")]); route=json.loads(data[str(episode/"route.json")])
    require(artifacts[str(episode/"route.json")]==task.ROUTE_SHA,"whole original TRAIN route changed")
    require(manifest["capture_contract"]["left45_protocol"]==task.fixed_contract("v2","pilot_00")
        and manifest["capture_contract"]["moving_task"]["plan_sha256"]==plan_sha
        and manifest["control_source"]=="Disclosed_BasicAgent_original_LEFT_teacher_not_NN", "actual capture is not the declared LEFT protocol")
    def rows(name): return [json.loads(line) for line in data[str(episode/name)].splitlines() if line.strip()]
    states,cameras,receipts,commands=(rows(name) for name in ("states.jsonl","camera_frames.jsonl","control_receipts.jsonl","actual_teacher_control.jsonl"))
    replay=task.reconstruct(plan,states,cameras,route["route"])
    require(replay.result(tail_states=sum(s["capture_phase"]=="stationary_tail" for s in states),goal_reached=False)
        ==manifest["result"]["moving_task"],"declared actual membership cannot be rewritten")
    selected=[dict(row,camera_index=next(i for i,c in enumerate(cameras) if c["frame"]==row["frame"])) for row in replay.selected]
    require(len(selected)==task.ANCHORS and len(states)==task.NATIVE_COUNT and len(cameras)==task.CAMERA_COUNT,
        "missing fixed membership/full capture is NOT_PROVEN, never an admitted subset")
    geometry=[json.loads(line) for line in gzip.decompress(data[str(case/"derived_body/all_native_bodies_and_sweeps.jsonl.gz")]).splitlines()]
    require(len(geometry)==len(states) and all(g["frame"]==s["frame"] for g,s in zip(geometry,states)),"whole body rows differ from actual native identity")
    timeline=frozen.raw.measured_timeline(states)
    pixels,images,anchors=frozen.raw.camera_audit(episode,manifest,cameras,timeline)
    for image in images: payload(episode/image["path"],image["sha256"],pins)
    owner.pixel_audit(episode,cameras)
    future_summary,futures=frozen.futures_math.geometry_futures(timeline,cameras,route,anchors,"town01_left_s0002_p03_actual_left45")
    windows,defects,strict_census=analyze(selected,states,cameras,anchors,geometry,futures)
    transport=owner.closing.analyze_transport(states,receipts,manifest["capture_contract"]["control_transport"]["bootstrap_observation"],wheelbase=2.85)
    camera=frozen.ack.analyze_camera_alignment(states,cameras)
    diagnostic=json.loads(data[str(case/"diagnostic_report.json")]); body=json.loads(data[str(case/"derived_body/summary.json")])
    for name,sha in body["input_sha256"].items(): payload(name,sha,pins)
    body_complete=body["full_body_denominator"]==task.NATIVE_COUNT and body["full_sweep_denominator"]==task.NATIVE_COUNT-1 \
        and body["ten_centimeter_inset_preserved"] is True
    violations=[window for window in windows if not window["necessary_checks_clear"]]
    denials=[]
    for validator in (lambda: frozen.adapter._require_native_admission_not_denied(manifest),
            lambda: frozen.contract._require_admission_not_denied(manifest,"actual LEFT",include_native_metadata=True)):
        try: validator()
        except Exception as error: denials.append(type(error).__name__+": "+str(error))
    quality=owner.motion_quality(states)
    clear=not violations and not defects["protected"] and not denials and quality["status"]=="PASS_SCALAR_ONLY" \
        and diagnostic["actual_teacher_capture_passed"] is True and body_complete and body["all_native_body_and_swept_clearance_passed"] is True \
        and pixels["all_pixel_and_metadata_checks_clear"] is True and transport["status"]==camera["status"]=="PASS"
    summary=dict(schema=SCHEMA,status="NECESSARY_CHECKS_CLEAR_NOT_ADMITTED" if clear else "FAIL_RETAINED_NOT_ADMITTED",
        necessary_checks_clear=bool(clear),case_plan_sha256=plan_sha,native_manifest_sha256=artifacts[str(episode/"manifest.json")],
        actual_teacher_capture_passed=diagnostic["actual_teacher_capture_passed"],raw_native_count=len(states),
        raw_camera_bundle_count=len(cameras),raw_JPEG_count=len(images),fixed_anchor_count=len(windows),
        true_future_point_count=sum(w["true_future_original_diagnostic"].get("valid_future_points",0) for w in windows),
        expected_true_future_points=task.ANCHORS*64,failed_fixed_window_count=len(violations),
        failure_reason_counts=dict(Counter(reason for w in windows for reason in w["failure_reasons"])),
        all_original_nine_future_metrics_retained=True,strict_defined_geometry_metric_counts=strict_census,
        protected_native_failures=defects["protected"],outside_protected_native_failures=defects["outside_protected"],
        all_native_motion_quality=quality,raw_native_denials=denials,raw_native_denials_overridden=False,
        original_future_summary=future_summary,whole_capture_body_summary=body,independent_transport=transport,
        independent_camera_alignment=camera,full_camera_pixel_decode_summary=pixels,
        selected_camera_span_ns=selected[-1]["timestamp_ns"]-selected[0]["timestamp_ns"],
        all_failed_windows_and_raw_startup_tail_retained=True,diagnostic_tail_context_not_STOP_training=True,
        original_V1_failed_capture_not_opened_or_promoted=True,research_training_authorized=False,training_data_approved=False,
        vehicle_control_approved=False,dataset_conversion_performed=False,learned_control=False,
        source_sha256=sources,input_sha256=dict(pins))
    owner.closure_check(case,plan)
    for name,sha in pins.items(): require(owner.digest(name)==sha,"source/input changed during true64 audit: "+name)
    return summary,windows,pins


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--caller-pins",type=Path);parser.add_argument("--caller-pins-sha256")
    parser.add_argument("--print-contract",action="store_true");parser.add_argument("--validate-only",action="store_true")
    args=parser.parse_args(argv)
    if args.print_contract:
        require(args.caller_pins is None and not args.validate_only,"contract-only does not inspect actual data")
        print(json.dumps(dict(schema=SCHEMA+".caller_pins",output=str(OUTPUT),source_sha256=source_pins(),
            required_artifact_paths=[str(path) for path in artifact_paths()],diagnostic_only=True),indent=2));return 0
    require(args.caller_pins is not None and args.caller_pins_sha256 is not None,"paired external caller/path SHA required")
    summary,windows,pins=inspect(args.caller_pins,args.caller_pins_sha256)
    if not args.validate_only:
        output=canonical(OUTPUT); require(not output.exists(),"create-only diagnostic output already exists")
        for name,sha in pins.items(): require(owner.digest(name)==sha,"input/source changed before first output write")
        output.mkdir(parents=True,exist_ok=False)
        with (output/"all347_true_future_windows.jsonl").open("x") as stream:
            for row in windows: stream.write(json.dumps(row,allow_nan=False)+"\n")
        owner.write_new(output/"summary.json",summary)
    print(json.dumps({key:summary[key] for key in ("status","fixed_anchor_count","true_future_point_count","failed_fixed_window_count","research_training_authorized")},indent=2))
    return 0 if summary["necessary_checks_clear"] else 2


if __name__=="__main__": raise SystemExit(main())
