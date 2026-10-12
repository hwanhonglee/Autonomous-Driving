"""HH_261002 - Admit exactly one externally authorized LEFT347 TRAIN source; unchanged native denials and frozen math."""
from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import json
from pathlib import Path
import shutil

from scripts.e2e import audit_town01_left45_true_future64_20261002 as qa
from scripts.e2e import qualify_town07_signed_recovery_common10_20261001 as frozen
from scripts.e2e import prepare_carla_common10_dataset as adapter

ROOT, owner, task = qa.ROOT, qa.owner, qa.task
require, safe, sha, document = frozen.require, frozen.safe, frozen.sha, frozen.document
SCHEMA = "carla.actual_town01_left45_common10_qualification.20261002.v2"
RESULT_SCHEMA = "carla.actual_town01_left45_common10_conversion.20261002.v2"
ADAPTER_ID = "carla_actual_town01_left347_to_common10_20261002_v2"
PREPARED = ROOT/"storage/datasets/prepared/town01-low-speed-left45-train-20261002-v2"
RECEIPT = ROOT/"storage/experiments/training/2026-10-02/town01_left45_admission_v2/qualification.json"
CONVERTER = ROOT/"tools/e2e/prepare_town01_left45_common10_20261002.py"
QA_SUMMARY = qa.OUTPUT/"summary.json"
QA_WINDOWS = qa.OUTPUT/"all347_true_future_windows.jsonl"
FROZEN = dict(qa.FROZEN, **{
    "tools/e2e/audit_town01_left45_true_future64_20261002.py":"f08dab566d6fcca8fac7a0e39b01cf1ba4a8dbad44091c0f9b5950cb76aecbda",
    "tools/e2e/prepare_carla_common10_dataset.py":"eab12ef9d83f7ca72d8396ac46d77082cbafbc9600b44c59d733cb76e0de7e65",
    "tools/e2e/qualify_carla_moving_common10.py":"520acdb96f133d062f5e4391f66270be6ac4c449758718cfd2c6ba0819470ffe",
    "learning/portable_e2e/temporal_dataset_v2.py":"9f27fd12f7302ed654df4c2c956697d794bb15243c9576681e0300a226cefafe",
    "learning/portable_e2e/dataset.py":"3b30a7b4db1907a10ad94002bb880deaa4bb545a978bad3beec1b0aacf3b9c95",
    "learning/portable_e2e/contract.py":"e2e3a2669fa94aa98cb7e50f20ffa7db433b99eead9a12c654392e4b293a5c09"})
SCOPE = dict(rolling_LEFT_only=True, startup_anchors_used_for_training=False,
    STOP_or_stationary_tail_supervision=False, raw_startup_and_tail_retained_as_context_only=True,
    fixed_driving_relative_anchor_offsets=[80,772], fixed_anchor_driving_seconds=[4.,38.6],
    new_independent_site_or_sealed_TEST_claimed=False, original_V1_failed_capture_not_qualified=True)


def public_contract():
    from scripts.e2e import audit_temporal_route_recovery_fast45_batch_20261001 as closure
    sources={str(path.relative_to(ROOT)):sha(path) for path in closure.source_closure(Path(__file__).resolve(),CONVERTER)}
    sources["learning/portable_e2e/config/common_10hz_v1.contract.json"]=sha(ROOT/"learning/portable_e2e/config/common_10hz_v1.contract.json")
    require(all(sources.get(name)==pin for name,pin in FROZEN.items()),"original numerical/denial providers changed")
    case,plan=task.paths("v2","pilot_00")
    return dict(schema=SCHEMA,source_sha256=dict(sorted(sources.items())),native_episode=str(case/"episode"),
        case_plan=str(plan),receipt_output=str(RECEIPT),prepared_output=str(PREPARED),dataset_id=PREPARED.name,
        QA_summary=str(QA_SUMMARY),QA_windows=str(QA_WINDOWS),split="train",episode_count=1,
        selected_camera_anchors=347,true_future_points=22208,validate_mode="planning",image_mode="copy",**SCOPE)


def read_pinned(path,expected,pins,*,rows=False):
    path=safe(path); raw=path.read_bytes(); actual=hashlib.sha256(raw).hexdigest()
    require(actual==frozen.independent.hash_value(expected),"external/first-read bytes changed: "+str(path))
    require(str(path) not in pins or pins[str(path)]==actual,"conflicting first-read source pin")
    pins[str(path)]=actual
    if rows:
        return tuple(adapter._loads_json(line.decode("utf-8"),str(path)+":"+str(index))
            for index,line in enumerate(raw.splitlines(),1) if line.strip())
    value=adapter._loads_json(raw.decode("utf-8"),str(path))
    require(type(value) is dict,"externally pinned JSON must be an object");return value


def scope_manifest(manifest):
    # HH_261002 - The original explicit-False guards remain authoritative; absence/PENDING is not approval by itself.
    frozen.scope_manifest(manifest)
    require(manifest["control_source"]=="Disclosed_BasicAgent_original_LEFT_teacher_not_NN"
        and manifest["capture_contract"]["left45_protocol"]==task.fixed_contract("v2","pilot_00"),
        "only the exact prospectively declared LEFT V2 native source may qualify")


def recheck(qualification):
    value=qualification["document"]
    require(sha(qualification["receipt_path"])==qualification["receipt_sha256"],"qualification receipt changed")
    for name,pin in value["input_sha256"].items():require(sha(name)==pin,"qualified raw or QA artifact changed")
    for name,pin in value["source_sha256"].items():require(sha(ROOT/name)==pin,"qualified conversion/math source changed")


def evidence(authority_path,authority_sha):
    wanted=public_contract();pins={}
    authority_path=safe(authority_path)
    require(authority_path.is_relative_to(ROOT/"storage/experiments/plans/2026-10-02"),"ROOT plan authority required")
    authority=read_pinned(authority_path,authority_sha,pins)
    require(authority.get("schema")==SCHEMA+".external_ROOT_authority" and authority.get("contract")==wanted
        and authority.get("research_TRAIN_admission_authorized") is True
        and authority.get("Common10_conversion_authorized") is True
        and all(authority.get(key) is False for key in ("vehicle_control_approved","model_promoted","formal_feature_approval")),
        "explicit exact-source bounded TRAIN authority required before native payloads")
    caller_path=safe(authority["QA_caller_path"])
    require(caller_path.is_relative_to(ROOT/"storage/experiments/plans/2026-10-02"),"original ROOT QA caller must remain in plans")
    caller=read_pinned(caller_path,authority["QA_caller_sha256"],pins)
    require(set(caller.get("artifact_sha256",{}))=={str(path) for path in qa.artifact_paths()},"all original16 actual QA inputs required")
    episode=Path(wanted["native_episode"])
    manifest=read_pinned(episode/"manifest.json",caller["artifact_sha256"][str(episode/"manifest.json")],pins)
    scope_manifest(manifest)
    saved=read_pinned(QA_SUMMARY,authority["QA_summary_sha256"],pins)
    saved_windows=read_pinned(QA_WINDOWS,authority["QA_windows_sha256"],pins,rows=True)
    actual,windows,replayed_pins=qa.inspect(caller_path,authority["QA_caller_sha256"])
    require(saved==actual and list(saved_windows)==windows,"full immutable347 QA must rederive exactly, not summary-only authorization")
    require(actual["status"]=="NECESSARY_CHECKS_CLEAR_NOT_ADMITTED" and actual["necessary_checks_clear"] is True
        and actual["actual_teacher_capture_passed"] is True and actual["fixed_anchor_count"]==347
        and actual["true_future_point_count"]==22208 and actual["failed_fixed_window_count"]==0
        and actual["raw_native_denials"]==[] and actual["raw_native_denials_overridden"] is False,
        "all original actual body/ACK/native/pixels/nine+strict future requirements must pass")
    for name,pin in replayed_pins.items():
        require(name not in pins or pins[name]==pin,"replayed QA input conflicts with external first-read pin");pins[name]=pin
    route=document(episode/"route.json")
    require(sha(episode/"route.json")==task.ROUTE_SHA and route["town"]=="Town01" and route["scenario"]=="left",
        "unchanged original TRAIN LEFT route required")
    entry=dict(case_id="pilot_00",native_episode=str(episode),dataset_split_intent="train",seed=task.CASE["seed"],
        case_plan_sha256=actual["case_plan_sha256"],native_manifest_sha256=sha(episode/"manifest.json"),
        native_state_sha256=sha(episode/"states.jsonl"),native_camera_sha256=sha(episode/"camera_frames.jsonl"),
        source_route_sha256=task.ROUTE_SHA,signed_lateral_offset_ros_m=0.,signed_yaw_offset_ros_deg=0.,
        selected_camera_frames=[row["frame"] for row in windows],
        selected_anchors=[{key:row[key] for key in ("frame","timestamp_ns","camera_index")} for row in windows],
        route_identity=dict(route_id=task.ROUTE.stem,route_sha256=task.ROUTE_SHA,
            identity_source="unchanged_original_TRAIN_catalog_route",coordinate_alignment_synthesized=False))
    require(len(set(entry["selected_camera_frames"]))==347 and all(not row["failure_reasons"] for row in windows),"entire fixed membership required")
    value=dict(schema=SCHEMA,status="QUALIFIED_RESEARCH_TRAIN_LEFT347",group="train",run_id="v2",dataset_id=PREPARED.name,
        prepared_output=str(PREPARED),episode_count=1,selected_camera_anchors=347,true_future_points=22208,cases=[entry],
        selected_camera_frames=entry["selected_camera_frames"],route_identity=entry["route_identity"],
        external_ROOT_authority_path=str(authority_path),external_ROOT_authority_sha256=authority_sha,
        original_QA_caller_sha256=authority["QA_caller_sha256"],original_QA_summary_sha256=authority["QA_summary_sha256"],
        original_QA_windows_sha256=authority["QA_windows_sha256"],all_original_QA_windows_retained_by_immutable_external_journal=True,
        source_sha256=wanted["source_sha256"],input_sha256=pins,
        research_training_authorized=True,raw_native_denials_overridden=False,labels_clipped=False,
        labels_or_pixels_transformed=False,synthetic_history_created=False,vehicle_control_approved=False,
        model_promoted=False,formal_MAIN9_approved=0,formal_sub30_approved=0,**SCOPE)
    require(public_contract()==wanted,"admission/conversion source closure changed during evidence")
    for name,pin in pins.items():require(sha(name)==pin,"raw/QA input changed during qualification")
    return value


def entry_for(spec,qualification):
    value=qualification["document"];entry=value["cases"][0]
    require(value["schema"]==SCHEMA and value["research_training_authorized"] is True and len(value["cases"])==1
        and spec.split=="train" and safe(spec.path)==safe(entry["native_episode"]),"exact one TRAIN347 source required; no DEV/TEST or V1")
    return entry


def authorize(args):
    require(getattr(args,"execute_reviewed_left45_conversion",False) is True,"explicit reviewed LEFT347 conversion required")
    path=safe(args.moving_qualification);expected=frozen.independent.hash_value(args.moving_qualification_sha256)
    require(path==RECEIPT and sha(path)==expected,"externally pinned exact LEFT qualification receipt required")
    value=document(path)
    require(value==evidence(value["external_ROOT_authority_path"],value["external_ROOT_authority_sha256"]),"receipt does not rederive all unchanged actual evidence")
    require(args.dataset_id==PREPARED.name and safe(args.output)==PREPARED and args.image_mode=="copy" and args.validate_mode=="planning"
        and len(args.episode)==1,"exact create-only copied single-episode planning output required")
    qualification=dict(receipt_path=str(path),receipt_sha256=expected,document=value,native_objects=[])
    entry=entry_for(args.episode[0],qualification)
    copied=sum((args.episode[0].path/f"images/{name}/{frame:08d}.jpg").stat().st_size
        for frame in entry["selected_camera_frames"] for name in frozen.contract.CAMERA_ORDER)
    require(shutil.disk_usage(ROOT).free>=5*1024**3+copied+512*1024**2,"5GiB floor +independent selected copies +512MiB reserve required")
    recheck(qualification);return qualification


def verify_native_scope(spec,manifest,qualification):
    scope_manifest(manifest);entry=entry_for(spec,qualification)
    require(sha(spec.path/"manifest.json")==entry["native_manifest_sha256"],"native manifest changed")


def native_route_id(spec,manifest,route,qualification):
    verify_native_scope(spec,manifest,qualification);entry=entry_for(spec,qualification)
    require(sha(spec.path/"route.json")==entry["source_route_sha256"],"full causal route changed")
    return entry["route_identity"]["route_id"]


def prepared_receipt(values):
    q=values["qualification"];value=q["document"];root=values["root"];loaded=values["loaded"];view=values["view"]
    require(len(values["case_records"])==1 and values["full_camera"]==339 and values["full_ego"]==338,
        "actual selected-only honest history denominators differ")
    return dict(schema=RESULT_SCHEMA,status="QUALIFIED_RESEARCH_TRAIN_LEFT347_COMMON10",dataset_manifest_sha256=sha(root/"dataset.json"),
        validation_report_sha256=sha(root/"validation_report.json"),qualification_receipt_sha256=q["receipt_sha256"],
        train_loader_fingerprint_sha256=loaded.fingerprint_sha256,temporal_dataset_fingerprint_sha256=view.fingerprint_sha256,
        episode_count=1,train_examples=347,val_examples=0,test_examples=0,true_future_points=22208,
        per_episode_native_qualification=values["case_records"],source_sha256=value["source_sha256"],
        external_ROOT_authority_sha256=value["external_ROOT_authority_sha256"],research_training_authorized=True,
        per_episode_first_nine_temporal_history_masks=values["boundaries"],full_five_camera_history_examples=339,full_ten_ego_history_examples=338,
        signed_current_state_audit_sha256=sha(values["signed_path"]),signed_state_diagnostic_not_model_input=True,
        exact_native_ego_navigation_and_target_reconstruction=True,unchanged_target_audit=values["measured"],
        causal_navigation_support_m=dict(minimum=min(values["supports"]),maximum=max(values["supports"])),
        actual_current_planar_speed_mps=dict(minimum=min(values["measured_speeds"]),maximum=max(values["measured_speeds"])),
        raw_native_denials_overridden=False,original_pending_metadata_unchanged=True,labels_clipped=False,
        labels_or_pixels_transformed=False,synthetic_history_created=False,future_labels_used_as_model_input=False,
        unselected_raw_context_preserved_in_source_manifest=True,actual_hypot_current_speed_no_clipping=True,
        vehicle_control_approved=False,model_promoted=False,formal_MAIN9_approved=0,formal_sub30_approved=0,**SCOPE)


def validation_function():
    # HH_261002 - ALL copied-pixel/nav/native/true64/mask mathematics are the exact frozen function; replace only recovery-specific result metadata.
    source=Path(frozen.__file__)
    require(sha(source)==FROZEN["tools/e2e/qualify_town07_signed_recovery_common10_20261001.py"],"frozen conversion proof changed")
    original=next(node for node in ast.parse(source.read_bytes()).body if isinstance(node,ast.FunctionDef) and node.name=="validate_prepared")
    changed=copy.deepcopy(original);require(isinstance(changed.body[-1],ast.Return),"exact original terminal result required")
    changed.body[-1]=ast.Return(value=ast.parse("prepared_receipt(locals())",mode="eval").body)
    inverse=copy.deepcopy(changed);inverse.body[-1]=copy.deepcopy(original.body[-1])
    require(ast.dump(inverse)==ast.dump(original),"another conversion numerical/denial/mask check changed")
    namespace=dict(vars(frozen),COUNT=347,CAMERA=528,ADAPTER_ID=ADAPTER_ID,recheck=recheck,entry_for=entry_for,
        sha=sha,document=document,prepared_receipt=prepared_receipt)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),__file__,"exec"),namespace)
    return namespace["validate_prepared"]


def validate_prepared(prepared,native,qualification,native_report):
    return validation_function()(prepared,native,qualification,native_report)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--print-contract",action="store_true");parser.add_argument("--authority",type=Path);parser.add_argument("--authority-sha256")
    args=parser.parse_args(argv)
    if args.print_contract:
        require(args.authority is None and args.authority_sha256 is None,"source contract opens no actual raw data")
        print(json.dumps(public_contract(),indent=2,allow_nan=False));return 0
    require(args.authority is not None and args.authority_sha256 is not None,"paired external ROOT authority required")
    output=safe(RECEIPT);require(not output.exists(),"fresh create-only qualification receipt required")
    value=evidence(args.authority,args.authority_sha256)
    for name,pin in value["input_sha256"].items():require(sha(name)==pin,"input changed before receipt write")
    require(public_contract()["source_sha256"]==value["source_sha256"],"source changed before receipt write")
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open("x") as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write("\n")
    print(json.dumps(dict(status=value["status"],receipt_path=str(output),receipt_sha256=sha(output),selected_anchors=347,
        research_training_authorized=True,vehicle_control_approved=False)));return 0


if __name__=="__main__":raise SystemExit(main())
