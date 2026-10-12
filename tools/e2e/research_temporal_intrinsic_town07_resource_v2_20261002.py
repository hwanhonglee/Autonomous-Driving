"""HH_261002 - Account only externally authenticated closed writes; preserve all six signed45 trials and original quotas/gates."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import shutil

from . import research_temporal_intrinsic_town07_signed45_20261002 as previous

ROOT,base,protocol=previous.ROOT,previous.base,previous.protocol
PARENT_SHA="8b69c31359cdab27b85d236d82f05623b2fcc89fe53d5dd906381b9427f67d33"
SCHEMA="portable_e2e.owned_intrinsic_pair_town07_signed45.20261002.v2"
AUTH_SCHEMA=SCHEMA+".ROOT_closed_case_resource_authority"
MODULE="scripts.e2e.research_temporal_intrinsic_town07_resource_v2_20261002"
ORDER=[(arm,index) for arm in previous.ARMS for index in range(3)]
require,digest=previous.require,previous.digest
_fixed,_sources=previous.fixed_contract,previous.runtime_sources
_active,_mode,_target,_resource_path,_resource_sha=False,None,None,None,None
_original={name:getattr(previous,name) for name in ("pair_root","SCHEMA","MODULE","fixed_contract","runtime_sources","disk_check")}


def pair_root():return ROOT/"storage/experiments/validation/2026-10-02/temporal_intrinsic_pair_town07_signed45_v2"


def authority_root():return ROOT/"storage/experiments/plans/2026-10-02/intrinsic_town07_resource_v2"


def read_json(path):
    require(path.resolve()==path and not any(p.is_symlink() for p in (path,*path.parents))
        and path.is_file() and path.stat().st_size<=4*1024**2,"bounded canonical resource receipt required")
    def pairs(items):
        require(len({k for k,v in items})==len(items),"duplicate resource JSON field")
        return dict(items)
    return json.loads(path.read_text(),object_pairs_hook=pairs,
        parse_constant=lambda x:(_ for _ in ()).throw(ValueError("nonfinite resource JSON")))


def runtime_sources():
    result=_sources()
    for path in base.imported_source_closure((Path(__file__).resolve(),)):
        name,sha=str(path.relative_to(ROOT)),digest(path)
        require(name not in result or result[name]==sha,"resource-only import closure conflicts")
        result[name]=sha
    require(result.get(str(Path(previous.__file__).relative_to(ROOT)))==PARENT_SHA,"frozen six-case owner changed")
    return result


def fixed_contract(run_id,longitudinal_pi):
    result=_fixed(run_id,longitudinal_pi)
    result.update(resource_accounting_revision=AUTH_SCHEMA,
        full_six_reserve_required_before_declaration=True,
        completed_write_reserve_released_only_by_external_ROOT_full_artifact_closure=True,
        fixed_physical_execution_order=[dict(arm=a,trial_index=i) for a,i in ORDER],
        closed_FAILED_cases_retained_in_three_plus_three_denominator=True,
        free_disk_floor_episode_sample_and_attempt_reserves_unchanged=True,
        resource_verification_runs_before_parent_preflight_not_inside_NN_policy=True)
    return result


def inventory(directory):
    require(directory.is_dir() and directory.resolve()==directory
        and not any(p.is_symlink() for p in (directory,*directory.parents)),"canonical actual case output required")
    result={}
    for path in sorted(directory.rglob("*")):
        require(not path.is_symlink() and path.resolve().is_relative_to(directory),"case artifact escape/symlink forbidden")
        if path.is_dir():continue
        require(path.is_file(),"regular actual case artifact required")
        result[str(path.relative_to(directory))]=dict(sha256=digest(path),bytes=path.stat().st_size)
    return result


def closed_case(row,ordinal,pair):
    arm,index=ORDER[ordinal]
    require(type(row) is dict and row.get("arm")==arm and type(row.get("trial_index")) is int
        and row["trial_index"]==index,"only exact closed ordered prefix, never selected cases")
    folder=pair_root()/arm/("trial_%02d"%index)
    batch=read_json(pair_root()/(arm+"_batch_plan.json"))
    require(digest(pair_root()/(arm+"_batch_plan.json"))==pair["arm_batch_sha256"][arm],"closed arm batch changed")
    case=batch["trials"][index];plan_path=Path(case["declaration_path"])
    plan=read_json(plan_path)
    require(digest(plan_path)==case["declaration_sha256"]==row.get("case_plan_sha256")
        and plan.get("output")==str(folder) and plan.get("arm")==arm and plan.get("trial_index")==index,
        "closed case differs from one of all six prospective declarations")
    require(digest(folder/"declaration.json")==case["declaration_sha256"],"closed archived declaration changed")
    own_path,diag_path=folder/"owner_result.json",folder/"diagnostic_report.json"
    require(digest(own_path)==row.get("owner_result_sha256") and digest(diag_path)==row.get("diagnostic_report_sha256"),
        "ROOT must pin actual owner and diagnostic bytes, not infer closure from plan")
    own,diagnostic=read_json(own_path),read_json(diag_path)
    require(own.get("status") in ("RESEARCH_CAPTURE_COMPLETE","CAPTURE_RETURNED_NONZERO_RETAINED")
        and own.get("declaration_sha256")==case["declaration_sha256"] and own.get("cleanup_errors")==[]
        and own.get("source_hashes_match") is True and own.get("external_source_hashes_match") is True
        and own.get("port_release",{}).get("original_TCP_UDP_bind_checks_passed") is True
        and diagnostic.get("ownership_passed") is True and diagnostic.get("schema")==SCHEMA+".diagnostic_report"
        and diagnostic.get("arm")==arm and diagnostic.get("original_owner")==own,
        "only actual same-case clean-closed ownership releases future-write reserve")
    started=read_json(folder/"owner_started.json")
    require(started.get("declaration_sha256")==case["declaration_sha256"]
        and all(type(started.get(k)) is dict and type(started[k].get("pid")) is int and started[k]["pid"]>0
            and type(started[k].get("start_ticks")) is int and started[k]["start_ticks"]>0
            for k in ("owner","server","socket_owner")),"actual owned launch receipt required, not NOT_EXECUTED")
    protocol.verify_derived_source_closure(folder,plan)
    actual=inventory(folder)
    require(actual==row.get("artifact_inventory") and len(actual)>5
        and {"owner_started.json","owner_result.json","diagnostic_report.json","declaration.json","server.log","collector.log"}<=set(actual),
        "complete actual closed artifact SHA/size inventory required")
    require(all(type(v) is dict and set(v)=={"sha256","bytes"} and previous.loader.is_sha(v["sha256"])
        and type(v["bytes"]) is int and v["bytes"]>=0 for v in row["artifact_inventory"].values()),"typed full artifact inventory required")
    episodes=[folder/name for name in ("episode","episode.partial") if (folder/name).is_dir()]
    require(len(episodes)==1,"one retained actual episode or failed partial required")
    episode_bytes=base.directory_bytes(episodes[0])
    require(type(row.get("episode_bytes")) is int and row["episode_bytes"]==episode_bytes<=base.EPISODE_QUOTA,
        "unchanged actual512MiB episode quota required")
    # HH_261002 - This immutable profile writes no exact tensor samples; deny unknown sample claims instead of exempting128MiB.
    require(plan.get("exact_normalized_tensor_archive_retained") is False
        and diagnostic.get("exact_normalized_tensor_archive_retained") is False
        and type(row.get("sampled_exact_input_bytes")) is int and row["sampled_exact_input_bytes"]==0<=base.SAMPLE_QUOTA,
        "original unused128MiB sampling allowance must remain unused")
    archived=sum(base.directory_bytes(folder/name) for name in ("source","source_external"))
    total=sum(v["bytes"] for v in actual.values())
    require(total-archived<=base.ARM_RESERVE,"closed nonarchive output exceeds unchanged original attempt reserve")
    require(inventory(folder)==actual,"closed actual files changed during resource authentication")
    return dict(arm=arm,trial_index=index,owner_status=own["status"],diagnostic_status=diagnostic.get("status"),
        episode_bytes=episode_bytes,total_retained_bytes=total,owner_result_sha256=row["owner_result_sha256"],
        diagnostic_report_sha256=row["diagnostic_report_sha256"],performance_PASS_not_required_for_accounting=True)


def closed_prefix():
    if _resource_path is None:return []
    require(previous.loader.is_sha(_resource_sha) and _resource_path.is_relative_to(authority_root())
        and digest(_resource_path)==_resource_sha,"external ROOT closed-case authority bytes/path required")
    value=read_json(_resource_path)
    require(value.get("schema")==AUTH_SCHEMA and value.get("resource_accounting_authorized") is True
        and value.get("paired_batch_sha256")==previous._pair_sha
        and value.get("completed_pair_authority_sha256")==previous._scope.authority_sha
        and value.get("closed_failed_cases_retained") is True and value.get("model_promoted") is False
        and value.get("vehicle_control_approved") is False,"exact same six-case ROOT resource scope required")
    pair=previous.verify_pair(previous._pair_sha)
    rows=value.get("closed_cases")
    require(type(rows) is list and len(rows)<=6,"full ordered closed-case list required")
    result=[closed_case(row,i,pair) for i,row in enumerate(rows)]
    require(digest(_resource_path)==_resource_sha,"ROOT resource authority changed during authentication")
    return result


def disk_check(free=None):
    require(previous._scope is not None,"actual same six-case asset scope required")
    completed=[] if _mode in ("declare","contract") else closed_prefix()
    if _mode in ("preflight","execute"):
        require(_target==len(completed),"next fixed case requires authenticated exact completed prefix, no skip or retry")
    remaining=6-len(completed)
    value=shutil.disk_usage(ROOT).free if free is None else free
    asset_bytes=sum(path.stat().st_size for path in previous._scope.assets)
    required=base.FLOOR+remaining*(base.ARM_RESERVE+asset_bytes)+base.SHARED_RESERVE
    require(type(value) is int and value>=required,"unchanged floor plus complete remaining-case write/archive reserve unavailable")
    return dict(free_bytes=value,required_bytes=required,prospective_trials=6,remaining_write_trials=remaining,
        authenticated_closed_trial_count=len(completed),closed_case_evidence=completed,
        external_ROOT_resource_authority_sha256=_resource_sha,full_six_required_at_declaration=True,
        episode_quota_bytes=base.EPISODE_QUOTA,sampled_exact_input_quota_bytes=base.SAMPLE_QUOTA,
        free_disk_floor_bytes=base.FLOOR,remaining_source_asset_archive_reserve_bytes=remaining*asset_bytes,
        all_six_failures_and_successes_remain_in_denominator=True)


@contextmanager
def activated(mode,target,path,pin):
    global _active,_mode,_target,_resource_path,_resource_sha
    require(not _active and not previous._active and digest(Path(previous.__file__))==PARENT_SHA,"frozen or nested six-case resource owner")
    require(all(getattr(previous,k) is v if callable(v) else getattr(previous,k)==v for k,v in _original.items()),"unknown original resource bindings")
    require((path is None)==(pin is None),"resource authority path+SHA must be supplied together")
    changes=dict(pair_root=pair_root,SCHEMA=SCHEMA,MODULE=MODULE,fixed_contract=fixed_contract,runtime_sources=runtime_sources,disk_check=disk_check)
    saved={k:getattr(previous,k) for k in changes}
    _active,_mode,_target,_resource_path,_resource_sha=True,mode,target,path,pin
    try:
        for k,v in changes.items():setattr(previous,k,v)
        yield
    finally:
        for k,v in saved.items():setattr(previous,k,v)
        _active,_mode,_target,_resource_path,_resource_sha=False,None,None,None,None


def main(argv=None):
    parser=argparse.ArgumentParser(add_help=False,allow_abbrev=False)
    parser.add_argument("--closed-cases-authority",type=Path);parser.add_argument("--closed-cases-authority-sha256")
    extra,remaining=parser.parse_known_args(argv)
    mode_parser=argparse.ArgumentParser(add_help=False,allow_abbrev=False)
    mode_parser.add_argument("--mode",default="contract");mode_parser.add_argument("--arm",choices=previous.ARMS)
    mode_parser.add_argument("--trial",type=int,choices=(0,1,2),default=0)
    selection,_=mode_parser.parse_known_args(remaining)
    target=ORDER.index((selection.arm,selection.trial)) if selection.arm is not None else None
    with activated(selection.mode,target,extra.closed_cases_authority,extra.closed_cases_authority_sha256):
        return previous.main(remaining)


if __name__=="__main__":raise SystemExit(main())
