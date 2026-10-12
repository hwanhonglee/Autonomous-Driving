"""HH_261002 - Prospectively bracket0.3/0.8 tracking with0.5; preserve every signed45 physical and latency guard."""

from __future__ import annotations

import ast
from contextlib import contextmanager
import copy
from pathlib import Path
import re

from . import research_temporal_route_joint_preview08_signed45_v2_town07_20261002 as previous
from . import collect_temporal_route_joint_preview05_signed45_20261002 as live

ROOT, base, parent, proto = previous.ROOT, previous.base, previous.parent, previous.previous
require, digest, read_json = previous.require, previous.digest, previous.previous.read_json
MODULE = "scripts.e2e.research_temporal_route_joint_preview05_signed45_town07_20261002"
SCHEMA = "portable_e2e.actual_route_joint_thread4_preview05_signed45_three_arms.20261002.v1"
FROZEN_OWNER_SHA = "f46594c8c065a2da658fb7e743ecfdb97fae48ceaf201a666feef0f8a1d80894"
PREVIEW08_BATCH = ROOT/"storage/experiments/validation/2026-10-02/temporal_route_joint_preview08_compat_v2_signed45_town07_v1"
PREVIEW08_PLAN = PREVIEW08_BATCH.with_name(PREVIEW08_BATCH.name+"_plan.json")
PREVIEW08_PLAN_SHA = "a7387543f35afb5eb04d1931b9a46d16a583ef02ea69b984e9ea236f620a867b"
_fixed, _sources = previous.fixed_contract, previous.runtime_sources
_contract, _configuration, _diagnostic, _summary = proto.calibration_contract, proto.configuration_report, proto.diagnostic_report, proto.batch_summary
_disk = proto.disk_check
_active, _context = False, None


def batch_paths(run_id):
    require(isinstance(run_id,str) and re.fullmatch(r"v[1-9][0-9]*",run_id), "fresh preview0.5 version required")
    output=ROOT/("storage/experiments/validation/2026-10-02/temporal_route_joint_preview05_signed45_town07_"+run_id)
    return output,output.with_name(output.name+"_plan.json")


def prior_preview08_context(expected):
    """HH_261002 - Retain all three completed0.8 attempts, not a chosen successful subset or new matched-model claim."""
    require(digest(PREVIEW08_PLAN)==PREVIEW08_PLAN_SHA,"fixed completed0.8 batch declaration changed")
    batch=read_json(PREVIEW08_PLAN)
    require(batch["schema"]=="portable_e2e.actual_route_joint_thread4_preview08_signed45_three_arms.20261002.v2.batch"
        and batch["prospective_trial_count"]==3 and [r["trial_index"] for r in batch["trials"]]==[0,1,2]
        and batch["capture_seconds_per_trial"]==45 and batch["net_progress_minimum_m"]==90.
        and batch["original_model_speed_envelope_mps"]==2.5,"same complete prospective0.8 protocol required")
    pins={str(PREVIEW08_PLAN):PREVIEW08_PLAN_SHA};trials=[]
    for index in range(3):
        output=PREVIEW08_BATCH/("trial_%02d"%index)
        files={name:output/name for name in ("diagnostic_report.json","owner_result.json","declaration.json")}
        for path in files.values():
            require(path.resolve()==path and not path.is_symlink(),"prior0.8 receipt path must remain canonical")
            pins[str(path)]=digest(path)
        declaration=read_json(files["declaration.json"])
        case=batch["trials"][index];declared=Path(case["declaration_path"])
        require(declared.is_absolute() and declared.resolve()==declared and declared.is_relative_to(ROOT)
            and not declared.is_symlink() and digest(declared)==case["declaration_sha256"]==pins[str(files["declaration.json"])],
            "original0.8 case and captured declaration identity differ")
        pins[str(declared)]=case["declaration_sha256"]
        require(declaration["trial_index"]==index and declaration["new_fixed_endpoint_export_sha256"]==dict(
            weights=expected["weights_sha256"],metadata=expected["metadata_sha256"])
            and declaration["source_checkpoint_sha256"]==expected["checkpoint_sha256"]
            and declaration["actual_training_declaration_sha256"]==expected["declaration_sha256"]
            and declaration["optimizer_updates"]==6000 and declaration["controller_target_cap_mps"]==2.4
            and declaration["causal_selected_speed_preview_calibration"]["configured_speed_preview_s"]==.8,
            "original0.8 comparison must use the exact same final6000/controller endpoint")
        report=read_json(files["diagnostic_report.json"]);owner=read_json(files["owner_result.json"])
        require(report["bounded_capture_complete"] is True and report["actual_ACK_receipt_count"]==904
            and report["actual_preview_configuration"]["passed"] is True
            and report["actual_preview_configuration"]["actual_controller"]["speed_preview_s"]==.8
            and owner["declaration_sha256"]==case["declaration_sha256"]
            and owner.get("source_hashes_match") is True and owner.get("external_source_hashes_match") is True
            and not owner.get("cleanup_errors",["missing"]),"all original0.8 attempts must be complete and owned")
        trials.append(dict(trial_index=index,original_status=report["status"],
            original_basic_screen_passed=report["learned_driving_screen_passed"],full45seconds_retained=True))
    return dict(batch_path=str(PREVIEW08_BATCH),batch_plan_sha256=PREVIEW08_PLAN_SHA,
        prospective_trial_count=3,trials=trials,receipt_source_sha256=pins,
        original_thresholds_and_failures_not_rewritten=True,full_raw_inputs_remain_in_original_owned_folders=True,
        independent_archive_audit_of0p8_not_claimed_by_this_context=True)


def calibration_contract():
    result=_contract()
    result.update(schema=live.PREVIEW_SCHEMA,configured_speed_preview_s=.5,
        single_factor_bracket_preview_seconds=[.3,.5,.8],no_prior_outcome_selected_or_relabelled=True)
    return result


def fixed_contract(run_id,longitudinal_pi):
    result=_fixed(run_id,longitudinal_pi)
    result.update(schema=SCHEMA,experiment="Same final6000/CPU4 signed45 with only causal speed preview0.5 calibration",
        causal_selected_speed_preview_calibration=calibration_contract(),
        collector_and_physical_preview_calibration_unchanged=False,
        exact_baseline_reader_compatibility_unchanged=True,prior_complete_preview08_context=_context,
        zero_new_training_and_no_checkpoint_selection=True,
        original0p3_and0p8_source_outputs_unchanged=True,
        prior_preview08_receipt_archive_bytes_per_trial=context_archive_bytes(),
        additional_three_trial_preview08_receipt_archive_reserve_bytes=3*context_archive_bytes())
    return result


def context_archive_bytes():
    require(_context is not None,"prior0.8 archive reserve requires fixed context")
    return sum(Path(name).stat().st_size for name in _context["receipt_source_sha256"])


def disk_check(free=None):
    result=_disk(free);extra=3*context_archive_bytes()
    require(result["free_bytes"]>=result["required_bytes"]+extra,"complete three-arm prior0.8 receipt reserve unavailable")
    return dict(result,prior_preview08_receipt_archive_reserve_bytes=extra,required_bytes=result["required_bytes"]+extra)


def configuration_report(episode):
    """HH_261002 - Change only the exact speed-preview readback predicate; preserve lateral lookahead0.8 and all ACK math."""
    require(digest(Path(proto.__file__))==previous.FROZEN_OWNER_SHA,"frozen preview owner changed")
    tree=ast.parse(Path(proto.__file__).read_bytes())
    original=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="configuration_report")
    changed=copy.deepcopy(original);count=0

    class Narrow(ast.NodeTransformer):
        def visit_Compare(self,node):
            nonlocal count
            node=self.generic_visit(node)
            if ast.dump(node.left)==ast.dump(ast.parse("controller['speed_preview_s']",mode="eval").body):
                require(len(node.ops)==1 and isinstance(node.ops[0],ast.Eq) and len(node.comparators)==1
                    and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value==.8,
                    "unexpected original speed-preview predicate")
                node.comparators[0].value=.5;count+=1
            return node

    changed=Narrow().visit(changed);require(count==1,"exactly one speed-preview predicate must change")
    restored=copy.deepcopy(changed)
    class Restore(ast.NodeTransformer):
        def visit_Compare(self,node):
            node=self.generic_visit(node)
            if ast.dump(node.left)==ast.dump(ast.parse("controller['speed_preview_s']",mode="eval").body):node.comparators[0].value=.8
            return node
    require(ast.dump(Restore().visit(restored))==ast.dump(original),"another preview audit statement changed")
    namespace=dict(vars(proto),live=live)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),
        str(Path(__file__))+":exact0p5_controller_readback","exec"),namespace)
    return namespace["configuration_report"](episode)


def verify_prior_context():
    require(_context is not None,"fixed prior0.8 context missing")
    require(all(digest(Path(path))==sha for path,sha in _context["receipt_source_sha256"].items()),
            "original0.8 completed receipt changed")


def diagnostic_report(output,plan,owner):
    result=_diagnostic(output,plan,owner)
    try:
        verify_prior_context();parent.verify_derived_source_closure(output,plan)
    except Exception as error:
        result["learned_driving_screen_passed"]=False
        result["prior_preview08_context_error"]=type(error).__name__+": "+str(error)
    result.update(schema=SCHEMA+".diagnostic_report",prior_complete_preview08_context=_context)
    result["status"]="PASS_BASIC_PREVIEW05_SIGNED_PRECURSOR" if result["learned_driving_screen_passed"] else "FAIL_BASIC_PREVIEW05_SIGNED_PRECURSOR"
    return result


def batch_summary(run_id,expected):
    verify_prior_context();result=_summary(run_id,expected)
    result.update(schema=SCHEMA+".batch_summary",prior_complete_preview08_context=_context)
    return result


def runtime_sources():
    require(digest(Path(previous.__file__))==FROZEN_OWNER_SHA,"frozen preview compatibility owner changed")
    live.verify_frozen();pins=_sources()
    for path in base.imported_source_closure((Path(__file__).resolve(),)):
        name,sha=str(path.relative_to(ROOT)),digest(path)
        require(name not in pins or pins[name]==sha,"preview0.5 source closure conflicts with frozen dependency")
        pins[name]=sha
    return pins


@contextmanager
def activated(pins):
    global _active,_context
    require(not _active and not previous._active and not proto._active,"preview0.5 owner cannot nest")
    context=prior_preview08_context(pins)
    values=dict(batch_paths=batch_paths,MODULE=MODULE,SCHEMA=SCHEMA,fixed_contract=fixed_contract,runtime_sources=runtime_sources)
    delegated=dict(live=live,calibration_contract=calibration_contract,configuration_report=configuration_report,
        diagnostic_report=diagnostic_report,batch_summary=batch_summary,disk_check=disk_check)
    saved={k:getattr(previous,k) for k in values};saved_proto={k:getattr(proto,k) for k in delegated};assets=base.ASSETS
    _active,_context=True,context
    try:
        for name,value in values.items():setattr(previous,name,value)
        for name,value in delegated.items():setattr(proto,name,value)
        base.ASSETS=dict(base.ASSETS)|{Path(path):sha for path,sha in context["receipt_source_sha256"].items()}
        with previous.activated(pins):yield
    finally:
        for name,value in reversed(tuple(saved_proto.items())):setattr(proto,name,value)
        for name,value in reversed(tuple(saved.items())):setattr(previous,name,value)
        base.ASSETS=assets;_active,_context=False,None


def main(argv=None):
    import argparse
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("weights","metadata","checkpoint","declaration"):parser.add_argument("--expected-"+name+"-sha256",required=True)
    args,remainder=parser.parse_known_args(argv)
    pins={name+"_sha256":getattr(args,"expected_"+name+"_sha256") for name in ("weights","metadata","checkpoint","declaration")}
    with activated(pins):return parent.main(remainder)


if __name__=="__main__":
    raise SystemExit(main())
