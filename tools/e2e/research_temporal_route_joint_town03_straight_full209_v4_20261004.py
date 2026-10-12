"""HH_261004 - Prepare a new actual-buffer/count38 full209 revision while preserving every frozen V1/V2/V3 failure."""

import argparse
from contextlib import contextmanager
from pathlib import Path
from types import FunctionType

from . import research_temporal_route_joint_town03_straight_full209_v3_20261004 as previous
from . import collect_temporal_route_joint_town03_straight_full209_v4_20261004 as live


base,ROOT=previous.base,previous.ROOT
require,digest=previous.require,previous.digest
MODULE="scripts.e2e.research_temporal_route_joint_town03_straight_full209_v4_20261004"
SCHEMA=live.OWNER_SCHEMA
PARENT_SHA256="6243c4ecf0d2259c0c252c0d74d62a9fa0d7192483cd84954e2a3954df737520"
_fixed,_sources=previous.fixed_contract,previous.runtime_sources
# HH_261004 - Preserve the complete genuine V3 diagnostic bytecode, changing only its private fresh live/schema identities.
_original_diagnostic=previous.diagnostic_report
_diagnostic=FunctionType(_original_diagnostic.__code__,{**_original_diagnostic.__globals__,"live":live,"SCHEMA":SCHEMA},
    _original_diagnostic.__name__,_original_diagnostic.__defaults__,_original_diagnostic.__closure__)
_diagnostic.__kwdefaults__=dict(_original_diagnostic.__kwdefaults__ or {})
_active=False


def paths(run_id):
    require(run_id=="v4","fresh full209 V4 requires exact v4 identity")
    output=ROOT/"storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v4"
    return output,output.with_name(output.name+"_plan.json")


def capture_argv(run_id,longitudinal_pi,declaration_sha=base.PLAN_TOKEN):
    # HH_261004 - Retain every original option; reuse the same original45-to-declared110s duration seam.
    require(_active and run_id=="v4","fresh capture requires its exclusive V4 owner scope")
    result=previous.previous.previous._original_capture(run_id,longitudinal_pi,declaration_sha)
    index=result.index("--max-duration-sec")+1
    require(result[index]=="45","original fullroute duration seam changed");result[index]="110"
    return result


def fixed_contract(run_id,longitudinal_pi):
    """HH_261004 - Declare genuine buffer snapshots and corrected complete Town03 inventory without a capability/gain claim."""
    result=_fixed(run_id,longitudinal_pi)
    result.update(schema=SCHEMA,experiment="Fresh known-DEV full209 V4 original-buffer snapshots and complete38-actor cleanup research",
        frozen_V3_preserved=True,capture_raw_ABI_schema=live.SCHEMA,
        actual_V3_raw_buffer_type_claimed=False,prior_V3_zero_JPEG_and_zero_NN_failure_preserved=True)
    jpeg=result["synchronous_parallel_JPEG_after_ACK"]
    jpeg.update(schema=live.jpeg.SCHEMA,source="tools/e2e/parallel_exact_jpeg_after_ack_v2_20261004.py",
        original_image_object_identity_and_frame_stamp_geometry_preserved=True,
        actual_source_ABI_logged_before_rejection=True,
        private_immutable_snapshot_used_only_for_original_encoder=True,
        full_original_pre_snapshot_post_BGRA_SHA_equals_selected_callback_required=True)
    signals=result["two_phase_signal_restoration"]
    signals.pop("all_original_43_actor_state_and_duration_readbacks_required")
    signals.update(schema=live.signals.SCHEMA,source="tools/e2e/two_phase_owned_signal_restoration_v2_20261004.py",
        expected_original_signal_count=38,all_original_38_actor_state_and_duration_readbacks_required=True,
        complete_original_live_saved_recorded_ID_membership_required=True,
        exact_pinned_wrapper_lexical_super_constructor_owner_guard_required=True,
        unused_helper_OWNER_SCHEMA_alias_forbidden=True,
        count_declaration_evidence_sha256=dict(live.signals.INVENTORY_EVIDENCE_PINS),
        prior_mistaken_actor_ID43_as_count_failure_preserved=True)
    result["prospective_visual_publication"]["gallery"]="docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/14_town03_full_route209_v4"
    require(result["worker_command"][result["worker_command"].index("-m")+1]==MODULE
        and result["policy_deadline_seconds"]==.1 and result["model_speed_cap_mps"]==2.5
        and result["native_actor_planar_speed_maximum_mps"]==2.7 and result["controller_target_cap_mps"]==2.4
        and result["speed_preview_s"]==.5 and result["body_clearance_screen"]["lane_inset_m"]==.1
        and result["total_capture_seconds"]==110 and result["original_catalog_goal_tolerance_m"]==2.5,
        "fresh V4 altered original controls/thresholds/goal or actual worker identity")
    return result


def runtime_sources():
    """HH_261004 - Archive the complete prior closure plus new code/tests and both genuine frozen inventory JSON pins."""
    require(digest(Path(previous.__file__))==PARENT_SHA256,"frozen full209 V3 owner changed")
    result=_sources();live.verify_frozen()
    seeds=[Path(__file__).resolve(),Path(live.__file__).resolve(),Path(live.jpeg.__file__).resolve(),
        Path(live.signals.__file__).resolve(),
        ROOT/"learning/tests/test_parallel_exact_jpeg_after_ack_v2_20261004.py",
        ROOT/"learning/tests/test_two_phase_owned_signal_restoration_v2_20261004.py",
        ROOT/"learning/tests/test_town03_full209_collector_v4_20261004.py",
        ROOT/"learning/tests/test_town03_full209_owner_v4_20261004.py"]
    require(all(path.resolve()==path and path.is_file() and not path.is_symlink() for path in seeds),"fresh V4 source/test closure is not prepared")
    for path in base.imported_source_closure(seeds):
        name,sha=str(path.relative_to(ROOT)),digest(path)
        require(name not in result or result[name]==sha,"fresh V4 source closure conflict");result[name]=sha
    for name,sha in live.signals.INVENTORY_EVIDENCE_PINS.items():
        path=ROOT/name
        require(path.resolve()==path and path.is_file() and not path.is_symlink() and digest(path)==sha
            and (name not in result or result[name]==sha),"authentic original inventory JSON archive pin differs")
        result[name]=sha
    return result


def diagnostic_report(output,plan,owner):
    # HH_261004 - Retain every genuine original/V3 gate and failure channel, with fresh strict helpers rather than legacy flags.
    result=_diagnostic(output,plan,owner)
    result.update(schema=SCHEMA+".diagnostic_report",fresh_owner_revision="v4",original_V3_preserved=True,
        capture_raw_ABI_schema=live.SCHEMA,actual_V3_raw_buffer_type_claimed=False,
        original_V3_zero_JPEG_failure_preserved=True,prior_mistaken_actor_ID43_as_count_failure_preserved=True)
    return result


@contextmanager
def activated(pins):
    """HH_261004 - Route only fresh delegates through the entire unchanged V3/V2/V1 ownership lifecycle, restoring all aliases."""
    global _active
    require(not _active and digest(Path(previous.__file__))==PARENT_SHA256,"fresh V4 owner cannot nest or use changed V3")
    changes=dict(paths=paths,capture_argv=capture_argv,fixed_contract=fixed_contract,runtime_sources=runtime_sources,
        diagnostic_report=diagnostic_report,live=live,MODULE=MODULE,SCHEMA=SCHEMA)
    saved={name:getattr(previous,name) for name in changes};_active=True
    try:
        for name,value in changes.items():setattr(previous,name,value)
        with previous.activated(pins):yield
    finally:
        for name,value in reversed(tuple(saved.items())):setattr(previous,name,value)
        _active=False


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("weights","metadata","checkpoint","declaration"):parser.add_argument("--expected-"+name+"-sha256",required=True)
    args,remaining=parser.parse_known_args(argv)
    pins={name+"_sha256":getattr(args,"expected_"+name+"_sha256") for name in ("weights","metadata","checkpoint","declaration")}
    with activated(pins):return base.main(remaining)


if __name__=="__main__":raise SystemExit(main())
