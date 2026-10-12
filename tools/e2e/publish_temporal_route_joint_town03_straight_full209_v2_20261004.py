"""HH_261004 - Prospectively seal V2 whole-interval media rules and reuse the exact frozen authentic V1 renderer.

V2 has a new owner identity, but explicitly preserves the V1 raw capture ABI.
No future episode is read while sealing the policy.  Publishing later requires
caller-pinned complete real archives, retains failures and unknown actual
denominators, and performs no model inference, simulator calls or promotion.
"""

from __future__ import annotations

import argparse
import ast
import copy
from datetime import datetime, timezone
import hashlib
import inspect
import json
from pathlib import Path
import stat
import textwrap
from types import FunctionType

from . import publish_temporal_route_joint_town03_straight_full209_v1_20261004 as previous


CODE_ROOT=Path(__file__).resolve().parents[2]
ROOT=CODE_ROOT
RUN=ROOT/"storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v2"
OUTPUT=ROOT/"docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/12_town03_full_route209_v2"
PLAN_PATH=RUN.with_name(RUN.name+"_plan.json")
POLICY_PATH=ROOT/"storage/experiments/plans/2026-10-04/town03_known_dev_straight_full209_v2/ROOT_prospective_publication_policy.json"
SCHEMA="portable_e2e.actual_town03_known_dev_straight_full209_publication.20261004.v2"
CALLER_SCHEMA=SCHEMA+".caller_pins"
OWNER_SCHEMA="portable_e2e.owned_route_joint_town03_known_dev_straight_full209.20261004.v2"
CAPTURE_SCHEMA="portable_e2e.actual_route_joint_preview05_town03_known_dev_straight_full209.20261004.v1"
POLICY_SCHEMA="portable_e2e.town03_known_dev_straight_full209_prospective_publication_policy.20261004.v2"
PREVIOUS_SHA="306c7a498def2972628a7aac89084162f53e3e74db51d06e2418f2e8a19920ca"
PREVIOUS_TEST_SHA="9a4fe0a349ed51a87fa00530fe5112b51a95e7d02400e7c7ac0383e3f926ddf4"
ROUTE_SHA=previous.ROUTE_SHA
PNG_NUMERATORS=previous.PNG_NUMERATORS
MODEL_CAMERA_ORDER=previous.MODEL_CAMERA_ORDER
require,prior,visual=previous.require,previous.prior,previous.visual
camera_indices,png_indices,playback=previous.camera_indices,previous.png_indices,previous.playback


def _canonical(path,*,regular=False):
    # HH_261004 - Fail closed on a symlink request or parent; a prospective policy cannot silently resolve a different target.
    path=Path(path)
    require(path.is_absolute() and path.resolve()==path,"canonical no-symlink publication path required")
    if regular:
        require(stat.S_ISREG(path.lstat().st_mode),"regular prospective declaration required")
    return path


def verify_frozen():
    # HH_261004 - Frozen code identity is independent of the new sealer's synthetic scoped-path fixtures.
    for path,pin in ((Path(previous.__file__),PREVIOUS_SHA),
            (CODE_ROOT/"learning/tests/test_publish_town03_straight_full209_v1_20261004.py",PREVIOUS_TEST_SHA),
            (Path(previous.previous.__file__),previous.PREVIOUS_SHA)):
        _canonical(path,regular=True)
        require(prior.digest(path)==pin,"frozen V1 publication source/test changed")


def publication_sources():
    verify_frozen()
    return previous.previous.core.audit.source_closure(Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py")


def _delegate(function,*,transform=None):
    """HH_261004 - Use exact original function code with private V2 identities, never mutate the frozen publisher's module globals."""
    verify_frozen()
    namespace={**function.__globals__,"ROOT":ROOT,"RUN":RUN,"OUTPUT":OUTPUT,"POLICY_PATH":POLICY_PATH,
        "SCHEMA":SCHEMA,"CALLER_SCHEMA":CALLER_SCHEMA,"OWNER_SCHEMA":OWNER_SCHEMA,"POLICY_SCHEMA":POLICY_SCHEMA,
        "publication_sources":publication_sources,"verify_policy":verify_policy,"load_archive":load_archive,
        "render":render,"markdown":markdown}
    if transform is None:
        return FunctionType(function.__code__,namespace,function.__name__,function.__defaults__,function.__closure__)
    original=ast.parse(textwrap.dedent(inspect.getsource(function))).body[0]
    changed=copy.deepcopy(original)
    undo=transform(changed)
    restored=copy.deepcopy(changed);undo(restored)
    require(ast.dump(restored)==ast.dump(original),"publisher changed beyond its exact V2 identity seam")
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),
        str(Path(__file__))+":inverse_proven_private_V2_identity","exec"),namespace)
    return namespace[function.__name__]


def _run_id_seam(tree):
    count=0
    for node in ast.walk(tree):
        if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v1":
            node.comparators[0].value="v2";count+=1
    require(count==1,"one frozen archive run-id V2 seam required")
    def undo(restored):
        for node in ast.walk(restored):
            if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                    and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v2":
                node.comparators[0].value="v1"
    return undo


def verify_policy(caller,pins,plan):
    policy=_delegate(previous.verify_policy)(caller,pins,plan)
    require(policy.get("owner_schema")==OWNER_SCHEMA and policy.get("capture_raw_ABI_schema")==CAPTURE_SCHEMA
        and plan.get("capture_raw_ABI_schema")==CAPTURE_SCHEMA
        and policy.get("actual_counts_unknown_before_capture") is True
        and policy.get("native_camera_counts_selected_from_actual_archive_later") is True,
        "fresh V2 owner, unchanged V1 capture ABI and prospectively unknown actual denominator required")
    return policy


def load_archive(caller_path,caller_sha,*,decode=False):
    return _delegate(previous.load_archive,transform=_run_id_seam)(caller_path,caller_sha,decode=decode)


def render(bundle,index):
    # HH_261004 - Keep exact original real six-camera composition, centered vehicle, full route/travel and prior-NN overlay.
    return _delegate(previous.render)(bundle,index)


def markdown(bundle,pngs):
    text=_delegate(previous.markdown)(bundle,pngs)
    return text.replace("# 실제 Town03 209m 전체 경로 시험\n","# 실제 Town03 209m 전체 경로 V2 계측 재시험\n",1)+(
        "\n<!-- HH_261004 - Disclose fresh owner identity separately from the unchanged original capture ABI and rendering math. -->\n\n"
        "새 V2 소유권 선언과 종점 계측을 사용하며 원본 캡처 형식은 V1입니다. 기존 V1 실패는 변경하지 않았습니다. "
        "실제 결과와 전체 기록으로 판정하며, 계측 추가나 검사기 수정만으로 모델 성능 향상·정식 기능 완료를 주장하지 않습니다.\n")


def publish(args):
    return _delegate(previous.publish)(args)


def policy_document(plan,declaration_sha256):
    """HH_261004 - Construct outcome-independent selection rules from the already sealed V2 owner declaration, not future records."""
    require(type(declaration_sha256) is str and len(declaration_sha256)==64
        and all(value in "0123456789abcdef" for value in declaration_sha256),"exact declaration SHA256 required")
    require(plan.get("schema")==OWNER_SCHEMA and plan.get("run_id")=="v2" and plan.get("output")==str(RUN)
        and plan.get("capture_raw_ABI_schema")==CAPTURE_SCHEMA and plan.get("town")=="Town03"
        and plan.get("scenario")=="straight" and plan.get("dataset_split")=="val"
        and plan.get("attempts")==1 and plan.get("automatic_retry") is False
        and plan.get("total_capture_seconds")==110 and plan.get("maximum_native_state_count")==2200
        and plan.get("maximum_camera_bundle_count")==1100 and plan.get("original_navigation_sha256")==ROUTE_SHA
        and plan.get("original_catalog_goal_tolerance_m")==2.5 and plan.get("known_DEV_site_reused") is True
        and plan.get("independent_validation") is False,"exact fresh V2 known-DEV bounded owner declaration required")
    for key in ("full_RTE01_approved","full_MAIN9_completion_claimed","training_data_approved",
            "vehicle_control_approved","model_promoted","learned_STOP_claim"):
        require(plan.get(key) is False,"prospective owner cannot imply approval or learned STOP: "+key)
    media=copy.deepcopy(plan.get("prospective_visual_publication",{}))
    require(media.get("gallery")==str(OUTPUT.relative_to(ROOT))
        and media.get("GIF_camera_indices")=="every5th actual camera bundle starting0 plus exact last; unique sorted"
        and media.get("GIF_playback")=="all selected timestamp intervals plus original final0.1s camera cadence"
        and media.get("PNG_camera_indices")=="floor((actual_camera_count-1)*num/4), num=0,1,2,3,4; unique sorted"
        and media.get("complete_actual_interval_including_failures_required") is True
        and media.get("outcome_or_failure_dependent_frame_selection") is False
        and media.get("real_six_cameras_required") is True and media.get("vehicle_centered") is True
        and media.get("original_full_route_and_actual_travel_and_prior_learned_ACK_trajectory_required") is True
        and media.get("final_native_state_and_final_camera_timestamp_distinction_retained") is True
        and media.get("actual_Autoware_or_RViz_screenshot_claim") is False
        and media.get("illustrative_or_generated_image_allowed") is False,"original prospective all-interval media rules required")
    return dict(schema=POLICY_SCHEMA,
        provenance="HH_261004 - Fix whole actual V2 interval and all failure-tail sampling before one drive; unchanged V1 raw ABI and rendering math are independently pinned, without approval or future-record claims.",
        created_UTC=datetime.now(timezone.utc).isoformat(),created_before_actual_episode=True,actual_episode_observed=False,
        run_root=str(RUN),output=str(OUTPUT),attempts=1,automatic_retry=False,media_selection=media,
        run_declaration=dict(path=str(PLAN_PATH),sha256=declaration_sha256),owner_schema=OWNER_SCHEMA,
        capture_raw_ABI_schema=CAPTURE_SCHEMA,actual_counts_unknown_before_capture=True,
        owner_source_sha256=copy.deepcopy(plan.get("source_sha256",{})),
        root_sealer_sha256=prior.digest(_canonical(Path(__file__),regular=True)),
        final_publisher_source_frozen_before_capture_claimed=False,
        final_publisher_and_fonts_must_be_pinned_before_rendering=True,
        native_camera_counts_selected_from_actual_archive_later=True,known_DEV_site_reused=True,independent_validation=False,
        full_RTE01_approved=False,full_MAIN9_completion_claimed=False,training_data_approved=False,
        vehicle_control_approved=False,model_promoted=False)


def _verify_declared_sources(plan):
    # HH_261004 - Authenticate every declared owner byte outside any actual observation interval, never follow an alias.
    sources=plan.get("source_sha256")
    require(type(sources) is dict and bool(sources),"nonempty declared owner source closure required")
    for name,pin in sources.items():
        require(type(name) is str,"owner source name must be text")
        relative=Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts and relative.as_posix()==name,
            "safe exact owner source member required")
        require(type(pin) is str and len(pin)==64 and all(value in "0123456789abcdef" for value in pin),
            "exact declared owner source SHA256 required")
        path=_canonical(ROOT/relative,regular=True)
        require(prior.digest(path)==pin,"owner source changed before visual declaration: "+name)


def seal_policy(expected_declaration_sha256):
    """HH_261004 - Create one fixed policy before the run root exists; never inspect episode payloads or overwrite a declaration."""
    verify_frozen()
    for path in (RUN,OUTPUT,PLAN_PATH,POLICY_PATH):
        _canonical(path,regular=path==PLAN_PATH)
    require(not RUN.exists(),"prospective policy must precede the creation of the actual run root")
    require(not OUTPUT.exists() and not POLICY_PATH.exists(),"new V2 gallery and publication policy are create-only")
    payload=PLAN_PATH.read_bytes()
    require(hashlib.sha256(payload).hexdigest()==expected_declaration_sha256,"prospective executed declaration SHA differs")
    plan=json.loads(payload)
    policy=policy_document(plan,expected_declaration_sha256)
    _verify_declared_sources(plan)
    POLICY_PATH.parent.mkdir(parents=True,exist_ok=True)
    _canonical(POLICY_PATH.parent)
    require(not RUN.exists() and not OUTPUT.exists() and not POLICY_PATH.exists(),"actual run or output appeared before policy sealing")
    require(PLAN_PATH.read_bytes()==payload,"prospective declaration changed before policy sealing")
    _verify_declared_sources(plan)
    with POLICY_PATH.open("x",encoding="utf-8") as stream:
        json.dump(policy,stream,sort_keys=True,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write("\n")
    require(PLAN_PATH.read_bytes()==payload,"prospective declaration changed while sealing")
    _verify_declared_sources(plan)
    return dict(path=str(POLICY_PATH),sha256=prior.digest(POLICY_PATH),created=True,
        actual_episode_observed=False,actual_counts_unknown_before_capture=True)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--seal-policy",action="store_true")
    parser.add_argument("--expected-declaration-sha256")
    parser.add_argument("--caller-pins");parser.add_argument("--caller-pins-sha256")
    parser.add_argument("--validate-only",action="store_true")
    args=parser.parse_args(argv)
    if args.seal_policy:
        require(args.expected_declaration_sha256 is not None and args.caller_pins is None
            and args.caller_pins_sha256 is None and not args.validate_only,"policy mode cannot publish or inspect future capture")
        result=seal_policy(args.expected_declaration_sha256)
    else:
        require(args.expected_declaration_sha256 is None and args.caller_pins is not None
            and args.caller_pins_sha256 is not None,"external caller pins required for actual archive publication")
        result=publish(args)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))
    return 0


if __name__=="__main__":
    raise SystemExit(main())
