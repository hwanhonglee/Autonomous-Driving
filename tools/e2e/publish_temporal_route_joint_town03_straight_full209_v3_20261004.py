"""HH_261004 - Prepare separate V3 whole-interval media identities while preserving frozen V2 policy guards and V1 rendering.

Only identities and readable revision disclosures change.  Selection is sealed
before capture; all actual raw/failure intervals are independently pinned before
rendering.  This module never imports a model, simulator or training runtime.
"""

from __future__ import annotations

import ast
import copy
import inspect
from pathlib import Path
import textwrap
from types import FunctionType

from . import publish_temporal_route_joint_town03_straight_full209_v2_20261004 as previous


CODE_ROOT=Path(__file__).resolve().parents[2]
ROOT=CODE_ROOT
RUN=ROOT/"storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v3"
OUTPUT=ROOT/"docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/13_town03_full_route209_v3"
PLAN_PATH=RUN.with_name(RUN.name+"_plan.json")
POLICY_PATH=ROOT/"storage/experiments/plans/2026-10-04/town03_known_dev_straight_full209_v3/ROOT_prospective_publication_policy.json"
SCHEMA="portable_e2e.actual_town03_known_dev_straight_full209_publication.20261004.v3"
CALLER_SCHEMA=SCHEMA+".caller_pins"
OWNER_SCHEMA="portable_e2e.owned_route_joint_town03_known_dev_straight_full209.20261004.v3"
CAPTURE_SCHEMA=previous.CAPTURE_SCHEMA
POLICY_SCHEMA="portable_e2e.town03_known_dev_straight_full209_prospective_publication_policy.20261004.v3"
PREVIOUS_SHA="354abb6ed79074af3de835ea4d0893d17755cf0a8010fd60917ef397de3e6669"
PREVIOUS_TEST_SHA="4556f07113a5254b851cd74fb61ca61138786f12ed5fcac4fc12b101decfa49c"
ROUTE_SHA,PNG_NUMERATORS,MODEL_CAMERA_ORDER=previous.ROUTE_SHA,previous.PNG_NUMERATORS,previous.MODEL_CAMERA_ORDER
require,prior,visual=previous.require,previous.prior,previous.visual
camera_indices,png_indices,playback=previous.camera_indices,previous.png_indices,previous.playback
_canonical=previous._canonical


def verify_frozen():
    # HH_261004 - Authenticate immutable V2 source/test and its full V1 chain before every private delegation.
    previous.verify_frozen()
    for path,pin in ((Path(previous.__file__),PREVIOUS_SHA),
            (CODE_ROOT/"learning/tests/test_publish_town03_straight_full209_v2_20261004.py",PREVIOUS_TEST_SHA)):
        _canonical(path,regular=True)
        require(prior.digest(path)==pin,"frozen V2 publication source/test changed")


def publication_sources():
    verify_frozen()
    return previous.previous.previous.core.audit.source_closure(
        Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py")


def _namespace(function):
    # HH_261004 - Retain each authentic function's own previous-module layer; replace no frozen module globals.
    return {**function.__globals__,"__file__":__file__,"CODE_ROOT":CODE_ROOT,"ROOT":ROOT,"RUN":RUN,
        "OUTPUT":OUTPUT,"PLAN_PATH":PLAN_PATH,"POLICY_PATH":POLICY_PATH,"SCHEMA":SCHEMA,
        "CALLER_SCHEMA":CALLER_SCHEMA,"OWNER_SCHEMA":OWNER_SCHEMA,"CAPTURE_SCHEMA":CAPTURE_SCHEMA,
        "POLICY_SCHEMA":POLICY_SCHEMA,"verify_frozen":verify_frozen,"publication_sources":publication_sources,
        "_delegate":_delegate,"_run_id_seam":_run_id_seam,"verify_policy":verify_policy,
        "load_archive":load_archive,"render":render,"markdown":markdown,"publish":publish,
        "policy_document":policy_document,"_verify_declared_sources":_verify_declared_sources,
        "seal_policy":seal_policy,"_canonical":_canonical}


def _private(function,*,transform=None):
    verify_frozen()
    namespace=_namespace(function)
    if transform is None:
        return FunctionType(function.__code__,namespace,function.__name__,function.__defaults__,function.__closure__)
    original=ast.parse(textwrap.dedent(inspect.getsource(function))).body[0]
    changed=copy.deepcopy(original)
    undo=transform(changed)
    restored=copy.deepcopy(changed);undo(restored)
    require(ast.dump(restored)==ast.dump(original),"V3 publication changed beyond its exact identity seam")
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),
        str(Path(__file__))+":inverse_proven_private_V3_identity","exec"),namespace)
    return namespace[function.__name__]


def _delegate(function,*,transform=None):
    # HH_261004 - Reuse the exact V2 delegation guard and exact V1 renderer/archive code under separate V3 identities.
    return _private(previous._delegate)(function,transform=transform)


def _run_id_seam(tree):
    count=0
    for node in ast.walk(tree):
        if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v1":
            node.comparators[0].value="v3";count+=1
    require(count==1,"one original archive run-id V3 seam required")
    def undo(restored):
        for node in ast.walk(restored):
            if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                    and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v3":
                node.comparators[0].value="v1"
    return undo


def _policy_identity_seams(tree):
    # HH_261004 - Change exactly the V2 run-id check and its revision disclosure, not selection/source/create-only guards.
    old="HH_261004 - Fix whole actual V2 interval and all failure-tail sampling before one drive; unchanged V1 raw ABI and rendering math are independently pinned, without approval or future-record claims."
    new=old.replace("actual V2 interval","actual V3 interval",1)
    count=provenance=0
    for node in ast.walk(tree):
        if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v2":
            node.comparators[0].value="v3";count+=1
        elif isinstance(node,ast.Constant) and node.value==old:
            node.value=new;provenance+=1
    require(count==provenance==1,"one exact V3 policy run identity and disclosure required")
    def undo(restored):
        for node in ast.walk(restored):
            if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                    and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v3":
                node.comparators[0].value="v2"
            elif isinstance(node,ast.Constant) and node.value==new: node.value=old
    return undo


def verify_policy(caller,pins,plan):
    return _private(previous.verify_policy)(caller,pins,plan)


def load_archive(caller_path,caller_sha,*,decode=False):
    return _private(previous.load_archive)(caller_path,caller_sha,decode=decode)


def render(bundle,index):
    return _private(previous.render)(bundle,index)


def markdown(bundle,pngs):
    # HH_261004 - Add readable V3 provenance without changing a single selected camera or render pixel.
    text=_delegate(previous.previous.markdown)(bundle,pngs)
    return text.replace("# 실제 Town03 209m 전체 경로 시험\n","# 실제 Town03 209m 전체 경로 V3 기록·정리 재시험\n",1)+(
        "\n<!-- HH_261004 - Disclose synchronous JPEG and two-phase signal cleanup independently of model behavior. -->\n\n"
        "V3는 같은 주행 모델·제어 설정에서 ACK 뒤 3개 작업자로 6개 카메라 JPEG를 인코딩하고, "
        "모든 작업 종료 뒤 원래 순서로 기록합니다. 신호등 복원은 캡처 밖 2개 실제 틱으로 원래 상태와 동결 설정을 각각 검사합니다. "
        "원본 캡처 형식은 V1이며 V1·V2 실패 기록은 그대로 보존합니다. "
        "이 변경이나 오프라인 JPEG 비교만으로 실제 주행 지연 개선·학습 성능 향상·정식 기능 완료를 주장하지 않습니다.\n")


def publish(args):
    return _private(previous.publish)(args)


def policy_document(plan,declaration_sha256):
    return _private(previous.policy_document,transform=_policy_identity_seams)(plan,declaration_sha256)


def _verify_declared_sources(plan):
    return _private(previous._verify_declared_sources)(plan)


def seal_policy(expected_declaration_sha256):
    return _private(previous.seal_policy)(expected_declaration_sha256)


def main(argv=None):
    return _private(previous.main)(argv)


if __name__=="__main__":raise SystemExit(main())
