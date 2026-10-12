"""HH_261005 - Prepare fresh V5 whole-interval media without reusing V3 policy or reinterpreting its failed prefix.

Frozen V4/V3/V2 policy guards and exact V1 rendering remain private delegates.
Only fresh identities/provenance change; no capture, model, training or simulator
is accessed during source preparation or prospective policy sealing.
"""

from __future__ import annotations

import ast
import copy
import inspect
from pathlib import Path
import textwrap
from types import FunctionType

from . import publish_temporal_route_joint_town03_straight_full209_v4_20261004 as previous


CODE_ROOT=Path(__file__).resolve().parents[2]
ROOT=CODE_ROOT
RUN=ROOT/"storage/experiments/validation/2026-10-05/temporal_route_joint_town03_known_dev_straight_full209_v5"
OUTPUT=ROOT/"docs/assets/validation/2026-10-05/portable_e2e_learning_cycle_v1/01_town03_full_route209_v5"
PLAN_PATH=RUN.with_name(RUN.name+"_plan.json")
POLICY_PATH=ROOT/"storage/experiments/plans/2026-10-05/town03_known_dev_straight_full209_v5/ROOT_prospective_publication_policy.json"
SCHEMA="portable_e2e.actual_town03_known_dev_straight_full209_publication.20261005.v5"
CALLER_SCHEMA=SCHEMA+".caller_pins"
OWNER_SCHEMA="portable_e2e.owned_route_joint_town03_known_dev_straight_full209.20261005.v5"
CAPTURE_SCHEMA=previous.CAPTURE_SCHEMA
POLICY_SCHEMA="portable_e2e.town03_known_dev_straight_full209_prospective_publication_policy.20261005.v5"
PREVIOUS_SHA="195f5a62de3346c22b39f41392e3566313714bcdbe2be293580e02232057ba54"
PREVIOUS_TEST_SHA="cfdcbb601ec25cdec6d9517b13030f158466f3ca2b5765086a9845038a1fa7e4"
ROUTE_SHA,PNG_NUMERATORS,MODEL_CAMERA_ORDER=previous.ROUTE_SHA,previous.PNG_NUMERATORS,previous.MODEL_CAMERA_ORDER
require,prior,visual=previous.require,previous.prior,previous.visual
camera_indices,png_indices,playback=previous.camera_indices,previous.png_indices,previous.playback
_canonical=previous._canonical


def verify_frozen():
    previous.verify_frozen()
    for path,pin in ((Path(previous.__file__),PREVIOUS_SHA),
            (CODE_ROOT/"learning/tests/test_publish_town03_straight_full209_v4_20261004.py",PREVIOUS_TEST_SHA)):
        _canonical(path,regular=True)
        require(prior.digest(path)==pin,"frozen V4 publication source/test changed")


def publication_sources():
    verify_frozen()
    return previous.previous.previous.previous.previous.core.audit.source_closure(
        Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py")


def _namespace(function):
    # HH_261005 - Preserve each function's real original previous-layer while replacing only its fresh private identities/providers.
    return {**function.__globals__,"__file__":__file__,"CODE_ROOT":CODE_ROOT,"ROOT":ROOT,"RUN":RUN,
        "OUTPUT":OUTPUT,"PLAN_PATH":PLAN_PATH,"POLICY_PATH":POLICY_PATH,"SCHEMA":SCHEMA,
        "CALLER_SCHEMA":CALLER_SCHEMA,"OWNER_SCHEMA":OWNER_SCHEMA,"CAPTURE_SCHEMA":CAPTURE_SCHEMA,
        "POLICY_SCHEMA":POLICY_SCHEMA,"verify_frozen":verify_frozen,"publication_sources":publication_sources,
        "_private":_private,"_delegate":_delegate,"_run_id_seam":_run_id_seam,"verify_policy":verify_policy,
        "load_archive":load_archive,"render":render,"markdown":markdown,"publish":publish,
        "policy_document":policy_document,"_verify_declared_sources":_verify_declared_sources,
        "seal_policy":seal_policy,"_canonical":_canonical}


def _private(function,*,transform=None):
    verify_frozen()
    namespace=_namespace(function)
    if transform is None:
        return FunctionType(function.__code__,namespace,function.__name__,function.__defaults__,function.__closure__)
    original=ast.parse(textwrap.dedent(inspect.getsource(function))).body[0]
    changed=copy.deepcopy(original);undo=transform(changed)
    restored=copy.deepcopy(changed);undo(restored)
    require(ast.dump(restored)==ast.dump(original),"V5 publication changed beyond its exact identity seam")
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),
        str(Path(__file__))+":inverse_proven_private_V5_identity","exec"),namespace)
    return namespace[function.__name__]


def _delegate(function,*,transform=None):
    return _private(previous.previous.previous._delegate)(function,transform=transform)


def _run_id_seam(tree):
    count=0
    for node in ast.walk(tree):
        if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v1":
            node.comparators[0].value="v5";count+=1
    require(count==1,"one original archive run-id V5 seam required")
    def undo(restored):
        for node in ast.walk(restored):
            if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                    and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v5":
                node.comparators[0].value="v1"
    return undo


def _policy_identity_seams(tree):
    old="HH_261004 - Fix whole actual V2 interval and all failure-tail sampling before one drive; unchanged V1 raw ABI and rendering math are independently pinned, without approval or future-record claims."
    new=old.replace("HH_261004","HH_261005",1).replace("actual V2 interval","actual V5 interval",1)
    count=provenance=0
    for node in ast.walk(tree):
        if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v2":
            node.comparators[0].value="v5";count+=1
        elif isinstance(node,ast.Constant) and node.value==old: node.value=new;provenance+=1
    require(count==provenance==1,"one original V5 policy run identity and disclosure required")
    def undo(restored):
        for node in ast.walk(restored):
            if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                    and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v5":
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
    # HH_261005 - Publish the actual verdict and all original intervals; runtime interventions are not new learned driving capability.
    text=_delegate(previous.previous.previous.previous.markdown)(bundle,pngs)
    return text.replace("# 실제 Town03 209m 전체 경로 시험\n","# 실제 Town03 209m 전체 경로 V5 처리시간 재시험\n",1)+(
        "\n<!-- HH_261005 - Disclose CPU6, exact all-valid masked-copy and every-entry native quota separately from unchanged weights. -->\n\n"
        "V5는 같은 GOOD6000 가중치를 사용하고 CPU6, 모든 입력이 유효할 때만 동등한 masked-copy 경로, "
        "매 카메라마다 전체 파일을 다시 세는 native 용량 계산을 명시적으로 사용한 별도 연구입니다. "
        "기존 카메라 원본 버퍼 복사·3작업자 JPEG·신호등2단계 복원과 원래 물리·제어·100ms 기준은 유지합니다. "
        "원본 판정과 전체 카메라·상태·실패 구간을 보존하며 이전 V1–V4 실패를 수정하지 않습니다. "
        "오프라인 입력 동등성은 실제 BGRA 센서 지연 개선이나 폐루프 성공을 증명하지 않으며, "
        "독립 기능 승인·학습한 목적지 정차·실차 안전을 주장하지 않습니다.\n")


def publish(args):
    return _private(previous.publish)(args)


def policy_document(plan,declaration_sha256):
    return _private(previous.previous.previous.policy_document,transform=_policy_identity_seams)(plan,declaration_sha256)


def _verify_declared_sources(plan):
    return _private(previous._verify_declared_sources)(plan)


def seal_policy(expected_declaration_sha256):
    return _private(previous.seal_policy)(expected_declaration_sha256)


def main(argv=None):
    return _private(previous.main)(argv)


if __name__=="__main__":raise SystemExit(main())
