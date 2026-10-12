"""HH_261004 - Prepare fresh V4 whole-interval media without reusing V3 policy or reinterpreting its failed prefix.

Frozen V3/V2 policy guards and exact V1 rendering remain private delegates.
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

from . import publish_temporal_route_joint_town03_straight_full209_v3_20261004 as previous


CODE_ROOT=Path(__file__).resolve().parents[2]
ROOT=CODE_ROOT
RUN=ROOT/"storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v4"
OUTPUT=ROOT/"docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/14_town03_full_route209_v4"
PLAN_PATH=RUN.with_name(RUN.name+"_plan.json")
POLICY_PATH=ROOT/"storage/experiments/plans/2026-10-04/town03_known_dev_straight_full209_v4/ROOT_prospective_publication_policy.json"
SCHEMA="portable_e2e.actual_town03_known_dev_straight_full209_publication.20261004.v4"
CALLER_SCHEMA=SCHEMA+".caller_pins"
OWNER_SCHEMA="portable_e2e.owned_route_joint_town03_known_dev_straight_full209.20261004.v4"
CAPTURE_SCHEMA=previous.CAPTURE_SCHEMA
POLICY_SCHEMA="portable_e2e.town03_known_dev_straight_full209_prospective_publication_policy.20261004.v4"
PREVIOUS_SHA="63b1b569eab1bf7db687065e21f7ec02420ffec6c410481509c62c2b14142d5d"
PREVIOUS_TEST_SHA="07cee3d9b4e63a825c719f799258e090eca1fe5526652c42b2e6d96622dd8e28"
ROUTE_SHA,PNG_NUMERATORS,MODEL_CAMERA_ORDER=previous.ROUTE_SHA,previous.PNG_NUMERATORS,previous.MODEL_CAMERA_ORDER
require,prior,visual=previous.require,previous.prior,previous.visual
camera_indices,png_indices,playback=previous.camera_indices,previous.png_indices,previous.playback
_canonical=previous._canonical


def verify_frozen():
    previous.verify_frozen()
    for path,pin in ((Path(previous.__file__),PREVIOUS_SHA),
            (CODE_ROOT/"learning/tests/test_publish_town03_straight_full209_v3_20261004.py",PREVIOUS_TEST_SHA)):
        _canonical(path,regular=True)
        require(prior.digest(path)==pin,"frozen V3 publication source/test changed")


def publication_sources():
    verify_frozen()
    return previous.previous.previous.previous.core.audit.source_closure(
        Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py")


def _namespace(function):
    # HH_261004 - Preserve each function's real original previous-layer while replacing only its fresh private identities/providers.
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
    require(ast.dump(restored)==ast.dump(original),"V4 publication changed beyond its exact identity seam")
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),
        str(Path(__file__))+":inverse_proven_private_V4_identity","exec"),namespace)
    return namespace[function.__name__]


def _delegate(function,*,transform=None):
    return _private(previous.previous._delegate)(function,transform=transform)


def _run_id_seam(tree):
    count=0
    for node in ast.walk(tree):
        if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v1":
            node.comparators[0].value="v4";count+=1
    require(count==1,"one original archive run-id V4 seam required")
    def undo(restored):
        for node in ast.walk(restored):
            if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                    and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v4":
                node.comparators[0].value="v1"
    return undo


def _policy_identity_seams(tree):
    old="HH_261004 - Fix whole actual V2 interval and all failure-tail sampling before one drive; unchanged V1 raw ABI and rendering math are independently pinned, without approval or future-record claims."
    new=old.replace("actual V2 interval","actual V4 interval",1)
    count=provenance=0
    for node in ast.walk(tree):
        if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v2":
            node.comparators[0].value="v4";count+=1
        elif isinstance(node,ast.Constant) and node.value==old: node.value=new;provenance+=1
    require(count==provenance==1,"one original V4 policy run identity and disclosure required")
    def undo(restored):
        for node in ast.walk(restored):
            if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')" \
                    and len(node.comparators)==1 and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value=="v4":
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
    text=_delegate(previous.previous.previous.markdown)(bundle,pngs)
    return text.replace("# 실제 Town03 209m 전체 경로 시험\n","# 실제 Town03 209m 전체 경로 V4 원본 버퍼·복원 재시험\n",1)+(
        "\n<!-- HH_261004 - Keep V3's missing-JPEG failure immutable and distinguish fresh source ownership from learned driving. -->\n\n"
        "V4는 기존 주행 모델·제어를 유지하고, 실제 카메라 원본 버퍼의 불변 복사본을 확보한 뒤 "
        "3개 작업자로 인코딩하고 모두 종료한 뒤 원래 순서로 기록하는 별도 연구입니다. "
        "신호등은 설치 전 실제 전체 목록을 기준으로 캡처 밖 2개 실제 틱에서 복원 상태를 검증합니다. "
        "V3의 JPEG0개 실패를 변경하거나 누락 이미지를 만들어 넣지 않았습니다. "
        "이 자료는 기록 기반 진단이며, 정식 기능 완료·실차 안전·실제 지연 개선은 별도 검증 전까지 주장하지 않습니다.\n")


def publish(args):
    return _private(previous.publish)(args)


def policy_document(plan,declaration_sha256):
    return _private(previous.previous.policy_document,transform=_policy_identity_seams)(plan,declaration_sha256)


def _verify_declared_sources(plan):
    return _private(previous._verify_declared_sources)(plan)


def seal_policy(expected_declaration_sha256):
    return _private(previous.seal_policy)(expected_declaration_sha256)


def main(argv=None):
    return _private(previous.main)(argv)


if __name__=="__main__":raise SystemExit(main())
