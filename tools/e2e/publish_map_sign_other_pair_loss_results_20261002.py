"""HH_261002 - Reuse the full350 saved-prediction audit for the separate sign/other pair-loss identity, not new inference."""
from __future__ import annotations

import argparse
import ast
import copy
from pathlib import Path
import sys
from types import FunctionType

from . import publish_map_sign_margin_loss_results_20261002 as previous

ROOT,frozen,require=previous.ROOT,previous.frozen,previous.require
PARENT_SHA="f9b4b9096c83593d6ab924d94fc858356730c5b32fa122b4b18aa8acbe7cad0e"
OUTPUT=ROOT/"docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/19_map_sign_other_pair_loss"
RUN=ROOT/"storage/experiments/training/2026-10-02/map_sign_other_margin_repair_v1"
NAME="map_sign_other_margin_repair_20261002_v1"
REMOTE_RUN="/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/"+NAME
FIT_SCHEMA="portable_e2e.map_sign_other_margin_repair.20261002.v1"
SCHEMA="portable_e2e.map_sign_other_pair_loss_results.20261002.v1"
CALLER_SCHEMA=SCHEMA+".caller_pins"
SOURCE=dict(previous.SOURCE,**{
    "map_sign_other_margin_loss_v1_20261002.py":"a1ac4318478645a25cbe55e302fc204b113e7263f826f3948da0e5dd6fccb1dc",
    "train_map_sign_other_margin_repair_20261002.py":"536a994b446b7575b15e1342860471a256bab6be927e42ac264ae114d641ca24"})
IDENTITY_KEYS=("actual_fit_authorization_sha256","equal_global_schedule_proof_document_sha256","loss_contract_document_sha256")


def verify_frozen():
    require("torch" not in sys.modules and "carla" not in sys.modules
        and frozen.contract.digest(Path(previous.__file__))==PARENT_SHA,
        "exact frozen full350 publication provider/no Torch or simulator required")


def publication_sources():
    verify_frozen()
    return previous.publication_sources()|{str(Path(__file__).resolve().relative_to(ROOT)):Path(__file__).resolve()}


def local_source(name):
    return ROOT/("learning/portable_e2e" if name in ("map_context_recovery.py","map_perception.py",
        "map_sign_margin_loss_v1_20261002.py","map_sign_other_margin_loss_v1_20261002.py") else "tools/e2e")/name


def revised_AST(name):
    # HH_261002 - The original full49/350/27 math and guards are inverse-proven; only identities/count labels change.
    verify_frozen();tree=ast.parse(Path(previous.__file__).read_bytes())
    original=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
    changed=copy.deepcopy(original);edits=[]
    text={"read_study":{"complete exact51 mirror required":"complete exact53 pair-loss mirror required"},
        "canvas":{"Candidate / CE +0.10 sign margin +4000":"Candidate / CE +0.10 sign-other pair +4000"},
        "markdown":{"# 표지 손실 교정: 실제 결과와 원본 화면":"# 표지↔배경 손실 교정: 실제 결과와 원본 화면",
            "같은 10,000회 시작 모델에서 대조군·후보군을 각각 4,000회 추가 학습했습니다. 양쪽 실제 optimizer 노출은 각각 32,000회이며, CUDA 사전검사 8회 노출씩은 별도입니다. 영상·정답·crop·자료 순서·FALSE 영상·원 CE는 같고 후보만 표지 margin 손실 0.10을 더했습니다.":
            "같은 10,000회 시작 모델에서 대조군·후보군을 각각 4,000회 추가 학습했습니다. 양쪽 실제 optimizer 노출은 각각 32,000회이며 CUDA 사전검사 8회 노출씩은 별도입니다. 영상·정답·crop·순서·FALSE 영상·원 CE는 같고 후보만 표지↔other 조건부 pair margin 손실 0.10을 더했습니다. 직접 보조 logit gradient는 두 분류뿐이지만 공유 특징을 통한 차선·재현율 퇴행은 가능합니다. 이전 표지 손실 실패를 덮거나 새로운 head·문턱값을 추가한 것이 아닙니다."}}.get(name,{})
    counts={key:0 for key in text}
    for node in ast.walk(changed):
        if isinstance(node,ast.Constant) and type(node.value) is str and node.value in text:
            old=node.value;edits.append((node,old));node.value=text[old];counts[old]+=1
        if name=="audit" and isinstance(node,ast.keyword) and node.arg=="source_count":
            require(isinstance(node.value,ast.Constant) and type(node.value.value) is int and node.value.value==10,
                "exact original10 source diagnostic count required")
            edits.append((node.value,10));node.value.value=12
    require(all(n==1 for n in counts.values()) and len(edits)==len(text)+(name=="audit"),"exact identity-only AST anchors required")
    result=copy.deepcopy(changed)
    for node,value in edits:node.value=value
    require(ast.dump(changed)==ast.dump(original),"an original pixel/metric/predicate/receipt/publication statement changed")
    return result


def reader_namespace(identity,destination=None):
    verify_frozen()
    require(type(identity) is dict and set(identity)==set(IDENTITY_KEYS),"three externally sealed actual experiment identities required")
    for pin in identity.values():frozen.sha_string(pin)
    values=dict(vars(previous),OUTPUT=OUTPUT,RUN=RUN,NAME=NAME,REMOTE_RUN=REMOTE_RUN,FIT_SCHEMA=FIT_SCHEMA,
        SCHEMA=SCHEMA,CALLER_SCHEMA=CALLER_SCHEMA,SOURCE=SOURCE,
        AUTH_SHA=identity[IDENTITY_KEYS[0]],EQUAL_PROOF_SHA=identity[IDENTITY_KEYS[1]],LOSS_CONTRACT_SHA=identity[IDENTITY_KEYS[2]],
        publication_sources=publication_sources,local_source=local_source)
    namespace={} if destination is None else destination;namespace.update(values)
    namespace["required_run_files"]=FunctionType(previous.required_run_files.__code__,namespace,"required_run_files")
    for name in ("read_study","audit","canvas","markdown"):
        node=revised_AST(name)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(__file__)+":identity_only_"+name,"exec"),namespace)
    original_reader=namespace["read_study"]
    def read_study(ref,pins):
        result=original_reader(ref,pins)
        summary=result["summary"]
        for key,wanted in dict(source_count=12,direct_auxiliary_logits_are_only_sign5_other0=True,
            shared_feature_indirect_lane_or_recall_regression_possible=True,new_head=False,
            inference_argmax_or_threshold_changed=False).items():
            require(type(summary.get(key)) is type(wanted) and summary[key]==wanted,"exact actual pair-loss identity differs: "+key)
        loss=summary["loss_contract"]
        require(loss.get("margin")=="sign_logit-other_logit" and loss.get("eligible_negative_target_class")==0
            and loss.get("direct_auxiliary_logit_gradient_classes")==[0,5]
            and loss.get("conditional_probability")=="exp(sign)/(exp(sign)+exp(other)); NOT seven-class sign probability",
            "original seven-class auxiliary cannot masquerade as conditional sign/other pair loss")
        return result
    namespace["read_study"]=read_study
    original_audit=namespace["audit"]
    def audit(path,pin):
        report,panels,pins=original_audit(path,pin)
        report.update(publication_identity_revision=SCHEMA,frozen_full350_publisher_sha256=PARENT_SHA,
            actual_experiment_identity=dict(identity),candidate_pair_conditional_probability_not_seven_class_probability=True,
            direct_auxiliary_logit_gradient_classes=[0,5],shared_feature_indirect_regression_possible=True)
        return report,panels,pins
    namespace["audit"]=audit
    namespace["publish"]=FunctionType(previous.publish.__code__,namespace,"publish")
    return namespace


def caller_namespace(path,pin):
    verify_frozen();pins=frozen.Pins();caller=pins.json(path,pin)
    require(caller.get("schema")==CALLER_SCHEMA and caller.get("output")==str(OUTPUT)
        and caller.get("fixed_indices")==previous.INDICES,"exact unchanged prospective fixed10 caller required")
    identity=caller.get("actual_experiment_identity")
    namespace=reader_namespace(identity)
    require(caller.get("run",{}).get("files",{}).get("authorization.json")==identity[IDENTITY_KEYS[0]],
        "externally pinned new fit authority must match the full actual53 artifact table")
    pins.recheck();return namespace


def audit(path,pin):return caller_namespace(Path(path),pin)["audit"](Path(path),pin)


def publish(args):return caller_namespace(Path(args.caller_pins),args.caller_pins_sha256)["publish"](args)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--caller-pins",required=True);parser.add_argument("--caller-pins-sha256",required=True)
    parser.add_argument("--validate-only",action="store_true")
    print(previous.json.dumps(publish(parser.parse_args(argv)),ensure_ascii=False,sort_keys=True),flush=True);return 0


if __name__=="__main__":raise SystemExit(main())
