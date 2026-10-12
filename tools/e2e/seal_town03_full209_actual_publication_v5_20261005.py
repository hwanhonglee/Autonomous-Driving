"""HH_261005 - Prepare create-only V5 caller/postcheck through frozen whole-archive guards and external declaration and policy pins.

Import/source preparation never reads an actual episode. Execution belongs only
after the owned run ends. Both required SHA values are supplied independently by
ROOT, so freezing this source need not wait for a policy file or change later.
"""

from __future__ import annotations

import argparse
import ast
import copy
import inspect
from pathlib import Path
import re
import textwrap
from types import FunctionType

from . import publish_temporal_route_joint_town03_straight_full209_v5_20261005 as p
from . import seal_town03_full209_actual_publication_v4_20261004 as previous


CODE_ROOT=Path(__file__).resolve().parents[2]
PREVIOUS_SHA="1422286e28620a1f7700c29553e8b9350a2f4ca32edbbfc515c38dfea8c7f4a9"
PREVIOUS_TEST_SHA="b01ca874cf15465587d5a3e2d7bae01bb28df7290ff1fdd4491fdc1a04b31ec6"
PUBLISHER_SHA="966ce015fc67c41259adecf11906c9fa3564469b7731a47854c1e5c70f9dcf8f"
TEST=CODE_ROOT/"learning/tests/test_publish_town03_straight_full209_v5_20261005.py"
TEST_SHA="a821134cbfaad1639bb343a6611420e49715530d76a6ef3e716fd046c1a8d4d8"
CALLER=p.POLICY_PATH.parent/"ROOT_actual_publication_caller.json"
POSTCHECK=CODE_ROOT/"storage/experiments/diagnostics/2026-10-05/town03_full209_v5_publication_postcheck/ROOT_actual_whole_run_and_media.json"
POSTCHECK_SCHEMA="root_town03_full209_whole_record_publication_postcheck.20261005.v5"
ORIGINAL_POSTCHECK_SCHEMA=previous.ORIGINAL_POSTCHECK_SCHEMA


def verify_sources():
    # HH_261005 - Source authentication is independent of the external declaration/policy pins and never inspects a capture.
    p.verify_frozen();previous.verify_sources()
    for path,pin in ((Path(previous.__file__),PREVIOUS_SHA),
            (CODE_ROOT/"learning/tests/test_seal_town03_full209_actual_publication_v4_20261004.py",PREVIOUS_TEST_SHA),
            (Path(p.__file__),PUBLISHER_SHA),(TEST,TEST_SHA)):
        p._canonical(path,regular=True)
        p.require(p.prior.digest(path)==pin,"frozen V4 sealer or V5 publisher/test source changed")


def _delegate_save_new():
    """HH_261005 - Keep the genuine original canonical, no-overwrite and JSON exception behavior byte-identical."""
    verify_sources()
    function=previous.previous.previous.previous.save_new
    namespace={**function.__globals__,"p":p,"__file__":__file__}
    return FunctionType(function.__code__,namespace,function.__name__,function.__defaults__,function.__closure__)


def _identity_seams(tree,*,undo=False):
    # HH_261005 - Five frozen publisher layers precede the original inventory; only two references and one schema may change.
    expected="p.previous.previous.previous.previous.previous.inventory" if undo else "p.previous.inventory"
    replacement="p.previous.inventory" if undo else "p.previous.previous.previous.previous.previous.inventory"
    old_schema=POSTCHECK_SCHEMA if undo else ORIGINAL_POSTCHECK_SCHEMA
    new_schema=ORIGINAL_POSTCHECK_SCHEMA if undo else POSTCHECK_SCHEMA
    inventories=schemas=0
    for node in ast.walk(tree):
        if isinstance(node,ast.Call) and ast.unparse(node.func)==expected:
            node.func=ast.parse(replacement,mode="eval").body;inventories+=1
        elif isinstance(node,ast.Constant) and node.value==old_schema:
            node.value=new_schema;schemas+=1
    p.require(inventories==2 and schemas==1,"exact two original inventory calls and one V5 postcheck schema required")


def _namespace(declaration_sha,policy_sha):
    return {**previous.previous.previous.previous.main.__globals__,"__file__":__file__,"p":p,
        "PLAN_SHA":declaration_sha,"POLICY_SHA":policy_sha,"PUBLISHER_SHA":PUBLISHER_SHA,
        "TEST":TEST,"TEST_SHA":TEST_SHA,"CALLER":CALLER,"POSTCHECK":POSTCHECK,
        "save_new":_delegate_save_new()}


def _delegate_main(declaration_sha,policy_sha):
    """HH_261005 - Inverse-prove every authentic whole-archive, font, failed-tail, JPEG and media guard unchanged."""
    verify_sources()
    p.require(type(declaration_sha) is str and re.fullmatch("[0-9a-f]{64}",declaration_sha),"external exact declaration SHA256 required")
    p.require(type(policy_sha) is str and re.fullmatch("[0-9a-f]{64}",policy_sha),"external exact policy SHA256 required")
    original=ast.parse(textwrap.dedent(inspect.getsource(previous.previous.previous.previous.main))).body[0]
    changed=copy.deepcopy(original);_identity_seams(changed)
    restored=copy.deepcopy(changed);_identity_seams(restored,undo=True)
    p.require(ast.dump(restored)==ast.dump(original),"V5 whole-run verification changed beyond identity paths")
    namespace=_namespace(declaration_sha,policy_sha)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),
        str(Path(__file__))+":inverse_proven_private_V5_identity","exec"),namespace)
    return namespace["main"]


def _verify_declaration_pins(declaration_sha,policy_sha):
    # HH_261005 - Authenticate the canonical independently sealed declaration and policy before and after the unchanged caller/postcheck work.
    plan_path=p.RUN.with_name(p.RUN.name+"_plan.json")
    p._canonical(plan_path,regular=True)
    p.require(p.prior.digest(plan_path)==declaration_sha,"external pre-capture declaration SHA changed")
    p._canonical(p.POLICY_PATH,regular=True)
    p.require(p.prior.digest(p.POLICY_PATH)==policy_sha,"external pre-capture policy SHA changed")


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--expected-declaration-sha256",required=True)
    parser.add_argument("--expected-policy-sha256",required=True)
    args,remaining=parser.parse_known_args(argv)
    declaration_sha=args.expected_declaration_sha256
    policy_sha=args.expected_policy_sha256
    p.require(type(declaration_sha) is str and re.fullmatch("[0-9a-f]{64}",declaration_sha),"external exact declaration SHA256 required")
    p.require(type(policy_sha) is str and re.fullmatch("[0-9a-f]{64}",policy_sha),"external exact policy SHA256 required")
    verify_sources();_verify_declaration_pins(declaration_sha,policy_sha)
    result=_delegate_main(declaration_sha,policy_sha)(remaining)
    _verify_declaration_pins(declaration_sha,policy_sha);verify_sources()
    return result


if __name__=="__main__":main()
