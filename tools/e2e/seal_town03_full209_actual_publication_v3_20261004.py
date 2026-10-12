"""HH_261004 - Prepare create-only V3 caller/postcheck through frozen whole-archive guards and an external policy pin.

Import/source preparation never reads an actual episode. Execution belongs only
after the owned run ends. The required policy SHA is supplied independently by
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

from . import publish_temporal_route_joint_town03_straight_full209_v3_20261004 as p
from . import seal_town03_full209_actual_publication_v2_20261004 as previous


CODE_ROOT=Path(__file__).resolve().parents[2]
PREVIOUS_SHA="093e46cacbe16afdf62165bf1e28ad75c50dcfacc2be036fcdba170abbe1c721"
PREVIOUS_TEST_SHA="65ccf87733d49a639e41de46747f7a8e9da7c453b949c21fff0177846fcc1be6"
PLAN_SHA="5d523c4c01dcff317b43fe36b445e9e380ffb2de902d9e2370957dd3cdaae2fb"
PUBLISHER_SHA="63b1b569eab1bf7db687065e21f7ec02420ffec6c410481509c62c2b14142d5d"
TEST=CODE_ROOT/"learning/tests/test_publish_town03_straight_full209_v3_20261004.py"
TEST_SHA="07cee3d9b4e63a825c719f799258e090eca1fe5526652c42b2e6d96622dd8e28"
CALLER=p.POLICY_PATH.parent/"ROOT_actual_publication_caller.json"
POSTCHECK=CODE_ROOT/"storage/experiments/diagnostics/2026-10-04/town03_full209_v3_publication_postcheck/ROOT_actual_whole_run_and_media.json"
POSTCHECK_SCHEMA="root_town03_full209_whole_record_publication_postcheck.20261004.v3"
ORIGINAL_POSTCHECK_SCHEMA=previous.ORIGINAL_POSTCHECK_SCHEMA


def verify_sources():
    # HH_261004 - Source authentication is independent of the external policy pin and never inspects a capture.
    p.verify_frozen();previous.verify_sources()
    for path,pin in ((Path(previous.__file__),PREVIOUS_SHA),
            (CODE_ROOT/"learning/tests/test_seal_town03_full209_actual_publication_v2_20261004.py",PREVIOUS_TEST_SHA),
            (Path(p.__file__),PUBLISHER_SHA),(TEST,TEST_SHA)):
        p._canonical(path,regular=True)
        p.require(p.prior.digest(path)==pin,"frozen V2 sealer or V3 publisher/test source changed")


def _delegate_save_new():
    """HH_261004 - Keep the genuine original canonical, no-overwrite and JSON exception behavior byte-identical."""
    verify_sources()
    function=previous.previous.save_new
    namespace={**function.__globals__,"p":p,"__file__":__file__}
    return FunctionType(function.__code__,namespace,function.__name__,function.__defaults__,function.__closure__)


def _identity_seams(tree,*,undo=False):
    # HH_261004 - Three frozen publisher layers precede the original inventory; only two references and one schema may change.
    expected="p.previous.previous.previous.inventory" if undo else "p.previous.inventory"
    replacement="p.previous.inventory" if undo else "p.previous.previous.previous.inventory"
    old_schema=POSTCHECK_SCHEMA if undo else ORIGINAL_POSTCHECK_SCHEMA
    new_schema=ORIGINAL_POSTCHECK_SCHEMA if undo else POSTCHECK_SCHEMA
    inventories=schemas=0
    for node in ast.walk(tree):
        if isinstance(node,ast.Call) and ast.unparse(node.func)==expected:
            node.func=ast.parse(replacement,mode="eval").body;inventories+=1
        elif isinstance(node,ast.Constant) and node.value==old_schema:
            node.value=new_schema;schemas+=1
    p.require(inventories==2 and schemas==1,"exact two original inventory calls and one V3 postcheck schema required")


def _namespace(policy_sha):
    return {**previous.previous.main.__globals__,"__file__":__file__,"p":p,
        "PLAN_SHA":PLAN_SHA,"POLICY_SHA":policy_sha,"PUBLISHER_SHA":PUBLISHER_SHA,
        "TEST":TEST,"TEST_SHA":TEST_SHA,"CALLER":CALLER,"POSTCHECK":POSTCHECK,
        "save_new":_delegate_save_new()}


def _delegate_main(policy_sha):
    """HH_261004 - Inverse-prove every authentic whole-archive, font, failed-tail, JPEG and media guard unchanged."""
    verify_sources()
    p.require(type(policy_sha) is str and re.fullmatch("[0-9a-f]{64}",policy_sha),"external exact policy SHA256 required")
    original=ast.parse(textwrap.dedent(inspect.getsource(previous.previous.main))).body[0]
    changed=copy.deepcopy(original);_identity_seams(changed)
    restored=copy.deepcopy(changed);_identity_seams(restored,undo=True)
    p.require(ast.dump(restored)==ast.dump(original),"V3 whole-run verification changed beyond identity paths")
    namespace=_namespace(policy_sha)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),
        str(Path(__file__))+":inverse_proven_private_V3_identity","exec"),namespace)
    return namespace["main"]


def _verify_policy_pin(policy_sha):
    # HH_261004 - Authenticate the canonical independently sealed policy before and after the unchanged caller/postcheck work.
    p._canonical(p.POLICY_PATH,regular=True)
    p.require(p.prior.digest(p.POLICY_PATH)==policy_sha,"external pre-capture policy SHA changed")


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--expected-policy-sha256",required=True)
    args,remaining=parser.parse_known_args(argv)
    policy_sha=args.expected_policy_sha256
    p.require(type(policy_sha) is str and re.fullmatch("[0-9a-f]{64}",policy_sha),"external exact policy SHA256 required")
    verify_sources();_verify_policy_pin(policy_sha)
    result=_delegate_main(policy_sha)(remaining)
    _verify_policy_pin(policy_sha);verify_sources()
    return result


if __name__=="__main__":main()
