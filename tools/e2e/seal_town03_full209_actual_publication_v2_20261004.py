"""HH_261004 - Pin and postcheck the completed actual V2 whole-drive archive using exact frozen V1 verification code.

Only immutable run/publication identities and the V2 publisher's extra module
layer change. No actual archive is accessed at import or source review. Caller
and postcheck writes are create-only; execution belongs after the owned run has
ended and does not start simulation, inference, training or model promotion.
"""

from __future__ import annotations

import ast
import copy
import inspect
from pathlib import Path
import textwrap
from types import FunctionType

from . import publish_temporal_route_joint_town03_straight_full209_v2_20261004 as p
from . import seal_town03_full209_actual_publication_v1_20261004 as previous


CODE_ROOT=Path(__file__).resolve().parents[2]
PREVIOUS_SHA="7dc6d07425ec20412a52d82f84d4c47428bc300acefb20cd90b8a1cc3f53c08b"
PLAN_SHA="797f217310db8f6a1533b063e6bfa07e975591873c8e36ed962e5a73d35e98b8"
POLICY_SHA="a386ae9955a6e6251095aa8da85165e83e8d8b899db1bc81bfc85470417c19a8"
PUBLISHER_SHA="354abb6ed79074af3de835ea4d0893d17755cf0a8010fd60917ef397de3e6669"
TEST=CODE_ROOT/"learning/tests/test_publish_town03_straight_full209_v2_20261004.py"
TEST_SHA="4556f07113a5254b851cd74fb61ca61138786f12ed5fcac4fc12b101decfa49c"
CALLER=p.POLICY_PATH.parent/"ROOT_actual_publication_caller.json"
POSTCHECK=CODE_ROOT/"storage/experiments/diagnostics/2026-10-04/town03_full209_v2_publication_postcheck/ROOT_actual_whole_run_and_media.json"
POSTCHECK_SCHEMA="root_town03_full209_whole_record_publication_postcheck.20261004.v2"
ORIGINAL_POSTCHECK_SCHEMA="root_town03_full209_whole_record_publication_postcheck.20261004.v1"


def verify_sources():
    # HH_261004 - Canonical source authentication never dereferences a changed source alias or reads a live episode.
    p.verify_frozen()
    for path,pin in ((Path(previous.__file__),PREVIOUS_SHA),(Path(p.__file__),PUBLISHER_SHA),(TEST,TEST_SHA)):
        p._canonical(path,regular=True)
        p.require(p.prior.digest(path)==pin,"frozen whole-run sealer or V2 publisher/test source changed")


def _namespace():
    return {**previous.main.__globals__,"__file__":__file__,"p":p,
        "PLAN_SHA":PLAN_SHA,"POLICY_SHA":POLICY_SHA,"PUBLISHER_SHA":PUBLISHER_SHA,
        "TEST":TEST,"TEST_SHA":TEST_SHA,"CALLER":CALLER,"POSTCHECK":POSTCHECK,
        "save_new":_delegate_save_new()}


def _delegate_save_new():
    """HH_261004 - Preserve original create-only canonical-path verification and byte serialization exactly."""
    verify_sources()
    function=previous.save_new
    namespace={**function.__globals__,"p":p,"__file__":__file__}
    return FunctionType(function.__code__,namespace,function.__name__,function.__defaults__,function.__closure__)


def _identity_seams(tree,*,undo=False):
    # HH_261004 - The V2 wrapper has one extra frozen publisher layer; unwrap only the two original inventory calls.
    expected="p.previous.previous.inventory" if undo else "p.previous.inventory"
    replacement="p.previous.inventory" if undo else "p.previous.previous.inventory"
    old_schema=POSTCHECK_SCHEMA if undo else ORIGINAL_POSTCHECK_SCHEMA
    new_schema=ORIGINAL_POSTCHECK_SCHEMA if undo else POSTCHECK_SCHEMA
    inventories=schemas=0
    for node in ast.walk(tree):
        if isinstance(node,ast.Call) and ast.unparse(node.func)==expected:
            node.func=ast.parse(replacement,mode="eval").body;inventories+=1
        elif isinstance(node,ast.Constant) and node.value==old_schema:
            node.value=new_schema;schemas+=1
    p.require(inventories==2 and schemas==1,"exact two authentic inventory paths and one V2 postcheck identity required")


def _delegate_main():
    """HH_261004 - Inverse-prove all original archive, source, font, JPEG, GIF, PNG, failed-tail and recheck guards unchanged."""
    verify_sources()
    original=ast.parse(textwrap.dedent(inspect.getsource(previous.main))).body[0]
    changed=copy.deepcopy(original)
    _identity_seams(changed)
    restored=copy.deepcopy(changed)
    _identity_seams(restored,undo=True)
    p.require(ast.dump(original)==ast.dump(restored),"whole-run caller/postcheck changed beyond its exact V2 identity seams")
    namespace=_namespace()
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),
        str(Path(__file__))+":inverse_proven_private_V2_identity","exec"),namespace)
    return namespace["main"]


def main(argv=None):
    verify_sources()
    result=_delegate_main()(argv)
    verify_sources()
    return result


if __name__=="__main__":
    main()
