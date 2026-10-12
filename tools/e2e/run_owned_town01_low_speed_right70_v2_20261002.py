"""HH_261002 - Fresh V2 parser-compatible ownership namespace; V1 preflight failure stays immutable and no capture is implicit."""
from __future__ import annotations

import ast
import copy
from pathlib import Path
import sys

from scripts.e2e import run_owned_town01_low_speed_right70_20261002 as previous
from scripts.e2e import town01_low_speed_right70_task_v2_20261002 as declared_task
from scripts.e2e import collect_town01_low_speed_right70_v2_20261002 as declared_worker

V1_SOURCE_SHA="df929a5db8762ad165bfe592b65cd9544cee7390130193bf8dba81f81b1ce16e"
OLD_MODULE="scripts.e2e.run_owned_town01_low_speed_right70_20261002"
NEW_MODULE="scripts.e2e.run_owned_town01_low_speed_right70_v2_20261002"


def identity_ast():
    declared_task.require(declared_task.sha(previous.__file__)==V1_SOURCE_SHA,"frozen RIGHT70 V1 owner changed")
    tree=ast.parse(Path(previous.__file__).read_bytes());original=ast.Module(body=[n for n in tree.body if not isinstance(n,ast.If)],type_ignores=[])
    declared_task.require(len(tree.body)-len(original.body)==1,"unique original main guard required")
    changed=copy.deepcopy(original);changes=[]
    names={"town01_low_speed_right70_task_20261002":"town01_low_speed_right70_task_v2_20261002",
        "collect_town01_low_speed_right70_20261002":"collect_town01_low_speed_right70_v2_20261002"}
    for node in ast.walk(changed):
        if isinstance(node,ast.ImportFrom):
            for alias in node.names:
                if alias.name in names:changes.append((alias,"name",alias.name));alias.name=names[alias.name]
        if isinstance(node,ast.Constant) and type(node.value) is str:
            if node.value in names:changes.append((node,"value",node.value));node.value=names[node.value]
            elif node.value=="v1":changes.append((node,"value",node.value));node.value="v2"
    inverse=copy.deepcopy(changed);new_nodes=list(ast.walk(changed));inverse_nodes=list(ast.walk(inverse))
    for node,key,before in changes:setattr(inverse_nodes[new_nodes.index(node)],key,before)
    declared_task.require(ast.dump(inverse)==ast.dump(original),"V2 changed ownership/physics/source predicates")
    return changed


revision_ast=identity_ast
exec(compile(ast.fix_missing_locations(revision_ast()),__file__,"exec"),globals())
require(task is declared_task and worker_module is declared_worker,"current V2 task/worker must be source-bound")
# HH_261002 - The older LEFT→RIGHT compiler generated the V1 module string; bind only this new worker identity in its private globals.
MODULE=NEW_MODULE
_originals["MODULE"]=MODULE
require(task.fixed_contract("v2","pilot_00")["closing_receipt_reader_schema"]==closing.SCHEMA,
    "the exact original parser must now see both owner metadata keys as fixed fields")
_original_main=main


def main(argv=None):
    values=list(sys.argv[1:] if argv is None else argv)
    if "--run-id" not in values:values=["--run-id","v2",*values]
    return _original_main(values)


if __name__=="__main__":raise SystemExit(main())
