"""HH_261002 - All597 actual RIGHT70 true64 windows; immutable original numerical gates, no TRAIN admission."""
from __future__ import annotations

import ast
import copy
import hashlib
from pathlib import Path

from scripts.e2e import audit_town01_left45_true_future64_20261002 as previous
from scripts.e2e import run_owned_town01_low_speed_right70_20261002 as declared_owner

PREVIOUS_SHA="f08dab566d6fcca8fac7a0e39b01cf1ba4a8dbad44091c0f9b5950cb76aecbda"


def identity_ast():
    raw=Path(previous.__file__).read_bytes()
    previous.require(hashlib.sha256(raw).hexdigest()==PREVIOUS_SHA,"frozen whole LEFT true64 diagnostic changed")
    tree=ast.parse(raw);old=ast.Module(body=[n for n in tree.body if not isinstance(n,ast.If)],type_ignores=[])
    previous.require(len(tree.body)-len(old.body)==1,"unique frozen main guard required")
    new=copy.deepcopy(old);changes=[]
    for node in ast.walk(new):
        if isinstance(node,ast.ImportFrom):
            for alias in node.names:
                if alias.name=="run_owned_town01_low_speed_left45_v2_20261002":
                    changes.append((alias,"name",alias.name));alias.name="run_owned_town01_low_speed_right70_20261002"
        if isinstance(node,ast.Constant) and type(node.value) is str:
            before=node.value
            if before==previous.SCHEMA:node.value="carla.actual_town01_right70_true_future64_diagnostic.20261002.v1"
            elif before==str(previous.OUTPUT.relative_to(previous.ROOT)):
                node.value="storage/experiments/diagnostics/2026-10-02/town01_low_speed_right70_future64_v1"
            elif before=="v2":node.value="v1"
            elif before=="town01_left_s0002_p03_actual_left45":node.value="town01_right_6p1_108p1_24p1_actual_right70"
            # HH_261002 - The frozen dependency table remains byte-for-byte unchanged; only active protocol/wording/journal identities change.
            elif not before.startswith("tools/e2e/"):
                node.value=before.replace("LEFT","RIGHT").replace("left45_protocol","right70_protocol").replace("all347_true_future","all597_true_future")
            if before!=node.value:changes.append((node,"value",before))
    inverse=copy.deepcopy(new);new_nodes=list(ast.walk(new));inverse_nodes=list(ast.walk(inverse))
    for node,key,before in changes:setattr(inverse_nodes[new_nodes.index(node)],key,before)
    previous.require(ast.dump(inverse)==ast.dump(old),"RIGHT70 changed true64/native/body/source/denial math")
    return new


exec(compile(ast.fix_missing_locations(identity_ast()),__file__,"exec"),globals())
require(owner is declared_owner,"actual RIGHT70 source provider must be statically bound")
FROZEN.update({"tools/e2e/audit_town01_left45_true_future64_20261002.py":PREVIOUS_SHA,
    "tools/e2e/audit_town01_right70_closing_control_20261002.py":owner.CLOSING_SHA,
    "tools/e2e/prepare_town01_right70_capture_route_20261002.py":"bb8fff98e029692efd8ef44156ebef90d831fc6bf21691f8d71b2cc8c473ef96",
    "tools/e2e/town01_low_speed_right70_task_20261002.py":"49704b8634e887c7d760a0c7b57ddf9726f8d9d8eef822a7493c475ce831f3ea",
    "tools/e2e/collect_town01_low_speed_right70_20261002.py":"f5bfbe8c4ed4426c3611f9858d326edb3bc193a273617b92de420b804a7c4be8",
    "tools/e2e/run_owned_town01_low_speed_right70_20261002.py":"df929a5db8762ad165bfe592b65cd9544cee7390130193bf8dba81f81b1ce16e"})


if __name__=="__main__":raise SystemExit(main())
