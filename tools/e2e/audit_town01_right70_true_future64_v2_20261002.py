"""HH_261002 - Fresh V2 RIGHT70 identity only; every true64/native/body/pixel/denial formula remains frozen and no TRAIN permission is created."""
from __future__ import annotations

import ast
import copy
from pathlib import Path

from scripts.e2e import audit_town01_right70_true_future64_20261002 as previous
from scripts.e2e import run_owned_town01_low_speed_right70_v2_20261002 as declared_owner

PREVIOUS_SHA="b6eba1a39889fbdedb664f0e49062546f01e929c53dc203e10cd5a7e911af21a"
IDENTITIES={previous.SCHEMA:"carla.actual_town01_right70_true_future64_diagnostic.20261002.v2",
    str(previous.OUTPUT.relative_to(previous.ROOT)):"storage/experiments/diagnostics/2026-10-02/town01_low_speed_right70_future64_v2",
    "v1":"v2"}


def identity_ast():
    previous.require(previous.owner.digest(previous.__file__)==PREVIOUS_SHA,"frozen RIGHT70 V1 full-future diagnostic changed")
    old=previous.identity_ast();new=copy.deepcopy(old);changes=[];imports=0;counts=dict.fromkeys(IDENTITIES,0)
    for node in ast.walk(new):
        if isinstance(node,ast.ImportFrom):
            for alias in node.names:
                if alias.name=="run_owned_town01_low_speed_right70_20261002":
                    changes.append((alias,"name",alias.name));alias.name="run_owned_town01_low_speed_right70_v2_20261002";imports+=1
        if isinstance(node,ast.Constant) and type(node.value) is str and node.value in IDENTITIES:
            before=node.value;counts[before]+=1;changes.append((node,"value",before));node.value=IDENTITIES[before]
    previous.require(imports==1 and counts=={previous.SCHEMA:1,str(previous.OUTPUT.relative_to(previous.ROOT)):1,"v1":4},
        "one current owner plus exact schema/output/four case identities required")
    inverse=copy.deepcopy(new);nodes=list(ast.walk(new));inverse_nodes=list(ast.walk(inverse))
    for node,key,before in changes:setattr(inverse_nodes[nodes.index(node)],key,before)
    previous.require(ast.dump(inverse)==ast.dump(old),"V2 changed true64/geometry/mask/native/body/source/denial checks")
    return new


exec(compile(ast.fix_missing_locations(identity_ast()),__file__,"exec"),globals())
require(owner is declared_owner and task is owner.task,"actual current V2 owner/task required, not preflight-failed V1")
FROZEN.update(previous.FROZEN)
FROZEN.update({"tools/e2e/audit_town01_right70_true_future64_20261002.py":PREVIOUS_SHA,
    "tools/e2e/town01_low_speed_right70_task_v2_20261002.py":"529955f75bdede135d99ce093dc8f00b675d60785277b031fa81740b9860f04f",
    "tools/e2e/collect_town01_low_speed_right70_v2_20261002.py":"af27e39776ccee1f5431a9c2307da58c8b03c9445ac9a9c7a2dc4bf9dff9d835",
    "tools/e2e/run_owned_town01_low_speed_right70_v2_20261002.py":"01f919763a05eab2f0b6b7df002f70e933859eac7c0d7687274673d94c600a3d"})


if __name__=="__main__":raise SystemExit(main())
