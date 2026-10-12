"""HH_261002 - RIGHT70 identity-only private reuse of the immutable LEFT collector/control/coast/Green physics."""
from __future__ import annotations

import ast
import copy
import hashlib
from pathlib import Path

from scripts.e2e import collect_town01_low_speed_left45_20261002 as previous
from scripts.e2e import town01_low_speed_right70_task_20261002 as right_task

PREVIOUS_SHA="e87b64c9f1df8348561c5a5ee3625e8bc6a90fde22e490b88dc45647dae36cb3"


def definition():
    raw=Path(previous.__file__).read_bytes()
    right_task.require(hashlib.sha256(raw).hexdigest()==PREVIOUS_SHA,"frozen LEFT collector changed")
    tree=ast.parse(raw);old=copy.deepcopy(tree);changes=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.ImportFrom):
            for alias in node.names:
                if alias.name=="town01_low_speed_left45_task_20261002":
                    changes.append((alias,"name",alias.name));alias.name="town01_low_speed_right70_task_20261002"
        if isinstance(node,ast.Constant) and type(node.value) is str:
            before=node.value;node.value=before.replace("LEFT45","RIGHT70").replace("LEFT","RIGHT").replace("left45","right70")
            if before!=node.value:changes.append((node,"value",before))
    inverse=copy.deepcopy(tree);new_nodes=list(ast.walk(tree));inverse_nodes=list(ast.walk(inverse))
    for node,key,before in changes:setattr(inverse_nodes[new_nodes.index(node)],key,before)
    right_task.require(ast.dump(inverse)==ast.dump(old),"RIGHT70 altered original capture/control/fixture mathematics")
    return tree


identity_definition=definition
exec(compile(ast.fix_missing_locations(identity_definition()),__file__,"exec"),globals())
