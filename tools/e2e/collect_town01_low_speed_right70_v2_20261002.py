"""HH_261002 - V2 task identity only; RIGHT70 original controls/Green/coast/closing physics are unchanged."""
from __future__ import annotations

import ast
import copy

from scripts.e2e import collect_town01_low_speed_right70_20261002 as previous
from scripts.e2e import town01_low_speed_right70_task_v2_20261002 as declared_task

PREVIOUS_SHA="f5bfbe8c4ed4426c3611f9858d326edb3bc193a273617b92de420b804a7c4be8"


def identity_ast():
    declared_task.require(declared_task.sha(previous.__file__)==PREVIOUS_SHA,"frozen RIGHT70 V1 worker changed")
    original=previous.identity_definition();changed=copy.deepcopy(original);imports=versions=0
    for node in ast.walk(changed):
        if isinstance(node,ast.ImportFrom):
            for alias in node.names:
                if alias.name=="town01_low_speed_right70_task_20261002":alias.name="town01_low_speed_right70_task_v2_20261002";imports+=1
        if isinstance(node,ast.Constant) and node.value=="v1":node.value="v2";versions+=1
    declared_task.require(imports==1 and versions==2,"one current task and two exact protocol identities required")
    inverse=copy.deepcopy(changed)
    for node in ast.walk(inverse):
        if isinstance(node,ast.ImportFrom):
            for alias in node.names:
                if alias.name=="town01_low_speed_right70_task_v2_20261002":alias.name="town01_low_speed_right70_task_20261002"
        if isinstance(node,ast.Constant) and node.value=="v2":node.value="v1"
    declared_task.require(ast.dump(inverse)==ast.dump(original),"V2 changed capture/control/fixture physics")
    return changed


exec(compile(ast.fix_missing_locations(identity_ast()),__file__,"exec"),globals())
