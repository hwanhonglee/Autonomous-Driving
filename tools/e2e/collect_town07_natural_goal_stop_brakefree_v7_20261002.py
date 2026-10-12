"""HH_261002 - Set only normal terminal-phase steering to zero, before untouched BasicAgent hazards."""

from __future__ import annotations

import ast

from scripts.e2e import town07_natural_goal_stop_brakefree_v7_20261002 as task_v7
from scripts.e2e import collect_town07_natural_goal_stop_brakefree_v6_20261002 as frozen_v6_collector


def _steering_intervention(tree):
    # HH_261002 - Add one normal terminal steer assignment and audit fields; all prior predicates/pedals/control ordering stay byte-identical.
    controller=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=="BrakeFreeLaunchCoastHoldControl")
    method=next(node for node in controller.body if isinstance(node,ast.FunctionDef) and node.name=="apply")
    test=ast.parse('phase in ("BRAKE_TO_STOP","HOLD")',mode="eval").body
    terminal=[node for node in method.body if isinstance(node,ast.If) and ast.dump(node.test,include_attributes=False)==ast.dump(test,include_attributes=False)]
    task_v7.require(len(terminal)==1,"one exact normal terminal phase branch required")
    assignment=ast.parse('result.steer=0.').body[0];terminal[0].body.append(assignment)
    update=[node.value for node in method.body if isinstance(node,ast.Expr) and isinstance(node.value,ast.Call)
        and isinstance(node.value.func,ast.Attribute) and node.value.func.attr=="update"]
    task_v7.require(len(update)==1,"one exact normal-policy audit receipt required")
    extra=[ast.keyword(arg="normal_terminal_steering_zero_declared",value=ast.Constant(value=True)),
        ast.keyword(arg="normal_terminal_steering_zero_applied",value=ast.parse('phase in ("BRAKE_TO_STOP","HOLD")',mode="eval").body),
        ast.keyword(arg="original_GO_steering_unchanged",value=ast.Constant(value=True))]
    update[0].keywords.extend(extra)
    def undo():
        terminal[0].body.remove(assignment)
        for keyword in extra:update[0].keywords.remove(keyword)
    return undo


task_v7.clone_frozen_provider(frozen_v6_collector,
    task_v7.V6_SOURCES["tools/e2e/collect_town07_natural_goal_stop_brakefree_v6_20261002.py"],globals(),
    literals={"v6":"v7",
        "Disclosed_BasicAgent_manual1_1p50_latched_brakefree_coast_low_speed_hold_v6":"Disclosed_BasicAgent_manual1_1p50_latched_brakefree_coast_low_speed_hold_v7",
        "Private brakefree v6 worker; use run_owned_town07_natural_goal_stop_brakefree_v6_20261002 --mode execute":"Private brakefree v7 worker; use run_owned_town07_natural_goal_stop_brakefree_v7_20261002 --mode execute"},
    imports={"town07_natural_goal_stop_brakefree_v6_20261002":"town07_natural_goal_stop_brakefree_v7_20261002"},
    intervention=_steering_intervention)

if __name__=="__main__":raise SystemExit(main())
