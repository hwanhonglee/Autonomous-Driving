"""HH_261002 - Prepare one terminal-only zero-normal-steering hypothesis; no capture or TRAIN permission.

The entire original213-point route and GO steering remain untouched. Only its
last12.5m must satisfy a strict source-geometry straightness precondition, before
any capture. Existing11.5m/1.48..1.58m/s coast timing and all safety gates remain.
"""

from __future__ import annotations

import ast
from copy import deepcopy
import math
from pathlib import Path

from scripts.e2e import town07_natural_goal_stop_brakefree_v6_20261002 as frozen_v6_task

V6_SOURCES = {
    "tools/e2e/town07_natural_goal_stop_brakefree_v6_20261002.py": "de86e3610992af663235d7b733868b7525aa4177d967fe936a1628ab642fb73c",
    "tools/e2e/collect_town07_natural_goal_stop_brakefree_v6_20261002.py": "bc83374a3dcd3f25b4776da3062b4009d73217ee15949d322b912d39bf239f73",
    "tools/e2e/run_owned_town07_natural_goal_stop_brakefree_v6_20261002.py": "07c628820da9e2eaae120ab7b0a4e595ce2f96c0063de03ac4f2b2964c498719",
}


def clone_frozen_provider(module, expected_sha, namespace, *, literals, imports=None, intervention=None):
    # HH_261002 - Mechanically revise enumerated identities, prove inverse AST parity, and execute in this real new module namespace.
    frozen_v6_task.require(frozen_v6_task.sha(module.__file__) == expected_sha, "frozen V6 provider source changed")
    original = ast.parse(Path(module.__file__).read_bytes()); revised = deepcopy(original)
    changes=[];counts={k:0 for k in literals};imports=imports or {};import_counts={k:0 for k in imports}
    for node in ast.walk(revised):
        if isinstance(node,ast.Constant) and type(node.value) is str and node.value in literals:
            old=node.value;changes.append((node,"value",old));node.value=literals[old];counts[old]+=1
        if isinstance(node,ast.alias) and node.name in imports:
            old=node.name;changes.append((node,"name",old));node.name=imports[old];import_counts[old]+=1
    frozen_v6_task.require(all(counts.values()) and all(import_counts.values()), "every exact V7 identity replacement must exist")
    undo=None
    if intervention is not None:undo=intervention(revised)
    proof=deepcopy(revised)
    if undo is not None:undo()
    for node,field,value in reversed(changes):setattr(node,field,value)
    frozen_v6_task.require(ast.dump(revised,include_attributes=False)==ast.dump(original,include_attributes=False),
        "an unrelated frozen provider statement changed")
    guards=[node for node in proof.body if isinstance(node,ast.If)
        and ast.dump(node.test,include_attributes=False)==ast.dump(ast.parse('__name__ == "__main__"',mode="eval").body,include_attributes=False)]
    frozen_v6_task.require(len(guards)<=1,"at most one frozen public entry guard expected")
    for node in guards:proof.body.remove(node)
    description=namespace.get("__doc__")
    exec(compile(ast.fix_missing_locations(proof),str(namespace["__file__"])+":authenticated_V6_provider","exec"),namespace)
    namespace["__doc__"]=description
    namespace["V7_literal_replacement_counts"]=counts
    namespace["V7_import_replacement_counts"]=import_counts


clone_frozen_provider(frozen_v6_task,V6_SOURCES["tools/e2e/town07_natural_goal_stop_brakefree_v6_20261002.py"],globals(),literals={
    "carla.actual_natural_goal_stop_teacher.20261002.v6":"carla.actual_natural_goal_stop_teacher.20261002.v7",
    "natural_goal_stop_manual1_1p50_coast11p5_hold0p05_brakefree.20261002.v6":"natural_goal_stop_manual1_1p50_coast11p5_hold0p05_brakefree.20261002.v7",
    "v6":"v7",
    "one fresh explicitly named brakefree v6 pilot only":"one fresh explicitly named brakefree v7 pilot only",
    "storage/experiments/validation/2026-10-02/town07_natural_goal_stop_teacher_brakefree_v6":"storage/experiments/validation/2026-10-02/town07_natural_goal_stop_teacher_brakefree_v7",
})
FROZEN=dict(FROZEN,**V6_SOURCES)
_v6_clone_verify, _v6_clone_fixed, _v6_clone_mission = verify_frozen, fixed_contract, mission_contract
STRAIGHT_TERMINAL_SPAN_M, MAX_TERMINAL_CROSS_TRACK_M, MAX_TERMINAL_HEADING_RAD = 12.5, .001, .001


def straight_terminal_geometry(route):
    # HH_261002 - This diagnostic precondition applies only where normal terminal steering will change, never claims whole-route collinearity.
    points=[(float(p["x"]),float(p["y"])) for p in route["route"]]
    require(len(points)==213 and all(type(p[k]) in (int,float) and math.isfinite(p[k]) for p in route["route"] for k in ("x","y")),
        "all original213 finite non-Boolean source XY points required")
    arc=[0.]
    for a,b in zip(points,points[1:]):arc.append(arc[-1]+math.dist(a,b))
    require(arc[-1]>STRAIGHT_TERMINAL_SPAN_M,"real terminal source span required")
    start=arc[-1]-STRAIGHT_TERMINAL_SPAN_M
    index=next(i for i in range(len(arc)-1) if arc[i]<=start<arc[i+1])
    ratio=(start-arc[index])/(arc[index+1]-arc[index])
    terminal=[(points[index][0]+ratio*(points[index+1][0]-points[index][0]),
        points[index][1]+ratio*(points[index+1][1]-points[index][1]))]+points[index+1:]
    dx,dy=terminal[-1][0]-terminal[0][0],terminal[-1][1]-terminal[0][1];chord=math.hypot(dx,dy)
    require(chord>0.,"actual terminal direction required")
    forward=dx/chord,dy/chord
    deviation=max(abs((x-terminal[0][0])*forward[1]-(y-terminal[0][1])*forward[0]) for x,y in terminal)
    turns=[]
    for a,b in zip(terminal,terminal[1:]):
        if a==b:continue
        sx,sy=b[0]-a[0],b[1]-a[1]
        turns.append(abs(math.atan2(sx*forward[1]-sy*forward[0],sx*forward[0]+sy*forward[1])))
    require(turns and deviation<=MAX_TERMINAL_CROSS_TRACK_M and max(turns)<=MAX_TERMINAL_HEADING_RAD,
        "terminal normal-steering intervention requires source12.5m straight to1mm and1e-3rad")
    return dict(source_point_count=213,assessed_terminal_span_m=STRAIGHT_TERMINAL_SPAN_M,
        maximum_terminal_perpendicular_error_m=deviation,maximum_terminal_segment_heading_error_rad=max(turns),
        maximum_allowed_perpendicular_error_m=MAX_TERMINAL_CROSS_TRACK_M,maximum_allowed_segment_heading_error_rad=MAX_TERMINAL_HEADING_RAD,
        original_route_never_rewritten=True,whole_route_collinearity_claim=False,
        no_future_states_labels_or_measured_vehicle_pose_used=True)


def straight_terminal_precondition():
    require(sha(ROUTE)==ROUTE_SHA,"original whole213-point route source changed")
    return dict(straight_terminal_geometry(read(ROUTE)),source_route_sha256=ROUTE_SHA)


def verify_frozen():
    _v6_clone_verify();straight_terminal_precondition()


def mission_contract():
    result=_v6_clone_mission()
    result.update(original_lateral_planner_and_downstream_BasicAgent_hazard_unchanged=False,
        original_GO_lateral_and_all_downstream_BasicAgent_hazards_unchanged=True,
        normal_COAST_and_HOLD_steering_zero=True,normal_COAST_HOLD_override_not_applied_to_hazard_outputs=True,
        terminal_only_straightness_precondition=straight_terminal_precondition(),
        zero_normal_steering_hypothesis_not_a_physics_or_future_geometry_repair_claim=True)
    return result


def fixed_contract(run_id,case_id):
    result=_v6_clone_fixed(run_id,case_id)
    result.update(teacher="Original GO steering and all BasicAgent hazards; NEW terminal-only normal COAST/HOLD steering zero hypothesis.",
        sole_physical_intervention="normal steer=0 only in causally latched BRAKE_TO_STOP/HOLD on the pinned source-straight final12.5m",
        straight_terminal_precondition=straight_terminal_precondition(),previous_failed_V6_source_retained=True)
    result["controller"].update(original_GO_steering_unchanged=True,normal_COAST_HOLD_steer=0.,
        downstream_hazard_steering_and_pedals_unchanged=True)
    return result
