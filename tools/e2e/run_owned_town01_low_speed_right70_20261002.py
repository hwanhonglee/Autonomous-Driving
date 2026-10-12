"""HH_261002 - One prospectively owned RIGHT70 teacher; unchanged full physics/ownership gates, no implicit capture or TRAIN authority."""
from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
from pathlib import Path

from scripts.e2e import run_owned_town01_low_speed_left45_20261002 as baseline
from scripts.e2e import town01_low_speed_right70_task_20261002 as declared_task
from scripts.e2e import collect_town01_low_speed_right70_20261002 as declared_worker
from scripts.e2e import audit_town01_right70_closing_control_20261002 as closing
from scripts.e2e import audit_carla_acknowledged_control as original_ack

PREVIOUS_SHA="87afc96b1da9e2ac71bf11e3b6a04583aaaea09c43a0a1e9d1af33f196c40eba"
CLOSING_SHA="7a496cff25a9b5129edbaf6c4348dfc279f454d5c0838e166c3a1cef918b7f7d"
_IMPORTS={"town01_low_speed_left45_task_20261002":"town01_low_speed_right70_task_20261002",
    "collect_town01_low_speed_left45_20261002":"collect_town01_low_speed_right70_20261002"}


def identity_ast():
    declared_task.require(declared_task.sha(Path(baseline.__file__))==PREVIOUS_SHA,"frozen LEFT owner changed")
    tree=ast.parse(Path(baseline.__file__).read_bytes())
    old=ast.Module(body=[n for n in tree.body if not isinstance(n,ast.If)],type_ignores=[])
    declared_task.require(len(tree.body)-len(old.body)==1,"unique original main guard required")
    new=copy.deepcopy(old);changes=[]
    for node in ast.walk(new):
        if isinstance(node,ast.keyword) and node.arg=="actual_left_turn_screen":
            changes.append((node,"arg",node.arg));node.arg="actual_right_turn_screen"
        if isinstance(node,ast.ImportFrom):
            for alias in node.names:
                if alias.name in _IMPORTS:changes.append((alias,"name",alias.name));alias.name=_IMPORTS[alias.name]
        if isinstance(node,ast.Constant) and type(node.value) is str:
            before=node.value
            if before=="tools/e2e/town01_train_left45_green_fixture_20261002.py":node.value=before
            elif before==str(baseline.NOMINAL.relative_to(baseline.ROOT)):node.value=str(declared_task.container.SUMMARY.relative_to(baseline.ROOT))
            elif before==baseline.ASSETS[baseline.NOMINAL]:node.value=declared_task.container.SUMMARY_SHA
            else:node.value=before.replace("LEFT45","RIGHT70").replace("LEFT","RIGHT").replace("left45","right70").replace("plus900","plus1400")
            if before=="60":node.value="90"
            if before!=node.value:changes.append((node,"value",before))
        elif isinstance(node,ast.Constant) and type(node.value) is int and node.value in (900,901,924):
            before=node.value;node.value={900:1400,901:1401,924:1424}[before];changes.append((node,"value",before))
    inverse=copy.deepcopy(new);new_nodes=list(ast.walk(new));inverse_nodes=list(ast.walk(inverse))
    for node,key,before in changes:setattr(inverse_nodes[new_nodes.index(node)],key,before)
    declared_task.require(ast.dump(inverse)==ast.dump(old),"RIGHT70 changed unrelated owner/control/quality math")
    return new


exec(compile(ast.fix_missing_locations(identity_ast()),__file__,"exec"),globals())
require(task is declared_task and worker_module is declared_worker,"actual RIGHT task/worker must be in the static closure")
_old_sources,_old_activation,_old_execution=runtime_sources,activated,execution_contract
ASSETS.update({task.container.SOURCE:task.container.SOURCE_SHA,
    task.container.BASE/"route_catalog.json":"4dda28989350aec739369ad60a4e10f0939aed4ef5f2db8d07042099312726bb",
    task.container.BASE/"all_nominal_bodies_and_sweeps.jsonl.gz":"7a4b7611867b66f2fe2d15ea4fce43fe064da402a055ca9a8bcacd7ba9102857"})


def nominal_witness():
    require(digest(NOMINAL)==task.container.SUMMARY_SHA,"exact completed whole nominal proof required")
    nominal=read_json(NOMINAL)
    require(nominal["status"]=="NOMINAL_CLEAR_NOT_DRIVING_PROOF" and nominal["generated_route_sha256"]==task.container.SOURCE_SHA
        and len(nominal["results"])==1,"one exact genuine TRAIN topology witness required")
    row=nominal["results"][0]
    require(row["case_id"]=="town01_right_centerline_6p1_108p1_24p1_20261002_v1"
        and row["nominal_sample_count"]==1941 and row["nominal_sweep_count"]==1940
        and all(not value["violation_sample_indices"] for value in row["geometry"].values()),"all original nominal body/sweep gates required")
    # HH_261002 - Seal/archive every original nominal input/source/API too; old32 and p00 failures are never reclassified.
    require(len(nominal["input_sha256"])==47 and len(nominal["source_sha256"])==29,"full nominal source/input census required")
    for name,sha in nominal["input_sha256"].items():
        path=Path(name);require(digest(path)==sha,"original nominal input/source/API changed")
        require(path not in ASSETS or ASSETS[path]==sha,"conflicting declared nominal asset")
        ASSETS[path]=sha
    return nominal


def runtime_sources():
    require(digest(closing.__file__)==CLOSING_SHA,"RIGHT closing reader changed");closing.verify_frozen()
    nominal_witness()
    pins=_old_sources()
    require(pins.get("tools/e2e/audit_town01_right70_closing_control_20261002.py")==CLOSING_SHA,
        "new receipt provider must be included in actual runtime/archive closure")
    return pins


def execution_contract(run_id,case_id):
    return dict(_old_execution(run_id,case_id),closing_receipt_reader_schema=closing.SCHEMA,
        original_LEFT_and_failed_RIGHT_candidates_not_requalified=True)


# HH_261002 - The frozen private build/verify functions resolve these final providers, not pre-overlay aliases.
_originals.update(runtime_sources=runtime_sources,execution_contract=execution_contract)


def turn_screen(states,route):
    points=route["route"]; ids=[i for i,p in enumerate(points) if p["road_option"]=="RIGHT"]
    require(bool(ids) and ids==list(range(ids[0],ids[-1]+1)) and 0<ids[0]<=ids[-1]<len(points)-1,
        "one complete original RIGHT block required")
    planned=math.degrees(math.atan2(math.sin(points[ids[-1]+1]["yaw"]-points[ids[0]-1]["yaw"]),
        math.cos(points[ids[-1]+1]["yaw"]-points[ids[0]-1]["yaw"])))
    require(planned<=-60.,"authentic planned RIGHT heading change required")
    exit_arc=sum(math.hypot(b["x"]-a["x"],b["y"]-a["y"]) for a,b in zip(points[:ids[-1]+1],points[1:ids[-1]+1]))
    driving=[s for s in states if s["capture_phase"]=="driving"]
    require(bool(driving),"actual turn requires all driving observations")
    rows=[s for s in driving if s["frame"]<=driving[0]["frame"]+task.LAST_FRAME]
    yaw=0.;previous=driving[0]["yaw"];entered=None;confirmed=None
    for row in rows:
        yaw+=math.atan2(math.sin(row["yaw"]-previous),math.cos(row["yaw"]-previous));previous=row["yaw"]
        progress=task.measured_progress(points,row["x"],row["y"])
        inside=math.degrees(yaw)<=-60. and abs(math.degrees(yaw)-planned)<=15. and progress>=exit_arc+10.
        if not inside:entered=None
        elif entered is None:entered=row
        elif confirmed is None and row["timestamp"]-entered["timestamp"]>=3.:
            confirmed=dict(entry_frame=entered["frame"],confirmation_frame=row["frame"],timestamp=row["timestamp"],
                signed_actual_yaw_change_deg=math.degrees(yaw),actual_measured_route_progress_m=progress,
                continuous_exit_plus10_and_heading_band_seconds=row["timestamp"]-entered["timestamp"])
    selected=[s for s in rows if s["frame"]>=driving[0]["frame"]+task.FIRST_FRAME and (s["frame"]-driving[0]["frame"])%2==0]
    length=sum(math.hypot(b["x"]-a["x"],b["y"]-a["y"]) for a,b in zip(points,points[1:]))
    support=bool(selected) and all(length-task.measured_progress(points,s["x"],s["y"])>=18. for s in selected)
    return dict(status="PASS" if confirmed is not None and support and len(selected)==task.ANCHORS else "FAIL",
        actual_exit_plus10_stable3s_witness=confirmed,planned_signed_yaw_change_deg=planned,planned_yaw_tolerance_deg=15.,
        minimum_actual_RIGHT_yaw_magnitude_deg=60.,original_encoded_exit_index=ids[-1],exact_route_XY_exit_arc_m=exit_arc,
        all_fixed_anchors_route_support_passed=support,minimum_causal_navigation_support_m=18.,selected_native_denominator=len(selected),
        not_traffic_rule_or_learned_turn_approval=True)


@contextmanager
def activated():
    require(lifecycle.independent_audit is original_ack,"unknown or nested original receipt reader")
    saved=lifecycle.independent_audit
    with _old_activation():
        try:lifecycle.independent_audit=closing.provider();yield
        finally:lifecycle.independent_audit=saved


def declare(run_id):
    batch,path=task.paths(run_id);output,case_path=task.paths(run_id,"pilot_00")
    require(not batch.exists() and not path.exists(),"one fresh prospective RIGHT attempt required")
    nominal_witness();require(digest(task.ROUTE)==task.ROUTE_SHA,"externally created capture container required")
    disk=disk_check();plan=build_plan(run_id,"pilot_00")
    batch.mkdir(parents=True,exist_ok=False);write_new(case_path,plan)
    receipt=dict(schema=SCHEMA,run_id=run_id,case_id="pilot_00",case_plan_sha256=digest(case_path),case_plan_path=str(case_path),
        attempts=1,automatic_retry=False,preflight_disk=disk,whole_nominal_summary_sha256=task.container.SUMMARY_SHA,
        original_generated_route_sha256=task.container.SOURCE_SHA,capture_container_sha256=task.ROUTE_SHA,
        nominal_clearance_not_actual_training_approval=True,training_data_approved=False,vehicle_control_approved=False,learned_control=False)
    write_new(path,receipt)
    return dict(receipt,batch_plan_path=str(path),batch_plan_sha256=digest(path),simulator_started=False)


if __name__=="__main__":raise SystemExit(main())
