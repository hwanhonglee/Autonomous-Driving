"""HH_261002 - Own one new brakefree full-route teacher, retaining bootstrap, tails, failures, sources and every original lifecycle guard."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import copy
from pathlib import Path
from types import SimpleNamespace
import sys

from scripts.e2e import run_owned_town07_natural_goal_stop_v2_20261002 as baseline
# HH_261002 - Include all declared frozen v3 bytes in the imported source closure; no original marker is removed.
from scripts.e2e import run_owned_town07_natural_goal_stop_brakefree_v3_20261002 as frozen_v3_owner
# HH_261002 - Archive the quota-failed v4 source unchanged in the new import-only closure.
from scripts.e2e import run_owned_town07_natural_goal_stop_brakefree_v4_20261002 as frozen_v4_owner
from scripts.e2e import collect_town07_natural_goal_stop_brakefree_v5_20261002 as worker_module
from scripts.e2e import town07_natural_goal_stop_brakefree_v5_20261002 as task

previous, ROOT, lifecycle = baseline.previous, baseline.ROOT, baseline.lifecycle
require,digest,read_json,write_new,jsonl = baseline.require,baseline.digest,baseline.read_json,baseline.write_new,baseline.jsonl
MODULE = "scripts.e2e.run_owned_town07_natural_goal_stop_brakefree_v5_20261002"
SCHEMA, CASE_IDS = task.SCHEMA+".owned_pilot",("pilot_00",)
ASSETS = dict(baseline.ASSETS,**{str(task.EVIDENCE_ROOT/k):v for k,v in task.COAST_EVIDENCE.items()})
# HH_261002 - Keep Path keys for inherited immutable asset/route comparisons.
ASSETS = {Path(k):v for k,v in ASSETS.items()}
_active = False


def capture_argv(run_id,case_id,plan_sha=lifecycle.PLAN_TOKEN):
    output,plan = task.paths(run_id,case_id)
    return ["--natural-goal-run-id",run_id,str(output/"episode"),str(task.ROUTE),"--host","127.0.0.1","--port","2440",
        "--control-transport","acknowledged_batch","--agent-initialization","after_bootstrap",
        "--stationary-warmup-sec","1.2","--stationary-tail-sec","6.5","--max-duration-sec","200",
        "--target-speed-kmh","5.4","--seed",str(task.SEED),"--weather","ClearNoon",
        "--mapping",str(previous.MAPPING),"--calibration",str(previous.CALIBRATION),"--spawn-z-offset-m",".5",
        "--wheelbase-m","2.85","--expected-live-xodr-sha256",ASSETS[previous.XODR],"--goal-stop-profile","comfortable_v1",
        "--goal-tolerance-m","1","--basic-agent-base-min-distance-m","2","--basic-agent-distance-ratio",".2",
        "--moving-task-plan",str(plan),"--moving-task-plan-sha256",plan_sha]


def execution_contract(run_id,case_id):
    output,_ = task.paths(run_id,case_id)
    return dict(owner_output=str(output),server_command=["bash","tools/e2e/run_carla_map.sh","Town07","--port","2440",
        "--quality","Epic","--startup-timeout-sec","120","--","-RenderOffScreen","-nosound"],
        worker_command=[sys.executable,"-B",str(ROOT/"tools/workspace_python.py"),"-m",MODULE,"--","--mode","worker",
            "--run-id",run_id,"--case-id",case_id,"--case-plan-sha256",lifecycle.PLAN_TOKEN],owner_wall_timeout_s=600.)


def runtime_sources():
    task.verify_frozen(); worker_module.manual.verify_frozen_dependencies()
    pins = baseline.runtime_sources()
    for path in lifecycle.imported_source_closure((Path(__file__).resolve(),)):
        pins[str(path.relative_to(ROOT))] = digest(path)
    require(all(pins.get(k)==v for k,v in lifecycle.FROZEN_GENERIC.items()),"frozen private ownership machinery changed")
    require(all(pins.get(k)==v for k,v in task.FROZEN.items()),"frozen teacher/coast closure changed")
    return dict(sorted(pins.items()))


def disk_check(free=None):
    # HH_261002 - Reuse the exact original floor/reserve check in the explicit NEW quota-provider namespace.
    from types import FunctionType
    original = baseline.disk_check
    revised = FunctionType(original.__code__,dict(original.__globals__,task=task),original.__name__,original.__defaults__)
    require(revised.__code__ is original.__code__,"original disk floor/reserve guard bytecode must remain identical")
    return revised(free)
compare_membership = baseline.compare_membership


def capture_passed(manifest,owner,checks,tail):
    return baseline.capture_passed(manifest,owner,checks,tail) and checks.get("new_causal_mission") is True and checks.get("full_native_original_bounds") is True


def body_with_bootstrap(output,plan,episode,states,receipts,manifest):
    # HH_261002 - The bootstrap is a real pinned manifest observation, not a synthesized/camera/TRAIN sample.
    bootstrap = dict(manifest["capture_contract"]["control_transport"]["bootstrap_observation"],capture_phase="pre_capture_bootstrap")
    report = previous.body_report(output,plan,episode,[bootstrap]+states,receipts,manifest)
    report.update(recorded_native_count=len(states),independently_retained_bootstrap_count=1,
        bootstrap_is_not_TRAIN_or_camera=True,full_bootstrap_warmup_driving_HOLD_tail_assessed=True)
    return report


def mission_audit(episode,manifest,states,receipts,route):
    names = ("actual_teacher_control.jsonl","actual_goal_governor.jsonl","actual_goal_target_assignments.jsonl","actual_hazard_overrides.jsonl")
    pins = {str(episode/n):digest(episode/n) for n in names}
    result = worker_module.audit_mission(manifest,states,receipts,*(jsonl(episode/n) for n in names[:3]),route,jsonl(episode/names[3]))
    require(all(digest(p)==v for p,v in pins.items()),"original mission journals changed during independent audit")
    result["input_sha256"] = pins
    return result


def revised_diagnostic():
    """HH_261002 - Change only exact metadata identity and append two stronger conjuncts; preserve every failed-source diagnostic statement."""
    task.verify_frozen()
    tree = ast.parse(Path(baseline.__file__).read_bytes())
    node = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="diagnostic_report")
    original = copy.deepcopy(node)
    identity = next(n for n in ast.walk(node) if isinstance(n,ast.FunctionDef) and n.name=="identity")
    old_identity = copy.deepcopy(identity.body)
    identity.body = ast.parse('''require(capture["natural_goal_stop_teacher"]==plan and capture["goal_stop_profile"]["profile_id"]==task.PROFILE,"exact fresh brakefree plan/profile required")
metadata = capture["teacher_controller"]
require(metadata["normal_brake_cap"]==0. and metadata["normal_throttle_cap"]==.20 and metadata["causal_brakefree_terminal_mission"]==task.mission_contract(),"actual zero-brake/service-HOLD protocol differs")
return True
''').body
    branch = next(n for n in node.body if isinstance(n,ast.If) and isinstance(n.test,ast.BoolOp))
    positions = [i for i,n in enumerate(branch.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=="passed" for t in n.targets)]
    require(len(positions)==1,"unique original final all-check conjunction required")
    added = ast.parse('''native_bounds = attempt("full_native_original_bounds",lambda:task.full_native_quality(manifest,states))
checks["full_native_original_bounds"] = native_bounds is not None and native_bounds.get("status")=="PASS_SCALAR_ONLY"
mission = attempt("new_causal_mission",lambda:mission_audit(episode,manifest,states,receipts,route))
checks["new_causal_mission"] = mission is not None and mission.get("status")=="PASS_CAUSAL_MISSION_AND_NORMAL_ACK_ONLY"
''').body
    index = positions[0]; branch.body[index:index] = added
    revised = copy.deepcopy(node)
    del branch.body[index:index+len(added)]; identity.body = old_identity
    require(ast.dump(node,include_attributes=False)==ast.dump(original,include_attributes=False),"every original quality/source/body/error/membership gate statement must remain AST-identical")
    proxy = SimpleNamespace(**dict(vars(previous),body_report=body_with_bootstrap))
    namespace = dict(vars(baseline),task=task,worker_module=worker_module,previous=proxy,SCHEMA=SCHEMA,
        capture_passed=capture_passed,mission_audit=mission_audit)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[revised],type_ignores=[])),str(__file__)+":same_diagnostics_plus_causal_mission","exec"),namespace)
    return namespace["diagnostic_report"]


def diagnostic_report(output,plan,owner):
    return revised_diagnostic()(output,plan,owner)


@contextmanager
def activated():
    global _active
    require(not _active and not previous._active and not baseline._active,"brakefree owner cannot nest or replace another owner")
    names = dict(task=task,worker_module=worker_module,MODULE=MODULE,SCHEMA=SCHEMA,CASE_IDS=CASE_IDS,ASSETS=ASSETS,
        capture_argv=capture_argv,execution_contract=execution_contract,runtime_sources=runtime_sources,
        disk_check=disk_check,diagnostic_report=diagnostic_report)
    saved = {k:getattr(previous,k) for k in names}; _active = True
    try:
        for k,v in names.items():setattr(previous,k,v)
        yield
    finally:
        for k,v in saved.items():setattr(previous,k,v)
        _active = False


def inherited_api(name):
    # HH_261002 - Frozen create-only declare/verify logic is executed in the new explicit provider namespace.
    original = getattr(baseline,name)
    tree = ast.parse(Path(baseline.__file__).read_bytes())
    node = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
    namespace = dict(vars(baseline),task=task,worker_module=worker_module,previous=previous,MODULE=MODULE,SCHEMA=SCHEMA,
        ASSETS=ASSETS,CASE_IDS=CASE_IDS,disk_check=disk_check)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(__file__)+":unchanged_"+name,"exec"),namespace)
    require(namespace[name].__code__.co_consts==original.__code__.co_consts,"frozen create-only API constants changed")
    return namespace[name]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","declare","preflight","execute","worker","summary"),default="contract")
    parser.add_argument("--run-id",choices=("v5",),default="v5")
    parser.add_argument("--case-id",choices=CASE_IDS,default="pilot_00")
    parser.add_argument("--plan-sha256");parser.add_argument("--case-plan-sha256")
    args = parser.parse_args(argv)
    with activated():
        if args.mode=="contract":result = task.fixed_contract(args.run_id,"pilot_00")
        elif args.mode=="declare":result = inherited_api("declare")(args.run_id)
        elif args.mode=="worker":return previous.worker(args.run_id,"pilot_00",args.case_plan_sha256)
        else:
            spec = inherited_api("verify_batch")(args.run_id,args.plan_sha256)
            if args.mode=="summary":
                output,_ = task.paths(args.run_id,"pilot_00"); path = output/"diagnostic_report.json"
                result = read_json(path) if path.is_file() else dict(status="NOT_EXECUTED",training_data_approved=False)
            else:
                with previous.scoped_case("pilot_00"):
                    if args.mode=="execute":
                        code = lifecycle.execute(args.run_id,spec["declaration_sha256"])
                        output,_ = task.paths(args.run_id,"pilot_00"); path = output/"diagnostic_report.json"
                        return 0 if code==0 and path.is_file() and read_json(path).get("actual_teacher_capture_passed") is True else 2
                    _,result = previous.preflight(args.run_id,spec["declaration_sha256"])
    import json
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False));return 0


if __name__=="__main__":raise SystemExit(main())
