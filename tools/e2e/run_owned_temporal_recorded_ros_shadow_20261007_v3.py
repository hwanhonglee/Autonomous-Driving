"""HH_261007 - Prepare or supervise one sealed, serial direct/DDS CPU recorded experiment.

Preparation writes a candidate only, never ROOT approval. Execution requires two
external SHA arguments. No CARLA, learning, CUDA, installation or actuators.
"""
from __future__ import annotations

import argparse
import ast
import fcntl
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
WORKER = Path(__file__).with_name("temporal_recorded_ros_shadow_worker_20261007_v3.py")


def bindings():
    spec = importlib.util.spec_from_file_location("_hh_owned_ros_protocol_261007", WORKER)
    module = importlib.util.module_from_spec(spec); sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def quaternion(matrix):
    # HH_261007 - Stable proper-rotation conversion; validate through the original runtime TF guard.
    m = [matrix[i:i+3] for i in (0,4,8)]; trace = sum(m[i][i] for i in range(3))
    if trace > 0:
        s = 2*math.sqrt(1+trace)
        q = [(m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s, (m[1][0]-m[0][1])/s, s/4]
    else:
        i = max(range(3), key=lambda j:m[j][j]); j, k = (i+1)%3, (i+2)%3
        s = 2*math.sqrt(1+m[i][i]-m[j][j]-m[k][k]); q = [0.,0.,0.,0.]
        q[i], q[j], q[k], q[3] = s/4, (m[j][i]+m[i][j])/s, (m[k][i]+m[i][k])/s, (m[k][j]-m[j][k])/s
    norm = math.sqrt(sum(v*v for v in q))
    return [v/norm for v in q]


def genuine_rig(manifest, worker):
    # HH_261007 - Compile exact original source exporter functions, without importing its dataset package.
    path = ROOT/"tools/e2e/prepare_carla_common10_dataset.py"
    functions = [node for node in ast.parse(path.read_bytes()).body if isinstance(node,ast.FunctionDef)
        and node.name in {"_number", "_require_close", "_matmul3", "_optical_transform", "_rig_document"}]
    worker.need(len(functions) == 5, "five genuine original rig functions required")
    pure = worker.frozen_bundle()
    common10 = SimpleNamespace(CAMERA_ORDER=pure.ROLES, RIG_SCHEMA_ID="autoware-e2e.common10.rig.v1")
    namespace = dict(math=math, hashlib=hashlib, json=json, common10=common10, AdapterError=ValueError)
    from collections.abc import Mapping
    namespace["Mapping"] = Mapping
    tree=ast.Module(body=[ast.ImportFrom(module="__future__",names=[ast.alias(name="annotations")],level=0),*functions],type_ignores=[])
    exec(compile(ast.fix_missing_locations(tree),str(path),"exec"),namespace)
    return namespace["_rig_document"](SimpleNamespace(manifest=manifest))


def model_contract(worker):
    training = ROOT/"storage/experiments/training"
    roots = [training/"2026-10-02/temporal_route_joint_v1/exports/step_006000",
        training/"2026-10-01/temporal_route_recovery_v3_v1/exports/step_003000",
        training/"2026-10-01/temporal_v2_balanced_replay_v1/exports/step_003000"]
    pins = ["b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625", "a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
        "0bdd747dffa6e37196e0ee6afb25fa9930d6aba6ca6f787c779ccfbeae3d4dc9", "4ace621d8cf1b8d3ce269bb77aabe9e89598f3f750e18ea707a0817086b3dc2d",
        "4e1623e5156b2a19b546f1889d7a70e9c7b312f72fbf7a0e5a99ccda7d8e5735", "88ffb8dcf9d21d0f756cad2c4ff478a74deee3b8e817f72a039a8885484bb4ec"]
    keys = ["weights", "metadata", "parent_weights", "parent_metadata", "backbone_parent_weights", "backbone_parent_metadata"]
    paths = [root/name for root in roots for name in ("model_state.npz", "metadata.json")]
    model = dict(zip(keys,map(str,paths))); model.update(weights_sha256=pins[0], metadata_sha256=pins[1])
    model["research_owner_binding"] = dict(schema="portable_e2e.temporal_route_joint_owner_binding.20261002.v1", mode="RESEARCH_ONLY", owner_id=worker.SCHEMA,
        purpose="offline_cpu_numeric_validation_only", simulator_control_authorized=False, vehicle_control_approved=False, model_promoted=False,
        fixed_endpoint_updates=6000, weights_sha256=pins[0], metadata_sha256=pins[1],
        checkpoint_sha256="43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88",
        declaration_sha256="0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5")
    return model, dict(zip(map(str,paths),pins))


def prepare(worker):
    worker.budget(startup=True)
    worker.need(not worker.PLAN_DIR.exists() and not worker.OUTPUT.exists(), "fresh preparation/output required; no retry overwrite")
    worker.PLAN_DIR.mkdir(parents=True)
    manifest = worker.bounded_json(worker.EPISODE/"manifest.json")
    cameras_path = worker.EPISODE/"camera_frames.jsonl"
    camera_rows = [worker.frozen_bundle().decode(line,256*1024) for line in cameras_path.read_bytes().splitlines()]
    worker.need(len(camera_rows) == 918, "original full918 census required")
    rig = genuine_rig(manifest,worker)
    calibration = tuple(tuple((camera["K"][0]/640, camera["K"][4]/360, camera["K"][2]/640, camera["K"][5]/360, *camera["T_base_from_camera"][:12])) for camera in rig["cameras"])
    identity = manifest["capture_contract"]["learned_research"]["stream_identity"]
    actual_id = hashlib.sha256(json.dumps(calibration,allow_nan=False).encode()).hexdigest()
    worker.need(identity["rig_id"] == actual_id and identity["split"] == "val", "original rig feature SHA/split differs")
    jpeg = {name:worker.digest(worker.EPISODE/name) for row in camera_rows for name in row["images"].values()}
    worker.need(len(jpeg) == 5508, "all original six×918 unique JPEG files required")
    route = worker.bounded_json(worker.EPISODE/"route.json")
    context = dict(schema=worker.frozen_bundle().SCHEMA+".context", episode_root=str(worker.EPISODE), episode_id=identity["episode_id"], split="val",
        rig_id=actual_id, stream_source_sha256=identity["source_sha256"], recorded_JPEG_only=True, camera_order=list(worker.frozen_bundle().ROLES), calibration=calibration,
        optical_frames={camera["name"]:camera["optical_frame"] for camera in rig["cameras"]}, jpeg_sha256=jpeg,
        route_xy_map_m=[(point["x"],point["y"]) for point in route["route"]], camera_transforms={camera["name"]:dict(translation=[camera["T_base_from_camera"][i] for i in (3,7,11)], quaternion=quaternion(camera["T_base_from_camera"])) for camera in rig["cameras"]})
    rig_path, context_path = worker.PLAN_DIR/"rig.json", worker.PLAN_DIR/"context.json"
    rig_sha, context_sha = worker.new_json(rig_path,rig), worker.new_json(context_path,context)
    sources = worker.source_pins()
    for name,pin in sources.items():
        archived = worker.PLAN_DIR/"sources"/name; archived.parent.mkdir(parents=True,exist_ok=True)
        raw = (ROOT/name).read_bytes(); worker.need(hashlib.sha256(raw).hexdigest()==pin,"source changed during archive")
        with archived.open("xb") as stream: stream.write(raw)
    model, input_pins = model_contract(worker)
    for path,pin in input_pins.items(): worker.need(worker.digest(path)==pin,"exact GOOD/parent numeric asset differs")
    scalar = {name:dict(path=str(path),sha256=worker.digest(path)) for name,path in dict(context=context_path,cameras=cameras_path,states=worker.EPISODE/"states.jsonl",callbacks=worker.EPISODE/"callback_timing_receipts.json").items()}
    input_pins.update({str(worker.EPISODE/name):pin for name,pin in jpeg.items()})
    input_pins.update({item["path"]:item["sha256"] for item in scalar.values()})
    contract_path = ROOT/"learning/portable_e2e/config/common_10hz_v1.contract.json"
    input_pins.update({str(rig_path):rig_sha, str(contract_path):worker.digest(contract_path), str(worker.EPISODE/"manifest.json"):worker.digest(worker.EPISODE/"manifest.json"), str(worker.EPISODE/"route.json"):worker.digest(worker.EPISODE/"route.json")})
    setup = {str(path):worker.digest(path) for path in (Path("/opt/ros/humble/setup.bash"),ROOT/"autonomy/install/local_setup.bash")}
    generated={}
    for package in ("autoware_planning_msgs","autoware_vehicle_msgs"):
        folder=ROOT/"autonomy/build"/package/"rosidl_generator_py"/package
        for path in sorted(folder.rglob("*")):
            if path.is_file() and path.suffix in (".py",".so"):
                actual=path.resolve(strict=True);worker.need(actual.is_relative_to(ROOT/"autonomy/build"/package),"generated message source escaped canonical build")
                generated[str(actual)]=worker.digest(actual)
        worker.need(any(Path(path).name=="__init__.py" and Path(path).parent==folder for path in generated),"canonical generated message package absent")
    environment=ROOT/"storage/experiments/diagnostics/2026-10-07/nontraining_cpu_ros_environment_v1/result.json"
    environment_pin=worker.digest(environment);environment_value=worker.bounded_json(environment)
    worker.need(environment_value["message_checks_passed"]==10 and environment_value["CUDA_initialized"] is False,"authentic clean environment scalar receipt required")
    input_pins[str(environment)]=environment_pin
    # HH_261007 - Controller geometry is authentic captured physics; no new vehicle-control request is issued.
    control_path=worker.EPISODE/"learned_control.jsonl"
    with control_path.open("rb") as stream:
        controller=next(value["controller"] for value in (worker.frozen_bundle().decode(line,256*1024) for line in stream) if type(value.get("controller")) is dict)
    input_pins[str(control_path)]=worker.digest(control_path)
    plan = dict(schema=worker.SCHEMA,fixed=worker.fixed_contract(),source_sha256=sources,input_sha256=input_pins,setup_sha256=setup,generated_ROS_sources=generated,
        expected_environment_origins=environment_value["origins"],expected_environment_versions=environment_value["versions"],
        scalar_inputs=scalar,model=model,rig=dict(path=str(rig_path),sha256=rig_sha),contract=dict(path=str(contract_path),sha256=input_pins[str(contract_path)]),controller=controller,
        replay_id="HH_261007-whole918-recorded-direct-DDS-v3",ROS_DOMAIN_ID=93,ROS_LOCALHOST_ONLY="1",RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
        TF_limits=dict(translation_m=.005,rotation_rad=.005),original_raw_run_status="FAIL_RETAINED_KNOWN_DEV_NOT_TRAIN",original_callback_identity_reaudited_by_RecordedIndex=True,
        recorded_JPEG_only=True,original_BGRA_parity_claimed=False,P3_approved=False)
    worker.frozen_bundle().load_index(plan)
    pin = worker.new_json(worker.PLAN_DIR/"candidate_plan.json",plan,worker.PLAN_LIMIT)
    return dict(status="PREPARED_NOT_AUTHORIZED",candidate_plan_sha256=pin,source_count=len(sources),input_count=len(input_pins),expected_anchors=918,
        ROOT_plan_required=str(worker.PLAN),ROOT_authority_required=str(worker.AUTHORITY),training_running=False)


def admit_process(process, argv, worker):
    identity = None; deadline = time.monotonic()+5
    while time.monotonic() < deadline:
        try:
            actual = worker.process_identity(process.pid)
            worker.need(actual["uid"] == os.getuid() and actual["ppid"] == os.getpid(),"owned child PPID/UID differs")
            if identity is None: identity = actual
            worker.need(actual["starttime"] == identity["starttime"],"child PID reused during exec")
            if actual["argv"] == argv and actual["pgid"] == actual["session"] == process.pid: return actual
        except FileNotFoundError:
            if process.poll() is not None: break
        time.sleep(.01)
    raise ValueError("owned child exec/session admission failed")


def stop_owned(process, identity, worker):
    if process.poll() is not None: return
    if identity is None:
        # HH_261007 - Popen owns this unreaped direct child; verify its core generation before signalling even on pre-exec failure.
        identity = worker.process_identity(process.pid)
        worker.need(identity["ppid"] == os.getpid() and identity["uid"] == os.getuid(),"unadmitted child is not ours")
    worker.same_generation(identity)
    # HH_261007 - Signal only this exact child, never a foreign process/group. Its role spawns no subprocesses.
    process.terminate()
    try: process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        worker.same_generation(identity); process.kill(); process.wait(timeout=5)


def near(a,b):
    if type(a) is dict and type(b) is dict: return set(a)==set(b) and all(near(a[k],b[k]) for k in a)
    if type(a) is list and type(b) is list: return len(a)==len(b) and all(near(x,y) for x,y in zip(a,b))
    return type(a) in (float,int) and type(b) in (float,int) and math.isfinite(a) and math.isfinite(b) and abs(a-b)<=1e-6+1e-6*abs(a)


def compare(worker):
    rows = {}
    for mode in ("direct","subscriber","publisher"):
        path = worker.OUTPUT/(mode+"_anchors.jsonl")
        values = [worker.frozen_bundle().decode(line,512*1024) for line in path.read_bytes().splitlines()] if path.exists() else []
        worker.need(len(values)<=918 and [row["ordinal"] for row in values]==list(range(len(values))),"ordered full failure-prefix required")
        rows[mode] = values
    counts = dict(input_CDR_equal=0,DDS_published_received_CDR_equal=0,tensor_both_available=0,tensor_equal=0,NN_both_executed=0,
        raw_comparable=0,raw_equal=0,trajectory_comparable=0,trajectory_equal=0,trajectory_timely=0,actual_full_publication_timely=0,status_equal=0,NN_presence_equal=0,tensor_presence_equal=0,NONCOMPARABLE=0,UNEXECUTED=0)
    for ordinal in range(918):
        if any(ordinal>=len(rows[mode]) for mode in rows):
            counts["UNEXECUTED"]+=1; result=dict(ordinal=ordinal,status="UNEXECUTED",original_declared_denominator=918)
        else:
            direct,sub,pub = (rows[mode][ordinal] for mode in ("direct","subscriber","publisher"))
            worker.need(all(row["native_frame"]==direct["native_frame"] and row["timestamp_ns"]==direct["timestamp_ns"] for row in (sub,pub)),"actual phase ordinal/frame/stamp differs")
            incoming=sub["actual_received_input_CDR_sha256"]; sent=pub["published_input_CDR_sha256"]
            input_equal=all(incoming.get(k)==v for k,v in direct["input_CDR_sha256"].items())
            DDS_equal=incoming==sent
            da=direct["policy_diagnostics"].get("owned_recorded_numerical_event",{}); sa=sub.get("policy_diagnostics",{}).get("owned_recorded_numerical_event",{})
            tensor_both=bool(da.get("tensor_sha256")) and bool(sa.get("tensor_sha256"))
            tensor_equal=tensor_both and da.get("tensor_sha256")==sa.get("tensor_sha256")
            NN_both=da.get("NN_executed") is True and sa.get("NN_executed") is True
            comparable=tensor_equal and da.get("NN_executed") is True and sa.get("NN_executed") is True
            raw_equal=comparable and near(da["raw_output"],sa["raw_output"])
            trajectory_comparable=comparable and direct["status"]==sub["status"]=="TRAJECTORY"
            trajectory_equal=trajectory_comparable and direct["converted_output_CDR_sha256"]=={k:v["sha256"] for k,v in pub["actual_DDS_outputs"].items()}
            timely=sub.get("actual_full_publication_deadline_passed") is True
            calls=sub.get("actual_output_publication_calls",[])
            trajectory_timely=trajectory_comparable and timely and len(calls)==2 and all(call.get("actually_published") is True and call.get("entry_deadline_passed") is True and call.get("completion_deadline_passed") is True for call in calls)
            for key,value in dict(input_CDR_equal=input_equal,DDS_published_received_CDR_equal=DDS_equal,tensor_both_available=tensor_both,tensor_equal=tensor_equal,NN_both_executed=NN_both,
                raw_comparable=comparable,raw_equal=raw_equal,trajectory_comparable=trajectory_comparable,trajectory_equal=trajectory_equal,status_equal=direct["status"]==sub["status"],NONCOMPARABLE=not comparable).items(): counts[key]+=int(value)
            counts["NN_presence_equal"]+=int(da.get("NN_executed")==sa.get("NN_executed"))
            counts["tensor_presence_equal"]+=int(bool(da.get("tensor_sha256"))==bool(sa.get("tensor_sha256")))
            counts["actual_full_publication_timely"]+=int(timely);counts["trajectory_timely"]+=int(trajectory_timely)
            row_status="FAIL_OR_NONCOMPARABLE_RETAINED"
            if input_equal and DDS_equal and raw_equal:
                if direct["status"]!=sub["status"]:row_status="RAW_ONLY_PASS_OUTPUT_STATUS_DIFFERS"
                elif trajectory_comparable and trajectory_equal and trajectory_timely:row_status="COMPARABLE_VALID_TRAJECTORY_PASS"
                elif not trajectory_comparable:row_status="RAW_ONLY_PASS_NO_VALID_TRAJECTORY"
            result=dict(ordinal=ordinal,native_frame=direct["native_frame"],timestamp_ns=direct["timestamp_ns"],input_CDR_equal=input_equal,DDS_input_CDR_equal=DDS_equal,
                tensor_equal=tensor_equal,raw_comparable=comparable,raw_equal=bool(raw_equal),trajectory_comparable=bool(trajectory_comparable),trajectory_equal=bool(trajectory_equal),
                actual_full_publication_timely=timely,trajectory_timely=bool(trajectory_timely),direct_status=direct["status"],DDS_status=sub["status"],status=row_status)
        worker.append(worker.OUTPUT/"comparison.jsonl",result)
    passed=counts["input_CDR_equal"]==counts["DDS_published_received_CDR_equal"]==counts["status_equal"]==counts["NN_presence_equal"]==counts["tensor_presence_equal"]==918 and 0<counts["tensor_both_available"]==counts["tensor_equal"] \
        and 0<counts["NN_both_executed"]==counts["raw_comparable"]==counts["raw_equal"] and 0<counts["trajectory_comparable"]==counts["trajectory_equal"]==counts["trajectory_timely"]
    return dict(expected=918,counts=counts,status="PASS_RECORDED_DIRECT_DDS_PARITY_ONLY" if passed else "FAIL_OR_NONCOMPARABLE_RETAINED",P3_approved=False,formal_main_count=0,formal_sub_count=0,
        actual_comparable_denominators_not_all918_NN_assertion=True,original_V5_terminal_equivalence=False,original_BGRA_equivalence=False,wall_10Hz_proven=False)


def execute(worker, plan_sha, authority_sha):
    view = worker.authorize(plan_sha,authority_sha); worker.budget(startup=True)
    owner_argv=["/usr/bin/python3","-B",str(Path(__file__)),"--mode","execute","--plan-sha256",plan_sha,"--authority-sha256",authority_sha]
    worker.need(worker.process_identity(os.getpid())["argv"]==owner_argv,"ROOT must invoke exact declared owner argv; no interpreter substitution")
    worker.need(not worker.OUTPUT.exists(),"fresh actual output required")
    lock = worker.LOCK.open("a+"); fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    worker.OUTPUT.mkdir(parents=True)
    identities=[]; processes=[]; launch_times={}; error=None; cleanup=[]; initial=time.monotonic()
    lock_stat=os.fstat(lock.fileno())
    worker.new_json(worker.OUTPUT/"owner_started.json",dict(identity=worker.process_identity(os.getpid()),plan_sha256=plan_sha,authority_sha256=authority_sha,
        lock_fd=lock.fileno(),lock_identity=dict(dev=lock_stat.st_dev,ino=lock_stat.st_ino),recorded_CPU_only=True,training=False,CARLA=False,actuation=False))
    def launch(mode):
        worker.budget(); argv=worker.worker_command(mode,plan_sha,authority_sha)
        with (worker.OUTPUT/(mode+".stdout")).open("xb") as stdout,(worker.OUTPUT/(mode+".stderr")).open("xb") as stderr:
            environment=dict(os.environ,HH_OWNED_RUNTIME_LOCK_FD=str(lock.fileno()))
            parent_pid=os.getpid()
            def death_guard():
                import ctypes
                if ctypes.CDLL(None,use_errno=True).prctl(1,signal.SIGTERM,0,0,0)!=0 or os.getppid()!=parent_pid:os._exit(125)
            process=subprocess.Popen(argv,stdout=stdout,stderr=stderr,start_new_session=True,env=environment,pass_fds=(lock.fileno(),),preexec_fn=death_guard)
            processes.append(process); identities.append(None)
            launch_times[process.pid]=time.monotonic()
            identity=admit_process(process,argv,worker); identities[-1]=identity
            worker.new_json(worker.OUTPUT/(mode+"_owner_child.json"),dict(identity=identity,argv=argv,mode=mode))
        return process
    def wait(process, mode):
        start=launch_times[process.pid]
        while process.poll() is None:
            worker.budget(); worker.need(time.monotonic()-start<=900 and time.monotonic()-initial<=1800,"prospective role/whole wall budget exceeded")
            time.sleep(.05)
        worker.need(process.returncode==0,mode+" exited nonzero; original complete logs retained")
    def interruption(signum,frame): raise KeyboardInterrupt("owned supervisor interrupted: "+str(signum))
    previous_handlers={number:signal.signal(number,interruption) for number in (signal.SIGTERM,signal.SIGHUP)}
    try:
        direct=launch("direct"); wait(direct,"direct")
        subscriber=launch("subscriber"); until=time.monotonic()+60
        while not (worker.OUTPUT/"subscriber_ready.json").exists():
            worker.budget(); worker.need(subscriber.poll() is None and time.monotonic()<until,"subscriber readiness failed/expired")
            time.sleep(.05)
        publisher=launch("publisher"); wait(publisher,"publisher"); wait(subscriber,"subscriber")
    except BaseException as failure: error=repr(failure)
    finally:
        for process,identity in zip(processes,identities):
            try: stop_owned(process,identity,worker)
            except BaseException as failure: cleanup.append(repr(failure))
        source_after={};inputs_after={};comparison=None
        try:
            source_after={name:worker.digest(ROOT/name) for name in view["source_sha256"]}
            inputs_after={name:worker.digest(name) for name in view["input_sha256"]}
            for name,pin in view["generated_ROS_sources"].items():worker.need(worker.digest(name)==pin,"generated ROS post-seal failure")
            for name,pin in view["setup_sha256"].items():worker.need(worker.digest(name)==pin,"canonical setup post-seal failure")
            for name,pin in view["source_sha256"].items(): worker.need(worker.digest(worker.PLAN_DIR/"sources"/name)==pin,"archived source post-seal failure")
            worker.need(source_after==view["source_sha256"] and inputs_after==view["input_sha256"],"actual source/input post-seal failure")
        except BaseException as failure: cleanup.append("post-seal: "+repr(failure))
        try: comparison=compare(worker)
        except BaseException as failure: cleanup.append("comparison: "+repr(failure))
        try:
            worker.new_json(worker.OUTPUT/"result.json",dict(schema=worker.SCHEMA+".owner_result",status="COMPLETE_RECORDED_ATTEMPT_NOT_FEATURE_APPROVAL" if error is None and not cleanup else "FAILED_RETAINED",
                error=error,cleanup_errors=cleanup,children=[dict(identity=i,exit=p.returncode) for p,i in zip(processes,identities)],
                source_sha256_after=source_after,input_sha256_after=inputs_after,comparison=comparison,wall_seconds=time.monotonic()-initial,
                forced_owner_SIGKILL_or_host_loss_supervision_unknown=True),worker.PLAN_LIMIT)
        finally:
            fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
            for number,handler in previous_handlers.items():signal.signal(number,handler)
    return 0 if error is None and not cleanup else 1


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);parser.add_argument("--mode",choices=("prepare","execute"),required=True)
    parser.add_argument("--plan-sha256");parser.add_argument("--authority-sha256");args=parser.parse_args(argv);worker=bindings()
    if args.mode=="prepare": print(json.dumps(prepare(worker),sort_keys=True));return 0
    worker.need(args.plan_sha256 and args.authority_sha256,"external ROOT raw plan/authority SHA required")
    return execute(worker,args.plan_sha256,args.authority_sha256)


if __name__=="__main__":raise SystemExit(main())
