"""HH_261007 - Preserve V1-3 failures; normalize only actual matched DDS endpoint GUIDs and retain raw RMW bytes.

No system writes, packages, CNN, training, GPU, CARLA or actuators. Real probe
payloads are explicitly synthetic; only actual DDS sender/capsule/CDR ABI is tested.
"""
from __future__ import annotations

import argparse
import ctypes
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import resource
import shlex
import signal
import stat
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
SELF=Path(__file__).resolve()
SOURCE=SELF.with_name("owned_dds_info_bridge_20261007_v2.c")
TEST=ROOT/"learning/tests/test_owned_dds_info_bridge_20261007_v4.py"
SCHEMA="portable_e2e.owned_native_DDS_info_bridge.20261007.v4"
OUT=ROOT/"storage/experiments/diagnostics/2026-10-07/owned_dds_info_bridge_v4"
PLAN_DIR=ROOT/"storage/experiments/plans/2026-10-07/owned_dds_info_bridge_v4"
PLAN=PLAN_DIR/"ROOT_plan.json"
LOCK=ROOT/"storage/assets/locks/autoware_e2e_runtime.lock"
MODULE="_hh_owned_dds_info_261007"
BINARY=OUT/"build"/(MODULE+".cpython-310-x86_64-linux-gnu.so")
CC=Path("/usr/bin/x86_64-linux-gnu-gcc-12")
INCLUDES=[Path("/usr/include/python3.10"),Path("/opt/ros/humble/include"),*(Path("/opt/ros/humble/include")/name for name in
    ("rcl","rmw","rcutils","rcl_yaml_param_parser","rosidl_runtime_c","rosidl_typesupport_interface"))]
LIBRARIES=[Path("/opt/ros/humble/lib")/name for name in ("librcl.so","librmw.so","librcutils.so")]
DDS_LIBRARY=Path("/opt/ros/humble/lib/x86_64-linux-gnu/libddsc.so.0.10.5")
LIBRARIES.append(DDS_LIBRARY)
TOOLS=[CC,Path("/usr/lib/gcc/x86_64-linux-gnu/12/cc1"),Path("/usr/lib/gcc/x86_64-linux-gnu/12/collect2"),Path("/usr/bin/x86_64-linux-gnu-as"),Path("/usr/bin/x86_64-linux-gnu-ld.bfd"),Path("/usr/bin/ldd")]
FLOOR,SHARED,QUOTA,CLOSING=2*1024**3,64*1024**2,64*1024**2,8*1024**2
NODES={("owned_dds_bridge_tx","/"),("owned_dds_bridge_rx","/")}
TOPICS={"/parameter_events":"rcl_interfaces/msg/ParameterEvent",
    "/portable_e2e/owned_native_info_probe/text":"std_msgs/msg/String",
    "/portable_e2e/owned_native_info_probe/bytes":"sensor_msgs/msg/CompressedImage"}
PROVENANCE="public_dds_matched_publication_data_for_actual_taken_handle"
PREDECESSOR_EVIDENCE=[ROOT/"storage/experiments/diagnostics/2026-10-07"/name/file for name,file in (
    ("owned_dds_info_bridge_v1","build_failure.json"),("owned_dds_info_bridge_v2","build_failure.json"),
    ("owned_dds_info_bridge_v3","build_result.json"),("owned_dds_info_bridge_v3","result.json"))]


def need(value,message):
    if not value:raise ValueError(message)


def encode(value):return (json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False)+"\n").encode()


def fingerprint(path):
    path=Path(path);actual=path.resolve(strict=True);before=actual.stat()
    need(stat.S_ISREG(before.st_mode),"regular source/header/library/tool required")
    digest=hashlib.sha256()
    with actual.open("rb") as stream:
        for block in iter(lambda:stream.read(1024**2),b""):digest.update(block)
    after=actual.stat()
    need(all(getattr(before,key)==getattr(after,key) for key in ("st_dev","st_ino","st_size","st_mtime_ns","st_ctime_ns")),"input generation changed")
    return dict(path=str(path),resolved_path=str(actual),sha256=digest.hexdigest(),bytes=before.st_size,dev=before.st_dev,ino=before.st_ino,mtime_ns=before.st_mtime_ns,ctime_ns=before.st_ctime_ns)


def json_read(path,pin=None):
    path=Path(path);need(path.resolve(strict=True)==path and path.stat().st_size<=4*1024**2,"bounded canonical metadata required")
    raw=path.read_bytes()
    if pin:need(hashlib.sha256(raw).hexdigest()==pin,"external raw metadata SHA differs")
    def unique(pairs):
        result={}
        for key,value in pairs:need(key not in result,"duplicate metadata key");result[key]=value
        return result
    return json.loads(raw,object_pairs_hook=unique,parse_constant=lambda value:need(False,"nonfinite metadata"))


def write_new(path,value):
    raw=encode(value);need(len(raw)<=4*1024**2,"bounded complete JSON required")
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("xb") as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
    return hashlib.sha256(raw).hexdigest()


def budget(startup=False):
    import shutil
    need(shutil.disk_usage(ROOT).free>=FLOOR+SHARED+(QUOTA if startup else CLOSING),"separate native CPU probe reserve denied")
    total=0
    for directory in (OUT,PLAN_DIR):
        if not directory.exists():continue
        for path in directory.rglob("*"):
            mode=path.lstat().st_mode;need(stat.S_ISREG(mode) or stat.S_ISDIR(mode),"special/symlink output denied")
            if stat.S_ISREG(mode):need(path.stat().st_size<=8*1024**2,"complete raw output file cap exceeded");total+=path.stat().st_size
    need(total<=QUOTA-CLOSING,"native output closing reserve exhausted")


def contract():
    return dict(module=MODULE,binary=str(BINARY),ROS_DOMAIN_ID=93,ROS_LOCALHOST_ONLY="1",RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
        ROS_LOG_DIR=str(OUT/"ros_logs"),payload_messages=6,payload_types=["std_msgs/msg/String","sensor_msgs/msg/CompressedImage"],
        no_NN=True,no_actuation=True,no_training=True,system_or_package_changes=False,publisher_gid_from_true_rmw_only=True,
        dependency_timeout_seconds=30,compiler_timeout_seconds=120,probe_timeout_seconds=60,receipt_timeout_seconds=5,
        command_lease_inherited=True,command_parent_death_signal="SIGTERM",clean_env_required=True,
        publisher_gid_normalization=dict(provenance=PROVENANCE,raw_RMW_gid_retained=True,actual_publication_handle_retained=True,
            public_DDS_only=True,private_RMW_data_access=False,graph_identity_substitution=False,
            exact_domain=93,exact_RCL_topic_rt_prefix=True,unique_reader_required=True,
            children_bound=256,entity_bound=512,depth_bound=8,reader_GUID_before_after=True,
            matched_endpoint_GUID_bytes=16,RMW_zero_padding_bytes=8,unresolved_or_ambiguous="DENY"),
        node_allowlist=[list(node) for node in sorted(NODES)],topic_allowlist=TOPICS,
        floor_bytes=FLOOR,shared_bytes=SHARED,quota_bytes=QUOTA,closing_bytes=CLOSING)


def source_paths():
    return [SOURCE,SELF,TEST,SELF.with_name("owned_dds_info_bridge_20261007_v1.c"),
        *(path for version in (1,2,3) for path in (SELF.with_name(f"probe_owned_dds_info_bridge_20261007_v{version}.py"),TEST.with_name(f"test_owned_dds_info_bridge_20261007_v{version}.py")))]


def source_pins():return {str(path.relative_to(ROOT)):fingerprint(path)["sha256"] for path in source_paths()}


def prepare():
    # HH_261007 - Validate every existing include root and correct the package spelling without installing headers.
    need(all(path.is_dir() and path.resolve(strict=True)==path for path in INCLUDES), "all declared existing include directories required")
    budget(True);need(sys.version_info[:2]==(3,10),"existing exact Python3.10 required")
    need(not (PLAN_DIR/"candidate_plan.json").exists() and not OUT.exists(),"fresh candidate/attempt required")
    for path in source_paths()+PREDECESSOR_EVIDENCE:need(path.resolve(strict=True)==path,"project source/evidence alias denied")
    toolchain={str(path):fingerprint(path) for path in TOOLS}
    libraries={str(path):fingerprint(path) for path in LIBRARIES}
    environment=ROOT/"storage/experiments/diagnostics/2026-10-07/nontraining_cpu_ros_environment_v1/result.json"
    plan=dict(schema=SCHEMA,fixed=contract(),source_sha256=source_pins(),toolchain=toolchain,libraries=libraries,
        preserved_evidence={str(path):fingerprint(path) for path in PREDECESSOR_EVIDENCE},
        environment_receipt=dict(path=str(environment),sha256=fingerprint(environment)["sha256"]),
        include_directories=list(map(str,INCLUDES)),compiler_flags=["-shared","-fPIC","-std=c11","-O2","-Wall","-Wextra","-Werror"],
        runtime_lock=str(LOCK),existing_native_binding=str(Path("/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so")))
    plan["native_binding"]=fingerprint(plan["existing_native_binding"])
    pin=write_new(PLAN_DIR/"candidate_plan.json",plan)
    return dict(status="SOURCE_PREPARED_NOT_AUTHORIZED",candidate_plan_sha256=pin)


def admit(phase,plan_sha,authority_sha):
    plan=json_read(PLAN,plan_sha);need(plan["schema"]==SCHEMA and plan["fixed"]==contract() and plan["source_sha256"]==source_pins(),"exact fresh source/contract required")
    grant=json_read(PLAN_DIR/("ROOT_"+phase+"_authority.json"),authority_sha)
    need(grant==dict(schema=SCHEMA+".ROOT_authority",phase=phase,plan_sha256=plan_sha,source_sha256=plan["source_sha256"],
        private_existing_RCL_only=True,training_authorized=False,GPU_authorized=False,CARLA_authorized=False,actuation_authorized=False,
        native_binary_sha256=None if phase=="build" else fingerprint(BINARY)["sha256"]),"separate external ROOT phase grant required")
    for mapping in (plan["toolchain"],plan["libraries"],plan["preserved_evidence"]):
        for path,value in mapping.items():need(fingerprint(path)==value,"frozen compiler/library generation differs")
    need(fingerprint(plan["existing_native_binding"])==plan["native_binding"],"original Python native binding changed")
    need(fingerprint(plan["environment_receipt"]["path"])["sha256"]==plan["environment_receipt"]["sha256"],"canonical environment receipt changed")
    need(sys.executable=="/usr/bin/python3" and sys.version_info[:2]==(3,10) and os.environ.get("CUDA_VISIBLE_DEVICES")=="" and os.environ.get("PYTHONNOUSERSITE")=="1","exact existing clean CPU interpreter required")
    for key in ("ROS_DOMAIN_ID","ROS_LOCALHOST_ONLY","RMW_IMPLEMENTATION","ROS_LOG_DIR"):need(os.environ.get(key)==str(contract()[key]),"exact owned ROS environment required")
    for key in ("LD_PRELOAD","LD_AUDIT","ROS_DISCOVERY_SERVER","CYCLONEDDS_URI","FASTRTPS_DEFAULT_PROFILES_FILE","RMW_FASTRTPS_USE_QOS_FROM_XML"):
        need(not os.environ.get(key),"undeclared loader/discovery override denied: "+key)
    need(not any(name=="torch" or name.startswith("torch.") or name=="carla" or name.startswith("carla.") for name in sys.modules),"Torch/CARLA must remain absent")
    return plan


def proc_identity(pid):
    root=Path("/proc")/str(pid);raw=(root/"stat").read_text();parts=raw[raw.rfind(")")+2:].split()
    return dict(pid=pid,starttime=int(parts[19]),ppid=int(parts[1]),pgid=int(parts[2]),session=int(parts[3]),uid=root.stat().st_uid)


def lease_guard(fd):
    actual=os.fstat(fd);named=LOCK.stat()
    need(stat.S_ISREG(actual.st_mode) and (actual.st_dev,actual.st_ino)==(named.st_dev,named.st_ino),"exact inherited runtime lease required")
    fcntl.fcntl(fd,fcntl.F_GETFD)


def session_members(identity):
    result=[]
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():continue
        try:current=proc_identity(int(entry.name))
        except (FileNotFoundError,ProcessLookupError):continue
        except PermissionError:raise ValueError("cannot establish owned compiler session census")
        if current["pgid"]==identity["pgid"] or current["session"]==identity["session"]:
            need(current["pgid"]==current["session"]==identity["pid"] and current["uid"]==identity["uid"],"foreign compiler session member denied")
            result.append(current)
    return result


def cleanup_command(process,identity):
    # HH_261007 - A live leader's generation, or all still-live original session members, proves the fresh group.
    if identity is None:
        need(process.poll() is not None,"unadmitted live command cannot be signalled")
        return dict(remaining=[],ownership_unavailable=True)
    for sig in (signal.SIGTERM,signal.SIGKILL):
        members=session_members(identity)
        if not members:break
        try:leader=proc_identity(identity["pid"])
        except FileNotFoundError:leader=None
        if leader is not None:need(leader==identity,"command leader PID generation changed")
        for member in members:need(proc_identity(member["pid"])==member,"compiler member generation changed before cleanup")
        os.killpg(identity["pgid"],sig)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:pass
        until=time.monotonic()+5
        while time.monotonic()<until and session_members(identity):time.sleep(.01)
    remaining=session_members(identity)
    need(not remaining,"owned compiler descendants remain; retained failure")
    return dict(remaining=[],ownership_unavailable=False)


def run_command(argv,label,timeout,lease_fd):
    budget();lease_guard(lease_fd);identity=None;process=None;error=None;code=None;cleanup=None
    owner=proc_identity(os.getpid());parent_pid=os.getpid()
    libc=ctypes.CDLL(None,use_errno=True)
    def limited():
        resource.setrlimit(resource.RLIMIT_FSIZE,(8*1024**2,8*1024**2));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
        if libc.prctl(1,int(signal.SIGTERM),0,0,0)!=0 or os.getppid()!=parent_pid:os._exit(125)
    with (OUT/(label+".stdout")).open("xb") as stdout,(OUT/(label+".stderr")).open("xb") as stderr:
        try:
            process=subprocess.Popen(argv,stdout=stdout,stderr=stderr,start_new_session=True,preexec_fn=limited,pass_fds=(lease_fd,))
            try:identity=proc_identity(process.pid)
            except FileNotFoundError:identity=None
            if identity is not None:need(identity["ppid"]==parent_pid and identity["pgid"]==identity["session"]==process.pid,"owned command generation required")
            until=time.monotonic()+timeout
            while process.poll() is None:
                lease_guard(lease_fd);need(proc_identity(parent_pid)==owner,"command owner generation changed")
                need(time.monotonic()<until,"owned command timeout")
                time.sleep(.02)
            code=process.wait()
        except BaseException as failure:error=repr(failure)
        finally:
            if process is not None:
                try:cleanup=cleanup_command(process,identity);code=process.poll()
                except BaseException as failure:cleanup=dict(error=repr(failure),remaining="UNKNOWN")
            receipt=dict(argv=argv,identity=identity,exit=code,error=error,cleanup=cleanup,fast_reaped_generation_unavailable=identity is None,
                stdout=fingerprint(OUT/(label+".stdout")),stderr=fingerprint(OUT/(label+".stderr")))
            write_new(OUT/(label+"_command.json"),receipt)
    need(code==0 and error is None and cleanup is not None and not cleanup.get("error"),"declared command failed; complete raw logs preserved: "+label)


def parse_dependencies(path):
    text=Path(path).read_text().replace("\\\n", " ");need(":" in text,"compiler dependency target absent")
    paths={Path(value).resolve(strict=True) for value in shlex.split(text.split(":",1)[1])}
    need(SOURCE in paths,"actual included source absent")
    roots=(Path("/usr/include"),Path("/usr/lib/gcc"),Path("/opt/ros/humble/include"))
    need(all(path==SOURCE or any(path.is_relative_to(root) for root in roots) for path in paths),"unexpected compiler include dependency")
    return {str(path):fingerprint(path) for path in sorted(paths)}


def shared_closure(lease_fd):
    # HH_261007 - ldd only examines the trusted pre-existing ROS libraries, never an untrusted binary.
    result={}
    for index,path in enumerate(LIBRARIES):
        run_command([str(Path("/usr/bin/ldd")),str(path)],"libraries_"+str(index),30,lease_fd)
        for line in (OUT/("libraries_"+str(index)+".stdout")).read_text().splitlines():
            parts=line.split()
            if "=>" in parts:
                i=parts.index("=>");need(parts[i+1]!="not","unresolved original shared dependency")
                name=parts[i+1]
            else:name=parts[0] if parts else ""
            if name.startswith("/"):
                info=fingerprint(name);result[info["resolved_path"]]=fingerprint(info["resolved_path"])
        actual=Path(path).resolve(strict=True);result[str(actual)]=fingerprint(actual)
    return result


def build(plan,lease_fd):
    need(not BINARY.exists() and not (OUT/"build_result.json").exists(),"fresh private build required")
    BINARY.parent.mkdir(parents=True,exist_ok=True)
    includes=[argument for path in INCLUDES for argument in ("-I",str(path))]
    dependency=OUT/"preliminary_dependencies.d"
    run_command([str(CC),*plan["compiler_flags"],"-M","-MF",str(dependency),"-MT","owned_bridge",*includes,str(SOURCE)],"dependencies",30,lease_fd)
    before=parse_dependencies(dependency);shared=shared_closure(lease_fd)
    actual=OUT/"actual_dependencies.d"
    argv=[str(CC),*plan["compiler_flags"],"-MD","-MF",str(actual),*includes,str(SOURCE),"-o",str(BINARY),"-L/opt/ros/humble/lib","-Wl,-rpath,/opt/ros/humble/lib", "-Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu","-lrcl","-lrmw","-lrcutils",str(DDS_LIBRARY)]
    run_command(argv,"compiler",120,lease_fd)
    after=parse_dependencies(actual);need(after==before,"included header/source generation changed across actual compilation")
    for path,info in shared.items():need(fingerprint(path)==info,"linked dependency changed during build")
    for mapping in (plan["toolchain"],plan["libraries"]):
        for path,info in mapping.items():need(fingerprint(path)==info,"toolchain/library generation changed during build")
    need(not closing_seals(plan),"build closing source/system seals failed; failure retained")
    result=dict(schema=SCHEMA+".build_result",status="BUILT_NOT_ROS_PROBED",native_module_path=str(BINARY),native_module_sha256=fingerprint(BINARY)["sha256"],
        dependency_sha256={path:info["sha256"] for path,info in (before|shared).items()},dependencies=before,shared_libraries=shared,source_sha256=plan["source_sha256"],system_or_package_changes=False)
    write_new(OUT/"build_result.json",result);return result


def graph_guard(node,complete=True):
    names=node.get_node_names_and_namespaces()
    need(len(names)==len(set(names)) and set(names)<=NODES,"unknown/duplicate DDS graph node")
    if complete:need(set(names)==NODES,"whole genuine owned graph required")
    for topic,types in node.get_topic_names_and_types():
        need(topic in TOPICS and types==[TOPICS[topic]],"unknown DDS topic/type; actuator/default deny")


def no_model_imports():
    need(not any(name=="torch" or name.startswith("torch.") or name=="carla" or name.startswith("carla.") for name in sys.modules),"Torch/CARLA import forbidden")


def system_module_seals():
    result={}
    for name,module in tuple(sys.modules.items()):
        path=getattr(module,"__file__",None)
        if path and (name==MODULE or name.startswith(("rclpy","std_msgs","sensor_msgs","rosidl","rmw"))):
            actual=Path(path).resolve(strict=True)
            need(actual==BINARY or actual.is_relative_to(Path("/opt/ros/humble")),"noncanonical ROS module origin denied")
            result[str(actual)]=fingerprint(path)
    return result


def closing_seals(plan,dependencies=None):
    errors=[]
    checks=[("sources",lambda:need(source_pins()==plan["source_sha256"],"whole sealed source changed")),
        ("native_binding",lambda:need(fingerprint(plan["existing_native_binding"])==plan["native_binding"],"original rclpy binary changed")),
        ("no_models",no_model_imports)]
    if "environment_receipt" in plan:
        item=plan["environment_receipt"]
        checks.append(("environment_receipt",lambda:need(fingerprint(item["path"])["sha256"]==item["sha256"],"canonical environment receipt changed")))
    for key in ("toolchain","libraries","preserved_evidence"):
        for path,value in plan[key].items():checks.append((path,lambda p=path,v=value:need(fingerprint(p)==v,"original installed input changed")))
    for path,pin in (dependencies or {}).items():checks.append((path,lambda p=path,v=pin:need(fingerprint(p)["sha256"]==v,"compiled dependency changed")))
    for label,check in checks:
        try:check()
        except BaseException as failure:errors.append(dict(stage=label,error=repr(failure)))
    return errors


def validate_normalization(info,topic):
    # HH_261007 - Validate native provenance and exact ABI, not produce or substitute an endpoint identity.
    need(type(info) is dict and info.get("implementation_identifier")=="rmw_cyclonedds_cpp","exact native Cyclone metadata required")
    for key,size in (("publisher_gid",24),("raw_rmw_publisher_gid",24),("dds_reader_guid",16)):
        need(type(info.get(key)) is bytes and len(info[key])==size,"exact native GUID/handle bytes required")
    raw=info["raw_rmw_publisher_gid"];handle=info.get("actual_publication_handle")
    need(type(handle) is int and 0<handle<2**64 and raw[8:]==bytes(16) and int.from_bytes(raw[:8],sys.byteorder)==handle,"malformed original publication handle refused")
    need(info.get("publisher_gid_provenance")==PROVENANCE and info["publisher_gid"][16:]==bytes(8),"actual matched-endpoint provenance/padding required")
    need(info.get("dds_topic_name")=="rt"+topic and type(info.get("dds_type_name")) is str and info["dds_type_name"],"exact native topic/type required")
    for key in ("dds_reader","dds_participant"):need(type(info.get(key)) is int and info[key]>0,"actual owned reader/participant required")
    need(type(info.get("dds_local_entity_count")) is int and 1<=info["dds_local_entity_count"]<=512,"whole native entity bound required")


def json_native_info(info):return {key:value.hex() if type(value) is bytes else value for key,value in info.items()}


def probe(plan,lease_fd):
    need(not (OUT/"result.json").exists(),"fresh one-shot probe result required before any ROS import")
    prior=json_read(OUT/"build_result.json");need(prior["native_module_sha256"]==fingerprint(BINARY)["sha256"] and prior["source_sha256"]==source_pins(),"actual compiled bridge seal differs")
    for path,pin in prior["dependency_sha256"].items():need(fingerprint(path)["sha256"]==pin,"compiled input dependency changed before probe")
    sender=None;receiver=None;rclpy=None;initialized=False;error=None;closing=[];started=time.monotonic()
    rows=[dict(ordinal=i,topic="/portable_e2e/owned_native_info_probe/"+("text" if i<3 else "bytes"),status="UNEXECUTED") for i in range(6)]
    cursor=None;import_seals={}
    try:
        lease_guard(lease_fd);no_model_imports()
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from rclpy.serialization import serialize_message
        from std_msgs.msg import String
        from sensor_msgs.msg import CompressedImage
        spec=importlib.util.spec_from_file_location(MODULE,BINARY);native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)
        need(native.abi_info()["publisher_gid_bytes"]==24,"exact installed RMW GID ABI required")
        import_seals=system_module_seals()
        rclpy.init(args=[]);initialized=True
        sender=Node("owned_dds_bridge_tx",enable_rosout=False,start_parameter_services=False)
        receiver=Node("owned_dds_bridge_rx",enable_rosout=False,start_parameter_services=False)
        qos=QoSProfile(depth=4,reliability=ReliabilityPolicy.RELIABLE)
        for name,cls in (("text",String),("bytes",CompressedImage)):
            topic="/portable_e2e/owned_native_info_probe/"+name
            publisher=sender.create_publisher(cls,topic,qos);subscription=receiver.create_subscription(cls,topic,lambda message:None,qos)
            deadline=time.monotonic()+30
            while publisher.get_subscription_count()!=1:
                lease_guard(lease_fd);graph_guard(receiver,False);budget()
                need(time.monotonic()<deadline,"owned probe discovery timeout")
                rclpy.spin_once(sender,timeout_sec=.01)
            for ordinal in range(3):
                cursor=ordinal+(0 if name=="text" else 3);rows[cursor]["status"]="STARTED"
                budget();lease_guard(lease_fd);graph_guard(receiver);need(time.monotonic()-started<=60,"fixed probe wall budget exceeded")
                message=cls()
                if name=="text":message.data="HH_261007 synthetic probe "+str(ordinal)+" λ"
                else:
                    message.header.frame_id="synthetic_not_camera_or_training_data";message.header.stamp.sec=1;message.header.stamp.nanosec=ordinal
                    message.format="synthetic_probe_bytes";message.data=bytes((i+ordinal)%256 for i in range(4096))
                expected=hashlib.sha256(serialize_message(message)).hexdigest();publisher.publish(message)
                deadline=time.monotonic()+5;received=None
                while received is None:
                    lease_guard(lease_fd);graph_guard(receiver);budget()
                    need(time.monotonic()<deadline,"genuine native take timeout; no retry")
                    received=native.take_owned(subscription)
                    if received is None:time.sleep(.001)
                actual,info=received;actual_hash=hashlib.sha256(serialize_message(actual)).hexdigest()
                need(type(info) is dict,"actual native metadata dict required")
                rows[cursor].update(expected_CDR_sha256=expected,actual_CDR_sha256=actual_hash,
                    actual_info=json_native_info(info))
                validate_normalization(info,topic)
                endpoints=receiver.get_publishers_info_by_topic(topic)
                need(len(endpoints)==1 and endpoints[0].node_name=="owned_dds_bridge_tx" and endpoints[0].node_namespace=="/","exact genuine owned DDS sender endpoint required")
                need(type(info) is dict and type(info["publisher_gid"]) is bytes and len(info["publisher_gid"])==24 and info["publisher_gid"]==bytes(endpoints[0].endpoint_gid),"true taken publisher GID differs; no graph substitution")
                need(expected==actual_hash,"actual generated capsule conversion changed CDR payload")
                rows[cursor].update(status="VERIFIED",observed_endpoint_gid=bytes(endpoints[0].endpoint_gid).hex(),
                    genuine_payload_and_gid_verified=True,synthetic_payload_not_model_data=True)
                write_new(OUT/("actual_message_"+str(cursor)+".json"),rows[cursor])
            receiver.destroy_subscription(subscription);sender.destroy_publisher(publisher)
        need(all(row["status"]=="VERIFIED" for row in rows),"whole prospective six-message denominator required")
    except BaseException as failure:
        error=repr(failure)
        if cursor is not None and rows[cursor]["status"]=="STARTED":rows[cursor].update(status="FAILED",error=error)
    finally:
        for label,obj,method in (("receiver",receiver,"destroy_node"),("sender",sender,"destroy_node"),("rclpy",rclpy if initialized else None,"shutdown")):
            if obj is not None:
                try:getattr(obj,method)()
                except BaseException as failure:closing.append(dict(stage=label,error=repr(failure)))
        try:
            post_imports=system_module_seals()
            need(all(post_imports.get(path)==value for path,value in import_seals.items()),"installed ROS module changed across probe")
            import_seals=post_imports
        except BaseException as failure:closing.append(dict(stage="system_ROS_imports",error=repr(failure)))
        closing.extend(closing_seals(plan,prior["dependency_sha256"]))
        try:need(fingerprint(BINARY)["sha256"]==prior["native_module_sha256"],"native probe binary drift")
        except BaseException as failure:closing.append(dict(stage="native_binary",error=repr(failure)))
    verified=sum(row["status"]=="VERIFIED" for row in rows);passed=error is None and not closing and verified==6
    result=dict(schema=SCHEMA+".result",status="PASS_OWNED_REAL_DDS_NO_NN" if passed else "FAILED_RETAINED",pure_probe_passed=passed,
        error=error,closing_errors=closing,expected_messages=6,received=verified,UNEXECUTED=sum(row["status"]=="UNEXECUTED" for row in rows),rows=rows,
        NN_forwards=0,NN_inferences=0,optimizer_updates=0,training_steps=0,actuation=False,Torch_and_CARLA_absent=not any(item.get("stage")=="no_models" for item in closing),
        native_module_path=str(BINARY),native_module_sha256=prior["native_module_sha256"],dependency_sha256=prior["dependency_sha256"],source_sha256=plan["source_sha256"],imported_system_module_fingerprints=import_seals)
    write_new(OUT/"result.json",result);return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);parser.add_argument("--mode",choices=("prepare","build","probe"),required=True)
    parser.add_argument("--plan-sha256");parser.add_argument("--authority-sha256");args=parser.parse_args(argv)
    if args.mode=="prepare":print(json.dumps(prepare(),sort_keys=True));return 0
    need(args.plan_sha256 and args.authority_sha256,"external ROOT phase hashes required")
    # HH_261007 - Signals/deadlines unwind owned command cleanup and retain whole failure denominators.
    def interrupted(signum,frame):raise RuntimeError("owned helper interrupted: "+str(signum))
    old={sig:signal.signal(sig,interrupted) for sig in (signal.SIGTERM,signal.SIGHUP,signal.SIGINT,signal.SIGALRM)}
    result=None;plan=None;lock=None
    try:
        plan=admit(args.mode,args.plan_sha256,args.authority_sha256);budget(True);OUT.mkdir(parents=True,exist_ok=True)
        lock=LOCK.open("a+");fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);lease_guard(lock.fileno())
        signal.alarm(300 if args.mode=="build" else 60)
        result=build(plan,lock.fileno()) if args.mode=="build" else probe(plan,lock.fileno())
    except BaseException as failure:
        result=dict(schema=SCHEMA+"."+args.mode+"_failure",status="FAILED_RETAINED",error=repr(failure),pure_probe_passed=False,
            expected_messages=6,received=0,UNEXECUTED=6,rows=[dict(ordinal=i,status="UNEXECUTED") for i in range(6)],
            NN_inferences=0,training_steps=0,actuation=False,command_failure_receipts_retained=True,external_caller_raw_closure="UNKNOWN_TO_HELPER",
            source_sha256=plan["source_sha256"] if plan else None,closing_errors=closing_seals(plan) if plan else [],
            forced_SIGKILL_receipt_guaranteed=False,forced_parent_loss_descendants_known=False)
        # HH_261007 - Never overwrite an existing success/failure or write outside the fixed attempt.
        write_new(OUT/("result.json" if args.mode=="probe" else "build_failure.json"),result)
    finally:
        signal.alarm(0)
        if lock is not None:lock.close()
        for sig,handler in old.items():signal.signal(sig,handler)
    print(json.dumps(dict(status=result["status"],native_module_sha256=result.get("native_module_sha256")),sort_keys=True))
    return 0 if result["status"] in ("BUILT_NOT_ROS_PROBED","PASS_OWNED_REAL_DDS_NO_NN") else 1


if __name__=="__main__":raise SystemExit(main())
