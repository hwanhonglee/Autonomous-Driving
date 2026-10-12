#!/usr/bin/env python3
"""HH_261005 - Prepare a genuine ROS recorded-JPEG publisher and GOOD6000 shadow node.

Default DENY. No launch, numerical replay or ROS integration has been measured.
An external ROOT record-only grant is needed; this never publishes actuators.
The input bytes are retained JPEG, NOT recovered original BGRA. Runtime ownership
and direct-versus-ROS whole918 parity must be reviewed before actual execution.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import asdict
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[3]


def pure_bindings():
    # HH_261005 - Load only our stdlib module; package __init__ / ROS / Torch remain untouched before the explicit grant.
    path = ROOT / "learning/portable_e2e/temporal_recorded_ros_bundle_v1_20261005.py"
    spec = importlib.util.spec_from_file_location("_hh_recorded_ros_bundle", path)
    module = importlib.util.module_from_spec(spec); sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def stamp_ns(stamp):
    if type(stamp.sec) is not int or type(stamp.nanosec) is not int or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("actual ROS stamp requires nonnegative integers; no bool/coercion")
    return stamp.sec*1_000_000_000 + stamp.nanosec


def stamp(message, timestamp):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(timestamp, 1_000_000_000)


def dependencies(authority, pure):
    # HH_261005 - The only ROS/model imports occur AFTER externally sealed authority and all scalar-source admission.
    pure.verify_ros_environment(authority)
    pure.verify_imported_project_modules(authority,sys.modules.values())
    sys.path.insert(0, str(ROOT / "learning"))
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from rclpy.clock import ClockType
    from sensor_msgs.msg import CameraInfo, CompressedImage
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import AccelWithCovarianceStamped, TransformStamped
    from autoware_vehicle_msgs.msg import SteeringReport
    from std_msgs.msg import String, Float32, Int8
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer
    path = ROOT / "autonomy/autoware_e2e_vad_launch/scripts/portable_e2e_shadow_node.py"
    pure.require(str(path.relative_to(ROOT)) in authority["source_sha256"], "genuine old ROS helper source must be sealed")
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location("_hh_frozen_ros_helpers", path)
    old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
    pure.verify_imported_project_modules(authority,sys.modules.values(),extra=(old,))
    return SimpleNamespace(**locals())


def message_set(anchor, rig, context, ros):
    # HH_261005 - Publish actual archived stamps/pose/features; never divide a timestamp to manufacture a native frame.
    state = anchor.state; odom = ros.Odometry(); stamp(odom, anchor.timestamp_ns)
    odom.header.frame_id = "map"; odom.child_frame_id = "base_link"
    for key in ("x", "y", "z"): setattr(odom.pose.pose.position, key, state[key])
    odom.pose.pose.orientation.z = math.sin(state["yaw"]/2)
    odom.pose.pose.orientation.w = math.cos(state["yaw"]/2)
    odom.twist.twist.linear.x, odom.twist.twist.linear.y = state["vx"], state["vy"]
    odom.twist.twist.angular.z = state["yaw_rate"]
    acceleration = ros.AccelWithCovarianceStamped(); stamp(acceleration, anchor.timestamp_ns)
    acceleration.header.frame_id = "base_link"
    acceleration.accel.accel.linear.x, acceleration.accel.accel.linear.y = state["ax"], state["ay"]
    steering = ros.SteeringReport(); steering.stamp.sec, steering.stamp.nanosec = divmod(anchor.timestamp_ns, 1_000_000_000)
    steering.steering_tire_angle = state["steering_tire_angle_rad"]
    infos, transforms = {}, []
    for camera in rig.cameras:
        info = ros.CameraInfo(); stamp(info, anchor.timestamp_ns); info.header.frame_id = camera.optical_frame
        info.width, info.height = camera.width_px, camera.height_px; info.distortion_model = "plumb_bob"
        # HH_261005 - Rig metadata stores K/D/extrinsic, not live CameraInfo R/P; derive the exact original rectified ABI.
        k = camera.intrinsic_k
        info.k, info.d = list(k), list(camera.distortion_d or (0.,)*5)
        info.r = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
        info.p = [k[0],k[1],k[2],0.,k[3],k[4],k[5],0.,k[6],k[7],k[8],0.]
        infos[camera.name] = info
        declared = context["camera_transforms"][camera.name]
        transform = ros.TransformStamped(); stamp(transform, anchor.timestamp_ns)
        transform.header.frame_id = "base_link"; transform.child_frame_id = camera.optical_frame
        for key, value in zip(("x", "y", "z"), declared["translation"]): setattr(transform.transform.translation, key, value)
        for key, value in zip(("x", "y", "z", "w"), declared["quaternion"]): setattr(transform.transform.rotation, key, value)
        transforms.append(transform)
    clock = ros.Clock(); clock.clock.sec, clock.clock.nanosec = divmod(anchor.timestamp_ns, 1_000_000_000)
    return odom, acceleration, steering, infos, ros.TFMessage(transforms=transforms), clock


def load_runtime(authority, index, pure):
    pure.verify_ros_environment(authority)
    from portable_e2e import temporal_autoware_shadow_adapter_v1_20261005 as core
    from portable_e2e import temporal_route_joint_export_research_20261002 as loader
    from portable_e2e.contract import load_contract
    from portable_e2e.runtime import load_rig_calibration, configure_runtime_execution
    from portable_e2e.simulation_research import ControllerConfig
    pure.verify_imported_project_modules(authority,sys.modules.values())
    pure.require(authority["model"]["research_owner_binding"]["purpose"] == "offline_cpu_numeric_validation_only",
        "recorded numerical model only, never live/control loader authority")
    execution = configure_runtime_execution("cpu")
    model_fields = dict(authority["model"])
    pure.require(set(model_fields) == {"weights","metadata","weights_sha256","metadata_sha256",
        "research_owner_binding","parent_weights","parent_metadata","backbone_parent_weights","backbone_parent_metadata"},
        "exact GOOD-only original numeric-loader kwargs required")
    for key in ("weights", "metadata", "parent_weights", "parent_metadata", "backbone_parent_weights", "backbone_parent_metadata"):
        model_fields[key] = pure.canonical(model_fields[key])
    model = loader.load_route_joint_research_npz(**model_fields, source_root=ROOT)
    contract_item = authority["contract"]
    pure.read_sealed(contract_item["path"], contract_item["sha256"], 1024**2)
    contract = load_contract(Path(contract_item["path"]))
    rig_item = authority["rig"]
    rig = load_rig_calibration(Path(rig_item["path"]), expected_sha256=rig_item["sha256"], contract=contract)
    pure.require(rig.features == index.calibration, "independent pinned ROS rig must equal actual archived rig features")
    controller = ControllerConfig(**authority["controller"])
    adapter = core.TemporalAutowareShadowPreparationV1(model, controller,
        episode_id=index.context["episode_id"], split="val", rig_id=index.context["rig_id"],
        source_sha256=index.context["stream_source_sha256"], expected_calibration=index.calibration)
    return adapter, rig, contract, execution


def build_observed_frame(anchor, payloads, history, mask, rig, contract, index):
    # HH_261005 - One unchanged original frame construction is shared by ROS and a future independently authorized direct-JPEG replay.
    from portable_e2e.contract import _canonical_route_in_base
    from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs
    from portable_e2e.temporal_simulation_research_v2_20261001 import ObservedTemporalFrameV2
    state = anchor.state
    position = state["x"], state["y"], state["z"]
    q = (0.,0.,math.sin(state["yaw"]/2),math.cos(state["yaw"]/2))
    route, _, _ = _canonical_route_in_base(index.context["route_xy_map_m"],
        position_m=position,orientation_xyzw=q,contract=contract,context="recorded-JPEG-ROS")
    runtime = RuntimeInputs(tuple(RuntimeCameraFrame(anchor.timestamp_ns,payloads[role]["data"],"jpeg")
        for role in index.context["camera_order"]),rig.features,history,mask,route)
    return ObservedTemporalFrameV2(anchor.timestamp_ns,runtime,position,q,anchor.timestamp_ns,
        index.context["episode_id"],"val",index.context["rig_id"],index.context["stream_source_sha256"])


def recorded_node(authority, index, pure, ros):
    from portable_e2e.runtime import validate_live_camera_calibration
    from autoware_planning_msgs.msg import Trajectory
    from nav_msgs.msg import Path as PathMessage
    adapter, rig, contract, execution = load_runtime(authority, index, pure)
    class ShadowNode(ros.Node):
        def __init__(self):
            super().__init__("portable_temporal_recorded_shadow")
            self.admission = pure.BundleAdmission(index); self.inputs = {}; self.first_camera_ns = {}
            self.tf = ros.Buffer(); self.tf_seen = {}; self.last = -1; self.failed = None
            qos = ros.QoSProfile(depth=16, reliability=ros.ReliabilityPolicy.RELIABLE,
                durability=ros.DurabilityPolicy.VOLATILE)
            self.receipt = self.fixed_publisher(ros.String, pure.ACK_TOPIC, qos)
            self.trajectory = self.fixed_publisher(Trajectory, ros.old.SHADOW_TRAJECTORY_TOPIC, qos)
            self.path = self.fixed_publisher(PathMessage, ros.old.SHADOW_PATH_TOPIC, qos)
            self.status = self.fixed_publisher(ros.String, ros.old.SHADOW_STATUS_TOPIC, qos)
            self.latency = self.fixed_publisher(ros.Float32, ros.old.SHADOW_LATENCY_TOPIC, qos)
            self.candidate = self.fixed_publisher(ros.Int8, ros.old.SHADOW_SELECTED_CANDIDATE_TOPIC, qos)
            self.fixed_subscription(ros.String, pure.PROVENANCE_TOPIC, lambda msg: self.put("provenance", msg), qos)
            self.fixed_subscription(ros.String, pure.INPUT_PREFIX+"/navigation", lambda msg: self.put("navigation", msg), qos)
            for role in pure.ROLES:
                self.fixed_subscription(ros.CompressedImage, pure.INPUT_PREFIX+"/"+role+"/compressed", lambda msg, role=role: self.put(role, msg), qos)
                self.fixed_subscription(ros.CameraInfo, pure.INPUT_PREFIX+"/"+role+"/camera_info", lambda msg, role=role: self.put(role+"_info", msg), qos)
            for key, cls in (("odometry", ros.Odometry), ("acceleration", ros.AccelWithCovarianceStamped),
                ("steering", ros.SteeringReport), ("clock", ros.Clock)):
                self.fixed_subscription(cls, pure.INPUT_PREFIX+"/"+key, lambda msg, key=key: self.put(key, msg), qos)
            self.fixed_subscription(ros.TFMessage, pure.INPUT_PREFIX+"/tf", self.on_tf, qos)
            self.create_timer(.01, self.poll)

        def fixed_publisher(self, cls, topic, qos):
            resolved = self.resolve_topic_name(topic)
            allowed = {pure.ACK_TOPIC,ros.old.SHADOW_TRAJECTORY_TOPIC,ros.old.SHADOW_PATH_TOPIC,
                ros.old.SHADOW_STATUS_TOPIC,ros.old.SHADOW_LATENCY_TOPIC,ros.old.SHADOW_SELECTED_CANDIDATE_TOPIC}
            pure.require(topic in allowed and resolved == topic, "exact recorded/shadow output allowlist required")
            return self.create_publisher(cls, topic, qos)

        def fixed_subscription(self, cls, topic, callback, qos):
            pure.require(topic.startswith(pure.INPUT_PREFIX+"/") and self.resolve_topic_name(topic) == topic,
                "isolated recorded input remapping forbidden")
            return self.create_subscription(cls,topic,callback,qos)

        def fail(self, error, anchor, stages=None):
            if self.failed is not None: return
            self.failed = repr(error); adapter.reset("recorded_ROS_admission_failure")
            receipt = dict(schema=pure.SCHEMA+".shadow_receipt",replay_id=authority["replay_id"],
                authority_sha256=authority["admitted_authority_sha256"],status="FAILED_RETAINED",
                ordinal=anchor.ordinal,native_frame=anchor.native_frame,timestamp_ns=anchor.timestamp_ns,
                error=self.failed,stages=stages or {},denominator=self.admission.denominator(),
                failing_bundle_may_have_executed=True,P3_approved=False)
            self.receipt.publish(ros.String(data=pure.encode(receipt).decode())); print(pure.encode(receipt).decode(),flush=True)

        def put(self, key, message):
            if self.failed is not None or self.last+1 == pure.EXPECTED_CAMERAS: return
            try:
                if key in pure.ROLES:
                    self.first_camera_ns.setdefault(stamp_ns(message.header.stamp), time.perf_counter_ns())
                self.inputs[key] = message
            except Exception as error: self.fail(error,index.anchors[self.last+1])

        def on_tf(self, message):
            if self.failed is not None or self.last+1 == pure.EXPECTED_CAMERAS: return
            try:
                for transform in message.transforms:
                    pure.require(transform.header.frame_id == "base_link"
                        and transform.child_frame_id in index.context["optical_frames"].values(), "recorded rig-only TF input")
                    self.tf.set_transform(transform, "ROOT-recorded-JPEG-replay")
                    self.tf_seen[transform.child_frame_id] = stamp_ns(transform.header.stamp)
            except Exception as error: self.fail(error,index.anchors[self.last+1])

        def poll(self):
            if self.failed is not None or self.last+1 == pure.EXPECTED_CAMERAS: return
            required = {"provenance", "navigation", "odometry", "acceleration", "steering", "clock", *pure.ROLES,
                *(role+"_info" for role in pure.ROLES)}
            anchor = index.anchors[self.last+1]; inputs = self.inputs
            if not required <= set(inputs) or any(self.tf_seen.get(name) != anchor.timestamp_ns
                    for name in index.context["optical_frames"].values()):
                first = self.first_camera_ns.get(anchor.timestamp_ns)
                if first is not None and time.perf_counter_ns()-first > 100_000_000:
                    self.fail(ValueError("current recorded input/TF settle deadline exceeded; no skip"),anchor)
                return
            # HH_261005 - Wait for the current archived stamp; no nearest/future state or timer-generated native identity.
            if any(stamp_ns(inputs[role].header.stamp) != anchor.timestamp_ns for role in pure.ROLES): return
            started = self.first_camera_ns[anchor.timestamp_ns]; stages = {}
            try:
                packet = pure.decode(inputs["provenance"].data.encode(), 64*1024)
                publisher_started = pure.verify_transport(packet,anchor,authority,started,time.perf_counter_ns())
                stages["publisher_bundle_started_ns"] = publisher_started
                stages["all_ROS_inputs_ready_ns"] = time.perf_counter_ns()
                odom, acc, steer = inputs["odometry"], inputs["acceleration"], inputs["steering"]
                pure.require(odom.header.frame_id == "map" and odom.child_frame_id == "base_link"
                    and acc.header.frame_id == "base_link" and stamp_ns(odom.header.stamp) == anchor.timestamp_ns
                    and stamp_ns(acc.header.stamp) == anchor.timestamp_ns and stamp_ns(steer.stamp) == anchor.timestamp_ns
                    and stamp_ns(inputs["clock"].clock) == anchor.timestamp_ns, "exact independent recorded pose/state/replay-clock stamps")
                state = deepcopy(anchor.state); position = odom.pose.pose.position; q = odom.pose.pose.orientation
                expected_q = (0., 0., math.sin(state["yaw"]/2), math.cos(state["yaw"]/2))
                pure.require((q.x,q.y,q.z,q.w) == expected_q, "observed quaternion differs; no angle roundtrip or pose retagging")
                state.update(x=position.x,y=position.y,z=position.z,vx=odom.twist.twist.linear.x,
                    vy=odom.twist.twist.linear.y,yaw_rate=odom.twist.twist.angular.z,
                    ax=acc.accel.accel.linear.x,ay=acc.accel.accel.linear.y,steering_tire_angle_rad=steer.steering_tire_angle)
                navigation=pure.decode(inputs["navigation"].data.encode(),1024**2)
                state["command"]=pure.verify_navigation(navigation,anchor,index,authority)
                for camera in rig.cameras:
                    info = inputs[camera.name+"_info"]
                    pure.require(stamp_ns(info.header.stamp) == anchor.timestamp_ns, "CameraInfo belongs to another anchor")
                    validate_live_camera_calibration(camera, ros.old.camera_info_message_to_live_calibration(info, camera_name=camera.name))
                    observed = self.tf.lookup_transform("base_link", camera.optical_frame,
                        ros.Time(nanoseconds=anchor.timestamp_ns,clock_type=ros.ClockType.ROS_TIME))
                    ros.old.validate_live_extrinsic(camera.base_from_camera, ros.old.transform_message_to_matrix(observed),
                        camera_name=camera.name, maximum_translation_error_m=authority["TF_limits"]["translation_m"],
                        maximum_rotation_error_rad=authority["TF_limits"]["rotation_rad"])
                payloads = {role: dict(timestamp_ns=stamp_ns(inputs[role].header.stamp),frame_id=inputs[role].header.frame_id,
                    format=inputs[role].format,data=bytes(inputs[role].data)) for role in pure.ROLES}
                anchor, history, mask = self.admission.admit(packet["provenance"],payloads,state,rig.features,
                    tf_verified_rig_id=index.context["rig_id"])
                stages["ROS_admission_finished_ns"] = time.perf_counter_ns()
                frame = build_observed_frame(anchor,payloads,history,mask,rig,contract,index)
                stages["policy_started_ns"] = time.perf_counter_ns()
                result = adapter.accept(frame,now_ns=anchor.timestamp_ns,ego_stamp_ns=anchor.timestamp_ns,route_stamp_ns=anchor.timestamp_ns)
                stages["policy_finished_ns"] = time.perf_counter_ns(); status = result.status
                if status == "TRAJECTORY":
                    trajectory = ros.old.make_shadow_trajectory(result,odom,anchor_ns=anchor.timestamp_ns)
                    path = ros.old.make_shadow_path(trajectory); stages["conversion_finished_ns"] = time.perf_counter_ns()
                    if (time.perf_counter_ns()-publisher_started) > 100_000_000: status = "REJECTED_ROS_END_TO_END_DEADLINE"
                    else:
                        self.trajectory.publish(trajectory); self.path.publish(path)
                        self.candidate.publish(ros.Int8(data=result.selected.candidate_index))
                stages["trajectory_publication_finished_ns"] = time.perf_counter_ns()
                seconds = (stages["trajectory_publication_finished_ns"]-publisher_started)/1.e9
                receipt = dict(schema=pure.SCHEMA+".shadow_receipt",ordinal=anchor.ordinal,native_frame=anchor.native_frame,
                    timestamp_ns=anchor.timestamp_ns,status=status,policy_status=result.status,first_camera_callback_ns=started,
                    replay_id=authority["replay_id"],authority_sha256=authority["admitted_authority_sha256"],
                    stages=stages,publisher_to_trajectory_publication_seconds=seconds,execution=execution,
                    source_selected_callback_provenance=anchor.provenance,selected=asdict(result.selected) if result.selected else None,
                    policy_diagnostics=result.diagnostics,original_BGRA_parity_claimed=False,
                    direct_vs_ROS_numerical_parity_verified=False,input_tensor_hash_parity_implemented=False,
                    DDS_sender_identity_authenticated=False,
                    actual_V5_terminal_policy_equivalence_claimed=False,P3_approved=False)
                self.latency.publish(ros.Float32(data=seconds*1000)); self.status.publish(ros.String(data=pure.encode(receipt).decode()))
                stages["shadow_outputs_publication_finished_ns"] = time.perf_counter_ns()
                stages["receipt_publication_entered_ns"] = time.perf_counter_ns()
                receipt["receipt_post_publish_completion_unavailable_in_this_message"] = True
                self.receipt.publish(ros.String(data=pure.encode(receipt).decode()))
                # HH_261005 - Only the actual post-publish console event can include completion of the receipt publication itself.
                stages["receipt_publication_finished_ns"] = time.perf_counter_ns()
                receipt["record_kind"] = "ACTUAL_ALL_PUBLICATION_CALLS_COMPLETED"
                receipt["receipt_post_publish_completion_unavailable_in_this_message"] = False
                receipt["publisher_to_all_publication_seconds"] = (stages["receipt_publication_finished_ns"]-publisher_started)/1.e9
                receipt["full_publication_deadline_passed"] = receipt["publisher_to_all_publication_seconds"] <= .1
                print(pure.encode(receipt).decode(),flush=True)
                self.last = anchor.ordinal; self.inputs.clear(); self.first_camera_ns.clear()
            except Exception as error:
                self.fail(error,anchor,stages)
    return ShadowNode()


def recorded_publisher(authority, index, pure, ros):
    pure.verify_ros_environment(authority)
    from portable_e2e.contract import load_contract
    from portable_e2e.runtime import load_rig_calibration
    pure.verify_imported_project_modules(authority,sys.modules.values())
    item = authority["contract"]; pure.read_sealed(item["path"],item["sha256"],1024**2)
    contract = load_contract(Path(item["path"])); item = authority["rig"]
    rig = load_rig_calibration(Path(item["path"]),expected_sha256=item["sha256"],contract=contract)
    pure.require(rig.features == index.calibration, "publisher actual pinned rig differs from recorded calibration")
    node = ros.Node("portable_temporal_recorded_publisher"); qos = ros.QoSProfile(depth=16,reliability=ros.ReliabilityPolicy.RELIABLE)
    topics = {"provenance":(ros.String,pure.PROVENANCE_TOPIC),"navigation":(ros.String,pure.INPUT_PREFIX+"/navigation"),
        "odometry":(ros.Odometry,pure.INPUT_PREFIX+"/odometry"),
        "acceleration":(ros.AccelWithCovarianceStamped,pure.INPUT_PREFIX+"/acceleration"),"steering":(ros.SteeringReport,pure.INPUT_PREFIX+"/steering"),
        "tf":(ros.TFMessage,pure.INPUT_PREFIX+"/tf"),"clock":(ros.Clock,pure.INPUT_PREFIX+"/clock")}
    topics.update({role:(ros.CompressedImage,pure.INPUT_PREFIX+"/"+role+"/compressed") for role in pure.ROLES})
    topics.update({role+"_info":(ros.CameraInfo,pure.INPUT_PREFIX+"/"+role+"/camera_info") for role in pure.ROLES})
    publishers = {}
    for key,(cls,topic) in topics.items():
        pure.require(node.resolve_topic_name(topic)==topic, "recorded input remapping denied")
        publishers[key] = node.create_publisher(cls,topic,qos)
    receipts = {}
    def receive(message):
        value=pure.decode(message.data.encode(),128*1024)
        pure.require(value.get("schema")==pure.SCHEMA+".shadow_receipt"
            and value.get("replay_id")==authority["replay_id"]
            and value.get("authority_sha256")==authority["admitted_authority_sha256"]
            and type(value.get("ordinal")) is int and value["ordinal"] not in receipts,
            "unique original recorded shadow receipt required")
        value["publisher_receipt_received_ns"]=time.perf_counter_ns();receipts[value["ordinal"]]=value
    pure.require(node.resolve_topic_name(pure.ACK_TOPIC)==pure.ACK_TOPIC,"shadow receipt remapping denied")
    node.create_subscription(ros.String,pure.ACK_TOPIC,receive,qos)
    completed = 0
    try:
        deadline = time.monotonic()+30
        while any(p.get_subscription_count()==0 for p in publishers.values()):
            pure.require(time.monotonic()<deadline,"recorded node subscriptions absent; no payload sent")
            ros.rclpy.spin_once(node,timeout_sec=.05)
        for anchor in index.anchors:
            odom,acc,steer,infos,tf,clock = message_set(anchor,rig,index.context,ros)
            began=time.perf_counter_ns();packet=pure.transport_packet(anchor,authority,began)
            for key,value in (("provenance",ros.String(data=pure.encode(packet).decode())),
                ("navigation",ros.String(data=pure.encode(pure.navigation_packet(anchor,index,authority)).decode())),
                ("odometry",odom),("acceleration",acc),("steering",steer),("tf",tf),("clock",clock)):
                publishers[key].publish(value)
            for source in anchor.cameras:
                role=source["role"]; publishers[role+"_info"].publish(infos[role])
                raw=pure.read_sealed(pure.canonical(Path(index.context["episode_root"])/source["jpeg_path"]),source["jpeg_sha256"],2*1024**2)
                message=ros.CompressedImage();stamp(message,anchor.timestamp_ns);message.header.frame_id=index.context["optical_frames"][role]
                message.format="jpeg";message.data=raw;publishers[role].publish(message)
            deadline=time.monotonic()+5
            while anchor.ordinal not in receipts:
                pure.require(time.monotonic()<deadline,"recorded shadow receipt timeout; no retry/skip")
                ros.rclpy.spin_once(node,timeout_sec=.01)
            receipt=receipts[anchor.ordinal]
            pure.require(type(receipt.get("native_frame")) is int and receipt["native_frame"]==anchor.native_frame
                and receipt.get("timestamp_ns")==anchor.timestamp_ns and receipt.get("status") in
                ("PREFILL","REJECTED","TRAJECTORY","REJECTED_ROS_END_TO_END_DEADLINE"),
                "recorded node failed; entire remaining tail unexecuted")
            print(pure.encode(receipt).decode(),flush=True)
            completed+=1
    finally:
        print(json.dumps(dict(expected=918,completed_receipts=completed,UNEXECUTED=918-completed,
            UNEXECUTED_notice="no completed receipt; a failing bundle may already have executed",
            recorded_JPEG_only=True,no_retry=True,P3_approved=False,wall_10Hz_replay_claimed=False)),flush=True)
        node.destroy_node()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--enable-recorded-replay",action="store_true")
    parser.add_argument("--authority",type=Path);parser.add_argument("--authority-sha256")
    parser.add_argument("--mode",choices=("publisher","node"),required=True)
    args=parser.parse_args(argv);pure=pure_bindings()
    authority=pure.authorize(args.authority,args.authority_sha256,enabled=args.enable_recorded_replay)
    index=pure.load_index(authority);ros=dependencies(authority,pure);ros.rclpy.init(args=[])
    try:
        if args.mode=="publisher":recorded_publisher(authority,index,pure,ros)
        else:
            node=recorded_node(authority,index,pure,ros)
            try:ros.rclpy.spin(node)
            finally:
                print(json.dumps(node.admission.denominator()),flush=True);node.destroy_node()
    finally:ros.rclpy.shutdown()
    return 0


if __name__=="__main__":raise SystemExit(main())
