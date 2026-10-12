"""HH_261005 - Admit recorded JPEG ROS bundles using actual archived callback identities.

No ROS, NumPy, Torch or model is imported here. This is recorded-input admission,
not proof that lossy JPEG equals the original CARLA BGRA or of P3 performance.
An external ROOT grant and immutable scalar archive are required before reads.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import struct


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.recorded_JPEG_ROS_shadow.20261005.v1"
ROLES = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
EXPECTED_CAMERAS, EXPECTED_NATIVE = 918, 1836
CORE = "learning/portable_e2e/temporal_autoware_shadow_adapter_v1_20261005.py"
CORE_SHA = "7d3cf7b6f4f131c2647921a00b62f08e4b41e30cd6b98f9583ad425267e46a0e"
NODE = "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_v1_20261005.py"
SELF = "learning/portable_e2e/temporal_recorded_ros_bundle_v1_20261005.py"
OLD_NODE = "autonomy/autoware_e2e_vad_launch/scripts/portable_e2e_shadow_node.py"
CLOSURE_HELPER = "tools/e2e/train_temporal_v2_balanced_replay_20261001.py"
CLOSURE_SHA = "a1e3ad4ed9302b02701c7e3aa56de88afacf6681b275c1bc6986f79ebfd79103"
# HH_261005 - V1 is a source-only prototype: no authority flag alone can open an unreviewed actual ROS/model execution.
ACTUAL_ENTRY_REVIEWED = False
INPUT_PREFIX = "/portable_e2e/recorded_replay"
PROVENANCE_TOPIC = INPUT_PREFIX + "/provenance"
ACK_TOPIC = INPUT_PREFIX + "/shadow_receipt"
TF_LIMITS = dict(translation_m=.005, rotation_rad=.005)


def require(value, message):
    if not value:
        raise ValueError(message)


def sha(value):
    return hashlib.sha256(value).hexdigest()


def is_sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def relative_name(value):
    require(type(value) is str and value and not Path(value).is_absolute()
        and str(Path(value)) == value and all(p not in ("", ".", "..") for p in value.split("/")),
        "canonical relative recorded name required")
    return value


def encode(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def decode(raw, limit):
    require(type(raw) is bytes and 0 < len(raw) <= limit, "bounded actual JSON bytes required")
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, "duplicate JSON key denied"); result[key] = value
        return result
    value = json.loads(raw, object_pairs_hook=unique,
        parse_constant=lambda value: require(False, "nonfinite JSON denied"))
    def finite(item):
        if type(item) is float: require(math.isfinite(item), "overflow/nonfinite JSON number denied")
        elif type(item) is dict:
            for child in item.values(): finite(child)
        elif type(item) is list:
            for child in item: finite(child)
    finite(value)
    return value


def canonical(path, boundary=ROOT):
    path = Path(path)
    require(path.is_absolute() and path == path.resolve(strict=True)
        and path.is_relative_to(boundary) and not any(p.is_symlink() for p in (path, *path.parents)),
        "canonical owned recorded path required")
    return path


def read_sealed(path, pin, limit):
    path = canonical(path)
    require(is_sha(pin) and path.is_file() and 0 < path.stat().st_size <= limit, "bounded regular recorded input required")
    raw = path.read_bytes(); require(sha(raw) == pin, "actual recorded bytes changed")
    return raw


def project_source_closure():
    # HH_261005 - Reuse ONLY the pinned stdlib AST walker, not its Torch-importing training module.
    raw = read_sealed(ROOT/CLOSURE_HELPER,CLOSURE_SHA,2*1024**2)
    function = next(n for n in ast.parse(raw).body if isinstance(n,ast.FunctionDef) and n.name=="imported_source_closure")
    namespace = dict(Path=Path,ast=ast,require=require)
    exec(compile(ast.Module(body=[function],type_ignores=[]),str(ROOT/CLOSURE_HELPER),"exec"),namespace)
    seeds = [ROOT/name for name in (SELF,NODE,CORE,OLD_NODE,CLOSURE_HELPER,"learning/portable_e2e/__init__.py")]
    # HH_261005 - The original walker intentionally skips autonomy; explicitly resolve every literal local import and dynamic spec seed.
    for entry in (NODE,OLD_NODE):
        for node in ast.walk(ast.parse(canonical(ROOT/entry).read_bytes())):
            names = [a.name for a in node.names] if isinstance(node,ast.Import) else (
                [node.module or ""]+[(node.module+"." if node.module else "")+a.name for a in node.names]
                if isinstance(node,ast.ImportFrom) else [])
            for name in names:
                if name.startswith("portable_e2e"):
                    base=ROOT/"learning"/Path(*name.split("."))
                    for path in (base.with_suffix(".py"),base/"__init__.py"):
                        if path.is_file(): seeds.append(canonical(path))
                else:
                    path=ROOT/"autonomy/autoware_e2e_vad_launch/scripts"/(name+".py")
                    if path.is_file(): seeds.append(canonical(path))
    return {str(canonical(p).relative_to(ROOT)) for p in namespace["imported_source_closure"](ROOT,seeds)}


def verify_ros_environment(authority, environ=None):
    environ = os.environ if environ is None else environ
    domain = authority.get("ROS_DOMAIN_ID");rmw=authority.get("RMW_IMPLEMENTATION")
    require(type(domain) is int and 1 <= domain <= 232 and environ.get("ROS_DOMAIN_ID") == str(domain)
        and authority.get("ROS_LOCALHOST_ONLY") == environ.get("ROS_LOCALHOST_ONLY") == "1"
        and rmw in ("rmw_cyclonedds_cpp","rmw_fastrtps_cpp") and environ.get("RMW_IMPLEMENTATION") == rmw,
        "fresh ROOT-owned nondefault DDS domain/local-only/RMW exact environment required")
    require(all(not environ.get(key) for key in ("ROS_DISCOVERY_SERVER","ROS_STATIC_PEERS",
        "CYCLONEDDS_URI","FASTRTPS_DEFAULT_PROFILES_FILE")), "unreviewed DDS discovery/profile override denied")


def verify_imported_project_modules(authority, modules, extra=()):
    for module in tuple(modules)+tuple(extra):
        name = getattr(module,"__name__","")
        filename = getattr(module,"__file__",None)
        known_name = (name in ("vad_route_logic","portable_e2e","scripts","canonical_workspace","workspace_python")
            or name.startswith(("portable_e2e.","scripts.")) or module in extra)
        local_path = type(filename) is str and Path(filename).is_absolute() and Path(filename).is_relative_to(ROOT)
        if not (known_name or local_path): continue
        path=canonical(filename or "");relative=str(path.relative_to(ROOT))
        require(relative in authority["source_sha256"], "foreign/unsealed cached project module path denied")
        read_sealed(path,authority["source_sha256"][relative],2*1024**2)


def authorize(path, expected_sha, *, enabled=False):
    # HH_261005 - Default entry denies BEFORE authority/source/archive/model/ROS access.
    require(enabled is True, "recorded replay default DENY; fresh ROOT authority required")
    require(ACTUAL_ENTRY_REVIEWED is True, "source-only V1 DENY; owned ROS supervisor and whole918 direct-JPEG parity protocol unreviewed")
    require(is_sha(expected_sha), "external ROOT raw authority SHA required")
    authority = decode(read_sealed(path, expected_sha, 1024**2), 1024**2)
    require(authority.get("schema") == SCHEMA + ".ROOT_authority" and authority.get("authorized") is True
        and authority.get("purpose") == "recorded_JPEG_shadow_replay_only"
        and type(authority.get("expected_camera_anchors")) is int and authority["expected_camera_anchors"] == EXPECTED_CAMERAS
        and type(authority.get("expected_native_states")) is int and authority["expected_native_states"] == EXPECTED_NATIVE
        and authority.get("original_full_callback_postcheck_PASS") is True
        and type(authority.get("replay_id")) is str and bool(authority["replay_id"])
        and authority.get("shared_monotonic_clock_localhost") is True
        and authority.get("external_owned_runtime_reviewed") is True
        and authority.get("TF_limits") == TF_LIMITS
        and all(authority.get(key) is False for key in ("live_sensor_authorized", "actuation_authorized",
            "TEST_access_authorized", "model_promoted", "original_BGRA_parity_claimed", "P3_approved")),
        "exact research recorded-only grant; no live sensor, actuator or capability approval")
    verify_ros_environment(authority)
    sources = authority.get("source_sha256", {})
    require(type(sources) is dict and set(sources)==project_source_closure()
        and sources.get(CORE)==CORE_SHA and sources.get(CLOSURE_HELPER)==CLOSURE_SHA,
        "exact complete project source closure, package initializers, VAD shim and frozen core required")
    for name, pin in sources.items():
        relative_name(name)
        read_sealed(ROOT / name, pin, 2 * 1024**2)
    authority["admitted_authority_sha256"] = expected_sha
    return authority


def load_index(authority):
    files = authority["scalar_inputs"]
    require(set(files) == {"context", "cameras", "states", "callbacks"}, "only declared scalar input names")
    loaded = {}
    for name, item in files.items():
        raw = read_sealed(item["path"], item["sha256"], 48 * 1024**2 if name == "callbacks" else 16 * 1024**2)
        loaded[name] = ([decode(line, 256 * 1024) for line in raw.splitlines()]
            if name in ("cameras", "states") else decode(raw, 48 * 1024**2))
    return RecordedIndex(**loaded)


def finite_vector(value, size):
    require(isinstance(value, (tuple, list)) and len(value) == size
        and all(type(v) in (int, float) and math.isfinite(v) for v in value), "finite exact-size numeric vector required")
    return tuple(value)


def features(state):
    # HH_261005 - Exact original state_features ABI, with strict scalar admission before ROS messages are built.
    command = state["command"]
    require(type(command) is int and command in range(6), "actual six-way observed command required")
    return (1., *finite_vector([state[k] for k in ("vx", "vy", "ax", "ay", "yaw_rate", "steering_tire_angle_rad")], 6),
        *(float(command == i) for i in range(6)))


def verify_observed_state(observed, original):
    # HH_261005 - SteeringReport is float32: admit ONLY exact original model-float32 bits, not a widened numeric tolerance.
    require(type(observed) is dict and set(observed) == set(original), "whole recorded state field identity required")
    field = "steering_tire_angle_rad"
    require(encode({k: v for k, v in observed.items() if k != field})
        == encode({k: v for k, v in original.items() if k != field}), "independent ROS state differs from actual native state")
    finite_vector([observed[field], original[field]], 2)
    require(struct.pack("<f", observed[field]) == struct.pack("<f", original[field]),
        "ROS steering changes original float32 model input bits")


def transport_packet(anchor, authority, started_ns):
    require(type(started_ns) is int and started_ns > 0, "actual publisher monotonic clock required")
    return dict(schema=SCHEMA+".transport", replay_id=authority["replay_id"],
        authority_sha256=authority["admitted_authority_sha256"], publisher_bundle_started_ns=started_ns,
        provenance=deepcopy(anchor.provenance))


def verify_transport(packet, anchor, authority, first_camera_ns, now_ns):
    require(type(packet) is dict and set(packet)=={"schema","replay_id","authority_sha256","publisher_bundle_started_ns","provenance"}
        and packet.get("schema") == SCHEMA+".transport"
        and packet.get("replay_id") == authority["replay_id"]
        and packet.get("authority_sha256") == authority["admitted_authority_sha256"]
        and encode(packet.get("provenance")) == encode(anchor.provenance), "exact recorded transport/source join required")
    begin = packet.get("publisher_bundle_started_ns")
    require(all(type(v) is int and v > 0 for v in (begin, first_camera_ns, now_ns))
        and begin <= first_camera_ns <= now_ns, "same-host actual DDS monotonic chronology required")
    return begin


def navigation_packet(anchor, index, authority):
    # HH_261005 - Independently transport the same observed command/causal catalog; no true future or map-assistance invention.
    return dict(schema=SCHEMA+".navigation",replay_id=authority["replay_id"],
        authority_sha256=authority["admitted_authority_sha256"],ordinal=anchor.ordinal,
        native_frame=anchor.native_frame,timestamp_ns=anchor.timestamp_ns,command=anchor.state["command"],
        route_xy_map_m=deepcopy(index.context["route_xy_map_m"]))


def verify_navigation(value, anchor, index, authority):
    require(type(value) is dict and all(type(value.get(k)) is int for k in
        ("ordinal","native_frame","timestamp_ns","command"))
        and encode(value)==encode(navigation_packet(anchor,index,authority)),
        "independent recorded navigation command/causal route/frame/stamp/source join required")
    return value["command"]


@dataclass(frozen=True)
class RecordedAnchor:
    ordinal: int
    native_frame: int
    timestamp_ns: int
    state: dict
    cameras: tuple
    provenance: dict


class RecordedIndex:
    def __init__(self, context, cameras, states, callbacks):
        require(len(cameras) == EXPECTED_CAMERAS and len(states) == EXPECTED_NATIVE, "entire918/1836 interval including failures required")
        require(context.get("schema") == SCHEMA + ".context" and context.get("split") == "val"
            and type(context.get("episode_id")) is str and bool(context["episode_id"])
            and is_sha(context.get("stream_source_sha256"))
            and context.get("camera_order") == list(ROLES)
            and context.get("recorded_JPEG_only") is True, "explicit knownDEV recorded context required")
        episode_root = Path(context["episode_root"])
        require(episode_root.is_absolute() and str(episode_root) == context["episode_root"]
            and episode_root.is_relative_to(ROOT) and ".." not in episode_root.parts,
            "declared canonical local recorded episode root required")
        calibration = tuple(finite_vector(row, 16) for row in context["calibration"])
        require(len(calibration) == 6 and context["rig_id"] == sha(json.dumps(calibration, allow_nan=False).encode()),
            "original ordered six-camera calibration rig identity required")
        require(set(context["optical_frames"]) == set(ROLES) and set(context["jpeg_sha256"]) == {p for row in cameras for p in row["images"].values()},
            "all real optical frames and every retained JPEG seal required")
        require(len(set(context["optical_frames"].values())) == 6
            and all(type(v) is str and v for v in context["optical_frames"].values()), "distinct actual optical frames required")
        for name, pin in context["jpeg_sha256"].items():
            require(relative_name(name).split("/")[0] == "images" and is_sha(pin), "retained JPEG path/SHA required")
        require(set(context["camera_transforms"]) == set(ROLES), "all six recorded extrinsics required")
        for row in context["camera_transforms"].values():
            finite_vector(row["translation"], 3); q = finite_vector(row["quaternion"], 4)
            require(abs(sum(v*v for v in q)-1.) <= 1.e-4, "normalized original camera quaternion required")
        finite_vector([point[0] for point in context["route_xy_map_m"]], len(context["route_xy_map_m"]))
        for point in context["route_xy_map_m"]: finite_vector(point, 2)
        require(len(context["route_xy_map_m"]) >= 2, "original causal catalog navigation points required")
        require(callbacks.get("errors") == [] and callbacks.get("inflight_callbacks") == 0
            and callbacks.get("all_callbacks_including_bootstrap_or_unconsumed_preserved") is True
            and callbacks.get("scalar_receipt_limit_exceeded") is False, "complete original callback archive required")
        bindings = callbacks["camera_bindings"]
        require(len(bindings) == 6 and {r["role"] for r in bindings} == set(ROLES)
            and all(type(r["actor_id"]) is int and r["actor_id"] > 0 for r in bindings)
            and len({r["actor_id"] for r in bindings}) == 6, "actual distinct six camera actors required")
        actors = {r["role"]: r["actor_id"] for r in bindings}; rows = {}
        require(type(callbacks["callback_count"]) is int
            and len(callbacks["callbacks"]) == callbacks["callback_count"] <= 24000, "full callback denominator required")
        for index, row in enumerate(callbacks["callbacks"], 1):
            key = row["role"], row["native_frame"], row["image_object_id"]
            require(all(type(row[k]) is int and row[k] > 0 for k in ("native_frame", "image_object_id", "receipt_sequence", "actor_id"))
                and type(row["source_image_timestamp_simulation_ns"]) is int and row["source_image_timestamp_simulation_ns"] >= 0
                and is_sha(row["source_BGRA_encoding_prefixed_sha256"])
                and key not in rows and row["receipt_sequence"] == index and row["actor_id"] == actors[row["role"]]
                and row.get("receipt_error") is None and row.get("original_queue_put_error") is None
                and row.get("original_queue_put_returned") is True and row.get("original_queue_put_calls") == 1
                and row["source_byte_count"] == 640*360*4 and row["source_pixels_changed"] is False
                and row["source_timestamp_changed"] is False, "actual callback identity/source proof required")
            rows[key] = row
        require(len(callbacks["bundles"]) == len(cameras), "all actual selected bundles required")
        native = [row["frame"] for row in states]
        require(all(type(frame) is int for frame in native) and all(b == a+1 for a,b in zip(native,native[1:]))
            and [r["frame"] for r in cameras] == native[::2], "native frames come from archive, never timestamps")
        by_frame = {r["frame"]: r for r in states}; anchors = []
        for state in states:
            require(type(state["timestamp"]) in (int, float) and math.isfinite(state["timestamp"]) and state["timestamp"] >= 0,
                "actual finite native simulation stamp required")
            finite_vector([state[k] for k in ("x", "y", "z", "yaw")], 4); features(state)
        for ordinal, (camera, bundle) in enumerate(zip(cameras, callbacks["bundles"])):
            frame = camera["frame"]; state = by_frame[frame]; stamp = int(round(state["timestamp"]*1.e9))
            require(type(frame) is int and type(bundle["native_frame"]) is int
                and type(camera["timestamp"]) in (int, float) and math.isfinite(camera["timestamp"])
                and camera["timestamp"] == state["timestamp"] and bundle["native_frame"] == frame
                and set(camera["images"]) == set(camera["source_timestamps"]) == set(ROLES)
                and camera["camera_order"] == list(ROLES) and tuple(r["role"] for r in bundle["cameras"]) == ROLES,
                "actual camera/state/selected bundle identity required")
            selected = []
            for source in bundle["cameras"]:
                role = source["role"]; row = rows[role, frame, source["image_object_id"]]
                require(type(camera["source_timestamps"][role]) in (int, float)
                    and math.isfinite(camera["source_timestamps"][role]), "finite actual camera source stamp required")
                require(type(source["image_object_id"]) is int and type(source["native_frame"]) is int
                    and type(source["source_image_timestamp_simulation_ns"]) is int
                    and source["native_frame"] == frame and source["source_image_timestamp_simulation_ns"]
                    == row["source_image_timestamp_simulation_ns"] == stamp
                    == int(round(camera["source_timestamps"][role]*1.e9)), "actual selected source tuple differs")
                selected.append(dict(role=role, native_frame=frame, receipt_sequence=row["receipt_sequence"],
                    original_process_image_object_id=row["image_object_id"], actor_id=row["actor_id"],
                    source_BGRA_encoding_prefixed_sha256=row["source_BGRA_encoding_prefixed_sha256"],
                    jpeg_path=camera["images"][role], jpeg_sha256=context["jpeg_sha256"][camera["images"][role]]))
            finite_vector([state[k] for k in ("x", "y", "z", "yaw")], 4); features(state)
            proof = dict(schema=SCHEMA + ".bundle", ordinal=ordinal, native_frame=frame, timestamp_ns=stamp,
                episode_id=context["episode_id"], split="val", rig_id=context["rig_id"],
                stream_source_sha256=context["stream_source_sha256"], cameras=selected,
                recorded_JPEG_only=True, original_BGRA_parity_claimed=False)
            anchors.append(RecordedAnchor(ordinal, frame, stamp, deepcopy(state), tuple(selected), proof))
        require(all(b.timestamp_ns-a.timestamp_ns in range(99_999_000,100_001_001)
            for a,b in zip(anchors,anchors[1:])), "real consecutive original10Hz camera stamps required")
        self.context, self.calibration, self.anchors = deepcopy(context), calibration, tuple(anchors)


class BundleAdmission:
    def __init__(self, index):
        self.index, self.next_ordinal, self.history = index, 0, deque(maxlen=10)

    def admit(self, provenance, cameras, state, calibration, *, tf_verified_rig_id):
        require(self.next_ordinal < len(self.index.anchors), "no extra/future/retry anchor")
        anchor = self.index.anchors[self.next_ordinal]
        require(encode(provenance) == encode(anchor.provenance), "independent actual callback provenance mismatch")
        require(tuple(calibration) == self.index.calibration and tf_verified_rig_id == self.index.context["rig_id"],
            "independently checked live CameraInfo/TF rig required")
        require(set(cameras) == set(ROLES), "actual six-camera ROS state join required")
        verify_observed_state(state, anchor.state)
        for source in anchor.cameras:
            message = cameras[source["role"]]
            require(type(message["timestamp_ns"]) is int and message["timestamp_ns"] == anchor.timestamp_ns
                and message["frame_id"] == self.index.context["optical_frames"][source["role"]]
                and message["format"] == "jpeg" and type(message["data"]) is bytes
                and 0 < len(message["data"]) <= 2*1024**2 and sha(message["data"]) == source["jpeg_sha256"],
                "real JPEG ROS payload/frame/stamp mismatch")
        # HH_261005 - The independently admitted SteeringReport maps to the same original float32 tensor bits; retain original scalar history.
        self.history.append(features(anchor.state)); self.next_ordinal += 1
        missing = 10-len(self.history)
        return anchor, ((0.,)*13,)*missing+tuple(self.history), (False,)*missing+(True,)*len(self.history)

    def denominator(self):
        return dict(expected=EXPECTED_CAMERAS, admitted=self.next_ordinal, UNEXECUTED=EXPECTED_CAMERAS-self.next_ordinal,
            UNEXECUTED_notice="no admitted completed bundle; a failing bundle may already have arrived or executed",
            P3_approved=False, original_BGRA_parity_claimed=False, full_DDS_to_publication_timing_verified=False)
