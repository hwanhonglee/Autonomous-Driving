"""HH_261008 - Preserve V1 and bind every later boundary to its original provided RPC listener.

SOURCE-ONLY integration contract, not a runnable owner or collector. A separately
sealed owner MUST obtain each observation from fresh kernel/CARLA observations,
seal the declaration/first-RPC receipt, hold the canonical shared lease, and check
resources, routes/assets and deadlines. This helper never reads /proc or files,
imports simulator/numeric packages, launches, signals, connects, ticks, destroys
actors or authorizes TRAIN. Passing caller-supplied dictionaries authenticates
nothing by itself. This narrow V2 admits direct shipping/collector child sessions;
historical shell-wrapper descendant trees require another explicit revision.
Path comparisons here are lexical; the owner must separately resolve actual
regular non-symlink files and hash bytes, observe FDs and verify the real flock.

Every later boundary revalidates the original receipt's provided ownership and
compares its fixed listener identity with fresh observations. Socket replacement
is a denial even when PID/start ticks remain equal. Neither receipt origin nor
immutability is authenticated here; the sealed owner must supply that guarantee.
The prior/raw/current binding witness is nested in registry.first_RPC_binding;
the V1 metadata list/string/dictionary/node bounds remain unchanged.

Raw input contract:
  declaration: schema, owner/collector/server_generation, shipping_path/sha256,
    host, port, network_namespace, lease{path,fd,dev,ino}, actor_specs.
  observation: fresh owner/collector/server_generation, shipping{path,sha256},
    lease{path,fd,dev,ino,owner_pid,owner_start_ticks,collector_pid,
      collector_start_ticks,collector_ppid,collector_fd,collector_dev,
      collector_ino,flock_verified}, listener{protocol,address,port,state,inode,
      pid,start_ticks,boot_id,network_namespace,shipping_fd_inodes}.
  generation: exact pid/start_ticks/boot_id/uid/argv/ppid/pgid/session dictionary.
  creation_events: ordered actual spawn-attempt records {ordinal,role,status,
    actor,error}; created actor={id,type_id,parent_id,role_name}; failed actor=None.
  live_actors: actual current vehicle/sensor census records {role,actor} only.
  receive_events: ordered camera-attempt records {ordinal,role,actor_id,
    required_frame,status,observed_header,error}; never fill unreceived headers.

First RPC is guarded before get_world/get_map. Later boundaries use the sealed
first-RPC receipt and fresh observations. Cleanup candidates are not a cleanup
result; the owner MUST recheck generation and exact actor identity before EACH
RPC/stop/destroy and retain actual action outcomes. Missing census actors are
not asserted destroyed. World snapshot/header correctness belongs to the separate
mapping_sensor_provenance_20261008_v2 helper, not to ownership validation here.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import PurePosixPath

SCHEMA = "portable_e2e.mapping_capture_ownership.20261008.v2"
DECLARATION_SCHEMA = SCHEMA + ".declaration"
# HH_261008 - These frozen parents remain source provenance only; this helper never reads their files.
OLD_V1_PINS = {
    "learning/portable_e2e/mapping_capture_ownership_20261008_v1.py": "74d1a09a16bf4ba633f3de22b1367fa44d55a1e53f4bf64103f22ce737734468",
    "learning/tests/test_mapping_capture_ownership_20261008_v1.py": "8f8a4521d9f4ddfb3d11f0f453692861c7224053e053ad51fdea887101652854",
}
LISTENER_FIXED_FIELDS = ("protocol", "address", "port", "state", "inode", "pid", "start_ticks", "boot_id", "network_namespace")
FIRST_RECEIPT_FALSE_FIELDS = {
    "owner_observation_authentication_implemented", "first_RPC_performed", "simulator_action_performed",
    "kernel_generation_or_lease_or_listener_verified_by_helper", "shipping_binary_bytes_verified_by_helper",
    "cleanup_performed", "actual_CARLA_capture_claimed", "TRAIN_admitted", "hd_map_approved",
    "failed_raw_capture_newly_TRAIN_eligible", "learned_model_control",
}
FIRST_RECEIPT_FIELDS = FIRST_RECEIPT_FALSE_FIELDS | {
    "schema", "provenance", "stage", "declaration_sha256", "observed", "failures", "status", "evidence_origin",
}
LOCK = "/home/a/autoware_e2e/storage/assets/locks/autoware_e2e_runtime.lock"
CAMERAS = ("rgb", "depth", "semantic")
TYPES = dict(ego="vehicle.toyota.prius", rgb="sensor.camera.rgb",
             depth="sensor.camera.depth", semantic="sensor.camera.semantic_segmentation",
             collision="sensor.other.collision")
GENERATION_KEYS = {"pid", "start_ticks", "boot_id", "uid", "argv", "ppid", "pgid", "session"}
DECLARATION_KEYS = {"schema", "owner_generation", "collector_generation", "server_generation",
                    "shipping_path", "shipping_sha256", "host", "port", "network_namespace", "lease", "actor_specs"}
OBSERVATION_KEYS = {"owner_generation", "collector_generation", "server_generation", "shipping", "lease", "listener"}
ACTOR_KEYS = {"id", "type_id", "parent_id", "role_name"}
EVENT_KEYS = {"ordinal", "role", "status", "actor", "error"}
RECEIVE_KEYS = {"ordinal", "role", "actor_id", "required_frame", "status", "observed_header", "error"}
STAGES = ("before_first_rpc", "before_actor_creation", "before_tick", "before_cleanup")


def integer(value, minimum=0):
    return type(value) is int and minimum <= value < 2**63


def exact(value, keys):
    return type(value) is dict and set(value) == keys


def text(value, maximum=4096):
    return type(value) is str and 0 < len(value) <= maximum and "\0" not in value


def absolute_path(value):
    if not text(value) or "\\" in value:
        return False
    path = PurePosixPath(value)
    return path.is_absolute() and str(path) == value and ".." not in path.parts and value != "/"


def sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def boot(value):
    return (type(value) is str and len(value) == 36 and
            all(value[i] == "-" for i in (8, 13, 18, 23)) and
            all(c in "0123456789abcdef" for i, c in enumerate(value) if i not in (8, 13, 18, 23)))


def network_namespace(value):
    if not text(value, 128) or not value.startswith("net:[") or not value.endswith("]"):
        return False
    number = value[5:-1]
    return bool(number) and all(c in "0123456789" for c in number) and number[0] != "0" and 0 < int(number) < 2**63


def retain(value, *, depth=0, counter=None):
    """HH_261008 - Bound malformed metadata, retaining explicit omissions rather than fake values."""
    counter = [0] if counter is None else counter
    counter[0] += 1
    if depth > 12 or counter[0] > 4096:
        return dict(raw_type=type(value).__name__, metadata_not_retained=True, bound_exceeded=True)
    if value is None or type(value) is bool:
        return value
    if type(value) is int:
        return value if value.bit_length() <= 64 else dict(raw_type="int", bit_length=value.bit_length(), accepted=False)
    if type(value) is float:
        return value if math.isfinite(value) else dict(raw_type="float", raw_repr=repr(value), accepted=False)
    if type(value) is str:
        return value if len(value) <= 4096 else dict(raw_type="str", length=len(value), prefix=value[:4096], truncated=True)
    if type(value) in (list, tuple):
        if len(value) > 64:
            return dict(raw_type=type(value).__name__, length=len(value), metadata_not_retained=True, bound_exceeded=True)
        return [retain(v, depth=depth+1, counter=counter) for v in value]
    if type(value) is dict:
        if len(value) > 32 or any(type(k) is not str or len(k) > 256 for k in value):
            return dict(raw_type="dict", length=len(value), metadata_not_retained=True, bound_exceeded=True)
        return {k: retain(v, depth=depth+1, counter=counter) for k, v in value.items()}
    return dict(raw_type=type(value).__name__, metadata_not_retained=True, accepted=False)


def generation(value):
    return (exact(value, GENERATION_KEYS) and all(integer(value[k], 1) for k in ("pid", "start_ticks", "pgid", "session"))
            and integer(value["ppid"], 1) and integer(value["uid"]) and boot(value["boot_id"])
            and type(value["argv"]) is list and 1 <= len(value["argv"]) <= 64
            and all(text(arg) for arg in value["argv"]))


def declaration_failures(declared):
    failures = []
    if not exact(declared, DECLARATION_KEYS):
        return ["exact_declaration_fields_required"]
    def need(ok, name):
        if not ok:
            failures.append(name)
    need(declared["schema"] == DECLARATION_SCHEMA, "declaration_schema_differs")
    for role in ("owner", "collector", "server"):
        need(generation(declared[role+"_generation"]), role+"_generation_invalid")
    owner, collector, server = (declared[k+"_generation"] for k in ("owner", "collector", "server"))
    if all(generation(v) for v in (owner, collector, server)):
        need(len({v["pid"] for v in (owner, collector, server)}) == 3, "distinct_process_generations_required")
        for role, child in (("collector", collector), ("server", server)):
            need(child["ppid"] == owner["pid"] and child["uid"] == owner["uid"]
                 and child["boot_id"] == owner["boot_id"], role+"_direct_owner_child_required")
            need(child["pgid"] == child["session"] == child["pid"], role+"_isolated_session_required")
            need(child["start_ticks"] >= owner["start_ticks"], role+"_cannot_predate_owner")
        need(server["argv"][0] == declared["shipping_path"], "shipping_argv0_differs")
    need(absolute_path(declared["shipping_path"]) and PurePosixPath(declared["shipping_path"]).name == "CarlaUE4-Linux-Shipping",
         "canonical_shipping_binary_required")
    need(sha(declared["shipping_sha256"]), "shipping_source_sha_invalid")
    need(declared["host"] == "127.0.0.1" and integer(declared["port"], 1024) and declared["port"] <= 65533,
         "loopback_three_port_range_required")
    need(network_namespace(declared["network_namespace"]), "declared_network_namespace_required")
    lease = declared["lease"]
    need(exact(lease, {"path", "fd", "dev", "ino"}) and lease.get("path") == LOCK
         and integer(lease.get("fd"), 3) and integer(lease.get("dev")) and integer(lease.get("ino"), 1), "canonical_declared_lease_invalid")
    specs = declared["actor_specs"]
    if type(specs) is not list or len(specs) not in (4, 5):
        need(False, "exact_ego_three_camera_optional_collision_specs_required")
        return failures
    roles = []
    for spec in specs:
        if not exact(spec, {"role", "type_id", "parent_role", "role_name"}):
            need(False, "exact_actor_spec_fields_required")
            continue
        role = spec["role"]
        need(type(role) is str and role in TYPES, "actor_spec_role_invalid")
        roles.append(role)
        need(type(role) is str and role in TYPES and spec["type_id"] == TYPES.get(role), "actor_spec_type_differs")
        need(spec["parent_role"] == (None if role == "ego" else "ego"), "actor_spec_parent_differs")
        need(text(spec["role_name"], 256) if role == "ego" else spec["role_name"] is None or text(spec["role_name"], 256),
             "actor_spec_role_name_invalid")
    need(roles in (["ego", *CAMERAS], ["ego", "collision", *CAMERAS]), "actor_spec_creation_order_differs")
    return failures


def digest_declaration(declared):
    if declaration_failures(declared):
        return None
    return hashlib.sha256(json.dumps(declared, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _ownership_failures(declared, observed):
    failures = declaration_failures(declared)
    if failures:
        return failures
    if not exact(observed, OBSERVATION_KEYS):
        return ["exact_fresh_owner_observation_fields_required"]
    def need(ok, name):
        if not ok:
            failures.append(name)
    for role in ("owner", "collector", "server"):
        actual = observed[role+"_generation"]
        need(generation(actual) and actual == declared[role+"_generation"], role+"_generation_drift_or_invalid")
    shipping = observed["shipping"]
    need(exact(shipping, {"path", "sha256"}) and absolute_path(shipping.get("path")) and sha(shipping.get("sha256"))
         and shipping.get("path") == declared["shipping_path"]
         and shipping.get("sha256") == declared["shipping_sha256"], "shipping_binary_path_or_source_drift")
    owner, collector, server = (declared[k+"_generation"] for k in ("owner", "collector", "server"))
    lease = observed["lease"]
    keys = {"path", "fd", "dev", "ino", "owner_pid", "owner_start_ticks", "collector_pid", "collector_start_ticks",
            "collector_ppid", "collector_fd", "collector_dev", "collector_ino", "flock_verified"}
    need(exact(lease, keys) and all(integer(lease.get(k), 1) for k in
         ("owner_pid", "owner_start_ticks", "collector_pid", "collector_start_ticks", "collector_ppid", "collector_fd", "collector_ino"))
         and integer(lease.get("collector_dev"))
         and all(lease.get(k) == v and type(lease.get(k)) is type(v) for k, v in declared["lease"].items())
         and lease.get("owner_pid") == owner["pid"] and lease.get("owner_start_ticks") == owner["start_ticks"]
         and lease.get("collector_pid") == collector["pid"] and lease.get("collector_start_ticks") == collector["start_ticks"]
         and lease.get("collector_ppid") == owner["pid"] and lease.get("collector_fd") == declared["lease"]["fd"]
         and lease.get("collector_dev") == declared["lease"]["dev"] and lease.get("collector_ino") == declared["lease"]["ino"]
         and lease.get("flock_verified") is True, "inherited_lease_parent_FD_inode_or_lock_claim_differs")
    listener = observed["listener"]
    keys = {"protocol", "address", "port", "state", "inode", "pid", "start_ticks", "boot_id", "network_namespace", "shipping_fd_inodes"}
    if not exact(listener, keys):
        need(False, "exact_listener_observation_required")
        return failures
    inodes = listener["shipping_fd_inodes"]
    need(type(inodes) is list and 1 <= len(inodes) <= 64 and all(integer(i, 1) for i in inodes)
         and len(set(inodes)) == len(inodes) and integer(listener["inode"], 1) and listener["inode"] in inodes,
         "RPC_listener_inode_not_in_observed_shipping_FDs")
    need(listener["protocol"] == "tcp" and listener["address"] == declared["host"] and listener["state"] == "LISTEN"
         and type(listener["port"]) is int and listener["port"] == declared["port"], "exact_declared_RPC_listener_required")
    need(type(listener["pid"]) is int and listener["pid"] == server["pid"]
         and type(listener["start_ticks"]) is int and listener["start_ticks"] == server["start_ticks"]
         and listener["boot_id"] == server["boot_id"] and listener["network_namespace"] == declared["network_namespace"],
         "RPC_listener_server_generation_or_namespace_differs")
    return failures


def _registry(declared, events):
    """HH_261008 - Keep every provided failed/created attempt; incomplete creation is not a full rig."""
    failures, created, failed_roles = [], [], []
    specs = declared.get("actor_specs", []) if type(declared) is dict else []
    if type(specs) is not list:
        specs = []
    if type(events) is not list or len(events) > 64:
        return dict(failures=["bounded_actual_creation_event_list_required"], created=[], failed_roles=[],
                    expected=len(specs), attempted="UNKNOWN", UNEXECUTED="UNKNOWN", complete=False)
    if len(events) > len(specs):
        failures.append("extra_creation_events_retained")
    for i, event in enumerate(events):
        if not exact(event, EVENT_KEYS) or type(event.get("ordinal")) is not int or event["ordinal"] != i:
            failures.append("creation_event_schema_or_ordinal_differs:"+str(i))
            continue
        spec = specs[i] if i < len(specs) and type(specs[i]) is dict else {}
        if event["role"] != spec.get("role") or failed_roles:
            failures.append("creation_role_order_or_post_failure_attempt:"+str(i))
        actor = event["actor"]
        if event["status"] == "failed":
            failed_roles.append(event["role"])
            if actor is not None or not text(event["error"], 4096):
                failures.append("failed_creation_must_retain_error_without_fabricated_actor:"+str(i))
            continue
        if event["status"] != "created" or not exact(actor, ACTOR_KEYS) or event["error"] is not None:
            failures.append("created_actor_record_invalid:"+str(i))
            continue
        expected_parent = None if event["role"] == "ego" else next((r["actor"]["id"] for r in created if r["role"] == "ego"), None)
        if (not _actor_shape(actor) or actor["type_id"] != spec.get("type_id")
                or actor["parent_id"] != expected_parent or actor["role_name"] != spec.get("role_name")
                or (actor["parent_id"] is not None and not integer(actor["parent_id"], 1))):
            failures.append("actual_created_actor_identity_differs:"+str(i))
        created.append(dict(role=event["role"], actor=actor))
    ids = [r["actor"]["id"] for r in created if integer(r["actor"]["id"], 1)]
    if len(ids) != len(created) or len(set(ids)) != len(ids):
        failures.append("actual_created_actor_ids_invalid_or_duplicate")
    return dict(failures=failures, created=created, failed_roles=failed_roles,
                expected=len(specs), attempted=len(events), UNEXECUTED=max(0, len(specs)-len(events)),
                complete=not failures and not failed_roles and len(events) == len(specs))


def _actor_shape(actor):
    return (exact(actor, ACTOR_KEYS) and integer(actor["id"], 1) and text(actor["type_id"], 256)
            and (actor["parent_id"] is None or integer(actor["parent_id"], 1))
            and (actor["role_name"] is None or text(actor["role_name"], 256)))


def _provided_created_ids(events):
    # HH_261008 - Even a rejected registry retains declared actual IDs without implying their authenticity.
    if type(events) is not list or len(events) > 64:
        return dict(status="UNKNOWN_IDS_NOT_RETAINED_INVALID_OR_OVERSIZED_REGISTRY", ids_retained=False,
                    provided_event_count=len(events) if type(events) is list else None)
    return [event["actor"].get("id") for event in events if type(event) is dict
            and event.get("status") == "created" and type(event.get("actor")) is dict and "id" in event["actor"]]


def _actor_id_witnesses(events):
    # HH_261008 - A malformed failed attempt's supplied ID remains a witness, never a cleanup candidate.
    if type(events) is not list or len(events) > 64:
        return dict(status="UNKNOWN_ACTOR_ID_WITNESSES_NOT_RETAINED", complete=False)
    return [dict(event_index=i, supplied_status=retain(event.get("status")), supplied_actor_id=retain(event["actor"]["id"]))
            for i, event in enumerate(events) if type(event) is dict and type(event.get("actor")) is dict and "id" in event["actor"]]


def _base(stage, declared, observed, failures):
    return dict(schema=SCHEMA, provenance="HH_261008 - Owner-provided observations only; no self-authenticated ownership or actuation.",
        stage=retain(stage), declaration_sha256=digest_declaration(declared), observed=retain(observed), failures=failures,
        status="PROVIDED_METADATA_CONSISTENT_NOT_EXECUTION_AUTHORITY" if not failures else "DENY_RETAINED_EVIDENCE",
        evidence_origin="future_sealed_owner_observations_NOT_authenticated_by_helper",
        owner_observation_authentication_implemented=False, first_RPC_performed=False, simulator_action_performed=False,
        kernel_generation_or_lease_or_listener_verified_by_helper=False, shipping_binary_bytes_verified_by_helper=False,
        cleanup_performed=False, actual_CARLA_capture_claimed=False, TRAIN_admitted=False, hd_map_approved=False,
        failed_raw_capture_newly_TRAIN_eligible=False, learned_model_control=False)


def _first_observation_binding(declared, observed, first_receipt):
    """HH_261008 - Revalidate provided prior ownership and pin the original RPC socket, not unrelated FD census entries."""
    failures = []
    first_observed = first_receipt.get("observed") if type(first_receipt) is dict else None
    if (not exact(first_receipt, FIRST_RECEIPT_FIELDS)
            or first_receipt.get("provenance") != "HH_261008 - Owner-provided observations only; no self-authenticated ownership or actuation."
            or first_receipt.get("evidence_origin") != "future_sealed_owner_observations_NOT_authenticated_by_helper"
            or any(first_receipt.get(k) is not False for k in FIRST_RECEIPT_FALSE_FIELDS)):
        failures.append("first_RPC_receipt_exact_observation_contract_or_non_authority_flags_differs")
    prior_failures = _ownership_failures(declared, first_observed)
    if prior_failures:
        failures.append("first_RPC_ownership_observation_missing_or_invalid")
    prior_listener = first_observed.get("listener") if type(first_observed) is dict else None
    current_listener = observed.get("listener") if type(observed) is dict else None
    def fields(listener):
        return {k: retain(listener[k]) if type(listener) is dict and k in listener
                else dict(field_absent=True) for k in LISTENER_FIXED_FIELDS}
    before, current = fields(prior_listener), fields(current_listener)
    changed = [k for k in LISTENER_FIXED_FIELDS if before[k] != current[k]]
    if changed:
        failures.append("original_first_RPC_listener_identity_drift")
    witness = dict(first_RPC_fixed_listener=before, current_fixed_listener=current, changed_fields=changed,
                   first_ownership_validation_failures=prior_failures,
                   shipping_FD_census_not_pinned=True,
                   original_RPC_inode_must_remain_owned_in_each_fresh_provided_FD_census=True,
                   first_receipt_authenticated_by_helper=False)
    return failures, witness


def assess_boundary(declared, observed, *, stage, first_rpc_receipt=None, creation_events=None, live_actors=None):
    """Validate only supplied observations; before_cleanup candidates NEVER authorize or perform cleanup."""
    failures = _ownership_failures(declared, observed)
    if type(stage) is not str or stage not in STAGES:
        failures.append("unknown_owner_boundary")
    if stage == "before_first_rpc":
        if first_rpc_receipt is not None or creation_events is not None or live_actors is not None:
            failures.append("first_RPC_boundary_must_precede_actor_or_world_observations")
        return _base(stage, declared, observed, failures)
    if (type(first_rpc_receipt) is not dict or first_rpc_receipt.get("schema") != SCHEMA
            or first_rpc_receipt.get("stage") != "before_first_rpc" or first_rpc_receipt.get("failures") != []
            or first_rpc_receipt.get("status") != "PROVIDED_METADATA_CONSISTENT_NOT_EXECUTION_AUTHORITY"
            or first_rpc_receipt.get("declaration_sha256") is None
            or first_rpc_receipt.get("declaration_sha256") != digest_declaration(declared)):
        failures.append("sealed_first_RPC_receipt_binding_missing_or_failed")
    prior_binding_failures, listener_witness = _first_observation_binding(declared, observed, first_rpc_receipt)
    failures.extend(prior_binding_failures)
    registry = _registry(declared, creation_events)
    failures.extend(registry["failures"])
    if stage == "before_tick" and not registry["complete"]:
        failures.append("tick_requires_complete_created_rig")
    if stage == "before_actor_creation" and registry["failed_roles"]:
        failures.append("new_actor_creation_denied_after_creation_failure")
    expected = registry["created"]
    provided_live_actors = live_actors
    if type(live_actors) is not list or len(live_actors) > 5:
        failures.append("bounded_actual_vehicle_sensor_census_required")
        live_actors = []
    valid_census = True
    for row in live_actors:
        if not exact(row, {"role", "actor"}) or not text(row.get("role"), 256) or not _actor_shape(row.get("actor")) or row not in expected:
            valid_census = False
    live_ids = [row["actor"]["id"] for row in live_actors if exact(row, {"role", "actor"})
                and exact(row["actor"], ACTOR_KEYS) and integer(row["actor"]["id"], 1)]
    if len(live_ids) != len(live_actors) or len(set(live_ids)) != len(live_ids):
        valid_census = False
    if not valid_census:
        failures.append("foreign_duplicate_or_changed_actor_in_actual_census")
    expected_ids = [row["actor"]["id"] for row in expected if integer(row["actor"]["id"], 1)]
    missing = [i for i in expected_ids if i not in live_ids]
    if stage != "before_cleanup" and missing:
        failures.append("created_actor_missing_before_capture_action")
    result = _base(stage, declared, observed, failures)
    actual_ids = _provided_created_ids(creation_events)
    candidates = [i for i in reversed(expected_ids) if i in live_ids] if stage == "before_cleanup" and not failures else []
    # HH_261008 - Nest the new proof without exceeding V1's 32-field stage-receipt retention bound.
    registry["first_RPC_binding"] = dict(first_rpc_receipt_evidence=retain(first_rpc_receipt),
        RPC_listener_binding_witness=listener_witness, prior_receipt_authenticated_by_helper=False)
    result.update(creation_events=retain(creation_events), registry=retain(registry), live_actors=retain(provided_live_actors),
        provided_actor_census_rows_valid_by_metadata=valid_census and type(provided_live_actors) is list,
        actual_world_census_completeness_verified_by_helper=False,
        provided_actual_created_ids=retain(actual_ids), uncleaned_actual_created_ids=retain(actual_ids),
        provided_actor_ID_witnesses=_actor_id_witnesses(creation_events),
        missing_created_actor_ids=missing, missing_actor_destruction_claimed=False,
        cleanup_policy="DENY_RETAIN_ALL_CREATED_IDS" if failures else "PROVIDED_METADATA_CANDIDATES_ONLY_RECHECK_BEFORE_EACH_ACTION",
        cleanup_candidate_ids=candidates, cleanup_execution_authorized_by_helper=False)
    return result


def assess_receive(declared, boundary_receipt, creation_events, *, ordinal, required_frame, world_snapshot, receive_events):
    """HH_261008 - Retain partial receive failures separately from snapshot/header correctness."""
    failures = declaration_failures(declared)
    if (type(boundary_receipt) is not dict or boundary_receipt.get("schema") != SCHEMA
            or boundary_receipt.get("stage") != "before_tick" or boundary_receipt.get("failures") != []
            or boundary_receipt.get("status") != "PROVIDED_METADATA_CONSISTENT_NOT_EXECUTION_AUTHORITY"
            or boundary_receipt.get("creation_events") != retain(creation_events)
            or boundary_receipt.get("declaration_sha256") is None
            or boundary_receipt.get("declaration_sha256") != digest_declaration(declared)):
        failures.append("prior_tick_boundary_binding_missing_or_failed")
    registry = _registry(declared, creation_events)
    failures.extend(registry["failures"])
    if not registry["complete"]:
        failures.append("camera_receive_requires_complete_created_rig")
    if not integer(ordinal) or not integer(required_frame):
        failures.append("strict_receive_ordinal_and_required_frame_required")
    if type(world_snapshot) is not dict:
        failures.append("actual_world_snapshot_evidence_missing")
    if type(receive_events) is not list or len(receive_events) > 64:
        failures.append("bounded_actual_camera_receive_attempts_required")
        events = []
        actual_attempt_count = len(receive_events) if type(receive_events) is list else "UNKNOWN"
        unexecuted = "UNKNOWN_UNRETAINED_ATTEMPT_DETAILS"
    else:
        actual_attempt_count = len(receive_events)
        events = receive_events[:3]
        unexecuted = 3-len(events)
        if len(receive_events) > 3:
            failures.append("extra_camera_attempts_retained_outside_declared_denominator")
    passed, failed, failed_seen = 0, 0, False
    for i, event in enumerate(events):
        if not exact(event, RECEIVE_KEYS):
            failures.append("receive_event_schema_differs:"+str(i)); failed += 1; continue
        role = CAMERAS[i]
        registered = next((r["actor"]["id"] for r in registry["created"] if r["role"] == role), None)
        if (type(event["ordinal"]) is not int or event["ordinal"] != ordinal or event["role"] != role
                or not integer(event["actor_id"], 1) or event["actor_id"] != registered
                or type(event["required_frame"]) is not int or event["required_frame"] != required_frame or failed_seen):
            failures.append("receive_actor_role_frame_or_post_failure_attempt:"+str(i))
        if event["status"] == "received":
            if (type(event["observed_header"]) is not dict or event["error"] is not None
                    or type(event["observed_header"].get("frame")) is not int or event["observed_header"]["frame"] != required_frame):
                failures.append("actual_received_header_or_frame_missing:"+str(i)); failed += 1
            else:
                passed += 1
        elif event["status"] in ("timeout", "error"):
            failed += 1; failed_seen = True
            if not text(event["error"], 4096) or (event["observed_header"] is not None and type(event["observed_header"]) is not dict):
                failures.append("receive_failure_evidence_missing:"+str(i))
            failures.append("actual_camera_receive_failed:"+role)
        else:
            failed += 1; failures.append("receive_status_invalid:"+str(i))
    if len(events) != 3:
        failures.append("incomplete_camera_receive_denominator_retained")
    result = _base("after_receive", declared, None, failures)
    result.update(ordinal=retain(ordinal), required_frame=retain(required_frame), actual_world_snapshot=retain(world_snapshot),
        receive_events=retain(receive_events), provided_actual_created_ids=retain(_provided_created_ids(creation_events)),
        provided_actor_ID_witnesses=_actor_id_witnesses(creation_events),
        expected_camera_attempts=3, attempted_camera_slots=len(events), received_header_records=passed,
        failed_camera_attempts=failed, UNEXECUTED_camera_attempts=unexecuted,
        actual_provided_camera_attempt_count=actual_attempt_count,
        extra_camera_attempt_count=max(0, actual_attempt_count-3) if type(actual_attempt_count) is int else "UNKNOWN",
        world_snapshot_correctness_verified=False, sensor_header_correctness_verified=False,
        required_separate_header_helper="portable_e2e.mapping_sensor_provenance_20261008_v2",
        missing_headers_fabricated=False, partial_raw_training_eligible=False,
        fresh_ownership_recheck_after_receive_by_owner_still_required=True)
    return result
