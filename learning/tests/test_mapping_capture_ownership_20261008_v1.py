"""HH_261008 - Pure fictional ownership observations, never kernel/CARLA or feature evidence.

SOURCE-READY fixtures for ROOT review. They do not acquire locks, inspect /proc,
spawn/signal/connect/drive/destroy, install packages or read actual capture arrays.
"""
from copy import deepcopy

import pytest

from portable_e2e import mapping_capture_ownership_20261008_v1 as subject


def fixture(*, collision=False):
    boot_id = "12345678-1234-1234-1234-123456789abc"
    shipping = "/home/a/carla-autoware-universe/CARLA_0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
    def generation(pid, parent, argv):
        return dict(pid=pid, start_ticks=pid*10, boot_id=boot_id, uid=1000,
                    argv=argv, ppid=parent, pgid=pid, session=pid)
    owner = generation(100, 50, ["/usr/bin/python3", "fictional_owner.py"])
    collector = generation(200, 100, ["/usr/bin/python3", "fictional_collector.py"])
    server = generation(300, 100, [shipping, "-carla-rpc-port=2600", "-RenderOffScreen"])
    roles = ["ego", "collision", *subject.CAMERAS] if collision else ["ego", *subject.CAMERAS]
    specs = [dict(role=role, type_id=subject.TYPES[role], parent_role=None if role == "ego" else "ego",
                  role_name="fictional_owned_mapping" if role == "ego" else None) for role in roles]
    declared = dict(schema=subject.DECLARATION_SCHEMA, owner_generation=owner, collector_generation=collector,
        server_generation=server, shipping_path=shipping, shipping_sha256="a"*64, host="127.0.0.1", port=2600,
        network_namespace="net:[12345]", lease=dict(path=subject.LOCK, fd=10, dev=8, ino=123), actor_specs=specs)
    observed = dict(owner_generation=deepcopy(owner), collector_generation=deepcopy(collector), server_generation=deepcopy(server),
        shipping=dict(path=shipping, sha256="a"*64), lease=dict(path=subject.LOCK, fd=10, dev=8, ino=123,
        owner_pid=100, owner_start_ticks=1000, collector_pid=200, collector_start_ticks=2000, collector_ppid=100,
        collector_fd=10, collector_dev=8, collector_ino=123, flock_verified=True),
        listener=dict(protocol="tcp", address="127.0.0.1", port=2600, state="LISTEN", inode=987,
        pid=300, start_ticks=3000, boot_id=boot_id, network_namespace="net:[12345]", shipping_fd_inodes=[986, 987]))
    events = [dict(ordinal=i, role=role, status="created", actor=dict(id=i+1, type_id=subject.TYPES[role],
        parent_id=None if role == "ego" else 1, role_name="fictional_owned_mapping" if role == "ego" else None),
        error=None) for i, role in enumerate(roles)]
    return declared, observed, events


def first(declared, observed):
    return subject.assess_boundary(declared, observed, stage="before_first_rpc")


def census(events):
    return [dict(role=e["role"], actor=deepcopy(e["actor"])) for e in events if e["status"] == "created"]


def boundary(stage="before_tick", *, collision=False):
    declared, observed, events = fixture(collision=collision)
    receipt = first(declared, observed)
    return declared, observed, events, receipt, subject.assess_boundary(declared, observed, stage=stage,
        first_rpc_receipt=receipt, creation_events=events, live_actors=census(events))


def receives(events, frame=42, ordinal=0):
    ids = {e["role"]: e["actor"]["id"] for e in events if e["status"] == "created"}
    return [dict(ordinal=ordinal, role=role, actor_id=ids[role], required_frame=frame, status="received",
        observed_header=dict(frame=frame, timestamp_s=4.2), error=None) for role in subject.CAMERAS]


def receive_result(events_override=None, *, snapshot=None):
    declared, _, events, _, tick = boundary()
    return subject.assess_receive(declared, tick, events, ordinal=0, required_frame=42,
        world_snapshot=dict(frame=42, elapsed_seconds=4.2, delta_seconds=.1) if snapshot is None else snapshot,
        receive_events=receives(events) if events_override is None else events_override)


def test_first_boundary_is_only_consistent_provided_metadata():
    declared, observed, _ = fixture()
    actual = first(declared, observed)
    assert actual["failures"] == []
    assert actual["status"] == "PROVIDED_METADATA_CONSISTENT_NOT_EXECUTION_AUTHORITY"
    assert not actual["owner_observation_authentication_implemented"]
    assert not actual["kernel_generation_or_lease_or_listener_verified_by_helper"]
    assert not actual["shipping_binary_bytes_verified_by_helper"]
    assert not actual["first_RPC_performed"] and not actual["simulator_action_performed"]
    assert not actual["TRAIN_admitted"] and not actual["hd_map_approved"]
    assert not actual["actual_CARLA_capture_claimed"]
    assert "NOT_authenticated" in actual["evidence_origin"]
    assert actual["observed"] == observed and actual["observed"] is not observed


@pytest.mark.parametrize("role", ["owner", "collector", "server"])
@pytest.mark.parametrize("field", ["pid", "start_ticks", "uid", "ppid", "pgid", "session"])
def test_each_generation_component_drift_denies(role, field):
    declared, observed, _ = fixture(); observed[role+"_generation"][field] += 1
    assert role+"_generation_drift_or_invalid" in first(declared, observed)["failures"]


@pytest.mark.parametrize("role", ["owner", "collector", "server"])
@pytest.mark.parametrize("field,bad", [("boot_id", "87654321-1234-1234-1234-123456789abc"),
    ("argv", ["/usr/bin/foreign"]), ("pid", True), ("start_ticks", "3000"),
    ("uid", -1), ("session", 0), ("ppid", None)])
def test_generation_wrong_type_or_boot_or_argv(role, field, bad):
    declared, observed, _ = fixture(); observed[role+"_generation"][field] = bad
    assert first(declared, observed)["failures"]


def test_process_generation_exact_fields_and_isolated_direct_children():
    declared, observed, _ = fixture(); observed["server_generation"]["extra"] = "not sealed"
    assert first(declared, observed)["failures"]
    declared, observed, _ = fixture(); declared["server_generation"]["ppid"] = 250
    assert "server_direct_owner_child_required" in first(declared, observed)["failures"]
    declared, observed, _ = fixture(); declared["server_generation"]["session"] = 250
    assert "server_isolated_session_required" in first(declared, observed)["failures"]


@pytest.mark.parametrize("field,bad", [("path", "/tmp/autoware_e2e_runtime.lock"), ("fd", 11), ("dev", 9),
    ("ino", 124), ("owner_pid", 101), ("owner_start_ticks", 1001), ("collector_pid", 201),
    ("collector_start_ticks", 2001), ("collector_ppid", 50), ("collector_fd", 12),
    ("collector_dev", 9), ("collector_ino", 124), ("flock_verified", False), ("collector_ppid", True)])
def test_inherited_lease_must_match_every_provided_component(field, bad):
    declared, observed, _ = fixture(); observed["lease"][field] = bad
    assert "inherited_lease_parent_FD_inode_or_lock_claim_differs" in first(declared, observed)["failures"]


@pytest.mark.parametrize("path", ["/", "relative", "/home/a/autoware_e2e/../other/CarlaUE4-Linux-Shipping",
    "/home/a//CarlaUE4-Linux-Shipping", "/home/a/CarlaUE4-Linux-Shipping/", "/home/a/UE4Editor"])
def test_declared_shipping_is_exact_canonical_binary_path(path):
    declared, observed, _ = fixture(); declared["shipping_path"] = path
    assert first(declared, observed)["failures"]


@pytest.mark.parametrize("field,bad", [("path", "/tmp/CarlaUE4-Linux-Shipping"), ("sha256", "b"*64),
    ("sha256", "G"*64), ("path", None)])
def test_observed_binary_path_or_source_hash_drift(field, bad):
    declared, observed, _ = fixture(); observed["shipping"][field] = bad
    assert "shipping_binary_path_or_source_drift" in first(declared, observed)["failures"]


@pytest.mark.parametrize("field,bad", [("protocol", "udp"), ("address", "0.0.0.0"), ("port", 2601),
    ("port", True), ("state", "ESTABLISHED"), ("inode", 999), ("shipping_fd_inodes", []),
    ("shipping_fd_inodes", [987,987]), ("shipping_fd_inodes", [True]), ("pid", 301),
    ("start_ticks", 3001), ("boot_id", "87654321-1234-1234-1234-123456789abc"),
    ("network_namespace", "net:[99999]")])
def test_listener_binds_inode_port_and_exact_shipping_generation(field, bad):
    declared, observed, _ = fixture(); observed["listener"][field] = bad
    assert first(declared, observed)["failures"]


def test_first_RPC_boundary_does_not_accept_already_observed_world_or_actors():
    declared, observed, events = fixture()
    actual = subject.assess_boundary(declared, observed, stage="before_first_rpc", live_actors=census(events))
    assert "first_RPC_boundary_must_precede_actor_or_world_observations" in actual["failures"]


def test_before_create_empty_world_is_provided_census_only():
    declared, observed, _ = fixture(); receipt = first(declared, observed)
    actual = subject.assess_boundary(declared, observed, stage="before_actor_creation", first_rpc_receipt=receipt,
        creation_events=[], live_actors=[])
    assert actual["failures"] == [] and actual["registry"]["UNEXECUTED"] == 4
    assert not actual["simulator_action_performed"]


@pytest.mark.parametrize("stage", ["before_actor_creation", "before_tick", "before_cleanup"])
def test_later_boundary_requires_first_RPC_binding(stage):
    declared, observed, events = fixture()
    actual = subject.assess_boundary(declared, observed, stage=stage, creation_events=events, live_actors=census(events))
    assert "sealed_first_RPC_receipt_binding_missing_or_failed" in actual["failures"]
    assert actual["cleanup_candidate_ids"] == []


def test_changed_declaration_cannot_reuse_old_first_receipt():
    declared, observed, events = fixture(); receipt = first(declared, observed)
    declared["port"] = observed["listener"]["port"] = 2601
    actual = subject.assess_boundary(declared, observed, stage="before_tick", first_rpc_receipt=receipt,
        creation_events=events, live_actors=census(events))
    assert "sealed_first_RPC_receipt_binding_missing_or_failed" in actual["failures"]


@pytest.mark.parametrize("collision", [False, True])
def test_complete_exact_created_actor_registry(collision):
    _, _, events, _, actual = boundary(collision=collision)
    assert actual["failures"] == [] and actual["registry"]["complete"]
    assert actual["provided_actual_created_ids"] == [e["actor"]["id"] for e in events]
    assert actual["cleanup_candidate_ids"] == []


@pytest.mark.parametrize("field,bad", [("id", True), ("id", 0), ("id", 1), ("type_id", "sensor.camera.depth"),
    ("parent_id", 999), ("parent_id", True), ("role_name", "foreign")])
def test_created_actor_identity_and_unique_registry(field, bad):
    declared, observed, events = fixture(); events[1]["actor"][field] = bad
    actual = subject.assess_boundary(declared, observed, stage="before_cleanup", first_rpc_receipt=first(declared, observed),
        creation_events=events, live_actors=census(events))
    assert actual["failures"] and actual["cleanup_candidate_ids"] == []
    expected_id = bad if field == "id" else 2
    assert actual["uncleaned_actual_created_ids"][1] == expected_id


def test_partial_creation_failure_retained_cleanup_candidates_not_completion():
    declared, observed, events = fixture()
    partial = events[:2] + [dict(ordinal=2, role="depth", status="failed", actor=None, error="fictional spawn failed")]
    receipt = first(declared, observed)
    actual = subject.assess_boundary(declared, observed, stage="before_cleanup", first_rpc_receipt=receipt,
        creation_events=partial, live_actors=census(partial))
    assert actual["failures"] == [] and actual["registry"]["failed_roles"] == ["depth"]
    assert actual["registry"]["UNEXECUTED"] == 1 and not actual["registry"]["complete"]
    assert actual["cleanup_candidate_ids"] == [2,1]
    assert actual["uncleaned_actual_created_ids"] == [1,2] and not actual["cleanup_performed"]
    tick = subject.assess_boundary(declared, observed, stage="before_tick", first_rpc_receipt=receipt,
        creation_events=partial, live_actors=census(partial))
    assert "tick_requires_complete_created_rig" in tick["failures"]


def test_post_creation_failure_attempt_and_extra_actor_remain_visible():
    declared, observed, events = fixture()
    events[1] = dict(ordinal=1, role="rgb", status="failed", actor=None, error="fictional failure")
    actual = subject.assess_boundary(declared, observed, stage="before_cleanup", first_rpc_receipt=first(declared, observed),
        creation_events=events, live_actors=census(events))
    assert actual["failures"] and actual["cleanup_candidate_ids"] == []
    declared, observed, events = fixture()
    events += [dict(ordinal=4, role="foreign", status="created", actor=dict(id=999, type_id="sensor.camera.rgb",
        parent_id=1, role_name=None), error=None)]
    actual = subject.assess_boundary(declared, observed, stage="before_cleanup", first_rpc_receipt=first(declared, observed),
        creation_events=events, live_actors=census(events))
    assert "extra_creation_events_retained" in actual["failures"]
    assert actual["uncleaned_actual_created_ids"] == [1,2,3,4,999]
    assert actual["registry"]["UNEXECUTED"] == 0 and actual["cleanup_candidate_ids"] == []


def test_failed_attempt_supplied_actor_ID_is_retained_as_ambiguous_witness_not_candidate():
    declared, observed, events = fixture(); partial = events[:2]
    partial += [dict(ordinal=2, role="depth", status="failed", actor=dict(id=999), error="fictional identity failure")]
    actual = subject.assess_boundary(declared, observed, stage="before_cleanup", first_rpc_receipt=first(declared, observed),
        creation_events=partial, live_actors=census(events)[:2])
    assert actual["failures"] and actual["cleanup_candidate_ids"] == []
    assert actual["provided_actor_ID_witnesses"][-1] == dict(event_index=2, supplied_status="failed", supplied_actor_id=999)
    assert actual["creation_events"][-1]["actor"]["id"] == 999


@pytest.mark.parametrize("stage", ["before_tick", "before_cleanup"])
def test_same_PID_new_generation_denies_every_action_and_retains_all_created_ids(stage):
    declared, observed, events = fixture(); receipt = first(declared, observed)
    observed["server_generation"]["start_ticks"] += 1
    actual = subject.assess_boundary(declared, observed, stage=stage, first_rpc_receipt=receipt,
        creation_events=events, live_actors=census(events))
    assert "server_generation_drift_or_invalid" in actual["failures"]
    assert actual["cleanup_policy"] == "DENY_RETAIN_ALL_CREATED_IDS"
    assert actual["uncleaned_actual_created_ids"] == [1,2,3,4] and actual["cleanup_candidate_ids"] == []


def test_cleanup_census_subset_does_not_invent_successful_destroy():
    declared, observed, events = fixture()
    actual = subject.assess_boundary(declared, observed, stage="before_cleanup", first_rpc_receipt=first(declared, observed),
        creation_events=events, live_actors=census(events)[1:])
    assert actual["failures"] == [] and actual["cleanup_candidate_ids"] == [4,3,2]
    assert actual["missing_created_actor_ids"] == [1] and actual["uncleaned_actual_created_ids"] == [1,2,3,4]
    assert not actual["missing_actor_destruction_claimed"]


@pytest.mark.parametrize("mutation", ["foreign", "duplicate", "changed_parent", "missing"])
def test_capture_census_refuses_foreign_changed_missing_or_duplicate_actors(mutation):
    declared, observed, events = fixture(); live = census(events)
    if mutation == "foreign":
        live[1]["actor"]["id"] = 999
    elif mutation == "duplicate":
        live[1] = deepcopy(live[0])
    elif mutation == "changed_parent":
        live[1]["actor"]["parent_id"] = True
    else:
        live.pop()
    actual = subject.assess_boundary(declared, observed, stage="before_tick", first_rpc_receipt=first(declared, observed),
        creation_events=events, live_actors=live)
    assert actual["failures"] and actual["cleanup_candidate_ids"] == []


def test_received_metadata_is_not_world_or_sensor_header_approval():
    actual = receive_result()
    assert actual["failures"] == [] and actual["expected_camera_attempts"] == actual["received_header_records"] == 3
    assert actual["UNEXECUTED_camera_attempts"] == 0
    assert not actual["world_snapshot_correctness_verified"] and not actual["sensor_header_correctness_verified"]
    assert actual["required_separate_header_helper"].endswith("20261008_v2")
    assert not actual["partial_raw_training_eligible"]


@pytest.mark.parametrize("failed_slot", [0,1,2])
def test_partial_camera_timeout_keeps_failure_and_unexecuted_slots(failed_slot):
    _, _, events, _, _ = boundary(); rows = receives(events)[:failed_slot+1]
    rows[-1].update(status="timeout", observed_header=None, error="fictional timeout")
    actual = receive_result(rows)
    assert actual["failures"] and actual["received_header_records"] == failed_slot
    assert actual["failed_camera_attempts"] == 1 and actual["UNEXECUTED_camera_attempts"] == 2-failed_slot
    assert actual["receive_events"] == rows and actual["receive_events"][-1]["observed_header"] is None
    assert not actual["missing_headers_fabricated"] and not actual["TRAIN_admitted"]


def test_partial_receive_error_keeps_actual_wrong_frame_header():
    _, _, events, _, _ = boundary(); rows = receives(events)[:2]
    rows[-1].update(status="error", observed_header=dict(frame=43, timestamp_s=float("nan")), error="fictional skipped frame")
    actual = receive_result(rows)
    assert actual["receive_events"][-1]["observed_header"]["frame"] == 43
    assert actual["receive_events"][-1]["observed_header"]["timestamp_s"]["accepted"] is False
    assert actual["UNEXECUTED_camera_attempts"] == 1 and actual["failures"]


@pytest.mark.parametrize("field,bad", [("actor_id", 999), ("actor_id", True), ("role", "semantic"),
    ("ordinal", True), ("required_frame", 43), ("status", "invented")])
def test_camera_receive_identity_mismatch_retained(field, bad):
    _, _, events, _, _ = boundary(); rows = receives(events); rows[0][field] = bad
    actual = receive_result(rows)
    assert actual["failures"] and actual["receive_events"][0][field] == bad


def test_extra_camera_attempt_is_not_negative_unexecuted_or_hidden():
    _, _, events, _, _ = boundary(); rows = receives(events)
    rows += [deepcopy(rows[-1])]
    actual = receive_result(rows)
    assert "extra_camera_attempts_retained_outside_declared_denominator" in actual["failures"]
    assert actual["UNEXECUTED_camera_attempts"] == 0 and actual["extra_camera_attempt_count"] == 1
    assert len(actual["receive_events"]) == actual["actual_provided_camera_attempt_count"] == 4


def test_receive_registry_cannot_replace_prior_tick_actors():
    declared, _, events, _, tick = boundary(); events[1]["actor"]["id"] = 999
    actual = subject.assess_receive(declared, tick, events, ordinal=0, required_frame=42,
        world_snapshot=dict(frame=42), receive_events=receives(events))
    assert "prior_tick_boundary_binding_missing_or_failed" in actual["failures"]


def test_no_world_snapshot_is_inferred_from_received_camera_times():
    declared, _, events, _, tick = boundary()
    actual = subject.assess_receive(declared, tick, events, ordinal=0, required_frame=42,
        world_snapshot=None, receive_events=receives(events))
    assert "actual_world_snapshot_evidence_missing" in actual["failures"]
    assert actual["actual_world_snapshot"] is None and not actual["world_snapshot_correctness_verified"]


def test_oversized_receive_metadata_retains_count_and_unknown_schedule_not_fake_UNEXECUTED3():
    _, _, events, _, _ = boundary(); rows = receives(events)[:1]*65
    actual = receive_result(rows)
    assert actual["failures"] and actual["actual_provided_camera_attempt_count"] == 65
    assert actual["UNEXECUTED_camera_attempts"] == "UNKNOWN_UNRETAINED_ATTEMPT_DETAILS"
    assert actual["receive_events"]["metadata_not_retained"]


@pytest.mark.parametrize("bad", ["", "net:[]", "net:[0]", "net:[01]", "net:[True]", "net:[-1]", "net:[18446744073709551616]", None])
def test_network_namespace_requires_exact_positive_inode_not_freeform_label(bad):
    declared, observed, _ = fixture(); declared["network_namespace"] = bad
    assert "declared_network_namespace_required" in first(declared, observed)["failures"]


@pytest.mark.parametrize("role", ["server", "collector"])
def test_direct_child_generation_cannot_predate_owner(role):
    declared, observed, _ = fixture(); declared[role+"_generation"]["start_ticks"] = 999
    assert role+"_cannot_predate_owner" in first(declared, observed)["failures"]


@pytest.mark.parametrize("bad", [None, [], {}, True, float("nan")])
def test_non_dictionary_observation_is_retained_and_denied(bad):
    declared, _, _ = fixture(); actual = first(declared, bad)
    assert "exact_fresh_owner_observation_fields_required" in actual["failures"]
    assert actual["observed"] == subject.retain(bad)


def test_unknown_nonfinite_boundary_retained_without_invalid_JSON_scalar():
    declared, observed, events = fixture()
    actual = subject.assess_boundary(declared, observed, stage=float("nan"), first_rpc_receipt=first(declared, observed),
        creation_events=events, live_actors=census(events))
    assert "unknown_owner_boundary" in actual["failures"] and actual["stage"]["accepted"] is False


def test_malformed_metadata_retains_unknown_not_fake_empty_or_unbounded_data():
    declared, observed, events = fixture()
    actual = subject.assess_boundary(declared, observed, stage="before_cleanup", first_rpc_receipt=first(declared, observed),
        creation_events=None, live_actors=None)
    assert actual["live_actors"] is None and actual["registry"]["UNEXECUTED"] == "UNKNOWN"
    assert actual["uncleaned_actual_created_ids"]["ids_retained"] is False
    assert subject.retain([0]*65)["metadata_not_retained"]
    assert subject.retain("x"*5000)["truncated"]
    assert subject.retain(2**2048)["accepted"] is False
    cyclic = []; cyclic.append(cyclic)
    assert "bound_exceeded" in str(subject.retain(cyclic))


@pytest.mark.parametrize("role_bad", [None, True, [], {}, "foreign"])
def test_malformed_actor_spec_role_fails_without_lookup_error(role_bad):
    declared, observed, _ = fixture(); declared["actor_specs"][1]["role"] = role_bad
    assert first(declared, observed)["failures"]


def test_only_stdlib_and_no_actions_are_present_in_helper_source():
    import ast
    from pathlib import Path
    tree = ast.parse(Path(subject.__file__).read_bytes())
    imports = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    imports |= {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert imports <= {"__future__", "hashlib", "json", "math", "pathlib"}
    forbidden = {"open", "read_text", "read_bytes", "write_text", "write_bytes", "unlink", "rmdir", "kill", "killpg",
                 "Popen", "run", "connect", "Client", "tick", "spawn_actor", "destroy", "stop", "flock", "stat", "fstat"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            name = node.func.id if isinstance(node.func, ast.Name) else node.func.attr if isinstance(node.func, ast.Attribute) else None
            assert name not in forbidden
