"""HH_261008 - Prepare fictional callback wiring tests without live simulator or storage authority.

SOURCE_READY for ROOT review; authoring this file does not run any fixture. Its
in-memory byte buffers are synthetic BGRA/encoded stand-ins, not image arrays,
actual captures, authentic storage artifacts or evidence of a driving capability.
"""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from types import SimpleNamespace

import pytest

from portable_e2e import mapping_capture_adapter_20261008_v1 as subject
from portable_e2e import mapping_capture_ownership_20261008_v2 as ownership


def owner_fixture():
    boot = "12345678-1234-1234-1234-123456789abc"
    shipping = "/fictional/carla/CarlaUE4-Linux-Shipping"
    def generation(pid, parent, argv):
        return dict(pid=pid, start_ticks=pid*10, boot_id=boot, uid=1000,
                    argv=argv, ppid=parent, pgid=pid, session=pid)
    owner = generation(100, 50, ["/usr/bin/python3", "fictional_owner.py"])
    collector = generation(200, 100, ["/usr/bin/python3", "fictional_collector.py"])
    server = generation(300, 100, [shipping])
    roles = ["ego", "rgb", "depth", "semantic"]
    specs = [dict(role=role, type_id=ownership.TYPES[role], parent_role=None if role == "ego" else "ego",
                  role_name="fictional_owned_mapping" if role == "ego" else None) for role in roles]
    declared = dict(schema=ownership.DECLARATION_SCHEMA, owner_generation=owner, collector_generation=collector,
        server_generation=server, shipping_path=shipping, shipping_sha256="a"*64, host="127.0.0.1", port=2600,
        network_namespace="net:[12345]", lease=dict(path=ownership.LOCK, fd=10, dev=8, ino=123), actor_specs=specs)
    observed = dict(owner_generation=deepcopy(owner), collector_generation=deepcopy(collector),
        server_generation=deepcopy(server), shipping=dict(path=shipping, sha256="a"*64),
        lease=dict(path=ownership.LOCK, fd=10, dev=8, ino=123, owner_pid=100, owner_start_ticks=1000,
        collector_pid=200, collector_start_ticks=2000, collector_ppid=100, collector_fd=10,
        collector_dev=8, collector_ino=123, flock_verified=True),
        listener=dict(protocol="tcp", address="127.0.0.1", port=2600, state="LISTEN", inode=987,
        pid=300, start_ticks=3000, boot_id=boot, network_namespace="net:[12345]", shipping_fd_inodes=[986, 987]))
    return declared, observed


class FictionalCollector:
    """HH_261008 - Track each provided action; no file, kernel, RPC or numeric-array operations."""
    def __init__(self):
        self.declared, self.observed = owner_fixture()
        self.first = ownership.assess_boundary(self.declared, self.observed, stage="before_first_rpc")
        self.actors, self.calls, self.provider_actions, self.saved = {}, [], [], {}
        self.frame = 41
        self.pose = [[1., 0., 0., 1.5], [0., 1., 0., 0.], [0., 0., 1., 2.], [0., 0., 0., 1.]]
        # HH_261008 - One fixed byte buffer is reused; no decoder or array library is imported.
        self.raw = bytes(640*360*4)
        self.images = []
        self.provider_mutation = None

    def provider(self, action):
        self.provider_actions.append(action)
        observed = deepcopy(self.observed)
        if self.provider_mutation is not None:
            self.provider_mutation(action, observed)
        live = [dict(role=role, actor=deepcopy(handle.identity)) for role, handle in self.actors.items()]
        return dict(observed=observed, live_actors=live)

    def create(self, spec, parent):
        self.calls.append("create:"+spec["role"])
        identity = dict(id=len(self.actors)+1, type_id=spec["type_id"],
                        parent_id=None if parent is None else parent.id, role_name=spec["role_name"])
        actor = SimpleNamespace(id=identity["id"], type_id=identity["type_id"])
        handle = subject.CreatedActor(actor, identity)
        self.actors[spec["role"]] = handle
        return handle

    def tick(self):
        self.calls.append("tick")
        self.frame += 1
        snapshot = SimpleNamespace(frame=self.frame,
            timestamp=SimpleNamespace(elapsed_seconds=self.frame/10, delta_seconds=.1))
        return subject.TickResult(self.frame, snapshot)

    def receive(self, role, actor, frame):
        self.calls.append("receive:"+role)
        pose = deepcopy(self.pose)
        image = SimpleNamespace(frame=frame, timestamp=frame/10, width=640, height=360, raw_data=self.raw,
                                transform=SimpleNamespace(get_matrix=lambda: deepcopy(pose)))
        self.images.append(image)
        return image

    def write(self, ordinal, snapshot, actors, images, headers):
        self.calls.append("write")
        assert tuple(actor.id for actor in actors) == (2, 3, 4)
        assert all(image.frame == snapshot.frame for image in images)
        assert headers["observed"]["sensors"][0]["measurement"]["raw_BGRA_sha256"] == hashlib.sha256(self.raw).hexdigest()
        artifacts = []
        for kind, roles, extension, content in (
            ("rgb_png", ["rgb"], "png", b"fictional encoded RGB bytes"),
            ("oracle_npz", ["depth", "semantic"], "npz", b"fictional encoded oracle bytes")):
            path = "frames/"+str(ordinal).zfill(4)+"."+extension
            self.saved[path] = content
            artifacts.append(dict(kind=kind, roles=roles, path=path, size_bytes=len(content),
                                  sha256=hashlib.sha256(content).hexdigest()))
        return artifacts

    def read(self, receipt):
        self.calls.append("read:"+receipt["kind"])
        return self.saved[receipt["path"]]

    def stop(self, role, actor):
        self.calls.append("stop:"+role)
        return True

    def destroy(self, role, actor):
        self.calls.append("destroy:"+role)
        del self.actors[role]
        return True

    def callbacks(self, *, read=True):
        return subject.Callbacks(create_actor=self.create, tick=self.tick, receive=self.receive,
            write_artifacts=self.write, destroy_actor=self.destroy, stop_sensor=self.stop,
            read_artifact=self.read if read else None)

    def run(self, *, frames=2, callbacks=None, metadata_bytes=subject.MAX_METADATA_BYTES):
        return subject.run_capture(self.declared, self.first, self.provider,
            self.callbacks() if callbacks is None else callbacks,
            limits=subject.Limits(frames, metadata_bytes=metadata_bytes))


def assert_denominator(result, expected, attempted, passed, failed):
    assert result["expected_frames"] == expected
    assert result["attempted_frame_slots"] == attempted
    assert result["passed_frames"] == passed
    assert result["failed_frames"] == failed
    assert result["UNEXECUTED_frames"] == expected-attempted
    assert passed+failed == attempted
    assert (result["received_camera_headers"]+result["failed_camera_attempts"]
            +result["UNEXECUTED_camera_slots"]) == expected*3


def test_complete_injected_wiring_is_nontraining_and_not_storage_authentication():
    fixture = FictionalCollector()
    result = fixture.run()
    assert result["primary_failure"] is None and result["cleanup_errors"] == []
    assert result["uncleaned_actor_ids"] == [] and fixture.actors == {}
    assert_denominator(result, 2, 2, 2, 0)
    assert result["actual_tick_callback_attempts"] == 2
    assert result["actual_receive_callback_attempts"] == 6
    assert result["actual_artifact_read_callback_attempts"] == 4
    assert result["actual_stop_callback_attempts"] == 3 and result["actual_destroy_callback_attempts"] == 4
    assert len(result["observations"]) < len(result["boundaries"])
    for frame in result["frames"]:
        assert frame["provided_headers_unchanged_after_artifact_callbacks"]
        assert all(check["matches_callback_receipt"] for check in frame["artifact_read_verification"])
        assert all(not check["storage_identity_authenticated"] for check in frame["artifact_read_verification"])
        assert not frame["header_receipt"]["artifact_byte_hash_verification_by_helper"]
    for key in ("first_RPC_performed_by_adapter", "provider_freshness_authenticated", "first_receipt_seal_authenticated",
        "kernel_ownership_or_file_identity_authenticated", "artifact_storage_identity_authenticated", "TRAIN_admitted",
        "actual_CARLA_capture_claimed", "learned_control", "failed_raw_capture_newly_TRAIN_eligible", "hd_map_approved"):
        assert result[key] is False
    encoded = json.dumps(result, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    assert len(encoded) == result["metadata_bytes"] <= 8*1024*1024


def test_callback_hash_claims_without_read_buffer_remain_explicitly_unverified():
    fixture = FictionalCollector()
    result = fixture.run(frames=1, callbacks=fixture.callbacks(read=False))
    assert result["frames"][0]["artifact_read_verification"] == "NOT_REQUESTED_CALLBACK_RECEIPTS_ONLY"
    assert result["actual_artifact_read_callback_attempts"] == 0
    assert not result["artifact_storage_identity_authenticated"]


def test_original_RPC_socket_replacement_denies_receive_and_cleanup_effects():
    fixture = FictionalCollector()
    def replace_socket(action, observed):
        if action.startswith("receive:") or action.startswith("cleanup:"):
            observed["listener"]["inode"] = 988
            observed["listener"]["shipping_fd_inodes"] = [986, 988]
    fixture.provider_mutation = replace_socket
    result = fixture.run(frames=3)
    assert_denominator(result, 3, 1, 0, 1)
    assert result["actual_tick_callback_attempts"] == 1 and result["actual_receive_callback_attempts"] == 0
    assert result["actual_destroy_callback_attempts"] == 0 and result["actual_stop_callback_attempts"] == 0
    assert result["uncleaned_actor_ids"] == [1, 2, 3, 4]
    assert "original_first_RPC_listener_identity_drift" in result["primary_failure"]["failures"]
    assert result["frames"][0]["receive_receipt"]["UNEXECUTED_camera_attempts"] == 3
    assert result["frames"][0]["header_receipt"] is None


def test_native_actor_identity_mismatch_is_retained_and_never_destroyed_by_claimed_id():
    fixture = FictionalCollector()
    base = fixture.create
    def wrong_native(spec, parent):
        handle = base(spec, parent)
        if spec["role"] == "rgb":
            handle.actor.id = 999
        return handle
    result = fixture.run(frames=1, callbacks=replace(fixture.callbacks(), create_actor=wrong_native))
    assert result["creation_events"][1]["status"] == "created"
    assert result["creation_events"][1]["actor"]["id"] == 2
    assert result["native_created_actor_witnesses"][1]["id"] == 999
    assert result["actual_destroy_callback_attempts"] == 0
    assert result["actual_stop_callback_attempts"] == 0
    assert result["uncleaned_actor_ids"] == [1, 2]
    assert result["UNEXECUTED_frames"] == 1


@pytest.mark.parametrize("process", ["owner", "collector", "server"])
def test_generation_change_before_first_create_denies_every_effect(process):
    fixture = FictionalCollector()
    def drift(action, observed):
        observed[process+"_generation"]["start_ticks"] += 1
    fixture.provider_mutation = drift
    result = fixture.run()
    assert not fixture.calls and result["creation_events"] == []
    assert not result["callback_effects_attempted"]
    assert_denominator(result, 2, 0, 0, 0)
    assert process+"_generation_drift_or_invalid" in result["primary_failure"]["failures"]


def test_receive_failure_retains_partial_headers_and_unexecuted_tail():
    fixture = FictionalCollector()
    def fail_depth(role, actor, frame):
        if role == "depth":
            raise TimeoutError("fictional native depth deadline")
        return fixture.receive(role, actor, frame)
    result = fixture.run(frames=5, callbacks=replace(fixture.callbacks(), receive=fail_depth))
    assert_denominator(result, 5, 1, 0, 1)
    assert result["actual_receive_callback_attempts"] == 2
    frame = result["frames"][0]
    assert [event["role"] for event in frame["receive_events"]] == ["rgb", "depth"]
    assert frame["receive_events"][1]["observed_header"] is None
    assert frame["receive_events"][1]["status"] == "timeout"
    assert frame["receive_receipt"]["UNEXECUTED_camera_attempts"] == 1
    assert frame["header_receipt"] is None and frame["saved_artifacts"] == []
    assert result["actual_artifact_write_callback_attempts"] == 0
    assert result["uncleaned_actor_ids"] == []


def test_partial_writer_failure_retains_actual_callback_artifact_refs_and_stops_future_frames():
    fixture = FictionalCollector()
    def fail_writer(*args):
        refs = fixture.write(*args)
        raise subject.ArtifactCallbackFailure("fictional NPZ failure after RGB save", artifacts=refs[:1])
    result = fixture.run(frames=4, callbacks=replace(fixture.callbacks(), write_artifacts=fail_writer))
    assert_denominator(result, 4, 1, 0, 1)
    frame = result["frames"][0]
    assert len(frame["saved_artifacts"]) == 1 and frame["saved_artifacts"][0]["kind"] == "rgb_png"
    assert not frame["artifact_callback_partial_outputs_unknown"]
    assert result["actual_artifact_read_callback_attempts"] == 0
    assert result["primary_failure"]["error"]["type"] == "ArtifactCallbackFailure"


def test_unknown_partial_writer_failure_is_not_claimed_artifact_absence():
    fixture = FictionalCollector()
    def fail_writer(*args):
        raise RuntimeError("fictional unknown partial storage effect")
    result = fixture.run(frames=1, callbacks=replace(fixture.callbacks(), write_artifacts=fail_writer))
    assert result["frames"][0]["artifact_callback_partial_outputs_unknown"]
    assert result["frames"][0]["saved_artifacts"] == []


def test_artifact_read_hash_failure_retains_the_actual_provided_buffer_hash():
    fixture = FictionalCollector()
    result = fixture.run(frames=2, callbacks=replace(fixture.callbacks(), read_artifact=lambda receipt: b"different bytes"))
    assert_denominator(result, 2, 1, 0, 1)
    check = result["frames"][0]["artifact_read_verification"][0]
    assert check["actual_provided_sha256"] == hashlib.sha256(b"different bytes").hexdigest()
    assert not check["matches_callback_receipt"] and not check["storage_identity_authenticated"]


def test_cleanup_errors_preserve_original_capture_failure_and_attempt_every_owned_actor():
    fixture = FictionalCollector()
    def fail_receive(role, actor, frame):
        raise TimeoutError("primary receive failure")
    def fail_destroy(role, actor):
        fixture.calls.append("destroy:"+role)
        raise RuntimeError("secondary cleanup failure:"+role)
    result = fixture.run(frames=2, callbacks=replace(fixture.callbacks(), receive=fail_receive, destroy_actor=fail_destroy))
    assert result["primary_failure"]["error"]["type"] == "TimeoutError"
    assert "primary receive failure" in result["primary_failure"]["error"]["repr"]
    assert len(result["cleanup_errors"]) == 4 and result["uncleaned_actor_ids"] == [1, 2, 3, 4]
    assert result["actual_destroy_callback_attempts"] == 4
    assert [call for call in fixture.calls if call.startswith("destroy:")] == [
        "destroy:semantic", "destroy:depth", "destroy:rgb", "destroy:ego"]


def test_cleanup_stop_and_destroy_each_have_their_own_fresh_boundary():
    fixture = FictionalCollector()
    result = fixture.run(frames=1)
    for role in ("rgb", "depth", "semantic"):
        assert "cleanup:stop:"+role in fixture.provider_actions
        assert "cleanup:destroy:"+role in fixture.provider_actions
    assert result["cleanup_errors"] == []


def test_disappeared_actor_is_not_reported_destroyed_or_clean():
    fixture = FictionalCollector()
    base = fixture.provider
    def missing(action):
        supplied = base(action)
        if action.startswith("cleanup:"):
            supplied["live_actors"] = [row for row in supplied["live_actors"] if row["role"] != "depth"]
        return supplied
    result = subject.run_capture(fixture.declared, fixture.first, missing, fixture.callbacks(), limits=subject.Limits(1))
    assert result["uncleaned_actor_ids"] == [3]
    assert "destroy:depth" not in fixture.calls
    assert result["primary_failure"]["boundary_stage"] == "cleanup"


def test_native_buffer_mutation_during_writer_is_failure_with_saved_refs_retained():
    fixture = FictionalCollector()
    def mutating_writer(*args):
        refs = fixture.write(*args)
        fixture.images[0].raw_data = b"x"*(640*360*4)
        return refs
    result = fixture.run(frames=2, callbacks=replace(fixture.callbacks(), write_artifacts=mutating_writer))
    assert_denominator(result, 2, 1, 0, 1)
    frame = result["frames"][0]
    assert not frame["provided_headers_unchanged_after_artifact_callbacks"]
    assert len(frame["saved_artifacts"]) == 2
    assert "native_headers_or_raw_buffers_changed_during_artifact_callbacks" in result["primary_failure"]["failures"]


@pytest.mark.parametrize("delta,expected", [(2e-5, True), (4e-5, False)])
def test_native_pose_matches_independent_scalar_allclose_formula(delta, expected):
    fixture = FictionalCollector()
    base = fixture.receive
    def pose_receive(role, actor, frame):
        image = base(role, actor, frame)
        matrix = deepcopy(fixture.pose)
        if role == "depth":
            matrix[0][3] += delta
        image.transform = SimpleNamespace(get_matrix=lambda: deepcopy(matrix))
        return image
    # HH_261008 - The original default rtol is explicitly evaluated independently of helper constants.
    reference, actual = 1.5, 1.5+delta
    independently_close = abs(reference-actual) <= 1e-5 + 1e-5*abs(actual)
    assert independently_close is expected
    result = fixture.run(frames=1, callbacks=replace(fixture.callbacks(), receive=pose_receive))
    assert (result["passed_frames"] == 1) is expected
    assert result["actual_artifact_write_callback_attempts"] == (1 if expected else 0)
    if not expected:
        assert "sensor_pose_mismatch" in result["frames"][0]["header_receipt"]["failures"]


@pytest.mark.parametrize("frames", [True, 0, 1210, 1.0])
def test_declared_limit_failure_precedes_provider_or_effect(frames):
    fixture = FictionalCollector()
    with pytest.raises(ValueError):
        fixture.run(frames=frames)
    assert fixture.provider_actions == [] and fixture.calls == []


def test_first_receipt_tampering_precedes_provider_and_callbacks():
    fixture = FictionalCollector()
    fixture.first["first_RPC_performed"] = True
    result = fixture.run(frames=1)
    assert not fixture.provider_actions and not fixture.calls
    assert result["UNEXECUTED_frames"] == 1 and not result["first_RPC_performed_by_adapter"]


def test_metadata_budget_failure_preserves_denominator_last_attempt_and_cleanup():
    fixture = FictionalCollector()
    result = fixture.run(frames=1209, metadata_bytes=128*1024)
    assert result["primary_failure"] is not None
    assert result["failed_frames"] == 1
    assert result["passed_frames"]+result["failed_frames"] == result["attempted_frame_slots"]
    assert result["attempted_frame_slots"]+result["UNEXECUTED_frames"] == 1209
    assert result["frames"][-1]["status"] == "FAILED_RETAINED"
    assert result["uncleaned_actor_ids"] == [] and fixture.actors == {}
    assert result["metadata_bytes"] <= 128*1024


def test_source_imports_and_entrypoints_do_not_implement_live_owner_or_array_decoder():
    import ast
    from pathlib import Path
    tree = ast.parse(Path(subject.__file__).read_bytes())
    imported = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    imported |= {name.name for node in ast.walk(tree) if isinstance(node, ast.Import) for name in node.names}
    assert not imported & {"numpy", "torch", "PIL", "carla", "rclpy", "socket", "subprocess", "os"}
    assert not any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                   and node.func.id in {"open", "exec", "eval"} for node in ast.walk(tree))
