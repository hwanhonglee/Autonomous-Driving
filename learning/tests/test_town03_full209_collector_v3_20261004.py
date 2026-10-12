"""HH_261004 - Check fresh recorder/fixture scopes and scalar source-flow proofs without CARLA.

Tiny files below are explicitly synthetic byte fixtures, not real JPEGs or
driving evidence.  They exercise the verifier and never approve a capability.
"""

from contextlib import contextmanager
import hashlib
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_route_joint_town03_straight_full209_v3_20261004 as m


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@contextmanager
def scalar_scope(*, admitted_owner=None, parent_exception=None, wrong_output=False,
                 legacy_recorder=False, legacy_fixture=False):
    """HH_261004 - Replace only the simulator parent scope; retain the actual helper aliases and ACK closure."""
    old, fixture = m.previous, m.previous.fixture
    collector = old.collector
    before = dict(owner=old.OWNER_SCHEMA, parent_active=old._active,
        fresh_active=m._active, fixture_owner=fixture.OWNER_SCHEMA,
        fixture_schema=fixture.SCHEMA, fixture_class=fixture.GreenSignalFixture,
        save=collector._save_jpeg, transport=collector.AcknowledgedControlTransport)
    observed = []

    @contextmanager
    def parent(model, args):
        assert old.OWNER_SCHEMA == m.OWNER_SCHEMA and m._active
        assert m.jpeg._active and m.signals._active
        assert fixture.GreenSignalFixture is m.signals.TwoPhaseGreenSignalFixture
        old._active = True
        output = args.output.with_name(args.output.name + (".wrong.partial" if wrong_output else ".partial"))
        recording = m.jpeg.original.deferred_recording if legacy_recorder else m.jpeg.parallel_deferred_recording
        try:
            with mock.patch.object(fixture, "OWNER_SCHEMA", admitted_owner or m.OWNER_SCHEMA), \
                    mock.patch.object(fixture, "SCHEMA", m.SCHEMA), recording(collector, output) as recorder:
                observed.append(dict(assert_owner=fixture.assert_owned_capture.__globals__["OWNER_SCHEMA"],
                    constructor_owner=fixture.GreenSignalFixture.__init__.__globals__["OWNER_SCHEMA"],
                    actual_raw_schema=fixture.assert_owned_capture.__globals__["SCHEMA"], recorder=recorder))
                if parent_exception is not None:
                    raise parent_exception
                if legacy_fixture:
                    with mock.patch.object(fixture, "GreenSignalFixture", m.signals.TwoPhaseGreenSignalFixture.__bases__[0]):
                        yield
                else:
                    yield
        finally:
            old._active = before["parent_active"]

    try:
        with mock.patch.object(old, "activated", parent):
            yield observed
    finally:
        assert old.OWNER_SCHEMA == before["owner"] and old._active is before["parent_active"]
        assert m._active is before["fresh_active"]
        assert fixture.OWNER_SCHEMA == before["fixture_owner"] and fixture.SCHEMA == before["fixture_schema"]
        assert fixture.GreenSignalFixture is before["fixture_class"]
        assert collector._save_jpeg is before["save"] and collector.AcknowledgedControlTransport is before["transport"]
        assert not m.jpeg._active and not m.signals._active
        assert not getattr(collector, "_exact_jpeg_after_ACK_active", False)
        assert all(row["recorder"].closed for row in observed if hasattr(row["recorder"], "closed"))


def test_exact_frozen_parent_and_original_capture_ABI():
    assert digest(m.previous.__file__) == m.PARENT_SHA256
    m.verify_frozen()
    assert m.OWNER_SCHEMA.endswith(".v3") and m.SCHEMA.endswith(".v1")
    assert m.generic is m.previous.generic


def test_actual_recorder_ACK_closure_and_both_fixture_globals_are_fresh(tmp_path):
    args = SimpleNamespace(output=tmp_path / "episode")
    with scalar_scope() as observed:
        with m.activated(object(), args):
            row = observed[0]
            assert row["assert_owner"] == row["constructor_owner"] == m.OWNER_SCHEMA
            assert row["actual_raw_schema"] == m.SCHEMA
            assert m.jpeg.assert_runtime_recorder(m.collector) is row["recorder"]
            assert row["recorder"].output == tmp_path / "episode.partial"
            assert m.fixture.GreenSignalFixture is m.signals.TwoPhaseGreenSignalFixture
        assert row["recorder"].closed


@pytest.mark.parametrize("change", ["stale_owner", "wrong_output", "legacy_recorder", "legacy_fixture"])
def test_stale_or_fallback_runtime_provider_rejects_and_restores(tmp_path, change):
    kwargs = {"admitted_owner": "STALE_CPU_ONLY_OWNER"} if change == "stale_owner" else {change: True}
    with scalar_scope(**kwargs) as observed:
        with pytest.raises(RuntimeError):
            with m.activated(object(), SimpleNamespace(output=tmp_path / "episode")):
                pytest.fail("unknown original/fallback providers must not enter actual capture")
        assert len(observed) == 1


@pytest.mark.parametrize("where", ["setup", "body"])
def test_original_exception_identity_and_all_aliases_restore(tmp_path, where):
    marker = RuntimeError("synthetic scope failure before any simulator action")
    with scalar_scope(parent_exception=marker if where == "setup" else None) as observed:
        with pytest.raises(RuntimeError) as caught:
            with m.activated(object(), SimpleNamespace(output=tmp_path / "episode")):
                raise marker
        assert caught.value is marker and len(observed) == 1


def test_nested_runtime_scope_rejects_without_replacing_outer_provider(tmp_path):
    args = SimpleNamespace(output=tmp_path / "episode")
    with scalar_scope() as observed:
        with m.activated(object(), args):
            recorder = m.jpeg.assert_runtime_recorder(m.collector)
            with pytest.raises(RuntimeError, match="cannot nest"):
                with m.activated(object(), args):
                    pytest.fail("nested capture must be denied")
            assert m.jpeg.assert_runtime_recorder(m.collector) is recorder is observed[0]["recorder"]


def flow_fixture(tmp_path, native_count=25):
    """HH_261004 - Build exact scalar bindings and tiny synthetic files; no sensor bytes or real futures are asserted."""
    episode = tmp_path / "episode.partial"
    episode.mkdir()
    states = [dict(frame=100 + index, timestamp=(100 + index) * .05) for index in range(native_count)]
    timings = [dict(ACK_sequence=index + 1, command_frame=100 + index - 2,
        ACK_server_accepted=True, ACK_finished_monotonic_ns=100_000 + max(0, index - 2) * 10_000 + 100)
        for index in range(native_count + 4)]
    callback = dict(callbacks=[], bundles=[], native_stage_marks=[dict(stage="world_tick_started",
        tick_index=index, native_frame=None, stage_finished_monotonic_ns=100_000 + index * 10_000)
        for index in range(native_count)])
    recordings, cameras, owned_sources, raw_pins = [], [], [], {}
    for index in range(0, native_count, 2):
        state = states[index]
        frame, timestamp = state["frame"], state["timestamp"]
        stamp, tick = round(timestamp * 1.e9), 100_000 + index * 10_000
        images, bound, rows, written = {}, [], [], []
        for ordinal, role in enumerate(m.jpeg.CAMERA_ORDER):
            relative = Path("images") / role / f"{frame:08d}.jpg"
            path = episode / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(f"CPU SYNTHETIC FLOW ONLY {frame} {role}".encode())
            encoded_sha = digest(path)
            raw_pins[str(path)] = encoded_sha
            images[role] = str(relative)
            written.append(str(path))
            identity = 10_000 + index * 6 + ordinal
            source_sha = hashlib.sha256(f"synthetic-source-metadata:{frame}:{role}".encode()).hexdigest()
            source = dict(role=role, native_frame=frame, image_object_id=identity,
                source_image_timestamp_simulation_ns=stamp,
                source_BGRA_encoding_prefixed_sha256=source_sha, source_byte_count=640*360*4,
                source_pixels_changed=False, source_timestamp_changed=False)
            bound.append(dict(source))
            callback["callbacks"].append(dict(source))
            rows.append(dict(ordinal=ordinal, role=role, native_frame=frame, image_object_id=identity,
                source_image_timestamp_simulation_ns=stamp, source_byte_count=640*360*4,
                source_BGRA_encoding_prefixed_sha256=source_sha, quality=95, worker_thread_id=500+ordinal%3,
                encoding_failure=None, source_pixels_changed=False, source_timestamp_changed=False,
                encoding_started_monotonic_ns=tick+201, encoding_finished_monotonic_ns=tick+250,
                write_started_monotonic_ns=tick+301+ordinal*2, write_finished_monotonic_ns=tick+302+ordinal*2,
                encoded_JPEG_sha256=encoded_sha, encoded_JPEG_byte_count=path.stat().st_size))
        callback["bundles"].append(dict(native_frame=frame, cameras=bound))
        cameras.append(dict(frame=frame, timestamp=timestamp, camera_order=list(m.jpeg.CAMERA_ORDER),
            source_timestamps={role:timestamp for role in m.jpeg.CAMERA_ORDER}, jpeg_quality=95, images=images))
        owned_sources.append(dict(native_frame=frame, source_image_timestamp_simulation_ns=stamp,
            exact_original_source_bytes_owned=6*640*360*4, immutable_bytes_backing=True,
            source_pixel_values_changed=False, source_SHA_recorded_by_policy=True, source_ownership_started_monotonic_ns=tick+110,
            source_ownership_finished_monotonic_ns=tick+150))
        recordings.append(dict(schema=m.jpeg.original.SCHEMA, frame=frame, ACK_receipt_sequence=index+3,
            recording_started_monotonic_ns=tick+200, recording_finished_monotonic_ns=tick+400,
            acknowledged_before_recording=True, enqueued_camera_count=6, written_camera_count=6,
            failure=None, all_queued_images_written=True, no_source_image_dropped=True,
            synchronous_flush_before_next_tick=True, source_timestamp_changed=False, source_pixels_changed=False,
            written_files=written, parallel_encoding_research=dict(schema=m.jpeg.SCHEMA, workers=3,
                submitted_encode_count=6, all_submitted_encodes_joined=True, original_role_ordered_writes=True,
                create_only_file_descriptors=True, background_work_after_flush=False, sources_or_quality_changed=False,
                controller_or_model_changed=False, all_encodes_joined_monotonic_ns=tick+300, encode_rows=rows)))
    return dict(recordings=recordings, cameras=cameras, owned_sources=owned_sources, callback=callback,
        timings=timings, states=states, episode=episode, raw_pins=raw_pins, digest=digest)


@pytest.mark.parametrize("native_count", [25, 26])
def test_whole_interval_role_order_and_only_observed_successor_native_tick(tmp_path, native_count):
    values = flow_fixture(tmp_path, native_count)
    result = m.audit_parallel_recordings(**values)
    assert result["passed"] and result["actual_camera_bundle_count"] == 13
    assert result["actual_BGRA_and_JPEG_rows"] == 78
    assert result["actual_worker_thread_count"] == 3
    last = result["rows"][-1]
    assert last["next_native_tick_observed"] is (native_count == 26)
    assert (last["next_native_tick_started_monotonic_ns"] is None) is (native_count == 25)
    assert all(row["no_unobserved_future_tick_fabricated"] for row in result["rows"])
    assert result["closed_loop_gain_claimed"] is result["formal_function_approval"] is False


@pytest.mark.parametrize("change", [
    "raw_pin", "worker_sha", "actual_file", "encoded_size", "source_sha", "object_identity", "source_stamp",
    "camera_stamp", "quality", "mutable_source", "source_bytes", "owned_after_start", "unaccepted_ACK",
    "ACK_after_start", "duplicate_ACK", "wrong_ACK_frame", "duplicate_callback", "missing_callback",
    "worker_role", "worker_frame", "encode_after_join", "write_before_join", "unordered_writes",
    "write_after_finish", "recording_crosses_next_tick", "missing_next_tick", "invented_tick_frame",
    "duplicate_tick", "missing_camera", "missing_record", "partial_write", "encode_failure",
    "legacy_fallback", "extra_worker", "background_work", "zero_clock", "wrong_target", "foreign_written_target",
    "symlink_image", "malformed_pin", "pixel_retag", "wrong_receipt_schema", "missing_policy_SHA",
    "unknown_worker", "fourth_worker",
])
def test_independent_source_join_write_and_failure_tail_mismatches_fail_closed(tmp_path, change):
    values = flow_fixture(tmp_path)
    record, camera, owned = values["recordings"][0], values["cameras"][0], values["owned_sources"][0]
    detail = record["parallel_encoding_research"]
    row = detail["encode_rows"][0]
    source = values["callback"]["callbacks"][0]
    path = values["episode"] / camera["images"][m.jpeg.CAMERA_ORDER[0]]
    if change == "raw_pin": values["raw_pins"][str(path)] = "0"*64
    elif change == "worker_sha": row["encoded_JPEG_sha256"] = "0"*64
    elif change == "actual_file": path.write_bytes(b"foreign bytes")
    elif change == "encoded_size": row["encoded_JPEG_byte_count"] += 1
    elif change == "source_sha": source["source_BGRA_encoding_prefixed_sha256"] = "0"*64
    elif change == "object_identity": row["image_object_id"] += 1
    elif change == "source_stamp": source["source_image_timestamp_simulation_ns"] += 1
    elif change == "camera_stamp": camera["source_timestamps"][m.jpeg.CAMERA_ORDER[0]] += .05
    elif change == "quality": row["quality"] = 90
    elif change == "mutable_source": owned["immutable_bytes_backing"] = False
    elif change == "source_bytes": row["source_byte_count"] -= 1
    elif change == "owned_after_start": owned["source_ownership_finished_monotonic_ns"] = 100_201
    elif change == "unaccepted_ACK": values["timings"][2]["ACK_server_accepted"] = False
    elif change == "ACK_after_start": values["timings"][2]["ACK_finished_monotonic_ns"] = 100_201
    elif change == "duplicate_ACK": values["timings"][-1]["ACK_sequence"] = values["timings"][0]["ACK_sequence"]
    elif change == "wrong_ACK_frame": values["timings"][2]["command_frame"] += 1
    elif change == "duplicate_callback": values["callback"]["callbacks"].append(dict(source))
    elif change == "missing_callback": values["callback"]["callbacks"].pop(0)
    elif change == "worker_role": row["role"] = m.jpeg.CAMERA_ORDER[1]
    elif change == "worker_frame": row["native_frame"] += 1
    elif change == "encode_after_join": row["encoding_finished_monotonic_ns"] = 100_301
    elif change == "write_before_join": row["write_started_monotonic_ns"] = 100_299
    elif change == "unordered_writes": detail["encode_rows"][1]["write_started_monotonic_ns"] = 100_301
    elif change == "write_after_finish": row["write_finished_monotonic_ns"] = 100_401
    elif change == "recording_crosses_next_tick": record["recording_finished_monotonic_ns"] = 110_001
    elif change == "missing_next_tick": values["callback"]["native_stage_marks"].pop(1)
    elif change == "invented_tick_frame": values["callback"]["native_stage_marks"][1]["native_frame"] = 101
    elif change == "duplicate_tick": values["callback"]["native_stage_marks"].append(dict(values["callback"]["native_stage_marks"][0]))
    elif change == "missing_camera": values["cameras"].pop()
    elif change == "missing_record": values["recordings"].pop()
    elif change == "partial_write": record["written_camera_count"] = 5
    elif change == "encode_failure": row["encoding_failure"] = "OSError: synthetic failure retained"
    elif change == "legacy_fallback": record.pop("parallel_encoding_research")
    elif change == "extra_worker": detail["workers"] = 4
    elif change == "background_work": detail["background_work_after_flush"] = True
    elif change == "zero_clock": row["encoding_started_monotonic_ns"] = 0
    elif change == "wrong_target": camera["images"][m.jpeg.CAMERA_ORDER[0]] = "images/CAM_BACK/00000100.jpg"
    elif change == "foreign_written_target": record["written_files"][0] += ".foreign"
    elif change == "symlink_image":
        destination = tmp_path / "outside"
        path.rename(destination)
        path.symlink_to(destination)
    elif change == "malformed_pin": row["encoded_JPEG_sha256"] = "INVALID_SHA"
    elif change == "pixel_retag": record["source_pixels_changed"] = True
    elif change == "wrong_receipt_schema": record["schema"] = m.OWNER_SCHEMA
    elif change == "missing_policy_SHA": owned.pop("source_SHA_recorded_by_policy")
    elif change == "unknown_worker": row["worker_thread_id"] = None
    elif change == "fourth_worker": row["worker_thread_id"] = 999
    with pytest.raises((RuntimeError, KeyError, ValueError)):
        m.audit_parallel_recordings(**values)


@pytest.mark.parametrize("count", [24, 2201])
def test_native_denominator_cannot_be_shortened_or_extended_to_pass(tmp_path, count):
    values = flow_fixture(tmp_path)
    values["states"] = [dict(frame=index) for index in range(count)]
    with pytest.raises(RuntimeError, match="denominator"):
        m.audit_parallel_recordings(**values)
