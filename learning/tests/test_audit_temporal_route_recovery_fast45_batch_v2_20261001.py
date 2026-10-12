"""HH_261001 - Exact writer order only; preserve original path/count/ACK gates and restore every offline binding."""

import json
from pathlib import Path

import pytest

from scripts.e2e import audit_temporal_route_recovery_fast45_batch_v2_20261001 as audit


@pytest.fixture
def full_records(tmp_path):
    # HH_261001 - Tiny explicit test bytes exercise all2700 files; they are never labelled decoded actual camera pixels.
    folder = tmp_path/"trial_00"; episode = folder/"episode"; episode.mkdir(parents=True)
    for name in audit.WRITER_CAMERA_ORDER: (episode/"images"/name).mkdir(parents=True)
    cameras, records, timing = [], [], []
    for i in range(450):
        frame = 1000+2*i
        images = {name: "images/%s/%08d.jpg"%(name,frame) for name in audit.WRITER_CAMERA_ORDER}
        for relative in images.values(): (episode/relative).write_bytes(b"synthetic test bytes not camera evidence")
        cameras.append(dict(frame=frame,images=images,camera_order=list(audit.WRITER_CAMERA_ORDER)))
        records.append(dict(frame=frame,enqueued_camera_count=6,written_camera_count=6,all_queued_images_written=True,
            failure=None,acknowledged_before_recording=True,ACK_receipt_sequence=i+1,
            recording_started_monotonic_ns=20,recording_finished_monotonic_ns=30,
            written_files=[str(folder/"episode.partial"/images[name]) for name in audit.WRITER_CAMERA_ORDER]))
        timing.append(dict(ACK_sequence=i+1,ACK_server_accepted=True,command_frame=frame,ACK_finished_monotonic_ns=10))
    return folder, episode, cameras, records, timing


def run(records):
    folder,episode,cameras,journal,timing = records
    (episode/"recording_after_ack.jsonl").write_text("".join(json.dumps(row)+"\n" for row in journal))
    with audit.activated():
        return audit.audit_post_ACK_recording(folder,episode,cameras,timing,{})


def test_correct_exact_writer_list_accepts_and_restores_original_display_order(full_records):
    old = audit.previous.frozen.CAMERAS
    result = run(full_records)
    assert result["status"] == "PASS" and result["image_count"] == 2700
    assert result["exact_writer_camera_order"] == list(audit.WRITER_CAMERA_ORDER)
    assert result["camera_permutation_accepted"] is False
    assert audit.previous.frozen.CAMERAS == old and audit.previous.audit_post_ACK_recording is audit._original_post_ACK
    assert not audit._active and not audit._order_active


@pytest.mark.parametrize("change",["display_order","permutation","native_order","missing_camera","path_escape","bad_ACK","duplicate_frame","duplicate_ACK"])
def test_original_exact_order_paths_counts_and_ACKs_are_not_relaxed(full_records,change):
    folder,episode,cameras,journal,timing = full_records
    if change=="display_order": journal[0]["written_files"]=[str(folder/"episode.partial"/cameras[0]["images"][name]) for name in audit.DISPLAY_CAMERA_ORDER]
    elif change=="permutation": journal[0]["written_files"][0:2]=reversed(journal[0]["written_files"][0:2])
    elif change=="native_order": cameras[0]["camera_order"]=list(audit.DISPLAY_CAMERA_ORDER)
    elif change=="missing_camera": cameras[0]["images"].pop(audit.WRITER_CAMERA_ORDER[0])
    elif change=="path_escape": cameras[0]["images"][audit.WRITER_CAMERA_ORDER[0]]="../escaped.jpg"
    elif change=="bad_ACK": timing[0]["ACK_finished_monotonic_ns"]=21
    elif change=="duplicate_frame": journal[-1]["frame"]=journal[0]["frame"]
    else: timing[-1]["ACK_sequence"]=timing[0]["ACK_sequence"]
    with pytest.raises((RuntimeError,KeyError)): run(full_records)
    assert audit.previous.frozen.CAMERAS == audit.DISPLAY_CAMERA_ORDER
    assert audit.previous.audit_post_ACK_recording is audit._original_post_ACK and not audit._active and not audit._order_active


def test_exact_source_order_is_read_from_frozen_collector():
    audit.verify_frozen_sources()
    assert audit.WRITER_CAMERA_ORDER != audit.DISPLAY_CAMERA_ORDER


def test_scope_rejects_nesting_and_restores_on_exception():
    with pytest.raises(ValueError):
        with audit.activated():
            with pytest.raises(RuntimeError):
                with audit.activated(): pass
            raise ValueError("fixture")
    assert audit.previous.audit_post_ACK_recording is audit._original_post_ACK and not audit._active


def test_unknown_original_source_fails_before_binding(monkeypatch):
    real = audit.digest
    monkeypatch.setattr(audit,"digest",lambda path: "0"*64 if Path(path)==Path(audit.previous.__file__) else real(path))
    with pytest.raises(RuntimeError,match="frozen v1"): 
        with audit.activated(): pass
    assert audit.previous.audit_post_ACK_recording is audit._original_post_ACK and not audit._active


def test_new_report_pins_wrapper_and_keeps_original_caller_schema(tmp_path,monkeypatch):
    result = dict(schema=audit.SCHEMA, source_sha256={})
    monkeypatch.setattr(audit,"_original_build_report",lambda *args:dict(result,source_sha256={}))
    built = audit.build_report("unchanged caller fixture")
    assert built["schema"] == audit.previous.SCHEMA and built["audit_revision_schema"] == audit.AUDIT_REVISION_SCHEMA
    assert built["source_sha256"][str(Path(audit.__file__))] == audit.digest(audit.__file__)
    assert built["captured_source_or_caller_receipts_rewritten"] is False


def test_existing_output_rejected_before_actual_scan(tmp_path,monkeypatch):
    monkeypatch.setattr(audit,"build_report",lambda *args:pytest.fail("must reject before scan"))
    with pytest.raises(RuntimeError,match="create-only"): audit.publish(tmp_path,"unused")
