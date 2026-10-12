"""HH_261004 - Verify private buffer snapshots and exact original synchronous recording using synthetic CPU sources only."""

from array import array
from dataclasses import FrozenInstanceError
import hashlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import parallel_exact_jpeg_after_ack_v2_20261004 as h


SIZE=640*360*4


def image(kind="bytes",ordinal=0):
    payload=bytes([ordinal])*SIZE
    raw={"bytes":lambda:payload,"bytearray":lambda:bytearray(payload),
        "readonly_view":lambda:memoryview(payload),"mutable_view":lambda:memoryview(bytearray(payload))}[kind]()
    return SimpleNamespace(frame=2,timestamp=.1,width=640,height=360,raw_data=raw)


def target(output,ordinal):
    path=output/"images"/h.CAMERA_ORDER[ordinal]/"00000002.jpg";path.parent.mkdir(parents=True,exist_ok=True);return path


def numerical(source,path,quality):
    assert isinstance(source,h.SnapshotImage) and type(source.raw_data) is bytes
    return b"SYNTHETIC_ENCODER_NOT_DRIVING:"+bytes([source.raw_data[0],quality])


@pytest.fixture
def recorder(monkeypatch,tmp_path):
    monkeypatch.setattr(h.previous,"exact_encoder",lambda function:numerical)
    instance=h.ParallelExactJPEGAfterACK(lambda *args:None,tmp_path)
    try:yield instance
    finally:instance.close()


def test_frozen_failed_revision_and_entire_flush_bytecode_are_retained():
    h.verify_frozen()
    assert h.ParallelExactJPEGAfterACK.flush.__code__ is h.previous.ParallelExactJPEGAfterACK.flush.__code__
    assert h.ParallelExactJPEGAfterACK.enqueue is h.original.ExactJPEGAfterACK.enqueue
    assert h.ParallelExactJPEGAfterACK._join_all is h.previous.ParallelExactJPEGAfterACK._join_all
    assert h.ParallelExactJPEGAfterACK.close is h.previous.ParallelExactJPEGAfterACK.close
    assert h.ParallelExactJPEGAfterACK.flush.__globals__["SCHEMA"]==h.SCHEMA


@pytest.mark.parametrize("kind",["bytes","bytearray","readonly_view","mutable_view"])
def test_actual_original_buffer_shapes_have_equal_private_snapshots_and_role_order(recorder,tmp_path,kind):
    sources=[image(kind,ordinal) for ordinal in range(6)]
    for ordinal,source in enumerate(sources):recorder.enqueue(source,target(tmp_path,ordinal),95)
    recorder.flush(acknowledged=True,receipt={"sequence":3})
    event=recorder.events[-1];detail=event["parallel_encoding_research"]
    assert event["schema"]==h.original.SCHEMA and detail["schema"]==h.SCHEMA
    assert detail["workers"]==3 and detail["all_submitted_encodes_joined"] is True
    assert detail["submitted_encode_count"]==6 and detail["background_work_after_flush"] is False
    assert event["written_files"]==[str(target(tmp_path,index)) for index in range(6)]
    for ordinal,(source,row) in enumerate(zip(sources,detail["encode_rows"])):
        expected=hashlib.sha256(b"bgra8"+bytes(source.raw_data)).hexdigest()
        assert row["image_object_id"]==id(source) and row["private_encoding_image_object_id"]!=id(source)
        assert row["native_frame"]==2 and row["source_image_timestamp_simulation_ns"]==100_000_000
        for name in ("source_pre_snapshot_BGRA_encoding_prefixed_sha256","immutable_snapshot_BGRA_encoding_prefixed_sha256",
                "source_post_encoding_BGRA_encoding_prefixed_sha256","source_BGRA_encoding_prefixed_sha256"):
            assert row[name]==expected
        assert row["private_snapshot_immutable"] and row["source_stable_through_encoding"]
        assert row["source_metadata_unchanged_after_encoding"] and row["immutable_snapshot_byte_count"]==SIZE
        assert row["source_ABI"]["buffer_protocol_observed"] and row["source_ABI"]["nbytes"]==SIZE
        assert row["source_ABI"]["readonly"] is (kind in ("bytes","readonly_view"))
        assert row["encoding_started_monotonic_ns"]<=row["source_snapshot_started_monotonic_ns"] \
            <=row["source_snapshot_finished_monotonic_ns"]<=row["encoding_finished_monotonic_ns"] \
            <=detail["all_encodes_joined_monotonic_ns"]<=row["write_started_monotonic_ns"]
        assert target(tmp_path,ordinal).read_bytes()==numerical(h.SnapshotImage(2,.1,640,360,bytes(source.raw_data)),None,95)


@pytest.mark.parametrize("kind",["bytes","bytearray","readonly_view","mutable_view"])
def test_real_original_encoder_byte_parity_on_explicitly_synthetic_sources(recorder,tmp_path,kind):
    source=image(kind,7)
    expected=h.previous._encode_image(source,tmp_path/"original.jpg",95)
    recorder.encoder=h.previous._encode_image
    payload,error,row=recorder._encode(source,target(tmp_path,0),95,0)
    assert error is None and payload==expected and row["encoded_JPEG_sha256"]==hashlib.sha256(expected).hexdigest()
    assert bytes(source.raw_data)==bytes([7])*SIZE


@pytest.mark.parametrize("bad",["width","height","bool_width","negative_frame","nan_stamp","wrong_len","nonbuffer",
    "noncontiguous","nonbyte_itemsize"])
def test_ABI_is_retained_before_rejection_and_no_failed_source_is_written(recorder,tmp_path,bad):
    source=image("bytearray")
    if bad=="width":source.width=639
    elif bad=="height":source.height=359
    elif bad=="bool_width":source.width=True
    elif bad=="negative_frame":source.frame=-1
    elif bad=="nan_stamp":source.timestamp=float("nan")
    elif bad=="wrong_len":source.raw_data=memoryview(bytes(SIZE-1))
    elif bad=="nonbuffer":source.raw_data="not a buffer"
    elif bad=="noncontiguous":source.raw_data=memoryview(bytearray(SIZE*2))[::2]
    elif bad=="nonbyte_itemsize":source.raw_data=memoryview(array("H",[0])*(SIZE//2))
    payload,error,row=recorder._encode(source,target(tmp_path,0),95,0)
    assert error is not None and row["encoding_failure"] and row["source_stable_through_encoding"] is False
    assert row["source_ABI"]["raw_data_type"] and "frame" in row["source_ABI"]
    if bad!="nonbuffer":assert "readonly" in row["source_ABI"] and "nbytes" in row["source_ABI"]
    if bad=="nan_stamp":assert row["source_ABI"]["timestamp"]=="nan"
    assert not target(tmp_path,0).exists()


@pytest.mark.parametrize("change",["pixels","frame","timestamp","width","height"])
def test_changes_during_encode_reject_with_observed_postproof_not_silently_retagged(recorder,tmp_path,change):
    source=image("bytearray")
    def mutate(private,path,quality):
        if change=="pixels":source.raw_data[0]=17
        else:setattr(source,change,getattr(source,change)+1)
        return numerical(private,path,quality)
    recorder.encoder=mutate
    payload,error,row=recorder._encode(source,target(tmp_path,0),95,0)
    assert error is not None and row["source_stable_through_encoding"] is False
    assert row["private_snapshot_immutable"] is True and row["private_snapshot_used_only_for_encoder"] is True
    assert row["source_ABI"]["frame"]==2 and not target(tmp_path,0).exists()
    if change=="pixels":assert row["source_post_encoding_BGRA_encoding_prefixed_sha256"]!=row["immutable_snapshot_BGRA_encoding_prefixed_sha256"]
    else:assert row["source_after_encoding_metadata"][change]!=row["source_ABI"][change]


def test_private_encoding_metadata_and_pixels_are_immutable():
    value=h.SnapshotImage(2,.1,640,360,bytes(SIZE))
    with pytest.raises(FrozenInstanceError):value.frame=3
    with pytest.raises(TypeError):value.raw_data[0]=1


def test_all_tasks_join_and_original_first_failure_prefix_remains(recorder,tmp_path):
    marker=ValueError("synthetic original role2 error")
    calls=[]
    def encode(source,path,quality):
        calls.append(source.raw_data[0])
        if source.raw_data[0]==2:raise marker
        return numerical(source,path,quality)
    recorder.encoder=encode
    sources=[image("mutable_view",ordinal) for ordinal in range(6)]
    for ordinal,source in enumerate(sources):recorder.enqueue(source,target(tmp_path,ordinal),95)
    with pytest.raises(ValueError) as caught:recorder.flush(acknowledged=True,receipt={"sequence":3})
    assert caught.value is marker and sorted(calls)==list(range(6))
    event=recorder.events[-1];rows=event["parallel_encoding_research"]["encode_rows"]
    assert event["written_camera_count"]==2 and len(rows)==6
    assert event["parallel_encoding_research"]["all_submitted_encodes_joined"] is True
    assert rows[2]["encoding_failure"]=="ValueError: synthetic original role2 error"
    assert all(row["source_ABI"]["buffer_protocol_observed"] for row in rows)
    assert [value[0] for value in recorder.pending]==sources[2:]
    with pytest.raises(RuntimeError,match="cannot be retried"):recorder.flush(acknowledged=False)
