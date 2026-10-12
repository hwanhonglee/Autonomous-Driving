"""HH_261004 - Prepare fresh synchronous three-worker recording and two-phase signal cleanup without changing driving math."""

from contextlib import contextmanager
import hashlib
from pathlib import Path
import re

from . import collect_temporal_route_joint_town03_straight_full209_v2_20261004 as previous
from . import parallel_exact_jpeg_after_ack_v1_20261004 as jpeg
from . import two_phase_owned_signal_restoration_v1_20261004 as signals


SCHEMA=previous.SCHEMA
OWNER_SCHEMA="portable_e2e.owned_route_joint_town03_known_dev_straight_full209.20261004.v3"
PARENT_SHA256="d35ee2942e63d9a24a6a19340b59ea998a85c11f4d5a21d50a011366922b1dac"
_active=False


def __getattr__(name): return getattr(previous,name)


def verify_frozen():
    previous.frozen._require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest()==PARENT_SHA256,
        "frozen full209 V2 collector changed")
    previous.verify_frozen();jpeg.verify_frozen();signals.verify_frozen()


@contextmanager
def activated(model,args):
    """HH_261004 - Bind the actual inherited recorder and fixture before capture, restoring every alias on all exits."""
    global _active
    verify_frozen()
    previous.frozen._require(not _active and not previous._active,"fresh full209 V3 collector cannot nest")
    saved=previous.OWNER_SCHEMA;_active=True
    try:
        previous.OWNER_SCHEMA=OWNER_SCHEMA
        with jpeg.activated(),signals.activated(),previous.activated(model,args):
            recorder=jpeg.assert_runtime_recorder(previous.collector)
            previous.frozen._require(recorder.output==args.output.with_name(args.output.name+".partial")
                and previous.fixture.GreenSignalFixture is signals.TwoPhaseGreenSignalFixture
                and previous.fixture.assert_owned_capture.__globals__["OWNER_SCHEMA"]==OWNER_SCHEMA
                and previous.SCHEMA==SCHEMA,"actual V3 recorder/fixture/owner/raw ABI binding differs")
            yield
    finally:
        previous.OWNER_SCHEMA=saved;_active=False


def audit_parallel_recordings(recordings,cameras,owned_sources,callback,timings,states,*,episode,raw_pins,digest):
    """HH_261004 - Verify every actual source object, BGRA hash, JPEG, join/write and observed next-tick boundary."""
    require=previous.frozen._require
    def clock(value):
        require(type(value) is int and value>0,"positive actual monotonic clock required")
        return value
    def sha(value):
        require(type(value) is str and re.fullmatch("[0-9a-f]{64}",value),"exact actual source/JPEG SHA required")
        return value
    require(type(episode) is Path or isinstance(episode,Path),"actual episode path required")
    episode=Path(episode)
    require(episode.resolve()==episode and not episode.is_symlink(),"canonical actual episode required")
    partial=episode if episode.name=="episode.partial" else episode.with_name(episode.name+".partial")
    require(25<=len(states)<=2200 and len(cameras)==(len(states)+1)//2<=1100
        and len(recordings)==len(owned_sources)==len(callback["bundles"])==len(cameras),"whole actual recording/source denominator required")
    native_frames=[row["frame"] for row in states]
    require(all(type(frame) is int for frame in native_frames)
        and native_frames==list(range(native_frames[0],native_frames[0]+len(states)))
        and [row["frame"] for row in cameras]==native_frames[::2],"unchanged complete native/camera schedule required")
    callback_by_identity={}
    for row in callback["callbacks"]:
        key=(row["role"],row["native_frame"],row["image_object_id"])
        require(key not in callback_by_identity,"duplicate callback object identity")
        callback_by_identity[key]=row
    ack={row["ACK_sequence"]:row for row in timings}
    require(len(ack)==len(timings)==len(states)+4,"complete actual ACK timing denominator required")
    ticks={}
    for row in callback["native_stage_marks"]:
        if row["stage"]=="world_tick_started":
            require(type(row["tick_index"]) is int and row["tick_index"] not in ticks
                and row["native_frame"] is None,"duplicate or invented next-tick identity")
            ticks[row["tick_index"]]=clock(row["stage_finished_monotonic_ns"])
    require(set(ticks)==set(range(len(states))),"every actual native dispatch must remain recorded")
    state_index={row["frame"]:index for index,row in enumerate(states)}
    require(len(state_index)==len(states),"unique actual state frames required")
    matched=[];sequences=[];workers=set()
    for recording,camera,owned,bound in zip(recordings,cameras,owned_sources,callback["bundles"]):
        frame=camera["frame"];stamp=round(camera["timestamp"]*1.e9)
        require(frame==recording["frame"]==owned["native_frame"]==bound["native_frame"]
            and frame in state_index and camera["timestamp"]==states[state_index[frame]]["timestamp"]
            and tuple(camera["camera_order"])==jpeg.CAMERA_ORDER
            and tuple(row["role"] for row in bound["cameras"])==jpeg.CAMERA_ORDER,
            "actual camera/recording/owned-source/bundle identity differs")
        require(owned.get("source_image_timestamp_simulation_ns")==stamp
            and owned.get("exact_original_source_bytes_owned")==6*640*360*4
            and owned.get("immutable_bytes_backing") is True and owned.get("source_pixel_values_changed") is False
            and owned.get("source_SHA_recorded_by_policy") is True,
            "actual immutable owned six-camera source differs")
        start=clock(recording["recording_started_monotonic_ns"]);finish=clock(recording["recording_finished_monotonic_ns"])
        require(clock(owned["source_ownership_started_monotonic_ns"])<=clock(owned["source_ownership_finished_monotonic_ns"])<=start,
            "actual owned source must precede its recording")
        sequence=recording["ACK_receipt_sequence"];require(type(sequence) is int and sequence in ack,"genuine recording ACK required")
        sequences.append(sequence);timing=ack[sequence]
        require(timing["command_frame"]==frame and timing["ACK_server_accepted"] is True
            and clock(timing["ACK_finished_monotonic_ns"])<=start<=finish
            and recording.get("schema")==jpeg.original.SCHEMA and recording.get("acknowledged_before_recording") is True
            and recording.get("enqueued_camera_count")==recording.get("written_camera_count")==6
            and recording.get("failure") is None,"complete acknowledged original six-camera recording required")
        for key in ("all_queued_images_written","no_source_image_dropped","synchronous_flush_before_next_tick"):
            require(recording.get(key) is True,"original whole-recording guard missing: "+key)
        require(recording.get("source_timestamp_changed") is False and recording.get("source_pixels_changed") is False,
            "source pixels/stamps may not be retagged")
        research=recording.get("parallel_encoding_research",{})
        require(research.get("schema")==jpeg.SCHEMA and research.get("workers")==3
            and research.get("submitted_encode_count")==6 and research.get("all_submitted_encodes_joined") is True
            and research.get("original_role_ordered_writes") is True and research.get("create_only_file_descriptors") is True
            and research.get("background_work_after_flush") is False and research.get("sources_or_quality_changed") is False
            and research.get("controller_or_model_changed") is False,"declared synchronous bounded three-worker proof required")
        rows=research.get("encode_rows");require(type(rows) is list and len(rows)==6,"all six worker rows required")
        joined=clock(research["all_encodes_joined_monotonic_ns"]);require(start<=joined<=finish,"all-join recording boundary invalid")
        written=[];prior_write=joined
        for ordinal,(row,source) in enumerate(zip(rows,bound["cameras"])):
            role=jpeg.CAMERA_ORDER[ordinal];identity=(role,frame,source["image_object_id"])
            require(type(row.get("worker_thread_id")) is int and row["worker_thread_id"]>0,
                "actual bounded worker thread identity required")
            workers.add(row["worker_thread_id"])
            require(len(workers)<=jpeg.WORKERS,"more than three actual JPEG worker threads observed")
            require(identity in callback_by_identity,"worker source is not the actual selected callback object")
            callback_source=callback_by_identity[identity]
            require(type(row.get("ordinal")) is int and row["ordinal"]==ordinal and row.get("role")==role
                and type(row.get("native_frame")) is int and row["native_frame"]==frame
                and type(row.get("image_object_id")) is int and row["image_object_id"]==source["image_object_id"]
                and row.get("source_image_timestamp_simulation_ns")==source["source_image_timestamp_simulation_ns"]==stamp
                and callback_source["source_image_timestamp_simulation_ns"]==stamp
                and round(camera["source_timestamps"][role]*1.e9)==stamp
                and row.get("source_byte_count")==640*360*4
                and callback_source.get("source_byte_count")==640*360*4
                and callback_source.get("source_pixels_changed") is False
                and callback_source.get("source_timestamp_changed") is False
                and sha(row.get("source_BGRA_encoding_prefixed_sha256"))==sha(callback_source["source_BGRA_encoding_prefixed_sha256"]),
                "worker full BGRA bytes differ from authentic selected callback source")
            require(row.get("quality")==camera["jpeg_quality"] and row.get("encoding_failure") is None
                and row.get("source_pixels_changed") is False and row.get("source_timestamp_changed") is False,
                "original JPEG quality/source/exception semantics differ")
            encode_start=clock(row["encoding_started_monotonic_ns"]);encode_end=clock(row["encoding_finished_monotonic_ns"])
            write_start=clock(row["write_started_monotonic_ns"]);write_end=clock(row["write_finished_monotonic_ns"])
            require(start<=encode_start<=encode_end<=joined<=prior_write<=write_start<=write_end<=finish,
                "all encodes must join before original-role ordered writes and recording finish")
            prior_write=write_end
            relative=Path(camera["images"][role])
            require(relative.parts==("images",role,f"{frame:08d}.jpg"),"original camera role/frame JPEG member required")
            path=episode/relative;written.append(str(partial/relative))
            require(path.resolve()==path and not path.is_symlink() and path.is_file()
                and sha(row.get("encoded_JPEG_sha256"))==raw_pins.get(str(path))==digest(path)
                and type(row.get("encoded_JPEG_byte_count")) is int and row["encoded_JPEG_byte_count"]==path.stat().st_size,
                "actual completed JPEG must match worker bytes and independent original raw pin")
        require(recording.get("written_files")==written,"original six-camera role-ordered written targets differ")
        index=state_index[frame];next_tick=ticks.get(index+1)
        require(next_tick is not None or index==len(states)-1,"missing real successor tick cannot be invented")
        if next_tick is not None: require(finish<=next_tick,"worker/ordered recording crossed the actual next native tick")
        matched.append(dict(frame=frame,ACK_sequence=sequence,source_BGRA_and_JPEG_rows=6,
            all_joined_before_ordered_writes=True,next_native_tick_observed=next_tick is not None,
            next_native_tick_started_monotonic_ns=next_tick,no_unobserved_future_tick_fabricated=True))
    require(len(set(sequences))==len(sequences),"recording ACK receipts must be unique")
    return dict(passed=True,actual_camera_bundle_count=len(cameras),actual_BGRA_and_JPEG_rows=6*len(cameras),
        actual_worker_thread_count=len(workers),all_actual_worker_sources_and_joins_and_writes_checked=True,rows=matched,
        raw_pixel_or_state_retagged=False,closed_loop_gain_claimed=False,formal_function_approval=False)
