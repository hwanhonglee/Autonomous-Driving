"""HH_261004 - Compare bounded three-worker JPEG encoding on every recorded V2 camera bundle, without simulation or training.

The recorded JPEGs are decoded to BGR and reconstituted as BGRA for this offline
probe. They are not original lossless CARLA pixels. All 901 bundles, including
the failed tail, receive byte-exact sequential/parallel parity checks. Timing
uses 30 outcome-independent AB/BA pairs of the actual bounded recorders,
including guards, ordered writes and receipt persistence. It cannot prove
live camera latency, a 10 Hz pipeline or a driving capability.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import sys
import time
from types import SimpleNamespace

from .benchmark_scandir_episode_quota_v1_20261004 import digest, require, safe_path, tree_members, write_new_json, read_json


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_full209_parallel_JPEG_v1_20261004.py"
HELPER = ROOT / "tools/e2e/parallel_exact_jpeg_after_ack_v1_20261004.py"
RUN = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v2"
EPISODE = RUN / "episode.partial"
CALLER = ROOT / "storage/experiments/plans/2026-10-04/town03_known_dev_straight_full209_v2/ROOT_actual_publication_caller.json"
CALLER_SHA = "ca223a3176954733ddd9a5109ee0fc2fe7f215ea90efc2ad03e9108253d7ef9c"
PLAN = ROOT / "storage/experiments/plans/2026-10-04/full209_parallel_JPEG_counterfactual_v1/declaration.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-04/full209_parallel_JPEG_counterfactual_v1"
SCHEMA = "portable_e2e.full209_parallel_JPEG_counterfactual.20261004.v1"
ROLES = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
CAMERAS, PAIRS, WORKERS, QUALITY = 901, 30, 3, 95
_modules = None
DISK_FLOOR = 5*1024**3+16*1024**2
NEXT_BUNDLE_RESERVE = 12*1024**2+128*1024


def fresh_output(path):
    path=Path(path)
    require(path.is_absolute() and path==path.resolve() and not path.exists() and not path.is_symlink(),"fresh canonical no-alias output required")
    safe_path(path,missing_tail=True)
    return path


def disk_guard():
    # HH_261004 - Preserve the closing floor plus a complete six-JPEG write reserve; never adjust disk reservations.
    require(shutil.disk_usage(ROOT).free>=DISK_FLOOR+NEXT_BUNDLE_RESERVE,"5GiB+16MiB closing floor and complete next-bundle reserve unavailable")


def append_journal(name,value):
    # HH_261004 - Persist completed prefixes before proceeding; failed trials cannot disappear into an in-memory summary.
    path=OUTPUT/name
    safe_path(path,missing_tail=True)
    with path.open("a" if path.exists() else "x",encoding="utf-8") as stream:
        stream.write(json.dumps(value,sort_keys=True,allow_nan=False)+"\n")


@contextmanager
def retain_attempt(parity,timings):
    try:
        yield
    except BaseException as error:
        write_new_json(OUTPUT/"FAILED_attempt.json",dict(schema=SCHEMA+".failed_attempt",status="FAILED",
            provenance="HH_261004 - Preserve every completed prefix and original exception; do not retry this declared output.",
            planned_camera_bundles=CAMERAS,completed_parity_count=len(parity),completed_timed_call_count=len(timings),
            next_unfinished_parity_ordinal=len(parity) if len(parity)<CAMERAS else None,
            exception_type=type(error).__name__,exception=str(error),all_completed_parity_rows=parity,all_completed_timing_rows=timings,
            failed_partial_recorder_files_retained=True,original_drive_FAIL_not_changed=True,formal_function_approved=False))
        raise


def modules():
    global _modules
    if _modules is None:
        require(os.environ.get("CUDA_VISIBLE_DEVICES") == "", "explicit GPU-disabled offline probe required")
        import cv2
        import numpy as np
        from . import parallel_exact_jpeg_after_ack_v1_20261004 as jpeg
        require("torch" not in sys.modules and "carla" not in sys.modules, "no model or simulator module may enter the JPEG-only probe")
        _modules = cv2, np, jpeg
    return _modules


def source_snapshot():
    paths = (HERE, TEST, HELPER, ROOT / "tools/e2e/defer_exact_jpeg_until_ACK_research_20261001.py",
        ROOT / "tools/e2e/collect_carla_vad_expert.py", ROOT / "tools/e2e/benchmark_scandir_episode_quota_v1_20261004.py",
        ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py")
    return {str(path.relative_to(ROOT)): digest(path) for path in paths}


def input_snapshot():
    require(digest(CALLER) == CALLER_SHA, "completed original V2 caller changed")
    caller = read_json(CALLER)
    require(caller["run_root"] == str(RUN) and caller["schema"].endswith(".v2.caller_pins"), "authentic full209 V2 caller required")
    inventory = tree_members(RUN)
    require(set(inventory["files"]) == set(caller["run_files_sha256"]), "all completed raw/source/failure files required")
    for name, expected in caller["run_files_sha256"].items():
        path = Path(name)
        require(not path.is_absolute() and ".." not in path.parts and path.as_posix() == name, "safe exact original member required")
        require(digest(RUN / name) == expected, "original V2 source or payload changed: " + name)
    require(tree_members(RUN) == inventory and digest(CALLER) == CALLER_SHA, "original input changed during sealing")
    images = {name:size for name,size in inventory["files"].items() if name.startswith("episode.partial/images/")}
    require(len(images) == 6*CAMERAS and all(0 < size <= 2*1024**2 for size in images.values()), "all5406 bounded JPEGs required")
    return dict(caller_sha256=CALLER_SHA, all_original_file_count=len(caller["run_files_sha256"]),
        all_original_SHA_verified=True, whole_inventory_sha256=hashlib.sha256(json.dumps(inventory,sort_keys=True).encode()).hexdigest(),
        original_JPEG_count=len(images))


def camera_catalog():
    rows = [json.loads(line) for line in safe_path(EPISODE/"camera_frames.jsonl").read_text().splitlines()]
    require(len(rows) == CAMERAS and len({row["frame"] for row in rows}) == CAMERAS, "all901 original camera bundles required")
    for index,row in enumerate(rows):
        require(type(row["frame"]) is int and tuple(row["camera_order"]) == ROLES and set(row["images"]) == set(ROLES), "original six-role ABI required")
        require(index == 0 or row["frame"] == rows[index-1]["frame"]+2, "complete original camera sequence required")
        for role in ROLES:
            require(row["images"][role] == f"images/{role}/{row['frame']:08d}.jpg", "original image membership required")
    return rows


def selected_indices(count=CAMERAS):
    require(type(count) is int and count >= PAIRS, "bounded thirty-pair count required")
    result = [(count-1)*index//(PAIRS-1) for index in range(PAIRS)]
    require(len(set(result)) == PAIRS and result[0] == 0 and result[-1] == count-1, "fixed first-to-last sampling required")
    return result


def contract():
    return dict(all_camera_bundles=CAMERAS, all_original_JPEGs=6*CAMERAS, workers=WORKERS, jpeg_quality=QUALITY,
        representative_indices=selected_indices(), pairs=PAIRS, timed_calls=2*PAIRS,
        pair_order="AB for even pair index, BA for odd; one exact same reconstructed six-camera input",
        all_pairs_and_failed_tails_retained=True, byte_exact_outputs_required=True,
        timer_scope="actual recorder.flush: six-image encode, candidate source/target guards, ordered writes and receipt persistence; every task joined",
        timer_excludes="original source integrity/read/JPEG decode/BGRA reconstruction/paired output comparison/recorder and pool setup/enqueue",
        recorder_setup_outside_timing_matches_persistent_actual_recorder=True,
        first_candidate_flush_retains_cold_worker_startup=True,
        completed_prefix_journals_and_failed_attempt_retained=True,
        disk_floor_bytes=DISK_FLOOR,next_complete_bundle_reserve_bytes=NEXT_BUNDLE_RESERVE,
        source_integrity_and_whole_parity_warm_cache_disclosed=True,
        lossy_JPEG_reconstruction=True, original_lossless_BGRA_parity_proven=False,
        actual_runtime_activated=False, actual_sensor_latency_gain_proven=False,
        no_model_NN_optimizer_simulator_or_vehicle_started=True, model_promoted=False, formal_function_approval=False)


def environment():
    cv2,_,_ = modules()
    require(sys.executable == "/usr/bin/python3" and sys.version_info[:2] == (3,10), "original existing local Python3.10 required")
    return dict(interpreter=sys.executable, python=platform.python_version(), OpenCV=cv2.__version__,
        OpenCV_module=str(Path(cv2.__file__).resolve()), OpenCV_module_SHA=digest(Path(cv2.__file__).resolve()),
        OpenCV_threads=cv2.getNumThreads(), OpenCV_thread_configuration_changed=False, GPU_used=False,
        environment={name:os.environ.get(name) for name in ("CUDA_VISIBLE_DEVICES","OMP_NUM_THREADS","MKL_NUM_THREADS","OPENBLAS_NUM_THREADS")})


def reconstruct(row, manifest):
    cv2,np,_ = modules()
    result = []
    for role in ROLES:
        path = safe_path(EPISODE/row["images"][role]);payload = path.read_bytes()
        require(hashlib.sha256(payload).hexdigest() == manifest[str(path.relative_to(RUN))], "exact observed JPEG SHA required")
        bgr = cv2.imdecode(np.frombuffer(payload,dtype=np.uint8),cv2.IMREAD_COLOR)
        require(bgr is not None and bgr.shape == (360,640,3) and bgr.dtype == np.uint8, "original640x360 JPEG decode required")
        bgra = np.empty((360,640,4),dtype=np.uint8);bgra[...,:3]=bgr;bgra[...,3]=255
        result.append(SimpleNamespace(width=640,height=360,frame=row["frame"],timestamp=row["timestamp"],raw_data=bgra.tobytes()))
    return tuple(result)


def encode_bundle(images,pool=None):
    _,_,jpeg = modules()
    require(len(images) == 6, "one full bounded camera bundle required")
    targets = [EPISODE/"OFFLINE_NO_WRITE"/f"{index}.jpg" for index in range(6)]
    if pool is None:
        return tuple(jpeg._encode_image(image,target,QUALITY) for image,target in zip(images,targets))
    futures=[];submission_error=None
    for image,target in zip(images,targets):
        try:futures.append(pool.submit(jpeg._encode_image,image,target,QUALITY))
        except BaseException as error:submission_error=error;break
    # HH_261004 - Join every submitted future before propagating any original ordered error; no sources may outlive this call.
    errors,values = {},[None]*6
    for index,future in enumerate(futures):
        try: values[index] = future.result()
        except BaseException as error: errors[index] = error
    if submission_error is not None:raise submission_error
    if errors: raise errors[min(errors)]
    return tuple(values)


def compare(a,b):
    require(len(a) == len(b) == 6 and all(type(x) is bytes and x == y for x,y in zip(a,b)), "every original ordered JPEG byte must equal")
    return [hashlib.sha256(value).hexdigest() for value in a]


def timed_recorder(images,recorder,*,parallel,pair):
    # HH_261004 - Measure the actual candidate's guards and ordered writes too, not a faster naked encoder surrogate.
    paths = [recorder.output/"images"/role/f"{images[0].frame:08d}.jpg" for role in ROLES]
    disk_guard()
    previous = len(recorder.events)
    for image,path in zip(images,paths): recorder.enqueue(image,path,QUALITY)
    started = time.perf_counter_ns()
    recorder.flush(acknowledged=False,receipt=None)
    finished = time.perf_counter_ns()
    require(not recorder.pending and len(recorder.events)==previous+1 and recorder.events[-1]["all_queued_images_written"] is True,
        "all six original-role writes and the complete receipt required")
    value = dict(pair=pair,arm="B" if parallel else "A",elapsed_ns=finished-started,elapsed_ms=(finished-started)/1e6,
        original_recording_receipt=recorder.events[-1],offline_ACK_claimed=False,first_candidate_flush_cold_workers=parallel and previous==0)
    return tuple(path.read_bytes() for path in paths),value


def stats(values):
    require(values and all(type(value) in (int,float) and math.isfinite(value) and value>=0 for value in values),
        "nonempty finite nonboolean nonnegative observed durations required")
    ordered = sorted(values);at = (len(ordered)-1)*.95;lo = int(at);hi = min(lo+1,len(ordered)-1)
    return dict(count=len(values),median_ms=(ordered[len(ordered)//2]+ordered[(len(ordered)-1)//2])/2,
        p95_ms=ordered[lo]+(ordered[hi]-ordered[lo])*(at-lo),maximum_ms=max(values),mean_ms=sum(values)/len(values))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("declare","execute"),required=True)
    parser.add_argument("--expected-declaration-sha256")
    args = parser.parse_args(argv)
    fresh_output(OUTPUT);disk_guard()
    before = dict(sources=source_snapshot(),inputs=input_snapshot(),environment=environment(),contract=contract())
    rows = camera_catalog();caller = read_json(CALLER)
    manifest = read_json(EPISODE/"manifest.json")
    require(manifest["capture_contract"]["jpeg_quality"] == QUALITY, "original declared JPEG quality required")
    if args.mode == "declare":
        require(args.expected_declaration_sha256 is None,"declaration mode cannot claim execution")
        fresh_output(PLAN)
        write_new_json(PLAN,dict(schema=SCHEMA+".prospective",provenance="HH_261004 - Fix all901 byte-parity and thirty outcome-independent timing pairs before offline JPEG processing.",**before))
        print(json.dumps(dict(status="DECLARED_NOT_MEASURED",declaration_sha256=digest(PLAN))));return 0
    require(args.expected_declaration_sha256 == digest(PLAN), "sealed counterfactual declaration required")
    plan = read_json(PLAN);require(all(plan[name] == value for name,value in before.items()), "sealed sources/inputs/environment/selection changed")
    parity,timings = [],[]
    # HH_261004 - The declaration precedes payload processing; fresh failed timing artifacts remain identifiable rather than retried.
    OUTPUT.mkdir(parents=True,exist_ok=False)
    with retain_attempt(parity,timings),ThreadPoolExecutor(max_workers=WORKERS) as pool:
        for index,row in enumerate(rows):
            images = reconstruct(row,caller["run_files_sha256"])
            a,b = encode_bundle(images),encode_bundle(images,pool)
            parity.append(dict(camera_ordinal=index,frame=row["frame"],all_six_JPEG_bytes_equal=True,encoded_SHA256=compare(a,b)))
            append_journal("all_parity_rows.jsonl",parity[-1])
        _,_,jpeg = modules()
        from . import collect_carla_vad_expert as collector
        require("torch" not in sys.modules and "carla" not in sys.modules,"recorder-only probe cannot import model/simulator")
        roots = {arm:OUTPUT/"timed_recorders"/arm for arm in ("A","B")}
        for root in roots.values():
            for role in ROLES: (root/"images"/role).mkdir(parents=True,exist_ok=False)
        recorders = {"A":jpeg.original.ExactJPEGAfterACK(collector._save_jpeg,roots["A"]),
            "B":jpeg.ParallelExactJPEGAfterACK(collector._save_jpeg,roots["B"])}
        try:
            for pair,index in enumerate(selected_indices()):
                images = reconstruct(rows[index],caller["run_files_sha256"]);values={}
                for arm in (("A","B") if pair%2 == 0 else ("B","A")):
                    output,value = timed_recorder(images,recorders[arm],parallel=arm=="B",pair=pair)
                    values[arm] = output
                    value.update(camera_ordinal=index,all_output_SHA256=[hashlib.sha256(item).hexdigest() for item in output])
                    timings.append(value)
                    append_journal("all_timing_rows.jsonl",value)
                compare(values["A"],values["B"])
        finally:
            recorders["B"].close()
    with retain_attempt(parity,timings):
        after = dict(sources=source_snapshot(),inputs=input_snapshot(),environment=environment(),contract=contract())
        require(before == after and len(parity) == CAMERAS and len(timings) == 2*PAIRS, "whole declared denominator and unchanged original inputs required")
        disk_guard()
        report = dict(schema=SCHEMA,provenance="HH_261004 - Report only offline lossy-reconstructed encoding parity and inclusive all-joined timing; no live or feature PASS.",
            declaration_sha256=args.expected_declaration_sha256,before=before,after=after,source_and_input_unchanged=True,
            all_parity_rows=parity,all_timing_rows=timings,sequential=stats([row["elapsed_ms"] for row in timings if row["arm"] == "A"]),
            three_worker=stats([row["elapsed_ms"] for row in timings if row["arm"] == "B"]),actual_runtime_activated=False,formal_function_approved=False)
        write_new_json(OUTPUT/"report.json",report)
        summary={name:report[name] for name in ("schema","source_and_input_unchanged","sequential","three_worker","actual_runtime_activated","formal_function_approved")}
        summary.update(all_camera_bundles=len(parity),all_original_JPEGs=6*len(parity),timed_calls=len(timings),report_sha256=digest(OUTPUT/"report.json"))
        write_new_json(OUTPUT/"summary.json",summary);print(json.dumps(summary,sort_keys=True));return 0


if __name__ == "__main__":raise SystemExit(main())
