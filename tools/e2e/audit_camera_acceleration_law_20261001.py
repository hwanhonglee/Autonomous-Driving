"""HH_261001 - Validate causal camera-interval acceleration against source-pinned native TRAIN states."""

import argparse
import json
from pathlib import Path

import numpy as np

from scripts.e2e.causal_camera_acceleration_research_20261001 import camera_acceleration, rotate
from scripts.e2e.audit_ah_speed_cap_trace_20261001 import ROOT, rows, sha


SOURCES = (
    ("town07", "carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a",
     "storage/datasets/raw/carla/common10_v1/2026-09-04/30kph/town07/straight/ClearNoon/seed_0000/run_001/episode",
     "d527489864f259d768b25918ade44b39b1e4b0248b757e39de6d33916fc63019"),
    ("ctrack_left", "carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7",
     "storage/datasets/raw/carla/common10_v1/2026-09-05/30kph/c_track_1_0_7/turn/qualified/b1_compact_lookahead/ClearNoon/seed_0000/run_001/episode",
     "2a161f30636bb3c168df0c9901aeb28d7c8bbd0d377ab1d6a7f62fa919f88734"))


def describe(records):
    if not records:
        return dict(count=0)
    result = dict(count=len(records))
    for key in ("exact_pose_ax_error", "exact_pose_ay_error", "naive_body_ax_error", "naive_body_ay_error"):
        values=np.abs(np.asarray([row[key] for row in records]))
        result[key]=dict(mean=float(values.mean()),p95=float(np.percentile(values,95)),maximum=float(values.max()))
    return result


def analyze(name,episode_id,relative,expected):
    directory=ROOT/relative
    if sha(directory/"states.jsonl") != expected:
        raise ValueError("original native TRAIN state source differs")
    prepared=ROOT/"storage/datasets/prepared/carla-common10-trainval-research-20260914-v1/episodes"/episode_id
    config=json.loads((prepared/"collection_config.json").read_text())
    if config["source_hashes"]["states.jsonl"] != expected:
        raise ValueError("native source not associated with admitted original TRAIN")
    episode=json.loads((prepared/"episode.json").read_text())
    targets=rows(prepared/episode["sample_jsonl"])
    allowed={row["anchor_timestamp_ns"] for row in targets}
    states=rows(directory/"states.jsonl"); cameras=rows(directory/"camera_frames.jsonl")
    lookup={row["frame"]:i for i,row in enumerate(states)}
    wheelbase=json.loads((directory/"manifest.json").read_text())["coordinate_contract"]["wheelbase_m"]
    result=[]
    for i,camera in enumerate(cameras):
        if i==0 or int(round(camera["timestamp"]*1e9)) not in allowed:
            continue
        now=states[lookup[camera["frame"]]]; before=states[lookup[cameras[i-1]["frame"]]]
        if lookup[camera["frame"]]-lookup[cameras[i-1]["frame"]] != 2:
            raise ValueError("two-native-tick camera interval required")
        middle=states[lookup[camera["frame"]]-1]
        got=camera_acceleration(now,before,wheelbase)
        dt1=middle["timestamp"]-before["timestamp"];dt2=now["timestamp"]-middle["timestamp"];span=dt1+dt2
        a1=rotate((middle["ax"],middle["ay"]),middle["yaw"]);a2=rotate((now["ax"],now["ay"]),now["yaw"])
        reference=rotate(((a1[0]*dt1+a2[0]*dt2)/span,(a1[1]*dt1+a2[1]*dt2)/span),-now["yaw"])
        naive=((now["vx"]-before["vx"])/span,(now["vy"]-before["vy"])/span)
        result.append(dict(frame=now["frame"],timestamp=now["timestamp"],vx=now["vx"],yaw_rate=now["yaw_rate"],
            derived_ax=got["ax"],derived_ay=got["ay"],native_causal_reference_ax=reference[0],native_causal_reference_ay=reference[1],
            exact_pose_ax_error=got["ax"]-reference[0],exact_pose_ay_error=got["ay"]-reference[1],
            naive_body_ax_error=naive[0]-reference[0],naive_body_ay_error=naive[1]-reference[1]))
    return dict(source=name,episode_id=episode_id,source_states_sha256=expected,wheelbase_m=wheelbase,
        all_retained_train_intervals=describe(result),turn_abs_yawrate_gt0p05=describe([r for r in result if abs(r["yaw_rate"])>.05]),records=result)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--output",type=Path,required=True);args=p.parse_args()
    if args.output.exists():raise ValueError("fresh audit output required")
    result=dict(schema="portable_e2e.camera_acceleration_law_audit.v1",sources=[analyze(*source) for source in SOURCES],
        source_raw20Hz_verified_episodes=2,town01_raw20Hz_not_locally_verified=True,
        previous_camera_pose_available_in_common10=True, yaw_rate_approximation_used=False,
        training_rows_mutated=False,neural_forwards=0,optimizer_steps=0)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x") as stream:json.dump(result,stream,indent=2,sort_keys=True,allow_nan=False);stream.write("\n")
    print(json.dumps({"output":str(args.output),"sources":[{k:r[k] for k in ("source","all_retained_train_intervals","turn_abs_yawrate_gt0p05")} for r in result["sources"]]},indent=2))


if __name__=="__main__":main()
