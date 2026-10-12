"""HH_261001 - Verify whether archived JPEGs can support exact AH raw-input replay before counterfactuals."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import torch

from portable_e2e.contract import _canonical_route_in_base, load_contract
from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs, build_runtime_tensors
from portable_e2e.simulation_research import FIELDS, load_ah_weights
from scripts.e2e.collect_ah_simulation_research import state_features
from scripts.e2e.prepare_carla_common10_dataset import _optical_transform
from scripts.e2e.audit_ah_speed_cap_trace_20261001 import ROOT, rows, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("fresh verifier output required")
    torch.set_num_threads(2)
    directory = ROOT/"storage/experiments/validation/2026-10-01/ah_town07_straight_research_v2/episode.partial"
    manifest = json.loads((directory/"manifest.json").read_text())
    states, cameras, decisions = (rows(directory/name) for name in ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl"))
    decision = next(row for row in decisions if row.get("application", {}).get("learned_control_applied")
                    and not row["held_between_camera_frames"])
    frame = decision["inference_frame"]
    lookup = {row["frame"]:row for row in states}
    state = lookup[frame]
    camera = next(row for row in cameras if row["frame"] == frame)
    history = [lookup[row["frame"]] for row in cameras if row["frame"] <= frame][-10:]
    calibration = []
    for index,spec in enumerate(manifest["cameras"]):
        if spec["model_index"] != index:
            raise ValueError("exact model camera order differs")
        k = spec["intrinsic"]
        calibration.append((k["fx"]/spec["width"], k["fy"]/spec["height"], k["cx"]/spec["width"], k["cy"]/spec["height"],
                            *_optical_transform(spec["ros_extrinsic"])[:12]))
    route = json.loads((directory/"route.json").read_text())["route"]
    local,_,_ = _canonical_route_in_base(tuple((row["x"],row["y"]) for row in route),
        position_m=(state["x"],state["y"],state["z"]),
        orientation_xyzw=(0.,0.,math.sin(state["yaw"]/2),math.cos(state["yaw"]/2)),
        contract=load_contract(),context="read-only JPEG replay verifier")
    images=[]
    paths=[]
    for name in camera["camera_order"]:
        path=directory/camera["images"][name];paths.append(path)
        with Image.open(path) as image:
            rgb=np.asarray(image.convert("RGB")).copy()
        images.append(RuntimeCameraFrame(int(round(camera["source_timestamps"][name]*1e9)),rgb,"rgb8"))
    inputs=RuntimeInputs(tuple(images),tuple(calibration),tuple(state_features(row) for row in history),(True,)*10,local)
    weights=ROOT/"storage/experiments/validation/2026-10-01/ah_simulation_weights/AH_simulation.npz"
    model=load_ah_weights(weights,weights.with_suffix(".json"),
        weights_sha256="5fd783def4d67aa8c2abe556187bbfaa06012b1cb2fb5bf04faad760771c9342",
        metadata_sha256="e1558f43b89fb2e967112692f0f348f8f6e457f4bed39b3d592ba02f3cf02de1")
    tensors=build_runtime_tensors(inputs,model.config)
    with torch.no_grad():
        outputs=model(*(tensors[name] for name in FIELDS))
    comparisons={}
    for name,tensor in zip(("candidate_xy","candidate_speed","candidate_logits"),outputs):
        actual=tensor[0].numpy();recorded=np.asarray(decision["raw_output"][name],dtype=np.float32)
        comparisons[name]=dict(exact_float32_equal=bool(np.array_equal(actual,recorded)),
            max_abs_difference=float(np.max(np.abs(actual-recorded))), mean_abs_difference=float(np.mean(np.abs(actual-recorded))))
    result=dict(schema="portable_e2e.ah_jpeg_replay_identity_verifier.v1", inference_frame=frame,
        original_input_encoding="raw sensor BGRA8", archived_image_encoding=manifest["capture_contract"]["image_encoding"],
        archived_jpeg_quality=manifest["capture_contract"]["jpeg_quality"], comparisons=comparisons,
        image_sha256={str(path.relative_to(ROOT)):sha(path) for path in paths},
        source_sha256={name:sha(directory/name) for name in ("states.jsonl","camera_frames.jsonl","learned_control.jsonl","manifest.json","route.json")},
        neural_forwards=1, counterfactual_forwards=0, exact_raw_input_identity_available=False,
        status="EXACT_INPUT_REPLAY_UNAVAILABLE_NO_COUNTERFACTUAL_STARTED", approximate_replay_not_relabelled_exact=True,
        reason="JPEG95 archive does not retain the raw BGRA pixels or exact normalized model tensors consumed live.")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result,stream,indent=2,sort_keys=True,allow_nan=False);stream.write("\n")
    print(json.dumps(dict(output=str(args.output),status=result["status"],comparisons=comparisons)))


if __name__ == "__main__":
    main()
