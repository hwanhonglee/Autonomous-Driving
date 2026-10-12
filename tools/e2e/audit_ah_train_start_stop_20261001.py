"""HH_261001 - Read only TRAIN annotations to measure start/stop and decoder supervision coverage."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys


def read_pinned(path, expected=None):
    data = path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if expected is not None and digest != expected:
        raise ValueError("source hash mismatch: " + str(path))
    return json.loads(data), digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.source))
    from portable_e2e.contract import ContractError
    from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory

    manifest, manifest_sha = read_pinned(args.dataset / "dataset.json")
    episodes = []
    total = Counter()
    selected = []
    for reference in manifest["episodes"]:
        path = args.dataset / reference["manifest"]
        episode, _ = read_pinned(path, reference["sha256"])
        if episode["split"] != "train":
            # HH_261001 - Split metadata is visible, but no DEV/TEST annotations or images are opened.
            continue
        config, _ = read_pinned(path.parent / episode["source_provenance"]["collection_config_file"],
                                episode["source_provenance"]["collection_config_sha256"])
        phases = config.get("native_result", {}).get("capture_phases", {})
        begin = phases.get("driving", {}).get("first_timestamp")
        end = phases.get("driving", {}).get("last_timestamp")
        data = (path.parent / episode["sample_jsonl"]).read_bytes()
        if hashlib.sha256(data).hexdigest() != episode["sample_jsonl_sha256"]:
            raise ValueError("sample annotation hash differs")
        counts = Counter()
        for line in data.splitlines():
            row = json.loads(line)
            target = row["labels"]["planning"]
            vx = row["ego"]["linear_velocity_base_mps"][0]
            valid_speed = [v for v, flag in zip(target["speed_mps"], target["valid"]) if flag]
            complete = all(target["valid"])
            stamp = row["anchor_timestamp_ns"] / 1e9
            phase = "unknown" if begin is None else ("warmup" if stamp < begin - .001 else
                    "driving" if stamp <= end + .001 else "tail")
            first_accel = (valid_speed[0] - max(0., vx)) / .1
            first_eligible = abs(first_accel) <= 2.9 + 1e-5
            near_eligible = all(abs((b-a)/.1) <= 2.9 + 1e-5
                                for a, b in zip([max(0., vx)] + valid_speed[:4], valid_speed[:5]))
            runtime_safe = False
            if complete:
                try:
                    validate_and_select_trajectory([target["positions_base_xy_m"]], [target["speed_mps"]],
                                                   [0.], RuntimeGateConfig(candidate_count=1), current_speed_mps=vx)
                    runtime_safe = True
                except ContractError:
                    pass
            categories = ["all", "phase_" + phase]
            if first_eligible: categories.append("first_acceleration_representable")
            if near_eligible: categories.append("first_0p5s_acceleration_representable")
            if runtime_safe: categories.append("full_runtime_target_pass")
            if abs(vx) <= .05: categories.append("stationary_vx_le0p05")
            if abs(vx) <= .5: categories.append("low_vx_le0p5")
            if abs(vx) <= 2.5: categories.append("low_vx_le2p5")
            if abs(vx) <= .05 and valid_speed[2] <= .05 and max(valid_speed) >= .5:
                categories.append("stationary_delayed_launch_after0p3")
            if abs(vx) <= .5 and valid_speed[2] > max(0., vx) + .1:
                categories.append("low_speed_immediate_launch_0p3")
            if phase == "driving" and first_eligible and first_accel > .1:
                categories.append("driving_representable_acceleration")
            if phase == "driving" and first_eligible and first_accel < -.1:
                categories.append("driving_representable_deceleration")
            if phase == "driving" and abs(vx) <= .5 and first_eligible and first_accel > .1:
                categories.append("driving_low_speed_representable_start")
                selected.append(dict(token=row["sample_id"], episode_id=episode["episode_id"],
                    sequence_index=row["sequence_index"], vx=vx, first_acceleration_mps2=first_accel,
                    first_0p5s_representable=near_eligible, full_runtime_target_pass=runtime_safe))
            for name in categories:
                counts[name] += 1
                if runtime_safe and name != "full_runtime_target_pass": counts[name + "__runtime_pass"] += 1
            if phase == "warmup":
                counts["warmup_near0p3_hold"] += valid_speed[2] <= .05
        total.update(counts)
        episodes.append(dict(episode_id=episode["episode_id"], route_id=episode["route_id"],
            sample_sha256=episode["sample_jsonl_sha256"], counts=dict(counts)))
    print(json.dumps(dict(schema="portable_e2e.ah_train_start_stop_audit.v1",
        dataset=str(args.dataset), dataset_manifest_sha256=manifest_sha, counts=dict(total),
        episodes=episodes, representable_start_samples=selected,
        dev_test_samples_read=False, images_read=False, training_data_approved=False), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
