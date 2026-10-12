#!/usr/bin/env python3
"""HH_260906 - Plot every actual coast-to-end state, pedal and pose under fixed physics; retain incomplete cases and raw timestamps."""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = Path(__file__).resolve().parent
CAMPAIGN = BASE


def run():
    ledger = {}

    def read(path):
        assert path.is_file() and not any(p.is_symlink() for p in (path, *path.parents))
        payload = path.read_bytes()
        pin = hashlib.sha256(payload).hexdigest()
        assert path not in ledger or ledger[path] == pin
        ledger[path] = pin
        return payload

    read(Path(__file__))
    plan = json.loads(read(CAMPAIGN / "plan.json"))
    summary = json.loads(read(CAMPAIGN / "analysis/summary.json"))
    assert summary["plan_sha256"] == ledger[CAMPAIGN / "plan.json"]
    assert summary["training_data_approved"] is summary["learned_model_control"] is False
    report = dict(schema="carla.terminal_approach_motion_plot.v1", comment=__doc__,
                  generated_at_utc=datetime.now(timezone.utc).isoformat(), cases=[],
                  training_data_approved=False, learned_model_control=False, targets_changed=False,
                  time_axis="Display seconds relative to each observed normal-PID-to-coast source; all coast-to-end native rows and their raw timestamps are retained.")
    figure, axes = plt.subplots(2, 3, figsize=(18, 10), constrained_layout=True)
    observed = 0
    for case, audit in zip(plan["cases"], summary["cases"]):
        assert case["id"] == audit["case_id"]
        item = dict(case_id=case["id"], arm=case["arm"], audited_status=audit["status"])
        report["cases"].append(item)
        if not audit.get("native_state_count"):
            item["status"] = "NO_AUDITED_NATIVE_STATES"
            continue
        path = CAMPAIGN / case["id"] / audit["original_episode_directory"] / "states.jsonl"
        states = [json.loads(line) for line in read(path).splitlines()]
        assert ledger[path] == audit["metadata_sha256"][str(path)]
        source = next((i for i, row in enumerate(states) if row["goal_stop"].get("pilot_transition") == "normal_pid_to_zero_pedal_coast"), None)
        if source is None:
            item["status"] = "COAST_SOURCE_NOT_OBSERVED"
            continue
        anchor = states[source]
        c, s = math.cos(anchor["yaw"]), math.sin(anchor["yaw"])
        rows = []
        for state in states[source:]:
            actor0, actor = anchor["actor_snapshot_transform_carla"], state["actor_snapshot_transform_carla"]
            actor_dx, actor_dy = actor["x"] - actor0["x"], -(actor["y"] - actor0["y"])
            base_dx, base_dy = state["x"] - anchor["x"], state["y"] - anchor["y"]
            v = state["world_velocity_carla"]
            rows.append(dict(frame=state["frame"], timestamp=state["timestamp"],
                source_elapsed_seconds=state["timestamp"] - anchor["timestamp"],
                actor_forward_displacement_mm=1000 * (c * actor_dx + s * actor_dy),
                virtual_base_left_displacement_mm=1000 * (-s * base_dx + c * base_dy),
                actor_forward_speed_mps=c * v[0] - s * v[1],
                measured_planar_speed_mps=math.hypot(state["vx"], state["vy"]),
                remaining_route_arc_m=state["goal_stop"]["remaining_route_arc_m"],
                measured_body_yaw_delta_mrad=1000 * math.atan2(math.sin(state["yaw"] - anchor["yaw"]), math.cos(state["yaw"] - anchor["yaw"])),
                current_brake=state["current_control"]["brake"], next_brake=state["next_control"]["brake"],
                current_throttle=state["current_control"]["throttle"], next_throttle=state["next_control"]["throttle"],
                terminal_reference_mps=state["goal_stop"]["terminal_approach"]["target_speed_mps"]))
        item.update(status="OBSERVED", source_frame=anchor["frame"], source_timestamp=anchor["timestamp"], rows=rows)
        label = case["id"] + " (fixed 10 ms maximum)"
        color = "#2166ac" if case["arm"] == "coast" else "#d95f02"
        style = "--" if case["id"].endswith(("A2", "B2")) else "-"
        for axis, key in zip(axes.flat, ("remaining_route_arc_m", "measured_planar_speed_mps", "current_throttle", "current_brake", "virtual_base_left_displacement_mm", "measured_body_yaw_delta_mrad")):
            axis.plot([r["source_elapsed_seconds"] for r in rows], [r[key] for r in rows], style, color=color, label=label)
        observed += 1
    for axis, title, unit in zip(axes.flat,
            ("Distance remaining to fixed route goal", "Measured planar speed", "Observed throttle readback", "Observed brake readback", "Actual virtual-base lateral displacement", "Measured body yaw change"),
            ("m along route", "m/s", "normalized throttle", "normalized brake", "mm left from coast source", "mrad from coast source")):
        axis.set(title=title, xlabel="Actual seconds from normal coast source", ylabel=unit)
        axis.axvline(0, color="gray", linewidth=.7)
        axis.grid(alpha=.25)
        if observed:
            axis.legend(fontsize=8)
    figure.suptitle("ACTUAL CARLA BASICAGENT | 20 Hz state/control, 10 Hz cameras\nFixed reference physics, coast vs terminal feedback: NOT Autoware / learned driving / data admission", fontsize=13)
    axes[0, 0].axhline(1., color="gray", linestyle=":", label="Original 1 m goal window")
    axes[0, 0].legend(fontsize=8)
    out = BASE / "motion_comparison"
    assert not out.exists()
    for path, pin in ledger.items():
        assert hashlib.sha256(path.read_bytes()).hexdigest() == pin
    out.mkdir()
    figure.savefig(out / "coast_to_stop_motion.png", dpi=120)
    plt.close(figure)
    report["input_sha256"] = {str(path): pin for path, pin in ledger.items()}
    with (out / "manifest.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    with (out / "SHA256SUMS").open("x") as stream:
        for path in sorted(out.iterdir()):
            if path.name != "SHA256SUMS":
                stream.write(hashlib.sha256(path.read_bytes()).hexdigest() + "  " + path.name + "\n")
    print(json.dumps(dict(observed_cases=observed, output=str(out), training_data_approved=False)))


if __name__ == "__main__":
    run()
