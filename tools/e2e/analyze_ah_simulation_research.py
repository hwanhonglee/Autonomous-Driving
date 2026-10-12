"""HH_261001 - Report actual AH command ownership and render all six recorded camera views."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def analyze(run):
    episode = run / "episode.partial"
    if not episode.exists():
        episode = run / "episode"
    commands = rows(episode / "learned_control.jsonl")
    states = rows(episode / "states.jsonl")
    cameras = rows(episode / "camera_frames.jsonl")
    manifest = json.loads((episode / "manifest.json").read_text())
    inferences = {r["inference_frame"]: r for r in commands if r.get("raw_output") is not None}
    learned = [r for r in commands if r["application"]["learned_control_applied"]]
    receipt_ids = {r["ack_receipt_sequence"] for r in learned}
    observed = [r for r in states if r.get("control_transport", {}).get("expected_receipt_sequence") in receipt_ids
                and r["control_transport"]["status"] == "PASS"]
    driving = [r for r in states if r["capture_phase"] == "driving"]
    trailing_stop = []
    for row in reversed(driving):
        if abs(row["vx"]) > 0.05:
            break
        trailing_stop.append(row)
    seconds = np.asarray([r["total_seconds"] for r in inferences.values()])
    last = list(inferences.values())[-1]
    chosen_speeds = last["selected"]["speed_mps"]
    summary = dict(schema="portable_e2e.ah_actual_control_analysis.v1", run=str(run),
        checkpoint_sha256=last["source_checkpoint_sha256"],
        source_data_sha256={name: hashlib.sha256((episode / name).read_bytes()).hexdigest()
                            for name in ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl", "manifest.json")},
        state_count=len(states), camera_bundle_count=len(cameras), captured_images=6 * len(cameras),
        declared_driving_command_count=len(commands), acknowledged_learned_commands=len(learned),
        acknowledged_learned_fraction=len(learned) / len(commands),
        learned_command_observations_with_ack_alignment=len(observed),
        command_sources=dict(Counter(r["control_source"] for r in commands)),
        emergency_reasons=dict(Counter(r["emergency_reason"] for r in commands if r.get("emergency_reason"))),
        unique_valid_output_inferences=len(inferences),
        latency_ms_p50_p95_max=np.percentile(seconds * 1000, [50, 95, 100]).tolist(),
        candidate_histogram=dict(Counter(r["selected"]["candidate_index"] for r in inferences.values())),
        progress_m=states[-1]["route_progress_m"] - states[0]["route_progress_m"],
        maximum_abs_cte_m=max(abs(r["route_cte_m"]) for r in states),
        maximum_speed_kph=max(r["vx"] for r in states) * 3.6,
        collision_events=sum(len(r["collision"]) for r in states),
        lane_invasion_events=sum(len(r["lane_invasion"]) for r in states),
        trailing_stationary_seconds=len(trailing_stop) / 20.,
        final_selected_speeds_mps_at_0p1_0p3_0p5_1_2_6p4s=[chosen_speeds[i] for i in (0, 2, 4, 9, 19, 63)],
        teacher_action_difference={name: dict(mean=float(np.mean([abs(r["control"][name] - r["teacher_proposal"][name]) for r in learned])),
                                             p95=float(np.percentile([abs(r["control"][name] - r["teacher_proposal"][name]) for r in learned], 95)))
                                   for name in ("throttle", "brake", "steer")},
        original_capture_status=manifest["status"], original_capture_error=manifest.get("error"),
        causal_interpretation="Selected near-term speed reaches zero while later future points accelerate; repeated replanning keeps the tracker stopped.",
        training_data_approved=False, feature_approval_count=0,
        dagger_label_status="Expert scalar control proposals are logged on learned states. No counterfactual expert future trajectory was recorded; these are not Common10 trajectory training targets.",
        same_training_route=True, independent_dev=False, actual_body_lane_containment_verified=False)
    owner_path = run / "owner_diagnostic_result.json"
    if not owner_path.exists():
        owner_path = run / "owner_result.json"
    if owner_path.exists():
        summary["owner_outcome"] = json.loads(owner_path.read_text())
    return episode, commands, states, cameras, summary


def render(episode, commands, states, cameras, summary, output):
    """HH_261001 - Sample the complete time range at 5 Hz; retain failures and stationary tail."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from PIL import Image

    route = json.loads((episode / "route.json").read_text())["route"]
    route_xy = np.asarray([(p["x"], p["y"]) for p in route])
    state_by_frame = {r["frame"]: r for r in states}
    decisions = {r.get("command_frame"): r for r in commands if r.get("command_frame") is not None}
    order = ("CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT",
             "CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT")
    t0 = states[0]["timestamp"]
    times = np.asarray([r["timestamp"] - t0 for r in states])
    speed = np.asarray([r["vx"] * 3.6 for r in states])
    images = []
    selected_indices = list(range(0, len(cameras), 2))
    if selected_indices[-1] != len(cameras) - 1:
        selected_indices.append(len(cameras) - 1)
    for frame_index, camera_index in enumerate(selected_indices):
        camera = cameras[camera_index]
        state = state_by_frame[camera["frame"]]
        decision = decisions.get(camera["frame"])
        fig = plt.figure(figsize=(14, 8), dpi=90)
        grid = fig.add_gridspec(3, 4, height_ratios=(1, 1, 0.9), width_ratios=(1, 1, 1, 1.1))
        for index, name in enumerate(order):
            ax = fig.add_subplot(grid[index // 3, index % 3])
            with Image.open(episode / camera["images"][name]) as image:
                ax.imshow(image)
            ax.set_title(name, fontsize=9)
            ax.axis("off")
        ax = fig.add_subplot(grid[:2, 3])
        yaw = state["yaw"]
        delta = route_xy - (state["x"], state["y"])
        forward = math.cos(yaw) * delta[:, 0] + math.sin(yaw) * delta[:, 1]
        left = -math.sin(yaw) * delta[:, 0] + math.cos(yaw) * delta[:, 1]
        ax.plot(-left, forward, "k--", lw=1, label="Route")
        if decision and decision.get("raw_output"):
            candidates = np.asarray(decision["raw_output"]["candidate_xy"])
            for path in candidates:
                ax.plot(-path[:, 1], path[:, 0], color="#abb5c5", alpha=0.5, lw=0.7)
            chosen = np.asarray(decision["selected"]["xy_base_m"])
            ax.plot(-chosen[:, 1], chosen[:, 0], color="#f2871d", lw=2, label="Selected AH plan")
        ax.add_patch(Rectangle((-1.0034, 1.425 - 2.2568), 2.0068, 4.5136,
                               facecolor="#1d7fae", alpha=0.85, label="Ego footprint"))
        ax.set(xlim=(-8, 8), ylim=(-3, 25), xlabel="Right / left (m)", ylabel="Forward (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.2)
        ax.legend(loc="upper right", fontsize=7)
        ax.set_title("Vehicle-centered route + all 12 plans", fontsize=9)
        ax = fig.add_subplot(grid[2, :3])
        ax.plot(times, speed, color="#147ca5", lw=1.5, label="Observed speed")
        ax.axhline(9., color="gray", ls="--", lw=0.8, label="Research cap: 9 km/h")
        ax.axvline(state["timestamp"] - t0, color="#f2871d", lw=1.5)
        ax.set(xlim=(0, 20), ylim=(-0.4, 10), xlabel="Recorded simulation time (s)", ylabel="km/h")
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8, loc="upper right")
        ax = fig.add_subplot(grid[2, 3])
        ax.axis("off")
        source = decision["control_source"] if decision else state["capture_phase"]
        text = (f"Frame {camera['frame']} | t={state['timestamp'] - t0:.2f}s\n"
                f"{source}\nSpeed {state['vx'] * 3.6:.2f} km/h\n"
                f"Progress {state['route_progress_m']:.2f} m\nCTE {state['route_cte_m']:.3f} m\n\n"
                "Recorded cameras + causal model output\nExpert startup is disclosed\n"
                "TRAIN route; no feature approval\nNo new training data admitted")
        ax.text(0, 1, text, va="top", fontsize=9)
        fig.suptitle("AH learned-control research | Town07 | full 20 s record including self-stop", fontsize=13)
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        fig.canvas.draw()
        frame = Image.frombytes("RGB", fig.canvas.get_width_height(), fig.canvas.tostring_rgb())
        if frame_index == len(selected_indices) // 3:
            frame.save(output / "vehicle_centered.png")
        if frame_index == len(selected_indices) - 1:
            frame.save(output / "stalled_final.png")
        images.append(frame.convert("P", palette=Image.ADAPTIVE, colors=128))
        plt.close(fig)
    images[0].save(output / "vehicle_centered_full.gif", save_all=True, append_images=images[1:],
                   duration=200, loop=0, optimize=False)
    return dict(rendered_frames=len(images), playback_fps=5, recorded_camera_bundles=len(cameras),
                sampled_every_second_bundle=True, six_original_views_per_frame=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    episode, commands, states, cameras, summary = analyze(args.run)
    summary["visualization"] = render(episode, commands, states, cameras, summary, args.output)
    with (args.output / "analysis.json").open("x") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
