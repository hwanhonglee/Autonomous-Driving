"""HH_261001 - Compare completed matched models and actual controls without hiding failed or stopped trials."""

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np


ARMS = ("A_ah_continuation", "B_preclamp_acceleration")


def read_json(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite_percentiles(values):
    array = np.asarray([value for value in values if isinstance(value, (int, float)) and math.isfinite(value)])
    return np.percentile(array, [50, 95, 100]).tolist() if len(array) else None


def analyze_trial(run, expected_sha):
    """HH_261001 - Rejected raw outputs remain counted; ownership requires explicit ACK, never an expert fallback."""
    episode = run / "episode.partial"
    if not episode.exists():
        episode = run / "episode"
    states, cameras = rows(episode / "states.jsonl"), rows(episode / "camera_frames.jsonl")
    commands = rows(episode / "learned_control.jsonl")
    manifest = read_json(episode / "manifest.json")
    owner = read_json(run / "owner_diagnostic_result.json")
    plan = read_json(run / "owner_plan.json")
    learned = [row for row in commands if row.get("application", {}).get("learned_control_applied") is True]
    receipt_ids = {row["ack_receipt_sequence"] for row in learned}
    observed = [row for row in states if row.get("control_transport", {}).get("expected_receipt_sequence") in receipt_ids
                and row["control_transport"]["status"] == "PASS"]
    inferences = {row["inference_frame"]: row for row in commands if row.get("raw_output") is not None}
    selected = [row for row in inferences.values() if row.get("selected") is not None]
    driving = [row for row in states if row["capture_phase"] == "driving"]
    tail = []
    for row in reversed(driving):
        if abs(row["vx"]) > .05:
            break
        tail.append(row)
    last_speeds = selected[-1]["selected"]["speed_mps"] if selected else None
    final_zero_near = last_speeds is not None and all(isinstance(v, (int, float)) and abs(v) <= .05 for v in last_speeds[:5])
    later_acceleration = last_speeds is not None and max(last_speeds[5:]) >= .5
    stalled = len(tail) >= 20 and final_zero_near and later_acceleration
    declaration = manifest["capture_contract"]
    controller = owner.get("learned", {}).get("controller")
    source_match = all(row.get("source_checkpoint_sha256") == expected_sha for row in commands)
    result = dict(schema="portable_e2e.initial_acceleration_actual_control.v1", run=str(run),
        checkpoint_sha256=expected_sha, all_command_checkpoint_identities_match=source_match,
        state_count=len(states), camera_bundle_count=len(cameras), image_count=6 * len(cameras),
        driving_command_count=len(commands), acknowledged_learned_commands=len(learned),
        acknowledged_learned_fraction=len(learned) / len(commands) if commands else None,
        next_state_ack_aligned_learned_commands=len(observed),
        command_sources=dict(Counter(row["control_source"] for row in commands)),
        emergency_reasons=dict(Counter(row["emergency_reason"] for row in commands if row.get("emergency_reason"))),
        unique_raw_output_inferences=len(inferences), selected_outputs=len(selected),
        unique_raw_outputs_with_emergency_reason=sum(bool(row.get("emergency_reason")) for row in inferences.values()),
        finite_latency_ms_p50_p95_max=finite_percentiles([row.get("total_seconds", float("nan")) * 1000 for row in inferences.values()]),
        candidate_histogram=dict(Counter(row["selected"]["candidate_index"] for row in selected)),
        progress_m=states[-1]["route_progress_m"] - states[0]["route_progress_m"] if states else None,
        maximum_abs_cte_m=max((abs(row["route_cte_m"]) for row in states), default=None),
        maximum_speed_kph=max((row["vx"] * 3.6 for row in states), default=None),
        collision_events=sum(len(row["collision"]) for row in states),
        lane_invasion_events=sum(len(row["lane_invasion"]) for row in states),
        trailing_stationary_seconds=len(tail) / declaration["physics_hz"],
        delayed_acceleration_terminal_stall_detected=bool(stalled),
        stall_definition="tail abs(vx)<=0.05 for >=1s, last selected first0.5s all<=0.05, later selected speed>=0.5",
        last_selected_speeds_mps_at_0p1_0p3_0p5_1_2_6p4s=[last_speeds[i] for i in (0, 2, 4, 9, 19, 63)] if last_speeds else None,
        teacher_action_difference={name: dict(mean=float(np.mean([abs(row["control"][name] - row["teacher_proposal"][name]) for row in learned])),
                                              p95=float(np.percentile([abs(row["control"][name] - row["teacher_proposal"][name]) for row in learned], 95)))
                                   for name in ("throttle", "brake", "steer")} if learned else None,
        capture_contract=dict(physics_hz=declaration["physics_hz"], camera_hz=declaration["camera_hz"],
            duration=declaration["maximum_total_duration_sec"], target_speed=declaration["target_speed_kmh"],
            controller=controller), route_sha256=sha(episode / "route.json"),
        source_sha256=plan["source_sha256"], source_data_sha256={name: sha(episode / name) for name in
            ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl", "manifest.json")},
        owner_outcome=owner, original_capture_status=manifest["status"], original_capture_error=manifest.get("error"),
        training_data_approved=False, feature_approval_count=0, independent_validation=False,
        body_lane_containment_verified=False, expert_counterfactual_future_labels_available=False)
    return episode, states, cameras, commands, manifest, result


def pair_checks(a, b):
    return dict(route_identical=a["route_sha256"] == b["route_sha256"],
        capture_and_controller_identical=a["capture_contract"] == b["capture_contract"],
        source_closure_identical=a["source_sha256"] == b["source_sha256"],
        checkpoints_bound=a["all_command_checkpoint_identities_match"] and b["all_command_checkpoint_identities_match"],
        clean_owned_runs=all(row["owner_outcome"].get("source_hashes_match") is True
                            and row["owner_outcome"].get("port_triplet_released") is True for row in (a, b)),
        complete_fixed_budgets=all(row["state_count"] == 400 and row["camera_bundle_count"] == 200 for row in (a, b)))


def render_trial(episode, states, cameras, commands, manifest, report, output, arm):
    """HH_261001 - Render every second six-view bundle across the full trial, including failures and stationary tails."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from PIL import Image

    route = read_json(episode / "route.json")["route"]
    route_xy = np.asarray([(point["x"], point["y"]) for point in route])
    state_by_frame = {row["frame"]: row for row in states}
    decisions = {row.get("command_frame"): row for row in commands}
    order = ("CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT")
    t0 = states[0]["timestamp"]
    times = np.asarray([row["timestamp"] - t0 for row in states])
    speed = np.asarray([row["vx"] * 3.6 for row in states])
    desired = [decisions.get(row["frame"], {}).get("control", {}).get("desired_speed_mps", float("nan")) * 3.6 for row in states]
    geometry = manifest["capture_contract"].get("actor_body_geometry", {})
    images = []
    indices = list(range(0, len(cameras), 2))
    if indices[-1] != len(cameras) - 1:
        indices.append(len(cameras) - 1)
    for rendered_index, camera_index in enumerate(indices):
        camera = cameras[camera_index]
        state = state_by_frame[camera["frame"]]
        decision = decisions.get(camera["frame"])
        figure = plt.figure(figsize=(14, 8), dpi=90)
        grid = figure.add_gridspec(3, 4, height_ratios=(1, 1, .9), width_ratios=(1, 1, 1, 1.1))
        for index, name in enumerate(order):
            axis = figure.add_subplot(grid[index // 3, index % 3])
            with Image.open(episode / camera["images"][name]) as image:
                axis.imshow(image)
            axis.set_title(name, fontsize=9); axis.axis("off")
        axis = figure.add_subplot(grid[:2, 3])
        yaw = state["yaw"]
        delta = route_xy - (state["x"], state["y"])
        forward = math.cos(yaw) * delta[:, 0] + math.sin(yaw) * delta[:, 1]
        left = -math.sin(yaw) * delta[:, 0] + math.cos(yaw) * delta[:, 1]
        axis.plot(-left, forward, "k--", lw=1, label="Route")
        if decision and decision.get("raw_output"):
            candidates = np.asarray(decision["raw_output"]["candidate_xy"], dtype=float)
            for path in candidates:
                if np.isfinite(path).all():
                    axis.plot(-path[:, 1], path[:, 0], color="#abb5c5", alpha=.5, lw=.7)
            if decision.get("selected"):
                chosen = np.asarray(decision["selected"]["xy_base_m"], dtype=float)
                axis.plot(-chosen[:, 1], chosen[:, 0], color="#f2871d", lw=2, label="Selected model plan")
        axis.scatter([0], [0], s=90, marker="^", color="#1d7fae", label="Ego base_link")
        axis.set(xlim=(-8, 8), ylim=(-3, 25), xlabel="Right / left (m)", ylabel="Forward (m)")
        axis.set_aspect("equal", adjustable="box"); axis.grid(alpha=.2)
        axis.legend(loc="upper right", fontsize=7)
        axis.set_title("Vehicle-centered route + all 12 plans", fontsize=9)
        axis = figure.add_subplot(grid[2, :3])
        axis.plot(times, speed, color="#147ca5", lw=1.5, label="Observed speed")
        axis.plot(times, desired, color="#7a44a6", lw=.9, alpha=.8, label="Tracker desired speed")
        axis.axhline(9., color="gray", ls="--", lw=.8, label="9 km/h research cap")
        axis.axvline(state["timestamp"] - t0, color="#f2871d", lw=1.5)
        axis.set(xlim=(0, 20), ylim=(-.4, 10), xlabel="Recorded simulation time (s)", ylabel="km/h")
        axis.grid(alpha=.2); axis.legend(fontsize=8, loc="upper right")
        axis = figure.add_subplot(grid[2, 3]); axis.axis("off")
        source = decision["control_source"] if decision else state["capture_phase"]
        reason = decision.get("emergency_reason") if decision else None
        text = (f"Frame {camera['frame']} | t={state['timestamp'] - t0:.2f}s\n{source}\n"
                f"Speed {state['vx'] * 3.6:.2f} km/h\nProgress {state['route_progress_m']:.2f} m\n"
                f"CTE {state['route_cte_m']:.3f} m\n"
                + (f"Emergency: {reason}\n" if reason else "")
                + "\nTRAIN route; feature approval 0\nNo new training data admitted")
        axis.text(0, 1, text, va="top", fontsize=9)
        figure.suptitle(arm + " | exact checkpoint " + report["checkpoint_sha256"][:12] + " | full 20 s record", fontsize=12)
        figure.tight_layout(rect=(0, 0, 1, .96)); figure.canvas.draw()
        frame = Image.frombytes("RGB", figure.canvas.get_width_height(), figure.canvas.tostring_rgb())
        if rendered_index == len(indices) // 3:
            frame.save(output / "vehicle_centered.png")
        if rendered_index == len(indices) - 1:
            frame.save(output / "final.png")
        images.append(frame.convert("P", palette=Image.ADAPTIVE, colors=128)); plt.close(figure)
    images[0].save(output / "vehicle_centered_full.gif", save_all=True, append_images=images[1:], duration=200, loop=0, optimize=False)
    return dict(rendered_frames=len(images), playback_fps=5, six_original_views_per_frame=True,
                all_recorded_time_included=True, ego_marker_is_reference_point_not_body_lane_containment=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-summary", type=Path, required=True)
    parser.add_argument("--a-run", type=Path, required=True)
    parser.add_argument("--b-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--no-render", action="store_true")
    args = parser.parse_args()
    summary = read_json(args.training_summary)
    if summary["schema"] != "portable_e2e.initial_acceleration_ab_20261001.v1":
        raise ValueError("fixed paired training summary required")
    args.output.mkdir(parents=True, exist_ok=False)
    trials = {}
    for arm, run in zip(ARMS, (args.a_run, args.b_run)):
        bundle = analyze_trial(run, summary["training"][arm]["checkpoint_sha256"])
        report = bundle[-1]
        folder = args.output / arm
        folder.mkdir()
        if not args.no_render:
            report["visualization"] = render_trial(*bundle[:-1], report, folder, arm)
        (folder / "analysis.json").write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
        trials[arm] = report
    a, b = (trials[arm] for arm in ARMS)
    result = dict(schema="portable_e2e.initial_acceleration_pair_report.v1", source_summary_sha256=sha(args.training_summary),
        pair_checks=pair_checks(a, b), trials=trials,
        paired_training_exposures_identical=summary["training"][ARMS[0]]["exposure_order_sha256"] == summary["training"][ARMS[1]]["exposure_order_sha256"],
        final_dev_strict_gates=summary["strict_gates"], final_dev_B_vs_A=summary["B_vs_A"],
        observed_B_minus_A={field: b[field] - a[field] for field in
            ("progress_m", "trailing_stationary_seconds", "maximum_abs_cte_m", "collision_events", "lane_invasion_events")},
        independent_validation=False, feature_approval_count=0, training_data_approved=False,
        limitation="One matched run per model on a TRAIN route; neither significance nor deployment safety is established.")
    (args.output / "pair_report.json").write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "trials"}, indent=2))


if __name__ == "__main__":
    main()
