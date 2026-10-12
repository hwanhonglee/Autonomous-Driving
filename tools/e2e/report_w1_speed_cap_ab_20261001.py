"""HH_261001 - Compare the complete two-regime experiment without claiming actual30kmh from its cap."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from scripts.e2e.report_initial_acceleration_pair_20261001 import analyze_trial, read_json
from scripts.e2e.audit_initial_acceleration_low_speed_20261001 import analyze as low_speed
from scripts.e2e import research_w1_speed_cap_ab_20261001 as study
from scripts.e2e.report_warmup_supervision_pair_20261001 import transitions

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
        axis.axhline(report["capture_contract"]["controller"]["speed_cap_mps"]*3.6, color="gray", ls="--", lw=.8, label="Declared tracker speed cap")
        axis.axvline(state["timestamp"] - t0, color="#f2871d", lw=1.5)
        axis.set(xlim=(0, 20), ylim=(-.4, 32), xlabel="Recorded simulation time (s)", ylabel="km/h")
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
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    trials={}
    for arm in study.CAPS:
        bundle=analyze_trial(study.output(arm),study.endpoint.CHECKPOINT)
        result=bundle[-1]
        result['schema']='portable_e2e.w1_speed_cap_actual_control.v1'
        result['low_speed_and_chatter']=low_speed(bundle[1],bundle[3])
        result['candidate_transitions']=transitions(bundle[1],bundle[3])
        folder=args.output/arm;folder.mkdir()
        with (folder/'analysis.json').open('x') as stream:
            json.dump(result,stream,indent=2,sort_keys=True,allow_nan=False)
        visual=render_trial(*bundle[:-1],result,folder,arm+' | same W1, teacher30')
        with (folder/'visualization.json').open('x') as stream:
            json.dump(visual,stream,indent=2,sort_keys=True,allow_nan=False)
        trials[arm]=result
    a,b=trials['C9'],trials['C30']
    ca,cb=dict(a['capture_contract']),dict(b['capture_contract'])
    xa,xb=dict(ca.pop('controller')),dict(cb.pop('controller'))
    cap_a,cap_b=xa.pop('speed_cap_mps'),xb.pop('speed_cap_mps')
    checks=dict(same_route=a['route_sha256']==b['route_sha256'],
        same_checkpoint=a['checkpoint_sha256']==b['checkpoint_sha256'],
        same_source_closure=a['source_sha256']==b['source_sha256'],
        same_capture_and_teacher=ca==cb and ca['target_speed']==30.,
        tracker_only_cap_differs=xa==xb and (cap_a,cap_b)==(2.5,30/3.6),
        complete_budgets=all(x['state_count']==400 and x['camera_bundle_count']==200 for x in (a,b)),
        clean_owned_runs=all(x['owner_outcome']['source_hashes_match'] and x['owner_outcome']['port_triplet_released'] and not x['owner_outcome']['owner_cleanup_errors'] for x in (a,b)))
    if not all(checks.values()):
        raise ValueError('cap experiment did not satisfy complete matched protocol: '+str(checks))
    result=dict(schema='portable_e2e.w1_speed_cap_pair_report.v1',checks=checks,trials=trials,
        learned_30kph_approved=False,feature_approval_count=0,training_data_approved=False,
        common_teacher_change_means_old_W1_is_not_this_control=True)
    with (args.output/'pair_report.json').open('x') as stream:
        json.dump(result,stream,indent=2,sort_keys=True,allow_nan=False)
    print(json.dumps(checks,indent=2))


if __name__=='__main__':
    main()
