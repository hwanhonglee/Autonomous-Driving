#!/usr/bin/env python3
"""HH_260906 - Render actual v9 cameras and measured ego-relative evidence, never a simulated UI or learned-control claim."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from io import BytesIO
import json
import math
from pathlib import Path

BASE = Path(__file__).resolve().parent
CASES = ("01_coast_A", "02_feedback_B")
CAMERAS = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
TRANSITIONS = ("launch_to_normal_pid", "normal_pid_to_zero_pedal_coast", "zero_pedal_coast_to_terminal_feedback", "zero_pedal_coast_to_service_brake_hold")
PHASES = ("stationary_warmup", "driving", "stationary_tail")
GIF_SAMPLES, GIF_DURATION_MS = 24, 400


def require(ok, message):
    if not ok:
        raise ValueError(message)


def safe(path):
    path = Path(path).absolute()
    require(".." not in path.parts and not any(p.is_symlink() for p in (path, *path.parents)), "unsafe or symlink path")
    return path


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def read(path, ledger, maximum=256 * 1024 * 1024):
    path = safe(path)
    require(path.is_file() and path.stat().st_size <= maximum, "missing or oversized input: " + str(path))
    payload = path.read_bytes()
    require(len(payload) <= maximum, "input grew beyond its limit")
    key = str(path)
    digest = sha(payload)
    require(key not in ledger or ledger[key] == digest, "input changed during rendering")
    ledger[key] = digest
    return payload


def loads(payload):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    def finite(value):
        if isinstance(value, float):
            require(math.isfinite(value), "nonfinite JSON value")
        elif isinstance(value, dict):
            for item in value.values(): finite(item)
        elif isinstance(value, list):
            for item in value: finite(item)
    result = json.loads(payload, object_pairs_hook=pairs)
    finite(result)
    return result


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), "finite numeric measurement required")
    return float(value)


def rows(payload):
    result = [loads(line) for line in payload.splitlines() if line.strip()]
    require(all(isinstance(row, dict) for row in result), "JSONL rows must be objects")
    return result


def stamp(row):
    return round(number(row["timestamp"]) * 1e9)


def selections(states, cameras):
    # HH_260906 - Fixed event names only; missing transitions receive no favorable substitute.
    records = []
    requests = [("first_saved_camera_anchor", states[0] if states else None)]
    for name in TRANSITIONS:
        matches = [row for row in states if row.get("goal_stop", {}).get("pilot_transition") == name]
        require(len(matches) <= 1, "ambiguous repeated transition: " + name)
        requests.append((name, matches[0] if matches else None))
    # HH_260906 - Show the first actual full-brake readback after the latch, not a mathematical request or an invented frame.
    full_hold = next((row for row in states if row.get("goal_stop", {}).get("hold_brake_latched") is True
                     and row["capture_phase"] == "driving" and row["current_control"]["brake"] == 1.0), None)
    requests.append(("first_full_hold_brake_readback", full_hold))
    requests.append(("last_saved_camera_anchor", states[-1] if states else None))
    for index, (name, reference) in enumerate(requests, 1):
        camera = (cameras[-1] if name == "last_saved_camera_anchor" and cameras else
                  next((row for row in cameras if reference and row["frame"] >= reference["frame"]), None))
        record = dict(selection=name, disposition="NOT_OBSERVED", png=None)
        if reference:
            record.update(reference_frame=reference["frame"], reference_timestamp=reference["timestamp"],
                          reference_kind="command-source observation" if name in TRANSITIONS else "native endpoint")
        if camera:
            record.update(disposition="OBSERVED", png=f"{index:02d}_{name}.png", frame=camera["frame"],
                          timestamp=camera["timestamp"], capture_phase=camera["capture_phase"],
                          camera_minus_reference_ns=stamp(camera)-stamp(reference))
        records.append(record)
    return records


def ego_xy(points, anchor):
    c, s = math.cos(anchor["yaw"]), math.sin(anchor["yaw"])
    delta = [(p["x"]-anchor["x"], p["y"]-anchor["y"]) for p in points]
    return [s*x-c*y for x, y in delta], [c*x+s*y for x, y in delta]


def dashboard(case_name, arm, episode, camera, state, states, route, label, footer, ledger):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image, ImageDraw, ImageFont

    canvas = Image.new("RGB", (1920, 1404), "white")
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 19)
    title = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 23)
    speed = math.hypot(state["vx"], state["vy"])
    draw.text((12, 3), f"ACTUAL CARLA BASICAGENT EXPERT | v9 {arm.upper()} | {case_name} | NOT AUTOWARE / LEARNED MODEL", fill="#18232f", font=title)
    draw.text((12, 34), f"{label} | {camera['capture_phase']} | frame {camera['frame']} | sim {camera['timestamp']:.9f}s | speed {speed:.6f}m/s", fill="#18232f", font=font)
    pixels = {}
    for index, role in enumerate(CAMERAS):
        x, y = (index % 3)*640, 72+(index//3)*386
        draw.rectangle((x, y, x+639, y+25), fill="#18232f")
        draw.text((x+8, y+1), role+" | original 640x360 FOV", fill="white", font=font)
        relative = camera["images"][role]
        require(relative == f"images/{role}/{camera['frame']:08d}.jpg", "unexpected camera image path")
        payload = read(episode/relative, ledger, 16*1024*1024)
        with Image.open(BytesIO(payload)) as picture:
            require(picture.format == "JPEG" and picture.size == (640, 360) and picture.mode == "RGB", "unexpected JPEG geometry/mode")
            picture.load()
            pixels[role] = dict(jpeg_sha256=sha(payload), decoded_rgb_sha256=sha(picture.tobytes()))
            canvas.paste(picture, (x, y+26))
    # HH_260906 - The plotted trail ends at the selected observation and retains only the preceding ten seconds.
    past = [r for r in states if stamp(state)-10_000_000_000 <= stamp(r) <= stamp(state)]
    rx, ry = ego_xy(route["route"], state)
    tx, ty = ego_xy(past, state)
    figure, axes = plt.subplots(1, 3, figsize=(19.2, 5.2), dpi=100, constrained_layout=True)
    axes[0].plot(rx, ry, color="#1685aa", linewidth=6., label="Fixed catalog route")
    axes[0].plot(tx, ty, color="#e46a1b", linewidth=1.8, label="Past 10 s actual virtual-base trail")
    axes[0].scatter([rx[-1]], [ry[-1]], color="#16853a", s=40, label="Fixed catalog goal")
    axes[0].scatter([0.], [0.], marker="^", s=150, color="#17212b", zorder=6, label="Ego at camera/native frame")
    axes[0].arrow(0., 0., 0., 4., width=.08, color="#17212b", zorder=5)
    axes[0].set(xlim=(-20,20), ylim=(-20,20), xlabel="Right (m)", ylabel="Forward (m)", title="EGO-CENTERED ROUTE | FIXED +/-20 m")
    axes[0].set_aspect("equal", adjustable="box")
    times = [(stamp(r)-stamp(state))/1e9 for r in past]
    axes[1].plot(times, [math.hypot(r["vx"], r["vy"]) for r in past], color="#1685aa", label="Measured planar speed")
    axes[1].set(xlim=(-10,0), ylim=(-.15,8.6), xlabel="Seconds relative to this image", ylabel="Speed (m/s)", title="ACTUAL PAST SPEED | NO FUTURE TRACE")
    for key, color in (("throttle", "#16853a"), ("brake", "#b72440"), ("steer", "#7558aa")):
        axes[2].step(times, [r["current_control"][key] for r in past], where="post", color=color, label="Observed "+key)
    axes[2].set(xlim=(-10,0), ylim=(-1.05,1.05), xlabel="Seconds relative to this image", ylabel="Normalized observed control", title=f"OBSERVED CONTROL | brake {state['current_control']['brake']:.1f}, steer {state['current_control']['steer']:+.4f}")
    for axis in axes:
        axis.grid(alpha=.25); axis.legend(loc="lower left", fontsize=8)
    buffer = BytesIO()
    figure.savefig(buffer, format="png", dpi=100)
    plt.close(figure)
    with Image.open(buffer) as chart:
        require(chart.size == (1920,520), "unexpected chart geometry")
        canvas.paste(chart.convert("RGB"), (0,844))
    draw.text((12,1375), footer, fill="#18232f", font=font)
    return canvas, pixels


def run(case_root, output):
    case, output = safe(case_root), safe(output)
    require(case.parent == BASE and case.name in CASES, "only the declared two v9 case roots are supported")
    require(output == BASE/"visual_evidence"/case.name and not output.exists(), "fixed fresh private visual output required")
    ledger = {}
    source_sha = sha(read(Path(__file__), ledger))
    owner = loads(read(case/"owner_result.json", ledger))
    plan = loads(read(case/"owner_plan.json", ledger))
    stopped = loads(read(case/"lifecycle/stopped.json", ledger))
    require(owner["capture_mode"] == "expert" and owner["learned_model_control"] is False and owner["vehicle_control_approved"] is False, "unexpected control ownership")
    require(owner["source_bytes_unchanged_and_archived"] is True and all(v is True for v in owner["source_checks"].values()) and set(owner["source_checks"]) == set(plan["source_sha256"]), "capture source receipt failed")
    require(stopped["status"] == "PASS", "owned simulator stop is not confirmed")
    for name, pin in plan["source_sha256"].items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts, "unsafe archived source path")
        require(sha(read(case/"provenance"/relative, ledger)) == pin, "archived capture source mismatch")
    episodes = [case/name for name in ("episode", "episode.partial") if (case/name).is_dir()]
    require(len(episodes) == 1, "require exactly one retained episode")
    episode = episodes[0]
    manifest = loads(read(episode/"manifest.json", ledger))
    config = manifest["capture_contract"]["goal_stop_profile"]
    require(config["profile_id"] == "comfortable_v9_terminal_feedback" and config["training_data_approved"] is False and manifest["result"]["training_data_approved"] is False, "v9 diagnostic scope/denial required")
    arm = config["terminal_approach_mode"]
    require(config["low_speed_steering_mode"] == "zero" and config["hold_brake_mode"] == "immediate", "v9 keeps identical zero-steering and immediate-brake control")
    require(arm == ("coast" if case.name == "01_coast_A" else "feedback"), "terminal mode/case mismatch")
    require(manifest["capture_contract"]["physics_substeps"]["profile"] == "reference_10ms", "v9 numerical setting must stay fixed")
    require(manifest["capture_contract"]["physics_substeps"]["status"] == "PASS", "explicit numerical setting must pass readback")
    route_bytes = read(episode/"route.json", ledger)
    require(sha(route_bytes) == plan["route_sha256"] == config["route_sha256"], "route SHA mismatch")
    route = loads(route_bytes)
    states = rows(read(episode/"states.jsonl", ledger))
    cameras = rows(read(episode/"camera_frames.jsonl", ledger))
    by_frame = {}
    for index, row in enumerate(states):
        require(type(row["frame"]) is int and row["frame"] not in by_frame and row["capture_phase"] in PHASES, "invalid native identity")
        for key in ("x","y","yaw","vx","vy"): number(row[key])
        for key in ("throttle","brake","steer"): number(row["current_control"][key])
        if index:
            require(row["frame"] == states[index-1]["frame"]+1 and abs(stamp(row)-stamp(states[index-1])-50_000_000) <= 500, "native frame/time gap")
        by_frame[row["frame"]] = row
    require([c["frame"] for c in cameras] == [r["frame"] for r in states[::2]], "camera anchors differ from recorded native decimation")
    for camera in cameras:
        state = by_frame[camera["frame"]]
        require(tuple(camera["camera_order"]) == CAMERAS and tuple(camera["images"]) == CAMERAS and tuple(camera["source_timestamps"]) == CAMERAS, "six-camera order differs")
        require(stamp(camera) == stamp(state) and camera["capture_phase"] == state["capture_phase"] and all(abs(round(number(t)*1e9)-stamp(state)) <= 1 for t in camera["source_timestamps"].values()), "camera/native frame-time-phase mismatch")
    selected = selections(states, cameras)
    gif_indices = sorted({round(i*(len(cameras)-1)/(min(GIF_SAMPLES,len(cameras))-1)) for i in range(min(GIF_SAMPLES,len(cameras)))}) if len(cameras)>1 else list(range(len(cameras)))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir()
    source_pixels, gif_records, frames = {}, [], []
    try:
        for item in selected:
            if item["disposition"] != "OBSERVED": continue
            camera = next(c for c in cameras if c["frame"] == item["frame"])
            footer = f"Reference frame {item['reference_frame']} ({item['reference_kind']}); image-reference {item['camera_minus_reference_ns']/1e9:+.9f}s. Original cameras; route reference; measured trail. NOT ADMITTED."
            picture, pixels = dashboard(case.name, arm, episode, camera, by_frame[camera["frame"]], states, route, item["selection"], footer, ledger)
            picture.save(output/item["png"]); picture.close()
            source_pixels[str(camera["frame"])] = pixels
        from PIL import Image
        for index in gif_indices:
            camera = cameras[index]
            elapsed = (stamp(camera)-stamp(cameras[0]))/1e9
            label = f"SAMPLED TIME-LAPSE | source camera index {index}/{len(cameras)-1} | actual elapsed {elapsed:.3f}s"
            footer = "24 uniformly indexed actual anchors (or all if fewer); 400 ms display/frame is NOT elapsed simulation time or real-time Autoware UI."
            picture, pixels = dashboard(case.name, arm, episode, camera, by_frame[camera["frame"]], states, route, label, footer, ledger)
            # HH_260906 - Use the already installed Pillow API; do not upgrade dependencies for evidence rendering.
            reduced = picture.resize((960,702), Image.LANCZOS)
            frames.append(reduced.quantize(colors=128, dither=Image.NONE))
            reduced.close(); picture.close()
            source_pixels[str(camera["frame"])] = pixels
            gif_records.append(dict(camera_index=index, frame=camera["frame"], timestamp=camera["timestamp"], elapsed_sim_seconds=elapsed))
        if frames:
            frames[0].save(output/"sampled_actual_camera_timelapse.gif", save_all=True, append_images=frames[1:], duration=GIF_DURATION_MS, loop=0, disposal=2, optimize=False)
        # HH_260906 - Every opened input is checked again; historical evidence and current capture code remain untouched.
        for name, pin in tuple(ledger.items()):
            require(sha(read(name, {})) == pin, "source input changed before completion")
        report = dict(schema="carla.v9_actual_visual_evidence.v1", status="RENDERED_NOT_ADMITTED", created_at_utc=datetime.now(timezone.utc).isoformat(),
            case=case.name, arm=arm, owner_exit_code=owner["exit_code"], raw_manifest_status=manifest["status"], training_data_approved=False,
            learned_model_control=False, autoware_control=False, source_code_sha256=source_sha, original_inputs_unchanged=True,
            native_states=len(states), camera_anchors=len(cameras), anchor_selection=selected,
            view=dict(ego_centered=True, extent_m=20, past_trace_seconds=10, coordinate_reference="recorded virtual-base ROS XY, not independently calibrated physical axle truth", png_camera_pixels="original decoded 640x360 RGB FOV, no crop or resize"),
            gif=dict(file="sampled_actual_camera_timelapse.gif" if frames else None, selection="round(i*(N-1)/(min(24,N)-1)); N=1 uses index0", display_frame_duration_ms=GIF_DURATION_MS, playback_is_realtime=False, size_px=[960,702], colors=128, source_frames=gif_records),
            source_camera_pixel_sha256=source_pixels, input_sha256={str(Path(name).relative_to(BASE)):pin for name,pin in ledger.items()},
            scope="Selected image bytes/decoded RGB and all camera/native frame-time metadata checked; no full pixel QA, scalar/future admission, ACK or physics re-audit. Missing transitions are NOT_OBSERVED.")
        (output/"manifest.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        payloads = sorted(p for p in output.iterdir() if p.is_file())
        (output/"SHA256SUMS").write_text("".join(f"{sha(p.read_bytes())}  {p.name}\n" for p in payloads))
        return report
    except Exception as error:
        (output/"RENDER_FAILED.json").write_text(json.dumps(dict(error_type=type(error).__name__, error=str(error), training_data_approved=False), indent=2)+"\n")
        raise
    finally:
        for frame in frames: frame.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--case-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = run(args.case_root, args.output)
    print(json.dumps({key:report[key] for key in ("status","case","arm","native_states","camera_anchors")}))


if __name__ == "__main__":
    main()
