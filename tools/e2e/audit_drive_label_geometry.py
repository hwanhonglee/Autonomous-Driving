#!/usr/bin/env python3
"""HH_260906 - Compare sealed AN/AO 6.4 s XY futures with speed/yaw integration, read-only."""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / "docs/assets/validation/2026-09-27/drive_control_ao/final_audit"
TRIALS = ROOT / "storage/experiments/training/2026-09-27"
INPUTS = {
    "AN": {
        "trial": TRIALS / "drive_control_an/goal_window_020_zero",
        "future": EVIDENCE / "goal_window_020_zero.future.jsonl.gz",
        "states_sha256": "67ed194fadaff0bae535482aaf0fc223891302cfbe0b6b65150079a85cf13890",
        "future_sha256": "cb8ac329252e5b872009e6106ecccb6d40352682216cd266c7b41665707cd8e0",
        "owner_sha256": "3577b31a68c56f413de20b0dbb0a4658b6982ad820b742eddd26a2e44100a181",
        "plan_sha256": "6bd7caa69a4011f2f4c46c988606845c0d2ad5be8cd13056ee609e762868613e",
        "manifest_sha256": "c3ff785bf306b9777a6cf4bc029940350427f2bff09608657682b416145f9b79",
    },
    "AO": {
        "trial": TRIALS / "drive_control_ao/start_zero_below_020",
        "future": EVIDENCE / "start_zero_below_020.future.jsonl.gz",
        "states_sha256": "2782102022990cccc23b3c572ab03c3db58bff10e7c1d73ad7e6a6b31916014a",
        "future_sha256": "1ff0e728f55d4d8ed0eebd71eb3eafec2bed38e9e06464f2d207f656b6ab2ca6",
        "owner_sha256": "5643026ca727a4e7eb03d46457854a01e574c14a628ba7bb9f05c65d645f30a8",
        "plan_sha256": "f85f5a545592feecac1b9e9737784bc2fe81579c1828e59839904b567cdef98f",
        "manifest_sha256": "38567994a6941b20f13039f59306cd442cde5a54ad6a4dd914be59319aca42b7",
    },
}
ROUTE_SHA256 = "3f94016e8e86da050f86fb90bf4deaff668ecf59e38357f53018036dbfdf005a"
WINDOWS = {"launch": (48, 62), "turn": (290, 300), "near_goal": (505, 558)}
DT_NS = 100_000_000
STEP_MIN_M = 1e-4
CURVATURE_LIMIT = 0.2
TOLERANCE = 1e-6


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def region(tick: int) -> str:
    for name, (begin, end) in WINDOWS.items():
        if begin <= tick <= end:
            return name
    return "outside_windows"


def interpolate(timeline: list[tuple[int, dict]], stamps: list[int], target_ns: int) -> dict:
    """HH_260906 - Reproduce the frozen linear native bracket without crossing missing 20 Hz frames."""
    upper = bisect_right(stamps, target_ns)
    if upper and stamps[upper - 1] == target_ns:
        return timeline[upper - 1][1]
    require(0 < upper < len(timeline), "future target leaves the native recording")
    first_ns, first = timeline[upper - 1]
    second_ns, second = timeline[upper]
    require(second["frame"] == first["frame"] + 1 and abs(second_ns - first_ns - 50_000_000) <= 500,
            "future target crosses an incomplete native bracket")
    fraction = (target_ns - first_ns) / (second_ns - first_ns)
    result = {key: first[key] + fraction * (second[key] - first[key])
              for key in ("x", "y", "vx", "vy")}
    result["yaw"] = wrap(first["yaw"] + fraction * wrap(second["yaw"] - first["yaw"]))
    return result


def path_curvature(previous_xy: tuple[float, float], xy: tuple[float, float],
                   previous_heading: float) -> tuple[float | None, float, float]:
    """HH_260906 - Use the unchanged heading threshold, zero initial heading and curvature formula."""
    dx, dy = xy[0] - previous_xy[0], xy[1] - previous_xy[1]
    distance = math.hypot(dx, dy)
    if distance <= STEP_MIN_M:
        return None, previous_heading, distance
    heading = math.atan2(dy, dx)
    return abs(wrap(heading - previous_heading)) / distance, heading, distance


def integrated_step(previous_xy: tuple[float, float], previous_speed: float, previous_yaw: float,
                    speed: float, yaw: float, dt: float = 0.1) -> tuple[float, float]:
    """HH_260906 - Trapezoidal world velocity assuming zero sideslip at both measured endpoints."""
    return (previous_xy[0] + 0.5 * dt * (previous_speed * math.cos(previous_yaw) + speed * math.cos(yaw)),
            previous_xy[1] + 0.5 * dt * (previous_speed * math.sin(previous_yaw) + speed * math.sin(yaw)))


def stats(values: list[float]) -> dict:
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    return {"count": len(values), "mean": sum(values) / len(values),
            "rms": math.sqrt(sum(x * x for x in values) / len(values)),
            "median": ordered[(len(ordered) - 1) // 2],
            "p95": ordered[math.ceil(0.95 * len(ordered)) - 1], "max": ordered[-1]}


def read_case(arm: str) -> tuple[list[tuple[int, dict]], list[dict], dict]:
    """HH_260906 - Pin sealed bytes and owner checks before reconstructing any label geometry."""
    item = INPUTS[arm]
    trial, future = item["trial"], item["future"]
    episode = trial / "episode"
    state_path, owner_path = episode / "states.jsonl", trial / "owner_result.json"
    plan_path, manifest_path = trial / "owner_plan.json", episode / "manifest.json"
    require(sha256(state_path) == item["states_sha256"] and sha256(future) == item["future_sha256"]
            and sha256(owner_path) == item["owner_sha256"] and sha256(plan_path) == item["plan_sha256"]
            and sha256(manifest_path) == item["manifest_sha256"]
            and sha256(episode / "route.json") == ROUTE_SHA256,
            f"{arm}: sealed input digest differs")
    owner = json.loads(owner_path.read_text())
    plan = json.loads(plan_path.read_text())
    manifest = json.loads(manifest_path.read_text())
    require(owner.get("exit_code") == 0 and owner.get("source_bytes_unchanged_and_archived") is True
            and all(owner.get("source_checks", {}).values()) and all(owner.get("external_source_checks", {}).values())
            and plan.get("route_sha256") == ROUTE_SHA256
            and manifest.get("status") == "complete" and manifest.get("cleanup", {}).get("completed") is True,
            f"{arm}: owner/episode seal is incomplete")
    states = [json.loads(line) for line in state_path.open()]
    timeline = [(round(state["timestamp"] * 1e9), state) for state in states]
    require(len(timeline) > 64 and all(b[0] > a[0] and b[1]["frame"] == a[1]["frame"] + 1
                                   and abs(b[0] - a[0] - 50_000_000) <= 500
                                   for a, b in zip(timeline, timeline[1:])), f"{arm}: native timeline changed")
    with gzip.open(future, "rt") as stream:
        rows = [json.loads(line) for line in stream]
    require(rows and all(row.get("disposition") in ("full_64_point_anchor", "incomplete_future_retained",
                                                     "tail_label_context_only") for row in rows),
            f"{arm}: unexpected future disposition")
    return timeline, rows, {"states_sha256": item["states_sha256"], "future_sha256": item["future_sha256"],
                             "owner_sha256": item["owner_sha256"], "plan_sha256": item["plan_sha256"],
                             "manifest_sha256": item["manifest_sha256"], "route_sha256": ROUTE_SHA256}


def analyze_case(arm: str) -> dict:
    timeline, rows, hashes = read_case(arm)
    stamps = [stamp for stamp, _ in timeline]
    states_by_stamp = dict(timeline)
    distances: dict[str, list[float]] = defaultdict(list)
    interval_errors: dict[str, list[float]] = defaultdict(list)
    endpoint_errors: dict[str, list[float]] = defaultdict(list)
    point_errors: dict[int, list[float]] = defaultdict(list)
    slip_angle: dict[str, list[float]] = defaultdict(list)
    original: dict[int, list[dict]] = defaultdict(list)
    integrated: dict[int, list[dict]] = defaultdict(list)
    former_failure_outcomes = Counter()
    assessed = Counter()
    replay_max_difference = 0.0
    full_anchors, step_count = 0, 0
    for row in rows:
        if row["disposition"] != "full_64_point_anchor":
            continue
        full_anchors += 1
        anchor_ns = row["anchor_timestamp_ns"]
        require(anchor_ns in states_by_stamp, f"{arm}: anchor lacks exact native state")
        anchor = states_by_stamp[anchor_ns]
        source = row["diagnostic"]
        require(len(source["steps"]) == 64 and row["valid_points"] == 64,
                f"{arm}: incomplete 6.4 s window")
        prev_raw = prev_integrated = (0.0, 0.0)
        prev_raw_heading = prev_integrated_heading = 0.0
        prev_speed, prev_yaw = math.hypot(anchor["vx"], anchor["vy"]), 0.0
        ca, sa = math.cos(anchor["yaw"]), math.sin(anchor["yaw"])
        for index, step in enumerate(source["steps"]):
            target_ns = anchor_ns + (index + 1) * DT_NS
            require(step["index"] == index and step["target_timestamp_ns"] == target_ns,
                    f"{arm}: future index/clock changed")
            future = interpolate(timeline, stamps, target_ns)
            dx, dy = future["x"] - anchor["x"], future["y"] - anchor["y"]
            raw_xy = (ca * dx + sa * dy, -sa * dx + ca * dy)
            speed = math.hypot(future["vx"], future["vy"])
            yaw = wrap(future["yaw"] - anchor["yaw"])
            proxy_xy = integrated_step(prev_integrated, prev_speed, prev_yaw, speed, yaw)
            raw_k, raw_heading, raw_distance = path_curvature(prev_raw, raw_xy, prev_raw_heading)
            proxy_k, proxy_heading, proxy_distance = path_curvature(prev_integrated, proxy_xy,
                                                                     prev_integrated_heading)
            frozen_k, frozen_fail = step["metrics"]["xy_curvature"]
            require((raw_k is None and frozen_k is None) or
                    (raw_k is not None and frozen_k is not None and
                     math.isclose(raw_k, frozen_k, rel_tol=1e-7, abs_tol=1e-7)),
                    f"{arm}: frozen XY curvature replay differs at frame {row['frame']} step {index}")
            require(frozen_fail == (raw_k is not None and raw_k > CURVATURE_LIMIT + TOLERANCE),
                    f"{arm}: frozen curvature verdict replay differs")
            if raw_k is not None:
                replay_max_difference = max(replay_max_difference, abs(raw_k - frozen_k))
            tick = step["episode_relative_100ms_tick"]
            require(tick == round((target_ns - stamps[0]) / DT_NS), f"{arm}: relative tick differs")
            name = region(tick)
            raw_fail = frozen_fail
            proxy_fail = proxy_k is not None and proxy_k > CURVATURE_LIMIT + TOLERANCE
            original[tick].append({"anchor_frame": row["frame"], "index": index,
                                   "curvature": raw_k, "fail": raw_fail, "distance": raw_distance})
            integrated[tick].append({"anchor_frame": row["frame"], "index": index,
                                     "curvature": proxy_k, "fail": proxy_fail, "distance": proxy_distance})
            assessed[("raw", name)] += raw_k is not None
            assessed[("integrated", name)] += proxy_k is not None
            if raw_fail:
                former_failure_outcomes[(name, "still_fails" if proxy_fail else
                                         "unassessed" if proxy_k is None else "assessed_pass")] += 1
            error = math.dist(raw_xy, proxy_xy)
            distances[name].append(error)
            point_errors[tick].append(error)
            if speed > 0.1:
                # HH_260906 - Recorded vx/vy are rear-axle ego-frame components, so atan2(vy,vx) is the slip proxy.
                slip_angle[name].append(abs(math.atan2(future["vy"], future["vx"])))
            interval_errors[name].append(math.dist((raw_xy[0] - prev_raw[0], raw_xy[1] - prev_raw[1]),
                                                    (proxy_xy[0] - prev_integrated[0],
                                                     proxy_xy[1] - prev_integrated[1])))
            if index in (9, 29, 63):
                endpoint_errors[f"{(index + 1) / 10:g}s"].append(error)
            prev_raw, prev_integrated = raw_xy, proxy_xy
            prev_raw_heading, prev_integrated_heading = raw_heading, proxy_heading
            prev_speed, prev_yaw = speed, yaw
            step_count += 1

    def curvature_summary(data: dict[int, list[dict]]) -> dict:
        all_entries = [(tick, item) for tick, entries in data.items() for item in entries]
        failures = [(tick, item) for tick, item in all_entries if item["fail"]]
        return {"assessed_step_count": sum(item["curvature"] is not None for _, item in all_entries),
                "unassessed_step_count": sum(item["curvature"] is None for _, item in all_entries),
                "violating_step_count": len(failures),
                "violating_anchor_count": len({item["anchor_frame"] for _, item in failures}),
                "unique_violating_tick_count": len({tick for tick, _ in failures}),
                "maximum_assessed_curvature_per_m": max(item["curvature"] for _, item in all_entries
                                                         if item["curvature"] is not None),
                "by_region": {name: {"unique_violating_tick_count": len({tick for tick, _ in failures
                                                                               if region(tick) == name}),
                                     "violating_step_count": sum(region(tick) == name for tick, _ in failures),
                                     "violating_anchor_count": len({item["anchor_frame"] for tick, item in failures
                                                                    if region(tick) == name}),
                                     "assessed_step_count": assessed[("raw" if data is original else "integrated", name)]}
                              for name in (*WINDOWS, "outside_windows")}}

    def tick_detail(data: dict[int, list[dict]], tick: int) -> dict:
        entries = data.get(tick, [])
        fails = [item for item in entries if item["fail"]]
        curves = [item["curvature"] for item in entries if item["curvature"] is not None]
        return {"overlapping_window_count": len(entries), "violating_window_count": len(fails),
                "max_assessed_curvature_per_m": max(curves) if curves else None,
                "first_future_point_only": len(fails) == 1 and fails[0]["index"] == 0,
                "minimum_step_distance_m": min((item["distance"] for item in entries), default=None)}

    baseline_ticks = {tick for tick, entries in original.items() if any(item["fail"] for item in entries)}
    integrated_ticks = {tick for tick, entries in integrated.items() if any(item["fail"] for item in entries)}
    special = {53, 54, 55, 56, 58, 59, 295, 509, 516, 544, 545, 551, 552, 556}
    output = {"arm": arm, "input_sha256": hashes, "future_rows_by_disposition": dict(Counter(
        row["disposition"] for row in rows)), "full_64_point_anchor_count": full_anchors,
        "full_horizon_point_count": step_count, "frozen_xy_replay_max_abs_curvature_difference_per_m": replay_max_difference,
        "curvature": {"frozen_raw_xy": curvature_summary(original),
                      "integrated_speed_yaw": curvature_summary(integrated),
                      "old_failure_tick_outcomes": {"still_fails": sorted(baseline_ticks & integrated_ticks),
                                                    "no_integrated_violation": sorted(baseline_ticks - integrated_ticks),
                                                    "new_integrated_violation": sorted(integrated_ticks - baseline_ticks)},
                      "old_failing_window_outcomes_by_region": {name: {
                          outcome: former_failure_outcomes[(name, outcome)] for outcome in
                          ("still_fails", "assessed_pass", "unassessed")}
                          for name in (*WINDOWS, "outside_windows")}},
        "displacement_error_m": {"all_overlapping_points": stats([x for values in distances.values() for x in values]),
                                 "by_target_tick_region": {name: stats(distances[name]) for name in
                                                           (*WINDOWS, "outside_windows")},
                                 "interval_by_target_tick_region": {name: stats(interval_errors[name]) for name in
                                                                     (*WINDOWS, "outside_windows")},
                                 "horizon_endpoint": {name: stats(values) for name, values in endpoint_errors.items()}},
        "selected_ticks": {str(tick): {"region": region(tick), "frozen_raw_xy": tick_detail(original, tick),
                                       "integrated_speed_yaw": tick_detail(integrated, tick),
                                       "point_displacement_error_m": stats(point_errors[tick])}
                           for tick in sorted(baseline_ticks | integrated_ticks | special)}}
    output["measured_rear_velocity_body_yaw_abs_slip_proxy_rad"] = {
        name: stats(slip_angle[name]) for name in (*WINDOWS, "outside_windows")}
    require(step_count == full_anchors * 64 and full_anchors == 576,
            f"{arm}: original complete-horizon denominator changed")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--indent", type=int, default=2, help="JSON output indent; stdout only")
    args = parser.parse_args()
    result = {"schema": "carla.drive_label_geometry_shadow.v1",
              "notice": "Retrospective no-sideslip proxy; no label replacement, gate change, training admission or safety claim.",
              "method": {"source": "sealed AN/AO BasicAgent native 20 Hz states and original full 64-point audit",
                         "alternative": "100 ms target planar speed magnitude and body yaw, trapezoidal integration of v[cos(yaw), sin(yaw)] from each anchor",
                         "raw_xy": "same original rear-axle pose interpolation and anchor-frame rotation",
                         "curvature": "original wrapped segment-heading difference / segment length, 0 initial heading, <=0.0001 m unassessed, >0.200001/m violating",
                         "regions_episode_relative_100ms_tick": WINDOWS,
                         "counting": "all 576 overlapping complete windows retained; unique target ticks are also reported; outside-window failures retained"},
              "script_sha256": sha256(Path(__file__)),
              "cases": {arm: analyze_case(arm) for arm in ("AN", "AO")}}
    print(json.dumps(result, indent=args.indent, allow_nan=False))


if __name__ == "__main__":
    main()
