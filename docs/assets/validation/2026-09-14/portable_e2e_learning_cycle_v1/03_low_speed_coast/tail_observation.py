#!/usr/bin/env python3
"""HH_260906 - Reproduce the descriptive signed-velocity tail audit; read-only and no data admission."""

import hashlib
import json
import math
from pathlib import Path


def summarize(repo):
    root = repo / "artifacts/training/2026-09-14/low_speed_coast_v1"
    result = {"comment": "HH_260906 - Post-capture descriptive analysis; not a predeclared model-selection metric or physical-cause proof.",
              "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), "cases": [],
              "scope": "Last ten observed seconds of coast, actual timestamp selection; signed vector integrals, no label edits."}
    for case in ("01_coast_repeat_1", "02_coast_repeat_2"):
        folder = root / case / "coast_probe"
        manifest = json.loads((folder / "manifest.json").read_text())
        payload = (folder / "native_states.jsonl").read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        assert manifest["status"] == "COMPLETE" and digest == manifest["artifact_sha256"]["native_states.jsonl"]
        native = [json.loads(line) for line in payload.splitlines()]
        coast = [row for row in native if row["phase"] == "coast"]
        rows = [row for row in coast if row["timestamp_ns"] >= coast[-1]["timestamp_ns"] - 10_000_000_000]
        speeds = [row["vx"] for row in rows]
        record = {"case_id": case, "native_sha256": digest, "first_tick": rows[0]["tick"], "last_tick": rows[-1]["tick"],
                  "sample_count": len(rows), "span_s": (rows[-1]["timestamp_ns"] - rows[0]["timestamp_ns"]) / 1e9,
                  "body_vx_sign_changes": sum(a * b < 0 for a, b in zip(speeds, speeds[1:])), "parity": {}, "references": {}}
        for parity, name in ((0, "even_saved_camera_ticks"), (1, "odd_unsaved_native_ticks")):
            values = [row["vx"] for row in rows if row["tick"] % 2 == parity]
            record["parity"][name] = {"count": len(values), "negative_count": sum(v < 0 for v in values),
                                      "positive_count": sum(v > 0 for v in values), "minimum_mps": min(values),
                                      "maximum_mps": max(values), "mean_mps": sum(values) / len(values)}
        for ref in ("actor", "base"):
            points, velocities = [], []
            for row in rows:
                if ref == "actor":
                    pose, velocity = row["actor_snapshot_transform_carla"], row["world_velocity_carla"]
                    points.append((pose["x"], -pose["y"]))
                    velocities.append((velocity[0], -velocity[1]))
                else:
                    c, s = math.cos(row["yaw"]), math.sin(row["yaw"])
                    points.append((row["x"], row["y"]))
                    velocities.append((c * row["vx"] - s * row["vy"], s * row["vx"] + c * row["vy"]))
            integrals, residuals, forward_steps, travel = [], [], [], 0.
            for i, (a, b) in enumerate(zip(rows, rows[1:])):
                dt = (b["timestamp_ns"] - a["timestamp_ns"]) / 1e9
                assert b["frame"] == a["frame"] + 1 and abs(dt - .05) < 5e-7
                delta = [points[i + 1][j] - points[i][j] for j in range(2)]
                integral = [(velocities[i][j] + velocities[i + 1][j]) * .5 * dt for j in range(2)]
                integrals.append(integral)
                residuals.append(math.dist(delta, integral))
                travel += math.hypot(*delta)
                forward_steps.append(math.cos(a["yaw"]) * delta[0] + math.sin(a["yaw"]) * delta[1])
            total_integral = [sum(item[j] for item in integrals) for j in range(2)]
            displacement = [points[-1][j] - points[0][j] for j in range(2)]
            record["references"][ref] = {"travel_m": travel, "net_displacement_m": math.hypot(*displacement),
                "signed_velocity_integral_xy_m": total_integral, "signed_velocity_integral_norm_m": math.hypot(*total_integral),
                "aggregate_vector_residual_m": math.dist(displacement, total_integral),
                "native_residual_rms_m": math.sqrt(sum(v * v for v in residuals) / len(residuals)),
                "native_residual_max_m": max(residuals), "forward_step_min_m": min(forward_steps),
                "forward_step_max_m": max(forward_steps), "positive_forward_step_count": sum(v > 0 for v in forward_steps)}
        result["cases"].append(record)
    return result


if __name__ == "__main__":
    print(json.dumps(summarize(Path(__file__).resolve().parents[6]), indent=2, allow_nan=False))
