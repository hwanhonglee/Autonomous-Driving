#!/usr/bin/env python3
"""HH_260906 - Plot actual brake readback and measured motion on source-relative display axes; never retime labels."""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

BASE = Path(__file__).resolve().parent


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output = BASE / "brake_comparison"
    if output.exists():
        raise ValueError("comparison output must be fresh")
    ledger = {}

    def read(path):
        payload = path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        if str(path) in ledger and ledger[str(path)] != digest:
            raise ValueError("input changed during comparison")
        ledger[str(path)] = digest
        return payload

    read(Path(__file__))
    plan = json.loads(read(BASE / "plan.json"))
    series = []
    for case in plan["cases"]:
        root = BASE / case["id"]
        assert json.loads(read(root / "owner_result.json"))["exit_code"] == 0
        states = [json.loads(line) for line in read(root / "episode/states.jsonl").splitlines()]
        sources = [r for r in states if r["goal_stop"]["pilot_transition"] == "zero_pedal_coast_to_service_brake_hold"]
        assert len(sources) == 1
        source = sources[0]
        rows = [r for r in states if -.10001 <= r["timestamp"] - source["timestamp"] <= .70001]
        series.append({"case_id": case["id"], "arm": case["arm"], "source_frame": source["frame"],
            "source_timestamp": source["timestamp"], "rows": [{"frame": r["frame"], "timestamp": r["timestamp"],
                "display_seconds_from_hold_source": r["timestamp"] - source["timestamp"],
                "current_brake": r["current_control"]["brake"], "next_brake": r["next_control"]["brake"],
                "speed_mps": math.hypot(r["vx"], r["vy"]),
                "displacement_from_source_mm": 1000 * math.hypot(r["x"] - source["x"], r["y"] - source["y"]),
                "source_body_forward_displacement_mm": 1000 * ((r["x"] - source["x"]) * math.cos(source["yaw"]) +
                                                               (r["y"] - source["y"]) * math.sin(source["yaw"])),
                "source_body_left_displacement_mm": 1000 * (-(r["x"] - source["x"]) * math.sin(source["yaw"]) +
                                                            (r["y"] - source["y"]) * math.cos(source["yaw"]))} for r in rows]})
    figure, axes = plt.subplots(1, 3, figsize=(15, 4.7), dpi=130, constrained_layout=True)
    for index, item in enumerate(series):
        rows = item["rows"]
        x = [row["display_seconds_from_hold_source"] for row in rows]
        style = {"color": "#176b9a" if item["arm"] == "immediate" else "#d16a20",
                 "linestyle": "-" if index < 2 else "--", "linewidth": 1.6,
                 "label": item["case_id"], "alpha": .9}
        axes[0].step(x, [r["current_brake"] for r in rows], where="post", **style)
        axes[1].plot(x, [r["speed_mps"] for r in rows], marker=".", markersize=4, **style)
        axes[2].plot(x, [r["source_body_forward_displacement_mm"] for r in rows], marker=".", markersize=4, **style)
    for axis in axes:
        axis.set_xlim(-.11, .71)
        axis.set_xlabel("Actual seconds from measured hold source")
        axis.grid(alpha=.25)
        axis.axvline(0, color="black", linewidth=.7)
    axes[0].set(title="Actual brake readback differs", ylabel="Normalized brake", ylim=(-.03, 1.06))
    axes[1].set(title="Actual planar speed (overlapping curves)", ylabel="m/s", ylim=(-.005, .13))
    axes[2].set(title="Recorded virtual-base forward displacement", ylabel="mm from hold source")
    axes[0].legend(fontsize=8, loc="lower right")
    figure.suptitle("V7 NORMAL BRAKE ONSET | 4 ACTUAL BASICAGENT RUNS | NO LEARNED CONTROL", fontsize=12)
    for path, digest in ledger.items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest
    output.mkdir()
    figure.savefig(output / "brake_readback_and_motion.png")
    plt.close(figure)
    (output / "manifest.json").write_text(json.dumps({"schema": "carla.hold_brake_motion_plot.v1",
        "comment": "HH_260906 - Source-relative axes are visualization only; every raw timestamp and future label remains unchanged.",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(), "series": series,
        "input_sha256": ledger, "training_data_approved": False, "model_executed": False,
        "scope": "Measured readback/virtual-base data only, not physical substep force, tire truth, or a cause proof."}, indent=2) + "\n")
    files = sorted(output.iterdir())
    (output / "SHA256SUMS").write_text("".join(hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.name + "\n" for p in files))
    print(json.dumps({"output": str(output), "case_count": len(series), "series_lengths": [len(s["rows"]) for s in series]}))


if __name__ == "__main__":
    main()
