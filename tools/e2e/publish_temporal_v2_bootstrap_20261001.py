"""HH_261001 - Plot raw bootstrap task losses separately without calling them driving performance."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = json.loads((args.run / "known_dev_raw_before.json").read_text())
    after = json.loads((args.run / "known_dev_raw_after.json").read_text())
    history = [json.loads(line) for line in (args.run / "history.jsonl").read_text().splitlines()]
    if [row["token"] for row in before["rows"]] != [row["token"] for row in after["rows"]]:
        raise ValueError("fixed probes changed")
    if [row["step"] for row in history] != list(range(1, 201)):
        raise ValueError("expected exact bounded200 updates")
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for axis, field, title in zip(axes, ("raw_curvature", "raw_preferred_speed"),
                                  ("Scaled curvature SmoothL1", "Preferred-speed SmoothL1")):
        axis.plot([row["step"] for row in history], [row[field] for row in history], lw=1)
        axis.set_ylabel(title)
        axis.grid(alpha=.25)
    axes[-1].set_xlabel("Actual optimizer update — different TRAIN minibatch each time")
    fig.suptitle("New spatial/temporal V2: raw200 wiring profile, not convergence / driving success")
    fig.tight_layout()
    fig.savefig(args.output / "raw_training_tasks.png", dpi=130)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    statistics = {}
    x = np.arange(len(before["rows"]))
    for axis, field, label in zip(axes, ("raw_curvature", "raw_preferred_speed"),
                                  ("Scaled curvature SmoothL1 — smaller is better", "Preferred-speed SmoothL1 — smaller is better")):
        left = np.array([row[field] for row in before["rows"]])
        right = np.array([row[field] for row in after["rows"]])
        axis.bar(x-.18, left, width=.36, label="Fresh weights")
        axis.bar(x+.18, right, width=.36, label="After200 updates")
        axis.set_xticks(x)
        axis.set_xticklabels([row["index"] for row in before["rows"]])
        axis.set_xlabel("Fixed known-DEV index, not independent TEST")
        axis.set_title(label, fontsize=10)
        axis.legend()
        statistics[field] = dict(mean_before=float(left.mean()), mean_after=float(right.mean()),
            percentage_change=100*float(right.mean()/left.mean()-1), improved_count=int((right < left).sum()), count=len(x))
    fig.suptitle("Speed improves; curvature worsens in6/8 probes — keep both visible")
    fig.tight_layout()
    fig.savefig(args.output / "known_dev_tasks_before_after.png", dpi=130)
    plt.close(fig)
    inputs = ("declaration.json", "summary.json", "known_dev_raw_before.json", "known_dev_raw_after.json", "history.jsonl")
    receipt = dict(schema="portable_e2e.raw_bootstrap_publication.20261001.v1", task_statistics=statistics,
        input_sha256={name: hashlib.sha256((args.run/name).read_bytes()).hexdigest() for name in inputs},
        publisher_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        independent_validation=False, vehicle_control_approved=False)
    with (args.output / "publication.json").open("x") as stream:
        json.dump(receipt, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
