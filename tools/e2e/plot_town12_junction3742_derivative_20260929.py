#!/usr/bin/env python3
"""HH_260929 - Plot a fixed offline Town12 route, never a simulator capture."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]
DIR = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static"
REPORT = DIR / "junction3742_derivative.json"
OUTPUT = DIR / "junction3742_supported_route_overview.png"
REPORT_SHA256 = "67fe9da01d529761e4276cec56167c46256e40c892fffa6ba3bb94ad35f5773c"
SELECTED_SHA256 = "629f4bfc07758e5fe787f51fcd01cef6315bce69f36fa0f3530e058dcac0eb31"


def main() -> None:
    if not REPORT.is_file() or REPORT.is_symlink() or OUTPUT.exists() or OUTPUT.is_symlink():
        raise ValueError("fixed derivative report required and plot output must be fresh")
    if hashlib.sha256(REPORT.read_bytes()).hexdigest() != REPORT_SHA256:
        raise ValueError("derivative report bytes changed")
    report = json.loads(REPORT.read_text())
    if report.get("status") != "DERIVATIVE_PARTIAL_NOT_FULL_SCOUT" or \
       report.get("unique_physical_site_count") != 1 or \
       report.get("nominal_pass_signal_record_summary", {}).get("approach_record_free_count") != 0:
        raise ValueError("derivative interpretation changed")
    supported = [row for row in report["physical_pass_routes"]
                 if row["nominal"]["status"] == "PASS" and
                 row["nominal"]["production_lane_gate_supported"] is True]
    selected = [row for row in supported if row["route_sha256"] == SELECTED_SHA256]
    if len(supported) != 4 or len(selected) != 1 or \
       selected[0]["upstream_m"] != 80 or selected[0]["downstream_m"] != 120:
        raise ValueError("selected supported route changed")
    route = selected[0]
    fig, ax = plt.subplots(figsize=(8.2, 7.2))
    fig.subplots_adjust(left=0.13, right=0.97, top=0.92, bottom=0.20)
    for row in supported:
        if row is route:
            continue
        xy = row["route_xy_ros_m"]
        ax.plot([point[0] for point in xy], [point[1] for point in xy],
                color="#7fb3c8", linewidth=1.5, alpha=0.7)
    xy = route["route_xy_ros_m"]
    ax.plot([point[0] for point in xy], [point[1] for point in xy],
            color="#104b71", linewidth=2.8, label="Selected supported static route")
    ax.scatter(*xy[0], s=72, marker="^", color="#168267", zorder=4, label="Start")
    ax.scatter(*xy[-1], s=72, marker="s", color="#8b3b58", zorder=4, label="Goal")
    center = route["site"]["center_xy_ros_m"]
    ax.scatter(*center, s=140, marker="*", color="#c1771e", zorder=5,
               label="Right-turn site (junction 3742)")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("ROS map x (m)")
    ax.set_ylabel("ROS map y (m)")
    ax.grid(alpha=0.23)
    ax.legend(loc="best", frameon=True, fontsize=8)
    ax.set_title("Town12 offline right-turn geometry — junction 3742", fontsize=12)
    fig.text(0.5, 0.045,
             "u=80 m, d=120 m  |  221.57 m route  |  10 cm inset sweep PASS  |  "
             "approach signal records present\n"
             "Derivative partial evidence only; production lane gate supported, "
             "no capture authorization",
             ha="center", va="bottom", fontsize=8)
    fig.savefig(OUTPUT, dpi=160, facecolor="white")
    plt.close(fig)
    print(OUTPUT)


if __name__ == "__main__":
    main()
