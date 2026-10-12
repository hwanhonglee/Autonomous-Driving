#!/usr/bin/env python3
"""HH_260906 - Draw a sealed Town13 route as offline geometry, never as a camera image."""

from __future__ import annotations

import json
from pathlib import Path
import sys

from . import rank_town13_static_sign_routes as rank


REPORT = rank.OUTPUT
IMAGE = REPORT.with_name("route_overview.png")


def main() -> int:
    try:
        document = rank.read_json(REPORT)
        rank.require(document.get("schema") == "portable_e2e.town13_static_sign_route_rank.v2"
                     and document.get("status") == "STATIC_PRIORITY_ONLY",
                     "valid v2 static rank is required")
        _, result, catalog = rank.completed_owner(rank.OWNER, rank.XODR)
        rank.require(document.get("catalog_sha256") == result["catalog"]["sha256"],
                     "static rank differs from the completed owner catalog")
        selected = document.get("selected_static_priority_route_id")
        choice = next((row for row in document["routes"] if row["route_id"] == selected), None)
        rank.require(choice is not None and choice["mapped_sign_count"] > 0,
                     "selected static opportunity absent")
        routes = rank.route_paths(rank.OWNER, catalog)
        route = next(((item, data) for item, data in routes if item["id"] == selected), None)
        rank.require(route is not None and route[0]["sha256"] == choice["route_sha256"],
                     "selected route bytes differ from rank")
        rank.require(not IMAGE.exists() and not IMAGE.is_symlink(),
                     "refusing to overwrite an existing route overview")
        roads = {point["road_id"] for point in route[1]["route"]}
        signs = rank.mapped_signs(rank.XODR, roads)
        wanted = {(row["road_id"], row["signal_id"])
                  for row in choice["mapped_sign_opportunities"]}
        signs = [sign for sign in signs if (sign["road_id"], sign["signal_id"]) in wanted]
        rank.require(len(signs) == choice["mapped_sign_count"],
                     "mapped sign count changed before rendering")
    except (rank.RankError, OSError, KeyError, StopIteration) as error:
        print(f"STOP_TOWN13_ROUTE_OVERVIEW: {error}", file=sys.stderr)
        return 2

    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    points = route[1]["route"]
    figure, axis = plt.subplots(figsize=(12, 7), dpi=160)
    figure.patch.set_facecolor("#f8fafc")
    axis.set_facecolor("#f8fafc")
    axis.plot([point["x"] for point in points], [point["y"] for point in points],
              color="#2563eb", linewidth=3, label="Serialized route (ROS XY)")
    axis.scatter(points[0]["x"], points[0]["y"], s=125, c="#16a34a",
                 marker="o", edgecolors="white", linewidths=1.5, zorder=5, label="Start")
    axis.scatter(points[-1]["x"], points[-1]["y"], s=125, c="#dc2626",
                 marker="s", edgecolors="white", linewidths=1.5, zorder=5, label="Goal")
    for sign in signs:
        axis.scatter(sign["x"], sign["y"], s=180, c="#f59e0b", marker="^",
                     edgecolors="#92400e", linewidths=1, zorder=6)
        axis.annotate(f"XODR {sign['name']} #{sign['signal_id']}",
                      (sign["x"], sign["y"]), xytext=(9, 9),
                      textcoords="offset points", fontsize=9, color="#78350f")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.25)
    axis.set_xlabel("ROS x (m)")
    axis.set_ylabel("ROS y (m)")
    axis.set_title(f"Town13 offline route geometry: {selected}", loc="left", fontsize=14)
    axis.legend(loc="best")
    figure.text(0.5, 0.02,
                "STATIC XODR + ROUTE ONLY | not CARLA RGB, Autoware driving, visible signs, or DEV eligibility",
                ha="center", fontsize=9, color="#b91c1c")
    figure.tight_layout(rect=(0, 0.05, 1, 1))
    figure.savefig(IMAGE, dpi=160)
    plt.close(figure)
    print(IMAGE)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
