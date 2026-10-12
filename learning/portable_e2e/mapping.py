"""HH_260906 - Shared live/replay observation fusion, not a learned map detector.

Adapters must supply calibrated, synchronized points already in a metric map
frame (x/y in metres). Cells are observed semantic evidence, not free space or
certified lane topology. No CARLA ground truth is silently treated as inference.
"""

from dataclasses import dataclass
import math


STATIC_LABELS = frozenset({
    "road", "lane_marking", "road_boundary", "sidewalk", "crosswalk",
    "stop_line", "traffic_light_fixture", "traffic_sign", "pole",
})
DYNAMIC_LABELS = frozenset({"vehicle", "pedestrian", "cyclist", "signal_state"})
SOURCES = frozenset({"oracle_sensor", "learned_prediction"})


@dataclass(frozen=True)
class Observation:
    timestamp_s: float
    frame_id: str
    source: str
    # HH_260906 - Each point is x, y, semantic label, confidence in [0, 1].
    points: tuple[tuple[float, float, str, float], ...]


class SemanticEvidenceMap:
    """Bounded sparse evidence grid; both streaming and replay call update()."""

    def __init__(self, frame_id: str, source: str, resolution_m: float = 0.2,
                 max_cells: int = 100_000):
        if not frame_id or source not in SOURCES:
            raise ValueError("explicit metric frame and valid provenance required")
        if not math.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError("resolution must be positive and finite")
        if isinstance(max_cells, bool) or not isinstance(max_cells, int) or max_cells < 1:
            raise ValueError("max_cells must be a positive integer")
        self.frame_id, self.source = frame_id, source
        self.resolution_m, self.max_cells = resolution_m, max_cells
        self.cells = {}
        self.timestamp_s = None
        self.dynamic = []

    def update(self, observation: Observation):
        # HH_260906 - Validate the complete frame before mutating accumulated evidence.
        if observation.frame_id != self.frame_id or observation.source != self.source:
            raise ValueError("frame/provenance mismatch")
        stamp = observation.timestamp_s
        if not math.isfinite(stamp) or (self.timestamp_s is not None and stamp <= self.timestamp_s):
            raise ValueError("timestamps must be finite and strictly increasing")
        votes, dynamic = {}, []
        for x, y, label, confidence in observation.points:
            if not all(math.isfinite(v) for v in (x, y, confidence)) or not 0 <= confidence <= 1:
                raise ValueError("invalid point or confidence")
            if label not in STATIC_LABELS | DYNAMIC_LABELS:
                raise ValueError("unknown label")
            if confidence == 0:
                continue
            if label in DYNAMIC_LABELS:
                dynamic.append((x, y, label, confidence))
                continue
            cell = (math.floor(x / self.resolution_m), math.floor(y / self.resolution_m))
            # HH_260906 - Dense points in one image must not count as independent frames.
            key = (cell, label)
            votes[key] = max(votes.get(key, 0), confidence)
        if len(set(self.cells) | {cell for cell, _ in votes}) > self.max_cells:
            raise ValueError("map capacity exceeded; segment the route explicitly")
        for (cell, label), confidence in votes.items():
            evidence = self.cells.setdefault(cell, {}).setdefault(label, [0, 0.0])
            evidence[0] += 1
            evidence[1] += confidence
        self.dynamic = dynamic  # Never persist moving objects as static road structure.
        self.timestamp_s = stamp

    def snapshot(self):
        return {
            "schema": "portable_e2e.semantic_evidence.v1",
            "frame_id": self.frame_id, "source": self.source,
            "timestamp_s": self.timestamp_s, "resolution_m": self.resolution_m,
            "unobserved_is_unknown": True, "hd_map_approved": False,
            "cells": [
                {"ix": cell[0], "iy": cell[1], "labels": {
                    label: {"observed_frames": values[0], "confidence_sum": values[1]}
                    for label, values in sorted(labels.items())}}
                for cell, labels in sorted(self.cells.items())
            ],
            "current_dynamic": list(self.dynamic),
        }


def replay(observations, **map_options):
    """HH_260906 - Replay uses exactly the live update path without looking ahead."""
    result = SemanticEvidenceMap(**map_options)
    for observation in observations:
        result.update(observation)
    return result
