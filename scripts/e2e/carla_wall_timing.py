"""HH_260906 - Optional wall-clock diagnostics only; never change simulator clocks, control, or capture admission."""

from __future__ import annotations

from contextlib import contextmanager, nullcontext
from copy import deepcopy
from datetime import datetime, timezone
import math
import time

PHASES = ("stationary_warmup", "driving", "stationary_tail")
CAMERAS = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
STAGES = ("world_tick_snapshot", "observation_control", "camera_queue_wait", "jpeg_encode_write", "control_rpc")


class TimingError(ValueError):
    """HH_260906 - Identify malformed diagnostic instrumentation without granting any capture qualification."""


def _require(passed, message):
    if not passed:
        raise TimingError(message)


def _utc_now():
    return datetime.now(timezone.utc).isoformat()


def _finite(value):
    _require(type(value) in (int, float) and math.isfinite(value), "timing metadata must be finite numeric values")
    return float(value)


def _distribution(values):
    values = sorted(values)
    if not values:
        return {"count": 0, "mean_ms": None, "p50_ms": None, "p95_ms": None, "p99_ms": None, "maximum_ms": None}
    def quantile(fraction):
        # HH_260906 - Nearest-rank percentiles disclose observed values and do not invent interpolated latency samples.
        return values[max(0, math.ceil(len(values) * fraction) - 1)] / 1_000_000
    return {"count": len(values), "mean_ms": sum(values) / len(values) / 1_000_000,
        "p50_ms": quantile(.5), "p95_ms": quantile(.95), "p99_ms": quantile(.99), "maximum_ms": values[-1] / 1_000_000}


class _DisabledTick:
    """HH_260906 - Disabled instrumentation must perform no clock, persistence, simulator, or metadata operations."""

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False

    def stage(self, *_):
        return nullcontext(self)

    def camera(self, *_):
        return nullcontext(self)

    def bind_observation(self, *_):
        pass

    def mark_state_recorded(self):
        pass

    def mark_camera_recorded(self):
        pass


class WallTimingRecorder:
    """HH_260906 - Record every attempted native tick; a persistence callback receives detached records and may fail safely."""

    def __init__(self, *, enabled=False, physics_hz=20., camera_hz=10., clock_ns=time.perf_counter_ns, utc_now=_utc_now, persist=None):
        _require(type(enabled) is bool, "enabled must be boolean")
        _require(callable(clock_ns) and callable(utc_now) and (persist is None or callable(persist)), "invalid timing callback")
        self.enabled, self.clock_ns, self.utc_now, self.persist = enabled, clock_ns, utc_now, persist
        self.physics_hz, self.camera_hz = _finite(physics_hz), _finite(camera_hz)
        _require(self.physics_hz > 0 and self.camera_hz > 0, "cadence metadata must be positive")
        self.interval = round(self.physics_hz / self.camera_hz)
        _require(self.interval >= 1 and abs(self.physics_hz / self.camera_hz - self.interval) <= 1e-9, "camera cadence must divide native cadence")
        self._records, self._persistence, self._errors = [], [], []
        self._active = None

    def _now(self):
        value = self.clock_ns()
        _require(type(value) is int and value >= 0, "perf_counter_ns must return a nonnegative integer")
        return value

    def tick(self, phase, tick_index, *, camera_expected):
        if not self.enabled:
            return _DisabledTick()
        _require(phase in PHASES and type(tick_index) is int and tick_index >= 0 and type(camera_expected) is bool,
                 "invalid phase, attempted tick index, or camera schedule")
        return _Tick(self, phase, tick_index, camera_expected)

    @property
    def records(self):
        return deepcopy(self._records)

    def _retain(self, row):
        self._records.append(deepcopy(row))
        if self.persist is None:
            return
        # HH_260906 - Journal overhead is measured separately and stays visible in later inter-tick/whole-span rates.
        started = None
        try:
            started = self._now()
            self.persist(deepcopy(row))
        except BaseException as error:
            self._errors.append({"sequence": row["sequence"], "operation": "persist", "error_type": type(error).__name__})
            if not isinstance(error, Exception):
                raise
        finally:
            try:
                ended = self._now()
                if started is not None:
                    _require(ended >= started, "persistence clock moved backwards")
                    self._persistence.append(ended - started)
            except Exception as error:
                self._errors.append({"sequence": row["sequence"], "operation": "persist_clock", "error_type": type(error).__name__})

    def summarize(self, *, expected_state_frames, expected_camera_frames, capture_succeeded, expected_phase_counts=None):
        """HH_260906 - Compare timing coverage with complete capture ledgers, retaining failed and noncamera tick attempts."""
        _require(type(capture_succeeded) is bool, "capture_succeeded must be boolean")
        if not self.enabled:
            return {"schema": "carla.expert_wall_timing.v1", "status": "DISABLED", "timing_enabled": False}
        _require(self._active is None, "cannot finalize timing while a tick is active")
        rows = self.records
        for values in (expected_state_frames, expected_camera_frames):
            _require(isinstance(values, (list, tuple)) and all(type(value) is int and value >= 0 for value in values), "invalid expected frame ledger")
            _require(len(set(values)) == len(values), "duplicate expected capture frame")
        _require(isinstance(expected_phase_counts, dict) and set(expected_phase_counts) == set(PHASES)
                 and all(type(count) is int and count >= 0 for count in expected_phase_counts.values()),
                 "all three original capture phase counts are required")
        states = [row for row in rows if row["state_recorded"]]
        cameras = [row for row in rows if row["camera_recorded"]]
        observed = [row for row in rows if row["frame"] is not None]
        flags = {"nonempty_attempts": bool(rows), "all_attempts_complete": all(row["status"] == "COMPLETE" for row in rows),
            "state_ledger_exact": [row["frame"] for row in states] == list(expected_state_frames),
            "camera_ledger_exact": [row["frame"] for row in cameras] == list(expected_camera_frames),
            "phase_ledger_exact": all(sum(row["phase"] == phase for row in states) == count for phase, count in expected_phase_counts.items())
                and sum(expected_phase_counts.values()) == len(expected_state_frames),
            "camera_schedule_exact": all(row["camera_expected"] == (row["tick_index"] % self.interval == 0) for row in rows),
            "consecutive_attempt_indices": [row["tick_index"] for row in rows] == list(range(len(rows))),
            "ordered_phase_progression": all(PHASES.index(b["phase"]) >= PHASES.index(a["phase"]) for a, b in zip(rows, rows[1:])),
            "no_tick_overlap_or_clock_reversal": all(a["end_ns"] is not None and b["start_ns"] >= a["end_ns"] for a, b in zip(rows, rows[1:])),
            "no_persistence_errors": not self._errors}
        total = [row["total_ns"] for row in rows if row["total_ns"] is not None]
        stage_stats = {}
        for name in STAGES:
            stage_stats[name] = _distribution([row["stages"][name]["duration_ns"] for row in rows
                if row["stages"][name]["duration_ns"] is not None and row["stages"][name]["status"] != "NOT_SCHEDULED"])
        gaps = [b["start_ns"] - a["end_ns"] for a, b in zip(rows, rows[1:]) if a["end_ns"] is not None and b["start_ns"] >= a["end_ns"]]
        def throughput(samples, field):
            if len(samples) < 2 or samples[-1][field] is None or samples[0][field] is None:
                return None
            elapsed = samples[-1][field] - samples[0][field]
            return (len(samples) - 1) * 1e9 / elapsed if elapsed > 0 else None
        native_hz = throughput(observed, "observation_wall_ns")
        camera_hz = throughput(cameras, "camera_recorded_wall_ns")
        sim_span = observed[-1]["sim_timestamp"] - observed[0]["sim_timestamp"] if len(observed) > 1 else None
        wall_span = observed[-1]["observation_wall_ns"] - observed[0]["observation_wall_ns"] if len(observed) > 1 else None
        sim_deltas = [b["sim_timestamp"] - a["sim_timestamp"] for a, b in zip(observed, observed[1:])]
        flags["observed_native_frame_chain"] = all(b["frame"] == a["frame"] + 1 for a, b in zip(observed, observed[1:]))
        flags["native_sim_cadence_matches_declared"] = all(abs(delta - 1 / self.physics_hz) <= 1e-6 for delta in sim_deltas)
        flags["camera_sim_cadence_matches_declared"] = all(b["frame"] - a["frame"] == self.interval
            and abs(b["sim_timestamp"] - a["sim_timestamp"] - 1 / self.camera_hz) <= 1e-6 for a, b in zip(cameras, cameras[1:]))
        combined = [sum(row["stages"][name]["duration_ns"] for name in ("observation_control", "control_rpc"))
                    for row in rows if all(row["stages"][name]["duration_ns"] is not None for name in ("observation_control", "control_rpc"))]
        per_camera = {stage: {camera: _distribution([child["duration_ns"] for row in rows for child in row["stages"][stage]["cameras"]
            if child["camera"] == camera and child["duration_ns"] is not None]) for camera in CAMERAS}
            for stage in ("camera_queue_wait", "jpeg_encode_write")}
        return {"schema": "carla.expert_wall_timing.v1", "status": "COMPLETE_DIAGNOSTIC" if capture_succeeded and all(flags.values()) else "PARTIAL_OR_FAILED_DIAGNOSTIC",
            "timing_enabled": True, "capture_succeeded": capture_succeeded, "flags": flags,
            "attempt_count": len(rows), "complete_attempt_count": sum(row["status"] == "COMPLETE" for row in rows),
            "failed_or_incomplete_attempt_count": sum(row["status"] != "COMPLETE" for row in rows),
            "first_utc": rows[0]["start_utc"] if rows else None, "last_utc": rows[-1]["end_utc"] if rows else None,
            "phase_counts": {phase: {"attempts": sum(row["phase"] == phase for row in rows),
                "states": sum(row["phase"] == phase for row in states), "camera_anchors": sum(row["phase"] == phase for row in cameras)} for phase in PHASES},
            "native_wall_observation_hz": native_hz, "camera_bundle_wall_completion_hz": camera_hz,
            "simulation_seconds_per_wall_second": sim_span * 1e9 / wall_span if wall_span and wall_span > 0 else None,
            "observed_native_sim_dt_min_seconds": min(sim_deltas) if sim_deltas else None,
            "observed_native_sim_dt_max_seconds": max(sim_deltas) if sim_deltas else None,
            "total_tick": _distribution(total), "stages": stage_stats, "inter_tick_gap": _distribution(gaps),
            "observation_control_plus_rpc": _distribution(combined), "per_camera": per_camera,
            "unattributed_in_tick": _distribution([row["unattributed_ns"] for row in rows if row.get("unattributed_ns") is not None]),
            "persistence": _distribution(self._persistence), "persistence_errors": deepcopy(self._errors),
            "tick_duration_above_ms_counts": {str(ms): sum(value > ms * 1_000_000 for value in total) for ms in (50, 100, 500, 1000)},
            "scope": {"all_attempts_retained": True, "sensor_timestamps_modified": False, "gui_display_fps_measured": False,
                "learned_inference_measured": False, "hardware_load_measured": False, "dataset_admission": False,
                "bootstrap_and_setup_teardown_in_native_tick_totals": False},
            "notice": "Monotonic durations measure collector work, not GUI display FPS or learned inference. Sequential camera waits are residual queue/matching time, not sensor generation latency. Tick totals exclude their own diagnostic persistence; inter-tick and observation-span rates include intervening work. Failed spans are retained; no row or sensor timestamp is rewritten."}


class _Tick:
    """HH_260906 - Require ordered disjoint stage spans and exact six-camera child coverage for scheduled camera ticks."""

    def __init__(self, recorder, phase, tick_index, camera_expected):
        self.recorder, self.phase, self.index, self.camera_expected = recorder, phase, tick_index, camera_expected
        self.active_stage, self.child_active, self.finished_stages = None, False, []
        self._manual_stage, self._manual_camera = None, None
        self.row = None

    def __enter__(self):
        _require(self.recorder._active is None, "overlapping tick scopes are forbidden")
        started, utc = self.recorder._now(), self.recorder.utc_now()
        self.row = {"sequence": len(self.recorder._records) + 1, "tick_index": self.index, "phase": self.phase,
            "camera_expected": self.camera_expected, "start_ns": started, "start_utc": utc,
            "end_ns": None, "end_utc": None, "total_ns": None, "frame": None, "sim_timestamp": None,
            "observation_wall_ns": None, "camera_recorded_wall_ns": None, "state_recorded": False, "camera_recorded": False,
            "status": "PARTIAL", "error_type": None, "instrumentation_errors": [],
            "unattributed_ns": None,
            "stages": {name: {"status": "NOT_REACHED", "start_ns": None, "end_ns": None, "duration_ns": None, "cameras": []} for name in STAGES}}
        for name in ("camera_queue_wait", "jpeg_encode_write"):
            if not self.camera_expected:
                self.row["stages"][name].update(status="NOT_SCHEDULED", duration_ns=0)
        self.recorder._active = self
        return self

    def _end_span(self, span, error):
        try:
            end = self.recorder._now()
            _require(end >= span["start_ns"], "monotonic stage clock reversed")
            span.update(end_ns=end, duration_ns=end - span["start_ns"], status="FAILED" if error else "COMPLETE")
        except Exception as timing_error:
            span.update(status="INVALID_TIMING", duration_ns=None)
            self.row["instrumentation_errors"].append(type(timing_error).__name__)

    @contextmanager
    def stage(self, name):
        order = [name for name in STAGES if self.camera_expected or name not in ("camera_queue_wait", "jpeg_encode_write")]
        _require(self.row is not None and self.recorder._active is self and self.active_stage is None
                 and len(self.finished_stages) < len(order) and name == order[len(self.finished_stages)], "missing, repeated, nested, or reordered stage")
        span = self.row["stages"][name]
        span["start_ns"], self.active_stage = self.recorder._now(), name
        error = None
        try:
            yield self
        except BaseException as caught:
            error = caught
            raise
        finally:
            self._end_span(span, error)
            self.active_stage = None
            self.finished_stages.append(name)

    def begin_stage(self, name):
        """HH_260906 - Support minimal collector hooks; tick exit closes an unfinished stage with the original failure."""
        _require(self._manual_stage is None, "manual stage already active")
        scope = self.stage(name)
        scope.__enter__()
        self._manual_stage = scope

    def end_stage(self):
        _require(self._manual_stage is not None and self._manual_camera is None, "no manual stage or unfinished camera child")
        scope, self._manual_stage = self._manual_stage, None
        scope.__exit__(None, None, None)

    def begin_camera(self, name):
        _require(self._manual_camera is None, "manual camera span already active")
        scope = self.camera(name)
        scope.__enter__()
        self._manual_camera = scope

    def end_camera(self):
        _require(self._manual_camera is not None, "no manual camera span active")
        scope, self._manual_camera = self._manual_camera, None
        scope.__exit__(None, None, None)

    @contextmanager
    def camera(self, name):
        _require(self.active_stage in ("camera_queue_wait", "jpeg_encode_write") and not self.child_active,
                 "camera span requires one active camera stage")
        spans = self.row["stages"][self.active_stage]["cameras"]
        _require(len(spans) < len(CAMERAS) and name == CAMERAS[len(spans)], "camera child order or multiplicity mismatch")
        span = {"camera": name, "start_ns": self.recorder._now(), "end_ns": None, "duration_ns": None, "status": "PARTIAL"}
        spans.append(span)
        self.child_active, error = True, None
        try:
            yield self
        except BaseException as caught:
            error = caught
            raise
        finally:
            self._end_span(span, error)
            self.child_active = False

    def bind_observation(self, frame, sim_timestamp):
        _require(self.active_stage == "world_tick_snapshot" and self.row["frame"] is None
                 and type(frame) is int and frame >= 0, "observation must bind exactly once inside world tick/snapshot stage")
        self.row.update(frame=frame, sim_timestamp=_finite(sim_timestamp), observation_wall_ns=self.recorder._now())

    def mark_state_recorded(self):
        _require(self.active_stage == "observation_control" and self.row["frame"] is not None and not self.row["state_recorded"],
                 "state marker must follow the single native state append")
        self.row["state_recorded"] = True

    def mark_camera_recorded(self):
        _require(self.active_stage == "jpeg_encode_write" and self.camera_expected and self.row["state_recorded"]
                 and not self.row["camera_recorded"], "camera marker must follow the scheduled camera record append")
        self.row.update(camera_recorded=True, camera_recorded_wall_ns=self.recorder._now())

    def __exit__(self, exception_type, exception, _traceback):
        try:
            for field in ("_manual_camera", "_manual_stage"):
                scope = getattr(self, field)
                if scope is not None:
                    setattr(self, field, None)
                    scope.__exit__(exception_type, exception, _traceback)
            self.row["end_utc"] = self.recorder.utc_now()
            self._end_span(self.row, exception)
            self.row["total_ns"] = self.row.pop("duration_ns")
            self.row["error_type"] = exception_type.__name__ if exception_type else None
            complete = self.row["status"] == "COMPLETE" and self.row["state_recorded"] and self.row["camera_recorded"] == self.camera_expected
            prior_end = self.row["start_ns"]
            for name in STAGES:
                span = self.row["stages"][name]
                if span["status"] == "NOT_SCHEDULED":
                    continue
                complete &= span["status"] == "COMPLETE" and span["start_ns"] is not None and span["end_ns"] is not None
                if span["start_ns"] is not None and span["end_ns"] is not None:
                    complete &= prior_end <= span["start_ns"] <= span["end_ns"] <= self.row["end_ns"]
                    prior_end = span["end_ns"]
                if name in ("camera_queue_wait", "jpeg_encode_write"):
                    complete &= [child["camera"] for child in span["cameras"]] == list(CAMERAS)
                    complete &= all(child["status"] == "COMPLETE" and span["start_ns"] <= child["start_ns"] <= child["end_ns"] <= span["end_ns"] for child in span["cameras"])
                    complete &= all(a["end_ns"] is not None and b["start_ns"] >= a["end_ns"] for a, b in zip(span["cameras"], span["cameras"][1:]))
            for wall_field, stage_name in (("observation_wall_ns", "world_tick_snapshot"), ("camera_recorded_wall_ns", "jpeg_encode_write")):
                value, span = self.row[wall_field], self.row["stages"][stage_name]
                if value is not None:
                    complete &= span["start_ns"] is not None and span["end_ns"] is not None and span["start_ns"] <= value <= span["end_ns"]
            self.row["status"] = "COMPLETE" if complete and not self.row["instrumentation_errors"] else "PARTIAL_OR_FAILED"
            measured = [span["duration_ns"] for span in self.row["stages"].values() if span["duration_ns"] is not None]
            if self.row["total_ns"] is not None and self.row["total_ns"] >= sum(measured):
                self.row["unattributed_ns"] = self.row["total_ns"] - sum(measured)
        except Exception as error:
            self.row["status"] = "INVALID_TIMING"
            self.row["instrumentation_errors"].append(type(error).__name__)
        finally:
            self.recorder._active = None
            self.recorder._retain(self.row)
        return False
