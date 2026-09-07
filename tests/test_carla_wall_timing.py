"""HH_260906 - Test diagnostic clocks and full failure coverage without a simulator, images, GPU, or packages."""

import copy
import json

import pytest

from scripts.e2e.carla_wall_timing import CAMERAS, PHASES, STAGES, TimingError, WallTimingRecorder


class Clock:
    def __init__(self):
        self.ns = 0

    def __call__(self):
        return self.ns

    def advance(self, ns):
        self.ns += ns


@pytest.fixture
def timing():
    clock = Clock()
    recorder = WallTimingRecorder(enabled=True, clock_ns=clock, utc_now=lambda: "2026-09-08T00:00:00+00:00")
    return recorder, clock


def complete_tick(recorder, clock, index, *, phase="driving", camera=None, frame=None, timestamp=None):
    camera = index % 2 == 0 if camera is None else camera
    with recorder.tick(phase, index, camera_expected=camera) as tick:
        with tick.stage("world_tick_snapshot"):
            clock.advance(10_000_000)
            tick.bind_observation(100 + index if frame is None else frame, 5. + index * .05 if timestamp is None else timestamp)
        with tick.stage("observation_control"):
            clock.advance(1_000_000)
            tick.mark_state_recorded()
        if camera:
            for stage in ("camera_queue_wait", "jpeg_encode_write"):
                with tick.stage(stage):
                    for name in CAMERAS:
                        with tick.camera(name):
                            clock.advance(1_000_000)
                    if stage == "jpeg_encode_write":
                        tick.mark_camera_recorded()
        with tick.stage("control_rpc"):
            clock.advance(2_000_000)


def summary(recorder, success=True):
    rows = recorder.records
    return recorder.summarize(expected_state_frames=[r["frame"] for r in rows if r["state_recorded"]],
        expected_camera_frames=[r["frame"] for r in rows if r["camera_recorded"]], capture_succeeded=success,
        expected_phase_counts={phase: sum(r["phase"] == phase and r["state_recorded"] for r in rows) for phase in PHASES})


def test_disabled_mode_has_zero_clock_persistence_and_metadata_side_effects():
    def forbidden(*_):
        pytest.fail("disabled timing called a clock or persistence function")
    recorder = WallTimingRecorder(clock_ns=forbidden, utc_now=forbidden, persist=forbidden)
    with recorder.tick("ignored", None, camera_expected=None) as tick:
        with tick.stage("ignored"):
            with tick.camera("ignored"):
                tick.bind_observation(None, float("nan"))
                tick.mark_state_recorded()
                tick.mark_camera_recorded()
    assert recorder.records == []
    assert recorder.summarize(expected_state_frames=[], expected_camera_frames=[], capture_succeeded=False)["status"] == "DISABLED"


def test_complete_camera_tick_has_ordered_six_camera_spans_and_exact_total(timing):
    recorder, clock = timing
    complete_tick(recorder, clock, 0)
    row = recorder.records[0]
    assert row["status"] == "COMPLETE" and row["total_ns"] == 25_000_000 and row["unattributed_ns"] == 0
    assert row["frame"] == 100 and row["sim_timestamp"] == 5.
    assert all(span["status"] == "COMPLETE" for span in row["stages"].values())
    assert [c["camera"] for c in row["stages"]["camera_queue_wait"]["cameras"]] == list(CAMERAS)
    assert summary(recorder)["status"] == "COMPLETE_DIAGNOSTIC"
    assert summary(recorder)["stages"]["jpeg_encode_write"]["mean_ms"] == 6.
    assert summary(recorder)["per_camera"]["camera_queue_wait"][CAMERAS[0]]["count"] == 1


def test_all_phases_and_noncamera_ticks_remain_in_complete_ledger(timing):
    recorder, clock = timing
    for index, phase in enumerate(PHASES):
        complete_tick(recorder, clock, index, phase=phase)
        clock.advance(3_000_000)
    rows = recorder.records
    assert rows[1]["stages"]["camera_queue_wait"]["status"] == "NOT_SCHEDULED"
    report = summary(recorder)
    assert report["status"] == "COMPLETE_DIAGNOSTIC" and report["attempt_count"] == 3
    assert [report["phase_counts"][p]["attempts"] for p in PHASES] == [1, 1, 1]
    assert report["stages"]["camera_queue_wait"]["count"] == 2
    assert report["inter_tick_gap"]["count"] == 2 and report["inter_tick_gap"]["mean_ms"] == 3
    assert report["scope"]["gui_display_fps_measured"] is False and report["scope"]["learned_inference_measured"] is False
    assert report["native_wall_observation_hz"] == pytest.approx(2e9 / 44_000_000)
    assert report["simulation_seconds_per_wall_second"] == pytest.approx(.1e9 / 44_000_000)


@pytest.mark.parametrize("stage", STAGES)
def test_exceptions_retain_partial_tick_and_completed_prefix_and_propagate_original(timing, stage):
    recorder, clock = timing
    original = RuntimeError("private path must not enter timing records")
    with pytest.raises(RuntimeError) as caught:
        with recorder.tick("driving", 0, camera_expected=True) as tick:
            for name in STAGES:
                with tick.stage(name):
                    clock.advance(20_000_000)
                    if name == stage:
                        raise original
                    if name == "world_tick_snapshot":
                        tick.bind_observation(100, 5.)
                    elif name == "observation_control":
                        tick.mark_state_recorded()
                    elif name in ("camera_queue_wait", "jpeg_encode_write"):
                        for camera in CAMERAS:
                            with tick.camera(camera):
                                clock.advance(1)
                        if name == "jpeg_encode_write":
                            tick.mark_camera_recorded()
    assert caught.value is original
    row = recorder.records[0]
    assert row["error_type"] == "RuntimeError" and row["status"] == "PARTIAL_OR_FAILED"
    assert row["stages"][stage]["status"] == "FAILED"
    assert "private path" not in json.dumps(row)
    assert summary(recorder, False)["failed_or_incomplete_attempt_count"] == 1


def test_camera_timeout_retains_failed_child_and_unreached_cameras(timing):
    recorder, clock = timing
    with pytest.raises(TimeoutError):
        with recorder.tick("stationary_warmup", 0, camera_expected=True) as tick:
            with tick.stage("world_tick_snapshot"):
                tick.bind_observation(100, 5.)
            with tick.stage("observation_control"):
                tick.mark_state_recorded()
            with tick.stage("camera_queue_wait"):
                with tick.camera(CAMERAS[0]):
                    clock.advance(1_500_000_000)
                    raise TimeoutError()
    row = recorder.records[0]
    assert row["state_recorded"] and not row["camera_recorded"]
    assert len(row["stages"]["camera_queue_wait"]["cameras"]) == 1
    assert summary(recorder, False)["tick_duration_above_ms_counts"] == {"50": 1, "100": 1, "500": 1, "1000": 1}


@pytest.mark.parametrize("mode", ["stage_order", "nested", "camera_order", "duplicate_state", "nan_timestamp", "wrong_stage_bind"])
def test_invalid_instrumentation_never_produces_complete_evidence(timing, mode):
    recorder, clock = timing
    with pytest.raises(TimingError):
        with recorder.tick("driving", 0, camera_expected=True) as tick:
            if mode == "stage_order":
                with tick.stage("control_rpc"):
                    pass
            with tick.stage("world_tick_snapshot"):
                if mode == "nested":
                    with tick.stage("observation_control"):
                        pass
                tick.bind_observation(100, float("nan") if mode == "nan_timestamp" else 5.)
            with tick.stage("observation_control"):
                if mode == "wrong_stage_bind":
                    tick.bind_observation(101, 6.)
                tick.mark_state_recorded()
                if mode == "duplicate_state":
                    tick.mark_state_recorded()
            with tick.stage("camera_queue_wait"):
                with tick.camera(CAMERAS[-1]):
                    pass
    assert recorder.records[0]["status"] != "COMPLETE"


def test_missing_camera_children_or_stage_cannot_pass(timing):
    recorder, clock = timing
    with recorder.tick("driving", 0, camera_expected=True) as tick:
        with tick.stage("world_tick_snapshot"):
            tick.bind_observation(100, 5.)
        with tick.stage("observation_control"):
            tick.mark_state_recorded()
        with tick.stage("camera_queue_wait"):
            pass
    assert summary(recorder)["status"] == "PARTIAL_OR_FAILED_DIAGNOSTIC"


def test_persistence_failure_is_disclosed_without_losing_row_or_masking_capture_exception(timing):
    recorder, clock = timing
    def fail(row):
        row.clear()
        clock.advance(4_000_000)
        raise OSError("secret filesystem path")
    recorder.persist = fail
    complete_tick(recorder, clock, 0)
    assert recorder.records[0]["frame"] == 100
    result = summary(recorder)
    assert result["status"] == "PARTIAL_OR_FAILED_DIAGNOSTIC" and result["persistence"]["mean_ms"] == 4.
    assert result["persistence_errors"] == [dict(sequence=1, operation="persist", error_type="OSError")]
    assert "secret" not in json.dumps(result)


def test_returned_records_are_detached_and_json_finite(timing):
    recorder, clock = timing
    complete_tick(recorder, clock, 0)
    original = copy.deepcopy(recorder.records)
    recorder.records[0]["frame"] = 999
    assert recorder.records == original
    json.dumps(summary(recorder), allow_nan=False)


@pytest.mark.parametrize("field", ["state", "camera"])
def test_full_original_frame_ledger_is_required_not_successful_subset(timing, field):
    recorder, clock = timing
    complete_tick(recorder, clock, 0)
    report = recorder.summarize(expected_state_frames=[] if field == "state" else [100],
        expected_camera_frames=[] if field == "camera" else [100], capture_succeeded=True,
        expected_phase_counts=dict(stationary_warmup=0, driving=1, stationary_tail=0))
    assert not report["flags"][field + "_ledger_exact"]


@pytest.mark.parametrize("kind", ["tick_gap", "frame_gap", "sim_dt", "phase_reversal", "camera_schedule"])
def test_cadence_and_full_phase_order_are_independent_of_wall_speed(timing, kind):
    recorder, clock = timing
    complete_tick(recorder, clock, 0, phase="driving")
    complete_tick(recorder, clock, 2 if kind == "tick_gap" else 1,
        phase="stationary_warmup" if kind == "phase_reversal" else "driving",
        frame=105 if kind == "frame_gap" else 101, timestamp=5.1 if kind == "sim_dt" else 5.05,
        camera=True if kind == "camera_schedule" else False)
    assert summary(recorder)["status"] == "PARTIAL_OR_FAILED_DIAGNOSTIC"


def test_clock_reversal_is_invalid_and_does_not_become_negative_latency(timing):
    recorder, clock = timing
    clock.ns = 100
    with recorder.tick("driving", 0, camera_expected=True) as tick:
        with tick.stage("world_tick_snapshot"):
            clock.ns = 99
    row = recorder.records[0]
    assert row["total_ns"] is None and row["stages"]["world_tick_snapshot"]["duration_ns"] is None
    assert summary(recorder, False)["status"] == "PARTIAL_OR_FAILED_DIAGNOSTIC"


@pytest.mark.parametrize("kwargs", [dict(enabled=1), dict(physics_hz=float("inf")), dict(camera_hz=0),
    dict(physics_hz=True), dict(camera_hz=7)])
def test_invalid_options_fail_before_use(kwargs):
    with pytest.raises(TimingError):
        WallTimingRecorder(**kwargs)


def test_missing_original_phase_ledger_cannot_claim_full_diagnostic(timing):
    recorder, clock = timing
    complete_tick(recorder, clock, 0)
    with pytest.raises(TimingError, match="all three"):
        recorder.summarize(expected_state_frames=[100], expected_camera_frames=[100], capture_succeeded=True)


def test_phase_count_mismatch_fails_even_with_same_frame_list(timing):
    recorder, clock = timing
    complete_tick(recorder, clock, 0)
    report = recorder.summarize(expected_state_frames=[100], expected_camera_frames=[100], capture_succeeded=True,
        expected_phase_counts=dict(stationary_warmup=1, driving=0, stationary_tail=0))
    assert report["flags"]["phase_ledger_exact"] is False


def test_utc_clock_jump_cannot_change_monotonic_duration(timing):
    recorder, clock = timing
    utc = iter(("2026-09-08T00:00:01+00:00", "2026-09-08T00:00:00+00:00"))
    recorder.utc_now = lambda: next(utc)
    complete_tick(recorder, clock, 0)
    assert recorder.records[0]["total_ns"] == 25_000_000
    assert summary(recorder)["first_utc"] > summary(recorder)["last_utc"]
