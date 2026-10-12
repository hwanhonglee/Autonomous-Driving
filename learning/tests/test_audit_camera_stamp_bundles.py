from scripts.analysis.audit_camera_stamp_bundles import front_anchored_coverage
from scripts.analysis.audit_camera_stamp_bundles import gap_histogram
from scripts.analysis.audit_camera_stamp_bundles import summarize


CAMERAS = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)


def test_gap_histogram_quantizes_simulation_ticks() -> None:
    assert gap_histogram([0, 200_000_001, 450_000_000]) == {"200ms": 1, "250ms": 1}


def test_summary_counts_exact_and_near_bundles() -> None:
    stamps = {camera: [0, 200_000_000, 400_000_000] for camera in CAMERAS}
    stamps["CAM_BACK_RIGHT"] = [50_000_000, 200_000_000, 400_000_000]

    result = summarize(stamps)

    assert result["exact_six_camera_count"] == 2
    assert result["within_50ms"]["matched"] == 3
    assert result["within_50ms"]["maximum_stamp_span_ms"] == 50.0


def test_front_anchored_coverage_rejects_outside_tolerance() -> None:
    stamps = {camera: [0, 200_000_000] for camera in CAMERAS}
    stamps["CAM_BACK"] = [100_000_000, 300_000_000]

    result = front_anchored_coverage(stamps, tolerance_ns=50_000_000)

    assert result["matched"] == 0
