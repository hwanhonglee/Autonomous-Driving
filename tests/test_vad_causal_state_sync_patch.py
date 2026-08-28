from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_builds_apply_causal_sync_after_frame_assembly() -> None:
    for name in ("build.sh", "build_full.sh"):
        source = (ROOT / "scripts/e2e" / name).read_text(encoding="utf-8")
        frame = source.index("apply_vad_frame_assembly_patch.sh")
        causal = source.index("apply_vad_causal_state_sync_patch.sh")
        carla = source.index("apply_carla_fast_sensor_patch.sh")
        assert frame < causal < carla


def test_causal_sync_patch_has_stable_applied_markers() -> None:
    apply_script = (
        ROOT / "scripts/e2e/apply_vad_causal_state_sync_patch.sh"
    ).read_text(encoding="utf-8")
    patch = (
        ROOT / "patches/autoware_tensorrt_vad_causal_state_sync.patch"
    ).read_text(encoding="utf-8")

    for marker in (
        "max_vehicle_state_samples",
        "find_latest_causal_kinematic_state",
        "kinematic_states_.upper_bound",
        "SelectsLatestCausalVehicleStateDespiteOutOfOrderCallbacks",
        "WaitsForCausalVehicleStateAndRetainsFutureSamplesForNextFrame",
    ):
        assert marker in apply_script
        assert marker in patch


def test_runtime_selects_only_state_at_or_before_camera_anchor() -> None:
    source = (
        ROOT
        / "src/universe/autoware_universe/e2e/autoware_tensorrt_vad/lib/"
        "synchronization_strategy.cpp"
    ).read_text(encoding="utf-8")
    assert "kinematic_states_.upper_bound(anchor_stamp_ns)" in source
    assert "accelerations_.upper_bound(anchor_stamp_ns)" in source
    assert "std::prev(after_anchor)->second" in source
