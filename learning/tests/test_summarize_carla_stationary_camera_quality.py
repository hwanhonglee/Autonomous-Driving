"""HH_260906 - Test stationary evidence accounting and pixel preservation using synthetic temporary fixtures only."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
from PIL import Image

from scripts.e2e import summarize_carla_stationary_camera_quality as audit


def states_fixture():
    """HH_260906 - A stationary160-tick fixture never opens a CARLA client or writes a real evidence tree."""
    return [dict(tick=tick, frame=1000 + tick, timestamp=tick * .05, planar_speed_mps=0.,
                 actor_transform={name: 0. for name in ("x", "y", "z", "roll", "pitch", "yaw")},
                 reported_control=dict(throttle=0., brake=1., steer=0., gear=0,
                                       hand_brake=False, reverse=False, manual_gear_shift=False)) for tick in range(1, 161)]


def test_all160_states_and_full_brake_pass():
    audit.validate_states(states_fixture())


@pytest.mark.parametrize("change", ["missing", "duplicate_tick", "wrong_frame", "timestamp_gap", "moving", "nan", "throttle", "released_brake", "steering", "handbrake"])
def test_state_incompleteness_or_motion_cannot_be_promoted(change):
    states = states_fixture()
    if change == "missing": states.pop()
    if change == "duplicate_tick": states[20]["tick"] = 20
    if change == "wrong_frame": states[20]["frame"] += 1
    if change == "timestamp_gap": states[20]["timestamp"] += .05
    if change == "moving": states[70]["planar_speed_mps"] = .10001
    if change == "nan": states[1]["planar_speed_mps"] = float("nan")
    if change == "throttle": states[1]["reported_control"]["throttle"] = .1
    if change == "released_brake": states[1]["reported_control"]["brake"] = .9
    if change == "steering": states[1]["reported_control"]["steer"] = .1
    if change == "handbrake": states[1]["reported_control"]["hand_brake"] = True
    with pytest.raises(audit.base.EvidenceError): audit.validate_states(states)


@pytest.fixture
def camera_fixture(tmp_path):
    """HH_260906 - Synthetic solid-color JPEGs stand in for saved bundles; they are never published as observations."""
    states, frames = states_fixture(), []
    images = tmp_path / "camera_audit/images"
    images.mkdir(parents=True)
    for tick in audit.TICKS:
        entry = dict(tick=tick, frame=1000 + tick, timestamp=tick * .05, complete=True, images={})
        for index, camera in enumerate(audit.CAMERAS):
            name = f"tick_{tick:03d}_{camera}.jpg"
            Image.new("RGB", (640, 360), (index * 30, tick, 50)).save(images / name, format="JPEG", quality=95)
            entry["images"][camera] = dict(path="images/" + name, timestamp=tick * .05, sha256=audit.base.sha(images / name))
        frames.append(entry)
    return tmp_path, states, frames


def test_all24_images_hashes_dimensions_and_anchors_are_verified(camera_fixture):
    root, states, frames = camera_fixture
    ledger = []
    result = audit.validate_frames(frames, states, root, ledger)
    assert len(result) == len(ledger) == 24
    assert {row["tick"] for row in result} == set(audit.TICKS)
    assert all(row["width"] == 640 and row["height"] == 360 for row in result)


@pytest.mark.parametrize("change", ["missing_anchor", "missing_camera", "incomplete_anchor", "wrong_timestamp", "wrong_hash", "unsafe_path", "extra_image", "wrong_dimensions"])
def test_camera_payload_drift_or_path_escape_is_rejected(camera_fixture, change):
    root, states, frames = camera_fixture
    image = frames[0]["images"]["CAM_FRONT"]
    if change == "missing_anchor": frames.pop()
    if change == "missing_camera": del frames[0]["images"]["CAM_BACK"]
    if change == "incomplete_anchor": frames[0]["complete"] = False
    if change == "wrong_timestamp": image["timestamp"] += .01
    if change == "wrong_hash": image["sha256"] = "a" * 64
    if change == "unsafe_path": image["path"] = "../foreign.jpg"
    if change == "extra_image": (root / "camera_audit/images/extra.jpg").write_bytes(b"extra")
    if change == "wrong_dimensions":
        path = root / "camera_audit" / image["path"]
        Image.new("RGB", (641, 360)).save(path, format="JPEG")
        image["sha256"] = audit.base.sha(path)
    with pytest.raises(audit.base.EvidenceError): audit.validate_frames(frames, states, root, [])


def test_front_png_preserves_decoded_jpeg_pixels_exactly(camera_fixture):
    root, _, _ = camera_fixture
    source = root / "camera_audit/images/tick_100_CAM_FRONT.jpg"
    destination = root / "native.png"
    result = audit.native_front_png(source, destination)
    with Image.open(destination) as png:
        assert png.size == (640, 360) and png.tobytes() == audit.decode_jpeg(source).tobytes()
        assert result["decoded_rgb_sha256"] == hashlib.sha256(png.tobytes()).hexdigest()
    with pytest.raises(audit.base.EvidenceError): audit.native_front_png(source, destination)


def test_contact_sheet_keeps_all12_full_fov_cells_at_native_scale(camera_fixture):
    root, _, _ = camera_fixture
    output = root / "contact.png"
    result = audit.contact_sheet(root, root, 100, output)
    assert len(result["cells"]) == 12 and result["width"] == 1920 and result["height"] == 1600
    with Image.open(output) as sheet:
        for cell in result["cells"]:
            pixels = sheet.crop((cell["x"], cell["y"], cell["x"] + 640, cell["y"] + 360))
            original = audit.decode_jpeg(root / f"camera_audit/images/tick_100_{cell['camera']}.jpg")
            assert pixels.tobytes() == original.tobytes()


@pytest.mark.parametrize("kind", ["directory", "file", "dangling_link"])
def test_publication_never_overwrites_a_previous_category(tmp_path, monkeypatch, kind):
    output = tmp_path / "published"
    if kind == "directory": output.mkdir()
    if kind == "file": output.write_text("keep")
    if kind == "dangling_link": output.symlink_to(tmp_path / "missing")
    monkeypatch.setattr(audit, "verify_pair", lambda *_: pytest.fail("must refuse output before reading evidence"))
    with pytest.raises(audit.base.EvidenceError, match="already exists"):
        audit.publish(tmp_path / "low", tmp_path / "epic", output)


def test_fixed_rig_and_source_sets_are_independent_of_live_probe_imports():
    assert len(audit.SOURCES) == len(set(audit.SOURCES)) == 11
    assert audit.WORKER in audit.SOURCES
    assert audit.TICKS == (100, 120, 140, 160) and len(audit.CAMERAS) == 6


@pytest.mark.parametrize("mutation", [None, "source", "rig", "settings", "position", "rotation"])
def test_low_epic_pair_binds_code_rig_timing_and_same_tick_pose(monkeypatch, mutation):
    """HH_260906 - A quality comparison cannot silently combine another code revision or camera viewpoint."""
    common = {"archived_source_sha256": {"worker": "a" * 64}, "source_commit": "b" * 40,
              "camera_specs": [{"name": "CAM_FRONT"}], "world_settings": {"fixed_delta_seconds": .05}}
    low, epic = copy.deepcopy(common), copy.deepcopy(common)
    low_states, epic_states = states_fixture(), states_fixture()
    if mutation == "source": epic["archived_source_sha256"]["worker"] = "c" * 64
    if mutation == "rig": epic["camera_specs"][0]["name"] = "CAM_BACK"
    if mutation == "settings": epic["world_settings"]["fixed_delta_seconds"] = .1
    if mutation == "position": epic_states[99]["actor_transform"]["x"] = .001
    if mutation == "rotation": epic_states[119]["actor_transform"]["yaw"] = .001
    monkeypatch.setattr(audit, "verify_run", lambda _, quality: (low, low_states) if quality == "Low" else (epic, epic_states))
    if mutation is not None:
        with pytest.raises(audit.base.EvidenceError): audit.verify_pair(Path("low"), Path("epic"))
    else:
        result = audit.verify_pair(Path("low"), Path("epic"))
        assert result["total_states"] == 320 and result["total_jpeg_images"] == 48
        assert all(row["position_difference_m"] == 0 for row in result["same_tick_pose_differences"])


def test_pilot_declaration_time_error_is_retained_not_certified_or_rewritten(tmp_path, monkeypatch):
    """HH_260906 - File creation metadata cannot repair a declaration timestamp that follows first admission."""
    low, epic = tmp_path / "low/run_001", tmp_path / "epic/run_001"
    low.mkdir(parents=True)
    epic.mkdir(parents=True)
    sources = {audit.WORKER: "a" * 64, "scripts/e2e/collect_carla_vad_expert.py": "b" * 64}
    comparison = {"runs": [{"source_commit": "c" * 40, "archived_source_sha256": sources}]}
    pilot = {"declared_at_utc": "2026-09-07T19:04:00Z", "source_commit": "c" * 40, "worker_sha256": "a" * 64,
        "collector_sha256": "b" * 64, "route_sha256": audit.ROUTE_SHA, "mapping_sha256": audit.MAPPING_SHA,
        "calibration_sha256": audit.CALIBRATION_SHA, "outputs_in_order": ["low/run_001", "epic/run_001"],
        "qualities_in_order": ["Low", "Epic"], "maximum_attempts_per_quality": 1, "automatic_retry": False,
        "map": "Town07", "weather": "ClearNoon", "vehicle": "vehicle.toyota.prius", "physics_hz": 20,
        "native_camera_sensor_tick_seconds": 0, "total_ticks_per_arm": 160, "saved_ticks_per_arm": list(audit.TICKS),
        "camera_count": 6, "command": {"throttle": 0., "brake": 1., "steer": 0.}, "training_data_approved": False,
        "learned_model_control": False, "physical_actuation_timing_proven": False, "camera_fps_benchmark": False}
    raw = (json.dumps(pilot) + "\n").encode()
    (tmp_path / "pilot_plan.json").write_bytes(raw)
    (low / "owner_plan.json").write_text(json.dumps({"planned_at_utc": "2026-09-07T19:03:53.568809+00:00"}))
    (epic / "owner_plan.json").write_text(json.dumps({"planned_at_utc": "2026-09-07T19:04:36.924019+00:00"}))
    monkeypatch.setattr(audit.subprocess, "check_output", lambda *_, **__: "2026-09-07 19:03:53.126584447 +0000\n")
    result = audit.pilot_plan_provenance(low, epic, comparison)
    assert result["raw_sha256"] == hashlib.sha256(raw).hexdigest() and result["raw_record"] == pilot
    assert result["declared_timestamp_minus_low_owner_plan_seconds"] == pytest.approx(6.431191)
    assert result["strict_preregistration_verified"] is False and result["filesystem_birth_time_is_immutable_proof"] is False
    assert (tmp_path / "pilot_plan.json").read_bytes() == raw
