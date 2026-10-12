"""HH_260928 - Guard the stationary Town13 location contrast before CARLA ownership."""

from contextlib import contextmanager
import json
import subprocess

import numpy as np
import pytest

from scripts.e2e import capture_town13_skydome_location_aba as worker
from scripts.e2e import run_owned_town13_skydome_location_aba as owner


def test_byte_pinned_camera_pose_translation_preserves_orientation():
    import carla

    matrix, delta, hashes = worker.pin_inputs()
    a = worker.camera_transform(carla, matrix, np.zeros(3))
    b = worker.camera_transform(carla, matrix, delta)
    a_matrix = np.asarray(a.get_matrix())
    b_matrix = np.asarray(b.get_matrix())
    assert hashes["baseline_matrix"] == worker.BASELINE_MATRIX_SHA256
    assert np.array_equal(a_matrix[:3, :3], b_matrix[:3, :3])
    assert np.allclose(b_matrix[:3, 3] - a_matrix[:3, 3], delta, atol=1e-5)
    assert np.linalg.norm(delta[:2]) == pytest.approx(4829.175382752607)
    assert owner._bound_inputs()["route_b.json"] == worker.ROUTE_B


def test_changed_baseline_matrix_is_rejected(monkeypatch):
    monkeypatch.setattr(worker, "BASELINE_MATRIX_SHA256", "0" * 64)
    with pytest.raises(worker.LocationProbeError, match="camera matrix bytes changed"):
        worker.pin_inputs()


def test_warning_score_uses_fixed_original_pixel_regions():
    rgb = np.zeros((360, 640, 3), dtype=np.uint8)
    rgb[:2, :50] = [200, 170, 20]
    score = worker.pixel_quality(rgb)
    assert score == {"yellow_top32_pixels": 100, "warning_detected": True,
                     "dark_center_sky_fraction": 1.0}
    rgb[30:75, 100:500] = [90, 90, 90]
    assert worker.pixel_quality(rgb)["dark_center_sky_fraction"] == 0.0


def test_default_is_static_and_existing_output_blocks_reuse(monkeypatch, tmp_path, capsys):
    bound = owner.preflight()
    monkeypatch.setattr(owner, "preflight", lambda: bound)
    monkeypatch.setattr(owner, "OUTPUT", tmp_path / "new_capture")
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("launched editor"))
    assert owner.main([]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["status"] == "STATIC_READY_NOT_LIVE_TESTED"
    assert result["simulator_started"] is False
    assert result["phase_order"] == ["A1", "B", "A2"]
    assert not owner.OUTPUT.exists()
    owner.OUTPUT.mkdir()
    assert owner.main([]) == 2


@contextmanager
def _owned_lock():
    yield


@pytest.mark.parametrize("blocker", ["competing_owner", "occupied_port"])
def test_competing_owner_or_port_aborts_before_output(monkeypatch, tmp_path, blocker):
    bound = owner.preflight()
    monkeypatch.setattr(owner, "preflight", lambda: bound)
    monkeypatch.setattr(owner, "OUTPUT", tmp_path / "never_created")
    monkeypatch.setattr(owner.catalog_owner, "_workspace_lock", _owned_lock)
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("launched editor"))
    if blocker == "competing_owner":
        monkeypatch.setattr(owner.catalog_owner, "_assert_no_other_carla",
                            lambda: (_ for _ in ()).throw(owner.catalog_owner.CatalogOwnerError("foreign editor")))
        monkeypatch.setattr(owner.catalog_owner, "_assert_ports_free", lambda: None)
    else:
        monkeypatch.setattr(owner.catalog_owner, "_assert_no_other_carla", lambda: None)
        monkeypatch.setattr(owner.catalog_owner, "_assert_ports_free",
                            lambda: (_ for _ in ()).throw(owner.catalog_owner.CatalogOwnerError("port occupied")))
    with pytest.raises(owner.catalog_owner.CatalogOwnerError):
        owner.execute(*bound)
    assert not owner.OUTPUT.exists()
