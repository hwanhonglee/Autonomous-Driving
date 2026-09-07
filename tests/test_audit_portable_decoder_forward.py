"""HH_260906 - Exercise forward-only numerical checks with tiny synthetic inputs and no optimizer, GPU or data admission."""

import copy

import pytest
import torch

from scripts.e2e import audit_portable_decoder_forward as forward


def fixture():
    item = {"valid_mask": [True] * 64, "route_xy": [[0., 0.], [100., 0.]], "current_vx_mps": 1.}
    row = {"candidates": [{"initialization_index": k, "numeric_status": "FINITE", "optimized_raw_latents": [[0., 0.] for _ in range(64)]}
        for k in range(6)]}
    config = forward.base.model.ModelConfig(model_id=forward.base.model.PHYSICAL_MODEL_ID, maximum_step_m=1.)
    return item, row, config


def test_exact_decoder_cpu_forward_only_preserves_six_candidates_and_inputs(monkeypatch):
    item, row, config = fixture(); before = copy.deepcopy((item, row))
    def forbidden(*args, **kwargs): raise AssertionError("no model parameters or optimizer allowed")
    monkeypatch.setattr(torch.optim, "Adam", forbidden)
    monkeypatch.setattr(torch.nn.Linear, "__init__", forbidden)
    xy, speed, pins = forward.replay_batch([item], [row], config)
    assert len(xy) == len(speed) == 1 and len(xy[0]) == 6
    assert xy[0][0][-1] == pytest.approx([6.4, 0.], abs=1e-5)
    assert speed[0][0] == [1.] * 64 and (item, row) == before
    assert len(pins["optimized_latent_float32_sha256"]) == 64


@pytest.mark.parametrize("mode", ["latent_short", "latent_width", "latent_nonfinite", "route_nonfinite", "vx_bool", "mask_short", "candidate_omit", "candidate_reorder", "float32_overflow"])
def test_bad_abi_and_nonfinite_never_forward(mode):
    item, row, config = fixture()
    if mode == "latent_short": row["candidates"][0]["optimized_raw_latents"].pop()
    elif mode == "latent_width": row["candidates"][0]["optimized_raw_latents"][0].append(0.)
    elif mode == "latent_nonfinite": row["candidates"][0]["optimized_raw_latents"][0][0] = float("nan")
    elif mode == "route_nonfinite": item["route_xy"][0][0] = float("inf")
    elif mode == "vx_bool": item["current_vx_mps"] = True
    elif mode == "mask_short": item["valid_mask"].pop()
    elif mode == "candidate_omit": row["candidates"].pop()
    elif mode == "candidate_reorder": row["candidates"][0]["initialization_index"] = 1
    else: row["candidates"][0]["optimized_raw_latents"][0][0] = 1e300
    with pytest.raises((ValueError, RuntimeError, OverflowError)): forward.replay_batch([item], [row], config)


def test_every_coordinate_and_speed_uses_unchanged_tolerance_with_retained_mismatches():
    xy, speed = [[0., 0.] for _ in range(64)], [1.] * 64
    actual_xy, actual_speed = copy.deepcopy(xy), list(speed)
    actual_xy[2][0] = .00002; actual_xy[3][1] = .00003; actual_speed[5] += .00004
    result = forward.compare_values(xy, speed, actual_xy, actual_speed)
    assert result["status"] == "UNVERIFIED" and result["compared_scalar_count"] == 192
    assert result["mismatch_scalar_count"] == 3
    assert [(x["point_index"], x["component"]) for x in result["mismatches"]] == [(2,"x_m"),(3,"y_m"),(5,"speed_mps")]
    assert forward.TOLERANCE == {"absolute": 1e-5, "relative": 1e-5}
    assert forward.compare_values(xy, speed, xy, speed)["status"] == "MATCH_WITH_PREDECLARED_TOLERANCE"


def test_nonfinite_output_comparison_cannot_claim_unverified_finite_evidence():
    xy, speed = [[0., 0.] for _ in range(64)], [1.] * 64
    speed[-1] = float("nan")
    with pytest.raises(ValueError): forward.compare_values(xy, speed, xy, speed)


def test_decoder_latent_mutation_is_detected(monkeypatch):
    item, row, config = fixture()
    original = forward.base.model.PerspectiveTrajectoryModel._decode_physical_v1
    def mutation(config_shell, latent, history, route, mask):
        assert not torch.is_grad_enabled()
        latent[0, 0, 0, 0] = .1
        return original(config_shell, latent, history, route, mask)
    monkeypatch.setattr(forward.base.model.PerspectiveTrajectoryModel, "_decode_physical_v1", mutation)
    with pytest.raises(ValueError, match="latents mutated"): forward.replay_batch([item], [row], config)


@pytest.mark.parametrize("visible,initialized", [(None,False),("0",False),("",True)])
def test_cpu_scope_is_fail_closed(monkeypatch, visible, initialized):
    if visible is None: monkeypatch.delenv("CUDA_VISIBLE_DEVICES", raising=False)
    else: monkeypatch.setenv("CUDA_VISIBLE_DEVICES", visible)
    monkeypatch.setattr(torch.cuda, "is_initialized", lambda: initialized)
    with pytest.raises(ValueError, match="hidden CUDA"): forward.cpu_guard()


def test_changed_released_source_is_rejected(monkeypatch):
    monkeypatch.setattr(forward.base, "sha", lambda path: "0" * 64)
    with pytest.raises(ValueError, match="source changed"): forward.source_pins()


def test_unverified_result_is_preserved_in_fresh_output(tmp_path):
    result = tmp_path / "new"
    forward.write_result(result, {"status": "UNVERIFIED_NOT_ADMITTED", "training_data_approved": False}, [{"mismatch_scalar_count": 1}])
    assert "UNVERIFIED_NOT_ADMITTED" in (result / "summary.json").read_text()
    assert len((result / "SHA256SUMS").read_text().splitlines()) == 2
    with pytest.raises(ValueError): forward.write_result(result, {}, [])


def test_cli_rejects_outputs_inside_originals_before_any_forward(tmp_path, monkeypatch):
    monkeypatch.setattr(forward, "audit", lambda *args: pytest.fail("must reject before reading"))
    with pytest.raises(ValueError):
        forward.main(["--owned-root", str(tmp_path), "--input-root", str(tmp_path / "inputs"),
            "--output-dir", str(tmp_path / "overwritten")])
