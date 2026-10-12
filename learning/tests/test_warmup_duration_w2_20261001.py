"""HH_261001 - Verify exact W1 update-body reuse, four-block accounting and the mandatory reproduction barrier on CPU."""

import ast
import copy
import inspect
import json
from pathlib import Path
import textwrap
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import run_ah_warmup_duration_w2_20261001 as runner


def schedule():
    batches = runner.supervision.phase.schedule(2787)
    mask = torch.zeros(2787, dtype=torch.bool)
    mask[:35] = True
    mask[309:344] = True
    mask[613:648] = True
    proof = dict(exposure_order_sha256=runner.supervision.digest_json(batches),
                 warmup_mask_sha256=runner.supervision.digest_json(mask.tolist()))
    return mask, batches, proof


def test_exact_original_w1_update_body_ast_matches():
    original = inspect.getsource(runner.w1.fit)
    start = original.index("            batch = base._move_batch(raw, device)")
    end = original.index("            updates +=", start)
    expected = textwrap.dedent(original[start:end]).replace(
        " if arm == ARMS[1] else [1.]*4", "")
    actual = ast.parse(inspect.getsource(runner.optimize_batch)).body[0].body[1:-1]
    assert ast.dump(ast.Module(body=actual, type_ignores=[]), include_attributes=False) == ast.dump(
        ast.parse(expected), include_attributes=False)


def test_schedule_mask_and_four_block_totals_are_exact():
    mask, batches, proof = schedule()
    runner.validate_schedule(mask, batches, proof)
    repeated = batches * 4
    skip = [i for i, ids in enumerate(repeated, 1) if all(bool(mask[j]) for j in ids)]
    assert skip == [1693, 4693, 7693, 10693]
    forwarded = [j for i, ids in enumerate(repeated, 1) if i not in skip for j in ids]
    assert len(repeated) == 12000 and len(forwarded) == 47984
    assert sum(not bool(mask[j]) for j in forwarded) == 46196
    assert (len(repeated) - len(skip)) == 11996
    assert runner.supervision.ACTIVE_WEIGHT == 11996 / 11549


def test_schedule_or_mask_mutation_is_rejected():
    mask, batches, proof = schedule()
    altered = list(batches)
    altered[0] = tuple(reversed(altered[0]))
    with pytest.raises(ValueError, match="schedule differs"):
        runner.validate_schedule(mask, altered, proof)
    mask[0] = False
    with pytest.raises(ValueError, match="mask differs"):
        runner.validate_schedule(mask, batches, proof)


def test_one_optimizer_one_seed_and_skip_branch_no_forward_or_step():
    tree = ast.parse(inspect.getsource(runner.fit))
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    names = [ast.unparse(node.func) for node in calls]
    assert names.count("torch.optim.AdamW") == 1
    assert names.count("base._seed_everything") == 1
    assert names.count("model.load_state_dict") == 1
    assert names.count("first_block_barrier") == 1
    skip = next(node for node in ast.walk(tree) if isinstance(node, ast.If)
                and ast.unparse(node.test) == "step in supervision.SKIP_ONE_BASED")
    skipped_calls = [ast.unparse(node.func) for node in ast.walk(skip) if isinstance(node, ast.Call)]
    assert "optimize_batch" not in skipped_calls and "optimizer.step" not in skipped_calls
    assert isinstance(skip.body[-1], ast.Continue)
    assert "DataLoader(dataset, batch_sampler=batches, num_workers=0)" in inspect.getsource(runner.fit)


def identity_fixture(tmp_path):
    model = torch.nn.Linear(2, 1)
    optimizer = torch.optim.AdamW(model.parameters(), lr=3e-5, weight_decay=1e-4)
    model(torch.tensor([[1., 2.]])).square().sum().backward()
    optimizer.step()
    folder = tmp_path / "arm/training"
    folder.mkdir(parents=True)
    cfg = SimpleNamespace(to_dict=lambda: {"toy_test_only": True})
    counts = dict(scheduled_steps=3000, global_step=2999, samples_seen=11996, active_supervision_exposures=11549)
    return model, cfg, optimizer, counts, folder


def test_first_block_mismatch_preserves_checkpoint_and_receipt_then_raises(tmp_path):
    model, cfg, optimizer, counts, folder = identity_fixture(tmp_path)
    with pytest.raises(ValueError, match="first-block state differs"):
        runner.first_block_barrier(model, cfg, optimizer, id(optimizer), counts, folder, "initial", {})
    receipt = json.loads((folder.parent / "first_block_identity.json").read_text())
    assert receipt["status"] == "FAIL_FIRST_BLOCK_IDENTITY"
    assert receipt["expected_model_state_sha256"] == runner.EXPECTED_W1_STATE
    assert receipt["actual_model_state_sha256"] == runner.base.tensor_digest(model.state_dict())
    assert runner.base.digest(folder / receipt["checkpoint"]) == receipt["checkpoint_sha256"]
    assert not receipt["checkpoint_used_for_model_selection"]
    assert not (folder / "final_checkpoint.pt").exists()


def test_successful_identity_barrier_preserves_optimizer_model_and_rng(tmp_path, monkeypatch):
    model, cfg, optimizer, counts, folder = identity_fixture(tmp_path)
    expected = runner.base.tensor_digest(model.state_dict())
    monkeypatch.setattr(runner, "EXPECTED_W1_STATE", expected)
    before = copy.deepcopy(optimizer.state_dict())
    rng = torch.random.get_rng_state().clone()
    receipt = runner.first_block_barrier(model, cfg, optimizer, id(optimizer), counts, folder, "initial", {})
    assert receipt["status"] == "PASS_EXACT_W1_STATE" and receipt["optimizer_same_object"]
    assert runner.base.tensor_digest(model.state_dict()) == expected
    assert torch.equal(torch.random.get_rng_state(), rng)
    after = optimizer.state_dict()
    assert before["param_groups"] == after["param_groups"]
    for key, state in before["state"].items():
        for field, value in state.items():
            assert torch.equal(value, after["state"][key][field]) if torch.is_tensor(value) else value == after["state"][key][field]


def test_exact_budget_boundary_has_no_automatic_extension(monkeypatch):
    monkeypatch.setattr(runner.time, "monotonic", lambda: 3700.)
    runner.check_fit_budget(100.)
    with pytest.raises(ValueError, match="3600-second"):
        runner.check_fit_budget(99.999)


def test_only_fixed_final_can_reach_dev_and_scope_restores():
    source = inspect.getsource(runner.main)
    assert source.index("receipt = fit(") < source.index('"fixed_final_checkpoint_complete.json"')
    assert source.index('"fixed_final_checkpoint_complete.json"') < source.index('dev_view = plan["views"]["Town03_DEV"]')
    assert source.count("previous.evaluate_final(") == 1
    assert source.count("w1.probe(") == 1
    before = runner.previous.SCHEMA, runner.previous.STEPS
    with pytest.raises(RuntimeError):
        with runner.final_evaluation_scope():
            assert runner.previous.SCHEMA == runner.SCHEMA and runner.previous.STEPS == 11996
            raise RuntimeError("restore")
    assert (runner.previous.SCHEMA, runner.previous.STEPS) == before


def test_plan_and_original_w1_sources_are_unchanged():
    root = Path(__file__).resolve().parents[2]
    assert runner.base.digest(root / runner.PLAN_RELATIVE) == runner.PLAN_SHA256
    reference = root / "storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1"
    if not reference.is_dir():
        reference = Path.home() / "personal/hwanhong/portable_e2e/runs/diagnostics/ah_warmup_supervision_ab_20261001_v1"
    declaration, receipt, files = runner.validate_reference(root, reference)
    assert len(declaration["source_sha256"]) == 24
    assert receipt["final_model_state_sha256"] == runner.EXPECTED_W1_STATE
    assert set(files) == {"W1_declaration.json", "W1_training_receipt.json"}
    assert all((root / name).is_file() for name in runner.NEW_SOURCES)


def test_static_contract_does_not_touch_environment_data_or_gpu(monkeypatch, capsys):
    monkeypatch.setattr(runner.base, "check_run_environment", lambda *args: pytest.fail("environment/data path forbidden"))
    monkeypatch.setattr(runner.base, "preflight_gpu", lambda: pytest.fail("GPU forbidden"))
    assert runner.main(["--contract-only"]) == 0
    contract = json.loads(capsys.readouterr().out)
    assert contract["status"] == "STATIC_CONTRACT_NO_DATA_OR_GPU"
    assert contract["optimizer_updates"] == 11996 and contract["forward_samples"] == 47984
    assert contract["optimizer_initializations"] == contract["final_dev_passes"] == 1
    assert not contract["optimizer_reset_between_blocks"] and not contract["rng_reset_between_blocks"]
    assert not contract["loss_changed"] and not contract["architecture_changed"]
    assert not contract["training_data_changed"] and not contract["model_promoted"]
