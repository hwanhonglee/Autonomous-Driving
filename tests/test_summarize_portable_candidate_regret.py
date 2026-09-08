"""HH_260906 - Exercise result-only candidate-regret verification with synthetic reports and no training."""

from copy import deepcopy
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest

from portable_e2e.contract import ContractError
from scripts.e2e import summarize_portable_candidate_regret as module
import test_summarize_portable_no_accel_development as old
from test_summarize_portable_no_accel_development import no_accel, expansion, campaign  # noqa: F401


@pytest.fixture
def regret(no_accel, tmp_path, monkeypatch):
    # HH_260906 - Adapt only fixture-owned reports; every six-arm artifact remains a newly materialized test object.
    root = tmp_path / "regret"
    previous = json.loads((no_accel / "status.json").read_text())
    plan = deepcopy(previous["plan"])
    plan.pop("model_configs")
    plan.update(schema=module.PLAN_SCHEMA, campaign_id=module.CAMPAIGN_ID, source_commit="b" * 40,
        arms=module.ARMS, model_config=module.MODEL, model_config_sha256=module.MODEL_SHA256,
        candidate_score_weight=.1, candidate_regret_weights=module.WEIGHTS,
        finish_before_utc=module.DEADLINE, finish_reserve_seconds=300, stage_timeout_seconds=module.STAGE_TIMEOUTS)
    original_source = module.base._git_bytes
    sources = {name: original_source(previous["plan"]["source_commit"], name) for name in module.CORE_FILES}
    monkeypatch.setattr(module, "git_bytes", lambda commit, path: sources[path])
    config = json.loads(sources[module.MODEL])
    config_sha = module.base._sha(module.canonical(config).encode())
    old.first._write(root / "plan.json", plan)
    state = deepcopy(previous)
    state.pop("model_configs")
    state.update(plan=plan, plan_sha256=module.sha_file(root / "plan.json"), source_commit=plan["source_commit"],
        model_config=config, model_config_sha256=module.MODEL_SHA256, model_config_canonical_sha256=config_sha,
        created_at_utc="2026-09-08T18:00:00+00:00", stages=[])
    mapping = dict(zip(old.module.ARMS, module.ARMS))
    for seed in module.base.SEEDS:
        for old_arm, arm in mapping.items():
            relative = f"seed_{seed}/{arm}"
            shutil.copytree(no_accel / f"seed_{seed}/{old_arm}", root / relative)
            for stage, filename in module.expansion.STAGE_FILES.items():
                path = root / relative / filename
                value = json.loads(path.read_text())
                value["model_parameter_count"] = 954590
                if stage == "train":
                    value["model_config"] = config
                    value["loss_config"] = dict(module.LOSS)
                    value["last_metrics"] = {"loss": 2.5}
                    if module.WEIGHTS[arm]:
                        value["loss_config"]["candidate_regret_weight"] = .1
                        value["last_metrics"]["candidate_regret_loss"] = .8
                else:
                    value["model_config_sha256"] = config_sha
                    if stage == "evaluate" and module.WEIGHTS[arm]:
                        value.update(loss_config=dict(module.LOSS, candidate_regret_weight=.1),
                            auxiliary_loss_metrics={"candidate_regret_loss": .75},
                            auxiliary_loss_metric_counts={"candidate_regret_loss": 337})
                old.first._write(path, value)
                record = deepcopy(next(r for r in previous["stages"] if r["run"] == f"seed_{seed}/{old_arm}" and r["stage"] == stage))
                command = [v.replace(old.module.CAMPAIGN_ID, module.CAMPAIGN_ID).replace(old_arm, arm)
                    .replace(old.module.MODELS[old_arm], module.MODEL) for v in record["command"]]
                if stage == "train":
                    command += ["--candidate-score-weight", "0.1", "--candidate-regret-weight", str(module.WEIGHTS[arm])]
                when = datetime(2026, 9, 8, 18, tzinfo=timezone.utc) + timedelta(seconds=2 * len(state["stages"]))
                record.update(run=relative, command=command, started_at_utc=when.isoformat(),
                    finished_at_utc=(when + timedelta(seconds=1)).isoformat())
                record["report"]["sha256"] = module.sha_file(path)
                state["stages"].append(record)
    archive = root / module.ARCHIVE
    archive.parent.mkdir(parents=True)
    archive.write_bytes(sources[module.WORKER])
    old.first._write(root / "status.json", state)
    return root


def seal(root, stage, change, arm="B_cost_aware_selector", seed=20260903):
    relative = f"seed_{seed}/{arm}"
    filename = module.expansion.STAGE_FILES[stage]
    old.first._mutate(root, relative + "/" + filename, change)
    state = json.loads((root / "status.json").read_text())
    next(r for r in state["stages"] if r["run"] == relative and r["stage"] == stage)["report"]["sha256"] = module.sha_file(root / relative / filename)
    old.first._write(root / "status.json", state)


def reseal_plan(root, change):
    plan = json.loads((root / "plan.json").read_text())
    change(plan)
    old.first._write(root / "plan.json", plan)
    old.first._mutate(root, "status.json", lambda s: s.update(plan=plan, plan_sha256=module.sha_file(root / "plan.json")))


def test_complete_six_fresh_runs_auxiliary_separate_and_never_promoted(regret):
    result = module.summarize_campaign(regret)
    assert result["status"] == "COMPLETE_NOT_PROMOTED"
    assert result["candidate_screen"] == result["absolute_quality"] == "PASS"
    assert len(result["pairs"]) == 3 and result["completed_stage_count"] == 18
    assert result["budget"]["samples_seen_per_run"] == 6155
    assert len(result["input_manifest"]) == 21
    assert result["automatic_promotion"] is result["vehicle_control_approved"] is False
    assert result["test_evaluated"] is result["test_used_for_training_or_selection"] is False
    for pair in result["pairs"]:
        assert pair["baseline"]["auxiliary_diagnostics"] == {"enabled": False, "weight": 0.0}
        assert pair["candidate"]["auxiliary_diagnostics"]["evaluation_sample_count"] == 337
        assert "candidate_regret_loss" not in pair["candidate"]["metrics"]
    assert "/synthetic/" not in json.dumps(result) and str(regret) not in json.dumps(result)
    assert "같은 목적함수" in module.render_markdown(result)


@pytest.mark.parametrize("stage,change", [
    ("train", lambda x: x["loss_config"].pop("candidate_regret_weight")),
    ("train", lambda x: x["loss_config"].update(candidate_regret_weight=.2)),
    ("train", lambda x: x["last_metrics"].pop("candidate_regret_loss")),
    ("train", lambda x: x.update(dataset_size=613)),
    ("train", lambda x: x["train_config"].update(seed=20260904)),
    ("train", lambda x: x["state"].update(domain_samples_seen={"carla": 6154})),
    ("evaluate", lambda x: x["loss_config"].update(candidate_regret_weight=0.0)),
    ("evaluate", lambda x: x.update(auxiliary_loss_metric_counts={"candidate_regret_loss": 336})),
    ("evaluate", lambda x: x["auxiliary_loss_metrics"].update(candidate_regret_loss=-.1)),
    ("evaluate", lambda x: x["auxiliary_loss_metrics"].update(candidate_regret_loss=True)),
    ("evaluate", lambda x: x.update(evaluation_split="test")),
    ("evaluate", lambda x: x["hardware"].update(device_uuid="other-device")),
    ("audit", lambda x: x["gate"]["thresholds"].update(maximum_deceleration_mps2=7)),
    ("audit", lambda x: x["implementation"].update(runtime_contract_sha256="f" * 64)),
    ("audit", lambda x: x.update(checkpoint_sha256="f" * 64)),
])
def test_resealed_semantic_tampering_fails(regret, stage, change):
    seal(regret, stage, change)
    with pytest.raises(ContractError):
        module.summarize_campaign(regret)


@pytest.mark.parametrize("stage,change", [
    ("train", lambda x: x["loss_config"].update(candidate_regret_weight=0.0)),
    ("train", lambda x: x["last_metrics"].update(candidate_regret_loss=0.0)),
    ("evaluate", lambda x: x.update(loss_config=module.LOSS)),
])
def test_zero_arm_old_six_key_contract_is_exact(regret, stage, change):
    seal(regret, stage, change, arm="A_original_loss")
    with pytest.raises(ContractError):
        module.summarize_campaign(regret)


@pytest.mark.parametrize("change", [
    lambda p: p.update(finish_before_utc="2026-09-10T01:00:00Z"),
    lambda p: p.update(candidate_regret_weights={"A_original_loss": .1, "B_cost_aware_selector": 0.0}),
    lambda p: p.update(model_config_sha256="f" * 64),
    lambda p: p.update(resume="old-baseline"),
    lambda p: p.update(seeds=[20260903]),
    lambda p: p.update(source_commit="bad"),
])
def test_fixed_plan_scope_rejected(regret, change):
    reseal_plan(regret, change)
    with pytest.raises(ContractError):
        module.summarize_campaign(regret)


@pytest.mark.parametrize("change", [
    lambda s: s.update(runner_sha256="f" * 64),
    lambda s: s["stages"].reverse(),
    lambda s: s["stages"][0]["command"].extend(["--resume", "old.pt"]),
    lambda s: s["stages"][0]["command"].__setitem__(-1, "0.1"),
    lambda s: s["stages"][0].update(started_at_utc="2026-09-09T00:59:59+00:00"),
    lambda s: s["stages"][0].update(status="FAILED"),
])
def test_source_order_command_and_deadline_fail_closed(regret, change):
    old.first._mutate(regret, "status.json", change)
    with pytest.raises(ContractError):
        module.summarize_campaign(regret)


def test_incomplete_has_all_missing_stage_denominators(regret):
    old.first._mutate(regret, "status.json", lambda s: s.update(stages=s["stages"][:3], status="RUNNING"))
    result = module.summarize_campaign(regret)
    assert result["status"] == "INCOMPLETE" and result["candidate_screen"] == "NOT_EVALUATED"
    assert result["completed_stage_count"] == 3 and len(result["not_started_stages"]) == 15 and not result["pairs"]


def test_missing_archive_incomplete_not_promoted(regret):
    (regret / module.ARCHIVE).rename(regret / "private-frozen-runner-backup")
    result = module.summarize_campaign(regret)
    assert result["status"] == "INCOMPLETE" and module.ARCHIVE in result["missing_artifacts"]


def test_changed_bytes_rejected_even_if_report_semantics_unchanged(regret):
    path = regret / "seed_20260903/A_original_loss/evaluation/metrics.json"
    path.write_bytes(path.read_bytes() + b"\n")
    with pytest.raises(ContractError, match="SHA-256"):
        module.summarize_campaign(regret)


def test_optional_checkpoint_bytes_hashed_not_loaded(regret):
    checkpoint = regret / "seed_20260903/B_cost_aware_selector/training/checkpoints/latest.pt"
    checkpoint.parent.mkdir()
    checkpoint.write_bytes(b"not-a-model; hash verification only")
    sha = module.sha_file(checkpoint)
    seal(regret, "evaluate", lambda x: x.update(checkpoint_sha256=sha))
    seal(regret, "audit", lambda x: x.update(checkpoint_sha256=sha))
    def command(state):
        next(r for r in state["stages"] if r["run"] == "seed_20260903/B_cost_aware_selector" and r["stage"] == "audit")["command"][-1] = sha
    old.first._mutate(regret, "status.json", command)
    assert module.summarize_campaign(regret)["pairs"][0]["candidate"]["checkpoint_bytes_locally_verified"]


def test_cli_fresh_output_and_private_path_free(regret, tmp_path):
    output = tmp_path / "summary"
    assert module.main([str(regret), "--output-dir", str(output)]) == 0
    assert module.main([str(regret), "--output-dir", str(output)]) == 2
    assert module.main([str(regret), "--output-dir", str(regret / "summary")]) == 2
    assert module.main([str(regret), "--output-dir", str(module.base.REPO / "datasets/fake-output")]) == 2
    for line in (output / "SHA256SUMS").read_text().splitlines():
        sha, name = line.split("  ")
        assert module.sha_file(output / name) == sha


def test_git_verification_no_network_and_bounded(monkeypatch):
    def run(command, **kwargs):
        assert command[:4] == ["git", "-c", "protocol.allow=never", "show"]
        assert kwargs["timeout"] == 15 and kwargs["env"]["GIT_NO_LAZY_FETCH"] == "1"
        assert kwargs["env"]["GIT_ALLOW_PROTOCOL"] == "" and kwargs["env"]["GIT_TERMINAL_PROMPT"] == "0"
        return SimpleNamespace(stdout=b"pinned bytes")
    monkeypatch.setattr(module.subprocess, "run", run)
    assert module.git_bytes("a" * 40, "portable_e2e/model.py") == b"pinned bytes"


def test_exact_commands_match_production_runner_without_execution(regret, monkeypatch):
    # HH_260906 - Construct argv only; no process, dataset loader, checkpoint or CUDA operation is invoked.
    from scripts.e2e import run_portable_training_campaign as runner
    workspace = Path("/synthetic/personal/hwanhong/portable_e2e")
    monkeypatch.setattr(runner, "WORKSPACE", workspace)
    plan = json.loads((regret / "plan.json").read_text())
    state = json.loads((regret / "status.json").read_text())
    generated = list(runner.commands(plan, workspace / "runs/campaigns" / module.CAMPAIGN_ID, workspace / "autoware_e2e"))
    assert len(generated) == 18
    for (_, _, command, stage), record in zip(generated, state["stages"]):
        if stage == "audit":
            command += ["--checkpoint-sha256", record["command"][-1]]
        assert command == record["command"]


def test_missing_final_report_is_incomplete_and_keeps_denominator(regret):
    target = regret / "seed_20260905/B_cost_aware_selector/gate_v8.json"
    target.rename(target.with_suffix(".retained"))
    result = module.summarize_campaign(regret)
    assert result["status"] == "INCOMPLETE" and result["expected_stage_count"] == 18
    assert result["missing_artifacts"] == ["seed_20260905/B_cost_aware_selector/gate_v8.json"]
    assert not result["pairs"]


def test_symlink_artifact_fails_even_to_identical_bytes(regret):
    target = regret / module.ARCHIVE
    backup = target.with_suffix(".retained")
    target.rename(backup)
    target.symlink_to(backup)
    with pytest.raises(ContractError, match="symlink"):
        module.summarize_campaign(regret)


def test_input_postcheck_rejects_concurrent_change(regret, monkeypatch):
    original = module.base.compare_reports
    def changed(*args):
        value = original(*args)
        path = regret / module.ARCHIVE
        path.write_bytes(path.read_bytes() + b"changed after precheck")
        return value
    monkeypatch.setattr(module.base, "compare_reports", changed)
    with pytest.raises(ContractError, match="changed during"):
        module.summarize_campaign(regret)
