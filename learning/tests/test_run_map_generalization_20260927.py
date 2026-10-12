"""HH_260906 - Exercise the sealed map runner without SSH, CUDA, installs or training."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import run_map_generalization_20260927 as runner


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


@pytest.fixture
def personal_tree(tmp_path, monkeypatch):
    workspace = tmp_path / "personal/hwanhong/portable_e2e"
    monkeypatch.setattr(runner, "WORKSPACE", workspace)
    monkeypatch.setattr(runner.sys, "prefix", str(workspace / "venvs/py312"))
    spec = runner.paths()
    for name in ("source", "frozen", "data", "parent"):
        spec[name].mkdir(parents=True)
    source_hashes = {}
    for filename in runner.SOURCE_SHA256:
        payload = ("fixture:" + filename).encode()
        (spec["source"] / filename).write_bytes(payload)
        source_hashes[filename] = sha(payload)
    monkeypatch.setattr(runner, "SOURCE_SHA256", source_hashes)
    train, dev, parent_data, checkpoint = b"train-fixture", b"dev-fixture", b"parent-fixture", b"checkpoint-fixture"
    for item, payload in ((spec["data"] / "train.npz", train),
                          (spec["data"] / "dev.npz", dev),
                          (spec["parent"] / "data_manifest.json", parent_data),
                          (spec["parent"] / "checkpoint.pt", checkpoint)):
        item.write_bytes(payload)
    monkeypatch.setattr(runner, "TRAIN_SHA256", sha(train))
    monkeypatch.setattr(runner, "DEV_SHA256", sha(dev))
    monkeypatch.setattr(runner, "PARENT_MANIFEST_SHA256", sha(parent_data))
    monkeypatch.setattr(runner, "PARENT_CHECKPOINT_SHA256", sha(checkpoint))
    package = {"parent_manifest_sha256": sha(parent_data),
               "splits": {"train": {"count": 913, "sha256": sha(train)},
                          "dev": {"count": 118, "sha256": sha(dev)}}}
    package_bytes = json.dumps(package).encode()
    (spec["data"] / "manifest.json").write_bytes(package_bytes)
    monkeypatch.setattr(runner, "PACKAGE_SHA256", sha(package_bytes))
    (spec["parent"] / "summary.json").write_text(json.dumps({"checkpoint_sha256": sha(checkpoint)}))
    (spec["parent"] / "started.json").write_text(json.dumps({"data_manifest_sha256": sha(parent_data)}))
    return spec


def test_fixed_personal_paths_hashes_and_command(personal_tree):
    spec = personal_tree
    identity = runner.preflight(spec)
    assert identity["package_sha256"] == runner.PACKAGE_SHA256
    assert identity["parent_checkpoint_sha256"] == runner.PARENT_CHECKPOINT_SHA256
    argv = runner.command(spec)
    assert argv[0] == runner.sys.executable
    assert argv[-1] == "--expanded-train"
    assert argv[argv.index("--data") + 1] == str(spec["data"])
    assert argv[argv.index("--parent") + 1] == str(spec["parent"])
    assert argv[argv.index("--output") + 1] == str(spec["output"])
    env = runner.prepare_environment()
    assert env["CUDA_VISIBLE_DEVICES"] == runner.GPU0_UUID
    assert env["PYTHONNOUSERSITE"] == "1"
    assert all(name not in env for name in ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP", "PYTHONUSERBASE"))


@pytest.mark.parametrize("changed", ["source", "package", "train", "dev", "parent_checkpoint", "output"])
def test_changed_bytes_or_used_output_fail_before_launch(personal_tree, changed):
    spec = personal_tree
    target = {"source": spec["source"] / "train_map_native_ah.py",
              "package": spec["data"] / "manifest.json",
              "train": spec["data"] / "train.npz", "dev": spec["data"] / "dev.npz",
              "parent_checkpoint": spec["parent"] / "checkpoint.pt",
              "output": spec["output"]}[changed]
    if changed == "output":
        target.mkdir()
    else:
        target.write_bytes(b"changed")
    with pytest.raises(ValueError):
        runner.preflight(spec)
    assert not spec["log"].exists() and not spec["receipt"].exists()


def test_symlinked_source_rejected(personal_tree):
    spec = personal_tree
    source = spec["source"] / "train_map_native_ah.py"
    source.rename(source.with_suffix(".original"))
    source.symlink_to(source.with_suffix(".original"))
    with pytest.raises(ValueError, match="regular file"):
        runner.preflight(spec)


def mock_training(monkeypatch, *, cpu_code=0, train_code=0, gpu_row=None):
    calls = []
    def fake_run(argv, **kwargs):
        calls.append((list(argv), kwargs["env"]))
        if "--validate-only" in argv and cpu_code == 0:
            kwargs["stdout"].write("NATIVE_MAPPING_CPU_PREFLIGHT_PASS\n")
        return SimpleNamespace(returncode=cpu_code if "--validate-only" in argv else train_code)
    def fake_gpu(argv, **kwargs):
        assert argv[0] == "nvidia-smi"
        return gpu_row or f"0, {runner.GPU0_UUID}, 0, 0\n1, GPU-other, 90000, 100\n"
    monkeypatch.setattr(runner.subprocess, "run", fake_run)
    monkeypatch.setattr(runner.subprocess, "check_output", fake_gpu)
    return calls


def test_cpu_validate_then_exactly_one_gpu0_training_with_receipt(personal_tree, monkeypatch):
    calls = mock_training(monkeypatch)
    assert runner.run() == 0
    assert len(calls) == 2
    assert calls[0][0] == calls[1][0] + ["--validate-only"]
    assert all(env["CUDA_VISIBLE_DEVICES"] == runner.GPU0_UUID for _, env in calls)
    result = json.loads(personal_tree["receipt"].read_text())
    assert result["phase"] == "train_once" and result["returncode"] == 0
    assert result["validation_run_count"] == result["optimization_run_count"] == 1
    assert result["automatic_retry"] is False and result["automatic_promotion"] is False
    assert result["model_approved"] is False


def test_failed_cpu_validation_records_receipt_without_gpu_training(personal_tree, monkeypatch):
    calls = mock_training(monkeypatch, cpu_code=7)
    assert runner.run() == 7
    assert len(calls) == 1
    result = json.loads(personal_tree["receipt"].read_text())
    assert result["phase"] == "cpu_validate" and result["optimization_run_count"] == 0
    assert result["returncode"] == 7 and "CPU validation failed" in result["error"]


def test_busy_gpu0_never_trains_or_uses_gpu1(personal_tree, monkeypatch):
    calls = mock_training(monkeypatch, gpu_row=f"0, {runner.GPU0_UUID}, 20, 5\n1, GPU-other, 0, 0\n")
    assert runner.run() == 1
    assert len(calls) == 1
    result = json.loads(personal_tree["receipt"].read_text())
    assert result["phase"] == "gpu0_preflight" and result["optimization_run_count"] == 0
    assert "occupied" in result["error"]


def test_failed_single_training_still_records_exit_without_retry(personal_tree, monkeypatch):
    calls = mock_training(monkeypatch, train_code=9)
    assert runner.run() == 9
    assert len(calls) == 2
    result = json.loads(personal_tree["receipt"].read_text())
    assert result["phase"] == "train_once" and result["optimization_run_count"] == 1
    assert result["returncode"] == 9 and result["automatic_retry"] is False


def test_only_approved_physical_uuid_can_pass_read_only_gpu_gate(personal_tree, monkeypatch):
    monkeypatch.setattr(runner.subprocess, "check_output",
                        lambda *a, **k: f"1, {runner.GPU0_UUID}, 0, 0\n")
    with pytest.raises(ValueError, match="physical GPU0"):
        runner.check_gpu0_idle()
