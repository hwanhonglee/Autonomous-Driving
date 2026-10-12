"""HH_260906 - Test one-shot personal GPU0 runner without SSH, GPU or training."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import run_map_town_rehearsal_20260927 as runner


def sha(blob):
    return hashlib.sha256(blob).hexdigest()


@pytest.fixture
def personal_tree(tmp_path, monkeypatch):
    workspace = tmp_path / "personal/hwanhong/portable_e2e"
    monkeypatch.setattr(runner, "WORKSPACE", workspace)
    monkeypatch.setattr(runner.sys, "prefix", str(workspace / "venvs/py312"))
    spec = runner.paths()
    for name in ("source", "frozen", "data", "parent", "reference_b"):
        spec[name].mkdir(parents=True)
    source_hashes = {}
    for filename in runner.SOURCE_SHA256:
        blob = ("source:" + filename).encode()
        (spec["source"] / filename).write_bytes(blob)
        source_hashes[filename] = sha(blob)
    monkeypatch.setattr(runner, "SOURCE_SHA256", source_hashes)
    blobs = {
        "TRAIN_SHA256": (spec["data"] / "train.npz", b"train"),
        "DEV_SHA256": (spec["data"] / "dev.npz", b"dev"),
        "PARENT_MANIFEST_SHA256": (spec["parent"] / "data_manifest.json", b"parent manifest"),
        "PARENT_CHECKPOINT_SHA256": (spec["parent"] / "checkpoint.pt", b"parent checkpoint"),
        "B_CHECKPOINT_SHA256": (spec["reference_b"] / "checkpoint.pt", b"B checkpoint"),
        "B_SOURCE_SHA256": (spec["reference_b"] / "source/train_map_native_ah.py", b"B trainer"),
    }
    for constant, (path, blob) in blobs.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(blob)
        monkeypatch.setattr(runner, constant, sha(blob))
    package = {"parent_manifest_sha256": runner.PARENT_MANIFEST_SHA256,
               "splits": {"train": {"count": 913, "sha256": runner.TRAIN_SHA256},
                          "dev": {"count": 118, "sha256": runner.DEV_SHA256}}}
    payload = json.dumps(package).encode()
    (spec["data"] / "manifest.json").write_bytes(payload)
    (spec["reference_b"] / "data_manifest.json").write_bytes(payload)
    monkeypatch.setattr(runner, "PACKAGE_SHA256", sha(payload))
    (spec["parent"] / "summary.json").write_text(json.dumps({"checkpoint_sha256": runner.PARENT_CHECKPOINT_SHA256}))
    (spec["parent"] / "started.json").write_text(json.dumps({"data_manifest_sha256": runner.PARENT_MANIFEST_SHA256}))
    (spec["reference_b"] / "summary.json").write_text(json.dumps({"checkpoint_sha256": runner.B_CHECKPOINT_SHA256}))
    (spec["reference_b"] / "started.json").write_text(json.dumps({"data_manifest_sha256": runner.PACKAGE_SHA256,
                                                                    "source_sha256": runner.B_SOURCE_SHA256}))
    return spec


def test_fixed_paths_hashes_and_cpu_gpu_environment(personal_tree):
    spec = personal_tree
    identity = runner.preflight(spec)
    assert identity["package_sha256"] == runner.PACKAGE_SHA256
    assert identity["b_checkpoint_sha256"] == runner.B_CHECKPOINT_SHA256
    argv = runner.command(spec)
    assert argv[1] == str(spec["source"] / "train_map_town_rehearsal_c.py")
    assert argv[argv.index("--reference-b") + 1] == str(spec["reference_b"])
    assert argv[argv.index("--output") + 1] == str(spec["output"])
    assert runner.prepare_environment(cpu_only=True)["CUDA_VISIBLE_DEVICES"] == ""
    env = runner.prepare_environment()
    assert env["CUDA_VISIBLE_DEVICES"] == runner.GPU0_UUID
    assert all(name not in env for name in ("PYTHONPATH", "PYTHONHOME", "PYTHONUSERBASE"))


@pytest.mark.parametrize("name", ["source", "package", "train", "dev", "parent", "b_source", "b_checkpoint", "output"])
def test_changed_or_used_input_refused_before_log(personal_tree, name):
    spec = personal_tree
    target = {"source": spec["source"] / "train_map_town_rehearsal_c.py",
              "package": spec["data"] / "manifest.json", "train": spec["data"] / "train.npz",
              "dev": spec["data"] / "dev.npz", "parent": spec["parent"] / "checkpoint.pt",
              "b_source": spec["reference_b"] / "source/train_map_native_ah.py",
              "b_checkpoint": spec["reference_b"] / "checkpoint.pt",
              "output": spec["output"]}[name]
    if name == "output":
        target.mkdir()
    else:
        target.write_bytes(b"changed")
    with pytest.raises(ValueError):
        runner.preflight(spec)
    assert not spec["log"].exists() and not spec["receipt"].exists()


def fake_processes(monkeypatch, cpu_code=0, train_code=0, gpu_row=None):
    calls = []

    def fake_run(argv, **kwargs):
        calls.append((list(argv), kwargs["env"]))
        if "--validate-only" in argv and cpu_code == 0:
            kwargs["stdout"].write("TOWN_REHEARSAL_CPU_PREFLIGHT_PASS\n")
        return SimpleNamespace(returncode=cpu_code if "--validate-only" in argv else train_code)

    monkeypatch.setattr(runner.subprocess, "run", fake_run)
    monkeypatch.setattr(runner.subprocess, "check_output", lambda *a, **k: gpu_row or
                        f"0, {runner.GPU0_UUID}, 0, 0\n1, GPU-other, 90000, 100\n")
    return calls


def test_cpu_then_one_training_and_terminal_receipt(personal_tree, monkeypatch):
    calls = fake_processes(monkeypatch)
    assert runner.run() == 0 and len(calls) == 2
    assert calls[0][0] == calls[1][0] + ["--validate-only"]
    assert calls[0][1]["CUDA_VISIBLE_DEVICES"] == ""
    assert calls[1][1]["CUDA_VISIBLE_DEVICES"] == runner.GPU0_UUID
    receipt = json.loads(personal_tree["receipt"].read_text())
    assert receipt["optimization_run_count"] == 1
    assert receipt["automatic_retry"] is False and receipt["model_approved"] is False
    with pytest.raises(ValueError, match="fresh output"):
        runner.preflight(personal_tree)


@pytest.mark.parametrize("case,code,row,expected_calls", [
    ("cpu", 7, None, 1),
    ("busy", 0, "0, GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5, 40, 5\n", 1),
    ("train", 9, None, 2),
])
def test_no_retry_on_cpu_gpu_or_training_failure(personal_tree, monkeypatch, case, code, row, expected_calls):
    calls = fake_processes(monkeypatch, cpu_code=code if case == "cpu" else 0,
                           train_code=code if case == "train" else 0, gpu_row=row)
    assert runner.run() != 0
    assert len(calls) == expected_calls
    receipt = json.loads(personal_tree["receipt"].read_text())
    assert receipt["automatic_retry"] is False
    assert receipt["optimization_run_count"] == (1 if case == "train" else 0)


def test_wrong_physical_index_cannot_pass_idle_gate(personal_tree, monkeypatch):
    monkeypatch.setattr(runner.subprocess, "check_output",
                        lambda *a, **k: f"1, {runner.GPU0_UUID}, 0, 0\n")
    with pytest.raises(ValueError, match="physical GPU0"):
        runner.check_gpu0_idle()
