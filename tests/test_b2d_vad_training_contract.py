import json
from pathlib import Path
import subprocess

import pytest

from scripts.e2e.b2d_vad_training_contract import ContractError
from scripts.e2e.b2d_vad_training_contract import DEPTH_CONTRACTS
from scripts.e2e.b2d_vad_training_contract import LICENSE_CONTRACT
from scripts.e2e.b2d_vad_training_contract import PRIVATE_TINY_CAMERA_ORDER
from scripts.e2e.b2d_vad_training_contract import PUBLIC_B2D_CAMERA_ORDER
from scripts.e2e.b2d_vad_training_contract import PUBLIC_FROM_PRIVATE_INDICES
from scripts.e2e.b2d_vad_training_contract import STORAGE_TARGETS_GIB
from scripts.e2e.b2d_vad_training_contract import _write_manifest
from scripts.e2e.b2d_vad_training_contract import detect_depth_layout
from scripts.e2e.b2d_vad_training_contract import is_official_remote
from scripts.e2e.b2d_vad_training_contract import load_manifest
from scripts.e2e.b2d_vad_training_contract import main
from scripts.e2e.b2d_vad_training_contract import storage_report
from scripts.e2e.b2d_vad_training_contract import training_recommendation
from scripts.e2e.b2d_vad_training_contract import validate_camera_adapter
from scripts.e2e.b2d_vad_training_contract import validate_depth_files


ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / "scripts/e2e/setup_b2d_vad_training.sh"


def test_public_camera_adapter_is_an_explicit_bijection() -> None:
    report = validate_camera_adapter()

    assert tuple(report["source_order"]) == PRIVATE_TINY_CAMERA_ORDER
    assert tuple(report["target_order"]) == PUBLIC_B2D_CAMERA_ORDER
    assert tuple(report["target_from_source_indices"]) == PUBLIC_FROM_PRIVATE_INDICES
    assert PUBLIC_FROM_PRIVATE_INDICES == (0, 2, 4, 1, 3, 5)
    assert tuple(
        PRIVATE_TINY_CAMERA_ORDER[index] for index in PUBLIC_FROM_PRIVATE_INDICES
    ) == PUBLIC_B2D_CAMERA_ORDER


def test_camera_adapter_rejects_a_wrong_or_non_bijective_mapping() -> None:
    with pytest.raises(ContractError, match="does not transform"):
        validate_camera_adapter(indices=(0, 1, 2, 3, 4, 5))
    with pytest.raises(ContractError, match="permutation"):
        validate_camera_adapter(indices=(0, 2, 4, 1, 3, 3))


def test_depth_contract_separates_legacy_png_and_v004_npz(tmp_path: Path) -> None:
    legacy = validate_depth_files("legacy", [tmp_path / "00000.png"])
    latest = validate_depth_files("v0.0.4", [tmp_path / "00000.npz"])

    assert legacy["legacy_prepare_b2d_compatible"] is True
    assert latest["legacy_prepare_b2d_compatible"] is False
    assert latest["requires_versioned_adapter"] is True
    assert DEPTH_CONTRACTS["v0.0.4"]["reader"] == "numpy.load"
    with pytest.raises(ContractError, match="must use .npz"):
        validate_depth_files("v0.0.4", [tmp_path / "00000.png"])
    with pytest.raises(ContractError, match="at least one"):
        validate_depth_files("v0.0.4", [])


def test_depth_layout_detector_rejects_unversioned_mixing(tmp_path: Path) -> None:
    depth = tmp_path / "route/camera/depth_front"
    depth.mkdir(parents=True)
    (depth / "00000.png").write_bytes(b"png")
    assert detect_depth_layout(tmp_path)["layout"] == "legacy"
    (depth / "00001.npz").write_bytes(b"npz")
    assert detect_depth_layout(tmp_path)["layout"] == "mixed_invalid"


def test_twelve_gib_gpu_is_planning_head_only() -> None:
    report = training_recommendation(12.0)

    assert report["profile"] == "planning-head-only"
    assert report["full_e2e_supported"] is False


def test_current_storage_scope_is_smoke_only_and_bulk_profiles_are_deferred(
    tmp_path: Path,
) -> None:
    report = storage_report(tmp_path)

    assert STORAGE_TARGETS_GIB == {"smoke": 20}
    assert report["active_scope"]["profile"] == "smoke"
    assert report["active_scope"]["minimum_free_gib"] == 20
    assert "base_ready" not in report
    assert "full_and_custom_ready" not in report
    planning = report["capacity_planning"]
    assert planning["external_storage_root"] is None
    assert planning["profiles"]["base_with_working_copy"]["status"] == (
        "deferred_external_storage"
    )
    full = planning["profiles"]["full_and_custom"]
    assert full["status"] == "excluded_current_scope"
    assert full["published_full_download_tb"] == 4
    assert full["active_capacity_check"] is False
    assert full["automatic_download"] is False


def test_license_contract_is_conservative_about_conflicting_metadata() -> None:
    assert LICENSE_CONTRACT["conflict"] is True
    assert LICENSE_CONTRACT["github_source"] == "CC-BY-NC-ND-4.0"
    assert LICENSE_CONTRACT["commercial_use"].startswith("blocked")


def test_official_remote_check_rejects_suffix_spoofing() -> None:
    assert is_official_remote(
        "https://github.com/Thinklab-SJTU/Bench2DriveZoo.git"
    )
    assert is_official_remote("git@github.com:Thinklab-SJTU/Bench2DriveZoo.git")
    assert not is_official_remote(
        "https://evilgithub.com/Thinklab-SJTU/Bench2DriveZoo.git"
    )


def test_manifest_records_exact_official_commit_and_never_claims_large_downloads(
    tmp_path: Path,
) -> None:
    prefix = tmp_path / "env"
    prefix.mkdir()
    commit = "a" * 40
    path = _write_manifest(
        prefix,
        {
            "official_origin": True,
            "commit": commit,
        },
        "uniad/vad",
        "2.1.2",
    )
    manifest = json.loads(path.read_text(encoding="utf-8"))

    assert manifest["repository"]["commit"] == commit
    assert manifest["repository"]["ref"] == "uniad/vad"
    assert manifest["large_artifacts"] == {
        "dataset_downloaded": False,
        "checkpoint_downloaded": False,
    }
    assert load_manifest(prefix)["valid"] is True


def test_manifest_rejects_an_unpinned_or_wrong_prefix(tmp_path: Path) -> None:
    prefix = tmp_path / "env"
    prefix.mkdir()
    path = prefix / "b2d_vad_training_manifest.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "environment_prefix": str(tmp_path / "another-env"),
                "python_requirement": "3.8",
                "cuda_requirement": "11.8",
                "repository": {
                    "url": "https://github.com/Thinklab-SJTU/Bench2DriveZoo.git",
                    "ref": "uniad/vad",
                    "commit": "main",
                },
                "large_artifacts": {
                    "dataset_downloaded": False,
                    "checkpoint_downloaded": False,
                },
            }
        ),
        encoding="utf-8",
    )

    report = load_manifest(prefix)
    assert report["valid"] is False
    assert any("40-character" in error for error in report["errors"])
    assert any("environment_prefix" in error for error in report["errors"])


def test_manifest_rejects_a_non_object_document(tmp_path: Path) -> None:
    prefix = tmp_path / "env"
    prefix.mkdir()
    (prefix / "b2d_vad_training_manifest.json").write_text("[]", encoding="utf-8")

    report = load_manifest(prefix)
    assert report["valid"] is False
    assert "schema_version must be 1" in report["errors"]


def test_install_refuses_before_mutation_without_license_acceptance(tmp_path: Path) -> None:
    prefix = tmp_path / "must_not_exist"
    exit_code = main(["--install", "--env-prefix", str(prefix)])

    assert exit_code == 2
    assert not prefix.exists()


def test_default_wrapper_is_read_only_check_and_has_no_large_download_commands() -> None:
    source = WRAPPER.read_text(encoding="utf-8")

    assert "set -- --check" in source
    assert "wget" not in source
    assert "curl" not in source
    assert "huggingface-cli" not in source
    assert 'exec python3 "${contract}" "$@"' in source


def test_check_reports_blocked_for_an_empty_isolated_prefix(tmp_path: Path) -> None:
    prefix = tmp_path / "empty-prefix"
    repo = tmp_path / "missing-repo"
    dataset = tmp_path / "missing-dataset"
    result = subprocess.run(
        [
            str(WRAPPER),
            "--check",
            "--env-prefix",
            str(prefix),
            "--repo-dir",
            str(repo),
            "--dataset-root",
            str(dataset),
        ],
        check=False,
        text=True,
        capture_output=True,
        cwd=ROOT,
    )

    assert result.returncode == 2
    report = json.loads(result.stdout)
    assert report["status"] == "BLOCKED"
    assert report["download_policy"]["dataset_auto_download"] is False
    assert report["download_policy"]["checkpoint_auto_download"] is False
    assert report["storage"]["active_scope"]["profile"] == "smoke"
    assert all(
        "Base dataset" not in warning and "Full plus custom" not in warning
        for warning in report["warnings"]
    )
    assert any("Python" in reason for reason in report["blocking_reasons"])
