from __future__ import annotations

import hashlib
import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).parents[1]
ENV_SCRIPT = ROOT / "scripts/e2e/env.sh"


def _source_env(config: Path, expected_sha256: str) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment.update(
        {
            "AUTOWARE_E2E_SKIP_INSTALL": "1",
            "AUTOWARE_E2E_PINNED_CYCLONEDDS_URI": config.resolve().as_uri(),
            "AUTOWARE_E2E_PINNED_CYCLONEDDS_SHA256": expected_sha256,
        }
    )
    return subprocess.run(
        [
            "bash",
            "-c",
            (
                'source "$1"\n'
                "status=$?\n"
                'if (( status != 0 )); then exit "${status}"; fi\n'
                'source "$1"\n'
                "status=$?\n"
                'if (( status != 0 )); then exit "${status}"; fi\n'
                'printf "%s" "${CYCLONEDDS_URI:-}"'
            ),
            "bash",
            str(ENV_SCRIPT),
        ],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )


def test_env_restores_only_hash_pinned_cyclonedds_uri(tmp_path: Path) -> None:
    config = tmp_path / "cyclonedds.xml"
    config.write_text("<CycloneDDS/>", encoding="utf-8")
    digest = hashlib.sha256(config.read_bytes()).hexdigest()

    completed = _source_env(config, digest)

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout == config.resolve().as_uri()


def test_env_rejects_pinned_cyclonedds_hash_drift(tmp_path: Path) -> None:
    config = tmp_path / "cyclonedds.xml"
    config.write_text("<CycloneDDS/>", encoding="utf-8")

    completed = _source_env(config, "0" * 64)

    assert completed.returncode != 0
    assert "config SHA-256 mismatch" in completed.stderr


def test_env_discards_untrusted_inherited_cyclonedds_uri() -> None:
    environment = os.environ.copy()
    environment.update(
        {
            "AUTOWARE_E2E_SKIP_INSTALL": "1",
            "CYCLONEDDS_URI": "file:///tmp/untrusted.xml",
        }
    )
    environment.pop("AUTOWARE_E2E_PINNED_CYCLONEDDS_URI", None)
    environment.pop("AUTOWARE_E2E_PINNED_CYCLONEDDS_SHA256", None)

    completed = subprocess.run(
        [
            "bash",
            "-c",
            'source "$1"\nprintf "%s" "${CYCLONEDDS_URI-unset}"',
            "bash",
            str(ENV_SCRIPT),
        ],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout == "unset"
