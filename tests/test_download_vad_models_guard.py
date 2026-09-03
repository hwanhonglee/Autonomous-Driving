from pathlib import Path
import os
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "e2e" / "download_vad_models.sh"


def _run(argument: str, data_root: Path) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment["AUTOWARE_E2E_DATA_PATH"] = str(data_root)
    return subprocess.run(
        ["bash", str(SCRIPT), argument],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
        timeout=5,
    )


def test_help_exits_without_creating_the_model_directory(tmp_path: Path) -> None:
    data_root = tmp_path / "models"

    completed = _run("--help", data_root)

    assert completed.returncode == 0
    assert "Usage:" in completed.stdout
    assert not data_root.exists()


def test_unknown_argument_is_rejected_without_download_or_mutation(tmp_path: Path) -> None:
    data_root = tmp_path / "models"

    completed = _run("--typo", data_root)

    assert completed.returncode == 2
    assert "accepts no arguments" in completed.stderr
    assert not data_root.exists()


def test_argument_guard_precedes_destination_creation_and_network_use() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    guard = source.index("if (( $# > 0 )); then")
    assert guard < source.index('mkdir -p "${dest}"')
    assert guard < source.index("curl --fail")
