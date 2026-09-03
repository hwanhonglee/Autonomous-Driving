from pathlib import Path
import shutil
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "setup-dev-env.sh"


def test_missing_ansible_collection_fails_before_host_mutation(tmp_path: Path) -> None:
    isolated_script = tmp_path / SCRIPT.name
    shutil.copyfile(SCRIPT, isolated_script)

    result = subprocess.run(
        ["bash", str(isolated_script)],
        input="y\n",
        text=True,
        capture_output=True,
        check=False,
        timeout=5,
    )

    assert result.returncode == 2
    assert "does not contain a complete upstream ./ansible collection" in result.stderr
    assert "No package has been installed or changed" in result.stderr
    assert "bootstrap_preflight.sh" in result.stderr
    assert "Are you sure" not in result.stdout + result.stderr


def test_missing_ansible_guard_precedes_package_install_commands() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    guard = source.index('if [ ! -f "$SCRIPT_DIR/ansible/galaxy.yml" ]')
    first_package_mutation = source.index("apt-get -y update")
    assert guard < first_package_mutation
    assert '[ ! -d "$SCRIPT_DIR/ansible/roles" ]' in source
    assert '<"$SCRIPT_DIR/amd64.env"' in source
    assert "<amd64.env" not in source


def test_incomplete_ansible_directory_cannot_bypass_guard(tmp_path: Path) -> None:
    isolated_script = tmp_path / SCRIPT.name
    shutil.copyfile(SCRIPT, isolated_script)
    (tmp_path / "ansible").mkdir()

    result = subprocess.run(
        ["bash", str(isolated_script)],
        input="y\n",
        text=True,
        capture_output=True,
        check=False,
        timeout=5,
    )

    assert result.returncode == 2
    assert "No package has been installed or changed" in result.stderr
    assert "Are you sure" not in result.stdout + result.stderr


def test_help_is_a_successful_read_only_operation() -> None:
    result = subprocess.run(
        ["bash", str(SCRIPT), "--help"],
        text=True,
        capture_output=True,
        check=False,
        timeout=5,
    )

    assert result.returncode == 0
    assert "Usage:" in result.stdout
    assert "No package has been installed" not in result.stderr
