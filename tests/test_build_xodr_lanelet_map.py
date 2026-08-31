from pathlib import Path
import os
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/build_xodr_lanelet_map.sh"
REQUIREMENTS = ROOT / "requirements-map.txt"


def write_executable(path: Path, source: str) -> None:
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def test_requirements_pin_verified_converter_versions() -> None:
    requirements = {
        line.strip()
        for line in REQUIREMENTS.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("#")
    }

    assert requirements == {
        "click==8.1.7",
        "commonroad-scenario-designer==0.8.5",
    }


def test_help_describes_both_conversion_stages() -> None:
    result = subprocess.run(
        [str(SCRIPT), "--help"],
        check=False,
        text=True,
        capture_output=True,
        cwd=ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert "SOURCE_XODR RAW_OSM OUTPUT_OSM" in result.stdout
    assert "CommonRoad Lanelet2" in result.stdout
    assert "--translation-z-m" in result.stdout
    assert "--json-report" in result.stdout


def test_install_dry_run_prints_plan_without_writing(tmp_path: Path) -> None:
    source_xodr = tmp_path / "source.xodr"
    source_xodr.write_text("<OpenDRIVE/>", encoding="utf-8")
    fake_finalizer_python = tmp_path / "finalizer-python"
    write_executable(fake_finalizer_python, "#!/usr/bin/env bash\nexit 0\n")
    venv = tmp_path / "new-venv"
    raw_osm = tmp_path / "output/raw.osm"
    output_osm = tmp_path / "output/autoware.osm"
    report = tmp_path / "output/finalize.json"

    result = subprocess.run(
        [
            str(SCRIPT),
            "--install",
            "--dry-run",
            "--venv",
            str(venv),
            "--finalizer-python",
            str(fake_finalizer_python),
            "--json-report",
            str(report),
            str(source_xodr),
            str(raw_osm),
            str(output_osm),
        ],
        check=False,
        text=True,
        capture_output=True,
        cwd=ROOT,
    )

    assert result.returncode == 0, result.stderr
    assert " -m venv " in result.stdout or "virtualenv" in result.stdout
    assert "requirements-map.txt" in result.stdout
    assert "--force-overwrite odrlanelet2" in result.stdout
    assert "finalize_xodr_lanelet_map.py" in result.stdout
    assert "--json" in result.stdout
    assert f"> {report}" in result.stdout
    assert not venv.exists()
    assert not raw_osm.exists()
    assert not output_osm.exists()
    assert not report.exists()


def test_mock_pipeline_runs_commonroad_before_finalizer(tmp_path: Path) -> None:
    source_xodr = tmp_path / "source.xodr"
    source_xodr.write_text("<OpenDRIVE/>", encoding="utf-8")
    venv = tmp_path / "venv"
    bin_dir = venv / "bin"
    bin_dir.mkdir(parents=True)
    log = tmp_path / "calls.log"

    write_executable(
        bin_dir / "python",
        """#!/usr/bin/env bash
set -euo pipefail
[[ "${1:-}" == "-c" ]] || exit 2
printf '0.8.5|8.1.7\\n'
""",
    )
    write_executable(
        bin_dir / "crdesigner",
        """#!/usr/bin/env bash
set -euo pipefail
printf 'converter %s\\n' "$*" >> "${MOCK_LOG}"
output=''
while [[ $# -gt 0 ]]; do
  if [[ "$1" == "--output-file" ]]; then output="$2"; shift 2; else shift; fi
done
[[ -n "${output}" ]]
printf '<osm version="0.6"/>\\n' > "${output}"
""",
    )
    fake_finalizer_python = tmp_path / "finalizer-python"
    write_executable(
        fake_finalizer_python,
        """#!/usr/bin/env bash
set -euo pipefail
if [[ "${2:-}" == "--help" ]]; then exit 0; fi
printf 'finalizer %s\\n' "$*" >> "${MOCK_LOG}"
printf '<osm version="0.6" generator="finalized"/>\\n' > "$4"
if [[ " $* " == *' --json '* ]]; then printf '{"status":"PASS"}\\n'; fi
""",
    )

    raw_osm = tmp_path / "generated/raw.osm"
    output_osm = tmp_path / "generated/autoware.osm"
    report = tmp_path / "generated/finalize.json"
    environment = os.environ.copy()
    environment["MOCK_LOG"] = str(log)
    result = subprocess.run(
        [
            str(SCRIPT),
            "--venv",
            str(venv),
            "--finalizer-python",
            str(fake_finalizer_python),
            "--translation-z-m",
            "-15",
            "--map-version",
            "test-map",
            "--json-report",
            str(report),
            str(source_xodr),
            str(raw_osm),
            str(output_osm),
        ],
        check=False,
        text=True,
        capture_output=True,
        cwd=ROOT,
        env=environment,
    )

    assert result.returncode == 0, result.stderr
    calls = log.read_text(encoding="utf-8").splitlines()
    assert calls[0].startswith("converter ")
    assert "--force-overwrite odrlanelet2" in calls[0]
    assert calls[1].startswith("finalizer ")
    assert "--translation-z-m -15" in calls[1]
    assert "--map-version test-map" in calls[1]
    assert raw_osm.read_text(encoding="utf-8").startswith("<osm")
    assert "finalized" in output_osm.read_text(encoding="utf-8")
    assert report.read_text(encoding="utf-8") == '{"status":"PASS"}\n'
    assert "XODR Lanelet map build completed" in result.stdout


def test_rejects_converter_version_drift(tmp_path: Path) -> None:
    source_xodr = tmp_path / "source.xodr"
    source_xodr.write_text("<OpenDRIVE/>", encoding="utf-8")
    venv = tmp_path / "venv"
    bin_dir = venv / "bin"
    bin_dir.mkdir(parents=True)
    write_executable(
        bin_dir / "python",
        "#!/usr/bin/env bash\nprintf '0.9.0|8.1.7\\n'\n",
    )
    write_executable(bin_dir / "crdesigner", "#!/usr/bin/env bash\nexit 0\n")
    fake_finalizer_python = tmp_path / "finalizer-python"
    write_executable(fake_finalizer_python, "#!/usr/bin/env bash\nexit 0\n")

    result = subprocess.run(
        [
            str(SCRIPT),
            "--venv",
            str(venv),
            "--finalizer-python",
            str(fake_finalizer_python),
            "--dry-run",
            str(source_xodr),
            str(tmp_path / "raw.osm"),
            str(tmp_path / "autoware.osm"),
        ],
        check=False,
        text=True,
        capture_output=True,
        cwd=ROOT,
    )

    assert result.returncode == 1
    assert "version mismatch" in result.stderr
    assert "expected 0.8.5|8.1.7, got 0.9.0|8.1.7" in result.stderr
