from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]
AUTOWARE_CORE = ROOT / "src/core/autoware_core"
PATCH = ROOT / "patches/autoware_map_loader_clean_shutdown.patch"
APPLY_SCRIPT = ROOT / "scripts/e2e/apply_map_loader_clean_shutdown_patch.sh"
TRACKED_FILES = (
    "map/autoware_map_loader/CMakeLists.txt",
    "map/autoware_map_loader/package.xml",
    "map/autoware_map_loader/script/map_hash_generator",
)


def test_patch_preserves_expected_shutdown_and_test_contract() -> None:
    patch = PATCH.read_text(encoding="utf-8")

    # HH_260906 - Bind the portable patch to its shutdown behavior and regression test.
    for marker in (
        "from rclpy.executors import ExternalShutdownException",
        "except (KeyboardInterrupt, ExternalShutdownException):",
        "node.destroy_node()",
        "if rclpy.ok():",
        "ament_add_pytest_test(test_map_hash_generator",
        "test_main_handles_expected_executor_shutdown",
    ):
        assert marker in patch


def test_apply_script_replays_patch_from_pinned_core_checkout(tmp_path: Path) -> None:
    workspace = tmp_path / "workspace"
    checkout = workspace / "src/core/autoware_core"
    checkout.mkdir(parents=True)

    for relative in TRACKED_FILES:
        destination = checkout / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        content = subprocess.run(
            ["git", "-C", str(AUTOWARE_CORE), "show", f"HEAD:{relative}"],
            check=True,
            capture_output=True,
        ).stdout
        destination.write_bytes(content)

    subprocess.run(["git", "init", "-q", str(checkout)], check=True)
    copied_patch = workspace / "patches" / PATCH.name
    copied_patch.parent.mkdir(parents=True)
    copied_patch.write_bytes(PATCH.read_bytes())
    copied_script = workspace / "scripts/e2e" / APPLY_SCRIPT.name
    copied_script.parent.mkdir(parents=True)
    shutil.copy2(APPLY_SCRIPT, copied_script)

    first = subprocess.run(
        ["bash", str(copied_script)], check=True, capture_output=True, text=True
    )
    assert "Applied map loader clean shutdown patch." in first.stdout
    assert (checkout / "map/autoware_map_loader/test/test_map_hash_generator.py").is_file()

    second = subprocess.run(
        ["bash", str(copied_script)], check=True, capture_output=True, text=True
    )
    assert "already applied" in second.stdout


def test_build_entrypoints_apply_clean_shutdown_patch_once() -> None:
    marker = "scripts/e2e/apply_map_loader_clean_shutdown_patch.sh"
    for name in ("build.sh", "build_full.sh"):
        source = (ROOT / "scripts/e2e" / name).read_text(encoding="utf-8")
        assert source.count(marker) == 1
        assert "autoware_map_loader" in source
