from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
AUTOWARE_LAUNCH = ROOT / "src/launcher/autoware_launch"
CONTROL_PATCH = ROOT / "patches/autoware_launch_lateral_param_override.patch"
SMART_MPC_PATCH = ROOT / "patches/autoware_launch_smart_mpc_runtime_param.patch"
APPLY_SCRIPT = ROOT / "scripts/e2e/apply_autoware_launch_control_override.sh"

AUTOWARE_XML = "autoware_launch/launch/autoware.launch.xml"
CONTROL_XML = (
    "autoware_launch/launch/components/tier4_control_component.launch.xml"
)
SMART_CONTROL_XML = "tier4_universe_launch/tier4_control_launch/launch/control.launch.xml"


def direct_args(root):
    return {node.get("name"): node for node in root.findall("./arg")}


def test_installed_launch_chain_keeps_both_overrides_opt_in():
    autoware = ET.parse(AUTOWARE_LAUNCH / AUTOWARE_XML).getroot()
    component = ET.parse(AUTOWARE_LAUNCH / CONTROL_XML).getroot()
    autoware_args = direct_args(autoware)
    component_args = direct_args(component)

    for args in (autoware_args, component_args):
        assert args["use_lateral_controller_param_override"].get("default") == "false"
        assert args["lateral_controller_param_path"].get("default") == ""
        assert args["use_longitudinal_controller_param_override"].get("default") == "false"
        assert args["longitudinal_controller_param_path"].get("default") == ""

    component_include = next(
        node
        for node in autoware.findall("./group/include")
        if "tier4_control_component.launch.xml" in node.get("file", "")
    )
    component_passed = {
        node.get("name"): node.get("value")
        for node in component_include.findall("./arg")
    }
    for name in (
        "use_lateral_controller_param_override",
        "lateral_controller_param_path",
        "use_longitudinal_controller_param_override",
        "longitudinal_controller_param_path",
    ):
        assert component_passed[name] == f"$(var {name})"

    lets = {node.get("name"): node for node in component.findall("./let")}
    assert lets["resolved_lateral_controller_param_path"].get("unless") == (
        "$(var use_lateral_controller_param_override)"
    )
    assert lets["resolved_longitudinal_controller_param_path"].get("unless") == (
        "$(var use_longitudinal_controller_param_override)"
    )

    control = next(
        node for node in component.findall("./include") if "control.launch.xml" in node.get("file", "")
    )
    passed = {node.get("name"): node.get("value") for node in control.findall("./arg")}
    assert passed["lat_controller_param_path"] == "$(var resolved_lateral_controller_param_path)"
    assert passed["lon_controller_param_path"] == "$(var resolved_longitudinal_controller_param_path)"


def test_apply_script_idempotence_requires_longitudinal_markers():
    source = APPLY_SCRIPT.read_text(encoding="utf-8")
    for marker in (
        'name="use_lateral_controller_param_override"',
        'name="resolved_lateral_controller_param_path"',
        'name="use_longitudinal_controller_param_override"',
        'name="resolved_longitudinal_controller_param_path"',
        "git -C \"${repository}\" apply --reverse --check",
    ):
        assert marker in source


def test_build_entrypoints_call_the_idempotent_control_patch_once():
    marker = "scripts/e2e/apply_autoware_launch_control_override.sh"
    for name in ("build.sh", "build_full.sh"):
        source = (ROOT / "scripts/e2e" / name).read_text(encoding="utf-8")
        assert source.count(marker) == 1


def test_control_and_smart_mpc_patches_replay_from_pinned_checkout(tmp_path):
    workspace = tmp_path / "workspace"
    checkout = workspace / "src/launcher/autoware_launch"
    checkout.mkdir(parents=True)

    for relative in (AUTOWARE_XML, CONTROL_XML, SMART_CONTROL_XML):
        destination = checkout / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        content = subprocess.run(
            ["git", "-C", str(AUTOWARE_LAUNCH), "show", f"HEAD:{relative}"],
            check=True,
            capture_output=True,
        ).stdout
        destination.write_bytes(content)

    subprocess.run(["git", "init", "-q", str(checkout)], check=True)
    copied_patch = workspace / "patches" / CONTROL_PATCH.name
    copied_patch.parent.mkdir(parents=True)
    copied_patch.write_bytes(CONTROL_PATCH.read_bytes())
    copied_script = workspace / "scripts/e2e" / APPLY_SCRIPT.name
    copied_script.parent.mkdir(parents=True)
    copied_script.write_bytes(APPLY_SCRIPT.read_bytes())

    first = subprocess.run(
        ["bash", str(copied_script)], check=True, capture_output=True, text=True
    )
    assert "Applied Autoware lateral and longitudinal" in first.stdout
    subprocess.run(
        [
            "git",
            "-C",
            str(checkout),
            "apply",
            "--reverse",
            "--check",
            str(CONTROL_PATCH),
        ],
        check=True,
    )
    second = subprocess.run(
        ["bash", str(copied_script)], check=True, capture_output=True, text=True
    )
    assert "already applied" in second.stdout
    subprocess.run(
        ["git", "-C", str(checkout), "apply", "--check", str(SMART_MPC_PATCH)],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(checkout), "apply", str(SMART_MPC_PATCH)], check=True
    )

    component = ET.parse(checkout / CONTROL_XML).getroot()
    args = direct_args(component)
    assert args["use_longitudinal_controller_param_override"].get("default") == "false"
    assert args["longitudinal_controller_param_path"].get("default") == ""
