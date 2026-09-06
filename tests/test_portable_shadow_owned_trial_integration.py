# HH_260906 - Verify the fail-closed Portable E2E shadow integration for owned route trials.
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).parents[1]
VALIDATOR = ROOT / "scripts/e2e/validate_portable_shadow_trial.py"
RECORDED_TRIAL = ROOT / "scripts/e2e/run_recorded_route_trial.sh"
OWNED_TRIAL = ROOT / "scripts/e2e/run_owned_carla_route_trial.sh"
FAST_RUNNER = ROOT / "scripts/e2e/run_route_vad_fast.sh"
RECORDER = ROOT / "scripts/e2e/record_turn_dynamics.sh"
RUNTIME_HEALTH_PROBE = ROOT / "scripts/e2e/probe_runtime_health.py"
PORTABLE_MAPPING = (
    ROOT / "autoware_e2e_vad_launch/config/sensor_mapping_portable_e2e_10hz.yaml"
)
SHADOW_LAUNCH = (
    ROOT / "autoware_e2e_vad_launch/launch/portable_e2e_shadow.launch.xml"
)
VAD_BEST_EFFORT_OVERRIDE = ROOT / (
    "autoware_e2e_vad_launch/config/"
    "vad_carla_tiny_camera_source_5hz_best_effort_image_depth1.param.yaml"
)
CYCLONEDDS_CONFIG = (
    ROOT / "autoware_e2e_vad_launch/config/cyclonedds_camera_depth1_localhost_v2.xml"
)
SHA = "1" * 64


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _portable_cpu_profile_command(
    tmp_path: Path,
    *cpu_sets: str,
    device: str = "cpu",
) -> list[str]:
    # HH_260906 - Build a complete profile so CPU policy errors precede route preflight.
    command = [
        str(RECORDED_TRIAL),
        "--portable-shadow-10hz",
        "--speed-30kph",
        "--portable-runtime-bundle",
        "unused.npz",
        "--portable-runtime-bundle-sha256",
        SHA,
        "--portable-source-checkpoint-sha256",
        SHA,
        "--portable-model-config-sha256",
        SHA,
        "--portable-corpus-fingerprint-sha256",
        SHA,
        "--portable-rig-file",
        "unused-rig.json",
        "--portable-rig-sha256",
        SHA,
        "--portable-contract-file",
        "unused-contract.json",
        "--portable-contract-sha256",
        SHA,
        "--portable-device",
        device,
    ]
    for cpu_set in cpu_sets:
        command.extend(("--portable-cpu-set", cpu_set))
    command.extend((str(tmp_path / "output"), str(tmp_path / "missing-route.json")))
    return command


def _available_portable_cpu_ids(count: int = 4) -> tuple[int, ...]:
    # HH_260906 - Select real affinity IDs so CPU policy tests remain topology-independent.
    available = tuple(sorted(os.sched_getaffinity(0)))
    if len(available) < count:
        pytest.skip(f"Portable CPU policy tests require {count} available CPUs")
    return available[:count]


def _fixture(tmp_path: Path) -> tuple[list[str], dict[str, Path]]:
    output = tmp_path / "trial"
    output.mkdir()
    source = tmp_path / "source.json"
    _write_json(
        source,
        {
            "schema_version": 1,
            "coordinate_reference": "base_link",
            "town": "Town07",
            "scenario": "straight",
            "route": [{"x": 0.0, "y": 0.0}],
        },
    )
    source_sha256 = _sha256(source)
    (output / "source_route.json").write_bytes(source.read_bytes())
    transform = {"x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "yaw_rad": 0.0}
    # HH_260906 - Mirror real Town07 metadata only in its authoritative map bundle.
    map_bundle_transform = {
        **transform,
        "kind": "identity_after_carla_interface_ros_handedness_conversion",
        "confidence": "lanelet_route_and_reflected_pcd_alignment_verified",
        "source": "packaged Town route/Lanelet2/HDMaps audit on 2026-08-31",
    }
    aligned = output / "aligned_route.json"
    _write_json(
        aligned,
        {
            "schema_version": 1,
            "coordinate_reference": "base_link",
            "town": "Town07",
            "scenario": "straight",
            "route": [{"x": 0.0, "y": 0.0}],
            "coordinate_alignment": {
                "schema_version": 1,
                "source_frame": "carla_map",
                "target_frame": "map",
                "source_route": str(source.resolve()),
                "source_route_sha256": source_sha256,
                "map_bundle_profile": "town07",
                "carla_to_map_transform": transform,
            },
        },
    )
    alignment = output / "route_alignment.json"
    _write_json(
        alignment,
        {
            "status": "PASS",
            "source_route": str(source.resolve()),
            "source_route_sha256": source_sha256,
            "aligned_route": str(aligned.resolve()),
            "aligned_route_sha256": _sha256(aligned),
            "profile": "town07",
            "already_aligned": False,
            "carla_to_map_transform": transform,
        },
    )
    map_bundle = output / "map_bundle.json"
    _write_json(
        map_bundle,
        {
            "schema_version": 1,
            "profile": "town07",
            "canonical_carla_map": "/Game/Carla/Maps/Town07",
            "carla_to_map_transform": map_bundle_transform,
        },
    )
    carla_probe = output / "carla_preflight_health.json"
    _write_json(
        carla_probe,
        {
            "schema_version": 1,
            "status": "PASS",
            "mode": "running",
            "stage": "trial_preflight",
            "expected_map": "Town07",
            "active_map_name": "Carla/Maps/Town07",
            "active_map_basename": "Town07",
            "rpc_sequence": ["get_world", "world.get_map", "world.get_snapshot"],
            "read_only": True,
        },
    )
    runtime_bundle = tmp_path / "runtime.npz"
    runtime_bundle.write_bytes(b"safe non-executable test bundle")
    contract = tmp_path / "contract.json"
    contract.write_text("{}\n", encoding="utf-8")
    rig = tmp_path / "rig.json"
    rig.write_text("{}\n", encoding="utf-8")
    shadow_launch = tmp_path / "shadow.launch.xml"
    shadow_launch.write_bytes(SHADOW_LAUNCH.read_bytes())
    installed_launch = tmp_path / "installed-shadow.launch.xml"
    installed_launch.symlink_to(shadow_launch)
    result = output / "portable_shadow_provenance/trial_binding.json"
    shadow_launch_snapshot = (
        output / "portable_shadow_provenance/runtime_shadow.launch.xml"
    )
    paths = {
        "output": output,
        "source": source,
        "aligned": aligned,
        "alignment": alignment,
        "map_bundle": map_bundle,
        "carla_probe": carla_probe,
        "runtime_bundle": runtime_bundle,
        "contract": contract,
        "rig": rig,
        "shadow_launch": shadow_launch,
        "installed_launch": installed_launch,
        "shadow_launch_snapshot": shadow_launch_snapshot,
        "result": result,
    }
    command = [
        "python3",
        str(VALIDATOR),
        "--output-dir",
        str(output),
        "--source-route",
        str(source),
        "--aligned-route",
        str(aligned),
        "--route-alignment",
        str(alignment),
        "--map-bundle",
        str(map_bundle),
        "--carla-probe",
        str(carla_probe),
        "--expected-map",
        "Town07",
        "--sensor-mapping",
        str(PORTABLE_MAPPING),
        "--vad-model-override",
        str(VAD_BEST_EFFORT_OVERRIDE),
        "--cyclonedds-config",
        str(CYCLONEDDS_CONFIG),
        "--shadow-launch",
        str(shadow_launch),
        "--installed-shadow-launch",
        str(installed_launch),
        "--shadow-launch-snapshot",
        str(shadow_launch_snapshot),
        "--runtime-bundle",
        str(runtime_bundle),
        "--runtime-bundle-sha256",
        _sha256(runtime_bundle),
        "--source-checkpoint-sha256",
        SHA,
        "--model-config-sha256",
        SHA,
        "--corpus-fingerprint-sha256",
        SHA,
        "--contract-file",
        str(contract),
        "--contract-sha256",
        _sha256(contract),
        "--rig-file",
        str(rig),
        "--rig-sha256",
        _sha256(rig),
        "--device",
        "cpu",
        "--output",
        str(result),
    ]
    return command, paths


def _launch_recheck_command(
    paths: dict[str, Path],
    output: Path,
    *,
    binding_sha256: str | None = None,
    shadow_launch_snapshot: Path | None = None,
) -> list[str]:
    return [
        "python3",
        str(VALIDATOR),
        "recheck-launch",
        "--binding",
        str(paths["result"]),
        "--binding-sha256",
        binding_sha256 or _sha256(paths["result"]),
        "--shadow-launch",
        str(paths["shadow_launch"]),
        "--installed-shadow-launch",
        str(paths["installed_launch"]),
        "--shadow-launch-snapshot",
        str(shadow_launch_snapshot or paths["shadow_launch_snapshot"]),
        "--output",
        str(output),
    ]


def _replace_transform(
    paths: dict[str, Path], location: str, transform: dict[str, object]
) -> None:
    if location == "map_bundle":
        payload = json.loads(paths["map_bundle"].read_text(encoding="utf-8"))
        payload["carla_to_map_transform"] = transform
        _write_json(paths["map_bundle"], payload)
        return
    if location == "alignment_report":
        payload = json.loads(paths["alignment"].read_text(encoding="utf-8"))
        payload["carla_to_map_transform"] = transform
        _write_json(paths["alignment"], payload)
        return
    if location == "aligned_route":
        payload = json.loads(paths["aligned"].read_text(encoding="utf-8"))
        payload["coordinate_alignment"]["carla_to_map_transform"] = transform
        _write_json(paths["aligned"], payload)
        report = json.loads(paths["alignment"].read_text(encoding="utf-8"))
        report["aligned_route_sha256"] = _sha256(paths["aligned"])
        _write_json(paths["alignment"], report)
        return
    raise AssertionError(f"unknown transform location: {location}")


def test_validator_binds_independent_map_route_and_pinned_inputs(tmp_path: Path) -> None:
    command, paths = _fixture(tmp_path)

    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode == 0, completed.stderr
    payload = json.loads(paths["result"].read_text(encoding="utf-8"))
    assert payload["status"] == "PASS"
    assert payload["execution_mode"] == "shadow_only"
    assert payload["controlling_planner"] == "autoware_vad"
    assert payload["vehicle_control_approved"] is False
    assert payload["canonical_or_control_publication_allowed"] is False
    binding = payload["route_binding"]
    assert binding["expected_map"] == "Town07"
    assert binding["declared_map_id"] == "town07"
    assert binding["observed_map_id"] == "town07"
    assert binding["observed_map_source"] == "carla_python_api_world_get_map"
    assert binding["aligned_route_sha256"] == _sha256(paths["aligned"])
    assert binding["map_bundle_sha256"] == _sha256(paths["map_bundle"])
    assert binding["map_transform"] == {
        "x_m": 0.0,
        "y_m": 0.0,
        "z_m": 0.0,
        "yaw_rad": 0.0,
    }
    bundle = json.loads(paths["map_bundle"].read_text(encoding="utf-8"))
    assert bundle["carla_to_map_transform"] == {
        "x_m": 0.0,
        "y_m": 0.0,
        "z_m": 0.0,
        "yaw_rad": 0.0,
        "kind": "identity_after_carla_interface_ros_handedness_conversion",
        "confidence": "lanelet_route_and_reflected_pcd_alignment_verified",
        "source": "packaged Town route/Lanelet2/HDMaps audit on 2026-08-31",
    }
    alignment = json.loads(paths["alignment"].read_text(encoding="utf-8"))
    aligned = json.loads(paths["aligned"].read_text(encoding="utf-8"))
    assert set(alignment["carla_to_map_transform"]) == {
        "x_m",
        "y_m",
        "z_m",
        "yaw_rad",
    }
    assert set(aligned["coordinate_alignment"]["carla_to_map_transform"]) == {
        "x_m",
        "y_m",
        "z_m",
        "yaw_rad",
    }
    assert payload["sensor_mapping"]["effective_camera_rate_hz"] == 10
    assert payload["sensor_mapping"]["requested_bridge_publish_cap_hz"] == 11
    assert payload["controlling_vad_transport"]["vad_image_reliability"] == "best_effort"
    assert payload["controlling_vad_transport"]["vad_image_depth"] == 1
    assert payload["controlling_vad_transport"]["network_interface"] == "lo"
    assert payload["shadow_launch"]["remaps_allowed"] is False
    assert payload["shadow_launch"]["exact_same_name_parameter_bindings"] is True
    assert payload["shadow_launch"]["installed_binding"]["symlink_install"] is True
    assert payload["shadow_launch"]["installed_binding"][
        "matches_validated_source"
    ] is True
    snapshot_binding = payload["shadow_launch"]["snapshot_binding"]
    assert snapshot_binding == {
        "file": str(paths["shadow_launch_snapshot"].resolve()),
        "sha256": payload["shadow_launch"]["sha256"],
        "mode_octal": "0444",
        "regular_file": True,
        "symlink": False,
        "source_sha256": payload["shadow_launch"]["sha256"],
        "exact_source_bytes": True,
        "direct_ros_launch": True,
    }
    assert paths["shadow_launch_snapshot"].is_file()
    assert not paths["shadow_launch_snapshot"].is_symlink()
    assert paths["shadow_launch_snapshot"].stat().st_mode & 0o777 == 0o444
    assert paths["shadow_launch_snapshot"].read_bytes() == paths[
        "shadow_launch"
    ].read_bytes()
    assert payload["shadow_launch"]["pinned_defaults"] == {
        "device": "cpu",
        "input_settle_timeout_s": "0.05",
        "maximum_tf_rotation_error_rad": "0.005",
        "maximum_tf_translation_error_m": "0.005",
        "research_acknowledged": "false",
        "use_sim_time": "true",
    }


# HH_260906 - Exercise every transform surface with fail-closed numeric corruption.
@pytest.mark.parametrize(
    "location, label",
    (
        ("map_bundle", "map bundle transform"),
        ("alignment_report", "alignment report transform"),
        ("aligned_route", "aligned route transform"),
    ),
)
@pytest.mark.parametrize(
    "mutation, field, value, expected",
    (
        ("missing", "x_m", None, " must contain the exact map transform fields"),
        ("nonfinite", "y_m", float("nan"), ".y_m must be finite"),
        ("nonfinite", "z_m", float("inf"), ".z_m must be finite"),
        ("nonfinite", "yaw_rad", float("-inf"), ".yaw_rad must be finite"),
        ("boolean", "x_m", True, ".x_m must be numeric"),
    ),
)
def test_validator_fails_closed_on_invalid_transform_numeric_fields(
    tmp_path: Path,
    location: str,
    label: str,
    mutation: str,
    field: str,
    value: object,
    expected: str,
) -> None:
    command, paths = _fixture(tmp_path)
    transform: dict[str, object] = {
        "x_m": 0.0,
        "y_m": 0.0,
        "z_m": 0.0,
        "yaw_rad": 0.0,
    }
    if location == "map_bundle":
        transform.update(
            {
                "kind": "identity_after_carla_interface_ros_handedness_conversion",
                "confidence": "lanelet_route_and_reflected_pcd_alignment_verified",
                "source": "packaged Town route/Lanelet2/HDMaps audit on 2026-08-31",
            }
        )
    if mutation == "missing":
        transform.pop(field)
    else:
        transform[field] = value
    _replace_transform(paths, location, transform)

    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode != 0
    assert f"{label}{expected}" in completed.stderr
    assert not paths["result"].exists()


# HH_260906 - Keep the authoritative bundle metadata vocabulary explicit and closed.
def test_validator_rejects_unknown_map_bundle_transform_metadata(
    tmp_path: Path,
) -> None:
    command, paths = _fixture(tmp_path)
    bundle = json.loads(paths["map_bundle"].read_text(encoding="utf-8"))
    transform = bundle["carla_to_map_transform"]
    transform["calibration_note"] = "unsupported unbound metadata"
    _write_json(paths["map_bundle"], bundle)

    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode != 0
    assert (
        "map bundle transform contains unsupported fields: ['calibration_note']"
        in completed.stderr
    )
    assert not paths["result"].exists()


# HH_260906 - Reject empty or non-string values for every known metadata key.
@pytest.mark.parametrize(
    "field, value",
    (
        ("kind", ""),
        ("confidence", "   "),
        ("source", False),
    ),
)
def test_validator_rejects_invalid_known_map_bundle_transform_metadata(
    tmp_path: Path, field: str, value: object
) -> None:
    command, paths = _fixture(tmp_path)
    bundle = json.loads(paths["map_bundle"].read_text(encoding="utf-8"))
    bundle["carla_to_map_transform"][field] = value
    _write_json(paths["map_bundle"], bundle)

    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode != 0
    assert (
        f"map bundle transform.{field} must be a non-empty string"
        in completed.stderr
    )
    assert not paths["result"].exists()


# HH_260906 - Keep derived report and route transforms canonical numeric objects.
@pytest.mark.parametrize(
    "location, label",
    (
        ("alignment_report", "alignment report transform"),
        ("aligned_route", "aligned route transform"),
    ),
)
@pytest.mark.parametrize("field", ("kind", "confidence", "source"))
def test_validator_rejects_metadata_on_canonical_trial_transforms(
    tmp_path: Path, location: str, label: str, field: str
) -> None:
    command, paths = _fixture(tmp_path)
    transform: dict[str, object] = {
        "x_m": 0.0,
        "y_m": 0.0,
        "z_m": 0.0,
        "yaw_rad": 0.0,
        field: "metadata is only legal in the map bundle",
    }
    _replace_transform(paths, location, transform)

    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode != 0
    assert f"{label} contains unsupported fields: ['{field}']" in completed.stderr
    assert not paths["result"].exists()


@pytest.mark.parametrize(
    "mutation, expected",
    (
        ("observed_map", "independent CARLA API map observation"),
        ("aligned_digest", "route alignment output digest"),
        ("canonical_map", "canonical CARLA map"),
        ("source_mutation", "source route copy does not match"),
        ("mapping_rate", "violates the 10 Hz QoS contract"),
        ("vad_reliable", "must use Best-Effort KEEP_LAST depth 1"),
        ("cyclone_interface", "must pin the loopback interface"),
        ("launch_remap", "must not contain remaps"),
        ("launch_binding", "not bound to its same-name input"),
        ("settle_default", "launch default changed"),
        ("tf_default", "launch default changed"),
        ("required_input_default", "required launch input gained a default"),
        ("installed_launch_mismatch", "does not match validated source"),
        ("rig_symlink", "regular non-symlink"),
    ),
)
def test_validator_fails_closed_on_binding_mutations(
    tmp_path: Path, mutation: str, expected: str
) -> None:
    command, paths = _fixture(tmp_path)
    if mutation == "observed_map":
        probe = json.loads(paths["carla_probe"].read_text(encoding="utf-8"))
        probe["active_map_name"] = "Carla/Maps/Town03"
        probe["active_map_basename"] = "Town03"
        _write_json(paths["carla_probe"], probe)
    elif mutation == "aligned_digest":
        alignment = json.loads(paths["alignment"].read_text(encoding="utf-8"))
        alignment["aligned_route_sha256"] = "0" * 64
        _write_json(paths["alignment"], alignment)
    elif mutation == "canonical_map":
        bundle = json.loads(paths["map_bundle"].read_text(encoding="utf-8"))
        bundle["canonical_carla_map"] = "/Game/Carla/Maps/Town03"
        _write_json(paths["map_bundle"], bundle)
    elif mutation == "source_mutation":
        paths["source"].write_text('{"town":"Town07"}\n', encoding="utf-8")
    elif mutation == "mapping_rate":
        mapping = tmp_path / "mapping.yaml"
        payload = yaml.safe_load(PORTABLE_MAPPING.read_text(encoding="utf-8"))
        payload["sensor_mappings"]["CAM_FRONT/camera_link"]["ros_config"][
            "frequency_hz"
        ] = 5
        mapping.write_text(yaml.safe_dump(payload), encoding="utf-8")
        index = command.index("--sensor-mapping") + 1
        command[index] = str(mapping)
    elif mutation == "vad_reliable":
        override = tmp_path / "vad.param.yaml"
        payload = yaml.safe_load(VAD_BEST_EFFORT_OVERRIDE.read_text(encoding="utf-8"))
        payload["/**"]["ros__parameters"]["sync_params"]["image_reliability"] = (
            "reliable"
        )
        override.write_text(yaml.safe_dump(payload), encoding="utf-8")
        command[command.index("--vad-model-override") + 1] = str(override)
    elif mutation == "cyclone_interface":
        cyclone = tmp_path / "cyclonedds.xml"
        cyclone.write_text(
            CYCLONEDDS_CONFIG.read_text(encoding="utf-8").replace(
                'name="lo"', 'name="eth0"'
            ),
            encoding="utf-8",
        )
        command[command.index("--cyclonedds-config") + 1] = str(cyclone)
    elif mutation == "launch_remap":
        launch = tmp_path / "shadow.launch.xml"
        source = SHADOW_LAUNCH.read_text(encoding="utf-8")
        launch.write_text(
            source.replace("</launch>", '<remap from="a" to="b"/>\n</launch>'),
            encoding="utf-8",
        )
        index = command.index("--shadow-launch") + 1
        command[index] = str(launch)
    elif mutation == "launch_binding":
        launch = tmp_path / "shadow.launch.xml"
        source = SHADOW_LAUNCH.read_text(encoding="utf-8")
        launch.write_text(
            source.replace(
                'name="contract_sha256" value="$(var contract_sha256)"',
                'name="contract_sha256" value="$(var rig_sha256)"',
            ),
            encoding="utf-8",
        )
        command[command.index("--shadow-launch") + 1] = str(launch)
    elif mutation == "settle_default":
        launch = tmp_path / "shadow.launch.xml"
        source = SHADOW_LAUNCH.read_text(encoding="utf-8")
        launch.write_text(
            source.replace(
                'name="input_settle_timeout_s"\n    default="0.05"',
                'name="input_settle_timeout_s"\n    default="0.50"',
            ),
            encoding="utf-8",
        )
        command[command.index("--shadow-launch") + 1] = str(launch)
    elif mutation == "tf_default":
        launch = tmp_path / "shadow.launch.xml"
        source = SHADOW_LAUNCH.read_text(encoding="utf-8")
        launch.write_text(
            source.replace(
                'name="maximum_tf_translation_error_m"\n    default="0.005"',
                'name="maximum_tf_translation_error_m"\n    default="0.05"',
            ),
            encoding="utf-8",
        )
        command[command.index("--shadow-launch") + 1] = str(launch)
    elif mutation == "required_input_default":
        launch = tmp_path / "shadow.launch.xml"
        source = SHADOW_LAUNCH.read_text(encoding="utf-8")
        launch.write_text(
            source.replace(
                'name="contract_file" description=',
                'name="contract_file" default="unsafe.json" description=',
            ),
            encoding="utf-8",
        )
        command[command.index("--shadow-launch") + 1] = str(launch)
    elif mutation == "installed_launch_mismatch":
        installed = tmp_path / "installed-mismatch.launch.xml"
        installed.write_text(
            SHADOW_LAUNCH.read_text(encoding="utf-8").replace(
                'name="portable_e2e_shadow"',
                'name="portable_e2e_shadow_changed"',
            ),
            encoding="utf-8",
        )
        command[command.index("--installed-shadow-launch") + 1] = str(installed)
    elif mutation == "rig_symlink":
        symlink = tmp_path / "rig-link.json"
        symlink.symlink_to(paths["rig"])
        index = command.index("--rig-file") + 1
        command[index] = str(symlink)

    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode != 0
    assert expected in completed.stderr
    assert not paths["result"].exists()


@pytest.mark.parametrize(
    ("mutation", "before", "after"),
    (
        (
            "top_level_include",
            "\n</launch>",
            '\n  <include file="/tmp/unsafe.launch.xml"/>\n</launch>',
        ),
        (
            "top_level_executable",
            "\n</launch>",
            '\n  <executable cmd="/bin/true"/>\n</launch>',
        ),
        (
            "top_level_group",
            "\n</launch>",
            "\n  <group/>\n</launch>",
        ),
        (
            "top_level_set_env",
            "\n</launch>",
            '\n  <set_env name="UNSAFE" value="1"/>\n</launch>',
        ),
        (
            "node_launch_prefix",
            '    output="screen"\n',
            '    output="screen"\n    launch-prefix="/bin/true"\n',
        ),
        (
            "node_extra_attribute",
            '    output="screen"\n',
            '    output="screen"\n    respawn="true"\n',
        ),
        (
            "node_extra_child",
            "  </node>",
            '    <executable cmd="/bin/true"/>\n  </node>',
        ),
        (
            "node_env",
            "  </node>",
            '    <env name="UNSAFE" value="1"/>\n  </node>',
        ),
        (
            "node_remap",
            "  </node>",
            '    <remap from="shadow" to="/planning/trajectory"/>\n  </node>',
        ),
        (
            "arg_extra_attribute",
            '<arg name="contract_file" description=',
            '<arg name="contract_file" if="$(var use_sim_time)" description=',
        ),
        (
            "arg_child",
            (
                '<arg name="contract_file" '
                'description="Pinned regular Common10 contract JSON file"/>'
            ),
            (
                '<arg name="contract_file" '
                'description="Pinned regular Common10 contract JSON file">'
                '<set_env name="UNSAFE" value="1"/></arg>'
            ),
        ),
        (
            "param_extra_attribute",
            '<param name="use_sim_time" value="$(var use_sim_time)"/>',
            (
                '<param name="use_sim_time" value="$(var use_sim_time)" '
                'if="$(var use_sim_time)"/>'
            ),
        ),
        (
            "param_child",
            '<param name="use_sim_time" value="$(var use_sim_time)"/>',
            (
                '<param name="use_sim_time" value="$(var use_sim_time)">'
                '<env name="UNSAFE" value="1"/></param>'
            ),
        ),
    ),
)
def test_validator_rejects_non_allowlisted_launch_structure(
    tmp_path: Path, mutation: str, before: str, after: str
) -> None:
    # HH_260906 - Reject every launch construct outside the pinned declarative allowlist.
    command, paths = _fixture(tmp_path)
    source = paths["shadow_launch"].read_text(encoding="utf-8")
    assert before in source, mutation
    paths["shadow_launch"].write_text(
        source.replace(before, after, 1),
        encoding="utf-8",
    )

    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode != 0, mutation
    assert "Portable E2E shadow launch" in completed.stderr
    assert not paths["result"].exists()


def test_validator_refuses_to_overwrite_binding(tmp_path: Path) -> None:
    command, paths = _fixture(tmp_path)
    subprocess.run(command, check=True, capture_output=True, text=True)

    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode != 0
    assert "output already exists" in completed.stderr
    assert json.loads(paths["result"].read_text(encoding="utf-8"))["status"] == "PASS"


def test_validator_accepts_an_exact_regular_install_copy(tmp_path: Path) -> None:
    # HH_260906 - Support non-symlink colcon installs when their launch content is exact.
    command, paths = _fixture(tmp_path)
    installed_copy = tmp_path / "installed-copy.launch.xml"
    installed_copy.write_bytes(SHADOW_LAUNCH.read_bytes())
    command[command.index("--installed-shadow-launch") + 1] = str(installed_copy)

    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode == 0, completed.stderr
    payload = json.loads(paths["result"].read_text(encoding="utf-8"))
    assert payload["shadow_launch"]["installed_binding"]["symlink_install"] is False


@pytest.mark.parametrize("install_mode", ("symlink", "regular_copy"))
def test_launch_recheck_accepts_the_unchanged_validated_install(
    tmp_path: Path, install_mode: str
) -> None:
    # HH_260906 - Admit unchanged colcon symlink and regular-copy install layouts.
    command, paths = _fixture(tmp_path)
    if install_mode == "regular_copy":
        paths["installed_launch"].unlink()
        paths["installed_launch"].write_bytes(paths["shadow_launch"].read_bytes())
    validated = subprocess.run(command, check=False, capture_output=True, text=True)
    assert validated.returncode == 0, validated.stderr
    binding_sha256 = _sha256(paths["result"])
    initial = json.loads(paths["result"].read_text(encoding="utf-8"))
    recheck_output = tmp_path / "launch-recheck.json"

    completed = subprocess.run(
        _launch_recheck_command(paths, recheck_output),
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    payload = json.loads(recheck_output.read_text(encoding="utf-8"))
    assert set(payload) == {
        "schema_version",
        "status",
        "checked_at",
        "check_stage",
        "trial_binding_file",
        "trial_binding_sha256",
        "source_launch",
        "installed_binding",
        "runtime_shadow_launch",
        "matches_initial_binding",
    }
    assert payload["schema_version"] == 1
    assert payload["status"] == "PASS"
    assert payload["check_stage"] == "immediately_before_ros_launch"
    assert Path(payload["trial_binding_file"]) == paths["result"].resolve()
    assert payload["trial_binding_sha256"] == binding_sha256
    assert payload["source_launch"]["sha256"] == initial["shadow_launch"]["sha256"]
    assert (
        payload["installed_binding"]
        == initial["shadow_launch"]["installed_binding"]
    )
    runtime_launch = paths["shadow_launch_snapshot"]
    runtime_binding = payload["runtime_shadow_launch"]
    assert runtime_launch.is_file()
    assert not runtime_launch.is_symlink()
    assert runtime_launch.stat().st_mode & 0o777 == 0o444
    assert runtime_launch.read_bytes() == paths["shadow_launch"].read_bytes()
    assert runtime_binding == initial["shadow_launch"]["snapshot_binding"]
    assert runtime_binding["file"] == str(runtime_launch.resolve())
    assert runtime_binding["sha256"] == _sha256(runtime_launch)
    assert runtime_binding["sha256"] == initial["shadow_launch"]["sha256"]
    assert payload["matches_initial_binding"] is True


def test_launch_recheck_rejects_a_mutated_trial_binding(tmp_path: Path) -> None:
    # HH_260906 - Bind the pre-launch check to the exact initial trial record bytes.
    command, paths = _fixture(tmp_path)
    validated = subprocess.run(command, check=False, capture_output=True, text=True)
    assert validated.returncode == 0, validated.stderr
    initial_binding_sha256 = _sha256(paths["result"])
    binding = json.loads(paths["result"].read_text(encoding="utf-8"))
    binding["status"] = "FAIL"
    _write_json(paths["result"], binding)
    recheck_output = tmp_path / "launch-recheck.json"

    completed = subprocess.run(
        _launch_recheck_command(
            paths,
            recheck_output,
            binding_sha256=initial_binding_sha256,
        ),
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert "trial binding SHA-256 mismatch" in completed.stderr
    assert not recheck_output.exists()


def test_launch_recheck_rejects_an_unmatched_binding_digest(tmp_path: Path) -> None:
    # HH_260906 - Reject a caller digest that does not bind the unchanged trial record.
    command, paths = _fixture(tmp_path)
    validated = subprocess.run(command, check=False, capture_output=True, text=True)
    assert validated.returncode == 0, validated.stderr
    recheck_output = tmp_path / "launch-recheck.json"

    completed = subprocess.run(
        _launch_recheck_command(
            paths,
            recheck_output,
            binding_sha256="0" * 64,
        ),
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert "trial binding SHA-256 mismatch" in completed.stderr
    assert not recheck_output.exists()


def test_launch_recheck_refuses_to_overwrite_evidence(tmp_path: Path) -> None:
    # HH_260906 - Preserve an existing pre-launch record instead of replacing evidence.
    command, paths = _fixture(tmp_path)
    validated = subprocess.run(command, check=False, capture_output=True, text=True)
    assert validated.returncode == 0, validated.stderr
    recheck_output = tmp_path / "launch-recheck.json"
    sentinel = b"existing evidence\n"
    recheck_output.write_bytes(sentinel)
    runtime_snapshot = paths["shadow_launch_snapshot"].read_bytes()

    completed = subprocess.run(
        _launch_recheck_command(paths, recheck_output),
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert "output already exists" in completed.stderr
    assert recheck_output.read_bytes() == sentinel
    assert paths["shadow_launch_snapshot"].read_bytes() == runtime_snapshot


def test_initial_validator_refuses_to_overwrite_runtime_snapshot(tmp_path: Path) -> None:
    # HH_260906 - Preserve an existing runtime launch instead of replacing executable evidence.
    command, paths = _fixture(tmp_path)
    runtime_launch = paths["shadow_launch_snapshot"]
    runtime_launch.parent.mkdir(parents=True)
    sentinel = b"existing runtime launch evidence\n"
    runtime_launch.write_bytes(sentinel)

    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert "already exists" in completed.stderr
    assert runtime_launch.read_bytes() == sentinel
    assert not paths["result"].exists()


@pytest.mark.parametrize(
    "mutation, expected",
    (
        (
            "snapshot_changed",
            "SHA-256",
        ),
        (
            "snapshot_mode_changed",
            "mode",
        ),
        (
            "snapshot_replaced_by_symlink",
            "regular non-symlink",
        ),
        (
            "snapshot_missing",
            "snapshot is missing",
        ),
        (
            "snapshot_identical_alternate_path",
            "path",
        ),
    ),
)
def test_launch_recheck_rejects_runtime_snapshot_mutation(
    tmp_path: Path, mutation: str, expected: str
) -> None:
    # HH_260906 - Reject runtime snapshot changes made after the immutable binding was written.
    command, paths = _fixture(tmp_path)
    validated = subprocess.run(command, check=False, capture_output=True, text=True)
    assert validated.returncode == 0, validated.stderr

    snapshot = paths["shadow_launch_snapshot"]
    recheck_snapshot = snapshot
    if mutation == "snapshot_changed":
        snapshot.chmod(0o644)
        snapshot.write_bytes(snapshot.read_bytes() + b"\n")
        snapshot.chmod(0o444)
    elif mutation == "snapshot_mode_changed":
        snapshot.chmod(0o644)
    elif mutation == "snapshot_replaced_by_symlink":
        snapshot.unlink()
        snapshot.symlink_to(paths["shadow_launch"])
    elif mutation == "snapshot_missing":
        snapshot.unlink()
    elif mutation == "snapshot_identical_alternate_path":
        recheck_snapshot = tmp_path / "identical-runtime-shadow.launch.xml"
        recheck_snapshot.write_bytes(paths["shadow_launch"].read_bytes())
        recheck_snapshot.chmod(0o444)
    else:
        raise AssertionError(f"unhandled mutation: {mutation}")

    recheck_output = tmp_path / "launch-recheck.json"
    recheck_command = _launch_recheck_command(
        paths,
        recheck_output,
        shadow_launch_snapshot=recheck_snapshot,
    )

    completed = subprocess.run(
        recheck_command, check=False, capture_output=True, text=True
    )

    assert completed.returncode != 0
    assert expected in completed.stderr
    assert not recheck_output.exists()


def test_trial_wrapper_has_owned_shadow_only_lifecycle_and_provenance() -> None:
    source = RECORDED_TRIAL.read_text(encoding="utf-8")

    assert "--portable-shadow-10hz" in source
    assert 'stack_command+=(--portable-shadow-10hz)' in source
    assert "sensor_mapping_portable_e2e_10hz.yaml" in source
    assert "validate_portable_shadow_trial.py" in source
    assert 'cyclonedds_config="$(realpath -- "${cyclonedds_config}")"' in source
    assert '--carla-probe "${output_dir}/carla_preflight_health.json"' in source
    assert 'portable_shadow_route_sha256="$(sha256sum' in source
    assert '"${portable_shadow_cpu_command[@]}" "${portable_shadow_command[@]}"' in source
    assert 'portable_shadow_pgid="${portable_shadow_pid}"' in source
    assert '"declared_map_id:=${portable_shadow_declared_map_id}"' in source
    assert '"route_sha256:=${portable_shadow_route_sha256}"' in source
    assert '"input_settle_timeout_s:=0.05"' in source
    assert '"maximum_tf_translation_error_m:=0.005"' in source
    assert '"maximum_tf_rotation_error_rad:=0.005"' in source
    assert '"research_acknowledged:=true"' in source
    assert '"/portable_e2e_shadow/arm_measurement"' in source
    assert '"/portable_e2e_shadow/capture_startup_boundary"' in source
    assert '"/portable_e2e_shadow/seal_measurement"' in source
    assert "measurement service is absent or duplicated" in source
    # HH_260906 - Keep startup evidence bound to the complete current geometry gate.
    assert (
        "from portable_e2e.runtime_contract import RUNTIME_GATE_ID, RuntimeGateConfig"
        in source
    )
    assert "expected_runtime_gate = asdict(RuntimeGateConfig())" in source
    assert '"runtime_gate_id": RUNTIME_GATE_ID' in source
    assert '"runtime_gate": expected_runtime_gate' in source
    assert "prove_portable_shadow_recorder_subscriptions" in source
    assert 'full_name(endpoint) == "/rosbag2_recorder"' in source
    assert "PORTABLE_SHADOW_RECORDER_SUBSCRIPTIONS_VERIFIED_BEFORE_ARM=true" in source
    assert 'final.get("measurement_sealed") is not True' in source
    assert 'final_bundle_counters.get("pending_bundle_count") != 0' in source
    assert "Portable E2E sealed boundary retains a pending camera bundle" in source
    assert "PORTABLE_SHADOW_STOPPED_BEFORE_RECORDER=true" in source
    assert 'pending_message = "cannot seal while a camera bundle remains pending"' in source
    assert "retry_interval_s = 0.025" in source
    assert "deadline = started + deadline_s" in source
    assert "timeout_sec=min(0.25, remaining)" in source
    assert 'if response.message != pending_message:' in source
    assert 'require_process(shadow_identity, "shadow")' in source
    assert 'require_process(recorder_identity, "recorder")' in source
    assert 'require_process(carla_identity, "CARLA")' in source
    assert 'graph_names.count("/portable_e2e_shadow")' in source
    assert 'graph_names.count("/rosbag2_recorder")' in source
    assert "Portable E2E aligned route changed during seal" in source
    assert "seal_receipt.json" in source
    seal_helper = source[source.index("seal_portable_shadow_measurement()") :]
    success_check = seal_helper.index("if response.success is True:")
    status_write = seal_helper.index("final_yaml = yaml.safe_dump(")
    assert success_check < status_write
    # HH_260906 - Preserve bounded initial discovery and strict post-discovery liveness.
    graph_helpers = seal_helper[
        seal_helper.index("def require_owned_inputs():") :
        seal_helper.index("def stage_bytes(path, payload):")
    ]
    assert "graph_discovery_deadline = min(deadline - 1.0, started + 1.5)" in source
    assert "service_discovery_deadline = deadline - 0.5" in source
    assert "if graph_discovery_complete:" in graph_helpers
    assert "shadow_count != 1 or recorder_count != 1" in graph_helpers
    assert "if shadow_count > 1:" in graph_helpers
    assert "if recorder_count > 1:" in graph_helpers
    retry_body = graph_helpers[graph_helpers.index("else:") :]
    assert "while True:" in retry_body
    assert "require_owned_inputs()" in retry_body
    assert "remaining = graph_discovery_deadline - time.monotonic()" in retry_body
    assert "min(0.05, remaining)" in retry_body
    assert "shadow_count={shadow_count} recorder_count={recorder_count}" in retry_body
    assert retry_body.index("require_owned_inputs()") < retry_body.index(
        "observe_graph("
    )
    assert graph_helpers.index("graph_discovery_complete = True") < (
        graph_helpers.index("liveness_check_count += 1")
    )
    service_discovery = seal_helper[
        seal_helper.index("client = node.create_client(Trigger, service_name)") :
        success_check
    ]
    assert "while True:" in service_discovery
    assert "require_liveness(node)" in service_discovery
    assert "remaining = service_discovery_deadline - time.monotonic()" in (
        service_discovery
    )
    assert "timeout_sec=min(0.05, remaining)" in service_discovery
    assert service_discovery.index("require_liveness(node)") < (
        service_discovery.index("client.wait_for_service(")
    )
    success_body = seal_helper[success_check:status_write]
    post_success_liveness = success_body.index("require_liveness(node)")
    assert success_body.index(
        "if time.monotonic() >= deadline:", post_success_liveness
    ) < success_body.index("break", post_success_liveness)
    artifact_body = seal_helper[status_write:]
    pre_stage_deadline = artifact_body.index(
        'raise SystemExit("Portable E2E seal artifact staging missed its deadline")'
    )
    staged_files = artifact_body.index("final_temporary = stage_bytes(")
    post_stage_deadline = artifact_body.index(
        'raise SystemExit("Portable E2E seal artifact staging exceeded its deadline")'
    )
    final_link = artifact_body.index("os.link(final_temporary, target)")
    assert pre_stage_deadline < staged_files < post_stage_deadline < final_link
    assert 'receipt.unlink(missing_ok=True)' in artifact_body
    assert 'target.unlink(missing_ok=True)' in artifact_body
    cleanup_body = source[source.index("cleanup() {") : source.index("on_signal() {")]
    assert cleanup_body.index('"${portable_shadow_pgid}"') < cleanup_body.index(
        '"${recorder_pgid}"'
    )
    assert '"portable_e2e_exact_bundle_10hz_v2"' in source
    assert 'expected_thresholds["minimum_camera_wall_rate_hz"] = 9.0' in source
    assert 'expected_thresholds["minimum_complete_bundle_count"] = 70' in source
    assert "Refusing to overlap owned Portable E2E shadow generations" in source
    assert "Portable E2E shadow trials reject node identity, topic, and control remaps" in source
    assert 'set(publishers) != allowed_publishers' in source
    assert 'topic == "/planning/trajectory"' in source
    assert 'binding.get("execution_mode") != "shadow_only"' in source
    assert 'route_binding.get("observed_map_id") != declared_map_id' in source
    assert '"autoware-e2e.portable-shadow-status.v3"' in source
    assert "portable_shadow_window_started_seconds" in source
    assert "capture_portable_shadow_healthy_heartbeat" in source
    # HH_260906 - Require an atomic node-owned startup boundary without CLI observation loss.
    heartbeat_start = source.index("capture_portable_shadow_healthy_heartbeat()")
    heartbeat_end = source.index("start_portable_shadow()", heartbeat_start)
    heartbeat_helper = source[heartbeat_start:heartbeat_end]
    assert "call_portable_shadow_status_service" in heartbeat_helper
    assert "/portable_e2e_shadow/capture_startup_boundary" in heartbeat_helper
    assert '"${target}" "${boundary_name}_boundary"' in heartbeat_helper
    assert "ros2 topic echo" not in heartbeat_helper
    assert 'status.get("healthy_now") is not True' in source
    assert "armed_status.yaml" in source
    assert "capture_portable_shadow_window_final" in source
    assert "--require-ten-hz-pass" in source
    assert '"schema_id": "autoware-e2e.portable-shadow-window-boundaries.v2"' in source
    assert "--window-boundaries-json" in source
    assert "analyze_portable_e2e_shadow.py" in source
    assert "shadow_evidence_analysis.json" in source
    assert "portable_shadow_provenance/SHA256SUMS" in source
    # HH_260906 - Keep the launch recheck ahead of ROS startup and linked into final evidence.
    assert 'python3 "${portable_shadow_validator}" recheck-launch' in source
    assert '--binding-sha256 "${portable_shadow_binding_sha256}"' in source
    assert 'PORTABLE_SHADOW_LAUNCH_RECHECK_FILE=%s' in source
    assert 'PORTABLE_SHADOW_LAUNCH_RECHECK_SHA256=%s' in source
    assert '"launch_recheck_sha256": launch_recheck_sha256' in source
    assert "launch_recheck.json" in source
    assert "launch_recheck.log" in source
    assert source.count('"/planning/portable_e2e/shadow_path"') >= 3
    assert source.count('"/planning/portable_e2e/shadow_trajectory"') >= 3
    assert "e2e_stop_owned_process_group \\" in source
    assert '"${portable_shadow_pgid}" "${portable_shadow_pid}" 30 5 2' in source
    assert source.index("route_file=\"${output_dir}/aligned_route.json\"") < source.index(
        '"${portable_shadow_cpu_command[@]}" "${portable_shadow_command[@]}"'
    )
    assert source.index('portable_shadow_route_sha256="$(sha256sum') < source.index(
        '"${portable_shadow_cpu_command[@]}" "${portable_shadow_command[@]}"'
    )
    recheck_index = source.index(
        'python3 "${portable_shadow_validator}" recheck-launch'
    )
    runtime_link_index = source.index(
        "PORTABLE_SHADOW_LAUNCH_RECHECK_FILE=%s", recheck_index
    )
    command_index = source.index("portable_shadow_command=(", recheck_index)
    spawn_index = source.index(
        '"${portable_shadow_cpu_command[@]}" "${portable_shadow_command[@]}"',
        command_index,
    )
    assert recheck_index < runtime_link_index < command_index < spawn_index
    manifest_index = source.index("portable_shadow_manifest_files=(")
    manifest_end = source.index("  )", manifest_index)
    manifest_body = source[manifest_index:manifest_end]
    assert "launch_recheck.json" in manifest_body
    assert "launch_recheck.log" in manifest_body


def test_trial_wrapper_executes_the_bound_runtime_launch_snapshot() -> None:
    # HH_260906 - Launch ROS from the immutable snapshot and bind it through every evidence layer.
    source = RECORDED_TRIAL.read_text(encoding="utf-8")

    assert (
        'portable_shadow_runtime_launch="${output_dir}/portable_shadow_provenance/'
        'runtime_shadow.launch.xml"' in source
    )
    assert source.count(
        '--shadow-launch-snapshot "${portable_shadow_runtime_launch}"'
    ) >= 2
    assert "PORTABLE_SHADOW_RUNTIME_LAUNCH_FILE=%s" in source
    assert "PORTABLE_SHADOW_RUNTIME_LAUNCH_SHA256=%s" in source
    assert "Portable E2E runtime shadow launch changed before startup validation" in source
    assert '"runtime_shadow_launch_sha256": runtime_shadow_launch_sha256' in source

    recheck_index = source.index(
        'python3 "${portable_shadow_validator}" recheck-launch'
    )
    recheck_snapshot_index = source.index(
        '--shadow-launch-snapshot "${portable_shadow_runtime_launch}"',
        recheck_index,
    )
    command_index = source.index("portable_shadow_command=(", recheck_snapshot_index)
    direct_launch_index = source.index(
        'ros2 launch "${portable_shadow_runtime_launch}"', command_index
    )
    spawn_index = source.index(
        '"${portable_shadow_cpu_command[@]}" "${portable_shadow_command[@]}"',
        direct_launch_index,
    )
    assert recheck_index < recheck_snapshot_index < command_index
    assert command_index < direct_launch_index < spawn_index
    command_body = source[command_index:spawn_index]
    assert "ros2 launch autoware_e2e_vad_launch portable_e2e_shadow.launch.xml" not in (
        command_body
    )

    manifest_index = source.index("portable_shadow_manifest_files=(")
    manifest_end = source.index("  )", manifest_index)
    manifest_body = source[manifest_index:manifest_end]
    assert "runtime_shadow.launch.xml" in manifest_body


def test_owned_summary_binds_the_exact_runtime_launch_snapshot() -> None:
    # HH_260906 - Carry the executed snapshot identity into attempt and selected-run summaries.
    source = OWNED_TRIAL.read_text(encoding="utf-8")

    assert 'launch_recheck.get("runtime_shadow_launch")' in source
    assert '"runtime_shadow_launch":' in source
    assert "selected Portable shadow attempt lacks an exact runtime launch snapshot" in source


def test_fast_runner_selects_portable_mapping_without_changing_legacy_profile() -> None:
    source = FAST_RUNNER.read_text(encoding="utf-8")

    assert "--portable-shadow-10hz" in source
    assert 'fast_mapping="${package_share}/config/sensor_mapping_portable_e2e_10hz.yaml"' in source
    assert "--portable-shadow-10hz requires --speed-30kph" in source
    assert "--portable-shadow-10hz and --camera-source-5hz are mutually exclusive" in source
    assert "--portable-shadow-10hz and --sensor-mapping are mutually exclusive" in source
    assert "vad_carla_tiny_camera_source_5hz_best_effort_image_depth1.param.yaml" in source
    assert (
        'fast_mapping="${package_share}/config/sensor_mapping_vad_fast_reliable_imu.yaml"'
        in source
    )
    assert (
        "sensor_mapping_vad_fast_imu_camera_source_5hz_best_effort_image_depth1.yaml"
        in source
    )


def test_runtime_health_probe_admits_exact_portable_transport_profile() -> None:
    source = RUNTIME_HEALTH_PROBE.read_text(encoding="utf-8")

    assert (
        'CAMERA_TRANSPORT_PROFILE_PORTABLE_10HZ = '
        '"portable_e2e_exact_bundle_10hz_v2"' in source
    )
    assert "PORTABLE_MINIMUM_CAMERA_WALL_RATE_HZ = 9.0" in source
    assert "PORTABLE_MINIMUM_COMPLETE_BUNDLES = 70" in source
    assert "EXACT_CAMERA_GRAPH_PROFILES" in source
    assert '"camera_source_sensor_tick_seconds": 0.1' in source
    assert '"bridge_publish_cap_hz": 11' in source
    assert '"declared_effective_camera_rate_hz": 10.0' in source


@pytest.mark.parametrize(
    "command, expected",
    (
        (
            [str(RECORDED_TRIAL), "--portable-runtime-bundle", "unused"],
            "require --portable-shadow-10hz",
        ),
        ([str(RECORDED_TRIAL), "--portable-shadow-10hz"], "requires --speed-30kph"),
        (
            [
                str(RECORDED_TRIAL),
                "--portable-shadow-10hz",
                "--portable-shadow-10hz",
            ],
            "may be specified only once",
        ),
        (
            [
                str(RECORDED_TRIAL),
                "--portable-shadow-10hz",
                "--speed-30kph",
                "--camera-source-5hz",
            ],
            "mutually exclusive",
        ),
        ([str(FAST_RUNNER), "--portable-shadow-10hz"], "requires --speed-30kph"),
        (
            [
                str(FAST_RUNNER),
                "--portable-shadow-10hz",
                "--portable-shadow-10hz",
            ],
            "may be specified only once",
        ),
        (
            [
                str(FAST_RUNNER),
                "--portable-shadow-10hz",
                "--speed-30kph",
                "--camera-source-5hz",
            ],
            "mutually exclusive",
        ),
    ),
)
def test_portable_profile_rejects_incomplete_duplicate_or_conflicting_options(
    command: list[str], expected: str
) -> None:
    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode == 2
    assert expected in completed.stderr


def test_recorded_trial_accepts_single_available_portable_cpu_set_option(
    tmp_path: Path,
) -> None:
    # HH_260906 - Prove one four-core policy option passes validation before route preflight.
    cpu_set = ",".join(str(value) for value in _available_portable_cpu_ids())

    completed = subprocess.run(
        _portable_cpu_profile_command(tmp_path, cpu_set),
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "Route file not found" in completed.stderr
    assert "--portable-cpu-set" not in completed.stderr


@pytest.mark.parametrize(
    "cpu_set",
    ("0-1", "0,,1", ",0", "0,", " 0", "0 ", "cpu0", "-1"),
)
def test_recorded_trial_rejects_malformed_portable_cpu_set(
    tmp_path: Path, cpu_set: str
) -> None:
    # HH_260906 - Reject every CPU list that is not comma-separated unsigned CPU IDs.
    completed = subprocess.run(
        _portable_cpu_profile_command(tmp_path, cpu_set),
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "must be a comma-separated list of CPU IDs" in completed.stderr


def test_recorded_trial_rejects_duplicate_portable_cpu_ids(tmp_path: Path) -> None:
    # HH_260906 - Reject duplicates independently of the four-unique-CPU minimum.
    available = _available_portable_cpu_ids()
    duplicated = ",".join(str(value) for value in (*available, available[0]))

    completed = subprocess.run(
        _portable_cpu_profile_command(tmp_path, duplicated),
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "must not contain duplicate CPU IDs" in completed.stderr


def test_recorded_trial_rejects_unavailable_portable_cpu_id(tmp_path: Path) -> None:
    # HH_260906 - Reject an unavailable ID independently of the four-CPU minimum.
    available = _available_portable_cpu_ids()
    unavailable_cpu = max(os.sched_getaffinity(0)) + 100_000
    cpu_set = ",".join(str(value) for value in (*available, unavailable_cpu))

    completed = subprocess.run(
        _portable_cpu_profile_command(tmp_path, cpu_set),
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "contains unavailable CPU IDs" in completed.stderr


@pytest.mark.parametrize("cpu_count", (1, 2, 3))
def test_recorded_trial_rejects_portable_cpu_set_smaller_than_thread_pool(
    tmp_path: Path, cpu_count: int
) -> None:
    # HH_260906 - Prevent a four-thread inference pool from oversubscribing its CPU set.
    cpu_set = ",".join(
        str(value) for value in _available_portable_cpu_ids()[:cpu_count]
    )

    completed = subprocess.run(
        _portable_cpu_profile_command(tmp_path, cpu_set),
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "at least four CPU IDs" in completed.stderr


def test_recorded_trial_rejects_duplicate_portable_cpu_set_option(
    tmp_path: Path,
) -> None:
    # HH_260906 - Permit the CPU policy option exactly once per owned shadow process.
    cpu_set = ",".join(str(value) for value in _available_portable_cpu_ids())

    completed = subprocess.run(
        _portable_cpu_profile_command(tmp_path, cpu_set, cpu_set),
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert (
        "Portable E2E option may be specified only once: --portable-cpu-set"
        in completed.stderr
    )


def test_recorded_trial_rejects_portable_cpu_set_for_cuda(tmp_path: Path) -> None:
    # HH_260906 - Keep taskset and CPU library policy out of UUID-pinned CUDA trials.
    cpu_set = ",".join(str(value) for value in _available_portable_cpu_ids())
    environment = {**os.environ, "CUDA_VISIBLE_DEVICES": "GPU-test-fixture"}

    completed = subprocess.run(
        _portable_cpu_profile_command(tmp_path, cpu_set, device="cuda:0"),
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert completed.returncode == 2
    assert "valid only with --portable-device cpu" in completed.stderr


def test_trial_wrapper_applies_and_records_portable_cpu_execution_policy() -> None:
    # HH_260906 - Bind CPU affinity and library thread limits into startup and evidence.
    help_result = subprocess.run(
        [str(RECORDED_TRIAL), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    source = RECORDED_TRIAL.read_text(encoding="utf-8")

    assert help_result.returncode == 0
    assert "--portable-cpu-set LIST" in help_result.stderr
    assert 'portable_cpu_set=""' in source
    assert 'claim_portable_option "$1"' in source[
        source.index("--portable-cpu-set)") : source.index(
            "--control-ab-pid-i40)", source.index("--portable-cpu-set)")
        )
    ]

    start_index = source.index("start_portable_shadow()")
    recheck_index = source.index(
        'python3 "${portable_shadow_validator}" recheck-launch', start_index
    )
    cpu_branch_index = source.index(
        'if [[ "${portable_shadow_device}" == "cpu" ]]', recheck_index
    )
    spawn_index = source.index(
        '"${portable_shadow_cpu_command[@]}" "${portable_shadow_command[@]}"',
        cpu_branch_index,
    )
    cpu_branch = source[cpu_branch_index:spawn_index]
    assert "portable_shadow_cpu_command=(setsid)" in cpu_branch
    assert 'taskset --cpu-list "${portable_cpu_set}"' in cpu_branch
    assert "CUDA_VISIBLE_DEVICES=''" in cpu_branch
    assert "OMP_NUM_THREADS=4" in cpu_branch
    assert "MKL_NUM_THREADS=4" in cpu_branch
    assert "OPENBLAS_NUM_THREADS=1" in cpu_branch
    assert "NUMEXPR_NUM_THREADS=1" in cpu_branch
    assert recheck_index < cpu_branch_index < spawn_index

    assert 'portable_cpu_affinity_label="taskset_cpu_list"' in source
    assert "PORTABLE_SHADOW_CPU_SET=%s\\n" in source
    assert "PORTABLE_SHADOW_CPU_AFFINITY=%s\\n" in source
    assert "PORTABLE_SHADOW_OMP_NUM_THREADS=4\\n" in source
    assert "PORTABLE_SHADOW_MKL_NUM_THREADS=4\\n" in source
    assert "PORTABLE_SHADOW_OPENBLAS_NUM_THREADS=1\\n" in source
    assert "PORTABLE_SHADOW_NUMEXPR_NUM_THREADS=1\\n" in source


def test_owned_runner_documents_forwarded_portable_profile() -> None:
    completed = subprocess.run(
        [str(OWNED_TRIAL), "--help"], check=False, capture_output=True, text=True
    )
    source = OWNED_TRIAL.read_text(encoding="utf-8")

    assert completed.returncode == 0
    assert "--portable-shadow-10hz" in completed.stderr
    assert "pinned Portable input" in completed.stderr
    assert "--portable-cpu-set" in completed.stderr
    assert 'portable_requested = "--portable-shadow-10hz" in option_values' in source
    assert '"attempt_bindings": portable_bindings' in source
    assert "selected Portable shadow attempt lacks an exact safe binding" in source
    assert 'selected_bindings[0]["declared_map_id"]' in source
    assert 'selected_bindings[0]["observed_map_id"]' in source


@pytest.mark.parametrize(
    "remap",
    (
        "__ns:=/unsafe",
        "shadow:=/planning/trajectory",
        "shadow:=/control/command/control_cmd",
        "shadow:=/vehicle/command/actuation_cmd",
    ),
)
def test_recorded_trial_rejects_shadow_identity_and_control_remaps_before_ros(
    tmp_path: Path, remap: str
) -> None:
    route = tmp_path / "route.json"
    _write_json(
        route,
        {
            "town": "Town07",
            "scenario": "straight",
            "coordinate_reference": "base_link",
        },
    )
    command = [
        str(RECORDED_TRIAL),
        "--portable-shadow-10hz",
        "--speed-30kph",
        "--portable-runtime-bundle",
        "unused.npz",
        "--portable-runtime-bundle-sha256",
        SHA,
        "--portable-source-checkpoint-sha256",
        SHA,
        "--portable-model-config-sha256",
        SHA,
        "--portable-corpus-fingerprint-sha256",
        SHA,
        "--portable-rig-file",
        "unused-rig.json",
        "--portable-rig-sha256",
        SHA,
        "--portable-contract-file",
        "unused-contract.json",
        "--portable-contract-sha256",
        SHA,
        str(tmp_path / "output"),
        str(route),
        remap,
    ]

    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode == 2
    assert "reject node identity, topic, and control remaps" in completed.stderr
    assert not (tmp_path / "output").exists()


def test_turn_recorder_captures_exact_isolated_shadow_topic_set() -> None:
    source = RECORDER.read_text(encoding="utf-8")
    expected = {
        "planning/portable_e2e/status",
        "planning/portable_e2e/latency_ms",
        "planning/portable_e2e/selected_candidate",
        "planning/portable_e2e/shadow_path",
        "planning/portable_e2e/shadow_trajectory",
    }

    assert {topic for topic in expected if f"topic_regex+='{topic}|'" in source} == expected
    assert "planning/portable_e2e/.*" not in source
    assert "planning/portable_e2e/|" not in source


def test_validator_rejects_symlinked_sensor_mapping(tmp_path: Path) -> None:
    command, paths = _fixture(tmp_path)
    mapping_link = tmp_path / "mapping-link.yaml"
    mapping_link.symlink_to(PORTABLE_MAPPING)
    command[command.index("--sensor-mapping") + 1] = str(mapping_link)

    completed = subprocess.run(command, check=False, capture_output=True, text=True)

    assert completed.returncode != 0
    assert "regular non-symlink" in completed.stderr
    assert not paths["result"].exists()
