"""HH_261004 - Own one fresh V3 rerun after proving the V2 fixture-schema integration failure."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from pathlib import Path

from . import collect_temporal_route_joint_town03_straight_v3_20261004 as live
from . import research_temporal_route_joint_town03_straight_v2_20261004 as previous


base, numeric, ROOT = previous.base, previous.numeric, previous.ROOT
require, digest, read_json = previous.require, previous.digest, previous.read_json
MODULE = "scripts.e2e.research_temporal_route_joint_town03_straight_v3_20261004"
SCHEMA = live.OWNER_SCHEMA
V2_OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_straight_fixed45.20261004.v2"
ROUTE, XODR, UMAP = previous.ROUTE, previous.XODR, previous.UMAP
XODR_SHA, UMAP_SHA = previous.XODR_SHA, previous.UMAP_SHA
PARENT_OWNER_SHA = "39e31e66f16413879e85c7cd033c1ef9a1452bab1d14e64ff3f49e1ae746320b"
V2_OUTPUT = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v2"
V2_PLAN = V2_OUTPUT.with_name(V2_OUTPUT.name + "_plan.json")
V2_EVIDENCE_SHA256 = {
    "declaration.json": "440875bc8d4eb2e0f8c60a144f79b878318cab11dfb27dc5aed9fc07af2c6d7a",
    "diagnostic_report.json": "cbf90455a36f4605c37a9e1f6e72d48c4992603582b2139d5aae43caaffced9a",
    "owner_result.json": "cbb751e8106341f4a23c1cfc70dbee9e3c304d8ccf4e54eff9d8fe5ccc35ea62",
    "episode.partial/manifest.json": "67b0d23d92fbed781ed6ac441b01e9349e373a1766567b59424a9381249d991b",
    "collector.log": "8d683c7ff1f7b15593014d3e18adf92889c5925ecbc5bac678f5c157ac306be6",
}
V2_PLAN_SHA256 = "440875bc8d4eb2e0f8c60a144f79b878318cab11dfb27dc5aed9fc07af2c6d7a"
FIXTURE_ERROR = "CollectionError: fixture is restricted to the declared original Town03 owner"
_parent_capture = previous.capture_argv
_parent_fixed = previous.fixed_contract
_parent_sources = previous.runtime_sources
_parent_diagnostic = previous.diagnostic_report
_active = False


def paths(run_id):
    require(run_id == "v3", "fresh fixture-schema Town03 straight rerun requires exact v3")
    output = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v3"
    return output, output.with_name(output.name + "_plan.json")


def capture_argv(run_id, longitudinal_pi, declaration_sha=base.PLAN_TOKEN):
    """HH_261004 - Reuse the exact V2 CLI while replacing only fresh owner paths and module identity."""
    require(run_id == "v3" and _active, "v3 capture CLI requires its exclusive owner scope")
    # HH_261004 - The frozen V2 function itself rejects another run ID; its captured V1 CLI is configuration-identical.
    return previous._parent_capture(run_id, longitudinal_pi, declaration_sha)


def _verify_v2_failure():
    """HH_261004 - Authenticate every declared root source and the complete zero-frame V2 failure denominator."""
    require(V2_OUTPUT.is_dir() and not V2_OUTPUT.is_symlink(), "archived v2 failure directory required")
    require(V2_PLAN.is_file() and not V2_PLAN.is_symlink() and digest(V2_PLAN) == V2_PLAN_SHA256,
            "archived v2 plan bytes changed")
    require(all(digest(V2_OUTPUT / name) == sha for name, sha in V2_EVIDENCE_SHA256.items()),
            "archived v2 failure evidence changed")
    declaration = read_json(V2_OUTPUT / "declaration.json")
    plan = read_json(V2_PLAN)
    require((V2_OUTPUT / "declaration.json").read_bytes() == V2_PLAN.read_bytes()
        and declaration == plan and plan.get("schema") == V2_OWNER_SCHEMA
        and plan.get("run_id") == "v2" and plan.get("town") == "Town03"
        and plan.get("scenario") == "straight" and plan.get("original_navigation_sha256") == live.ROUTE_SHA,
        "v2 declared plan identity differs")
    sources = plan.get("source_sha256")
    require(isinstance(sources, dict) and sources
        and all(isinstance(name, str) and not Path(name).is_absolute()
                and digest(ROOT / name) == sha for name, sha in sources.items()),
        "one or more declared v2 root source bytes changed")
    external = plan.get("external_source_sha256")
    require(isinstance(external, dict) and external
        and all(Path(name).is_absolute() and digest(name) == sha for name, sha in external.items()),
        "one or more declared v2 external source bytes changed")
    diagnostic = read_json(V2_OUTPUT / "diagnostic_report.json")
    owner = read_json(V2_OUTPUT / "owner_result.json")
    manifest = read_json(V2_OUTPUT / "episode.partial/manifest.json")
    result = manifest.get("result", {})
    counts = result.get("learned_research_counts", {})
    raw = diagnostic.get("raw_source_sha256")
    require(diagnostic.get("status") == "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
        and diagnostic.get("original_capture_error") == FIXTURE_ERROR
        and diagnostic.get("original_capture_status") == "failed"
        and diagnostic.get("actual_native_state_count") == 0
        and diagnostic.get("actual_camera_bundle_count") == 0
        and diagnostic.get("actual_ACK_receipt_count") == 0
        and diagnostic.get("bounded_capture_complete") is False
        and diagnostic.get("learned_driving_screen_passed") is False
        and isinstance(raw, dict) and len(raw) == 9
        and all(Path(name).is_absolute() and digest(name) == sha for name, sha in raw.items())
        and manifest.get("status") == "failed" and manifest.get("error") == FIXTURE_ERROR
        and result.get("state_count") == result.get("camera_anchor_count") == 0
        and result.get("capture_phases", {}).get("observed_total_ticks") == 0
        and counts == {"learned_commands_acknowledged": 0, "expert_startup_commands": 0,
                       "emergency_commands": 0, "inference_count": 0, "prefill_count": 0}
        and manifest.get("cleanup") == {"completed": True, "server_available": True, "errors": []}
        and owner.get("status") == "CAPTURE_RETURNED_NONZERO_RETAINED"
        and owner.get("worker_exit_code") == 1 and owner.get("cleanup_errors") == []
        and owner.get("source_hashes_match") is True and owner.get("external_source_hashes_match") is True
        and owner.get("declaration_sha256") == V2_PLAN_SHA256,
        "v2 zero-frame fixture failure identity or cleanup differs")
    return diagnostic, owner, manifest


def fixed_contract(run_id, longitudinal_pi):
    """HH_261004 - Change only the stale fixture owner-schema seam in a fresh create-only V3 run."""
    diagnostic, _, _ = _verify_v2_failure()
    result = _parent_fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, output=str(paths(run_id)[0]), declaration_path=str(paths(run_id)[1]),
        experiment="Fresh fixed45 Town03 straight rerun with the actual Green fixture owner schema admitted",
        failed_v2_archived_reference=dict(path=str(V2_OUTPUT), plan_path=str(V2_PLAN),
            sha256=dict(V2_EVIDENCE_SHA256), plan_sha256=V2_PLAN_SHA256,
            status=diagnostic["status"], original_capture_error=FIXTURE_ERROR,
            native_state_count=0, camera_bundle_count=0, ACK_receipt_count=0,
            failure_before_any_frame=True, overwritten=False, promoted=False),
        fixture_owner_schema_admission=dict(
            declared_owner_schema=SCHEMA,
            actual_fixture_module_global_required=True,
            inherited_v2_deferred_recording_unchanged=True,
            inherited_authenticated_start_corridor_unchanged=True,
            model_weights_changed=False, controller_changed=False, policy_changed=False,
            route_or_map_changed=False, camera_or_ACK_changed=False,
            speed_or_acceleration_gate_changed=False, latency_gate_changed=False,
            body_area_or_inset_gate_changed=False),
        attempts=1, automatic_retry=False, full_RTE01_approved=False,
        full_MAIN9_completion_claimed=False, training_data_approved=False,
        vehicle_control_approved=False, model_promoted=False)
    require(result["total_capture_seconds"] == 45 and result["maximum_native_state_count"] == 900
        and result["maximum_camera_bundle_count"] == 450 and result["maximum_driving_native_ticks"] == 876
        and result["model_speed_cap_mps"] == 2.5 and result["native_actor_planar_speed_maximum_mps"] == 2.7
        and result["policy_deadline_seconds"] == 0.1 and result["controller_target_cap_mps"] == 2.4
        and result["speed_preview_s"] == 0.5
        and result["precursor_motion_screen"]["net_learned_progress_minimum_m"] == 90.0
        and result["precursor_motion_screen"]["cte_p95_maximum_m"] == 0.35
        and result["precursor_motion_screen"]["cte_absolute_maximum_m"] == 0.75
        and result["body_clearance_screen"]["lane_inset_m"] == 0.1
        and result["additive_native_speed_rate_guard_mps2"] ==
            {"minimum": -6.0, "maximum": 3.0, "warmup_to_first_driving_interval_included": True}
        and result["actual_deferred_JPEG_after_ACK_required"] is True
        and result["authenticated_start_corridor"]["upstream_lane"] == [67, 0, 2]
        and result["worker_command"][result["worker_command"].index("-m") + 1] == MODULE,
        "V2 model/controller/route/recording/body/latency contract changed")
    return result


def runtime_sources():
    """HH_261004 - Archive the fresh owner/collector in addition to the entire unchanged V2 closure."""
    require(digest(Path(previous.__file__)) == PARENT_OWNER_SHA, "archived Town03 straight v2 owner changed")
    _verify_v2_failure()
    result = _parent_sources()
    live.verify_frozen()
    imported = base.imported_source_closure((Path(__file__).resolve(), Path(live.__file__).resolve()))
    for path in imported:
        name, sha = str(path.relative_to(ROOT)), digest(path)
        require(name not in result or result[name] == sha, "fresh v3 source closure conflict")
        result[name] = sha
    return result


def diagnostic_report(output, plan, owner):
    """HH_261004 - Retain the V2 failure identity as an additional prerequisite, never a success substitute."""
    result = _parent_diagnostic(output, plan, owner)
    result.update(schema=SCHEMA + ".diagnostic_report", archived_v2_fixture_failure_preserved=False)
    try:
        _verify_v2_failure()
        result["archived_v2_fixture_failure_preserved"] = True
    except Exception as error:
        result["v2_failure_archive_error"] = type(error).__name__ + ": " + str(error)
    for key in ("learned_driving_screen_passed", "straight_motion_body_policy_wall100ms_passed"):
        result[key] = bool(result.get(key)) and result["archived_v2_fixture_failure_preserved"]
    result.update(full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    result["status"] = "PASS_KNOWN_DEV_STRAIGHT_PRECURSOR" if result["learned_driving_screen_passed"] \
        else "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
    return result


@contextmanager
def activated(pins):
    """HH_261004 - Scope V3 owner aliases through the frozen V2 ownership/lock/port/map lifecycle."""
    global _active
    require(not _active and digest(Path(previous.__file__)) == PARENT_OWNER_SHA,
            "fresh Town03 straight v3 owner cannot nest or use a changed v2 owner")
    _verify_v2_failure()
    values = dict(paths=paths, capture_argv=capture_argv, fixed_contract=fixed_contract,
        runtime_sources=runtime_sources, diagnostic_report=diagnostic_report,
        live=live, MODULE=MODULE, SCHEMA=SCHEMA)
    saved = {name: getattr(previous, name) for name in values}
    _active = True
    try:
        for name, value in values.items():
            setattr(previous, name, value)
        with previous.activated(pins):
            yield
    finally:
        for name, value in reversed(tuple(saved.items())):
            setattr(previous, name, value)
        _active = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("weights", "metadata", "checkpoint", "declaration"):
        parser.add_argument("--expected-" + name + "-sha256", required=True)
    args, remaining = parser.parse_known_args(argv)
    pins = {name + "_sha256": getattr(args, "expected_" + name + "_sha256")
            for name in ("weights", "metadata", "checkpoint", "declaration")}
    with activated(pins):
        return base.main(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
