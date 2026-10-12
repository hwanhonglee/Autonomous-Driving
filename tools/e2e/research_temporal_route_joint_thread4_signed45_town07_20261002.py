"""HH_261002 - Prospectively test four CPU threads without relaxing signed45 physical or latency gates."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import re

import torch

from . import research_temporal_route_joint_signed45_town07_20261002 as previous
from . import collect_temporal_route_joint_thread4_signed45_20261002 as live

ROOT, base, parent = previous.ROOT, previous.base, previous.parent
MODULE = "scripts.e2e.research_temporal_route_joint_thread4_signed45_town07_20261002"
SCHEMA = "portable_e2e.actual_route_joint_thread4_signed45_three_arms.20261002.v1"
FROZEN_OWNER_SHA = "6926855c4eb66e8235b343a2aa8e16f5d6affa86ad20c1342bb6927f2d97f190"
BENCHMARK = ROOT / "storage/experiments/validation/2026-10-02/temporal_route_joint_cpu_threads_v1"
BENCHMARK_PINS = {
    "declaration.json": "0658f3b3a58bc5a808aed4acc2655c0723ca00c3ddc17d7cd5eea724db0cff2a",
    "summary.json": "4a031cf3cb459f94f6f0257134e588f184dd06300fb152eebe33259bf722643d",
    "threads_1.json": "e739e57b2f057842dc2afa54f01ce04031f879e6b2b66d987d172dcfaa63d147",
    "threads_2.json": "3e16a0a402b833fa8ec7b860ab38fa863582d4c44fdd47166b676ac1cab67d98",
    "threads_4.json": "3a14ce19b08a85360ced3f77d7ffbdf9e0ecacfc1b5e98fdf6c9299e06c04c60",
}
require, digest, read_json = base.require, base.digest, base.read_json
_fixed, _sources, _load, _diagnostic, _summary = previous.fixed_contract, previous.runtime_sources, previous.load_model, previous.diagnostic_report, previous.batch_summary
_environment = base.environment
_disk = parent.disk_check
_active, _evidence_assets = False, None


def batch_paths(run_id):
    require(isinstance(run_id, str) and re.fullmatch(r"v[1-9][0-9]*", run_id), "fresh thread4 research version required")
    output = ROOT / ("storage/experiments/validation/2026-10-02/temporal_route_joint_thread4_signed45_town07_" + run_id)
    return output, output.with_name(output.name + "_plan.json")


def evidence_assets(pins):
    """HH_261002 - Preserve all three offline outcomes, actual selected inputs and their sealed source copies."""
    assets = {BENCHMARK / name: sha for name, sha in BENCHMARK_PINS.items()}
    require(all(digest(path) == sha for path, sha in assets.items()), "fixed benchmark receipts changed")
    declaration, summary = read_json(BENCHMARK / "declaration.json"), read_json(BENCHMARK / "summary.json")
    require(declaration["thread_arms"] == [1, 2, 4] and declaration["interop_threads"] == 1
        and declaration["fixed_camera_ordinals"] == list(range(55, 183))
        and declaration["input_spec"]["final_endpoint"] == pins, "thread benchmark endpoint/window differs")
    require(summary["all_raw_decoded_selected_proposed_control_equivalent"] is False
        and summary["all_exact_normalized_inputs_equal"] is True
        and summary["all_failures_retained"] is True and summary["all_source_and_archive_pins_unchanged"] is True
        and summary["actual_physical_latency_or_driving_pass"] is False,
        "original strict-parity failure and offline scope must remain explicit")
    require([row["threads"] for row in summary["attempts"]] == [1, 2, 4]
        and all(row["returncode"] == 0 and row["result_sha256"] == BENCHMARK_PINS["threads_%d.json" % row["threads"]]
            for row in summary["attempts"]), "all three original arm outcomes required")
    for field in ("source_and_asset_sha256", "archived_source_and_asset_sha256"):
        require(isinstance(declaration[field], dict) and declaration[field], "complete sealed benchmark source assets required")
        for name, sha in declaration[field].items():
            path = Path(name)
            require(path.is_absolute() and path.resolve() == path and path.is_relative_to(ROOT) and not path.is_symlink()
                and re.fullmatch(r"[0-9a-f]{64}", sha) and digest(path) == sha, "benchmark source/archive identity changed")
            require(path not in assets or assets[path] == sha, "conflicting benchmark identity")
            assets[path] = sha
    return assets


def numerical_preset():
    return dict(schema=live.THREAD_SCHEMA, intra_op_threads=4, inter_op_threads=1,
        strict_original_offline_parity_passed=False, numerical_bitwise_equivalence_claimed=False,
        all_128_normalized_inputs_identical=True, raw_outputs_all_arms_atol_rtol_1e_6_passed=True,
        decoded_and_selected_atol_rtol_1e_6_passed=False, thread4_proposed_control_atol_rtol_1e_6_passed=True,
        observed_maximum_decoded_XY_difference_m=3.4570693969726562e-6,
        observed_maximum_decoded_speed_difference_mps=1.9311904907226562e-5,
        differences_are_observations_not_replacement_or_relaxed_tolerances=True,
        JPEG_reconstructed_BGRA_not_original_lossless_pixels=True,
        offline_warmed_windows_not_continuous_actuation=True,
        unchanged_hard_policy_deadline_seconds=.1, unchanged_bundle_delivery_to_ACK_deadline_seconds=.1,
        runtime_settings_readback_before_all_normal_proposals=True,
        no_automatic_promotion=True, full_MAIN9_completion_claimed=False,
        benchmark_sha256=dict(BENCHMARK_PINS))


def fixed_contract(run_id, longitudinal_pi):
    result = _fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, experiment="Fixed final6000 numerically distinct4-thread signed45 research preset",
        cpu_thread_numerical_preset=numerical_preset(), benchmark_assets_sha256={str(p):s for p,s in _evidence_assets.items()},
        CPU_thread_environment=dict(OMP_NUM_THREADS="4", MKL_NUM_THREADS="4", OPENBLAS_NUM_THREADS="4"),
        numerical_preset_is_not_exact_equivalence_or_model_approval=True)
    result["sealed_benchmark_asset_archive_bytes_per_trial"] = sum(path.stat().st_size for path in _evidence_assets)
    result["additional_three_trial_benchmark_archive_reserve_bytes"] = 3 * result["sealed_benchmark_asset_archive_bytes_per_trial"]
    return result


def runtime_sources():
    require(digest(Path(previous.__file__)) == FROZEN_OWNER_SHA, "frozen joint physical owner changed")
    live.verify_frozen()
    pins = _sources()
    for path in base.imported_source_closure((Path(__file__).resolve(),)):
        name = str(path.relative_to(ROOT)); sha = digest(path)
        require(name not in pins or pins[name] == sha, "new import closure conflicts with frozen source")
        pins[name] = sha
    return pins


def configure_threads():
    # HH_261002 - Inter-op is process-scoped and cannot be undone safely after parallel work; the worker is isolated.
    if torch.get_num_interop_threads() != 1:
        torch.set_num_interop_threads(1)
    torch.set_num_threads(4)
    return live.thread_readback()


def load_model(weights, metadata, **pins):
    # HH_261002 - The inherited preflight/worker sets1; apply the declared4 preset immediately before numeric loading.
    configure_threads()
    model = _load(weights, metadata, **pins)
    live.thread_readback()
    model._research_cpu_thread_numerical_revision = live.THREAD_SCHEMA
    return model


def environment():
    result = _environment()
    result.update(OMP_NUM_THREADS="4", MKL_NUM_THREADS="4", OPENBLAS_NUM_THREADS="4")
    return result


def disk_check(free=None):
    result = _disk(free)
    # HH_261002 - Account for all selected JPEG/input/source proof copies before launching any of the three arms.
    extra = 3 * sum(path.stat().st_size for path in _evidence_assets)
    require(result["free_bytes"] >= result["required_bytes"] + extra, "complete three-trial benchmark proof archive reserve unavailable")
    return dict(result, benchmark_archive_reserve_bytes=extra, required_bytes=result["required_bytes"] + extra)


def readback_report(episode):
    records = parent.jsonl(episode / "cpu_thread_readback.jsonl")
    timing = parent.jsonl(episode / "wall_timing.jsonl")
    expected = [(row["command_frame"], row["command_reason"]) for row in timing
        if row.get("command_reason") in ("initial_drive_control", "next_driving_control") and row.get("command_frame") is not None]
    actual = [(row["command_frame"], row["command_reason"]) for row in records]
    passed = bool(expected) and actual == expected and len(actual) == len(set(actual)) and all(
        row.get("schema") == live.THREAD_SCHEMA and row.get("intra_op_threads") == 4
        and row.get("inter_op_threads") == 1 and row.get("before_original_propose") is True for row in records)
    return dict(status="PASS_COMPLETE_THREAD4_READBACK" if passed else "FAIL_THREAD4_READBACK",
        passed=passed, normal_proposal_denominator=len(expected), readback_count=len(actual),
        observed_settings_before_prefill_handoff_and_hold=True,
        source_sha256={str(episode / name): digest(episode / name) for name in ("cpu_thread_readback.jsonl", "wall_timing.jsonl")})


def diagnostic_report(output, plan, owner):
    result = _diagnostic(output, plan, owner)
    episode = output / "episode" if (output / "episode").is_dir() else output / "episode.partial"
    try:
        readback = readback_report(episode)
        parent.verify_derived_source_closure(output, plan)
    except Exception as error:
        readback = dict(passed=False, status="FAIL_THREAD4_READBACK", error=type(error).__name__ + ": " + str(error))
    result.update(schema=SCHEMA + ".diagnostic_report", cpu_thread_numerical_preset=numerical_preset(),
                  actual_thread_settings=readback)
    result["learned_driving_screen_passed"] = bool(result.get("learned_driving_screen_passed")) and readback["passed"]
    result["status"] = "PASS_BASIC_THREAD4_SIGNED_PRECURSOR" if result["learned_driving_screen_passed"] else "FAIL_BASIC_THREAD4_SIGNED_PRECURSOR"
    return result


def batch_summary(run_id, expected):
    result = _summary(run_id, expected)
    root, _ = batch_paths(run_id)
    readbacks = [bool(read_json(root / ("trial_%02d" % index) / "diagnostic_report.json")
        .get("actual_thread_settings", {}).get("passed", False)) for index in range(3)]
    result.update(schema=SCHEMA + ".batch_summary", cpu_thread_numerical_preset=numerical_preset(),
        all_three_actual_thread_readbacks_passed=all(readbacks), thread_readback_case_count=3)
    for field in ("all_trials_motion_body_policy_AND_wall100ms_passed", "all_trials_basic_driving_passed"):
        if field in result: result[field] = bool(result[field]) and all(readbacks)
    return result


@contextmanager
def activated(pins):
    """HH_261002 - Restore delegated aliases on success/error; inter-op1 is truthfully process-scoped."""
    global _active, _evidence_assets
    require(not _active and not previous._active, "thread4 owner scope cannot nest")
    assets = evidence_assets(pins)
    require(digest(Path(previous.__file__)) == FROZEN_OWNER_SHA, "frozen physical owner changed")
    values = dict(batch_paths=batch_paths, MODULE=MODULE, SCHEMA=SCHEMA, live=live, fixed_contract=fixed_contract,
        runtime_sources=runtime_sources, load_model=load_model, diagnostic_report=diagnostic_report, batch_summary=batch_summary)
    saved = {name: getattr(previous, name) for name in values}
    old_assets, old_environment, old_disk, old_threads = base.ASSETS, base.environment, parent.disk_check, torch.get_num_threads()
    _active = True; _evidence_assets = assets
    try:
        configure_threads()
        for name, value in values.items(): setattr(previous, name, value)
        base.ASSETS = dict(base.ASSETS) | assets; base.environment = environment; parent.disk_check = disk_check
        with previous.activated(pins):
            yield
    finally:
        for name, value in reversed(tuple(saved.items())): setattr(previous, name, value)
        base.ASSETS, base.environment, parent.disk_check = old_assets, old_environment, old_disk
        torch.set_num_threads(old_threads)
        _active = False; _evidence_assets = None


def main(argv=None):
    # HH_261002 - Reuse exact four-identity CLI parsing but activate NEW bindings around its actual public lifecycle.
    import argparse
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("weights", "metadata", "checkpoint", "declaration"):
        parser.add_argument("--expected-" + name + "-sha256", required=True)
    args, remainder = parser.parse_known_args(argv)
    pins = {name + "_sha256": getattr(args, "expected_" + name + "_sha256")
            for name in ("weights", "metadata", "checkpoint", "declaration")}
    with activated(pins):
        return parent.main(remainder)


if __name__ == "__main__":
    raise SystemExit(main())
