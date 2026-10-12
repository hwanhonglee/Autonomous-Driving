"""HH_261005 - Bind explicit frozen masked-copy GOOD6000 and CPU6 to fresh full209 capture, retaining all V4 guards."""

import ast
import copy
from contextlib import contextmanager
import hashlib
import inspect
from pathlib import Path
import textwrap

import torch

from portable_e2e.temporal_all_valid_masked_copy_research_v1_20261004 import AllValidMaskedCopyRouteRecoveryTemporalModelV3
from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
from . import collect_temporal_route_joint_town03_straight_full209_v4_20261004 as previous
from . import collect_temporal_route_recovery_fast45_20261001 as recovery
from . import collect_temporal_route_joint_thread4_signed45_20261002 as threads
from . import full_tree_quota_native_v1_20261005 as native_quota


SCHEMA = previous.SCHEMA
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_straight_full209.20261005.v5"
THREAD_SCHEMA = "portable_e2e.route_joint_thread6_masked_copy_numerical_preset.20261005.v1"
PARENT_SHA256 = "a501ce28d294a78463cb05b83d8b03a169b14e11945f83f14fa89fbbd970bac5"
CANDIDATE_SOURCE = previous.ROOT / "learning/portable_e2e/temporal_all_valid_masked_copy_research_v1_20261004.py"
CANDIDATE_TEST = previous.ROOT / "learning/tests/test_temporal_all_valid_masked_copy_research_v1_20261004.py"
CANDIDATE_SHA256 = "3cffa23b9a63965739d38cd47f980a799a6748cbf374e61ab35b3ceb91d66d5b"
CANDIDATE_TEST_SHA256 = "7dba2dbb77494101827b4b3399050ea1d05c24f7f15ccd96fc49dd2b0ea67d3a"
NATIVE_HELPER_SHA256 = "bed5ec04c912451f724c23d33cd12b74235f2012df01fb6730c01b08ecabd726"
NATIVE_C_SHA256 = "b09459895b9f1697a97aa05ad1f0bbf0c83ff501635ffa94bb6c6ed5304a360e"
NATIVE_LIBRARY = previous.ROOT / "storage/experiments/plans/2026-10-05/full209_native_quota_v2/lib_full_tree_quota_v2.so"
NATIVE_LIBRARY_SHA256 = "f072e50b9a14d620d0e2739edf7e5fb57e49a3ce8cbb5079304452605283438d"
_original_recovery_activation = recovery.activated
_active = False


def __getattr__(name):
    return getattr(previous, name)


def verify_frozen():
    # HH_261005 - The original factory's constructor global must never be blindly rebound to the candidate.
    require = previous.frozen._require
    for path, expected in ((Path(previous.__file__), PARENT_SHA256),
                           (CANDIDATE_SOURCE, CANDIDATE_SHA256), (CANDIDATE_TEST, CANDIDATE_TEST_SHA256)):
        require(path.resolve() == path and path.is_file() and not path.is_symlink()
            and hashlib.sha256(path.read_bytes()).hexdigest() == expected, "frozen V4/candidate source or test changed")
    require(recovery.RouteRecoveryTemporalModelV3 is RouteRecoveryTemporalModelV3,
        "original recovery constructor global was replaced")
    previous.verify_frozen()
    verify_native_artifacts()


def verify_native_artifacts():
    for path, expected in ((Path(native_quota.__file__), NATIVE_HELPER_SHA256),
                           (native_quota.SOURCE, NATIVE_C_SHA256), (NATIVE_LIBRARY, NATIVE_LIBRARY_SHA256)):
        previous.frozen._require(path.resolve() == path and path.is_file() and not path.is_symlink()
            and hashlib.sha256(path.read_bytes()).hexdigest() == expected, "frozen native quota helper/C/library changed")


def native_counter(engine, partial, original_counter):
    """HH_261005 - Substitute only the same owned partial tree; every call still visits every entry and retains failed receipts."""
    previous.frozen._require(type(engine) is native_quota.NativeFullCensus and engine.library == NATIVE_LIBRARY
        and engine.source_sha256 == NATIVE_C_SHA256 and engine.library_sha256 == NATIVE_LIBRARY_SHA256,
        "exact admitted native quota engine required")
    def count(directory, *, receipt=None):
        if Path(directory) != partial:
            return original_counter(directory, receipt=receipt)
        previous.frozen._require(type(receipt) is dict, "actual owned native quota walk receipt required")
        receipt.update(native_census_schema=native_quota.SCHEMA, native_census_source_sha256=NATIVE_C_SHA256,
            native_census_helper_sha256=NATIVE_HELPER_SHA256, native_census_library_sha256=NATIVE_LIBRARY_SHA256,
            native_census_library_path=str(NATIVE_LIBRARY), native_library_admitted_by_pinned_no_follow_descriptor=True)
        return engine.directory_bytes(directory, receipt=receipt)
    return count


def audit_quota_walks(value, cameras, timings):
    # HH_261005 - Add native engine/count identity to every retained row, then require every unchanged original chronology/quota gate.
    verify_native_artifacts()
    require = previous.frozen._require
    for row in value["rows"]:
        require(row.get("native_census_schema") == native_quota.SCHEMA
            and row.get("native_census_source_sha256") == NATIVE_C_SHA256
            and row.get("native_census_helper_sha256") == NATIVE_HELPER_SHA256
            and row.get("native_census_library_sha256") == NATIVE_LIBRARY_SHA256
            and row.get("native_census_library_path") == str(NATIVE_LIBRARY)
            and row.get("native_library_admitted_by_pinned_no_follow_descriptor") is True,
            "actual native quota row helper/C/library identity missing or changed")
        if row.get("status") == "PASS":
            require(row.get("directory_generation_postcheck_required") is True
                and type(row.get("directories_including_root")) is int and row["directories_including_root"] >= 1
                and type(row.get("special_nodes_ignored_like_original")) is int and row["special_nodes_ignored_like_original"] >= 0
                and row["entries"] == row["regular_files"] + row["special_nodes_ignored_like_original"] + row["directories_including_root"] - 1,
                "complete native entry/directory/regular/special census differs")
    result = previous.audit_quota_walks(value, cameras, timings)
    result.update(native_helper_C_and_library_pins_verified=True, complete_native_count_and_original_clock_checks=True,
        native_library_sha256=NATIVE_LIBRARY_SHA256, native_artifacts_unchanged_after_cleanup=True)
    return result


def thread_readback():
    value = dict(intra_op_threads=torch.get_num_threads(), inter_op_threads=torch.get_num_interop_threads())
    previous.frozen._require(value == dict(intra_op_threads=6, inter_op_threads=1), "actual V5 CPU6/interop1 preset differs")
    return value


def validate_candidate(model):
    require = previous.frozen._require
    require(type(model) is AllValidMaskedCopyRouteRecoveryTemporalModelV3
        and getattr(model, "_research_cpu_thread_numerical_revision", None) == THREAD_SCHEMA,
        "exact pinned actual masked-copy candidate and CPU6 owner marker required")
    require(getattr(model, "_research_masked_copy_actual_binding", None) == dict(
        source_sha256=CANDIDATE_SHA256, original_constructor_global_preserved=True,
        all101_original_state_tensors_equal=True, cpu_intraop_threads=6, cpu_interop_threads=1),
        "strict original factory/state-copy receipt required")
    require(len(model.state_dict()) == 101 and not model._recovery_active and model._recovery_features is None
        and model.image_encoder.request is None
        and set(model.curvature_head._forward_hooks) == {model._recovery_hook_handle.id}
        and model.curvature_head._forward_hooks[model._recovery_hook_handle.id] is model._recovery_hook_function
        and bool(torch.count_nonzero(model.route_recovery_branch[-1].weight).item()),
        "unchanged nonzero recovery branch, permanent head hook and clean original cache context required")
    thread_readback()


def candidate_activation_factory():
    """HH_261005 - Inverse-prove only the explicit admitted type predicate; retain original constructor global and all actuation guards."""
    original = ast.parse(textwrap.dedent(inspect.getsource(_original_recovery_activation))).body[0]
    changed, count = copy.deepcopy(original), 0
    for node in ast.walk(changed):
        if isinstance(node, ast.Compare) and ast.unparse(node) == "type(model) is not RouteRecoveryTemporalModelV3":
            node.comparators[0].id = "AllValidMaskedCopyRouteRecoveryTemporalModelV3"
            count += 1
    previous.frozen._require(count == 1, "exact one original model-type admission predicate required")
    restored = copy.deepcopy(changed)
    for node in ast.walk(restored):
        if isinstance(node, ast.Compare) and ast.unparse(node) == "type(model) is not AllValidMaskedCopyRouteRecoveryTemporalModelV3":
            node.comparators[0].id = "RouteRecoveryTemporalModelV3"
    previous.frozen._require(ast.dump(restored) == ast.dump(original), "another recovery actuation guard or policy operation changed")
    namespace = dict(vars(recovery), AllValidMaskedCopyRouteRecoveryTemporalModelV3=AllValidMaskedCopyRouteRecoveryTemporalModelV3)
    previous.frozen._require(namespace["RouteRecoveryTemporalModelV3"] is RouteRecoveryTemporalModelV3,
        "private activation cannot alias the original constructor")
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
        str(Path(__file__)) + ":inverse_proven_candidate_type_only", "exec"), namespace)
    return namespace["activated"]


@contextmanager
def candidate_activation(model, args):
    # HH_261005 - Mirror the genuine module's serial flag while the unchanged private guard executes; restore on every failure.
    previous.frozen._require(not recovery._active, "actual candidate recovery activation cannot nest")
    validate_candidate(model)
    activation = candidate_activation_factory()
    recovery._active = True
    try:
        with activation(model, args):
            yield
    finally:
        recovery._active = False


@contextmanager
def activated(model, args):
    """HH_261005 - Retain the genuine V4 recorder/fixture/factory chain, changing only candidate admission and CPU readback."""
    global _active
    verify_frozen()
    previous.frozen._require(not _active and not previous._active and recovery.activated is _original_recovery_activation,
        "fresh V5 cannot nest or replace an unknown recovery activation")
    validate_candidate(model)
    changes = {previous: dict(OWNER_SCHEMA=OWNER_SCHEMA),
        recovery: dict(activated=candidate_activation),
        threads: dict(THREAD_SCHEMA=THREAD_SCHEMA, thread_readback=thread_readback)}
    saved = {module: {name: getattr(module, name) for name in values} for module, values in changes.items()}
    _active = True
    try:
        for module, values in changes.items():
            for name, value in values.items():
                setattr(module, name, value)
        with previous.activated(model, args):
            collector = previous.collector
            original_collect = collector.collect_episode
            quota_module = previous.quota_counter
            original_counter = quota_module.scandir_directory_bytes
            engine = native_quota.NativeFullCensus(NATIVE_LIBRARY,
                expected_source_sha256=NATIVE_C_SHA256, expected_library_sha256=NATIVE_LIBRARY_SHA256)
            partial = args.output.with_name(args.output.name + ".partial")
            quota_module.scandir_directory_bytes = native_counter(engine, partial, original_counter)
            def collect(*positional, **kwargs):
                validate_candidate(model)
                positional[6].setdefault("capture_contract", {})["actual_masked_copy_CPU6"] = dict(
                    candidate_type=type(model).__module__ + "." + type(model).__name__,
                    source_sha256=CANDIDATE_SHA256, thread_schema=THREAD_SCHEMA, **thread_readback(),
                    strict_original_constructor_global_preserved=True, all101_original_state_tensors_equal=True,
                    original_finite_hash_cache_decoder_and_permanent_head_guards_retained=True,
                    complete_tree_quota_semantics_unchanged=True, original100ms_thresholds_unchanged=True,
                    actual_sensor_latency_gain_proven=False, model_promoted=False)
                positional[6]["capture_contract"]["actual_native_full_tree_quota"] = dict(
                    schema=native_quota.SCHEMA, helper_sha256=NATIVE_HELPER_SHA256, C_source_sha256=NATIVE_C_SHA256,
                    library=str(NATIVE_LIBRARY), library_sha256=NATIVE_LIBRARY_SHA256,
                    every_entry_every_call=True, incremental_cache_used=False, original_quota_free_floor_reserves_unchanged=True,
                    original_walk_before_exact_camera_bundle_unchanged=True, unrelated_paths_original_delegate=True)
                return original_collect(*positional, **kwargs)
            collector.collect_episode = collect
            try:
                yield
            finally:
                collector.collect_episode = original_collect
                quota_module.scandir_directory_bytes = original_counter
                verify_native_artifacts()
    finally:
        for module, values in reversed(tuple(saved.items())):
            for name, value in values.items():
                setattr(module, name, value)
        _active = False
