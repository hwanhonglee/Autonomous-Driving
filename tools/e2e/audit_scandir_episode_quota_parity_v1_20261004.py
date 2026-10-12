"""HH_261004 - Prepare complete owned-tree quota accounting with fewer metadata probes; never replace a frozen runtime guard."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import stat


ROOT = Path(__file__).resolve().parents[2]
ORIGINAL_SOURCE = ROOT / "tools/e2e/research_temporal_v2_owned_camera_town07_20261001.py"
ORIGINAL_SHA = "fbeb39844f07a4e1928665253ded08038fa0cc27f705d2c6fad2728d934d800b"
SCHEMA = "portable_e2e.complete_scandir_episode_quota_counterfactual.20261004.v1"
TEST_SOURCE = ROOT / "learning/tests/test_scandir_episode_quota_parity_v1_20261004.py"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def original_directory_bytes():
    """HH_261004 - Compile only the sealed original accounting function with stdlib globals; import no owner, Torch or simulator."""
    require(digest(ORIGINAL_SOURCE) == ORIGINAL_SHA, "frozen original quota source changed")
    tree = ast.parse(ORIGINAL_SOURCE.read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "directory_bytes")
    namespace = dict(Path=Path, require=require)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                 "SEALED_original_directory_bytes_CPU_reference_only", "exec"), namespace)
    return namespace[function.name]


def scandir_directory_bytes(directory, *, receipt=None):
    """HH_261004 - Walk every entry every call, deny symlinks, and sum every regular-file st_size without payload reads or a cache."""
    root = Path(directory)
    try:
        root_state = root.lstat()
    except FileNotFoundError:
        return 0
    require(stat.S_ISDIR(root_state.st_mode) and not stat.S_ISLNK(root_state.st_mode), "owned episode directory required")
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    pending, descriptors = [], set()
    total, entries, directories, regular, special = 0, 0, 0, 0, 0

    def open_directory(name, expected, *, parent_fd=None):
        # HH_261004 - Open relative to the admitted parent descriptor, never follow a replaced directory symlink.
        descriptor = os.open(name, flags, dir_fd=parent_fd)
        descriptors.add(descriptor)
        actual = os.fstat(descriptor)
        require(stat.S_ISDIR(actual.st_mode) and (actual.st_dev, actual.st_ino) == (expected.st_dev, expected.st_ino),
                "episode directory identity changed during complete traversal")
        iterator = os.scandir(descriptor)
        pending.append((descriptor, iterator))

    try:
        open_directory(root, root_state)
        directories += 1
        while pending:
            descriptor, iterator = pending[-1]
            try:
                entry = next(iterator)
            except StopIteration:
                iterator.close()
                os.close(descriptor)
                descriptors.remove(descriptor)
                pending.pop()
                continue
            entries += 1
            require(not entry.is_symlink(), "symlink forbidden in episode quota")
            if entry.is_dir(follow_symlinks=False):
                metadata = entry.stat(follow_symlinks=False)
                require(stat.S_ISDIR(metadata.st_mode), "episode directory type changed during complete traversal")
                open_directory(entry.name, metadata, parent_fd=descriptor)
                directories += 1
            elif entry.is_file(follow_symlinks=False):
                metadata = entry.stat(follow_symlinks=False)
                require(stat.S_ISREG(metadata.st_mode), "episode regular-file type changed during complete traversal")
                total += metadata.st_size
                regular += 1
            else:
                # HH_261004 - Original accounting ignores FIFO/socket/device sizes; still check every special node for a raced symlink.
                metadata = entry.stat(follow_symlinks=False)
                require(not stat.S_ISLNK(metadata.st_mode), "symlink forbidden in episode quota")
                require(not stat.S_ISDIR(metadata.st_mode) and not stat.S_ISREG(metadata.st_mode),
                        "episode special-node type changed during complete traversal")
                special += 1
        if receipt is not None:
            receipt.update(complete_walk=True, entries=entries, directories_including_root=directories,
                regular_files=regular, special_nodes_ignored_like_original=special, regular_file_bytes=total,
                cached_or_incremental_byte_count_used=False, source_payload_read=False)
        return total
    finally:
        for _, iterator in pending:
            iterator.close()
        for descriptor in descriptors:
            os.close(descriptor)


def prospective_contract():
    """HH_261004 - Specify separate metadata timing and real sensor-latency experiments; do not claim an unmeasured runtime gain."""
    return dict(schema=SCHEMA + ".prospective_contract", original_source=str(ORIGINAL_SOURCE), original_sha256=ORIGINAL_SHA,
        fresh_helper_sha256=digest(Path(__file__).resolve()),
        original_reference_scope="only the exact sealed directory_bytes AST; no runtime activation/import",
        candidate_scope="every-call complete fd-relative scandir DFS with no symlink following",
        all_regular_file_bytes_and_hidden_or_nested_entries_required=True,
        incremental_byte_count_or_membership_cache=False, original_quota_or_free_floor_removed=False,
        original_12MiB_next_bundle_and_closing_reserve_checks_unchanged=True,
        original_per_bundle_free_disk_check_unchanged=True,
        no_source_SHA_or_pixels_or_sensor_order_changed=True,
        exact_count_equivalence_scope="stable readable owned directory trees",
        additional_fail_closed_scope=["dangling symlink root", "unreadable directory", "directory replacement during traversal", "metadata OSError"],
        original_Path_rglob_partial_error_suppression_not_treated_as_safe=True,
        special_FIFO_socket_device_nodes_counted_as_regular_files=False,
        real_tree_counterfactual=dict(status="PREPARED_NOT_EXECUTED_REQUIRES_ROOT_SEAL",
            run="temporal_route_joint_town03_known_dev_straight_callback45_v3",
            read_only_metadata_only_no_JPEG_payload=True, exact_original_and_new_byte_count_required=True,
            full_final_tree_pairing_not_a_live_prefix_measurement=True,
            all450_prefix_comparison_requires_separately_declared_synthetic_metadata_mirrors=True,
            original_exact_per_bundle_full_directory_membership_not_recorded=True,
            all450_existing_timing_rows_and_six_camera_recording_membership_retained=True,
            paired_iteration_order="fixed AB/BA after prospective source/input seal",
            full_unsorted_failed_measurements_retained=True, timing_does_not_prove_actual_sensor_latency=True),
        later_live_intervention=dict(status="NOT_ACTIVATED_REQUIRES_NEW_CALLBACK45_V4_SOURCE_AND_OWNER_DECLARATION",
            exact_own_output_scope_required=True, original_owner_lock_PID_generation_and_source_seals_required=True,
            original_full450_bundle900_state904_ACK_denominator_retained=True,
            new_per_quota_walk_and_disk_check_monotonic_boundaries_required=True,
            same_first_callback_plan_and_ACK100ms_gates_required=True,
            native7200_or_explicit_new_prospective_mark_denominator_required=True,
            scheduling_vs_scan_causation_not_proven_before_new_actual_run=True),
        actual_runtime_edited_or_activated=False, CARLA_GPU_SSH_install_or_training=False,
        model_promoted=False, formal_function_approval=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--prospective-only", action="store_true", required=True)
    parser.parse_args(argv)
    require(digest(ORIGINAL_SOURCE) == ORIGINAL_SHA, "frozen quota source differs")
    print(json.dumps(prospective_contract(), indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
