"""HH_261002 - Prepare whole-archive callers only under external ROOT authority; no render, inference, or verdict repair."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

from scripts.e2e import publish_temporal_intrinsic_town03_right_v2_20261002 as publisher

ROOT, paired, base = publisher.ROOT, publisher.previous, publisher.previous.previous
SCHEMA = "portable_e2e.actual_intrinsic_pair_publication_caller_preparation.20261002.v1"
PUBLISHER_SHA = "670742427969b4894fc37580dad3baa5487f618b0e6258c4d8d6dd8069ac88f1"
PAIR_AUTHORITY = ROOT/"storage/experiments/plans/2026-10-02/temporal_intrinsic_curve_completed_pair_v2/root_completed_pair_authority.json"
PAIR_AUTHORITY_SHA = "05d17727e0faadcdb31885c9332d87df29032f0e6f05ddf1137dd2e57e34323f"
OUTPUT = ROOT/"storage/experiments/plans/2026-10-02/temporal_intrinsic_pair_town03_publication_v2/prepared_callers_v1"
FONTS = tuple(Path("/usr/share/fonts/truetype/dejavu")/name for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf"))
MINIMUM_FREE_BYTES = 5*1024**3 + 64*1024**2
LEADING_NAMES = ("declaration.json", "owner_result.json", "diagnostic_report.json")
FAILED_STATUS = "FAIL_KNOWN_DEV_RIGHT_TURN_PRECURSOR"
require, canonical, strict_json, valid_sha = base.require, base.canonical, base.strict_json, base.sha


def digest(path):
    path = canonical(path); value = hashlib.sha256()
    require(path.is_file(), "regular source/archive file required")
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024**2), b""): value.update(chunk)
    return value.hexdigest()


def contract():
    require("torch" not in sys.modules and "carla" not in sys.modules, "metadata-only process required")
    require(digest(Path(publisher.__file__).resolve()) == PUBLISHER_SHA, "exact frozen publisher revision required")
    publisher.verify_frozen()
    return dict(schema=SCHEMA, helper_sha256=digest(Path(__file__).resolve()), publisher_sha256=PUBLISHER_SHA,
        publication_source_sha256={str(path.relative_to(ROOT)): digest(path) for path in sorted(publisher.publication_sources())},
        font_sha256={str(path): digest(path) for path in FONTS},
        run_roots={arm: str(paired.PAIR_ROOT/arm) for arm in paired.ARMS},
        publication_outputs={arm: str(paired.output_for(paired.PAIR_ROOT/arm)) for arm in paired.ARMS},
        caller_output=str(OUTPUT), completed_pair_authority_path=str(PAIR_AUTHORITY),
        completed_pair_authority_sha256=PAIR_AUTHORITY_SHA, PNG_index_policy="first_quarter_half_threequarters_exact_last",
        minimum_free_bytes=MINIMUM_FREE_BYTES, actual_archive_read_authorized_by_import=False,
        renderer_or_model_execution=False, original_FAIL_verdicts_preserved=True, TRAIN_permission=False)


def inventory(run):
    run = canonical(run); require(run.is_dir(), "original completed arm directory required")
    members = list(run.rglob("*"))
    require(not any(path.is_symlink() for path in members), "no symlink member or directory in original archive")
    require(all(path.is_file() or path.is_dir() for path in members), "no non-regular archive member")
    files = sorted(path for path in members if path.is_file())
    return {path.relative_to(run).as_posix(): digest(path) for path in files}


def parsed(path, expected, pins):
    path = canonical(path); require(path.is_file(), "regular externally pinned JSON required")
    raw = path.read_bytes(); actual = hashlib.sha256(raw).hexdigest()
    require(actual == valid_sha(expected), "external or first-read JSON bytes differ")
    require(str(path) not in pins or pins[str(path)] == actual, "conflicting first-read JSON pin")
    pins[str(path)] = actual
    return strict_json(raw)


def recheck(pins, tables):
    for name, expected in pins.items(): require(digest(Path(name)) == expected, "input/source/font changed before caller write")
    for run, table in tables.items(): require(inventory(Path(run)) == table, "whole original archive changed before caller write")


def prepare(authority_path, authority_sha):
    # HH_261002 - The ROOT-reviewed helper/source contract is authenticated BEFORE any real arm archive is opened.
    wanted = contract(); pins = {}
    authority_path = canonical(authority_path)
    require(authority_path.is_relative_to(ROOT/"storage/experiments/plans/2026-10-02"), "owned plan authority required")
    authority = parsed(authority_path, authority_sha, pins)
    require(authority.get("schema") == SCHEMA+".external_ROOT_authority" and authority.get("contract") == wanted
        and authority.get("caller_preparation_authorized") is True
        and all(authority.get(key) is False for key in ("publication_authorized", "renderer_authorized", "model_execution_authorized", "TRAIN_authorized")),
        "explicit metadata-only ROOT authority required")
    roots = {arm: canonical(Path(name)) for arm, name in wanted["run_roots"].items()}
    expected_leading = {str(run/name) for run in roots.values() for name in LEADING_NAMES}
    leading = authority.get("original_outcome_file_sha256")
    require(type(leading) is dict and set(leading) == expected_leading, "ROOT must independently pin all six original outcome files")
    output = canonical(OUTPUT)
    require(not output.exists() and not output.is_symlink(), "fresh create-only caller output required")
    require(not any(output == run or output.is_relative_to(run) for run in roots.values())
        and not any(output == canonical(name) or output.is_relative_to(canonical(name))
            for name in wanted["publication_outputs"].values()), "caller metadata must stay outside original archives and publication outputs")
    ancestor = next(path for path in output.parents if path.exists())
    require(shutil.disk_usage(ancestor).free >= MINIMUM_FREE_BYTES, "5GiB floor plus64MiB metadata reserve required")
    pins.update({str(ROOT/name): expected for name, expected in wanted["publication_source_sha256"].items()})
    pins.update(wanted["font_sha256"]); pins[str(Path(__file__).resolve())] = wanted["helper_sha256"]
    pair_authority = parsed(PAIR_AUTHORITY, PAIR_AUTHORITY_SHA, pins)
    require(pair_authority.get("schema") == paired.AUTHORITY_SCHEMA and pair_authority.get("both_fixed_endpoints_completed") is True
        and pair_authority.get("arm_order") == list(paired.ARMS) and pair_authority.get("model_promoted") is False
        and pair_authority.get("vehicle_control_approved") is False, "original completed pair authority/denials required")
    callers, tables, outcomes = {}, {}, {}
    for arm, run in roots.items():
        plan, owner, diagnostic = [parsed(run/name, leading[str(run/name)], pins) for name in LEADING_NAMES]
        require(plan.get("schema") == paired.OWNER_SCHEMA and plan.get("arm") == arm and plan.get("output") == str(run)
            and plan.get("completed_pair_authority") == str(PAIR_AUTHORITY)
            and plan.get("completed_pair_authority_sha256") == PAIR_AUTHORITY_SHA, "exact declared arm/authority required")
        require(diagnostic.get("original_owner") == owner and diagnostic.get("status") == FAILED_STATUS
            and all(diagnostic.get(key) is False for key in ("full_MAIN9_completion_claimed", "full_RTE03_approved", "training_data_approved", "vehicle_control_approved", "model_promoted")),
            "original failed model outcome and all denials must remain unchanged")
        table = inventory(run); tables[str(run)] = table
        require(all(table.get(name) == leading[str(run/name)] for name in LEADING_NAMES), "original outcome bytes changed during inventory")
        callers[arm] = dict(schema=paired.CALLER_SCHEMA, attempts=1, arm=arm, run_root=str(run),
            output=wanted["publication_outputs"][arm], PNG_index_policy=wanted["PNG_index_policy"],
            completed_pair_authority_sha256=PAIR_AUTHORITY_SHA, run_files_sha256=table,
            publication_source_sha256=wanted["publication_source_sha256"], font_sha256=wanted["font_sha256"],
            preparation_helper_schema=SCHEMA, actual_publication_authorized=False)
        outcomes[arm] = dict(status=diagnostic["status"], original_capture_status=diagnostic.get("original_capture_status"),
            original_diagnostic_sha256=leading[str(run/"diagnostic_report.json")], archived_file_count=len(table))
    require(contract() == wanted, "helper/publication source contract changed during preparation")
    recheck(pins, tables)
    require(shutil.disk_usage(ancestor).free >= MINIMUM_FREE_BYTES, "metadata reserve changed before first output write")
    output.mkdir(parents=True, exist_ok=False)
    receipts = {}
    for arm, caller in callers.items():
        path = output/(arm+"_caller_pins.json")
        with path.open("x") as stream: json.dump(caller, stream, indent=2, allow_nan=False); stream.write("\n")
        receipts[arm] = dict(path=str(path), sha256=digest(path), **outcomes[arm])
    recheck(pins, tables)
    summary = dict(schema=SCHEMA, status="CALLERS_PREPARED_ROOT_PUBLICATION_REVIEW_REQUIRED", callers=receipts,
        external_ROOT_authority_sha256=authority_sha, helper_sha256=wanted["helper_sha256"],
        source_and_metadata_input_sha256=pins, original_raw_rewritten=False, publication_executed=False,
        renderer_executed=False, model_executed=False, TRAIN_permission=False)
    with (output/"summary.json").open("x") as stream: json.dump(summary, stream, indent=2, allow_nan=False); stream.write("\n")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--print-contract", action="store_true")
    parser.add_argument("--authority", type=Path); parser.add_argument("--authority-sha256")
    args = parser.parse_args(argv)
    if args.print_contract:
        require(args.authority is None and args.authority_sha256 is None, "contract-only opens no original archive")
        print(json.dumps(contract(), indent=2, allow_nan=False)); return 0
    require(args.authority is not None and args.authority_sha256 is not None, "paired ROOT authority path/SHA required")
    result = prepare(args.authority, args.authority_sha256)
    print(json.dumps(dict(status=result["status"], callers=result["callers"], publication_executed=False), allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
