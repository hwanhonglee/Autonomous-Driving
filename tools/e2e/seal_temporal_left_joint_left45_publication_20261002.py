"""HH_261002 - Hash both closed original attempts for ROOT's separate publication reader; never infer, actuate, edit results or approve driving."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

from scripts.e2e import publish_temporal_left_joint_left45_comparison_20261002 as pub

ROOT = pub.ROOT
SCHEMA = "portable_e2e.ROOT_left45_full_archive_sealing.20261002.v1"
OUTPUT = ROOT / "storage/experiments/plans/2026-10-02/temporal_left_joint_left45_v1/ROOT_publication_caller.json"
PUBLISHER_SHA = "4742365be59cf6ea02d833368e11f76b40ab74ebdaa726f264897e3397dc9d53"


def document(path, expected):
    path = pub.archive.canonical(path)
    pub.require(path.is_relative_to(ROOT), "ROOT workspace document required")
    data = path.read_bytes()
    pub.require(len(data) <= 4*1024**2 and hashlib.sha256(data).hexdigest() == expected,
        "externally sealed first-read document differs: " + str(path))
    value = pub.archive.strict_json(data)
    pub.require(type(value) is dict, "exact ROOT document object required")
    return value


def seal(authority_path, authority_sha):
    pub.require("torch" not in sys.modules and "carla" not in sys.modules,
        "stdlib/PIL reader only; no simulator or model process")
    pub.require(pub.recorded.digest(pub.__file__) == PUBLISHER_SHA,
        "reviewed publisher bytes changed")
    authority = document(authority_path, authority_sha)
    pub.require(authority.get("schema") == SCHEMA and authority.get("purpose") == "BOTH_closed_archives_publication_only"
        and authority.get("model_promoted") is False and authority.get("vehicle_control_approved") is False
        and authority.get("pair") == str(pub.PAIR) and authority.get("output") == str(OUTPUT)
        and authority.get("source_tool_sha256") == pub.recorded.digest(__file__),
        "separate ROOT BOTH-arm hash-only scope required")
    expected = authority.get("document_sha256", {})
    roles = {"batch": pub.PAIR/"batch_plan.json", "completion": Path(authority["completed_run_authority"])}
    for arm in pub.ARMS:
        roles.update({arm+"_plan": pub.PAIR/(arm+"_plan.json"),
            arm+"_owner": pub.PAIR/arm/"owner_result.json",
            arm+"_diagnostic": pub.PAIR/arm/"diagnostic_report.json"})
    pub.require(set(expected) == set(roles), "all eight original closed documents required")
    values = {name: document(path, expected[name]) for name, path in roles.items()}
    batch = values["batch"]
    pub.require(batch["arm_order"] == list(pub.ARMS) and batch["physical_case_count"] == 2
        and batch["completed_run_authority_sha256"] == expected["completion"]
        and batch["case_sha256"] == {a: expected[a+"_plan"] for a in pub.ARMS}, "unchanged BOTH prospective plans required")
    for arm in pub.ARMS:
        owner, diagnostic = values[arm+"_owner"], values[arm+"_diagnostic"]
        pub.require(owner["cleanup_errors"] == [] and owner["source_hashes_match"] is True
            and owner["external_source_hashes_match"] is True and diagnostic["original_owner"] == owner
            and diagnostic["model_promoted"] is False and diagnostic["vehicle_control_approved"] is False,
            "closed source-stable original outcome required regardless of PASS or FAIL")
    inventories = {a: {n: pub.recorded.digest(pub.PAIR/a/n)
        for n in sorted(pub.archive.inventory(pub.PAIR/a))} for a in pub.ARMS}
    result = dict(contract=pub.public_contract(), batch_plan_sha256=expected["batch"],
        case_plan_sha256=batch["case_sha256"], completed_run_authority=authority["completed_run_authority"],
        completed_run_authority_sha256=expected["completion"], runs=inventories, publication_authorized=True)
    for name, path in roles.items(): pub.require(pub.recorded.digest(path) == expected[name], "closed document mutated while sealing")
    for arm in pub.ARMS:
        pub.require(pub.archive.inventory(pub.PAIR/arm) == set(inventories[arm]), "whole attempt membership changed")
        for name, sha in inventories[arm].items(): pub.require(pub.recorded.digest(pub.PAIR/arm/name) == sha, "whole attempt bytes changed")
    document(authority_path, authority_sha)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--authority", type=Path, required=True)
    parser.add_argument("--authority-sha256", required=True)
    args = parser.parse_args(argv)
    pub.require(not OUTPUT.exists(), "fresh ROOT caller only; no overwrite")
    result = seal(args.authority, args.authority_sha256)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT.open("x") as stream: json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(dict(status="BOTH_CLOSED_ARCHIVES_SEALED_NO_RENDER_NO_APPROVAL", caller=str(OUTPUT),
        caller_sha256=pub.recorded.digest(OUTPUT), files_per_arm={a:len(result["runs"][a]) for a in pub.ARMS})))
    return 0


if __name__ == "__main__": raise SystemExit(main())
