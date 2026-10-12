#!/usr/bin/env python3
"""HH_260929 - Scope one prospective C-track map capture without changing V1."""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / "learning/config/site_split_guard_20260929_v2.json"
PARENT_V1_SHA256 = {
    "tools/e2e/site_split_guard.py":
        "68529151bfce90fad97bbf3519d59cc5607f7dcc2f92c6c91a3c2a00ade17f45",
    "learning/config/site_split_guard_20260929.json":
        "da0245f3a863b3b03dc0e364eb067354a27d36fc123d2bb860e1a4bff7a9690c",
}
PARENT_REGISTRY = "learning/config/site_split_guard_20260929.json"
READINESS_REF = "docs/assets/validation/2026-09-29/map_ctrack_capture_readiness/README.md"
SCHEMA = "portable_e2e.integrated_site_split_guard.v2"
TARGET_SITE = "C_track_1_0_7"
TARGET_MODEL = "map"
CAPTURE_OPERATION = "owned_train_capture_pending_qa"
MODELS = ("map", "drive")
OPERATIONS = (
    "reuse_existing_train", "reuse_exploratory_dev", "offline_train_scout",
    "offline_dev_scout", "quality_diagnostic", CAPTURE_OPERATION,
)
CAPTURE_CONTRACT = {
    "site": TARGET_SITE,
    "model": TARGET_MODEL,
    "operation": CAPTURE_OPERATION,
    "owned_capture_only": True,
    "dataset_admitted": False,
    "train_eligible": False,
    "independent_dev_approved": False,
    "test_use_authorized": False,
    "next_gate": "Exact owned route/map binding and full RGB-semantic-depth QA before separate dataset-admission review.",
}
V2_COMMENT = (
    "HH_260929 - Permit only an owned C-track map TRAIN-candidate capture; "
    "captured frames remain unadmitted pending QA."
)
V2_SCOPE = (
    "Prospective C-track map owned capture only; no dataset admission, "
    "training eligibility, independent DEV, or TEST use."
)
V2_TARGET_REASON = (
    "Existing driving TRAIN exposure makes C-track ineligible as independent DEV. "
    "Only an owned RGB-semantic-depth map TRAIN-candidate capture is permitted; "
    "frames remain unadmitted until separate QA and admission review."
)


def _checked_workspace_file(relative: str) -> Path:
    """HH_260929 - Reject symlinked or missing policy evidence before a grant."""
    path = ROOT / relative
    if (not path.is_file() or path.is_symlink()
            or not path.resolve().is_relative_to(ROOT)):
        raise ValueError(f"policy evidence missing or outside workspace: {relative}")
    return path


def _frozen_parent_registry() -> dict:
    """HH_260929 - Pin both V1 bytes so historical split meaning cannot drift."""
    for relative, expected_sha256 in PARENT_V1_SHA256.items():
        actual_sha256 = hashlib.sha256(
            _checked_workspace_file(relative).read_bytes()).hexdigest()
        if actual_sha256 != expected_sha256:
            raise ValueError(f"frozen V1 SHA256 changed: {relative}")
    parent = json.loads(_checked_workspace_file(PARENT_REGISTRY).read_text(encoding="utf-8"))
    if (parent.get("schema") != "portable_e2e.integrated_site_split_guard.v1"
            or parent.get("independent_dev_sites") != []
            or not isinstance(parent.get("sites"), dict)
            or len(parent["sites"]) != 19
            or parent["sites"].get(TARGET_SITE, {}).get(TARGET_MODEL) != []):
        raise ValueError("frozen V1 split contract differs from reviewed parent")
    return parent


def _expected_registry(parent: dict) -> dict:
    """HH_260929 - Derive exactly one new role from the byte-pinned V1 ledger."""
    expected = deepcopy(parent)
    expected["schema"] = SCHEMA
    expected["_comment"] = V2_COMMENT
    expected["scope"] = V2_SCOPE
    expected["operations"] = list(OPERATIONS)
    expected["sites"][TARGET_SITE][TARGET_MODEL] = [CAPTURE_OPERATION]
    expected["sites"][TARGET_SITE]["reason"] = V2_TARGET_REASON
    expected["source_refs"] = [*parent["source_refs"], READINESS_REF]
    expected["parent_v1_sha256"] = PARENT_V1_SHA256
    expected["capture_contract"] = CAPTURE_CONTRACT
    return expected


def validate_registry(value: dict) -> dict:
    """HH_260929 - Deny every unreviewed V2 policy or evidence mutation."""
    parent = _frozen_parent_registry()
    expected = _expected_registry(parent)
    if not isinstance(value, dict) or value != expected:
        raise ValueError("V2 registry differs from the single approved V1 role delta")
    for key in ("owned_capture_only", "dataset_admitted", "train_eligible",
                "independent_dev_approved", "test_use_authorized"):
        if type(value["capture_contract"][key]) is not bool:
            raise ValueError(f"capture contract must use a boolean: {key}")
    for relative in value["source_refs"]:
        _checked_workspace_file(relative)
    return value


def check(site: str, model: str, operation: str, registry: dict) -> bool:
    """HH_260929 - An ALLOW means capture permission, never dataset admission."""
    validate_registry(registry)
    if model not in MODELS or operation not in OPERATIONS:
        raise ValueError("unknown model or operation")
    return operation in registry["sites"].get(site, {}).get(model, [])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site")
    parser.add_argument("--model", choices=MODELS)
    parser.add_argument("--operation", choices=OPERATIONS)
    args = parser.parse_args()
    registry = validate_registry(json.loads(REGISTRY.read_text(encoding="utf-8")))
    specified = (args.site is not None, args.model is not None,
                 args.operation is not None)
    if any(specified) and not all(specified):
        parser.error("--site, --model and --operation must be supplied together")
    if not any(specified):
        print(json.dumps({"status": "REGISTRY_VALID_V2", "sites": len(registry["sites"]),
                          "independent_dev_sites": 0, "dataset_admitted": False,
                          "train_eligible": False, "capture_authorized": False,
                          "permission_scope": "capture_only"},
                         sort_keys=True))
        return
    allowed = check(args.site, args.model, args.operation, registry)
    capture_authorized = (allowed and args.site == TARGET_SITE
                          and args.model == TARGET_MODEL
                          and args.operation == CAPTURE_OPERATION)
    print(json.dumps({"status": "ALLOW" if allowed else "DENY", "site": args.site,
                      "model": args.model, "operation": args.operation,
                      "dataset_admitted": False, "train_eligible": False,
                      "capture_authorized": capture_authorized,
                      "permission_scope": "capture_only" if capture_authorized else "no_capture_grant",
                      "independent_dev": False},
                     sort_keys=True))
    if not allowed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
