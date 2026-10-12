#!/usr/bin/env python3
"""HH_260929 - Reject prospective cross-model split leakage before data work."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / "learning/config/site_split_guard_20260929.json"
MODELS = ("map", "drive")
OPERATIONS = (
    "reuse_existing_train", "reuse_exploratory_dev", "offline_train_scout",
    "offline_dev_scout", "quality_diagnostic",
)
SEALED = {"Town04", "Town06", "Town07", "Town10HD_Opt", "Town15"}
LOCKED_ROLES = {
    "Town01": {"map": ["reuse_existing_train", "reuse_exploratory_dev"],
               "drive": ["reuse_existing_train"]},
    "Town02_Opt": {"map": ["reuse_existing_train"],
                   "drive": ["offline_train_scout"]},
    "Town03": {"map": [],
               "drive": ["reuse_exploratory_dev", "offline_dev_scout"]},
    "Town05_Opt": {"map": ["reuse_exploratory_dev"],
                   "drive": ["offline_dev_scout"]},
    "Town12": {"map": [], "drive": ["offline_train_scout"]},
    "Town13": {"map": ["quality_diagnostic"], "drive": []},
    "C_track_1_0_7": {"map": [], "drive": ["reuse_existing_train"]},
    "Woraksan_v1_0_3_parking_lot_hegiht_fit": {
        "map": [], "drive": ["offline_train_scout"]},
}
NO_GRANT_SITES = {
    "Town02", "Town05", "Town08", "Town09", "Town10HD", "Town11",
}
EXPECTED_SITES = set(LOCKED_ROLES) | SEALED | NO_GRANT_SITES
SOURCE_REFS = (
    "docs/assets/validation/2026-09-29/map_new_dev_feasibility/README.md",
    "docs/assets/validation/2026-09-27/map_next_training_design/README.md",
    "docs/assets/validation/2026-09-27/drive_next_data_plan/README.md",
    "tools/e2e/carla_expert_suite.yaml",
)


def validate_registry(value: dict) -> dict:
    """HH_260929 - Fail closed on unknown roles, inconsistent sites or false independent DEV."""
    if (value.get("schema") != "portable_e2e.integrated_site_split_guard.v1"
            or value.get("independent_dev_sites") != []
            or value.get("operations") != list(OPERATIONS)):
        raise ValueError("split registry contract changed")
    sites = value.get("sites")
    # HH_260929 - A new site must require a reviewed code change, not just a JSON grant.
    if not isinstance(sites, dict) or set(sites) != EXPECTED_SITES:
        raise ValueError("site registry differs from the frozen suite coverage")
    for site, entry in sites.items():
        if not isinstance(site, str) or not isinstance(entry, dict) or not entry.get("reason"):
            raise ValueError("invalid site entry")
        for model in MODELS:
            granted = entry.get(model)
            if (not isinstance(granted, list) or len(granted) != len(set(granted))
                    or any(operation not in OPERATIONS for operation in granted)):
                raise ValueError(f"invalid {model} permissions for {site}")
        if site in SEALED and any(entry[model] for model in MODELS):
            raise ValueError(f"held-out site reopened: {site}")
        if site in NO_GRANT_SITES and any(entry[model] for model in MODELS):
            raise ValueError(f"unavailable or unverified site reopened: {site}")
    for site, roles in LOCKED_ROLES.items():
        if site not in sites or any(sites[site][model] != roles[model] for model in MODELS):
            raise ValueError(f"locked site role changed: {site}")
    if value.get("source_refs") != list(SOURCE_REFS):
        raise ValueError("split evidence list changed")
    for relative in SOURCE_REFS:
        path = ROOT / relative
        if (not path.is_file()
                or path.is_symlink() or not path.resolve().is_relative_to(ROOT)):
            raise ValueError("split evidence missing or outside workspace")
    return value


def check(site: str, model: str, operation: str, registry: dict) -> bool:
    """HH_260929 - Only exact, explicitly listed diagnostic/reuse work is allowed."""
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
        print(json.dumps({"status": "REGISTRY_VALID", "sites": len(registry["sites"]),
                          "independent_dev_sites": 0}, sort_keys=True))
        return
    allowed = check(args.site, args.model, args.operation, registry)
    print(json.dumps({"status": "ALLOW" if allowed else "DENY", "site": args.site,
                      "model": args.model, "operation": args.operation,
                      "independent_dev": False}, sort_keys=True))
    if not allowed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
