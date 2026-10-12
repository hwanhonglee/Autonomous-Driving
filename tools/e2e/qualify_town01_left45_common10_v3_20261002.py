"""HH_261002 - Fresh LEFT347 admission identity; normalize only JSON containers, never labels or denial checks."""
from __future__ import annotations

import ast
import copy
import json
from pathlib import Path

from scripts.e2e import qualify_town01_left45_common10_20261002 as baseline
from scripts.e2e import prepare_town01_left45_common10_20261002 as baseline_converter

PARENT_SHA = "0e33ceb14e28ff463bea27b57170b1591af75229c6ad833d2e031af945bc7f19"
PARENT_CONVERTER_SHA = "0484849bf7f56a8924b47c708cee51b24f04102589983dee29733cb00bba7b59"
IDENTITIES = {
    "carla.actual_town01_left45_common10_qualification.20261002.v2": "carla.actual_town01_left45_common10_qualification.20261002.v3",
    "carla.actual_town01_left45_common10_conversion.20261002.v2": "carla.actual_town01_left45_common10_conversion.20261002.v3",
    "carla_actual_town01_left347_to_common10_20261002_v2": "carla_actual_town01_left347_to_common10_20261002_v3",
    "storage/datasets/prepared/town01-low-speed-left45-train-20261002-v2": "storage/datasets/prepared/town01-low-speed-left45-train-20261002-v3",
    "storage/experiments/training/2026-10-02/town01_left45_admission_v2/qualification.json": "storage/experiments/training/2026-10-02/town01_left45_admission_v3/qualification.json",
    "tools/e2e/prepare_town01_left45_common10_20261002.py": "tools/e2e/prepare_town01_left45_common10_v3_20261002.py",
}
OLD_COMPARISON = ast.parse("saved==actual and list(saved_windows)==windows", mode="eval").body
NEW_COMPARISON = ast.parse("saved==actual and list(saved_windows)==json_containers(windows)", mode="eval").body


def json_containers(value):
    # HH_261002 - Tuple/list containers alone change. Reject non-JSON keys/scalars and preserve integer/float/bool/NULL values.
    def check(item):
        if type(item) in (tuple, list):
            for child in item: check(child)
        elif type(item) is dict:
            baseline.require(all(type(key) is str for key in item), "JSON normalization must not cast mapping keys")
            for child in item.values(): check(child)
        else:
            baseline.require(type(item) in (str, int, float, bool, type(None)), "JSON normalization must not cast scalar types")
    check(value)
    return json.loads(json.dumps(value, allow_nan=False))


def admission_ast():
    baseline.require(baseline.sha(baseline.__file__) == PARENT_SHA
        and baseline.sha(baseline_converter.__file__) == PARENT_CONVERTER_SHA, "frozen V2 admission/converter changed")
    original = ast.parse(Path(baseline.__file__).read_bytes())
    baseline.require(ast.dump(original.body[-1].test) == ast.dump(ast.parse('__name__=="__main__"', mode="eval").body),
        "exact original CLI guard required")
    original.body.pop(); changed = copy.deepcopy(original); counts = dict.fromkeys(IDENTITIES, 0)
    for node in ast.walk(changed):
        if isinstance(node, ast.Constant) and type(node.value) is str and node.value in IDENTITIES:
            counts[node.value] += 1; node.value = IDENTITIES[node.value]
    baseline.require(all(count == 1 for count in counts.values()), "exact six new admission identities required")
    evidence = next(node for node in changed.body if isinstance(node, ast.FunctionDef) and node.name == "evidence")
    matches = [node for node in ast.walk(evidence) if isinstance(node, ast.Call) and node.args
        and ast.dump(node.args[0]) == ast.dump(OLD_COMPARISON)]
    baseline.require(len(matches) == 1 and isinstance(matches[0].func, ast.Name) and matches[0].func.id == "require",
        "one exact whole347 saved-window comparison required")
    matches[0].args[0] = copy.deepcopy(NEW_COMPARISON)
    inverse = copy.deepcopy(changed)
    for node in ast.walk(inverse):
        if isinstance(node, ast.Constant) and type(node.value) is str and node.value in IDENTITIES.values():
            node.value = next(old for old, new in IDENTITIES.items() if new == node.value)
        if isinstance(node, ast.Call) and node.args and ast.dump(node.args[0]) == ast.dump(NEW_COMPARISON):
            node.args[0] = copy.deepcopy(OLD_COMPARISON)
    baseline.require(ast.dump(inverse) == ast.dump(original), "another original admission/QA/math/denial statement changed")
    return changed


exec(compile(ast.fix_missing_locations(admission_ast()), __file__, "exec"), globals())
FROZEN = dict(FROZEN, **{"tools/e2e/qualify_town01_left45_common10_20261002.py": PARENT_SHA,
    "tools/e2e/prepare_town01_left45_common10_20261002.py": PARENT_CONVERTER_SHA})

if __name__ == "__main__": raise SystemExit(main())
