"""HH_261002 - All697 prospective RIGHT80 true64 windows; frozen numerical checks and prior failures remain, never TRAIN permission."""
from __future__ import annotations

import ast
import copy

from scripts.e2e import audit_town01_right70_true_future64_v2_20261002 as previous
from scripts.e2e import run_owned_town01_low_speed_right80_20261002 as declared_owner

PREVIOUS_SHA = "33f1371dab2f1ef68f14d9e28a87e79edc404a494fc40786bd9c14a837133531"
IDENTITIES = {previous.SCHEMA: "carla.actual_town01_right80_true_future64_diagnostic.20261002.v1",
    str(previous.OUTPUT.relative_to(previous.ROOT)): "storage/experiments/diagnostics/2026-10-02/town01_low_speed_right80_future64_v1",
    "v2": "v1", "town01_right_6p1_108p1_24p1_actual_right70": "town01_right_6p1_108p1_24p1_actual_right80"}


def identity_ast():
    previous.require(previous.owner.digest(previous.__file__) == PREVIOUS_SHA, "frozen RIGHT70 V2 true64 provider changed")
    old = previous.identity_ast(); new = copy.deepcopy(old); changes = []; imports = 0; counts = dict.fromkeys(IDENTITIES, 0)
    for node in ast.walk(new):
        if isinstance(node, ast.ImportFrom):
            for alias in node.names:
                if alias.name == "run_owned_town01_low_speed_right70_v2_20261002":
                    changes.append((alias, "name", alias.name)); alias.name = "run_owned_town01_low_speed_right80_20261002"; imports += 1
        if isinstance(node, ast.Constant) and type(node.value) is str:
            before = node.value
            if before in IDENTITIES:
                counts[before] += 1; node.value = IDENTITIES[before]
            elif not before.startswith("tools/e2e/"):
                node.value = before.replace("right70_protocol", "right80_protocol").replace("all597_true_future", "all697_true_future")
            if node.value != before: changes.append((node, "value", before))
    previous.require(imports == 1 and counts == {previous.SCHEMA: 1, str(previous.OUTPUT.relative_to(previous.ROOT)): 1,
        "v2": 4, "town01_right_6p1_108p1_24p1_actual_right70": 1}, "only current owner/schema/output/case/scenario identities required")
    inverse = copy.deepcopy(new); nodes = list(ast.walk(new)); inverse_nodes = list(ast.walk(inverse))
    for node, key, before in changes: setattr(inverse_nodes[nodes.index(node)], key, before)
    previous.require(ast.dump(inverse) == ast.dump(old), "RIGHT80 changed true64/masks/geometry/native/body/pixel/denial mathematics")
    return new


exec(compile(ast.fix_missing_locations(identity_ast()), __file__, "exec"), globals())
require(owner is declared_owner and task is owner.task, "actual RIGHT80 providers required; failed RIGHT70 is never promoted")
FROZEN.update(previous.FROZEN)
FROZEN.update({"tools/e2e/audit_town01_right70_true_future64_v2_20261002.py": PREVIOUS_SHA,
    "tools/e2e/town01_low_speed_right80_task_20261002.py": "39c95445fb4c91168933903357adef8cfc46bd49788ba0a03ae6da1355c17aa0",
    "tools/e2e/collect_town01_low_speed_right80_20261002.py": "2f1426ab7e0ce56b9fd85d306a7dc896eb881760d1dcdff03cb9da34a766d432",
    "tools/e2e/run_owned_town01_low_speed_right80_20261002.py": "4aae2152c047e56db60324f5bc834840a8489deb82da091cfdc32f937c0a6cdc",
    "tools/e2e/audit_town01_right80_closing_control_20261002.py": "f724029fb1ce44147d0586d474ba209ebfc5a199048fb1f1d560c45a9596ebb9"})


if __name__ == "__main__": raise SystemExit(main())
