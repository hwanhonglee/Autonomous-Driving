"""HH_261002 - Conditional entire RIGHT697 research TRAIN admission; original native denials, LEFTv3/Common10 math and failed RIGHT70 stay immutable."""
from __future__ import annotations

import ast
import copy

from scripts.e2e import qualify_town01_left45_common10_v3_20261002 as previous
from scripts.e2e import prepare_town01_left45_common10_v3_20261002 as previous_converter
from scripts.e2e import audit_town01_right80_true_future64_20261002 as declared_qa

PARENT_SHA = "ff804fd3cbcee035c351e0dc97c57c83143adc65f9ca006338d86bac5cab3325"
PARENT_CONVERTER_SHA = "ab19f12bb78a2dfa5989be76d14332d67caed08a626e491a3824eb92faf6bc02"
QA_SHA = "50aafb4bdccaeddd83f62b4671c7479648d24d0b13da501cfb792b097ca8e272"
IDENTITIES = {previous.SCHEMA: "carla.actual_town01_right80_common10_qualification.20261002.v1",
    previous.RESULT_SCHEMA: "carla.actual_town01_right80_common10_conversion.20261002.v1",
    previous.ADAPTER_ID: "carla_actual_town01_right697_to_common10_20261002_v1",
    str(previous.PREPARED.relative_to(previous.ROOT)): "storage/datasets/prepared/town01-low-speed-right80-train-20261002-v1",
    str(previous.RECEIPT.relative_to(previous.ROOT)): "storage/experiments/training/2026-10-02/town01_right80_admission_v1/qualification.json",
    str(previous.CONVERTER.relative_to(previous.ROOT)): "tools/e2e/prepare_town01_right80_common10_20261002.py",
    "left": "right", "v2": "v1", "unchanged_original_TRAIN_catalog_route": "NEW_PINNED_XODR_CENTRELINE_SAME_EXISTING_TRAIN_CORRIDOR"}
COUNTS = {347: 697, 22208: 44608, 339: 689, 338: 688, 528: 878, 772: 1472, 38.6: 73.6}
json_containers = previous.json_containers


def admission_ast():
    previous.require(previous.sha(previous.__file__) == PARENT_SHA and previous.sha(previous_converter.__file__) == PARENT_CONVERTER_SHA
        and previous.sha(declared_qa.__file__) == QA_SHA, "frozen admission/converter/current all697 QA provider changed")
    original = previous.admission_ast(); changed = copy.deepcopy(original); changes = []; imports = 0
    for node in ast.walk(changed):
        if isinstance(node, ast.ImportFrom):
            for alias in node.names:
                if alias.name == "audit_town01_left45_true_future64_20261002":
                    changes.append((alias, "name", alias.name)); alias.name = "audit_town01_right80_true_future64_20261002"; imports += 1
        if isinstance(node, ast.Constant) and type(node.value) is str:
            before = node.value
            if before in IDENTITIES: node.value = IDENTITIES[before]
            elif not before.startswith("tools/e2e/"):
                node.value = before.replace("LEFT347", "RIGHT697").replace("LEFT", "RIGHT").replace("left45", "right80")
                node.value = node.value.replace("all347_true_future", "all697_true_future").replace("immutable347", "immutable697")
                node.value = node.value.replace("TRAIN347", "TRAIN697").replace("unchanged original TRAIN RIGHT route", "exact full generated TRAIN RIGHT route")
                node.value = node.value.replace("declared RIGHT V2 native", "declared RIGHT80 V1 native")
            if before != node.value: changes.append((node, "value", before))
        elif isinstance(node, ast.Constant) and type(node.value) in (int, float) and node.value in COUNTS:
            before = node.value; node.value = COUNTS[before]; changes.append((node, "value", before))
        if isinstance(node, ast.keyword) and node.arg == "rolling_LEFT_only": changes.append((node, "arg", node.arg)); node.arg = "rolling_RIGHT_only"
    previous.require(imports == 1, "one exact current RIGHT80 full QA import required")
    inverse = copy.deepcopy(changed); nodes = list(ast.walk(changed)); inverse_nodes = list(ast.walk(inverse))
    for node, key, before in changes: setattr(inverse_nodes[nodes.index(node)], key, before)
    previous.require(ast.dump(inverse) == ast.dump(original), "RIGHT697 changed numerical/native denial/authority/JSON-container checks")
    return changed


exec(compile(ast.fix_missing_locations(admission_ast()), __file__, "exec"), globals())
require(qa is declared_qa and task is declared_qa.task, "current RIGHT80 V1 native source only; no failed RIGHT70 promotion")
FROZEN.update(previous.FROZEN)
FROZEN.update({"tools/e2e/qualify_town01_left45_common10_v3_20261002.py": PARENT_SHA,
    "tools/e2e/prepare_town01_left45_common10_v3_20261002.py": PARENT_CONVERTER_SHA,
    "tools/e2e/audit_town01_right80_true_future64_20261002.py": QA_SHA})
SCOPE.update(full_generated_navigation_route_retained=True, capture_container_is_declared_not_measured_geometry=True,
    original32_and_failed_generated_p00_not_requalified=True, failed_RIGHT70_attempts_not_requalified=True,
    whole_nominal_clearance_is_not_actual_training_approval=True)


if __name__ == "__main__": raise SystemExit(main())
