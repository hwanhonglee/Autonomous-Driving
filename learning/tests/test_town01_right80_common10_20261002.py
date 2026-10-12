"""HH_261002 - Every original conditional597 negative replays at697; no actual raw/window/dataset reads or TRAIN authority."""
import ast
import copy
import hashlib
from pathlib import Path

from scripts.e2e import qualify_town01_right80_common10_20261002 as admission
from scripts.e2e import prepare_town01_right80_common10_20261002 as convert

SOURCE = Path(__file__).with_name("test_town01_right70_common10_v2_20261002.py")
SOURCE_SHA = "a5dc2680b84e2081482fb1ae212007211d56702e3ed7d7b49a979f81cd354232"
NAMES = {"qualify_town01_right70_common10_v2_20261002": "qualify_town01_right80_common10_20261002",
    "prepare_town01_right70_common10_v2_20261002": "prepare_town01_right80_common10_20261002"}
COUNTS = {597: 697, 596: 696, 38208: 44608, 38207: 44607, 589: 689, 588: 688, 778: 878, 1272: 1472, 63.6: 73.6}


def definition():
    raw = SOURCE.read_bytes(); assert hashlib.sha256(raw).hexdigest() == SOURCE_SHA
    old = ast.parse(raw); new = copy.deepcopy(old); changes = []; imports = 0
    for node in ast.walk(new):
        if isinstance(node, ast.ImportFrom):
            for alias in node.names:
                if alias.name in NAMES:
                    changes.append((alias, "name", alias.name)); alias.name = NAMES[alias.name]; imports += 1
        if isinstance(node, ast.keyword) and node.arg == "right70_protocol":
            changes.append((node, "arg", node.arg)); node.arg = "right80_protocol"
        if isinstance(node, ast.Constant) and type(node.value) in (int, float) and node.value in COUNTS:
            before = node.value; node.value = COUNTS[before]; changes.append((node, "value", before))
        elif isinstance(node, ast.Constant) and type(node.value) is str:
            before = node.value
            # HH_261002 - Exchange exact old/new run strings simultaneously; keep the wrong-run negative wrong, not accepted.
            if before == "v2": node.value = "v1"
            elif before == "v1": node.value = "v2"
            else:
                node.value = before.replace("right70", "right80").replace("RIGHT597", "RIGHT697")
                node.value = node.value.replace("-20261002-v2", "-20261002-v1").replace("actual_right_v2", "actual_right80_v1")
                node.value = node.value.replace("all597", "all697").replace("through1272", "through1472")
            if before != node.value: changes.append((node, "value", before))
    assert imports == 2
    inverse = copy.deepcopy(new); nodes = list(ast.walk(new)); inverse_nodes = list(ast.walk(inverse))
    for node, key, before in changes: setattr(inverse_nodes[nodes.index(node)], key, before)
    assert ast.dump(inverse) == ast.dump(old)
    return new


exec(compile(ast.fix_missing_locations(definition()), __file__, "exec"), globals())


def test_failed_right70_never_requalified_and_all697_history_scope():
    assert admission.SCHEMA == "carla.actual_town01_right80_common10_qualification.20261002.v1"
    assert admission.RESULT_SCHEMA == "carla.actual_town01_right80_common10_conversion.20261002.v1"
    assert admission.ADAPTER_ID == "carla_actual_town01_right697_to_common10_20261002_v1"
    assert admission.task.SCHEMA == "carla.actual_town01_right80_teacher.20261002.v1"
    assert admission.SCOPE["failed_RIGHT70_attempts_not_requalified"] is True
    assert admission.task.ANCHORS == 697 and admission.QA_WINDOWS.name == "all697_true_future_windows.jsonl"
    assert admission.RECEIPT.parent.name == "town01_right80_admission_v1"
    assert admission.validation_function().__globals__["COUNT"] == 697
    assert admission.validation_function().__globals__["CAMERA"] == 878
    assert admission.qa.FROZEN.items() <= admission.FROZEN.items()


def test_frozen_original_denial_weather_and_json_container_provider_untouched():
    assert admission.json_containers is admission.previous.json_containers
    assert not hasattr(admission, "verify_weather_provenance")
    assert convert.base._require_native_admission_not_denied is admission.adapter._require_native_admission_not_denied
    assert admission.previous.SCOPE["fixed_driving_relative_anchor_offsets"] == [80, 772]
    assert admission.previous.SCHEMA.endswith(".v3")
