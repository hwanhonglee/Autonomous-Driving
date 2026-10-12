"""HH_261002 - Exact retained RIGHT70 negatives under RIGHT80 synthetic counts; no actual receipt payload."""
import ast
import copy
import hashlib
from pathlib import Path

from scripts.e2e import audit_town01_right80_closing_control_20261002 as audit

SOURCE = Path(__file__).with_name("test_audit_town01_right70_closing_control_20261002.py")
SOURCE_SHA = "88de7c1f3be330daec3939e69832c70cb2562c7820d7535ccca3c077702025b2"
COUNTS = {1555: 1755, 1560: 1760, 1425: 1625, 1424: 1624, 1400: 1600, 1401: 1601,
    1450: 1650, 1525: 1725, 1429: 1629, 1655: 1855}


def definition():
    raw = SOURCE.read_bytes(); assert hashlib.sha256(raw).hexdigest() == SOURCE_SHA
    old = ast.parse(raw); new = copy.deepcopy(old); changes = []
    for node in ast.walk(new):
        if isinstance(node, ast.ImportFrom):
            for alias in node.names:
                if alias.name == "audit_town01_right70_closing_control_20261002":
                    changes.append((alias, "name", alias.name)); alias.name = "audit_town01_right80_closing_control_20261002"
        if isinstance(node, ast.Constant) and type(node.value) is int and node.value in COUNTS:
            changes.append((node, "value", node.value)); node.value = COUNTS[node.value]
        elif isinstance(node, ast.Constant) and type(node.value) is str:
            before = node.value; node.value = before.replace("all1555", "all1755")
            if node.value != before: changes.append((node, "value", before))
    inverse = copy.deepcopy(new); nodes = list(ast.walk(new)); inverse_nodes = list(ast.walk(inverse))
    for node, key, before in changes: setattr(inverse_nodes[nodes.index(node)], key, before)
    assert ast.dump(inverse) == ast.dump(old)
    # HH_261002 - This one AST assertion targets the previous function-body ABI; all actual guard fixtures remain untouched.
    new.body = [node for node in new.body if not isinstance(node, ast.FunctionDef)
        or node.name != "test_inverse_AST_four_count_values_and_only_count_error_labels"]
    return new


exec(compile(ast.fix_missing_locations(definition()), __file__, "exec"), globals())


def test_inverse_AST_four_count_values_and_only_count_error_labels():
    old = audit.previous.definition(); new = audit.definition()
    for node in ast.walk(new):
        if isinstance(node, ast.Constant) and type(node.value) is int and node.value in audit.COUNTS.values():
            node.value = next(before for before, after in audit.COUNTS.items() if after == node.value)
        elif isinstance(node, ast.Constant) and type(node.value) is str:
            node.value = node.value.replace("RIGHT80", "RIGHT70").replace("1755", "1555").replace("1601", "1401")
    assert ast.dump(new) == ast.dump(old)
