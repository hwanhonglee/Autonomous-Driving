"""HH_261002 - Fresh entire RIGHT697 provider only; generic native/weather/targets/history/denial guards stay frozen."""
from __future__ import annotations

import ast
import copy

from scripts.e2e import prepare_town01_left45_common10_v3_20261002 as previous
from scripts.e2e import qualify_town01_right80_common10_20261002 as declared_authority


def converter_ast():
    declared_authority.require(declared_authority.sha(previous.__file__) == declared_authority.PARENT_CONVERTER_SHA,
        "frozen LEFTv3 converter changed")
    original = previous.converter_ast(); changed = copy.deepcopy(original); changes = []; imports = 0
    for node in ast.walk(changed):
        if isinstance(node, ast.ImportFrom):
            for alias in node.names:
                if alias.name == "qualify_town01_left45_common10_v3_20261002":
                    changes.append((alias, "name", alias.name)); alias.name = "qualify_town01_right80_common10_20261002"; imports += 1
        if isinstance(node, ast.FunctionDef) and node.name == "bound_left_authority": changes.append((node, "name", node.name)); node.name = "bound_right_authority"
        if isinstance(node, ast.Name) and node.id == "bound_left_authority": changes.append((node, "id", node.id)); node.id = "bound_right_authority"
        if isinstance(node, ast.Attribute) and node.attr == "execute_reviewed_left45_conversion":
            changes.append((node, "attr", node.attr)); node.attr = "execute_reviewed_right80_conversion"
        if isinstance(node, ast.Constant) and type(node.value) is str:
            before = node.value
            node.value = before.replace("LEFT347", "RIGHT697").replace("LEFT", "RIGHT").replace("left45", "right80")
            node.value = node.value.replace("through772_all347", "through1472_all697")
            if before != node.value: changes.append((node, "value", before))
    declared_authority.require(imports == 1, "one exact RIGHT697 qualifier import required")
    inverse = copy.deepcopy(changed); nodes = list(ast.walk(changed)); inverse_nodes = list(ast.walk(inverse))
    for node, key, before in changes: setattr(inverse_nodes[nodes.index(node)], key, before)
    declared_authority.require(ast.dump(inverse) == ast.dump(original), "RIGHT697 changed original native/conversion/math/denial predicates")
    return changed


exec(compile(ast.fix_missing_locations(converter_ast()), __file__, "exec"), globals())
authority.require(authority is declared_authority, "current exact RIGHT697 authority required")


if __name__ == "__main__": raise SystemExit(main())
