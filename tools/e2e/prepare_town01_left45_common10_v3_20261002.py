"""HH_261002 - Fresh V3 receipt provider only; the original scoped Common10 converter and native guards are unchanged."""
from __future__ import annotations

import ast
import copy
from pathlib import Path

from scripts.e2e import prepare_town01_left45_common10_20261002 as baseline
from scripts.e2e import qualify_town01_left45_common10_v3_20261002 as v3_authority


def converter_ast():
    v3_authority.require(v3_authority.sha(baseline.__file__) == v3_authority.PARENT_CONVERTER_SHA,
        "frozen V2 converter changed")
    original = ast.parse(Path(baseline.__file__).read_bytes())
    v3_authority.require(ast.dump(original.body[-1].test) == ast.dump(ast.parse('__name__=="__main__"', mode="eval").body),
        "exact original converter CLI guard required")
    original.body.pop(); changed = copy.deepcopy(original)
    matches = [alias for node in changed.body if isinstance(node, ast.ImportFrom) and node.module == "scripts.e2e"
        for alias in node.names if alias.name == "qualify_town01_left45_common10_20261002" and alias.asname == "authority"]
    v3_authority.require(len(matches) == 1, "one exact original authority import required")
    matches[0].name = "qualify_town01_left45_common10_v3_20261002"
    inverse = copy.deepcopy(changed)
    for node in inverse.body:
        if isinstance(node, ast.ImportFrom) and node.module == "scripts.e2e":
            for alias in node.names:
                if alias.name == "qualify_town01_left45_common10_v3_20261002": alias.name = "qualify_town01_left45_common10_20261002"
    v3_authority.require(ast.dump(inverse) == ast.dump(original), "another original converter/native/denial statement changed")
    return changed


exec(compile(ast.fix_missing_locations(converter_ast()), __file__, "exec"), globals())

if __name__ == "__main__": raise SystemExit(main())
