#!/usr/bin/env python3
"""HH_260906 - Retain every previous v8 regression suite and add the terminal approach suites; run CPU-only without package changes."""
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
old = ROOT / "artifacts/training/2026-09-15/native_boundary_and_substeps/regression.xml"
paths = set()
for case in ET.parse(old).findall(".//testcase"):
    name = case.attrib["classname"].split(".")
    if name[-1].startswith("Test"):
        name = name[:-1]
    paths.add("/".join(name) + ".py")
paths.update(("tests/test_carla_terminal_approach_profile.py", "tests/test_carla_terminal_approach_integration.py",
              "tests/test_summarize_carla_terminal_approach.py",
              "artifacts/training/2026-09-15/terminal_approach_v9/test_v9_run_case.py"))
assert all((ROOT / path).is_file() for path in paths)
output = BASE / "regression.xml"
assert not output.exists()
environment = dict(os.environ, CUDA_VISIBLE_DEVICES="")
print(f"Historical closure plus new suites: {len(paths)} files", flush=True)
raise SystemExit(subprocess.call([sys.executable, "-m", "pytest", "-q", *sorted(paths), "--junitxml", str(output)], cwd=ROOT, env=environment))
