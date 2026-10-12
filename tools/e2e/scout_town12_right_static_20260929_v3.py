#!/usr/bin/env python3
"""HH_260929 - Reuse the pinned immutable Town12 map during nominal sweeps."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from scripts.e2e import scout_town12_right_static_20260929 as base
from scripts.e2e import scout_town12_right_static_20260929_v2 as v2


ROOT = Path(__file__).resolve().parents[2]
PREREG = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static/preregister_v3.json"
OUTPUT = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static/report_v3.json"
V2_STOP = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static/v2_stop.json"


class CachedCarlaMap:
    """HH_260929 - Return one map only when every requested XODR byte matches."""

    def __init__(self, carla_module: object, expected_sha256: str):
        self._carla = carla_module
        self._expected_sha256 = expected_sha256
        self._map = None
        self.requests = 0

    def __getattr__(self, name: str) -> object:
        return getattr(self._carla, name)

    def Map(self, name: str, xodr: str) -> object:
        base.require(isinstance(xodr, str) and
                     hashlib.sha256(xodr.encode("utf-8")).hexdigest() ==
                     self._expected_sha256,
                     "cached map request differs from pinned Town12 XODR")
        self.requests += 1
        if self._map is None:
            self._map = self._carla.Map(name, xodr)
        return self._map


def preregistered_inputs() -> dict:
    manifest = base.preregistered_inputs()
    base.require(PREREG.is_file() and not PREREG.is_symlink(),
                 "v3 preregistration is missing")
    plan = json.loads(PREREG.read_text())
    base.require(plan.get("schema") == "portable_e2e.town12_right_static_preregister.v3" and
                 plan.get("output_path") == str(OUTPUT.relative_to(ROOT)) and
                 plan.get("base_preregister_sha256") == base.digest(base.PREREG) and
                 plan.get("v2_preregister_sha256") == base.digest(v2.PREREG) and
                 plan.get("base_search_script_sha256") == base.digest(Path(base.__file__)) and
                 plan.get("v2_script_sha256") == base.digest(Path(v2.__file__)) and
                 plan.get("v3_script_sha256") == base.digest(Path(__file__)) and
                 plan.get("map_reuse_only") is True and
                 plan.get("threshold_change") is False and
                 plan.get("scope_change") is False,
                 "v3 preregistration changed")
    base.require(not OUTPUT.exists() and not OUTPUT.is_symlink() and
                 OUTPUT.parent.is_dir() and not OUTPUT.parent.is_symlink(),
                 "v3 output must be new in a regular directory")
    if v2.OUTPUT.is_file() and not v2.OUTPUT.is_symlink():
        report = json.loads(v2.OUTPUT.read_text())
        base.require(report.get("schema") == "portable_e2e.town12_right_static_scout.v2" and
                     report.get("preregistration_sha256") == base.digest(v2.PREREG) and
                     report.get("counts", {}).get("physical_turn_pass") ==
                     len(report.get("physical_pass_routes", [])),
                     "v2 output is incomplete or not pinned")
    else:
        base.require(V2_STOP.is_file() and not V2_STOP.is_symlink(),
                     "v3 requires completed v2 output or a documented v2 stop")
        stop = json.loads(V2_STOP.read_text())
        pid = stop.get("pid")
        base.require(stop.get("schema") == "portable_e2e.town12_v2_stop.v1" and
                     stop.get("status") == "STOPPED_WITHOUT_REPORT" and
                     plan.get("v2_stop_sha256") == base.digest(V2_STOP) and
                     pid == plan.get("v2_stop_pid") == 1850015 and
                     stop.get("process_start_ticks") ==
                     plan.get("v2_stop_start_ticks") == 275552608 and
                     stop.get("termination_signal") == "SIGTERM" and
                     stop.get("post_stop_proc_path_absent") is True and
                     stop.get("post_stop_ps_entry_absent") is True and
                     stop.get("base_preregistration_sha256") == base.digest(base.PREREG) and
                     stop.get("v2_preregistration_sha256") == base.digest(v2.PREREG) and
                     stop.get("base_script_sha256") == base.digest(Path(base.__file__)) and
                     stop.get("v2_script_sha256") == base.digest(Path(v2.__file__)) and
                     stop.get("base_plan_sha256") == base.digest(base.PREREG.parent / "PLAN.md") and
                     stop.get("v2_plan_sha256") == base.digest(base.PREREG.parent / "PLAN_V2.md") and
                     stop.get("source_xodr_sha256") == base.digest(base.SOURCE_XODR) and
                     stop.get("log_path") == "/tmp/town12_static_v2_20260929.log" and
                     stop.get("log_size_bytes") == 0 and
                     stop.get("log_sha256") == base.digest(Path(stop["log_path"])) and
                     stop.get("report_absent") is True and not v2.OUTPUT.exists() and
                     type(pid) is int and pid > 0 and
                     stop.get("process_exit_observed") is True and
                     isinstance(stop.get("process_start_ticks"), int) and
                     stop["process_start_ticks"] > 0 and
                     not Path(f"/proc/{pid}").exists(),
                     "v2 stop evidence is incomplete or PID remains active")
    return manifest


def main() -> None:
    manifest = preregistered_inputs()
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    cached = CachedCarlaMap(carla, base.digest(base.SOURCE_XODR))
    base.nominal.provisional_nominal_clearance = v2.guarded_clearance
    report = base.scan(cached, GlobalRoutePlanner, manifest)
    report["schema"] = "portable_e2e.town12_right_static_scout.v3"
    report["preregistration_sha256"] = base.digest(PREREG)
    report["search_source_sha256"] = base.digest(Path(__file__))
    report["base_search_source_sha256"] = base.digest(Path(base.__file__))
    report["typed_exception_source_sha256"] = base.digest(Path(v2.__file__))
    report["map_reuse"] = {
        "same_xodr_sha256_required_for_every_request": True,
        "map_requests": cached.requests,
        "map_constructions": 1,
        "threshold_change": False,
        "scope_change": False,
    }
    base.require(not OUTPUT.exists() and not OUTPUT.is_symlink(),
                 "registered v3 output appeared during scan")
    OUTPUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["counts"], indent=2))


if __name__ == "__main__":
    main()
