"""HH_261005 - Separate fake executable cannot import or connect to CARLA; it only returns synthetic scalar wheel reads."""

import importlib.util
from pathlib import Path
import sys
import time
from multiprocessing.connection import Connection

root = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("wheel_fake_workspace", root / "tools/workspace_python.py")
workspace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workspace)
workspace.configure(root)
from scripts.e2e import owned_parallel_wheel_reads_v2_20261005 as shared


class SyntheticWheelBackend:
    def __init__(self, proof, role, asset_admission):
        shared.require(asset_admission is None, "synthetic executable cannot receive actual CARLA asset admission")
        if proof.get("synthetic_fail_initialization") is True:
            raise RuntimeError("synthetic initialization failed")
        self.proof, self.role = proof, role

    def read(self, message, rpc_seconds):
        began = time.perf_counter_ns()
        time.sleep(self.proof.get("synthetic_delay_seconds", .025))
        if self.proof.get("synthetic_failed_role") == self.role:
            raise RuntimeError("synthetic wheel read failed")
        angle = -1.25 if self.role == "FL" else 2.5
        if self.proof.get("synthetic_nonfinite_role") == self.role:
            angle = float("nan")
        return angle, dict(wheel_call_started_monotonic_ns=began, wheel_call_finished_monotonic_ns=time.perf_counter_ns())


if __name__ == "__main__":
    shared.require(len(sys.argv) == 2, "private synthetic worker connection required")
    shared.worker_loop(Connection(int(sys.argv[1])), SyntheticWheelBackend, synthetic=True)
