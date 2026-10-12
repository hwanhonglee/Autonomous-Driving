"""HH_261005 - Fresh-process genuine read-only CARLA wheel backend, never selected by the synthetic executable."""

import importlib.util
from pathlib import Path
import sys
import time
from multiprocessing.connection import Connection

root = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("wheel_workspace", root / "tools/workspace_python.py")
workspace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workspace)
workspace.configure(root)
from scripts.e2e import owned_parallel_wheel_reads_v1_20261005 as shared


class CarlaWheelBackend:
    def __init__(self, proof, role):
        shared.require(proof["synthetic_only"] is False, "synthetic proof cannot enter real CARLA executable")
        shared.require(shared.digest(proof["carla_egg_path"]) == proof["carla_egg_sha256"]
            and sys.version_info[:2] == (3, 10), "declared CARLA Python ABI/asset required")
        sys.path.insert(0, proof["carla_egg_path"])
        import carla
        shared.require(str(carla.__file__).startswith(proof["carla_egg_path"] + "/"), "unsealed CARLA module origin")
        self.carla, self.proof, self.role = carla, proof, role
        self.client = carla.Client(proof["host"], proof["port"])
        self.client.set_timeout(shared.RPC_SECONDS)

    def read(self, message, rpc_seconds):
        # HH_261005 - No map API, settings change, tick, actor creation or actuation exists in this backend.
        world = self.client.get_world()
        shared.require(world.id == self.proof["world_id"], "worker world/map epoch changed")
        snapshot = world.get_snapshot()
        while snapshot.frame < message["frame"] and time.perf_counter_ns() < message["deadline_monotonic_ns"]:
            time.sleep(.001)
            snapshot = world.get_snapshot()
        shared.require(snapshot.frame == message["frame"], "worker snapshot is not exact requested parent frame")
        actor = world.get_actor(self.proof["actor_id"])
        shared.require(actor is not None and actor.id == self.proof["actor_id"]
            and actor.type_id == self.proof["type_id"] and isinstance(actor, self.carla.Vehicle), "owned ego actor type/identity changed")
        began = time.perf_counter_ns()
        angle = actor.get_wheel_steer_angle(getattr(self.carla.VehicleWheelLocation, self.role + "_Wheel"))
        ended = time.perf_counter_ns()
        shared.require(world.id == self.proof["world_id"] and world.get_snapshot().frame == message["frame"]
            and self.client.get_world().id == self.proof["world_id"]
            and actor.id == self.proof["actor_id"] and actor.type_id == self.proof["type_id"], "worker frame/ego/epoch changed during read")
        return angle, dict(wheel_call_started_monotonic_ns=began, wheel_call_finished_monotonic_ns=ended)


if __name__ == "__main__":
    shared.require(len(sys.argv) == 2, "private dedicated worker connection required")
    shared.worker_loop(Connection(int(sys.argv[1])), CarlaWheelBackend, synthetic=False)
