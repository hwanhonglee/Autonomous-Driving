#!/usr/bin/env python3
"""HH_261001 - Bound-signal Yellow/Red normal-PID stop policy; original Red emergency stays intact."""

from contextlib import contextmanager
import json
import math

from scripts.e2e import capture_town12_j15824_expert9_v2stop_20261001 as inherited


lift = inherited.inherited.inherited
base, collector, scout = lift.base, lift.collector, lift.scout
POLICY_ID = "bound_signal_yellow_red_upstream12m_zero_target_v1"


def policy_record():
    return {"policy_id": POLICY_ID, "signal_opendrive_id": scout.SIGNAL_ID,
        "active_states": ["Yellow", "Red"], "front_bumper_distance_interval_m": [0., 12.],
        "normal_pid_target_speed_kmh": 0., "normal_brake_cap": .15,
        "normal_throttle_cap": .20, "original_basic_agent_red_emergency_unchanged": True,
        "traffic_light_states_written": False, "strict_boundary_gate_changed": False,
        "planned_target_restored_after_each_step": True,
        "scope": "Same-frame measured all-body-corner trigger-plane distance, before BasicAgent.run_step; not predictive signal timing."}


def should_stop(signal_state, distance):
    scout.require(signal_state in {"Red", "Yellow", "Green"}, "bound signal state unavailable before control")
    scout.require(math.isfinite(distance), "nonfinite bound signal distance before control")
    return signal_state in {"Red", "Yellow"} and 0. <= distance <= 12.


def apply_target_policy(agent, original_step, signal_state, distance, *args, **kwargs):
    planned = float(agent.get_local_planner()._target_speed)
    active = should_stop(signal_state, distance)
    try:
        if active:
            agent.set_target_speed(0.)
        control = original_step(*args, **kwargs)
    finally:
        if active:
            agent.set_target_speed(planned)
    return control, {"active": active, "planned_target_speed_kmh": planned,
        "effective_normal_pid_target_speed_kmh": 0. if active else planned,
        "signal_state": signal_state, "front_bumper_stop_distance_m": distance,
        "target_restored_after_step": True}


@contextmanager
def configured_policy():
    original_instrument = base.instrumented_collector

    @contextmanager
    def instrument(output, route, receipt):
        import agents.navigation.basic_agent as basic
        with original_instrument(output, route, receipt):
            parent_class = basic.BasicAgent
            original_observe, original_write = collector.traffic_light_observation, collector._write_json
            signal_id = int(receipt["controlling_signal"]["actor_id"])
            stop = receipt["controlling_signal"]["selected_stop_waypoint"]
            stop_x, stop_y, _ = stop["xyz_carla_m"]
            stop_angle = math.radians(stop["yaw_carla_deg"])

            class BoundSignalApproachAgent(parent_class):
                def run_step(self, *args, **kwargs):
                    world, vehicle = self._world, self._vehicle
                    snapshot = world.get_snapshot()
                    light = world.get_actor(signal_id)
                    scout.require(int(world.id) == receipt["world_id"] and light is not None and not light.is_dormant
                        and str(light.get_opendrive_id()) == scout.SIGNAL_ID,
                        "bound controlling light/world identity changed before expert control")
                    vertices = vehicle.bounding_box.get_world_vertices(vehicle.get_transform())
                    distance = min((stop_x - point.x) * math.cos(stop_angle) + (stop_y - point.y) * math.sin(stop_angle)
                                   for point in vertices)
                    state = str(light.get_state()).rsplit(".", 1)[-1]
                    scout.require(world.get_snapshot().frame == snapshot.frame,
                                  "signal target-policy geometry crossed physics frames")
                    control, record = apply_target_policy(self, super().run_step, state, distance, *args, **kwargs)
                    scout.require(world.get_snapshot().frame == snapshot.frame, "original BasicAgent step crossed physics frames")
                    self.town12_signal_approach = {**record, "policy_id": POLICY_ID, "frame": int(snapshot.frame),
                        "timestamp": float(snapshot.timestamp.elapsed_seconds), "signal_actor_id": signal_id,
                        "original_basic_agent_red_hazard": getattr(self, "town12_hazard", None),
                        "returned_control": collector.control_dict(control)}
                    with (output / "signal_approach_controls.jsonl").open("a", encoding="utf-8") as stream:
                        stream.write(json.dumps(self.town12_signal_approach, sort_keys=True, allow_nan=False) + "\n")
                    return control

            def observe(world, ego, agent, frame, *, basic_agent_run_step_this_frame):
                try:
                    observation = original_observe(world, ego, agent, frame,
                        basic_agent_run_step_this_frame=basic_agent_run_step_this_frame)
                except Exception:
                    # HH_261001 - Preserve the aborting snapshot separately; never invent an accepted training state.
                    try:
                        snapshot = world.get_snapshot()
                        scout.require(int(snapshot.frame) == frame, "aborting frame changed before snapshot preservation")
                        state, raw_snapshot = collector.acknowledged_snapshot_state(snapshot, ego, collector.WHEELBASE_M)
                        current_control = collector.control_dict(ego.get_control())
                        scout.require(world.get_snapshot().frame == frame, "abort control read crossed physics frame")
                        journal = output / "episode.partial/control_receipts.jsonl"
                        lines = journal.read_text().splitlines()
                        record = {"schema": "town12.signal_abort_snapshot.v1", "frame": int(frame),
                            "timestamp": float(snapshot.timestamp.elapsed_seconds), "recorded_training_state": False,
                            "native_states_journal_modified": False, "actor_id": int(ego.id), **state, **raw_snapshot,
                            "current_control": current_control,
                            "last_issued_control_receipt_before_abort": json.loads(lines[-1]) if lines else None,
                            "signal_approach_policy": getattr(agent, "town12_signal_approach", None),
                            "interpretation": "Uncommitted abort-frame evidence only; future cleanup command ACK remains in its original append-only journal."}
                        scout.write_new(output / "signal_abort_snapshot.json", record)
                    except Exception as error:
                        scout.write_new(output / "signal_abort_snapshot_error.json", {"error": f"{type(error).__name__}: {error}"})
                    raise
                record = getattr(agent, "town12_signal_approach", None) if basic_agent_run_step_this_frame else None
                scout.require(record is None or record["frame"] == frame, "policy/control witness is not same-frame")
                return {**observation, "town12_signal_approach_policy": record}

            def write(path, value):
                if path.name == "manifest.json":
                    value = {**value, "town12_bound_signal_approach_policy": policy_record()}
                original_write(path, value)

            try:
                basic.BasicAgent = BoundSignalApproachAgent
                collector.traffic_light_observation, collector._write_json = observe, write
                yield
            finally:
                basic.BasicAgent = parent_class
                collector.traffic_light_observation, collector._write_json = original_observe, original_write

    try:
        base.instrumented_collector = instrument
        yield
    finally:
        base.instrumented_collector = original_instrument


def main(argv=None):
    with configured_policy():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
