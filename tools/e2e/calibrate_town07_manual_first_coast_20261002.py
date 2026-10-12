"""HH_261002 - One actual manual-first zero-pedal response measurement, never TRAIN or goal evidence."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import random
import signal
import shutil

from scripts.e2e import collect_carla_vad_expert as capture
from scripts.e2e import calibrate_carla_low_speed_response as physics

SCHEMA = "carla.town07_manual_first_zero_pedal_calibration.20261002.v1"
SEED, DT, WARMUP_TICKS, COAST_TICKS = 2026102300, .05, 24, 900
ENTRY_MPS, MAX_SPEED_MPS, MAX_LAUNCH_SECONDS = 1.5, 2.5, 20.
FLOOR, RESERVE, QUOTA = 5*1024**3, 16*1024**2, 128*1024**2
CONTROL_KEYS = ("throttle", "brake", "steer", "hand_brake", "reverse", "gear", "manual_gear_shift")
WEATHER_FIELDS = ("cloudiness", "precipitation", "precipitation_deposits", "wind_intensity",
    "sun_azimuth_angle", "sun_altitude_angle", "fog_density", "fog_distance", "fog_falloff", "wetness")


def require(condition, message):
    if not condition: raise RuntimeError(message)


def weather_observation(value):
    observed={name:float(getattr(value,name)) for name in WEATHER_FIELDS}
    require(all(math.isfinite(number) for number in observed.values()),"finite actual weather fields required")
    return observed


def protocol():
    # HH_261002 - Declare an open-loop preparation, not an unchanged BasicAgent/PID control experiment.
    return dict(schema=SCHEMA, seed=SEED, maximum_attempts=1, automatic_retry=False,
        vehicle="vehicle.toyota.prius", map="Town07", weather="ClearNoon", wheelbase_m=2.85,
        physics_hz=20., fixed_delta_seconds=DT, substepping=True, max_substep_delta_time=.01, max_substeps=10,
        bootstrap_ticks=1, stationary_warmup_ticks=WARMUP_TICKS,
        launch_throttle=.11, launch_handoff_mps=.5, maximum_handoff_seconds=8.,
        post_handoff_throttle_ramp_per_second=.025, post_handoff_cap=.20,
        maximum_launch_seconds=MAX_LAUNCH_SECONDS, measured_coast_entry_threshold_mps=ENTRY_MPS,
        entry_speed_reset_or_clipping=False, coast_ticks=COAST_TICKS, coast_duration_seconds=45.,
        coast_control=dict(throttle=0., brake=0., steer=0., hand_brake=False, reverse=False, gear=1, manual_gear_shift=True),
        preparation_controller="NEW disclosed open-loop fixed pedal ramp; no PID or BasicAgent",
        BasicAgent_constructor_or_PID_initialization_claimed=False,
        manual_initialization_checker="frozen checked_first_gear_initialization_observation(controller=None)",
        physics_snapshots=["initial", "coast_entry", "coast_end"],
        maximum_actual_speed_mps=MAX_SPEED_MPS, maximum_actual_abs_route_CTE_m=.75,
        physical_decoder_acceleration_mps2=[-2.9, 2.9], runtime_speed_rate_mps2=[-6., 3.],
        quality_violation_policy="retain all fixed coast ticks unless separate operational safety abort; report FAIL",
        cleanup_policy="full brake only after the 900th coast observation, or explicit safety/error abort",
        cleanup_journal="cleanup_states.jsonl", camera_capture=False, learned_control=False, goal_stop=False,
        training_data_approved=False, vehicle_control_approved=False,
        comparison_scope="manual1 pure coast; old AUTO coast at a different entry speed is context, not matched A/B")


@dataclass
class OpenLoopPreparation:
    """HH_261002 - Use actual time/v0 for two fixed transitions, never a target-speed feedback loop."""
    started_at: float | None = None
    handoff_at: float | None = None
    previous_time: float | None = None

    def throttle(self, speed, timestamp):
        require(type(speed) in (int, float) and type(timestamp) in (int, float)
            and math.isfinite(speed) and math.isfinite(timestamp) and speed >= 0, "finite measured speed/time required")
        if self.started_at is None: self.started_at = timestamp
        require(timestamp >= self.started_at and (self.previous_time is None or timestamp>=self.previous_time), "native preparation time reversed")
        self.previous_time=timestamp
        if self.handoff_at is None and speed >= .5: self.handoff_at = timestamp
        require(self.handoff_at is not None or timestamp-self.started_at < 8., "measured 0.5 m/s launch handoff timed out")
        require(timestamp-self.started_at < MAX_LAUNCH_SECONDS, "measured 1.5 m/s coast entry timed out")
        return .11 if self.handoff_at is None else min(.20, .11+.025*(timestamp-self.handoff_at+DT))


def quality(rows):
    """HH_261002 - Report original scalar dv/dt separately from actor API acceleration and preserve all preparation."""
    failures, intervals = [], []
    for row in rows:
        require(type(row["frame"]) is int and row["frame"]>0
            and all(type(row[name]) in (int,float) and math.isfinite(row[name]) for name in ("timestamp","vx","vy")),
            "finite actual scalar fields and strict frame integer required")
    for before, after in zip(rows, rows[1:]):
        elapsed = after["timestamp"]-before["timestamp"]
        require(after["frame"] == before["frame"]+1 and abs(elapsed-DT) <= 1e-6, "full native chain has a gap")
        rate = (math.hypot(after["vx"],after["vy"])-math.hypot(before["vx"],before["vy"]))/elapsed
        bounds = []
        if not -2.9 <= rate <= 2.9: bounds.append("physical_decoder_2_9")
        if not -6. <= rate <= 3.: bounds.append("runtime_speed_rate_3_minus6")
        row = dict(from_frame=before["frame"], to_frame=after["frame"], phase=after["capture_phase"],
            speed_rate_mps2=rate, failed_bounds=bounds)
        intervals.append(row)
        if bounds: failures.append(row)
    return dict(status="PASS_SCALAR_ONLY" if intervals and not failures else "FAIL",
        assessed_intervals=len(intervals), failed_interval_count=len(failures), failed_intervals=failures,
        minimum_native_speed_rate_mps2=min((row["speed_rate_mps2"] for row in intervals),default=None),
        maximum_native_speed_rate_mps2=max((row["speed_rate_mps2"] for row in intervals),default=None),
        coast_interval_count=sum(row["phase"] == "coast" for row in intervals),
        coast_failed_interval_count=sum(row["phase"] == "coast" for row in failures),
        scalar_speed_reference="unchanged collector rear-base hypot(vx,vy), including the original yaw-rate lever-arm term",
        actor_world_planar_speed_is_separate_raw_reference=True,
        all_preparation_retained=True, cleanup_excluded_from_quality_but_raw_retained=True,
        actual_API_world_acceleration_is_not_this_speed_difference=True, training_data_approved=False)


def control(carla, throttle=0., brake=0.):
    return carla.VehicleControl(throttle=throttle, brake=brake, steer=0., hand_brake=False,
        reverse=False, gear=1, manual_gear_shift=True)


def measured_control_is_manual_first(value, *, coast=False):
    require(set(value) == set(CONTROL_KEYS) and value["manual_gear_shift"] is True
        and type(value["gear"]) is int and value["gear"] == 1
        and value["hand_brake"] is False and value["reverse"] is False and value["steer"] == 0.,
        "actual manual1/zero-steering/flags differ")
    if coast: require(value["throttle"] == value["brake"] == 0., "actual coast pedals are not zero")


def append(stream, value):
    stream.write(json.dumps(value,allow_nan=False,sort_keys=True)+"\n"); stream.flush()


def run(output, plan, verify_owner):
    """HH_261002 - A private owned simulator is mandatory; no reload, foreign attach, cameras or learned model."""
    from scripts.e2e import run_owned_town07_signed_recovery_v3_20261001 as manual
    manual.verify_frozen_dependencies()
    import carla
    require(str(carla.__file__).startswith(str(Path(plan["python_api"]))+"/"), "foreign CARLA API")
    require(plan["protocol"] == protocol(), "declared calibration protocol differs")
    require(type(SEED) is int and 0 <= SEED < 2**32, "legal fixed uint32 seed required")
    random.seed(SEED)
    verify_owner()
    require(output == output.resolve(),"canonical output without symlink ancestors required")
    require(not output.exists() and not output.is_symlink(), "fresh create-only calibration output required")
    output.mkdir()
    client=carla.Client("127.0.0.1",2440); client.set_timeout(15.)
    world=client.get_world(); carla_map=world.get_map()
    require(carla_map.name.split("/")[-1] == "Town07", "actual map is not Town07; reload forbidden")
    _, xml=capture.verify_live_opendrive_sha256(carla_map,plan["xodr_sha256"])
    require(not any(actor.type_id.startswith(("vehicle.","sensor.","walker.")) for actor in world.get_actors()),
        "private calibration requires an empty owned world")
    original=world.get_settings(); original_weather=world.get_weather()
    rows,cleanup_rows,actors,streams=[],[],[],[]
    record=dict(schema=SCHEMA,status="RUNNING",protocol=protocol(),training_data_approved=False,
        vehicle_control_approved=False,physics_snapshots={},initialization_witnesses=[],errors=[],cleanup_errors=[])
    events=capture.EventRecorder(); ego=transport=None; stop=[False]; handlers={}; previous_frame=None
    def interrupted(signum,_): stop[0]=True
    try:
        for signum in (signal.SIGINT,signal.SIGTERM): handlers[signum]=signal.signal(signum,interrupted)
        settings=world.get_settings(); settings.synchronous_mode=True; settings.fixed_delta_seconds=DT
        settings.substepping=True; settings.max_substep_delta_time=.01; settings.max_substeps=10
        world.apply_settings(settings)
        physics.validate_world_timing(world.get_settings())
        observed_settings=capture._settings_dict(world.get_settings())
        require(observed_settings == dict(capture._settings_dict(original),synchronous_mode=True,
            fixed_delta_seconds=DT,substepping=True,max_substep_delta_time=.01,max_substeps=10), "undeclared world setting change")
        world.set_weather(carla.WeatherParameters.ClearNoon)
        requested_weather=weather_observation(carla.WeatherParameters.ClearNoon)
        require(weather_observation(world.get_weather())==requested_weather,"actual ClearNoon weather differs")
        record["actual_ClearNoon_weather"]=requested_weather
        route_payload=Path(plan["route_file"]).read_bytes()
        require(hashlib.sha256(route_payload).hexdigest() == plan["route_sha256"], "route bytes changed")
        route=json.loads(route_payload); capture.load_route(Path(plan["route_file"]))
        with (output/"route.json").open("xb") as stream: stream.write(route_payload)
        with (output/"active_map_opendrive.xodr").open("xb") as stream: stream.write(xml)
        spawn=capture.apply_spawn_z_offset(capture.shift_transform_local_x(route["start_carla_transform"],1.425),.5)
        blueprint=world.get_blueprint_library().find("vehicle.toyota.prius")
        blueprint.set_attribute("role_name","autoware_e2e_manual_first_coast_nontraining")
        ego=world.try_spawn_actor(blueprint,capture._carla_transform(carla,spawn))
        require(ego is not None,"single fixed actor spawn failed; no retry")
        actors.append(ego); record.update(actor_id=int(ego.id),actor_body_geometry=capture.actor_body_geometry(ego),
            requested_actor_spawn_carla=spawn,actual_settings=observed_settings,
            client_version=client.get_client_version(),server_version=client.get_server_version())
        record["active_map_opendrive"]=dict(status="RECORDED",source="carla.Map.to_opendrive",
            file="active_map_opendrive.xodr",sha256=hashlib.sha256(xml).hexdigest(),size_bytes=len(xml),map_name=carla_map.name)
        for kind,callback in (("sensor.other.collision",events.on_collision),("sensor.other.lane_invasion",events.on_lane_invasion)):
            actor=world.spawn_actor(world.get_blueprint_library().find(kind),carla.Transform(),attach_to=ego)
            actors.append(actor); actor.listen(callback)
        native_stream=(output/"states.jsonl").open("x"); cleanup_stream=(output/"cleanup_states.jsonl").open("x")
        ack_stream=(output/"control_receipts.jsonl").open("x"); streams.extend((native_stream,cleanup_stream,ack_stream))
        transport=capture.AcknowledgedControlTransport(client,carla,ego,lambda:int(world.get_snapshot().frame),lambda row:append(ack_stream,row))
        previous_frame=int(world.get_snapshot().frame)
        def advance(command,phase,*,cleanup=False):
            nonlocal previous_frame
            verify_owner()
            if not cleanup: require(not stop[0],"calibration interrupted; preserve raw partial")
            require(shutil.disk_usage(output).free >= FLOOR+RESERVE,"fixed disk closing reserve reached")
            require(sum(p.stat().st_size for p in output.iterdir() if p.is_file())+RESERVE <= QUOTA,"calibration quota reached")
            before=int(world.get_snapshot().frame)
            require(before == previous_frame,"world advanced outside owned advance")
            ack=transport.send(command,phase,expected_before_frame=before)
            frame=world.tick(15.); snapshot=world.get_snapshot()
            require(frame == before+1 == int(snapshot.frame),"native world tick skipped or duplicated")
            state,raw=capture.acknowledged_snapshot_state(snapshot,ego,2.85)
            control_before=int(world.get_snapshot().frame); actual_control=ego.get_control()
            control_after=int(world.get_snapshot().frame)
            observed=capture.AcknowledgedControlTransport.checked_control(actual_control)
            alignment=transport.compare_observation(actual_control,ack,frame,control_before,control_after)
            prior=cleanup_rows[-1] if cleanup_rows else rows[-1] if rows else None
            row=dict(frame=frame,timestamp=float(snapshot.timestamp.elapsed_seconds),capture_phase=phase,**state,**raw,
                current_control=observed,requested_control=capture.control_dict(command),control_alignment=alignment,
                receipt_sequence=ack["sequence"],events_seen=events.snapshot(),training_data_approved=False,
                actor_world_planar_speed_mps=math.hypot(raw["world_velocity_carla"][0],raw["world_velocity_carla"][1]),
                actual_API_world_acceleration_planar_mps2=math.hypot(raw["world_acceleration_carla"][0],raw["world_acceleration_carla"][1]),
                scalar_speed_rate_mps2=None)
            if prior is not None:
                elapsed=row["timestamp"]-prior["timestamp"]
                row["scalar_speed_rate_mps2"]=(math.hypot(row["vx"],row["vy"])-math.hypot(prior["vx"],prior["vy"]))/elapsed if elapsed>0 else None
            projection_error=None
            try:
                arc,cte,segment=capture.bounded_route_projection(route["route"],row["x"],row["y"],
                    prior.get("route_progress_m",0.) if prior is not None else 0.,1.)
                row.update(route_progress_m=arc,route_cte_m=cte,route_segment_index=segment)
            except Exception as error:
                projection_error=error; row["route_projection_error"]=type(error).__name__+": "+str(error)
            (cleanup_rows if cleanup else rows).append(row); append(cleanup_stream if cleanup else native_stream,row)
            previous_frame=frame
            if projection_error is not None: raise projection_error
            if prior is not None: require(abs(elapsed-DT)<=1e-6,"actual native timestamp cadence changed")
            require(alignment["status"] == "PASS","actual ACK/command/native control alignment failed")
            measured_control_is_manual_first(observed,coast=phase=="coast")
            if not cleanup:
                require(math.hypot(row["vx"],row["vy"]) <= MAX_SPEED_MPS
                    and row["actor_world_planar_speed_mps"] <= MAX_SPEED_MPS,"actual speed exceeds 2.5 m/s")
                require(cte <= .75,"actual route CTE exceeds .75 m")
                require(not any(row["events_seen"]),"actual collision/lane invasion; safety abort retained")
            return row
        observed=advance(control(carla,brake=1.),"bootstrap")
        witness={}
        manual.checked_first_gear_initialization_observation(world,ego,observed["frame"],witness,controller=None)
        record["initialization_witnesses"].append(witness)
        for _ in range(WARMUP_TICKS): observed=advance(control(carla,brake=1.),"stationary_warmup")
        witness={}
        manual.checked_first_gear_initialization_observation(world,ego,observed["frame"],witness,controller=None)
        record["initialization_witnesses"].append(witness)
        require(math.hypot(observed["vx"],observed["vy"]) <= .05,"actual fresh manual1 actor did not settle")
        def snapshot(name,row):
            before=int(world.get_snapshot().frame)
            value=physics.physics_snapshot(ego.get_physics_control())
            after=int(world.get_snapshot().frame)
            require(before == after == row["frame"],"physics snapshot is not bound to current actual frame")
            record["physics_snapshots"][name]=dict(frame=before,timestamp=row["timestamp"],
                actual_control=row["current_control"],physics=value,internal_engine_or_wheel_RPM_measured=False)
        snapshot("initial",observed)
        preparation=OpenLoopPreparation()
        while math.hypot(observed["vx"],observed["vy"]) < ENTRY_MPS:
            throttle=preparation.throttle(math.hypot(observed["vx"],observed["vy"]),observed["timestamp"])
            observed=advance(control(carla,throttle=throttle),"open_loop_launch")
        record["entry"]=dict(frame=observed["frame"],timestamp=observed["timestamp"],
            measured_rear_base_speed_mps=math.hypot(observed["vx"],observed["vy"]),
            measured_actor_world_speed_mps=observed["actor_world_planar_speed_mps"],
            actual_control=observed["current_control"],first_observed_threshold=True,threshold_mps=ENTRY_MPS,
            measured_speed_not_reset_or_clipped=True,preparation_handoff_at=preparation.handoff_at)
        snapshot("coast_entry",observed)
        for _ in range(COAST_TICKS): observed=advance(control(carla),"coast")
        snapshot("coast_end",observed)
        require(capture._settings_dict(world.get_settings())==observed_settings,"world settings changed during measurement")
        require(weather_observation(world.get_weather())==requested_weather,"weather changed during measurement")
        record["measurement_completed"]=True
    except BaseException as error:
        record["errors"].append(type(error).__name__+": "+str(error))
    finally:
        # HH_261002 - The first cleanup full brake follows fixed data completion or an explicit retained abort.
        record["cleanup_reason"]="after_fixed_900_coast_ticks" if record.get("measurement_completed") else "retained_measurement_error_or_safety_abort"
        if ego is not None and transport is not None:
            try:
                for _ in range(60):
                    stopped=advance(control(carla,brake=1.),"cleanup_full_brake",cleanup=True)
                    if math.hypot(stopped["vx"],stopped["vy"]) <= .05: break
                else: raise RuntimeError("cleanup full-brake did not reach measured .05 m/s within 60 ticks")
            except BaseException as error:
                record["cleanup_errors"].append(type(error).__name__+": "+str(error))
                try: transport.send(control(carla,brake=1.),"cleanup_error_full_brake",expected_before_frame=int(world.get_snapshot().frame))
                except Exception as second: record["cleanup_errors"].append(type(second).__name__+": "+str(second))
        for signum,handler in handlers.items(): signal.signal(signum,handler)
        for stream in streams: stream.close()
        for actor in reversed(actors):
            try:
                if hasattr(actor,"stop"): actor.stop()
                actor.destroy()
            except Exception as error: record["cleanup_errors"].append(type(error).__name__+": "+str(error))
        try: world.apply_settings(original); world.set_weather(original_weather)
        except Exception as error: record["cleanup_errors"].append(type(error).__name__+": "+str(error))
        try: measured_quality=quality(rows)
        except Exception as error: measured_quality=dict(status="NOT_PROVEN",error=type(error).__name__+": "+str(error))
        try: restored=capture._settings_dict(world.get_settings()) == capture._settings_dict(original)
        except Exception as error:
            restored=False; record["cleanup_errors"].append(type(error).__name__+": "+str(error))
        if not restored: record["cleanup_errors"].append("original world settings not restored")
        try: weather_restored=weather_observation(world.get_weather())==weather_observation(original_weather)
        except Exception as error:
            weather_restored=False; record["cleanup_errors"].append(type(error).__name__+": "+str(error))
        if not weather_restored:record["cleanup_errors"].append("original weather not restored")
        record.update(native_state_count=len(rows),cleanup_state_count=len(cleanup_rows),
            native_motion_quality=measured_quality,events=events.snapshot(),
            actual_control_transport_receipt_count=len(transport.receipts) if transport is not None else 0,
            original_world_settings_restored=restored,original_weather_restored=weather_restored)
        record["status"]="PASS_MEASURED_CALIBRATION_NOT_TRAIN" if record.get("measurement_completed") is True \
            and not record["errors"] and not record["cleanup_errors"] and measured_quality["status"] == "PASS_SCALAR_ONLY" else "FAIL_RETAINED"
        with (output/"result.json").open("x") as stream: json.dump(record,stream,indent=2,allow_nan=False)
        manifest=dict(schema=SCHEMA,status=record["status"],training_data_approved=False,vehicle_control_approved=False,
            coordinate_contract=dict(wheelbase_m=2.85),capture_contract=dict(protocol=protocol(),
                actor_body_geometry=record.get("actor_body_geometry"),active_map_opendrive=record.get("active_map_opendrive")),
            raw_file_sha256={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in output.iterdir() if path.is_file()})
        with (output/"manifest.json").open("x") as stream: json.dump(manifest,stream,indent=2,allow_nan=False)
    return record
