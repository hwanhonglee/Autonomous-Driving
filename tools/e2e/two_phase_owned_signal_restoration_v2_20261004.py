"""HH_261004 - Restore original owned Town03 signals with separate frozen-state and natural-unfreeze proofs.

This fresh protocol never edits an old fixture or reclassifies its failure.
Constructor, installation, per-decision hazard checks and exact20Hz fresh-tick
guards are inherited. Only a future explicitly owned capture may activate the
new subclass. There is no stand-alone simulator connection or actuation entry.
"""

from contextlib import contextmanager
import ast
import copy
import hashlib
import inspect
import json
import math
from pathlib import Path
import textwrap

from . import collect_temporal_route_joint_town03_right_v2_20261002 as previous


SCHEMA="portable_e2e.owned_town03_two_phase_signal_restoration.20261004.v2"
EXPECTED_LIGHT_COUNT=38
PARENT_SHA256="3473cbb37d771137a01459d7d634a4ce372e551059043904dd0c538bce0ffa04"
# HH_261004 - Correct a cardinality/actor-ID confusion using immutable actual failed inventories, never relabel those failures.
FROZEN_V1_SOURCE_SHA256="7e769ab576b9ca65d6b463c6062b2d807e51e5f11463d43188cba9595f590005"
FROZEN_V1_TEST_SHA256="18d3ad56aefdcbaa04168f8fb4502c178755c96243f3f0071da9849c5b2e082c"
INVENTORY_EVIDENCE_PINS={
    "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v2/episode.partial/owned_green_signal_fixture.json":
        "897edb2945e5449489388405635248315dc3e83e31c8720edc5bbc50f1501f21",
    "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v3/episode.partial/owned_green_signal_fixture.json":
        "ad3058f013a411be1d551f43e7db3e48f493f85fd51740a63a4127f4214e6f59",
}
INVENTORY_ADMISSION_SCHEMA=SCHEMA+".complete_original_Town03_inventory"
_active=False


def _verify_inventory_declaration():
    # HH_261004 - Seal the observed count, not the first actor ID; fresh server generations bind their own complete live IDs.
    root=previous.ROOT
    old_pins={"tools/e2e/two_phase_owned_signal_restoration_v1_20261004.py":FROZEN_V1_SOURCE_SHA256,
        "learning/tests/test_two_phase_owned_signal_restoration_v1_20261004.py":FROZEN_V1_TEST_SHA256}
    for name,sha in {**old_pins,**INVENTORY_EVIDENCE_PINS}.items():
        path=root/name
        previous.frozen._require(path.is_file() and not path.is_symlink() and path.resolve()==path
            and path.stat().st_size<=262144 and hashlib.sha256(path.read_bytes()).hexdigest()==sha,
            "frozen count38 declaration or original failed v1 evidence changed: "+name)
        if name in INVENTORY_EVIDENCE_PINS:
            record=json.loads(path.read_text(encoding="utf-8"))
            rows=record.get("original_actor_ids")
            previous.frozen._require(record.get("state")=="RESTORE_FAILED" and type(rows) is list
                and len(rows)==EXPECTED_LIGHT_COUNT and all(type(row) is dict
                    and type(row.get("actor_id")) is int and row["actor_id"]>0 for row in rows),
                "actual original failed Town03 inventory must retain all38 positive live IDs")
            ids=[row["actor_id"] for row in rows]
            previous.frozen._require(ids==sorted(set(ids)),"sealed original38 IDs must be ordered and unique")


def verify_frozen():
    _verify_inventory_declaration()
    path=Path(previous.__file__)
    previous.frozen._require(path.is_file() and not path.is_symlink() and path.resolve()==path
        and hashlib.sha256(path.read_bytes()).hexdigest()==PARENT_SHA256,"original signal fixture source changed")
    previous.verify_frozen()


def _inherited_restore_settings(self,original_settings):
    # HH_261004 - Compile the exact original finally statements with their original globals; no settings guard is rewritten.
    verify_frozen()
    method=ast.parse(textwrap.dedent(inspect.getsource(TwoPhaseGreenSignalFixture.__bases__[0].restore))).body[0]
    tries=[node for node in method.body if isinstance(node,ast.Try)]
    previous.frozen._require(len(tries)==1 and len(tries[0].finalbody)==2,"one exact original settings-finally suite required")
    body=copy.deepcopy(tries[0].finalbody)
    function=ast.FunctionDef(name="restore_original_settings",args=ast.arguments(posonlyargs=[],
        args=[ast.arg(arg="self"),ast.arg(arg="original_settings")],kwonlyargs=[],kw_defaults=[],defaults=[]),
        body=body,decorator_list=[])
    previous.frozen._require(ast.dump(ast.Module(body=body,type_ignores=[]))
        ==ast.dump(ast.Module(body=tries[0].finalbody,type_ignores=[])),"original finally statements changed")
    namespace=dict(TwoPhaseGreenSignalFixture.__bases__[0].restore.__globals__)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),
        str(Path(__file__))+":exact_original_settings_finally","exec"),namespace)
    namespace["restore_original_settings"](self,original_settings)


class TwoPhaseGreenSignalFixture(previous.GreenSignalFixture):
    """HH_261004 - Do not require a naturally advancing unfrozen signal to retain its prior phase on the next tick."""

    def __init__(self,*args,**kwargs):
        # HH_261004 - Admit the complete actual original inventory before install can mutate any signal.
        verify_frozen()
        super().__init__(*args,**kwargs)
        self._original_actor_ids=tuple(previous.actor_id(actor) for actor in self.actors)
        previous.frozen._require(len(self._original_actor_ids)==EXPECTED_LIGHT_COUNT
            and self._original_actor_ids==tuple(sorted(set(self._original_actor_ids))),
            "declared Town03 requires exactly38 original live traffic actors before mutation")
        self.record.update(restoration_inventory_admission_schema=INVENTORY_ADMISSION_SCHEMA,
            restoration_expected_light_count=EXPECTED_LIGHT_COUNT,
            restoration_complete_original_actor_ids=list(self._original_actor_ids),
            restoration_count_declaration_evidence_sha256=dict(INVENTORY_EVIDENCE_PINS),
            mistaken_v1_43_count_failure_preserved=True,
            original_actor_ID_43_not_misinterpreted_as_count=True)
        self._owned_empty_inventory()

    def _owned_empty_inventory(self):
        previous.assert_owned_capture(self.args)
        previous.collector._preflight_exclusive(self.world)
        previous.collector._preflight_no_walkers(self.world)
        ids=tuple(sorted(previous.actor_id(actor) for actor in self.world.get_actors().filter("*traffic_light*")))
        expected=tuple(previous.actor_id(actor) for actor in self.actors)
        previous.frozen._require(len(expected)==EXPECTED_LIGHT_COUNT and ids==expected,
            "all38 unchanged original live traffic actors required")
        # HH_261004 - A count-only replacement cannot pass: bind every live/saved/recorded ID to the getter-only constructor census.
        previous.frozen._require(expected==self._original_actor_ids
            and tuple(previous.actor_id(saved[0]) for saved in self.saved)==self._original_actor_ids
            and tuple(row["actor_id"] for row in self.record["original_actor_ids"])==self._original_actor_ids
            and self.record["restoration_complete_original_actor_ids"]==list(self._original_actor_ids),
            "complete live/saved/recorded original38 actor membership changed")

    def _readback(self,key):
        # HH_261004 - Retain every getter result, including failed actors, before checking the complete same-frame phase.
        self._owned_empty_inventory()
        frame=self.world.get_snapshot().frame
        rows=[];self.record[key]=rows
        for light in self.actors:
            row=dict(actor_id=previous.actor_id(light),frame=frame)
            rows.append(row)
            try:
                _,row["state"]=previous.light_state(light)
                row["frozen"]=light.is_frozen()
                previous.frozen._require(type(row["frozen"]) is bool,"actual restoration frozen bool required")
                row["green_yellow_red_seconds"]=[previous.finite(getattr(light,"get_"+name+"_time")())
                    for name in ("green","yellow","red")]
                row["elapsed_seconds"]=previous.finite(light.get_elapsed_time())
            except Exception as error:
                row["error"]=type(error).__name__+": "+str(error)
        previous.frozen._require(self.world.get_snapshot().frame==frame,"restoration readback crossed physics frames")
        previous.frozen._require(len(rows)==EXPECTED_LIGHT_COUNT and not any("error" in row for row in rows),
            "complete finite same-frame all38 signal readbacks required")
        return rows

    def restore(self):
        verify_frozen()
        previous.assert_owned_capture(self.args)
        previous.frozen._require(tuple(sorted(previous.actor_id(actor) for actor in self.world.get_actors().filter("*traffic_light*")))
            ==tuple(previous.actor_id(actor) for actor in self.actors),"refuse restoration into a changed actor inventory")
        errors=[];original_settings=self.world.get_settings()
        self.record.update(restoration_protocol_schema=SCHEMA,
            restoration_state_verified_before_unfreeze=False,
            unfrozen_natural_phase_change_permitted_only_after_exact_state_proof=True,
            elapsed_signal_phase_not_rewound=True,restoration_planned_native_tick_count=2,
            restoration_original_settings=previous.collector._settings_dict(original_settings),
            natural_phase_changes=[])
        try:
            self._owned_empty_inventory()
            settings=self.world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05
            self.world.apply_settings(settings)
            # HH_261004 - Keep the world-global signal freeze while restoring every original duration and phase.
            self.actors[0].freeze(True)
            for light,state,frozen_value,times in self.saved:
                for name,value in zip(("green","yellow","red"),times): getattr(light,"set_"+name+"_time")(value)
                light.set_state(state)
            self.fresh_tick("post_vehicle_restoration_tick")
            first=self._readback("frozen_state_restore_readback")
            original=self.record["original_actor_ids"]
            previous.frozen._require(len(original)==EXPECTED_LIGHT_COUNT and all(
                row["actor_id"]==saved["actor_id"] and row["state"]==saved["state"] and row["frozen"] is True
                and row["green_yellow_red_seconds"]==saved["green_yellow_red_seconds"]
                for row,saved in zip(first,original)),"original state/durations not restored while frozen")
            self.record["frozen_state_duration_proof_passed"]=True
            self.record["restoration_state_verified_before_unfreeze"]=True
            self._owned_empty_inventory()
            self.actors[0].freeze(self.original_frozen)
            self.fresh_tick("post_vehicle_unfreeze_tick")
            final=self._readback("original_freeze_restore_readback")
            previous.frozen._require(all(row["actor_id"]==saved["actor_id"] and row["frozen"] is saved["frozen"]
                and row["green_yellow_red_seconds"]==saved["green_yellow_red_seconds"]
                for row,saved in zip(final,original)),"original freeze/durations not restored after unfreeze")
            self.record["natural_phase_changes"]=[dict(actor_id=row["actor_id"],
                frozen_verified_state=before["state"],post_unfreeze_state=row["state"],
                before_elapsed_seconds=before["elapsed_seconds"],after_elapsed_seconds=row["elapsed_seconds"])
                for before,row in zip(first,final) if before["state"]!=row["state"]]
            previous.frozen._require(not self.original_frozen or not self.record["natural_phase_changes"],
                "a signal that remained frozen changed its verified phase")
            self.record["original_freeze_duration_proof_passed"]=True
        except Exception as error:
            errors.append(type(error).__name__+": "+str(error))
        finally:
            # HH_261004 - Preserve the exact inherited settings restoration and equality check on every exit.
            _inherited_restore_settings(self,original_settings)
            self.record["restoration_final_settings"]=previous.collector._settings_dict(self.world.get_settings())
        self.restored=not errors
        self.record.update(state="RESTORED" if self.restored else "RESTORE_FAILED",
            original_state_freeze_and_duration_restored=self.restored,
            state_component_proved_at_frozen_tick_not_after_natural_unfreeze=self.record.get("frozen_state_duration_proof_passed",False),
            restoration_native_tick_count=sum(key in self.record for key in
                ("post_vehicle_restoration_tick","post_vehicle_unfreeze_tick")),restore_errors=errors)
        previous.frozen._require(self.restored,"two-phase signal fixture restoration failed")


def verify_record(record):
    """HH_261004 - Independently verify complete retained actor/tick/settings evidence; a legacy restoration flag alone cannot pass."""
    verify_frozen()
    require=previous.frozen._require
    def finite(value):
        require(type(value) in (int,float) and math.isfinite(value),"finite nonboolean two-phase evidence required")
        return float(value)
    require(type(record) is dict and record.get("restoration_protocol_schema")==SCHEMA
        and record.get("state")=="RESTORED" and record.get("restore_errors")==[]
        and record.get("restoration_planned_native_tick_count")==2
        and record.get("restoration_native_tick_count")==2,"complete fresh two-phase restoration record required")
    for key in ("elapsed_signal_phase_not_rewound","restoration_state_verified_before_unfreeze",
            "unfrozen_natural_phase_change_permitted_only_after_exact_state_proof","frozen_state_duration_proof_passed",
            "original_freeze_duration_proof_passed","state_component_proved_at_frozen_tick_not_after_natural_unfreeze",
            "original_state_freeze_and_duration_restored"):
        require(record.get(key) is True,"actual two-phase proof missing: "+key)
    originals=record.get("original_actor_ids")
    require(type(originals) is list and len(originals)==EXPECTED_LIGHT_COUNT,"complete all38 original actors required")
    require(record.get("restoration_inventory_admission_schema")==INVENTORY_ADMISSION_SCHEMA
        and type(record.get("restoration_expected_light_count")) is int
        and record["restoration_expected_light_count"]==EXPECTED_LIGHT_COUNT
        and record.get("restoration_complete_original_actor_ids")==[row.get("actor_id") for row in originals]
        and record.get("restoration_count_declaration_evidence_sha256")==INVENTORY_EVIDENCE_PINS
        and record.get("mistaken_v1_43_count_failure_preserved") is True
        and record.get("original_actor_ID_43_not_misinterpreted_as_count") is True,
        "fresh count38 declaration and complete original actor admission must remain bound")
    ids=[];flags=[]
    for row in originals:
        require(type(row) is dict and type(row.get("actor_id")) is int and row["actor_id"]>0
            and row.get("state") in {"Red","Yellow","Green","Off","Unknown"}
            and type(row.get("frozen")) is bool,"exact original traffic actor/state/freeze ABI required")
        ids.append(row["actor_id"]);flags.append(row["frozen"])
        times=row.get("green_yellow_red_seconds")
        require(type(times) is list and len(times)==3 and all(finite(value)>=0 for value in times),"original signal durations invalid")
        finite(row.get("elapsed_seconds"))
    require(ids==sorted(set(ids)) and len(set(flags))==1,"original ordered unique actor inventory and uniform global freeze required")
    ticks=[];channels=[]
    for role,key in (("post_vehicle_restoration_tick","frozen_state_restore_readback"),
            ("post_vehicle_unfreeze_tick","original_freeze_restore_readback")):
        tick=record.get(role)
        require(type(tick) is dict and type(tick.get("before_frame")) is int
            and type(tick.get("after_frame")) is int and tick["after_frame"]==tick["before_frame"]+1,
            "one exact owned native restoration tick required")
        require(abs(finite(tick.get("after_timestamp"))-finite(tick.get("before_timestamp"))-.05)<=1.e-6,
            "unchanged exact20Hz restoration timestamp required")
        require(tick.get("outside_capture") is True and tick.get("vehicle_or_sensor_existed") is False
            and tick.get("counted_as_NN_or_capture_native") is False,"restoration tick cannot become driving evidence")
        rows=record.get(key)
        require(type(rows) is list and len(rows)==EXPECTED_LIGHT_COUNT,"complete all38 phase readbacks required")
        for row,saved in zip(rows,originals):
            require(type(row) is dict and "error" not in row and type(row.get("actor_id")) is int
                and row["actor_id"]==saved["actor_id"] and type(row.get("frame")) is int
                and row["frame"]==tick["after_frame"] and row.get("state") in {"Red","Yellow","Green","Off","Unknown"}
                and type(row.get("frozen")) is bool and row.get("green_yellow_red_seconds")==saved["green_yellow_red_seconds"],
                "same-frame exact original actor/duration phase readback required")
            times=row.get("green_yellow_red_seconds")
            require(type(times) is list and len(times)==3 and all(finite(value)>=0 for value in times),
                "actual phase durations must retain finite nonboolean physical values")
            finite(row.get("elapsed_seconds"))
        ticks.append(tick);channels.append(rows)
    first,last=channels
    require(all(row["state"]==saved["state"] and row["frozen"] is True for row,saved in zip(first,originals)),
        "original phase must be exactly proved while globally frozen")
    require(ticks[1]["before_frame"]==ticks[0]["after_frame"]
        and ticks[1]["before_timestamp"]==ticks[0]["after_timestamp"],"unfreeze proof must immediately follow frozen-state proof")
    require(all(row["frozen"] is saved["frozen"] for row,saved in zip(last,originals)),"original global freeze must be restored")
    changes=[dict(actor_id=row["actor_id"],frozen_verified_state=before["state"],post_unfreeze_state=row["state"],
        before_elapsed_seconds=before["elapsed_seconds"],after_elapsed_seconds=row["elapsed_seconds"])
        for before,row in zip(first,last) if before["state"]!=row["state"]]
    require(record.get("natural_phase_changes")==changes and (not flags[0] or not changes),
        "all natural phase changes must be explicitly retained, never permit a frozen-state change")
    settings=record.get("restoration_original_settings")
    require(type(settings) is dict and set(settings)=={"synchronous_mode","fixed_delta_seconds","no_rendering_mode",
        "substepping","max_substep_delta_time","max_substeps"}
        and record["restoration_original_settings"]==record.get("restoration_final_settings"),"exact original physics settings proof required")
    return dict(schema=SCHEMA+".independent_record_verification",passed=True,
        original_actor_count=EXPECTED_LIGHT_COUNT,frozen_state_readback_count=len(first),
        original_freeze_readback_count=len(last),outside_capture_native_ticks=2,
        natural_phase_changes=changes,state_proved_before_unfreeze=True,elapsed_rewound=False,
        original_physics_settings_restored=True,neural_driving_or_traffic_rule_performance_proven=False)


@contextmanager
def activated():
    """HH_261004 - Scope the actual inner factory's late-bound class only; restore its exact identity on every exit."""
    global _active
    verify_frozen()
    previous.frozen._require(not _active and previous.GreenSignalFixture is TwoPhaseGreenSignalFixture.__bases__[0],
        "fresh restoration fixture cannot nest or replace an unknown provider")
    original=previous.GreenSignalFixture;_active=True
    previous.GreenSignalFixture=TwoPhaseGreenSignalFixture
    try:
        yield
    finally:
        previous.GreenSignalFixture=original;_active=False
