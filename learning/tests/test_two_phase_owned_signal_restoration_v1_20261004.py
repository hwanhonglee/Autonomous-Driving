"""HH_261004 - Prove fresh all43 two-phase restoration with cached synthetic20Hz getters, never actual actuation evidence."""

import copy
from contextlib import contextmanager
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import two_phase_owned_signal_restoration_v1_20261004 as m


class State:
    def __init__(self,name): self.name=name


RED,GREEN,YELLOW=State("Red"),State("Green"),State("Yellow")


class Light:
    def __init__(self,world,identity,frozen):
        self.world,self.id,self.is_alive=world,identity,True
        self.state=RED if identity==1 else (YELLOW if identity%2 else GREEN)
        self.frozen=frozen;self.times=[8.,3.,2.];self.elapsed=1.9901 if identity==1 else .1
        self.calls=[]
    def get_state(self): return self.state
    def set_state(self,value): self.calls.append(("state",value.name));self.world.pending.setdefault(self.id,{})["state"]=value
    def is_frozen(self): return self.frozen
    def freeze(self,value): self.world.freeze_calls.append(value);self.world.pending_frozen=value
    def get_green_time(self): return self.times[0]
    def get_yellow_time(self): return self.times[1]
    def get_red_time(self): return self.times[2]
    def set_green_time(self,value): self.world.pending.setdefault(self.id,{})[0]=value
    def set_yellow_time(self,value): self.world.pending.setdefault(self.id,{})[1]=value
    def set_red_time(self,value): self.world.pending.setdefault(self.id,{})[2]=value
    def get_elapsed_time(self): return self.elapsed
    def get_transform(self):
        return SimpleNamespace(location=SimpleNamespace(x=float(self.id),y=2.,z=3.),
            rotation=SimpleNamespace(yaw=0.,pitch=0.,roll=0.))


class Actors:
    def __init__(self,world): self.world=world
    def filter(self,pattern):
        if pattern=="*traffic_light*": return self.world.lights
        return self.world.intrusions.get(pattern,[])


class World:
    def __init__(self,frozen=False):
        self.frame=50;self.pending={};self.pending_frozen=None;self.freeze_calls=[];self.apply_calls=[];self.intrusions={}
        self.lights=[Light(self,index,frozen) for index in range(1,44)]
        self.settings=SimpleNamespace(synchronous_mode=True,fixed_delta_seconds=.05,no_rendering_mode=False,
            substepping=True,max_substep_delta_time=.01,max_substeps=5)
        self.fault=None
    def get_settings(self): return SimpleNamespace(**vars(self.settings))
    def apply_settings(self,value): self.apply_calls.append(dict(vars(value)));self.settings=SimpleNamespace(**vars(value))
    def get_actors(self): return Actors(self)
    def get_map(self): return SimpleNamespace(name="/Game/Maps/Town03")
    def get_snapshot(self): return SimpleNamespace(frame=self.frame,timestamp=SimpleNamespace(elapsed_seconds=self.frame*.05))
    def tick(self):
        assert self.settings.synchronous_mode and self.settings.fixed_delta_seconds==.05
        for light in self.lights:
            row=self.pending.get(light.id,{})
            if "state" in row: light.state=row["state"]
            for index in range(3):
                if index in row: light.times[index]=row[index]
            if self.pending_frozen is not None: light.frozen=self.pending_frozen
            if not light.frozen:
                light.elapsed+=.05
                if light.state is RED and light.elapsed>=light.times[2]:
                    light.state=GREEN;light.elapsed-=light.times[2]
        self.pending={};self.pending_frozen=None;self.frame+=1
        if self.frame==52 and self.fault=="frozen_state": self.lights[0].state=GREEN
        if self.frame==53 and self.fault=="final_duration": self.lights[2].times[1]+=1.
        if self.frame==53 and self.fault=="final_freeze": self.lights[2].frozen=True
        if self.frame==53 and self.fault=="nonfinite": self.lights[2].elapsed=float("nan")
        return self.frame


@contextmanager
def prepared(frozen=False):
    world=World(frozen)
    with mock.patch.object(m.previous,"assert_owned_capture") as owner_guard:
        fixture=m.TwoPhaseGreenSignalFixture(world,SimpleNamespace(),{},GREEN)
        fixture.install()
        # HH_261004 - The inherited collector restores its asynchronous settings before the separate fixture suffix.
        world.settings.synchronous_mode=False;world.settings.fixed_delta_seconds=None
        yield fixture,owner_guard


def test_frozen_source_and_real_provider_scope_restore_on_success_and_exception():
    m.verify_frozen()
    original=m.previous.GreenSignalFixture
    assert m.TwoPhaseGreenSignalFixture.__bases__==(original,)
    for raises in (False,True):
        try:
            with m.activated():
                assert m.previous.GreenSignalFixture is m.TwoPhaseGreenSignalFixture
                with pytest.raises(Exception):
                    with m.activated(): pass
                if raises: raise RuntimeError("synthetic original capture error")
        except RuntimeError: assert raises
        assert m.previous.GreenSignalFixture is original and m._active is False


@pytest.mark.parametrize("original_frozen",[False,True])
def test_all43_exact_state_first_then_original_freeze_and_natural_phase(original_frozen):
    with prepared(original_frozen) as (fixture,guard):
        original_settings=vars(fixture.world.get_settings())
        fixture.restore()
        result=m.verify_record(fixture.record)
        assert result["passed"] is True and result["original_actor_count"]==43
        assert result["frozen_state_readback_count"]==result["original_freeze_readback_count"]==43
        assert result["outside_capture_native_ticks"]==2
        assert fixture.world.frame==53 and fixture.world.freeze_calls==[True,True,original_frozen]
        assert fixture.record["restoration_final_settings"]==fixture.record["restoration_original_settings"]
        assert vars(fixture.world.get_settings())==original_settings and guard.call_count>=9
        assert all(row["state"]==saved["state"] and row["frozen"] is True
            for row,saved in zip(fixture.record["frozen_state_restore_readback"],fixture.record["original_actor_ids"]))
        changes=fixture.record["natural_phase_changes"]
        assert len(changes)==(0 if original_frozen else 1)
        if changes:
            assert changes[0]["actor_id"]==1 and changes[0]["frozen_verified_state"]=="Red"
            assert changes[0]["post_unfreeze_state"]=="Green"
        assert result["neural_driving_or_traffic_rule_performance_proven"] is False


def test_original_single_unfreeze_tick_rejects_same_natural_phase_change_but_new_protocol_does_not_rewrite_it():
    with prepared(False) as (fixture,guard):
        original_class=m.TwoPhaseGreenSignalFixture.__bases__[0]
        with pytest.raises(Exception,match="signal fixture restoration failed"):
            original_class.restore(fixture)
        assert fixture.record["state"]=="RESTORE_FAILED" and fixture.world.lights[0].state is GREEN
        with pytest.raises(Exception): m.verify_record(fixture.record)


@pytest.mark.parametrize("fault",["frozen_state","final_duration","final_freeze","nonfinite"])
def test_failed_readbacks_retain_complete43_rows_settings_and_never_pass(fault):
    with prepared(False) as (fixture,guard):
        fixture.world.fault=fault
        original_settings=vars(fixture.world.get_settings())
        with pytest.raises(Exception,match="two-phase signal fixture restoration failed"): fixture.restore()
        assert fixture.record["state"]=="RESTORE_FAILED" and fixture.record["restore_errors"]
        assert len(fixture.record["frozen_state_restore_readback"])==43
        if fault!="frozen_state": assert len(fixture.record["original_freeze_restore_readback"])==43
        assert vars(fixture.world.get_settings())==original_settings
        with pytest.raises(Exception): m.verify_record(fixture.record)


@pytest.mark.parametrize("intrusion",["vehicle.*","sensor.*","walker.*","controller.ai.walker"])
def test_owner_and_empty_world_guards_preserved_before_signal_mutation(intrusion):
    with prepared(False) as (fixture,guard):
        before=list(fixture.world.freeze_calls)
        fixture.world.intrusions[intrusion]=[SimpleNamespace(id=600,type_id=intrusion,attributes={})]
        with pytest.raises(Exception): fixture.restore()
        assert fixture.world.freeze_calls==before
        assert fixture.record["state"]=="RESTORE_FAILED"


def test_owner_generation_failure_rejects_before_any_settings_or_signal_mutation():
    with prepared(False) as (fixture,guard):
        before=(list(fixture.world.freeze_calls),list(fixture.world.apply_calls))
        guard.side_effect=RuntimeError("synthetic PID generation changed")
        with pytest.raises(RuntimeError,match="generation"): fixture.restore()
        assert before==(fixture.world.freeze_calls,fixture.world.apply_calls)


@pytest.mark.parametrize("fault",["missing_actor","duplicate_actor","actor_frame","actor_bool","original_state",
    "first_state","first_freeze","last_freeze","last_duration","duration_bool","duration_nonfinite",
    "unfreeze_gap","timestamp_gap","counted_as_NN","extra_natural_change","missing_natural_change",
    "settings","empty_settings","flag_only","error_hidden","wrong_schema","elapsed_bool"])
def test_pure_verifier_recomputes_every_actor_phase_not_legacy_PASS_flag(fault):
    with prepared(False) as (fixture,guard): fixture.restore();record=copy.deepcopy(fixture.record)
    first=record["frozen_state_restore_readback"];last=record["original_freeze_restore_readback"]
    if fault=="missing_actor": first.pop()
    elif fault=="duplicate_actor": last[-1]=copy.deepcopy(last[-2])
    elif fault=="actor_frame": last[0]["frame"]-=1
    elif fault=="actor_bool": first[0]["actor_id"]=True
    elif fault=="original_state": record["original_actor_ids"][0]["state"]="Yellow"
    elif fault=="first_state": first[0]["state"]="Green"
    elif fault=="first_freeze": first[0]["frozen"]=False
    elif fault=="last_freeze": last[0]["frozen"]=True
    elif fault=="last_duration": last[0]["green_yellow_red_seconds"][0]+=1.
    elif fault=="duration_bool": last[0]["green_yellow_red_seconds"][0]=True
    elif fault=="duration_nonfinite": last[0]["green_yellow_red_seconds"][0]=float("nan")
    elif fault=="unfreeze_gap": record["post_vehicle_unfreeze_tick"]["before_frame"]+=1
    elif fault=="timestamp_gap": record["post_vehicle_unfreeze_tick"]["after_timestamp"]+=.01
    elif fault=="counted_as_NN": record["post_vehicle_unfreeze_tick"]["counted_as_NN_or_capture_native"]=True
    elif fault=="extra_natural_change": record["natural_phase_changes"].append(copy.deepcopy(record["natural_phase_changes"][0]))
    elif fault=="missing_natural_change": record["natural_phase_changes"]=[]
    elif fault=="settings": record["restoration_final_settings"]["fixed_delta_seconds"]=.05
    elif fault=="empty_settings": record["restoration_original_settings"]=record["restoration_final_settings"]={}
    elif fault=="flag_only": record.pop("frozen_state_restore_readback")
    elif fault=="error_hidden": last[0]["error"]="getter failed"
    elif fault=="wrong_schema": record["restoration_protocol_schema"]="legacy"
    else: last[0]["elapsed_seconds"]=True
    with pytest.raises(Exception): m.verify_record(record)
