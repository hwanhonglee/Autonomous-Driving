"""HH_261002 - Prove preview0.5 is the only tracking-factor change with full STOP, ownership and readback guards."""

from contextlib import contextmanager, ExitStack
from dataclasses import asdict, replace
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_route_joint_preview05_signed45_20261002 as live
from scripts.e2e import research_temporal_route_joint_preview05_signed45_town07_20261002 as owner
from test_temporal_route_joint_preview08_signed45_20261002 import selected, config as old_config, journal_fixture, args, pins


def config():
    return replace(old_config(),speed_preview_s=.5)


@pytest.mark.parametrize("definition",["actor_controller_config","preview_selection","trajectory_control","Preview08ObservedSession","activated"])
def test_ast_restores_every_nonpreview_statement_exactly(definition):
    assert live.transformed_definition(definition)


def test_actual_geometry_factory_changes_only_preview_and_new_index_is4():
    original=replace(config(),speed_preview_s=.3)
    with mock.patch.object(live,"_actor_config",return_value=original) as delegate:
        actual=live.actor_controller_config("real-physics",2.5)
    delegate.assert_called_once_with("real-physics",2.5)
    assert asdict(actual)==dict(asdict(original),speed_preview_s=.5)
    receipt=live.preview_selection(selected(),actual)
    assert receipt["original_preview_index"]==2 and receipt["proposed_preview_index"]==4
    assert receipt["effective_selected_target_mps"]==selected().speed_mps[4]
    assert receipt["schema"]==live.PREVIEW_SCHEMA
    assert not receipt["selected_trajectory_changed"] and not receipt["actual_measured_speed_clipped"]


@pytest.mark.parametrize("index,value",[(2,0.),(2,.05),(4,0.),(4,.05)])
def test_either_preview_STOP_preserves_entire_old_tracker(index,value):
    speeds=[.6]*64;speeds[index]=value;chosen=selected(speeds);session=SimpleNamespace(_preview_tracker_receipt=None)
    with mock.patch.object(live,"_active",True),mock.patch.object(live,"_proposal_session",session):
        actual=live.trajectory_control(chosen,.4,config())
    assert actual==live._trajectory_control(chosen,.4,replace(config(),speed_preview_s=.3))
    assert session._preview_tracker_receipt["effective_speed_preview_s"]==.3


@pytest.mark.parametrize("intent",[1,2,True,None,"GO"])
def test_non_GO_always_preserves_old_preview(intent):
    assert live.preview_selection(SimpleNamespace(**asdict(selected()),intent=intent),config())["effective_speed_preview_s"]==.3


def test_tracker_called_once_selected_and_frozen_source_unmodified():
    chosen=selected();before=asdict(chosen);session=SimpleNamespace(_preview_tracker_receipt=None)
    original_source=Path(live.source.__file__).read_bytes()
    with mock.patch.object(live,"_active",True),mock.patch.object(live,"_proposal_session",session), \
         mock.patch.object(live,"_trajectory_control",wraps=live._trajectory_control) as delegate:
        actual=live.trajectory_control(chosen,.5,config())
        with pytest.raises(Exception):live.trajectory_control(chosen,.5,config())
    assert delegate.call_count==1 and actual["desired_speed_mps"]==chosen.speed_mps[4]
    assert asdict(chosen)==before and Path(live.source.__file__).read_bytes()==original_source


@pytest.mark.parametrize("failure",[False,True])
def test_collector_full_restoration_and_separate0p8_state(failure):
    before=(live.previous.SCHEMA,live.previous.Thread4ObservedSession,live.frozen.original.actor_controller_config,
            live.runtime.trajectory_control,live.collector.collect_episode)
    @contextmanager
    def inherited(*a):
        assert live.previous.Thread4ObservedSession is live.Preview05ObservedSession
        assert live.runtime.trajectory_control is live.trajectory_control
        assert not live.source._active and live.source._proposal_session is None
        if failure:raise OSError("original")
        yield
    with mock.patch.object(live,"verify_frozen"),mock.patch.object(live.previous.previous,"validate_model_binding"), \
         mock.patch.object(live.previous,"activated",side_effect=inherited):
        try:
            with live.activated(SimpleNamespace(_research_speed_preview_calibration=live.PREVIEW_SCHEMA),args()):
                with pytest.raises(Exception):
                    with live.activated(SimpleNamespace(),args()):pass
        except OSError:assert failure
    assert before==(live.previous.SCHEMA,live.previous.Thread4ObservedSession,live.frozen.original.actor_controller_config,
        live.runtime.trajectory_control,live.collector.collect_episode)
    assert not live._active and not live.source._active


def journal05():
    records,wall,raw=journal_fixture();controller=asdict(config());chosen=selected()
    receipt=dict(live.preview_selection(chosen,config()),original_tracker_control=live._trajectory_control(chosen,.5,config()))
    for index,(record,command) in enumerate(zip(records,raw)):
        record.update(schema=live.PREVIEW_SCHEMA,actual_policy_controller=controller,tracker_receipt=receipt if index==1 else None)
        if index:
            command.update(controller=controller,control=dict(desired_speed_mps=chosen.speed_mps[4]))
            record["actual_returned_control"]=command["control"]
    return records,wall,raw


@pytest.mark.parametrize("change",[None,"speedpreview","lateralpreview","missing","duplicate","heldtracker","target","timing","ACK"])
def test_exact_complete_preview_readbacks_and_lateral0p8_guard(change,tmp_path):
    records,wall,raw=journal05()
    if change=="speedpreview":records[1]["actual_policy_controller"]=dict(records[1]["actual_policy_controller"],speed_preview_s=.8)
    elif change=="lateralpreview":
        for row in records:row["actual_policy_controller"]=dict(row["actual_policy_controller"],lookahead_time_s=.5)
    elif change=="missing":records.pop()
    elif change=="duplicate":records[2]=records[1]
    elif change=="heldtracker":records[2]["tracker_receipt"]=records[1]["tracker_receipt"]
    elif change=="target":records[1]["tracker_receipt"]["effective_selected_target_mps"]+=.01
    elif change=="timing":records[1]["proposal_finished_monotonic_ns"]=51
    elif change=="ACK":raw[2]["ack_receipt_sequence"]=2
    data={"speed_preview_configuration.jsonl":records,"wall_timing.jsonl":wall,"learned_control.jsonl":raw}
    for name,rows in data.items():
        (tmp_path/name).write_text("".join(json.dumps(row)+"\n" for row in rows))
    with mock.patch.object(owner.parent,"jsonl",side_effect=lambda p:data[p.name]):
        if change is None:
            with mock.patch.object(owner,"digest",side_effect=lambda p:owner.previous.FROZEN_OWNER_SHA if Path(p)==Path(owner.proto.__file__) else "a"*64):
                result=owner.configuration_report(tmp_path)
            assert result["passed"] and result["fresh_tracker_count"]==1
            assert result["actual_controller"]["speed_preview_s"]==.5 and result["actual_controller"]["lookahead_time_s"]==.8
        else:
            with pytest.raises(Exception):owner.configuration_report(tmp_path)


def fake_context(path):
    return dict(receipt_source_sha256={str(path):owner.digest(path)},trials=[dict(trial_index=i) for i in range(3)])


@pytest.mark.parametrize("trial",[0,1,2])
def test_real_public_preflight_entire_scope_chain_worker_and_unchanged_all_gates(tmp_path,trial):
    compat=owner.previous;proto=owner.proto;thread=proto.previous;joint=thread.previous
    report=tmp_path/"proof";report.write_bytes(b"x")
    baseline=dict(source_sha256={},final_endpoint_sha256=pins(),
        prospective_preview_acceleration_guard_applied_to_all_original_baseline_arms=[])
    model=SimpleNamespace(_research_temporal_checkpoint_sha256="c"*64)
    api=SimpleNamespace(stdout=str(owner.base.EGG)+"/__init__.py\n"+str(owner.base.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n1.8.0\n")
    digest=compat.digest
    patches=(mock.patch.object(owner,"prior_preview08_context",return_value=fake_context(report)),
        mock.patch.object(proto,"BASELINE",report),mock.patch.object(compat,"BASELINE",report),
        mock.patch.object(compat,"baseline_proof",return_value=baseline),mock.patch.object(compat,"digest",side_effect=lambda p:
            compat.FAILURE_SHA if Path(p)==compat.FAILURE else digest(p)),mock.patch.object(proto,"digest",return_value=proto.FROZEN_OWNER_SHA),
        mock.patch.object(thread,"evidence_assets",return_value={report:"f"*64}),mock.patch.object(thread,"configure_threads"),
        mock.patch.object(thread.torch,"get_num_threads",return_value=1),mock.patch.object(thread.torch,"set_num_threads"),
        mock.patch.object(joint,"verify_exports",return_value={}),mock.patch.object(owner,"runtime_sources",return_value={}),
        mock.patch.object(owner.base,"verify_plan",return_value=dict(longitudinal_pi=True)),
        mock.patch.object(owner.base.lifecycle,"_assert_no_other_carla"),mock.patch.object(owner.base,"ports_free"),
        mock.patch.object(owner.base.subprocess,"run",return_value=api),mock.patch.object(owner.base,"environment",return_value={}),
        mock.patch.object(live.collector,"load_route",return_value=dict(town="Town07",scenario="straight")))
    with ExitStack() as stack:
        for patch in patches:stack.enter_context(patch)
        with owner.activated(pins()),owner.parent.scoped_trial(trial):
            with mock.patch.object(compat,"external_pins",return_value={}):contract=owner.base.fixed_contract("v1",True)
            assert contract["worker_command"][contract["worker_command"].index("-m")+1]==owner.MODULE
            assert contract["schema"]==owner.SCHEMA and owner.base.live is live
            assert contract["causal_selected_speed_preview_calibration"]["configured_speed_preview_s"]==.5
            assert contract["precursor_motion_screen"]["net_learned_progress_minimum_m"]==90.
            assert contract["precursor_motion_screen"]["post_handoff_base_planar_speed_maximum_mps"]==2.5
            assert contract["causal_selected_speed_preview_calibration"]["additional_prospective_full_driving_actor_world_XY_speed_rate_guard_mps2"]==dict(minimum=-6.,maximum=3.)
            assert contract["controller_target_cap_mps"]==2.4 and contract["total_capture_seconds"]==45
            assert contract["sensor_and_ACK_screen"]["native_state_count"]==900
            assert contract["body_clearance_screen"]["all_snapshot_linear_sweeps"]==899
            assert len(json.dumps(contract))<4*1024**2
            with mock.patch.object(owner.base,"paths",return_value=(tmp_path/"absent",None)), \
                 mock.patch.object(owner.base,"disk_check",return_value={}),mock.patch.object(owner.base,"load_npz",return_value=model):
                _,result=owner.base.preflight("v1","e"*64)
            assert result["status"]=="PREFLIGHT_PASS_NO_SIMULATOR" and not live._active


@pytest.mark.parametrize("failure",[False,True])
def test_owner_every_binding_restored_after_success_or_original_exception(tmp_path,failure):
    path=tmp_path/"context";path.write_bytes(b"raw")
    names=("batch_paths","MODULE","SCHEMA","fixed_contract","runtime_sources")
    delegated=("live","calibration_contract","configuration_report","diagnostic_report","batch_summary","disk_check")
    before={n:getattr(owner.previous,n) for n in names};before_proto={n:getattr(owner.proto,n) for n in delegated};assets=owner.base.ASSETS
    @contextmanager
    def inherited(*a):
        assert owner.proto.live is live and owner.previous.MODULE==owner.MODULE
        if failure:raise OSError("original")
        yield
    with mock.patch.object(owner,"prior_preview08_context",return_value=fake_context(path)), \
         mock.patch.object(owner.previous,"activated",side_effect=inherited):
        try:
            with owner.activated(pins()):
                with pytest.raises(Exception):
                    with owner.activated(pins()):pass
        except OSError:assert failure
    assert all(getattr(owner.previous,n) is v for n,v in before.items())
    assert all(getattr(owner.proto,n) is v for n,v in before_proto.items()) and owner.base.ASSETS is assets
    assert not owner._active and owner._context is None


def test_prior0p8_receipt_mutation_fails_after_declaration(tmp_path):
    path=tmp_path/"raw";path.write_bytes(b"original")
    context=fake_context(path);path.write_bytes(b"changed")
    with mock.patch.object(owner,"_context",context),pytest.raises(Exception):owner.verify_prior_context()


def test_unknown_run_id_and_no0p8_path_reuse():
    output,_=owner.batch_paths("v1")
    assert "preview05" in str(output) and output!=owner.PREVIEW08_BATCH
    for value in ("../v1","v0","old",True):
        with pytest.raises(Exception):owner.batch_paths(value)


@pytest.mark.parametrize("free,allowed",[(109,True),(108,False)])
def test_complete_prior0p8_receipt_copies_have_declared_disk_reserve(tmp_path,free,allowed):
    path=tmp_path/"receipt";path.write_bytes(b"123")
    with mock.patch.object(owner,"_context",fake_context(path)), \
         mock.patch.object(owner,"_disk",return_value=dict(free_bytes=free,required_bytes=100)):
        if allowed:
            assert owner.disk_check()["required_bytes"]==109
        else:
            with pytest.raises(Exception):owner.disk_check()


@pytest.mark.parametrize("change",[None,"missingcase","wrongweights","wrongcaseSHA","preview","incomplete","ownership"])
def test_all_three_prior0p8_closed_attempts_exact_same_endpoint_no_subset(change,tmp_path):
    root=tmp_path;batchroot=root/"original08";batchroot.mkdir();plan=root/"batch_plan.json"
    cases=[]
    for index in range(3):
        folder=batchroot/("trial_%02d"%index);folder.mkdir()
        declaration=dict(trial_index=index,new_fixed_endpoint_export_sha256=dict(weights=pins()["weights_sha256"],metadata=pins()["metadata_sha256"]),
            source_checkpoint_sha256=pins()["checkpoint_sha256"],actual_training_declaration_sha256=pins()["declaration_sha256"],
            optimizer_updates=6000,controller_target_cap_mps=2.4,causal_selected_speed_preview_calibration=dict(configured_speed_preview_s=.8))
        if change=="wrongweights" and index==1:declaration["new_fixed_endpoint_export_sha256"]["weights"]="f"*64
        actual=root/("case%d.json"%index);actual.write_text(json.dumps(declaration))
        (folder/"declaration.json").write_bytes(actual.read_bytes())
        case=dict(trial_index=index,declaration_path=str(actual),declaration_sha256=owner.digest(actual));cases.append(case)
        status=dict(bounded_capture_complete=True,actual_ACK_receipt_count=904,status="FAIL_PRESERVED",learned_driving_screen_passed=False,
            actual_preview_configuration=dict(passed=True,actual_controller=dict(speed_preview_s=.8)))
        if change=="preview" and index==1:status["actual_preview_configuration"]["actual_controller"]["speed_preview_s"]=.3
        if change=="incomplete" and index==1:status["bounded_capture_complete"]=False
        (folder/"diagnostic_report.json").write_text(json.dumps(status))
        result=dict(declaration_sha256=case["declaration_sha256"],source_hashes_match=True,external_source_hashes_match=True,cleanup_errors=[])
        if change=="ownership" and index==1:result["source_hashes_match"]=False
        (folder/"owner_result.json").write_text(json.dumps(result))
    if change=="missingcase":cases.pop()
    if change=="wrongcaseSHA":cases[1]["declaration_sha256"]="0"*64
    plan.write_text(json.dumps(dict(schema="portable_e2e.actual_route_joint_thread4_preview08_signed45_three_arms.20261002.v2.batch",
        prospective_trial_count=3,trials=cases,capture_seconds_per_trial=45,net_progress_minimum_m=90.,original_model_speed_envelope_mps=2.5)))
    with mock.patch.object(owner,"ROOT",root),mock.patch.object(owner,"PREVIEW08_BATCH",batchroot), \
         mock.patch.object(owner,"PREVIEW08_PLAN",plan),mock.patch.object(owner,"PREVIEW08_PLAN_SHA",owner.digest(plan)):
        if change is None:
            result=owner.prior_preview08_context(pins())
            assert len(result["trials"])==3 and len(result["receipt_source_sha256"])==13
            assert all(row["original_basic_screen_passed"] is False for row in result["trials"])
        else:
            with pytest.raises(Exception):owner.prior_preview08_context(pins())
