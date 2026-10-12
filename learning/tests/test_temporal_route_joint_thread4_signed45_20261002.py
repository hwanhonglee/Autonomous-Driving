"""HH_261002 - Prospectively preserve every signed45 guard while declaring changed floating CPU-thread math."""

from contextlib import contextmanager
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_route_joint_thread4_signed45_20261002 as live
from scripts.e2e import research_temporal_route_joint_thread4_signed45_town07_20261002 as owner


def pins():
    return {name + "_sha256": letter * 64 for name, letter in
        (("weights", "a"), ("metadata", "b"), ("checkpoint", "c"), ("declaration", "d"))}


def args(seed=2026102100):
    return SimpleNamespace(seed=seed, wheelbase_m=2.85, host="127.0.0.1", port=2440, allow_map_load=False,
        control_transport="acknowledged_batch", goal_stop_profile="disabled", agent_initialization="after_bootstrap",
        physics_hz=20, capture_hz=10, stationary_warmup_sec=1.2, max_duration_sec=45, target_speed_kmh=9,
        weather="ClearNoon", spawn_z_offset_m=.5, research_speed_cap_mps=2.5, research_max_cte_m=1.,
        research_source_sha256="e"*64, research_longitudinal_pi=True, route_file=Path("unused"))


def model():
    return SimpleNamespace(_research_cpu_thread_numerical_revision=live.THREAD_SCHEMA)


@pytest.mark.parametrize("trial", [0, 1, 2])
def test_same_exact_uint32_profiles_and_original_validator(trial):
    assert live.PROFILES[trial] == live.previous.PROFILES[trial]
    assert live.validate_capture is live.previous.validate_capture
    live.validate_capture(args(live.PROFILES[trial]["seed"]), dict(town="Town07", scenario="straight"))


@pytest.mark.parametrize("field,value", [("seed", 20261002100), ("seed", True), ("max_duration_sec", 44),
    ("port", 2000), ("research_speed_cap_mps", 2.51), ("research_longitudinal_pi", False),
    ("agent_initialization", "before_bootstrap"), ("goal_stop_profile", "comfortable_v1")])
def test_no_capture_gate_relaxation(field, value):
    used = args(); setattr(used, field, value)
    with pytest.raises(Exception): live.validate_capture(used, dict(town="Town07", scenario="straight"))


@pytest.mark.parametrize("intra,inter", [(1, 1), (2, 1), (4, 2), (0, 1)])
def test_actual_readback_mismatch_rejects(intra, inter):
    with mock.patch.object(live.torch, "get_num_threads", return_value=intra), \
         mock.patch.object(live.torch, "get_num_interop_threads", return_value=inter), pytest.raises(Exception):
        live.thread_readback()


def test_readback_and_original_proposal_exactly_once(tmp_path):
    used = object.__new__(live.Thread4ObservedSession)
    used.states = [dict(frame=123)]; used.handed_off=False; used.output=tmp_path
    actual_control, actual_decision = object(), object()
    with mock.patch.object(live, "thread_readback", return_value=dict(intra_op_threads=4, inter_op_threads=1)), \
         mock.patch.object(live.torch, "get_num_threads", return_value=4), \
         mock.patch.object(live.torch, "get_num_interop_threads", return_value=1), \
         mock.patch.object(live._original_session, "propose", return_value=(actual_control, actual_decision)) as delegate:
        assert used.propose("transport", "teacher", "next_driving_control") == (actual_control, actual_decision)
    delegate.assert_called_once_with("transport", "teacher", "next_driving_control")
    record=json.loads((tmp_path/"cpu_thread_readback.jsonl").read_text())
    assert record["command_frame"] == 123 and record["before_handoff"]
    assert record["settings_readback_outside_policy_deadline_inside_bundle_to_ACK"]
    assert not record["numerical_bitwise_equivalence_claimed"]


def test_mismatched_readback_is_retained_and_original_proposal_not_called(tmp_path):
    used=object.__new__(live.Thread4ObservedSession)
    used.states=[dict(frame=123)];used.handed_off=True;used.output=tmp_path
    with mock.patch.object(live.torch,"get_num_threads",return_value=2), \
         mock.patch.object(live.torch,"get_num_interop_threads",return_value=1), \
         mock.patch.object(live._original_session,"propose") as delegate,pytest.raises(Exception):
        used.propose("t","teacher","next_driving_control")
    delegate.assert_not_called()
    assert json.loads((tmp_path/"cpu_thread_readback.jsonl").read_text())["intra_op_threads"] == 2


@pytest.mark.parametrize("failure", [False, True])
def test_collector_scope_restores_class_schema_manifest_hook_on_any_exit(failure):
    original=(live.previous.SCHEMA,live.owned.OwnedCameraTemporalLiveResearchSessionV2,live.collector.collect_episode)
    @contextmanager
    def inherited(given,used):
        assert given._research_cpu_thread_numerical_revision == live.THREAD_SCHEMA
        assert live.owned.OwnedCameraTemporalLiveResearchSessionV2 is live.Thread4ObservedSession
        assert live.previous.SCHEMA == live.SCHEMA
        if failure: raise OSError("preserved")
        yield
    with mock.patch.object(live,"verify_frozen"),mock.patch.object(live.previous,"validate_model_binding"), \
         mock.patch.object(live,"thread_readback",return_value=dict(intra_op_threads=4,inter_op_threads=1)), \
         mock.patch.object(live.previous,"activated",side_effect=inherited):
        try:
            with live.activated(model(),args()):
                with pytest.raises(Exception):
                    with live.activated(model(),args()):pass
        except OSError: assert failure
    assert original == (live.previous.SCHEMA,live.owned.OwnedCameraTemporalLiveResearchSessionV2,live.collector.collect_episode)
    assert not live._active


def test_no_numeric_only_or_unknown_thread_binding():
    with mock.patch.object(live,"verify_frozen"),mock.patch.object(live.previous,"validate_model_binding"),pytest.raises(Exception):
        with live.activated(SimpleNamespace(),args()):pass


def test_setting_occurs_after_inherited_one_thread_and_before_same_numeric_loader():
    loaded=model();events=[]
    def configure():events.append("4/1")
    def load(*args,**kwargs):events.append("original_numeric");return loaded
    with mock.patch.object(owner,"configure_threads",side_effect=configure),mock.patch.object(owner,"_load",side_effect=load) as delegate, \
         mock.patch.object(live,"thread_readback"):
        assert owner.load_model("weights","meta",weights_sha256="a"*64,metadata_sha256="b"*64) is loaded
    assert events == ["4/1","original_numeric"]
    delegate.assert_called_once_with("weights","meta",weights_sha256="a"*64,metadata_sha256="b"*64)
    assert loaded._research_cpu_thread_numerical_revision == live.THREAD_SCHEMA


def test_environment_does_not_expose_cuda_or_change_other_values():
    with mock.patch.object(owner,"_environment",return_value={"CUDA_VISIBLE_DEVICES":"","keep":"yes","OMP_NUM_THREADS":"1"}):
        result=owner.environment()
    assert result == dict(CUDA_VISIBLE_DEVICES="",keep="yes",OMP_NUM_THREADS="4",MKL_NUM_THREADS="4",OPENBLAS_NUM_THREADS="4")


def test_preset_retains_false_strict_parity_and_no_threshold_replacement():
    result=owner.numerical_preset()
    assert not result["strict_original_offline_parity_passed"] and not result["decoded_and_selected_atol_rtol_1e_6_passed"]
    assert result["thread4_proposed_control_atol_rtol_1e_6_passed"]
    assert result["unchanged_hard_policy_deadline_seconds"] == .1
    assert result["unchanged_bundle_delivery_to_ACK_deadline_seconds"] == .1
    assert result["differences_are_observations_not_replacement_or_relaxed_tolerances"]
    assert not result["full_MAIN9_completion_claimed"]


@pytest.mark.parametrize("trial",[0,1,2])
def test_real_all_aliases_fixed_contract_public_capture_validator_worker_cli(tmp_path,trial):
    proof=tmp_path/"proof";proof.write_bytes(b"x")
    with mock.patch.object(owner,"evidence_assets",return_value={proof:"f"*64}), \
         mock.patch.object(owner,"digest",return_value=owner.FROZEN_OWNER_SHA), \
         mock.patch.object(owner,"configure_threads"),mock.patch.object(owner.torch,"get_num_threads",return_value=1), \
         mock.patch.object(owner.torch,"set_num_threads"), \
         mock.patch.object(owner.previous,"verify_exports",return_value={}),mock.patch.object(owner,"runtime_sources",return_value={}):
        with owner.activated(pins()),owner.parent.scoped_trial(trial):
            result=owner.base.fixed_contract("v1",True)
            route_args=owner.live.frozen.parse_args(result["capture_argv"])
            route_args.research_source_sha256="e"*64
            owner.live.frozen.validate_capture(route_args,dict(town="Town07",scenario="straight"))
            assert owner.base.load_npz is owner.load_model and owner.parent.live is live
    assert result["schema"] == owner.SCHEMA
    assert result["seed"] == live.PROFILES[trial]["seed"]
    assert result["worker_command"][result["worker_command"].index("-m")+1] == owner.MODULE
    assert result["optimizer_updates"]==6000 and result["trainable_state_tensor_count"]==54 and result["frozen_state_tensor_count"]==47
    assert result["controller_target_cap_mps"]==2.4 and result["total_capture_seconds"]==45
    assert result["precursor_motion_screen"]["net_learned_progress_minimum_m"]==90.
    assert result["precursor_motion_screen"]["post_handoff_base_planar_speed_maximum_mps"]==2.5
    assert result["sensor_and_ACK_screen"]["native_state_count"]==900 and result["body_clearance_screen"]["all_snapshot_linear_sweeps"]==899
    assert result["additional_three_trial_benchmark_archive_reserve_bytes"]==3
    assert result["cpu_thread_numerical_preset"]["strict_original_offline_parity_passed"] is False


@pytest.mark.parametrize("failure",[False,True])
def test_owner_aliases_and_thread_environment_restore_on_error(tmp_path,failure):
    proof=tmp_path/"p";proof.write_bytes(b"x")
    watched={name:getattr(owner.previous,name) for name in ("batch_paths","MODULE","SCHEMA","live","fixed_contract","runtime_sources","load_model","diagnostic_report","batch_summary")}
    saved=(owner.base.ASSETS,owner.base.environment,owner.parent.disk_check)
    @contextmanager
    def inherited(p):
        if failure:raise OSError("prior failure preserved")
        yield
    with mock.patch.object(owner,"evidence_assets",return_value={proof:"f"*64}),mock.patch.object(owner,"digest",return_value=owner.FROZEN_OWNER_SHA), \
         mock.patch.object(owner,"configure_threads"),mock.patch.object(owner.torch,"get_num_threads",return_value=1), \
         mock.patch.object(owner.torch,"set_num_threads") as thread_restore,mock.patch.object(owner.previous,"activated",side_effect=inherited):
        try:
            with owner.activated(pins()):
                with pytest.raises(Exception):
                    with owner.activated(pins()):pass
        except OSError:assert failure
    assert all(getattr(owner.previous,name) is value for name,value in watched.items())
    assert (owner.base.ASSETS,owner.base.environment,owner.parent.disk_check)==saved
    thread_restore.assert_called_once_with(1)
    assert not owner._active and owner._evidence_assets is None


def test_readback_denominator_includes_held_and_rejected_rows(tmp_path):
    rows=[dict(command_frame=i,command_reason="next_driving_control") for i in range(3)]
    records=[dict(row,schema=live.THREAD_SCHEMA,intra_op_threads=4,inter_op_threads=1,before_original_propose=True) for row in rows]
    def read(path):return records if path.name=="cpu_thread_readback.jsonl" else rows
    with mock.patch.object(owner.parent,"jsonl",side_effect=read),mock.patch.object(owner,"digest",return_value="a"*64):
        assert owner.readback_report(tmp_path)["passed"]
        records.pop()
        assert not owner.readback_report(tmp_path)["passed"]


@pytest.mark.parametrize("kind",["empty","duplicate","wrong_threads","wrong_schema","wrong_reason"])
def test_readback_never_passes_incomplete_or_changed_proof(tmp_path,kind):
    expected=[dict(command_frame=1,command_reason="next_driving_control")]
    actual=[dict(expected[0],schema=live.THREAD_SCHEMA,intra_op_threads=4,inter_op_threads=1,before_original_propose=True)]
    if kind=="empty":actual=[]
    elif kind=="duplicate":actual*=2
    elif kind=="wrong_threads":actual[0]["intra_op_threads"]=1
    elif kind=="wrong_schema":actual[0]["schema"]="old"
    else:actual[0]["command_reason"]="abort_control"
    with mock.patch.object(owner.parent,"jsonl",side_effect=lambda p:actual if p.name=="cpu_thread_readback.jsonl" else expected), \
         mock.patch.object(owner,"digest",return_value="a"*64):
        assert not owner.readback_report(tmp_path)["passed"]


def proof_fixture(root):
    declared=dict(thread_arms=[1,2,4],interop_threads=1,fixed_camera_ordinals=list(range(55,183)),
        input_spec=dict(final_endpoint=pins()),source_and_asset_sha256={str(root/"live.py"):"a"*64},
        archived_source_and_asset_sha256={str(root/"archive.py"):"b"*64})
    summary=dict(all_raw_decoded_selected_proposed_control_equivalent=False,all_exact_normalized_inputs_equal=True,
        all_failures_retained=True,all_source_and_archive_pins_unchanged=True,actual_physical_latency_or_driving_pass=False,
        attempts=[dict(threads=n,returncode=0,result_sha256=owner.BENCHMARK_PINS["threads_%d.json"%n]) for n in (1,2,4)])
    return declared,summary


@pytest.mark.parametrize("change",[None,"endpoint","empty_archive","path_escape","changed_source","promote_parity","drop_arm","bad_order","physical_claim"])
def test_benchmark_full_live_archive_and_false_parity_proof_required(tmp_path,change):
    declaration,summary=proof_fixture(tmp_path)
    if change=="endpoint":declaration["input_spec"]["final_endpoint"]["weights_sha256"]="f"*64
    elif change=="empty_archive":declaration["archived_source_and_asset_sha256"]={}
    elif change=="path_escape":declaration["source_and_asset_sha256"]={str(tmp_path/"../outside"):"a"*64}
    elif change=="promote_parity":summary["all_raw_decoded_selected_proposed_control_equivalent"]=True
    elif change=="drop_arm":summary["attempts"].pop()
    elif change=="bad_order":declaration["thread_arms"]=[4,2,1]
    elif change=="physical_claim":summary["actual_physical_latency_or_driving_pass"]=True
    benchmark=tmp_path/"benchmark"
    def digest(path):
        path=Path(path)
        if path.parent==benchmark:return owner.BENCHMARK_PINS[path.name]
        return "f"*64 if change=="changed_source" else ("a"*64 if path.name=="live.py" else "b"*64)
    with mock.patch.object(owner,"ROOT",tmp_path),mock.patch.object(owner,"BENCHMARK",benchmark), \
         mock.patch.object(owner,"digest",side_effect=digest), \
         mock.patch.object(owner,"read_json",side_effect=lambda p:declaration if p.name=="declaration.json" else summary):
        if change is None:
            result=owner.evidence_assets(pins())
            assert result[tmp_path/"live.py"]=="a"*64 and result[tmp_path/"archive.py"]=="b"*64
            assert len(result)==7
        else:
            with pytest.raises(Exception):owner.evidence_assets(pins())


def test_complete_new_import_closure_supplements_frozen_26_source_scope(tmp_path):
    path=tmp_path/"new.py";path.write_text("# HH_261002 - Fixture source.\n")
    with mock.patch.object(owner,"ROOT",tmp_path),mock.patch.object(owner,"digest",return_value=owner.FROZEN_OWNER_SHA), \
         mock.patch.object(owner.live,"verify_frozen"),mock.patch.object(owner,"_sources",return_value={"existing.py":"a"*64}), \
         mock.patch.object(owner.base,"imported_source_closure",return_value={path}):
        result=owner.runtime_sources()
    assert result=={"existing.py":"a"*64,"new.py":owner.FROZEN_OWNER_SHA}


@pytest.mark.parametrize("free,passed",[(100,True),(99,False)])
def test_all_three_benchmark_archive_disk_reserve(tmp_path,free,passed):
    proof=tmp_path/"proof";proof.write_bytes(b"1234567890")
    with mock.patch.object(owner,"_evidence_assets",{proof:"a"*64}),mock.patch.object(owner,"_disk",return_value=dict(free_bytes=free,required_bytes=70)):
        if passed:
            result=owner.disk_check();assert result["required_bytes"]==100 and result["benchmark_archive_reserve_bytes"]==30
        else:
            with pytest.raises(Exception):owner.disk_check()
