"""HH_261002 - Test fixed offline thread-study selection/provenance/parity and exact restored telemetry without NN runs."""

import ast
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import profile_temporal_route_joint_cpu_threads_20261002 as profile


@pytest.mark.parametrize("start,count",[(11,1),(40,64),(322,128),(449,1)])
def test_fixed_selection_preserves_every_prospectively_declared_anchor(start,count):
    assert profile.fixed_ordinals(start,count) == tuple(range(start,start+count))


@pytest.mark.parametrize("start,count",[(True,20),(40,True),(10,20),(450,1),(40,0),(40,129),(400,51),(40,1.0)])
def test_timestamp_or_quality_selected_missing_history_or_noninteger_bounds_rejected(start,count):
    with pytest.raises(ValueError):profile.fixed_ordinals(start,count)


@pytest.mark.parametrize("failure",[False,True])
def test_telemetry_calls_each_exact_function_once_and_preserves_error_and_restoration(failure):
    calls=[]; stages=[]
    def original(value):
        calls.append(value)
        if failure:raise RuntimeError("original safety rejection")
        return value+1
    target=SimpleNamespace(function=original)
    try:
        with profile.measured_calls([(target,"function","guard")],stages):
            assert target.function.__wrapped__ is original
            with pytest.raises(ValueError,match="nonnested"):
                with profile.measured_calls([],[]):pass
            assert target.function(3)==4
    except RuntimeError as error:
        assert failure and str(error)=="original safety rejection"
    assert calls==[3] and target.function is original and not profile._timing_active
    assert profile.stage_summary(stages)["guard"]["calls"]==1


def decision():
    return dict(owned_camera_history=dict(normalized_input_tensor_sha256={"images":"a"*64,"route_xy":"b"*64}),
        raw_output=dict(curvature_knots=[0.,.1],preferred_speed_knots=[1.,2.]),
        decoded_output=dict(candidate_xy=[[1.,0.],[2.,.1]],candidate_speed=[1.,2.]),
        selected=dict(index=0,xy_base_m=[[1.,0.],[2.,.1]],speed_mps=[1.,2.]),
        longitudinal_pi_research=dict(proposed_normal_tracker_control=dict(throttle=.2,brake=0.,steer=.01)),
        learned_control_proposed=True,control=dict(throttle=.2,brake=0.,steer=.01))


def arms():
    return [dict(threads=value,rows=[dict(camera_ordinal=40,decision=decision())]) for value in profile.THREAD_ARMS]


def test_same_input_and_predeadline_control_parity_is_not_a_rejected_actuation_pass():
    values=arms();values[1]["rows"][0]["decision"].update(learned_control_proposed=False,control=dict(throttle=0.,brake=1.,steer=0.))
    result=profile.compare_arms(values)
    assert all(row["all_exact_normalized_inputs_equal"] and row["all_raw_decoded_selected_proposed_control_equivalent"] for row in result)
    assert not result[1]["rows"][0]["actual_fail_closed_decision_equivalence_claimed"]


@pytest.mark.parametrize("mutation",["input_SHA","raw","decoded","selected","PI","missing_NN","missing_PI"])
def test_any_input_or_mathematical_difference_or_missing_evidence_is_retained_as_failure(mutation):
    values=arms();target=values[2]["rows"][0]["decision"]
    if mutation=="input_SHA":target["owned_camera_history"]["normalized_input_tensor_sha256"]["images"]="f"*64
    elif mutation=="raw":target["raw_output"]["curvature_knots"][0]=.001
    elif mutation=="decoded":target["decoded_output"]["candidate_xy"][0][0]+=.001
    elif mutation=="selected":target["selected"]["index"]=1
    elif mutation=="PI":target["longitudinal_pi_research"]["proposed_normal_tracker_control"]["throttle"]+=.001
    elif mutation=="missing_NN":target["raw_output"]=None
    else:target.pop("longitudinal_pi_research")
    report=profile.compare_arms(values)[2]
    assert not (report["all_exact_normalized_inputs_equal"] and report["all_raw_decoded_selected_proposed_control_equivalent"])


@pytest.mark.parametrize("mutation",["empty","order","missing_arm","ordinal","missing_anchor"])
def test_fixed_arm_and_anchor_denominators_cannot_be_shortened_or_permuted(mutation):
    values=arms()
    if mutation=="empty":
        for value in values:value["rows"]=[]
    elif mutation=="order":values.reverse()
    elif mutation=="missing_arm":values.pop()
    elif mutation=="ordinal":values[1]["rows"][0]["camera_ordinal"]=41
    else:values[2]["rows"]=[]
    with pytest.raises(ValueError):profile.compare_arms(values)


def test_numeric_tolerance_is_exactly_one_micro_unit_relative_and_absolute_no_nonfinite_or_boolean_coercion():
    assert profile.equivalent(1.,1.0000019)
    assert not profile.equivalent(1.,1.0000021)
    assert not profile.equivalent(float("nan"),float("nan"))
    assert not profile.equivalent(True,1) and not profile.equivalent(None,None)


def test_summary_is_nonvacuous_and_retains_full_latency_values():
    assert profile.stats([])==dict(count=0,mean=None,p95=None,maximum=None)
    assert profile.stats([.08,.1,.12])["maximum"]==.12
    with pytest.raises(ValueError):profile.stats([float("inf")])


def observations(tmp_path):
    states=[dict(frame=6000+i,timestamp=1.+i*.05) for i in range(900)]
    cameras=[]
    for i in range(450):
        stamp=states[i*2]["timestamp"]
        cameras.append(dict(frame=6000+i*2,timestamp=stamp,camera_order=list(profile.MODEL_CAMERA_ORDER),
            images={name:f"{name}/{i:06d}.jpg" for name in profile.MODEL_CAMERA_ORDER},
            source_timestamps={name:stamp for name in profile.MODEL_CAMERA_ORDER}))
    for name,values in (("states.jsonl",states),("camera_frames.jsonl",cameras)):
        (tmp_path/name).write_text("".join(json.dumps(row)+"\n" for row in values))
    return states,cameras


def test_actual_all900_450_frame_and_causal_timestamp_chains_are_required(tmp_path):
    expected=observations(tmp_path)
    assert profile.observation_rows(tmp_path)==expected


@pytest.mark.parametrize("mutation",["native_drop","camera_drop","display_order","missing_cam","duplicate_pixel_file","future_pose_time","clock","frame"])
def test_observation_chain_and_exact_writer_order_reject_unknown_variants(tmp_path,mutation):
    states,cameras=observations(tmp_path)
    if mutation=="native_drop":states.pop()
    elif mutation=="camera_drop":cameras.pop()
    elif mutation=="display_order":cameras[0]["camera_order"].reverse()
    elif mutation=="missing_cam":cameras[0]["images"].pop(profile.MODEL_CAMERA_ORDER[0])
    elif mutation=="duplicate_pixel_file":cameras[1]["images"]=dict(cameras[0]["images"])
    elif mutation=="future_pose_time":cameras[0]["source_timestamps"][profile.MODEL_CAMERA_ORDER[0]]+=.05
    elif mutation=="clock":states[5]["timestamp"]+=.001
    else:cameras[5]["frame"]+=1
    for name,values in (("states.jsonl",states),("camera_frames.jsonl",cameras)):
        (tmp_path/name).write_text("".join(json.dumps(row)+"\n" for row in values))
    with pytest.raises(ValueError):profile.observation_rows(tmp_path)


def test_no_torch_numpy_model_or_simulator_import_occurs_in_declaration_module():
    tree=ast.parse(Path(profile.__file__).read_text())
    imports=[name.name for node in tree.body if isinstance(node,ast.Import) for name in node.names]
    assert "torch" not in imports and "numpy" not in imports
    assert not any(isinstance(node,ast.ImportFrom) and node.module and node.module.startswith("portable_e2e") for node in tree.body)


def test_create_only_json_preserves_original_failure_and_refuses_overwrite(tmp_path):
    path=tmp_path/"failure.json";original=dict(accepted=False,reason="deadline",MAIN9=0)
    profile.write_new(path,original)
    with pytest.raises(FileExistsError):profile.write_new(path,dict(accepted=True))
    assert json.loads(path.read_text())==original


def test_camera_members_cannot_escape_owned_episode_or_accept_absolute_paths(tmp_path):
    for value in ("../outside.jpg","/outside.jpg"):
        with pytest.raises(ValueError):profile.camera_path(tmp_path,value)


def actual_controller():
    return dict(wheelbase_m=2.819186218917395,maximum_steer_angle_rad=.6108652381980153,speed_cap_mps=2.5,
        minimum_lookahead_m=1.5,lookahead_time_s=.8,speed_preview_s=.3,throttle_gain=.7,brake_gain=.7,maximum_throttle=.5)


def test_actual_native_physics_controller_is_not_replaced_by_nominal_rear_reference_geometry():
    value=actual_controller();actual=profile.recorded_controller(dict(controller=value))
    assert actual==value and actual["wheelbase_m"]!=2.85 and actual is not value


@pytest.mark.parametrize("mutation",["missing","unknown","bool","NaN","speed","steer","throttle","preview"])
def test_all_nine_original_controller_fields_and_bounds_remain_strict(mutation):
    value=actual_controller()
    if mutation=="missing":value.pop("brake_gain")
    elif mutation=="unknown":value["nominal_reference_wheelbase"]=2.85
    elif mutation=="bool":value["wheelbase_m"]=True
    elif mutation=="NaN":value["wheelbase_m"]=float("nan")
    elif mutation=="speed":value["speed_cap_mps"]=2.6
    elif mutation=="steer":value["maximum_steer_angle_rad"]=2.
    elif mutation=="throttle":value["maximum_throttle"]=1.1
    else:value["speed_preview_s"]=6.5
    with pytest.raises(ValueError):profile.recorded_controller(dict(controller=value))


def test_declare_does_not_invoke_models_or_children_before_caller_pin_validation(tmp_path):
    with mock.patch.object(profile,"digest",return_value="a"*64),mock.patch.object(profile,"worker") as worker, \
            mock.patch.object(profile,"execute") as execute:
        with pytest.raises(ValueError,match="caller input"):
            profile.declare(tmp_path/"fresh",tmp_path/"inputs.json","b"*64)
    worker.assert_not_called();execute.assert_not_called()


def test_actual_canonical_workspace_parser_forwards_worker_options_only_after_remainder_separator(tmp_path):
    command=profile.worker_command(tmp_path,"a"*64,2)
    assert command[command.index("-m")+1:command.index("--worker")]==[profile.MODULE,"--"]
    spec=importlib.util.spec_from_file_location("HH_thread_probe_workspace_parser",profile.ROOT/"tools/workspace_python.py")
    runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
    captured=[]
    with mock.patch.object(runner,"configure"),mock.patch.object(runner._adapter,"execute",side_effect=lambda path,root,args,**kw:captured.append(args)):
        runner.main(command[command.index("-m"):])
    assert captured==[["--worker","--output",str(tmp_path),"--declaration-sha256","a"*64,"--threads","2"]]


def test_complete_exact26_metadata_sources_are_explicitly_added_before_declaration_archive_not_import_glob():
    tree=ast.parse(Path(profile.__file__).read_text())
    function=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=="declare")
    text=ast.unparse(function)
    assert 'data[\'metadata\'][\'source_sha256\'].items()' in text
    assert "source_path(ROOT / name)" in text and "digest(path) == sha" in text and "sources.add(path)" in text
    assert "rglob" not in text and "glob(" not in text
