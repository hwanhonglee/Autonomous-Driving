"""HH_260906 - CPU-only native-reference and frozen-evidence tests; no simulator/model/image execution."""
import copy
import importlib.util
import json
import math
from pathlib import Path

import pytest

from scripts.e2e import audit_carla_native_boundary as audit


def old_test_helpers():
    spec=importlib.util.spec_from_file_location("conditioning_fixture_helpers",Path(__file__).with_name("test_audit_carla_curvature_conditioning.py"))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def state(index,*,yaw=0.,pitch=0.,omega=0.,velocity=(.1,0.,0.)):
    row=old_test_helpers().state(index)
    pose=row["actor_snapshot_transform_carla"]
    pose.update(yaw=yaw,pitch=pitch)
    yc,pc=math.radians(yaw),math.radians(pitch)
    row.update(x=pose["x"]-1.425*math.cos(pc)*math.cos(yc),y=-(pose["y"]-1.425*math.cos(pc)*math.sin(yc)),
        yaw=audit.conditioning.wrap(-yc),world_velocity_carla=list(velocity),world_angular_velocity_carla_deg_s=[0.,0.,omega],
        yaw_rate=-math.radians(omega))
    row["vx"]=math.cos(yc)*velocity[0]+math.sin(yc)*velocity[1]
    row["vy"]=-(-math.sin(yc)*velocity[0]+math.cos(yc)*velocity[1])-1.425*row["yaw_rate"]
    return row


@pytest.fixture
def numerical():
    helper=old_test_helpers()
    states=[helper.state(i) for i in range(134)]
    cameras,futures,published=helper.originals(states)
    return states,cameras,futures,published


def test_existing_planar_correction_is_present_and_exact():
    row=state(0,yaw=90.,omega=10.,velocity=(.2,.3,0.))
    result=audit.planar_velocity_replay(row)
    assert result["existing_planar_formula_exact_match"] is True
    assert result["raw_center_body_forward_left_mps"]==pytest.approx([.3,.2])
    assert result["existing_planar_offset_correction_body_mps"]==pytest.approx([0.,1.425*math.radians(10.)])
    row["vy"]+=1e-16
    with pytest.raises(ValueError,match="formula mismatch"):audit.planar_velocity_replay(row)


def test_signed_left_right_and_trapezoid_not_speed_magnitude():
    result=audit.integrate_interval([.01,-.02],[1.,-2.],[-1.,2.],.05,0.)
    assert result["left_endpoint"]["velocity_integral_world_ros_xy_m"]==[.05,-.1]
    assert result["right_endpoint"]["velocity_integral_world_ros_xy_m"]==[-.05,.1]
    assert result["trapezoid"]["velocity_integral_world_ros_xy_m"]==[0.,0.]
    assert result["trapezoid"]["signed_pose_minus_integral_source_forward_left_m"]==[.01,-.02]


def test_reference_offset_yaw_first_then_pitch_closes_without_angular_assumption():
    first,last=state(0,yaw=-89.,pitch=.1,omega=1.),state(1,yaw=-89.01,pitch=.2,omega=-1.)
    got=audit.native_interval(first,last)
    d=got["pose_offset_decomposition"]
    assert max(map(abs,d["arithmetic_closure_residual_ros_xy_m"]))<1e-12
    assert d["angular_velocity_axis_assumption_added"] is False
    assert d["yaw_first_offset_change_ros_xy_m"]!=[0.,0.]
    assert d["pitch_second_offset_change_ros_xy_m"]!=[0.,0.]
    assert d["observed_virtual_base_minus_actor_delta_ros_xy_m"]==pytest.approx([a+b for a,b in zip(d["yaw_first_offset_change_ros_xy_m"],d["pitch_second_offset_change_ros_xy_m"])],abs=1e-12)


def test_composite_keeps_observed_midpoint_instead_of_one_large_trapezoid():
    first,middle,last=state(0,velocity=(0.,0.,0.)),state(1,velocity=(1.,0.,0.)),state(2,velocity=(0.,0.,0.))
    left,right=audit.native_interval(first,middle),audit.native_interval(middle,last,first)
    got=audit.composite_100ms(first,middle,last,left,right)
    assert got["midpoint_retained"] is True
    assert got["actor_signed_position_velocity"]["estimates"]["trapezoid"]["composite_velocity_integral_world_ros_xy_m"]==[.05,0.]


def test_all_original_flags_rows_and_boundary_neighborhoods_retained(numerical):
    before=copy.deepcopy(numerical)
    result,boundaries,intervals=audit.analyze_case(*numerical)
    assert numerical==before
    assert result["curvature_replay_exact_value_and_flag_match"] is True
    assert len(intervals)==len(numerical[0])-1
    assert result["counts"]["original_violating_window_step_count"]==sum(row["duplicate_window_failure_count"] for row in boundaries)
    assert len(boundaries)==result["counts"]["unique_episode_relative_100ms_violation_tick_count"]
    assert all(row["original_failure_retained"] for row in boundaries)
    for row in boundaries:
        assert len(row["surrounding_native_intervals"])==len(row["surrounding_native_interval_indices"])<=21
        assert row["native_100ms_composite_signed_integration"]["midpoint_retained"] is True


def test_first_point_has_no_fabricated_third_position(numerical):
    states=numerical[0]
    first=audit.exact_window_points(states,states[0]["frame"],0)
    assert len(first["last_up_to_three_points"])==2
    assert first["first_point_uses_body_heading_not_a_third_position"] is True
    interior=audit.exact_window_points(states,states[0]["frame"],2)
    assert len(interior["last_up_to_three_points"])==3
    assert interior["previous_assessed_heading_segment"] is not None
    with pytest.raises(ValueError):audit.exact_window_points(states,states[0]["frame"],-1)


@pytest.mark.parametrize("change",[
    lambda s,c,f,p:s[4].update(frame=99),
    lambda s,c,f,p:s[4].update(timestamp=float("nan")),
    lambda s,c,f,p:s[4].update(vy=s[4]["vy"]+.01),
    lambda s,c,f,p:s[4].update(x=s[4]["x"]+.01),
    lambda s,c,f,p:c[0].update(frame=c[0]["frame"]+1),
    lambda s,c,f,p:c[0].update(timestamp=.00001),
    lambda s,c,f,p:f[0]["diagnostic"]["steps"][0]["metrics"]["xy_curvature"].__setitem__(0,1.),
    lambda s,c,f,p:f[0]["diagnostic"]["steps"][0]["metrics"]["xy_curvature"].__setitem__(1,True),
    lambda s,c,f,p:f[0]["valid_mask"].__setitem__(0,False),
    lambda s,c,f,p:f.pop(),
])
def test_corrupt_native_or_original_future_evidence_fails_closed(numerical,change):
    change(*numerical)
    with pytest.raises(ValueError):audit.analyze_case(*numerical)


def encoded(value):
    return (json.dumps(value,allow_nan=False)+"\n").encode()


def write(path,payload):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(payload)
    return audit.sha(payload)


@pytest.fixture
def files(tmp_path,monkeypatch,numerical):
    sourcepin=write(tmp_path/"source.py",b"pinned source witness\n")
    monkeypatch.setattr(audit,"ROOT",tmp_path)
    monkeypatch.setattr(audit,"PUBLISHED_SOURCE_PATHS",{"source.py"})
    monkeypatch.setattr(audit.conditioning,"MATH_SOURCE_PINS",{"source.py":sourcepin})
    cp=write(tmp_path/"scripts/e2e/audit_carla_curvature_conditioning.py",b"conditioning source identity test\n")
    monkeypatch.setattr(audit,"CONDITIONING_SHA256",cp)
    campaign,evidence=tmp_path/"campaign",tmp_path/"evidence"
    states,cameras,futures,published=numerical
    summaries,windows,cases=[],[],[]
    for case_id,arm in audit.CASES:
        root=campaign/case_id
        raw={"manifest.json":encoded(dict(capture_contract=dict(goal_stop_profile=dict(profile_id="comfortable_v7_hold_brake_ramp",hold_brake_mode=arm,low_speed_steering_mode="zero")),
            result=dict(training_data_approved=False),coordinate_contract=dict(wheelbase_m=2.85))),
            "states.jsonl":b"".join(encoded(row) for row in states),"camera_frames.jsonl":b"".join(encoded(row) for row in cameras)}
        metadata={str(root/"episode"/name):write(root/"episode"/name,payload) for name,payload in raw.items()}
        summaries.append(dict(case_id=case_id,arm=arm,status="COMPLETE_DIAGNOSTIC_NOT_ADMITTED",native_state_count=len(states),camera_anchor_count=len(cameras),
            metadata_sha256=metadata,**copy.deepcopy(published)))
        windows.extend(dict(case_id=case_id,arm=arm,**row) for row in copy.deepcopy(futures))
        cases.append(dict(id=case_id,arm=arm,command=["bash","wrapper",str(root.relative_to(tmp_path))]))
    plan=dict(schema="carla.hold_brake_ramp_abba_plan.v1",cases=cases,training_data_approved=False,gate_changes=False,
        **{category:{"source.py":sourcepin} for category in ("source_sha256","input_sha256","audit_math_sha256","orchestration_sha256")})
    planpath=tmp_path/"plan.json"
    planpin=write(planpath,encoded(plan));monkeypatch.setattr(audit,"PLAN_SHA256",planpin)
    summary=dict(schema="carla.hold_brake_ramp_abba_summary.v1",status="DIAGNOSED_NOT_ADMITTED",plan_sha256=planpin,
        training_data_approved=False,dataset_export=False,target_retiming=False,gate_changes=False,planned_case_count=4,
        cases=summaries,future_anchor_record_count=len(windows),audited_camera_anchor_count=len(windows),source_sha256=dict(files={"source.py":sourcepin}))
    summarypin=write(evidence/"summary.json",encoded(summary));monkeypatch.setattr(audit,"SUMMARY_SHA256",summarypin)
    futurepin=write(evidence/"future_windows.jsonl",b"".join(encoded(row) for row in windows));monkeypatch.setattr(audit,"FUTURE_SHA256",futurepin)
    for name in ("paired_comparisons.json","image_inventory.jsonl","control_policy_witnesses.jsonl"):write(evidence/name,b"unchanged unexecuted prior evidence\n")
    seal=b"".join(f"{audit.sha(path.read_bytes())}  {path.name}\n".encode() for path in sorted(evidence.iterdir()))
    sealpin=write(evidence/"SHA256SUMS",seal);monkeypatch.setattr(audit,"SEAL_SHA256",sealpin)
    return campaign,evidence,planpath,planpin,sealpin,audit.sha(Path(audit.__file__).read_bytes())


def test_full_run_preserves_future_bytes_and_every_native_interval_with_seal(files,tmp_path):
    before=(files[1]/"future_windows.jsonl").read_bytes()
    output=tmp_path/"new_analysis"
    summary=audit.run(*files,output_dir=output)
    assert (output/"future_windows.jsonl").read_bytes()==before
    assert summary["original_anchor_record_count"]==268 and summary["native_state_count"]==536
    assert summary["native_interval_count"]==532
    assert len((output/"native_intervals.jsonl").read_text().splitlines())==532
    assert len((output/"boundary_witnesses.jsonl").read_text().splitlines())==summary["unique_case_endpoint_failure_count"]
    assert summary["scope"]["training_data_approved"] is False and summary["scope"]["images_opened"] is False
    for line in (output/"SHA256SUMS").read_text().splitlines():
        pin,name=line.split("  ");assert audit.sha((output/name).read_bytes())==pin
    with pytest.raises(ValueError,match="fresh output"):audit.run(*files,output_dir=output)


def test_verify_only_writes_nothing(files,tmp_path):
    before=set(tmp_path.rglob("*"))
    audit.run(*files)
    assert set(tmp_path.rglob("*"))==before


@pytest.mark.parametrize("target",["summary.json","future_windows.jsonl","SHA256SUMS"])
def test_frozen_evidence_tamper_rejected(files,target):
    path=files[1]/target
    path.write_bytes(path.read_bytes()+b" ")
    with pytest.raises(ValueError,match="SHA"):audit.run(*files)


def test_source_raw_and_output_alias_refused(files,tmp_path):
    raw=files[0]/audit.CASES[0][0]/"episode/states.jsonl"
    raw.write_bytes(raw.read_bytes()+b" ")
    with pytest.raises(ValueError,match="SHA"):audit.run(*files)
    with pytest.raises(ValueError,match="parent traversal"):audit.safe(tmp_path/"other"/".."/"campaign")
    link=tmp_path/"symlink";link.symlink_to(files[0],target_is_directory=True)
    with pytest.raises(ValueError,match="symlink"):audit.safe(link/"anything")


def test_output_must_not_be_inside_frozen_inputs(files):
    with pytest.raises(ValueError,match="disjoint"):audit.run(*files,output_dir=files[0]/"new")


def test_json_nonfinite_duplicates_and_optional_pin_are_rejected(tmp_path):
    for payload in (b'{"x":1e999}',b'{"x":NaN}',b'{"x":1,"x":2}'):
        with pytest.raises(ValueError):audit.loads(payload)
    path=tmp_path/"file";path.write_bytes(b"value")
    with pytest.raises(ValueError):audit.read(path,None,{})
    ledger={str(path):"a"*64}
    with pytest.raises(ValueError,match="between reads"):audit.read(path,audit.sha(path.read_bytes()),ledger)

