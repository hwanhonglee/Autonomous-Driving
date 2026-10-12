"""HH_260906 - Test only small synthetic fixtures and mocked campaign persistence, never captured data or training."""

from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import signal

import pytest
import torch

from scripts.e2e import probe_portable_stop_primitive as probe


@pytest.fixture(autouse=True)
def cpu_only(monkeypatch):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "")
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def test_fixed_plan_preserves_all_speeds_profiles_dtypes_and_runtime():
    plan = probe.build_plan()
    assert plan["case_count"] == 100 and plan["candidate_count"] == 600
    assert plan["future_point_count"] == 38400
    assert len(plan["current_speed_grid_mps"]) == 10 and len(plan["duration_logits_grid"]) == 5
    assert plan["dtypes"] == ["float32", "float64"]
    assert plan["profiles"][3]["curvature_logits"] == [20., -20.] * 32
    sine = plan["profiles"][4]["curvature_logits"]
    assert sine[0] == 0. and abs(sine[-1]) < 1e-14
    assert sine[16] == pytest.approx(2*math.sin(2*math.pi*16/63))
    assert plan["source_pins"]["portable_e2e/runtime_contract.py"]["sha256"] == probe.RUNTIME_SHA
    assert plan["scope"]["stop_decision_assumed_by_caller"] is True
    assert not plan["scope"]["training_data_approved"] and not plan["scope"]["optimizer"]


@pytest.mark.parametrize("visible", [None, "0", "1", "GPU-unknown"])
def test_cpu_visibility_is_explicit(visible,monkeypatch):
    if visible is None: monkeypatch.delenv("CUDA_VISIBLE_DEVICES", raising=False)
    else: monkeypatch.setenv("CUDA_VISIBLE_DEVICES", visible)
    with pytest.raises(ValueError, match="hidden CUDA"): probe.cpu_guard()


def test_initialized_cuda_is_forbidden(monkeypatch):
    monkeypatch.setattr(torch.cuda, "is_initialized", lambda: True)
    with pytest.raises(ValueError, match="uninitialized"): probe.cpu_guard()


def test_runtime_source_cannot_be_substituted(monkeypatch):
    monkeypatch.setattr(probe, "sha", lambda *_: "0"*64)
    with pytest.raises(ValueError, match="gate substitution"): probe.build_plan()


@pytest.mark.parametrize("dtype,speed,duration", [("float32", .075, -20.), ("float64", 1., 0.)])
def test_small_real_synthetic_case_replays_all64_points_without_mutation(dtype,speed,duration):
    rng = torch.get_rng_state().clone()
    row = probe.run_case(dtype,speed,duration,7)
    assert torch.equal(torch.get_rng_state(), rng)
    assert row["case_index"] == 7 and row["input_tensors_unchanged"] is True
    assert len(row["candidates"]) == 6
    for index,candidate in enumerate(row["candidates"]):
        assert len(candidate["xy_base_m"]) == len(candidate["speed_mps"]) == len(candidate["heading_rad"]) == 64
        assert candidate["runtime_selection_logits"] == [float(i == index) for i in range(6)]
        assert candidate["independent_reference"]["compared_scalar_count"] == 257
        assert candidate["independent_reference"]["status"] == "MATCH"
        assert candidate["numerical_invariants"]["status"] == "PASS"
        assert candidate["speed_mps"][-1] == 0.
        assert candidate["runtime_gate"]["status"] in ("PASS", "FAIL")


def test_independent_straight_reference_is_discrete_not_continuous():
    value = probe.independent_reference(1.,0.,[0.]*64)
    duration = (max(.1,1./2.9)+6.4)/2
    assert value["stop_duration_s"] == duration
    assert value["xy_base_m"][-1][0] == pytest.approx(sum(max(0.,1.-.1*(i+1)/duration)*.1 for i in range(64)))
    assert value["xy_base_m"][-1][0] < .5*duration
    assert all(point[1] == 0. for point in value["xy_base_m"])


def test_mismatched_reference_is_reported_without_relaxing_tolerance():
    ref = probe.independent_reference(.4,0.,[0.]*64)
    actual = json.loads(json.dumps(ref)); actual["xy_base_m"][20][0] += .01
    result = probe.compare_reference(actual,ref,torch.finfo(torch.float64).eps)
    assert result["status"] == "UNVERIFIED" and result["mismatch_scalar_count"] == 1
    assert result["mismatches"][0]["scalar_index"] == 40


def test_runtime_selects_every_slot_and_retains_all_rejections(monkeypatch):
    calls = []
    def reject(xy,speed,logits,**kwargs):
        calls.append((logits,kwargs)); raise probe.ContractError("synthetic fixture rejection")
    monkeypatch.setattr(probe.runtime,"validate_and_select_trajectory",reject)
    row = probe.run_case("float64",.1,0.,0)
    assert len(calls) == 6 and len(row["candidates"]) == 6
    assert all(c["runtime_gate"] == {"status":"FAIL", "reason":"synthetic fixture rejection", "selected_index":i}
        for i,c in enumerate(row["candidates"]))
    assert probe.summarize([row])["runtime_gate_counts"] == {"FAIL":6}


@pytest.mark.parametrize("fault", ["shape", "nan", "gradient", "mutated_input"])
def test_primitive_output_contract_rejected(fault,monkeypatch):
    original = probe.primitive.decode_stop_primitive
    def bad(current,duration,curvature):
        value = original(current,duration,curvature)
        if fault == "shape": return replace(value,speed_mps=value.speed_mps[:,:,:63])
        if fault == "nan":
            value.xy_base_m[0,0,0,0] = float('nan'); return value
        if fault == "gradient": return replace(value,speed_mps=value.speed_mps.requires_grad_())
        current.add_(1.); return value
    monkeypatch.setattr(probe.primitive,"decode_stop_primitive",bad)
    with pytest.raises(ValueError,match="ABI|mutated"): probe.run_case("float64",1.,0.,0)


def fake_case(dtype,speed,logit,index):
    # HH_260906 - Mock campaign rows exercise persistence only; they are never published as actual primitive outputs.
    return {"case_index":index,"dtype":dtype,"requested_current_speed_mps":speed,"duration_logit":logit,
        "candidates":[{"candidate_index":i,"independent_reference":{"compared_scalar_count":257,"status":"MATCH"},
            "numerical_invariants":{"status":"PASS"},"runtime_gate":{"status":"FAIL","reason":"unit-fixture"}}
            for i in range(6)]}


@pytest.fixture
def declared(tmp_path,monkeypatch):
    path = tmp_path/"declaration.json"
    path.write_text(json.dumps(probe.build_plan()))
    monkeypatch.setattr(probe,"run_case",fake_case)
    return path,tmp_path/"output"


def test_mock_complete_campaign_counts_and_sources_with_no_real_grid(declared):
    plan,output = declared
    before = {name:signal.getsignal(name) for name in (signal.SIGINT,signal.SIGTERM)}
    threads = torch.get_num_threads()
    result = probe.run_probe(plan,output)
    assert result["status"] == "COMPLETE_NOT_ADMITTED"
    assert result["reported_case_count"] == 100 and result["reported_candidate_count"] == 600
    assert result["runtime_gate_counts"] == {"FAIL":600}
    assert result["compared_scalar_count"] == 154200
    assert (output/"plan.json").read_bytes() == plan.read_bytes()
    assert all(signal.getsignal(name)==handler for name,handler in before.items())
    assert torch.get_num_threads() == threads
    for line in (output/"SHA256SUMS").read_text().splitlines():
        digest,name = line.split("  "); assert probe.sha(output/name) == digest


@pytest.mark.parametrize("fault", ["existing", "symlink", "dataset_alias", "plan_changed", "plan_symlink"])
def test_bad_plan_or_output_rejected_before_execution(declared,tmp_path,monkeypatch,fault):
    plan,output = declared
    if fault == "existing": output.mkdir()
    elif fault == "symlink": output.symlink_to(tmp_path/"absent",target_is_directory=True)
    elif fault == "dataset_alias":
        repo=tmp_path/"repo"; repo.mkdir(); data=tmp_path/"data"; data.mkdir()
        (repo/"datasets").symlink_to(data,target_is_directory=True)
        monkeypatch.setattr(probe,"ROOT",repo); output=data/"child"
    elif fault == "plan_changed":
        value=json.loads(plan.read_text()); value["current_speed_grid_mps"].pop(); plan.write_text(json.dumps(value))
    else:
        link=tmp_path/"linked.json"; link.symlink_to(plan); plan=link
    monkeypatch.setattr(probe,"run_case",lambda *_:pytest.fail("must refuse before decode"))
    with pytest.raises(ValueError): probe.run_probe(plan,output)


@pytest.mark.parametrize("kind", ["exception", "signal", "plan_mutation", "startup_write"])
def test_failed_partial_stays_unadmitted_and_restores_handlers(declared,monkeypatch,kind):
    plan,output = declared
    handlers={name:signal.getsignal(name) for name in (signal.SIGINT,signal.SIGTERM)}
    if kind == "startup_write":
        original=probe.write_json
        def fail_start(path,value):
            if path.name == "started.json": raise OSError("unit startup failure")
            return original(path,value)
        monkeypatch.setattr(probe,"write_json",fail_start)
    else:
        def fail(dtype,speed,logit,index):
            if index==2:
                if kind=="exception": raise ValueError("unit stopped")
                if kind=="signal": signal.raise_signal(signal.SIGTERM)
                if kind=="plan_mutation": plan.write_text('{}')
            return fake_case(dtype,speed,logit,index)
        monkeypatch.setattr(probe,"run_case",fail)
    with pytest.raises((ValueError,OSError)): probe.run_probe(plan,output)
    result=json.loads((output/"summary.json").read_text())
    assert result["status"] == "FAILED_PARTIAL_NOT_ADMITTED"
    assert result["reported_case_count"] == {"exception":2,"signal":3,"plan_mutation":100,"startup_write":0}[kind]
    assert all(signal.getsignal(name)==handler for name,handler in handlers.items())
    assert result["scope"]["training_data_approved"] is False


def test_persisted_tail_keeps_every_complete_row(tmp_path):
    path=tmp_path/"rows.jsonl"
    row=probe.canonical(fake_case("float32",0.,0.,0))
    path.write_bytes((row+'\n'+row[:20]).encode())
    rows,ledger=probe.persisted_rows(path)
    assert len(rows)==1 and ledger=={"complete_record_count":1,"unparsed_tail_bytes":20}


@pytest.mark.parametrize("raw", ['{"a":1,"a":2}', '{"x":NaN}', '{"x":Infinity}'])
def test_invalid_plan_json_is_rejected(raw):
    with pytest.raises(ValueError): probe.read_json(raw)
