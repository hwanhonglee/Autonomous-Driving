"""HH_261002 - Pure fake GPU queries/scoped source contracts; no actual CUDA, model, data, or optimizer."""
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from scripts.e2e import train_temporal_intrinsic_curve_pair_v2_20261002 as revision


@pytest.fixture
def fake_gpu(monkeypatch):
    events=[];values=dict(memory=1,utilization=0,processes="",index="0",uuid=revision.pair.raw_parent.GPU_UUID,
        initialized=False,available=True,count=1,torch_uuid=revision.pair.raw_parent.GPU_UUID)
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES",revision.pair.raw_parent.GPU_UUID)
    def query(argv,**kwargs):
        assert argv[:3]==["nvidia-smi","-i","0"] and kwargs==dict(text=True,timeout=15)
        events.append("query")
        return values["processes"] if "--query-compute-apps=gpu_uuid,pid" in argv else ",".join(str(values[k]) for k in ("index","uuid","memory","utilization"))
    def initialized():events.append("is_initialized");return values["initialized"]
    def available():events.append("CUDA_available");return values["available"]
    def count():events.append("CUDA_count");return values["count"]
    def properties(index):
        assert index==0;events.append("CUDA_properties");return SimpleNamespace(uuid=values["torch_uuid"],name="fakeGPU0")
    monkeypatch.setattr(revision.pair.raw_parent.subprocess,"check_output",query)
    monkeypatch.setattr(revision.pair.torch,"cuda",SimpleNamespace(is_initialized=initialized,is_available=available,
        device_count=count,get_device_properties=properties))
    return values,events


@pytest.mark.parametrize("memory",[0,1,16])
def test_driver_residual_only_no_compute_process_and_idle_before_torch(fake_gpu,memory):
    values,events=fake_gpu;values["memory"]=memory
    result=revision.gpu_preflight()
    assert events==["is_initialized","query","query","CUDA_available","CUDA_count","CUDA_properties"]
    assert result["physical_index"]==0 and result["driver_residual_MiB"]==memory
    assert result["GPU0_compute_process_count"]==0 and result["idle_before_torch"] is True
    assert result["gpu_preflight_revision"]==revision.GPU_REVISION


@pytest.mark.parametrize("changes",[{"memory":17},{"memory":-1},{"memory":"N/A"},{"utilization":1},
    {"processes":"GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5, 123\n"},{"memory":0,"processes":"GPU0, 123\n"},
    {"processes":"N/A\n"},{"index":"1"},{"uuid":"GPU-other"}])
def test_competing_process_or_unproven_idle_rejected_before_cuda(fake_gpu,changes):
    values,events=fake_gpu;values.update(changes)
    with pytest.raises((ValueError,RuntimeError)):revision.gpu_preflight()
    assert not any(event.startswith("CUDA_") for event in events)


def test_wrong_visible_device_rejected_before_any_physical_or_torch_query(fake_gpu,monkeypatch):
    _,events=fake_gpu;monkeypatch.setenv("CUDA_VISIBLE_DEVICES","1")
    with pytest.raises(ValueError,match="exact physical GPU0"):revision.gpu_preflight()
    assert events==[]


def test_existing_cuda_context_is_not_idle_bootstrap_proof(fake_gpu):
    values,events=fake_gpu;values["initialized"]=True
    with pytest.raises(ValueError,match="precede CUDA"):revision.gpu_preflight()
    assert events==["is_initialized"]


@pytest.mark.parametrize("changes",[{"available":False},{"count":2},{"torch_uuid":"GPU-other"}])
def test_torch_must_confirm_one_exact_physical_gpu(fake_gpu,changes):
    values,_=fake_gpu;values.update(changes)
    with pytest.raises(ValueError):revision.gpu_preflight()


def test_torch_uuid_prefix_representation_matches_original_contract(fake_gpu):
    values,_=fake_gpu;values["torch_uuid"]=values["uuid"].removeprefix("GPU-")
    assert revision.gpu_preflight()["physical_index"]==0


def test_public_contract_adds_only_disclosed_preflight_revision_and_one_source():
    original=revision._contract();new=revision.contract();pins=revision.source_snapshot()
    assert len(original["sources"])==28 and len(pins)==29 and new["sources"]==pins
    assert all(pins[name]==sha for name,sha in original["sources"].items())
    assert new["gpu_preflight_revision"]==revision.GPU_REVISION
    new.pop("gpu_preflight_revision");new["sources"]=original["sources"]
    assert new==original and new["fixed_new_updates_per_arm"]==6000
    assert new["trainable_tensor_count"]==54 and new["frozen_tensor_count"]==47
    assert new["coefficient"]==.05 and new["examples_per_arm"]==24000
    assert not revision.pair.torch.cuda.is_initialized()


def test_scoped_main_delegates_exact_frozen_pair_and_original_lease(monkeypatch):
    old=(revision.pair.source_snapshot,revision.pair.contract,revision.pair.raw_parent.gpu_preflight)
    lease=revision.pair.raw_parent.gpu0_lease;calls=[]
    def main(argv):
        calls.append(argv)
        assert revision.pair.source_snapshot is revision.source_snapshot
        assert revision.pair.contract is revision.contract and revision.pair.raw_parent.gpu_preflight is revision.gpu_preflight
        assert revision.pair.raw_parent.gpu0_lease is lease
        assert revision.pair.contract()["sources"]==revision.source_snapshot()
        return 9
    monkeypatch.setattr(revision.pair,"main",main)
    assert revision.main(["--mode","contract"])==9 and calls==[["--mode","contract"]]
    assert old==(revision.pair.source_snapshot,revision.pair.contract,revision.pair.raw_parent.gpu_preflight)
    assert not revision._active


def test_exception_restores_all_scope_bindings():
    old=(revision.pair.source_snapshot,revision.pair.contract,revision.pair.raw_parent.gpu_preflight)
    with pytest.raises(RuntimeError,match="fixture"):
        with revision.activated():raise RuntimeError("fixture")
    assert old==(revision.pair.source_snapshot,revision.pair.contract,revision.pair.raw_parent.gpu_preflight)
    assert not revision._active


def test_nested_scope_rejected_and_outer_scope_restored():
    with revision.activated():
        with pytest.raises(ValueError,match="nested"):revision.main(["--mode","contract"])
        assert revision.pair.raw_parent.gpu_preflight is revision.gpu_preflight
    assert revision.pair.raw_parent.gpu_preflight is revision._gpu_preflight


@pytest.mark.parametrize("module,name",[(revision.pair,"source_snapshot"),(revision.pair,"contract"),(revision.pair.raw_parent,"gpu_preflight")])
def test_unknown_preexisting_binding_rejected_without_overwrite(monkeypatch,module,name):
    unknown=lambda:None;monkeypatch.setattr(module,name,unknown)
    with pytest.raises(ValueError,match="unknown"):revision.main(["--mode","contract"])
    assert getattr(module,name) is unknown and not revision._active


def test_actual_public_main_contract_prints_new29_source_protocol_without_cuda(capsys):
    assert revision.main(["--mode","contract"])==0
    result=json.loads(capsys.readouterr().out)
    assert len(result["sources"])==29 and result["gpu_preflight_revision"]==revision.GPU_REVISION
    assert result["schema"]==revision.pair.SCHEMA and result["coefficient"]==.05
    assert not revision.pair.torch.cuda.is_initialized()


def test_original_pair_and_gpu_provider_source_pins_match_exact_files():
    assert revision.digest(Path(revision.pair.__file__))==revision.FROZEN_PAIR_SHA
    assert revision.digest(Path(revision.pair.raw_parent.__file__))==revision.FROZEN_GPU_PROVIDER_SHA
