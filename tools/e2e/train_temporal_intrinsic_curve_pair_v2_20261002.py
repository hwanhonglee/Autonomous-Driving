"""HH_261002 - NEW GPU0 driver-residual preflight; all frozen paired TRAIN mathematics stay unchanged."""
from __future__ import annotations
from contextlib import contextmanager
from pathlib import Path

if __name__ == "__mp_main__":
    # HH_261002 - Spawn workers reconstruct canonical namespaces only; they never initialize CUDA.
    import importlib.util
    _entry=Path(__file__).resolve().parents[1]/"workspace_python.py"
    _spec=importlib.util.spec_from_file_location("hh_intrinsic_pair_v2_worker",_entry)
    _module=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_module)
    _module.configure(Path(__file__).resolve().parents[2])

from scripts.e2e import train_temporal_intrinsic_curve_pair_20261002 as pair

ROOT=Path(__file__).resolve().parents[2]
SCHEMA="portable_e2e.temporal_intrinsic_curve_gpu_preflight_wrapper.20261002.v2"
original,require,digest,write_new,PARENT=pair.original,pair.require,pair.digest,pair.write_new,pair.PARENT
validate_authorization,authorize_parent_history=pair.validate_authorization,pair.authorize_parent_history
GPU_REVISION=dict(schema="portable_e2e.gpu0_driver_residual_preflight.20261002.v2",
    physical_index=0,uuid=pair.raw_parent.GPU_UUID,maximum_driver_residual_MiB=16,
    required_utilization_percent=0,required_GPU0_compute_process_count=0,
    physical_queries_before_torch_initialization=True,original_gpu0_lease_unchanged=True,
    training_math_or_labels_changed=False)
FROZEN_PAIR_SHA="3da40c323c69b1d3e1a64b77f68955b4a8c169a895b5698db312f86d4b23c0c8"
FROZEN_GPU_PROVIDER_SHA="85b24d472112364e8c3a468bbe7849c10f4a802a3f7c2ea391c1049ddda5648e"
_source_snapshot,_contract,_gpu_preflight=pair.source_snapshot,pair.contract,pair.raw_parent.gpu_preflight
_active=False


def verify_frozen():
    pair.require(pair.digest(Path(pair.__file__))==FROZEN_PAIR_SHA
        and pair.digest(Path(pair.raw_parent.__file__))==FROZEN_GPU_PROVIDER_SHA,
        "original paired trainer and GPU provider must remain byte-identical")


def source_snapshot():
    verify_frozen();old=_source_snapshot()
    paths=set(pair.replay.imported_source_closure(ROOT,[Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py"]))
    paths.add(ROOT/"learning/portable_e2e/config/common_10hz_v1.contract.json")
    result={str(path.relative_to(ROOT)):pair.digest(path) for path in sorted(paths)}
    pair.require(len(old)==28 and len(result)==29 and all(result.get(name)==sha for name,sha in old.items()),
        "NEW wrapper plus all28 immutable learning sources required")
    return result


def contract():
    # HH_261002 - Public contract mode and dispatch inspection are also truthful outside the activated fit scope.
    return dict(_contract(),sources=source_snapshot(),gpu_preflight_revision=dict(GPU_REVISION))


def gpu_preflight():
    # HH_261002 - No CUDA initialization before both physical GPU0 queries prove no competing compute process.
    provider=pair.raw_parent;require=pair.require
    require(pair.os.environ.get("CUDA_VISIBLE_DEVICES")==provider.GPU_UUID,"only exact physical GPU0 UUID may be visible")
    require(not pair.torch.cuda.is_initialized(),"physical idle evidence must precede CUDA initialization")
    row=provider.subprocess.check_output(["nvidia-smi","-i","0",
        "--query-gpu=index,uuid,memory.used,utilization.gpu","--format=csv,noheader,nounits"],text=True,timeout=15).strip().split(",")
    processes=provider.subprocess.check_output(["nvidia-smi","-i","0",
        "--query-compute-apps=gpu_uuid,pid","--format=csv,noheader,nounits"],text=True,timeout=15)
    require(len(row)==4 and row[0].strip()=="0" and row[1].strip()==provider.GPU_UUID
        and 0<=int(row[2])<=16 and int(row[3])==0 and not processes.strip(),
        "physical GPU0 requires <=16MiB driver residual, zero utilization and NO compute process")
    require(pair.torch.cuda.is_available() and pair.torch.cuda.device_count()==1,"exactly one approved CUDA GPU required")
    properties=pair.torch.cuda.get_device_properties(0)
    require(str(properties.uuid) in (provider.GPU_UUID,provider.GPU_UUID.removeprefix("GPU-")),"Torch physical GPU UUID differs")
    return dict(physical_index=0,uuid=provider.GPU_UUID,name=properties.name,idle_before_torch=True,
        driver_residual_MiB=int(row[2]),GPU0_compute_process_count=0,gpu_preflight_revision=dict(GPU_REVISION))


@contextmanager
def activated():
    global _active
    pair.require(not _active and pair.source_snapshot is _source_snapshot and pair.contract is _contract
        and pair.raw_parent.gpu_preflight is _gpu_preflight,"nested or unknown paired/GPU preflight scope prohibited")
    verify_frozen();_active=True
    try:
        pair.source_snapshot,pair.contract,pair.raw_parent.gpu_preflight=source_snapshot,contract,gpu_preflight
        yield
    finally:
        pair.source_snapshot,pair.contract,pair.raw_parent.gpu_preflight=_source_snapshot,_contract,_gpu_preflight
        _active=False


def main(argv=None):
    with activated():return pair.main(argv)


if __name__=="__main__":raise SystemExit(main())
