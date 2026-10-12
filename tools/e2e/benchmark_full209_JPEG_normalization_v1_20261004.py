"""HH_261004 - Prepare whole918x6 lossy-JPEG normalization byte parity and a sealed thirty-pair CPU counterfactual, never a BGRA/live proof."""

from __future__ import annotations

import argparse
import ast
import copy
from dataclasses import asdict
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import sys
import time

from .benchmark_scandir_episode_quota_v1_20261004 import digest, require, safe_path, tree_members, write_new_json, read_json


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_full209_JPEG_normalization_v1_20261004.py"
RUN = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v1"
EPISODE = RUN / "episode"
CALLER = ROOT / "storage/experiments/plans/2026-10-04/town03_known_dev_straight_full209_v1/ROOT_actual_publication_caller.json"
CALLER_SHA = "021cec4b62777dbceaa0c053c01b2e7afa03b92d85ad7e4c758f540c416bf209"
CALLER_SCHEMA = "portable_e2e.actual_town03_known_dev_straight_full209_publication.20261004.v1.caller_pins"
PLAN = ROOT / "storage/experiments/plans/2026-10-04/full209_JPEG_normalization_counterfactual_v1/declaration.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-04/full209_JPEG_normalization_counterfactual_v1"
SCHEMA = "portable_e2e.full209_JPEG_owned_normalization_counterfactual.20261004.v1"
CAMERAS, JPEG_LIMIT, PAIRS = 918, 2 * 1024**2, 30
ROLES = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
FIXED_SOURCE_SHA = {
    "tools/e2e/benchmark_scandir_episode_quota_v1_20261004.py": "3ed5b103fe30db4fac9ac13a0d4c6c5f7990ed17fc3b747ab5a06f883b81c73c",
    "tools/e2e/audit_owned_image_normalization_parity_v1_20261004.py": "9313b006d6aaff0a9c37cdcfa353e5e5823406ed129f039cfcfd540b1909c98b",
    "learning/tests/test_owned_image_normalization_parity_v1_20261004.py": "3a594c1a08116566c7cb71df2c52a2a9dda415c3b0902faa7403c7ee657c48ff",
    "learning/portable_e2e/runtime.py": "bd1f672efe5cf571744bd4c5ef7655435dc67dddadc41c23fddddb82d721a57b",
    "learning/portable_e2e/torch_dataset.py": "a81d26b1c9aaf67db56cf993a41857133dccac78e42047754e106cff4c8987c9",
    "learning/portable_e2e/model.py": "fbbedef10b97add830e0d887400107b2bcd6dfb5f0c5c4901e399d2e4717fd6e",
    "tools/canonical_workspace.py": "5d5ee618dba8dfdd84cc6da2877fb2dce8b5ba0017e4a503eb3c55d28401d0ed",
    "tools/workspace_python.py": "2221cdf71e4ec840566fb19c180329887e53035ef1f319b02221c547141a5b21",
}
_modules = None


def modules():
    global _modules
    if _modules is None:
        require(os.environ.get("CUDA_VISIBLE_DEVICES") == "", "explicit GPU-disabled environment required before Torch import")
        proof = importlib.import_module("scripts.e2e.audit_owned_image_normalization_parity_v1_20261004")
        _modules = proof, proof.torch, proof.np, proof.runtime
    return _modules


def source_snapshot():
    result = {name: digest(ROOT / name) for name in FIXED_SOURCE_SHA}
    require(result == FIXED_SOURCE_SHA, "sealed original runtime/normalization/helper source differs")
    result.update({str(path.relative_to(ROOT)): digest(path) for path in (HERE, TEST)})
    return result


def environment(*, configure=False):
    """HH_261004 - Use the original existing local CPU installation read-only, with4intra/1inter threads and no visible GPU."""
    expected = dict(CUDA_VISIBLE_DEVICES="", OMP_NUM_THREADS="4", MKL_NUM_THREADS="4", OPENBLAS_NUM_THREADS="1")
    require(all(os.environ.get(name) == value for name, value in expected.items()), "exact CPU benchmark environment required")
    _, torch, _, _ = modules()
    require(sys.executable == "/usr/bin/python3" and sys.version_info[:2] == (3, 10)
            and torch.__version__.startswith("1.8.0a0"), "same original Python3.10/Torch1.8 installation required")
    require(Path(torch.__file__).resolve() == Path("/usr/lib/python3/dist-packages/torch/__init__.py"), "same original Torch source required")
    if configure:
        torch.set_num_threads(4)
        torch.set_num_interop_threads(1)
    require(torch.get_num_threads() == 4 and torch.get_num_interop_threads() == 1, "actual4/1Torch readback required")
    return dict(interpreter=sys.executable, interpreter_resolved=str(Path(sys.executable).resolve()), python=platform.python_version(),
        torch_version=torch.__version__, torch_source=str(Path(torch.__file__).resolve()),
        torch_source_sha256=digest(Path(torch.__file__).resolve()), environment=expected,
        torch_intra_threads=4, torch_interop_threads=1, process_cpu_affinity=sorted(os.sched_getaffinity(0)),
        process_affinity_changed=False, GPU_used=False)


def metadata_sha(document):
    return hashlib.sha256(json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def input_snapshot():
    require(digest(CALLER) == CALLER_SHA, "original full209 manifest SHA differs")
    caller = read_json(CALLER)
    require(caller.get("schema") == CALLER_SCHEMA and caller.get("run_root") == str(RUN), "exact full209 manifest identity required")
    members = caller.get("run_files_sha256")
    require(isinstance(members, dict) and 0 < len(members) <= 15000, "complete bounded original manifest required")
    inventory = tree_members(RUN)
    require(set(inventory["files"]) == set(members), "complete original full209 file membership differs")
    for name, sha in members.items():
        path = Path(name)
        require(not path.is_absolute() and path.parts and all(part not in (".", "..") for part in path.parts)
                and path.as_posix() == name and type(sha) is str and len(sha) == 64, "canonical sealed original member required")
        require(digest(RUN / name) == sha, "full209 original member SHA differs: " + name)
    require(digest(CALLER) == CALLER_SHA and tree_members(RUN) == inventory, "original full209 input changed during verification")
    images = {name: size for name, size in inventory["files"].items() if name.startswith("episode/images/")}
    require(len(images) == 6 * CAMERAS and all(0 < size <= JPEG_LIMIT for size in images.values()), "all918x6 bounded original JPEGs required")
    return dict(original_manifest_sha256=CALLER_SHA, whole_run_file_count=len(members),
        original_whole_run_SHA_verified=True, full_metadata_inventory_sha256=metadata_sha(inventory),
        JPEG_count=len(images), JPEG_logical_bytes=sum(images.values()), all_JPEGs_SHA_verified=True)


def camera_catalog():
    path = safe_path(EPISODE / "camera_frames.jsonl")
    require(path.stat().st_size <= 2 * 1024**2, "bounded camera metadata required")
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    require(len(rows) == CAMERAS and len({row["frame"] for row in rows}) == CAMERAS, "all actual918 camera rows required")
    for index, row in enumerate(rows):
        require(type(row["frame"]) is int and tuple(row["camera_order"]) == ROLES and set(row["images"]) == set(ROLES)
            and set(row["source_timestamps"]) == set(ROLES), "exact six-camera order/membership required")
        require(index == 0 or row["frame"] == rows[index-1]["frame"] + 2, "consecutive original camera source frames required")
        for role in ROLES:
            require(row["images"][role] == f"images/{role}/{row['frame']:08d}.jpg"
                and int(round(row["source_timestamps"][role] * 1e9)) == int(round(row["timestamp"] * 1e9)), "original camera path/stamp differs")
    return rows


def selected_indices(count=CAMERAS):
    require(type(count) is int and count >= PAIRS, "thirty distinct representative ordinals required")
    result = [(count - 1) * index // (PAIRS - 1) for index in range(PAIRS)]
    require(len(set(result)) == PAIRS and result[0] == 0 and result[-1] == count - 1, "fixed complete-interval ordinal selection required")
    return result


def contract():
    return dict(all_camera_bundles=CAMERAS, all_JPEGs=6*CAMERAS, normalization_config=dict(width=320,height=180,mean=[.485,.456,.406],std=[.229,.224,.225]),
        representative_indices=selected_indices(), paired_iterations=PAIRS, timed_calls=2*PAIRS,
        representative_selection="floor((918-1)*i/(30-1)), i=0..29; includes exact first and last independent of outcome",
        pair_order="odd ordinal pairs AB, even BA; each call processes original six JPEGs in model order",
        timed_scope="JPEG bytes already read; original JPEG decode/PIL resize/ownedCHW/div255/normalization and six-image stack",
        candidate_scope="only owned private CPUCHW (tensor-mean)/std becomes tensor.sub_(mean).div_(std); explicit allocation guard retained",
        parity_witness_hashes_and_source_integrity_IO_outside_timing=True,
        original_and_candidate_every_output_byte_equal_required=True, every_source_payload_and_RGB_resize_mean_std_unchanged_required=True,
        no_normalization_cache_or_source_SHA_omission=True, bounded_JPEG_bytes_per_bundle=6*JPEG_LIMIT,
        all918_camera_bundles_including_warmup_and_failed_tail_required=True, all_failed_cases_and_timed_calls_retained=True,
        warm_cache=True, warm_cache_note="Full integrity hashing and whole interval parity precede timing; filesystem caches and decoder kernels may be warm.",
        lossy_JPEG_replay=True, actual_original_live_BGRA_tensor_parity_proven=False,
        actual_sensor_latency_gain_proven=False, model_NN_forward_or_control_test=False,
        actual_runtime_activated=False, model_promoted=False, formal_function_approval=False, installation=False)


def config_document():
    proof, _, _, _ = modules()
    config = proof.ModelConfig()
    config.validate()
    return asdict(config)


def declare():
    require(not PLAN.exists() and not OUTPUT.exists(), "fresh prospective normalization plan/output required")
    sources = source_snapshot()
    inputs = input_snapshot()
    catalog = camera_catalog()
    document = dict(schema=SCHEMA + ".prospective", provenance="HH_261004 - Pin all actual JPEGs and deterministic timing pairs before CPU payload processing.",
        run_root=str(RUN), episode=str(EPISODE), output=str(OUTPUT), original_manifest=str(CALLER),
        source_sha256=sources, input_seal=inputs, camera_catalog_sha256=metadata_sha(catalog),
        config=config_document(), environment=environment(), contract=contract())
    write_new_json(PLAN, document)
    return dict(status="DECLARED_NOT_MEASURED", declaration=str(PLAN), declaration_sha256=digest(PLAN))


def private_work_guard(tensor, array, resized_array, mean, std):
    proof, torch, np, _ = modules()
    require(tensor.device.type == "cpu" and tensor.dtype == torch.float32 and tensor.is_contiguous()
        and tensor._base is None and tensor.storage_offset() == 0 and not tensor.requires_grad
        and proof.storage_info(tensor)["byte_count"] == tensor.numel()*tensor.element_size()
        and not np.shares_memory(tensor.numpy(), array) and not np.shares_memory(tensor.numpy(), resized_array),
        "owned private complete CHW allocation required for candidate")
    require(len({proof.storage_info(value)["pointer"] for value in (tensor,mean,std)}) == 3, "original mean/std must not alias candidate work")


def capture_prefix(frame, array, resized_array, tensor, mean, std):
    """HH_261004 - JPEG payloads are immutable bytes; prove decoded RGB/resize/statistics unchanged without editing the frozen ndarray-only helper."""
    proof, _, _, _ = modules()
    private_work_guard(tensor,array,resized_array,mean,std)
    require(type(frame.payload) is bytes, "actual replay payload must be immutable JPEG bytes")
    return dict(payload=frame.payload, payload_sha256=hashlib.sha256(frame.payload).hexdigest(),
        array=array, array_sha256=proof.array_digest(array), resized_array=resized_array,
        resized_sha256=proof.array_digest(resized_array), tensor=tensor, tensor_before=proof.tensor_bytes(tensor),
        mean=mean, mean_before=proof.tensor_bytes(mean), std=std, std_before=proof.tensor_bytes(std),
        work_pointer=proof.storage_info(tensor)["pointer"])


def compile_pipeline(*, candidate=False, witness=False):
    """HH_261004 - Preserve the exact eight original image AST statements; inverse removes only the disclosed allocation guard/normalization/witness."""
    proof, _, _, runtime = modules()
    original = proof.original_image_statements()
    body = copy.deepcopy(original)
    if candidate:
        body[-1] = ast.parse(proof.NORMALIZATION_CANDIDATE).body[0]
        body.insert(-1,ast.parse("private_work_guard(tensor,array,resized_array,mean,std)").body[0])
    if witness:
        body.insert(-1,ast.parse("witnesses.append(capture_prefix(frame,array,resized_array,tensor,mean,std))").body[0])
    inverse = copy.deepcopy(body)
    inverse = [node for node in inverse if ast.unparse(node) not in (
        "private_work_guard(tensor, array, resized_array, mean, std)",
        "witnesses.append(capture_prefix(frame, array, resized_array, tensor, mean, std))")]
    inverse[-1] = copy.deepcopy(original[-1])
    require(ast.dump(ast.Module(body=inverse,type_ignores=[])) == ast.dump(ast.Module(body=original,type_ignores=[])), "normalization inverse AST changed another image statement")
    setup = ast.parse("image_tensors=[]\nwitnesses=[]\nresampling=getattr(Image,'Resampling',Image).BILINEAR").body
    loop = ast.parse("for camera_name,frame in zip(roles,frames):\n    pass").body[0]
    loop.body = body
    tail = ast.parse("return torch.stack(image_tensors,dim=0).unsqueeze(0),witnesses").body
    function = ast.FunctionDef(name="six_actual_JPEG_normalization_only",args=ast.arguments(posonlyargs=[],
        args=[ast.arg(arg=name) for name in ("frames","config","roles")],kwonlyargs=[],kw_defaults=[],defaults=[]),
        body=setup+[loop]+tail,decorator_list=[])
    namespace = dict(runtime.build_runtime_tensors.__globals__,private_work_guard=private_work_guard,capture_prefix=capture_prefix)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),"SEALED_JPEG_CPU_normalization_counterfactual_only","exec"),namespace)
    return namespace[function.name]


def read_bundle(camera, manifest):
    _, _, _, runtime = modules()
    frames = []
    for role in ROLES:
        path = safe_path(EPISODE / camera["images"][role])
        require(0 < path.stat().st_size <= JPEG_LIMIT, "bounded JPEG payload required")
        payload = path.read_bytes()
        name = str(path.relative_to(RUN))
        require(hashlib.sha256(payload).hexdigest() == manifest[name], "exact replay JPEG source SHA differs")
        frames.append(runtime.RuntimeCameraFrame(int(round(camera["timestamp"]*1e9)),payload,"jpeg"))
    require(sum(len(frame.payload) for frame in frames) <= 6*JPEG_LIMIT,"bounded one-bundle JPEG payload required")
    return tuple(frames)


def bundle_parity(frames, config, functions):
    proof, torch, _, _ = modules()
    reference, witnesses_a = functions["A"](frames,config,ROLES)
    candidate, witnesses_b = functions["B"](frames,config,ROLES)
    require(reference.shape == candidate.shape == (1,6,3,180,320) and reference.dtype == candidate.dtype == torch.float32,
            "original normalized six-camera CPU ABI differs")
    require(proof.tensor_bytes(reference) == proof.tensor_bytes(candidate),"every normalized tensor byte must match")
    receipts = []
    for index,(frame,old,new) in enumerate(zip(frames,witnesses_a,witnesses_b)):
        for witness in (old,new):
            require(hashlib.sha256(frame.payload).hexdigest() == witness["payload_sha256"]
                and proof.array_digest(witness["array"]) == witness["array_sha256"]
                and proof.array_digest(witness["resized_array"]) == witness["resized_sha256"]
                and proof.tensor_bytes(witness["mean"]) == witness["mean_before"]
                and proof.tensor_bytes(witness["std"]) == witness["std_before"],"source JPEG/RGB/resize/mean/std was mutated")
        require(proof.tensor_bytes(old["tensor"]) == old["tensor_before"],"original out-of-place work tensor changed")
        require(proof.storage_info(new["tensor"])["pointer"] == new["work_pointer"],"candidate did not retain its exact owned allocation")
        receipts.append(dict(role=ROLES[index],JPEG_sha256=old["payload_sha256"],
            normalized_tensor_sha256=hashlib.sha256(proof.tensor_bytes(candidate[0,index])).hexdigest(),
            exact_output_bytes_equal=True,original_payload_RGB_resize_mean_std_unchanged=True,private_CHW_allocation_only_mutated=True))
    return dict(passed=True,normalized_shape=list(candidate.shape),all_six_actual_JPEGs=receipts)


def all_parity(catalog, manifest, config):
    functions = {name:compile_pipeline(candidate=name=="B",witness=True) for name in ("A","B")}
    rows = []
    for index,camera in enumerate(catalog):
        row = dict(ordinal=index,native_frame=camera["frame"],timestamp_simulation_seconds=camera["timestamp"],passed=False)
        try:
            row.update(bundle_parity(read_bundle(camera,manifest),config,functions))
        except Exception as error:
            row["error"] = dict(type=type(error).__name__,message=str(error))
        rows.append(row)
    return rows


def measured_pairs(catalog, manifest, config, *, clock=time.perf_counter_ns):
    functions = {name:compile_pipeline(candidate=name=="B") for name in ("A","B")}
    rows = []
    for pair,index in enumerate(selected_indices(len(catalog)),1):
        order = "AB" if pair%2 else "BA"
        try:
            frames,setup_error = read_bundle(catalog[index],manifest),None
        except Exception as error:
            frames,setup_error = None,dict(type=type(error).__name__,message=str(error))
        for position,name in enumerate(order,1):
            row = dict(sequence=len(rows)+1,pair=pair,pair_order=order,position=position,ordinal=index,
                native_frame=catalog[index]["frame"],implementation=name,passed=False)
            if setup_error is not None:
                row.update(error=setup_error,timed_call_attempted=False,elapsed_ns=None)
            else:
                started=clock()
                tensor=None
                try:
                    tensor,_=functions[name](frames,config,ROLES)
                    row["passed"]=True
                except Exception as error:
                    row["error"]=dict(type=type(error).__name__,message=str(error))
                finally:
                    finished=clock()
                    row.update(timed_call_attempted=True,started_monotonic_ns=started,finished_monotonic_ns=finished,elapsed_ns=finished-started)
                    row["passed"] = row["passed"] and finished>started
                # HH_261004 - Output-byte witness is outside the measured image pipeline, identically for both implementations.
                if tensor is not None:
                    row["normalized_tensor_sha256"]=hashlib.sha256(modules()[0].tensor_bytes(tensor)).hexdigest()
            rows.append(row)
    for pair in range(1,PAIRS+1):
        both=[row for row in rows if row["pair"]==pair]
        matched=all(row["passed"] for row in both) and both[0].get("normalized_tensor_sha256")==both[1].get("normalized_tensor_sha256")
        for row in both:row["paired_output_SHA_equal"]=matched
    return rows


def statistics_for(rows):
    result = {}
    for name in ("A","B"):
        selected=[row for row in rows if row["implementation"]==name]
        values=sorted(row["elapsed_ns"]/1e6 for row in selected if row.get("elapsed_ns") is not None)
        result[name]=dict(planned_calls=len(selected),actual_timed_calls=len(values),failures=sum(not row["passed"] for row in selected),
            all_attempted_timings_included=True,milliseconds=None if not values else dict(min=min(values),median=statistics.median(values),p95_nearest_rank=values[math.ceil(.95*len(values))-1],max=max(values)))
    return result


def execute(expected_sha):
    require(digest(PLAN)==expected_sha,"prospective normalization declaration SHA differs")
    plan=read_json(PLAN)
    require(plan.get("schema")==SCHEMA+".prospective" and plan.get("contract")==contract()
        and plan.get("run_root")==str(RUN) and plan.get("episode")==str(EPISODE)
        and plan.get("output")==str(OUTPUT) and not OUTPUT.exists(),"exact fresh normalization scope required")
    sources,inputs,env=source_snapshot(),input_snapshot(),environment()
    catalog=camera_catalog();config=modules()[0].ModelConfig()
    require(sources==plan["source_sha256"] and inputs==plan["input_seal"] and env==plan["environment"]
        and asdict(config)==plan["config"] and metadata_sha(catalog)==plan["camera_catalog_sha256"],"prospective source/input/config/environment seal differs")
    safe_path(OUTPUT,missing_tail=True);OUTPUT.mkdir(parents=True,exist_ok=False)
    manifest=read_json(CALLER)["run_files_sha256"]
    parity=all_parity(catalog,manifest,config)
    timings=measured_pairs(catalog,manifest,config)
    post,errors={},[]
    for name,operation in (("sources",source_snapshot),("inputs",input_snapshot),("environment",environment),("config",config_document)):
        try:post[name]=operation()
        except Exception as error:errors.append(dict(check=name,type=type(error).__name__,message=str(error)))
    unchanged=not errors and post==dict(sources=sources,inputs=inputs,environment=env,config=asdict(config))
    passed=unchanged and len(parity)==CAMERAS and all(row["passed"] for row in parity) and len(timings)==2*PAIRS \
        and all(row["passed"] and row["paired_output_SHA_equal"] for row in timings)
    report=dict(schema=SCHEMA+".actual_JPEG_CPU_only",provenance="HH_261004 - Retain every actual lossy-JPEG source and failed timing; do not promote or regrade full209 V1.",
        status="PASS_JPEG_BYTE_PARITY" if passed else "FAIL_JPEG_COUNTERFACTUAL",declaration_sha256=expected_sha,
        contract=contract(),source_sha256_before=sources,source_sha256_after=post.get("sources"),input_seal_before=inputs,input_seal_after=post.get("inputs"),
        environment_before=env,environment_after=post.get("environment"),config_before=asdict(config),config_after=post.get("config"),
        source_input_config_environment_unchanged=unchanged,post_seal_errors=errors,all918_bundle_parity_rows=parity,
        all60_timing_rows=timings,timing=statistics_for(timings),original_full209_FAIL_unchanged=True,
        actual_BGRA_parity_or_sensor_latency_gain_proven=False,actual_runtime_activated=False,formal_function_approval=False)
    write_new_json(OUTPUT/"report.json",report)
    summary=dict(status=report["status"],parity_bundles=len(parity),parity_passed=sum(row["passed"] for row in parity),
        JPEGs_compared=6*sum(row["passed"] for row in parity),timing=report["timing"],source_input_config_environment_unchanged=unchanged,
        report_sha256=digest(OUTPUT/"report.json"),declaration_sha256=expected_sha,original_full209_FAIL_unchanged=True)
    write_new_json(OUTPUT/"summary.json",summary)
    return summary


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    modes=parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--declare",action="store_true");modes.add_argument("--execute",action="store_true")
    parser.add_argument("--expected-declaration-sha256")
    args=parser.parse_args(argv)
    require(bool(args.execute)==bool(args.expected_declaration_sha256),"execute requires exact prospective SHA")
    environment(configure=True)
    result=execute(args.expected_declaration_sha256) if args.execute else declare()
    print(json.dumps(result,indent=2,sort_keys=True,allow_nan=False))
    return 0 if result["status"] in ("DECLARED_NOT_MEASURED","PASS_JPEG_BYTE_PARITY") else 1


if __name__=="__main__":raise SystemExit(main())
