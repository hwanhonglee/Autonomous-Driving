"""HH_261005 - SOURCE ONLY persistent CNN transport/cache candidate, never automatic activation.

All original causal/window/pixel/state/finite/cache/recovery-hook math remains in
the inherited cached API. Only its cold30 and warm missing-image CNN sites use
the explicit backend. Warm12/18/24/30 are ordered6-image native calls; original
cached cnn_calls remains the original one encoder invocation, not native calls.
The new encoder is not forged as the old trusted type: masked-copy correctly
falls back to original allocation. Engine build is not numerical/runtime parity.
"""

from __future__ import annotations

import ast
import copy
import hashlib
import math
import os
from pathlib import Path
import select
import struct
import time

SCHEMA = "portable_e2e.cached_tensorrt_backend.20261005.v1.source_only"
ROOT = Path(__file__).resolve().parents[2]
ROOT_PLAN = ROOT / "storage/experiments/plans/2026-10-05/good6000_cnn_tensorrt_runtime_v1/ROOT_plan.json"
ROOT_AUTHORITY = ROOT_PLAN.with_name("ROOT_execution_authority.json")
MAGIC, VERSION = 0x4848434E, 1
REQUEST = struct.Struct("<IIQII32s")
RESPONSE = struct.Struct("<IIQIII32s32s")
REQUEST_SECONDS = .100
GOOD6000_WEIGHTS_SHA256 = "b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625"
GOOD6000_METADATA_SHA256 = "a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04"
PARENT_SOURCE_PINS = {
    "temporal_model_v2.py": "93f709c6c229e705899fcf1778f9dd253ea867e88a6f237ad3f0fb8efdbbf435",
    "temporal_cached_encoder_v2.py": "55ef7371e0a1f4ec2b9cb854282e4d51b6c406b055ec5eceb8e286737f03adff",
    "temporal_route_recovery_v3_20261001.py": "41494aeb6175c7e8bb054b0c099b121e1897144681a6e3591919bc0740925a25",
    "temporal_all_valid_masked_copy_research_v1_20261004.py": "3cffa23b9a63965739d38cd47f980a799a6748cbf374e61ab35b3ceb91d66d5b",
}


def need(condition, message):
    if not condition: raise RuntimeError(message)


def _require_actual_admission():
    # HH_261005 - No bool/caller dict can enable V1; a separately reviewed fresh runtime owner/revision and exact ROOT plan+authority are required.
    raise RuntimeError("SOURCE_ONLY_V1: actual Torch/native/GPU/model activation unconditionally denied")


def require_parent_sources():
    for name, pin in PARENT_SOURCE_PINS.items():
        path = Path(__file__).with_name(name)
        need(path.is_file() and not path.is_symlink() and hashlib.sha256(path.read_bytes()).hexdigest() == pin,
            "original frozen source changed: " + name)


def native_batches(image_count, *, cold):
    need(type(image_count) is int and image_count in (6, 12, 18, 24, 30) and type(cold) is bool,
        "exact original missing-image batch count required")
    if cold:
        need(image_count == 30, "cold cache always encodes all30 masked images")
        return (30,)
    return (6,) * (image_count // 6)


def request_header(batch, sequence, input_sha256):
    need(type(batch) is int and batch in (6, 30) and type(sequence) is int and 0 < sequence < 2**64,
        "fixed6/30 sequential request required")
    need(type(input_sha256) is bytes and len(input_sha256) == 32, "exact normalized input raw SHA bytes required")
    return REQUEST.pack(MAGIC, VERSION, sequence, batch, batch*3*180*320*4, input_sha256)


def finite_float32(raw):
    need(type(raw) is bytes and len(raw) % 4 == 0, "whole little-endian FP32 bytes required")
    need(all(math.isfinite(value[0]) for value in struct.iter_unpack("<f", raw)), "nonfinite native feature bytes denied")


def decode_response(header, payload, *, sequence, batch, input_sha256):
    need(type(header) is bytes and len(header) == RESPONSE.size, "whole bounded native response header required")
    magic, version, ordinal, count, length, status, source, output = RESPONSE.unpack(header)
    need(magic == MAGIC and version == VERSION and ordinal == sequence and count == batch
        and source == input_sha256 and type(payload) is bytes and len(payload) == length,
        "native output sequence/batch/input-source/length differs")
    need(hashlib.sha256(payload).digest() == output, "native output raw byte SHA differs")
    if status == 2:
        need(length <= 1024, "bounded failed native request text required")
        raise RuntimeError("native failed request retained: " + payload.decode("utf8"))
    need(status == 1 and batch in (6,30) and length == batch*60*128*4, "fixed FP32 features/status required")
    finite_float32(payload)
    return payload


def verify_native_identity(observed, expected_core, expected_argv):
    need(observed is not None and observed.get("state") != "Z"
        and all(observed.get(key) == expected_core[key] for key in ("pid","ppid","pgid","sid","start_ticks"))
        and observed.get("argv") == expected_argv and expected_core["pid"] == expected_core["pgid"] == expected_core["sid"],
        "exact current owned native generation/ancestry/argv/session required")


def _bounded_pipe(fd, data, *, writing, deadline):
    # HH_261005 - Poll bounded nonblocking byte FDs, never pickle or connect a network endpoint; source/engine/lease/ownership admission precedes actual use.
    _require_actual_admission()
    done = 0; target = data if writing else bytearray(data)
    while done < len(target):
        remaining = deadline-time.monotonic(); need(remaining > 0, "fixed native request deadline exceeded")
        readers, writers, errors = select.select([] if writing else [fd], [fd] if writing else [], [fd], min(.05,remaining))
        need(not errors, "native protocol FD error")
        if not (writers if writing else readers): continue
        try:
            if writing: amount = os.write(fd,memoryview(target)[done:])
            else:
                chunk = os.read(fd,min(1024**2,len(target)-done)); amount = len(chunk); target[done:done+amount] = chunk
        except BlockingIOError: continue
        need(amount > 0, "native protocol EOF/write failure")
        done += amount
    return None if writing else bytes(target)


class PreparedTensorRTClientV1:
    """HH_261005 - Unconstructible actual V1 client; future admitted revision must pin successful engines, binary/assets, owned lifecycle and all observation journals."""

    def __init__(self, *args, **kwargs):
        _require_actual_admission()

    def _request(self, values, batch):
        _require_actual_admission()
        self._guard_owner_lease_sources_engines()
        begin = time.monotonic(); deadline = begin + REQUEST_SECONDS
        raw = bytes(memoryview(values.detach().contiguous().numpy()).cast("B"))
        need(len(raw) == batch*3*180*320*4, "exact full normalized request bytes required")
        source = hashlib.sha256(raw).digest(); self.sequence += 1
        request = request_header(batch,self.sequence,source)
        record = dict(sequence=self.sequence,batch=batch,input_sha256=source.hex(),status="FAIL_NOT_VERIFIED")
        self.requests.append(record)  # Retain a failed/cold request before any transport, never reuse an ordinal.
        try:
            _bounded_pipe(self.input_fd,request,writing=True,deadline=deadline)
            _bounded_pipe(self.input_fd,raw,writing=True,deadline=deadline)
            header = _bounded_pipe(self.output_fd,RESPONSE.size,writing=False,deadline=deadline)
            unpacked = RESPONSE.unpack(header); length,status = unpacked[4:6]
            need(length <= (batch*60*128*4 if status==1 else 1024), "bounded native response before allocation required")
            payload = _bounded_pipe(self.output_fd,length,writing=False,deadline=deadline)
            result = decode_response(header,payload,sequence=self.sequence,batch=batch,input_sha256=source)
            need(hashlib.sha256(bytes(memoryview(values.detach().contiguous().numpy()).cast("B"))).digest() == source,
                "original normalized input changed during native request")
            self._guard_owner_lease_sources_engines()
            need(time.monotonic() <= deadline, "fixed native request deadline exceeded after guards")
            record.update(status="FINITE_SOURCE_BOUND_FP32_FEATURE_BYTES_NOT_MODEL_PARITY",output_sha256=hashlib.sha256(result).hexdigest())
            return result
        except BaseException as error:
            record["error"] = type(error).__name__ + ": " + str(error); raise
        finally: record["seconds"] = time.monotonic()-begin

    def encode(self, images, *, cold):
        _require_actual_admission()
        import numpy as np
        import torch
        need(type(images) is torch.Tensor and images.device.type == "cpu" and images.dtype == torch.float32
            and images.is_contiguous() and not images.requires_grad and not torch.is_grad_enabled()
            and tuple(images.shape[1:]) == (3,180,320), "original readonly CPU float32 CNN ABI required")
        batches = native_batches(images.shape[0],cold=cold)
        results,offset = [],0
        for batch in batches:
            raw = self._request(images[offset:offset+batch],batch)
            results.append(torch.from_numpy(np.frombuffer(raw,dtype="<f4").copy()).reshape(batch,60,128)); offset += batch
        result = results[0] if len(results)==1 else torch.cat(results,dim=0)
        # HH_261005 - Native LINEAR bytes are row-major; restore the original projection.flatten(2).transpose(1,2) strides before cache clones/conditioning.
        result = result.transpose(1,2).contiguous().transpose(1,2)
        need(bool(torch.isfinite(result).all().item()), "all native features finite before original conditioning/cache")
        return result


def create_backend_model(original, client):
    # HH_261005 - No original class/global/instance method is patched; this gated physical subclass keeps all101 state names and complete original cache body.
    _require_actual_admission()
    require_parent_sources()
    import torch
    from .contract import ContractError
    from .temporal_cached_encoder_v2 import _CachedSpatialImageEncoderV2, _Entry, _tensor_sha256
    from .temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
    from .temporal_all_valid_masked_copy_research_v1_20261004 import AllValidMaskedCopyRouteRecoveryTemporalModelV3
    from scripts.e2e.prepare_good6000_cnn_legacy_onnx_v3_20261005 import callbacks_and_cache, cnn_structure
    need(type(original) is RouteRecoveryTemporalModelV3 and type(client) is PreparedTensorRTClientV1,
        "genuine strict original101 loader plus exact declared native client required")
    need(getattr(original,"_research_temporal_export_npz_sha256",None)==GOOD6000_WEIGHTS_SHA256
        and getattr(original,"_research_temporal_metadata_sha256",None)==GOOD6000_METADATA_SHA256,
        "same original GOOD6000 strict source/export binding required")
    need(all(value.dtype==torch.float32 and value.device.type=="cpu" and not value.requires_grad
        for value in original.state_dict().values()),"all101 original readonly CPU FP32 tensors required")
    # HH_261005 - Future admitted client must receive this exact full101 manifest from ROOT-bound successful original producer/engine proofs, never self-declare it.
    actual101 = {name:dict(shape=list(value.shape),dtype="float32",
        raw_sha256=hashlib.sha256(memoryview(value.detach().contiguous().numpy()).cast("B")).hexdigest())
        for name,value in original.state_dict().items()}
    need(len(actual101)==101 and actual101==client.ROOT_bound_original101_manifest,
        "CPU full101 bytes/shapes/dtypes differ from the exact engine producer")
    need(not torch.is_grad_enabled() and not torch.nn.modules.module._global_forward_hooks
        and not torch.nn.modules.module._global_forward_pre_hooks and not torch.nn.modules.module._global_backward_hooks,
        "original readonly/no-global-callback inference contract required")
    original_callbacks = callbacks_and_cache(original)
    original_structure = cnn_structure(original.image_encoder,torch.nn)
    class TensorRTCachedSpatialEncoderV1(_CachedSpatialImageEncoderV2):
        def __init__(self, parent, backend):
            need(not parent.entries and parent.request is None,"idle original cache required at declared attachment")
            super().__init__(parent); self.backend = backend

        def forward(self, images):
            if self.request is None:
                return super().forward(images)
            frames = self.request
            cold = not self.entries
            hits = {}
            missing = []
            pixels = {}
            for slot, frame in enumerate(frames):
                if frame is None:
                    continue
                pixels[slot] = _tensor_sha256(images[slot * 6:(slot + 1) * 6])
                entry = self.entries.get(frame)
                if entry is not None and entry.pixel_sha256 == pixels[slot]:
                    hits[slot] = entry.features
                else:
                    missing.append(slot)
            if cold:
                encoded = self.backend.encode(images, cold=True)
                for slot in missing:
                    hits[slot] = encoded[slot * 6:(slot + 1) * 6].detach().clone()
                cnn_images = 30
            else:
                encoded = images.new_zeros((30, 60, 128))
                cnn_images = len(missing) * 6
                if missing:
                    new_images = torch.cat([images[slot * 6:(slot + 1) * 6] for slot in missing])
                    new_features = self.backend.encode(new_images, cold=False)
                    for index, slot in enumerate(missing):
                        hits[slot] = new_features[index * 6:(index + 1) * 6].detach().clone()
                for slot, features in hits.items():
                    encoded[slot * 6:(slot + 1) * 6] = features
            for slot in missing:
                frame = frames[slot]
                for previous in tuple(self.entries):
                    if previous.timestamp_ns == frame.timestamp_ns:
                        del self.entries[previous]
                self.entries[frame] = _Entry(pixels[slot], hits[slot])
            self.work = (cnn_images, len(hits) - len(missing), len(missing), cold)
            return encoded

    def behavior_identity(model):
        need(not torch.nn.modules.module._global_forward_hooks and not torch.nn.modules.module._global_forward_pre_hooks
            and not torch.nn.modules.module._global_backward_hooks,"global callbacks would change the original CNN graph")
        modules = tuple(model.named_modules())
        need(not any(key in vars(module) for name,module in modules for key in ("forward","__call__","_call_impl","_conv_forward"))
            and not any(key in vars(client) for key in ("encode","_request")),"after-factory method replacement denied")
        return (id(torch.nn.Module.__call__),id(torch.nn.Module._call_impl),id(torch.nn.Conv2d._conv_forward),
            id(type(client).encode),id(type(client)._request),tuple((name,id(type(module).forward)) for name,module in modules))

    class TensorRTCachedRouteRecoveryV1(AllValidMaskedCopyRouteRecoveryTemporalModelV3):
        def _state_identity(self):
            state = super()._state_identity()
            seal = getattr(self,"_native_state_seal",None)
            if seal is not None and state != seal:
                raise ContractError("full101/topology/callback state changed; prebuilt CNN engine is no longer bound")
            behavior = getattr(self,"_native_behavior_seal",None)
            if behavior is not None and behavior_identity(self) != behavior:
                raise ContractError("after-factory module/transport behavior changed")
            return state

    before = original._state_identity()
    candidate = TensorRTCachedRouteRecoveryV1(original.config,original.recovery_config)
    candidate.load_state_dict({name:value.detach().clone() for name,value in original.state_dict().items()},strict=True)
    candidate.requires_grad_(False).eval()
    state = {name:_tensor_sha256(value) for name,value in candidate.state_dict().items()}
    need(len(state)==101 and state=={name:_tensor_sha256(value) for name,value in original.state_dict().items()},"strict all101 learned tensor bytes equal")
    encoder = candidate.image_encoder
    pointers = {name:value.data_ptr() for name,value in candidate.state_dict().items()}
    candidate.image_encoder = TensorRTCachedSpatialEncoderV1(encoder,client)
    need(state=={name:_tensor_sha256(value) for name,value in candidate.state_dict().items()}
        and pointers=={name:value.data_ptr() for name,value in candidate.state_dict().items()},"seam changed original101 bytes/storage/names")
    for name in ("_research_temporal_checkpoint_sha256","_research_temporal_export_npz_sha256","_research_temporal_metadata_sha256"):
        setattr(candidate,name,getattr(original,name))
    candidate._native_state_seal = candidate._state_identity()
    candidate._native_behavior_seal = behavior_identity(candidate)
    callbacks_and_cache(candidate)
    need(original_structure==cnn_structure(candidate.image_encoder,torch.nn)
        and before==original._state_identity() and original_callbacks==callbacks_and_cache(original),
        "original strict101 model/structure/callbacks/cache changed")
    return candidate


def cache_inverse_ast_proof():
    """HH_261005 - Source-only proof; remove annotations then invert exactly two CNN expression substitutions, with every original cache statement retained."""
    require_parent_sources()
    original = ast.parse(Path(__file__).with_name("temporal_cached_encoder_v2.py").read_text())
    parent = next(node for node in original.body if isinstance(node,ast.ClassDef) and node.name=="_CachedSpatialImageEncoderV2")
    old = copy.deepcopy(next(node for node in parent.body if isinstance(node,ast.FunctionDef) and node.name=="forward"))
    fresh = ast.parse(Path(__file__).read_text())
    child = next(node for node in ast.walk(fresh) if isinstance(node,ast.ClassDef) and node.name=="TensorRTCachedSpatialEncoderV1")
    new = copy.deepcopy(next(node for node in child.body if isinstance(node,ast.FunctionDef) and node.name=="forward"))
    old.returns = new.returns = None
    for method in (old,new):
        for argument in method.args.args: argument.annotation=None
        for node in ast.walk(method):
            if isinstance(node,ast.AnnAssign):
                replacement=ast.Assign(targets=[node.target],value=node.value)
                for parent_node in ast.walk(method):
                    for field,value in ast.iter_fields(parent_node):
                        if isinstance(value,list) and node in value:value[value.index(node)]=replacement
    matches=0
    for node in ast.walk(new):
        if (isinstance(node,ast.Assign) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute)
                and ast.unparse(node.value.func)=="self.backend.encode"):
            arg=node.value.args[0];node.value=ast.Call(func=ast.Attribute(value=ast.Call(func=ast.Name(id="super",ctx=ast.Load()),args=[],keywords=[]),attr="forward",ctx=ast.Load()),args=[arg],keywords=[]);matches+=1
    need(matches==2 and ast.dump(old,include_attributes=False)==ast.dump(new,include_attributes=False),"complete cached forward inverse differs")
    return dict(two_CNN_sites_only=True,original_cache_inverse_equal=True,ordinary_request_None_uses_original_CPU=True,
        masked_copy_trust_forged=False,whole888_strict5head_numerical_parity_verified=False)
