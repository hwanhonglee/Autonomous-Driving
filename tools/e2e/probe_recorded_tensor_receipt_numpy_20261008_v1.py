"""HH_261008 - Check actual NumPy buffer ABI with synthetic bytes, not model inference or generic tensor support."""
from __future__ import annotations

import argparse
import ast
import fcntl
import hashlib
import json
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[2]
PINS = {
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v14.py": "eb3bb116972b37b01b1b737e1c5dc9007974672ac0befadc91ddfb2f16b97d8a",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v15.py": "73a8177e736504542624a4889d0913d43d307578ad3b024d019e992a1a29eb2c",
    "/usr/lib/python3/dist-packages/numpy/__init__.py": "02e54b39a5825a5eb95c74639b22593a8af816212f9bb1fdcb78f17112a5d27f",
}
# HH_261008 - These positive shapes/dtypes are the actual V14 direct ordinal0 audit headers; values below are synthetic, never archived images.
ABI = {
    "calibration": ((1, 6, 16), "float32"),
    "ego_history": ((1, 10, 13), "float32"),
    "ego_history_mask": ((1, 10), "bool"),
    "images": ((1, 5, 6, 3, 180, 320), "float32"),
    "relative_pose": ((1, 5, 3), "float32"),
    "route_mask": ((1, 128), "bool"),
    "route_xy": ((1, 128, 2), "float32"),
    "time_offsets_s": ((1, 5), "float32"),
    "time_valid": ((1, 5), "bool"),
    "acceleration_prediction": ((1, 1), "float32"),
    "curvature_knots": ((1, 33), "float32"),
    "intent_logits": ((1, 3), "float32"),
    "preferred_speed_knots": ((1, 33), "float32"),
    "stop_distance_raw": ((1, 1), "float32"),
}


def source_functions():
    functions = []
    for name, pin in PINS.items():
        path = Path(name) if name.startswith("/") else ROOT / name
        if hashlib.sha256(path.read_bytes()).hexdigest() != pin:
            raise ValueError("reviewed source/provider changed: " + name)
        if name.startswith("/"):
            continue
        node = next(x for x in ast.parse(path.read_bytes()).body if isinstance(x, ast.FunctionDef) and x.name == "tensor_receipt")
        namespace = {"hashlib": hashlib}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), namespace)
        functions.append(namespace["tensor_receipt"])
    return functions


def high_water_bytes():
    # HH_261008 - Fresh-process VmHWM is observed, not inherited ru_maxrss and not OS containment.
    text = Path("/proc/self/status").read_text()
    return int(next(x.split()[1] for x in text.splitlines() if x.startswith("VmHWM:"))) * 1024


def run():
    import numpy as np
    if np.__version__ != "1.21.5" or np.__file__ != "/usr/lib/python3/dist-packages/numpy/__init__.py":
        raise ValueError("exact already-installed local NumPy provider required")
    old, new = source_functions()
    class Chain:
        # HH_261008 - Actual NumPy arrays, but only a scalar call-chain stand-in: no claim of genuine Torch detach/device operations.
        def __init__(self, array): self.array, self.events = array, []
        def detach(self): self.events.append("detach"); return self
        def cpu(self): self.events.append("cpu"); return self
        def contiguous(self):
            self.events.append("contiguous")
            if not self.array.flags.c_contiguous: self.array = np.ascontiguousarray(self.array)
            return self
        def numpy(self): self.events.append("numpy"); return self.array
    rows = []
    started = time.monotonic()
    for name, (shape, dtype) in ABI.items():
        array = np.zeros(shape, dtype=dtype)
        array.flat[::4093] = True if dtype == "bool" else np.float32(.75)
        left, right = Chain(array), Chain(array)
        before = hashlib.sha256(array.tobytes()).hexdigest()
        a, b = old({name: left}), new({name: right})
        after = hashlib.sha256(array.tobytes()).hexdigest()
        rows.append(dict(name=name, shape=list(shape), dtype=dtype, bytes=array.nbytes,
            old_receipt=a[name], new_receipt=b[name], equal=a == b, bytes_unchanged=before == after,
            chain_equal=left.events == right.events == ["detach", "cpu", "contiguous", "numpy"]))
        del array, left, right
        if high_water_bytes() > 256 * 1024**2 or time.monotonic() - started > 60:
            raise ValueError("sampled synthetic byte probe resource bound exceeded")
    generic = []
    arrays = {
        "noncontiguous_offset": np.arange(40, dtype=np.float32).reshape(5, 8)[1::2, ::2],
        "negative_stride": np.arange(30, dtype=np.float32).reshape(5, 6)[::-1, ::-2],
        "zero_dimensional": np.array(np.float32(.75)),
        "empty_1d": np.empty((0,), dtype=np.float32),
        "empty_2d": np.empty((0, 3), dtype=np.float32),
    }
    for name, array in arrays.items():
        outcomes = []
        for function in (old, new):
            try: outcomes.append(dict(receipt=function({name: Chain(array)})[name], error=None))
            except BaseException as error: outcomes.append(dict(receipt=None, error=repr(error)))
        generic.append(dict(name=name, original_shape=list(array.shape), original_strides=list(array.strides),
            old=outcomes[0], new=outcomes[1], equivalent=outcomes[0] == outcomes[1], declared_recorded_ABI=False))
    success = all(x["equal"] and x["bytes_unchanged"] and x["chain_equal"] for x in rows)
    return dict(provenance="HH_261008 - Actual local NumPy arrays with synthetic contents; not decoded images, Torch/model execution, timing gain or generic all-shape support.",
        schema="portable_e2e.recorded_tensor_receipt_numpy.20261008.v1", status="PASS_DECLARED_POSITIVE_SHAPES_ONLY" if success else "FAIL_RETAINED",
        source_provider_sha256=PINS, numpy_version=np.__version__, declared_cases=14, completed=len(rows), rows=rows,
        generic_probe_non_equivalences_retained=generic, all_generic_shapes_equivalent=all(x["equivalent"] for x in generic),
        actual_Torch_operations_verified=False, model_inference=False, training_steps=0, GPU_used=False, CARLA=False,
        elapsed_seconds=time.monotonic() - started, observed_VmHWM_bytes=high_water_bytes(),
        memory_and_time_sampled_not_OS_containment=True, speed_gain_claimed=False, formal_main_count=0, formal_sub_count=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--run-synthetic-numpy", action="store_true", required=True)
    parser.parse_args()
    lock_path = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
    if lock_path.resolve(strict=True) != lock_path: raise ValueError("canonical runtime lock required")
    with lock_path.open("a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = run()
        print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["status"] == "PASS_DECLARED_POSITIVE_SHAPES_ONLY" else 1


if __name__ == "__main__": raise SystemExit(main())
