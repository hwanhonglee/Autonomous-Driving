"""HH_261005 - Independently read sealed V3 export artifacts and numeric NPZ bytes; never import Torch/NumPy/ONNX or execute neural/native code."""

import ast
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import struct
import zipfile

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-05/good6000_cnn_legacy_onnx_v3"
PLAN = ROOT / "storage/experiments/plans/2026-10-05/good6000_cnn_legacy_onnx_v3/ROOT_plan.json"
AUTHORITY = PLAN.with_name("ROOT_execution_authority.json")
PLAN_SHA = "7b8c44774dd4961a632f8cac74fd783c5179857e03847a359a7068c4d52fc873"
AUTH_SHA = "0f35c45c9aaf30f90dc7deac943e16e6a9e7504e239365b564407289b0ef1050"
SCHEMA = "portable_e2e.good6000_cnn_legacy_onnx_feasibility.20261005.v3"


def need(value, message):
    if not value: raise RuntimeError(message)


def sha(path):
    path = Path(path)
    need(path.is_absolute() and path == path.resolve(strict=True) and not any(p.is_symlink() for p in (path, *path.parents))
        and path.is_file(), "canonical regular file required")
    before = path.stat()
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""): digest.update(block)
    after = path.stat()
    fields = ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")
    need(tuple(getattr(before, field) for field in fields) == tuple(getattr(after, field) for field in fields), "file generation changed")
    return digest.hexdigest()


def read_json(path):
    need(Path(path).stat().st_size <= 4 * 1024**2, "bounded JSON required")
    pin = sha(path)
    raw = Path(path).read_bytes()
    need(hashlib.sha256(raw).hexdigest() == pin, "JSON changed during read")
    def unique(pairs):
        result = {}
        for key, value in pairs:
            need(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    return json.loads(raw, object_pairs_hook=unique, parse_constant=lambda value: need(False, "nonfinite JSON"))


def npz_inventory(path, recorded):
    # HH_261005 - NPY header fields follow installed numpy/lib/format.py primary format specification; decode numeric raw bytes only.
    counts = 0
    with zipfile.ZipFile(path) as archive:
        entries = archive.infolist()
        need(len(entries) == 101 and {row.filename for row in entries} == {name + ".npy" for name in recorded}
            and len({row.filename for row in entries}) == 101 and sum(row.file_size for row in entries) <= 32 * 1024**2,
            "all101 bounded numeric members required")
        for entry in entries:
            need(not entry.is_dir() and not (entry.flag_bits & 1), "numeric NPZ regular unencrypted entry required")
            data = archive.read(entry)
            need(data[:6] == b"\x93NUMPY" and data[6:8] in (b"\x01\x00", b"\x02\x00"), "known numeric NPY version required")
            width = 2 if data[6] == 1 else 4
            length = int.from_bytes(data[8:8 + width], "little")
            need(length <= 4096, "bounded numeric NPY header required")
            description = ast.literal_eval(data[8 + width:8 + width + length].decode("latin1"))
            need(set(description) == {"descr", "fortran_order", "shape"} and description["descr"] == "<f4"
                and description["fortran_order"] is False and type(description["shape"]) is tuple
                and all(type(size) is int and size > 0 for size in description["shape"]), "finite contiguous FP32-only archive ABI required")
            row, raw = recorded[entry.filename[:-4]], data[8 + width + length:]
            count = math.prod(description["shape"])
            need(len(raw) == count * 4 == row["raw_bytes"] and list(description["shape"]) == row["shape"]
                and row["dtype"] == "float32" and hashlib.sha256(raw).hexdigest() == row["raw_sha256"], "original101 raw state manifest differs")
            need(all(math.isfinite(value) for (value,) in struct.iter_unpack("<f", raw)), "nonfinite original state")
            counts += count
    return counts


def audit():
    need(sha(PLAN) == PLAN_SHA and sha(AUTHORITY) == AUTH_SHA, "exact fixed ROOT plan/authority required")
    plan, authority = read_json(PLAN), read_json(AUTHORITY)
    result, report = read_json(OUTPUT / "result.json"), read_json(OUTPUT / "export/feasibility.json")
    need(plan["schema"] == SCHEMA + ".ROOT_plan" and authority == dict(schema=SCHEMA + ".ROOT_execution_authority",
        explicit_cpu_only_export_approved=True, plan_path=str(PLAN), plan_sha256=PLAN_SHA,
        source_sha256=plan["source_sha256"], model_asset_sha256=plan["model_asset_sha256"], runtime_source_sha256=plan["runtime_source_sha256"]),
        "ROOT authority structure mismatch")
    need(result["schema"] == SCHEMA + ".result" and result["status"] == "EXPORTED_FEASIBILITY_ONLY_NOT_RUNTIME_PARITY"
        and result["exit_code"] == 0 and result["elapsed_seconds"] <= 60 and not any(key.endswith("error") for key in result), "producer failure retained")
    need(report["schema"] == SCHEMA and report["status"] == "EXPORTED_FEASIBILITY_ONLY"
        and report["all_original101_state_callbacks_cache_unchanged"] is True and report["prospective_sources_unchanged"] is True,
        "producer state/cache/hook/source failure retained")
    need(len(plan["source_sha256"]) == 34 and len(plan["runtime_source_sha256"]) == 106 and len(plan["model_asset_sha256"]) == 7,
        "complete original declared closure counts differ")
    expected = {}
    for original, archive, pin in [(ROOT / name, OUTPUT / "archive" / name, pin) for name, pin in plan["source_sha256"].items()] + [
        (Path(name), OUTPUT / "archive" / Path(name).relative_to(ROOT), pin) for name, pin in plan["model_asset_sha256"].items()] + [
        (Path(name), OUTPUT / "archive_external" / Path(name).relative_to("/"), pin) for name, pin in plan["runtime_source_sha256"].items()] + [
        (PLAN, OUTPUT / "ROOT_plan.json", PLAN_SHA), (AUTHORITY, OUTPUT / "ROOT_execution_authority.json", AUTH_SHA)]:
        need(sha(original) == pin == sha(archive), "actual/archive sealed source/asset/proof differs")
        expected[str(archive.relative_to(OUTPUT))] = pin
    need(sha(plan["interpreter_path"]) == plan["interpreter_sha256"], "interpreter seal differs")
    full, cnn = report["full101_state"], report["exact14_CNN_state"]
    need(len(full) == 101 and len(cnn) == 14 and cnn == {name.removeprefix("image_encoder."): row
        for name, row in full.items() if name.startswith("image_encoder.")}, "exact14 shared storage/full101 manifest mismatch")
    weights = next(Path(name) for name in plan["model_asset_sha256"] if "step_006000/model_state.npz" in name)
    values = npz_inventory(weights, full)
    rows = report["all_fixed_batches_including_failures"]
    need(rows == result["all_planned_batches"] and [row["contract"]["batch"] for row in rows] == [6, 30], "full fixed first/last denominator differs")
    for batch, row, declared in zip((6, 30), rows, plan["fixed_batches"]):
        value, size = row["normalized_input_unchanged_manifest"], batch * 3 * 180 * 320 * 4
        need(row["contract"] == declared and row["status"] == "EXPORTED_BUILTIN_CHECKED_ONLY" and row["error"] is None
            and row["original101_state_callbacks_cache_unchanged"] is True and value["raw_bytes"] == size
            and value["shape"] == [batch, 3, 180, 320] and value["dtype"] == "float32" and value["version"] == 0
            and value["stride"] == [172800, 57600, 320, 1] and value["raw_sha256"] == hashlib.sha256(bytes(size)).hexdigest(),
            "complete actual zero-input manifest/builtin export row differs")
        path = OUTPUT / "export" / ("cnn_fixed%d.onnx" % batch)
        need(path.stat().st_size == row["bytes"] <= 32 * 1024**2 and sha(path) == row["sha256"], "ONNX bytes/SHA differ")
        expected[str(path.relative_to(OUTPUT))] = row["sha256"]
    for name in ("result.json", "export/feasibility.json", "stdout.log", "stderr.log"):
        expected[name] = sha(OUTPUT / name)
    actual, total = {}, 0
    for parent, directories, files in os.walk(OUTPUT, followlinks=False):
        for name in directories + files:
            path = Path(parent) / name
            mode = path.lstat().st_mode
            need(stat.S_ISDIR(mode) or stat.S_ISREG(mode), "special/symlink output object")
            if stat.S_ISREG(mode): actual[str(path.relative_to(OUTPUT))] = sha(path); total += path.stat().st_size
    need(actual == expected and total == result["closing_budget"]["used_bytes"] + (OUTPUT / "result.json").stat().st_size,
        "complete output census or exact closing bytes differ")
    need(total <= plan["attempt_quota_bytes"] - plan["closing_reserve_bytes"] and (OUTPUT / "stderr.log").stat().st_size <= plan["log_limit_bytes"],
        "complete output budget/log cap exceeded")
    core = result["child_creation_core"]
    argv = result["actual_child_argv"]
    need(argv[:len(plan["owner_prefix"])] == plan["owner_prefix"] and argv[argv.index("--plan-sha256") + 1] == PLAN_SHA
        and argv[argv.index("--authority-sha256") + 1] == AUTH_SHA and core["pgid"] == core["sid"] == core["pid"], "owned child CLI/core differs")
    records = 0
    for path in Path("/proc").iterdir():
        if path.name.isdigit():
            try: fields = (path / "stat").read_text().rsplit(")", 1)[1].split()
            except (FileNotFoundError, ProcessLookupError): continue
            records += 1
            need(int(fields[2]) != core["pgid"], "owned export process-group survivor")
    need(not Path("/proc/" + str(core["pid"])).exists(), "original child PID still exists; inspect generation")
    free = os.statvfs(ROOT).f_bavail * os.statvfs(ROOT).f_frsize
    need(free >= plan["free_floor_bytes"] + plan["shared_reserve_bytes"] + plan["closing_reserve_bytes"], "current unchanged reserve required")
    return dict(explanation="HH_261005 - Readonly independent stdlib postcheck; no Torch/NumPy/ONNX import, reexport, native parser, GPU or model execution.",
        status="PASS_EXPORTED_FEASIBILITY_ONLY_NOT_NUMERICAL_RUNTIME_PARITY", plan_sha256=PLAN_SHA, authority_sha256=AUTH_SHA,
        source_pairs_verified=34, runtime_Python_source_pairs_verified=106, model_asset_pairs_verified=7,
        full101_NPZ_shapes_dtypes_finite_values_raw_SHA_verified=True, finite_FP32_values=values, exact14_shared_producer_storage_manifest_verified=True,
        producer_all101_state_callbacks_cache_structure_unchanged_guards=True, complete_batch_denominator=[6, 30],
        whole_actual_input_bytes=[4147200, 20736000], ONNX_sha256={"6": rows[0]["sha256"], "30": rows[1]["sha256"]},
        producer_builtin_checker_rows_complete=True, independent_ONNX_initializer_ABI_or_engine_parity_verified=False,
        complete_output_file_count=len(actual), complete_output_bytes=total, current_bavail_bytes=free,
        declared_original_child_PID_generation_and_group_gone=True, process_metadata_records_checked=records,
        complete_output_file_sha256=actual, Torch_or_export_or_GPU_executed=False,
        runtime_source_binary_complete_claimed=False, driving_gain_or_feature_approval_claimed=False)


if __name__ == "__main__":
    try: print(json.dumps(audit(), indent=2, allow_nan=False))
    except BaseException as error:
        print(json.dumps(dict(status="FAIL_OR_UNVERIFIED_RETAINED", error=type(error).__name__ + ": " + str(error))))
        raise SystemExit(2)
