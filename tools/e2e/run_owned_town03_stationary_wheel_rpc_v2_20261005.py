"""HH_261005 - Reuse a verified full private native input; retain V1 failure, ownership, quota and stationary-only limits."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import shutil
import signal
import subprocess
import sys
import time
import stat
import zipfile

from . import run_owned_ctrack_isolated_probe_20261001 as guards

ROOT = Path(__file__).resolve().parents[2]
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
EGG = CARLA_ROOT / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
EGG_SHA256 = "6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a"
BINARY = CARLA_ROOT / "CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
OUTPUT = ROOT / "storage/experiments/validation/2026-10-05/owned_town03_stationary_wheel_rpc_v2"
PLAN = OUTPUT.with_name(OUTPUT.name + "_plan.json")
LAUNCH_AUTHORITY = ROOT / "storage/experiments/plans/2026-10-05/owned_town03_stationary_wheel_rpc_v2/ROOT_launch_authority.json"
SCHEMA = "portable_e2e.owned_town03_stationary_wheel_rpc.20261005.v2"
MODULE = "scripts.e2e.run_owned_town03_stationary_wheel_rpc_v2_20261005"
CAPTURE_MODULE = "scripts.e2e.collect_owned_town03_stationary_wheel_rpc_v2_20261005"
PLAN_TOKEN, AUTH_TOKEN = "EXACT_DECLARATION_SHA256_AT_EXECUTION", "EXACT_ROOT_LAUNCH_AUTHORITY_SHA256_AT_EXECUTION"
FLOOR, ATTEMPT, SHARED, CLOSING = 5 * 1024**3, 768 * 1024**2, 64 * 1024**2, 16 * 1024**2
PORT = 2440
COLLECTOR = ROOT / "tools/e2e/collect_carla_vad_expert.py"
COLLECTOR_SHA256 = "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08"
NATIVE_SIZE = 153958688
NATIVE_SHA256 = "d60de268813db6e1c94b8f613888ed5f9eaed39cb02ac2debb4e1a05aedb8966"
NATIVE_MEMBER = "carla/libcarla.cpython-310-x86_64-linux-gnu.so"
PROBE_SOURCE_PINS = {
    "tools/e2e/probe_carla_import_only_v1_20261005.py": "4503ece4195977374dbe2fa80435dcf69418c9706cc28f8efe9ee14cc3e765a9",
    "learning/tests/test_carla_import_only_probe_v1_20261005.py": "56168d49a3f15aa97c96003164950a8f165f14dba0d899be9c0f092a568988f9",
}
FROZEN_V1_PINS = {
    "tools/e2e/run_owned_town03_stationary_wheel_rpc_v1_20261005.py": "a5c28afc91e387cf5f97cc8f717c8a7ad8dd45a541e73c1610be3d4484200d1c",
    "tools/e2e/collect_owned_town03_stationary_wheel_rpc_v1_20261005.py": "ab66487694c4df4fb485332e08cd51966aa8af2a1f4fb7da666b3724f80f1725",
    "learning/tests/test_owned_town03_stationary_wheel_rpc_owner_v1_20261005.py": "d9062573220fade80a527f13e8a45f1c0dc1355e46df26f036c4e38051b363c9",
    "learning/tests/test_owned_town03_stationary_wheel_rpc_capture_v1_20261005.py": "51b02313e9d10f7d4ceb55621e3070eb2513bcfc8f045e5a7ca07e7765de170a",
}
ASSET_SHA256 = {
    str(EGG): EGG_SHA256,
    str(BINARY): "34a97404e1dbf61d2ebfb6dae18970ec5684f65c08c934cc7f65a5bd2b7f5192",
    str(CARLA_ROOT / "CarlaUE4/Content/Carla/Maps/Town03.umap"): "3409354bb605409012f4153fa7ee1191a4205ad71303eb5873e6ef7a9cea78e4",
    str(CARLA_ROOT / "CarlaUE4/Content/Carla/Maps/OpenDrive/Town03.xodr"): "abf177bcd9b66cfefb7ac4919f6456815f1384f4229f1343c8a6a90bbb7713c9",
    str(CARLA_ROOT / "CarlaUE4/Config/DefaultEngine.ini"): "c948c7b8ada9fafa38485a8523fb331a9b76af29fc6675fe77fe2e26c560206c",
}


def require(value, message):
    if not value:
        raise RuntimeError(message)


def canonical(path, *, exists=True):
    path = Path(path).absolute()
    require(".." not in path.parts and path == path.resolve(strict=exists)
            and all(not part.is_symlink() for part in (path, *path.parents)), "canonical non-symlink path required")
    return path


def digest(path):
    path = canonical(path)
    require(path.is_file(), "regular sealed file required")
    before = path.stat()
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            result.update(block)
    after = path.stat()
    require((before.st_dev, before.st_ino, before.st_size, before.st_ctime_ns, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_ctime_ns, after.st_mtime_ns), "sealed file generation changed")
    return result.hexdigest()


def read_json(path, *, limit=4 * 1024**2):
    path = canonical(path)
    require(path.is_file() and path.stat().st_size <= limit, "bounded regular JSON required")
    expected = digest(path)
    raw = path.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == expected, "JSON changed while read")
    def unique(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    return json.loads(raw, object_pairs_hook=unique,
                      parse_constant=lambda value: require(False, "nonfinite JSON forbidden"))


def write_new(path, value):
    path = canonical(path, exists=False)
    require(path.parent.is_dir(), "existing canonical output parent required")
    raw = (json.dumps(value, indent=2, allow_nan=False) + "\n").encode()
    require(len(raw) <= 4 * 1024**2, "bounded scalar receipt required")
    with path.open("xb") as stream:
        stream.write(raw)


def source_pins():
    from . import owned_parallel_wheel_reads_v2_20261005 as wheel
    result = wheel.sources()
    for name, sha in {**FROZEN_V1_PINS, **PROBE_SOURCE_PINS}.items():
        require(digest(ROOT / name) == sha, "frozen V1/import-only source changed")
        result[name] = sha
    for name in ("tools/e2e/run_owned_town03_stationary_wheel_rpc_v2_20261005.py",
                 "tools/e2e/collect_owned_town03_stationary_wheel_rpc_v2_20261005.py",
                 "learning/tests/test_owned_town03_stationary_wheel_rpc_owner_v2_20261005.py",
                 "learning/tests/test_owned_town03_stationary_wheel_rpc_capture_v2_20261005.py",
                 "tools/e2e/collect_carla_vad_expert.py"):
        result[name] = digest(ROOT / name)
    require(result[str(COLLECTOR.relative_to(ROOT))] == COLLECTOR_SHA256, "original measurement source changed")
    return result


def engine_ini():
    package = "/Game/Carla/Maps/Town03.Town03"
    return "[/Script/EngineSettings.GameMapsSettings]\n" + "".join(
        key + "=" + package + "\n" for key in ("EditorStartupMap", "GameDefaultMap", "ServerDefaultMap", "TransitionMap"))


def fixed_contract(gpu_uuid):
    require(type(gpu_uuid) is str and re.fullmatch(r"GPU-[0-9a-fA-F-]{36}", gpu_uuid), "exact prospective local GPU UUID required")
    # HH_261005 - A fresh direct shipping child avoids rewriting shared Engine.ini or invoking a chmod wrapper.
    server = [str(BINARY), "CarlaUE4", "-prefernvidia", "-graphicsadapter=0", "-quality-level=Epic",
              "-carla-port=2440", "-EngineINI=" + str(OUTPUT / "Town03.Engine.ini"),
              "-UserDir=" + str(OUTPUT / "UserData"), "-abslog=" + str(OUTPUT / "server-engine.log"),
              "-RenderOffScreen", "-nosound"]
    prefix = ["/usr/bin/python3", "-B", str(ROOT / "tools/workspace_python.py"), "-m"]
    tail = ["--plan-sha256", PLAN_TOKEN, "--launch-authority-sha256", AUTH_TOKEN]
    return dict(schema=SCHEMA, output=str(OUTPUT), declaration_path=str(PLAN), map="Town03",
                port_triplet=[2440, 2441, 2442], gpu_index=0, gpu_uuid=gpu_uuid,
                gpu_memory_limit_MiB=512, gpu_utilization_limit_percent=30,
                server_command=server, owner_command=prefix + [MODULE, "--", "--mode", "execute"] + tail,
                worker_command=prefix + [CAPTURE_MODULE, "--"] + tail, shared_runtime_lease=str(guards.LOCK),
                free_disk_floor_bytes=FLOOR, attempt_quota_bytes=ATTEMPT, closing_reserve_bytes=CLOSING,
                shared_safety_reserve_bytes=SHARED, startup_timeout_s=120, authority_wait_s=120,
                owner_wall_timeout_s=240, individual_log_file_limit_bytes=16 * 1024**2,
                measured_frames=30, bootstrap_native_ticks=1, physics_hz=20, fixed_delta_seconds=.05, variant_calls=60,
                wheel_rpc_calls=120, order="AB_even_BA_odd", first_cold_calls_retained=True,
                vehicle_blueprint="vehicle.toyota.prius", spawn_index=0, spawn_z_offset_m=.5,
                actorphysics_disabled=True, stationary_position_tolerance_m=0., stationary_speed_limit_mps=0.,
                world_reload=False, images=False, NN=False, weights=False, train=False, test=False,
                teacher_drive=False, retry=False, original_measurement_math_unchanged=True,
                moving_physics_equivalence_claimed=False, learned_driving_success_claimed=False,
                formal_main_domains_approved=0, formal_subfunctions_approved=0,
                launch_authority_path=str(LAUNCH_AUTHORITY), source_sha256=source_pins(),
                asset_sha256=dict(ASSET_SHA256), generated_engine_ini_sha256=hashlib.sha256(engine_ini().encode()).hexdigest(),
                working_engine_ini_is_mutable=True, pristine_engine_ini_retained=True,
                native_runtime_input=native_runtime_input_contract(), native_runtime_input_copied=False,
                original_16MiB_child_file_limit_unchanged=True)


def verify_plan(expected):
    require(type(expected) is str and re.fullmatch(r"[0-9a-f]{64}", expected), "exact declaration SHA required")
    plan = read_json(PLAN)
    require(digest(PLAN) == expected and json.dumps(plan, sort_keys=True, allow_nan=False)
            == json.dumps(fixed_contract(plan.get("gpu_uuid")), sort_keys=True, allow_nan=False), "prospective plan/source/contract differs")
    require(all(digest(path) == sha for path, sha in ASSET_SHA256.items()), "original CARLA asset seal differs")
    return plan


def _probe():
    require(all(digest(ROOT / name) == sha for name, sha in PROBE_SOURCE_PINS.items()), "reviewed import-only source changed")
    from . import probe_carla_import_only_v1_20261005 as probe
    return probe


def native_runtime_input_contract():
    # HH_261005 - A successful sealed import-only result must exist; this function cannot manufacture readiness or copy another native file.
    probe = _probe()
    result_path = probe.OUTPUT / "result.json"
    producer_plan, result = read_json(probe.PLAN), read_json(result_path)
    plan_sha, authority_sha = digest(probe.PLAN), digest(probe.AUTHORITY)
    require(json.dumps(producer_plan, sort_keys=True) == json.dumps(probe.contract(), sort_keys=True),
            "exact reviewed import-only producer declaration required")
    authority = read_json(probe.AUTHORITY)
    require(authority == dict(schema=probe.SCHEMA + ".ROOT_execution_authority",
            explicit_import_only_execution_approved=True, plan_path=str(probe.PLAN), plan_sha256=plan_sha,
            source_sha256=producer_plan["source_sha256"], asset_sha256=producer_plan["asset_sha256"]),
            "genuine producer ROOT authority required")
    require(result.get("schema") == probe.SCHEMA + ".result"
            and result.get("status") == "IMPORT_ONLY_COMPLETE_NOT_DRIVING"
            and type(result.get("exit_code")) is int and result["exit_code"] == 0
            and result.get("native_endpoint_calls_verified_absent") is True
            and result.get("trace_audit", {}).get("actual_trace_complete") is True
            and result.get("trace_audit", {}).get("native_endpoint_calls_observed") is False
            and not any(result.get(key) for key in ("error", "cleanup_error", "final_seal_error")),
            "successful complete import-only result required before stationary declaration")
    require(argv_value(result.get("actual_child_argv", []), "--plan-sha256") == plan_sha
            and argv_value(result.get("actual_child_argv", []), "--authority-sha256") == authority_sha,
            "actual producer child must bind its exact declaration and authority")
    native = canonical(probe.PRIVATE_NATIVE)
    row = native.stat()
    with zipfile.ZipFile(EGG) as archive:
        info = archive.getinfo(NATIVE_MEMBER)
        timestamp = time.mktime(info.date_time + (0, 0, -1))
    require(stat.S_ISREG(row.st_mode) and row.st_nlink == 1 and row.st_size == info.file_size == NATIVE_SIZE
            and row.st_mtime == timestamp, "full private native size/ZIP mtime/single regular inode required")
    files = [probe.PLAN, probe.AUTHORITY, result_path]
    files += [probe.OUTPUT / name for name in ("strace.log", "stdout.log", "stderr.log")]
    provenance = {str(path): digest(path) for path in files}
    require(all(path.stat().st_size <= 16 * 1024**2 for path in files), "bounded complete import-only provenance required")
    return dict(cache_path=str(probe.PRIVATE_CACHE), native_path=str(native), native_sha256=NATIVE_SHA256,
                native_size_bytes=NATIVE_SIZE, native_member=NATIVE_MEMBER, zip_mtime_local=list(info.date_time),
                native_generation={key: getattr(row, key) for key in
                    ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns", "st_mode", "st_nlink")},
                producer_source_sha256=dict(PROBE_SOURCE_PINS), producer_provenance_sha256=provenance,
                unchanged_original_pkg_resources_bootstrap=True, extra_native_copy=False)


def verify_native_runtime_input(plan):
    expected = plan["native_runtime_input"]
    require(json.dumps(expected, sort_keys=True) == json.dumps(native_runtime_input_contract(), sort_keys=True),
            "native runtime input/provenance generation differs")
    require(digest(expected["native_path"]) == NATIVE_SHA256 == expected["native_sha256"],
            "full native runtime bytes differ; extraction or truncated cache denied")
    return dict(path=expected["native_path"], sha256=NATIVE_SHA256, size_bytes=NATIVE_SIZE,
                native_generation=expected["native_generation"], full_private_input_verified=True,
                extra_native_copy=False, original_file_limit_bytes=16 * 1024**2)


def preserve_engine_ini_input(plan):
    # HH_261005 - Preserve the pristine input separately; Unreal may expand its owned working INI but never a shared asset.
    working = OUTPUT / "Town03.Engine.ini"
    require(digest(working) == plan["generated_engine_ini_sha256"], "initial owned map selection differs")
    target = OUTPUT / "source_generated/Town03.Engine.pristine.ini"
    target.parent.mkdir(exist_ok=False)
    with working.open("rb") as source, target.open("xb") as copy:
        shutil.copyfileobj(source, copy)
    require(digest(target) == plan["generated_engine_ini_sha256"], "pristine generated input copy differs")


def engine_map_keys(text):
    section, found = None, {}
    keys = ("EditorStartupMap", "GameDefaultMap", "ServerDefaultMap", "TransitionMap")
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith(("#", ";")) or not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line
        elif section == "[/Script/EngineSettings.GameMapsSettings]" and "=" in line:
            key, value = (part.strip() for part in line.split("=", 1))
            if key in keys:
                require(key not in found, "duplicate owned map-selection key")
                found[key] = value
    require(found == {key: "/Game/Carla/Maps/Town03.Town03" for key in keys},
            "all four authentic Town03 map-selection keys required")
    return found


def engine_ini_postcheck(plan, *, require_actual_map):
    pristine, working = OUTPUT / "source_generated/Town03.Engine.pristine.ini", OUTPUT / "Town03.Engine.ini"
    require(digest(pristine) == plan["generated_engine_ini_sha256"], "pristine input seal changed")
    require(canonical(working).stat().st_size <= 1024**2, "bounded actual working INI required")
    before = digest(working)
    raw = working.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == before, "working INI changed during final read")
    keys = engine_map_keys(raw.decode("utf-8"))
    capture_path = OUTPUT / "capture_result.json"
    capture_sha = digest(capture_path) if capture_path.exists() else None
    capture = read_json(capture_path) if capture_sha else {}
    actual = capture.get("actual_map_identity")
    require(not require_actual_map or actual in ("Town03", "/Game/Carla/Maps/Town03"),
            "successful capture requires its authentic Town03 world-map identity")
    receipt = dict(pristine_path=str(pristine), pristine_sha256=plan["generated_engine_ini_sha256"],
                   working_path=str(working), working_sha256=before, working_size_bytes=len(raw),
                   working_INI_mutated=(before != plan["generated_engine_ini_sha256"]),
                   original_four_map_keys=keys, actual_map_identity=actual, capture_result_sha256=capture_sha,
                   mutable_working_INI_explicitly_declared=True, shared_INI_overwritten=False)
    write_new(OUTPUT / "working_engine_ini_receipt.json", receipt)
    return receipt


def argv_value(argv, flag):
    require(argv.count(flag) == 1 and argv.index(flag) + 1 < len(argv), "one CLI-bound " + flag + " required")
    value = argv[argv.index(flag) + 1]
    require(re.fullmatch(r"[0-9a-f]{64}", value), "exact CLI-bound SHA required")
    return value


def verify_launch_authority(plan, expected, launch_sha=None):
    launch_sha = launch_sha or argv_value(sys.argv, "--launch-authority-sha256")
    authority = read_json(LAUNCH_AUTHORITY, limit=32768)
    require(digest(LAUNCH_AUTHORITY) == launch_sha and authority.get("schema") == SCHEMA + ".ROOT_launch_authority"
            and authority.get("explicit_owned_stationary_launch_approved") is True
            and authority.get("declaration_path") == str(PLAN) and authority.get("declaration_sha256") == expected
            and authority.get("source_sha256") == plan["source_sha256"]
            and authority.get("asset_sha256") == plan["asset_sha256"], "genuine CLI-bound ROOT launch approval required")
    return authority


def process_identity(pid):
    row = guards.process_record(pid)
    require(row is not None and row["state"] != "Z", "live owned PID required")
    return {key: value for key, value in row.items() if key != "state"}


def same_process(record):
    require(process_identity(record["pid"]) == record, "owned process generation or ancestry changed")


def same_creation_core(core):
    row = guards.process_record(core["pid"])
    require(row is not None and row["state"] != "Z" and all(row[key] == value for key, value in core.items()),
            "fresh child creation generation or ancestry changed")
    require(core["ppid"] == os.getpid() and core["pgid"] == core["pid"]
            and os.getsid(core["pid"]) == core["pid"], "fresh owned child session required")
    return row


def admit_owned_child(process, command, identities, name, *, timeout_s=5.):
    # HH_261005 - Store the authentic creation generation before executable admission; a transient pre-exec argv never leaves an untracked child.
    require(timeout_s == 5., "fixed exec-admission budget required")
    row = guards.process_record(process.pid)
    require(row is not None, "child creation record unavailable")
    core = {key: row[key] for key in ("pid", "start_ticks", "ppid", "pgid")}
    identities[name] = core
    deadline = time.monotonic() + timeout_s
    while True:
        require(time.monotonic() <= deadline, "exact child executable readiness deadline")
        row = same_creation_core(core)
        if row["argv"] == command:
            identity = {key: value for key, value in row.items() if key != "state"}
            identities[name] = identity
            return identity
        require(process.poll() is None and time.monotonic() < deadline, "exact child executable readiness failed")
        time.sleep(.001)


def substituted(command, expected, launch_sha):
    return [expected if item == PLAN_TOKEN else launch_sha if item == AUTH_TOKEN else item for item in command]


def verify_owner_receipt(plan, expected):
    receipt = read_json(OUTPUT / "owner_started.json", limit=32768)
    require(receipt.get("schema") == SCHEMA + ".owner_started" and receipt["declaration_sha256"] == expected,
            "authentic owned-start receipt required")
    for name in ("owner", "server", "socket_owner"):
        same_process(receipt[name])
    owner, server, shipping = (receipt[name] for name in ("owner", "server", "socket_owner"))
    require(owner["argv"] == substituted(plan["owner_command"], expected, receipt["launch_authority_sha256"])
            and server["argv"] == plan["server_command"] and shipping == server
            and server["ppid"] == owner["pid"] and server["pgid"] == server["pid"]
            and os.getsid(server["pid"]) == server["pid"]
            and guards.listening_socket_owned(server["pid"], PORT), "exact direct shipping owner/session/socket proof required")
    verify_launch_authority(plan, expected, receipt["launch_authority_sha256"])
    return receipt


def directory_bytes(directory):
    total = 0
    for path in canonical(directory).rglob("*"):
        require(not path.is_symlink(), "no symlink in bounded attempt")
        require(path.is_file() or path.is_dir(), "no special object in bounded attempt")
        if path.is_file():
            total += path.stat().st_size
    return total


def check_budget(output=OUTPUT, *, startup=False, free=None):
    used = directory_bytes(output) if Path(output).exists() else 0
    available = shutil.disk_usage(ROOT).free if free is None else free
    require(type(available) is int and available >= FLOOR + SHARED + (ATTEMPT if startup else CLOSING),
            "unchanged free floor/shared/attempt-or-closing reserve required")
    require(used <= ATTEMPT - CLOSING, "attempt quota closing reserve reached")
    return dict(used_bytes=used, available_bytes=available, attempt_quota_bytes=ATTEMPT,
                free_disk_floor_bytes=FLOOR, shared_safety_reserve_bytes=SHARED, closing_reserve_bytes=CLOSING)


def gpu_available(plan, *, output=None):
    inventory = subprocess.run(["nvidia-smi", "--query-gpu=index,uuid,memory.used,utilization.gpu", "--format=csv,noheader,nounits"],
                               check=True, capture_output=True, text=True, timeout=10).stdout
    rows = [line.split(",") for line in inventory.splitlines()]
    selected = [row for row in rows if row[0].strip() == "0"]
    require(len(selected) == 1 and len(selected[0]) == 4, "one exact local GPU0 required")
    _, uuid, memory, utilization = (item.strip() for item in selected[0])
    require(uuid == plan["gpu_uuid"] and int(memory) <= plan["gpu_memory_limit_MiB"]
            and int(utilization) <= plan["gpu_utilization_limit_percent"], "declared GPU0 unavailable")
    apps = subprocess.run(["nvidia-smi", "--query-compute-apps=gpu_uuid,pid,process_name", "--format=csv,noheader,nounits"],
                          check=True, capture_output=True, text=True, timeout=10).stdout
    require(not any(line.split(",")[0].strip() == uuid for line in apps.splitlines()), "foreign GPU0 compute job present")
    # HH_261005 - Preserve desktop graphics load instead of claiming an idle or uncontended moving-vehicle benchmark.
    full = subprocess.run(["nvidia-smi"], check=True, capture_output=True, text=True, timeout=10).stdout
    require(len(full.encode()) <= 32768 and len(apps.encode()) <= 32768, "bounded full NVIDIA/compute inventory required")
    graphics_ids = [int(match.group(1)) for line in full.splitlines()
                   if (match := re.match(r"^\|\s*0\s+\S+\s+\S+\s+(\d+)\s+[CG+]+\s", line))]
    return dict(gpu_uuid=uuid, gpu_index=0, memory_used_MiB=int(memory), utilization_percent=int(utilization),
                compute_app_inventory=apps, full_NVIDIA_graphics_inventory=full,
                NVIDIA_GPU0_process_identities=[process_identity(pid) for pid in graphics_ids],
                stationary_under_desktop_load_only=True)


def no_other_carla():
    for row in guards.all_processes():
        if row["argv"] and ("/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" in row["argv"][0]
                            or "/Engine/Binaries/Linux/UE4Editor" in row["argv"][0]):
            raise RuntimeError("another CARLA process exists; no foreign attach")


def environment(plan, *, server=False):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", CUDA_VISIBLE_DEVICES=plan["gpu_uuid"] if server else "",
               OMP_NUM_THREADS="1", MKL_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
               XDG_CACHE_HOME=str(OUTPUT / "UserCache"), CUDA_CACHE_PATH=str(OUTPUT / "UserCache/CUDA"),
               PYTHON_EGG_CACHE=plan["native_runtime_input"]["cache_path"])
    for name in ("PYTHONPATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
        env.pop(name, None)
    return env


def file_limit():
    # HH_261005 - This limit affects only a freshly started child, not the host or another test.
    resource.setrlimit(resource.RLIMIT_FSIZE, (16 * 1024**2, 16 * 1024**2))


def archive_sources(plan, expected, launch_sha):
    records = [(ROOT / name, OUTPUT / "source" / name, sha) for name, sha in plan["source_sha256"].items()]
    records += [(Path(name), OUTPUT / "source_external" / Path(name).relative_to(CARLA_ROOT), sha)
                for name, sha in plan["asset_sha256"].items()]
    records += [(Path(name), OUTPUT / "runtime_input_provenance" / str(index).zfill(2) / Path(name).name, sha)
                for index, (name, sha) in enumerate(plan["native_runtime_input"]["producer_provenance_sha256"].items())]
    native_receipt = verify_native_runtime_input(plan)
    receipt_path = OUTPUT / "runtime_native_input_receipt.json"
    write_new(receipt_path, dict(native_receipt, sealed_plan_native_runtime_input=plan["native_runtime_input"]))
    records.append((receipt_path, OUTPUT / "source_runtime_input/native_receipt.json", digest(receipt_path)))
    records += [(PLAN, OUTPUT / "declaration.json", expected),
                (LAUNCH_AUTHORITY, OUTPUT / "ROOT_launch_authority.json", launch_sha)]
    require(sum(source.stat().st_size for source, _, _ in records) < ATTEMPT - CLOSING, "sealed archive exceeds attempt quota")
    for source, target, sha in records:
        require(digest(source) == sha, "source changed before archive")
        target.parent.mkdir(parents=True, exist_ok=True)
        with source.open("rb") as original, target.open("xb") as destination:
            shutil.copyfileobj(original, destination, 1024**2)
        require(digest(source) == sha == digest(target), "actual/archive bytes differ")
    return records


def verify_archived_sources(plan, expected, launch_sha):
    # HH_261005 - The capture may import the real API only after every actual/source archive and ROOT launch receipt agrees.
    require(all(digest(ROOT / name) == sha == digest(OUTPUT / "source" / name)
                for name, sha in plan["source_sha256"].items()), "actual/archived source closure differs")
    require(all(digest(name) == sha == digest(OUTPUT / "source_external" / Path(name).relative_to(CARLA_ROOT))
                for name, sha in plan["asset_sha256"].items()), "actual/archived CARLA assets differ")
    require(digest(PLAN) == expected == digest(OUTPUT / "declaration.json")
            and digest(LAUNCH_AUTHORITY) == launch_sha == digest(OUTPUT / "ROOT_launch_authority.json"), "actual/archived approval differs")
    require(all(digest(name) == sha == digest(OUTPUT / "runtime_input_provenance" / str(index).zfill(2) / Path(name).name)
                for index, (name, sha) in enumerate(plan["native_runtime_input"]["producer_provenance_sha256"].items())),
            "actual/archive native input provenance differs")
    verify_native_runtime_input(plan)
    native_receipt = read_json(OUTPUT / "runtime_native_input_receipt.json")
    require(native_receipt["sealed_plan_native_runtime_input"] == plan["native_runtime_input"]
            and digest(OUTPUT / "runtime_native_input_receipt.json") == digest(OUTPUT / "source_runtime_input/native_receipt.json"),
            "current/archive runtime native receipt differs")
    return True


def stop_owned(process, identity):
    if process is None:
        return
    if process.poll() is None:
        require(identity is not None, "creation generation unavailable; cannot safely signal")
        verify = same_process if "argv" in identity else same_creation_core
        verify(identity)
        require(identity["pgid"] == process.pid and os.getsid(process.pid) == process.pid,
                "owned child session required before signal")
        for sig, timeout in ((signal.SIGTERM, 5), (signal.SIGKILL, 2)):
            if process.poll() is not None:
                break
            verify(identity)
            os.killpg(process.pid, sig)
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                pass
    require(process.poll() is not None and not any(row["pgid"] == process.pid and row["state"] != "Z"
            for row in guards.all_processes()), "owned child/session survivors remain")


def wait_release():
    deadline = time.monotonic() + 75.
    while True:
        try:
            guards.ports_free(PORT)
            return dict(TCP_UDP_triplet_released=True, port_triplet=[2440, 2441, 2442])
        except OSError:
            require(time.monotonic() < deadline, "original owned ports did not naturally release")
            time.sleep(.25)


def preflight(expected, launch_sha=None):
    plan = verify_plan(expected)
    require(sys.version_info[:2] == (3, 10) and not OUTPUT.exists() and os.environ.get("CUDA_VISIBLE_DEVICES") == ""
            and "torch" not in sys.modules, "fresh CPU-only owner/Python3.10/output required")
    canonical(OUTPUT, exists=False)
    verify_native_runtime_input(plan)
    no_other_carla()
    guards.ports_free(PORT)
    disk, gpu = check_budget(startup=True), gpu_available(plan)
    if launch_sha is not None:
        verify_launch_authority(plan, expected, launch_sha)
    return plan, dict(status="PREPARED_NO_CARLA_CONNECTION", disk=disk, GPU0=gpu)


def execute(expected, launch_sha):
    # HH_261005 - ROOT launch authority and the later dynamic wheel authority remain separate, never issued by this owner.
    with guards.runtime_lease():
        plan, pre = preflight(expected, launch_sha)
        OUTPUT.mkdir()
        write_new(OUTPUT / "preflight.json", pre)
        server = capture = None
        identities, records, cleanup_errors, error, capture_code = {}, [], [], None, None
        engine_receipt = None
        started = time.monotonic()
        handlers = {}
        def interrupted(number, _frame):
            raise RuntimeError("owner interrupted by signal " + str(number))
        try:
            for number in (signal.SIGINT, signal.SIGTERM):
                handlers[number] = signal.signal(number, interrupted)
            records = archive_sources(plan, expected, launch_sha)
            with (OUTPUT / "Town03.Engine.ini").open("x") as stream:
                stream.write(engine_ini())
            require(digest(OUTPUT / "Town03.Engine.ini") == plan["generated_engine_ini_sha256"], "generated owned map selection changed")
            preserve_engine_ini_input(plan)
            check_budget()
            no_other_carla()
            guards.ports_free(PORT)
            gpu_available(plan)
            with (OUTPUT / "server.log").open("xb") as logfile:
                server = subprocess.Popen(plan["server_command"], cwd=CARLA_ROOT, env=environment(plan, server=True),
                                          stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True, preexec_fn=file_limit)
            admit_owned_child(server, plan["server_command"], identities, "server")
            deadline = time.monotonic() + plan["startup_timeout_s"]
            while not guards.listening_socket_owned(server.pid, PORT):
                same_process(identities["server"])
                require(time.monotonic() < deadline and server.poll() is None, "owned server readiness failed")
                check_budget()
                time.sleep(.1)
            write_new(OUTPUT / "owner_started.json", dict(schema=SCHEMA + ".owner_started", owner=process_identity(os.getpid()),
                      server=identities["server"], socket_owner=identities["server"], declaration_sha256=expected,
                      launch_authority_sha256=launch_sha, wheel_workers_authorized=False))
            verify_owner_receipt(plan, expected)
            command = substituted(plan["worker_command"], expected, launch_sha)
            with (OUTPUT / "capture.log").open("xb") as logfile:
                capture = subprocess.Popen(command, cwd=ROOT, env=environment(plan), stdout=logfile,
                                           stderr=subprocess.STDOUT, start_new_session=True, preexec_fn=file_limit)
            admit_owned_child(capture, command, identities, "capture")
            write_new(OUTPUT / "capture_started.json", dict(schema=SCHEMA + ".capture_started",
                      capture=identities["capture"], declaration_sha256=expected, launch_authority_sha256=launch_sha))
            while capture.poll() is None:
                require(time.monotonic() - started < plan["owner_wall_timeout_s"], "fixed owner wall budget reached")
                same_process(identities["capture"])
                same_process(identities["server"])
                check_budget()
                time.sleep(.1)
            capture_code = capture.returncode
            require(capture_code == 0, "capture failed; all attempted/unexecuted receipts retained")
        except BaseException as exc:
            error = type(exc).__name__ + ": " + str(exc)
        finally:
            for number, handler in handlers.items():
                signal.signal(number, handler)
            for name, process in (("capture", capture), ("server", server)):
                try:
                    stop_owned(process, identities.get(name))
                except BaseException as exc:
                    cleanup_errors.append(name + ": " + type(exc).__name__ + ": " + str(exc))
            release = None
            try:
                release = wait_release()
            except BaseException as exc:
                cleanup_errors.append("ports: " + str(exc))
            try:
                require(all(digest(source) == sha == digest(target) for source, target, sha in records), "actual/archive sources changed")
                require(digest(PLAN) == expected and source_pins() == plan["source_sha256"], "plan/source closure changed")
                verify_native_runtime_input(plan)
                final_budget = check_budget()
            except BaseException as exc:
                cleanup_errors.append("seals/budget: " + str(exc))
                final_budget = None
            try:
                engine_receipt = engine_ini_postcheck(plan, require_actual_map=capture_code == 0)
            except BaseException as exc:
                cleanup_errors.append("working INI: " + type(exc).__name__ + ": " + str(exc))
            result = dict(schema=SCHEMA + ".owner_result", status="FAIL_RETAINED" if error or cleanup_errors else "OWNED_STATIONARY_CAPTURE_COMPLETE",
                          error=error, cleanup_errors=cleanup_errors,
                          capture_exit_code=capture.poll() if capture is not None else capture_code, declared_frames=30,
                          declared_variant_calls=60, declaration_sha256=expected, launch_authority_sha256=launch_sha,
                          port_release=release, final_budget=final_budget, owner_elapsed_seconds=time.monotonic() - started,
                          working_engine_ini=engine_receipt, native_runtime_input=plan["native_runtime_input"],
                          original_driving_gates_changed=False, driving_or_10Hz_gain_proven=False, dataset_admitted=False,
                          formal_main_domains_approved=0, formal_subfunctions_approved=0)
            write_new(OUTPUT / "owner_result.json", result)
        print(json.dumps(result, allow_nan=False))
        return 0 if not error and not cleanup_errors else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("declare", "preflight", "execute"), default="preflight")
    parser.add_argument("--gpu-uuid")
    parser.add_argument("--plan-sha256")
    parser.add_argument("--launch-authority-sha256")
    args = parser.parse_args(argv)
    if args.mode == "declare":
        plan = fixed_contract(args.gpu_uuid)
        require(all(digest(path) == sha for path, sha in ASSET_SHA256.items()), "declared asset seals differ")
        write_new(PLAN, plan)
        print(json.dumps(dict(declaration_path=str(PLAN), declaration_sha256=digest(PLAN), actual_CARLA_connected=False)))
        return 0
    if args.mode == "execute":
        require(args.launch_authority_sha256 is not None, "ROOT CLI-bound launch authority required")
        return execute(args.plan_sha256, args.launch_authority_sha256)
    _, report = preflight(args.plan_sha256, args.launch_authority_sha256)
    print(json.dumps(report, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
