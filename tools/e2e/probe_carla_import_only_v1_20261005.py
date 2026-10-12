"""HH_261005 - Diagnose native import only using a sealed private prefilled cache; no Client, server or shared-cache writes.

Python audit checks are not an OS sandbox for native C++ syscalls. Strace and
faulthandler retain actual failures; this changed cache path is not V1 replay.
"""

import argparse
from contextlib import contextmanager
import fcntl
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
import zipfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_carla_import_only_probe_v1_20261005.py"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-05/carla_import_only_v1"
PLAN = ROOT / "storage/experiments/plans/2026-10-05/carla_import_only_v1/ROOT_plan.json"
AUTHORITY = PLAN.with_name("ROOT_execution_authority.json")
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
EGG = Path("/home/a/carla-autoware-universe/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg")
MEMBER = "carla/libcarla.cpython-310-x86_64-linux-gnu.so"
CACHE_NATIVE = Path("/home/a/.cache/Python-Eggs") / (EGG.name + "-tmp") / MEMBER
PRIVATE_CACHE = OUTPUT / "Python-Eggs"
PRIVATE_NATIVE = PRIVATE_CACHE / (EGG.name + "-tmp") / MEMBER
NATIVE_SIZE = 153958688
NATIVE_SHA = "d60de268813db6e1c94b8f613888ed5f9eaed39cb02ac2debb4e1a05aedb8966"
SCHEMA = "portable_e2e.carla_import_only_probe.20261005.v1"
ASSETS = {
    str(EGG): "6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a",
    str(CACHE_NATIVE): NATIVE_SHA,
    "/usr/bin/python3.10": "a2f33a6e006989270f4340528eb61f8f97366e00a5d1b602ac8672ea44fc56ae",
    "/usr/bin/strace": "38a5c75cb29dd85ddd7780d54f5bf595554d7a1b5c42524b23065f5dc4c4b01d",
    "/usr/lib/python3/dist-packages/pkg_resources/__init__.py": "b809cfabc16c4d71c01071562bb514f4085d363138a394a47e4f02c9f6f3b4ef",
}
SMALL_MEMBERS = {"carla/__init__.py": "f0f260da9eafaf3168d89244ee0353dc013709fac905fd08422bc6eee2238062",
                 "carla/libcarla.py": "76fb04fc4a8522b3560c2a7b4fee1ace104955c5f525f2f53200e52b370fe667"}


def require(value, message):
    if not value: raise RuntimeError(message)


def canonical(path, exists=True):
    path = Path(path).absolute()
    require(".." not in path.parts and path == path.resolve(strict=exists)
            and all(not part.is_symlink() for part in (path, *path.parents)), "canonical non-symlink path required")
    return path


def stream_sha(stream):
    value = hashlib.sha256()
    for block in iter(lambda: stream.read(1024**2), b""): value.update(block)
    return value.hexdigest()


def digest(path):
    path = canonical(path)
    require(path.is_file(), "regular sealed file required")
    before = path.stat()
    with path.open("rb") as stream: sha = stream_sha(stream)
    after = path.stat()
    require(tuple(getattr(before, key) for key in ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns"))
            == tuple(getattr(after, key) for key in ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")), "sealed file generation changed")
    return sha


def contract():
    return dict(schema=SCHEMA, output=str(OUTPUT), asset_sha256=ASSETS,
        source_sha256={str(path.relative_to(ROOT)): digest(path) for path in (HERE, TEST)},
        native_size=NATIVE_SIZE, native_member=MEMBER, native_sha256=NATIVE_SHA,
        exact_egg_source_members=SMALL_MEMBERS, private_cache=str(PRIVATE_CACHE),
        file_limit_bytes=16*1024**2, timeout_seconds=30, free_floor_bytes=5*1024**3,
        attempt_reserve_bytes=768*1024**2, use_strace=True,
        shared_reserve_bytes=64*1024**2, closing_reserve_bytes=16*1024**2, runtime_lock=str(LOCK),
        original_full_import_bootstrap_unchanged=True, Python_audit_is_OS_sandbox=False,
        Client_or_server_or_socket_or_GPU_or_NN=False, V1_exact_environment_replay=False)


def read_pinned(path, expected):
    require(type(expected) is str and re.fullmatch(r"[0-9a-f]{64}", expected), "explicit hexadecimal authority SHA required")
    path = canonical(path)
    require(path.stat().st_size <= 1024**2 and digest(path) == expected, "bounded pinned JSON required")
    raw = path.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == expected, "pinned JSON changed during read")
    def pairs(items):
        value = {}
        for key, item in items:
            require(key not in value, "duplicate JSON key")
            value[key] = item
        return value
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=lambda value: require(False, "nonfinite JSON forbidden"))


def verify(plan_sha, authority_sha):
    plan, authority = read_pinned(PLAN, plan_sha), read_pinned(AUTHORITY, authority_sha)
    require(plan == contract() and authority == dict(schema=SCHEMA+".ROOT_execution_authority",
            explicit_import_only_execution_approved=True, plan_path=str(PLAN), plan_sha256=plan_sha,
            source_sha256=plan["source_sha256"], asset_sha256=plan["asset_sha256"]), "fresh exact ROOT import-only authority required")
    require(all(digest(path) == sha for path, sha in ASSETS.items()), "sealed interpreter/trace/egg/cache assets differ")
    return plan


def stage_native():
    # HH_261005 - Copy existing verified bytes, never extract the egg or modify the shared cache; exact zip mtime keeps original _is_current true.
    require(digest(CACHE_NATIVE) == NATIVE_SHA and CACHE_NATIVE.stat().st_size == NATIVE_SIZE, "existing native cache differs")
    with zipfile.ZipFile(EGG) as archive:
        info = archive.getinfo(MEMBER)
        require(info.file_size == NATIVE_SIZE and all(hashlib.sha256(archive.read(name)).hexdigest() == sha
                for name, sha in SMALL_MEMBERS.items()), "authentic egg bootstrap/geometry differs")
        with archive.open(MEMBER) as stream: require(stream_sha(stream) == NATIVE_SHA, "zip native member and cache differ")
        timestamp = time.mktime(info.date_time + (0, 0, -1))
    canonical(PRIVATE_NATIVE, exists=False).parent.mkdir(parents=True, exist_ok=False)
    with CACHE_NATIVE.open("rb") as original, PRIVATE_NATIVE.open("xb") as private:
        shutil.copyfileobj(original, private, 1024**2)
    os.utime(PRIVATE_NATIVE, (timestamp, timestamp))
    require(digest(CACHE_NATIVE) == NATIVE_SHA == digest(PRIVATE_NATIVE)
            and PRIVATE_NATIVE.stat().st_size == NATIVE_SIZE and PRIVATE_NATIVE.stat().st_mtime == timestamp, "private exact native copy failed")


def verify_private_native():
    with zipfile.ZipFile(EGG) as archive:
        info = archive.getinfo(MEMBER)
        timestamp = time.mktime(info.date_time + (0, 0, -1))
    require(PRIVATE_NATIVE.stat().st_size == info.file_size == NATIVE_SIZE
            and PRIVATE_NATIVE.stat().st_mtime == timestamp and digest(PRIVATE_NATIVE) == NATIVE_SHA,
            "full native size/zip mtime/SHA required before import; truncated or stale cache denied")


def budget(startup=False):
    require(shutil.disk_usage(ROOT).free >= 5*1024**3 + 64*1024**2 + (768*1024**2 if startup else 16*1024**2), "unchanged free floor/shared/reserve required")
    total = 0
    if OUTPUT.exists():
        for path in OUTPUT.rglob("*"):
            canonical(path)
            require(path.is_file() or path.is_dir(), "special output object denied")
            if path.is_file(): total += path.stat().st_size
    require(total <= (768-16)*1024**2, "import-only quota closing reserve required")


@contextmanager
def runtime_lease():
    with canonical(LOCK).open("a+") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try: yield
        finally: fcntl.flock(stream, fcntl.LOCK_UN)


def process_core(pid):
    try: raw = Path("/proc/"+str(pid)+"/stat").read_text()
    except FileNotFoundError: return None
    fields = raw.rsplit(")", 1)[1].split()
    return dict(pid=pid, start_ticks=int(fields[19]), ppid=int(fields[1]), pgid=int(fields[2]), sid=int(fields[3]))


def stop_owned(process, core):
    if process is not None and process.poll() is None:
        require(core and process_core(process.pid) == core and core["ppid"] == os.getpid()
                and core["pgid"] == core["sid"] == process.pid, "exact owned diagnostic PID generation/session required")
        os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=2)


def no_owned_survivors(core):
    require(core is not None, "owned diagnostic creation core required")
    for directory in Path("/proc").iterdir():
        if directory.name.isdigit():
            row = process_core(int(directory.name))
            require(row is None or row["pgid"] != core["pgid"], "diagnostic process group survivor retained")


def audit_trace(raw, stdout):
    require(len(raw) <= 16*1024**2 and len(stdout) <= 16*1024**2, "bounded trace/phase logs required")
    phases = [json.loads(line)["phase"] for line in stdout.splitlines()]
    require(phases == ["before_import", "after_import"], "complete actual import phase logs required")
    require('execve("/usr/bin/python3.10"' in raw and "exit_group(0)" in raw
            and "<unfinished ...>" not in raw and "<... " not in raw, "complete successful native trace required")
    require(re.search(r"\b(?:connect|bind|listen|accept|accept4|sendto|sendmsg|recvfrom|recvmsg)\(", raw) is None
            and "socket(AF_INET" not in raw, "native network call observed; no-endpoint claim denied")
    return dict(actual_trace_complete=True, native_endpoint_calls_observed=False, Python_Client_constructor_in_source=False,
                native_syscalls_OS_sandboxed=False)


def audit_policy(event, args):
    # HH_261005 - Reject Python extraction/writes/processes/network, not unobserved native syscalls; existing private mkdir may be idempotent.
    if event == "open":
        require(not ((args[2] or 0) & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND)), "Python write/extraction denied")
    elif event == "os.mkdir":
        path = canonical(args[0])
        require(path.is_dir() and path.is_relative_to(PRIVATE_CACHE), "only existing private cache directory permitted")
    elif event.startswith("socket.") or event in {"os.remove", "os.rename", "os.rmdir", "os.chmod", "os.chown", "os.utime", "os.link", "os.symlink", "os.truncate", "subprocess.Popen", "os.system", "os.exec", "os.fork"}:
        raise RuntimeError("Python mutation/process/socket denied: " + event)


def limits():
    resource.setrlimit(resource.RLIMIT_FSIZE, (16*1024**2, 16*1024**2))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def child():
    require(sys.version_info[:2] == (3, 10) and os.environ.get("CUDA_VISIBLE_DEVICES") == ""
            and os.environ.get("PYTHON_EGG_CACHE") == str(PRIVATE_CACHE)
            and not any(name in sys.modules for name in ("carla", "torch")), "fresh exact CPU-only Python/cache required")
    require(resource.getrlimit(resource.RLIMIT_FSIZE) == (16*1024**2, 16*1024**2), "original file limit required")
    verify_private_native()
    print(json.dumps(dict(phase="before_import", file_limit=resource.getrlimit(resource.RLIMIT_FSIZE))), flush=True)
    sys.addaudithook(audit_policy)
    sys.path.insert(0, str(EGG))
    import carla
    require(str(carla.__file__) == str(EGG)+"/carla/__init__.py", "CARLA import source differs")
    print(json.dumps(dict(phase="after_import", Python_Client_constructor_in_source=False, native_syscalls_OS_sandboxed=False)), flush=True)


def environment():
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", CUDA_VISIBLE_DEVICES="", PYTHON_EGG_CACHE=str(PRIVATE_CACHE),
               OMP_NUM_THREADS="1", MKL_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
               XDG_CACHE_HOME=str(OUTPUT/"UserCache"), CUDA_CACHE_PATH=str(OUTPUT/"UserCache/CUDA"))
    for key in ("PYTHONPATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"): env.pop(key, None)
    return env


def _execute(plan_sha, authority_sha):
    plan = verify(plan_sha, authority_sha)
    budget(startup=True)
    canonical(OUTPUT, exists=False).mkdir()
    result = dict(schema=SCHEMA+".result", status="FAIL_RETAINED", Python_Client_constructor_in_source=False,
                  native_endpoint_calls_verified_absent=False, actual_driving_gain_proven=False)
    process, core = None, None
    try:
        stage_native()
        verify_private_native(); budget()
        argv = ["/usr/bin/strace", "-f", "-qq", "-e", "trace=%file,%process,%network,%signal", "-o", str(OUTPUT/"strace.log"),
                "/usr/bin/python3.10", "-B", "-X", "faulthandler", str(HERE), "--mode", "child",
                "--plan-sha256", plan_sha, "--authority-sha256", authority_sha]
        with (OUTPUT/"stdout.log").open("xb") as out, (OUTPUT/"stderr.log").open("xb") as err:
            process = subprocess.Popen(argv, cwd=ROOT, env=environment(), stdout=out, stderr=err, start_new_session=True, preexec_fn=limits)
            core = process_core(process.pid)
            require(core and core["ppid"] == os.getpid() and core["pgid"] == core["sid"] == process.pid, "fresh diagnostic child core required")
            try: result["exit_code"] = process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                stop_owned(process, core); result["exit_code"] = process.returncode
                raise RuntimeError("bounded import-only timeout; failed logs retained")
        require(result["exit_code"] == 0, "native import child failed; faulthandler/trace retained")
        result["trace_audit"] = audit_trace((OUTPUT/"strace.log").read_text(), (OUTPUT/"stdout.log").read_text())
        result["native_endpoint_calls_verified_absent"] = True
        result["status"] = "IMPORT_ONLY_COMPLETE_NOT_DRIVING"
    except BaseException as error: result["error"] = type(error).__name__+": "+str(error)
    finally:
        try:
            stop_owned(process, core)
            if process is not None: no_owned_survivors(core)
        except BaseException as error: result.update(status="FAIL_RETAINED", cleanup_error=str(error))
        result["child_creation_core"] = core
        result["actual_child_argv"] = argv if process is not None else None
        try: verify(plan_sha, authority_sha); verify_private_native(); budget()
        except BaseException as error: result.update(status="FAIL_RETAINED", final_seal_error=str(error))
        with (OUTPUT/"result.json").open("x") as stream: json.dump(result, stream, indent=2, allow_nan=False)
    return result


def execute(plan_sha, authority_sha):
    with runtime_lease(): return _execute(plan_sha, authority_sha)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("execute", "child"), required=True)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--authority-sha256", required=True)
    args = parser.parse_args(argv)
    if args.mode == "child": verify(args.plan_sha256, args.authority_sha256); child(); return 0
    return 0 if execute(args.plan_sha256, args.authority_sha256)["status"] == "IMPORT_ONLY_COMPLETE_NOT_DRIVING" else 2


if __name__ == "__main__": raise SystemExit(main())
