"""HH_261005 - Preserve frozen V1 read contracts while binding V2's exact CARLA egg to ROOT, fresh plan and actual/archive seals before import.

Only raw FL/FR angles are returned. The caller must retain the exact original
front_steering_measurement and cached get_speed_limit/speed_limit_measurement,
including every original 20Hz sample. Synthetic and real executable paths differ.
"""

import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from multiprocessing import Pipe

from .run_owned_ctrack_isolated_probe_20261001 import process_record, listening_socket_owned


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
REAL_WORKER = HERE.with_name("owned_parallel_wheel_worker_v2_20261005.py")
SYNTHETIC_WORKER = ROOT / "learning/tests/synthetic_parallel_wheel_worker_v2_20261005.py"
TEST = ROOT / "learning/tests/test_owned_parallel_wheel_reads_v2_20261005.py"
AUTHORITY = ROOT / "storage/experiments/plans/2026-10-05/owned_parallel_wheel_reads_v2/ROOT_read_only_endpoint_authority.json"
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
CARLA_EGG = CARLA_ROOT / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
CARLA_EGG_SHA256 = "6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a"
ROLES = ("FL", "FR")
RPC_SECONDS = .05
REQUEST_SECONDS = .1
MAX_REQUESTS = 2200
MAX_MESSAGE = 32768
SCHEMA = "portable_e2e.owned_parallel_wheel_read_prototype.20261005.v2"
FROZEN = {
    "tools/e2e/run_owned_ctrack_isolated_probe_20261001.py": "9b3a8403f576b8567325c51c7acb645832fcd120d7613c717df20a37d1c5cdfd",
    "tools/e2e/prepare_ctrack_isolated_probe_20261001.py": "54cab01d14619dca7fac902144ba52e9ebe1f179ecc72c2e660ccc6642dfd3a0",
}
CHILD_CORE = ("pid", "start_ticks", "ppid", "pgid")


def require(value, message):
    if not value:
        raise RuntimeError(message)


class WheelReadError(RuntimeError):
    """HH_261005 - Carry retained startup/read/cleanup receipts even when construction fails."""
    def __init__(self, message, receipt):
        super().__init__(message)
        self.receipt = receipt


def digest(path):
    path = Path(path).absolute()
    require(".." not in path.parts and path == path.resolve(strict=True)
        and all(not item.is_symlink() for item in (path, *path.parents)) and path.is_file(), "canonical regular source required")
    before = path.stat()
    value = hashlib.sha256(path.read_bytes()).hexdigest()
    after = path.stat()
    require((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns)
        == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns), "source generation changed")
    return value


def sources():
    # HH_261005 - The pinned guard imports only its pinned preparation module and stdlib; scripts/e2e must stay a source-free namespace.
    require(not (ROOT / "tools/e2e/__init__.py").exists(), "unsealed e2e package initializer forbidden")
    values = {name: digest(ROOT / name) for name in FROZEN}
    require(values == FROZEN, "original lightweight PID/socket guard changed")
    for path in (HERE, REAL_WORKER, SYNTHETIC_WORKER, TEST, ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"):
        values[str(path.relative_to(ROOT))] = digest(path)
    return values


def send(connection, value):
    payload = json.dumps(value, separators=(",", ":"), allow_nan=False).encode()
    require(len(payload) <= MAX_MESSAGE, "bounded scalar worker message required")
    connection.send_bytes(payload)


def receive(connection):
    def pairs(items):
        value = {}
        for key, item in items:
            require(key not in value, "duplicate worker JSON key")
            value[key] = item
        return value
    return json.loads(connection.recv_bytes(MAX_MESSAGE), object_pairs_hook=pairs,
        parse_constant=lambda value: require(False, "nonfinite worker JSON forbidden"))


def same_generation(expected):
    actual = process_record(expected["pid"])
    require(actual and actual["state"] != "Z" and all(actual[key] == expected[key]
        for key in ("pid", "start_ticks", "ppid", "pgid", "argv")), "owned PID generation/identity changed")
    return {key: value for key, value in actual.items() if key != "state"}


def same_child_core(expected):
    # HH_261005 - Pre-exec PID ancestry/generation proves only ownership for bounded admission or cleanup, never permission for a wheel request.
    actual = process_record(expected["pid"])
    require(actual and actual["state"] != "Z" and all(actual[key] == expected[key] for key in CHILD_CORE),
        "creating child PID generation/ancestry changed")
    return actual


def admit_child(core, argv, deadline):
    # HH_261005 - Pin creation generation first; admit the exact Python argv only after its bounded exec transition completes.
    while True:
        require(time.monotonic() < deadline, "exact child executable readiness deadline")
        actual = same_child_core(core)
        if actual["argv"] == argv:
            return same_generation(actual)
        time.sleep(.001)


def verify_endpoint(proof, *, synthetic, admission=False):
    for name in ("capture", "owner", "server", "socket_owner"):
        same_generation(proof[name])
    require(proof["host"] == "127.0.0.1" and type(proof["port"]) is int and 1024 <= proof["port"] <= 65532
        and type(proof["actor_id"]) is int and proof["actor_id"] > 0
        and isinstance(proof["type_id"], str) and proof["type_id"].startswith("vehicle.")
        and type(proof["world_id"]) is int and proof["world_id"] >= 0
        and type(proof["first_frame"]) is int and proof["first_frame"] >= 0,
        "exact declared local endpoint/ego/epoch/first-frame required")
    if synthetic:
        require(proof["synthetic_only"] is True, "synthetic executable cannot admit real endpoint evidence")
    else:
        require(proof["synthetic_only"] is False and proof["capture"]["ppid"] == proof["owner"]["pid"]
            and proof["server"]["ppid"] == proof["owner"]["pid"] and proof["server"]["pgid"] == proof["server"]["pid"]
            and proof["socket_owner"]["pgid"] == proof["server"]["pid"]
            and os.getsid(proof["socket_owner"]["pid"]) == os.getsid(proof["server"]["pid"]) == proof["server"]["pid"],
            "exact owned launcher/shipping process ancestry and group required")
        if admission:
            require(listening_socket_owned(proof["socket_owner"]["pid"], proof["port"]), "exact owned shipping RPC socket required")


def admitted_json(path, expected, limit):
    path = Path(path).absolute()
    require(path.stat().st_size <= limit and re.fullmatch(r"[0-9a-f]{64}", expected) is not None
        and digest(path) == expected, "bounded canonical pinned admission JSON required")
    raw = path.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == expected, "admission JSON changed while read")
    return json.loads(raw)


def admit_carla_asset(proof, authority, plan):
    # HH_261005 - Caller-provided egg identity alone is insufficient: exact ROOT and fresh-plan seals must agree with both canonical actual and owned archive bytes.
    expected = CARLA_EGG_SHA256
    run = Path(plan["output"])
    archive = run / "source_external" / CARLA_EGG.relative_to(CARLA_ROOT)
    require(run.is_relative_to(ROOT / "storage/experiments/validation/2026-10-05")
        and proof["carla_egg_path"] == authority["carla_egg_path"] == str(CARLA_EGG)
        and type(proof["carla_egg_sha256"]) is str and re.fullmatch(r"[0-9a-f]{64}", proof["carla_egg_sha256"]) is not None
        and proof["carla_egg_sha256"] == authority["carla_egg_sha256"] == expected
        and plan["asset_sha256"].get(str(CARLA_EGG)) == expected
        and authority["carla_egg_archive_path"] == str(archive), "exact ROOT/plan canonical CARLA egg and archived asset seal required")
    require(digest(CARLA_EGG) == expected and digest(archive) == expected,
        "declared actual and archived CARLA egg bytes differ")
    return dict(carla_egg_path=str(CARLA_EGG), carla_egg_sha256=expected,
        carla_egg_archive_path=str(archive), actual_and_archived_bytes_verified=True)


def admit_real_authority(proof):
    # HH_261005 - No self-signed arbitrary owner JSON can authorize a connection: this fixed ROOT authority and prospective plan are mandatory.
    require(Path(proof["authority_path"]) == AUTHORITY, "exact new ROOT-approved endpoint authority path required")
    authority = admitted_json(AUTHORITY, proof["authority_sha256"], MAX_MESSAGE)
    require(authority["schema"] == SCHEMA + ".ROOT_read_only_endpoint_authority"
        and authority["explicit_read_only_wheel_workers_approved"] is True
        and authority["source_sha256"] == sources(), "root review/source closure does not admit this adapter")
    declared = Path(authority["declaration_path"])
    require(declared == Path(proof["declaration_path"]) and declared.is_relative_to(ROOT / "storage/experiments/validation/2026-10-05")
        and declared.name.endswith("_plan.json") and declared.name != "temporal_route_joint_town03_known_dev_straight_full209_v5_plan.json",
        "fresh exact Oct5 prospective actual plan required; V5 is immutable")
    require(authority["declaration_sha256"] == proof["declaration_sha256"], "authority/declaration SHA differs")
    plan = admitted_json(declared, proof["declaration_sha256"], 4 * 1024**2)
    run = Path(plan["output"])
    require(plan["schema"] == authority["declaration_schema"] and plan["declaration_path"] == str(declared)
        and run.is_relative_to(ROOT / "storage/experiments/validation/2026-10-05")
        and Path(proof["owner_receipt_path"]) == run / "owner_started.json"
        and proof["owner_receipt_sha256"] == authority["owner_receipt_sha256"]
        and plan["port_triplet"] == [proof["port"], proof["port"] + 1, proof["port"] + 2]
        and plan["server_command"] == proof["server"]["argv"]
        and authority["shipping_argv"] == proof["socket_owner"]["argv"]
        and "-carla-port=" + str(proof["port"]) in proof["socket_owner"]["argv"]
        and all(plan["source_sha256"].get(name) == value for name, value in authority["source_sha256"].items()),
        "approved run/port/server argv/source binding differs")
    owner_argv = proof["owner"]["argv"]
    require("--plan-sha256" in owner_argv and owner_argv[owner_argv.index("--plan-sha256") + 1] == proof["declaration_sha256"],
        "owning parent was not launched against exact prospective plan")
    return admit_carla_asset(proof, authority, plan)


def admit_receipt(proof, *, synthetic):
    verify_endpoint(proof, synthetic=synthetic, admission=True)
    require(type(proof["declaration_sha256"]) is str and re.fullmatch(r"[0-9a-f]{64}", proof["declaration_sha256"]) is not None,
        "explicit hexadecimal declaration SHA required")
    require(Path(proof["owner_receipt_path"]).stat().st_size <= MAX_MESSAGE, "bounded original owner receipt required")
    require(digest(proof["owner_receipt_path"]) == proof["owner_receipt_sha256"], "sealed owner receipt changed")
    raw = Path(proof["owner_receipt_path"]).read_bytes()
    require(len(raw) <= MAX_MESSAGE and hashlib.sha256(raw).hexdigest() == proof["owner_receipt_sha256"],
        "owner receipt changed during admission")
    receipt = json.loads(raw)
    require(receipt["declaration_sha256"] == proof["declaration_sha256"] and all(receipt[name] == proof[name]
        for name in ("owner", "server", "socket_owner")), "original owner receipt/PID identities differ")
    if not synthetic:
        return admit_real_authority(proof)


def exact_parent_identity(identity, frame, proof):
    require(identity == (frame, proof["world_id"], proof["actor_id"], proof["type_id"]),
        "parent snapshot frame, ego identity or map epoch changed")


def worker_loop(connection, backend_factory, *, synthetic):
    # HH_261005 - Executable selection, never an IPC mode flag, decides whether a backend can import CARLA.
    init = receive(connection)
    proof, role = init["proof"], init["role"]
    require(role in ROLES and os.environ.get("CUDA_VISIBLE_DEVICES") == "" and "torch" not in sys.modules,
        "fresh CPU-only fixed-role worker required")
    require(os.getppid() == proof["capture"]["pid"], "worker lacks exact creating capture parent")
    libc = ctypes.CDLL(None, use_errno=True)
    require(libc.prctl(1, signal.SIGTERM, 0, 0, 0) == 0 and os.getppid() == proof["capture"]["pid"],
        "owned worker parent-death guard failed")
    asset_admission = admit_receipt(proof, synthetic=synthetic)
    require(sources() == init["sources_sha256"], "worker source closure changed before initialization")
    backend = backend_factory(proof, role, asset_admission)
    worker_identity = same_generation(process_record(os.getpid()))
    send(connection, dict(kind="ready", role=role, token=init["session"], identity=worker_identity,
        CUDA_VISIBLE_DEVICES="", torch_imported=False))
    expected_sequence = 1
    try:
        while True:
            message = receive(connection)
            if message.get("kind") == "shutdown":
                require(message["session"] == init["session"], "shutdown session differs")
                break
            started = time.perf_counter_ns()
            row = dict(message, worker=worker_identity, worker_started_monotonic_ns=started,
                status="FAIL", error=None)
            try:
                require(message["kind"] == "read" and message["role"] == role
                    and message["sequence"] == expected_sequence and message["token"] == init["session"] + ":" + str(expected_sequence)
                    and type(message["frame"]) is int and message["frame"] >= 0, "fixed role/request sequence/token/frame required")
                expected_sequence += 1
                same_generation(worker_identity)
                verify_endpoint(proof, synthetic=synthetic)
                require(time.perf_counter_ns() <= message["deadline_monotonic_ns"], "request already exceeded fixed deadline")
                row["backend_read_started_monotonic_ns"] = time.perf_counter_ns()
                try:
                    value, clocks = backend.read(message, RPC_SECONDS)
                finally:
                    row["backend_read_finished_monotonic_ns"] = time.perf_counter_ns()
                require(type(value) is float and math.isfinite(value), "finite physical wheel float required")
                verify_endpoint(proof, synthetic=synthetic)
                same_generation(worker_identity)
                require(time.perf_counter_ns() <= message["deadline_monotonic_ns"], "wheel response exceeded fixed request deadline")
                row.update(status="PASS", wheel_angle_degrees=value, **clocks)
            except Exception as error:
                row["error"] = type(error).__name__ + ": " + str(error)
            row["worker_finished_monotonic_ns"] = time.perf_counter_ns()
            send(connection, row)
    finally:
        connection.close()


class OwnedWheelReader:
    def __init__(self, proof, *, synthetic=False, startup_seconds=3., shutdown_seconds=.5):
        self.proof = json.loads(json.dumps(proof, allow_nan=False))
        self.synthetic, self.shutdown_seconds = synthetic, shutdown_seconds
        self.rows, self.cleanup_errors, self.workers, self.lifecycle = [], [], {}, []
        self.closed, self.failed = False, False
        require(0 < startup_seconds <= 5 and 0 < shutdown_seconds <= 1, "bounded initialization/cleanup required")
        admit_receipt(self.proof, synthetic=synthetic)
        require(self.proof["capture"]["pid"] == os.getpid(), "adapter must be created by exact admitted capture")
        self.source_seal = sources()
        self.session = os.urandom(16).hex()
        self.script = SYNTHETIC_WORKER if synthetic else REAL_WORKER
        self.lifecycle.append(dict(kind="startup", started_monotonic_ns=time.perf_counter_ns(), status="STARTED"))
        try:
            deadline = time.monotonic() + startup_seconds
            for role in ROLES:
                parent, child = Pipe(duplex=True)
                env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONDONTWRITEBYTECODE="1")
                argv = [sys.executable, str(self.script), str(child.fileno())]
                try:
                    process = subprocess.Popen(argv,
                        pass_fds=(child.fileno(),), close_fds=True, env=env, stdin=subprocess.DEVNULL,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except BaseException:
                    parent.close()
                    raise
                finally:
                    child.close()
                self.workers[role] = dict(process=process, connection=parent, identity=None, creation_core=None)
                created = process_record(process.pid)
                require(created and created["state"] != "Z" and created["ppid"] == os.getpid()
                    and created["pgid"] == os.getpgrp(), "fresh owned child creation record required")
                self.workers[role]["creation_core"] = {key: created[key] for key in CHILD_CORE}
                self.workers[role]["identity"] = admit_child(self.workers[role]["creation_core"], argv, deadline)
                send(parent, dict(proof=self.proof, role=role, session=self.session, sources_sha256=self.source_seal))
            for role, worker in self.workers.items():
                require(worker["connection"].poll(max(0., deadline - time.monotonic())), "owned worker readiness deadline")
                ready = receive(worker["connection"])
                identity = same_generation(worker["identity"])
                require(identity["ppid"] == os.getpid() and ready == dict(kind="ready", role=role,
                    token=self.session, identity=identity, CUDA_VISIBLE_DEVICES="", torch_imported=False), "exact child readiness required")
            require(time.monotonic() <= deadline, "complete owned pair startup deadline")
            self.lifecycle[-1].update(status="PASS", finished_monotonic_ns=time.perf_counter_ns())
        except BaseException as error:
            self.lifecycle[-1].update(status="FAIL", error=type(error).__name__ + ": " + str(error),
                finished_monotonic_ns=time.perf_counter_ns())
            self.failed = True
            self.close()
            raise WheelReadError("owned wheel startup failed", self.receipt()) from error

    def read(self, frame, parent_identity):
        require(not self.closed and not self.failed and len(self.rows) < MAX_REQUESTS, "live healthy bounded wheel reader required")
        sequence, began = len(self.rows) + 1, time.perf_counter_ns()
        row = dict(sequence=sequence, frame=frame, token=self.session + ":" + str(sequence),
            requested_monotonic_ns=began, status="FAIL", role_requests={}, role_results={}, error=None)
        self.rows.append(row)
        try:
            verify_endpoint(self.proof, synthetic=self.synthetic)
            require(type(frame) is int and frame == self.proof["first_frame"] + sequence - 1,
                "every consecutive original 20Hz native frame must be read; no skipped sampling")
            exact_parent_identity(parent_identity(), frame, self.proof)
            deadline = began + int(REQUEST_SECONDS * 1e9)
            for role, worker in self.workers.items():
                same_generation(worker["identity"])
                message = dict(kind="read", sequence=sequence, frame=frame, role=role, token=row["token"],
                    deadline_monotonic_ns=deadline)
                row["role_requests"][role] = dict(message, send_started_monotonic_ns=time.perf_counter_ns())
                send(worker["connection"], message)
                row["role_requests"][role]["send_finished_monotonic_ns"] = time.perf_counter_ns()
            row["both_sent_monotonic_ns"] = time.perf_counter_ns()
            response_errors = []
            for role, worker in self.workers.items():
                try:
                    require(worker["connection"].poll(max(0., (deadline - time.perf_counter_ns()) / 1e9)), "fixed request response deadline")
                    result = receive(worker["connection"])
                    row["role_results"][role] = result
                    require(result["role"] == role and result["sequence"] == sequence and result["token"] == row["token"]
                        and result["frame"] == frame and result["worker"] == worker["identity"], "wheel response role/token/identity differs")
                except Exception as error:
                    response_errors.append(dict(role=role, error=type(error).__name__ + ": " + str(error)))
            row["response_errors"] = response_errors
            require(not response_errors, "incomplete/invalid pair response; retained both role attempts")
            require(all(value["status"] == "PASS" for value in row["role_results"].values()), "a wheel failed; never fabricate a pair")
            verify_endpoint(self.proof, synthetic=self.synthetic)
            exact_parent_identity(parent_identity(), frame, self.proof)
            for role, worker in self.workers.items():
                same_generation(worker["identity"])
            require(time.perf_counter_ns() <= deadline, "complete pair exceeded original fixed deadline")
            row["status"] = "PASS"
            return tuple(row["role_results"][role]["wheel_angle_degrees"] for role in ROLES)
        except BaseException as error:
            row["error"] = type(error).__name__ + ": " + str(error)
            self.failed = True
            raise
        finally:
            row["finished_monotonic_ns"] = time.perf_counter_ns()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.lifecycle.append(dict(kind="shutdown", started_monotonic_ns=time.perf_counter_ns()))
        for role, worker in self.workers.items():
            process = worker["process"]
            try:
                if worker["identity"] is None:
                    # HH_261005 - An unadmitted child receives no message; close its dedicated pipe before bounded join so pre-init EOF can finish it.
                    worker["connection"].close()
                elif process.poll() is None:
                    same_generation(worker["identity"])
                    send(worker["connection"], dict(kind="shutdown", session=self.session))
                process.wait(timeout=self.shutdown_seconds)
                require(process.returncode == 0, "owned child failed before clean shutdown")
                # HH_261005 - Late failed response clocks are retained after join, never converted into a timely PASS.
                while worker["connection"].poll(0):
                    try:
                        result = receive(worker["connection"])
                    except EOFError:
                        break
                    sequence = result.get("sequence")
                    require(type(sequence) is int and 1 <= sequence <= len(self.rows), "unmatched late worker response")
                    row = self.rows[sequence - 1]
                    require(result["role"] == role and result["token"] == row["token"]
                        and result["frame"] == row["frame"] and result["worker"] == worker["identity"], "late response identity differs")
                    require(role not in row["role_results"], "duplicate worker response")
                    row["role_results"][role] = result
                    row.setdefault("late_roles_retained_without_regrading", []).append(role)
            except BaseException as error:
                self.cleanup_errors.append(dict(role=role, error=type(error).__name__ + ": " + str(error)))
                if process.poll() is None:
                    try:
                        if worker["identity"] is None:
                            same_child_core(worker["creation_core"])
                        else:
                            same_generation(worker["identity"])
                        process.terminate()
                        try:
                            process.wait(timeout=self.shutdown_seconds)
                        except subprocess.TimeoutExpired:
                            if worker["identity"] is None:
                                same_child_core(worker["creation_core"])
                            else:
                                same_generation(worker["identity"])
                            process.kill()
                            process.wait(timeout=self.shutdown_seconds)
                    except BaseException as final_error:
                        self.cleanup_errors.append(dict(role=role, error=type(final_error).__name__ + ": " + str(final_error)))
            finally:
                worker["connection"].close()
        try:
            require(sources() == self.source_seal, "worker sources changed during lifecycle")
            admit_receipt(self.proof, synthetic=self.synthetic)
        except BaseException as error:
            self.cleanup_errors.append(dict(role="final_admission", error=type(error).__name__ + ": " + str(error)))
        self.lifecycle[-1].update(finished_monotonic_ns=time.perf_counter_ns(),
            status="FAIL" if self.cleanup_errors else "PASS")

    def receipt(self):
        return dict(schema=SCHEMA, synthetic_only=self.synthetic, all_requests_including_failures=self.rows,
            lifecycle=self.lifecycle, cleanup_errors=self.cleanup_errors, all_owned_children_joined=self.closed and
                all(worker["process"].poll() is not None for worker in self.workers.values()),
            owned_children={role: dict(pid=worker["process"].pid, creation_core=worker["creation_core"],
                admitted_identity=worker["identity"], returncode=worker["process"].poll()) for role, worker in self.workers.items()},
            source_sha256=self.source_seal, original_20Hz_sampling_or_measurement_math_changed=False,
            runtime_gain_proven=False, actual_capture_activated=False)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        if exc[0] is None:
            require(not self.cleanup_errors, "owned wheel worker cleanup failed; receipts retained")
