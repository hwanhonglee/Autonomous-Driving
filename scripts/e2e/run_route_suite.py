#!/usr/bin/env python3

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SUITE = Path(__file__).with_name("route_suite.default.json")
NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
SCENARIOS = {"lane_follow", "straight", "left", "right", "any"}
ROUTE_FIELDS = {
    "town",
    "weather",
    "scenario",
    "start_index",
    "goal_index",
    "min_distance",
    "max_distance",
    "preferred_distance",
    "sampling_resolution",
    "max_traces",
}
EVALUATION_FIELDS = {
    "ready_timeout",
    "service_timeout",
    "engage_timeout",
    "sim_timeout",
    "wall_timeout",
    "stall_timeout",
    "min_progress",
    "max_cte",
    "max_trajectory_correction",
    "max_goal_distance",
    "max_stop_speed",
    "data_stale_timeout",
    "report_interval",
    "path_sample_distance",
}
PROTECTED_LAUNCH_ARGS = {
    "carla_host",
    "carla_port",
    "map_path",
    "route_file",
    "spawn_point",
    "use_route_manager",
}
STACK_NODE_NAMES = {
    "autoware_carla_interface",
    "carla_ros2_interface",
    "vad_carla_tiny",
    "vad_route_manager",
}


class SuiteError(RuntimeError):
    pass


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run CARLA VAD route episodes sequentially while restarting the bridge and "
            "VAD temporal state for every episode. The CARLA server is not managed."
        )
    )
    parser.add_argument("--suite", type=Path, default=DEFAULT_SUITE)
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="artifact directory; defaults to artifacts/regressions/route_suites/<UTC timestamp>",
    )
    parser.add_argument("--host", default=os.environ.get("CARLA_HOST", "localhost"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("CARLA_PORT", "2000")))
    parser.add_argument(
        "--case",
        action="append",
        dest="cases",
        default=[],
        help="run only this episode name; may be repeated",
    )
    parser.add_argument(
        "--launch-arg",
        action="append",
        default=[],
        help="additional ros2 launch NAME:=VALUE argument; may be repeated",
    )
    parser.add_argument(
        "--fail-fast", action="store_true", help="stop after the first failed episode"
    )
    parser.add_argument(
        "--skip-stack-preflight",
        action="store_true",
        help="skip the ROS graph check for an already running bridge or VAD stack",
    )
    parser.add_argument(
        "--shutdown-timeout",
        type=float,
        default=30.0,
        help="seconds allowed for graceful stack shutdown before SIGTERM",
    )
    parser.add_argument(
        "--episode-gap",
        type=float,
        default=2.0,
        help="seconds between stack shutdown and the next CARLA route preparation",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate and print commands without contacting CARLA or starting ROS nodes",
    )
    args = parser.parse_args()
    if not 0 < args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if args.shutdown_timeout <= 0:
        parser.error("--shutdown-timeout must be positive")
    if args.episode_gap < 0:
        parser.error("--episode-gap cannot be negative")
    validate_launch_args(args.launch_arg, "--launch-arg")
    return args


def require_mapping(value, label):
    if not isinstance(value, dict):
        raise SuiteError(f"{label} must be a JSON object")
    return value


def require_positive_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SuiteError(f"{label} must be a number")
    if not math.isfinite(value) or value <= 0:
        raise SuiteError(f"{label} must be positive and finite")


def validate_launch_args(values, label):
    if not isinstance(values, list) or not all(isinstance(value, str) for value in values):
        raise SuiteError(f"{label} must be a list of strings")
    for value in values:
        if not value or "\0" in value:
            raise SuiteError(f"{label} contains an empty or invalid value")
        key, separator, _ = value.partition(":=")
        if not separator or not key:
            raise SuiteError(f"{label} value must use NAME:=VALUE: {value!r}")
        if key in PROTECTED_LAUNCH_ARGS:
            raise SuiteError(f"{label} cannot override suite-owned launch argument {key!r}")


def validate_evaluation(values, label):
    require_mapping(values, label)
    unknown = set(values) - EVALUATION_FIELDS
    if unknown:
        raise SuiteError(f"{label} has unknown fields: {', '.join(sorted(unknown))}")
    missing = EVALUATION_FIELDS - set(values)
    if missing:
        raise SuiteError(f"{label} is missing fields: {', '.join(sorted(missing))}")
    for key, value in values.items():
        require_positive_number(value, f"{label}.{key}")


def validate_episode(episode):
    label = f"episode {episode.get('name', '<unnamed>')!r}"
    name = episode.get("name")
    if not isinstance(name, str) or not NAME_PATTERN.fullmatch(name):
        raise SuiteError(f"{label} name must match {NAME_PATTERN.pattern}")
    for key in ("town", "weather"):
        if not isinstance(episode.get(key), str) or not episode[key]:
            raise SuiteError(f"{label}.{key} must be a non-empty string")
    if episode.get("scenario") not in SCENARIOS:
        raise SuiteError(f"{label}.scenario must be one of {', '.join(sorted(SCENARIOS))}")

    start_index = episode.get("start_index")
    goal_index = episode.get("goal_index")
    if (start_index is None) != (goal_index is None):
        raise SuiteError(f"{label} must provide both start_index and goal_index")
    for key in ("start_index", "goal_index"):
        value = episode.get(key)
        invalid_index = isinstance(value, bool) or not isinstance(value, int) or value < 0
        if value is not None and invalid_index:
            raise SuiteError(f"{label}.{key} must be a non-negative integer")
    for key in ("min_distance", "max_distance", "preferred_distance", "sampling_resolution"):
        require_positive_number(episode.get(key), f"{label}.{key}")
    if episode["min_distance"] > episode["max_distance"]:
        raise SuiteError(f"{label} has min_distance greater than max_distance")
    max_traces = episode.get("max_traces")
    if isinstance(max_traces, bool) or not isinstance(max_traces, int) or max_traces <= 0:
        raise SuiteError(f"{label}.max_traces must be a positive integer")
    validate_evaluation(episode["evaluation"], f"{label}.evaluation")
    validate_launch_args(episode["launch_args"], f"{label}.launch_args")


def load_suite(path):
    suite_path = path.expanduser().resolve()
    try:
        payload = json.loads(suite_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SuiteError(f"failed to read suite {suite_path}: {error}") from error
    require_mapping(payload, "suite")
    if payload.get("schema_version") != 1:
        raise SuiteError("suite.schema_version must be 1")

    defaults = require_mapping(payload.get("defaults", {}), "suite.defaults")
    allowed_defaults = ROUTE_FIELDS | {"evaluation", "launch_args"}
    unknown_defaults = set(defaults) - allowed_defaults
    if unknown_defaults:
        raise SuiteError(
            f"suite.defaults has unknown fields: {', '.join(sorted(unknown_defaults))}"
        )
    default_evaluation = require_mapping(
        defaults.get("evaluation", {}), "suite.defaults.evaluation"
    )
    default_launch_args = defaults.get("launch_args", [])
    validate_launch_args(default_launch_args, "suite.defaults.launch_args")

    raw_episodes = payload.get("episodes")
    if not isinstance(raw_episodes, list) or not raw_episodes:
        raise SuiteError("suite.episodes must be a non-empty list")

    episodes = []
    names = set()
    allowed_episode_fields = ROUTE_FIELDS | {"name", "enabled", "evaluation", "launch_args"}
    for index, raw_episode in enumerate(raw_episodes):
        raw_episode = require_mapping(raw_episode, f"suite.episodes[{index}]")
        unknown = set(raw_episode) - allowed_episode_fields
        if unknown:
            raise SuiteError(
                f"suite.episodes[{index}] has unknown fields: {', '.join(sorted(unknown))}"
            )
        enabled = raw_episode.get("enabled", True)
        if not isinstance(enabled, bool):
            raise SuiteError(f"suite.episodes[{index}].enabled must be boolean")
        if not enabled:
            continue

        episode = {key: value for key, value in defaults.items() if key in ROUTE_FIELDS}
        episode.update({key: value for key, value in raw_episode.items() if key in ROUTE_FIELDS})
        episode["name"] = raw_episode.get("name")
        episode["evaluation"] = dict(default_evaluation)
        episode["evaluation"].update(
            require_mapping(
                raw_episode.get("evaluation", {}),
                f"episode {episode['name']} evaluation",
            )
        )
        episode["launch_args"] = list(default_launch_args) + list(
            raw_episode.get("launch_args", [])
        )
        validate_episode(episode)
        if episode["name"] in names:
            raise SuiteError(f"duplicate episode name: {episode['name']}")
        names.add(episode["name"])
        episodes.append(episode)

    if not episodes:
        raise SuiteError("suite has no enabled episodes")
    return suite_path, episodes


def atomic_write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def build_prepare_command(episode, route_path, host, port):
    command = [
        str(ROOT / "scripts/e2e/prepare_carla_route.sh"),
        "--host",
        host,
        "--port",
        str(port),
        "--town",
        episode["town"],
        "--weather",
        episode["weather"],
        "--scenario",
        episode["scenario"],
        "--min-distance",
        str(episode["min_distance"]),
        "--max-distance",
        str(episode["max_distance"]),
        "--preferred-distance",
        str(episode["preferred_distance"]),
        "--sampling-resolution",
        str(episode["sampling_resolution"]),
        "--max-traces",
        str(episode["max_traces"]),
        "--output",
        str(route_path),
    ]
    if episode.get("start_index") is not None:
        command.extend(
            [
                "--start-index",
                str(episode["start_index"]),
                "--goal-index",
                str(episode["goal_index"]),
            ]
        )
    return command


def build_stack_command(episode, route_path, host, port, extra_launch_args):
    return [
        str(ROOT / "scripts/e2e/run_route_vad.sh"),
        str(route_path),
        *episode["launch_args"],
        *extra_launch_args,
        f"carla_host:={host}",
        f"carla_port:={port}",
    ]


def build_evaluator_command(episode, route_path, result_path):
    command = [
        str(ROOT / "scripts/e2e/route_test.sh"),
        "--route-file",
        str(route_path),
        "--result",
        str(result_path),
    ]
    for key in sorted(episode["evaluation"]):
        command.extend([f"--{key.replace('_', '-')}", str(episode["evaluation"][key])])
    return command


def build_renderer_command(route_path, result_path, image_path):
    return [
        str(ROOT / "scripts/e2e/render_route_result.sh"),
        str(route_path),
        str(result_path),
        "--output",
        str(image_path),
    ]


def command_string(command):
    import shlex

    return shlex.join(command)


def conflicting_stack_nodes():
    try:
        completed = subprocess.run(
            ["ros2", "node", "list", "--no-daemon", "--spin-time", "2"],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=8.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise SuiteError(
            f"failed to inspect the ROS graph before CARLA preparation: {error}"
        ) from error
    if completed.returncode != 0:
        detail = completed.stderr.strip() or f"exit code {completed.returncode}"
        raise SuiteError(f"failed to inspect the ROS graph before CARLA preparation: {detail}")

    conflicts = []
    for node in completed.stdout.splitlines():
        node = node.strip()
        basename = node.rsplit("/", 1)[-1]
        if basename in STACK_NODE_NAMES or "carla_ros_bridge" in basename:
            conflicts.append(node)
    return sorted(set(conflicts))


class StackProcess:
    def __init__(self, command, log_path, environment, shutdown_timeout):
        self.command = command
        self.log_path = log_path
        self.environment = environment
        self.shutdown_timeout = shutdown_timeout
        self.process = None
        self.log_stream = None

    def start(self):
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_stream = self.log_path.open("w", encoding="utf-8")
        self.log_stream.write(f"command: {command_string(self.command)}\n")
        self.log_stream.flush()
        try:
            self.process = subprocess.Popen(
                self.command,
                cwd=ROOT,
                env=self.environment,
                stdout=self.log_stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            self.log_stream.close()
            self.log_stream = None
            raise

    def poll(self):
        return self.process.poll() if self.process is not None else None

    def _group_alive(self):
        if self.process is None:
            return False
        self.process.poll()
        try:
            os.killpg(self.process.pid, 0)
            return True
        except ProcessLookupError:
            return False

    def _wait_group(self, timeout):
        deadline = time.monotonic() + timeout
        while self._group_alive() and time.monotonic() < deadline:
            time.sleep(0.2)
        return not self._group_alive()

    def stop(self):
        if self.process is None:
            if self.log_stream is not None:
                self.log_stream.close()
                self.log_stream = None
            return None
        try:
            if self._group_alive():
                os.killpg(self.process.pid, signal.SIGINT)
                if not self._wait_group(self.shutdown_timeout):
                    os.killpg(self.process.pid, signal.SIGTERM)
                    if not self._wait_group(10.0):
                        os.killpg(self.process.pid, signal.SIGKILL)
                        self._wait_group(5.0)
            try:
                return_code = self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                return_code = self.process.poll()
            return return_code
        finally:
            if self.log_stream is not None:
                self.log_stream.close()
            self.process = None
            self.log_stream = None


def load_result(path):
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return None, f"failed to read evaluator result: {error}"
    if not isinstance(payload, dict):
        return None, "evaluator result is not a JSON object"
    return payload, None


def pump_output(stream, log_stream, prefix):
    for line in iter(stream.readline, ""):
        log_stream.write(line)
        log_stream.flush()
        print(f"[{prefix}] {line}", end="", flush=True)
    stream.close()


def run_evaluator(command, log_path, stack, episode_name, state):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log_stream:
        log_stream.write(f"command: {command_string(command)}\n")
        log_stream.flush()
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        state["evaluator"] = process
        pump = threading.Thread(
            target=pump_output,
            args=(process.stdout, log_stream, episode_name),
            daemon=True,
        )
        pump.start()
        stack_exited_early = False
        while process.poll() is None:
            if stack.poll() is not None:
                stack_exited_early = True
                process.send_signal(signal.SIGINT)
                break
            time.sleep(0.5)
        try:
            return_code = process.wait(timeout=45.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                return_code = process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                process.kill()
                return_code = process.wait()
        pump.join(timeout=5.0)
        state["evaluator"] = None
    return return_code, stack_exited_early


def make_output_dir(requested):
    if requested is not None:
        return requested.expanduser().resolve()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    return (ROOT / "artifacts/regressions/route_suites" / stamp).resolve()


def filter_episodes(episodes, requested):
    if not requested:
        return episodes
    requested_set = set(requested)
    known = {episode["name"] for episode in episodes}
    missing = requested_set - known
    if missing:
        raise SuiteError(f"unknown --case values: {', '.join(sorted(missing))}")
    return [episode for episode in episodes if episode["name"] in requested_set]


def print_dry_run(episodes, args, output_dir):
    print(f"suite={args.suite.expanduser().resolve()}")
    print(f"output_dir={output_dir}")
    for index, episode in enumerate(episodes, start=1):
        route_path = output_dir / "routes" / f"{episode['name']}.json"
        result_path = output_dir / "results" / f"{episode['name']}.json"
        print(f"\n[{index}/{len(episodes)}] {episode['name']}")
        prepare_command = build_prepare_command(episode, route_path, args.host, args.port)
        print(f"prepare: {command_string(prepare_command)}")
        print(
            "stack:   "
            + command_string(
                build_stack_command(episode, route_path, args.host, args.port, args.launch_arg)
            )
        )
        print(
            "test:    "
            + command_string(
                build_evaluator_command(episode, route_path, result_path)
            )
        )


def main():
    try:
        args = parse_args()
        suite_path, episodes = load_suite(args.suite)
        args.suite = suite_path
        episodes = filter_episodes(episodes, args.cases)
        output_dir = make_output_dir(args.output_dir)
        if args.dry_run:
            print_dry_run(episodes, args, output_dir)
            return 0
    except SuiteError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    environment = os.environ.copy()
    environment["CARLA_HOST"] = args.host
    environment["CARLA_PORT"] = str(args.port)
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "summary.json"
    resolved_suite_path = output_dir / "suite.resolved.json"
    atomic_write_json(
        resolved_suite_path,
        {
            "schema_version": 1,
            "source_suite": str(suite_path),
            "carla": {"host": args.host, "port": args.port},
            "episodes": episodes,
        },
    )
    summary = {
        "schema_version": 1,
        "success": False,
        "suite": str(suite_path),
        "resolved_suite": str(resolved_suite_path),
        "output_directory": str(output_dir),
        "carla": {"host": args.host, "port": args.port, "managed_by_suite": False},
        "started_at": utc_now(),
        "finished_at": None,
        "interrupted_by": None,
        "counts": {"total": len(episodes), "passed": 0, "failed": 0, "not_run": len(episodes)},
        "episodes": [],
    }
    state = {"stack": None, "evaluator": None, "signal": None}

    def request_stop(signum, _frame):
        state["signal"] = signum
        evaluator = state.get("evaluator")
        if evaluator is not None and evaluator.poll() is None:
            evaluator.send_signal(signum)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    exit_code = 0
    try:
        if not args.skip_stack_preflight:
            conflicts = conflicting_stack_nodes()
            if conflicts:
                raise SuiteError(
                    "an existing CARLA/VAD stack is visible in this ROS domain: "
                    + ", ".join(conflicts)
                )

        for index, episode in enumerate(episodes, start=1):
            if state["signal"] is not None:
                break
            name = episode["name"]
            route_path = output_dir / "routes" / f"{name}.json"
            result_path = output_dir / "results" / f"{name}.json"
            prepare_log_path = output_dir / "logs" / f"{name}.prepare.log"
            stack_log_path = output_dir / "logs" / f"{name}.stack.log"
            evaluator_log_path = output_dir / "logs" / f"{name}.evaluator.log"
            renderer_log_path = output_dir / "logs" / f"{name}.renderer.log"
            image_path = output_dir / "images" / f"{name}.png"
            record = {
                "name": name,
                "town": episode["town"],
                "weather": episode["weather"],
                "scenario": episode["scenario"],
                "started_at": utc_now(),
                "finished_at": None,
                "success": False,
                "reason": "episode did not start",
                "route": str(route_path),
                "result": str(result_path),
                "image": str(image_path),
                "logs": {
                    "prepare": str(prepare_log_path),
                    "stack": str(stack_log_path),
                    "evaluator": str(evaluator_log_path),
                    "renderer": str(renderer_log_path),
                },
            }
            summary["episodes"].append(record)
            route_path.unlink(missing_ok=True)
            result_path.unlink(missing_ok=True)
            print(
                f"\n=== [{index}/{len(episodes)}] {name}: {episode['town']} "
                f"{episode['weather']} {episode['scenario']} ===",
                flush=True,
            )

            if not args.skip_stack_preflight:
                conflicts = conflicting_stack_nodes()
                if conflicts:
                    record["reason"] = (
                        "existing CARLA/VAD stack before route preparation: "
                        + ", ".join(conflicts)
                    )
                    record["finished_at"] = utc_now()
                    summary["counts"]["failed"] += 1
                    summary["counts"]["not_run"] -= 1
                    atomic_write_json(summary_path, summary)
                    if args.fail_fast:
                        break
                    continue

            prepare_command = build_prepare_command(episode, route_path, args.host, args.port)
            prepare_log_path.parent.mkdir(parents=True, exist_ok=True)
            with prepare_log_path.open("w", encoding="utf-8") as prepare_log:
                prepare_log.write(f"command: {command_string(prepare_command)}\n")
                prepare_log.flush()
                prepare_result = subprocess.run(
                    prepare_command,
                    cwd=ROOT,
                    env=environment,
                    stdout=prepare_log,
                    stderr=subprocess.STDOUT,
                    check=False,
                )
            if prepare_result.returncode != 0:
                record["reason"] = (
                    f"route preparation failed with exit code {prepare_result.returncode}"
                )
                record["finished_at"] = utc_now()
                summary["counts"]["failed"] += 1
                summary["counts"]["not_run"] -= 1
                atomic_write_json(summary_path, summary)
                print(f"FAILED: {record['reason']} (log: {prepare_log_path})", flush=True)
                if args.fail_fast:
                    break
                continue

            stack_command = build_stack_command(
                episode, route_path, args.host, args.port, args.launch_arg
            )
            stack = StackProcess(stack_command, stack_log_path, environment, args.shutdown_timeout)
            state["stack"] = stack
            stack.start()
            evaluator_command = build_evaluator_command(
                episode, route_path, result_path
            )
            evaluator_code, stack_exited_early = run_evaluator(
                evaluator_command, evaluator_log_path, stack, name, state
            )
            stack_code = stack.stop()
            state["stack"] = None

            evaluator_result, result_error = load_result(result_path)
            renderer_code = None
            if result_error is None:
                image_path.parent.mkdir(parents=True, exist_ok=True)
                renderer_log_path.parent.mkdir(parents=True, exist_ok=True)
                with renderer_log_path.open("w", encoding="utf-8") as renderer_log:
                    renderer_command = build_renderer_command(
                        route_path, result_path, image_path
                    )
                    renderer_log.write(f"command: {command_string(renderer_command)}\n")
                    renderer_log.flush()
                    renderer_result = subprocess.run(
                        renderer_command,
                        cwd=ROOT,
                        env=environment,
                        stdout=renderer_log,
                        stderr=subprocess.STDOUT,
                        check=False,
                    )
                    renderer_code = renderer_result.returncode
            record["renderer_exit_code"] = renderer_code
            record["visualization_success"] = renderer_code == 0
            if stack_exited_early:
                record["reason"] = "VAD/bridge stack exited before the evaluator completed"
            elif result_error:
                record["reason"] = result_error
            elif evaluator_code != 0 or not evaluator_result.get("success", False):
                record["reason"] = evaluator_result.get(
                    "reason", f"evaluator failed with exit code {evaluator_code}"
                )
            else:
                record["success"] = True
                record["reason"] = evaluator_result.get("reason", "goal reached")
            if evaluator_result is not None:
                record["assessment"] = evaluator_result.get("assessment")
            record["evaluator_exit_code"] = evaluator_code
            record["stack_exit_code"] = stack_code
            record["finished_at"] = utc_now()
            summary["counts"]["not_run"] -= 1
            if record["success"]:
                summary["counts"]["passed"] += 1
                print(f"PASSED: {name}", flush=True)
            else:
                summary["counts"]["failed"] += 1
                print(f"FAILED: {name}: {record['reason']}", flush=True)
                exit_code = 1

            atomic_write_json(summary_path, summary)
            if state["signal"] is not None or (args.fail_fast and not record["success"]):
                break
            if args.episode_gap > 0:
                time.sleep(args.episode_gap)
    except SuiteError as error:
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        summary["error"] = str(error)
        exit_code = 2
    except Exception as error:
        print(f"ERROR: unexpected suite failure: {error}", file=sys.stderr, flush=True)
        summary["error"] = f"unexpected suite failure: {error}"
        exit_code = 2
    finally:
        evaluator = state.get("evaluator")
        if evaluator is not None and evaluator.poll() is None:
            evaluator.send_signal(signal.SIGINT)
            try:
                evaluator.wait(timeout=45.0)
            except subprocess.TimeoutExpired:
                evaluator.terminate()
        stack = state.get("stack")
        if stack is not None:
            stack.stop()
        if state["signal"] is not None:
            summary["interrupted_by"] = signal.Signals(state["signal"]).name
            exit_code = 128 + state["signal"]
        summary["finished_at"] = utc_now()
        summary["success"] = (
            summary["counts"]["passed"] == summary["counts"]["total"]
            and summary["counts"]["failed"] == 0
            and state["signal"] is None
        )
        if not summary["success"] and exit_code == 0:
            exit_code = 1
        atomic_write_json(summary_path, summary)

    print(
        f"\nSuite {'passed' if summary['success'] else 'failed'}: "
        f"{summary['counts']['passed']}/{summary['counts']['total']} passed "
        f"(summary: {summary_path})",
        flush=True,
    )
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
