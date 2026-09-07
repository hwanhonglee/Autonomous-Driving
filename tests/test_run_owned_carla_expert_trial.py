"""HH_260906 - Exercise owned-wrapper boundaries with fake children, never CARLA or a GPU."""

from __future__ import annotations

import json
import hashlib
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / "scripts/e2e/run_owned_carla_expert_trial.sh"


@pytest.fixture
def harness(tmp_path):
    # HH_260906 - Replace every simulator and collection entrypoint in a private temporary repository.
    root = tmp_path / "fake-repo"
    scripts = root / "scripts/e2e"
    scripts.mkdir(parents=True)
    shutil.copy2(WRAPPER, scripts / WRAPPER.name)
    shutil.copy2(ROOT / "scripts/e2e/process_group_cleanup.sh", scripts / "process_group_cleanup.sh")
    (scripts / "env.sh").write_text("# HH_260906 - No real ROS/CARLA environment is sourced by this fixture.\n")
    (scripts / "workspace_runtime_lock.sh").write_text(
        "# HH_260906 - Simulate lock refusal without touching the real workspace lock.\n"
        "e2e_acquire_workspace_runtime_lock() {\n"
        ' if [[ -n ${E2E_TRIAL_TEST_RACE_OUTPUT:-} ]]; then mkdir -- "$E2E_TRIAL_TEST_RACE_OUTPUT"; printf keep-original > "$E2E_TRIAL_TEST_RACE_OUTPUT/server.log"; fi\n'
        " [[ ${E2E_TRIAL_TEST_LOCKED:-0} == 0 ]]; }\n")
    (scripts / "carla_goal_stop_profile.py").write_text("# HH_260906 - Fixture-only hash placeholder, never vehicle control.\n")
    (scripts / "carla_low_speed_response_matrix.py").write_text("# HH_260906 - Fixture-only matrix source archive.\n")
    # HH_260906 - Archive both bound sources without importing a model or requiring Torch in this ownership fixture.
    (root / "portable_e2e").mkdir()
    for name in ("model.py", "runtime_contract.py"):
        (root / "portable_e2e" / name).write_text("# HH_260906 - Fixture-only scalar bound provenance.\n")
    (scripts / "run_carla_map.sh").write_text(
        "#!/usr/bin/env bash\n# HH_260906 - The fake process does not open a CARLA port.\n"
        'exec python3 scripts/e2e/fake_server.py "$@"\n')
    (scripts / "fake_server.py").write_text(
        "# HH_260906 - Record only this harmless child and exit gracefully on owner signals.\n"
        "import json,os,signal,time\n"
        "with open(os.environ['E2E_TRIAL_TEST_EVENTS'],'a') as f: f.write(json.dumps({'role':'server','pid':os.getpid(),'pgid':os.getpgrp()})+'\\n')\n"
        "if os.environ.get('E2E_TRIAL_TEST_FAIL_START')=='1': raise SystemExit(3)\n"
        "signal.signal(signal.SIGINT,lambda *_: exit(0))\n"
        "signal.signal(signal.SIGTERM,lambda *_: exit(0))\n"
        "print('CARLA_READY simulated_only=true',flush=True)\n"
        "while True: time.sleep(.1)\n")
    (scripts / "collect_carla_vad_expert.py").write_text(
        "# HH_260906 - Match real argparse abbreviation behavior without importing a simulator.\n"
        "import argparse,json,os,sys,time\n"
        "def parse_args(argv=None):\n"
        " p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('route_file')\n"
        " p.add_argument('--host');p.add_argument('--port',type=int);p.add_argument('--allow-map-load',action='store_true')\n"
        " p.add_argument('--seed',type=int,default=0);p.add_argument('--max-duration-sec',type=float,default=1)\n"
        " return p.parse_args(argv)\n"
        "if __name__=='__main__':\n"
        " a=parse_args()\n"
        " with open(os.environ['E2E_TRIAL_TEST_EVENTS'],'a') as f: f.write(json.dumps({'role':'collector','pid':os.getpid(),'pgid':os.getpgrp(),'host':a.host,'port':a.port})+'\\n')\n"
        " if os.environ.get('E2E_TRIAL_TEST_MUTATE_SOURCE')=='1':\n"
        "  with open(__file__,'a') as f: f.write('\\n# HH_260906 - Fixture-only source mutation.\\n')\n"
        " if a.seed==99: time.sleep(30)\n"
        " sys.exit(7 if a.seed==98 else 0)\n")
    # HH_260906 - The second named worker remains a harmless process in these ownership tests.
    shutil.copy2(scripts / "collect_carla_vad_expert.py", scripts / "calibrate_carla_low_speed_response.py")
    # HH_260906 - The third named worker is also a harmless fake; no camera or vehicle is created by tests.
    shutil.copy2(scripts / "collect_carla_vad_expert.py", scripts / "probe_carla_stationary_camera_quality.py")
    (scripts / "probe_carla_server.py").write_text(
        "# HH_260906 - Verify fixture process ownership only; no socket/RPC probe is performed.\n"
        "import argparse,json,os\n"
        "from pathlib import Path\n"
        "p=argparse.ArgumentParser();p.add_argument('--output');p.add_argument('--owner-pid',type=int)\n"
        "p.add_argument('--owner-pgid',type=int);p.add_argument('--expect-stopped',action='store_true')\n"
        "a,_=p.parse_known_args()\n"
        "try: pgid=os.getpgid(a.owner_pid)\n"
        "except ProcessLookupError: pgid=None\n"
        "ok=pgid is None if a.expect_stopped else pgid==a.owner_pgid\n"
        "Path(a.output).write_text(json.dumps({'status':'PASS' if ok else 'FAIL','fixture_only':True,'owner_pgid':pgid}))\n"
        "raise SystemExit(0 if ok else 1)\n")
    route = tmp_path / "route.json"
    route.write_text(json.dumps({"town": "Town07"}))
    events = tmp_path / "events.jsonl"
    environment = dict(os.environ, E2E_TRIAL_TEST_EVENTS=str(events))
    environment.pop("AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD", None)
    # HH_260906 - Avoid the real simulator's usual port even though fixture children never bind a server.
    port = None
    for _ in range(20):
        sockets = []
        try:
            first = socket.socket()
            sockets.append(first)
            first.bind(("127.0.0.1", 0))
            candidate = first.getsockname()[1]
            if candidate > 65533:
                continue
            for following in (candidate + 1, candidate + 2):
                probe = socket.socket()
                sockets.append(probe)
                probe.bind(("127.0.0.1", following))
            port = candidate
            break
        except OSError:
            continue
        finally:
            for probe in sockets:
                probe.close()
    assert port is not None, "fixture could not find three available temporary ports"
    # HH_260906 - Create only a temporary local commit so real provenance collection has a valid HEAD.
    for arguments in (("init", "--quiet"), ("add", "scripts"),
                      ("-c", "user.name=Fixture", "-c", "user.email=fixture@example.invalid",
                       "-c", "core.hooksPath=/dev/null", "-c", "commit.gpgsign=false",
                       "commit", "--quiet", "-m", "Fixture-only owned wrapper sources")):
        subprocess.run(["git", *arguments], cwd=root, check=True, capture_output=True, env=environment)
    return {"root": root, "runner": scripts / WRAPPER.name, "route": route,
            "output": tmp_path / "trial", "events": events, "env": environment, "port": port}


def _run(harness, *options):
    command = ["bash", str(harness["runner"]), str(harness["output"]), str(harness["route"]),
               "--port", str(harness["port"]), *options]
    child = subprocess.Popen(command, cwd=harness["root"], env=harness["env"],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
    try:
        stdout, stderr = child.communicate(timeout=15)
    except subprocess.TimeoutExpired:
        # HH_260906 - Signal only this fixture-owned session if a regression leaves the test waiting.
        os.killpg(child.pid, signal.SIGTERM)
        child.communicate(timeout=35)
        raise
    return subprocess.CompletedProcess(command, child.returncode, stdout, stderr)


@pytest.mark.parametrize("options", [
    ["--", "--host", "127.0.0.2"], ["--", "--host=127.0.0.2"],
    ["--", "--ho", "127.0.0.2"], ["--", "--hos=127.0.0.2"],
    ["--", "--port", "2999"], ["--", "--po", "2999"], ["--", "--por=2999"],
    ["--", "--allow-map-load"], ["--", "--allow-m"],
])
def test_endpoint_and_world_override_spellings_fail_before_launch(harness, options):
    result = _run(harness, *options)
    assert result.returncode != 0, result.stderr
    assert not harness["events"].exists(), "protected argv reached a server or collector child"
    assert not harness["output"].exists()


@pytest.mark.parametrize("flag", ["--help", "-h"])
def test_forwarded_help_never_launches_a_world(harness, flag):
    # HH_260906 - argparse normally exits zero for help; that is not a successful capture preflight.
    assert _run(harness, "--", flag).returncode == 0
    assert not harness["events"].exists()
    assert not harness["output"].exists()


@pytest.mark.parametrize("options", [
    ["--port"], ["--quality"], ["--wall-timeout-sec"], ["--port", "0"],
    ["--port", "65534"], ["--wall-timeout-sec", "0"], ["--wall-timeout-sec", "3601"],
    ["--quality", "Medium"],
    ["--capture-mode"], ["--capture-mode", "arbitrary-command"],
    ["--finish-before-utc"],
])
def test_invalid_owner_options_fail_before_launch(harness, options):
    result = _run(harness, *options)
    assert result.returncode == 2, result.stderr
    assert not harness["events"].exists()
    assert not harness["output"].exists()


def test_known_c_track_low_crash_is_rejected_before_launch(harness):
    harness["route"].write_text(json.dumps({"town": "C_track_1_0_7"}))
    result = _run(harness)
    assert result.returncode == 2
    assert "requires Epic quality" in result.stderr
    assert not harness["events"].exists()


@pytest.mark.parametrize("deadline", ["", "2026-09-08", "2026-09-08T10:00:00+09:00",
                                     "2026-13-08T01:00:00Z", "2000-01-01T00:00:00Z"])
def test_invalid_or_expired_finish_boundary_never_starts_a_child(harness, deadline):
    # HH_260906 - Empty explicit values and ambiguous/non-UTC dates must not disable the deadline guard.
    result = _run(harness, "--finish-before-utc", deadline)
    assert result.returncode == 2
    assert not harness["events"].exists()
    assert not harness["output"].exists()


def test_finish_budget_reserves_startup_and_cleanup_before_capture(harness):
    # HH_260906 - A future instant is insufficient unless it covers the whole declared admission budget.
    from datetime import datetime, timedelta, timezone
    deadline = (datetime.now(timezone.utc) + timedelta(seconds=120)).strftime("%Y-%m-%dT%H:%M:%SZ")
    result = _run(harness, "--wall-timeout-sec", "1", "--finish-before-utc", deadline)
    assert result.returncode == 2
    assert "Insufficient time" in result.stderr
    assert not harness["events"].exists()


def test_valid_finish_boundary_is_recorded_without_changing_owned_scope(harness):
    # HH_260906 - Use a distant valid date so clock drift cannot make this process-scope test flaky.
    result = _run(harness, "--finish-before-utc", "2099-01-01T00:00:00Z")
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    assert plan["finish_before_utc"] == "2099-01-01T00:00:00Z"
    assert plan["finish_budget_policy"]["prelaunch_overhead_sec"] == 330
    assert plan["finish_budget_policy"]["precapture_overhead_sec"] == 120
    assert json.loads((harness["output"] / "lifecycle/stopped.json").read_text())["status"] == "PASS"


def test_existing_output_is_not_modified(harness):
    harness["output"].mkdir()
    sentinel = harness["output"] / "server.log"
    sentinel.write_text("keep original bytes")
    assert _run(harness).returncode == 2
    assert sentinel.read_text() == "keep original bytes"
    assert not harness["events"].exists()


def test_atomic_output_claim_does_not_truncate_a_directory_created_after_preflight(harness):
    harness["env"]["E2E_TRIAL_TEST_RACE_OUTPUT"] = str(harness["output"])
    assert _run(harness).returncode != 0
    assert (harness["output"] / "server.log").read_text() == "keep-original"
    assert not harness["events"].exists()


def test_startup_failure_retains_prelaunch_contract_and_stopped_owner_evidence(harness):
    harness["env"]["E2E_TRIAL_TEST_FAIL_START"] = "1"
    assert _run(harness).returncode != 0
    assert not (harness["output"] / "owner_started.json").exists()
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    assert plan["schema"] == "portable_e2e.owned_expert_trial.v1"
    assert len(plan["source_head_commit"]) == 40
    assert len(plan["source_sha256"]) == 10
    assert plan["bounds_source_bytes_archived"] is True
    for name, expected in plan["source_sha256"].items():
        assert hashlib.sha256((harness["root"] / name).read_bytes()).hexdigest() == expected
        assert hashlib.sha256((harness["output"] / "provenance" / name).read_bytes()).hexdigest() == expected
    stopped = json.loads((harness["output"] / "lifecycle/stopped.json").read_text())
    assert stopped["status"] == "PASS"


def test_workspace_lock_refusal_does_not_launch_or_create_output(harness):
    harness["env"]["E2E_TRIAL_TEST_LOCKED"] = "1"
    assert _run(harness).returncode != 0
    assert not harness["events"].exists()
    assert not harness["output"].exists()


def test_occupied_local_port_is_not_taken_over(harness):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        port = listener.getsockname()[1]
        result = _run(harness, "--port", str(port))
        assert result.returncode != 0
        assert "already occupied" in result.stderr
        assert listener.getsockname()[1] == port
    assert not harness["events"].exists()


def test_named_actuation_worker_has_separate_output_and_exact_source_provenance(harness):
    # HH_260906 - Direct pedal measurement is distinct from a six-camera expert episode.
    result = _run(harness, "--capture-mode", "actuation-response")
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    assert plan["capture_mode"] == "actuation-response"
    assert plan["worker_path"] == "scripts/e2e/calibrate_carla_low_speed_response.py"
    assert plan["collector_argv"][0] == str(harness["output"] / "actuation")
    assert len(plan["source_sha256"]) == 12
    assert plan["bounds_source_bytes_archived"] is True
    assert plan["learned_model_control"] is False
    result = json.loads((harness["output"] / "owner_result.json").read_text())
    assert result["capture_mode"] == "actuation-response"
    assert result["source_bytes_unchanged_and_archived"] is True


def test_named_stationary_camera_worker_has_an_independent_output_and_no_actuation_matrix(harness):
    # HH_260906 - A visual-only probe must not be misidentified as a training episode or pedal response matrix.
    result = _run(harness, "--capture-mode", "stationary-camera")
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    assert plan["worker_path"] == "scripts/e2e/probe_carla_stationary_camera_quality.py"
    assert plan["collector_argv"][0] == str(harness["output"] / "camera_audit")
    assert len(plan["source_sha256"]) == 11
    assert "scripts/e2e/carla_low_speed_response_matrix.py" not in plan["source_sha256"]
    assert plan["learned_model_control"] is False


def test_source_change_during_capture_is_retained_and_rejected(harness):
    # HH_260906 - A successful child is insufficient if its source contract changes during execution.
    harness["env"]["E2E_TRIAL_TEST_MUTATE_SOURCE"] = "1"
    result = _run(harness)
    assert result.returncode == 1, result.stderr
    proof = json.loads((harness["output"] / "owner_result.json").read_text())
    assert proof["source_bytes_unchanged_and_archived"] is False
    assert proof["source_checks"]["scripts/e2e/collect_carla_vad_expert.py"] is False
    assert json.loads((harness["output"] / "lifecycle/stopped.json").read_text())["status"] == "PASS"


@pytest.mark.parametrize("seed,expected_status", [(0, 0), (98, 7), (99, 124)])
def test_success_failure_and_timeout_clean_only_fixture_owned_groups(harness, seed, expected_status):
    # HH_260906 - An unrelated child must survive every cleanup path, including collector timeout.
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True)
    try:
        result = _run(harness, "--wall-timeout-sec", "1", "--", "--seed", str(seed))
        assert result.returncode == expected_status, result.stderr
        assert unrelated.poll() is None
        events = [json.loads(line) for line in harness["events"].read_text().splitlines()]
        assert {event["role"] for event in events} == {"server", "collector"}
        for event in events:
            with pytest.raises(ProcessLookupError):
                os.kill(event["pid"], 0)
        owner = json.loads((harness["output"] / "owner_result.json").read_text())
        assert owner["exit_code"] == expected_status
        assert owner["learned_model_control"] is False
        stopped = json.loads((harness["output"] / "lifecycle/stopped.json").read_text())
        assert stopped["status"] == "PASS" and stopped["fixture_only"] is True
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=5)
