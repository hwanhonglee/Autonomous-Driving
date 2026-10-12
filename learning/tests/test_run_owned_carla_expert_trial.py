# HH_260906 - Locate the repository independently of the relocated test directory.
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

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
WRAPPER = _hh_source_path(ROOT, "scripts/e2e/run_owned_carla_expert_trial.sh")


@pytest.fixture
def harness(tmp_path):
    # HH_260906 - Replace every simulator and collection entrypoint in a private temporary repository.
    root = tmp_path / "fake-repo"
    scripts = root / "tools/e2e"
    scripts.mkdir(parents=True)
    shutil.copy2(WRAPPER, scripts / WRAPPER.name)
    shutil.copy2(_hh_source_path(ROOT, "scripts/e2e/process_group_cleanup.sh"), scripts / "process_group_cleanup.sh")
    (scripts / "env.sh").write_text("# HH_260906 - No real ROS/CARLA environment is sourced by this fixture.\n")
    (scripts / "workspace_runtime_lock.sh").write_text(
        "# HH_260906 - Simulate lock refusal without touching the real workspace lock.\n"
        "e2e_acquire_workspace_runtime_lock() {\n"
        ' if [[ -n ${E2E_TRIAL_TEST_RACE_OUTPUT:-} ]]; then mkdir -- "$E2E_TRIAL_TEST_RACE_OUTPUT"; printf keep-original > "$E2E_TRIAL_TEST_RACE_OUTPUT/server.log"; fi\n'
        " [[ ${E2E_TRIAL_TEST_LOCKED:-0} == 0 ]]; }\n")
    (scripts / "carla_goal_stop_profile.py").write_text("# HH_260906 - Fixture-only hash placeholder, never vehicle control.\n")
    (scripts / "carla_low_speed_response_matrix.py").write_text("# HH_260906 - Fixture-only matrix source archive.\n")
    # HH_260906 - Archive both bound sources without importing a model or requiring Torch in this ownership fixture.
    (root / "learning/portable_e2e").mkdir(parents=True)
    # HH_260906 - Test the actual canonical entrypoint, without recreating any legacy aliases.
    for name in ('workspace_python.py', 'canonical_workspace.py'):
        shutil.copy2(ROOT/'tools'/name, root/'tools'/name)
    shutil.copy2(ROOT/'learning/portable_e2e/workspace_paths.py', root/'learning/portable_e2e/workspace_paths.py')
    for name in ("model.py", "runtime_contract.py", "mapping.py", "mapping_projection.py"):
        (root / "learning/portable_e2e" / name).write_text("# HH_260906 - Fixture-only scalar bound provenance.\n")
    (scripts / "run_carla_map.sh").write_text(
        "#!/usr/bin/env bash\n# HH_260906 - The fake process does not open a CARLA port.\n"
        'exec python3 tools/e2e/fake_server.py "$@"\n')
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
        " # HH_260906 - A real collector requires package context for sibling imports after canonical relocation.\n"
        " assert __package__ == 'scripts.e2e', 'collector lost its preflight package context'\n"
        " from . import carla_goal_stop_profile\n"
        " a=parse_args()\n"
        " with open(os.environ['E2E_TRIAL_TEST_EVENTS'],'a') as f: f.write(json.dumps({'role':'collector','pid':os.getpid(),'pgid':os.getpgrp(),'host':a.host,'port':a.port})+'\\n')\n"
        " if os.environ.get('E2E_TRIAL_TEST_MUTATE_SOURCE')=='1':\n"
        "  with open(__file__,'a') as f: f.write('\\n# HH_260906 - Fixture-only source mutation.\\n')\n"
        " if a.seed==99: time.sleep(30)\n"
        " sys.exit(7 if a.seed==98 else 0)\n")
    # HH_260906 - The second named worker remains a harmless process in these ownership tests.
    shutil.copy2(scripts / "collect_carla_vad_expert.py", scripts / "calibrate_carla_low_speed_response.py")
    # HH_260906 - The fixed pedal worker uses the same owned module boundary, without a real simulator.
    shutil.copy2(scripts / "collect_carla_vad_expert.py", scripts / "collect_carla_pedal_caps_aj.py")
    # HH_260906 - Exercise the new mapping allowlist without any simulator import.
    shutil.copy2(scripts / "collect_carla_vad_expert.py", scripts / "collect_carla_mapping_probe.py")
    # HH_260906 - The third named worker is also a harmless fake; no camera or vehicle is created by tests.
    shutil.copy2(scripts / "collect_carla_vad_expert.py", scripts / "probe_carla_stationary_camera_quality.py")
    # HH_260906 - Preserve the fourth named worker boundary without importing actual simulator helpers.
    shutil.copy2(scripts / "collect_carla_vad_expert.py", scripts / "probe_carla_stationary_motion.py")
    # HH_260906 - The fifth named worker remains a fake process; coast tests never move an actual vehicle.
    shutil.copy2(scripts / "collect_carla_vad_expert.py", scripts / "probe_carla_low_speed_coast.py")
    # HH_260906 - The sixth named worker only adds a required arm flag to this non-simulator fixture.
    (scripts / "probe_carla_coast_brake_transition.py").write_text(
        (scripts / "collect_carla_vad_expert.py").read_text().replace(
            " p.add_argument('--seed',type=int,default=0)",
            " p.add_argument('--arm',choices=('coast','service'),required=True)\n p.add_argument('--seed',type=int,default=0)"))
    (scripts / "carla_physics_substeps.py").write_text("# HH_260906 - Fixture-only physics readback helper.\n")
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
    for arguments in (("init", "--quiet"), ("add", "tools"),
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


def test_mapping_mode_archives_projection_without_driving_claim(harness):
    result = _run(harness, "--capture-mode", "mapping-probe")
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    assert plan["learned_model_control"] is False
    assert "portable_e2e/mapping_projection.py" in plan["source_sha256"]
    assert plan["worker_path"] == "scripts/e2e/collect_carla_mapping_probe.py"


def test_ctrack_v2_mode_owns_epic_worker_and_archives_split_provenance(harness):
    # HH_260929 - Exercise the new owner selection with fake children, never CARLA.
    scripts = harness["root"] / "tools/e2e"
    (scripts / "collect_carla_mapping_ctrack_v2.py").write_text(
        "import argparse,json,os\n"
        "def parse_args(argv=None):\n"
        " p=argparse.ArgumentParser(allow_abbrev=False)\n"
        " p.add_argument('output');p.add_argument('route_file')\n"
        " p.add_argument('--host',choices=['127.0.0.1']);p.add_argument('--port',type=int)\n"
        " p.add_argument('--motion-mode',choices=['basic-agent'],required=True)\n"
        " p.add_argument('--frames',type=int,choices=[240],required=True)\n"
        " p.add_argument('--ignore-traffic-lights',action='store_true',required=True)\n"
        " p.add_argument('--ctrack-straight',action='store_true',required=True)\n"
        " a=p.parse_args(argv);a.allow_map_load=False;return a\n"
        "if __name__=='__main__':\n"
        " a=parse_args()\n"
        " with open(os.environ['E2E_TRIAL_TEST_EVENTS'],'a') as f:\n"
        "  f.write(json.dumps({'role':'ctrack_fixture_collector','port':a.port})+'\\n')\n")
    (scripts / "site_split_guard.py").write_text("# HH_260929 - Frozen fixture V1.\n")
    (scripts / "site_split_guard_v2.py").write_text("# HH_260929 - Fixture-only V2 scope.\n")
    config = harness["root"] / "learning/config"
    config.mkdir(parents=True)
    (config / "site_split_guard_20260929.json").write_text('{"fixture":"v1"}')
    (config / "site_split_guard_20260929_v2.json").write_text('{"fixture":"v2"}')
    harness["route"].write_text(json.dumps({"town": "C_track_1_0_7"}))
    options = ("--capture-mode", "mapping-ctrack-v2", "--quality", "Epic", "--",
               "--motion-mode", "basic-agent", "--frames", "240",
               "--ignore-traffic-lights", "--ctrack-straight")
    result = _run(harness, *options)
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    final = json.loads((harness["output"] / "owner_result.json").read_text())
    assert plan["capture_mode"] == final["capture_mode"] == "mapping-ctrack-v2"
    assert plan["worker_path"] == "scripts/e2e/collect_carla_mapping_ctrack_v2.py"
    assert plan["quality"] == "Epic" and plan["map"] == "C_track_1_0_7"
    assert plan["training_data_admitted"] is final["training_data_admitted"] is False
    assert plan["independent_dev_approved"] is final["independent_dev_approved"] is False
    for name in ("scripts/e2e/collect_carla_mapping_ctrack_v2.py",
                 "scripts/e2e/collect_carla_mapping_probe.py",
                 "scripts/e2e/site_split_guard_v2.py",
                 "config/site_split_guard_20260929_v2.json",
                 "portable_e2e/mapping_projection.py"):
        assert name in plan["source_sha256"]
        assert final["source_checks"][name] is True
    assert json.loads((harness["output"] / "lifecycle/stopped.json").read_text())["status"] == "PASS"


def test_pedal_mode_archives_original_and_research_worker(harness):
    result = _run(harness, "--capture-mode", "expert-pedal-caps")
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    assert "scripts/e2e/collect_carla_vad_expert.py" in plan["source_sha256"]
    assert "scripts/e2e/collect_carla_pedal_caps_aj.py" in plan["source_sha256"]
    assert plan["learned_model_control"] is False


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
    assert 'moving_task_capture' not in plan
    assert plan["bounds_source_bytes_archived"] is True
    for name, expected in plan["source_sha256"].items():
        assert hashlib.sha256(_hh_source_path(harness["root"], name).read_bytes()).hexdigest() == expected
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


@pytest.mark.parametrize("port_offset", [1, 2])
def test_occupied_auxiliary_port_blocks_before_output_or_children(harness, port_offset):
    # HH_260906 - Reproduce the v6 prelaunch boundary on temporary fixture ports, without retrying a real case.
    port = harness["port"] + port_offset
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", port))
        listener.listen()
        result = _run(harness)
        assert result.returncode != 0
        assert f"CARLA port {port} is already occupied" in result.stderr
        assert listener.getsockname()[1] == port
        assert not harness["output"].exists()
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


def test_stationary_motion_archives_all_helpers_without_becoming_an_expert_episode(harness):
    # HH_260906 - Fixed held-brake measurements must have separate output and complete imported helper provenance.
    result = _run(harness, "--capture-mode", "stationary-motion")
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    assert plan["worker_path"] == "scripts/e2e/probe_carla_stationary_motion.py"
    assert plan["collector_argv"][0] == str(harness["output"] / "motion_probe")
    assert len(plan["source_sha256"]) == 15
    assert plan["learned_model_control"] is plan["vehicle_control_approved"] is False
    for name in ("probe_carla_stationary_camera_quality.py", "calibrate_carla_low_speed_response.py",
                 "carla_low_speed_response_matrix.py", "carla_physics_substeps.py"):
        assert f"scripts/e2e/{name}" in plan["source_sha256"]
    for name, expected in plan["source_sha256"].items():
        assert hashlib.sha256((harness["output"] / "provenance" / name).read_bytes()).hexdigest() == expected
    owner = json.loads((harness["output"] / "owner_result.json").read_text())
    assert owner["source_bytes_unchanged_and_archived"] is True


def test_low_speed_coast_has_exact_closure_and_separate_output(harness):
    # HH_260906 - A low-speed coast measurement cannot silently become an expert or stationary episode.
    result = _run(harness, "--capture-mode", "low-speed-coast")
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    assert plan["worker_path"] == "scripts/e2e/probe_carla_low_speed_coast.py"
    assert plan["collector_argv"][0] == str(harness["output"] / "coast_probe")
    assert plan["capture_mode"] == "low-speed-coast"
    assert len(plan["source_sha256"]) == 16
    assert plan["learned_model_control"] is plan["vehicle_control_approved"] is False
    for name in ("probe_carla_stationary_motion.py", "probe_carla_stationary_camera_quality.py",
                 "calibrate_carla_low_speed_response.py", "carla_low_speed_response_matrix.py",
                 "carla_physics_substeps.py"):
        assert f"scripts/e2e/{name}" in plan["source_sha256"]
    for name, expected in plan["source_sha256"].items():
        assert hashlib.sha256((harness["output"] / "provenance" / name).read_bytes()).hexdigest() == expected
    owner = json.loads((harness["output"] / "owner_result.json").read_text())
    assert owner["exit_code"] == 0 and owner["source_bytes_unchanged_and_archived"] is True


@pytest.mark.parametrize("options", [["--host", "127.0.0.2"], ["--wall-timing"]])
def test_coast_rejects_redirection_and_expert_timing_before_launch(harness, options):
    # HH_260906 - The new mode inherits the same no-takeover contract and cannot borrow expert-only flags.
    result = _run(harness, "--capture-mode", "low-speed-coast", "--", *options)
    assert result.returncode != 0
    assert not harness["events"].exists() and not harness["output"].exists()


def test_brake_transition_has_complete_source_closure_and_explicit_arm(harness):
    # HH_260906 - A brake-transition diagnostic is neither an old coast run nor an expert training episode.
    result = _run(harness, "--capture-mode", "coast-brake-transition", "--", "--arm", "service")
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    assert plan["capture_mode"] == "coast-brake-transition"
    assert plan["worker_path"] == "scripts/e2e/probe_carla_coast_brake_transition.py"
    assert plan["collector_argv"][0] == str(harness["output"] / "brake_probe")
    assert plan["collector_argv"][-2:] == ["--arm", "service"]
    assert len(plan["source_sha256"]) == 17
    assert all("scripts/e2e/" + name in plan["source_sha256"] for name in (
        "probe_carla_low_speed_coast.py", "probe_carla_stationary_motion.py", "carla_physics_substeps.py"))
    for name, expected in plan["source_sha256"].items():
        assert hashlib.sha256((harness["output"] / "provenance" / name).read_bytes()).hexdigest() == expected
    owner = json.loads((harness["output"] / "owner_result.json").read_text())
    assert owner["exit_code"] == 0 and owner["source_bytes_unchanged_and_archived"]
    assert owner["learned_model_control"] is owner["vehicle_control_approved"] is False


@pytest.mark.parametrize("options", [[], ["--arm", "parking"], ["--arm", "coast", "--host", "remote"]])
def test_brake_transition_argv_denial_precedes_server_start(harness, options):
    # HH_260906 - Missing/unknown arms and endpoint redirection must not spawn either child.
    result = _run(harness, "--capture-mode", "coast-brake-transition", "--", *options)
    assert result.returncode != 0
    assert not harness["events"].exists() and not harness["output"].exists()


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
        assert 'moving_task_capture' not in owner
        stopped = json.loads((harness["output"] / "lifecycle/stopped.json").read_text())
        assert stopped["status"] == "PASS" and stopped["fixture_only"] is True
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=5)


def moving_harness(harness):
    """HH_260906 - Add only the new opt-in ABI to a fake collector, not to legacy fixture modes."""
    from datetime import datetime, timezone
    scripts = harness['root'] / 'tools/e2e'
    helper = scripts / 'carla_moving_capture.py'
    helper.write_text(
        "# HH_260906 - Fixture-only plan binding; no simulator or task controller.\n"
        "import hashlib,json\nfrom pathlib import Path\nfrom learning.portable_e2e.workspace_paths import source_path\n"
        "def configuration_from_args(a):\n"
        " if a.moving_task_plan is None or a.moving_task_plan_sha256 is None: raise ValueError('paired moving flags required')\n"
        " p=Path(a.moving_task_plan)\n"
        " if p.is_symlink() or not p.is_file(): raise ValueError('regular moving plan required')\n"
        " raw=p.read_bytes()\n"
        " if hashlib.sha256(raw).hexdigest()!=a.moving_task_plan_sha256: raise ValueError('moving plan SHA differs')\n"
        " d=json.loads(raw)\n"
        " if d['schema']!='carla.moving_task_capture_plan.v1' or d['output']!=a.output: raise ValueError('moving plan scope differs')\n"
        " for name,sha in d['source_sha256'].items():\n"
        "  if hashlib.sha256(source_path(Path.cwd(),name).read_bytes()).hexdigest()!=sha: raise ValueError('moving source changed')\n"
        " for name,sha in d['input_sha256'].items():\n"
        "  if hashlib.sha256(Path(d['input_paths'][name]).read_bytes()).hexdigest()!=sha: raise ValueError('moving input changed')\n"
        " return d\n")
    worker = scripts / 'collect_carla_vad_expert.py'
    text = worker.read_text().replace(" return p.parse_args(argv)",
        " p.add_argument('--moving-task-plan');p.add_argument('--moving-task-plan-sha256')\n"
        " a=p.parse_args(argv)\n"
        " if a.moving_task_plan is not None or a.moving_task_plan_sha256 is not None: moving_capture_module(a).configuration_from_args(a)\n"
        " return a\n"
        "def moving_capture_module(a):\n"
        " try: from scripts.e2e import carla_moving_capture as module\n"
        " except ModuleNotFoundError: import carla_moving_capture as module\n"
        " return module")
    text = text.replace(" if a.seed==99: time.sleep(30)",
        " if a.moving_task_plan is not None:\n"
        "  from pathlib import Path\n"
        "  Path(a.output).mkdir()\n"
        "  payload=json.loads(os.environ['E2E_TRIAL_TEST_MOVING_MANIFEST'])\n"
        "  (Path(a.output)/'manifest.json').write_text(json.dumps(payload))\n"
        "  if os.environ.get('E2E_TRIAL_TEST_MUTATE_MOVING_PLAN')=='1':\n"
        "   with open(a.moving_task_plan,'a') as f: f.write(' ')\n"
        "  if os.environ.get('E2E_TRIAL_TEST_MUTATE_MOVING_SOURCE')=='1':\n"
        "   with open('tools/e2e/carla_moving_capture.py','a') as f: f.write('# HH_260906 - Fixture-only mutation.\\n')\n"
        " if a.seed==99: time.sleep(30)")
    worker.write_text(text)
    mapping, calibration = harness['root']/'mapping.yaml', harness['root']/'calibration.yaml'
    mapping.write_text('fixture_mapping: true\n'); calibration.write_text('fixture_calibration: true\n')
    names = ('scripts/e2e/collect_carla_vad_expert.py', 'scripts/e2e/carla_moving_capture.py',
             'scripts/e2e/carla_goal_stop_profile.py', 'scripts/e2e/carla_physics_substeps.py')
    inputs = dict(route_file=str(harness['route']), mapping=str(mapping), calibration=str(calibration))
    plan = dict(schema='carla.moving_task_capture_plan.v1', declared_at_utc=datetime.now(timezone.utc).isoformat(), owner_wall_timeout_sec=300,
        output=str(harness['output']/'episode'), source_sha256={name:hashlib.sha256(_hh_source_path(harness['root'],name).read_bytes()).hexdigest() for name in names},
        input_paths=inputs, input_sha256={key:hashlib.sha256(Path(value).read_bytes()).hexdigest() for key,value in inputs.items()})
    plan_path = harness['root']/'moving-plan.json'
    plan_path.write_text(json.dumps(plan)); digest=hashlib.sha256(plan_path.read_bytes()).hexdigest()
    manifest=dict(status='complete', cleanup=dict(completed=True), result=dict(task_complete=True, goal_reached=False,
        admission_status='PENDING_QA', dataset_conversion_performed=False))
    harness['env']['E2E_TRIAL_TEST_MOVING_MANIFEST']=json.dumps(manifest)
    return plan_path,digest,manifest


def moving_run(harness,*options):
    return _run(harness,'--quality','Epic','--wall-timeout-sec','300',*options)


@pytest.mark.parametrize('equals',[False,True])
def test_moving_opt_in_archives_plan_all_inputs_and_two_helpers_only(harness,equals):
    path,digest,_=moving_harness(harness)
    options=([f'--moving-task-plan={path}',f'--moving-task-plan-sha256={digest}'] if equals else
             ['--moving-task-plan',str(path),'--moving-task-plan-sha256',digest])
    result=moving_run(harness,'--',*options)
    assert result.returncode==0,result.stderr
    plan=json.loads((harness['output']/'owner_plan.json').read_text())
    assert len(plan['source_sha256'])==12
    assert 'physics_substeps_schema' not in plan
    assert 'scripts/e2e/carla_moving_capture.py' in plan['source_sha256']
    task=plan['moving_task_capture']
    assert task['plan']['sha256']==digest
    assert set(task['inputs'])=={'route_file','mapping','calibration'}
    for ref in [task['plan'],*task['inputs'].values()]:
        assert Path(ref['path']).read_bytes()==(harness['output']/ref['archived_path']).read_bytes()
    owner=json.loads((harness['output']/'owner_result.json').read_text())
    assert owner['exit_code']==0 and owner['source_bytes_unchanged_and_archived']
    assert owner['moving_task_capture']['capture_complete_pending_qa'] is True
    assert owner['moving_task_capture']['source_plan_and_inputs_unchanged_and_archived'] is True
    assert owner['moving_task_capture']['dataset_admission_granted'] is False


@pytest.mark.parametrize('failure',['missing_pin','missing_plan','wrong_pin','wrong_mode','future_declaration','source_change'])
def test_moving_declaration_failure_never_launches_a_child(harness,failure):
    path,digest,_=moving_harness(harness)
    if failure=='source_change':
        with (harness['root']/'tools/e2e/carla_moving_capture.py').open('a') as stream:stream.write('# changed\n')
    if failure=='future_declaration':
        plan=json.loads(path.read_text());plan['declared_at_utc']='2099-01-01T00:00:00Z';path.write_text(json.dumps(plan))
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
    options=[] if failure=='missing_plan' else ['--moving-task-plan',str(path)]
    if failure!='missing_pin':options+=['--moving-task-plan-sha256','0'*64 if failure=='wrong_pin' else digest]
    result=moving_run(harness,*(['--capture-mode','stationary-motion'] if failure=='wrong_mode' else []),'--',*options)
    assert result.returncode!=0
    assert not harness['events'].exists()


@pytest.mark.parametrize('field,value',[('task_complete',False),('task_complete',1),('goal_reached',True),
    ('admission_status','APPROVED'),('dataset_conversion_performed',True),('training_data_approved',False),
    ('training_data_approved',True),('development_only',True),('cleanup',False),('manifest',[])])
def test_moving_child_exit_zero_does_not_replace_pending_candidate_receipt(harness,field,value):
    path,digest,manifest=moving_harness(harness)
    if field=='manifest':manifest=value
    elif field=='cleanup':manifest['cleanup']['completed']=value
    else:manifest['result'][field]=value
    harness['env']['E2E_TRIAL_TEST_MOVING_MANIFEST']=json.dumps(manifest)
    result=moving_run(harness,'--','--moving-task-plan',str(path),'--moving-task-plan-sha256',digest)
    assert result.returncode==1,result.stderr
    owner=json.loads((harness['output']/'owner_result.json').read_text())
    assert owner['exit_code']==1 and owner['moving_task_capture']['capture_complete_pending_qa'] is False
    assert json.loads((harness['output']/'lifecycle/stopped.json').read_text())['status']=='PASS'


@pytest.mark.parametrize('target',['PLAN','SOURCE'])
def test_moving_runtime_tamper_is_preserved_and_rejected(harness,target):
    path,digest,_=moving_harness(harness)
    original=path.read_bytes()
    harness['env']['E2E_TRIAL_TEST_MUTATE_MOVING_'+target]='1'
    result=moving_run(harness,'--','--moving-task-plan',str(path),'--moving-task-plan-sha256',digest)
    assert result.returncode==1,result.stderr
    owner=json.loads((harness['output']/'owner_result.json').read_text())
    assert owner['exit_code']==1
    assert (harness['output']/'provenance/moving_task_plan.json').read_bytes()==original
    assert json.loads((harness['output']/'lifecycle/stopped.json').read_text())['status']=='PASS'


def test_moving_plan_changed_during_startup_blocks_collector_but_cleans_owned_server(harness):
    path,digest,_=moving_harness(harness)
    server=harness['root']/'tools/e2e/fake_server.py'
    server.write_text(server.read_text().replace("print('CARLA_READY simulated_only=true',flush=True)",
        f"with open({str(path)!r},'a') as f: f.write(' ')\nprint('CARLA_READY simulated_only=true',flush=True)"))
    result=moving_run(harness,'--','--moving-task-plan',str(path),'--moving-task-plan-sha256',digest)
    assert result.returncode==1,result.stderr
    events=[json.loads(row) for row in harness['events'].read_text().splitlines()]
    assert [row['role'] for row in events]==['server']
    assert json.loads((harness['output']/'lifecycle/stopped.json').read_text())['status']=='PASS'


@pytest.mark.parametrize('owner_options',[['--quality','Low'],['--wall-timeout-sec','900']])
def test_moving_owner_quality_and_budget_cannot_drift_from_declared_protocol(harness,owner_options):
    path,digest,_=moving_harness(harness)
    result=moving_run(harness,*owner_options,'--','--moving-task-plan',str(path),'--moving-task-plan-sha256',digest)
    assert result.returncode==2,result.stderr
    assert not harness['events'].exists() and not harness['output'].exists()
