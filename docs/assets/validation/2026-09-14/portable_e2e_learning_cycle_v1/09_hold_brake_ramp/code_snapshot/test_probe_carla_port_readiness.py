"""HH_260906 - Exercise local readiness on temporary ports and a fake clock, never launch CARLA."""
from pathlib import Path
import hashlib
import json
import socket
import subprocess
from types import SimpleNamespace

import pytest

from scripts.e2e import probe_carla_port_readiness as probe


class Clock:
    def __init__(self): self.now, self.sleeps = 0., []
    def monotonic(self): return self.now
    def sleep(self, seconds):
        assert 0 < seconds <= 2
        self.sleeps.append(seconds)
        self.now += seconds


@pytest.fixture
def fake_clock(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(probe, "monotonic", clock.monotonic)
    monkeypatch.setattr(probe, "sleep", clock.sleep)
    return clock


@pytest.fixture
def temporary_ports():
    # HH_260906 - Allocate an explicit test-only high port triplet; no busy production port is changed.
    for _ in range(30):
        with socket.socket() as seed:
            seed.bind(("127.0.0.1", 0)); port = seed.getsockname()[1]
        if port <= 65533 and all(row["status"] == "CLEAR" for row in probe.bind_probe(port)):
            return port
    pytest.fail("could not find a clear temporary port triplet")


def bindings(clear):
    return [dict(host=probe.HOST, port=25000+i//2, protocol="TCP" if i%2==0 else "UDP",
                 status="CLEAR" if clear else "BLOCKED", **({} if clear else {"errno":98,"error":"test busy"})) for i in range(6)]


def test_real_six_bindings_are_released(temporary_ports):
    first = probe.bind_probe(temporary_ports)
    assert len(first)==6 and all(row["status"]=="CLEAR" for row in first)
    assert probe.bind_probe(temporary_ports)==first


@pytest.mark.parametrize("offset", [0,1,2])
@pytest.mark.parametrize("kind,protocol", [(socket.SOCK_STREAM,"TCP"),(socket.SOCK_DGRAM,"UDP")])
def test_temporary_port_conflict_is_not_reused_or_moved(temporary_ports, offset, kind, protocol):
    with socket.socket(socket.AF_INET, kind) as owner:
        owner.bind((probe.HOST,temporary_ports+offset))
        rows=probe.bind_probe(temporary_ports)
        blocked=[r for r in rows if r["status"]=="BLOCKED"]
        assert len(blocked)==1 and blocked[0]["port"]==temporary_ports+offset and blocked[0]["protocol"]==protocol
        assert blocked[0]["errno"] and owner.getsockname()[1]==temporary_ports+offset
    assert all(row["status"]=="CLEAR" for row in probe.bind_probe(temporary_ports))


def test_clear_three_observations_are_spaced_and_journaled(tmp_path, fake_clock, monkeypatch):
    monkeypatch.setattr(probe,"bind_probe",lambda _:bindings(True))
    monkeypatch.setattr(probe,"ss_snapshot",lambda *args:pytest.fail("ss called for clear sample"))
    output=tmp_path/"fresh.json"
    report=probe.run(25000,output)
    assert report["status"]=="READY_OBSERVED_NOT_RESERVED" and report["elapsed_seconds"]==4
    assert [s["elapsed_seconds"] for s in report["samples"]]==[0,2,4]
    assert [s["consecutive_clear_checks"] for s in report["samples"]]==[1,2,3]
    assert not report["wrapper_invoked"] and not report["port_reserved_after_probe"] and not report["automatic_trial_retry"]
    journal=output.with_name(output.name+".samples.jsonl")
    assert hashlib.sha256(journal.read_bytes()).hexdigest()==report["sample_journal_sha256"]
    records=[json.loads(line) for line in journal.read_text().splitlines()]
    assert [r["kind"] for r in records]==["started","bind_sample","bind_sample","bind_sample","finished"]
    assert json.loads(output.read_text())==report


def test_busy_observation_resets_consecutive_clear_count(tmp_path,fake_clock,monkeypatch):
    sequence=iter([True,True,False,True,True,True])
    monkeypatch.setattr(probe,"bind_probe",lambda _:bindings(next(sequence)))
    calls=[]
    def snapshot(port,protocol,deadline):
        calls.append((port,protocol,deadline));return dict(protocol=protocol,status="COMPLETE",output="busy socket")
    monkeypatch.setattr(probe,"ss_snapshot",snapshot)
    report=probe.run(25000,tmp_path/"report.json")
    assert report["elapsed_seconds"]==10 and [s["consecutive_clear_checks"] for s in report["samples"]]==[1,2,0,1,2,3]
    assert [c[1] for c in calls]==["TCP","UDP"] and len(report["samples"][2]["ss"])==2


def test_deadline_retains_every_failure_without_trial_retry(tmp_path,fake_clock,monkeypatch):
    monkeypatch.setattr(probe,"bind_probe",lambda _:bindings(False))
    monkeypatch.setattr(probe,"ss_snapshot",lambda p,kind,d:dict(protocol=kind,status="COMPLETE",output="busy"))
    report=probe.run(25000,tmp_path/"report.json")
    assert not report["ready_observed"] and report["elapsed_seconds"]==60
    assert len(report["samples"])==30 and all(len(s["ss"])==2 for s in report["samples"])
    assert all(s["bindings"][0]["error"]=="test busy" for s in report["samples"])


def test_diagnostic_time_consumes_same_deadline(tmp_path,fake_clock,monkeypatch):
    monkeypatch.setattr(probe,"bind_probe",lambda _:bindings(False))
    def snapshot(port,protocol,deadline):
        fake_clock.now+=min(2,deadline-fake_clock.now)
        return dict(protocol=protocol,status="TIMEOUT",output="")
    monkeypatch.setattr(probe,"ss_snapshot",snapshot)
    report=probe.run(25000,tmp_path/"report.json")
    assert report["elapsed_seconds"]==60 and len(report["samples"])==15
    assert not report["ready_observed"]


@pytest.mark.parametrize("port", [True,1023,65534,2100.0,"2100"])
def test_invalid_port_refused_before_output(port,tmp_path):
    with pytest.raises(ValueError):probe.run(port,tmp_path/"report.json")
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("target", ["report.json","report.json.samples.jsonl"])
def test_fresh_output_and_journal_cannot_be_overwritten(target,tmp_path,monkeypatch):
    existing=tmp_path/target;existing.write_text("original")
    monkeypatch.setattr(probe,"bind_probe",lambda _:pytest.fail("probe started before fresh check"))
    with pytest.raises(ValueError,match="fresh"):probe.run(25000,tmp_path/"report.json")
    assert existing.read_text()=="original"


def test_symlink_and_parent_paths_refused(tmp_path):
    (tmp_path/"link").symlink_to(tmp_path,target_is_directory=True)
    for path in (tmp_path/"link"/"new",tmp_path/".."/"new"):
        with pytest.raises(ValueError):probe.checked_path(path)


@pytest.mark.parametrize("mode",["complete","failed","timeout","missing"])
def test_ss_output_and_timeout_are_bounded(mode,monkeypatch,fake_clock):
    calls=[]
    def child(command,**kwargs):
        calls.append((command,kwargs))
        if mode=="timeout":raise subprocess.TimeoutExpired(command,kwargs["timeout"],output=b"x"*9000)
        if mode=="missing":raise FileNotFoundError("ss missing")
        return SimpleNamespace(stdout=b"x"*9000,stderr=b"error",returncode=0 if mode=="complete" else 1)
    monkeypatch.setattr(probe.subprocess,"run",child)
    result=probe.ss_snapshot(25000,"UDP",60)
    assert len(result["output"].encode())<=8192 and calls[0][1]["timeout"]==2
    assert calls[0][0]==["ss","-H","-aunp","( sport = :25000 or sport = :25001 or sport = :25002 )"]
    assert not calls[0][1].get("shell",False)
    assert result["status"]==dict(complete="COMPLETE",failed="COMMAND_FAILED",timeout="TIMEOUT",missing="UNAVAILABLE")[mode]


def test_ss_near_deadline_does_not_wait_past_budget(monkeypatch,fake_clock):
    fake_clock.now=59.5
    def child(command,**kwargs):
        assert kwargs["timeout"]==.5
        return SimpleNamespace(stdout=b"",stderr=b"",returncode=0)
    monkeypatch.setattr(probe.subprocess,"run",child)
    assert probe.ss_snapshot(25000,"TCP",60)["status"]=="COMPLETE"
    fake_clock.now=60
    monkeypatch.setattr(probe.subprocess,"run",lambda *a,**kw:pytest.fail("child started after deadline"))
    assert probe.ss_snapshot(25000,"TCP",60)["status"]=="SKIPPED_DEADLINE"


def test_tcp_diagnostics_include_all_states_and_both_endpoints(monkeypatch,fake_clock):
    commands=[]
    def child(command,**kwargs):
        commands.append(command)
        return SimpleNamespace(stdout=b"TIME-WAIT 127.0.0.1:25001",stderr=b"",returncode=0)
    monkeypatch.setattr(probe.subprocess,"run",child)
    result=probe.ss_snapshot(25000,"TCP",60)
    assert commands==[["ss","-H","-antp",
        "( sport = :25000 or dport = :25000 or sport = :25001 or dport = :25001 or sport = :25002 or dport = :25002 )"]]
    assert result["output"].startswith("TIME-WAIT")


def test_ss_invalid_utf8_remains_bounded(monkeypatch,fake_clock):
    monkeypatch.setattr(probe.subprocess,"run",lambda *args,**kwargs:
        SimpleNamespace(stdout=b"\xff"*8192,stderr=b"",returncode=0))
    result=probe.ss_snapshot(25000,"TCP",60)
    assert len(result["output"].encode("utf-8"))<=8192 and result["output_truncated"]


@pytest.mark.parametrize("failure",[KeyboardInterrupt,OSError])
def test_interruption_retains_prior_samples(tmp_path,fake_clock,monkeypatch,failure):
    calls=0
    def sample(port):
        nonlocal calls
        calls+=1
        if calls==2:raise failure("test interruption")
        return bindings(True)
    monkeypatch.setattr(probe,"bind_probe",sample)
    result=probe.run(25000,tmp_path/"report.json")
    assert result["status"]=="OBSERVATION_FAILED_NOT_READY" and len(result["samples"])==1
    assert failure.__name__ in result["error"]


def test_source_mutation_cannot_report_ready(tmp_path,fake_clock,monkeypatch):
    hashes=iter(["a"*64,"b"*64])
    monkeypatch.setattr(probe,"source_sha",lambda:next(hashes))
    monkeypatch.setattr(probe,"bind_probe",lambda _:bindings(True))
    assert not probe.run(25000,tmp_path/"report.json")["ready_observed"]


@pytest.mark.parametrize("argument",["--host","--interval","--max-wait","--port-redirect"])
def test_cli_rejects_runtime_scope_overrides(argument):
    with pytest.raises(SystemExit):probe.main(["--port","25000","--output","unused",argument,"1"])


def test_cli_return_code_is_readiness_not_launch(tmp_path,fake_clock,monkeypatch):
    monkeypatch.setattr(probe,"bind_probe",lambda _:bindings(True))
    assert probe.main(["--port","25000","--output",str(tmp_path/"ready.json")])==0
    monkeypatch.setattr(probe,"bind_probe",lambda _:bindings(False))
    monkeypatch.setattr(probe,"ss_snapshot",lambda p,kind,d:dict(protocol=kind,status="COMPLETE",output="busy"))
    assert probe.main(["--port","25000","--output",str(tmp_path/"blocked.json")])==1
