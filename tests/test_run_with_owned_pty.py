from __future__ import annotations

import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).parents[1]
OWNED_PTY = ROOT / "scripts/e2e/run_with_owned_pty.py"


def _read_line(process: subprocess.Popen[bytes], timeout: float = 5.0) -> str:
    assert process.stdout is not None
    ready, _, _ = select.select([process.stdout.fileno()], [], [], timeout)
    assert ready
    return process.stdout.readline().decode("utf-8").strip()


def test_owned_pty_keeps_child_in_relay_group_and_relays_space() -> None:
    child = (
        "import os,sys,termios,tty;"
        "tty.setcbreak(sys.stdin.fileno());"
        "print(f'READY tty={os.isatty(0)} pgid={os.getpgrp()}',flush=True);"
        "value=os.read(0,1);"
        "print(f'VALUE={value.hex()}',flush=True)"
    )
    process = subprocess.Popen(
        ["setsid", sys.executable, str(OWNED_PTY), sys.executable, "-c", child],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=False,
    )
    try:
        ready = _read_line(process)
        assert ready == f"READY tty=True pgid={process.pid}"
        assert process.stdin is not None
        process.stdin.write(b" ")
        process.stdin.flush()
        assert _read_line(process) == "VALUE=20"
        assert process.wait(timeout=5) == 0
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def test_owned_pty_forced_group_kill_cannot_orphan_signal_ignoring_child() -> None:
    child = (
        "import os,signal,time;"
        "signal.signal(signal.SIGINT,signal.SIG_IGN);"
        "signal.signal(signal.SIGTERM,signal.SIG_IGN);"
        "print(f'READY pid={os.getpid()} pgid={os.getpgrp()}',flush=True);"
        "time.sleep(60)"
    )
    process = subprocess.Popen(
        ["setsid", sys.executable, str(OWNED_PTY), sys.executable, "-c", child],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=False,
    )
    child_pid = None
    try:
        ready = _read_line(process)
        fields = dict(item.split("=", 1) for item in ready.split()[1:])
        child_pid = int(fields["pid"])
        assert int(fields["pgid"]) == process.pid

        os.killpg(process.pid, signal.SIGINT)
        time.sleep(0.05)
        assert process.poll() is None
        assert Path(f"/proc/{child_pid}").exists()

        os.killpg(process.pid, signal.SIGTERM)
        time.sleep(0.05)
        assert process.poll() is None
        assert Path(f"/proc/{child_pid}").exists()

        os.killpg(process.pid, signal.SIGKILL)
        assert process.wait(timeout=5) == -signal.SIGKILL
        deadline = time.monotonic() + 5.0
        while child_pid is not None and Path(f"/proc/{child_pid}").exists():
            assert time.monotonic() < deadline
            time.sleep(0.02)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
