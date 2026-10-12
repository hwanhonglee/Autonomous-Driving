"""HH_261001 - Wait for owned sandbox descendants AND ports after Unreal shutdown."""

from __future__ import annotations

import os
import signal
import time


def drain_owned_sandbox(launcher, original: dict, editor: dict | None, port: int,
                        *, inspect_process, list_processes, assert_ports_free,
                        grace_seconds: float = 35.0, kill_after: float = 20.0) -> dict:
    """Never treat reaping bubblewrap alone as proof that its namespace is gone."""
    pgid = launcher.pid
    current = inspect_process(pgid)
    if current and (current["start_ticks"] != original["start_ticks"] or current["pgid"] != pgid):
        raise RuntimeError("launcher generation or PGID changed; refusing cleanup signal")
    initial = {r["pid"]: r for r in list_processes() if r["pgid"] == pgid}
    sent = []
    if editor:
        live_editor = inspect_process(editor["pid"])
        if live_editor and (live_editor["start_ticks"] != editor["start_ticks"] or
                            live_editor["pgid"] != pgid):
            raise RuntimeError("editor generation or PGID changed; refusing cleanup signal")
        if live_editor:
            # Let bubblewrap reap its child normally before falling back to group signals.
            try:
                os.kill(editor["pid"], signal.SIGTERM)
                sent.append({"signal": "SIGTERM", "target_pid": editor["pid"]})
            except ProcessLookupError:
                pass
    elif current:
        try:
            os.killpg(pgid, signal.SIGTERM)
            sent.append({"signal": "SIGTERM", "target_pgid": pgid})
        except ProcessLookupError:
            pass
    started = time.monotonic()
    stable, killed = 0, False
    samples = []
    while time.monotonic() - started < grace_seconds:
        launcher.poll()  # Reap the launcher independently of child/namespace completion.
        survivors = [r for r in list_processes() if r["pgid"] == pgid and r["state"] != "Z"]
        ports_released, port_error = True, None
        try:
            assert_ports_free(port)
        except OSError as exc:
            ports_released, port_error = False, str(exc)
        elapsed = time.monotonic() - started
        samples.append({"elapsed_seconds": round(elapsed, 3),
                        "live_owned_pids": [r["pid"] for r in survivors],
                        "ports_released": ports_released})
        stable = stable + 1 if not survivors and ports_released else 0
        if stable >= 2:
            launcher.wait(timeout=1)
            return {"status": "PASS", "owned_processes_stopped_and_ports_released": True,
                    "elapsed_seconds": elapsed, "signals": sent, "samples": samples}
        if elapsed >= kill_after and survivors and not killed:
            # Keep a generation witness from the original owned group before escalation.
            witnesses = [r for r in survivors if r["pid"] in initial and
                         r["start_ticks"] == initial[r["pid"]]["start_ticks"]]
            if not witnesses:
                raise RuntimeError("no original owned process generation remains; refusing group kill")
            os.killpg(pgid, signal.SIGKILL)
            sent.append({"signal": "SIGKILL", "target_pgid": pgid})
            killed = True
        time.sleep(0.2)
    raise RuntimeError(f"owned shutdown deadline: survivors={survivors}; ports={port_error}")
