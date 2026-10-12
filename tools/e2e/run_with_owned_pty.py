#!/usr/bin/env python3
"""HH_260906 - Run a PTY command inside the owner's process group."""

from __future__ import annotations

import os
import pty
import select
import signal
import subprocess
import sys
from typing import Sequence


class OwnedPtyError(RuntimeError):
    """HH_260906 - Report a PTY child that violates the ownership contract."""


def _normalize_returncode(returncode: int) -> int:
    return returncode if returncode >= 0 else 128 - returncode


def run(command: Sequence[str]) -> int:
    if not command:
        raise OwnedPtyError("a child command is required")

    # HH_260906 - Keep the relay and child in one killable process group.
    if os.getpgrp() != os.getpid():
        os.setpgid(0, 0)
    owner_pgid = os.getpgrp()
    master_fd, slave_fd = pty.openpty()
    child: subprocess.Popen[bytes] | None = None
    input_open = True

    def retain_group_signal(_signum: int, _frame: object) -> None:
        # HH_260906 - The owned group signal already reaches both relay and child.
        return

    signal.signal(signal.SIGINT, retain_group_signal)
    signal.signal(signal.SIGTERM, retain_group_signal)
    try:
        child = subprocess.Popen(
            list(command),
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            close_fds=True,
            start_new_session=False,
        )
        os.close(slave_fd)
        slave_fd = -1
        if os.getpgid(child.pid) != owner_pgid:
            raise OwnedPtyError("PTY child escaped the owned process group")

        while True:
            readers = [master_fd]
            if input_open:
                readers.append(sys.stdin.fileno())
            ready, _, _ = select.select(readers, [], [], 0.1)
            if sys.stdin.fileno() in ready:
                data = os.read(sys.stdin.fileno(), 4096)
                if data:
                    os.write(master_fd, data)
                else:
                    input_open = False
            if master_fd in ready:
                try:
                    data = os.read(master_fd, 65536)
                except OSError as error:
                    if error.errno != 5:
                        raise
                    data = b""
                if data:
                    os.write(sys.stdout.fileno(), data)
                elif child.poll() is not None:
                    break
            if child.poll() is not None and master_fd not in ready:
                break
        return _normalize_returncode(child.wait())
    finally:
        if slave_fd >= 0:
            os.close(slave_fd)
        os.close(master_fd)
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


def main(argv: Sequence[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    try:
        return run(arguments)
    except (OSError, OwnedPtyError) as error:
        print(f"owned PTY failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
