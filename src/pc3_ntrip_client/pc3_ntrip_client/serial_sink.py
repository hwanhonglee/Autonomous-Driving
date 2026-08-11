"""Small dependency-free POSIX serial writer."""

# HH_260811 - Feed validated corrections to NovAtel port0 without pyserial.

import errno
import fcntl
import os
import select
import termios
import time
from typing import Optional


class SerialError(RuntimeError):
    pass


_BAUD_CONSTANTS = {
    1200: termios.B1200,
    2400: termios.B2400,
    4800: termios.B4800,
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
}

# HH_260811 - Configure correction auto-detection in receiver RAM without persistence.
NOVATEL_RTCM_INPUT_COMMAND = b"INTERFACEMODE THISPORT AUTO NONE OFF\r\n"


class SerialSink:
    """Open, configure, retry, and write a dedicated POSIX TTY."""

    def __init__(
        self,
        path: str,
        baud: int,
        reopen_interval_sec: float = 5.0,
        configure_novatel_rtcm_input: bool = False,
    ) -> None:
        self.path = path
        self.baud = baud
        self.reopen_interval_sec = reopen_interval_sec
        self.configure_novatel_rtcm_input = configure_novatel_rtcm_input
        self._fd: Optional[int] = None
        self._retry_after = 0.0
        self.bytes_written = 0
        self.write_errors = 0
        self.setup_attempts = 0
        self.setup_successes = 0
        self.setup_succeeded = not configure_novatel_rtcm_input

    @property
    def is_open(self) -> bool:
        return self._fd is not None

    def _open(self) -> bool:
        if self._fd is not None:
            return True
        if time.monotonic() < self._retry_after:
            return False
        speed = _BAUD_CONSTANTS.get(self.baud)
        if speed is None:
            raise SerialError("unsupported serial baud rate")
        fd: Optional[int] = None
        try:
            fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = speed
            attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
            self._fd = fd
            if self.configure_novatel_rtcm_input:
                self.setup_attempts += 1
                self.setup_succeeded = False
                self._write_all(NOVATEL_RTCM_INPUT_COMMAND)
                termios.tcdrain(self._fd)
                self.setup_successes += 1
                self.setup_succeeded = True
            return True
        except (OSError, SerialError) as exc:
            self.write_errors += 1
            self._retry_after = time.monotonic() + self.reopen_interval_sec
            if self._fd is not None:
                self.close()
            elif fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
            if isinstance(exc, SerialError):
                raise SerialError("NovAtel RTCM input setup failed") from exc
            raise SerialError("serial correction port cannot be opened") from exc

    def _write_all(self, data: bytes) -> None:
        if self._fd is None:
            raise SerialError("serial correction port is not open")
        view = memoryview(data)
        offset = 0
        while offset < len(view):
            _, writable, _ = select.select([], [self._fd], [], 2.0)
            if not writable:
                raise SerialError("serial correction port write timed out")
            try:
                count = os.write(self._fd, view[offset:])
            except OSError as exc:
                if exc.errno in (errno.EINTR, errno.EAGAIN):
                    continue
                raise
            if count <= 0:
                raise SerialError("serial correction port write made no progress")
            offset += count

    def write(self, data: bytes) -> bool:
        if not self._open():
            return False
        try:
            self._write_all(data)
            self.bytes_written += len(data)
            return True
        except (OSError, SerialError) as exc:
            self.write_errors += 1
            self.close()
            self._retry_after = time.monotonic() + self.reopen_interval_sec
            if isinstance(exc, SerialError):
                raise
            raise SerialError("serial correction port write failed") from exc

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
