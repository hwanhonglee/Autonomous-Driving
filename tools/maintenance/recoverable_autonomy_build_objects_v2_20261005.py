"""HH_261005 - Preserve V1 inspection and narrowly authenticate five installed non-build services before recoverable object cleanup."""
from __future__ import annotations

import ast
from contextlib import contextmanager
import importlib.util
import inspect
from pathlib import Path
import re
import stat

ROOT = Path(__file__).resolve().parents[2]
BASE_PATH = ROOT / "tools/maintenance/recoverable_autonomy_build_objects_v1_20261005.py"
BASE_TEST = ROOT / "learning/tests/test_recoverable_autonomy_build_objects_v1_20261005.py"
TEST = ROOT / "learning/tests/test_recoverable_autonomy_build_objects_v2_20261005.py"
PINS = {BASE_PATH: "bcd0a20a4ec4303f86fd78b42673dd732c3d4af4bb30083c2555ff6d4c1b0e07",
        BASE_TEST: "1c4dcc29231414f2e928fd6053b2d81cb38b7a2f1b7bea502dc5f47a501f24a6"}
spec = importlib.util.spec_from_file_location("_approved_cmake_objects_v1_20261005", BASE_PATH)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
ARCHIVE_SOURCES = (BASE_PATH, BASE_TEST, Path(__file__).resolve(), TEST)
OUT = ROOT / "storage/experiments/maintenance/2026-10-05/autonomy_build_objects_v2"
REMOTE = base.REMOTE_PERSONAL / "portable_e2e/archives/autonomy_build_objects_20261005_v2"
SCHEMA = "portable_e2e.recoverable_autonomy_cmake_objects.20261005.v2"
AUTH_SCHEMA = "portable_e2e.root_autonomy_cmake_objects_authority.20261005.v2"
ORIGINAL_SERVICE = base.exact_system_service


def sources():
    for path, value in PINS.items():
        base.require(base.digest(path) == value, "frozen V1 cleanup source changed")
    return {str(path.relative_to(ROOT)): base.digest(path) for path in ARCHIVE_SOURCES}


def trusted_unit(raw):
    # HH_261005 - Only this exact ROOT-reviewed /etc unit may extend V1's installed /usr and /snap source prefixes.
    if raw != "/etc/systemd/system/anydesk.service":
        return base.trusted_installed_file(raw, text=True)
    path = base.canonical(raw)
    before = base.stamp(path.lstat())
    base.require(stat.S_ISREG(before["mode"]) and before["uid"] == 0 and not stat.S_IMODE(before["mode"]) & 0o022 and before["size"] <= 1024**2, "exact root-owned AnyDesk unit required")
    base.require(all(p.stat().st_uid == 0 and not stat.S_IMODE(p.stat().st_mode) & 0o022 for p in path.parents), "AnyDesk unit ancestor untrusted")
    payload = path.read_bytes()
    base.require(base.stamp(path.lstat()) == before and b"ExecStart=/usr/bin/anydesk --service" in payload.splitlines(), "AnyDesk unit generation/ExecStart changed")
    return dict(requested=raw, resolved=str(path), generation=before, sha256=base.sha(payload))


def parent_evidence(folder, status, *, proc, argv, uid_four, cgroup, installed):
    ppid = int(re.search(r"(?m)^PPid:\s+(\d+)$", status).group(1))
    parent = proc / str(ppid)
    start = (parent / "stat").read_text().rsplit(")", 1)[1].split()[19]
    current_uid = [int(v) for v in re.search(r"(?m)^Uid:\s+(.+)$", (parent / "status").read_text()).group(1).split()]
    base.require(current_uid == uid_four and (parent / "cmdline").read_bytes() == ("\0".join(argv)+"\0").encode() and (parent / "cgroup").read_text().strip() == "0::"+cgroup, "exact installed service parent identity required")
    result = dict(pid=ppid, start=start, uid_four=current_uid, exact_argv=argv, cgroup="0::"+cgroup, installed=base.trusted_installed_file(installed))
    base.require((parent / "stat").read_text().rsplit(")", 1)[1].split()[19] == start, "service parent PID generation changed")
    return result


def exact_system_service(folder, row, status, *, proc=Path("/proc")):
    """HH_261005 - Authenticate exact installed AnyDesk/PAM/FUSE/key agents; no general user, shell, Python, or unknown exemption."""
    try:
        old = ORIGINAL_SERVICE(folder, row, status, proc=proc)
        if old is not None:
            return old
    except FileNotFoundError:
        # HH_261005 - AnyDesk's root-owned unit is in /etc; every new branch below independently authenticates all fields.
        pass
    comm = (folder / "comm").read_text().strip()
    cgroup = (folder / "cgroup").read_text().strip()
    argv = [part for part in row.get("cmdline", "").split("\0") if part]
    session = "/user.slice/user-1000.slice/session-499.scope"
    user_init = "/user.slice/user-1000.slice/user@1000.service/init.scope"
    portal = "/user.slice/user-1000.slice/user@1000.service/session.slice/xdg-document-portal.service"
    vte = "/user.slice/user-1000.slice/user@1000.service/app.slice/vte-spawn-a7a9504f-ba5d-4b2b-8fb1-d4ac343632e3.scope"
    result = dict(**row, comm=comm, cgroup=cgroup, exact_argv=argv, fd_maps_proof_available=False)
    if row["uid_four"] == [0]*4 and comm == "anydesk" and argv == ["/usr/bin/anydesk", "--service"] and cgroup == "0::/system.slice/anydesk.service":
        result.update(installed=base.trusted_installed_file("/usr/bin/anydesk"), unit=trusted_unit("/etc/systemd/system/anydesk.service"))
        result["parent"] = parent_evidence(folder, status, proc=proc, argv=["/lib/systemd/systemd", "--system", "--deserialize", "31", "splash"], uid_four=[0]*4, cgroup="/init.scope", installed="/lib/systemd/systemd")
        base.require(result["parent"]["pid"] == 1, "AnyDesk real init parent required")
    elif row["uid_four"] == [1000]*4 and comm == "(sd-pam)" and argv == ["(sd-pam)"] and cgroup == "0::"+user_init:
        result.update(installed=base.trusted_installed_file("/lib/systemd/systemd"), unit=trusted_unit("/lib/systemd/system/user@.service"))
        result["parent"] = parent_evidence(folder, status, proc=proc, argv=["/lib/systemd/systemd", "--user"], uid_four=[1000]*4, cgroup=user_init, installed="/lib/systemd/systemd")
    elif row["uid_four"] == [1000, 0, 0, 0] and comm == "fusermount3" and argv == ["fusermount3", "-o", "rw,nosuid,nodev,fsname=portal,auto_unmount,subtype=portal", "--", "/run/user/1000/doc"] and cgroup == "0::"+portal:
        result["installed"] = base.trusted_installed_file("/usr/bin/fusermount3")
        base.require(result["installed"]["generation"]["mode"] & stat.S_ISUID, "original setuid FUSE helper required")
        result["unit"] = trusted_unit("/usr/lib/systemd/user/xdg-document-portal.service")
        result["parent"] = parent_evidence(folder, status, proc=proc, argv=["/usr/libexec/xdg-document-portal"], uid_four=[1000]*4, cgroup=portal, installed="/usr/libexec/xdg-document-portal")
    elif row["uid_four"] == [1000]*4 and comm == "ssh-agent" and ((argv == ["/usr/bin/ssh-agent", "-D", "-a", "/run/user/1000/keyring/.ssh"] and cgroup == "0::"+session) or (argv == ["ssh-agent", "-s"] and cgroup == "0::"+vte)):
        result["installed"] = base.trusted_installed_file("/usr/bin/ssh-agent")
        base.require(result["installed"]["generation"]["mode"] & stat.S_ISGID, "original setgid SSH key helper required")
        if argv[0] == "/usr/bin/ssh-agent":
            result["parent"] = parent_evidence(folder, status, proc=proc, argv=["/usr/bin/gnome-keyring-daemon", "--daemonize", "--login"], uid_four=[1000]*4, cgroup=session, installed="/usr/bin/gnome-keyring-daemon")
        else:
            result["parent"] = parent_evidence(folder, status, proc=proc, argv=["/lib/systemd/systemd", "--user"], uid_four=[1000]*4, cgroup=user_init, installed="/lib/systemd/systemd")
            result["unit"] = trusted_unit("/lib/systemd/system/user@.service")
    else:
        return None
    base.require((folder / "stat").read_text().rsplit(")", 1)[1].split()[19] == row["start"], "newly authenticated service generation changed")
    return result


def prepared_main():
    # HH_261005 - The single inverse-proven source seam only archives all four authentic source files; lifecycle/unlink guards stay original.
    original = inspect.getsource(base.main)
    old = "for p in (Path(__file__).resolve(), TEST)"
    new = "for p in ARCHIVE_SOURCES"
    base.require(original.count(old) == 1, "exact source archive seam changed")
    amended = original.replace(old, new)
    base.require(ast.dump(ast.parse(amended.replace(new, old)), include_attributes=False) == ast.dump(ast.parse(original), include_attributes=False), "main inverse source proof failed")
    namespace = dict(vars(base), ARCHIVE_SOURCES=ARCHIVE_SOURCES)
    exec(compile(amended, str(Path(__file__).resolve())+":private_v1_main", "exec"), namespace)
    return namespace["main"]


@contextmanager
def activated():
    sources()
    replacements = dict(OUT=OUT, REMOTE=REMOTE, SCHEMA=SCHEMA, AUTH_SCHEMA=AUTH_SCHEMA, TEST=TEST, sources=sources, exact_system_service=exact_system_service)
    old = {name: getattr(base, name) for name in replacements}
    try:
        for name, value in replacements.items():
            setattr(base, name, value)
        yield
    finally:
        for name, value in old.items():
            setattr(base, name, value)


def main(argv=None):
    with activated():
        return prepared_main()(argv)


if __name__ == "__main__":
    raise SystemExit(main())
