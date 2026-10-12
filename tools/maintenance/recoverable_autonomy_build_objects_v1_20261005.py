"""HH_261005 - Back up and individually remove only explicitly approved old CMake object files."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import stat
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "autonomy/build"
INSTALL = ROOT / "autonomy/install"
OUT = ROOT / "storage/experiments/maintenance/2026-10-05/autonomy_build_objects_v1"
REMOTE_PERSONAL = Path("/home/pro6000/personal/hwanhong")
REMOTE = REMOTE_PERSONAL / "portable_e2e/archives/autonomy_build_objects_20261005_v1"
REMOTE_PYTHON = REMOTE_PERSONAL / "portable_e2e/venvs/py312/bin/python"
TEST = ROOT / "learning/tests/test_recoverable_autonomy_build_objects_v1_20261005.py"
SCHEMA = "portable_e2e.recoverable_autonomy_cmake_objects.20261005.v1"
AUTH_SCHEMA = "portable_e2e.root_autonomy_cmake_objects_authority.20261005.v1"
UID = 1000
SSH = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15", "pro6000-training"]
BUILD_TOOLS = {"make", "gmake", "ninja", "cmake", "colcon", "gcc", "g++", "c++", "cc", "cc1", "cc1plus", "clang", "clang++", "ld", "ld.gold", "ld.lld"}
TEXT_SUFFIXES = {".cmake", ".pc", ".la", ".txt", ".json", ".yaml", ".yml", ".xml", ".py", ".sh", ".bash", ".rsp"}
# HH_261005 - ROOT-reviewed exact service identities are not generic root/python/name exemptions; all denials remain evidence.
SYSTEM_SERVICE_ROWS = """
0|systemd|/init.scope|/lib/systemd/systemd|/lib/systemd/systemd --system --deserialize 31 splash
0|systemd-journal|systemd-journald|/lib/systemd/systemd-journald|/lib/systemd/systemd-journald
0|systemd-udevd|systemd-udevd|/lib/systemd/systemd-udevd|/lib/systemd/systemd-udevd
101|systemd-resolve|systemd-resolved|/lib/systemd/systemd-resolved|/lib/systemd/systemd-resolved
0|accounts-daemon|accounts-daemon|/usr/libexec/accounts-daemon|/usr/libexec/accounts-daemon
0|acpid|acpid|/usr/sbin/acpid|/usr/sbin/acpid
114|avahi-daemon|avahi-daemon|/usr/sbin/avahi-daemon|'avahi-daemon: running [a-2.local]'
114|avahi-daemon|avahi-daemon|/usr/sbin/avahi-daemon|'avahi-daemon: chroot helper'
0|cron|cron|/usr/sbin/cron|/usr/sbin/cron -f -P
102|dbus-daemon|dbus|/usr/bin/dbus-daemon|@dbus-daemon --system --address=systemd: --nofork --nopidfile --systemd-activation --syslog-only
0|NetworkManager|NetworkManager|/usr/sbin/NetworkManager|/usr/sbin/NetworkManager --no-daemon
0|irqbalance|irqbalance|/usr/sbin/irqbalance|/usr/sbin/irqbalance --foreground
0|networkd-dispat|networkd-dispatcher|/usr/bin/networkd-dispatcher|/usr/bin/python3 /usr/bin/networkd-dispatcher --run-startup-triggers
0|power-profiles-|power-profiles-daemon|/usr/libexec/power-profiles-daemon|/usr/libexec/power-profiles-daemon
0|switcheroo-cont|switcheroo-control|/usr/libexec/switcheroo-control|/usr/libexec/switcheroo-control
0|systemd-logind|systemd-logind|/lib/systemd/systemd-logind|/lib/systemd/systemd-logind
0|thermald|thermald|/usr/sbin/thermald|/usr/sbin/thermald --systemd --dbus-enable --adaptive
0|udisksd|udisks2|/usr/libexec/udisks2/udisksd|/usr/libexec/udisks2/udisksd
0|wpa_supplicant|wpa_supplicant|/sbin/wpa_supplicant|/sbin/wpa_supplicant -u -s -O /run/wpa_supplicant
0|unattended-upgr|unattended-upgrades|/usr/share/unattended-upgrades/unattended-upgrade-shutdown|/usr/bin/python3 /usr/share/unattended-upgrades/unattended-upgrade-shutdown --wait-for-signal
0|anydesk|anydesk|/usr/bin/anydesk|/usr/bin/anydesk --service
129|chronyd|chrony|/usr/sbin/chronyd|/usr/sbin/chronyd -F 1
0|gdm3|gdm|/usr/sbin/gdm3|/usr/sbin/gdm3
116|rtkit-daemon|rtkit-daemon|/usr/libexec/rtkit-daemon|/usr/libexec/rtkit-daemon
0|upowerd|upower|/usr/libexec/upowerd|/usr/libexec/upowerd
0|packagekitd|packagekit|/usr/libexec/packagekitd|/usr/libexec/packagekitd
123|colord|colord|/usr/libexec/colord|/usr/libexec/colord
113|kerneloops|kerneloops|/usr/sbin/kerneloops|/usr/sbin/kerneloops --test
113|kerneloops|kerneloops|/usr/sbin/kerneloops|/usr/sbin/kerneloops
108|systemd-oomd|systemd-oomd|/lib/systemd/systemd-oomd|/lib/systemd/systemd-oomd
0|polkitd|polkit|/usr/libexec/polkitd|/usr/libexec/polkitd --no-debug
0|ModemManager|ModemManager|/usr/sbin/ModemManager|/usr/sbin/ModemManager
104|rsyslogd|rsyslog|/usr/sbin/rsyslogd|/usr/sbin/rsyslogd -n -iNONE
0|snapd|snapd|/snap/snapd/current/usr/lib/snapd/snapd|/snap/snapd/current/usr/lib/snapd/snapd
0|cupsd|cups|/usr/sbin/cupsd|/usr/sbin/cupsd -l
0|cups-browsed|cups-browsed|/usr/sbin/cups-browsed|/usr/sbin/cups-browsed
7|dbus|cups|/usr/lib/cups/notifier/dbus|/usr/lib/cups/notifier/dbus dbus://
0|fwupd|fwupd|/usr/libexec/fwupd/fwupd|/usr/libexec/fwupd/fwupd
0|gdm-session-wor|/user.slice/user-1000.slice/session-499.scope|/usr/libexec/gdm-session-worker|'gdm-session-worker [pam/gdm-password]'
"""


def require(value, message):
    if not value:
        raise ValueError(message)


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def sha(value):
    return hashlib.sha256(value).hexdigest()


def canonical(raw):
    path = Path(raw).absolute()
    require(".." not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)), "symlink-free absolute path required")
    require(path.resolve() == path, "canonical path required")
    return path


def stamp(st):
    # HH_261005 - Exclude atime changes caused by reading, but pin complete inode generation and allocation.
    return dict(device=st.st_dev, inode=st.st_ino, mode=st.st_mode, uid=st.st_uid, gid=st.st_gid,
                size=st.st_size, blocks=st.st_blocks, nlink=st.st_nlink, mtime_ns=st.st_mtime_ns, ctime_ns=st.st_ctime_ns)


def directory_identity(st):
    return [st.st_dev, st.st_ino, st.st_mode, st.st_uid, st.st_gid]


def scoped(relative):
    path = Path(relative)
    return (not path.is_absolute() and bool(path.parts) and ".." not in path.parts
            and "CMakeFiles" in path.parts[:-1] and path.suffix == ".o"
            and not any("canonical_rebuild" in part for part in path.parts)
            and not any(ord(char) < 32 for char in str(path)))


def opened_object(root, relative, directories=None):
    """HH_261005 - Traverse exact parent descriptors without symlink following; caller owns both returned descriptors."""
    require(scoped(relative), "object is outside approved CMake scope")
    root = canonical(root)
    parent = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    fd = None
    try:
        require(directories is None or directory_identity(os.fstat(parent)) == directories["."], "build root identity changed")
        parts = Path(relative).parts
        for index, name in enumerate(parts[:-1]):
            next_fd = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent)
            os.close(parent); parent = next_fd
            key = str(Path(*parts[:index+1]))
            require(directories is None or directory_identity(os.fstat(parent)) == directories[key], "object parent identity changed")
        fd = os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW, dir_fd=parent)
        require(stat.S_ISREG(os.fstat(fd).st_mode), "object must be a regular file")
        return parent, fd
    except BaseException:
        if fd is not None:
            os.close(fd)
        os.close(parent)
        raise


def fd_digest(fd):
    value = hashlib.sha256()
    os.lseek(fd, 0, os.SEEK_SET)
    for chunk in iter(lambda: os.read(fd, 1024 ** 2), b""):
        value.update(chunk)
    return value.hexdigest()


def digest(path):
    path = canonical(path)
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        before = stamp(os.fstat(fd)); require(stat.S_ISREG(before["mode"]), "regular input required")
        result = fd_digest(fd)
        require(stamp(os.fstat(fd)) == before and stamp(path.lstat()) == before, "input generation changed")
        return result
    finally:
        os.close(fd)


def inventory(build=BUILD):
    """HH_261005 - Census only the approved root; never enter the canonical rebuild or follow links."""
    build = canonical(build)
    require(build.is_dir(), "approved build root missing")
    directories, objects, excluded_links = {}, {}, []
    def walk(folder):
        generation = stamp(folder.lstat())
        before = directory_identity(folder.lstat())
        require(stat.S_ISDIR(before[2]), "regular directory required")
        key = str(folder.relative_to(build))
        directories[key] = before
        for entry in sorted(os.scandir(folder), key=lambda e: e.name):
            relative = str(Path(entry.path).relative_to(build))
            st = entry.stat(follow_symlinks=False)
            if stat.S_ISLNK(st.st_mode):
                excluded_links.append(relative)
            elif stat.S_ISDIR(st.st_mode):
                if not any("canonical_rebuild" in part for part in Path(relative).parts):
                    walk(Path(entry.path))
            elif scoped(relative):
                require(stat.S_ISREG(st.st_mode) and st.st_uid == UID and st.st_nlink == 1, "regular UID1000 single-link object required")
                parent, fd = opened_object(build, relative, directories)
                try:
                    before_file = stamp(st)
                    require(stamp(os.fstat(fd)) == before_file, "object changed before hashing")
                    content_sha = fd_digest(fd)
                    require(stamp(os.fstat(fd)) == before_file and stamp(os.stat(entry.name, dir_fd=parent, follow_symlinks=False)) == before_file, "object changed during hashing")
                    objects[relative] = dict(generation=before_file, sha256=content_sha)
                finally:
                    os.close(fd); os.close(parent)
        require(stamp(folder.lstat()) == generation, "directory generation changed during census")
    walk(build)
    require(objects, "no approved object files")
    return dict(build_root=str(build), objects=objects, directories=directories,
                excluded_symlinks=excluded_links, object_count=len(objects),
                logical_bytes=sum(v["generation"]["size"] for v in objects.values()),
                allocated_bytes=sum(v["generation"]["blocks"]*512 for v in objects.values()),
                objects_sha256=sha(encoded(objects)))


def trusted_installed_file(raw, *, text=False):
    # HH_261005 - Legitimate /lib and snap current links are resolved, then every installed ancestor must be root-owned and non-writable.
    path = Path(raw).resolve(strict=True)
    require(path.is_absolute() and any(path.is_relative_to(Path(p)) for p in ("/usr", "/snap")), "installed system file outside trusted prefix")
    before = stamp(path.lstat())
    require(stat.S_ISREG(before["mode"]) and before["uid"] == 0 and not stat.S_IMODE(before["mode"]) & 0o022, "root-owned non-writable installed file required")
    require(all(p.stat().st_uid == 0 and not stat.S_IMODE(p.stat().st_mode) & 0o022 for p in path.parents), "installed ancestor is untrusted")
    result = dict(requested=str(raw), resolved=str(path), generation=before)
    if text:
        require(before["size"] <= 1024**2, "bounded installed service source required")
        data = path.read_bytes()
        require(stamp(path.lstat()) == before, "installed service source changed")
        result["sha256"] = sha(data)
    return result


def exact_system_service(folder, row, status, *, proc=Path("/proc")):
    """HH_261005 - Positively identify a finite ROOT-reviewed installed service, without claiming inaccessible fd/maps were inspected."""
    comm = (folder/"comm").read_text().strip()
    cgroup = (folder/"cgroup").read_text().strip()
    argv = [p for p in row.get("cmdline", "").split("\0") if p]
    for line in SYSTEM_SERVICE_ROWS.strip().splitlines():
        uid, expected_comm, service, installed, command = line.split("|", 4)
        expected_cgroup = service if service.startswith("/") else "/system.slice/"+service+".service"
        if row["uid_four"] != [int(uid)]*4 or comm != expected_comm or cgroup != "0::"+expected_cgroup or argv != shlex.split(command):
            continue
        result = dict(**row, comm=comm, cgroup=cgroup, exact_argv=argv, installed=trusted_installed_file(installed), fd_maps_proof_available=False)
        if service == "/init.scope":
            require(row["pid"] == 1, "init identity must be PID1")
        elif service.startswith("/user.slice/"):
            ppid = int(re.search(r"(?m)^PPid:\s+(\d+)$", status).group(1))
            parent = proc/str(ppid)
            parent_start = (parent/"stat").read_text().rsplit(")", 1)[1].split()[19]
            require((parent/"cmdline").read_bytes() == b"/usr/sbin/gdm3\0" and (parent/"cgroup").read_text().strip() == "0::/system.slice/gdm.service", "real gdm parent required")
            require(re.search(r"(?m)^Uid:\s+0\s+0\s+0\s+0$", (parent/"status").read_text()), "gdm parent UID0 required")
            result["parent"] = dict(pid=ppid, start=parent_start, installed=trusted_installed_file("/usr/sbin/gdm3"))
            require((parent/"stat").read_text().rsplit(")", 1)[1].split()[19] == parent_start, "gdm parent generation changed")
            result["unit"] = trusted_installed_file("/lib/systemd/system/gdm.service", text=True)
        else:
            result["unit"] = trusted_installed_file("/lib/systemd/system/"+service+".service", text=True)
        if argv and Path(argv[0]).name.startswith("python"):
            require(len(argv) >= 2 and argv[1] == installed, "generic Python cannot be a service exemption")
            result["interpreter"] = trusted_installed_file(argv[0])
            result["script"] = trusted_installed_file(installed, text=True)
        return result
    return None


def process_audit(build=BUILD, proc=Path("/proc")):
    """HH_261005 - Disclose every unreadable userspace field; an unknown reference cannot authorize unlink."""
    build = str(canonical(build))
    hits, unknown, userspace, kernel, services, denials = [], [], [], [], [], []
    observer = os.getpid()
    initial = {p.name for p in proc.iterdir() if p.name.isdigit()}
    for name in sorted(initial, key=int):
        folder = proc / name
        try:
            raw_stat = (folder/"stat").read_text()
            tail = raw_stat.rsplit(")", 1)[1].split()
            start, state = tail[19], tail[0]
            status = (folder/"status").read_text()
            uids = [int(v) for v in re.search(r"(?m)^Uid:\s+(.+)$", status).group(1).split()]
            require(len(uids) == 4, "complete UID identity required")
            kthread = re.search(r"(?m)^Kthread:\s+(\d+)$", status)
            if kthread and kthread.group(1) == "1" and uids == [0, 0, 0, 0]:
                require((folder/"stat").read_text().rsplit(")", 1)[1].split()[19] == start, "kernel PID generation changed")
                kernel.append(dict(pid=int(name), start=start, uid_four=uids, Kthread=1)); continue
            if int(name) == observer or state == "Z":
                continue
            row = dict(pid=int(name), start=start, uid_four=uids)
            userspace.append(row)
            row_unknown = []
            for field in ("cmdline", "cwd", "root", "exe", "maps", "fd"):
                try:
                    path = folder/field
                    if field in ("cwd", "root", "exe"):
                        values = [os.readlink(path)]
                    elif field == "fd":
                        values = []
                        for item in os.scandir(path):
                            try:
                                values.append(os.readlink(item.path))
                            except FileNotFoundError:
                                pass
                    else:
                        with path.open("rb") as stream:
                            payload = stream.read(16*1024**2+1)
                        require(len(payload) <= 16*1024**2, "bounded proc field required")
                        values = [payload.decode("utf-8", "replace")]
                    for value in values:
                        build_tool = field == "cmdline" and any(Path(part).name in BUILD_TOOLS for part in value.split("\0") if part)
                        if build in value or build_tool:
                            hits.append(dict(**row, field=field, reference=value[:2000]))
                        if field == "cmdline":
                            row["cmdline"] = value[:2000]
                except FileNotFoundError:
                    pass
                except (OSError, ValueError) as error:
                    row_unknown.append(dict(**row, field=field, error=type(error).__name__))
            require((folder/"stat").read_text().rsplit(")", 1)[1].split()[19] == start, "userspace PID generation changed")
            service = None
            if row_unknown and all(v["error"] == "PermissionError" and v["field"] in {"cwd", "root", "exe", "maps", "fd"} for v in row_unknown):
                try:
                    service = exact_system_service(folder, row, status, proc=proc)
                    require((folder/"stat").read_text().rsplit(")", 1)[1].split()[19] == start, "service PID generation changed")
                except (OSError, ValueError, AttributeError, IndexError):
                    service = None
            if service is not None and not any(h["pid"] == int(name) for h in hits):
                services.append(service); denials.extend(row_unknown)
            else:
                unknown.extend(row_unknown)
        except FileNotFoundError:
            pass
        except (OSError, ValueError, AttributeError, IndexError) as error:
            unknown.append(dict(pid=int(name), field="identity_or_generation", error=type(error).__name__))
    # HH_261005 - Newly created userspace may hold a reference; do not silently discard scan-time births.
    for name in sorted({p.name for p in proc.iterdir() if p.name.isdigit()} - initial, key=int):
        try:
            status = (proc/name/"status").read_text()
            if not (re.search(r"(?m)^Kthread:\s+1$", status) and re.search(r"(?m)^Uid:\s+0\s+0\s+0\s+0$", status)):
                unknown.append(dict(pid=int(name), field="new_userspace_during_scan"))
        except FileNotFoundError:
            pass
        except OSError as error:
            unknown.append(dict(pid=int(name), field="new_identity", error=type(error).__name__))
    return dict(hits=hits, unknown=unknown, userspace=userspace, proven_kernel_threads=kernel,
                exact_root_reviewed_system_services=services, system_service_permission_denials=denials,
                all_uid_userspace_inspection_complete=not unknown and not denials,
                no_unresolved_potential_build_consumer=not hits and not unknown,
                no_unknown_or_active_reference=not hits and not unknown)


def install_audit(manifest, install=INSTALL):
    """HH_261005 - Check all install links/inodes and bounded runtime metadata without deleting an install artifact."""
    install = canonical(install)
    require(install.is_dir(), "install root missing")
    objects = {str(Path(manifest["build_root"])/name) for name in manifest["objects"]}
    identities = {(v["generation"]["device"], v["generation"]["inode"]) for v in manifest["objects"].values()}
    rows, hits, unknown = {}, [], []
    def walk(folder):
        before_directory = stamp(folder.lstat())
        for entry in sorted(os.scandir(folder), key=lambda e: e.name):
            path = Path(entry.path); st = entry.stat(follow_symlinks=False)
            key = str(path.relative_to(install)); row = dict(generation=stamp(st))
            if stat.S_ISDIR(st.st_mode):
                walk(path)
            elif stat.S_ISLNK(st.st_mode):
                try:
                    target = os.readlink(path); resolved = str(path.resolve(strict=False)); row.update(link=target, resolved=resolved, dangling=not path.exists())
                    if resolved in objects:
                        hits.append(dict(path=key, kind="symlink_to_object", target=resolved))
                except (OSError, RuntimeError) as error:
                    unknown.append(dict(path=key, kind="unresolved_link", error=type(error).__name__))
            elif stat.S_ISREG(st.st_mode):
                if (st.st_dev, st.st_ino) in identities:
                    hits.append(dict(path=key, kind="object_inode_alias"))
                if path.suffix in TEXT_SUFFIXES or path.name in {"local_setup.bash", "setup.bash", "local_setup.sh", "setup.sh"}:
                    try:
                        require(st.st_size <= 16*1024**2, "bounded install metadata required")
                        payload = path.read_bytes(); row["sha256"] = sha(payload)
                        require(stamp(path.lstat()) == stamp(st), "install metadata generation changed")
                        absolute_object_ref = str(BUILD).encode() in payload and b".o" in payload
                        if re.search(rb"(?:CMakeFiles|IMPORT_OBJECTS)[^\n]*\.o(?:[\s\"';)]|$)", payload) or absolute_object_ref:
                            hits.append(dict(path=key, kind="metadata_object_reference"))
                    except (OSError, ValueError) as error:
                        unknown.append(dict(path=key, kind="unreadable_metadata", error=type(error).__name__))
            else:
                unknown.append(dict(path=key, kind="special_install_entry"))
            rows[key] = row
        require(stamp(folder.lstat()) == before_directory, "install directory generation changed during census")
    walk(install)
    return dict(install_root=str(install), entry_count=len(rows), rows_sha256=sha(encoded(rows)), rows=rows,
                symlink_count=sum("link" in row for row in rows.values()), dangling_symlink_count=sum(row.get("dangling") is True for row in rows.values()),
                hits=hits, unknown=unknown, no_unknown_or_object_reference=not hits and not unknown)


def write_new(path, value):
    path = canonical(path)
    with path.open("xb") as stream:
        stream.write(encoded(value)); stream.flush(); os.fsync(stream.fileno())


def read(path, expected):
    path = canonical(path); require(path.stat().st_size <= 64*1024**2, "bounded receipt required")
    payload = path.read_bytes(); require(sha(payload) == expected, "externally pinned receipt changed")
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, "duplicate JSON key"); result[key] = value
        return result
    return json.loads(payload, object_pairs_hook=unique, parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))


def sources():
    return {str(p.relative_to(ROOT)): digest(p) for p in (Path(__file__).resolve(), TEST)}


def remote_program(phase):
    """HH_261005 - Remote verification reads every archived object, never a local second object copy."""
    require(phase in ("create", "verify"), "unknown remote phase")
    code = "import os,json,hashlib,stat\nfrom pathlib import Path\n"
    for function in (require, encoded, sha, canonical, stamp, fd_digest, digest, scoped):
        import inspect
        code += inspect.getsource(function) + "\n"
    code += "data=json.loads(__import__('sys').stdin.buffer.read()); archive=canonical(" + repr(str(REMOTE)) + "); personal=canonical(" + repr(str(REMOTE_PERSONAL)) + ")\n"
    code += "require(archive.is_relative_to(personal) and all(scoped(p) for p in data['manifest']['objects']),'exact remote scope required')\n"
    code += "require(all(sha(bytes.fromhex(v))==data['sources'][k] for k,v in data['source_archive'].items()) and set(data['sources'])==set(data['source_archive']),'source archive differs')\n"
    code += "for p in (archive.parent,*archive.parent.parents):\n if p.exists() and p.is_relative_to(personal): require(p.stat().st_uid==os.getuid() and not stat.S_IMODE(p.stat().st_mode)&0o022,'private remote ancestor required')\n"
    if phase == "create":
        code += "require(not archive.exists(),'remote archive must be create-only')\nmissing=[]; p=archive.parent\nwhile not p.exists():\n require(p.is_relative_to(personal),'remote ancestor outside private root'); missing.append(p); p=p.parent\nfor p in reversed(missing): p.mkdir(mode=0o700)\narchive.mkdir(mode=0o700); (archive/'objects').mkdir(mode=0o700)\n"
        code += "for name in ('manifest','sources','source_archive','authority'):\n with (archive/(name+'.json')).open('xb') as f: f.write(encoded(data[name]))\nprint(json.dumps(dict(created=True,remote=str(archive))))\n"
    else:
        code += "require(archive.stat().st_uid==os.getuid() and stat.S_IMODE(archive.stat().st_mode)==0o700,'owned0700 archive required')\n"
        code += "require(set(p.name for p in archive.iterdir())=={'objects','manifest.json','sources.json','source_archive.json','authority.json'},'remote archive entries differ')\nfor name in ('manifest','sources','source_archive','authority'):\n require((archive/(name+'.json')).read_bytes()==encoded(data[name]),'remote metadata differs')\n"
        code += "actual={}; objectroot=canonical(archive/'objects')\nfor base,dirs,files in os.walk(objectroot,followlinks=False):\n require(all(not(Path(base)/d).is_symlink() for d in dirs),'remote symlink directory denied')\n for name in files:\n  path=canonical(Path(base)/name); relative=str(path.relative_to(objectroot)); require(relative in data['manifest']['objects'],'unlisted archived object'); st=path.lstat(); require(stat.S_ISREG(st.st_mode) and st.st_uid==os.getuid() and st.st_nlink==1,'owned regular archived object required'); expected=data['manifest']['objects'][relative]; require(st.st_size==expected['generation']['size'] and stat.S_IMODE(st.st_mode)==stat.S_IMODE(expected['generation']['mode']),'remote object size/mode differs'); value=digest(path); require(value==expected['sha256'],'remote object hash differs'); actual[relative]=value\n"
        code += "require(set(actual)==set(data['manifest']['objects']),'missing archived object'); print(json.dumps(dict(remote=str(archive),object_count=len(actual),all_object_hashes_verified=True,sha256=sha(encoded(actual)),logical_bytes=sum(v['generation']['size'] for v in data['manifest']['objects'].values()))))\n"
    return code


def checked_authority(args, inspection):
    value = read(args.authority, args.authority_sha256)
    require(value.get("schema") == AUTH_SCHEMA and value.get("mode") == args.mode
            and value.get("approved_build_root") == str(BUILD) and value.get("remote_archive") == str(REMOTE)
            and value.get("inspection_sha256") == args.inspection_sha256
            and value.get("objects_sha256") == inspection["manifest"]["objects_sha256"]
            and value.get("source_sha256") == sources() and value.get("explicit_user_approved_exact_o_scope") is True
            and value.get("other_artifacts_must_not_be_removed") is True, "exact separate ROOT authority required")
    if args.mode == "remove":
        require(value.get("backup_receipt_sha256") == args.backup_receipt_sha256
                and value.get("fail_closed_unknown_process_and_install_risk") is True, "backup-bound fail-closed removal authority required")
    return value


def verify_object_before_unlink(relative, expected, manifest, *, do_unlink=False, before_unlink=None):
    """HH_261005 - Verify pinned inode generation and content on open descriptors immediately before one unlink."""
    parent, fd = opened_object(Path(manifest["build_root"]), relative, manifest["directories"])
    try:
        require(stamp(os.fstat(fd)) == expected["generation"], "object generation changed before removal")
        require(fd_digest(fd) == expected["sha256"], "object hash changed before removal")
        require(stamp(os.fstat(fd)) == expected["generation"], "object generation changed while verifying")
        if do_unlink:
            require(callable(before_unlink), "durable per-object intent required before unlink")
            before_unlink(dict(relative=relative, sha256=expected["sha256"], generation=expected["generation"], unlink_attempted=True))
        require(stamp(os.stat(Path(relative).name, dir_fd=parent, follow_symlinks=False)) == expected["generation"], "object entry changed immediately before unlink")
        if do_unlink:
            os.unlink(Path(relative).name, dir_fd=parent)
            try:
                os.stat(Path(relative).name, dir_fd=parent, follow_symlinks=False)
                raise ValueError("removed object entry still exists or was recreated")
            except FileNotFoundError:
                pass
        return dict(relative=relative, sha256=expected["sha256"], generation=expected["generation"], removed=do_unlink)
    finally:
        os.close(fd); os.close(parent)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("inspect", "backup", "remove"), required=True)
    parser.add_argument("--inspection"); parser.add_argument("--inspection-sha256")
    parser.add_argument("--authority"); parser.add_argument("--authority-sha256")
    parser.add_argument("--backup-receipt"); parser.add_argument("--backup-receipt-sha256")
    args = parser.parse_args(argv)
    require(os.getuid() == UID and str(canonical(BUILD)) == str(BUILD) and str(canonical(INSTALL)) == str(INSTALL), "exact local root/UID required")
    if args.mode == "inspect":
        require(not any((args.inspection, args.inspection_sha256, args.authority, args.authority_sha256, args.backup_receipt, args.backup_receipt_sha256)), "inspect cannot carry destructive authority")
        require(not OUT.exists(), "fresh inspection output required")
        OUT.mkdir(parents=True, mode=0o700)
        try:
            manifest = inventory()
            usage, refs = process_audit(), install_audit(manifest)
            value = dict(schema=SCHEMA, manifest=manifest, process=usage, install=refs, source_sha256=sources(), local_removed=False)
            write_new(OUT/"inspection.json", value)
        except BaseException as error:
            write_new(OUT/"inspection_failure.json", dict(schema=SCHEMA, error=type(error).__name__+": "+str(error), local_removed=False))
            raise
        print(json.dumps(dict(output=str(OUT/"inspection.json"), inspection_sha256=digest(OUT/"inspection.json"),
                             object_count=manifest["object_count"], logical_bytes=manifest["logical_bytes"], allocated_bytes=manifest["allocated_bytes"],
                             process_hits=len(usage["hits"]), unknown_process_fields=len(usage["unknown"]), install_hits=len(refs["hits"]), unknown_install_fields=len(refs["unknown"]), install_symlinks=refs["symlink_count"], dangling_install_symlinks=refs["dangling_symlink_count"])))
        return 0
    require(all((args.inspection, args.inspection_sha256, args.authority, args.authority_sha256)), "pinned inspection and ROOT authority required")
    require(canonical(args.inspection) == OUT/"inspection.json", "exact inspection path required")
    inspection = read(args.inspection, args.inspection_sha256)
    require(inspection.get("schema") == SCHEMA and inspection["source_sha256"] == sources(), "inspection/source identity differs")
    authority = checked_authority(args, inspection)
    manifest = inspection["manifest"]
    require(inventory() == manifest, "approved object census changed")
    refs, usage = install_audit(manifest), process_audit()
    require(refs["no_unknown_or_object_reference"] and not usage["hits"], "install or active build reference blocks backup/removal")
    removed = []
    stage = OUT/(args.mode+"_started.json")
    write_new(stage, dict(schema=SCHEMA, authority_sha256=args.authority_sha256, source_sha256=sources(), process=usage, install_rows_sha256=refs["rows_sha256"]))
    payload = dict(manifest=manifest, sources=sources(), authority=authority,
                   source_archive={str(p.relative_to(ROOT)): p.read_bytes().hex() for p in (Path(__file__).resolve(), TEST)})
    def invoke(command, name, data=None):
        result = subprocess.run(command, input=data, capture_output=True, timeout=1800)
        write_new(OUT/(name+".json"), dict(argv=command, returncode=result.returncode, stdout=result.stdout.decode("utf-8", "replace"), stderr=result.stderr.decode("utf-8", "replace")))
        require(result.returncode == 0, "external backup/verification failed: "+name)
        return json.loads(result.stdout) if name.startswith("remote_") else None
    def remote(phase, name):
        command = SSH+[shlex.quote(str(REMOTE_PYTHON))+" -c "+shlex.quote(remote_program(phase))]
        return invoke(command, name, encoded(payload))
    try:
        if args.mode == "backup":
            require(not args.backup_receipt and not args.backup_receipt_sha256, "backup must not use removal receipt")
            remote("create", "remote_create")
            command = ["rsync", "--archive", "--no-owner", "--no-group", "--no-links", "--protect-args", "--ignore-existing", "--from0", "--files-from=-", "-e", "ssh -o BatchMode=yes -o ConnectTimeout=15", "--", str(BUILD)+"/", "pro6000-training:"+str(REMOTE/"objects")+"/"]
            invoke(command, "rsync_objects", b"\0".join(p.encode() for p in manifest["objects"])+b"\0")
            verification = remote("verify", "remote_verify_backup")
            require(inventory() == manifest and sources() == inspection["source_sha256"], "local objects/source changed during backup")
            write_new(OUT/"backup_complete.json", dict(schema=SCHEMA, inspection_sha256=args.inspection_sha256, source_sha256=sources(), authority=authority,
                remote_archive=str(REMOTE), remote_verification=verification, objects_sha256=manifest["objects_sha256"], local_removed=False))
        else:
            require(all((args.backup_receipt, args.backup_receipt_sha256)) and canonical(args.backup_receipt) == OUT/"backup_complete.json", "pinned complete backup required")
            backup = read(args.backup_receipt, args.backup_receipt_sha256)
            require(backup.get("schema") == SCHEMA and backup.get("inspection_sha256") == args.inspection_sha256
                and backup.get("objects_sha256") == manifest["objects_sha256"] and backup.get("source_sha256") == sources()
                and backup.get("remote_archive") == str(REMOTE) and backup.get("local_removed") is False
                and backup["remote_verification"].get("all_object_hashes_verified") is True, "complete authenticated backup differs")
            # HH_261005 - Reauthenticate the original backup metadata, not the fresh removal authority stored only locally.
            payload["authority"] = backup["authority"]
            remote("verify", "remote_verify_before_removal")
            require(usage["no_unknown_or_active_reference"], "unknown userspace reference risk blocks removal")
            # HH_261005 - A single create-only fsynced journal retains intent and success without thousands of small files.
            with (OUT/"removal_journal.jsonl").open("xb") as journal:
                def append_journal(row):
                    journal.write(encoded(dict(schema=SCHEMA, **row))+b"\n"); journal.flush(); os.fsync(journal.fileno())
                for index, (relative, expected) in enumerate(manifest["objects"].items(), 1):
                    if (index-1) % 128 == 0:
                        usage = process_audit()
                        append_journal(dict(kind="process_audit", before_object=index, receipt=usage))
                        require(usage["no_unknown_or_active_reference"], "new/unknown active reference blocks remaining removals")
                        require(install_audit(manifest)["rows_sha256"] == refs["rows_sha256"], "install references/generation changed during cleanup")
                        require(sources() == inspection["source_sha256"], "cleanup source changed")
                    receipt = verify_object_before_unlink(relative, expected, manifest, do_unlink=True,
                        before_unlink=lambda row: append_journal(dict(kind="unlink_intent", **row)))
                    removed.append(receipt)
                    append_journal(dict(kind="removed", **receipt, remote_archive=str(REMOTE), backup_receipt_sha256=args.backup_receipt_sha256))
            require(install_audit(manifest)["rows_sha256"] == refs["rows_sha256"] and sources() == inspection["source_sha256"], "final install/source changed")
            verification = remote("verify", "remote_verify_after_removal")
            write_new(OUT/"removal_result.json", dict(schema=SCHEMA, removed=removed, removed_count=len(removed),
                logical_bytes=manifest["logical_bytes"], allocated_bytes=manifest["allocated_bytes"], remote_verification=verification,
                remote_backup_retained=True, nonobject_artifacts_removed=False, source_sha256=sources()))
    except BaseException as error:
        write_new(OUT/(args.mode+"_failure.json"), dict(schema=SCHEMA, error=type(error).__name__+": "+str(error), removed=removed,
                  removed_count=len(removed), remote_backup_retained=True, nonobject_artifacts_removed=False))
        raise
    print(json.dumps(dict(mode=args.mode, completed=True, removed_count=len(removed), remote_backup_retained=True)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
