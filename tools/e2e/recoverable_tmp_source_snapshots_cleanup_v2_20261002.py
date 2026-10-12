"""HH_261002 - Preserve exactly eight audited old source trees before separately authorized cleanup."""
from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import os
from pathlib import Path
import shlex
import shutil
import stat
import subprocess

ROOT = Path(__file__).resolve().parents[2]
TARGET_INODES = {
    "/tmp/tmp.FqnZIl7q7b": 6309608,
    "/tmp/tmp.1Oh0JE3w8S": 6300049,
    "/tmp/tmp.6NTXOvu0tl": 6309607,
    "/tmp/tmp.Mw7a4HNBM6": 6299103,
    "/tmp/tmp.LCnMqH1cV5": 6298157,
    "/tmp/tmp.nSZWnclU8x": 6297211,
    "/tmp/tmp.FcfFEGU7S3": 6300050,
    "/tmp/tmp.91zoyANmpS": 6292411,
}
TARGETS = tuple(TARGET_INODES)
SOURCE_UID = 1000
REMOTE_PERSONAL = Path("/home/pro6000/personal/hwanhong")
REMOTE = REMOTE_PERSONAL / "portable_e2e/archives/tmp_contract_snapshots_20261002_v2"
PYTHON = REMOTE_PERSONAL / "portable_e2e/venvs/py312/bin/python"
RECEIPT = ROOT / "storage/experiments/maintenance/2026-10-02/tmp_contract_snapshots_cleanup_v2"
TEST = ROOT / "learning/tests/test_recoverable_tmp_source_snapshots_cleanup_v2_20261002.py"
SCHEMA = "portable_e2e.recoverable_tmp_source_snapshots_cleanup.20261002.v2"
AUTH_SCHEMA = "portable_e2e.root_tmp_source_snapshots_cleanup_authority.20261002.v2"
SSH = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15", "pro6000-training"]


def require(value, message):
    if not value:
        raise ValueError(message)


def canonical(path):
    path = Path(path).absolute()
    require(".." not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
            "canonical path without symlink ancestors required")
    return path


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def digest(path):
    path = canonical(path)
    require(stat.S_ISREG(path.stat().st_mode), "regular input required")
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 ** 2), b""):
            value.update(chunk)
    return value.hexdigest()


def read(path, expected=None):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    path = canonical(path)
    require(path.stat().st_size <= 128 * 1024 ** 2, "bounded JSON required")
    payload = path.read_bytes()
    require(expected is None or sha(payload) == expected, "externally pinned JSON changed")
    return json.loads(payload, object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def stamp(value):
    # HH_261002 - Reading changes atime, not the inode/ctime/mtime generation guarded here.
    return [value.st_dev, value.st_ino, value.st_mode, value.st_uid, value.st_gid,
            value.st_size, value.st_mtime_ns, value.st_ctime_ns, value.st_nlink]


def census(paths, local=False):
    entries, identities, hardlinks = {}, {}, {}
    for raw in paths:
        root = canonical(raw)
        st = root.stat()
        require(stat.S_ISDIR(st.st_mode), "regular tree root required")
        if local:
            require(str(root) in TARGET_INODES and st.st_ino == TARGET_INODES[str(root)]
                    and st.st_uid == SOURCE_UID and stat.S_IMODE(st.st_mode) == 0o700,
                    "audited exact inode/UID/0700 root required")
        root_before = stamp(st)
        def walk(folder):
            before = folder.lstat()
            relative = str(folder.relative_to(root))
            key = root.name + ("/" + relative if relative != "." else "")
            require(not any(ord(c) < 32 for c in key), "unsafe filename")
            item = dict(mode=stat.S_IMODE(before.st_mode), mtime_ns=before.st_mtime_ns)
            if stat.S_ISDIR(before.st_mode):
                item["kind"] = "directory"
                with os.scandir(folder) as stream:
                    children = sorted((Path(row.path) for row in stream), key=lambda p: p.name)
                for child in children:
                    walk(child)
            elif stat.S_ISREG(before.st_mode):
                flags = os.O_RDONLY | os.O_NOFOLLOW
                with os.fdopen(os.open(folder, flags), "rb") as stream:
                    require(stamp(os.fstat(stream.fileno())) == stamp(before), "file replaced before full read")
                    value = hashlib.sha256()
                    for chunk in iter(lambda: stream.read(1024 ** 2), b""):
                        value.update(chunk)
                    require(stamp(os.fstat(stream.fileno())) == stamp(before), "file changed during full read")
                item.update(kind="file", size=before.st_size, sha256=value.hexdigest())
                first = hardlinks.setdefault((before.st_dev, before.st_ino), key)
                item["hardlink_first"] = first
            elif stat.S_ISLNK(before.st_mode):
                target = os.readlink(folder)
                require(not Path(target).is_absolute() and (folder.parent / target).resolve(strict=True).is_relative_to(root),
                        "only existing internal symlink targets supported; no traversal during copying")
                item.update(kind="symlink", target=target)
            else:
                raise ValueError("special files cannot be archived by this helper")
            require(stamp(folder.lstat()) == stamp(before), "tree entry changed during census")
            entries[key], identities[key] = item, stamp(before)
        walk(root)
        require(stamp(root.stat()) == root_before, "tree root generation changed")
    entries, identities = dict(sorted(entries.items())), dict(sorted(identities.items()))
    return dict(entries=entries, identities=identities, payload_sha256=sha(encoded(entries)),
                generation_sha256=sha(encoded(identities)), file_bytes=sum(v.get("size", 0) for v in entries.values()),
                tree_payload_sha256={Path(p).name: sha(encoded({k: v for k, v in entries.items() if k.split("/", 1)[0] == Path(p).name})) for p in paths},
                tree_generation_sha256={Path(p).name: sha(encoded({k: v for k, v in identities.items() if k.split("/", 1)[0] == Path(p).name})) for p in paths})


def tree_pins(value):
    return {path: dict(payload_sha256=value["tree_payload_sha256"][Path(path).name],
                       generation_sha256=value["tree_generation_sha256"][Path(path).name],
                       root_identity=value["identities"][Path(path).name]) for path in TARGETS}


def single_tree_payload(entries, target):
    prefix = Path(target).name
    selected = {key: dict(value) for key, value in entries.items() if key.split("/", 1)[0] == prefix}
    groups = {}
    for key, value in selected.items():
        if value["kind"] == "file":
            groups.setdefault(value["hardlink_first"], []).append(key)
    for value in selected.values():
        if value["kind"] == "file":
            value["hardlink_first"] = min(groups[value["hardlink_first"]])
    return selected


def verify_single_target(target, expected):
    # HH_261002 - Re-read ALL child bytes immediately before deletion; only owned earlier hardlink ctime/nlink changes are exempt.
    actual = census([target], local=True)
    require(single_tree_payload(actual["entries"], target) == single_tree_payload(expected["entries"], target),
            "exact per-target payload changed before removal")
    for key, identity in actual["identities"].items():
        pinned = expected["identities"][key]
        require(identity[:7] == pinned[:7] if actual["entries"][key]["kind"] == "file" else identity == pinned,
                "per-target inode/owner/mode/mtime generation changed before removal")
    return actual


def references(files):
    result = {}
    for raw in files:
        path = canonical(raw)
        require(path.is_relative_to(ROOT) and stat.S_ISREG(path.stat().st_mode), "exact existing repository declaration required")
        payload = path.read_bytes()
        require(not any(target.encode() in payload for target in TARGETS), "declared reference to temporary tree found")
        result[str(path)] = sha(payload)
    require(result, "ROOT must supply its complete current declaration file list")
    return dict(sorted(result.items()))


def process_usage(proc_root=Path("/proc")):
    hits, inaccessible = [], []
    def related(text):
        return any(target in text for target in TARGETS)
    initial = {p.name for p in proc_root.iterdir() if p.name.isdigit()}
    for folder in sorted((proc_root / name for name in initial), key=lambda p: p.name):
        if not folder.name.isdigit():
            continue
        try:
            identity = folder.stat().st_uid
            start = (folder / "stat").read_text().rsplit(")", 1)[1].split()[19]
        except FileNotFoundError:
            continue
        except PermissionError:
            inaccessible.append(dict(pid=int(folder.name), uid=None, field="identity"))
            continue
        for field in ("cwd", "root", "exe", "cmdline", "maps", "fd"):
            try:
                path = folder / field
                if field in ("cwd", "root", "exe"):
                    values = [os.readlink(path)]
                elif field == "fd":
                    values = []
                    for fd in path.iterdir():
                        try:
                            values.append(os.readlink(fd))
                        except FileNotFoundError:
                            pass
                else:
                    values = [path.read_bytes().decode("utf-8", "replace")]
                for value in values:
                    if related(value):
                        hits.append(dict(pid=int(folder.name), uid=identity, start=start, field=field, reference=value))
            except FileNotFoundError:
                pass
            except PermissionError:
                inaccessible.append(dict(pid=int(folder.name), uid=identity, start=start, field=field))
        try:
            require((folder / "stat").read_text().rsplit(")", 1)[1].split()[19] == start,
                    "PID generation changed during no-use scan")
        except FileNotFoundError:
            pass
    for name in sorted({p.name for p in proc_root.iterdir() if p.name.isdigit()} - initial):
        inaccessible.append(dict(pid=int(name), uid=None, field="new_process_after_scan"))
    return dict(hits=hits, inaccessible=inaccessible, all_uid_inspection_complete=not inaccessible,
                same_uid_inspection_complete=not any(row["uid"] in (None, SOURCE_UID) for row in inaccessible))


def write_new(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)


def remote_program(phase):
    functions = (require, canonical, encoded, sha, digest, stamp, census)
    program = "import hashlib,json,os,stat,shutil,sys\nfrom pathlib import Path\n"
    program += "\n".join(inspect.getsource(fn) for fn in functions)
    program += "\nREMOTE=" + repr(str(REMOTE)) + "\nPERSONAL=" + repr(str(REMOTE_PERSONAL)) + "\n"
    program += "data=json.loads(sys.stdin.buffer.read()); archive=canonical(REMOTE); personal=canonical(PERSONAL)\n"
    program += "require(set(data['source_archive'])==set(data['source_sha256']) and all(sha(bytes.fromhex(value))==data['source_sha256'][name] for name,value in data['source_archive'].items()), 'archived helper bytes differ from ROOT source pins')\n"
    program += "require(personal.is_dir() and personal.stat().st_uid==os.getuid(), 'private remote owner differs')\n"
    program += "\nfor p in (archive.parent,*archive.parent.parents):\n if p.is_relative_to(personal) and p.exists(): require(p.stat().st_uid==os.getuid() and not stat.S_IMODE(p.stat().st_mode)&0o022,'remote ancestor writable by another identity')\n"
    if phase == "create":
        program += "require(not archive.exists(), 'remote backup must be create-only')\n"
        program += "require(shutil.disk_usage(personal).free>=5*1024**3+128*1024**2+data['census']['file_bytes'], 'remote free floor/full backup reserve')\n"
        program += "missing=[]; p=archive.parent\nwhile not p.exists():\n require(p.is_relative_to(personal),'new ancestor outside private personal root'); missing.append(p); p=p.parent\n"
        program += "\nfor p in reversed(missing): p.mkdir(mode=0o700); canonical(p)\narchive.mkdir(mode=0o700)\n"
        program += "\nfor name,value in data.items():\n with (archive/(name+'.json')).open('xb') as f: f.write(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode())\n"
        program += "print(json.dumps({'created':True,'remote':REMOTE}))\n"
    else:
        program += "require(archive.stat().st_uid==os.getuid() and stat.S_IMODE(archive.stat().st_mode)==0o700, 'private remote 0700 backup required')\n"
        program += "require(set(p.name for p in archive.iterdir())==set(data['names'])|{'census.json','authority.json','source_sha256.json','references.json','source_archive.json'}, 'unexpected remote backup entry')\n"
        program += "\nfor name in ('census','authority','source_sha256','references','source_archive'):\n require((archive/(name+'.json')).read_bytes()==json.dumps(data[name],sort_keys=True,separators=(',',':'),allow_nan=False).encode(), 'remote provenance changed')\n"
        program += "result=census([archive/name for name in data['names']]); require(result['entries']==data['census']['entries'],'full remote content/mode/mtime/link/hardlink census differs'); print(json.dumps(result,sort_keys=True))\n"
    return SSH + [shlex.join(["env", "-u", "PYTHONPATH", "-u", "PYTHONHOME", "-u", "PYTHONUSERBASE",
                              "-u", "PYTHONSTARTUP", "PYTHONNOUSERSITE=1", "CUDA_VISIBLE_DEVICES=", str(PYTHON), "-I", "-B", "-c", program])]


def sources():
    return {str(path.relative_to(ROOT)): digest(path) for path in (Path(__file__), TEST)}


def authorization(args, inspection):
    document = read(args.authorization, args.authorization_sha256)
    require(document["schema"] == AUTH_SCHEMA and document["mode"] == args.mode
            and document["source_sha256"] == sources() and document["targets"] == list(TARGETS)
            and document["remote_backup"] == str(REMOTE) and document["inspection_sha256"] == digest(RECEIPT / "inspection.json")
            and document["payload_sha256"] == inspection["census"]["payload_sha256"]
            and document["generation_sha256"] == inspection["census"]["generation_sha256"]
            and document["tree_pins"] == tree_pins(inspection["census"])
            and document["declared_reference_sha256"] == inspection["references"]
            and document["current_root_declaration_set_complete"] is True,
            "separate ROOT authority must bind exact source/eight trees/generations/current declarations")
    return document


def removal_policy(authority, usage):
    require(not usage["hits"], "a process references a cleanup target")
    if usage["all_uid_inspection_complete"]:
        require(authority["process_policy"] == "ALL_UID_COMPLETE", "explicit complete-inspection deletion policy required")
    else:
        # HH_261002 - This is an external ROOT acceptance, never a fabricated all-UID absence assertion.
        require(authority["process_policy"] == "ROOT_ACCEPTS_EXACT_DISCLOSED_LIMITATIONS"
                and authority["accepted_process_usage_sha256"] == sha(encoded(usage))
                and authority["other_uid_absence_not_proven"] is True
                and authority["same_uid_incomplete_also_disclosed"] is (not usage["same_uid_inspection_complete"]),
                "incomplete /proc proof blocks removal unless ROOT separately accepts the exact disclosure")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("inspect", "backup", "remove"), required=True)
    parser.add_argument("--expected-tool-sha256", required=True)
    parser.add_argument("--reference-file", action="append", default=[])
    parser.add_argument("--authorization")
    parser.add_argument("--authorization-sha256")
    args = parser.parse_args(argv)
    require(digest(__file__) == args.expected_tool_sha256, "reviewed helper source required")
    canonical(RECEIPT)
    if args.mode == "inspect":
        require(not RECEIPT.exists() and not args.authorization and not args.authorization_sha256, "fresh read-only inspection receipt required")
        ancestor = next(p for p in RECEIPT.parents if p.exists())
        require(shutil.disk_usage(ancestor).free >= 5 * 1024 ** 3 + 128 * 1024 ** 2,
                "local 5GiB floor/manifest write reserve required; no deletion to bypass it")
        before = census(TARGETS, local=True)
        refs, usage = references(args.reference_file), process_usage()
        require(census(TARGETS, local=True) == before and references(args.reference_file) == refs, "inspection inputs changed")
        RECEIPT.mkdir(mode=0o700, parents=True)
        write_new(RECEIPT / "inspection.json", dict(schema=SCHEMA, census=before, references=refs, process_usage=usage,
                  source_sha256=sources(), deletion_authorized=False, remote_mutated=False))
        print(json.dumps(dict(inspection=str(RECEIPT / "inspection.json"), sha256=digest(RECEIPT / "inspection.json"),
                             payload_sha256=before["payload_sha256"], generation_sha256=before["generation_sha256"],
                             tree_pins=tree_pins(before), process_usage_sha256=sha(encoded(usage)),
                             process_hit_count=len(usage["hits"]), inaccessible_process_field_count=len(usage["inaccessible"]),
                             all_uid_inspection_complete=usage["all_uid_inspection_complete"],
                             same_uid_inspection_complete=usage["same_uid_inspection_complete"])))
        return 0
    require(args.authorization and args.authorization_sha256 and not args.reference_file, "externally pinned ROOT authority required")
    require(RECEIPT.stat().st_uid == os.getuid() and stat.S_IMODE(RECEIPT.stat().st_mode) == 0o700,
            "private current-UID 0700 receipt directory required")
    inspection = read(RECEIPT / "inspection.json")
    authority = authorization(args, inspection)
    expected = inspection["census"]
    require(census(TARGETS, local=True) == expected and references(inspection["references"]) == inspection["references"], "local trees/declarations changed since ROOT seal")
    require(sources() == inspection["source_sha256"], "reviewed source changed")
    require(not process_usage()["hits"], "current process reference blocks even backup stage")
    stage = RECEIPT / (args.mode + "_started.json")
    write_new(stage, dict(schema=SCHEMA, authority_sha256=args.authorization_sha256, source_sha256=sources()))
    def invoke(command, label, payload=None):
        result = subprocess.run(command, input=payload, capture_output=True, timeout=1800)
        for suffix, data in (("stdout", result.stdout), ("stderr", result.stderr)):
            with (RECEIPT / (label + "_" + suffix + ".txt")).open("xb") as stream:
                stream.write(data)
        require(result.returncode == 0, "backup command failed; preserve partial: " + label)
        return json.loads(result.stdout) if payload is not None else None
    try:
        package = dict(census=expected, authority=authority, source_sha256=sources(), references=inspection["references"],
                       source_archive={str(p.relative_to(ROOT)): p.read_bytes().hex() for p in (Path(__file__), TEST)})
        if args.mode == "backup":
            invoke(remote_program("create"), "remote_create", encoded(package))
            invoke(["rsync", "--archive", "--hard-links", "--numeric-ids", "--safe-links", "--protect-args", "--ignore-existing",
                    "-e", "ssh -o BatchMode=yes -o ConnectTimeout=15", "--", *TARGETS, "pro6000-training:" + str(REMOTE) + "/"], "rsync")
        else:
            require(digest(RECEIPT / "backup_complete.json") == authority["backup_receipt_sha256"], "ROOT-pinned verified full backup prerequisite")
            previous = read(RECEIPT / "backup_complete.json", authority["backup_receipt_sha256"])
            require(previous["local_removed"] is False and previous["census"] == expected, "completed full backup required")
            package["authority"] = previous["backup_authority"]
        package["names"] = [Path(path).name for path in TARGETS]
        first = invoke(remote_program("verify"), args.mode + "_remote_before", encoded(package))
        require(census(TARGETS, local=True) == expected and references(inspection["references"]) == inspection["references"], "full local pre/post backup pins differ")
        usage = process_usage()
        second = invoke(remote_program("verify"), args.mode + "_remote_after", encoded(package))
        require(first == second and sources() == inspection["source_sha256"]
                and digest(args.authorization) == args.authorization_sha256, "backup/generation/source/ROOT seal changed")
        if args.mode == "backup":
            write_new(RECEIPT / "backup_complete.json", dict(schema=SCHEMA, remote_backup=str(REMOTE), census=expected,
                      remote_census=second, backup_authority=authority, process_usage=usage, local_removed=False,
                      restoration_owner_ids_in_source_manifest=True, deletion_authorized=False))
        else:
            removal_policy(authority, usage)
            require(shutil.rmtree.avoids_symlink_attacks, "descriptor-based safe rmtree required")
            require(census(TARGETS, local=True) == expected and references(inspection["references"]) == inspection["references"], "last exact local/declaration pin check failed")
            removed = []
            for target in TARGETS:
                last = verify_single_target(target, expected)
                shutil.rmtree(target)
                removed.append(target)
                write_new(RECEIPT / ("removed_%02d.json" % len(removed)), dict(path=target, remote_backup=str(REMOTE / Path(target).name),
                          full_backup_payload_sha256=expected["payload_sha256"], exact_pre_removal_payload_sha256=last["payload_sha256"]))
            final = invoke(remote_program("verify"), "remove_remote_retained", encoded(package))
            require(final == second, "recovery backup changed after cleanup")
            write_new(RECEIPT / "removal_result.json", dict(schema=SCHEMA, removed=removed, remote_backup_retained=True,
                      process_usage=usage, all_uid_absence_claimed=usage["all_uid_inspection_complete"], backup_receipt_sha256=authority["backup_receipt_sha256"]))
    except Exception as error:
        write_new(RECEIPT / (args.mode + "_failure.json"), dict(schema=SCHEMA, error=type(error).__name__ + ": " + str(error),
                  partial_remote_backup_retained=True, completed_local_removal_receipts_retained=True))
        raise
    result_path = RECEIPT / ("backup_complete.json" if args.mode == "backup" else "removal_result.json")
    print(json.dumps(dict(status=args.mode.upper() + "_COMPLETE", receipt=str(result_path), receipt_sha256=digest(result_path),
                         remote_backup=str(REMOTE), process_usage_sha256=sha(encoded(usage)))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
