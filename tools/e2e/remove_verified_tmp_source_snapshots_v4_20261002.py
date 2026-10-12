"""HH_261002 - Remove only the eight fully backed-up trees under a fresh, externally pinned V4 authority.

Only proven UID0/Kthread:1 PID churn may differ. All userspace limitations,
actual target references, source generations and full backup bytes stay guarded.
Inspection does not use SSH; ROOT alone authorizes a single removal attempt.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess

from . import recoverable_tmp_source_snapshots_cleanup_v2_20261002 as frozen

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.remove_verified_tmp_source_snapshots.20261002.v4"
AUTH_SCHEMA = SCHEMA + ".ROOT_authority"
RECEIPT = ROOT / "storage/experiments/maintenance/2026-10-02/tmp_contract_snapshots_remove_v4"
TEST = ROOT / "learning/tests/test_remove_verified_tmp_source_snapshots_v4_20261002.py"
OLD_INSPECTION = frozen.RECEIPT / "inspection.json"
OLD_BACKUP = frozen.RECEIPT / "backup_complete.json"
OLD_INSPECTION_SHA = "d3ea0b5a93f2907a472fe653c2058da48bad7f318184cec150b5ae192b22102c"
OLD_BACKUP_SHA = "9b7ad93a8c40b790d8d3699c46ca1574cb5c20ba02f59e298cd6dd0141396ab2"
OLD_SOURCE_SHA = {
    "tools/e2e/recoverable_tmp_source_snapshots_cleanup_v2_20261002.py": "0843071c7c722f13757318a9a1b2972e75b70e871f12a94b8c3b53359b4d86c2",
    "learning/tests/test_recoverable_tmp_source_snapshots_cleanup_v2_20261002.py": "a82f768d0b93767aaf33192c90d5d2684fadb7471ee905c6b3d3dec69ef6628d",
}
REQUIRED_NEW_REFERENCES = tuple(ROOT / "storage/experiments/plans/2026-10-02/map_native_fusion_pair_v3" / name
    for name in ("ROOT_fixed20k_structure_pair_authorization.json", "ROOT_detached_launch_authorization.json"))
POLICY = "ROOT_ACCEPTS_ONLY_PROVEN_UID0_KTHREAD_CHURN"
require, canonical, digest, read, sha, encoded = (getattr(frozen, name)
    for name in ("require", "canonical", "digest", "read", "sha", "encoded"))


def sources():
    result = dict(OLD_SOURCE_SHA)
    require(all(digest(ROOT / name) == pin for name, pin in result.items()), "frozen V2 source or tests changed")
    result.update({str(path.relative_to(ROOT)): digest(path) for path in (Path(__file__), TEST)})
    require(len(result) == 4, "two original and two fresh externally pinned sources required")
    return dict(sorted(result.items()))


def original_evidence():
    sources()
    inspection = read(OLD_INSPECTION, OLD_INSPECTION_SHA)
    backup = read(OLD_BACKUP, OLD_BACKUP_SHA)
    authority = backup["backup_authority"]
    require(inspection["schema"] == backup["schema"] == frozen.SCHEMA
        and inspection["source_sha256"] == OLD_SOURCE_SHA and backup["census"] == inspection["census"]
        and backup["remote_census"]["entries"] == inspection["census"]["entries"]
        and backup["local_removed"] is False and backup["deletion_authorized"] is False
        and backup["remote_backup"] == str(frozen.REMOTE)
        and authority["schema"] == frozen.AUTH_SCHEMA and authority["mode"] == "backup"
        and authority["targets"] == list(frozen.TARGETS) and authority["source_sha256"] == OLD_SOURCE_SHA
        and authority["inspection_sha256"] == OLD_INSPECTION_SHA
        and authority["tree_pins"] == frozen.tree_pins(inspection["census"])
        and authority["declared_reference_sha256"] == inspection["references"],
        "exact verified original backup, source, full census and authority required")
    return inspection, backup


def current_references(files, old):
    refs = frozen.references(files)
    require(all(refs.get(path) == pin for path, pin in old["references"].items())
        and all(str(path) in refs for path in REQUIRED_NEW_REFERENCES),
        "all original references plus both current V3 map authorities required")
    return refs


def process_usage(proc_root=Path("/proc")):
    # HH_261002 - Kernel proof uses status flags and all four UIDs, never a command-name or UID-only whitelist.
    hits, inaccessible, processes, generation_errors = [], [], [], []
    initial = {p.name for p in proc_root.iterdir() if p.name.isdigit()}
    def metadata(folder):
        uid = folder.stat().st_uid
        start = (folder / "stat").read_text().rsplit(")", 1)[1].split()[19]
        require(start.isdecimal(), "kernel PID generation must be numeric")
        fields = {}
        try:
            fields = {line.split(":", 1)[0]: line.split(":", 1)[1].strip()
                for line in (folder / "status").read_text().splitlines() if ":" in line}
        except PermissionError:
            pass
        ids = fields.get("Uid", "").split()
        proven = uid == 0 and ids == ["0"] * 4 and fields.get("Kthread") == "1"
        return dict(pid=int(folder.name), uid=uid, start=start, kernel_thread_proven=proven,
            status_Kthread=fields.get("Kthread"), status_four_UIDs=ids)
    def limitation(meta, field):
        return {key: meta.get(key) for key in ("pid", "uid", "start", "kernel_thread_proven")} | {"field": field}
    for name in sorted(initial, key=int):
        folder = proc_root / name
        try:
            meta = metadata(folder); processes.append(meta)
        except FileNotFoundError:
            continue
        except (PermissionError, ValueError, IndexError) as error:
            row = dict(pid=int(name), uid=None, start=None, kernel_thread_proven=False, field="identity")
            inaccessible.append(row); generation_errors.append(dict(row, error=type(error).__name__)); continue
        for field in ("cwd", "root", "exe", "cmdline", "maps", "fd"):
            try:
                path = folder / field
                if field in ("cwd", "root", "exe"):
                    values = [os.readlink(path)]
                elif field == "fd":
                    values = []
                    for fd in path.iterdir():
                        try: values.append(os.readlink(fd))
                        except FileNotFoundError: pass
                else:
                    values = [path.read_bytes().decode("utf-8", "replace")]
                for value in values:
                    if any(target in value for target in frozen.TARGETS):
                        hits.append(dict(limitation(meta, field), reference=value))
            except FileNotFoundError:
                pass
            except PermissionError:
                inaccessible.append(limitation(meta, field))
        try:
            if (folder / "stat").read_text().rsplit(")", 1)[1].split()[19] != meta["start"]:
                generation_errors.append(dict(meta, error="PID_GENERATION_CHANGED"))
        except FileNotFoundError:
            pass
        except PermissionError:
            generation_errors.append(dict(meta, error="PID_GENERATION_UNREADABLE_AT_RECHECK"))
    # HH_261002 - New userspace arrivals remain a limitation and cannot be silently treated as a complete scan.
    for name in sorted({p.name for p in proc_root.iterdir() if p.name.isdigit()} - initial, key=int):
        try:
            folder = proc_root / name
            meta = metadata(folder); processes.append(meta)
            if (folder / "stat").read_text().rsplit(")", 1)[1].split()[19] != meta["start"]:
                generation_errors.append(dict(meta, error="NEW_PID_GENERATION_CHANGED"))
        except FileNotFoundError:
            continue
        except (PermissionError, ValueError, IndexError) as error:
            meta = dict(pid=int(name), uid=None, start=None, kernel_thread_proven=False)
            generation_errors.append(dict(meta, error=type(error).__name__))
        inaccessible.append(limitation(meta, "new_process_after_scan"))
    return dict(hits=hits, inaccessible=inaccessible, process_metadata=processes,
        PID_generation_validation_complete=not generation_errors, generation_errors=generation_errors,
        all_uid_inspection_complete=not inaccessible,
        same_uid_inspection_complete=not any(r["uid"] in (None, frozen.SOURCE_UID) for r in inaccessible),
        other_UID_absence_proven=False)


def userspace_limitations(usage):
    return sorted((dict(row) for row in usage["inaccessible"] if row["kernel_thread_proven"] is not True),
        key=lambda row: (row["pid"], str(row["uid"]), str(row["start"]), row["field"]))


def removal_policy(authority, usage, inspection):
    require(not usage["hits"] and usage["PID_generation_validation_complete"],
        "a target reference or incomplete PID generation blocks removal")
    expected = userspace_limitations(inspection["process_usage"])
    actual = userspace_limitations(usage)
    require(authority["process_policy"] == POLICY and authority["proven_UID0_Kthread1_PID_churn_accepted"] is True
        and authority["accepted_userspace_limitations"] == expected == actual
        and authority["accepted_userspace_limitations_sha256"] == sha(encoded(expected))
        and authority["other_uid_absence_not_proven"] is True
        and authority["same_uid_incomplete_also_disclosed"] is (not usage["same_uid_inspection_complete"]),
        "only proven UID0 Kthread1 churn may differ; every userspace limitation stays exactly pinned")


def authorization(path, expected, inspection):
    value = read(path, expected)
    require(value["schema"] == AUTH_SCHEMA and value["mode"] == "remove" and value["source_sha256"] == sources()
        and value["targets"] == list(frozen.TARGETS) and value["remote_backup"] == str(frozen.REMOTE)
        and value["original_inspection_sha256"] == OLD_INSPECTION_SHA and value["backup_receipt_sha256"] == OLD_BACKUP_SHA
        and value["fresh_inspection_sha256"] == digest(RECEIPT / "inspection.json")
        and value["payload_sha256"] == inspection["census"]["payload_sha256"]
        and value["generation_sha256"] == inspection["census"]["generation_sha256"]
        and value["tree_pins"] == frozen.tree_pins(inspection["census"])
        and value["declared_reference_sha256"] == inspection["references"]
        and value["current_root_declaration_set_complete"] is True and value["deletion_authorized"] is True
        and value["user_approved_full_backup_then_exact8_removal"] is True,
        "separate ROOT authority must pin both sources, original verified backup and fresh exact scope")
    return value


def record_usage(name):
    try:
        usage = process_usage()
    except Exception as error:
        # HH_261002 - A failed scan is retained as unavailable evidence, never as an invented empty process list.
        frozen.write_new(RECEIPT / (name + ".scan_failure.json"), dict(error_type=type(error).__name__,
            error=str(error), observed_process_usage_available=False))
        raise
    frozen.write_new(RECEIPT / name, usage)
    return usage


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("inspect", "remove"), required=True)
    parser.add_argument("--expected-tool-sha256", required=True)
    parser.add_argument("--reference-file", action="append", default=[])
    parser.add_argument("--authorization"); parser.add_argument("--authorization-sha256")
    args = parser.parse_args(argv)
    require(digest(__file__) == args.expected_tool_sha256, "externally reviewed new helper required")
    canonical(RECEIPT)
    old, backup = original_evidence(); expected = old["census"]
    if args.mode == "inspect":
        require(not RECEIPT.exists() and not args.authorization and not args.authorization_sha256,
            "fresh read-only inspection namespace required")
        require(shutil.disk_usage(ROOT).free >= 5 * 1024 ** 3 + 128 * 1024 ** 2, "unchanged local manifest floor required")
        require(frozen.census(frozen.TARGETS, local=True) == expected, "all original tree bytes and generations changed")
        refs = current_references(args.reference_file, old); usage = process_usage()
        require(frozen.census(frozen.TARGETS, local=True) == expected and current_references(refs, old) == refs,
            "inspection tree or declaration changed")
        RECEIPT.mkdir(parents=True, mode=0o700)
        frozen.write_new(RECEIPT / "inspection.json", dict(schema=SCHEMA, census=expected, references=refs,
            process_usage=usage, source_sha256=sources(), original_inspection_sha256=OLD_INSPECTION_SHA,
            backup_receipt_sha256=OLD_BACKUP_SHA, deletion_authorized=False, remote_mutated=False))
        print(json.dumps(dict(status="INSPECTION_ONLY_NO_SSH_NO_REMOVAL", inspection_sha256=digest(RECEIPT / "inspection.json"),
            userspace_limitations_sha256=sha(encoded(userspace_limitations(usage))), process_hit_count=len(usage["hits"]),
            inaccessible_count=len(usage["inaccessible"]), source_sha256=sources())))
        return 0
    require(args.authorization and args.authorization_sha256 and not args.reference_file, "external ROOT removal authority required")
    require(RECEIPT.stat().st_uid == os.getuid() and stat.S_IMODE(RECEIPT.stat().st_mode) == 0o700,
        "private current-UID 0700 fresh receipt directory required")
    inspection = read(RECEIPT / "inspection.json")
    require(inspection["schema"] == SCHEMA and inspection["census"] == expected and inspection["source_sha256"] == sources(),
        "fresh inspection source or entire original backup differs")
    authority = authorization(args.authorization, args.authorization_sha256, inspection)
    require(not (RECEIPT / "remove_started.json").exists(), "one declared attempt only; no retry or overwrite")
    frozen.write_new(RECEIPT / "remove_started.json", dict(schema=SCHEMA, authority_sha256=args.authorization_sha256, source_sha256=sources()))
    removed = []
    def check():
        require(sources() == inspection["source_sha256"] and digest(args.authorization) == args.authorization_sha256
            and digest(OLD_INSPECTION) == OLD_INSPECTION_SHA and digest(OLD_BACKUP) == OLD_BACKUP_SHA
            and current_references(inspection["references"], old) == inspection["references"], "source, authority, backup or references changed")
    def remote(label):
        # HH_261002 - Read-only remote verification reuses the original provenance verbatim; no archive creation or rsync exists here.
        package = dict(census=expected, authority=backup["backup_authority"], source_sha256=OLD_SOURCE_SHA,
            references=old["references"], source_archive={name: (ROOT / name).read_bytes().hex() for name in OLD_SOURCE_SHA},
            names=[Path(path).name for path in frozen.TARGETS])
        result = subprocess.run(frozen.remote_program("verify"), input=encoded(package), capture_output=True, timeout=1800)
        for suffix, payload in (("stdout", result.stdout), ("stderr", result.stderr)):
            with (RECEIPT / (label + "_" + suffix + ".txt")).open("xb") as stream: stream.write(payload)
        require(result.returncode == 0, "original full remote backup verification failed")
        actual = json.loads(result.stdout)
        require(actual["entries"] == expected["entries"], "remote full payload, mode, mtime or links differ")
        return actual
    try:
        usage = record_usage("observed_process_usage_00_before_remote.json"); removal_policy(authority, usage, inspection)
        check(); require(frozen.census(frozen.TARGETS, local=True) == expected, "original local bytes or generations changed")
        first = remote("remove_remote_before")
        require(frozen.census(frozen.TARGETS, local=True) == expected, "full local census changed between remote checks")
        second = remote("remove_remote_after"); require(first == second, "remote backup generation changed")
        usage = record_usage("observed_process_usage_01_after_remote.json"); removal_policy(authority, usage, inspection)
        check(); require(shutil.rmtree.avoids_symlink_attacks, "descriptor-based safe rmtree required")
        require(frozen.census(frozen.TARGETS, local=True) == expected, "last entire local census changed")
        for index, target in enumerate(frozen.TARGETS, 1):
            last = frozen.verify_single_target(target, expected)
            check(); usage = record_usage("observed_process_usage_%02d_before_target.json" % (index + 1))
            removal_policy(authority, usage, inspection)
            # HH_261002 - Recheck every sealed declaration and complete target bytes after the process scan, immediately before removal.
            check(); last = frozen.verify_single_target(target, expected)
            shutil.rmtree(target); removed.append(target)
            frozen.write_new(RECEIPT / ("removed_%02d.json" % index), dict(path=target,
                remote_backup=str(frozen.REMOTE / Path(target).name), full_backup_payload_sha256=expected["payload_sha256"],
                exact_pre_removal_payload_sha256=last["payload_sha256"], userspace_limitations_sha256=sha(encoded(userspace_limitations(usage)))))
        final = remote("remove_remote_retained"); require(final == second, "recovery backup changed after cleanup")
        check()
        frozen.write_new(RECEIPT / "removal_result.json", dict(schema=SCHEMA, removed=removed, remote_backup_retained=True,
            backup_receipt_sha256=OLD_BACKUP_SHA, process_usage=usage, all_uid_absence_claimed=False,
            userspace_limitations_exactly_preserved=True, proven_UID0_Kthread1_PID_churn_only=True))
    except Exception as error:
        frozen.write_new(RECEIPT / "remove_failure.json", dict(schema=SCHEMA, error=type(error).__name__ + ": " + str(error),
            removed=removed, observed_process_usage_preserved=any(RECEIPT.glob("observed_process_usage_*.json")),
            original_backup_and_failure_receipts_unchanged=True))
        raise
    print(json.dumps(dict(status="EXACT_EIGHT_REMOVED_RECOVERABLE", receipt_sha256=digest(RECEIPT / "removal_result.json"),
        remote_backup=str(frozen.REMOTE), all_uid_absence_claimed=False)))
    return 0


if __name__ == "__main__": raise SystemExit(main())
