"""HH_261008 - Read externally curated local status without stale-job guessing.

This is metadata verification, not input/model replay, feature approval or a
training monitor. No SSH, subprocess, framework, installation or file writes.
An old started receipt without a boot identifier cannot prove RUNNING.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import sys

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
SCHEMA = "portable_e2e.workspace_status.20261008.v1"
TOOL_NAMES = (
    "tools/maintenance/report_workspace_status_20261008_v1.py",
    "learning/tests/test_report_workspace_status_20261008_v1.py",
)
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
INPUT_LIMIT = 4 * 1024**2
INPUT_COUNT_LIMIT = 32
OUTPUT_LIMIT = 256 * 1024
PROC_LIMIT = 64 * 1024
IDENTITY_KEYS = {"pid", "starttime", "ppid", "pgid", "session", "uid", "argv", "boot_id"}
STAT_FIELDS = ("st_mode", "st_uid", "st_gid", "st_nlink", "st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")


def need(value, message):
    if not value:
        raise ValueError(message)


def sha64(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")


def unique_object(items):
    value = {}
    for key, item in items:
        need(key not in value, "duplicate JSON key")
        value[key] = item
    return value


def decoded(raw):
    need(type(raw) is bytes and len(raw) <= INPUT_LIMIT, "bounded metadata bytes required")
    return json.loads(raw, object_pairs_hook=unique_object,
        parse_constant=lambda _: need(False, "nonfinite JSON denied"))


def canonical(path):
    need(type(path) is str, "exact path string required")
    item = Path(path)
    need(item.is_absolute() and item == item.resolve(strict=True)
        and item.is_relative_to(ROOT) and not any(p.is_symlink() for p in (item, *item.parents)),
        "canonical workspace path without symlink required")
    return item


class MetadataReader:
    """HH_261008 - Retain per-read byte seals, not a claim of continuous immutability."""
    def __init__(self):
        self.records = {}

    def read(self, reference, *, json_document=True, allowed_suffixes=(".json",)):
        need(type(reference) is dict and set(reference) == {"path", "sha256"}
            and sha64(reference["sha256"]), "exact RAW metadata reference required")
        path = canonical(reference["path"])
        need(path.suffix in allowed_suffixes, "metadata-only suffix required")
        need(str(path) in self.records or len(self.records) < INPUT_COUNT_LIMIT, "maximum32 input files exceeded")
        before = path.lstat()
        need(stat.S_ISREG(before.st_mode) and before.st_nlink == 1 and before.st_size <= INPUT_LIMIT,
            "single-link bounded regular metadata required")
        with path.open("rb") as stream:
            opened = os.fstat(stream.fileno())
            raw = stream.read(INPUT_LIMIT + 1)
            after_fd = os.fstat(stream.fileno())
        after = path.lstat()
        fingerprint = tuple(getattr(before, name) for name in STAT_FIELDS)
        need(all(tuple(getattr(item, name) for name in STAT_FIELDS) == fingerprint
            for item in (opened, after_fd, after)) and len(raw) == before.st_size,
            "metadata generation changed during read")
        pin = hashlib.sha256(raw).hexdigest()
        need(pin == reference["sha256"], "RAW metadata SHA differs")
        record = dict(path=str(path), sha256=pin, bytes=len(raw), stat_fingerprint=list(fingerprint))
        need(str(path) not in self.records or self.records[str(path)] == record, "metadata changed between references")
        self.records[str(path)] = record
        value = decoded(raw) if json_document else raw
        if json_document:
            need(type(value) is dict, "JSON object required")
        return value


def value_at(document, keys):
    need(type(keys) is list and 1 <= len(keys) <= 8 and all(type(k) is str and k for k in keys),
        "bounded exact metadata key path required")
    value = document
    for key in keys:
        need(type(value) is dict and key in value, "declared metadata key missing")
        value = value[key]
    return value


def read_small(path, limit=PROC_LIMIT):
    with Path(path).open("rb") as stream:
        raw = stream.read(limit + 1)
    need(len(raw) <= limit, "bounded process evidence required")
    return raw


def process_identity(pid):
    need(type(pid) is int and pid > 0, "nonboolean positive PID required")
    base = Path("/proc") / str(pid)
    first = read_small(base / "stat").decode("utf-8")
    fields = first[first.rfind(")") + 2:].split()
    need(len(fields) >= 20 and fields[0] not in ("Z", "X"), "zombie/exited PID is not RUNNING")
    argv = [part.decode("utf-8") for part in read_small(base / "cmdline").split(b"\0") if part]
    need(argv, "empty process argv")
    identity = dict(pid=pid, starttime=int(fields[19]), ppid=int(fields[1]), pgid=int(fields[2]),
        session=int(fields[3]), uid=base.stat().st_uid, argv=argv,
        boot_id=read_small("/proc/sys/kernel/random/boot_id", 128).decode("ascii").strip())
    second = read_small(base / "stat").decode("utf-8")
    later = second[second.rfind(")") + 2:].split()
    need(len(later) >= 20 and later[0] not in ("Z", "X") and fields[1:4] == later[1:4]
        and fields[19] == later[19], "process generation changed during observation")
    return identity


def lease_probe(identity, receipt):
    """HH_261008 - Observe only the declared owner's FD; never acquire/signal anything."""
    number = receipt["lock_fd"]
    declared = receipt["lock_identity"]
    need(type(number) is int and number >= 3 and type(declared) is dict and set(declared) == {"dev", "ino"}
        and all(type(n) is int and n >= 0 for n in declared.values()), "exact lease FD identity required")
    path = canonical(str(LOCK)); expected = path.lstat()
    need(stat.S_ISREG(expected.st_mode) and expected.st_nlink == 1
        and (expected.st_dev, expected.st_ino) == (declared["dev"], declared["ino"]), "declared runtime lock replaced")
    fd = Path("/proc") / str(identity["pid"]) / "fd" / str(number)
    need(os.readlink(fd) == str(path), "owner FD no longer refers to the exact runtime lease")
    actual = fd.stat()
    need((actual.st_dev, actual.st_ino) == (expected.st_dev, expected.st_ino), "owner FD lease generation differs")
    info = read_small(fd.parent.parent / "fdinfo" / str(number)).decode("ascii")
    locks = [line.split() for line in info.splitlines() if line.startswith("lock:")]
    device = f"{os.major(expected.st_dev):02x}:{os.minor(expected.st_dev):02x}:{expected.st_ino}"
    need(len(locks) == 1 and len(locks[0]) == 9 and locks[0][2:5] == ["FLOCK", "ADVISORY", "WRITE"]
        and locks[0][5] == str(identity["pid"]) and locks[0][6:] == [device, "0", "EOF"],
        "declared private owner FLOCK evidence unavailable or differs")
    later = path.lstat(); after_fd = fd.stat()
    need((later.st_dev, later.st_ino) == (after_fd.st_dev, after_fd.st_ino) == (expected.st_dev, expected.st_ino),
        "lease changed during observation")
    return dict(fd=number, dev=expected.st_dev, ino=expected.st_ino, observation="OWNER_FD_FLOCK_SNAPSHOT_ONLY")


def live_status(reader, live, evidence):
    if live is None:
        return dict(status="UNKNOWN_ACTIVE_OR_STOPPED", reason="No independently boot-bound live receipt declared.")
    need(type(live) is dict and set(live) == {"started"}, "exact live receipt declaration required")
    receipt = reader.read(live["started"])
    identity = receipt.get("identity")
    if type(identity) is not dict or "boot_id" not in identity:
        return dict(status="UNKNOWN_ACTIVE_OR_STOPPED", reason="Historical started receipt lacks boot binding; not manufactured retrospectively.")
    need(set(identity) == IDENTITY_KEYS and all(type(identity[k]) is int and identity[k] >= 0
        for k in IDENTITY_KEYS - {"argv", "boot_id"}) and identity["pid"] > 0 and identity["starttime"] > 0,
        "exact nonboolean generation identity required")
    need(type(identity["argv"]) is list and identity["argv"] and all(type(v) is str for v in identity["argv"])
        and type(identity["boot_id"]) is str and bool(identity["boot_id"]), "exact argv/boot receipt required")
    need(identity["uid"] == os.getuid(), "only this user's explicitly declared private process may be observed")
    need(receipt.get("plan_sha256") == evidence["plan"]["sha256"]
        and receipt.get("authority_sha256") == evidence["authority"]["sha256"], "live receipt RAW plan/authority join differs")
    observation_stage = "generation_before_lease"
    try:
        first = process_identity(identity["pid"])
        need(first == identity, "actual PID/boot/generation/argv differs")
        observation_stage = "lease"
        lease = lease_probe(identity, receipt)
        observation_stage = "generation_after_lease"
        need(process_identity(identity["pid"]) == first, "live generation changed across lease observation")
        return dict(status="RUNNING", identity=first, lease=lease, scope="FRESH_POINT_IN_TIME_NOT_CONTINUOUS_LIVENESS")
    except FileNotFoundError:
        return dict(status="UNKNOWN_STOPPED" if observation_stage == "generation_before_lease" else "UNKNOWN_ACTIVE_OR_STOPPED",
            reason="Declared generation/FD not observable at " + observation_stage + "; absence alone does not prove completion.")
    except (OSError, ValueError, UnicodeError) as error:
        return dict(status="UNKNOWN_ACTIVE_OR_STOPPED", reason=type(error).__name__ + ": " + str(error)[:512])


def entry_status(reader, entry):
    need(type(entry) is dict and set(entry) == {"name", "mode", "declared_outcome", "evidence", "checks", "live"},
        "exact curated status entry required")
    need(type(entry["name"]) is str and 0 < len(entry["name"]) <= 128
        and entry["mode"] in ("completed", "prepared", "active")
        and entry["declared_outcome"] in ("FAIL", "DIAGNOSTIC_ONLY", "PENDING"), "curated name/mode/outcome required")
    evidence = entry["evidence"]
    required = {"plan", "authority", "result", "postcheck"} if entry["mode"] == "completed" else ({"plan"} if entry["mode"] == "prepared" else {"plan", "authority"})
    need(type(evidence) is dict and set(evidence) == required, "exact named evidence roles required")
    need(type(entry["checks"]) is list and len(entry["checks"]) <= 16, "bounded curated metadata assertions required")
    documents = {role: reader.read(reference) for role, reference in evidence.items()}
    if entry["mode"] == "active":
        need(documents["authority"].get("plan_sha256") == evidence["plan"]["sha256"]
            and documents["authority"].get("plan_path") == evidence["plan"]["path"],
            "active RAW authority/plan identity differs")
    for check in entry["checks"]:
        need(type(check) is dict and set(check) == {"document", "keys", "expected"} and check["document"] in documents,
            "exact curated scalar assertion required")
        need(encoded(value_at(documents[check["document"]], check["keys"])) == encoded(check["expected"]),
            "declared metadata assertion differs; numeric types are not normalized")
    if entry["mode"] == "prepared":
        need(entry["live"] is None and entry["declared_outcome"] == "PENDING", "prepared entry cannot claim execution")
        return dict(name=entry["name"], status="PREPARED_NOTRUNNING", outcome="PENDING", evidence=evidence)
    observed = live_status(reader, entry["live"], evidence)
    if entry["mode"] == "active":
        need(entry["declared_outcome"] == "PENDING", "active entry outcome is pending")
        return dict(name=entry["name"], outcome="PENDING", evidence=evidence, **observed)
    need({check["document"] for check in entry["checks"]} >= {"result", "postcheck"}, "completed result and postcheck assertions required")
    status = "UNKNOWN_ACTIVE_COMPLETION_CONFLICT" if observed["status"] == "RUNNING" else "STORED_COMPLETED_WITH_VERIFIED_METADATA"
    return dict(name=entry["name"], status=status, outcome=entry["declared_outcome"], evidence=evidence,
        live_observation=observed, completion_scope="EXTERNAL_CURATED_RAW_RESULT_AND_POSTCHECK_PINS_NOT_FULL_INPUT_REHASH_OR_REPEAT")


def report(manifest_path, manifest_sha256):
    need(HERE == ROOT / TOOL_NAMES[0] and canonical(str(HERE)) == HERE, "exact fresh helper pathname required")
    reader = MetadataReader()
    manifest = reader.read(dict(path=manifest_path, sha256=manifest_sha256))
    need(set(manifest) == {"schema", "provenance", "tool_sha256", "curated_index", "entries"}
        and manifest["schema"] == SCHEMA and type(manifest["provenance"]) is str
        and manifest["provenance"].startswith("HH_261008 - "), "externally curated manifest schema required")
    sources = manifest["tool_sha256"]
    need(type(sources) is dict and set(sources) == set(TOOL_NAMES), "exact fresh helper/test source closure required")
    for name, pin in sources.items():
        reader.read(dict(path=str(ROOT / name), sha256=pin), json_document=False, allowed_suffixes=(".py",))
    index = manifest["curated_index"]
    need(type(index) is dict and set(index) == {"reference", "main_approved", "sub_approved", "scope"}
        and type(index["main_approved"]) is int and index["main_approved"] == 0
        and type(index["sub_approved"]) is int and index["sub_approved"] == 0
        and index["scope"] == "CURATED_INDEX_NOT_TEST_PASS_OR_AUTOMATIC_PROMOTION", "explicit curated0/9 and0/30 required")
    reader.read(index["reference"], json_document=False, allowed_suffixes=(".md",))
    entries = manifest["entries"]
    need(type(entries) is list and 1 <= len(entries) <= 6, "bounded explicitly named latest entries required")
    need(all(type(entry) is dict and type(entry.get("name")) is str for entry in entries)
        and len({entry["name"] for entry in entries}) == len(entries), "unique declared latest names required")
    values = []
    for entry in entries:
        try:
            values.append(entry_status(reader, entry))
        except (OSError, ValueError, KeyError, TypeError, UnicodeError) as error:
            values.append(dict(name=entry["name"], status="UNKNOWN_STORED_EVIDENCE", reason=type(error).__name__ + ": " + str(error)[:512]))
    value = dict(schema=SCHEMA + ".report", provenance="HH_261008 - Verify only externally pinned named local metadata; no stale September fallback.",
        formal_main_approved=0, formal_main_total=9, formal_sub_approved=0, formal_sub_total=30,
        formal_count_scope=index["scope"], curated_index=index["reference"], entries=values,
        metadata_reads=list(reader.records.values()), verified_metadata_file_count=len(reader.records),
        interpreter=dict(executable=sys.executable, version=sys.version, scope="OBSERVED_STDLIB_PROCESS_NOT_WHOLE_PROVIDER_EQUIVALENCE"),
        full_input_or_model_rehash=False, independent_experiment_repeat=False, training_or_NN_or_ROS_started=False,
        remote_checked=False, automatic_latest_discovery=False, model_promoted=False, files_written=False,
        containment="BOUNDED_METADATA_AND_OUTPUT_NOT_OS_CONTAINMENT_OR_HARD_PREEMPTION")
    need(len(encoded(value)) <= OUTPUT_LIMIT, "complete256KiB status output exceeded; no truncation")
    return value


def manifest_template():
    # HH_261008 - Unfilled skeleton is never an authority or executable job declaration.
    return dict(schema=SCHEMA, provenance="HH_261008 - ROOT must fill and externally seal this manual latest-status manifest.",
        tool_sha256={name: "ROOT_MUST_FILL_SHA256" for name in TOOL_NAMES},
        curated_index=dict(reference=dict(path=str(ROOT / "docs/assets/validation/2026-10-02/README.md"), sha256="ROOT_MUST_FILL_SHA256"),
            main_approved=0, sub_approved=0, scope="CURATED_INDEX_NOT_TEST_PASS_OR_AUTOMATIC_PROMOTION"),
        entries=[dict(name="ROOT_MUST_NAME_LATEST_COMPLETED_LOCAL_RECORD", mode="completed", declared_outcome="FAIL",
            evidence={role: dict(path="ROOT_MUST_FILL_CANONICAL_JSON_PATH", sha256="ROOT_MUST_FILL_SHA256")
                for role in ("plan", "authority", "result", "postcheck")},
            checks=[dict(document="result", keys=["status"], expected="ROOT_MUST_FILL_EXACT_STORED_STATUS"),
                dict(document="postcheck", keys=["status"], expected="ROOT_MUST_FILL_EXACT_POST_STATUS")], live=None)])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--manifest")
    parser.add_argument("--manifest-sha256")
    parser.add_argument("--template", action="store_true")
    args = parser.parse_args(argv)
    if args.template:
        need(args.manifest is None and args.manifest_sha256 is None, "template is separate from status verification")
        value = manifest_template()
    else:
        need(args.manifest is not None and sha64(args.manifest_sha256), "external RAW manifest path/SHA required")
        value = report(args.manifest, args.manifest_sha256)
    raw = encoded(value); need(len(raw) <= OUTPUT_LIMIT, "bounded full output required")
    sys.stdout.buffer.write(raw); sys.stdout.buffer.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
