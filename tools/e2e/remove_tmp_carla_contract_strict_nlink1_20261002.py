"""HH_261002 - Remove-only full-generation guard for the externally inspected four nlink1 trees; no backup reinterpretation."""
from __future__ import annotations

import argparse
from pathlib import Path

from scripts.e2e import recoverable_tmp_carla_contract_cleanup_20261002 as parent

SCHEMA="portable_e2e.recoverable_tmp_contract_strict_nlink1_remove.20261002.v1"
PARENT_SHA="fd3baf3042f279174cd78cc8e552f656606247dec11e3efdca0afc34b9d328e8"
PARENT_TEST_SHA="b83ad0b13f7156f8572370f60e34e644c37d05331163c0635efa8f93282a800b"
TEST=parent.ROOT/"learning/tests/test_remove_tmp_carla_contract_strict_nlink1_20261002.py"
REGULAR_COUNT=25096
_original_verify,_main=parent.verify_single_target,parent.main
_active=False


def sources():
    return {str(path.relative_to(parent.ROOT)):parent.digest(path) for path in (Path(__file__),TEST)}


def strict_verify_single_target(target,expected):
    # HH_261002 - No ctime/nlink exception exists: even identical bytes restored under the same inode must match the complete generation.
    parent.require(str(target) in parent.TARGETS,"only the original exact four targets are supported")
    files=[key for key,row in expected["entries"].items() if row["kind"]=="file"]
    parent.require(len(files)==REGULAR_COUNT and all(expected["identities"][key][-1]==1 for key in files),
        "exact inspected25096 nlink1 regular files required; hardlinks are unsupported in this revision")
    prefix=Path(target).name
    wanted={key:row for key,row in expected["entries"].items() if key.split("/",1)[0]==prefix}
    identities={key:row for key,row in expected["identities"].items() if key.split("/",1)[0]==prefix}
    actual=parent.census([target],local=True)
    parent.require(actual["entries"]==wanted and actual["identities"]==identities,
        "STRICT full per-target payload/inode/owner/mode/mtime/ctime/nlink generation changed")
    return actual


def main(argv=None):
    global _active
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--expected-tool-sha256",required=True)
    parser.add_argument("--authorization",type=Path,required=True);parser.add_argument("--authorization-sha256",required=True)
    args=parser.parse_args(argv)
    parent.require(parent.digest(__file__)==args.expected_tool_sha256 and parent.digest(parent.__file__)==PARENT_SHA
        and parent.digest(parent.TEST)==PARENT_TEST_SHA,"reviewed new revision and frozen parent/test bytes required")
    value=parent.read(args.authorization,args.authorization_sha256);pins=sources()
    parent.require(value.get("schema")==parent.AUTH_SCHEMA and value.get("mode")=="remove"
        and value.get("remove_revision_schema")==SCHEMA and value.get("remove_revision_authorized") is True
        and value.get("remove_revision_source_sha256")==pins and type(value.get("remove_revision_regular_file_count")) is int
        and value["remove_revision_regular_file_count"]==REGULAR_COUNT and value.get("remove_revision_nlink_must_equal_one") is True,
        "separate ROOT remove-only revision/source/count/nlink authority required")
    parent.require(not _active and parent.verify_single_target is _original_verify,"non-nested original cleanup provider required")
    def verified(target,expected):
        actual=strict_verify_single_target(target,expected)
        parent.require(sources()==pins and parent.digest(args.authorization)==args.authorization_sha256,"new revision/source/authority changed before removal")
        return actual
    _active=True;parent.verify_single_target=verified
    try:
        result=_main(["--mode","remove","--expected-tool-sha256",PARENT_SHA,"--authorization",str(args.authorization),
            "--authorization-sha256",args.authorization_sha256])
        parent.require(sources()==pins and parent.digest(args.authorization)==args.authorization_sha256,"new revision/source/authority changed after cleanup")
        return result
    finally:parent.verify_single_target=_original_verify;_active=False


if __name__=="__main__":raise SystemExit(main())
