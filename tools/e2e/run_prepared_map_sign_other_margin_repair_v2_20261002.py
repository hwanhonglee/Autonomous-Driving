"""HH_261002 - NEW dispatch identity only: preserve12 TRAIN sources and retain the failed v1 authority/receipts."""
from __future__ import annotations

from contextlib import contextmanager
import hashlib
import json
from pathlib import Path

from . import run_prepared_map_sign_other_margin_repair_20261002 as previous

PARENT_SHA="432d2172ea95220af0ea069b2ff513a2c1202d1e0f5e98b76278d9d233d2f800"
PARENT_FILE=Path(previous.__file__)
OUTPUT=previous.ROOT/"storage/experiments/training/2026-10-02/map_sign_other_margin_fit_dispatch_v2"
REMOTE_AUTH=previous.REMOTE_PLAN/"authorization_v2.json"
SCHEMA="portable_e2e.map_sign_other_margin_explicit_ROOT_fit_dispatch.20261002.v2"
require,digest=previous.require,previous.digest
_identity,_write=previous.identities,previous.write_new
_old_output,_old_auth,_old_schema,_old_check=previous.OUTPUT,previous.REMOTE_AUTH,previous.SCHEMA,previous.REMOTE_CHECK
_active=False


def identities(args):
    result=_identity(args)
    value,auth,_=result
    # HH_261002 - Python dict equality masks int5 versus float5.0; retain the frozen validator's exact JSON comparison.
    expected=json.dumps(value["fit_contract"],sort_keys=True,allow_nan=False)
    require(json.dumps(auth["contract"],sort_keys=True,allow_nan=False)==expected,
        "authority contract JSON numeric representation differs; preserve original float5.0, never normalize math")
    return result


def write_new(path,value):
    if path==OUTPUT/"declaration.json":
        own_sha=digest(Path(__file__))
        value=dict(value,compatibility_dispatcher_sha256=own_sha,frozen_parent_dispatcher_sha256=PARENT_SHA,
            dispatch_revision_only=True,TRAIN_source_count_unchanged=12,
            v1_failure_authority_and_raw_receipts_preserved=True,
            contract_JSON_numeric_representation_must_match_exactly=True)
        archive=OUTPUT/"compatibility_source";archive.mkdir()
        with (archive/Path(__file__).name).open("xb") as stream:stream.write(Path(__file__).read_bytes())
    if path==OUTPUT/"source_postcheck.json":
        require(digest(PARENT_FILE)==PARENT_SHA
            and digest(Path(__file__))==digest(OUTPUT/"compatibility_source"/Path(__file__).name),
            "frozen original/new dispatch source changed")
    return _write(path,value)


@contextmanager
def activated():
    global _active
    require(digest(PARENT_FILE)==PARENT_SHA,"frozen failed v1 dispatcher changed")
    require(not _active and previous.OUTPUT is _old_output and previous.REMOTE_AUTH is _old_auth
        and previous.SCHEMA==_old_schema and previous.REMOTE_CHECK==_old_check
        and previous.identities is _identity and previous.write_new is _write,"unknown or nested v2 fit dispatch scope")
    literal=str(_old_auth.relative_to(previous.WORKSPACE))
    require(_old_check.count(literal)==1,"one exact frozen remote authority-path literal required")
    # HH_261002 - Only the fresh authority filename changes; no source/model/GPU/data/evaluation predicate changes.
    changes=dict(OUTPUT=OUTPUT,REMOTE_AUTH=REMOTE_AUTH,SCHEMA=SCHEMA,
        REMOTE_CHECK=_old_check.replace(literal,str(REMOTE_AUTH.relative_to(previous.WORKSPACE))),identities=identities,write_new=write_new)
    saved={key:getattr(previous,key) for key in changes};_active=True
    try:
        for key,value in changes.items():setattr(previous,key,value)
        yield
    finally:
        for key,value in saved.items():setattr(previous,key,value)
        _active=False


def main(argv=None):
    with activated():return previous.main(argv)


if __name__=="__main__":raise SystemExit(main())
