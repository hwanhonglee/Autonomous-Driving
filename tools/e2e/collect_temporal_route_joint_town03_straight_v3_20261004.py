"""HH_261004 - Admit the fresh straight owner through the unchanged owned Town03 Green fixture."""

from __future__ import annotations

from contextlib import contextmanager
import hashlib
from pathlib import Path

from . import collect_temporal_route_joint_town03_straight_v2_20261004 as previous


# HH_261004 - V3 changes only the owner-schema aliases that the inherited fixture reads at installation time.
straight = previous.straight
fixture = straight.fixture
frozen, collector, ROOT = previous.frozen, previous.collector, previous.ROOT
SEED, ROUTE_SHA, ROUTE_LENGTH = previous.SEED, previous.ROUTE_SHA, previous.ROUTE_LENGTH
ROUTE_POINTS = previous.ROUTE_POINTS
ENTRY, EXIT, EXPECTED_TURN_DEG = previous.ENTRY, previous.EXIT, previous.EXPECTED_TURN_DEG
PREVIEW_SCHEMA, THREAD_SCHEMA = previous.PREVIEW_SCHEMA, previous.THREAD_SCHEMA
RightTurnObserver = previous.RightTurnObserver
StraightIntervalObserver = previous.StraightIntervalObserver
SCHEMA = "portable_e2e.actual_route_joint_preview05_town03_known_dev_straight_fixed45.20261004.v3"
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_straight_fixed45.20261004.v3"
PARENT_SHA = "0286de67c7bf16497f311a16bf0b9d9d5406fedda55108a085cfa604052bd574"
_PARENT_SCHEMA, _PARENT_OWNER_SCHEMA = previous.SCHEMA, previous.OWNER_SCHEMA
_STRAIGHT_SCHEMA, _STRAIGHT_OWNER_SCHEMA = straight.SCHEMA, straight.OWNER_SCHEMA
_FIXTURE_SCHEMA, _FIXTURE_OWNER_SCHEMA = fixture.SCHEMA, fixture.OWNER_SCHEMA
_active = False


def verify_frozen():
    """HH_261004 - Preserve V2's deferred recorder, controller, model, route, and geometry bytes."""
    frozen._require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == PARENT_SHA,
                    "archived Town03 straight v2 collector changed")
    previous.verify_frozen()


def validate_capture(args, route):
    previous.validate_capture(args, route)


def _actual_fixture_owner_schema():
    """HH_261004 - Read the exact module global used by the real fixture admission function."""
    owner = fixture.assert_owned_capture.__globals__.get("OWNER_SCHEMA")
    constructor_owner = fixture.GreenSignalFixture.__init__.__globals__.get("OWNER_SCHEMA")
    frozen._require(owner == constructor_owner, "fixture admission and constructor owner schemas differ")
    return owner


@contextmanager
def activated(model, args):
    """HH_261004 - Scope only both copied schema sources, then restore every frozen alias."""
    global _active
    frozen._require(not _active and not previous._active and not straight._active
        and previous.SCHEMA == _PARENT_SCHEMA and previous.OWNER_SCHEMA == _PARENT_OWNER_SCHEMA
        and straight.SCHEMA == _STRAIGHT_SCHEMA and straight.OWNER_SCHEMA == _STRAIGHT_OWNER_SCHEMA
        and fixture.SCHEMA == _FIXTURE_SCHEMA and fixture.OWNER_SCHEMA == _FIXTURE_OWNER_SCHEMA,
        "fresh Town03 straight v3 schema admission cannot nest or replace unknown aliases")
    verify_frozen()
    validate_capture(args, collector.load_route(args.route_file))
    saved = previous.SCHEMA, previous.OWNER_SCHEMA, straight.SCHEMA, straight.OWNER_SCHEMA
    _active = True
    try:
        # HH_261004 - V2 compiles from its globals; frozen straight propagates its globals to right-v2's real fixture.
        previous.SCHEMA, previous.OWNER_SCHEMA = SCHEMA, OWNER_SCHEMA
        straight.SCHEMA, straight.OWNER_SCHEMA = SCHEMA, OWNER_SCHEMA
        with previous.activated(model, args):
            frozen._require(_actual_fixture_owner_schema() == OWNER_SCHEMA
                and fixture.SCHEMA == SCHEMA and fixture.OWNER_SCHEMA == OWNER_SCHEMA,
                "fresh declaration schema did not reach the actual owned Green fixture")
            yield
    finally:
        previous.SCHEMA, previous.OWNER_SCHEMA, straight.SCHEMA, straight.OWNER_SCHEMA = saved
        _active = False
