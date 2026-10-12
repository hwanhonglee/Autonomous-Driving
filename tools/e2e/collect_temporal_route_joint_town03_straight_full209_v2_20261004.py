"""HH_261004 - Prepare fresh terminal timing research through the frozen full209 capture ABI.

The original V1 raw schema stays explicit and unchanged; the new owner revision
and timing receipt have separate identities. No V1 source, data, decoder,
control law, safety threshold or failure result is overwritten.
"""

from __future__ import annotations

from contextlib import contextmanager
import hashlib
from pathlib import Path

from . import collect_temporal_route_joint_town03_straight_full209_v1_20261004 as previous


SCHEMA = previous.SCHEMA
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_straight_full209.20261004.v2"
PARENT_SHA256 = "22ae923a0bfba4b0eb9c283e0083ef480fd2782c6e19dbf682ec16476a9556e0"
_active = False


def __getattr__(name):
    return getattr(previous, name)


def verify_frozen():
    # HH_261004 - Reuse the frozen acquisition/control closure, while giving the experimental owner its own identity.
    previous.frozen._require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == PARENT_SHA256,
        "frozen full209 V1 collector changed")
    previous.verify_frozen()


@contextmanager
def activated(model, args):
    """HH_261004 - Change only scoped owner admission and the exact-math timing bridge; restore on any failure."""
    global _active
    from portable_e2e.temporal_navigation_terminal_policy_v3_timing_20261004 import navigation_terminal_policy_class
    verify_frozen()
    previous.frozen._require(not _active and not previous._active, "fresh full209 V2 collector cannot nest")
    saved = previous.OWNER_SCHEMA, previous.navigation_terminal_policy_class
    _active = True
    try:
        previous.OWNER_SCHEMA = OWNER_SCHEMA
        previous.navigation_terminal_policy_class = navigation_terminal_policy_class
        with previous.activated(model, args):
            previous.frozen._require(previous.fixture.assert_owned_capture.__globals__["OWNER_SCHEMA"] == OWNER_SCHEMA,
                "fresh V2 owner did not reach actual Green fixture source guard")
            yield
    finally:
        previous.OWNER_SCHEMA, previous.navigation_terminal_policy_class = saved
        _active = False
