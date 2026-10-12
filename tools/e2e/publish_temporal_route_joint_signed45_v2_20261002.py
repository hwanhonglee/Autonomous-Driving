"""HH_261002 - Bind the omitted exact rear-error reader for publication without editing frozen study sources."""

from contextlib import contextmanager
from pathlib import Path

from . import publish_temporal_route_joint_signed45_batch_20261002 as previous

AUDIT_SHA = "844f1586eaf331b2ff867e265c5955322980f5d172b7a7c1375ce9de4a2621f3"
PUBLISHER_SHA = "e69c24827331f535fb00d64e6362a88e027a867e7448e5dfdd9310fade403a5b"
_load_archive = previous.load_archive
_active = False


def load_archive(*args, **kwargs):
    # HH_261002 - Preserve every existing full-denominator check and add this adapter to immutable evidence.
    report, bundles, pins = _load_archive(*args, **kwargs)
    audit = previous.audit
    for path in audit.source_closure(Path(__file__), audit.ROOT / "tools/workspace_python.py",
                                     audit.ROOT / "tools/canonical_workspace.py"):
        audit.add_pin(pins, path)
    audit.frozen.assert_pins(pins)
    return report, bundles, pins


@contextmanager
def activated():
    # HH_261002 - The previous publication failed before creating outputs; only a missing namespace alias is repaired.
    global _active
    audit = previous.audit
    audit.require(not _active and not previous._active, "publication adapter cannot nest")
    audit.require(audit.digest(Path(audit.__file__)) == AUDIT_SHA
                  and audit.digest(Path(previous.__file__)) == PUBLISHER_SHA,
                  "unchanged frozen joint audit and publisher required")
    audit.require(not hasattr(audit, "measured_rear_errors") and previous.load_archive is _load_archive,
                  "unknown publication reader binding")
    _active = True
    audit.measured_rear_errors = audit.previous.measured_rear_errors
    previous.load_archive = load_archive
    try:
        yield
    finally:
        previous.load_archive = _load_archive
        del audit.measured_rear_errors
        _active = False


def main(argv=None):
    with activated():
        return previous.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
