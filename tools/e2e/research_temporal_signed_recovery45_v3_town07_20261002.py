"""HH_261002 - NEW actual preflight/worker validator binding; retain aborted v1/v2 attempts and all fixed physical gates."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path

from . import research_temporal_signed_recovery45_v2_town07_20261002 as previous
from . import collect_temporal_signed_recovery45_v3_20261002 as live

ROOT,base,parent=previous.ROOT,previous.base,previous.parent
MODULE="scripts.e2e.research_temporal_signed_recovery45_v3_town07_20261002"
SCHEMA="portable_e2e.actual_signed_recovery45_three_arms.20261002.v3"
V2_SHA="ac029ad1b840a96c7105ce1cf9aad2531bbc177112ff8a0c689fa24a1804b7bd"
require,digest=previous.require,previous.digest
_fixed,_runtime=previous.fixed_contract,previous.runtime_sources
_active=False


def fixed_contract(run_id,longitudinal_pi):
    result=_fixed(run_id,longitudinal_pi)
    result.update(validator_profile_namespace_explicit_before_collector_activation=True,
        failed_v2_declaration_and_preflight_retained=True,
        v2_preflight_failure_was_before_actual_simulator_execution=True)
    return result


def runtime_sources():
    live.verify_frozen()
    require(digest(Path(previous.__file__)) == V2_SHA, "already-declared v2 owner must remain exact")
    pins=_runtime()
    pins.update({str(path.relative_to(ROOT)):digest(path)
        for path in sorted(base.imported_source_closure((Path(__file__).resolve(),)))})
    return pins


@contextmanager
def activated():
    """HH_261002 - The inherited trial scope installs this NEW public validator before base.preflight, not only before collector activation."""
    global _active
    require(not _active and not previous._active, "v3 signed owner cannot nest")
    runtime_sources()
    names=dict(live=live,MODULE=MODULE,SCHEMA=SCHEMA,fixed_contract=fixed_contract,runtime_sources=runtime_sources)
    saved={name:getattr(previous,name) for name in names};_active=True
    try:
        for name,value in names.items():setattr(previous,name,value)
        with previous.activated():yield
    finally:
        for name,value in saved.items():setattr(previous,name,value)
        _active=False


def main(argv=None):
    with activated():return parent.main(argv)


if __name__ == "__main__":raise SystemExit(main())
