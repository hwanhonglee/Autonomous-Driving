"""HH_261002 - Prospectively declare three fresh uint32-safe arms; preserve failed v1 declarations and raw outcomes."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path

from . import research_temporal_signed_recovery45_town07_20261002 as previous
from . import collect_temporal_signed_recovery45_v2_20261002 as live

ROOT,base,parent=previous.ROOT,previous.base,previous.parent
MODULE="scripts.e2e.research_temporal_signed_recovery45_v2_town07_20261002"
SCHEMA="portable_e2e.actual_signed_recovery45_three_arms.20261002.v2"
V1_SHA="429af0b7d49889419caba872d25e15655bad3fabab8d3ba1ce871ed14e8510e3"
require,digest=previous.require,previous.digest
_fixed,_runtime=previous.fixed_contract,previous.runtime_sources
_active=False


def fixed_contract(run_id,longitudinal_pi):
    spec=previous.trial_profile(); live.validate_seed(spec["seed"])
    result=_fixed(run_id,longitudinal_pi)
    result.update(unsigned32_seed_prevalidated=True,actual_seed_interval_inclusive=[0,2**32-1],
        failed_v1_seeds_and_output_retained=True,original_exception_not_masked_by_no_spawn_finalizer=True,
        seed_revision="Fresh2026102100/101/102 fit actual np.random.seed uint32 ABI; no outcome-selected retries.")
    return result


def runtime_sources():
    live.verify_frozen()
    require(digest(Path(previous.__file__)) == V1_SHA, "frozen v1 owner must remain exact")
    pins=_runtime()
    pins.update({str(path.relative_to(ROOT)):digest(path)
        for path in sorted(base.imported_source_closure((Path(__file__).resolve(),)))})
    return pins


@contextmanager
def activated():
    """HH_261002 - Validate every seed before declaration/preflight; delegate unchanged actual private-worker lifecycle."""
    global _active
    require(not _active and not previous._active, "v2 signed owner cannot nest")
    runtime_sources()
    names=dict(live=live,MODULE=MODULE,SCHEMA=SCHEMA,fixed_contract=fixed_contract,runtime_sources=runtime_sources)
    saved={name:getattr(previous,name) for name in names}; _active=True
    try:
        for name,value in names.items(): setattr(previous,name,value)
        with previous.activated(): yield
    finally:
        for name,value in saved.items(): setattr(previous,name,value)
        _active=False


def main(argv=None):
    with activated(): return parent.main(argv)


if __name__ == "__main__": raise SystemExit(main())
