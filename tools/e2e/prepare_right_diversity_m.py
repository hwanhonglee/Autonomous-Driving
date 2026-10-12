#!/usr/bin/env python3
"""HH_260906 - Convert pinned native TRAIN routes without inventing map-alignment metadata."""
from contextlib import contextmanager
import json
from pathlib import Path
from scripts.e2e import prepare_carla_common10_dataset as base

ROOT = Path(__file__).resolve().parents[2]
TRIAL_ROOT = ROOT / 'artifacts/training/2026-09-22/right_diversity_m_v1'
TRIALS = ('03_town01_p02_historical_lookahead', '04_town07_p03_historical_lookahead',
          '06_town01_p02_ground_spawn')


def authorized_routes(specs):
    """HH_260906 - Bind actual capture provenance to unchanged catalog route bytes."""
    routes=[]
    for spec in specs:
        if spec.split != 'train' or spec.path not in [TRIAL_ROOT/n/'episode' for n in TRIALS]:
            raise base.AdapterError('only the explicitly listed M TRAIN captures are supported')
        owner=base._json(spec.path.parent/'owner_plan.json')
        completion=base._json(spec.path.parent/'owner_result.json')
        manifest=base._json(spec.path/'manifest.json')
        base._require_native_admission_not_denied(manifest)
        source=Path(owner['route_path'])
        if (completion.get('exit_code') != 0 or completion.get('source_bytes_unchanged_and_archived') is not True
                or owner['map'] not in ('Town01','Town07') or source.is_symlink()
                or not source.resolve().is_relative_to(ROOT/'artifacts/validation/2026-09-01')
                or base._sha256(source) != owner['route_sha256']
                or base._sha256(spec.path/'route.json') != owner['route_sha256']
                or manifest['provenance']['route_sha256'] != owner['route_sha256']):
            raise base.AdapterError('native catalog route provenance mismatch')
        route=base._json(source)
        if 'coordinate_alignment' in route or route['scenario'] != 'right' or route['town'] != owner['map']:
            raise base.AdapterError('unaligned TRAIN right route required')
        routes.append((route, source.stem))
    return routes


@contextmanager
def native_identity(routes):
    """HH_260906 - Override route naming only; retain all event, temporal and physical validators."""
    original=base._native_route_id
    def identify(route):
        """HH_260906 - Match full route content instead of attaching fictitious alignment provenance."""
        matches=[name for payload,name in routes if payload == route]
        if len(matches) != 1:
            raise base.AdapterError('native route does not uniquely match authenticated catalog')
        return base._identifier(matches[0], 'route')
    base._native_route_id=identify
    try:
        yield
    finally:
        base._native_route_id=original


def main(argv=None):
    """HH_260906 - Preserve original evidence and save the alternate route-identity receipt."""
    args=base._parse_args(argv)
    routes=authorized_routes(args.episode)
    with native_identity(routes):
        result=base.main(argv)
    if result == 0:
        receipt=dict(adapter='native_catalog_identity_m_v1',
                     route_ids=[name for _,name in routes], source_route_bytes_modified=False,
                     validation_math_modified=False, training_executed=False,
                     dataset_sha256=base._sha256(args.output/'dataset.json'))
        with (args.output/'native_catalog_identity.json').open('x') as stream:
            json.dump(receipt,stream,indent=2)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
