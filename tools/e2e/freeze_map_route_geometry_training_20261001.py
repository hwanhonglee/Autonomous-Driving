#!/usr/bin/env python3
"""HH_261001 - Archive a tiny local training source closure without arrays, fit approval or remote work."""

import hashlib
import json
from pathlib import Path
import shutil

try:
    from . import map_route_geometry_contract_20261001 as c
except ImportError:
    from scripts.e2e import map_route_geometry_contract_20261001 as c

ROOT = Path(__file__).resolve().parents[2]
DOC = ROOT/'docs/assets/validation/2026-10-01/map_unused_route_inventory'
OUTPUT = DOC/'training_source_v1'


def main():
    c.base.require(not OUTPUT.exists(),'Fresh source freeze directory required')
    c.base.require(c.base.digest(DOC/'remote_package_manifest.json') == c.DATA_SHA
        and c.base.digest(DOC/'package_authorization.json') == c.PACKAGE_APPROVAL_SHA
        and c.base.digest(DOC/'reference_plan.json') == c.PLAN_SHA,'Reviewed package or references changed')
    paths = {name:ROOT/'tools/e2e'/name for name in c.SOURCE_FILES if name.endswith('.py')}
    for name in c.base.ARCHITECTURE_SHA:
        paths[name] = ROOT/'learning/portable_e2e'/name
    paths['training_plan.md'] = DOC/'TRAINING_PLAN.md'
    paths['weights_correction.md'] = DOC/'WEIGHT_PROVENANCE_CORRECTION.md'
    fingerprints = {name:c.base.digest(path) for name,path in paths.items()}
    for name,sha in c.PINNED_SOURCE.items():
        c.base.require(fingerprints[name] == sha,'Existing frozen source changed')
    OUTPUT.mkdir()
    for name,path in paths.items():
        with path.open('rb') as src,(OUTPUT/name).open('xb') as dst:
            shutil.copyfileobj(src,dst)
        c.base.require(c.base.digest(OUTPUT/name) == fingerprints[name],'Source archive differs')
    bundle = {'schema':'portable_e2e.map_geometry_training_source.v1','study_id':c.NAME,
        'files':fingerprints,'prepared_manifest_sha256':c.DATA_SHA,'reference_plan_sha256':c.PLAN_SHA,
        'package_authorization_sha256':c.PACKAGE_APPROVAL_SHA,'fit_authorized_by_source_bundle':False}
    with (OUTPUT/'bundle.json').open('x') as stream:
        json.dump(bundle,stream,indent=2,sort_keys=True)
        stream.write('\n')
    sha = c.base.digest(OUTPUT/'bundle.json')
    c.source_binding(OUTPUT,sha)
    c.base.require(sum(p.stat().st_size for p in OUTPUT.iterdir()) < 1024*1024,'Tiny source-only budget exceeded')
    print(json.dumps({'status':'SOURCE_FROZEN_NO_FIT','source_bundle_sha256':sha,'source':str(OUTPUT)}))


if __name__ == '__main__':
    main()
