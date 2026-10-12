"""HH_261004 - Seal the entire failed callback45v2 actual archive without editing any recorded byte."""
from __future__ import annotations

import json
from pathlib import Path
from . import publish_temporal_route_joint_town03_straight_callback45_v2_20261004 as publication


def main():
    prior = publication.prior
    plan = publication.POLICY_PATH.parent
    caller_path = plan / "ROOT_actual_publication_caller.json"
    policy = json.loads(publication.POLICY_PATH.read_text())
    files = publication.previous.inventory(publication.RUN)
    hashes = {name: prior.digest(publication.RUN / name) for name in sorted(files)}
    sources = {str(path.relative_to(publication.ROOT)): prior.digest(path)
               for path in publication.publication_sources()}
    publication.require(sources == policy["publication_source_sha256"], "prospective source seal changed")
    caller = dict(schema=publication.CALLER_SCHEMA, attempts=1, run_root=str(publication.RUN),
        output=str(publication.OUTPUT), PNG_index_policy=publication.PNG_INDEX_POLICY,
        PNG_percent_numerators=publication.PNG_PERCENT_NUMERATORS,
        expected_full_PNG_camera_indices=publication.FULL450_PNG_CAMERA_INDICES,
        run_files_sha256=hashes, publication_source_sha256=sources, font_sha256=policy["font_sha256"],
        prospective_publication_policy=dict(path=str(publication.POLICY_PATH), sha256=prior.digest(publication.POLICY_PATH)),
        provenance="HH_261004 - Actual full failed interval retained; this caller cannot approve driving or training.")
    publication.require(not caller_path.exists() and not caller_path.is_symlink(), "exclusive fresh actual caller required")
    with caller_path.open("x", encoding="utf-8") as stream:
        json.dump(caller, stream, sort_keys=True, indent=2, allow_nan=False); stream.write("\n")
    publication.require(publication.previous.inventory(publication.RUN) == files and
        all(prior.digest(publication.RUN / name) == sha for name, sha in hashes.items()), "actual archive changed during seal")
    print(json.dumps(dict(status="ROOT_ENTIRE_CALLBACK45V2_ACTUAL_ARCHIVE_SEALED_NO_PROMOTION",
                         file_count=len(files), caller_path=str(caller_path), caller_sha256=prior.digest(caller_path))))


if __name__ == "__main__":
    main()
