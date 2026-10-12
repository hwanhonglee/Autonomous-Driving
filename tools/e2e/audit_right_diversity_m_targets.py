#!/usr/bin/env python3
"""HH_260906 - Check every recorded future against unchanged runtime gates before training admission."""
import argparse
from collections import Counter
import json
from pathlib import Path
from portable_e2e.contract import ContractError, sha256_file
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory


def main():
    """HH_260906 - Keep rejected labels intact and report first gate failures without clipping."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    manifest=json.loads((args.dataset/'dataset.json').read_text())
    failures=[];total=0
    for ref in manifest['episodes']:
        episode_path=args.dataset/ref['manifest']
        episode=json.loads(episode_path.read_text())
        if episode['split'] != 'train':
            raise ValueError('TRAIN-only target audit')
        for line in (episode_path.parent/episode['sample_jsonl']).read_text().splitlines():
            row=json.loads(line); target=row['labels']['planning'];total+=1
            try:
                validate_and_select_trajectory([target['positions_base_xy_m']], [target['speed_mps']], [0.0],
                    RuntimeGateConfig(candidate_count=1),current_speed_mps=row['ego']['linear_velocity_base_mps'][0])
            except ContractError as error:
                failures.append(dict(sample_id=row['sample_id'], reason=str(error)))
    report=dict(dataset_sha256=sha256_file(args.dataset/'dataset.json'),sample_count=total,
                passing_targets=total-len(failures),first_failure_counts=dict(Counter(x['reason'] for x in failures)),
                failures=failures,training_admitted=False,labels_modified=False,
                notice='Runtime target feasibility diagnostic; no model inference or optimization.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k != 'failures'},indent=2))


if __name__ == '__main__':
    main()
