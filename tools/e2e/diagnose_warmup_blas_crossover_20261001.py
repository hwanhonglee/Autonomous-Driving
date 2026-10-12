#!/usr/bin/env python3
"""HH_261001 - Test a reversible BLAS-only thread intervention on fixed archived inputs."""

import argparse
import json
import os
from pathlib import Path
import torch
from threadpoolctl import threadpool_info, threadpool_limits
from portable_e2e import simulation_research as research
from scripts.e2e.capture_exact_simulation_inputs_20261001 import array_digest, output_digest
from scripts.e2e.diagnose_warmup_cpu_replay_20261001 import ARMS, ROOT, digest, write_new
from scripts.e2e.replay_exact_warmup_inputs_20261001 import load_snapshot, read_rows
from scripts.e2e.research_warmup_supervision_candidate_20261001 import identity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--diagnosis', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('CUDA_VISIBLE_DEVICES') != '':
        raise ValueError('explicit CPU-only process required')
    references = {name: json.loads((args.diagnosis / (name + '.json')).read_text())
                  for name in ('unset', 'live2')}
    torch.set_num_threads(2)
    if not torch.backends.mkldnn.enabled:
        raise ValueError('unchanged live MKLDNN backend required')
    results = {}
    for arm in ARMS:
        trial = Path(references['live2']['arms'][arm]['trial'])
        prior_pins = references['live2']['arms'][arm]['evidence_sha256']
        for path, sha in prior_pins.items():
            if digest(Path(path)) != sha:
                raise ValueError('original evidence changed')
        argv = json.loads((trial / 'owner_plan.json').read_text())['collector_command']
        def value(flag):
            return argv[argv.index(flag) + 1]
        checkpoint = value('--checkpoint-sha256')
        with identity(checkpoint):
            model = research.load_ah_weights(value('--research-weights'), value('--research-metadata'),
                weights_sha256=value('--research-weights-sha256'), metadata_sha256=value('--research-metadata-sha256'))
        rows = []
        for row in read_rows(trial / 'exact_runtime_inputs/index.jsonl')[:3]:
            arrays = load_snapshot(trial / 'exact_runtime_inputs', row)
            tensors = {key: torch.from_numpy(value) for key, value in arrays.items()}
            for threads in (24, 2, 24):
                # HH_261001 - Same model, input pointers, strides and Torch/OpenMP settings; only BLAS pool width changes.
                with threadpool_limits(limits=threads, user_api='blas'):
                    pools = threadpool_info()
                    if torch.get_num_threads() != 2 or any(p['num_threads'] != threads for p in pools if p['user_api'] == 'blas'):
                        raise ValueError('BLAS-only intervention did not take effect')
                    with torch.no_grad():
                        xy, speed, logits = model(*(tensors[key] for key in research.FIELDS))
                    actual = dict(candidate_xy=xy[0].tolist(), candidate_speed=speed[0].tolist(),
                                  candidate_logits=logits[0].tolist())
                out_sha = output_digest(actual)
                profile = 'live2' if threads == 2 else 'unset'
                expected = next(p for p in references[profile]['arms'][arm]['probes']
                                if p['sequence'] == row['sequence'] and p['ownership'] == 'numpy')
                if out_sha != expected['output_sha256']:
                    raise ValueError('BLAS intervention did not reproduce the independent startup profile')
                if any(array_digest(t.numpy()) != row['tensor_sha256'][key] for key, t in tensors.items()):
                    raise ValueError('input values changed')
                rows.append(dict(sequence=row['sequence'], blas_threads=threads,
                    torch_intraop_threads=torch.get_num_threads(), native_threadpools=pools,
                    output_sha256=out_sha, exact_live_output_match=out_sha == row['raw_output_sha256'],
                    independent_startup_profile_match=True))
        if any(digest(Path(path)) != sha for path, sha in prior_pins.items()):
            raise ValueError('original evidence changed')
        results[arm] = rows
    write_new(args.output, dict(schema='portable_e2e.warmup_blas_crossover.v1',
        status='BLAS_ONLY_CAUSAL_CROSSOVER_CONFIRMED', arms=results,
        source_sha256=digest(Path(__file__)), original_source_and_evidence_unchanged=True,
        references={name: digest(args.diagnosis / (name + '.json')) for name in references},
        simulator_started=False, training_steps=0, device='cpu', exact_equality_not_relaxed=True))
    print('BLAS_ONLY_CAUSAL_CROSSOVER_CONFIRMED: 24 -> 2 -> 24, first three inputs per arm')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
