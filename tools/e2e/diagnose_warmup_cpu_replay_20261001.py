#!/usr/bin/env python3
"""HH_261001 - Separate lossless input identity from CPU-backend bitwise replay."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
ARMS = ('W0_full_supervision', 'W1_warmup_anchor_off')
PROFILES = ('unset', 'omp2', 'openblas2', 'live2', 'live2_mkldnn_off')
MODES = ('numpy', 'clone', 'runtime_owned')
ENV_KEYS = ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS')
SOURCE_PATHS = ('learning/portable_e2e/model.py', 'learning/portable_e2e/runtime.py',
    'learning/portable_e2e/simulation_research.py',
    'tools/e2e/capture_exact_simulation_inputs_20261001.py',
    'tools/e2e/research_warmup_supervision_candidate_20261001.py',
    'tools/e2e/replay_exact_warmup_inputs_20261001.py',
    'tools/e2e/run_owned_ah_town07_research_20261001.py',
    'tools/e2e/collect_ah_simulation_research.py')


def digest(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError('regular evidence file required: ' + str(path))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_new(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def profile_environment(name):
    if name not in PROFILES:
        raise ValueError('unknown diagnostic environment')
    env = dict(os.environ, CUDA_VISIBLE_DEVICES='', PYTHONDONTWRITEBYTECODE='1')
    for key in ENV_KEYS:
        env.pop(key, None)
    if name in ('omp2', 'live2', 'live2_mkldnn_off'):
        env['OMP_NUM_THREADS'] = '2'
    if name in ('openblas2', 'live2', 'live2_mkldnn_off'):
        env['OPENBLAS_NUM_THREADS'] = '2'
    return env


def tensorize(arrays, mode):
    import torch
    if mode not in MODES:
        raise ValueError('unknown tensor ownership mode')
    result = {}
    for name, array in arrays.items():
        tensor = torch.from_numpy(array)
        if mode == 'clone':
            tensor = tensor.clone()
        elif mode == 'runtime_owned':
            # HH_261001 - Match the live builder's owned contiguous storage and leading unsqueeze.
            tensor = torch.tensor(array[0], dtype=tensor.dtype).unsqueeze(0)
        result[name] = tensor
    return result


def error_metrics(actual, expected):
    import numpy as np
    result = {}
    for name in actual:
        left = np.asarray(actual[name], dtype=np.float32)
        right = np.asarray(expected[name], dtype=np.float32)
        if left.shape != right.shape or not np.isfinite(left).all() or not np.isfinite(right).all():
            raise ValueError('finite matching output ABI required')

        def ordered(value):
            bits = value.view(np.uint32).astype(np.int64)
            return np.where(bits < 0x80000000, bits + 0x80000000,
                            0x80000000 - (bits & 0x7fffffff))

        result[name] = dict(max_absolute_error=float(np.max(np.abs(left.astype(np.float64) - right))),
            max_float32_ulp_distance=int(np.max(np.abs(ordered(left) - ordered(right)))),
            unequal_elements=int(np.count_nonzero(left != right)), elements=int(left.size))
    return result


def published_outputs(trial, journal):
    from scripts.e2e.replay_exact_warmup_inputs_20261001 import read_rows
    from scripts.e2e.capture_exact_simulation_inputs_20261001 import output_digest
    episode = trial / 'episode.partial'
    if not episode.exists():
        episode = trial / 'episode'
    states = {row['frame']: row for row in read_rows(episode / 'states.jsonl')}
    published = {}
    for row in read_rows(episode / 'learned_control.jsonl'):
        if row.get('raw_output') is None or row.get('held_between_camera_frames'):
            continue
        stamp = round(states[row['command_frame']]['timestamp'] * 1e9)
        if stamp in published:
            raise ValueError('ambiguous publication')
        published[stamp] = row
    if len(published) != len(journal):
        raise ValueError('archive/publication coverage differs')
    for seq, row in enumerate(journal):
        decision = published[row['timing']['now_ns']]
        if (seq != row['sequence'] or output_digest(decision['raw_output']) != row['raw_output_sha256']
                or decision['source_checkpoint_sha256'] != row['source_checkpoint_sha256']):
            raise ValueError('journal/publication source or output differs')
    return published, episode


def worker(args):
    import numpy as np
    import torch
    from portable_e2e import simulation_research as research
    from scripts.e2e.capture_exact_simulation_inputs_20261001 import array_digest, output_digest
    from scripts.e2e.replay_exact_warmup_inputs_20261001 import load_snapshot, read_rows, replay
    from scripts.e2e.research_warmup_supervision_candidate_20261001 import identity
    if os.environ.get('CUDA_VISIBLE_DEVICES') != '':
        raise ValueError('CPU-only process required')
    initial_threads = torch.get_num_threads()
    torch.set_num_threads(2)
    torch.backends.mkldnn.enabled = args.profile != 'live2_mkldnn_off'
    try:
        from threadpoolctl import threadpool_info
        pools = threadpool_info()
    except ImportError:
        pools = None
    report = dict(schema='portable_e2e.warmup_cpu_replay_diagnosis.v1', profile=args.profile,
        source_sha256=digest(Path(__file__)), python=sys.version, executable=sys.executable,
        torch_version=torch.__version__, numpy_version=np.__version__,
        environment={key: os.environ.get(key) for key in (*ENV_KEYS, 'CUDA_VISIBLE_DEVICES')},
        torch_initial_intraop=initial_threads, torch_forward_intraop=torch.get_num_threads(),
        torch_interop=torch.get_num_interop_threads(), mkldnn_enabled=torch.backends.mkldnn.enabled,
        torch_build=torch.__config__.show(), native_threadpools=pools,
        device='cpu', training_steps=0, simulator_started=False, original_evidence_modified=False,
        tolerance_acceptance_added=False, arms={})
    for arm in ARMS:
        trial = ROOT / 'storage/experiments/validation/2026-10-01' / ('warmup_supervision_' + arm + '_town07_v1')
        plan = json.loads((trial / 'owner_plan.json').read_text())
        argv = plan['collector_command']
        def value(flag):
            return argv[argv.index(flag) + 1]
        pins = {ROOT / name: plan['source_sha256'][name] for name in SOURCE_PATHS if name in plan['source_sha256']}
        pins.update({Path(value('--research-weights')): value('--research-weights-sha256'),
                     Path(value('--research-metadata')): value('--research-metadata-sha256')})
        for path, sha in pins.items():
            if digest(path) != sha:
                raise ValueError('live source or weight pin changed: ' + str(path))
        folder = trial / 'exact_runtime_inputs'
        journal = read_rows(folder / 'index.jsonl')
        published, episode = published_outputs(trial, journal)
        evidence = [trial / 'owner_plan.json', trial / 'exact_input_replay.json', folder / 'index.jsonl',
                    episode / 'states.jsonl', episode / 'learned_control.jsonl', *pins]
        before = {str(path): digest(path) for path in evidence}
        with identity(plan['source_checkpoint_sha256']):
            model = research.load_ah_weights(value('--research-weights'), value('--research-metadata'),
                weights_sha256=value('--research-weights-sha256'), metadata_sha256=value('--research-metadata-sha256'))
        results = []
        for row in journal[:3]:
            arrays = load_snapshot(folder, row)
            for mode in MODES:
                tensors = tensorize(arrays, mode)
                layouts = {name: dict(shape=list(t.shape), stride=list(t.stride()),
                    contiguous=t.is_contiguous(), storage_offset=t.storage_offset(), pointer_modulo_64=t.data_ptr() % 64,
                    numpy_storage_shared=t.data_ptr() == arrays[name].ctypes.data)
                    for name, t in tensors.items()}
                for name, tensor in tensors.items():
                    if array_digest(tensor.numpy()) != row['tensor_sha256'][name]:
                        raise ValueError('diagnostic changed an input value')
                hashes = []
                for repeat in range(2):
                    with torch.no_grad():
                        xy, speed, logits = model(*(tensors[key] for key in research.FIELDS))
                    actual = dict(candidate_xy=xy[0].tolist(), candidate_speed=speed[0].tolist(),
                                  candidate_logits=logits[0].tolist())
                    hashes.append(output_digest(actual))
                if hashes[0] != hashes[1]:
                    raise ValueError('same-buffer repeated forward differs')
                if any(array_digest(t.numpy()) != row['tensor_sha256'][name] for name, t in tensors.items()):
                    raise ValueError('forward mutated an archived input')
                expected = published[row['timing']['now_ns']]['raw_output']
                results.append(dict(sequence=row['sequence'], ownership=mode, layouts=layouts,
                    exact_hash_match=hashes[0] == row['raw_output_sha256'], output_sha256=hashes[0],
                    repeated_forward_bitwise_equal=True, error_metrics=error_metrics(actual, expected)))
        arm_report = dict(original_replay_status=json.loads((trial / 'exact_input_replay.json').read_text())['status'],
            trial=str(trial), checkpoint_sha256=plan['source_checkpoint_sha256'], probes=results)
        if args.profile == 'live2':
            # HH_261001 - Invoke the untouched exact validator; no tolerance or ownership fix is substituted.
            replay_args = argparse.Namespace(arm=arm, trial=trial,
                checkpoint_sha256=plan['source_checkpoint_sha256'], weights=Path(value('--research-weights')),
                metadata=Path(value('--research-metadata')), weights_sha256=value('--research-weights-sha256'),
                metadata_sha256=value('--research-metadata-sha256'))
            arm_report['unchanged_validator_full_replay'] = replay(replay_args)
        after = {str(path): digest(path) for path in evidence}
        if before != after:
            raise ValueError('historical evidence or source changed during read-only diagnosis')
        arm_report.update(evidence_sha256=before, source_and_evidence_fingerprints_unchanged=True)
        report['arms'][arm] = arm_report
    write_new(args.output, report)
    print(json.dumps(dict(profile=args.profile,
        probe_matches={arm: sum(p['exact_hash_match'] for p in result['probes'])
                       for arm, result in report['arms'].items()})))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--profile', choices=PROFILES)
    args = parser.parse_args()
    if args.profile:
        worker(args)
        return 0
    args.output.mkdir(parents=True, exist_ok=False)
    outcomes = {}
    for profile in PROFILES:
        result_path = args.output / (profile + '.json')
        command = [sys.executable, '-B', str(ROOT / 'tools/workspace_python.py'), str(Path(__file__)),
                   '--profile', profile, '--output', str(result_path)]
        completed = subprocess.run(command, env=profile_environment(profile), check=True,
            capture_output=True, text=True, timeout=180)
        outcomes[profile] = dict(receipt=str(result_path), sha256=digest(result_path), stdout=completed.stdout)
        print(completed.stdout.strip(), flush=True)
    write_new(args.output / 'index.json', dict(schema='portable_e2e.warmup_cpu_replay_diagnosis_index.v1',
        source_sha256=digest(Path(__file__)), profiles=outcomes, original_validator_unchanged=True,
        exact_equality_not_relaxed=True, device='cpu', training_steps=0, simulator_started=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
