"""HH_261001 - Archive actual model tensors without modifying inputs, outputs or control policy."""

from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from portable_e2e import simulation_research as research


def array_digest(value):
    array = np.ascontiguousarray(value)
    header = json.dumps(dict(shape=list(array.shape), dtype=str(array.dtype)), sort_keys=True).encode()
    return hashlib.sha256(header + b'\0' + array.tobytes()).hexdigest()


def output_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def save_input_snapshot(folder, sequence, tensors, timing, decision, capture_seconds):
    # HH_261001 - Uncompressed arrays minimize synchronous disk overhead; no pickle is written.
    name = f'{sequence:06d}_{timing["now_ns"]}.npz'
    path = folder / name
    with path.open('xb') as stream:
        np.savez(stream, **tensors)
    return dict(schema='portable_e2e.exact_runtime_tensors.v1', sequence=sequence,
        tensor_file=name, tensor_file_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        tensor_sha256={key:array_digest(value) for key, value in tensors.items()},
        timing=timing, source_checkpoint_sha256=decision['source_checkpoint_sha256'],
        raw_output_sha256=output_digest(decision['raw_output']),
        input_capture_seconds=capture_seconds, original_policy_total_seconds=decision['total_seconds'],
        archive_write_outside_policy_deadline=True, inputs_or_outputs_modified=False,
        training_data_approved=False)


@contextmanager
def exact_tensor_archive(directory):
    """HH_261001 - A temporary process-local observer copies each builder result before forward."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    original_step = research.SimulationResearchPolicy.step
    original_builder = research.build_runtime_tensors
    sequence = 0
    journal = (directory / 'index.jsonl').open('x')

    def step(policy, inputs, *, now_ns, ego_stamp_ns, route_stamp_ns, emergency_reason=None):
        nonlocal sequence
        captured = []

        def builder(*args, **kwargs):
            tensors = original_builder(*args, **kwargs)
            start = time.perf_counter()
            arrays = {key:tensors[key].detach().cpu().numpy().copy() for key in research.FIELDS}
            captured.append((arrays, time.perf_counter() - start))
            return tensors

        # HH_261001 - The unchanged policy retains the original deadline; copy overhead is included.
        research.build_runtime_tensors = builder
        try:
            decision = original_step(policy, inputs, now_ns=now_ns, ego_stamp_ns=ego_stamp_ns,
                route_stamp_ns=route_stamp_ns, emergency_reason=emergency_reason)
        finally:
            research.build_runtime_tensors = original_builder
        if len(captured) > 1:
            raise ValueError('one exact tensor build per policy step required')
        if captured:
            arrays, copying = captured[0]
            started = time.perf_counter()
            timing = dict(now_ns=now_ns, ego_stamp_ns=ego_stamp_ns, route_stamp_ns=route_stamp_ns,
                          camera_stamps_ns=[frame.timestamp_ns for frame in inputs.camera_frames])
            row = save_input_snapshot(directory, sequence, arrays, timing, decision, copying)
            row['archive_wall_seconds_outside_policy'] = time.perf_counter() - started
            journal.write(json.dumps(row, sort_keys=True, allow_nan=False) + '\n')
            journal.flush()
            sequence += 1
        return decision

    research.SimulationResearchPolicy.step = step
    try:
        yield
    finally:
        research.SimulationResearchPolicy.step = original_step
        research.build_runtime_tensors = original_builder
        journal.close()
