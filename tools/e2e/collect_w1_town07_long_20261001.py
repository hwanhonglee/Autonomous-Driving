"""HH_261001 - One prospective long W1 diagnostic with unchanged control and sparse exact inputs."""

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import torch
from portable_e2e import simulation_research as research
from scripts.e2e import collect_ah_simulation_research as short
from scripts.e2e.research_warmup_supervision_candidate_20261001 import identity
from scripts.e2e.capture_exact_simulation_inputs_20261001 import exact_tensor_archive

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / 'storage/experiments/validation/2026-10-01/warmup_supervision_W1_town07_long_v1'
CHECKPOINT = 'b9ee946ada6824e7d79f51bc1693463dc506db3faaa9038ddaa05816cf7ecf76'
WEIGHTS_SHA = '4176ebe38bccd85394de480946614bfb306c571eb9a677282dd840d78c5354cf'
METADATA_SHA = '4b559b17387138faf2570c28f22974bacc9aa67b59a8c810ba5962a946128e1f'
BUDGET = 150.


def validate(args, route):
    # HH_261001 - Explicit new protocol, not a relaxation of the original worker's 30-second contract.
    if (args.host != '127.0.0.1' or args.port != 2440 or args.allow_map_load
            or args.control_transport != 'acknowledged_batch' or args.goal_stop_profile != 'disabled'
            or args.physics_hz != 20 or args.capture_hz != 10 or args.stationary_warmup_sec != 1.2
            or args.max_duration_sec != BUDGET or args.research_speed_cap_mps != 2.5
            or args.target_speed_kmh != 9 or args.research_max_cte_m != 1.
            or args.weather != 'ClearNoon' or args.seed != 0 or route['town'] != 'Town07'
            or route['scenario'] != 'straight' or args.research_weights_sha256 != WEIGHTS_SHA
            or args.research_metadata_sha256 != METADATA_SHA
            or args.output.resolve() != OUTPUT / 'episode'):
        raise ValueError('fixed prospective W1 long-run contract differs')


@contextmanager
def sampled_inputs(directory):
    original = research.SimulationResearchPolicy.step
    counter = 0
    with exact_tensor_archive(directory):
        archived = research.SimulationResearchPolicy.step

        def step(*args, **kwargs):
            nonlocal counter
            selected = counter % 10 == 0
            counter += 1
            return (archived if selected else original)(*args, **kwargs)

        research.SimulationResearchPolicy.step = step
        try:
            yield
        finally:
            research.SimulationResearchPolicy.step = archived


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, add_help=False)
    parser.add_argument('--research-weights', type=Path, required=True)
    parser.add_argument('--research-metadata', type=Path, required=True)
    parser.add_argument('--research-weights-sha256', required=True)
    parser.add_argument('--research-metadata-sha256', required=True)
    extra, remaining = parser.parse_known_args(argv)
    args = short.collector.parse_args(remaining)
    for key, value in vars(extra).items():
        setattr(args, key, value)
    args.research_speed_cap_mps, args.research_max_cte_m = 2.5, 1.
    route = json.loads(args.route_file.read_text())
    validate(args, route)
    torch.set_num_threads(2)
    with identity(CHECKPOINT, short):
        model = research.load_ah_weights(args.research_weights, args.research_metadata,
            weights_sha256=WEIGHTS_SHA, metadata_sha256=METADATA_SHA)
        with sampled_inputs(OUTPUT / 'sampled_exact_runtime_inputs'):
            return short.run(model, args)


if __name__ == '__main__':
    main()
