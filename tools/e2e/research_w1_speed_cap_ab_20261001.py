"""HH_261001 - Compare two explicit W1 simulation speed caps with one common teacher policy."""

import argparse
import json
from pathlib import Path
import torch
from portable_e2e import simulation_research as research
from scripts.e2e import collect_ah_simulation_research as short
from scripts.e2e import collect_w1_town07_long_20261001 as endpoint
from scripts.e2e.research_warmup_supervision_candidate_20261001 import identity

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / 'docs/assets/validation/2026-10-01/warmup_supervision_ab/SPEED_CAP_AB_PLAN.md'
CAPS = {'C9':2.5, 'C30':30./3.6}
EXPORTS = ROOT / 'storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1/exports'


def output(arm):
    if arm not in CAPS:
        raise ValueError('undeclared cap arm')
    return ROOT / ('storage/experiments/validation/2026-10-01/warmup_W1_speed_cap_' + arm + '_v1')


def validate(args, route, arm):
    # HH_261001 - The common teacher change is explicit; the safety override is never disabled.
    if (arm not in CAPS or args.host != '127.0.0.1' or args.port != 2440 or args.allow_map_load
            or args.control_transport != 'acknowledged_batch' or args.goal_stop_profile != 'disabled'
            or args.physics_hz != 20 or args.capture_hz != 10 or args.stationary_warmup_sec != 1.2
            or args.max_duration_sec != 20 or args.research_speed_cap_mps != CAPS[arm]
            or args.target_speed_kmh != 30 or args.research_max_cte_m != 1.
            or args.weather != 'ClearNoon' or args.seed != 0 or route['town'] != 'Town07'
            or route['scenario'] != 'straight' or args.output.resolve() != output(arm)/'episode'):
        raise ValueError('fixed W1 cap comparison contract differs')


def worker(arm, remaining):
    args = short.collector.parse_args(remaining)
    args.research_speed_cap_mps, args.research_max_cte_m = CAPS[arm],1.
    validate(args,json.loads(args.route_file.read_text()),arm)
    torch.set_num_threads(2)
    with identity(endpoint.CHECKPOINT,short):
        model = research.load_ah_weights(EXPORTS/'W1_warmup_anchor_off.npz',
            EXPORTS/'W1_warmup_anchor_off.json',weights_sha256=endpoint.WEIGHTS_SHA,
            metadata_sha256=endpoint.METADATA_SHA)
        with endpoint.sampled_inputs(output(arm)/'sampled_exact_runtime_inputs'):
            return short.run(model,args)


def trial(mode,arm):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    from scripts.e2e import run_owned_ah_town07_research_v2_20261001 as outcome
    original = owner.collector_command
    old_weights,old_metadata = owner.WEIGHTS,owner.METADATA
    owner.WEIGHTS,owner.METADATA = EXPORTS/'W1_warmup_anchor_off.npz',EXPORTS/'W1_warmup_anchor_off.json'
    owner.PINS = {p:sha for p,sha in owner.PINS.items() if p not in (old_weights,old_metadata)}
    owner.PINS.update({owner.WEIGHTS:endpoint.WEIGHTS_SHA,owner.METADATA:endpoint.METADATA_SHA,
                      PLAN:owner.digest(PLAN)})
    owner.OUTPUT = output(arm)

    def command():
        argv = original()
        index = argv.index('scripts.e2e.collect_ah_simulation_research')
        start = argv.index(str(owner.OUTPUT/'episode'))
        rest = argv[start:]
        rest[rest.index('--target-speed-kmh')+1] = '30'
        return argv[:index]+['scripts.e2e.research_w1_speed_cap_ab_20261001','--',
            '--mode','worker','--arm',arm]+rest

    owner.collector_command = command
    with identity(endpoint.CHECKPOINT,short):
        research.load_ah_weights(owner.WEIGHTS,owner.METADATA,
            weights_sha256=endpoint.WEIGHTS_SHA,metadata_sha256=endpoint.METADATA_SHA)
        plan = owner.preflight()
        argv = command()
        args = short.collector.parse_args(argv[argv.index(str(owner.OUTPUT/'episode')):])
        args.research_speed_cap_mps,args.research_max_cte_m = CAPS[arm],1.
        validate(args,json.loads(owner.ROUTE.read_text()),arm)
        plan.update(schema='portable_e2e.w1_cap_ab.v1',arm=arm,source_checkpoint_sha256=endpoint.CHECKPOINT,
            research_speed_cap_kph=CAPS[arm]*3.6,common_teacher_target_kph=30,
            preregistered_plan_sha256=owner.PINS[PLAN],
            interpretation='One of two new matched cap regimes; prior teacher9 runs are not this control.')
        if mode == 'preflight':
            if owner.OUTPUT.exists():
                raise ValueError('fresh cap comparison output required')
            return plan
        # HH_261001 - Owner verifies the plan before/after execution; retain its exact study bytes separately.
        plan_bytes = PLAN.read_bytes()
        owner.execute(plan)
        with (owner.OUTPUT/'study_plan.md').open('xb') as stream:
            stream.write(plan_bytes)
        result = outcome.final_report(owner.OUTPUT)
        result.update(arm=arm,source_checkpoint_sha256=endpoint.CHECKPOINT,
            cap_kph=CAPS[arm]*3.6,teacher_kph=30,learned_goal_stop_claimed=False)
        owner.write_json(owner.OUTPUT/'owner_diagnostic_result.json',result)
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--mode',required=True,choices=('preflight','execute','worker'))
    parser.add_argument('--arm',required=True,choices=tuple(CAPS))
    args,remaining = parser.parse_known_args(argv)
    if args.mode == 'worker':
        return worker(args.arm,remaining)
    if remaining:
        parser.error('unexpected owner arguments')
    result = trial(args.mode,args.arm)
    print(json.dumps(result,indent=2))
    return 0 if args.mode=='preflight' or result['status'] in ('DIAGNOSTIC_BUDGET_REACHED','DIAGNOSTIC_CAPTURE_COMPLETE') else 1


if __name__ == '__main__':
    raise SystemExit(main())
