#!/usr/bin/env python3
"""HH_260906 - Replay final K checkpoints for post-hoc candidate diagnostics without training."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import torch
from torch.utils.data import DataLoader
from portable_e2e.model import ModelConfig, PerspectiveTrajectoryModel
from portable_e2e.torch_dataset import Common10TorchDataset
from scripts.e2e import run_moving_multiroute_recovery_k as r


def diagnose(run_root, output):
    """HH_260906 - Keep this second diagnostic pass separate from the sealed first-pass decision."""
    plan = r.read_json(run_root / 'plan.json')
    summary = r.read_json(run_root / 'summary.json')
    r.require(summary['status'] == 'COMPLETE_NOT_PROMOTED', 'both final fits and evaluations required')
    r.require(r.digest(run_root / 'plan.json') == summary['plan_sha256'], 'plan changed')
    r.require(r.digest(run_root / 'decision.json') == summary['decision_sha256'], 'decision changed')
    r.validate_plan(plan)
    r.verify_sources(plan)
    workspace = r.check_run_environment(plan)
    r.require(output.absolute().is_relative_to(workspace / 'runs/diagnostics'), 'output outside personal diagnostics')
    r.require(not output.exists(), 'fresh diagnostic output required')
    with r.gpu0_lease(workspace):
        gpu = r.preflight_gpu()
        loaded, examples, scope = r._load_view(Path(plan['views']['Town03_DEV']['path']), 'val', 'Town03_DEV')
        r._verify_loaded_view(plan['views']['Town03_DEV'], loaded, examples)
        config = ModelConfig.from_mapping(r.read_json(ROOT / r.MODEL_RELATIVE))
        dataset = Common10TorchDataset(examples, config, verify_image_sha256=True, split='val')
        device = torch.device('cuda:0')
        output.mkdir(parents=True)
        r.write_new(output / 'started.json', dict(status='POSTHOC_DIAGNOSTIC_STARTED',
            gpu=gpu, evaluation_pass_index=2, training_performed=False,
            original_decision_modified=False, diagnostic_source_sha256=r.digest(Path(__file__))))
        results = {}
        for arm in r.ARMS:
            ref = summary['training'][arm]
            path = run_root / ref['checkpoint']
            r.require(r.digest(path) == ref['checkpoint_sha256'], 'checkpoint digest differs')
            payload = torch.load(path, map_location='cpu')
            model = PerspectiveTrajectoryModel(config).to(device)
            model.load_state_dict(payload['model_state_dict'], strict=True)
            model.eval().requires_grad_(False)
            r.require(r.tensor_digest(model.state_dict()) == ref['final_model_state_sha256'], 'model changed')
            rows, traces = [], []
            within_episode = Counter()
            with torch.no_grad():
                for raw in DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0):
                    batch = r._move_batch(raw, device)
                    xy, speed, logits = model(batch['images'], batch['calibration'], batch['ego_history'],
                        batch['ego_history_mask'], batch['route_xy'], batch['route_mask'])
                    r.require(bool(batch['target_valid'].all().item()), 'diagnostic requires complete target horizons')
                    errors = torch.linalg.vector_norm(xy - batch['target_xy'][:, None], dim=-1).mean(-1)
                    chosen, oracle = logits.argmax(-1), errors.argmin(-1)
                    for index, episode in enumerate(raw['episode_id']):
                        c, o = int(chosen[index]), int(oracle[index])
                        rank = within_episode[episode]
                        within_episode[episode] += 1
                        row = dict(sample_id=raw['sample_id'][index], episode_id=episode, episode_index=rank,
                            selected_index=c, oracle_index=o, selected_ade_m=float(errors[index,c]),
                            oracle_ade_m=float(errors[index,o]),
                            candidate_ade_m=errors[index].cpu().tolist(), logits=logits[index].cpu().tolist())
                        rows.append(row)
                        if rank in (0, 82, 164, 246, 327):
                            traces.append(dict(row,
                                selected_xy=xy[index,c].cpu().tolist(), oracle_xy=xy[index,o].cpu().tolist(),
                                target_xy=batch['target_xy'][index].cpu().tolist(),
                                route_xy=batch['route_xy'][index][batch['route_mask'][index].bool()].cpu().tolist()))
            original = r.read_json(run_root / summary['evaluations'][arm]['path'])
            groups = {}
            for group, episode in zip(('town03_right_turn_dev','town03_straight_dev'), r.DEV_EPISODE_COUNTS):
                subset = [row for row in rows if row['episode_id'] == episode]
                groups[group] = dict(count=len(subset),
                    selected_counts=dict(Counter(row['selected_index'] for row in subset)),
                    oracle_counts=dict(Counter(row['oracle_index'] for row in subset)),
                    selected_matches_oracle=sum(row['selected_index']==row['oracle_index'] for row in subset),
                    regret_above_1m=sum(row['selected_ade_m']-row['oracle_ade_m']>1 for row in subset),
                    mean_selected_ade_m=sum(row['selected_ade_m'] for row in subset)/len(subset),
                    mean_oracle_ade_m=sum(row['oracle_ade_m'] for row in subset)/len(subset))
                for measured, expected in [('mean_selected_ade_m','ade_6p4s_m'),('mean_oracle_ade_m','oracle_ade_m')]:
                    r.require(abs(groups[group][measured]-original['subgroups'][group]['metrics'][expected])<1e-5,
                              'post-hoc replay does not reproduce original DEV metrics')
            r.write_new(output / (arm + '.json'), dict(rows=rows, traces=traces, groups=groups))
            results[arm] = groups
            del model
            torch.cuda.empty_cache()
        report = dict(status='POSTHOC_REPLAY_MATCHED_ORIGINAL_METRICS', results=results,
            source_summary_sha256=r.digest(run_root/'summary.json'), evaluation_pass_index=2,
            training_performed=False, original_decision_modified=False,
            oracle_uses_future_labels_for_analysis_only=True, release_approved=False)
        r.write_new(output / 'summary.json', report)
        return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(diagnose(args.run_root, args.output)))
