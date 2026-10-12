"""HH_261001 - Read-only fixed-probe accounting after both warmup-pair endpoints; never select a checkpoint."""

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics


ARMS = ("W0_full_supervision", "W1_warmup_anchor_off")
SCHEMA = "portable_e2e.warmup_supervision_ab_20261001.v1"
RIGHT = "carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f"
LEFT = "carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7"


def read(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mean(values):
    return statistics.fmean(values) if values else None


def compact(row, source, phase):
    selected, oracle, current = row["selected"], row["oracle"], row["current_speed_mps"]
    speeds, costs = row["candidate_speed"], row["candidate_composite_costs"]
    valid, target = row["target_valid"], row["target_speed"]
    if (len(speeds) != 12 or any(len(candidate) != 64 for candidate in speeds)
            or len(costs) != 12 or len(valid) != 64 or len(target) != 64
            or not 0 <= selected < 12 or not 0 <= oracle < 12
            or min(range(12), key=costs.__getitem__) != oracle
            or not all(math.isfinite(v) for candidate in speeds for v in candidate)
            or not all(math.isfinite(v) for v in costs)):
        raise ValueError("probe ABI, original cost oracle, or finite output differs")
    near = [abs(speeds[selected][i]-target[i]) for i in range(5) if valid[i]]
    if not near:
        raise ValueError("near supervision absent from declared fixed TRAIN probe")
    route = "right" if source["episode_id"] == RIGHT else "left" if source["episode_id"] == LEFT else "straight"
    return dict(index=row["index"], token=row["token"], episode_id=source["episode_id"],
        sequence_index=source["sequence_index"], route_group=route, original_phase_id=phase,
        acquisition_phase=source["capture_phase"], current_speed_mps=current,
        selected=selected, oracle=oracle, oracle_agreement=selected == oracle,
        logit_margin=row["logit_margin"], selected_composite_regret=costs[selected]-costs[oracle],
        first5_selected_speed_mae_mps=mean(near), selected_runtime_gate_pass=row["candidate_gates"]["selected_geometry_pass"],
        selected_speed_at_0p3_mps=speeds[selected][2], target_speed_at_0p3_mps=target[2],
        selected_delta_speed_at_0p3_mps=speeds[selected][2]-current,
        target_delta_speed_at_0p3_mps=target[2]-current,
        slows_while_target_accelerates=bool(valid[2] and target[2] > max(0., current)+.1 and speeds[selected][2] < current-.05))


def summarize(records):
    return dict(count=len(records), oracle_agreement=sum(r["oracle_agreement"] for r in records),
        selected_counts=dict(Counter(r["selected"] for r in records)), oracle_counts=dict(Counter(r["oracle"] for r in records)),
        mean_selected_composite_regret=mean([r["selected_composite_regret"] for r in records]),
        mean_first5_speed_mae_mps=mean([r["first5_selected_speed_mae_mps"] for r in records]),
        mean_selected_delta_speed_at_0p3_mps=mean([r["selected_delta_speed_at_0p3_mps"] for r in records]),
        mean_target_delta_speed_at_0p3_mps=mean([r["target_delta_speed_at_0p3_mps"] for r in records]),
        selected_runtime_gate_pass=sum(r["selected_runtime_gate_pass"] for r in records),
        slows_while_target_accelerates=sum(r["slows_while_target_accelerates"] for r in records))


def groups(records):
    selections = dict(all_fixed_probes=records,
        original_immediate_start18=[r for r in records if r["original_phase_id"] == 1],
        protected_stop_hold200=[r for r in records if r["original_phase_id"] == 0],
        moving_speed_1p5_to_2p5=[r for r in records if 1.5 <= r["current_speed_mps"] <= 2.5],
        right_fixed_probes=[r for r in records if r["route_group"] == "right"],
        acquisition_warmup_probes=[r for r in records if r["acquisition_phase"] == "warmup"])
    for route in ("straight", "left", "right"):
        selections[route+"_speed_1p5_to_2p5"] = [r for r in selections["moving_speed_1p5_to_2p5"] if r["route_group"] == route]
        selections[route+"_immediate_start"] = [r for r in selections["original_immediate_start18"] if r["route_group"] == route]
    return {name: summarize(rows) for name, rows in selections.items()}


def consecutive_switches(records):
    lookup = {(row["episode_id"], row["sequence_index"]): row for row in records}
    transitions = []
    for row in records:
        before = lookup.get((row["episode_id"], row["sequence_index"]-1))
        if before is not None and before["selected"] == 5 and row["selected"] == 1:
            transitions.append(dict(episode_id=row["episode_id"], from_index=before["index"], to_index=row["index"],
                from_sequence=before["sequence_index"], to_sequence=row["sequence_index"],
                from_speed_mps=before["current_speed_mps"], to_speed_mps=row["current_speed_mps"],
                new_selected_delta_speed_at_0p3_mps=row["selected_delta_speed_at_0p3_mps"],
                target_delta_speed_at_0p3_mps=row["target_delta_speed_at_0p3_mps"]))
    return transitions


def report(training):
    paths = [training/name for name in ("summary.json", "declaration.json", "provenance_receipt.json",
                                        "both_final_checkpoints_complete.json", "parent_train_probe.json")]
    summary, declaration, proof, finished, parent = map(read, paths)
    if summary["schema"] != SCHEMA or summary["training"] != finished or not summary["source_hashes_match"]:
        raise ValueError("completed immutable matched pair required")
    probes = {"parent_AH": parent}
    for arm in ARMS:
        path = training/arm/"train_probe_after.json"
        probes[arm] = read(path); paths.append(path)
        fit = finished[arm]["fit"]
        if (fit["global_step"] != 2999 or fit["scheduled_steps"] != 3000 or fit["samples_seen"] != 11996
                or fit["skipped_schedule_steps_one_based"] != [1693]):
            raise ValueError("actual optimizer/exposure accounting differs")
    wanted = declaration["fixed_probe_indices"]
    if len(wanted) != 279:
        raise ValueError("fixed279 TRAIN probe set differs")
    models = {}
    parent_by_index = {row["index"]: row for row in parent}
    for name, rows in probes.items():
        if [row["index"] for row in rows] != wanted:
            raise ValueError("probe coverage/order differs between parent and endpoints")
        for row in rows:
            reference = parent_by_index[row["index"]]
            if any(row[key] != reference[key] for key in ("token", "current_speed_mps", "target_speed", "target_xy", "target_valid")):
                raise ValueError("probe label or input identity differs")
        records = [compact(row, proof["rows"][row["index"]], proof["phase_ids"][row["index"]]) for row in rows]
        measures = groups(records)
        if measures["original_immediate_start18"]["count"] != 18 or measures["protected_stop_hold200"]["count"] != 200:
            raise ValueError("fixed start/STOP probe coverage differs")
        models[name] = dict(groups=measures, compact_records=records,
            consecutive_probe_5_to_1_transitions=consecutive_switches(records),
            sparse_probe_transitions_not_closed_loop_progress=True)
    changes = {}
    for arm in ARMS:
        changes[arm] = {group: {field: (value-models["parent_AH"]["groups"][group][field]) if value is not None else None
            for field, value in measures.items() if field in ("oracle_agreement", "mean_selected_composite_regret",
                "mean_first5_speed_mae_mps", "slows_while_target_accelerates", "selected_runtime_gate_pass")}
            for group, measures in models[arm]["groups"].items()}
    return dict(schema="portable_e2e.warmup_supervision_pair_probe_report.v1",
        source_sha256={str(path): sha(path) for path in paths}, models=models, endpoint_minus_parent=changes,
        strict_known_dev_gates=summary["strict_gates"], known_dev_W1_vs_W0=summary["W1_vs_W0"],
        original_masked_warmup=105, protected_stop_hold=200, forward_positions=11996,
        active_exposures={arm: finished[arm]["fit"]["active_supervision_exposures"] for arm in ARMS},
        independent_validation=False, new_neural_forwards=0, optimizer_steps=0,
        training_data_approved=False, model_promoted=False,
        limitation="TRAIN diagnostic only. The18 immediate-start labels are original observations, not proof of full physical admissibility; no live cure follows from probe gains.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("fresh report output required")
    result = report(args.training)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False); stream.write("\n")
    print(json.dumps(dict(output=str(args.output), groups={name: row["groups"] for name, row in result["models"].items()}), indent=2))


if __name__ == "__main__":
    main()
