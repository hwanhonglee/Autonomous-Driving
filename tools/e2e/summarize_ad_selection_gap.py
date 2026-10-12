"""HH_260906 - Separate candidate-set oracle error from selection gap, offline only."""
import argparse
import hashlib
import json
from pathlib import Path


def summarize(baseline, continuation):
    rows = {}
    for group in ("town03_right_turn_dev", "town03_straight_dev"):
        group_rows = {}
        for name, evaluation in (("original_k1", baseline), ("ad", continuation["evaluation"])):
            m = evaluation["subgroups"][group]["metrics"]
            selected, oracle = m["selected_ade_m"], m["oracle_ade_m"]
            if not 0 <= oracle <= selected + 1e-6:
                raise ValueError("invalid oracle/selection ordering")
            group_rows[name] = dict(selected_ade_m=selected, oracle_ade_m=oracle,
                                    selection_gap_m=selected-oracle)
        group_rows["delta_ad_minus_k1"] = {
            k: group_rows["ad"][k]-group_rows["original_k1"][k]
            for k in group_rows["ad"]}
        rows[group] = group_rows
    return dict(groups=rows, trained=False, additional_inference=False,
                oracle_uses_future_truth=True, deployment_approved=False,
                interpretation="Aggregate diagnostic, not per-scene causality; oracle is not a deployable selector")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("baseline", type=Path); p.add_argument("continuation", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    result = summarize(json.loads(a.baseline.read_text()), json.loads(a.continuation.read_text()))
    result["input_sha256"] = {str(f): hashlib.sha256(f.read_bytes()).hexdigest()
                              for f in (a.baseline, a.continuation)}
    with a.output.open("x") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))
