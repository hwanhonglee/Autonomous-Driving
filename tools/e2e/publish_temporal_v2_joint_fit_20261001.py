"""HH_261001 - Publish complete joint-fit evidence and known-scene regression without promotion."""

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "storage/experiments/training/2026-10-01/temporal_v2_low_speed_joint_v2"
OUTPUT = ROOT / "docs/assets/validation/2026-10-01/temporal_v2_low_speed_joint_results_v2"


def require(value, reason):
    if not value:
        raise ValueError(reason)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def main():
    require(not OUTPUT.exists(), "create-only publication directory required")
    paths = [RUN / "summary.json", RUN / "declaration.json", RUN / "history.jsonl"]
    paths += [RUN / name for name in ("train_joint_before.json", "train_joint_step_003000.json",
        "known_dev_raw_before.json", "known_dev_raw_step_003000.json")]
    hashes = {str(path.relative_to(ROOT)): sha(path) for path in paths}
    result, declaration = read(paths[0]), read(paths[1])
    journal = [json.loads(line) for line in paths[2].read_text().splitlines()]
    require(result["optimizer_updates"] == len(journal) == 3000
            and [row["step"] for row in journal] == list(range(1, 3001))
            and result["examples_seen"] == 12000 and result["source_hashes_match"] is True
            and result["model_promoted"] is False and result["main_training"] is False,
            "actual bounded-fit completion evidence differs")
    for name, expected in declaration["source_sha256"].items():
        require(sha(ROOT / name) == sha(RUN / "source" / name) == expected,
                "frozen source mismatch: " + name)
    before, after, dev_before, dev_after = [read(path) for path in paths[3:]]
    require(before["full_fixed_denominator"] == after["full_fixed_denominator"] == 327
            and len(before["rows"]) == len(after["rows"]) == 327
            and dev_before["independent_validation"] is dev_after["independent_validation"] is False,
            "TRAIN or previously used DEV denominator differs")
    require([row["token"] for row in before["rows"]] == [row["token"] for row in after["rows"]]
            and [row["token"] for row in dev_before["rows"]] == [row["token"] for row in dev_after["rows"]],
            "fixed evaluation membership changed")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    OUTPUT.mkdir(parents=True)
    figure, axes = plt.subplots(2, 2, figsize=(12, 8))
    steps = [row["step"] for row in journal]
    for name in ("raw_curvature", "raw_preferred_speed", "temporal_xy", "temporal_speed"):
        axes[0, 0].plot(steps, [row["loss"][name] for row in journal], linewidth=.7, alpha=.7, label=name)
    axes[0, 0].set(yscale="log", xlabel="actual optimizer updates", ylabel="batch loss", title="All 3000 recorded updates")
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].bar(["Before", "After 3000"], [before["mean_ade_6p4s_m"], after["mean_ade_6p4s_m"]], color=["#89939e", "#00897b"])
    axes[0, 1].set(ylabel="6.4s ADE (m)", title="All 327 TRAIN anchors; not independent")
    axes[1, 0].bar(["Before", "After 3000"], [before["mean_instantaneous_speed_mae_mps"], after["mean_instantaneous_speed_mae_mps"]], color=["#89939e", "#d95f02"])
    axes[1, 0].set(ylabel="instantaneous speed MAE (m/s)", title="TRAIN speed error increased")
    axes[1, 1].bar(["Before", "After 3000"], [dev_before["mean_total"], dev_after["mean_total"]], color=["#89939e", "#d95f02"])
    axes[1, 1].set(ylabel="raw spatial loss (not trajectory ADE)", title="Previously used DEV8 regression; not promoted")
    for axis in axes.flat:
        axis.grid(axis="y", alpha=.2)
    figure.suptitle("Actual GPU0 joint fit on one low-speed episode; no driving or feature approval")
    figure.tight_layout()
    figure.savefig(OUTPUT / "actual_joint_training_and_regression.png", dpi=140)
    plt.close(figure)
    receipt = dict(comment="HH_261001 - Keep complete training and evaluation denominators separate from control success.",
        optimizer_updates=3000, examples_seen=12000, distinct_training_episodes=1,
        evaluated_train_anchors=327, evaluated_known_dev_raw_anchors=8,
        independent_test_opened=False, model_promoted=False, source_sha256=hashes,
        publisher_sha256=sha(Path(__file__)), plot_sha256=sha(OUTPUT / "actual_joint_training_and_regression.png"))
    require(all(sha(ROOT / name) == expected for name, expected in hashes.items()), "input changed during publication")
    with (OUTPUT / "publication_provenance.json").open("x") as stream:
        json.dump(receipt, stream, indent=2, allow_nan=False)
    print(json.dumps(dict(status="COMPLETE_FIT_AND_REGRESSION_PUBLISHED_NOT_PROMOTED", output=str(OUTPUT))))


if __name__ == "__main__":
    main()
