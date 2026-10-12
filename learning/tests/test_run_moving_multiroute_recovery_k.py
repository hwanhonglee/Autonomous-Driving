# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - CPU-only tests for the sealed K0/K1 performance-recovery runner."""

from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
PATH = _hh_source_path(ROOT, "scripts/e2e/run_moving_multiroute_recovery_k.py")
SPEC = importlib.util.spec_from_file_location("moving_multiroute_recovery_k_test", PATH)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


def example(token, episode, route, tags):
    return SimpleNamespace(
        token=token,
        episode_id=episode,
        route_id=route,
        scenario_tags=tuple(tags),
        domain="carla",
        source_manifest_sha256="a" * 64,
    )


def repeated(episode, count, route, tags):
    return [example(f"{episode}:{index:04d}", episode, route, tags) for index in range(count)]


def cohorts():
    k0 = []
    k0.extend(repeated(m.T07_30_EPISODE, 309, "town07_straight_s0000_p00", ("straight",)))
    k0.extend(repeated(m.T07_15_EPISODE, 328, "town07_straight_s0000_p00", ("straight",)))
    k0.extend(repeated(m.LEFT_EPISODE, 304, "c_track_1_0_7_left_s0000_p01", ("left",)))
    k0.extend(repeated(m.RIGHT_EPISODE, 534, "town01_right_s0001_p00", ("right",)))
    k1 = list(k0)
    new_ids = ("new_t07_p02", "new_ct_p06", "new_ct_p03", "new_t01_p02")
    for episode, route in zip(new_ids, m.NEW_ROUTE_IDS):
        k1.extend(repeated(episode, 328, route, ("straight",)))
    dev = []
    for episode, count in m.DEV_EPISODE_COUNTS.items():
        tags = ("right",) if count == 337 else ("straight",)
        dev.extend(repeated(episode, count, "town03_dev_" + str(count), tags))
    return tuple(k0), tuple(k1), tuple(dev)


def plan_fixture():
    k0, k1, dev = cohorts()
    schedules = {
        m.ARMS[0]: m.schedule_contract(k0, m.ARMS[0]),
        m.ARMS[1]: m.schedule_contract(k1, m.ARMS[1]),
    }
    def view_contract(examples, marker):
        ids = sorted(m.episode_inventory(examples))
        return {
            "path": f"/home/pro6000/personal/hwanhong/dataset/{marker}",
            "dataset_manifest_sha256": marker * 64,
            "loader_fingerprint_sha256": "6" * 64,
            "corpus_fingerprint_sha256": "7" * 64,
            "examples_sha256": "8" * 64,
            "sample_count": len(examples),
            "episode_counts": m.episode_inventory(examples),
            "scope_precheck": {
                "status": "NO_TEST_OR_STATIONARY_REFERENCES_BEFORE_LOADER",
                "episode_ids": ids,
                "episode_reference_count": len(ids),
            },
        }
    return {
        "schema": m.SCHEMA,
        "declared_at_utc": "2026-09-21T00:00:00Z",
        "repo_root": "/home/pro6000/personal/hwanhong/portable_e2e/source",
        "output_root": "/home/pro6000/personal/hwanhong/portable_e2e/runs/k",
        "runner_sha256": "1" * 64,
        "source_sha256": {name: "2" * 64 for name in m.SOURCE_RELATIVES},
        "model": {
            "path": m.MODEL_RELATIVE,
            "file_sha256": "3" * 64,
            "canonical_sha256": "4" * 64,
            "model_id": "portable_e2e.perspective_trajectory.physical_stopmix.v1",
            "parameter_count": m.EXPECTED_PARAMETER_COUNT,
            "fresh_full_parameter_fit": True,
            "seeded_initial_model_state_sha256": "5" * 64,
        },
        "gpu": {"physical_index": 0, "uuid": m.GPU_UUID, "visible_count": 1},
        "views": {
            m.ARMS[0]: view_contract(k0, "a"),
            m.ARMS[1]: view_contract(k1, "b"),
            "Town03_DEV": view_contract(dev, "c"),
        },
        "inventory": {
            "K0_episode_counts": m.episode_inventory(k0),
            "K1_episode_counts": m.episode_inventory(k1),
            "DEV_episode_counts": m.episode_inventory(dev),
            "new_episode_ids": ["new_ct_p03", "new_ct_p06", "new_t01_p02", "new_t07_p02"],
            "new_route_episode_ids": dict(zip(m.NEW_ROUTE_IDS, ("new_t07_p02", "new_ct_p06", "new_ct_p03", "new_t01_p02"))),
        },
        "fit": {
            "seed": m.SEED,
            "optimizer": "AdamW",
            "learning_rate": m.LEARNING_RATE,
            "weight_decay": m.WEIGHT_DECAY,
            "maximum_gradient_norm": m.MAXIMUM_GRADIENT_NORM,
            "batch_size": m.BATCH_SIZE,
            "logical_epochs": m.LOGICAL_EPOCHS,
            "batches_per_epoch": m.BATCHES_PER_EPOCH,
            "steps": m.MAX_STEPS,
            "samples_seen": m.SAMPLES_SEEN,
            "loss_config": m.LOSS_CONFIG.to_dict(),
            "same_initial_model_state_required": True,
            "optimizer_state_loaded": False,
        },
        "sampling": schedules,
        "dev": {
            "split": "val",
            "episodes": m.DEV_EPISODE_COUNTS,
            "sample_count": 665,
            "final_checkpoint_only": True,
            "right_C_maximum": m.RIGHT_MAXIMUM,
            "straight_D_maximum": m.STRAIGHT_MAXIMUM,
            "combined_D_maximum": m.COMBINED_MAXIMUM,
            "geometry_minimum": m.GEOMETRY_MINIMUM,
            "K1_vs_K0_nonregression_count": 9,
        },
        "interpretation": {
            "j_status": "FAIL_4_OF_9",
            "j_combined_ade_excess_m": 0.00035987527213054915,
            "j_right_regressed": True,
            "town03_is_repeatedly_observed_dev": True,
            "single_seed_exploratory_only": True,
        },
        **m.DENIALS,
    }


def metric_report(scale=1.0):
    groups = {
        "town03_right_turn_dev": m.RIGHT_MAXIMUM,
        "town03_straight_dev": m.STRAIGHT_MAXIMUM,
        "combined_val": m.COMBINED_MAXIMUM,
    }
    return {
        "subgroups": {
            group: {"metrics": {metric: value * scale for metric, value in values.items()}}
            for group, values in groups.items()
        },
        "geometry": {
            "selected_result": {"geometry_pass_count": 664},
            "any_candidate_geometry_pass_count": 664,
            "all_candidates_geometry_pass_count": 664,
        },
    }


def test_exact_cohorts_accept_four_successes_and_reject_failed_t01_s02():
    k0, k1, dev = cohorts()
    result = m.validate_examples(k0, k1, dev)
    assert len(result["new_episode_ids"]) == 4
    assert set(result["new_route_episode_ids"]) == set(m.NEW_ROUTE_IDS)
    broken = list(k1)
    broken[-1] = example(
        broken[-1].token,
        broken[-1].episode_id,
        m.EXCLUDED_ROUTE_ID,
        ("straight",),
    )
    with pytest.raises(ValueError, match="excluded failed T01-S02"):
        m.validate_examples(k0, broken, dev)


def test_manifest_scope_is_screened_before_loader_can_touch_test_or_stationary(tmp_path):
    root = tmp_path / "view"
    root.mkdir()
    new_ids = [f"carla_{route}_successful" for route in m.NEW_ROUTE_IDS]
    ids = list(m.K0_EPISODE_COUNTS) + new_ids + list(m.DEV_EPISODE_COUNTS)
    manifest = {
        "episodes": [
            {"episode_id": episode_id, "manifest": f"episodes/{episode_id}/episode.json"}
            for episode_id in ids
        ]
    }
    (root / "dataset.json").write_bytes(m.encoded(manifest))
    scope = m.manifest_scope_precheck(root, m.ARMS[1])
    assert scope["episode_reference_count"] == 10
    manifest["episodes"].append(
        {"episode_id": "hidden_test_episode", "manifest": "episodes/test/episode.json"}
    )
    (root / "dataset.json").write_bytes(m.encoded(manifest))
    with pytest.raises(ValueError, match="TEST reference forbidden"):
        m.manifest_scope_precheck(root, m.ARMS[1])
    manifest["episodes"][-1] = {
        "episode_id": "hidden_stationary_episode",
        "manifest": "episodes/stationary/episode.json",
    }
    (root / "dataset.json").write_bytes(m.encoded(manifest))
    with pytest.raises(ValueError, match="stationary reference forbidden"):
        m.manifest_scope_precheck(root, m.ARMS[1])


def test_k1_epoch_is_exact_2_1_1_and_four_group_cyclic_uniform():
    _k0, k1, _dev = cohorts()
    batches, proof = m.build_epoch_batches(k1, m.ARMS[1], 0)
    assert len(batches) == 750
    assert proof["direction_exposures"] == {"straight": 1500, "left": 750, "right": 750}
    assert set(proof["straight_group_exposures"].values()) == {375}
    assert set(proof["straight_group_exposures"]) == {
        "town07_p00_15_30",
        "town07_p02",
        "c_track_p06_p03",
        "town01_p02",
    }
    for first, second, left, right in batches:
        assert len({first, second, left, right}) == 4
        assert k1[left].episode_id == m.LEFT_EPISODE
        assert k1[right].episode_id == m.RIGHT_EPISODE
        assert "straight" in k1[first].scenario_tags
        assert "straight" in k1[second].scenario_tags


def test_twenty_epoch_exposure_budget_and_determinism_are_exact():
    k0, k1, _dev = cohorts()
    k0_contract = m.schedule_contract(k0, m.ARMS[0])
    k1_contract = m.schedule_contract(k1, m.ARMS[1])
    assert k0_contract == m.schedule_contract(k0, m.ARMS[0])
    assert k1_contract == m.schedule_contract(k1, m.ARMS[1])
    assert k0_contract["steps"] == k1_contract["steps"] == 15000
    assert k0_contract["samples_seen"] == k1_contract["samples_seen"] == 60000
    assert k0_contract["total_straight_group_exposures"] == {"town07_p00_15_30": 30000}
    assert set(k1_contract["total_straight_group_exposures"].values()) == {7500}
    assert k0_contract["total_episode_exposures"][m.LEFT_EPISODE] == 15000
    assert k0_contract["total_episode_exposures"][m.RIGHT_EPISODE] == 15000
    assert k0_contract["total_episode_exposures"][m.T07_30_EPISODE] == 15000
    assert k0_contract["total_episode_exposures"][m.T07_15_EPISODE] == 15000
    hashes = [row["token_batch_schedule_sha256"] for row in k1_contract["epoch_proofs"]]
    assert len(hashes) == 20 and len(set(hashes)) == 20


@pytest.mark.parametrize(
    "mutation",
    ("steps", "seed", "optimizer", "resume", "early", "test", "stationary", "group", "dev", "extra"),
)
def test_plan_fails_closed_on_training_or_scope_mutation(mutation):
    plan = plan_fixture()
    if mutation == "steps":
        plan["fit"]["steps"] = 14999
    elif mutation == "seed":
        plan["fit"]["seed"] = 20260907
    elif mutation == "optimizer":
        plan["fit"]["optimizer"] = "SGD"
    elif mutation == "resume":
        plan["optimizer_resume"] = True
    elif mutation == "early":
        plan["early_stopping"] = True
    elif mutation == "test":
        plan["test_read_or_use"] = True
    elif mutation == "stationary":
        plan["stationary_read_or_use"] = True
    elif mutation == "group":
        plan["sampling"][m.ARMS[1]]["total_straight_group_exposures"]["town01_p02"] -= 1
    elif mutation == "dev":
        plan["dev"]["K1_vs_K0_nonregression_count"] = 6
    else:
        plan["unreviewed"] = True
    with pytest.raises(ValueError):
        m.validate_plan(plan)


def test_plan_accepts_only_exact_k_contract():
    plan = plan_fixture()
    m.validate_plan(plan)
    assert plan["fit"]["loss_config"]["candidate_regret_weight"] == 0.1
    assert plan["fit"]["same_initial_model_state_required"] is True
    assert plan["fit"]["optimizer_state_loaded"] is False
    assert plan["dev"]["final_checkpoint_only"] is True


def test_strict_gate_requires_nine_metrics_and_three_geometry_checks():
    report = metric_report()
    gate = m.strict_gate(report)
    assert gate["pass"] is True
    assert gate["metric_pass_count"] == 9
    assert gate["geometry_pass_count"] == 3
    report["subgroups"]["combined_val"]["metrics"]["ade_6p4s_m"] += 0.000001
    assert m.strict_gate(report)["pass"] is False
    report = metric_report()
    report["geometry"]["all_candidates_geometry_pass_count"] = 663
    assert m.strict_gate(report)["pass"] is False


def test_k1_must_nonregress_all_nine_metrics_including_right():
    k0 = metric_report()
    k1 = metric_report(0.99)
    gate = m.paired_nonregression(k0, k1)
    assert gate["pass"] is True and gate["pass_count"] == 9
    k1["subgroups"]["town03_right_turn_dev"]["metrics"]["speed_mae_6p4s_mps"] = (
        k0["subgroups"]["town03_right_turn_dev"]["metrics"]["speed_mae_6p4s_mps"]
        + 1.0e-12
    )
    gate = m.paired_nonregression(k0, k1)
    assert gate["pass"] is False and gate["pass_count"] == 8


def test_paired_execution_finishes_both_fits_before_any_dev_evaluation():
    calls = []

    def train(arm):
        calls.append(("train", arm))
        return "final:" + arm

    def evaluate(arm, checkpoint):
        calls.append(("evaluate", arm, checkpoint))
        return "metrics:" + arm

    trained, evaluated = m.paired_execution(train, evaluate)
    assert list(trained) == list(m.ARMS)
    assert list(evaluated) == list(m.ARMS)
    assert calls == [
        ("train", m.ARMS[0]),
        ("train", m.ARMS[1]),
        ("evaluate", m.ARMS[0], "final:" + m.ARMS[0]),
        ("evaluate", m.ARMS[1], "final:" + m.ARMS[1]),
    ]


def test_same_seed_produces_identical_full_model_state_and_capacity():
    config = m.ModelConfig.from_mapping(m.read_json(_hh_source_path(ROOT, m.MODEL_RELATIVE)))
    digests = []
    for _index in range(2):
        m._seed_everything(m.SEED, torch.device("cpu"))
        model = m.PerspectiveTrajectoryModel(config)
        assert m.parameter_count(model) == 1_056_362
        assert all(parameter.requires_grad for parameter in model.parameters())
        digests.append(m.tensor_digest(model.state_dict()))
    assert len(set(digests)) == 1


def test_new_source_comments_and_docstrings_follow_hh_marker():
    for path in (PATH, Path(__file__)):
        text = path.read_text(encoding="utf-8")
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith("#"):
                assert stripped.startswith("# HH_260906 - ")
        assert '"""HH_260906 - ' in text
