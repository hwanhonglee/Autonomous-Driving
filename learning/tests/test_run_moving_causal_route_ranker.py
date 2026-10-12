"""HH_260906 - Test J raw-chain, leakage, gate and create-only runner helpers."""

import hashlib
import json
import fcntl
from pathlib import Path
from types import SimpleNamespace

import pytest

from portable_e2e import frozen_causal_route_ranker as core
from portable_e2e.contract import ContractError
from scripts.e2e import run_moving_causal_route_ranker as module


def json_bytes(value):
    """HH_260906 - Encode one compact finite synthetic source document."""
    return (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode()


def sha(path):
    """HH_260906 - Hash one synthetic chain file."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def raw_row(episode, index, command=3, turn=False):
    """HH_260906 - Build one minimal raw row with deliberately ignored forbidden payloads."""
    route = []
    for point in range(4):
        route.append([float(point), float(point if turn else 0.0)])
    return {
        "sample_id": f"{episode}:{index:06d}",
        "episode_id": episode,
        "sequence_index": index,
        "anchor_timestamp_ns": 100 + index,
        "camera_bundle": ["forbidden-image"],
        "labels": {"planning": {"target": "forbidden-target"}},
        "ego": {
            "timestamp_ns": 100 + index,
            "linear_velocity_base_mps": [4.0, 0.0, 0.0],
            "linear_acceleration_base_mps2": [0.0, 0.0, 0.0],
        },
        "navigation": {
            "command": command,
            "command_timestamp_ns": 100 + index,
            "route_source_timestamp_ns": 99 + index,
            "route_polyline_base_m": route,
        },
    }


def raw_tree(tmp_path):
    """HH_260906 - Create a six-episode manifest chain with TRAIN and DEV rows."""
    root = tmp_path / "raw"
    root.mkdir()
    contracts = {}
    references = []
    cache_ids = {"train": [], "val": []}
    specifications = (
        ("train_straight_a", "train", "town07_same_physical_route", False),
        ("train_turn", "train", "c_track", True),
        ("train_straight_b", "train", "town01", False),
        ("dev_right", "val", "town03_dev_right", True),
        ("dev_straight", "val", "town03_dev_straight", False),
        ("dev_extra", "val", "town03_dev_straight", False),
    )
    for episode, split, group, turn in specifications:
        episode_root = root / "episodes" / episode
        episode_root.mkdir(parents=True)
        rows = [
            raw_row(episode, 0, command=0 if turn else 3, turn=turn),
            raw_row(episode, 1, command=3, turn=False),
        ]
        samples = episode_root / "samples.jsonl"
        samples.write_bytes(b"".join(json_bytes(row) for row in rows))
        sample_sha = sha(samples)
        episode_document = {
            "schema_id": "autoware-e2e.common10.episode.v1",
            "episode_id": episode,
            "split": split,
            "sample_count": len(rows),
            "sample_jsonl": "samples.jsonl",
            "sample_jsonl_sha256": sample_sha,
        }
        episode_path = episode_root / "episode.json"
        episode_path.write_bytes(json_bytes(episode_document))
        references.append(
            {
                "episode_id": episode,
                "manifest": f"episodes/{episode}/episode.json",
                "sha256": sha(episode_path),
            }
        )
        contracts[episode] = {
            "split": split,
            "route_group": group,
            "sample_count": len(rows),
            "sample_jsonl_sha256": sample_sha,
        }
        cache_ids[split].extend(row["sample_id"] for row in rows)
    dataset = {
        "schema_id": "autoware-e2e.common10.dataset.v1",
        "contract_id": "common_10hz_v1",
        "episodes": references,
    }
    dataset_path = root / "dataset.json"
    dataset_path.write_bytes(json_bytes(dataset))
    caches = {
        split: SimpleNamespace(sample_ids=tuple(values))
        for split, values in cache_ids.items()
    }
    return root, contracts, caches, sha(dataset_path)


def metric_row(values, count):
    """HH_260906 - Build one complete fixed-gate metric row."""
    return {
        "sample_count": count,
        "metrics": dict(values),
        "metric_counts": {name: count for name in values},
        "selected_histogram": [0] * 12,
        "composite_oracle_histogram": [0] * 12,
        "selected_composite_oracle_agreement_count": 0,
    }


def test_manifest_chain_builds_ordered_input_only_sidecars_and_causal_latch(tmp_path):
    """HH_260906 - Verify dataset to episode to samples before exact cache-ID alignment."""
    root, contracts, caches, dataset_sha = raw_tree(tmp_path)
    sidecars, proof = module.build_causal_sidecars(
        root,
        caches,
        core,
        expected_dataset_sha256=dataset_sha,
        episode_contract=contracts,
        expected_total_count=12,
    )
    assert proof["dataset_to_episode_to_samples_chain_verified"] is True
    assert proof["cache_sample_count"] == proof["raw_sample_count"] == 12
    assert proof["ordered_sample_coverage"] == {"train": True, "val": True}
    assert sidecars["train"].sample_ids == caches["train"].sample_ids
    turn_indices = [
        index
        for index, sample_id in enumerate(sidecars["train"].sample_ids)
        if sample_id.startswith("train_turn:")
    ]
    assert turn_indices == [2, 3]
    assert not sidecars["train"].straight_ranker_mask[turn_indices].any()
    assert sidecars["train"].straight_ranker_mask[[0, 1, 4, 5]].all()
    assert len(proof["episode_proofs"]) == 6
    assert proof["forbidden_ranker_fields_serialized"] == []
    assert proof["route_source_timestamp_causality"] == {
        "verified_count": 12,
        "future_count": 0,
        "anchor_minus_route_source_minimum_ns": 1,
        "anchor_minus_route_source_maximum_ns": 1,
    }


def test_projected_parser_never_decodes_forbidden_values(monkeypatch):
    """HH_260906 - Skip a nonrepresentable poison number in labels without materialization."""
    row = raw_row("episode", 0)
    line = json.dumps(row, sort_keys=True, separators=(",", ":"))
    line = line[:-1] + ',"poison":{"huge":1e999999,"nested":[{"value":"ignored"}]}}'
    contexts = []
    decoded_payloads = []
    original = module._decode_slice
    original_loads = module.json.loads

    def audited_decode(text, span, context):
        """HH_260906 - Record every explicitly projected value decode context."""
        contexts.append(context)
        return original(text, span, context)

    def audited_loads(payload, *args, **kwargs):
        """HH_260906 - Record every JSON fragment that reaches materialization."""
        decoded_payloads.append(str(payload))
        return original_loads(payload, *args, **kwargs)

    monkeypatch.setattr(module.json, "loads", audited_loads)
    monkeypatch.setattr(module, "_decode_slice", audited_decode)
    projected = module.read_projected_json_line(line, "row")
    assert projected["sample_id"] == "episode:000000"
    assert set(projected) == {
        "sample_id",
        "episode_id",
        "sequence_index",
        "anchor_timestamp_ns",
        "ego",
        "navigation",
    }
    assert contexts
    assert contexts == [
        "row.sample_id",
        "row.episode_id",
        "row.sequence_index",
        "row.anchor_timestamp_ns",
        "row.ego.linear_acceleration_base_mps2",
        "row.ego.linear_velocity_base_mps",
        "row.ego.timestamp_ns",
        "row.navigation.command",
        "row.navigation.command_timestamp_ns",
        "row.navigation.route_polyline_base_m",
        "row.navigation.route_source_timestamp_ns",
    ]
    assert all(
        word not in " ".join(contexts).lower()
        for word in ("label", "camera", "poison", "future", "image", "target")
    )
    decoded_text = "\n".join(decoded_payloads)
    assert "1e999999" not in decoded_text
    assert "forbidden-target" not in decoded_text
    assert '"ignored"' not in decoded_text


def test_projected_parser_still_rejects_duplicate_keys_inside_skipped_payload():
    """HH_260906 - Validate skipped JSON syntax and duplicate keys without decoding values."""
    row = raw_row("episode", 0)
    line = json.dumps(row, sort_keys=True, separators=(",", ":"))
    line = line[:-1] + ',"poison":{"same":1,"same":2}}'
    with pytest.raises(ValueError, match="duplicates key same"):
        module.read_projected_json_line(line, "row")


def test_projected_parser_rejects_escaped_duplicate_key_inside_skipped_payload():
    """HH_260906 - Decode skipped object keys solely to reject escaped duplicates."""
    row = raw_row("episode", 0)
    line = json.dumps(row, sort_keys=True, separators=(",", ":"))
    line = line[:-1] + ',"poison":{"same":1,"s\\u0061me":2}}'
    with pytest.raises(ValueError, match="duplicates key same"):
        module.read_projected_json_line(line, "row")


def test_projected_parser_rejects_allowlisted_float_overflow_and_future_route_time():
    """HH_260906 - Reject nonfinite projected state and future-sourced route geometry."""
    row = raw_row("episode", 0)
    line = json.dumps(row, sort_keys=True, separators=(",", ":"))
    line = line.replace('"linear_velocity_base_mps":[4.0', '"linear_velocity_base_mps":[1e9999')
    with pytest.raises(ValueError, match="nonfinite float"):
        module.read_projected_json_line(line, "row")
    row = raw_row("episode", 0)
    row["navigation"]["route_source_timestamp_ns"] = 101
    projected = module.read_projected_json_line(
        json.dumps(row, sort_keys=True, separators=(",", ":")), "row"
    )
    with pytest.raises(ContractError, match="future"):
        core.extract_causal_row(projected)


@pytest.mark.parametrize(
    "poison",
    [
        '[1,]',
        '[1 2]',
        '{"a":1,}',
        '{"a":1 "b":2}',
        '{"a":1',
        '{"a" 1}',
        '"bad\\q"',
        '"bad\\u12xz"',
        '"unterminated',
        '[1',
        '01',
        '1.',
        '+1',
        'NaN',
        'Infinity',
        '-Infinity',
        'truex',
        '{} {}',
    ],
)
def test_projected_parser_rejects_malformed_skipped_values(poison):
    """HH_260906 - Validate forbidden JSON syntax recursively without loading its value."""
    row = raw_row("episode", 0)
    line = json.dumps(row, sort_keys=True, separators=(",", ":"))
    line = line[:-1] + ',"poison":' + poison + "}"
    with pytest.raises(ValueError):
        module.read_projected_json_line(line, "row")


def test_read_json_rejects_exponent_overflow(tmp_path):
    """HH_260906 - Keep manifest numbers finite even when JSON float syntax is valid."""
    path = tmp_path / "overflow.json"
    path.write_text('{"value":1e9999}')
    with pytest.raises(ValueError, match="nonfinite float"):
        module.read_json(path)


def test_manifest_chain_rejects_changed_samples_after_episode_hash_is_sealed(tmp_path):
    """HH_260906 - Fail closed when sample bytes no longer match the episode manifest."""
    root, contracts, caches, dataset_sha = raw_tree(tmp_path)
    sample = root / "episodes" / "train_straight_a" / "samples.jsonl"
    sample.write_bytes(sample.read_bytes() + b"\n")
    with pytest.raises(ValueError, match="sample JSONL SHA256"):
        module.build_causal_sidecars(
            root,
            caches,
            core,
            expected_dataset_sha256=dataset_sha,
            episode_contract=contracts,
            expected_total_count=12,
        )


def test_manifest_chain_rejects_cache_coverage_gap(tmp_path):
    """HH_260906 - Reject even one raw sample not represented by the exact H cache IDs."""
    root, contracts, caches, dataset_sha = raw_tree(tmp_path)
    caches["train"] = SimpleNamespace(sample_ids=caches["train"].sample_ids[:-1])
    with pytest.raises(ValueError, match="coverage"):
        module.build_causal_sidecars(
            root,
            caches,
            core,
            expected_dataset_sha256=dataset_sha,
            episode_contract=contracts,
            expected_total_count=12,
        )


def test_manifest_chain_rejects_cache_order_difference(tmp_path):
    """HH_260906 - Bind the raw and H sample orders instead of silently reordering rows."""
    root, contracts, caches, dataset_sha = raw_tree(tmp_path)
    caches["train"] = SimpleNamespace(
        sample_ids=tuple(reversed(caches["train"].sample_ids))
    )
    with pytest.raises(ValueError, match="ordered sample coverage"):
        module.build_causal_sidecars(
            root,
            caches,
            core,
            expected_dataset_sha256=dataset_sha,
            episode_contract=contracts,
            expected_total_count=12,
        )


def test_projected_parser_requires_route_source_timestamp():
    """HH_260906 - Refuse route geometry without its causal source timestamp."""
    row = raw_row("episode", 0)
    del row["navigation"]["route_source_timestamp_ns"]
    with pytest.raises(ValueError, match="projected causal field"):
        module.read_projected_json_line(
            json.dumps(row, sort_keys=True, separators=(",", ":")), "row"
        )


def test_sidecar_payload_has_explicit_ranker_allowlist_and_no_forbidden_tensor():
    """HH_260906 - Keep IDs and fold groups as metadata, never numerical ranker features."""
    rows = {
        raw_row("s", index)["sample_id"]: core.extract_causal_row(
            raw_row("s", index)
        )
        for index in range(2)
    }
    value = core.make_sidecar(
        "train",
        tuple(rows),
        rows,
        {sample_id: "town07_same_physical_route" for sample_id in rows},
        {sample_id: True for sample_id in rows},
    )
    payload = module.sidecar_payload(value, core)
    assert payload["ranker_input_tensor_keys"] == [
        "route_xy",
        "route_mask",
        "ego_speed_mps",
        "ego_acceleration_mps2",
    ]
    assert set(payload["tensors"]) == set(core._TENSOR_FIELDS)
    assert payload[
        "contains_target_label_future_ego_image_fused_map_or_episode_tensor"
    ] is False
    tensor_key_text = " ".join(payload["tensors"]).lower()
    assert all(
        word not in tensor_key_text
        for word in ("target", "label", "future", "image", "fused", "map", "episode")
    )


def test_strict_gate_requires_all_nine_metrics_geometry_and_preservation():
    """HH_260906 - Preserve the prior absolute metric thresholds without promotion."""
    metrics = {
        "town03_right_turn_dev": metric_row(module.RIGHT_MAXIMUM, 337),
        "town03_straight_dev": metric_row(module.STRAIGHT_MAXIMUM, 328),
        "combined_val": metric_row(module.COMBINED_MAXIMUM, 665),
    }
    geometry = {
        "selected_geometry_pass_count": 664,
        "any_candidate_geometry_pass_count": 664,
        "all_candidates_geometry_pass_count": 664,
        "sample_count": 665,
    }
    proof = {
        split: {
            "mode_change_count": 0,
            "fallback_row_logit_mismatch_count": 0,
            "reranked_drive_group_maximum_mismatch_count": 0,
            "pass": True,
        }
        for split in ("train", "val")
    }
    result = module.strict_gate(metrics, geometry, proof)
    assert result["pass"] is True
    assert result["metric_pass_count"] == result["metric_check_count"] == 9
    assert result["geometry_pass_count"] == result["geometry_check_count"] == 3
    assert result["preservation_pass_count"] == result["preservation_check_count"] == 2
    assert result["promotion_eligible"] is False
    proof["val"]["fallback_row_logit_mismatch_count"] = 1
    proof["val"]["pass"] = False
    assert module.strict_gate(metrics, geometry, proof)["pass"] is False


def test_create_only_json_and_manifest(tmp_path):
    """HH_260906 - Never overwrite a prior plan, report or checksum inventory."""
    output = tmp_path / "output"
    module.write_new(output / "one.json", {"value": 1})
    with pytest.raises(FileExistsError):
        module.write_new(output / "one.json", {"value": 2})
    proof = module.manifest(output)
    assert proof["file_count_excluding_manifest"] == 1
    assert (output / "SHA256SUMS").is_file()
    with pytest.raises(FileExistsError):
        module.manifest(output)


def test_exact_manifest_verifier_rejects_tampering(tmp_path):
    """HH_260906 - Rehash every sealed I-context artifact and its full inventory."""
    root = tmp_path / "sealed"
    module.write_new(root / "one.json", {"value": 1})
    sealed = module.manifest(root)
    proof = module.verify_exact_manifest(root, sealed["sha256"], "synthetic")
    assert proof["all_files_verified"] is True
    (root / "one.json").write_text("tampered")
    with pytest.raises(ValueError, match="artifact differs"):
        module.verify_exact_manifest(root, sealed["sha256"], "synthetic")


def test_external_gpu_lock_requires_the_caller_held_common_flock(tmp_path, monkeypatch):
    """HH_260906 - Prove J cannot start outside the shared campaign GPU0 lock."""
    workspace = tmp_path / "portable_e2e"
    lock_path = workspace / module.GPU_LOCK_RELATIVE
    lock_path.parent.mkdir(parents=True)
    lock_path.write_bytes(b"")
    monkeypatch.setenv("PORTABLE_E2E_EXTERNAL_FLOCK_PATH", str(lock_path.absolute()))
    with lock_path.open("rb") as holder:
        fcntl.flock(holder.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        proof = module.verify_external_gpu_lock(workspace)
        assert proof["external_lock_held"] is True
        fcntl.flock(holder.fileno(), fcntl.LOCK_UN)
    with pytest.raises(ValueError, match="not externally held"):
        module.verify_external_gpu_lock(workspace)


def test_fixed_parent_gpu_counts_and_denials():
    """HH_260906 - Pin J to the exact raw parent, physical groups and GPU0."""
    assert module.RAW_DATASET_SHA256 == core.RAW_DATASET_SHA256
    assert module.GPU_UUID == "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
    assert module.GPU_LOCK_RELATIVE == "runs/campaigns/.gpu0_training.lock"
    assert module.I_PLAN_SHA256 == "1fee39ef45d008f8082ad0e501ef1467e793c869469854e29d78c85495788a82"
    assert module.I_SUMMARY_SHA256 == "67d13d580c43bc42bcee818339c64e207a84d7ed66d848176c3dac895126b0cf"
    assert module.I_MANIFEST_SHA256 == "72b741f9bfe815e88cdcc85a0b7067e965230d9e3581b845226bc57926c1ad08"
    assert module.EXPECTED_TRAIN_DRIVE_BY_GROUP == {
        "town07_same_physical_route": 625,
        "c_track": 289,
        "town01": 318,
    }
    assert module.EXPECTED_TRAIN_POLICY_DRIVE_BY_GROUP == {
        "town07_same_physical_route": 625,
        "c_track": 0,
        "town01": 156,
    }
    assert module.EXPECTED_DEV_POLICY_DRIVE_BY_GROUP == {
        "town03_dev_right": 129,
        "town03_dev_straight": 328,
    }
    assert sum(module.EXPECTED_TRAIN_DRIVE_BY_GROUP.values()) == 1232
    assert all(value is False for value in module.DENIALS.values())
