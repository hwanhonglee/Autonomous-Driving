from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

from portable_e2e.dataset import fingerprint_examples
from portable_e2e.dataset import load_training_examples
from tests.test_portable_e2e_contract import _sha
from tests.test_portable_e2e_contract import _write_dataset
from tests.test_portable_e2e_contract import _write_json


def _set_source_metadata(
    root: Path,
    source_updates: dict[str, object],
) -> Path:
    dataset_path = root / "dataset.json"
    dataset = json.loads(dataset_path.read_text(encoding="utf-8"))
    reference = dataset["episodes"][0]
    episode_path = root / reference["manifest"]
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    source_path = episode_path.parent / episode["source_provenance"][
        "source_manifest_file"
    ]
    source_manifest = json.loads(source_path.read_text(encoding="utf-8"))
    source_manifest.update(source_updates)
    _write_json(source_path, source_manifest)
    episode["source_provenance"]["source_manifest_sha256"] = _sha(source_path)
    _write_json(episode_path, episode)
    reference["sha256"] = _sha(episode_path)
    _write_json(dataset_path, dataset)
    return source_path


def test_loader_preserves_episode_and_source_manifest_metadata(tmp_path: Path) -> None:
    root = tmp_path / "dataset"
    _write_dataset(root)
    source_path = _set_source_metadata(
        root,
        {
            "source_dataset_id": "nuplan",
            "source_dataset_version": "v1.1-mini",
            "license_id": "CC-BY-NC-SA-4.0",
            "future_schema": {"may_change": True},
        },
    )

    loaded = load_training_examples(root, split="train", mode="schema")

    assert len(loaded.examples) == 11
    assert {
        (
            example.domain,
            example.source_dataset_id,
            example.source_dataset_version,
            example.license_id,
            example.scenario_tags,
            example.map_id,
            example.site_id,
            example.route_id,
            example.source_manifest_sha256,
        )
        for example in loaded.examples
    } == {
        (
            "carla",
            "nuplan",
            "v1.1-mini",
            "CC-BY-NC-SA-4.0",
            ("lane_follow", "straight"),
            "Town07",
            "Town07",
            "route_01",
            _sha(source_path),
        )
    }


def test_future_source_manifest_details_do_not_define_loader_metadata_abi(
    tmp_path: Path,
) -> None:
    root = tmp_path / "dataset"
    _write_dataset(root)
    _set_source_metadata(
        root,
        {
            "source_dataset_id": "source-manifest-value",
            "source_dataset_version": "source-manifest-version",
            "license_id": "source-manifest-license",
            "future_schema": {"may_change_without_affecting_the_loader": True},
        },
    )

    example = load_training_examples(root, split="train", mode="schema").examples[0]

    assert example.source_dataset_id == "source-manifest-value"
    assert example.source_dataset_version == "source-manifest-version"
    assert example.license_id == "source-manifest-license"
    assert example.domain == "carla"
    assert example.scenario_tags == ("lane_follow", "straight")
    assert example.map_id == example.site_id == "Town07"
    assert example.route_id == "route_01"


def test_provenance_metadata_is_bound_into_example_fingerprint(tmp_path: Path) -> None:
    root = tmp_path / "dataset"
    _write_dataset(root)
    _set_source_metadata(
        root,
        {
            "source_dataset_id": "carla",
            "source_dataset_version": "0.9.15",
            "license_id": "CARLA-DATASET-TERMS",
        },
    )
    example = load_training_examples(root, split="train", mode="schema").examples[0]
    original = fingerprint_examples((example,))

    changes = (
        ("domain", "real"),
        ("source_dataset_id", "different-source"),
        ("source_dataset_version", "different-version"),
        ("license_id", "different-license"),
        ("scenario_tags", ("turn",)),
        ("map_id", "different-map"),
        ("site_id", "different-site"),
        ("route_id", "different-route"),
        ("source_manifest_sha256", "f" * 64),
    )

    for field, value in changes:
        changed = fingerprint_examples((replace(example, **{field: value}),))
        assert changed != original, field
