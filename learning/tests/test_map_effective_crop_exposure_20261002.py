"""HH_261002 - Synthetic crop/exposure/provenance tests; never read actual TRAIN or model payloads."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/e2e"))
import audit_map_effective_crop_exposure_20261002 as audit


def synthetic_schedule():
    ids = np.resize(np.arange(1209, dtype=np.int64), 32000).reshape(4000, 8)
    boxes = np.tile(np.asarray([0, 0, 360, 640], dtype=np.int16), (4000, 8, 1))
    bits = np.zeros((4000, 8), dtype=np.uint8)
    return dict(control_indices=ids, candidate_indices=ids.copy(), shared_crops=boxes, shared_FALSE_view_bits=bits)


def rows_and_info(schedule):
    rows = [dict(step=i + 1, physical_slot_indices=schedule["control_indices"][i].tolist(),
        shared_crop_boxes=schedule["shared_crops"][i].tolist(),
        actual_paired_TRUE_bits=schedule["shared_FALSE_view_bits"][i].tolist()) for i in range(4000)]
    info = {key:audit.mining.array_sha(schedule[name]) for key, name in (
        ("sample_sequence_sha256", "control_indices"), ("crop_sequence_sha256", "shared_crops"),
        ("view_sequence_sha256", "shared_FALSE_view_bits"))}
    info["physical_slot_exposures"] = np.bincount(schedule["control_indices"].ravel(), minlength=1209).tolist()
    return rows, info


@pytest.fixture(scope="module")
def chronology():
    schedule = synthetic_schedule(); rows, info = rows_and_info(schedule)
    return schedule, rows, info


def test_contract_no_model_fit_or_independent_site_and_exact_full_denominators():
    value = audit.prepared_contract()
    assert value["exposures_per_arm"] == 32000 and value["full_exposure_denominator"] == 128000
    assert value["physical_TRAIN_slots"] == 1209 and len(value["source_groups"]) == 4
    assert value["required_current_unchanged_fit_backend_versions"] == dict(pillow="12.3.0", numpy="2.5.2")
    assert value["runtime_binary_and_python_fingerprints_externally_pinned"]
    for key in ("known_DEV_or_HOLDOUT_payload_access", "training_authorized", "model_promoted",
                "vehicle_control_approved", "target_relabeling", "crop_reroll", "efficacy_or_sampler_causality_proven"):
        assert value[key] is False
    assert audit.contract.apply_crop is audit.contract.frozen.apply_crop


def test_actual_chronology_full4000_and32000_not_unique_sample_claim(chronology):
    schedule, rows, info = chronology
    audit.verify_rows(rows, "control", schedule, info)
    assert sum(info["physical_slot_exposures"]) == 32000 and min(info["physical_slot_exposures"]) >= 26


@pytest.mark.parametrize("kind", ["tail_missing", "repeat_step", "bool_step", "float_index", "bool_bit", "TRUE",
    "heldout_index", "crop_reroll", "unknown_field", "digest", "census"])
def test_any_actual_chronology_or_scope_mutation_rejected(chronology, kind):
    schedule, original, info = chronology
    rows, metadata = copy.deepcopy(original), copy.deepcopy(info)
    if kind == "tail_missing": rows.pop()
    elif kind == "repeat_step": rows[-1]["step"] = 3999
    elif kind == "bool_step": rows[0]["step"] = True
    elif kind == "float_index": rows[0]["physical_slot_indices"][0] = 0.0
    elif kind == "bool_bit": rows[0]["actual_paired_TRUE_bits"][0] = False
    elif kind == "TRUE": rows[0]["actual_paired_TRUE_bits"][0] = 1
    elif kind == "heldout_index": rows[0]["physical_slot_indices"][0] = 1209
    elif kind == "crop_reroll": rows[0]["shared_crop_boxes"][0][0] = 1
    elif kind == "unknown_field": rows[0]["pseudo_label"] = 1
    elif kind == "digest": metadata["crop_sequence_sha256"] = "0" * 64
    else: metadata["physical_slot_exposures"][0] -= 1
    with pytest.raises(ValueError): audit.verify_rows(rows, "control", schedule, metadata)


def pixels():
    rgb = np.zeros((360, 640, 3), dtype=np.uint8)
    target = np.zeros((360, 640), dtype=np.uint8)
    target[0:2, 0:2] = 5
    return rgb, target


def test_full_FOV_byteexact_histogram_and_source_unchanged():
    rgb, target = pixels(); before = rgb.tobytes(), target.tobytes()
    raw, histogram = audit.count_crop(rgb, target, [0, 0, 360, 640])
    assert raw == histogram[5] == 4 and sum(histogram) == 230400
    assert (rgb.tobytes(), target.tobytes()) == before


def test_positive_frame_crop_can_produce_zero_actual_sign_target():
    rgb, target = pixels()
    raw, histogram = audit.count_crop(rgb, target, [50, 50, 300, 500])
    row = audit.empty_counts(); audit.add_counts(row, 4, raw, histogram)
    assert row["full_frame_sign_positive_exposures"] == 1
    assert row["crop_zero_sign_exposures_from_positive_frames"] == 1
    assert row["raw_crop_cut_all_sign_exposures"] == 1
    assert row["actual_resized_sign_pixels_sum"] == 0


def test_partial_crop_separates_original_source_and_resized_pixel_denominators():
    rgb, target = pixels(); box = [1, 0, 200, 400]
    raw, histogram = audit.count_crop(rgb, target, box)
    _, expected = audit.contract.frozen.apply_crop(rgb, target, box)
    assert raw == 2 and histogram == tuple(np.bincount(expected.ravel(), minlength=7))
    row = audit.empty_counts(); audit.add_counts(row, 4, raw, histogram)
    assert row["crop_cut_some_sign_exposures"] == 1
    assert row["full_frame_sign_pixels_sum"] == 4 and row["source_crop_sign_pixels_sum"] == 2
    assert row["actual_resized_sign_pixels_sum"] == histogram[5]


@pytest.mark.parametrize("box", [[-1, 0, 360, 640], [0, 0, 0, 640], [0, 1, 360, 640],
                                [False, 0, 360, 640], [0.0, 0, 360, 640]])
def test_no_rect_expansion_boolean_alias_or_crop_reroll(box):
    with pytest.raises(ValueError): audit.count_crop(*pixels(), box)


def test_full_arm_cached_scalar_histograms_and_groups_keep_all128000_semantics(monkeypatch, chronology):
    _, rows, _ = chronology
    class Frames:
        def __getitem__(self, index): return index
    class ForbiddenTRUE:
        def __getitem__(self, index): pytest.fail("TRUE view accessed for target reconstruction")
    calls = []
    def count(rgb, target, box):
        calls.append((target, tuple(box)))
        return 1, (230399, 0, 0, 0, 0, 1, 0)
    monkeypatch.setattr(audit, "count_crop", count)
    inventory = dict(records=[dict(sign_target_pixels=1) for _ in range(1209)],
        strata=dict(small_sign=list(range(391)), large_sign=list(range(391, 584)), hard_negative=list(range(584, 729))))
    arrays = (Frames(), Frames(), Frames(), ForbiddenTRUE(), Frames())
    cache = {}; deadline = time.monotonic() + 20
    first = audit.count_arm(rows, arrays, inventory, cache, deadline)
    second = audit.count_arm(rows, arrays, inventory, cache, deadline)
    assert first == second and len(calls) == len(cache) == 1209
    assert first["overall"]["exposures"] == 32000
    assert sum(first["overall"]["actual_resized_class_pixels"]) == 32000 * 230400
    assert sum(row["exposures"] for row in first["source_groups"].values()) == 32000
    assert first["source_groups"]["Town01_p02_FALSE240"]["exposures"] > 0
    for name, row in first["overlapping_original_strata"].items():
        assert sum(group[name]["exposures"] for group in first["source_group_x_overlapping_original_strata"].values()) == row["exposures"]
    assert all(type(value[0]) is int and type(value[1]) is tuple for value in cache.values())


def test_count_deadline_and_incomplete_denominator_fail(chronology, monkeypatch):
    _, rows, _ = chronology
    inventory = dict(records=[dict(sign_target_pixels=0) for _ in range(1209)], strata={})
    with pytest.raises(ValueError, match="time exhausted"):
        audit.count_arm(rows, (None,) * 5, inventory, {}, time.monotonic() - 1)
    with pytest.raises(ValueError, match="denominator"):
        audit.count_arm([], (None,) * 5, inventory, {}, time.monotonic() + 1)


def test_first_read_exact_parsed_bytes_and_mutation_rejected(tmp_path):
    path = tmp_path / "input.json"; path.write_bytes(b'{"count":5.0}')
    pins = audit.Pins(); sha = hashlib.sha256(path.read_bytes()).hexdigest()
    assert pins.document(path, sha) == {"count": 5.0}
    assert pins.values[str(path)]["sha256"] == sha
    path.write_bytes(b'{"count":6.0}')
    with pytest.raises(ValueError): pins.recheck()
    with pytest.raises(ValueError): pins.file(path)


@pytest.mark.parametrize("raw", ['{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}'])
def test_duplicate_nonfinite_receipts_rejected(raw):
    with pytest.raises(ValueError): audit.strict_json(raw)


def test_fresh_output_and_all_symlink_ancestors_rejected(tmp_path):
    dirs = {key:tmp_path / key for key in ("data", "mining", "v1", "v2", "source")}
    for path in dirs.values(): path.mkdir()
    paths = {**dirs, "output":tmp_path / "fresh"}
    assert audit.checked_paths({key:str(path) for key, path in paths.items()}) == paths
    for output in (dirs["v1"], dirs["v1"] / "derived"):
        with pytest.raises(ValueError): audit.checked_paths({**{key:str(path) for key, path in paths.items()}, "output":str(output)})
    link = tmp_path / "link"; link.symlink_to(dirs["data"], target_is_directory=True)
    with pytest.raises(ValueError): audit.contract.canonical(link / "nonexistent")


@pytest.mark.parametrize("field", ["source_sha256", "runtime_identity", "paths", "contract", "research_diagnostic_authorized",
    "native_denials_preserved", "known_DEV_or_HOLDOUT_payload_access", "fit_authorized", "automatic_model_promotion", "vehicle_control_approved"])
def test_external_exact_authorization_cannot_expand_scope(field):
    source = {"synthetic": "a" * 64}; identity = dict(pillow="synthetic"); paths = dict(output=Path("/synthetic"))
    value = dict(schema=audit.AUTHORIZATION_SCHEMA, source_sha256=source, runtime_identity=identity,
        paths={key:str(path) for key, path in paths.items()}, contract=audit.prepared_contract(),
        research_diagnostic_authorized=True, native_denials_preserved=True, known_DEV_or_HOLDOUT_payload_access=False,
        fit_authorized=False, automatic_model_promotion=False, vehicle_control_approved=False, reviewer="synthetic-source-only")
    audit.validate_authorization(value, source, identity, paths)
    value[field] = not value[field] if type(value[field]) is bool else None
    with pytest.raises(ValueError): audit.validate_authorization(value, source, identity, paths)


def test_current_backend_local_substitution_rejected_before_data(monkeypatch):
    monkeypatch.setattr(audit.os, "uname", lambda:type("Host", (), dict(nodename="local"))())
    with pytest.raises(ValueError, match="private fit venv"):
        audit.environment(dict(pillow="9.0.1", numpy="1.24.4"))


def test_default_contract_no_actual_input_reads_no_torch(monkeypatch, capsys):
    monkeypatch.setattr(audit, "run", lambda args:pytest.fail("actual TRAIN read without authorization"))
    assert audit.main([]) == 0
    assert json.loads(capsys.readouterr().out)["full_exposure_denominator"] == 128000


def test_fresh_ten_file_import_contract_without_torch(tmp_path):
    for name in audit.SOURCE_NAMES:
        path = ROOT / ("learning/portable_e2e" if name in audit.contract.ARCHITECTURE_SHA else "tools/e2e") / name
        shutil.copyfile(path, tmp_path / name)
    code = "import sys; import audit_map_effective_crop_exposure_20261002 as a; assert 'torch' not in sys.modules; assert len(a.SOURCE_NAMES)==10; a.main([])"
    result = subprocess.run([sys.executable, "-B", "-c", code], cwd=tmp_path,
        env={"PATH":"/usr/bin:/bin", "PYTHONDONTWRITEBYTECODE":"1"}, capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["training_authorized"] is False
