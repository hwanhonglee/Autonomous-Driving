"""HH_261008 - Actual bounded synthetic arrays; no TRAIN payloads, models or CARLA."""
from copy import deepcopy
import hashlib
from pathlib import Path

import numpy as np
import pytest

from portable_e2e import map_crop_batch_worker_20261008_v1 as worker

ROOT = Path(__file__).resolve().parents[2]
CROP = ROOT / "tools/e2e/map_appearance_matched_contract_20261001.py"


class Reader:
    def __init__(self, positive=True):
        yy, xx = np.indices((360, 640))
        self.rgb = np.stack((xx % 256, yy % 256, (xx + yy) % 256), axis=2).astype(np.uint8)
        self.target = np.zeros((360, 640), dtype=np.uint8)
        if positive:
            self.target[300:320, 550:570] = 5
        self.calls = []

    def read(self, slot):
        self.calls.append(slot)
        return self.rgb, self.target


def baseline(reader, crop, step=1, box=None):
    boxes = [list(box or [0, 0, 120, 300]) for _ in range(8)]
    rgb, gt = crop(reader.rgb, reader.target, boxes[0])
    return dict(step=step, physical_slot_indices=[0, 1, 728, 729, 730, 1208, 42, 42],
        shared_crop_boxes=boxes, actual_paired_TRUE_bits=[0] * 8,
        RGB_sha256=hashlib.sha256(memoryview(np.stack([rgb] * 8)).cast("B")).hexdigest(),
        target_sha256=hashlib.sha256(memoryview(np.stack([gt] * 8)).cast("B")).hexdigest())


def test_original_source_AST_compiler_and_byte_aligned_rescue():
    crop = worker.compile_original_crop(CROP)
    reader = Reader()
    row = baseline(reader, crop)
    before = deepcopy(row)
    source_rgb, source_gt = reader.rgb.copy(), reader.target.copy()
    rgb, gt, receipt = worker.build_batch(row, reader, crop)
    assert row == before and reader.calls == row["physical_slot_indices"]
    assert np.array_equal(reader.rgb, source_rgb) and np.array_equal(reader.target, source_gt)
    for pos in range(8):
        expected = crop(source_rgb, source_gt, [0, 0, 360, 640] if pos % 2 == 0 else row["shared_crop_boxes"][pos])
        assert np.array_equal(rgb[pos], expected[0]) and np.array_equal(gt[pos], expected[1])
    assert receipt["journal"]["rescued"] == [True, False] * 4
    assert receipt["journal"]["full_GT_sign_pixels"] == [400] * 8
    assert receipt["journal"]["original_NEAREST_sign_pixels"] == [0] * 8
    assert rgb.flags.owndata and gt.flags.owndata
    assert not np.shares_memory(rgb, reader.rgb) and not np.shares_memory(gt, reader.target)
    assert receipt["actual_baseline_RGB_and_GT_hashes_reproduced"] is True
    assert receipt["source_and_baseline_RAW_authenticated_by_worker"] is False


@pytest.mark.parametrize("positive,box", [(False, [0, 0, 120, 300]), (True, [0, 0, 360, 640]),
    (True, [280, 530, 80, 110])])
def test_no_rescue_actual_two_hashes_equal_and_negative_background_preserved(positive, box):
    crop = worker.compile_original_crop(CROP)
    reader = Reader(positive)
    row = baseline(reader, crop, step=20000, box=box)
    rgb, gt, receipt = worker.build_batch(row, reader, crop)
    assert not any(receipt["journal"]["rescued"])
    assert worker.array_hash(rgb) == row["RGB_sha256"] and worker.array_hash(gt) == row["target_sha256"]


@pytest.mark.parametrize("field", ["RGB_sha256", "target_sha256"])
def test_original_hash_mismatch_returns_no_candidate(field):
    crop = worker.compile_original_crop(CROP)
    reader = Reader()
    row = baseline(reader, crop)
    row[field] = "f" * 64
    with pytest.raises(ValueError, match="historical original"):
        worker.build_batch(row, reader, crop)


@pytest.mark.parametrize("fault", ["TRUE", "bool_slot", "wrong_step", "bad_box"])
def test_invalid_metadata_denied_before_reader_or_crop(fault):
    crop = worker.compile_original_crop(CROP)
    reader = Reader()
    row = baseline(reader, crop)
    if fault == "TRUE": row["actual_paired_TRUE_bits"][0] = 1
    elif fault == "bool_slot": row["physical_slot_indices"][0] = True
    elif fault == "wrong_step": row["step"] = 0
    else: row["shared_crop_boxes"][0] = [0, 0, 361, 640]
    with pytest.raises(ValueError):
        worker.build_batch(row, reader, crop)
    assert reader.calls == []


@pytest.mark.parametrize("fault", ["dtype", "shape", "labels"])
def test_reader_ABI_and_labels_rejected(fault):
    crop = worker.compile_original_crop(CROP)
    reader = Reader()
    row = baseline(reader, crop)
    if fault == "dtype": reader.rgb = reader.rgb.astype(np.float32)
    elif fault == "shape": reader.target = reader.target[:100]
    else: reader.target[0, 0] = 7
    with pytest.raises(ValueError):
        worker.build_batch(row, reader, crop)


def test_crop_mutation_and_reader_failure_and_checkpoint_keep_exception():
    crop = worker.compile_original_crop(CROP)
    reader = Reader()
    row = baseline(reader, crop)
    def bad_crop(rgb, target, box):
        original = crop(rgb, target, box)
        rgb[0, 0, 0] ^= 1
        return original
    with pytest.raises(ValueError, match="changed its native"):
        worker.build_batch(row, reader, bad_crop)
    def denied(): raise RuntimeError("owner deadline denied")
    with pytest.raises(RuntimeError, match="deadline"):
        worker.build_batch(row, reader, crop, checkpoint=denied)
    def failed(slot): raise OSError("reader denied")
    reader.read = failed
    with pytest.raises(OSError, match="reader denied"):
        worker.build_batch(row, reader, crop)


def test_real_create_only_writer_byte_checks_and_exact_partial_denominator(tmp_path):
    crop, reader = worker.compile_original_crop(CROP), Reader()
    row = baseline(reader, crop)
    rgb, gt, receipt = worker.build_batch(row, reader, crop)
    path = tmp_path / "candidate.jsonl"
    writer = worker.BatchJournalWriter(path, [1, 2])
    writer.append(rgb, gt, receipt)
    with pytest.raises(ValueError, match="order"):
        writer.append(rgb, gt, receipt)
    assert writer.summary(True) == dict(declared_batches=2, PASS_batches=1, FAIL_batches=1,
        UNEXECUTED_batches=0, original20k_batches_outside_this_declaration=19998,
        written_bytes=path.stat().st_size, candidate_journal_not_original_full20k=True)
    writer.close()
    with pytest.raises(ValueError, match="fresh"):
        worker.BatchJournalWriter(path, [1])
    other = worker.BatchJournalWriter(tmp_path / "changed.jsonl", [1])
    rgb[0, 0, 0, 0] ^= 1
    with pytest.raises(ValueError, match="arrays changed"):
        other.append(rgb, gt, receipt)
    assert other.count == 0
    other.close()


def test_writer_quota_and_close_path_swap_denied(tmp_path):
    crop, reader = worker.compile_original_crop(CROP), Reader()
    rgb, gt, receipt = worker.build_batch(baseline(reader, crop), reader, crop)
    writer = worker.BatchJournalWriter(tmp_path / "small.jsonl", [1], maximum_bytes=1)
    with pytest.raises(ValueError, match="allowance"):
        writer.append(rgb, gt, receipt)
    writer.close()
    path = tmp_path / "swap.jsonl"
    writer = worker.BatchJournalWriter(path, [1])
    path.rename(tmp_path / "preserved.jsonl")
    path.write_bytes(b"replacement")  # HH_261008 - Controlled temporary synthetic path swap, not a project asset.
    with pytest.raises(ValueError, match="replaced"):
        writer.close()


@pytest.mark.parametrize("steps", [[], [True], [0], [20001], [2, 1], [1, 1]])
def test_writer_declared_sequence_denials(tmp_path, steps):
    with pytest.raises(ValueError):
        worker.BatchJournalWriter(tmp_path / "never_created.jsonl", steps)
    assert not (tmp_path / "never_created.jsonl").exists()
