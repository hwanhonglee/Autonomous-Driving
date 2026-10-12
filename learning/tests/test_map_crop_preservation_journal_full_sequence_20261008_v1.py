"""HH_261008 - Exercise the genuine batch validator for all20k synthetic metadata rows without a stub, pixels, file writer or training."""

from portable_e2e import map_crop_preservation_journal_20261008_v1 as journal


def synthetic_complete_sequence():
    """HH_261008 - Fixed fake SHA/count values test scalar rules only, not historical RGB/GT bytes or real sample chronology."""
    baselines, candidates = [], []
    for step in range(1, 20001):
        baseline = dict(step=step, physical_slot_indices=[(step + position) % 1209 for position in range(8)],
            shared_crop_boxes=[[0, 0, 240, 400] for _ in range(8)], actual_paired_TRUE_bits=[0] * 8,
            RGB_sha256="a" * 64, target_sha256="b" * 64)
        rescue = step % 701 == 0
        full = [3 if rescue and position == 0 else 0 for position in range(8)]
        selected = [[0, 0, 360, 640] if rescue and position == 0 else list(box)
            for position, box in enumerate(baseline["shared_crop_boxes"])]
        candidate = journal.batch_receipt(baseline, selected, full, [0] * 8,
            "c" * 64 if rescue else baseline["RGB_sha256"],
            "d" * 64 if rescue else baseline["target_sha256"])
        baselines.append(baseline)
        candidates.append(candidate)
    return baselines, candidates


def test_all20k_genuine_policy_and_batch_validation_then_last_failed_tail_no_monkeypatch():
    baselines, candidates = synthetic_complete_sequence()
    assert sum(sum(row["rescued"]) for row in candidates) == 20000 // 701
    assert all(row["schema"] == journal.SCHEMA for row in candidates)
    result = journal.audit_sequence(baselines, candidates)
    assert (result["PASS_batches"], result["FAIL_batches"], result["UNEXECUTED_batches"]) == (20000, 0, 0)
    assert result["status"] == "COMPLETE_PROVIDED_SCALAR_CONTRACT_ONLY"
    assert result["PASS_exposures"] == 160000 and result["candidate_examined_rows"] == 20000
    assert result["actual_array_bytes_authenticated"] is result["training_or_model_approved"] is False
    # HH_261008 - A last supplied failure remains inside the original denominator; never discard a failed tail after a valid prefix.
    candidates[-1]["protected"][0] = False
    failed = journal.audit_sequence(baselines, candidates)
    assert (failed["PASS_batches"], failed["FAIL_batches"], failed["UNEXECUTED_batches"]) == (19999, 1, 0)
    assert failed["failure"]["ordinal"] == 19999 and failed["candidate_examined_rows"] == 20000
    assert failed["PASS_exposures"] + failed["FAIL_exposures"] + failed["UNEXECUTED_exposures"] == 160000
    assert baselines[-1]["step"] == 20000 and baselines[-1]["shared_crop_boxes"][0] == [0, 0, 240, 400]
