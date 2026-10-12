"""HH_261008 - Prepared scalar-only checker fixtures; no real file/hash/array replay.

These fictional metadata fixtures use only standard-library values. ROOT must
read the checker and this source before executing pytest. No actual evidence,
NPZ, provider, actor, model, journal writer or checker inspect/main is invoked.
"""
from copy import deepcopy

import pytest

from scripts.e2e import check_map_crop_batch_bytes_saved_20261008_v1 as check


def fiction():
    slots = [0, 1, 728, 729, 730, 1208, 42, 740]
    original = dict(step=1, physical_slot_indices=slots, shared_crop_boxes=[[0, 0, 360, 640] for _ in slots],
        actual_paired_TRUE_bits=[0]*8, RGB_sha256="a"*64, target_sha256="b"*64)
    expected = dict(step=1, RAW_line_sha256="f"*64, baseline=original)
    journal = dict(schema="portable_e2e.map_crop_preservation_journal.20261008.v1",
        policy_source_sha256="566b3e2afb7a0c879071c806c0cbfef6d31b8e974df99423339b7bf48cd6209f",
        baseline_exposure=deepcopy(original), selected_crop_boxes=deepcopy(original["shared_crop_boxes"]),
        full_GT_sign_pixels=[0]*8, original_NEAREST_sign_pixels=[0]*8,
        protected=[True, False]*4, rescued=[False]*8, candidate_RGB_sha256="a"*64, candidate_target_sha256="b"*64)
    receipt = dict(schema="portable_e2e.map_crop_batch_worker.20261008.v3.batch", step=1, journal=journal,
        native_slots=[dict(position=i, slot=slot, native_RGB_sha256="c"*64, native_target_sha256="d"*64,
            reader_provenance_authenticated_by_worker=False) for i, slot in enumerate(slots)],
        actual_baseline_RGB_and_GT_hashes_reproduced=True, candidate_hashes_computed_from_actual_owned_arrays=True,
        same_box_for_actual_RGB_and_GT=True, source_and_baseline_RAW_authenticated_by_worker=False, training_steps=0, NN=0)
    package = check.DATA / "paired_train480.npz"
    members = {}
    for name, pixels in (("rgb_false.npy", 360*640*3), ("target.npy", 360*640)):
        members[name] = dict(selected_payload_bytes=pixels, compression=8,
            prefix_body_bytes_inflated=11*pixels, member_EOF_and_trailer_checked=False, whole_member_crc_verified=False)
    witness = dict(step=1, last_slot_read=dict(slot=740, package_path=str(package), package_sha256=check.INPUTS[str(package)],
        rgb_true_decoded=False, caller_admission_authenticated_by_reader=False, closing_whole_file_SHA_checked=False,
        cache_payload_bytes=0, scratch_limit_bytes=1024**2, members=members))
    return expected, receipt, witness


def test_fictional_positive_join_preserves_supplied_containers():
    expected, receipt, witness = fiction()
    before = deepcopy((expected, receipt, witness))
    assert check.check_batch(receipt, expected)["native_receipts"] == 8
    check.check_last_slot(expected, witness)
    assert (expected, receipt, witness) == before


@pytest.mark.parametrize("fault", ["bool_step", "float_step", "bool_NN", "TRUE", "native_SHA", "wrong_slot", "extra_receipt"])
def test_scalar_worker_receipt_forgery_denied(fault):
    expected, receipt, _ = fiction()
    if fault == "bool_step": receipt["step"] = True
    elif fault == "float_step": receipt["step"] = 1.0
    elif fault == "bool_NN": receipt["NN"] = False
    elif fault == "TRUE": receipt["journal"]["baseline_exposure"]["actual_paired_TRUE_bits"][0] = 1
    elif fault == "native_SHA": receipt["native_slots"][0]["native_RGB_sha256"] = "A"*64
    elif fault == "wrong_slot": receipt["native_slots"][0]["slot"] = 1
    else: receipt["extra"] = "unrequested"
    with pytest.raises(ValueError):
        check.check_batch(receipt, expected)


@pytest.mark.parametrize("fault", ["path", "SHA", "TRUE_member", "TRUE_decoded", "bool_cache", "scratch", "prefix", "CRC"])
def test_last_slot_package_and_partial_read_scope_denied(fault):
    expected, _, witness = fiction()
    read = witness["last_slot_read"]
    if fault == "path": read["package_path"] = str(check.DATA / "original_train729.npz")
    elif fault == "SHA": read["package_sha256"] = "f"*64
    elif fault == "TRUE_member": read["members"]["rgb_true.npy"] = read["members"].pop("rgb_false.npy")
    elif fault == "TRUE_decoded": read["rgb_true_decoded"] = True
    elif fault == "bool_cache": read["cache_payload_bytes"] = False
    elif fault == "scratch": read["scratch_limit_bytes"] = 2*1024**2
    elif fault == "prefix": read["members"]["target.npy"]["prefix_body_bytes_inflated"] = 0
    else: read["members"]["target.npy"]["whole_member_crc_verified"] = True
    with pytest.raises(ValueError):
        check.check_last_slot(expected, witness)


@pytest.mark.parametrize("raw", [b'{"step":1,"step":2}', b'{"n":NaN}', b'{"n":Infinity}', b'{"n":1e309}'])
def test_duplicate_or_nonfinite_JSON_denied(raw):
    with pytest.raises(ValueError):
        check.decode(raw)
