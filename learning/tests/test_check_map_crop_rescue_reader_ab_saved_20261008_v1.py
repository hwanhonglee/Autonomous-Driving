"""HH_261008 - Scalar-only synthetic saved-check contracts; no real evidence access.

Fictional dictionaries/byte containers only. These fixtures do not prepare or
execute a producer/checker, hash NPZs, import arrays, raster-decode PNGs, assert
real rescue, authenticate ownership, train or approve model functionality.
"""
import ast
from copy import deepcopy
from pathlib import Path
import struct
import zlib

import pytest

from scripts.e2e import check_map_crop_rescue_reader_ab_saved_20261008_v1 as check


def expected(step=1):
    row = dict(step=step, physical_slot_indices=[1, 200, 333, 729, 830, 930, 1030, 1208],
        shared_crop_boxes=[[0, 0, 300, 400] for _ in range(8)], actual_paired_TRUE_bits=[0]*8,
        RGB_sha256="a"*64, target_sha256="b"*64)
    return dict(step=step, RAW_line_sha256="c"*64, baseline=row)


def receipt(rescue=False):
    original = expected()
    journal = dict(schema="portable_e2e.map_crop_preservation_journal.20261008.v1", policy_source_sha256=check.POLICY_PIN,
        baseline_exposure=original["baseline"], selected_crop_boxes=deepcopy(original["baseline"]["shared_crop_boxes"]),
        full_GT_sign_pixels=[3]+[0]*7, original_NEAREST_sign_pixels=[3]+[0]*7,
        protected=[True, False]*4, rescued=[False]*8, candidate_RGB_sha256="a"*64, candidate_target_sha256="b"*64)
    if rescue:
        journal["original_NEAREST_sign_pixels"][0] = 0
        journal["selected_crop_boxes"][0] = [0, 0, 360, 640]
        journal["rescued"][0] = True
        journal["candidate_RGB_sha256"], journal["candidate_target_sha256"] = "d"*64, "e"*64
    return dict(schema="portable_e2e.map_crop_batch_worker.20261008.v3.batch", step=1, journal=journal,
        native_slots=[dict(position=p, slot=slot, native_RGB_sha256="f"*64, native_target_sha256="0"*64,
            reader_provenance_authenticated_by_worker=False) for p, slot in enumerate(original["baseline"]["physical_slot_indices"])],
        actual_baseline_RGB_and_GT_hashes_reproduced=True, candidate_hashes_computed_from_actual_owned_arrays=True,
        same_box_for_actual_RGB_and_GT=True, source_and_baseline_RAW_authenticated_by_worker=False, training_steps=0, NN=0)


def actual(row):
    return dict(step=1, original_RGB_GT_equal=True, candidate_RGB_sha256=row["candidate_RGB_sha256"],
        candidate_target_sha256=row["candidate_target_sha256"], selected_crop_boxes=deepcopy(row["selected_crop_boxes"]),
        selected_GT_sign_pixels=[3]+[0]*7, rescued_slots=sum(row["rescued"]), batch_elapsed_seconds=0.01)


@pytest.mark.parametrize("raw", [b'{"a":1,"a":2}', b'{"a":NaN}', b'{"a":Infinity}', b'{"a":1e309}'])
def test_duplicate_nonfinite_metadata_is_denied(raw):
    with pytest.raises(ValueError):
        check.decode(raw)


def test_raw_JSON_int64_nanoseconds_are_not_rounded_or_coerced():
    value = 1791453600123456789
    assert check.decode(check.encoded({"ns": value}))["ns"] == value
    assert not check.same(value, float(value))
    assert not check.same(0, False)
    assert not check.same([3], [3.0])


@pytest.mark.parametrize("value", [True, 1.0, "1", -1, 2**63])
def test_integer_field_validation_does_not_coerce(value):
    assert not check.integer(value)


@pytest.mark.parametrize("value", ["a", "A"*64, "g"*64, True, 1, None])
def test_SHA_field_requires_exact_lowercase_string(value):
    assert not check.sha(value)


def test_FIXED_16_same_V3_cache_bool_only_caps_and_authority_are_exact():
    contract = check.contract()
    assert check.ARMS == ("CONTROL_V3_CACHE0", "CANDIDATE_V3_CACHE12M")
    assert contract["declared_steps"] == list(range(1, 17))
    assert contract["expected_batches_per_arm"] == 16 and contract["expected_slot_exposures_per_arm"] == 128
    assert contract["original20k_batches_outside_declaration_per_arm"] == 19984
    assert contract["candidate_checkpoint_max_states"] == 48
    assert contract["candidate_cache_account_limit_bytes"] == 12*1024**2
    assert contract["sampled_self_VmHWM_cap_bytes"] == 512*1024**2
    assert contract["deadline_seconds"] == 900
    assert contract["state_accounting_is_reservation_not_exact_native_allocation"] is True
    grant = check.authority("a"*64)
    assert grant["training_authorized"] is grant["NN_authorized"] is False
    assert grant["old_target_only_authority_extended"] is False
    assert len(check.SOURCE_NAMES) == 18 and len(check.INPUTS) == 8


@pytest.mark.parametrize("rescue", [False, True])
def test_fictional_B8_metadata_native_SHA_box_and_positive_rescue_join(rescue):
    saved = receipt(rescue)
    row = check.check_batch(saved, expected())
    rescues = check.check_actual_batch(actual(row), row, 1)
    assert len(rescues) == int(rescue)
    if rescue:
        assert rescues[0]["selected_NEAREST_GT_sign_pixels"] == 3
        assert rescues[0]["proposed_NEAREST_GT_sign_pixels"] == 0
        assert rescues[0]["selected_box"] == [0, 0, 360, 640]
        assert rescues[0]["actual_same_box_for_RGB_GT"] is True


@pytest.mark.parametrize("mutation", ["bool_step", "native_slot", "bad_SHA", "FALSE_bit", "same_box", "selected_box"])
def test_falsebool_stringSHA_path_or_slot_replacements_do_not_pass_B8(mutation):
    saved = receipt(True)
    if mutation == "bool_step":
        saved["step"] = True
    elif mutation == "native_slot":
        saved["native_slots"][0]["slot"] = "1"
    elif mutation == "bad_SHA":
        saved["native_slots"][0]["native_RGB_sha256"] = 7
    elif mutation == "FALSE_bit":
        saved["journal"]["baseline_exposure"]["actual_paired_TRUE_bits"][0] = False
    elif mutation == "same_box":
        saved["same_box_for_actual_RGB_and_GT"] = 1
    else:
        saved["journal"]["selected_crop_boxes"][0] = [0, 0, 300, 400]
    with pytest.raises(ValueError):
        check.check_batch(saved, expected())


def test_fullFOV_rescue_selected_count_must_equal_positive_full_support():
    row = check.check_batch(receipt(True), expected())
    forged = actual(row)
    forged["selected_GT_sign_pixels"][0] = 1
    with pytest.raises(ValueError):
        check.check_actual_batch(forged, row, 1)
    forged["selected_GT_sign_pixels"][0] = 0
    with pytest.raises(ValueError):
        check.check_actual_batch(forged, row, 1)


def stats(enabled):
    return dict(current_checkpoint_states=0, maximum_checkpoint_states_observed=0,
        max_checkpoint_states=48 if enabled else 0, checkpoint_state_account_bytes=256*1024,
        cache_accounted_bytes=0, maximum_cache_accounted_bytes=0,
        cache_account_limit_bytes=12*1024**2 if enabled else 0, checkpoint_cache_enabled=enabled,
        owner_checkpoint_calls=3, archive_generation_guard_calls=16, owner_checkpoint_guard_stride=16,
        checkpoint_hits=0, checkpoint_misses=0, checkpoint_creations=0, checkpoint_evictions=0,
        total_prefix_body_bytes_inflated=0, total_selected_payload_bytes=921600,
        native_row_cache_payload_bytes=0, state_accounting_is_reservation_not_exact_native_allocation=True,
        checkpoint_stride_bytes=32*1024**2)


@pytest.mark.parametrize("enabled", [False, True])
def test_exact_scalar_cache_stats_valid_under_fixed_bounds(enabled):
    check.check_stats(stats(enabled), enabled)
    check.check_stats(stats(enabled), enabled, after_close=True)


@pytest.mark.parametrize("fault", ["bool", "over_states", "over_bytes", "cache0_hit", "allocation_claim"])
def test_cache_caps_and_native_allocation_limits_cannot_be_replaced_by_metadata(fault):
    enabled = fault != "cache0_hit"
    value = stats(enabled)
    if fault == "bool":
        value["current_checkpoint_states"] = False
    elif fault == "over_states":
        value["maximum_checkpoint_states_observed"] = 49
    elif fault == "over_bytes":
        value["maximum_cache_accounted_bytes"] = 12*1024**2+1
    elif fault == "cache0_hit":
        value["checkpoint_hits"] = 1
    else:
        value["state_accounting_is_reservation_not_exact_native_allocation"] = False
    with pytest.raises(ValueError):
        check.check_stats(value, enabled)


def last_read():
    members = {}
    for name, pixels in (("rgb_false.npy", 691200), ("target.npy", 230400)):
        members[name] = dict(selected_payload_bytes=pixels, uncompressed_position=128+480*pixels,
            prefix_body_bytes_inflated=479*pixels, logical_prefix_body_bytes=479*pixels,
            prefix_body_bytes_reused=0, checkpoint_hit=False, checkpoint_restored_uncompressed_offset=0,
            header_bytes_decoded_before_native_slice=128, whole_member_crc_verified=True,
            member_EOF_and_trailer_checked=True, compression=8)
        if name == "target.npy":
            members[name].update(all_selected_GT_labels_checked_0_to_6=True, skipped_prefix_GT_labels_checked=False)
    package = check.DATA/"paired_train480.npz"
    return dict(slot=1208, package_path=str(package), package_sha256=check.INPUTS[str(package)], members=members,
        rgb_true_decoded=False, caller_admission_authenticated_by_reader=False, closing_whole_file_SHA_checked=False,
        cache_payload_bytes=0, scratch_limit_bytes=1024**2, checkpoint_cache=stats(False))


def test_last_slot_metadata_fixed_FALSE_route_only_not_all128_stream_history():
    check.check_last_slot(last_read(), expected(step=16), False)


@pytest.mark.parametrize("fault", ["path", "pin", "TRUE_member", "admission", "CRC", "cache0_hit", "position", "header"])
def test_last_slot_path_SHA_permission_or_TRUE_replacements_are_denied(fault):
    read = last_read()
    if fault == "path":
        read["package_path"] = "/fictional/other.npz"
    elif fault == "pin":
        read["package_sha256"] = "a"*64
    elif fault == "TRUE_member":
        read["members"]["rgb_true.npy"] = {}
    elif fault == "admission":
        read["caller_admission_authenticated_by_reader"] = True
    elif fault == "CRC":
        read["members"]["target.npy"]["whole_member_crc_verified"] = False
    elif fault == "position":
        read["members"]["target.npy"]["uncompressed_position"] += 1
    elif fault == "header":
        read["members"]["target.npy"]["header_bytes_decoded_before_native_slice"] = 9
    else:
        read["members"]["target.npy"]["checkpoint_hit"] = True
    with pytest.raises(ValueError):
        check.check_last_slot(read, expected(step=16), False)


def chunk(kind, raw):
    return struct.pack(">I", len(raw))+kind+raw+struct.pack(">I", zlib.crc32(raw, zlib.crc32(kind)) & 0xffffffff)


def fake_png():
    # HH_261008 - Intentionally non-raster IDAT proves that this helper authenticates container, not pixels.
    return b"\x89PNG\r\n\x1a\n"+chunk(b"IHDR", struct.pack(">IIBBBBB", 1920, 768, 8, 2, 0, 0, 0))+chunk(b"IDAT", b"not-raster-data")+chunk(b"IEND", b"")


def test_PNG_container_inspection_never_implies_raster_replay():
    witness = check.png_container(fake_png())
    assert witness["width"] == 1920 and witness["height"] == 768
    assert witness["chunk_CRC_checked"] is True
    assert witness["IDAT_inflated"] is witness["raster_pixels_decoded"] is False


@pytest.mark.parametrize("fault", ["CRC", "trailer", "IHDR", "signature"])
def test_bad_PNG_container_crc_or_geometry_is_denied_without_decode(fault):
    raw = fake_png()
    if fault == "CRC":
        raw = raw[:-1]+bytes((raw[-1] ^ 1,))
    elif fault == "trailer":
        raw += b"tail"
    elif fault == "IHDR":
        raw = b"\x89PNG\r\n\x1a\n"+chunk(b"IHDR", struct.pack(">IIBBBBB", 1, 1, 8, 2, 0, 0, 0))+chunk(b"IDAT", b"x")+chunk(b"IEND", b"")
    else:
        raw = b"bad"+raw[3:]
    with pytest.raises(ValueError):
        check.png_container(raw)


def test_overlapping_seals_cannot_silently_replace_pin_or_path_type():
    assert check.merge_seals({"/fictional/source": "a"*64}, {"/fictional/source": "a"*64}) == {"/fictional/source": "a"*64}
    with pytest.raises(ValueError):
        check.merge_seals({"/fictional/source": "a"*64}, {"/fictional/source": "b"*64})
    with pytest.raises(ValueError):
        check.merge_seals({1: "a"*64})


def test_source_is_stdlib_only_create_only_and_never_imports_producers_or_array_loaders():
    tree = ast.parse(Path(check.__file__).read_bytes())
    imports = {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
    imports |= {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    assert not imports & {"numpy", "PIL", "torch", "carla", "rclpy", "zipfile", "subprocess", "importlib"}
    calls = {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)}
    assert not calls & {"np.load", "torch.load", "zlib.decompress", "zlib.decompressobj", "Image.open", "exec", "eval"}
    source = ast.unparse(tree)
    assert "'xb'" in source and "close_seals" in source
    assert "candidate_pixel_hashes_independently_recomputed=False" in source
    assert "pixel_SHA_second_recomputations=0" in source
