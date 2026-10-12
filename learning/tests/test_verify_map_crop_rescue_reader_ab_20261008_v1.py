"""HH_261008 - Prepare scalar/source contracts for FIRST16 same-reader cache A/B.

These fixtures use fictional metadata and temporary text sources. They do not
admit/read actual packages, decode image arrays, prepare a real experiment or
claim genuine rescue, kernel ownership, PNG evidence, training or model gain.
"""
import ast
from copy import deepcopy
from pathlib import Path

import pytest

from scripts.e2e import verify_map_crop_rescue_reader_ab_20261008_v1 as check


def baseline(step=1):
    return dict(step=step, physical_slot_indices=[1, 200, 333, 729, 830, 930, 1030, 1208],
        shared_crop_boxes=[[0, 0, 300, 400] for _ in range(8)], actual_paired_TRUE_bits=[0]*8,
        RGB_sha256="a"*64, target_sha256="b"*64)


def rows(count):
    return [dict(step=step, RAW_line_sha256="c"*64, baseline=baseline(step)) for step in range(1, count+1)]


def test_fixed_FIRST16_same_V3_source_and_only_cache_bool_changes():
    contract = check.fixed()
    assert check.STEPS == list(range(1, 17))
    assert check.ARMS == ("CONTROL_V3_CACHE0", "CANDIDATE_V3_CACHE12M")
    assert contract["expected_batches_per_arm"] == 16 and contract["expected_slot_exposures_per_arm"] == 128
    assert contract["original20k_batches_outside_declaration_per_arm"] == 19984
    assert contract["reader_source_same_in_both_arms"] is True
    assert contract["CONTROL_checkpoint_cache_enabled"] is False
    assert contract["CANDIDATE_checkpoint_cache_enabled"] is True
    assert contract["candidate_checkpoint_max_states"] == 48
    assert contract["candidate_cache_account_limit_bytes"] == 12*1024**2
    assert contract["state_accounting_is_reservation_not_exact_native_allocation"] is True
    assert contract["paired_TRUE_member_decode_authorized"] is False
    assert contract["baseline_both_RGB_and_GT_SHA_required"] is True
    assert contract["same_worker_crop_policy_original_rows_FALSE_and_proposals"] is True
    assert contract["RNG_draws_added"] == contract["training_steps"] == contract["NN"] == 0
    assert contract["metadata_positive_partial_crop_hints_do_not_prove_original_GT_zero"] is True


def test_unchanged_budgets_are_not_relaxed_for_visual_or_larger_scope():
    contract = check.fixed()
    assert contract["CPU_affinity"] == [3] and contract["threads"] == 1
    assert contract["sampled_self_VmHWM_cap_bytes"] == 512*1024**2
    assert contract["deadline_seconds"] == 900
    assert contract["floor_bytes"] == 2*1024**3
    assert contract["reserve_bytes"] == 64*1024**2
    assert contract["maximum_combined_PLAN_OUTPUT_bytes"] == 2*1024**2
    assert contract["closing_bytes"] == contract["optional_first_actual_rescue_PNG_maximum_bytes"] == 512*1024
    assert contract["journal_maximum_bytes_per_arm"] == 256*1024


def test_original14_plus_new4_are_all_explicitly_archived():
    assert len(check.OLD_SOURCES) == 14
    assert len(check.SOURCES) == len(set(check.SOURCES)) == 18
    assert set(check.OLD_SOURCES) <= set(check.SOURCES)
    assert "learning/portable_e2e/map_native_pair_reader_20261008_v2.py" in check.OLD_SOURCES
    assert "learning/portable_e2e/map_native_pair_reader_20261008_v3.py" in check.SOURCES
    assert "tools/e2e/verify_map_crop_rescue_reader_ab_20261008_v1.py" in check.SOURCES
    assert "learning/tests/test_verify_map_crop_rescue_reader_ab_20261008_v1.py" in check.SOURCES
    assert len(check.INPUTS) == 8
    assert check.INPUTS[str(check.SELECTION)] == "998fa695a1f2437d4f34d4bc1341c7c43256f038ef5cde072f4a7918f3b1f8e1"


def fictional_selection():
    selected = rows(16)
    candidates = []
    for step, position in ((2, 2), (5, 4), (6, 2), (7, 6), (9, 2), (10, 4), (11, 2)):
        row = selected[step-1]["baseline"]
        candidates.append(dict(step=step, position_zero_based=position, global_ordinal_zero_based=(step-1)*8+position,
            slot=row["physical_slot_indices"][position], fullGT_sign_pixels=3,
            proposed_box=deepcopy(row["shared_crop_boxes"][position]), actual_crop_GT_zero_unknown=True))
    selection = dict(schema="portable_e2e.map_crop_rescue_reader_ab_metadata_selection.20261008.v2",
        state="METADATA_ONLY_PREPARED_NOT_PAYLOAD_VERIFIED", metadata_scan_steps=list(range(1, 17)), metadata_exposures_examined=128,
        first16_original_RAW_lines_sha256="e"*64, policy_changed=False, proposals_rerolled=False,
        successful_crop_or_rescue_cases_selected_after_payload_decode=False, candidate_metadata_is_independent_TEST=False,
        actual_RGB_rescue_effect_proven=False, model_performance_gain_proven=False, existing_thresholds_or_resource_caps_lowered=False,
        prospective_FIRST16_fixed_before_new_payload_decode=True, new_work={"fictional_array_decodes": 0},
        resource_caps_preserved=dict(sampled_VmHWM_bytes=512*1024**2, deadline_seconds=900,
            floor_bytes=2*1024**3, reserve_bytes=64*1024**2), candidate_count=7, candidates=candidates,
        prospective_arms={name: dict(reader_revision="V3", cache_enabled=index == 1, declared_batches=16,
            declared_slot_exposures=128, PASS_batches=0, FAIL_batches=0, UNEXECUTED_batches=16,
            original20000_batches_OUT_OF_SCOPE=19984) for index, name in enumerate(check.ARMS)})
    return selection, selected


def test_upfront_metadata_candidates_join_original_positions_without_claiming_cropped_GT_zero():
    selection, selected = fictional_selection()
    actual = check.selection_scope(selection, selected, "e"*64)
    assert len(actual["candidates"]) == 7
    assert actual["cropped_GT_zero_verified"] is False
    assert actual["actual_RGB_rescue_effect_proven"] is False
    assert actual["candidate_metadata_is_independent_TEST"] is False


@pytest.mark.parametrize("field,bad", [("slot", 999), ("position_zero_based", 1),
    ("actual_crop_GT_zero_unknown", False), ("fullGT_sign_pixels", 0), ("global_ordinal_zero_based", 11)])
def test_candidate_hint_mismatch_or_unobserved_zero_claim_is_denied(field, bad):
    selection, selected = fictional_selection()
    selection["candidates"][0][field] = bad
    with pytest.raises(ValueError):
        check.selection_scope(selection, selected, "e"*64)


def test_ROOT_raw_grant_does_not_admit_training_or_old_target_only_authority():
    grant = check.authority({}, "d"*64)
    assert grant["plan_sha256"] == "d"*64
    assert grant["training_authorized"] is grant["NN_authorized"] is False
    assert grant["old_target_only_authority_extended"] is False
    assert grant["same_source_reader_cache_AB_authorized"] is True
    assert grant["DEV_HOLDOUT_TEST_weights_TRUE_deniedRAW_decode_authorized"] is False
    changed = deepcopy(grant)
    changed["contract"]["CONTROL_checkpoint_cache_enabled"] = 0
    assert not check.typed_equal(changed, grant)


@pytest.mark.parametrize("raw", [b'{"a":1,"a":2}', b'{"a":NaN}', b'{"a":Infinity}', b'{"a":1e309}'])
def test_duplicate_nonfinite_ROOT_metadata_denied(raw):
    with pytest.raises(ValueError):
        check.decode(raw)


@pytest.mark.parametrize("left,right", [(1, True), (1, 1.0), ([3], [3.0]), ({"x":False}, {"x":0})])
def test_exact_TYPED_prospective_ROOT_metadata_does_not_coerce(left, right):
    assert not check.typed_equal(left, right)


def test_quota_projection_16_32_64_preserves_per_journal_png_and_closing_bounds():
    forecasts = [check.prospective_quota(rows(count), 512*1024, 24*1024) for count in (16, 32, 64)]
    for forecast in forecasts:
        expected = (512*1024 + 4*24*1024 + 2*forecast["estimated_maximum_journal_bytes_per_arm"]
                    +512*1024+512*1024+64*1024)
        assert forecast["estimated_combined_PLAN_OUTPUT_bytes"] == expected
        assert forecast["maximum_combined_PLAN_OUTPUT_bytes"] == 2*1024**2
        assert forecast["unchanged_journal_cap_bytes_per_arm"] == 256*1024
        assert forecast["prospective_scope_change_authorized"] is False
        assert forecast["scalar_envelope_not_actual_RGB_GT_or_rescue_evidence"] is True
    assert forecasts[0]["fits_unchanged_limits"] is True
    assert forecasts[2]["fits_unchanged_limits"] is False
    assert forecasts[0]["estimated_combined_PLAN_OUTPUT_bytes"] < forecasts[1]["estimated_combined_PLAN_OUTPUT_bytes"]
    assert forecasts[1]["estimated_combined_PLAN_OUTPUT_bytes"] < forecasts[2]["estimated_combined_PLAN_OUTPUT_bytes"]


@pytest.mark.parametrize("count", [0, 8, 15, 17, 65])
def test_quota_input_range_is_prospective_and_cannot_silently_change_actual_scope(count):
    with pytest.raises(ValueError):
        check.prospective_quota(rows(count), 100, 100)


def test_prepare_quota_denial_occurs_before_creating_plan_or_output(tmp_path, monkeypatch):
    # HH_261008 - Every mocked file is an isolated fictional text source; INPUTS is empty in this fixture only.
    monkeypatch.setattr(check, "ROOT", tmp_path)
    monkeypatch.setattr(check, "PLAN_DIR", tmp_path / "fictional_plan")
    monkeypatch.setattr(check, "OUT", tmp_path / "fictional_output")
    for index, relative in enumerate(check.SOURCES):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("fictional_source_"+str(index))
    class FakeBudget:
        def guard(self):
            pass
    monkeypatch.setattr(check, "Budget", FakeBudget)
    monkeypatch.setattr(check, "environment", lambda: {})
    monkeypatch.setattr(check, "graphics_environment", lambda env: {})
    monkeypatch.setattr(check, "INPUTS", {})
    monkeypatch.setattr(check, "registration", lambda: {"fictional": True})
    monkeypatch.setattr(check, "baseline_rows", lambda limit: rows(limit))
    monkeypatch.setattr(check, "selection_metadata", lambda selected: ({"fictional": True}, {}))
    monkeypatch.setattr(check, "lease_guard", lambda: [123])
    monkeypatch.setattr(check, "process_identity", lambda: {"fictional": True})
    monkeypatch.setattr(check, "prospective_quota", lambda *args: {"fits_unchanged_limits": False})
    with pytest.raises(ValueError, match="FIRST16.*quota denied"):
        check.prepare()
    assert not check.PLAN_DIR.exists() and not check.OUT.exists()


def test_bounded_last_read_does_not_promote_raw_metadata_to_permission():
    raw = dict(slot=1208, package_path="/fictional/package.npz", package_sha256="a"*64,
        members={"target.npy": {"whole_member_crc_verified": False, "member_EOF_and_trailer_checked": False,
        "prefix_body_bytes_inflated": 123, "checkpoint_reused_prefix_bytes": 456}},
        rgb_true_decoded=False, caller_admission_authenticated_by_reader=False)
    actual = check.bounded_reader_witness(raw)
    assert actual == raw and actual is not raw
    assert actual["members"]["target.npy"]["whole_member_crc_verified"] is False
    for flag in ("rgb_true_decoded", "caller_admission_authenticated_by_reader"):
        forged = deepcopy(raw)
        forged[flag] = True
        with pytest.raises(ValueError):
            check.bounded_reader_witness(forged)


@pytest.mark.parametrize("bad", [[1], b"bytes", -1, 2**63, "x"*4097, "nul\0"])
def test_oversized_or_non_scalar_reader_witness_fails_instead_of_omitting(bad):
    raw = dict(rgb_true_decoded=False, caller_admission_authenticated_by_reader=False, extra=bad)
    with pytest.raises(ValueError):
        check.bounded_reader_witness(raw)


def stats(enabled):
    return dict(checkpoint_cache_enabled=enabled, checkpoint_state_account_bytes=256*1024,
        checkpoint_stride_bytes=32*1024**2, max_checkpoint_states=48 if enabled else 0,
        cache_account_limit_bytes=12*1024**2 if enabled else 0, maximum_cache_accounted_bytes=0,
        maximum_checkpoint_states_observed=0, native_row_cache_payload_bytes=0,
        state_accounting_is_reservation_not_exact_native_allocation=True, owner_checkpoint_calls=16,
        owner_checkpoint_guard_stride=16)


@pytest.mark.parametrize("enabled", [False, True])
def test_stats_remain_exact_raw_dictionary_with_no_fabricated_admission_flags(enabled):
    class Reader:
        def stats_snapshot(self):
            return stats(enabled)
    assert check.reader_stats(Reader(), enabled) == stats(enabled)
    assert "caller_admission_authenticated_by_reader" not in check.reader_stats(Reader(), enabled)


def test_secondary_reader_close_observations_are_retained_without_a_new_denominator():
    class Reader:
        closing_errors = [dict(scope="original:before_close_guard", type="ValueError", message="secondary generation failure")]
    actual = check.closing_observations(Reader())
    assert actual == Reader.closing_errors and actual is not Reader.closing_errors
    assert "FAIL_batches" not in actual[0]


@pytest.mark.parametrize("passed,active,expected", [(0, False, (0, 0, 16)), (0, True, (0, 1, 15)),
    (7, False, (7, 0, 9)), (7, True, (7, 1, 8)), (16, False, (16, 0, 0))])
def test_post_PASS_guard_failure_does_not_recount_completed_batch_or_pair(passed, active, expected):
    actual = check.denominator(passed, active_failure=active)
    assert tuple(actual[key] for key in ("PASS_batches", "FAIL_batches", "UNEXECUTED_batches")) == expected


def test_completed16_cannot_also_have_active_failed_attempt():
    with pytest.raises(ValueError):
        check.denominator(16, active_failure=True)


def rescued_receipt():
    row = baseline()
    return dict(step=1, same_box_for_actual_RGB_and_GT=True,
        journal=dict(baseline_exposure=row, rescued=[True]+[False]*7,
            full_GT_sign_pixels=[3]+[0]*7, original_NEAREST_sign_pixels=[0]*8,
            selected_crop_boxes=[[0, 0, 360, 640]]+[deepcopy(box) for box in row["shared_crop_boxes"][1:]]))


def test_rescue_receipt_requires_actual_positive_selected_GT_and_retains_occurrence():
    receipt = rescued_receipt()
    result = check.rescue_witnesses(receipt, [3]+[0]*7)
    assert result == [dict(step=1, position=0, slot=1, full_GT_sign_pixels=3,
        proposed_NEAREST_GT_sign_pixels=0, selected_NEAREST_GT_sign_pixels=3,
        proposed_box=[0, 0, 300, 400], selected_box=[0, 0, 360, 640], actual_same_box_for_RGB_GT=True)]
    with pytest.raises(ValueError, match="actual rescue"):
        check.rescue_witnesses(receipt, [0]*8)
    receipt["same_box_for_actual_RGB_and_GT"] = False
    with pytest.raises(ValueError):
        check.rescue_witnesses(receipt, [3]+[0]*7)


def test_no_rescue_receipt_does_not_invent_an_occurrence():
    receipt = rescued_receipt()
    receipt["journal"]["rescued"] = [False]*8
    assert check.rescue_witnesses(receipt, [0]*8) == []


def test_source_gate_archive_cleanup_and_full_denominator_are_explicit():
    source = Path(check.__file__).read_text()
    tree = ast.parse(source)
    module_imports = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
    for node in module_imports:
        names = [alias.name for alias in node.names] if isinstance(node, ast.Import) else [node.module]
        assert not set(names) & {"numpy", "PIL", "torch", "carla", "rclpy"}
    prepare = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "prepare")
    calls = {ast.unparse(node.func) for node in ast.walk(prepare) if isinstance(node, ast.Call)}
    assert not calls & {"np.load", "native.BoundedNativePairReader", "worker.build_batch", "torch.load"}
    assert {"baseline_rows", "registration", "environment", "prospective_quota"} <= calls
    assert '"xb"' in source and 'OUT.mkdir(exist_ok=False)' in source
    assert 'fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)' in source
    assert 'process_identity() == self.process_generation' in source
    assert 'raw_plan == canonical(PLAN_DIR / "candidate_plan.json").read_bytes()' in source
    assert 'reader.close()' in source and 'value.close()' in source and 'cleanup.append' in source
    assert 'arm["PASS_batches"]+arm["FAIL_batches"]+arm["UNEXECUTED_batches"] == 16' in source
    assert 'COMPLETE_BYTES_RESCUE_NOT_OBSERVED' in source
    assert 'model_performance_improved=False' in source
    assert 'old8_batch74second_run_used_as_comparable_timing_baseline=False' in source
    assert 'ego_or_route_observed=False, invented_ego_or_route_drawn=False' in source
