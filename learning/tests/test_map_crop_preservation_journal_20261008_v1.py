"""HH_261008 - Pure provided-scalar/AST fixtures; no arrays, writer, real journals or training."""
import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import pytest
from portable_e2e import map_crop_preservation_journal_20261008_v1 as journal


def baseline(step=1):
    return dict(step=step, physical_slot_indices=[0,1,728,729,730,1208,42,42],
        shared_crop_boxes=[[0,0,240,400] for _ in range(8)], actual_paired_TRUE_bits=[0]*8,
        RGB_sha256="a"*64, target_sha256="b"*64)


def receipt(step=1, rescue=False):
    original = baseline(step)
    full = [3]*8 if rescue else [0]*8
    selected = [([0,0,360,640] if rescue and position % 2 == 0 else list(box))
        for position, box in enumerate(original["shared_crop_boxes"])]
    return journal.batch_receipt(original, selected, full, [0]*8,
        "c"*64 if rescue else original["RGB_sha256"], "d"*64 if rescue else original["target_sha256"])


def test_exact_policy_source_pin_and_no_array_file_writer_or_training_imports():
    path = Path(journal.policy.__file__)
    assert hashlib.sha256(path.read_bytes()).hexdigest() == journal.POLICY_SOURCE_SHA256
    assert path.name == "map_train_sign_preserving_crop_policy_20261008_v1.py"
    source = Path(journal.__file__).read_bytes()
    tree = ast.parse(source)
    imports = {n.module for n in tree.body if isinstance(n, ast.ImportFrom)}
    assert imports == {"copy", "scripts.e2e"}
    assert not any(isinstance(n, ast.Import) for n in tree.body)
    calls = {ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n, ast.Call)}
    assert not calls & {"open", "Path", "np.load", "np.save", "torch.load", "appendwriter", "exec", "eval"}
    assert {"policy.select_box", "policy.exposure_index", "policy.checked_box"} <= calls
    assert journal.STEPS == 20000 and journal.BATCH == 8 and journal.TOTAL == 1209


@pytest.mark.parametrize("step", [1,2,20000])
def test_fixed_exposure_parity_and_distinct_schema_proposals_not_selected(step):
    row = receipt(step, True)
    assert row["protected"] == row["rescued"] == [True,False]*4
    assert row["selected_crop_boxes"][0] == [0,0,360,640]
    assert row["baseline_exposure"]["shared_crop_boxes"][0] == [0,0,240,400]
    assert row["candidate_RGB_sha256"] == "c"*64 and row["candidate_target_sha256"] == "d"*64
    result = journal.validate_batch(row, baseline(step))
    assert result["rescued"] == 4 and result["actual_array_bytes_authenticated"] is False
    assert result["actual_RNG_or_baseline_RAW_bytes_authenticated"] is False
    assert result["training_or_model_approved"] is False
    assert json.loads(json.dumps(row, allow_nan=False)) == row


def test_builder_no_baseline_alias_no_mutation_and_no_rescue_exact_actual_hashes():
    original = baseline()
    before = deepcopy(original)
    full = [0]*8
    selected = deepcopy(original["shared_crop_boxes"])
    row = journal.batch_receipt(original, selected, full, [0]*8, "a"*64, "b"*64)
    assert original == before and row["baseline_exposure"] is not original
    row["baseline_exposure"]["shared_crop_boxes"][0][0] = 1
    row["selected_crop_boxes"][1][0] = 1
    row["full_GT_sign_pixels"][0] = 1
    assert original == before and selected[1][0] == 0 and full == [0]*8


def test_unknown_selected_object_denied_before_deepcopy_hook():
    class Foreign:
        def __deepcopy__(self, memo):
            raise AssertionError("foreign hook must never execute")
    with pytest.raises(ValueError):
        journal.batch_receipt(baseline(), [Foreign()]*8, [0]*8, [0]*8, "a"*64, "b"*64)


@pytest.mark.parametrize("field", ["candidate_RGB_sha256", "candidate_target_sha256"])
def test_NO_rescue_hash_change_denied(field):
    row = receipt()
    row[field] = "f"*64
    with pytest.raises(ValueError, match="NO-rescue"):
        journal.validate_batch(row, baseline())


def test_rescue_hashes_preserved_not_forced_equal_or_claimed_array_authenticity():
    row = receipt(rescue=True)
    row["candidate_RGB_sha256"] = "e"*64
    row["candidate_target_sha256"] = "f"*64
    journal.validate_batch(row, baseline())
    assert row["candidate_RGB_sha256"] == "e"*64 and row["candidate_target_sha256"] == "f"*64


@pytest.mark.parametrize("field,value", [("step",True),("step",1.),("step",0),("step",20001),
    ("physical_slot_indices",[True]*8),("physical_slot_indices",[-1]*8),("physical_slot_indices",[1209]*8),
    ("physical_slot_indices",[0]*7),("actual_paired_TRUE_bits",[False]*8),
    ("actual_paired_TRUE_bits",[1]*8),("actual_paired_TRUE_bits",[0.]*8),
    ("RGB_sha256","A"*64),("target_sha256","g"*64),("target_sha256",False)])
def test_original_six_field_strict_types_shapes_FALSE_only_and_sha(field,value):
    row = baseline()
    row[field] = value
    with pytest.raises(ValueError):
        journal.validate_baseline(row)


@pytest.mark.parametrize("bad", [[],[0,0,360],[-1,0,360,640],[0,0,361,640],
    [0,1,360,640],[True,0,240,400],[0.,0,240,400],(0,0,240,400)])
def test_original_proposal_bounds_no_coercion_or_tuple_JSON_alias(bad):
    row = baseline()
    row["shared_crop_boxes"][0] = bad
    with pytest.raises(ValueError):
        journal.validate_baseline(row)


@pytest.mark.parametrize("fault", ["old_schema","old_row","extra_field","policy_pin","baseline_step",
    "baseline_ids","baseline_proposal","baseline_hash","selected","protected_int","protected_parity",
    "rescue_int","false_rescue","GT_bool","GT_float","GT_negative","GT_large",
    "NEAREST_bool","NEAREST_large","negative_invented","short_flags","selected_tuple"])
def test_candidate_contract_denials_do_not_relabel_or_change_original_proposal(fault):
    row = receipt()
    if fault == "old_schema":row["schema"] = "old.exposure"
    elif fault == "old_row":row = baseline()
    elif fault == "extra_field":row["unused"] = True
    elif fault == "policy_pin":row["policy_source_sha256"] = "f"*64
    elif fault == "baseline_step":row["baseline_exposure"]["step"] = 2
    elif fault == "baseline_ids":row["baseline_exposure"]["physical_slot_indices"][0] = 5
    elif fault == "baseline_proposal":row["baseline_exposure"]["shared_crop_boxes"][0][1] = 1
    elif fault == "baseline_hash":row["baseline_exposure"]["RGB_sha256"] = "f"*64
    elif fault == "selected":row["selected_crop_boxes"][0] = [0,0,360,640]
    elif fault == "protected_int":row["protected"][0] = 1
    elif fault == "protected_parity":row["protected"][1] = True
    elif fault == "rescue_int":row["rescued"][0] = 0
    elif fault == "false_rescue":row["rescued"][0] = True
    elif fault == "GT_bool":row["full_GT_sign_pixels"][0] = False
    elif fault == "GT_float":row["full_GT_sign_pixels"][0] = 0.
    elif fault == "GT_negative":row["full_GT_sign_pixels"][0] = -1
    elif fault == "GT_large":row["full_GT_sign_pixels"][0] = journal.PIXELS+1
    elif fault == "NEAREST_bool":row["original_NEAREST_sign_pixels"][0] = False
    elif fault == "NEAREST_large":row["original_NEAREST_sign_pixels"][0] = journal.PIXELS+1
    elif fault == "negative_invented":row["original_NEAREST_sign_pixels"][0] = 1
    elif fault == "short_flags":row["rescued"].pop()
    else:row["selected_crop_boxes"][0] = (0,0,240,400)
    with pytest.raises(ValueError):
        journal.validate_batch(row, baseline())


def test_full_FOV_original_copy_support_and_visible_or_unprotected_preserved():
    original = baseline()
    original["shared_crop_boxes"] = [[0,0,360,640] for _ in range(8)]
    row = journal.batch_receipt(original, deepcopy(original["shared_crop_boxes"]), [4]*8, [4]*8, "a"*64, "b"*64)
    assert not any(row["rescued"])
    row["original_NEAREST_sign_pixels"][0] = 3
    with pytest.raises(ValueError):
        journal.validate_batch(row, original)
    row = receipt(rescue=True)
    row["selected_crop_boxes"][1] = [0,0,360,640]
    with pytest.raises(ValueError):
        journal.validate_batch(row, baseline())


def full_baseline():
    return [baseline(step) for step in range(1,20001)]


def test_full20k_baseline_required_partial_and_failed_candidate_denominators_no_padding():
    originals = full_baseline()
    rows = [receipt(),receipt(2)]
    result = journal.audit_sequence(originals, rows)
    assert (result["PASS_batches"],result["FAIL_batches"],result["UNEXECUTED_batches"]) == (2,0,19998)
    assert result["candidate_provided_rows"] == result["candidate_examined_rows"] == 2
    rows[1]["protected"][0] = False
    rows.append(receipt(3))
    result = journal.audit_sequence(originals, rows)
    assert (result["PASS_batches"],result["FAIL_batches"],result["UNEXECUTED_batches"]) == (1,1,19998)
    assert result["supplied_rows_after_first_failure_unexamined"] == 1
    assert result["PASS_exposures"]+result["FAIL_exposures"]+result["UNEXECUTED_exposures"] == 160000
    assert result["failure"]["ordinal"] == 1 and originals[1] == baseline(2)
    with pytest.raises(ValueError):journal.audit_sequence(originals[:-1], [])
    originals[17]["step"] = 17
    with pytest.raises(ValueError):journal.audit_sequence(originals, [])


def test_full_complete20k_and_extra_row_kept_outside_declared_denominator(monkeypatch):
    # HH_261008 - Real20k chronology/sequence branches, with batch validator isolated after its separate genuine policy fixtures.
    originals = full_baseline()
    calls = []
    def checked(row, expected):
        assert row is expected
        calls.append(expected["step"])
    monkeypatch.setattr(journal, "validate_batch", checked)
    result = journal.audit_sequence(originals, list(originals))
    assert calls == list(range(1,20001)) and result["PASS_batches"] == 20000
    assert result["FAIL_batches"] == result["UNEXECUTED_batches"] == 0
    assert result["status"] == "COMPLETE_PROVIDED_SCALAR_CONTRACT_ONLY"
    assert result["actual_array_bytes_authenticated"] is result["training_or_model_approved"] is False
    result = journal.audit_sequence(originals, [*originals,{"EXTRA":"retained by caller"}])
    assert result["status"] == "FAILED_PROVIDED_SCALAR_CONTRACT" and result["extra_supplied_rows"] == 1
    assert (result["PASS_batches"],result["FAIL_batches"],result["UNEXECUTED_batches"]) == (20000,0,0)
    assert result["failure"]["outside_declared_denominator"] is True


def test_duplicate_out_of_order_and_unknown_supplied_sequences_deny():
    originals = full_baseline()
    row = receipt()
    result = journal.audit_sequence(originals,[row,row])
    assert result["PASS_batches"] == result["FAIL_batches"] == 1 and result["UNEXECUTED_batches"] == 19998
    with pytest.raises(ValueError):journal.audit_sequence(tuple(originals),[])
    with pytest.raises(ValueError):journal.audit_sequence(originals,iter([]))
    with pytest.raises(ValueError):journal.audit_sequence(originals,originals)


def test_sequence_error_prefix_bounded_and_baseline_unknown_fields_not_accepted(monkeypatch):
    originals = full_baseline()
    def broken(*_):raise ValueError("x"*1000)
    monkeypatch.setattr(journal,"validate_batch",broken)
    result = journal.audit_sequence(originals,[{}])
    assert len(result["failure"]["error_prefix"]) == 256 and result["failure"]["error_text_truncated"] is True
    originals[0]["schema"] = "old"
    with pytest.raises(ValueError):journal.audit_sequence(originals,[])
