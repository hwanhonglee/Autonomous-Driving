"""HH_261008 - Scalar-only proposed-versus-selected TRAIN crop journal contract.

No arrays, file I/O, append writer, training, registry change or permission is
implemented here. The caller must seal the original policy source, unchanged
whole baseline RAW journal and actual RGB/GT bytes independently. Supplied GT
counts and SHA strings do NOT authenticate pixels, RNG state or array bytes.
This distinct candidate schema must never overwrite/relabel an old journal.
"""
from copy import deepcopy

from scripts.e2e import map_train_sign_preserving_crop_policy_20261008_v1 as policy

SCHEMA = "portable_e2e.map_crop_preservation_journal.20261008.v1"
POLICY_SOURCE_PATH = "tools/e2e/map_train_sign_preserving_crop_policy_20261008_v1.py"
POLICY_SOURCE_SHA256 = "566b3e2afb7a0c879071c806c0cbfef6d31b8e974df99423339b7bf48cd6209f"
STEPS, BATCH, TOTAL, PIXELS = 20000, 8, 1209, 360 * 640
BASELINE_FIELDS = frozenset(("step", "physical_slot_indices", "shared_crop_boxes",
    "actual_paired_TRUE_bits", "RGB_sha256", "target_sha256"))
FIELDS = frozenset(("schema", "policy_source_sha256", "baseline_exposure",
    "selected_crop_boxes", "full_GT_sign_pixels", "original_NEAREST_sign_pixels",
    "protected", "rescued", "candidate_RGB_sha256", "candidate_target_sha256"))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def eight(value, message):
    require(type(value) is list and len(value) == BATCH, message)
    return value


def box(value):
    require(type(value) is list, "JSON list crop box required; no implicit coercion")
    policy.checked_box(value)
    return value


def validate_baseline(row, expected_step=None):
    require(type(row) is dict and set(row) == BASELINE_FIELDS, "exact original six-field baseline metadata required")
    step = row["step"]
    require(type(step) is int and 1 <= step <= STEPS
        and (expected_step is None or step == expected_step), "exact one-based original step/order required")
    indices = eight(row["physical_slot_indices"], "exact original eight slots required")
    require(all(type(i) is int and 0 <= i < TOTAL for i in indices), "original physical slot integer/range differs")
    for proposal in eight(row["shared_crop_boxes"], "exact eight original proposals required"):
        box(proposal)
    bits = eight(row["actual_paired_TRUE_bits"], "exact original eight FALSE bits required")
    require(all(type(bit) is int and bit == 0 for bit in bits), "TRAIN FALSE-only bits must be integer zero")
    require(sha(row["RGB_sha256"]) and sha(row["target_sha256"]), "original actual RGB/GT SHA metadata invalid")
    return row


def validate_batch(row, expected_baseline):
    """HH_261008 - Validate provided metadata, never certify actual pixel/proposal authenticity."""
    validate_baseline(expected_baseline)
    require(type(row) is dict and set(row) == FIELDS and type(row["schema"]) is str and row["schema"] == SCHEMA
        and type(row["policy_source_sha256"]) is str and row["policy_source_sha256"] == POLICY_SOURCE_SHA256,
        "distinct exact candidate schema/policy pin required")
    baseline = validate_baseline(row["baseline_exposure"])
    require(baseline == expected_baseline, "original six-field baseline/proposal/indices/FALSE metadata changed")
    selected = eight(row["selected_crop_boxes"], "exact eight selected boxes required")
    full = eight(row["full_GT_sign_pixels"], "eight native GT support counts required")
    original = eight(row["original_NEAREST_sign_pixels"], "eight original NEAREST support counts required")
    protected = eight(row["protected"], "eight protection booleans required")
    rescued = eight(row["rescued"], "eight rescue booleans required")
    require(all(type(v) is bool for v in protected + rescued), "protection/rescue require actual bool, not integer")
    for position, proposal in enumerate(baseline["shared_crop_boxes"]):
        box(selected[position])
        ordinal = policy.exposure_index(baseline["step"] - 1, position)
        choice = policy.select_box(ordinal, proposal, full[position], original[position])
        require(selected[position] == choice["selected_box"] and protected[position] is choice["protected"]
            and rescued[position] is choice["rescued"], "selected box/support/parity/rescue metadata contradicts exact policy")
    require(sha(row["candidate_RGB_sha256"]) and sha(row["candidate_target_sha256"]), "new actual RGB/GT SHA metadata invalid")
    if not any(rescued):
        require(row["candidate_RGB_sha256"] == baseline["RGB_sha256"]
            and row["candidate_target_sha256"] == baseline["target_sha256"], "NO-rescue batch actual RGB and GT hashes must remain original")
    # Rescue hashes are preserved as supplied, not forced equal/different or treated as authenticated array bytes.
    return dict(step=baseline["step"], exposures=BATCH, protected=sum(protected), rescued=sum(rescued),
        metadata_contract_valid=True, actual_array_bytes_authenticated=False,
        actual_RNG_or_baseline_RAW_bytes_authenticated=False, training_or_model_approved=False)


def batch_receipt(baseline, selected_boxes, full_GT_sign_pixels, original_NEAREST_sign_pixels,
                  candidate_RGB_sha256, candidate_target_sha256):
    """HH_261008 - Return a separate owned JSON-safe wrapper; never mutate the supplied baseline."""
    validate_baseline(baseline)
    for selected in eight(selected_boxes, "eight selected JSON boxes required before copying"):
        box(selected)
    eight(full_GT_sign_pixels, "eight GT counts required")
    eight(original_NEAREST_sign_pixels, "eight original counts required")
    require(sha(candidate_RGB_sha256) and sha(candidate_target_sha256), "typed candidate hashes required before copying")
    choices = [policy.select_box(policy.exposure_index(baseline["step"] - 1, position), proposal,
        full_GT_sign_pixels[position], original_NEAREST_sign_pixels[position])
        for position, proposal in enumerate(baseline["shared_crop_boxes"])]
    row = dict(schema=SCHEMA, policy_source_sha256=POLICY_SOURCE_SHA256, baseline_exposure=deepcopy(baseline),
        selected_crop_boxes=deepcopy(selected_boxes), full_GT_sign_pixels=deepcopy(full_GT_sign_pixels),
        original_NEAREST_sign_pixels=deepcopy(original_NEAREST_sign_pixels),
        protected=[c["protected"] for c in choices], rescued=[c["rescued"] for c in choices],
        candidate_RGB_sha256=candidate_RGB_sha256, candidate_target_sha256=candidate_target_sha256)
    validate_batch(row, baseline)
    return row


def audit_sequence(baseline_rows, candidate_rows):
    """HH_261008 - Check supplied ordered prefixes; retain all20k PASS/FAIL/UNEXECUTED, never pad rows.

    The complete immutable baseline is REQUIRED as provided scalar metadata.
    A first invalid candidate leaves later supplied rows explicitly unexamined;
    extra rows have a separate witness and never subtract from the20k denominator.
    Original RAW journal preservation is the external caller's responsibility.
    """
    require(type(baseline_rows) is list and len(baseline_rows) == STEPS, "complete original20k baseline required")
    require(type(candidate_rows) is list and candidate_rows is not baseline_rows,
        "separate provided candidate list required; no baseline container reuse/implicit iterator padding")
    for ordinal, baseline in enumerate(baseline_rows):
        validate_baseline(baseline, ordinal + 1)
    passed = failed = examined = 0
    failure = None
    for ordinal, row in enumerate(candidate_rows[:STEPS]):
        examined += 1
        try:
            validate_batch(row, baseline_rows[ordinal])
        except ValueError as error:
            failed = 1
            message = str(error)
            failure = dict(ordinal=ordinal, step=ordinal + 1, error_type=type(error).__name__,
                error_prefix=message[:256], error_text_truncated=len(message) > 256,
                failed_supplied_row_retained_by_caller=True)
            break
        passed += 1
    extra = max(0, len(candidate_rows) - STEPS)
    if failure is None and extra:
        failure = dict(first_extra_ordinal=STEPS, extra_supplied_rows=extra, outside_declared_denominator=True)
    status = ("FAILED_PROVIDED_SCALAR_CONTRACT" if failure else
        "COMPLETE_PROVIDED_SCALAR_CONTRACT_ONLY" if passed == STEPS else "INCOMPLETE_PROVIDED_SCALAR_PREFIX")
    return dict(schema=SCHEMA + ".sequence", status=status, expected_batches=STEPS,
        PASS_batches=passed, FAIL_batches=failed, UNEXECUTED_batches=STEPS-passed-failed,
        expected_candidate_exposures=STEPS*BATCH, PASS_exposures=passed*BATCH,
        FAIL_exposures=failed*BATCH, UNEXECUTED_exposures=(STEPS-passed-failed)*BATCH,
        baseline_provided_batches=len(baseline_rows), baseline_rows_modified=False,
        candidate_provided_rows=len(candidate_rows), candidate_examined_rows=examined,
        supplied_rows_after_first_failure_unexamined=max(0, len(candidate_rows)-examined) if failed else 0,
        extra_supplied_rows=extra, failure=failure, no_padding_or_overwrite=True,
        actual_array_bytes_authenticated=False, actual_RNG_or_baseline_RAW_bytes_authenticated=False,
        training_or_model_approved=False)
