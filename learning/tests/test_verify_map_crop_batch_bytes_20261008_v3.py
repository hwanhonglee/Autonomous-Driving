"""HH_261008 - Non-payload verifier contracts; no actual source admission or dataset replay."""
import ast
from pathlib import Path

import pytest

from scripts.e2e import verify_map_crop_batch_bytes_20261008_v3 as check


def test_fixed_limited_steps_no_full20k_or_TRAIN_permission():
    fixed = check.fixed()
    assert check.STEPS == [1, 2, 50, 100, 500, 1000, 10000, 20000]
    assert fixed["expected_batches"] == 8 and fixed["expected_slot_exposures"] == 64
    assert fixed["original20k_batches_outside_declaration"] == 19992
    assert fixed["paired_TRUE_member_decode_authorized"] is False
    assert fixed["baseline_both_RGB_and_GT_SHA_required"] is True
    assert fixed["sampled_self_VmHWM_cap_bytes"] == 512 * 1024**2
    assert fixed["maximum_combined_PLAN_OUTPUT_bytes"] == 2 * 1024**2
    assert fixed["closing_bytes"] == 512 * 1024
    grant = check.authority({}, "a" * 64)
    assert grant["old_target_only_authority_extended"] is False
    assert grant["limited_original_and_FALSE_RGB_GT_verification_authorized"] is True
    assert grant["training_authorized"] is grant["NN_authorized"] is False


@pytest.mark.parametrize("changed", ["wrong_key", "wrong_view_count", "bool_original", "float_slots"])
def test_genuine_five_field_registered_counts_remain_typed(changed):
    # HH_261008 - Use actual sealed metadata field names; never confuse1689 views with1209 physical slots.
    counts = dict(available_rgb_views=1689, logical_slots_per_arm=1209,
        new_rgb_views=960, original=729, physical_pairs=480)
    assert check.typed_equal(counts, check.NATIVE_COUNTS)
    if changed == "wrong_key": counts["logical_slots"] = counts.pop("logical_slots_per_arm")
    elif changed == "wrong_view_count": counts["available_rgb_views"] = 1209
    elif changed == "bool_original": counts["original"] = True
    else: counts["logical_slots_per_arm"] = 1209.0
    assert not check.typed_equal(counts, check.NATIVE_COUNTS)


@pytest.mark.parametrize("raw", [b'{"a":1,"a":2}', b'{"a":NaN}', b'{"a":Infinity}', b'{"a":1e309}'])
def test_duplicate_or_nonfinite_JSON_denied(raw):
    with pytest.raises(ValueError):
        check.decode(raw)


@pytest.mark.parametrize("left,right", [(1, True), (1, 1.0), ([3], [3.0]), ({"x":False}, {"x":0})])
def test_exact_TYPED_ROOT_declarations_do_not_coerce(left, right):
    assert not check.typed_equal(left, right)
    assert check.typed_equal(left, left)


def test_no_model_or_data_payload_loading_before_execute_source_gate():
    tree = ast.parse(Path(check.__file__).read_bytes())
    imports = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    assert not any(isinstance(n, ast.Import) and any(v.name in ('numpy','PIL','torch','carla','rclpy') for v in n.names) for n in imports)
    prepare = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'prepare')
    calls = {ast.unparse(n.func) for n in ast.walk(prepare) if isinstance(n, ast.Call)}
    assert not calls & {'np.load','native.BoundedNativePairReader','worker.build_batch','torch.load'}
    assert {'baseline_rows', 'registration', 'environment'} <= calls


def test_missing_actual_lease_denied():
    assert check.HELD_LEASE is None
    with pytest.raises(ValueError, match='held runtime'):
        check.lease_guard()


@pytest.mark.parametrize("cores", [0, 1, 2])
def test_optional_Pillow_plugins_are_not_the_RGB_resize_core(tmp_path, cores):
    # HH_261008 - Bounded synthetic names test the real native-provider census boundary.
    for suffix in ('tk','ft','cms','math','morph'):
        (tmp_path / ('_imaging' + suffix + '.cpython-310-test.so')).touch()
    for ordinal in range(cores):
        (tmp_path / ('_imaging.cpython-' + str(ordinal) + '-test.so')).touch()
    if cores == 1:
        assert check.selected_imaging(tmp_path).name == '_imaging.cpython-0-test.so'
    else:
        with pytest.raises(ValueError, match='exact Pillow'):
            check.selected_imaging(tmp_path)


def test_owner_metadata_writes_create_only_and_denominator_binding():
    tree = ast.parse(Path(check.__file__).read_bytes())
    source = Path(check.__file__).read_text()
    assert '"xb"' in source and 'OUT.mkdir(exist_ok=False)' in source
    assert 'UNEXECUTED_batches=8 - passed - failed' in source
    assert 'original20k_batches_outside_declaration=19992' in source
    assert 'value.close()' in source and 'cleanup.append' in source
    assert 'lease_guard()' in source and 'process_identity()' in source
