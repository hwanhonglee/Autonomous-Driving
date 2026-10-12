"""HH_261005 - Check new V5 media/policy identities with frozen pure synthetic tests; never reinterpret a failed actual run."""

import ast
import copy
import hashlib
import inspect
from pathlib import Path
import sys
import textwrap
from types import FunctionType

import pytest

from scripts.e2e import publish_temporal_route_joint_town03_straight_full209_v5_20261005 as m
import test_publish_town03_straight_full209_v2_20261004 as frozen_tests
import test_publish_town03_straight_full209_v3_20261004 as V3_tests


def _test_delegate(function):
    namespace={**function.__globals__,"m":m,"plan_values":plan_values,"selection":selection,
        "policy_values":policy_values,"FakePins":frozen_tests.FakePins}
    result=FunctionType(function.__code__,namespace,function.__name__,function.__defaults__,function.__closure__)
    result.__dict__.update(function.__dict__)
    return result


def selection(): return _test_delegate(frozen_tests.selection)()


def plan_values():
    value=_test_delegate(frozen_tests.plan_values)();value["run_id"]="v5"
    return value


def policy_values(): return _test_delegate(frozen_tests.policy_values)()


# HH_261005 - Exact existing test code/parameterization exercises new private paths and complete unknown/failed denominators.
prospective_files=pytest.fixture(_test_delegate(frozen_tests.prospective_files.__wrapped__))
for _name in (
    "test_all_prospective_unknown_camera_denominators_preserve_fixed_complete_selection",
    "test_invalid_camera_denominators_cannot_select_a_prefix",
    "test_odd_even_final_native_and_camera_hold_are_disclosed_not_fabricated",
    "test_failed_maximum_interval_never_changes_sampling_to_success_only",
    "test_playback_refuses_selection_or_timestamp_relabeling",
    "test_policy_binds_fresh_owner_and_original_raw_ABI_to_declaration",
    "test_prospective_document_requires_real_centered_route_and_native_camera_disclosures",
    "test_prospective_document_cannot_approve_a_future_episode_or_learned_stop",
    "test_sealer_creates_exact_pinned_policy_before_any_actual_episode",
    "test_sealer_refuses_started_existing_or_mismatched_inputs_without_overwrite",
    "test_sealer_refuses_unsafe_source_names_and_nonexact_SHA_pins",
    "test_sealer_refuses_symlink_targets_and_parent_aliases",
    "test_validate_only_never_creates_gallery_or_decodes_records",
    "test_cross_run_archive_is_denied_before_camera_decode",
): globals()[_name]=_test_delegate(getattr(frozen_tests,_name))

# HH_261005 - Adapt only the frozen test's date-prefix expectation; inverse AST proves every source/policy/create-only assertion retained.
_date_function=frozen_tests.test_sealer_creates_exact_pinned_policy_before_any_actual_episode
_date_tree=ast.parse(textwrap.dedent(inspect.getsource(_date_function))).body[0]
_date_original=copy.deepcopy(_date_tree)
_date_count=0
for _row in ast.walk(_date_tree):
    if isinstance(_row,ast.Constant) and _row.value=="HH_261004 - ":
        _row.value="HH_261005 - ";_date_count+=1
assert _date_count==1
_date_restored=copy.deepcopy(_date_tree)
for _row in ast.walk(_date_restored):
    if isinstance(_row,ast.Constant) and _row.value=="HH_261005 - ": _row.value="HH_261004 - "
assert ast.dump(_date_restored)==ast.dump(_date_original)
_date_namespace=_test_delegate(_date_function).__globals__
exec(compile(ast.fix_missing_locations(ast.Module(body=[_date_tree],type_ignores=[])),__file__+":date_only_test","exec"),_date_namespace)
test_sealer_creates_exact_pinned_policy_before_any_actual_episode=_date_namespace[_date_function.__name__]
test_cross_revision_or_outcome_adaptive_policy_never_admitted=_test_delegate(
    V3_tests.test_cross_revision_or_outcome_adaptive_policy_never_admitted)


def test_new_owner_raw_ABI_paths_and_whole_pure_source_closure():
    m.verify_frozen()
    assert m.OWNER_SCHEMA.endswith(".v5") and m.CAPTURE_SCHEMA.endswith(".v1")
    assert m.RUN.name.endswith("full209_v5") and m.OUTPUT.name=="01_town03_full_route209_v5"
    assert m.POLICY_PATH.parent.name=="town03_known_dev_straight_full209_v5"
    assert m.POLICY_PATH != m.previous.POLICY_PATH
    assert "torch" not in sys.modules and "carla" not in sys.modules
    sources=m.publication_sources()
    assert Path(m.__file__).resolve() in sources and Path(m.previous.__file__).resolve() in sources
    assert not any("research_temporal_route_joint_town03_straight_full209" in str(path) for path in sources)


@pytest.mark.parametrize("seam,function",[("archive",m.previous.previous.previous.previous.load_archive),
    ("policy",m.previous.previous.previous.policy_document)])
def test_private_run_identity_inverse_AST_retains_all_guard_statements(seam,function):
    tree=ast.parse(textwrap.dedent(inspect.getsource(function))).body[0]
    changed=copy.deepcopy(tree);undo=(m._run_id_seam if seam=="archive" else m._policy_identity_seams)(changed)
    assert "plan.get('run_id') == 'v5'" in ast.unparse(changed)
    undo(changed);assert ast.dump(changed)==ast.dump(tree)


def test_V3_modules_and_policy_are_not_mutated_or_reused():
    before=dict(vars(m.previous))
    value=m.policy_document(plan_values(),"a"*64)
    assert "actual V5 interval" in value["provenance"] and value["owner_schema"]==m.OWNER_SCHEMA
    assert value["run_root"]==str(m.RUN) and value["output"]==str(m.OUTPUT)
    assert value["root_sealer_sha256"]==hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest()
    assert value["actual_episode_observed"] is value["final_publisher_source_frozen_before_capture_claimed"] is False
    assert dict(vars(m.previous))==before


def test_real_nested_archive_delegate_reaches_original_guard_with_new_actual_identities(tmp_path):
    # HH_261005 - No positive stub replaces archive validation: the genuine chain must reject before decode at its exact caller-SHA guard.
    path=tmp_path/"missing_caller.json"
    with pytest.raises((RuntimeError,ValueError,FileNotFoundError)):
        m.load_archive(path,"a"*64,decode=False)
    namespace=m._delegate(m.previous.previous.previous.previous.load_archive,transform=m._run_id_seam).__globals__
    assert namespace["RUN"]==m.RUN and namespace["OWNER_SCHEMA"]==m.OWNER_SCHEMA
    assert namespace["verify_policy"] is m.verify_policy
    assert namespace["load_archive"].__globals__ is namespace
    assert namespace["publication_sources"] is m.publication_sources
    publish_namespace=m._delegate(m.previous.previous.previous.previous.publish).__globals__
    assert publish_namespace["load_archive"] is m.load_archive


def test_declared_sampling_and_raw_render_math_are_the_exact_original_code():
    original=m.previous.previous.previous.previous
    assert m.camera_indices.__code__ is original.camera_indices.__code__
    assert m.png_indices.__code__ is original.png_indices.__code__
    assert m.playback.__code__ is original.playback.__code__
    assert m._delegate(original.render).__code__ is original.render.__code__
    assert m._delegate(original.render).__globals__["RUN"]==m.RUN
