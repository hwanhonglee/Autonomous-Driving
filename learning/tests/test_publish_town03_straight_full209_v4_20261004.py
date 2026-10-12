"""HH_261004 - Check new V4 media/policy identities with frozen pure synthetic tests; never reinterpret a failed actual run."""

import ast
import copy
import hashlib
import inspect
from pathlib import Path
import sys
import textwrap
from types import FunctionType

import pytest

from scripts.e2e import publish_temporal_route_joint_town03_straight_full209_v4_20261004 as m
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
    value=_test_delegate(frozen_tests.plan_values)();value["run_id"]="v4"
    return value


def policy_values(): return _test_delegate(frozen_tests.policy_values)()


# HH_261004 - Exact existing test code/parameterization exercises new private paths and complete unknown/failed denominators.
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
test_cross_revision_or_outcome_adaptive_policy_never_admitted=_test_delegate(
    V3_tests.test_cross_revision_or_outcome_adaptive_policy_never_admitted)


def test_new_owner_raw_ABI_paths_and_whole_pure_source_closure():
    m.verify_frozen()
    assert m.OWNER_SCHEMA.endswith(".v4") and m.CAPTURE_SCHEMA.endswith(".v1")
    assert m.RUN.name.endswith("full209_v4") and m.OUTPUT.name=="14_town03_full_route209_v4"
    assert m.POLICY_PATH.parent.name=="town03_known_dev_straight_full209_v4"
    assert m.POLICY_PATH != m.previous.POLICY_PATH
    assert "torch" not in sys.modules and "carla" not in sys.modules
    sources=m.publication_sources()
    assert Path(m.__file__).resolve() in sources and Path(m.previous.__file__).resolve() in sources
    assert not any("research_temporal_route_joint_town03_straight_full209" in str(path) for path in sources)


@pytest.mark.parametrize("seam,function",[("archive",m.previous.previous.previous.load_archive),
    ("policy",m.previous.previous.policy_document)])
def test_private_run_identity_inverse_AST_retains_all_guard_statements(seam,function):
    tree=ast.parse(textwrap.dedent(inspect.getsource(function))).body[0]
    changed=copy.deepcopy(tree);undo=(m._run_id_seam if seam=="archive" else m._policy_identity_seams)(changed)
    assert "plan.get('run_id') == 'v4'" in ast.unparse(changed)
    undo(changed);assert ast.dump(changed)==ast.dump(tree)


def test_V3_modules_and_policy_are_not_mutated_or_reused():
    before=dict(vars(m.previous))
    value=m.policy_document(plan_values(),"a"*64)
    assert "actual V4 interval" in value["provenance"] and value["owner_schema"]==m.OWNER_SCHEMA
    assert value["run_root"]==str(m.RUN) and value["output"]==str(m.OUTPUT)
    assert value["root_sealer_sha256"]==hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest()
    assert value["actual_episode_observed"] is value["final_publisher_source_frozen_before_capture_claimed"] is False
    assert dict(vars(m.previous))==before
