"""HH_261004 - Reuse frozen pure media tests under a private V3 namespace and test both exact identity seams.

All archive bytes are synthetic test-owned fixtures.  Tests never read an actual
drive, render a real episode, call CARLA, import Torch or approve a capability.
"""

import ast
import copy
import hashlib
import inspect
from pathlib import Path
import sys
import textwrap
from types import FunctionType

import pytest

from scripts.e2e import publish_temporal_route_joint_town03_straight_full209_v3_20261004 as m
import test_publish_town03_straight_full209_v2_20261004 as frozen_tests


def _test_delegate(function):
    namespace={**function.__globals__,"m":m,"plan_values":plan_values,"selection":selection,
        "policy_values":policy_values,"FakePins":frozen_tests.FakePins}
    result=FunctionType(function.__code__,namespace,function.__name__,function.__defaults__,function.__closure__)
    result.__dict__.update(function.__dict__)
    return result


def selection():
    return _test_delegate(frozen_tests.selection)()


def plan_values():
    value=_test_delegate(frozen_tests.plan_values)()
    value["run_id"]="v3"
    return value


def policy_values():
    return _test_delegate(frozen_tests.policy_values)()


# HH_261004 - Frozen test code is reused byte-identically; its fixture's fixed paths point only to its private temporary root.
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
):
    globals()[_name]=_test_delegate(getattr(frozen_tests,_name))


def test_separate_V3_owner_original_raw_ABI_and_pure_source_closure():
    m.verify_frozen()
    assert m.OWNER_SCHEMA.endswith(".v3") and m.CAPTURE_SCHEMA.endswith(".v1")
    assert m.RUN.name.endswith("full209_v3") and m.OUTPUT.name=="13_town03_full_route209_v3"
    assert m.POLICY_PATH.parent.name=="town03_known_dev_straight_full209_v3"
    assert "torch" not in sys.modules and "carla" not in sys.modules
    sources=m.publication_sources()
    assert Path(m.__file__).resolve() in sources and Path(m.previous.__file__).resolve() in sources
    assert not any("research_temporal_route_joint_town03_straight_full209" in str(path) for path in sources)


def test_original_module_globals_and_exact_delegated_code_are_not_mutated():
    before=dict(vars(m.previous))
    private=m._private(m.previous.seal_policy)
    assert private.__code__ is m.previous.seal_policy.__code__
    assert private.__globals__ is not m.previous.seal_policy.__globals__
    assert private.__globals__["RUN"]==m.RUN and private.__globals__["seal_policy"] is m.seal_policy
    assert dict(vars(m.previous))==before


@pytest.mark.parametrize("seam,function", [("archive",m.previous.previous.load_archive),("policy",m.previous.policy_document)])
def test_run_identity_AST_inverse_preserves_every_original_guard(seam,function):
    tree=ast.parse(textwrap.dedent(inspect.getsource(function))).body[0]
    changed=copy.deepcopy(tree)
    undo=(m._run_id_seam if seam=="archive" else m._policy_identity_seams)(changed)
    assert "plan.get('run_id') == 'v3'" in ast.unparse(changed)
    undo(changed)
    assert ast.dump(changed)==ast.dump(tree)


@pytest.mark.parametrize("seam", ["archive","policy"])
def test_missing_run_identity_AST_anchor_fails_closed(seam):
    function=m.previous.previous.load_archive if seam=="archive" else m.previous.policy_document
    tree=ast.parse(textwrap.dedent(inspect.getsource(function))).body[0]
    for node in ast.walk(tree):
        if isinstance(node,ast.Compare) and ast.unparse(node.left)=="plan.get('run_id')":
            node.comparators[0].value="MALFORMED_FROZEN_ANCHOR"
    with pytest.raises(RuntimeError): (m._run_id_seam if seam=="archive" else m._policy_identity_seams)(tree)


@pytest.mark.parametrize("fault", ["owner","raw_policy","raw_plan","late","observed","retry","gallery",
    "GIF","PNG","hold","crop","adapt","UI","generated","declaration","approval","independent"])
def test_cross_revision_or_outcome_adaptive_policy_never_admitted(fault):
    caller,pins,plan,policy=policy_values()
    if fault=="owner": policy["owner_schema"]=m.previous.OWNER_SCHEMA
    elif fault=="raw_policy": policy["capture_raw_ABI_schema"]=m.OWNER_SCHEMA
    elif fault=="raw_plan": plan["capture_raw_ABI_schema"]="arbitrary_raw_ABI"
    elif fault=="late": policy["created_before_actual_episode"]=False
    elif fault=="observed": policy["actual_episode_observed"]=True
    elif fault=="retry": policy["automatic_retry"]=True
    elif fault=="gallery": policy["media_selection"]["gallery"]+="_other"
    elif fault=="GIF": policy["media_selection"]["GIF_camera_indices"]="best scenes"
    elif fault=="PNG": policy["media_selection"]["PNG_camera_indices"]="successful segment"
    elif fault=="hold": policy["media_selection"]["GIF_playback"]="new future camera timestamp"
    elif fault=="crop": policy["media_selection"]["complete_actual_interval_including_failures_required"]=False
    elif fault=="adapt": policy["media_selection"]["outcome_or_failure_dependent_frame_selection"]=True
    elif fault=="UI": policy["media_selection"]["actual_Autoware_or_RViz_screenshot_claim"]=True
    elif fault=="generated": policy["media_selection"]["illustrative_or_generated_image_allowed"]=True
    elif fault=="declaration": caller["run_files_sha256"]["declaration.json"]="c"*64
    elif fault=="approval": policy["full_RTE01_approved"]=True
    else: policy["independent_validation"]=True
    with pytest.raises(RuntimeError): m.verify_policy(caller,pins,plan)


def test_V3_policy_uses_its_own_source_provenance_not_frozen_V2_sealer():
    value=m.policy_document(plan_values(),"a"*64)
    assert "actual V3 interval" in value["provenance"]
    assert value["root_sealer_sha256"]==hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest()
    assert value["final_publisher_source_frozen_before_capture_claimed"] is False
