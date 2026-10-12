"""HH_261002 - Route only the auxiliary through true sign/other logits; shared-feature regressions remain possible."""
from __future__ import annotations

import ast
import copy
import hashlib
from pathlib import Path

try:
    from . import map_sign_margin_loss_v1_20261002 as previous
except ImportError:
    import map_sign_margin_loss_v1_20261002 as previous

SCHEMA = "portable_e2e.map_source_positive_image_sign_other_margin_loss.20261002.v1"
PARENT_SHA = "50029d77d4d216833a04ea140757a4f7b5d6cb8b183c9685bedacb05411e4286"
SIGN_CLASS, OTHER_CLASS, CLASSES, HARD_NEGATIVES, COEFFICIENT = 5, 0, 7, 64, .10
SOURCE_GROUPS = previous.SOURCE_GROUPS
require, _tensor_inputs = previous.require, previous._tensor_inputs
deterministic_hard_negative_mean = previous.deterministic_hard_negative_mean


def verify_frozen():
    require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == PARENT_SHA,
        "original seven-class auxiliary source changed")


def prepared_contract():
    verify_frozen(); result = previous.prepared_contract()
    result.update(schema=SCHEMA, margin="sign_logit-other_logit",
        conditional_probability="exp(sign)/(exp(sign)+exp(other)); NOT seven-class sign probability",
        eligible_negative_target_class=OTHER_CLASS,
        negative_reduction="mean_min(64,actual_GT_other_count)_hardest_softplus(m)_per_image_then_ALL_source_images",
        empty_other_image="differentiable_zero; remains in ALL-images negative denominator",
        direct_auxiliary_logit_gradient_classes=[OTHER_CLASS, SIGN_CLASS],
        nonpair_class_auxiliary_logit_gradients_exactly_zero=True,
        shared_feature_indirect_other_class_changes_possible=True,
        inherited_auxiliary_source_sha256=PARENT_SHA,
        candidate="same_original_weighted_semantic_CE_plus_0.10_pair_routed_auxiliary",
        performance_improvement_or_causality_proven=False)
    result.pop("empty_nonsign_image")
    return result


def revised_auxiliary_AST():
    # HH_261002 - Exactly TWO mathematical substitutions: conditional pair margin and actual-GT-other eligibility.
    verify_frozen(); tree = ast.parse(Path(previous.__file__).read_bytes())
    original = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "sign_margin_auxiliary")
    changed = copy.deepcopy(original); replacements = []
    for node in ast.walk(changed):
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "margin" for t in node.targets):
            require(ast.unparse(node.value) == "logits[:, SIGN_CLASS] - torch.logsumexp(logits[:, nonsign], dim=1)",
                "exact original one-versus-rest margin required")
            replacements.append((node, "value", node.value))
            node.value = ast.parse("logits[:, SIGN_CLASS] - logits[:, OTHER_CLASS]", mode="eval").body
        if isinstance(node, ast.Subscript) and ast.unparse(node) == "negative_nll[i][~positive]":
            replacements.append((node, "slice", node.slice))
            node.slice = ast.parse("target[i] == OTHER_CLASS", mode="eval").body
    require(len(replacements) == 2, "only exact pair margin and true-other mask may change")
    revised = copy.deepcopy(changed)
    for node, field, value in replacements: setattr(node, field, value)
    require(ast.dump(changed) == ast.dump(original), "another source/image/tie/empty reduction statement changed")
    return revised


def sign_margin_auxiliary(logits, target, source_groups):
    namespace = dict(vars(previous), OTHER_CLASS=OTHER_CLASS, SCHEMA=SCHEMA)
    node = revised_auxiliary_AST()
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])),
        str(__file__)+":exact_pair_routed_reduction", "exec"), namespace)
    value, stats = namespace["sign_margin_auxiliary"](logits, target, source_groups)
    # HH_261002 - Name the measured eligibility honestly; nonpair GT pixels are not auxiliary negatives.
    for row in stats["images"]: row["valid_other_pixels"] = row.pop("valid_nonsign_pixels")
    for row in stats["groups"].values(): row["actual_GT_other_pixels"] = row.pop("actual_GT_nonsign_pixels")
    stats.update(eligible_negative_target_class=OTHER_CLASS,
        conditional_pair_probability_not_seven_class_probability=True,
        direct_auxiliary_logit_gradient_classes=[OTHER_CLASS, SIGN_CLASS],
        shared_feature_indirect_other_class_changes_possible=True)
    return value, stats


def candidate_loss(logits, target, weights, source_groups, semantic_loss):
    # HH_261002 - Delegate the entire original one-call CE/coefficient/validation law, replacing ONLY its auxiliary provider.
    from types import FunctionType
    verify_frozen()
    namespace = dict(previous.candidate_loss.__globals__, sign_margin_auxiliary=sign_margin_auxiliary)
    return FunctionType(previous.candidate_loss.__code__, namespace, previous.candidate_loss.__name__)(
        logits, target, weights, source_groups, semantic_loss)
