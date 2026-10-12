"""HH_261002 - Prepare a fixed loss-only pair-routing A/B; no fit is authorized by importing or reading this contract."""
from __future__ import annotations

from contextlib import contextmanager
import hashlib
from pathlib import Path

try:
    from . import train_map_sign_margin_repair_20261002 as previous
except ImportError:
    import train_map_sign_margin_repair_20261002 as previous
try:
    from portable_e2e import map_sign_other_margin_loss_v1_20261002 as losses
except ImportError:
    import map_sign_other_margin_loss_v1_20261002 as losses

NAME = "map_sign_other_margin_repair_20261002_v1"
SCHEMA = "portable_e2e.map_sign_other_margin_repair.20261002.v1"
SCHEDULE_SCHEMA, AUTHORIZATION_SCHEMA = SCHEMA+".equal_global_schedule", SCHEMA+".authorization"
WORKSPACE, DATA, MINING_OUTPUT = previous.WORKSPACE, previous.DATA, previous.MINING_OUTPUT
SOURCE, OUTPUT = WORKSPACE/"source_worktrees"/NAME, WORKSPACE/"runs/diagnostics"/NAME
PARENT_RUNNER_SHA = "3ede51218b97e4172a9685b349b727ef9bef0c1a6e2c555b44ca94cc15e4d43f"
FROZEN_SOURCE_SHA = dict(previous.FROZEN_SOURCE_SHA,
    **{Path(previous.__file__).name:PARENT_RUNNER_SHA, Path(losses.previous.__file__).name:losses.PARENT_SHA})
SOURCE_NAMES = set(FROZEN_SOURCE_SHA)|{Path(__file__).name, Path(losses.__file__).name}
require, document_sha = previous.require, previous.document_sha
_contract, _runtime, _postcheck = previous.prepared_contract, previous.verify_runtime_authorization, previous.postcheck
_old_loss, _old_source, _active = previous.losses, previous.SOURCE, False


def verify_frozen():
    require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == PARENT_RUNNER_SHA,
        "completed old loss-study runner changed")
    losses.verify_frozen()


def prepared_contract():
    # HH_261002 - Preserve all27 screens and every same-parent/schedule/crop/optimizer law; pair eligibility is the sole factor.
    from types import FunctionType
    verify_frozen()
    namespace = dict(_contract.__globals__, NAME=NAME, SCHEMA=SCHEMA, SOURCE=SOURCE, OUTPUT=OUTPUT, losses=losses)
    result = FunctionType(_contract.__code__, namespace, _contract.__name__)()
    result.update(sole_intervention="original_CE versus original_CE+0.10_sign_other_pair_routed_auxiliary",
        inherited_loss_study_runner_sha256=PARENT_RUNNER_SHA, inherited_loss_module_sha256=losses.PARENT_SHA,
        all_existing_twenty_plus_seven_screens_unchanged=True,
        direct_auxiliary_logits_are_only_sign5_other0=True,
        shared_feature_indirect_lane_or_recall_regression_possible=True,
        no_DEV_driven_coefficient_sampler_or_threshold_search=True,
        actual_old_study_21_of_27_FAILURE_retained=True,
        new_head=False, inference_argmax_or_threshold_changed=False,
        source_count=12, training_authorized=False, model_promoted=False, vehicle_control_approved=False)
    return result


def source_snapshot(root):
    previous.contract.canonical(root)
    result = {name:previous.contract.digest(root/name) for name in sorted(SOURCE_NAMES)}
    require(len(result) == 12 and all(result[name] == sha for name,sha in FROZEN_SOURCE_SHA.items()),
        "all TEN frozen sources plus exact new runner/loss required")
    return result


def verify_runtime_authorization(value, torch=None):
    identity = _runtime(value, torch)
    require(Path(previous.__file__).resolve() == SOURCE/Path(previous.__file__).name
        and Path(losses.previous.__file__).resolve() == SOURCE/Path(losses.previous.__file__).name,
        "old runner/auxiliary import escaped the new sealed flat package")
    return identity


def postcheck(source, manifest, authorization, authorization_sha):
    _postcheck(source, manifest, authorization, authorization_sha)
    verify_frozen()
    require(source_snapshot(SOURCE) == source, "pair-only source closure changed after evaluation")


@contextmanager
def activated():
    global _active
    require(not _active and not previous._active and previous.losses is _old_loss
        and previous.SOURCE is _old_source and previous.prepared_contract is _contract,
        "unknown or nested completed loss provider binding")
    verify_frozen()
    changes = dict(NAME=NAME, SCHEMA=SCHEMA, SCHEDULE_SCHEMA=SCHEDULE_SCHEMA, AUTHORIZATION_SCHEMA=AUTHORIZATION_SCHEMA,
        SOURCE=SOURCE, OUTPUT=OUTPUT, SOURCE_NAMES=SOURCE_NAMES, losses=losses,
        prepared_contract=prepared_contract, source_snapshot=source_snapshot,
        verify_runtime_authorization=verify_runtime_authorization, postcheck=postcheck)
    saved = {name:getattr(previous,name) for name in changes}; _active = True
    try:
        for name,value in changes.items(): setattr(previous,name,value)
        yield
    finally:
        for name,value in saved.items(): setattr(previous,name,value)
        _active = False


def main(argv=None):
    with activated(): return previous.main(argv)


if __name__ == "__main__": raise SystemExit(main())
