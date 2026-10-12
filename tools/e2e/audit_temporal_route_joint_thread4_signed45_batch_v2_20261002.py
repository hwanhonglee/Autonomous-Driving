"""HH_261002 - Authenticate the exact final6000 metadata path while preserving every duplicated benchmark proof hash."""

from __future__ import annotations

import ast
from contextlib import contextmanager
import copy
from pathlib import Path

from . import audit_temporal_route_joint_thread4_signed45_batch_20261002 as previous

V1_SHA="3259164c8b53599f3c7ac9d45c316b455211113ccf8e5bd1275f2545133f8751"
REVISION_SCHEMA="portable_e2e.actual_thread4_archive_exact_endpoint_metadata_selector.20261002.v2"
EXACT_METADATA=previous.previous.RUN/"exports/step_006000/metadata.json"
_reader,_build=previous.transformed_trial_reader,previous.build_report
_active=False


def exact_metadata_reader():
    """HH_261002 - Recompile the frozen generator with one additional exact-path predicate, not a skipped asset hash."""
    tree=ast.parse(Path(previous.__file__).read_bytes())
    function=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=="transformed_trial_reader")
    original=copy.deepcopy(function)
    anchors=[i for i,node in enumerate(function.body) if isinstance(node,ast.Assign)
        and any(isinstance(target,ast.Name) and target.id=="namespace" for target in node.targets)]
    previous.require(len(anchors)==1,"exact frozen reader namespace anchor required")
    inserted=ast.parse("""metadata_selectors=[node for node in ast.walk(function) if isinstance(node,ast.If) and ast.dump(node.test,include_attributes=False)==ast.dump(ast.parse('section==\"asset_sha256\" and source.name==\"metadata.json\" and value==endpoint[\"metadata\"]',mode='eval').body,include_attributes=False)]
require(len(metadata_selectors)==1,'unique exact original endpoint selector required; all asset loops stay unchanged')
selector=metadata_selectors[0]
selector.test=ast.BoolOp(op=ast.And(),values=[selector.test,ast.Compare(left=ast.Name(id='source',ctx=ast.Load()),ops=[ast.Eq()],comparators=[ast.Call(func=ast.Name(id='Path',ctx=ast.Load()),args=[ast.Constant(value=str(EXACT_METADATA))],keywords=[])])])
""").body
    function.body[anchors[0]:anchors[0]]=inserted
    parity=copy.deepcopy(function);del parity.body[anchors[0]:anchors[0]+len(inserted)]
    previous.require(ast.dump(parity,include_attributes=False)==ast.dump(original,include_attributes=False),
        "all frozen generator/source/archive/control/gate statements remain exact")
    namespace=dict(vars(previous),EXACT_METADATA=EXACT_METADATA)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),
        str(Path(__file__))+":unchanged_reader_plus_exact_final6000_metadata_path","exec"),namespace)
    return namespace["transformed_trial_reader"]()


@contextmanager
def activated():
    global _active
    previous.require(not _active and not previous._active and previous.transformed_trial_reader is _reader,
        "non-nested unchanged thread4v1 reader binding required")
    previous.require(previous.digest(Path(previous.__file__))==V1_SHA,"frozen rejected v1 auditor source changed")
    _active=True;previous.transformed_trial_reader=exact_metadata_reader
    try:yield
    finally:previous.transformed_trial_reader=_reader;_active=False


def build_report(*args):
    with activated():report=_build(*args)
    for path in previous.source_closure(Path(__file__),previous.ROOT/"tools/workspace_python.py",previous.ROOT/"tools/canonical_workspace.py"):
        previous.add_pin(report["source_sha256"],path)
    failure_history=previous.ROOT/"storage/experiments/validation/2026-10-02/temporal_route_joint_thread4_archive_audit_failed_invocations_v1.json"
    previous.add_pin(report["source_sha256"],failure_history)
    report.update(audit_revision_schema=REVISION_SCHEMA,exact_endpoint_metadata_path=str(EXACT_METADATA),
        all_benchmark_metadata_and_other_asset_copies_hashed=True,original_v1_audit_source_sha256=V1_SHA,
        frozen_v1_failed_before_output_creation=True,metadata_selection_is_not_check_relaxation=True,
        retained_failed_audit_invocation_history=str(failure_history))
    previous.frozen.assert_pins(report["source_sha256"]);return report


def main(argv=None):
    # HH_261002 - Keep the original caller schema/CLI and create-only publication; replace only its report builder temporarily.
    old=previous.build_report
    previous.require(old is _build,"unknown v1 report builder binding")
    previous.build_report=build_report
    try:return previous.main(argv)
    finally:previous.build_report=old


if __name__=="__main__":raise SystemExit(main())
