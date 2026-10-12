"""HH_261002 - Fix only exact-baseline JSON/external-pin compatibility; preserve frozen preview08 physical semantics."""

from __future__ import annotations

import ast
from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
import re

from . import research_temporal_route_joint_preview08_signed45_town07_20261002 as previous

ROOT, base, parent, live = previous.ROOT, previous.base, previous.parent, previous.live
require, digest = previous.require, previous.digest
BASELINE, BASELINE_SHA = previous.BASELINE, previous.BASELINE_SHA
MODULE = "scripts.e2e.research_temporal_route_joint_preview08_signed45_v2_town07_20261002"
SCHEMA = "portable_e2e.actual_route_joint_thread4_preview08_signed45_three_arms.20261002.v2"
COMPATIBILITY_SCHEMA = "portable_e2e.preview08_exact_baseline_reader_compatibility.20261002.v2"
FROZEN_OWNER_SHA = "e2627b08eaf61f70e854ee1c9163a46adb6ab4869e5ca7eddfb15a6f40025090"
FAILURE = ROOT/"storage/experiments/validation/2026-10-02/temporal_route_joint_preview08_v1_failed_declaration_context.json"
FAILURE_SHA = "30b1aac21720b6b104f61f33497cbde853df9e27d322f527901caf756b1a67a3"
_sources, _active = previous.runtime_sources, False


def batch_paths(run_id):
    require(isinstance(run_id,str) and re.fullmatch(r"v[1-9][0-9]*",run_id), "fresh preview compatibility research version required")
    output=ROOT/("storage/experiments/validation/2026-10-02/temporal_route_joint_preview08_compat_v2_signed45_town07_"+run_id)
    return output,output.with_name(output.name+"_plan.json")


def exact_baseline_json(path):
    """HH_261002 - Permit32MiB only for this exact previously authenticated report; general readers keep4MiB."""
    path=Path(path)
    require(path == BASELINE and path.resolve() == path and not path.is_symlink() and digest(path) == BASELINE_SHA,
            "only the exact complete baseline report can use the new reader")
    require(path.stat().st_size <= 32*1024**2,"bounded exact baseline report required")
    return json.loads(path.read_text())


def external_pins():
    # HH_261002 - Allow only exact inherited external-source/asset names and hashes, never an arbitrary external root.
    result=base.external_sources() | {str(path):sha for path,sha in base.ASSETS.items() if not path.is_relative_to(ROOT)}
    require(all(Path(name).is_absolute() and Path(name).resolve() == Path(name) and not Path(name).is_symlink()
        and isinstance(sha,str) and re.fullmatch(r"[0-9a-f]{64}",sha) for name,sha in result.items()),
        "inherited exact external pin shape differs")
    return result


def transformed(name):
    """HH_261002 - Preserve every old statement except two exact baseline-predicate clauses or one plan-mapping keyword."""
    require(digest(Path(previous.__file__)) == FROZEN_OWNER_SHA,"frozen failed v1 preview owner changed")
    tree=ast.parse(Path(previous.__file__).read_bytes())
    original=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
    changed=copy.deepcopy(original);counts=dict(reader=0,external=0,census=0)

    class Narrow(ast.NodeTransformer):
        def visit_Call(self,node):
            node=self.generic_visit(node)
            if name=="baseline_proof" and isinstance(node.func,ast.Name) and node.func.id=="read_json" \
                    and ast.dump(node.args[0])==ast.dump(ast.Name(id="BASELINE",ctx=ast.Load())):
                node.func.id="exact_baseline_json";counts["reader"]+=1
            elif name=="baseline_proof" and isinstance(node.func,ast.Attribute) and node.func.attr=="is_relative_to" \
                    and ast.dump(node.func.value)==ast.dump(ast.Name(id="path",ctx=ast.Load())) \
                    and len(node.args)==1 and isinstance(node.args[0],ast.Name) and node.args[0].id=="ROOT":
                counts["external"]+=1
                return ast.BoolOp(op=ast.Or(),values=[node,ast.parse("_external.get(name) == sha",mode="eval").body])
            return node

        def visit_keyword(self,node):
            node=self.generic_visit(node)
            if name=="fixed_contract" and node.arg=="baseline_actual_output_source_sha256":
                node.arg="baseline_complete_source_output_census"
                node.value=ast.parse("source_census(_baseline)",mode="eval").body;counts["census"]+=1
            return node

    changed=Narrow().visit(changed)
    require(counts == (dict(reader=1,external=1,census=0) if name=="baseline_proof" else dict(reader=0,external=0,census=1)),
            "unexpected frozen baseline function transformation")
    restored=copy.deepcopy(changed)

    class Restore(ast.NodeTransformer):
        def visit_Call(self,node):
            node=self.generic_visit(node)
            if isinstance(node.func,ast.Name) and node.func.id=="exact_baseline_json":node.func.id="read_json"
            return node
        def visit_BoolOp(self,node):
            node=self.generic_visit(node)
            if isinstance(node.op,ast.Or) and len(node.values)==2 and ast.dump(node.values[1])==ast.dump(ast.parse("_external.get(name) == sha",mode="eval").body):
                return node.values[0]
            return node
        def visit_keyword(self,node):
            node=self.generic_visit(node)
            if node.arg=="baseline_complete_source_output_census":
                node.arg="baseline_actual_output_source_sha256";node.value=ast.parse("dict(_baseline['source_sha256'])",mode="eval").body
            return node

    require(ast.dump(Restore().visit(restored))==ast.dump(original),"frozen original baseline statements changed")
    namespace=dict(vars(previous),ROOT=ROOT,BASELINE=BASELINE,BASELINE_SHA=BASELINE_SHA,SCHEMA=SCHEMA,
        exact_baseline_json=exact_baseline_json,_external=external_pins() if name=="baseline_proof" else {},source_census=source_census)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),
        str(Path(__file__))+":exact_baseline_compatibility_only","exec"),namespace)
    return namespace[name]


def source_census(report):
    sources=report["source_sha256"]
    return dict(pin_count=len(sources),canonical_mapping_sha256=hashlib.sha256(json.dumps(sources,
        sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest(),
        exact_report_path=str(BASELINE),exact_report_sha256=BASELINE_SHA,
        all_original_pins_live_verified_before_and_after=True,full_mapping_archived_in_exact_report_asset=True,
        original_all_three_failures_retained=True,no_camera_source_pin_dropped=True)


def baseline_proof(pins):
    return transformed("baseline_proof")(pins)


def fixed_contract(run_id,longitudinal_pi):
    result=transformed("fixed_contract")(run_id,longitudinal_pi)
    result.update(compatibility_revision_schema=COMPATIBILITY_SCHEMA,frozen_v1_owner_source_sha256=FROZEN_OWNER_SHA,
        failed_v1_declaration_context_path=str(FAILURE),failed_v1_declaration_context_sha256=FAILURE_SHA,
        exact_baseline_only_JSON_limit_bytes=32*1024**2,general_JSON_reader_limit_unchanged_bytes=4*1024**2,
        collector_and_physical_preview_calibration_unchanged=True)
    return result


def runtime_sources():
    pins=_sources()
    for path in base.imported_source_closure((Path(__file__).resolve(),)):
        name,sha=str(path.relative_to(ROOT)),digest(path)
        require(name not in pins or pins[name]==sha,"compatibility import closure conflicts with frozen source")
        pins[name]=sha
    return pins


@contextmanager
def activated(pins):
    global _active
    require(not _active and not previous._active,"preview compatibility owner cannot nest")
    require(digest(FAILURE)==FAILURE_SHA,"frozen failed-declaration context changed")
    values=dict(batch_paths=batch_paths,MODULE=MODULE,SCHEMA=SCHEMA,baseline_proof=baseline_proof,
        fixed_contract=fixed_contract,runtime_sources=runtime_sources)
    saved={name:getattr(previous,name) for name in values};assets=base.ASSETS;_active=True
    try:
        for name,value in values.items():setattr(previous,name,value)
        base.ASSETS=dict(base.ASSETS)|{FAILURE:FAILURE_SHA}
        with previous.activated(pins):yield
    finally:
        for name,value in reversed(tuple(saved.items())):setattr(previous,name,value)
        base.ASSETS=assets;_active=False


def main(argv=None):
    import argparse
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("weights","metadata","checkpoint","declaration"):
        parser.add_argument("--expected-"+name+"-sha256",required=True)
    args,remainder=parser.parse_known_args(argv)
    pins={name+"_sha256":getattr(args,"expected_"+name+"_sha256") for name in ("weights","metadata","checkpoint","declaration")}
    with activated(pins):return parent.main(remainder)


if __name__=="__main__":
    raise SystemExit(main())
