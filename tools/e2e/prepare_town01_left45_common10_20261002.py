"""HH_261002 - Scoped receipt provider for one LEFT347 source; every original Common10 formula/denial guard stays unchanged."""
from __future__ import annotations

from contextlib import contextmanager
import importlib
from pathlib import Path

from scripts import e2e
from scripts.e2e import prepare_carla_common10_dataset as base
from scripts.e2e import qualify_town01_left45_common10_20261002 as authority

_active=False


@contextmanager
def bound_left_authority():
    global _active
    original_provider=importlib.import_module("scripts.e2e.qualify_carla_moving_common10")
    attribute=getattr(e2e,"qualify_carla_moving_common10",original_provider)
    load,write=base._load_native,base._write_json
    authority.require(not _active and attribute is original_provider and load.__module__==base.__name__
        and write.__module__==base.__name__,"non-nested exact frozen converter providers required")
    def load_native(spec,*,moving_qualification=None):
        authority.require(moving_qualification is not None,"no unqualified LEFT converter fallthrough")
        authority.entry_for(spec,moving_qualification)
        native=load(spec,moving_qualification=moving_qualification)
        moving_qualification["native_objects"].append(native);return native
    def write_metadata(path,value):
        if Path(path).name=="collection_config.json":
            authority.require(value.get("adapter_id")=="carla_fixed_moving_task_to_common10_v1","unexpected original adapter")
            value=dict(value,adapter_id=authority.ADAPTER_ID,future_selection_policy="prospective_actual_native_frames80_through772_all347_LEFT_anchors_no_quality_filter",**authority.SCOPE)
        elif Path(path).name=="episode.json":
            provenance=value.get("source_provenance",{})
            authority.require(provenance.get("adapter_id")=="carla_fixed_moving_task_to_common10_v1","unexpected original episode adapter")
            value=dict(value,source_provenance=dict(provenance,adapter_id=authority.ADAPTER_ID))
        return write(path,value)
    _active=True
    try:
        e2e.qualify_carla_moving_common10=authority;base._load_native,base._write_json=load_native,write_metadata
        yield
    finally:
        e2e.qualify_carla_moving_common10=attribute;base._load_native,base._write_json=load,write;_active=False


def prepare_dataset(args):
    authority.require(getattr(args,"execute_reviewed_left45_conversion",False) is True
        and getattr(args,"moving_qualification",None) is not None and getattr(args,"moving_qualification_sha256",None) is not None,
        "reviewed receipt/SHA and explicit LEFT347 conversion flag required")
    with bound_left_authority():return base.prepare_dataset(args)


def main(argv=None):
    import sys
    values=list(sys.argv[1:] if argv is None else argv);flag="--execute-reviewed-left45-conversion"
    authority.require(values.count(flag)==1,"exactly one reviewed LEFT conversion flag required")
    values.remove(flag);args=base._parse_args(values);args.execute_reviewed_left45_conversion=True
    output,report=prepare_dataset(args)
    print(f"PASS research-only LEFT347 dataset={report['dataset_id']} episodes={report['episode_count']} samples={report['sample_count']} output={output}; vehicle_control_approved=false")
    return 0


if __name__=="__main__":raise SystemExit(main())
