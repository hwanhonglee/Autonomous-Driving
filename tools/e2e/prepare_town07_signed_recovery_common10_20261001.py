"""HH_261001 - Convert all qualified V4 rolling recovery cases with frozen Common10 math and strict denial guards."""

from __future__ import annotations

from contextlib import contextmanager
import importlib
from pathlib import Path

from scripts import e2e
from scripts.e2e import prepare_carla_common10_dataset as base
from scripts.e2e import qualify_town07_signed_recovery_common10_20261001 as authority


@contextmanager
def bound_recovery_authority():
    """HH_261001 - Dispatch only receipt authority and per-case membership; never bypass native Boolean denial checks."""
    previous=importlib.import_module("scripts.e2e.qualify_carla_moving_common10")
    attribute=getattr(e2e,"qualify_carla_moving_common10",previous)
    original_load,original_write=base._load_native,base._write_json

    def load_native(spec,*,moving_qualification=None):
        authority.require(moving_qualification is not None,"no unqualified recovery converter fallthrough")
        entry=authority.entry_for(spec,moving_qualification)
        # HH_261001 - The same full receipt is archived per episode, but each reader sees its own declared frame set.
        local=dict(moving_qualification,document=dict(moving_qualification["document"],
            selected_camera_frames=entry["selected_camera_frames"],route_identity=entry["route_identity"]))
        native=original_load(spec,moving_qualification=local)
        moving_qualification["native_objects"].append(native)
        return native

    def write_metadata(path,value):
        if Path(path).name=="collection_config.json":
            authority.require(value.get("adapter_id")=="carla_fixed_moving_task_to_common10_v1","unexpected original moving adapter")
            value=dict(value,adapter_id=authority.ADAPTER_ID,
                future_selection_policy="prospective_actual_native_frame80_through682_all302_rolling_anchors_no_quality_filter",
                **authority.ROLLING_SCOPE)
        elif Path(path).name=="episode.json":
            provenance=value.get("source_provenance",{})
            authority.require(provenance.get("adapter_id")=="carla_fixed_moving_task_to_common10_v1","unexpected original episode adapter")
            value=dict(value,source_provenance=dict(provenance,adapter_id=authority.ADAPTER_ID))
        return original_write(path,value)

    authority.require(base._load_native.__module__==base.__name__ and base._write_json.__module__==base.__name__,"unexpected converter provider already active")
    e2e.qualify_carla_moving_common10=authority
    base._load_native,base._write_json=load_native,write_metadata
    try:
        yield
    finally:
        base._load_native,base._write_json=original_load,original_write
        e2e.qualify_carla_moving_common10=attribute


def prepare_dataset(args):
    authority.require(getattr(args,"execute_reviewed_recovery_conversion",False) is True
        and getattr(args,"moving_qualification",None) is not None and getattr(args,"moving_qualification_sha256",None) is not None,
        "paired reviewed receipt/SHA and explicit recovery conversion flag required")
    with bound_recovery_authority():return base.prepare_dataset(args)


def main(argv=None):
    import sys
    values=list(sys.argv[1:] if argv is None else argv)
    flag="--execute-reviewed-recovery-conversion"
    authority.require(values.count(flag)==1,"exactly one reviewed recovery conversion flag required")
    values.remove(flag);args=base._parse_args(values);args.execute_reviewed_recovery_conversion=True
    output,report=prepare_dataset(args)
    print(f"PASS research-only actual recovery dataset={report['dataset_id']} episodes={report['episode_count']} samples={report['sample_count']} output={output}; vehicle_control_approved=false")
    return 0


if __name__=="__main__":raise SystemExit(main())
