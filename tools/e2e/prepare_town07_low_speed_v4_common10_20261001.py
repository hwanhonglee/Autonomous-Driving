"""HH_261001 - Convert exactly one authenticated V4 cruise TRAIN episode with unchanged Common10 math."""

from contextlib import contextmanager
import importlib
from pathlib import Path

from scripts import e2e
from scripts.e2e import prepare_carla_common10_dataset as base
from scripts.e2e import qualify_town07_low_speed_v4_research_train_20261001 as authority


ADAPTER_ID = authority.ADAPTER_ID


@contextmanager
def bound_research_authority():
    # HH_261001 - Restore all serial process-local dispatch on success or exception; no frozen file is edited.
    previous = importlib.import_module('scripts.e2e.qualify_carla_moving_common10')
    old_attribute = getattr(e2e, 'qualify_carla_moving_common10', previous)
    old_writer = base._write_json

    def write_metadata(path, value):
        # HH_261001 - Correct only new converter identity/selection text, never native evidence or approval flags.
        if Path(path).name == 'collection_config.json':
            if value.get('adapter_id') != 'carla_fixed_moving_task_to_common10_v1':
                raise base.AdapterError('unexpected base moving adapter identity')
            value = dict(value, adapter_id=ADAPTER_ID,
                future_selection_policy='prospective_40_to_120m_all_327_anchors_no_quality_filter')
        elif Path(path).name == 'episode.json':
            provenance = value.get('source_provenance', {})
            if provenance.get('adapter_id') != 'carla_fixed_moving_task_to_common10_v1':
                raise base.AdapterError('unexpected base moving episode identity')
            value = dict(value, source_provenance=dict(provenance, adapter_id=ADAPTER_ID))
        return old_writer(path, value)

    e2e.qualify_carla_moving_common10 = authority
    base._write_json = write_metadata
    try:
        yield
    finally:
        base._write_json = old_writer
        e2e.qualify_carla_moving_common10 = old_attribute


def require_qualification(args):
    # HH_261001 - Refuse every unqualified fall-through to the generic adapter.
    if (getattr(args, 'moving_qualification', None) is None
            or getattr(args, 'moving_qualification_sha256', None) is None
            or getattr(args, 'execute_reviewed_research_conversion', False) is not True):
        raise base.AdapterError('paired receipt/SHA and explicit reviewed research conversion flag required')


def prepare_dataset(args):
    require_qualification(args)
    with bound_research_authority():
        return base.prepare_dataset(args)


def main(argv=None):
    # HH_261001 - Consume one explicit versioned research flag; leave the reviewed generic CLI unchanged.
    import sys
    values = list(sys.argv[1:] if argv is None else argv)
    flag = '--execute-reviewed-research-conversion'
    if values.count(flag) != 1:
        raise base.AdapterError('exactly one reviewed research conversion flag required')
    values.remove(flag)
    args = base._parse_args(values)
    args.execute_reviewed_research_conversion = True
    require_qualification(args)
    output, report = prepare_dataset(args)
    print(f"PASS research TRAIN dataset={report['dataset_id']} episodes={report['episode_count']} "
          f"samples={report['sample_count']} output={output}; vehicle_control_approved=false")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
