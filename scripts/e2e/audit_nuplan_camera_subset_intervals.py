#!/usr/bin/env python3
"""HH_260906 - Bound all six-camera subsets from saved native timestamp intervals without selecting a rig or reading DB payloads."""
import argparse
from itertools import combinations
import hashlib
import json
from pathlib import Path

CHANNELS = ('CAM_B0', 'CAM_F0', 'CAM_L0', 'CAM_L1', 'CAM_L2', 'CAM_R0', 'CAM_R1', 'CAM_R2')
REPORT_SHA256 = '0b9a2a3de73c5e96af869307873a235c8279b4eaeaf6f328950973c3c97ef83e'

def require(condition, message):
    if not condition:
        raise ValueError(message)

def interval_lower_bound(intervals):
    # HH_260906 - Every selected observation lies in its interval, so max(lower)-min(upper) is a conservative span bound.
    require(len(intervals) == 6, 'exactly six channels required')
    require(all(len(pair) == 2 and all(type(v) is int for v in pair) and pair[0] <= pair[1] for pair in intervals),
            'interval endpoints must be ordered integer microseconds')
    return max(0, max(pair[0] for pair in intervals) - min(pair[1] for pair in intervals))

def analyze(report):
    require(report.get('schema') == 'nuplan.single_db_metadata_probe.v1' and report.get('status') == 'METADATA_INSPECTED_NOT_READY',
            'complete original metadata evidence required')
    alignment = report['metadata']['camera_alignment']
    require(alignment['reference_channel'] == 'CAM_F0' and alignment['reference_anchor_count'] == 3760,
            'fixed front-reference denominator required')
    offsets = alignment['per_channel_signed_offset_us']
    require(set(offsets) == set(CHANNELS), 'all eight native channels required')
    intervals = {}
    for channel in CHANNELS:
        values = offsets[channel]['offset']
        require(values['count'] == 3760 and type(values['minimum']) is int and type(values['maximum']) is int,
                'complete native integer intervals required')
        require(values['minimum'] <= values['maximum'], 'unordered interval')
        intervals[channel] = [values['minimum'], values['maximum']]
    require(intervals['CAM_F0'] == [0, 0], 'front-reference offset must be exactly zero')
    subsets = [{'channels': list(subset), 'span_lower_bound_us': interval_lower_bound([intervals[c] for c in subset]),
                'includes_front_reference_camera': 'CAM_F0' in subset} for subset in combinations(CHANNELS, 6)]
    require(len(subsets) == 28, 'subset enumeration incomplete')
    best = min(row['span_lower_bound_us'] for row in subsets)
    front_best = min(row['span_lower_bound_us'] for row in subsets if row['includes_front_reference_camera'])
    return {'schema': 'nuplan.six_camera_native_interval_bound.v1', 'source_report_sha256': REPORT_SHA256,
        'status': 'SAVED_METADATA_BOUND_ONLY', 'threshold_us': 20000, 'reference_anchor_count': 3760,
        'channel_intervals_us': intervals, 'subset_count': 28, 'subsets': subsets,
        'minimum_six_subset_lower_bound_us': best, 'minimum_front_including_subset_lower_bound_us': front_best,
        'all_nearest_front_six_subsets_exceed_20ms': all(row['span_lower_bound_us'] > 20000 for row in subsets),
        'rig_selected': False, 'training_data_approved': False, 'timestamp_modified': False,
        'new_db_or_camera_payload_read': False,
        'scope': 'Only the original nearest-CAM_F0 eight-channel associations. Interval extrema form a conservative lower bound, not exact per-anchor subset skews or a sensor-rig selection.',
        'limitation': 'This does not evaluate motion compensation, alternative timestamp-aware model contracts, calibration/FOV suitability or causal image availability.'}

def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('report', type=Path)
    args = parser.parse_args()
    require(args.report.is_file() and not args.report.is_symlink(), 'regular saved report required')
    raw = args.report.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == REPORT_SHA256, 'original report SHA mismatch')
    result = analyze(json.loads(raw))
    require(args.report.read_bytes() == raw, 'saved report changed during analysis')
    print(json.dumps(result, indent=2, sort_keys=True))

if __name__ == '__main__':
    main()
