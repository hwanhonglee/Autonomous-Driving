#!/usr/bin/env python3
"""HH_260906 - Show actual fixed-window boundary cameras and centered measured traces, never synthetic driving."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from scripts.e2e import render_carla_raw_trial as renderer


def render(trial, diagnostic, output):
    diagnostic, output = Path(diagnostic), Path(output)
    if output.exists() or output.is_symlink() or output.resolve().is_relative_to(Path(trial).resolve()):
        raise ValueError('fresh boundary gallery outside raw data required')
    pin = renderer.digest(diagnostic)
    report = json.loads(diagnostic.read_text())
    if report['status'] != 'DIAGNOSED_NOT_ADMITTED' or not report['moving_scope_necessary_checks_clear']:
        raise ValueError('this fixed completed-window gallery requires a clear necessary-condition diagnostic')
    data = renderer.load_trial(trial)
    for name, expected in data['source_metadata_sha256'].items():
        recorded = next(row for row in report['source_manifest'] if row['path'] == data['episode'].name + '/' + name)
        if expected != recorded['sha256']:
            raise ValueError('raw source no longer matches moving diagnostic')
    scope = report['moving_scope']
    positions = {row['frame']: i for i, row in enumerate(data['cameras'])}
    first_tail = next(row for row in data['cameras'] if row['capture_phase'] == 'stationary_tail')
    supports = [row for row in data['cameras'] if row['frame'] <= scope['independently_observed_support']['frame']]
    selected = {'01_first_raw_camera': 0,
                '02_first_selected_moving_anchor': positions[scope['first_selected_camera']['frame']],
                '03_last_selected_moving_anchor': positions[scope['last_selected_camera']['frame']],
                '04_last_camera_before_cleanup_braking': positions[supports[-1]['frame']],
                '05_first_tail_camera': positions[first_tail['frame']],
                '06_last_raw_camera': len(data['cameras']) - 1}
    output.mkdir(parents=True)
    for name, index in selected.items():
        renderer.render_frame(data, index, 10, 10.).save(output / (name + '.png'))
    if renderer.digest(diagnostic) != pin or any(renderer.digest(data['episode'] / name) != expected
            for name, expected in {**data['source_metadata_sha256'], **data['displayed_image_sha256']}.items()):
        raise ValueError('raw/diagnostic changed during boundary rendering')
    provenance = dict(schema='carla.moving_boundary_gallery.v1', diagnostic_sha256=pin,
        renderer_sha256=renderer.digest(Path(__file__)), raw_renderer_sha256=renderer.digest(Path(renderer.__file__)),
        layout_sha256=renderer.digest(Path(renderer.view.__file__)), native_metadata_sha256=data['source_metadata_sha256'],
        displayed_image_sha256=data['displayed_image_sha256'], selections={name: dict(camera_index=index,
            frame=data['cameras'][index]['frame'], timestamp=data['cameras'][index]['timestamp']) for name, index in selected.items()},
        png_sha256={name + '.png': renderer.digest(output / (name + '.png')) for name in selected},
        learned_model_control=False, live_autoware_screenshot=False, dataset_admission_granted=False,
        notice='Recorded six-camera pixels, full FOV, ego-centered route and measured history/future. Future support is a native-state timestamp; the displayed preceding camera may be up to one camera interval earlier.')
    with (output / 'visual_provenance.json').open('x') as stream:
        json.dump(provenance, stream, indent=2)
        stream.write('\n')
    return dict(status='ACTUAL_BOUNDARY_GALLERY_NOT_ADMITTED', png_count=len(selected), output=str(output))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('trial', 'diagnostic', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    print(json.dumps(render(**vars(parser.parse_args(argv)))))


if __name__ == '__main__':
    main()
