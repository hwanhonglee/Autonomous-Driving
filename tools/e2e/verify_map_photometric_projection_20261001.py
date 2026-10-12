#!/usr/bin/env python3
"""HH_261001 - Replay every fixed BEV point and grid from saved masks/confidence without inference."""

from io import BytesIO
import json
from pathlib import Path
import numpy as np
from PIL import Image
from scripts.e2e import project_map_photometric_fixed_20261001 as p


def verify():
    reg = p.validate(); summary = p.c.read_json(p.OUTPUT/'summary.json')
    p.c.require(summary['status'] == 'FIXED_THREE_FRAME_ORACLE_ASSISTED_PLUMBING'
        and summary['preregistration_sha256'] == p.c.digest(p.PREREG) and summary['bindings'] == reg['bindings']
        and p.encoded(summary['policy']) == p.encoded(p.policy()) and summary['confidence_forwards'] == 36
        and summary['original_evidence_unchanged'] is True,'projection source/boundary differs')
    for name,expected in summary['artifact_sha256'].items():
        p.c.require(Path(name).name == name and p.c.digest(p.OUTPUT/name) == expected,'projection artifact differs')
    checked,grids_checked,png_checked = 0,0,0
    for scenario,binding in reg['bindings']['scenarios'].items():
        source = Path(binding['source']); records = p.c.read_json(source/'records.json')
        predictions = p.saved_predictions(scenario,binding['conditions'])
        frame_id = 'carla_ue:'+binding['town']; maps = {'oracle':p.new_grid(frame_id,'oracle_sensor')}
        maps.update({key:p.new_grid(frame_id,'learned_prediction') for key in predictions})
        published = summary['scenarios'][scenario]
        p.c.require(published['role'] == binding['evaluation_role']
            and published['original_owner_status'] == binding['original_owner_status']
            and published['original_live_replay_exact_match'] == binding['original_live_replay_exact_match'],'role/LEFT forensic distinction changed')
        for sample,index in enumerate(p.INDICES):
            record = records[index]; row = published['rows'][sample]
            p.c.require(row['index'] == index and row['timestamp_s'] == record['timestamp_s'],'fixed frame sequence changed')
            raw_path = source/(record['stem']+'.npz')
            p.c.require(p.c.digest(raw_path) == binding['frames'][str(index)]['artifact_sha256'],'raw geometry changed')
            with np.load(raw_path,allow_pickle=False) as raw:
                depth,semantic,pose = (raw[key] for key in ('depth','semantic','world_from_camera'))
            observations = {'oracle':p.project_frame(depth,semantic,pose,frame_id=frame_id,timestamp_s=record['timestamp_s'],
                fov_deg=90,label_table=p.RAW_TABLE,stride=3,max_depth_m=40)}
            for key in predictions:
                path = p.OUTPUT/f'{scenario}_{index:03d}_{key}_confidence.npz'
                with np.load(path,allow_pickle=False) as data:
                    p.c.require(data.files == ['confidence_stride3'],'confidence payload keys differ')
                    confidence = p.restore_confidence(data['confidence_stride3'])
                observations[key] = p.project_predicted_frame(p.metric_depth(depth),predictions[key][sample],confidence,pose,
                    frame_id=frame_id,timestamp_s=record['timestamp_s'],fov_deg=90,label_table=p.TABLE,
                    stride=3,max_depth_m=40,min_confidence=.5)
                p.c.require(row['projections'][key]['exact_saved_argmax'] is True,'saved mask match absent')
                checked += 1
            for key,observation in observations.items():
                expected = dict(row['projections'][key]); expected.pop('exact_saved_argmax',None)
                p.c.require(p.point_receipt(observation) == expected,'independent projected point/source hash differs')
                maps[key].update(observation)
            p.c.require({key:len(grid.cells) for key,grid in maps.items()} == row['accumulated_cells'],'partial accumulated counts differ')
            comparisons = {key:p.comparison(maps['oracle'],grid) for key,grid in maps.items() if key != 'oracle'}
            p.c.require(comparisons == row['comparisons'],'fixed cell diagnostics differ')
            path = p.OUTPUT/row['png']; actual = np.asarray(Image.open(path))
            repeated = np.asarray(Image.open(BytesIO(p.render(scenario,index,record,maps))))
            p.c.require(np.array_equal(actual,repeated),'all rendered BEV pixels differ')
            for column in range(len(maps)):
                p.c.require(tuple(actual[255,column*640+320]) == (255,110,65),'ego not centered')
                p.c.require(tuple(actual[61,column*640+1]) == (22,31,43),'unknown background differs')
            png_checked += 1
        for key,grid in maps.items():
            meta = published['grids'][key]
            with np.load(p.OUTPUT/meta['file'],allow_pickle=False) as archive:
                arrays = {name:archive[name].copy() for name in archive.files}
            recovered = p.unpack_grid(arrays,meta)
            p.c.require(recovered.snapshot() == grid.snapshot() and list(recovered.cells) == sorted(grid.cells)
                and all(list(recovered.cells[cell]) == list(grid.cells[cell]) for cell in grid.cells),
                'grid confidence sums, frame counts or dominant-label tie order differ')
            p.c.require(p.sha(p.encoded(grid.snapshot())) == meta['snapshot_sha256'],'grid snapshot hash differs')
            grids_checked += 1
    p.c.require((checked,grids_checked,png_checked) == (36,16,12),'complete36/16/12 replay required')
    p.validate()
    before_bytes = sum(path.stat().st_size for path in p.OUTPUT.iterdir())
    result = dict(schema='portable_e2e.map_photometric_projection_replay.v1',status='PASS',
        projected_learned_frames=checked,final_grids_replayed=grids_checked,all_png_pixels_replayed=png_checked,
        summary_sha256=p.c.digest(p.OUTPUT/'summary.json'),bytes_before_replay_receipt=before_bytes,
        independent_replay_inference_count=0,training_steps=0,gpu_used=False,simulator_started=False,
        source_and_input_hashes_unchanged=True,unknown_remains_unknown=True,oracle_geometry=True,
        skipped_frames_not_accumulated=237,ctrack_train_eligible=False,model_promotion=False,hd_map_approved=False)
    p.c.require(before_bytes+len(p.encoded(result)) <= p.LIMIT,'complete output cap exceeded')
    with (p.OUTPUT/'independent_replay.json').open('xb') as stream: stream.write(p.encoded(result))
    return result


if __name__ == '__main__':
    print(json.dumps(verify(),sort_keys=True))
