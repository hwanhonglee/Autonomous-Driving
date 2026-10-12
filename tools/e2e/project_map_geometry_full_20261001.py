#!/usr/bin/env python3
"""HH_261001 - Full-recording oracle-assisted evidence accumulation; never an HD-map claim."""

import argparse
from datetime import datetime, timezone
from io import BytesIO
import json
import math
from pathlib import Path
import shutil
import signal
import time
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scripts.e2e import evaluate_map_geometry_endpoints_20261001 as endpoint
from scripts.e2e import project_map_photometric_fixed_20261001 as previous

ROOT, c = endpoint.ROOT, endpoint.contract
DOCS = ROOT/'docs/assets/validation/2026-10-01/map_geometry_full_projection'
PREREG, OUTPUT = DOCS/'preregistration.json', DOCS/'run_v1'
FIT_SHA = 'caa1a60bcff8179b80de0a5861aa2ab0497d838638fbf80a3f4790e8074e11ef'
ENDPOINT_PREREG_SHA = 'b91586fbbf905b085b861a73db127d4b5c22dbb905d20170d78790a9646b857a'
SCENARIOS, ARMS, INDICES = ('ctrack_left','ctrack_straight'), ('control','geometry'), (0,119,239)
FRAMES, LIMIT, FAILURE_RESERVE, FLOOR, DEADLINE = 240, 128*1024**2, 2*1024**2, 5*1024**3, 1800
FONT = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
LABELS, TABLE, RAW_TABLE, COLORS = previous.LABELS, previous.TABLE, previous.RAW_TABLE, previous.COLORS
encoded, sha, npz_bytes = previous.encoded, previous.sha, previous.npz_bytes
new_grid, pack_grid, restore_confidence = previous.new_grid, previous.pack_grid, previous.restore_confidence
metric_depth, point_receipt, comparison = previous.metric_depth, previous.point_receipt, previous.comparison
project_frame, project_predicted_frame = previous.project_frame, previous.project_predicted_frame


def policy():
    return dict(scenarios=list(SCENARIOS),arms=list(ARMS),frames_per_route=FRAMES,
        frame_indices=list(range(FRAMES)),fixed_png_indices=list(INDICES),confidence_forwards=960,
        oracle_frames=480,learned_map_updates=960,final_grids=6,pngs=8,skipped_frames=0,
        chronological_updates=True,stride=3,depth_strict_range_m=[.1,40],minimum_confidence=.5,
        resolution_m=.2,max_cells=200000,ground_height_filter=None,label_table=TABLE,other_excluded=True,
        confidence_storage='exact float32 stride3 120x214; off-lattice values never queried',
        confidence_raw_bytes=960*120*214*4,exact_saved_argmax_required=True,
        oracle_depth=True,oracle_pose=True,oracle_semantics_reference_only=True,
        learned_projection_accepts_no_semantic_target=True,unknown_is_not_free_space=True,
        fixed_png_viewport='640x390 at 6 pixels/metre, ego centered; older/offscreen cells clipped explicitly',
        overview='all stored cell footprints and all240 poses; shared route bounds/scale, no selected frames',
        maximum_output_bytes=LIMIT,failure_reserve_bytes=FAILURE_RESERVE,free_disk_floor_bytes=FLOOR,
        execute_deadline_seconds=DEADLINE,independent_replay_deadline_seconds=DEADLINE,
        failure_policy='retain every written artifact and partial failure; no resume, frame dropping or cap change',
        thread_count=2,openblas_threads=2,gpu_used=False,training_steps=0,simulator_started=False,
        ctrack_train_eligible=False,model_promotion=False,independent_validation=False,hd_map_approved=False,
        online_means='chronological recorded-frame updates, not a new live simulator run',
        left_caveat='original owner FAIL preserved; forensic raw QA, original live replay unavailable')


def unpack_grid(arrays,meta):
    # HH_261001 - Only the serialization count bound changes from three frames to the fixed full240.
    c.require(set(arrays) == {'cell_xy','label_order','observed_frames','confidence_sum'},'grid keys differ')
    cells,order,counts,sums = (arrays[key] for key in ('cell_xy','label_order','observed_frames','confidence_sum'))
    n = len(cells)
    c.require(n <= 200000 and cells.shape == (n,2) and cells.dtype == np.int64
        and order.shape == counts.shape == sums.shape == (n,6) and order.dtype == np.int8
        and counts.dtype == np.int16 and sums.dtype == np.float64 and np.isfinite(sums).all()
        and np.all((order >= -1)&(order < 6)) and np.all((counts >= 0)&(counts <= FRAMES))
        and np.all((sums >= 0)&(sums <= counts)),'full240 grid ABI differs')
    grid = new_grid(meta['frame_id'],meta['source'])
    last = None
    for cell,labels,number,weight in zip(cells,order,counts,sums):
        key = tuple(map(int,cell)); entries = [int(i) for i in labels if i >= 0]
        c.require((last is None or key > last) and len(entries) == len(set(entries))
            and set(entries) == set(np.nonzero(number)[0])
            and np.all(weight[number == 0] == 0),'grid order, counts or unused sums differ')
        grid.cells[key] = {LABELS[i+1]:[int(number[i]),float(weight[i])] for i in entries}
        last = key
    c.require(math.isfinite(meta['timestamp_s']),'finite final timestamp required')
    grid.timestamp_s = meta['timestamp_s']; return grid


def grid_meta(grid,file):
    return dict(file=file,frame_id=grid.frame_id,source=grid.source,timestamp_s=grid.timestamp_s,
        cells=len(grid.cells),snapshot_sha256=sha(encoded(grid.snapshot())),
        unobserved_is_unknown=True,hd_map_approved=False)


def assert_argmax(logit_argmax,probability_argmax,saved):
    c.require(saved.shape == (360,640) and saved.dtype == np.uint8 and int(saved.max()) < 7
        and np.array_equal(logit_argmax,saved) and np.array_equal(probability_argmax,saved),
        'exact saved full-frame argmax differs; no tolerance or skipped frame permitted')


def load_predictions(scenario):
    result = {}
    for arm in ARMS:
        path = endpoint.output_path(scenario)/(arm+'_true_all240_predictions.npz')
        with np.load(path,allow_pickle=False) as data:
            c.require(data.files == ['prediction'],'prediction archive keys differ')
            result[arm] = data['prediction'].copy()
        c.require(result[arm].shape == (FRAMES,360,640) and result[arm].dtype == np.uint8
            and int(result[arm].max()) < 7,'native full240 masks required')
    return result


def bindings():
    registration = endpoint.validate_registration(); fit = endpoint.fit_binding(FIT_SHA)
    c.require(c.digest(endpoint.PREREG) == ENDPOINT_PREREG_SHA,'endpoint registration changed')
    scenarios = {}
    for scenario in SCENARIOS:
        output = endpoint.output_path(scenario)
        summary,audit = (c.read_json(output/name) for name in ('summary.json','prediction_audit.json'))
        c.require(summary['status'] == 'DIAGNOSTIC_COMPLETE_NOT_PROMOTED' and audit['status'] == 'PASS'
            and audit['summary_sha256'] == c.digest(output/'summary.json')
            and summary['fit_artifact_sha256'] == fit and summary['frames_per_model_condition'] == FRAMES,
            'complete independently recounted endpoint required')
        source,records,conditions,qa,distinction = endpoint.capture_binding(scenario,registration)
        original = c.read_json(source/'summary.json'); camera = original['sensor_intrinsics']
        c.require(len(records) == FRAMES and tuple(conditions) == ('true',) and camera['horizontal_fov_deg'] == 90
            and (camera['width'],camera['height']) == (640,360) and camera['lens_k'] == camera['lens_kcube'] == 0
            and original['oracle_depth'] is True and original['oracle_pose'] is True,'fixed oracle geometry ABI differs')
        stamps = [row['timestamp_s'] for row in records]
        c.require(all(math.isfinite(t) for t in stamps) and all(a < b for a,b in zip(stamps,stamps[1:])),
            'strict chronological full sequence required')
        for name,expected in summary['prediction_artifacts'].items():
            c.require(c.digest(output/name) == expected,'saved prediction changed')
        scenarios[scenario] = dict(source=str(source),town=original['town'],
            records_sha256=c.digest(source/'records.json'),source_summary_sha256=c.digest(source/'summary.json'),
            endpoint_summary_sha256=c.digest(output/'summary.json'),endpoint_audit_sha256=c.digest(output/'prediction_audit.json'),
            predictions=summary['prediction_artifacts'],raw_receipt=registration['raw_receipts'][scenario],
            frames=[dict(index=i,stem=row['stem'],timestamp_s=row['timestamp_s'],
                sensor_frame=row['frame'],artifact_sha256=row['artifact_sha256']) for i,row in enumerate(records)],**distinction)
    return dict(fit_artifacts=fit,scenarios=scenarios)


def sources():
    result = endpoint.sources()
    paths = [Path(__file__).resolve(),ROOT/'tools/e2e/verify_map_geometry_full_projection_20261001.py',
        ROOT/'learning/tests/test_map_geometry_full_projection_20261001.py',
        ROOT/'learning/tests/test_map_geometry_full_projection_verifier_20261001.py',
        DOCS/'PLAN.md',Path(previous.__file__).resolve(),FONT,
        ROOT/'learning/portable_e2e/mapping.py',ROOT/'learning/portable_e2e/mapping_projection.py']
    result.update({str(path):c.digest(path) for path in paths})
    for name,expected in {'mapping.py':'264f9dda759f6986bde9c614e5cf3ae21680a22ae58d855db11564ceb3b3b867',
                         'mapping_projection.py':'c42ab7d20454475f544b8073d6a23a104a0a2bb1cc2cbb7be092fa3e871cbbee'}.items():
        c.require(result[str(ROOT/'learning/portable_e2e'/name)] == expected,'unchanged historical projection/fusion required')
    return result


def preregister():
    c.require(not OUTPUT.exists() and not PREREG.exists(),'registration before new inference, fresh paths only')
    value = dict(schema='portable_e2e.map_geometry_full_projection_preregistration.v1',
        created_at_utc=datetime.now(timezone.utc).isoformat(),policy=policy(),bindings=bindings(),sources=sources(),
        prior_image_results_already_known=True,new_confidence_forwards_performed=False,
        full_recording_projection_outputs_created=False,parent_review_required_before_registration=True)
    endpoint.frozen.write_new(PREREG,value)
    return dict(status='REGISTERED_BEFORE_NEW_CONFIDENCE_AND_ACCUMULATION',sha256=c.digest(PREREG))


def validate():
    value = c.read_json(PREREG)
    c.require(value['schema'] == 'portable_e2e.map_geometry_full_projection_preregistration.v1'
        and encoded(value['policy']) == encoded(policy()) and value['sources'] == sources()
        and value['bindings'] == bindings() and value['new_confidence_forwards_performed'] is False
        and value['full_recording_projection_outputs_created'] is False,'frozen full-recording study changed')
    return value


def used_bytes():
    if not OUTPUT.exists(): return 0
    paths = list(OUTPUT.iterdir())
    c.require(not OUTPUT.is_symlink() and all(p.is_file() and not p.is_symlink() for p in paths),
        'flat nonsymlink evidence directory required')
    return sum(p.stat().st_size for p in paths)


def write_artifact(name,payload,failure=False):
    c.require(Path(name).name == name and name not in ('','.','..'),'simple output basename required')
    c.require(used_bytes()+len(payload) <= LIMIT-(0 if failure else FAILURE_RESERVE),'128MiB aggregate cap; retain failure')
    c.require(shutil.disk_usage(OUTPUT.parent).free-len(payload) >= FLOOR,'five-GiB free disk floor')
    with (OUTPUT/name).open('xb') as stream: stream.write(payload)
    return c.digest(OUTPUT/name)


def _timeout(signum,frame):
    raise TimeoutError('fixed 1800-second wall budget exceeded; no partial PASS')


def counted_forward(model,tensor,progress):
    # HH_261001 - Count a returned model call before any validation or artifact write can fail.
    progress['attempted_forwards'] += 1; progress['current']['stage'] = 'model_call_attempted'
    logits = model(tensor)
    progress['completed_forwards'] += 1; progress['current']['stage'] = 'model_call_returned'
    return logits


def render_fixed(scenario,index,record,maps):
    canvas = Image.new('RGB',(1920,490),(18,27,40)); draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype(str(FONT),16)
    draw.text((8,5),f'{scenario} | chronological frames 0..{index:03d} | CARLA ORACLE depth/pose | NOT HD MAP',font=font,fill='white')
    yaw = math.radians(record['ego_yaw_deg']); cosine,sine = math.cos(yaw),math.sin(yaw)
    for column,(name,grid) in enumerate(maps.items()):
        offset = column*640
        draw.text((offset+8,32),name+(' semantics' if name == 'oracle' else ' learned segmentation'),font=font,fill='white')
        draw.rectangle((offset,60,offset+639,449),fill=(22,31,43))
        for (ix,iy),labels in grid.cells.items():
            label = max(labels,key=lambda key:labels[key][1]); color = tuple(map(int,COLORS[LABELS.index(label)]))
            dx,dy = (ix+.5)*.2-record['ego_x'],(iy+.5)*.2-record['ego_y']
            forward,right = cosine*dx+sine*dy,-sine*dx+cosine*dy
            u,v = offset+320+right*6,255-forward*6
            if offset+2 <= u < offset+638 and 80 <= v < 446:
                draw.rectangle((u-1,v-1,u+1,v+1),fill=color)
        draw.rectangle((offset+314,243,offset+326,267),fill=(255,110,65),outline='white')
        draw.polygon([(offset+320,231),(offset+314,241),(offset+326,241)],fill='white')
    draw.text((8,458),'Dark = unknown. Fixed ego viewport clips older/off-screen cells; full-route overview and exact final grids are separate. No height filter.',font=font,fill='white')
    stream = BytesIO(); canvas.save(stream,format='PNG'); return stream.getvalue()


def overview_bounds(records,maps):
    c.require(len(records) == FRAMES,'overview retains the entire fixed route')
    xs,ys = [r['ego_x'] for r in records],[r['ego_y'] for r in records]
    for grid in maps.values():
        for ix,iy in grid.cells:
            xs.extend((ix*.2,(ix+1)*.2)); ys.extend((iy*.2,(iy+1)*.2))
    c.require(all(math.isfinite(v) for v in xs+ys),'finite complete overview bounds required')
    return [min(xs)-1,min(ys)-1,max(xs)+1,max(ys)+1]


def render_overview(scenario,records,maps):
    bounds = overview_bounds(records,maps); xmin,ymin,xmax,ymax = bounds
    scale = min(600/(xmax-xmin),560/(ymax-ymin))
    canvas = Image.new('RGB',(1920,710),(18,27,40)); draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype(str(FONT),16)
    draw.text((8,6),f'{scenario} | ALL 240 frames and stored cells | same bounds/scale | ORACLE depth/pose | NOT HD MAP',font=font,fill='white')
    centerx,centery = (xmin+xmax)/2,(ymin+ymax)/2
    for column,(name,grid) in enumerate(maps.items()):
        offset = 640*column
        draw.text((offset+8,32),name+(' semantics' if name == 'oracle' else ' learned segmentation'),font=font,fill='white')
        draw.rectangle((offset,60,offset+639,639),fill=(22,31,43))
        def pixel(x,y): return offset+320+(x-centerx)*scale,350-(y-centery)*scale
        for (ix,iy),labels in grid.cells.items():
            label = max(labels,key=lambda key:labels[key][1]); color = tuple(map(int,COLORS[LABELS.index(label)]))
            left,bottom = pixel(ix*.2,iy*.2); right,top = pixel((ix+1)*.2,(iy+1)*.2)
            draw.rectangle((left,top,right,bottom),fill=color)
        path = [pixel(r['ego_x'],r['ego_y']) for r in records]
        draw.line(path,fill=(255,255,255),width=2)
        u,v = path[0]; draw.ellipse((u-3,v-3,u+3,v+3),outline=(50,255,50),width=2)
        u,v = path[-1]; draw.ellipse((u-4,v-4,u+4,v+4),fill=(255,110,65),outline='white')
        yaw = math.radians(records[-1]['ego_yaw_deg'])
        draw.line((u,v,u+12*math.cos(yaw),v-12*math.sin(yaw)),fill=(255,110,65),width=3)
    draw.text((8,650),'Dark = unknown; white = all240 oracle ego poses; green = start; orange = final pose. Six static labels, no height filter.',font=font,fill='white')
    draw.text((8,678),f'World x/y bounds (m): {bounds}; scale={scale:.6f}px/m. Full stored cells are included; raster overlaps are not omissions.',font=font,fill='white')
    stream = BytesIO(); canvas.save(stream,format='PNG')
    return stream.getvalue(),dict(bounds_xy_m=bounds,pixels_per_m=scale,pose_count=FRAMES,all_stored_cell_extents_included=True)


def run():
    c.require(not OUTPUT.exists(),'fresh output required; failed attempts are never resumed or overwritten')
    started_at = time.monotonic(); previous_handler = signal.signal(signal.SIGALRM,_timeout)
    signal.setitimer(signal.ITIMER_REAL,DEADLINE)
    completed,artifacts,scenarios = [],{},{}
    progress = dict(attempted_forwards=0,completed_forwards=0,oracle_map_updates=0,learned_map_updates=0,current=None)
    try:
        registration = validate()
        c.require(shutil.disk_usage(DOCS).free >= FLOOR+LIMIT,'reserve full128MiB above unchanged five-GiB floor')
        OUTPUT.mkdir()
        artifacts['started.json'] = write_artifact('started.json',encoded(dict(
            schema='portable_e2e.map_geometry_full_projection_started.v1',policy=policy(),
            preregistration_sha256=c.digest(PREREG),bindings=registration['bindings'],sources=registration['sources'],
            created_at_utc=datetime.now(timezone.utc).isoformat())))
        progress['current'] = dict(scenario=None,index=None,arm=None,stage='load_frozen_cpu_models',confidence_persisted=False)
        torch,models,pools = endpoint.load_models(registration['bindings']['fit_artifacts'])
        for scenario in SCENARIOS:
            binding = registration['bindings']['scenarios'][scenario]
            source = Path(binding['source']); records = c.read_json(source/'records.json')
            predictions = load_predictions(scenario); frame_id = 'carla_ue:'+binding['town']
            maps = {'oracle':new_grid(frame_id,'oracle_sensor')}
            maps.update({arm:new_grid(frame_id,'learned_prediction') for arm in ARMS})
            rows,pngs = [],[]
            for index,record in enumerate(records):
                progress['current'] = dict(scenario=scenario,index=index,arm='oracle',stage='raw_frame_read',confidence_persisted=False)
                raw_path = source/(record['stem']+'.npz')
                c.require(c.digest(raw_path) == binding['frames'][index]['artifact_sha256']['.npz'],'raw frame changed')
                with np.load(raw_path,allow_pickle=False) as raw:
                    depth,semantic,pose = (raw[key] for key in ('depth','semantic','world_from_camera'))
                    rgb = raw['rgb'][...,[2,1,0]].copy()
                c.require(rgb.shape == (360,640,3) and rgb.dtype == np.uint8,'native RGB required')
                oracle = project_frame(depth,semantic,pose,frame_id=frame_id,timestamp_s=record['timestamp_s'],
                    fov_deg=90,label_table=RAW_TABLE,stride=3,max_depth_m=40)
                progress['current']['stage'] = 'oracle_map_update'
                maps['oracle'].update(oracle); progress['oracle_map_updates'] += 1
                reports = {'oracle':point_receipt(oracle)}
                tensor = torch.from_numpy(rgb.transpose(2,0,1).copy()).float().unsqueeze(0)/255
                for arm in ARMS:
                    progress['current'] = dict(scenario=scenario,index=index,arm=arm,stage='before_model_call',confidence_persisted=False)
                    start = time.perf_counter()
                    with torch.no_grad():
                        logits = counted_forward(models[arm],tensor,progress)
                        progress['current'].update(logits_shape=list(logits.shape),logits_dtype=str(logits.dtype))
                        c.require(bool(torch.isfinite(logits).all()),'finite model logits required')
                        confidence,classes = logits.softmax(1).max(1)
                    logit_argmax,probability_argmax = logits.argmax(1)[0].numpy(),classes[0].numpy()
                    saved = predictions[arm][index]
                    progress['current'].update(stage='full_frame_argmax_check',argmax=dict(
                        shape=list(logit_argmax.shape),dtype=str(logit_argmax.dtype),
                        logit_sha256=sha(logit_argmax.tobytes()),probability_sha256=sha(probability_argmax.tobytes()),
                        saved_dtype=str(saved.dtype),saved_sha256=sha(saved.tobytes()),
                        logit_equal_saved=bool(np.array_equal(logit_argmax,saved)),
                        probability_equal_saved=bool(np.array_equal(probability_argmax,saved))))
                    assert_argmax(logit_argmax,probability_argmax,saved)
                    full = confidence[0].numpy().astype(np.float32)
                    sampled = full[::3,::3].copy(); restored = restore_confidence(sampled)
                    name = f'{scenario}_{index:03d}_{arm}_confidence.npz'
                    progress['current'].update(stage='confidence_write',confidence_file=name,
                        confidence_shape=list(sampled.shape),confidence_dtype=str(sampled.dtype),
                        confidence_numeric_sha256=sha(sampled.tobytes()))
                    artifacts[name] = write_artifact(name,npz_bytes(confidence_stride3=sampled))
                    progress['current'].update(stage='project_full_and_sampled_confidence',confidence_persisted=True)
                    kwargs = dict(frame_id=frame_id,timestamp_s=record['timestamp_s'],fov_deg=90,
                        label_table=TABLE,stride=3,max_depth_m=40,min_confidence=.5)
                    observation = project_predicted_frame(metric_depth(depth),predictions[arm][index],full,pose,**kwargs)
                    replayed = project_predicted_frame(metric_depth(depth),predictions[arm][index],restored,pose,**kwargs)
                    c.require(observation == replayed,'stride3 confidence storage changed projected points')
                    progress['current']['stage'] = 'learned_map_update'
                    maps[arm].update(observation); progress['learned_map_updates'] += 1
                    reports[arm] = dict(**point_receipt(observation),exact_saved_argmax=True,
                        confidence_file=name,confidence_numeric_sha256=sha(sampled.tobytes()),
                        forward_and_projection_ms=(time.perf_counter()-start)*1000)
                row = dict(index=index,sensor_frame=record['frame'],timestamp_s=record['timestamp_s'],
                    projections=reports,accumulated_cells={key:len(grid.cells) for key,grid in maps.items()},
                    accumulated_snapshot_sha256={key:sha(encoded(grid.snapshot())) for key,grid in maps.items()})
                if index in INDICES:
                    name = f'{scenario}_{index:03d}_bev.png'
                    artifacts[name] = write_artifact(name,render_fixed(scenario,index,record,maps)); pngs.append(name)
                    row['png'] = name
                    row['comparisons'] = {arm:comparison(maps['oracle'],maps[arm]) for arm in ARMS}
                name = f'{scenario}_{index:03d}_frame.json'
                progress['current']['stage'] = 'frame_receipt_write'
                artifacts[name] = write_artifact(name,encoded(row)); rows.append(name)
                completed.append([scenario,index])
                progress['current']['stage'] = 'frame_receipt_persisted'
                if (index+1)%40 == 0: print(json.dumps(dict(scenario=scenario,completed=index+1,
                    forwards=progress['completed_forwards'])),flush=True)
            grids = {}
            for key,grid in maps.items():
                name = f'{scenario}_{key}_grid.npz'
                progress['current'].update(stage='final_grid_write',arm=key,grid_file=name)
                artifacts[name] = write_artifact(name,npz_bytes(**pack_grid(grid)))
                grids[key] = grid_meta(grid,name)
            name = f'{scenario}_full_route_bev.png'
            progress['current'].update(stage='full_route_overview_write',arm=None)
            png,overview = render_overview(scenario,records,maps)
            artifacts[name] = write_artifact(name,png); pngs.append(name)
            scenarios[scenario] = dict(rows=rows,grids=grids,pngs=pngs,overview=overview,
                comparisons={arm:comparison(maps['oracle'],maps[arm]) for arm in ARMS},
                evaluation_role=binding['evaluation_role'],source_qa_status=binding['source_qa_status'],
                original_owner_status=binding['original_owner_status'],
                original_live_replay_exact_match=binding['original_live_replay_exact_match'])
        c.require(progress['attempted_forwards'] == progress['completed_forwards'] == progress['learned_map_updates'] == 960
            and progress['oracle_map_updates'] == len(completed) == 480,'full fixed denominator required')
        progress['current'].update(stage='final_source_input_revalidation',arm=None)
        validate()
        summary = dict(schema='portable_e2e.map_geometry_full_projection.v1',
            status='FULL_RECORDING_ORACLE_ASSISTED_ACCUMULATION_DIAGNOSTIC',policy=policy(),
            preregistration_sha256=c.digest(PREREG),bindings=registration['bindings'],sources=registration['sources'],
            scenarios=scenarios,confidence_forwards=progress['completed_forwards'],attempted_forwards=progress['attempted_forwards'],
            oracle_frames=progress['oracle_map_updates'],learned_map_updates=progress['learned_map_updates'],
            completed_frame_count=len(completed),native_threadpools=pools,torch_version=torch.__version__,
            source_and_inputs_unchanged=True,elapsed_seconds=time.monotonic()-started_at,
            artifact_sha256=artifacts,online_offline_equality='PENDING_SEPARATE_NO_FORWARD_REPLAY',
            evaluation_is_not_live=True)
        progress['current']['stage'] = 'final_summary_write'
        write_artifact('summary.json',encoded(summary))
        return dict(status=summary['status'],summary_sha256=c.digest(OUTPUT/'summary.json'),bytes_written=used_bytes())
    except BaseException as error:
        signal.setitimer(signal.ITIMER_REAL,0)
        failure = dict(schema='portable_e2e.map_geometry_full_projection_failure.v1',status='FAILED_PRESERVED_NO_PARTIAL_PASS',
            error=repr(error),confidence_forwards=progress['completed_forwards'],attempted_forwards=progress['attempted_forwards'],
            completed_map_updates=dict(oracle=progress['oracle_map_updates'],learned=progress['learned_map_updates']),
            current_frame_context=progress['current'],completed_frames=completed,completed_scenarios=list(scenarios),
            known_artifact_sha256=artifacts,partial_artifacts_retained=True,no_resume=True,frames_not_dropped=True,
            source_and_inputs_unchanged='NOT_CLAIMED_ON_FAILURE',elapsed_seconds=time.monotonic()-started_at)
        if OUTPUT.exists():
            try: write_artifact('failure.json',encoded(failure),failure=True)
            except BaseException: print(json.dumps(failure),flush=True)
        else: print(json.dumps(failure),flush=True)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL,0); signal.signal(signal.SIGALRM,previous_handler)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--preregister',action='store_true'); modes.add_argument('--execute',action='store_true')
    args = parser.parse_args()
    print(json.dumps(preregister() if args.preregister else run() if args.execute else validate(),sort_keys=True),flush=True)
