#!/usr/bin/env python3
"""HH_261001 - Recover fixed-frame confidence and replay unchanged oracle-assisted map plumbing."""

import argparse
from datetime import datetime, timezone
from io import BytesIO
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from portable_e2e.mapping import SemanticEvidenceMap
from portable_e2e.mapping_projection import project_frame, project_predicted_frame
from scripts.e2e import evaluate_map_photometric_endpoints_20261001 as endpoint

ROOT, c = endpoint.ROOT, endpoint.contract
DOCS = ROOT/'docs/assets/validation/2026-10-01/map_photometric_projection'
PREREG, OUTPUT = DOCS/'preregistration.json', DOCS/'run_v1'
FIT_SHA = '27f2951c6a62da6c4ee76ccd1dae24350afaf85cc1b41824ae50dbeecada6c6d'
ENDPOINT_PREREG_SHA = '7019e192782a05eb4cfc2066a43c01ef02c1e8c32a5573341355a090e91e2711'
LABELS = endpoint.frozen.LABELS
TABLE = dict(enumerate(LABELS[1:],1))
RAW_TABLE = {1:'road',2:'sidewalk',24:'lane_marking',7:'traffic_light_fixture',8:'traffic_sign',6:'pole'}
INDICES, STRIDE, LIMIT = (0,119,239), 3, 10*1024*1024
COLORS = endpoint.frozen.COLORS


def encoded(value):
    return (json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def npz_bytes(**arrays):
    stream = BytesIO(); np.savez_compressed(stream,**arrays); return stream.getvalue()


def restore_confidence(sampled):
    c.require(sampled.shape == (120,214) and sampled.dtype == np.float32
        and np.isfinite(sampled).all() and np.all((sampled >= 0)&(sampled <= 1)), 'exact sampled confidence ABI differs')
    full = np.zeros((360,640),np.float32); full[::STRIDE,::STRIDE] = sampled
    return full


def metric_depth(raw):
    c.require(raw.shape == (360,640,4) and raw.dtype == np.uint8,'CARLA BGRA depth required')
    d = raw.astype(np.float64)
    return ((d[...,2]+d[...,1]*256+d[...,0]*65536)*(1000./16777215.)).astype(np.float32)


def pack_grid(grid):
    cells = sorted(grid.cells)
    order = np.full((len(cells),6),-1,np.int8)
    counts = np.zeros((len(cells),6),np.int16); sums = np.zeros((len(cells),6),np.float64)
    for row,cell in enumerate(cells):
        for position,(label,values) in enumerate(grid.cells[cell].items()):
            index = LABELS.index(label)-1; order[row,position] = index
            counts[row,index],sums[row,index] = values
    return dict(cell_xy=np.asarray(cells,dtype=np.int64).reshape(-1,2),label_order=order,
                observed_frames=counts,confidence_sum=sums)


def unpack_grid(arrays,meta):
    c.require(set(arrays) == {'cell_xy','label_order','observed_frames','confidence_sum'},'grid keys differ')
    cells,order,counts,sums = (arrays[key] for key in ('cell_xy','label_order','observed_frames','confidence_sum'))
    n = len(cells)
    c.require(cells.shape == (n,2) and cells.dtype == np.int64 and order.shape == counts.shape == sums.shape == (n,6)
        and order.dtype == np.int8 and counts.dtype == np.int16 and sums.dtype == np.float64
        and np.isfinite(sums).all() and np.all((order >= -1)&(order < 6))
        and np.all((counts >= 0)&(counts <= 3)) and np.all((sums >= 0)&(sums <= counts)), 'grid array ABI differs')
    grid = new_grid(meta['frame_id'],meta['source'])
    for cell,labels,number,weight in zip(cells,order,counts,sums):
        key = tuple(map(int,cell)); c.require(key not in grid.cells,'duplicate grid cell')
        entries = [int(index) for index in labels if index >= 0]
        c.require(len(entries) == len(set(entries)) and all(index < 6 for index in entries)
            and set(entries) == set(np.nonzero(number)[0]),'grid label order differs')
        grid.cells[key] = {LABELS[index+1]:[int(number[index]),float(weight[index])] for index in entries}
    grid.timestamp_s = meta['timestamp_s']
    c.require(grid.snapshot()['unobserved_is_unknown'] is True and not grid.snapshot()['hd_map_approved'],'unknown/approval boundary differs')
    return grid


def new_grid(frame_id,source):
    return SemanticEvidenceMap(frame_id,source,resolution_m=.2,max_cells=200000)


def point_receipt(observation):
    return dict(source=observation.source,points=len(observation.points),points_sha256=sha(encoded(observation.points)))


def comparison(oracle,learned):
    common,union = set(oracle.cells)&set(learned.cells),set(oracle.cells)|set(learned.cells)
    matrix = np.zeros((7,7),np.int64)
    for cell in union:
        labels = []
        for grid in (oracle,learned):
            evidence = grid.cells.get(cell)
            labels.append(0 if evidence is None else LABELS.index(max(evidence,key=lambda name:evidence[name][1])))
        matrix[tuple(labels)] += 1
    support,predicted = matrix.sum(1),matrix.sum(0)
    return dict(common_cells=len(common),union_cells=len(union),
        occupancy_jaccard=len(common)/len(union) if union else None,
        dominant_label_agreement_in_common_cells=float(np.trace(matrix[1:,1:])/len(common)) if common else None,
        confusion_unknown_then_six_classes=matrix.tolist(),unknown_is_not_free_space=True,
        classes={label:dict(oracle_cells=int(support[i]),learned_cells=int(predicted[i]),
            true_positive_cells=int(matrix[i,i]),false_positive_cells=int(predicted[i]-matrix[i,i]),
            iou=float(matrix[i,i]/(support[i]+predicted[i]-matrix[i,i])) if support[i] else None)
                 for i,label in TABLE.items()})


def render(scenario,index,record,maps):
    width = 640*len(maps)
    image = Image.new('RGB',(width,490),(18,27,40)); draw = ImageDraw.Draw(image)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    draw.text((8,5),f'{scenario} | only 000/119/239 accumulated | fixed {index:03d} | CARLA ORACLE depth/pose | NOT HD MAP',font=font,fill='white')
    yaw = math.radians(record['ego_yaw_deg']); cosine,sine = math.cos(yaw),math.sin(yaw)
    for column,(name,grid) in enumerate(maps.items()):
        offset = column*640
        draw.text((offset+8,32),name,font=font,fill='white')
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
    draw.text((8,458),'Dark = unknown; six static classes; 0.2m evidence cells; no ground-height filter or skipped-frame fill.',font=font,fill='white')
    data = BytesIO(); image.save(data,format='PNG'); return data.getvalue()


def bindings():
    registration = endpoint.validate_registration(); fit = endpoint.fit_binding(FIT_SHA)
    c.require(c.digest(endpoint.PREREG) == ENDPOINT_PREREG_SHA,'endpoint preregistration changed')
    scenarios = {}
    for scenario in endpoint.SCENARIOS:
        output = endpoint.output_path(scenario); summary = c.read_json(output/'summary.json'); audit = c.read_json(output/'prediction_audit.json')
        c.require(audit['status'] == 'PASS' and audit['summary_sha256'] == c.digest(output/'summary.json')
            and summary['fit_artifact_sha256'] == fit,'complete endpoint audit required')
        source,records,conditions,qa,distinction = endpoint.capture_binding(scenario,registration)
        original = c.read_json(source/'summary.json')
        intrinsics = original['sensor_intrinsics']
        c.require(intrinsics['horizontal_fov_deg'] == 90 and intrinsics['width'] == 640 and intrinsics['height'] == 360
            and intrinsics['lens_k'] == intrinsics['lens_kcube'] == 0
            and original['oracle_depth'] is True and original['oracle_pose'] is True,'fixed oracle geometry ABI differs')
        scenarios[scenario] = dict(source=str(source),conditions=list(conditions),town=original['town'],
            endpoint_summary_sha256=c.digest(output/'summary.json'),endpoint_audit_sha256=c.digest(output/'prediction_audit.json'),
            records_sha256=c.digest(source/'records.json'),source_summary_sha256=c.digest(source/'summary.json'),
            frames={str(i):dict(artifact_sha256=records[i]['artifact_sha256']['.npz'],timestamp_s=records[i]['timestamp_s']) for i in INDICES},
            predictions=summary['prediction_artifacts'],**distinction)
        for name,expected in summary['prediction_artifacts'].items():
            c.require(c.digest(output/name) == expected,'saved semantic mask changed')
    return dict(fit_artifacts=fit,scenarios=scenarios)


def sources():
    paths = [Path(__file__).resolve(),ROOT/'tools/e2e/verify_map_photometric_projection_20261001.py',
        ROOT/'learning/tests/test_map_photometric_projection_20261001.py',DOCS/'PLAN.md',DOCS/'PROPOSAL.md',
        ROOT/'learning/portable_e2e/mapping.py',ROOT/'learning/portable_e2e/mapping_projection.py']
    result = {str(path):c.digest(path) for path in paths}
    for name,expected in {'mapping.py':'264f9dda759f6986bde9c614e5cf3ae21680a22ae58d855db11564ceb3b3b867',
                         'mapping_projection.py':'c42ab7d20454475f544b8073d6a23a104a0a2bb1cc2cbb7be092fa3e871cbbee'}.items():
        c.require(result[str(ROOT/'learning/portable_e2e'/name)] == expected,'historical projection or accumulation source changed')
    return result


def policy():
    return dict(indices=list(INDICES),scenarios=list(endpoint.SCENARIOS),arms=list(endpoint.ARMS),forwards=36,
        stride=3,depth_strict_range_m=[.1,40],minimum_confidence=.5,resolution_m=.2,max_cells=200000,
        label_table=TABLE,other_excluded=True,confidence='maximum full-float32 softmax; exact saved argmax required',
        confidence_storage='exact stride3 samples only; off-lattice values are unqueried zeros',
        skipped_frames_per_capture=237,output_byte_cap=LIMIT,training_steps=0,gpu_used=False,simulator_started=False,
        oracle_depth=True,oracle_pose=True,oracle_semantics_reference_only=True,ground_height_filter=None,
        unknown_is_not_free_space=True,ctrack_train_eligible=False,model_promotion=False,hd_map_approved=False)


def preregister():
    c.require(not OUTPUT.exists(),'register before projection outputs')
    value = dict(schema='portable_e2e.map_photometric_projection_preregistration.v1',
        created_at_utc=datetime.now(timezone.utc).isoformat(),policy=policy(),bindings=bindings(),sources=sources(),
        confidence_recovery_authorized=True,confidence_forwards_performed=False,projection_outputs_created=False)
    endpoint.frozen.write_new(PREREG,value)
    return dict(status='PREREGISTERED',sha256=c.digest(PREREG))


def validate():
    value = c.read_json(PREREG)
    # HH_261001 - JSON keys are strings; canonical encoding normalizes the fixed label table.
    c.require(encoded(value['policy']) == encoded(policy()) and value['bindings'] == bindings()
        and value['sources'] == sources() and value['confidence_forwards_performed'] is False
        and value['projection_outputs_created'] is False,'frozen projection contract changed')
    return value


def saved_predictions(scenario,conditions):
    result = {}
    for condition in conditions:
        for arm in endpoint.ARMS:
            key = arm+'_'+condition
            with np.load(endpoint.output_path(scenario)/(key+'_all240_predictions.npz'),allow_pickle=False) as data:
                c.require(data.files == ['prediction'],'saved prediction archive differs')
                result[key] = data['prediction'][list(INDICES)].copy()
    return result


def run():
    registration = validate(); c.require(not OUTPUT.exists(),'fresh output required')
    torch,models,pools = endpoint.load_models(registration['bindings']['fit_artifacts'])
    blobs,scenarios,forwards = {},{},0
    for scenario,binding in registration['bindings']['scenarios'].items():
        source = Path(binding['source']); records = c.read_json(source/'records.json'); conditions = binding['conditions']
        predictions = saved_predictions(scenario,conditions); frame_id = 'carla_ue:'+binding['town']
        maps = {'oracle':new_grid(frame_id,'oracle_sensor')}
        maps.update({key:new_grid(frame_id,'learned_prediction') for key in predictions})
        rows = []
        for sample,index in enumerate(INDICES):
            record = records[index]
            with np.load(source/(record['stem']+'.npz'),allow_pickle=False) as raw:
                depth,semantic,pose = (raw[key] for key in ('depth','semantic','world_from_camera'))
                rgb = {condition:raw['rgb' if len(conditions)==1 else 'rgb_'+condition][...,[2,1,0]].copy() for condition in conditions}
            oracle = project_frame(depth,semantic,pose,frame_id=frame_id,timestamp_s=record['timestamp_s'],
                fov_deg=90,label_table=RAW_TABLE,stride=3,max_depth_m=40)
            maps['oracle'].update(oracle); reports = {'oracle':point_receipt(oracle)}
            for condition in conditions:
                x = torch.from_numpy(rgb[condition].transpose(2,0,1).copy()).float().unsqueeze(0)/255
                for arm,model in models.items():
                    key = arm+'_'+condition
                    with torch.no_grad():
                        logits = model(x); probabilities = logits.softmax(1); confidence,classes = probabilities.max(1)
                    predicted = predictions[key][sample]
                    c.require(np.array_equal(logits.argmax(1)[0].numpy(),predicted)
                        and np.array_equal(classes[0].numpy(),predicted),'recovered forward argmax differs from saved mask')
                    confidence = confidence[0].numpy().astype(np.float32)
                    sampled = confidence[::3,::3].copy(); restored = restore_confidence(sampled)
                    observation = project_predicted_frame(metric_depth(depth),predicted,confidence,pose,frame_id=frame_id,
                        timestamp_s=record['timestamp_s'],fov_deg=90,label_table=TABLE,stride=3,max_depth_m=40,min_confidence=.5)
                    replayed = project_predicted_frame(metric_depth(depth),predicted,restored,pose,frame_id=frame_id,
                        timestamp_s=record['timestamp_s'],fov_deg=90,label_table=TABLE,stride=3,max_depth_m=40,min_confidence=.5)
                    c.require(observation == replayed,'sampled confidence changed any projected point')
                    maps[key].update(observation); reports[key] = dict(**point_receipt(observation),exact_saved_argmax=True)
                    name = f'{scenario}_{index:03d}_{key}_confidence.npz'
                    blobs[name] = npz_bytes(confidence_stride3=sampled); forwards += 1
            previews = {name:grid for name,grid in maps.items()}
            png = f'{scenario}_{index:03d}_bev.png'; blobs[png] = render(scenario,index,record,previews)
            rows.append(dict(index=index,timestamp_s=record['timestamp_s'],projections=reports,
                accumulated_cells={key:len(grid.cells) for key,grid in maps.items()},
                comparisons={key:comparison(maps['oracle'],grid) for key,grid in maps.items() if key != 'oracle'},png=png))
        grid_meta = {}
        for key,grid in maps.items():
            name = f'{scenario}_{key}_grid.npz'; blobs[name] = npz_bytes(**pack_grid(grid))
            grid_meta[key] = dict(file=name,frame_id=grid.frame_id,source=grid.source,timestamp_s=grid.timestamp_s,
                snapshot_sha256=sha(encoded(grid.snapshot())),unobserved_is_unknown=True,hd_map_approved=False)
        scenarios[scenario] = dict(rows=rows,grids=grid_meta,role=binding['evaluation_role'],
            original_owner_status=binding['original_owner_status'],original_live_replay_exact_match=binding['original_live_replay_exact_match'],
            original_rgb_panels=str(endpoint.output_path(scenario)))
    c.require(forwards == 36,'all fixed confidence forwards required')
    validate()
    result = dict(schema='portable_e2e.map_photometric_projection.v1',status='FIXED_THREE_FRAME_ORACLE_ASSISTED_PLUMBING',
        created_at_utc=datetime.now(timezone.utc).isoformat(),preregistration_sha256=c.digest(PREREG),policy=policy(),
        bindings=registration['bindings'],scenarios=scenarios,confidence_forwards=forwards,native_threadpools=pools,
        original_evidence_unchanged=True,artifact_sha256={name:sha(raw) for name,raw in blobs.items()})
    blobs['summary.json'] = encoded(result)
    c.require(sum(map(len,blobs.values()))+65536 <= LIMIT,'complete outputs exceed10MiB; no adaptive omission allowed')
    OUTPUT.mkdir()
    for name,raw in blobs.items():
        with (OUTPUT/name).open('xb') as stream: stream.write(raw)
    return dict(status=result['status'],bytes_written=sum(map(len,blobs.values())),forwards=forwards,
                summary_sha256=c.digest(OUTPUT/'summary.json'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--preregister',action='store_true'); parser.add_argument('--execute',action='store_true')
    args = parser.parse_args()
    c.require(not(args.preregister and args.execute),'one mode required')
    print(json.dumps(preregister() if args.preregister else run() if args.execute else validate(),sort_keys=True))
