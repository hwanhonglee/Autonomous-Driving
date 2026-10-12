"""HH_260906 - Prepare disjoint-route RGB/semantic pilot; depth never enters model data."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
# HH_260906 - Keep label names importable without loading Torch during preparation.
LABELS=("other","road","sidewalk","lane_marking","traffic_light_fixture","traffic_sign","pole")


def prepare(train,dev,output,*,image_size=(320,180)):
    # HH_260906 - Preserve the original pilot default; native resolution is a separate dataset.
    if image_size not in ((320,180),(640,360)):
        raise ValueError('only pilot or native capture resolution is supported')
    roots={"train":train,"dev":dev}
    records={s:json.loads((p/'records.json').read_text()) for s,p in roots.items()}
    # HH_260906 - Repeated stationary images must not inflate the tiny perception pilot.
    for split,rows in records.items():
        kept=[]
        for row in rows:
            if not kept or np.hypot(row['ego_x']-kept[-1]['ego_x'],row['ego_y']-kept[-1]['ego_y'])>=.25:
                kept.append(row)
        if len(kept)<24:raise ValueError('fewer than 24 spatially separated frames')
        records[split]=kept
    xy={s:np.array([[r['ego_x'],r['ego_y']] for r in rows]) for s,rows in records.items()}
    separation=float(np.sqrt(((xy['train'][:,None]-xy['dev'][None])**2).sum(-1)).min())
    if separation<80:raise ValueError('pilot routes must be at least 80m apart')
    manifest=dict(labels=LABELS,source_kind='CARLA_BasicAgent_oracle_semantic_labels',
                  model_inputs=['RGB'],depth_input=False,minimum_route_separation_m=separation,
                  split_policy='separate physical routes; >=0.25m frame spacing; same Town/weather, exploratory DEV only',splits={})
    if image_size != (320,180):
        manifest['image_size_wh']=list(image_size)
        manifest['new_physical_routes']=False
    output.mkdir(parents=True,exist_ok=False)
    for split,root in roots.items():
        summary=json.loads((root/'summary.json').read_text())
        if summary['status']!='COMPLETE_ORACLE_PLUMBING_ONLY' or not summary['physics_driving']:
            raise ValueError('completed physical collection required')
        if summary['collision_events'] or summary['travel_m']<3:raise ValueError('bad capture')
        xs=[];ys=[];source_hashes={}
        for record in records[split]:
            rgb=root/(record['stem']+'.png'); raw=root/(record['stem']+'.npz')
            for p in (rgb,raw):source_hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
            resampling=getattr(Image,'Resampling',Image)
            xs.append(np.asarray(Image.open(rgb).convert('RGB').resize(image_size,resampling.BILINEAR)))
            with np.load(raw,allow_pickle=False) as data:
                tags=np.asarray(Image.fromarray(data['semantic'][...,2]).resize(image_size,resampling.NEAREST))
            y=np.zeros(tags.shape,dtype=np.uint8)
            for tag,name in summary['label_table'].items():
                y[tags==int(tag)]=LABELS.index(name)
            ys.append(y)
        target=output/(split+'.npz')
        np.savez_compressed(target,rgb=np.stack(xs),target=np.stack(ys))
        manifest['splits'][split]=dict(file=target.name,sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
            count=len(xs),source_summary=summary,source_records=records[split],source_hashes=source_hashes,
            class_pixels=np.bincount(np.stack(ys).ravel(),minlength=len(LABELS)).tolist())
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps({s:{k:v for k,v in r.items() if k in ('count','class_pixels','sha256')} for s,r in manifest['splits'].items()}))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('train',type=Path);p.add_argument('dev',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--native-resolution',action='store_true')
    a=p.parse_args();prepare(a.train,a.dev,a.output,image_size=(640,360) if a.native_resolution else (320,180))
