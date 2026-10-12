"""HH_260906 - Show native Town01 DEV A/B/C panels while Town07 remains sealed."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT=Path(__file__).resolve().parents[2]
DATA=ROOT/'storage/experiments/training/2026-09-27/map_generalization_town03_train_package_v1'
A=ROOT/'docs/assets/validation/2026-09-26/dual_data_ai/results/mapping_ai_v2'
B=ROOT/'docs/assets/validation/2026-09-27/map_generalization_train_results/remote_run'
OUTPUT=ROOT/'docs/assets/validation/2026-09-27/map_town_rehearsal_c'
C=OUTPUT/'remote_run_v2'
COLORS=np.asarray(((30,30,40),(67,92,123),(141,88,147),(246,222,90),
                   (246,100,70),(60,220,180),(179,190,199)),dtype=np.uint8)


def require(condition,message):
    if not condition:raise ValueError(message)


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def load_predictions(folder,target):
    summary=json.loads((folder/'summary.json').read_text())
    with np.load(folder/'dev_predictions.npz',allow_pickle=False) as archive:
        prediction=archive['prediction']
    require(prediction.shape==target.shape==(118,360,640)
            and prediction.dtype==np.uint8 and int(prediction.max())<7,
            'native DEV prediction shape or labels differ')
    confusion=np.bincount((target.astype(np.int64)*7+prediction).ravel(),
                          minlength=49).reshape(7,7)
    require(confusion.tolist()==summary['confusion'],
            'stored prediction differs from model summary')
    return prediction


def render(index,rgb,target,predictions,font):
    """HH_260906 - Keep every RGB/label/prediction at the original 640x360 field of view."""
    canvas=Image.new('RGB',(1920,818),(16,23,33));draw=ImageDraw.Draw(canvas)
    draw.text((12,8),f'Town01 repeated DEV frame {index:03d} | same RGB, frozen A/B/C | NOT independent TEST',
              fill='white',font=font)
    panels=(('RGB input',rgb),('CARLA oracle label',COLORS[target]),
            ('A: AI v2 baseline',COLORS[predictions[0]]),
            ('B: uniform Town01+Town03',COLORS[predictions[1]]),
            ('C: 7:1 Town rehearsal',COLORS[predictions[2]]))
    for position,(label,values) in enumerate(panels):
        left=640*(position%3);top=39+388*(position//3)
        draw.text((left+8,top),label,fill='white',font=font)
        canvas.paste(Image.fromarray(values),(left,top+26))
    left=1280;top=427
    for offset,line in enumerate(('C improves on B, but fails the',
                                  'predeclared AI v2 DEV gate.',
                                  'Town07 stays sealed.',
                                  'No learned driving / HD-map approval.')):
        draw.text((left+18,top+offset*42),line,fill='white',font=font)
    return canvas


def main():
    manifest=json.loads((DATA/'manifest.json').read_text())
    require(digest(DATA/'dev.npz')==manifest['splits']['dev']['sha256']
            =='417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3',
            'frozen Town01 DEV bytes differ')
    with np.load(DATA/'dev.npz',allow_pickle=False) as archive:
        rgb,target=archive['rgb'],archive['target']
    require(rgb.shape==(118,360,640,3),'native DEV RGB differs')
    predictions=[load_predictions(folder,target) for folder in (A,B,C)]
    font_path=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font=ImageFont.truetype(str(font_path),18) if font_path.is_file() else ImageFont.load_default()
    chosen=(0,59,117);paths=[OUTPUT/f'town01_dev_{i:03d}_abc_full_fov.png' for i in chosen]
    gif=OUTPUT/'town01_dev_abc_full_fov.gif'
    require(all(not p.exists() for p in (*paths,gif)),
            'fresh visual paths required; do not overwrite evidence')
    frames=[]
    for i,path in zip(chosen,paths):
        frame=render(i,rgb[i],target[i],[prediction[i] for prediction in predictions],font)
        frame.save(path);frames.append(frame)
    frames[0].save(gif,save_all=True,append_images=frames[1:],duration=1300,loop=0)
    print(json.dumps({'split':'Town01 repeated DEV','indices':chosen,
                      'png':[str(p) for p in paths],'gif':str(gif),
                      'town07_inspected':False}))


if __name__=='__main__':main()
