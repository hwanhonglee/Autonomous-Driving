"""HH_260906 - Render fixed Town01 DEV predictions for map A/B without opening Town07."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT=Path(__file__).resolve().parents[2]
DATA=ROOT/'storage/experiments/training/2026-09-27/map_generalization_town03_train_package_v1'
RESULT=ROOT/'docs/assets/validation/2026-09-27/map_generalization_train_results'
RUN=RESULT/'remote_run'
BASELINE=ROOT/'docs/assets/validation/2026-09-26/dual_data_ai/results/mapping_ai_v2'
COLORS=np.asarray(((30,30,40),(67,92,123),(141,88,147),(246,222,90),
                   (246,100,70),(60,220,180),(179,190,199)),dtype=np.uint8)


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def require(condition,message):
    if not condition:raise ValueError(message)


def render(index,rgb,target,parent,candidate,font):
    """HH_260906 - Keep the native 640x360 field of view in every panel."""
    canvas=Image.new('RGB',(1280,818),(16,23,33));draw=ImageDraw.Draw(canvas)
    draw.text((12,8),f'Town01 repeated DEV frame {index:03d} | fixed A/B input | NOT independent TEST',
              fill='white',font=font)
    for position,(title,values) in enumerate((
            ('RGB input',rgb),('CARLA oracle label',COLORS[target]),
            ('AI v2 baseline',COLORS[parent]),('Town03-data candidate',COLORS[candidate]))):
        left=640*(position%2);top=39+388*(position//2)
        draw.text((left+8,top),title,fill='white',font=font)
        canvas.paste(Image.fromarray(values),(left,top+26))
    return canvas


def main():
    """HH_260906 - Pin both stored predictions and DEV bytes before making a visual aid."""
    manifest=json.loads((DATA/'manifest.json').read_text())
    candidate=json.loads((RUN/'summary.json').read_text())
    baseline=json.loads((BASELINE/'summary.json').read_text())
    require(digest(DATA/'dev.npz')==manifest['splits']['dev']['sha256']
            =='417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3',
            'frozen Town01 DEV bytes differ')
    require(candidate['parent_native']['confusion']==baseline['confusion']
            and candidate['status']=='PILOT_COMPLETE_NOT_PROMOTED'
            and candidate['deployment_approved'] is False,
            'A/B result contract differs')
    with np.load(DATA/'dev.npz',allow_pickle=False) as archive:
        rgb,target=archive['rgb'],archive['target']
    with np.load(BASELINE/'dev_predictions.npz',allow_pickle=False) as archive:
        parent=archive['prediction']
    with np.load(RUN/'dev_predictions.npz',allow_pickle=False) as archive:
        new=archive['prediction']
    require(rgb.shape==(118,360,640,3)
            and target.shape==parent.shape==new.shape==(118,360,640),
            'native DEV prediction shapes differ')
    for label,prediction,expected in (('A',parent,baseline['confusion']),
                                      ('B',new,candidate['confusion'])):
        confusion=np.bincount((target.astype(np.int64)*7+prediction).ravel(),
                              minlength=49).reshape(7,7)
        require(confusion.tolist()==expected,f'{label} stored prediction confusion differs')
    font_path=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font=ImageFont.truetype(str(font_path),17) if font_path.is_file() else ImageFont.load_default()
    chosen=(0,59,117);paths=[RESULT/f'town01_dev_{index:03d}_full_fov.png' for index in chosen]
    gif_path=RESULT/'town01_dev_ab_full_fov.gif'
    require(all(not path.exists() for path in (*paths,gif_path)),
            'fresh visual paths required; do not overwrite evidence')
    frames=[]
    for index,path in zip(chosen,paths):
        image=render(index,rgb[index],target[index],parent[index],new[index],font)
        image.save(path)
        frames.append(image)
    frames[0].save(gif_path,save_all=True,append_images=frames[1:],duration=1300,loop=0)
    print(json.dumps({'town':'Town01','split':'repeated DEV','indices':chosen,
                      'png':[str(path) for path in paths],'gif':str(gif_path),
                      'town07_inspected':False}))


if __name__=='__main__':main()
