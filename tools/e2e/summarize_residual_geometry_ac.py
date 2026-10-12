"""HH_260906 - Bound the residual audit; do not use it as a universal training gate."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from shapely import wkt
from shapely.geometry import Polygon
from shapely.ops import unary_union


def characterize(corridor, body):
    """HH_260906 - Describe residual shape without claiming that a thin excess is automatically safe."""
    outside=body.difference(corridor)
    if outside.is_empty:return dict(outside_area_m2=0.,short_side_m=0.,long_side_m=0.,hole_area_m2=0.)
    rect=np.asarray(outside.minimum_rotated_rectangle.exterior.coords)
    lengths=np.linalg.norm(np.diff(rect,axis=0),axis=1)
    parts=list(corridor.geoms) if hasattr(corridor,'geoms') else [corridor]
    filled=unary_union([Polygon(part.exterior) for part in parts])
    return dict(outside_area_m2=outside.area,short_side_m=float(lengths.min()),long_side_m=float(lengths.max()),
                hole_area_m2=outside.intersection(filled).area,
                note='Rectangle dimensions characterize the residual, not vehicle penetration depth.')


def main():
    """HH_260906 - Record only worst sampled poses; preserve the unresolved strict audit status."""
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();rows=[]
    for token,shape in json.loads(a.input.read_text()).items():
        rows.append(dict(token=token,**characterize(wkt.loads(shape['corridor_wkt']),wkt.loads(shape['worst_body_wkt'])),
                         strict_containment_approved=False,scope='worst previously sampled pose only'))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as stream:json.dump(dict(rows=rows,input_sha256=hashlib.sha256(a.input.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),stream,indent=2)
    print(json.dumps(rows))


if __name__=='__main__':main()
