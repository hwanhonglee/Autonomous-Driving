"""HH_260906 - Show worst footprint excess without disguising an offline audit as driving footage."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from shapely import wkt


def main():
    """HH_260906 - Render selected lane polygons and outside body area at the worst sampled pose."""
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();shapes=json.loads((a.run/'shapes.json').read_text())
    fig,axes=plt.subplots(2,2,figsize=(10,10))
    for ax,(token,data) in zip(axes.flat,shapes.items()):
        lane=wkt.loads(data['corridor_wkt']);body=wkt.loads(data['worst_body_wkt']);outside=body.difference(lane)
        for geom,color,alpha in [(lane,'lightgray',1),(body,'royalblue',.4),(outside,'red',1)]:
            parts=list(geom.geoms) if hasattr(geom,'geoms') else [geom]
            for part in parts:
                if part.is_empty or not hasattr(part,'exterior'):continue
                ax.fill(*part.exterior.xy,color=color,alpha=alpha)
                for hole in part.interiors:ax.fill(*hole.xy,color='white')
        x0,y0,x1,y1=body.bounds
        ax.set_xlim(x0-1,x1+1);ax.set_ylim(y0-1,y1+1);ax.set_aspect('equal')
        ax.set_title(('C_track ' if 'c_track' in token else 'Town01 ')+token.rsplit(':',1)[1])
    fig.suptitle('OFFLINE: gray = intended lanes; blue = body; red = outside\nWorst sampled pose; not actual driving')
    fig.tight_layout();a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=150)


if __name__=='__main__':main()
