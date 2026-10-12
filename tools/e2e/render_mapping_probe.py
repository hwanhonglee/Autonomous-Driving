"""HH_260906 - Render measured oracle sensor maps with ego-centered, labeled panels."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from portable_e2e.mapping import SemanticEvidenceMap
from portable_e2e.mapping_projection import project_frame

COLORS = {"road": (67, 92, 123), "sidewalk": (141, 88, 147), "lane_marking": (246, 222, 90),
          "traffic_light_fixture": (246, 100, 70), "traffic_sign": (60, 220, 180), "pole": (179, 190, 199)}
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def font(size):
    return ImageFont.truetype(FONT_PATH, size)


def render(source, output):
    summary = json.loads((source / "summary.json").read_text())
    if summary["status"] != "COMPLETE_ORACLE_PLUMBING_ONLY":
        raise ValueError("completed owned probe required")
    output.mkdir(parents=True, exist_ok=False)
    records = json.loads((source / "records.json").read_text())
    reference = json.loads((source / "logical_map_reference.json").read_text())
    label_table = {int(k): v for k, v in summary["label_table"].items()}
    accumulated = SemanticEvidenceMap("carla_ue:Town01", "oracle_sensor", max_cells=200000)
    pictures, distances, counts = [], [], Counter()
    # HH_260906 - Reference centerline corridors are a diagnostic proxy, not exact rendered surface GT.
    from shapely.geometry import Polygon, Point
    from shapely.ops import unary_union
    from shapely.prepared import prep
    polygons = []
    for w in reference:
        angle = math.radians(w["yaw"])
        forward = np.array([math.cos(angle), math.sin(angle)])
        side = np.array([-forward[1], forward[0]]) * w["width"] / 2
        center = np.array([w["x"], w["y"]])
        polygons.append(Polygon([center-forward*.65-side, center+forward*.65-side,
                                 center+forward*.65+side, center-forward*.65+side]))
    corridor = unary_union(polygons)
    prepared = prep(corridor)
    out_of_proxy, checked = 0, 0
    for index, record in enumerate(records):
        with np.load(source / (record["stem"] + ".npz"), allow_pickle=False) as data:
            obs = project_frame(data["depth"], data["semantic"], data["world_from_camera"],
                                frame_id=accumulated.frame_id, timestamp_s=record["timestamp_s"],
                                fov_deg=90, label_table=label_table)
            semantic = data["semantic"][..., 2]
        accumulated.update(obs)
        counts.update(p[2] for p in obs.points)
        # HH_260906 - This is point exposure, not independent scenes or map accuracy.
        for x, y, label, _ in obs.points[::30]:
            if label == "road":
                point = Point(x, y)
                checked += 1
                if not prepared.covers(point):
                    out_of_proxy += 1
                    distances.append(corridor.distance(point))
                else:
                    distances.append(0.)
        frame_stride=max(1,len(records)//48)
        if index%frame_stride and index not in {len(records)//2,len(records)-1}:
            continue
        canvas = Image.new("RGB", (1280, 1000), (16, 23, 33))
        draw = ImageDraw.Draw(canvas)
        draw.text((24, 12), "TOWN01 | ORACLE SENSOR MAPPING | NOT AUTONOMOUS DRIVING", font=font(23), fill="white")
        motion="BasicAgent physics drive" if summary.get('physics_driving') else "Kinematic scan"
        draw.text((24, 45), f"{motion} {index+1}/{len(records)} | exact depth/labels/pose | no learned perception", font=font(17), fill="#bbc6d4")
        rgb = Image.open(source / (record["stem"] + ".png")).convert("RGB")
        canvas.paste(rgb, (0, 105))
        palette = np.zeros((*semantic.shape, 3), dtype=np.uint8)
        for tag, label in label_table.items():
            palette[semantic == tag] = COLORS[label]
        canvas.paste(Image.fromarray(palette), (640, 105))
        draw.text((20, 80), "RGB: front camera", font=font(18), fill="white")
        draw.text((660, 80), "CARLA semantic truth (not model predictions)", font=font(18), fill="white")
        yaw = math.radians(record["ego_yaw_deg"])
        c, s = math.cos(yaw), math.sin(yaw)
        def pixel(x, y, offset):
            dx, dy = x-record["ego_x"], y-record["ego_y"]
            forward, right = c*dx+s*dy, -s*dx+c*dy
            return offset+320+right*6, 730-forward*6
        # HH_260906 - Both BEV panels center ego; +forward is up and +right is right.
        for offset, points, title in (
            (0, [(x,y,label) for x,y,label,_ in obs.points], "CURRENT OBSERVATION | ego centered"),
            (640, [( (ix+.5)*.2, (iy+.5)*.2, max(labels, key=lambda k: labels[k][1]))
                   for (ix,iy), labels in accumulated.cells.items()], "ACCUMULATED MAP | ego centered")):
            draw.rectangle((offset, 495, offset+639, 935), fill=(23, 31, 43))
            for x, y, label in points:
                u, v = pixel(x, y, offset)
                if offset+2 <= u < offset+638 and 515 <= v < 932:
                    draw.rectangle((u-1,v-1,u+1,v+1), fill=COLORS[label])
            for w in reference:
                u,v = pixel(w["x"], w["y"], offset)
                if offset+2 <= u < offset+638 and 515 <= v < 932:
                    draw.point((u,v), fill=(230,230,235))
            for r in records[:index+1]:
                u,v = pixel(r["ego_x"], r["ego_y"], offset)
                if offset+2 <= u < offset+638 and 515 <= v < 932:
                    draw.ellipse((u-2,v-2,u+2,v+2), fill=(40,220,230))
            draw.rectangle((offset+314,716,offset+326,744), fill=(255,110,65), outline="white")
            draw.polygon([(offset+320,706),(offset+315,714),(offset+325,714)],fill="white")
            draw.text((offset+15,472), title, font=font(18), fill="white")
            draw.text((offset+15,910), "white dots: logical lane centers | cyan: scan path", font=font(14), fill="white")
        draw.text((20, 945), "Yellow: road markings | blue: road | purple: sidewalk | dark: unknown/not shown",font=font(16),fill="white")
        draw.text((20, 974), "Live/replay match = processing consistency, NOT HD-map accuracy. No traffic-light state inference.",font=font(15),fill="#bbc6d4")
        pictures.append(canvas)
        if index in {0, len(records)//2, len(records)-1}:
            canvas.save(output / f"comparison_{index:04d}.png")
    duration=round((records[1]['timestamp_s']-records[0]['timestamp_s'])*1000*frame_stride)
    pictures[0].save(output / "ego_centered_mapping.gif", save_all=True, append_images=pictures[1:], duration=duration, loop=0)
    final_same = accumulated.snapshot() == json.loads((source / "map.json").read_text())
    # JSON tuples/lists differ only for dynamics (none in this static probe).
    if not final_same:
        raise ValueError("published reconstruction differs from capture")
    metrics = dict(frame_count=len(records), live_replay_exact_match=summary["live_replay_exact_match"],
                   publication_replay_match=final_same, observed_cells=len(accumulated.cells),
                   point_exposures_by_label=dict(counts), hd_map_approved=False,
                   road_point_proxy_check_count=checked, road_points_outside_logical_proxy=out_of_proxy,
                   road_to_logical_proxy_mean_m=float(np.mean(distances)),
                   road_to_logical_proxy_p95_m=float(np.percentile(distances,95)),
                   caveat="Non-independent oracle-point diagnostic against sampled lane rectangles, not rendered GT accuracy or learned map quality")
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    p=argparse.ArgumentParser()
    p.add_argument("source",type=Path);p.add_argument("output",type=Path)
    a=p.parse_args();render(a.source,a.output)
