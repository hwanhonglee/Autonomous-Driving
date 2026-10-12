"""HH_261002 - Prepared whole-record same-LEFT20k lateral A/B gallery; no inference, actuation or verdict repair."""
from __future__ import annotations

import ast
import copy
from pathlib import Path
from types import FunctionType

from . import publish_temporal_left_joint_left45_comparison_20261002 as previous

for _name, _value in vars(previous).items():
    if not _name.startswith("__"):
        globals()[_name] = _value
SCHEMA = "portable_e2e.actual_left_joint_lateral_pair_publication.20261002.v1"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
CAPTURE_SCHEMA = "portable_e2e.actual_left_joint_lateral_pair.20261002.v1"
OWNER_SCHEMA = CAPTURE_SCHEMA + ".owned"
ARMS = ("baseline", "short_lookahead")
PROFILES = dict(baseline=(1.5,.8),short_lookahead=(1.,.5))
PAIR = ROOT / "storage/experiments/validation/2026-10-02/temporal_left_joint_lateral_pair_v1"
OUTPUT = ROOT / "docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/24_actual_left_joint_lateral_lookahead_pair"
CALIBRATION_SCHEMA = "portable_e2e.causal_selected_lateral_lookahead_pair.20261002.v1"
FROZEN_PUBLISHER_SHA = "4742365be59cf6ea02d833368e11f76b40ab74ebdaa726f264897e3397dc9d53"
FROZEN = dict(previous.FROZEN, **{
    "tools/e2e/publish_temporal_left_joint_left45_comparison_20261002.py":FROZEN_PUBLISHER_SHA,
    "tools/e2e/collect_temporal_left_joint_lateral_pair_20261002.py":"cf7635f9e39adbb9c39ff936c9bb7b681b7b4cadb70c8b521ea59faaad89b320",
    "tools/e2e/research_temporal_left_joint_lateral_pair_20261002.py":"1104742533e8dda8056de9eab3d1cdf243ceef4276e1b28a578cc9801edb039c",
    "learning/tests/test_temporal_left_joint_lateral_pair_20261002.py":"ec5d70586dd29606e28d44025f24585c8316cf2ac3225430a4f3fb3ab64c27e4"})


def definition(name):
    require(recorded.digest(Path(previous.__file__)) == FROZEN_PUBLISHER_SHA,"exact immutable LEFT comparison publisher required")
    original = next(n for n in ast.parse(Path(previous.__file__).read_bytes()).body
        if isinstance(n,ast.FunctionDef) and n.name == name)
    changed = copy.deepcopy(original); replaced = []
    if name == "load_pair":
        targets = (('authority["parent"] if arm == ARMS[0] else authority["endpoint"]','authority["endpoint"]'),
                   ('6000 if arm == ARMS[0] else 20000','20000'))
        for old,new in targets:
            old_node,new_node = (ast.parse(v,mode="eval").body for v in (old,new))
            hits = [n for n in ast.walk(changed) if ast.dump(n)==ast.dump(old_node)]
            require(len(hits)==1,"one exact original model identity branch required")
            wanted=hits[0]
            class Replace(ast.NodeTransformer):
                def generic_visit(self,node):
                    return copy.deepcopy(new_node) if node is wanted else super().generic_visit(node)
            changed=Replace().visit(changed)
            replaced.append((old_node,new_node,wanted.lineno))
    elif name == "publish":
        hits = [n for n in ast.walk(changed) if isinstance(n,ast.Constant) and isinstance(n.value,str)
            and "GOOD6000/LEFT20k 비교입니다" in n.value]
        require(len(hits)==1,"one original human comparison identity paragraph required")
        old=hits[0].value
        hits[0].value=old.replace("GOOD6000/LEFT20k 비교입니다","같은 LEFT20k의 lateral1.5m/0.8s 대조군과1.0m/0.5s 후보 비교입니다")
        replaced.append((ast.Constant(value=old),copy.deepcopy(hits[0]),hits[0].lineno))
    inverse=copy.deepcopy(changed)
    for old,new,line in reversed(replaced):
        class Restore(ast.NodeTransformer):
            def generic_visit(self,node):
                return copy.deepcopy(old) if getattr(node,"lineno",None)==line and ast.dump(node)==ast.dump(new) else super().generic_visit(node)
        # HH_261002 - Replacement literals carry no position until compilation; exact unique expression restores both identities.
        hits=[n for n in ast.walk(inverse) if ast.dump(n)==ast.dump(new)]
        require(len(hits)==1,"one exact inverse identity expression required")
        wanted=hits[0]
        class Undo(ast.NodeTransformer):
            def generic_visit(self,node):return copy.deepcopy(old) if node is wanted else super().generic_visit(node)
        inverse=Undo().visit(inverse)
    require(ast.dump(inverse)==ast.dump(original),"another frozen archive/math/render statement changed")
    return ast.fix_missing_locations(changed)


for _name, _value in vars(previous).items():
    if isinstance(_value,FunctionType) and _value.__module__==previous.__name__:
        exec(compile(ast.Module(body=[definition(_name)],type_ignores=[]),__file__,"exec"),globals())
_load_pair,_render=load_pair,render


def load_pair(caller_path,caller_sha,*,decode=False):
    caller,bundles,pins=_load_pair(caller_path,caller_sha,decode=decode)
    for bundle in bundles:
        arm=bundle["arm"];minimum,seconds=PROFILES[arm]
        plan=pins.json(PAIR/(arm+"_plan.json"));calibration=plan.get("lateral_lookahead_calibration",{})
        require(calibration.get("schema")==CALIBRATION_SCHEMA and calibration.get("arm")==arm
            and calibration.get("minimum_lookahead_m")==minimum and calibration.get("lookahead_time_s")==seconds
            and calibration.get("selected_NN_trajectory_unchanged") is True and calibration.get("NN_weights_unchanged") is True
            and calibration.get("original_STOP_and_non_GO_tracker_preserved") is True
            and plan.get("same_final_LEFT20k_weights_both_arms") is True and plan.get("matched_baseline_repeat") is True
            and bundle["diagnostic"].get("lateral_lookahead_calibration")==calibration,
            "exact same20k/profile/unaltered original diagnostic required")
        bundle["lateral_lookahead_calibration"]=calibration
    require(bundles[0]["endpoint"]==bundles[1]["endpoint"],"BOTH physical profiles must use identical four final20k pins")
    pins.recheck();return caller,bundles,pins


def render(bundle,index):
    # HH_261002 - Replace only the short second header line; six-camera FOV, ego map and prior-ACK learned path remain exact.
    image=_render(bundle,index);draw=ImageDraw.Draw(image);minimum,seconds=PROFILES[bundle["arm"]]
    text=f"Same LEFT20k | lateral{minimum:g}m/{seconds:g}s | CPU4 PI2.4 preview0.5 | TRAIN-seen diagnostic | no promotion"
    require(draw.textbbox((0,0),text,font=visual.font(18,True))[2]<=1652,"whole readable lateral header must fit")
    draw.rectangle((20,56,1680,93),fill=visual.BG)
    draw.text((24,58),text,fill=visual.MUTED,font=visual.font(18,True))
    return image


del _name,_value
if __name__=="__main__":raise SystemExit(main())
