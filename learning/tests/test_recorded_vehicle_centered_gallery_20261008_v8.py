"""HH_261008 - V8 exact cleanup-only inverse and scalar combined-fault fixtures; no real pixels, jobs or observations.
Frozen V6 functions and complete denominators stay pinned; new encodings require real ROOT probe evidence.
"""
import ast
from copy import deepcopy
import hashlib
import importlib.util
from io import BytesIO
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/"tools/e2e/report_recorded_vehicle_centered_gallery_20261008_v8.py"
OLD=ROOT/"tools/e2e/report_recorded_vehicle_centered_gallery_20261008_v6.py"
PREVIOUS=ROOT/"tools/e2e/report_recorded_vehicle_centered_gallery_20261008_v7.py"

@pytest.fixture
def gallery():
    spec=importlib.util.spec_from_file_location("_hh_gallery8_pure",SOURCE);value=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value);return value

def functions(path):return {n.name:n for n in ast.parse(path.read_bytes()).body if isinstance(n,ast.FunctionDef)}
def dump(value):return ast.dump(value,include_attributes=False)

def restore_only_v7(function,original):
    """HH_261008 - Undo exact image-recipe and PNG-order interventions, never waive original execute or geometry."""
    value=deepcopy(function);baseline=deepcopy(original)
    if value.name=="fixed":
        call=next(n for n in ast.walk(value) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=="dict")
        added={"source_canvas_size_px","GIF_size_px","GIF_palette_colors","GIF_resample","GIF_palette",
            "PNG_full_resolution_before_GIF","GIF_frame_dropping","feasibility_samples_do_not_prove_full_interval_size"}
        selected=[kw for kw in call.keywords if kw.arg in added];assert len(selected)==len(added)
        call.keywords=[kw for kw in call.keywords if kw.arg not in added]
    if value.name=="append_gif_frame":
        first=value.body[0];assert isinstance(first,ast.Assign) and ast.unparse(first.targets[0])=="raw"
        assert ast.unparse(first.value)=="encode_gif_frame_bytes(image, plugin, delay, first)"
        begin=next(i for i,n in enumerate(baseline.body) if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=="used")
        value.body[0:1]=deepcopy(baseline.body[:begin])
    if value.name=="execute":
        original_png=next(n for n in ast.walk(baseline) if isinstance(n,ast.For) and ast.unparse(n.iter)=="PNG_ORDINALS")
        working=next(n for n in ast.walk(value) if isinstance(n,ast.Try) and any(isinstance(n,ast.For) and ast.unparse(n.iter)=="PNG_ORDINALS" for n in n.body))
        current_png=next(n for n in working.body if isinstance(n,ast.For) and ast.unparse(n.iter)=="PNG_ORDINALS")
        inserts=[n for n in current_png.body if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=="result['PNG_status'][str(ordinal)]"]
        assert len(inserts)==1 and isinstance(inserts[0].value,ast.Constant) and inserts[0].value.value=="PASS"
        current_png.body.remove(inserts[0]);assert dump(current_png)==dump(original_png)
        working.body.remove(current_png)
        status_assignment=next(n for n in working.body if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=="result['PNG_status']")
        assert isinstance(status_assignment.value,ast.DictComp);working.body.remove(status_assignment)
        arm=next(n for n in working.body if isinstance(n,ast.For) and ast.unparse(n.target)=="(arm, observed)")
        working.body.insert(working.body.index(arm)+1,current_png)
        original_image=next(n for n in ast.walk(baseline) if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=="image")
        image=next(n for n in ast.walk(value) if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=="image")
        assert ast.unparse(image.value)=="gif_palette_frame(fig, Image)";image.value=deepcopy(original_image.value)
        frame_loop=next(n for n in ast.walk(value) if isinstance(n,ast.For) and ast.unparse(n.target)=="(ordinal, row)")
        old_frame=next(n for n in ast.walk(baseline) if isinstance(n,ast.For) and ast.unparse(n.target)=="(ordinal, row)")
        close=next(n for n in frame_loop.body if isinstance(n,ast.Try) and n.finalbody)
        assert len(close.body)==len(close.finalbody)==1
        assert ast.unparse(close.body[0])=="append_gif_frame(stream, image, GifImagePlugin, record['GIF_delay_ms'], ordinal == 0)"
        assert ast.unparse(close.finalbody[0])=="image.close()"
        expected=[n for n in old_frame.body if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call)
            and ast.unparse(n.value.func) in ("append_gif_frame","image.close")]
        assert len(expected)==2;place=frame_loop.body.index(close);frame_loop.body[place:place+1]=deepcopy(expected)
        failed=working.handlers[0].body
        added=[n for n in failed if isinstance(n,ast.If) and "cursor[0] == 'PNG'" in ast.unparse(n.test)]
        assert len(added)==1 and len(added[0].body)==1;failed.remove(added[0])
    return value


def restore_only_v8(function,previous):
    """HH_261008 - Invert only the exact two figure closers, frame primary-retention clause and extra result error field."""
    value=deepcopy(function)
    if value.name!="execute":return value
    fields=[n for n in ast.walk(value) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=="dict"
        and any(k.arg=="artifact_cleanup_errors" for k in n.keywords)]
    assert len(fields)==1
    field=next(k for k in fields[0].keywords if k.arg=="artifact_cleanup_errors")
    assert dump(field.value)==dump(ast.parse("[]",mode="eval").body);fields[0].keywords.remove(field)
    expected=ast.parse('finish_diagnostic_cleanup(result,None,(("figure",lambda:plt.close(view[0])),))').body[0]
    replacement=ast.parse('plt.close(view[0])').body[0]
    class ExactFigureInverse(ast.NodeTransformer):
        count=0
        def visit_Expr(self,node):
            if dump(node)==dump(expected):
                self.count+=1;return deepcopy(replacement)
            return self.generic_visit(node)
    inverse=ExactFigureInverse();value=inverse.visit(value);assert inverse.count==2
    frame=next(n for n in ast.walk(value) if isinstance(n,ast.For) and ast.unparse(n.target)=="(ordinal, row)")
    expected_clause=ast.parse(
        'frame_error=None\n'
        'try:append_gif_frame(stream,image,GifImagePlugin,record["GIF_delay_ms"],ordinal==0)\n'
        'except BaseException as failure:frame_error=failure\n'
        'finally:finish_diagnostic_cleanup(result,frame_error,(("GIF image",image.close),))\n').body
    matches=[i for i in range(len(frame.body)-1) if [dump(n) for n in frame.body[i:i+2]]==[dump(n) for n in expected_clause]]
    assert len(matches)==1
    old_frame=next(n for n in ast.walk(previous) if isinstance(n,ast.For) and ast.unparse(n.target)=="(ordinal, row)")
    old_clause=next(n for n in old_frame.body if isinstance(n,ast.Try) and n.finalbody)
    frame.body[matches[0]:matches[0]+2]=[deepcopy(old_clause)]
    return value

def test_every_original_V6_function_except_explicit_tables_exact_after_narrow_inverse(gallery):
    old,new=functions(OLD),functions(SOURCE)
    assert new.keys()==old.keys()|{"gif_palette_frame","encode_gif_frame_bytes","finish_diagnostic_cleanup"}
    previous=functions(PREVIOUS)
    for name in old.keys()-{"tables"}:assert dump(restore_only_v7(restore_only_v8(new[name],previous[name]),old[name]))==dump(old[name]),name
    assert not any(name in sys.modules for name in ("numpy","matplotlib","PIL","torch","rclpy","carla"))

@pytest.mark.parametrize("name",["draw","stored_selection","source_records","verify_native_geometry","gif_delay","own_run_transport_proof","execute"])
def test_original_geometry_source_timer_guard_extra_call_never_waived(gallery,name):
    old,new=functions(OLD),functions(SOURCE);changed=deepcopy(new[name])
    changed.body.append(ast.Expr(value=ast.Call(func=ast.Name(id="unapproved_extra_call",ctx=ast.Load()),args=[],keywords=[])))
    assert dump(restore_only_v7(restore_only_v8(changed,functions(PREVIOUS)[name]),old[name]))!=dump(old[name])

def test_full_denominators_quotas_and_PNG_before_any_GIF(gallery):
    fixed=gallery.fixed();assert fixed["DIRECT_GIF_frames"]==918 and fixed["ROS_GIF_actual_prefix_frames"]==29 and fixed["ROS_UNEXECUTED"]==889
    assert fixed["source_canvas_size_px"]==[800,400] and fixed["GIF_size_px"]==[600,300] and fixed["GIF_palette_colors"]==16
    assert fixed["PNG_full_resolution_before_GIF"] and not fixed["GIF_frame_dropping"]
    assert gallery.TOTAL==32*1024**2 and gallery.CLOSING==4*1024**2 and gallery.GIFS==12*1024**2
    assert gallery.PNG==1024**2 and gallery.PNG_ORDINALS==(0,459,917) and gallery.RSS==512*1024**2 and gallery.WALL==900
    execute=functions(SOURCE)["execute"];working=next(n for n in ast.walk(execute) if isinstance(n,ast.Try) and any(isinstance(n,ast.For) and ast.unparse(n.iter)=="PNG_ORDINALS" for n in n.body))
    png=next(i for i,n in enumerate(working.body) if isinstance(n,ast.For) and ast.unparse(n.iter)=="PNG_ORDINALS")
    gif=next(i for i,n in enumerate(working.body) if isinstance(n,ast.For) and ast.unparse(n.target)=="(arm, observed)")
    assert png<gif and "savefig(buffer, format='png', dpi=120)" in ast.unparse(working.body[png])
    assert fixed["actual_RViz_screenshot"] is fixed["new_drive"] is False and fixed["formal_main_approved"]==fixed["formal_sub_approved"]==0

class FakeImage:
    def __init__(self,name,events,fault=None):
        self.name,self.events,self.fault=name,events,fault;self.size=(600,300);self.mode="P";self.palette=SimpleNamespace(tobytes=lambda:b"x"*48)
    def resize(self,size,**kwargs):
        self.events.append(("resize",size,kwargs))
        if self.fault=="resize":raise ValueError("synthetic_resize_failure")
        return FakeImage("small",self.events,self.fault)
    def quantize(self,**kwargs):
        self.events.append(("quantize",kwargs))
        if self.fault=="quantize":raise ValueError("synthetic_quantize_failure")
        value=FakeImage("palette",self.events,self.fault)
        if self.fault=="shape":value.size=(800,400)
        if self.fault=="palette":value.palette=SimpleNamespace(tobytes=lambda:b"x"*49)
        return value
    def close(self):self.events.append(("close",self.name))

def fake_provider(events,fault=None):
    def frombytes(mode,size,raw):
        events.append(("frombytes",mode,size,raw));return FakeImage("rgb",events,fault)
    return SimpleNamespace(BILINEAR=2,MEDIANCUT=0,NONE=0,frombytes=frombytes)

@pytest.mark.parametrize("fault",[None,"resize","quantize","shape","palette"])
def test_fake_GIF_recipe_exact_size_palette_lifetime_no_actual_pixels(gallery,fault):
    events=[];fig=SimpleNamespace(canvas=SimpleNamespace(get_width_height=lambda:(800,400),tostring_rgb=lambda:b"SYNTHETIC_NOT_REAL_PIXELS"))
    if fault:
        with pytest.raises(ValueError):gallery.gif_palette_frame(fig,fake_provider(events,fault))
        assert ("close","rgb") in events
    else:
        image=gallery.gif_palette_frame(fig,fake_provider(events))
        assert ("resize",(600,300),{"resample":2}) in events
        assert ("quantize",{"colors":16,"method":0,"kmeans":0,"dither":0}) in events
        assert ("close","small") in events and ("close","rgb") in events and ("close","palette") not in events
        image.close();assert events[-1]==("close","palette")

def test_fake_GIF_byte_recipe_retains_delay_order_and_whole_quota(gallery,monkeypatch):
    calls=[]
    plugin=SimpleNamespace(getheader=lambda image,**kwargs:([b"GIF89a"],None),getdata=lambda image,**kwargs:(calls.append(kwargs) or [b"SYNTHETIC_FRAME"]))
    monkeypatch.setattr(gallery,"budget",lambda *args,**kwargs:{"gif":0})
    stream=BytesIO();gallery.append_gif_frame(stream,"fake",plugin,100,True);gallery.append_gif_frame(stream,"fake",plugin,0,False)
    assert [c["duration"] for c in calls]==[100,0] and all(c["disposal"]==2 and c["optimize"] is False for c in calls)
    before=stream.getvalue();monkeypatch.setattr(gallery,"budget",lambda *args,**kwargs:{"gif":gallery.GIFS})
    with pytest.raises(ValueError):gallery.append_gif_frame(stream,"fake",plugin,100,False)
    assert stream.getvalue()==before

def test_actual_V6_failure_and_incomplete_GIF_are_sealed_not_repaired(gallery):
    assert len(gallery.PRESERVED_V6_PINS)==8 and len(gallery.V6_PARTIAL_GIF_INPUT)==1 and len(gallery.PROBE_SOURCE_NAMES)==2
    for name,pin in gallery.PRESERVED_V6_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    result=gallery.read_json(ROOT/"storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v6/result.json")
    assert result["status"]=="FAILED_RETAINED" and result["completed_GIF_frames"]=={"DIRECT":509,"ROS":0}
    assert all(len(rows)==918 for rows in result["whole_render_ledger"].values()) and result["PNG_sha256"]=={}
    assert result["closing_errors"]==[] and result["NN_inferences"]==result["training_steps"]==0
    assert "inputs.update(expected_failed)" in SOURCE.read_text() and "complete failed V6 output census differs" in SOURCE.read_text()

# HH_261008 - A synthetic closer is not a real Pillow/Matplotlib resource observation.
@pytest.mark.parametrize("primary_present",[False,True])
@pytest.mark.parametrize("failed_closers",[(),(0,),(1,),(0,1)])
def test_primary_error_and_every_independent_closer_preserved(gallery,primary_present,failed_closers):
    calls=[];result=dict(artifact_cleanup_errors=[]);primary=RuntimeError("synthetic_original_append_or_draw_or_write") if primary_present else None
    def closer(number):
        def close():
            calls.append(number)
            if number in failed_closers:raise OSError("synthetic_closer_"+str(number))
        return close
    closers=(("image",closer(0)),("figure",closer(1)))
    if primary is not None or failed_closers:
        with pytest.raises(BaseException) as seen:gallery.finish_diagnostic_cleanup(result,primary,closers)
        if primary is not None:assert seen.value is primary
        else:assert "cleanup errors retained separately" in str(seen.value)
    else:gallery.finish_diagnostic_cleanup(result,primary,closers)
    assert calls==[0,1] and [v["resource"] for v in result["artifact_cleanup_errors"]]==[("image","figure")[i] for i in failed_closers]
    assert all("synthetic_closer_" in v["error"] for v in result["artifact_cleanup_errors"])

def test_every_frozen_V7_function_except_tables_exact_after_cleanup_inverse(gallery):
    old,new=functions(PREVIOUS),functions(SOURCE)
    assert new.keys()==old.keys()|{"finish_diagnostic_cleanup"}
    for name in old.keys()-{"tables"}:assert dump(restore_only_v8(new[name],old[name]))==dump(old[name]),name

def test_unexecuted_V7_four_sources_and_source_finding_preserved(gallery):
    assert len(gallery.PRESERVED_V7_SOURCE_ONLY_PINS)==5
    assert not any("result.json" in name or "failure_v1" in name for name in gallery.PRESERVED_V7_SOURCE_ONLY_PINS)
    for name,pin in gallery.PRESERVED_V7_SOURCE_ONLY_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    assert "SOURCE_READY only" in SOURCE.read_text()

