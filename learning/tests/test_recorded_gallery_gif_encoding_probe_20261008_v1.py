"""HH_261008 - Five-row probe scalar/metadata/tmp-tree fixtures; no real pixel encoding, renderer or NN."""
import ast
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/"tools/e2e/probe_recorded_gallery_gif_encoding_20261008_v1.py"

@pytest.fixture
def probe():
    spec=importlib.util.spec_from_file_location("_hh_gallery_probe_pure",SOURCE);value=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value);return value

def test_exact_five_declared_slots_and_no_full_render_permission(probe):
    assert probe.SLOTS==(("DIRECT",0),("DIRECT",459),("DIRECT",917),("ROS",0),("ROS",28))
    fixed=probe.fixed();assert fixed["slots"]==[list(slot) for slot in probe.SLOTS]
    assert fixed["sample_only"] and fixed["encoded_sample_envelope_is_not_future_worstcase_proof"]
    assert not fixed["full947_render_authorized"] and not fixed["source_frame_dropping"]
    assert fixed["GIF_size_px"]==[600,300] and fixed["GIF_palette_colors"]==16
    assert probe.TOTAL==2*1024**2 and probe.CLOSING==512*1024 and probe.IMAGE==64*1024
    assert probe.SOURCES+probe.PLANS+probe.CACHE+5*probe.IMAGE<=probe.TOTAL-probe.CLOSING
    assert not any(name in sys.modules for name in ("numpy","matplotlib","PIL","torch","rclpy","carla"))

@pytest.mark.parametrize("rows",[
    [],[{"status":"UNEXECUTED","encoded_file_bytes":None}],
    [{"status":"PASS","encoded_file_bytes":100},{"status":"FAILED_RETAINED","encoded_file_bytes":200}],
    [{"status":"PASS","encoded_file_bytes":100},{"status":"PASS","encoded_file_bytes":200}],
])
def test_sample_byte_envelope_never_becomes_fullproof_or_permission(probe,rows):
    value=probe.sample_estimate(rows);passed=[r["encoded_file_bytes"] for r in rows if r["status"]=="PASS"]
    assert value["observed_complete_samples"]==len(passed) and value["full947_verified"] is False
    assert value["estimate_only_NOT_guarantee_or_permission"] is True
    assert value["sample_max_times947_bytes"]==(max(passed)*947 if passed else None)
    assert value["full_GIF_sum_cap_bytes"]==12*1024**2

@pytest.fixture
def cohort(probe):
    old={f"ancestor/{i}.py":"a"*64 for i in range(173)}
    extra={name:"b"*64 for name in probe.SOURCE_NAMES}
    extra.update({f"history/{i}.json":"c"*64 for i in range(6)})
    plan=dict(source_sha256=old)
    result=dict(source_sha256=deepcopy(old),closing_errors=[])
    pins={str((probe.V6PD/"ROOT_plan.json").relative_to(probe.ROOT)):"d"*64,
        "storage/experiments/diagnostics/2026-10-08/recorded_vehicle_centered_gallery_v6/result.json":"e"*64}
    def read(path,pin):
        return deepcopy(plan if Path(path).name=="ROOT_plan.json" else result)
    def sha(path):return old[str(Path(path).relative_to(probe.V6PD/"sources"))]
    return SimpleNamespace(old=old,extra=extra,g=SimpleNamespace(PRESERVED_V6_PINS=pins,read_json=read,sha=sha))

def test_all173_true_prior_archive_paths_and_exact10_fresh_copies(probe,cohort):
    sources={**cohort.old,**cohort.extra};actual,copied=probe.mapping(cohort.g,sources)
    assert len(actual)==183 and copied==set(cohort.extra)
    assert all(actual[name]==str(probe.V6PD/"sources"/name) for name in cohort.old)
    assert all(actual[name]==str(probe.PD/"sources"/name) for name in cohort.extra)

@pytest.mark.parametrize("fault",["missing_ancestor","changed_ancestor","extra","missing_probe_source"])
def test_mixed_source_archive_denials(probe,cohort,fault):
    sources={**cohort.old,**cohort.extra}
    if fault=="missing_ancestor":sources.pop(next(iter(cohort.old)))
    elif fault=="changed_ancestor":sources[next(iter(cohort.old))]="f"*64
    elif fault=="extra":sources["foreign.py"]="f"*64
    else:
        sources.pop(probe.SOURCE_NAMES[0]);sources["foreign_replacement.py"]="f"*64
    with pytest.raises(ValueError):probe.mapping(cohort.g,sources)

def test_output_cap_fails_before_writing_and_fresh_file_is_create_only(probe,tmp_path,monkeypatch):
    monkeypatch.setattr(probe,"PD",tmp_path/"plans");monkeypatch.setattr(probe,"OUT",tmp_path/"out")
    probe.OUT.mkdir();g=SimpleNamespace(canonical=lambda p,exists=True:Path(p).resolve(strict=exists))
    monkeypatch.setattr(probe,"budget",lambda *args,**kwargs:{})
    path=probe.OUT/"sample.gif"
    with pytest.raises(ValueError):probe.write_new(g,path,b"x"*(probe.IMAGE+1),probe.IMAGE)
    assert not path.exists()
    probe.write_new(g,path,b"SYNTHETIC_NOT_PIXEL_DATA",probe.IMAGE)
    with pytest.raises(FileExistsError):probe.write_new(g,path,b"OVERWRITE_DENIED",probe.IMAGE)
    assert path.read_bytes()==b"SYNTHETIC_NOT_PIXEL_DATA"

def test_execution_separate_grant_before_providers_and_no_full_renderer_call():
    tree=ast.parse(SOURCE.read_bytes());functions={n.name:n for n in tree.body if isinstance(n,ast.FunctionDef)}
    body=ast.unparse(functions["execute"])
    assert body.index("grant == dict")<body.index("verify(g, plan)")<body.index("import matplotlib")
    assert "full_render_authorized=False" in body and "five_source_rows_only=True" in body
    calls={ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)}
    assert not any(name in calls for name in ("g.execute","forward","train","fit","torch.load"))
    assert {"g.draw","g.gif_palette_frame","g.encode_gif_frame_bytes","g.own_run_transport_proof"}<=calls
    assert "stream.read(PLANS + 1)" in ast.unparse(functions["main"])
    assert "image.close()" in body and "plt.close(view[0])" in body
