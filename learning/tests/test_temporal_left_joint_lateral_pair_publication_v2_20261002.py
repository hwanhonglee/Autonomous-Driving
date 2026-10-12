"""HH_261002 - Synthetic-only V2 fixture representation fixes; original publisher and gates stay frozen."""
import ast
import copy
import gzip
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from PIL import Image, ImageColor
import pytest
from scripts.e2e import publish_temporal_left_joint_lateral_pair_20261002 as pub


def fixtures():
    # HH_261002 - The fixture source remains the real test repository even after the synthetic publisher ROOT is monkeypatched.
    source=Path(__file__).resolve().parents[2]/"learning/tests/test_temporal_left_joint_left45_comparison_publication_20261002.py"
    tree=ast.parse(source.read_bytes());names=("control","fixture","learned_fixture","context_fixture","recorder","synthetic_images","write_json","write_rows","pinned_pair")
    nodes=[copy.deepcopy(n) for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names]
    assert len(nodes)==len(names)
    namespace=dict(globals())
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes,type_ignores=[])),__file__,"exec"),namespace)
    return namespace


def pinned_pair(tmp_path,monkeypatch):
    f=fixtures();path,sha,caller=f["pinned_pair"](tmp_path,monkeypatch)
    for arm in pub.ARMS:
        run=pub.PAIR/arm;planpath=pub.PAIR/(arm+"_plan.json");plan=json.loads(planpath.read_text())
        minimum,seconds=pub.PROFILES[arm]
        calibration=dict(schema=pub.CALIBRATION_SCHEMA,arm=arm,minimum_lookahead_m=minimum,lookahead_time_s=seconds,
            selected_NN_trajectory_unchanged=True,NN_weights_unchanged=True,original_STOP_and_non_GO_tracker_preserved=True)
        plan.update(lateral_lookahead_calibration=calibration,same_final_LEFT20k_weights_both_arms=True,matched_baseline_repeat=True)
        caller["case_plan_sha256"][arm]=f["write_json"](planpath,plan);f["write_json"](run/"declaration.json",plan)
        owner=json.loads((run/"owner_result.json").read_text());owner["declaration_sha256"]=caller["case_plan_sha256"][arm]
        f["write_json"](run/"owner_result.json",owner)
        manifestpath=run/"episode.partial/manifest.json";manifest=json.loads(manifestpath.read_text())
        manifest["capture_contract"]["learned_LEFT45_protocol"]={k:v for k,v in plan.items() if k not in ("source_sha256","asset_sha256","external_source_sha256")}
        f["write_json"](manifestpath,manifest)
        diagpath=run/"diagnostic_report.json";diag=json.loads(diagpath.read_text());diag.update(original_owner=owner,lateral_lookahead_calibration=calibration)
        diag["raw_source_sha256"][str(manifestpath)]=pub.recorded.digest(manifestpath);f["write_json"](diagpath,diag)
        caller["runs"][arm]={name:pub.recorded.digest(run/name) for name in pub.archive.inventory(run)}
    batchpath=pub.PAIR/"batch_plan.json";batch=json.loads(batchpath.read_text());batch["case_sha256"]=caller["case_plan_sha256"]
    caller["batch_plan_sha256"]=f["write_json"](batchpath,batch)
    return path,f["write_json"](path,caller),caller


def test_sourceonly_contract_same_counts_newidentity_no_dynamic_neural_import():
    c=pub.public_contract()
    assert c["arm_order"]==["baseline","short_lookahead"] and c["expected_complete_counts"]==dict(native=1055,camera=528,ACK=1060,JPEG=3168)
    assert c["both_outcomes_required"] and not c["model_inference_executed"] and not c["model_promoted"]
    assert all(c["source_sha256"][p]==sha for p,sha in pub.FROZEN.items())
    for name in ("validate_records","prior_ack","validate_recording","records_validator","screenshot_indices"):
        original=next(n for n in ast.parse(Path(pub.previous.__file__).read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name==name)
        assert ast.dump(pub.definition(name))==ast.dump(original)


def test_exact_TWO_identity_AST_branches_only():
    node=pub.definition("load_pair");dump=ast.dump(node)
    assert 'value=6000' not in dump and 'value=20000' in dump
    assert "exact original GOOD6000/LEFT20k runtime/declaration/denials required" in dump
    assert "all original raw failure bytes required" in dump and "all30 immutable training sources required" in dump


def test_REAL_fullsealed_BOTH_reader_retains_same20k_failures_raw_and_afterACK(tmp_path,monkeypatch):
    path,sha,caller=pinned_pair(tmp_path,monkeypatch)
    actual,bundles,pins=pub.load_pair(path,sha)
    assert actual==caller and [b["arm"] for b in bundles]==list(pub.ARMS)
    assert bundles[0]["endpoint"]==bundles[1]["endpoint"]
    assert all(b["diagnostic"]["status"]=="FAIL_RETAINED" and b["recording_census"]["actual_after_ACK_count"]==2 for b in bundles)
    assert all(b["lateral_lookahead_calibration"]["arm"]==b["arm"] for b in bundles)
    pins.recheck()


@pytest.mark.parametrize("fault",["profile","oldschema","wrongarm","trajectory","weights","STOP","notmatched","diagnostic","promotion","missingcase","raw","inventory","endpoint"])
def test_REAL_reader_rejects_wrongprofiles_and_all_old_archive_mutations(tmp_path,monkeypatch,fault):
    path,sha,caller=pinned_pair(tmp_path,monkeypatch);f=fixtures();arm="short_lookahead";run=pub.PAIR/arm
    planpath=pub.PAIR/(arm+"_plan.json");plan=json.loads(planpath.read_text());cal=plan["lateral_lookahead_calibration"]
    if fault=="profile":cal["lookahead_time_s"]=.8
    elif fault=="oldschema":cal["schema"]="old"
    elif fault=="wrongarm":cal["arm"]="baseline"
    elif fault=="trajectory":cal["selected_NN_trajectory_unchanged"]=False
    elif fault=="weights":cal["NN_weights_unchanged"]=False
    elif fault=="STOP":cal["original_STOP_and_non_GO_tracker_preserved"]=False
    elif fault=="notmatched":plan["matched_baseline_repeat"]=False
    elif fault=="endpoint":plan["selected_four_endpoint_sha256"]["weights_sha256"]="d"*64
    if fault in ("profile","oldschema","wrongarm","trajectory","weights","STOP","notmatched","endpoint"):
        caller["case_plan_sha256"][arm]=f["write_json"](planpath,plan);f["write_json"](run/"declaration.json",plan)
        op=run/"owner_result.json";own=json.loads(op.read_text());own["declaration_sha256"]=caller["case_plan_sha256"][arm];f["write_json"](op,own)
        mp=run/"episode.partial/manifest.json";m=json.loads(mp.read_text());m["capture_contract"]["learned_LEFT45_protocol"].update({k:v for k,v in plan.items() if k not in ("source_sha256","asset_sha256","external_source_sha256")});f["write_json"](mp,m)
        dp=run/"diagnostic_report.json";d=json.loads(dp.read_text());d.update(original_owner=own,lateral_lookahead_calibration=cal);d["raw_source_sha256"][str(mp)]=pub.recorded.digest(mp);f["write_json"](dp,d)
        bp=pub.PAIR/"batch_plan.json";b=json.loads(bp.read_text());b["case_sha256"]=caller["case_plan_sha256"];caller["batch_plan_sha256"]=f["write_json"](bp,b)
        caller["runs"][arm]={n:pub.recorded.digest(run/n) for n in pub.archive.inventory(run)}
    elif fault in ("diagnostic","promotion"):
        p=run/"diagnostic_report.json";d=json.loads(p.read_text())
        if fault=="promotion":d["model_promoted"]=True
        else:d["lateral_lookahead_calibration"]["minimum_lookahead_m"]=1.5
        caller["runs"][arm]["diagnostic_report.json"]=f["write_json"](p,d)
    elif fault=="missingcase":caller["runs"].pop(arm)
    elif fault=="raw":(run/"episode.partial/states.jsonl").write_text("changed")
    else:(run/"unsealed.txt").write_text("changed")
    sha=f["write_json"](path,caller)
    with pytest.raises((ValueError,RuntimeError)):pub.load_pair(path,sha)


@pytest.mark.parametrize("arm",pub.ARMS)
def test_synthetic_render_multiline_exactfullFOV_same20k_profile(tmp_path,arm):
    b=fixtures()["synthetic_images"](tmp_path,arm);picture=pub.render(b,0)
    assert picture.size==(1700,860) and picture.mode=="RGB"
    assert pub.displayed_receipt(b,0)["purple_plan_displayed"] is False
    # HH_261002 - Compare the observed RGB tuple to the same declared color, not to its textual hex representation.
    assert picture.getpixel((24,150))==ImageColor.getrgb(pub.visual.BG)


@pytest.mark.parametrize("complete",[True,False])
def test_wholeclock_and_screens_do_not_select_success(complete):
    n=528 if complete else 300
    cameras=[dict(timestamp=1.+i*.1) for i in range(n)]
    indices=pub.recorded.sampled_indices(n);durations=pub.recorded.playback_durations(cameras,indices)
    assert indices[-1]==n-1 and sum(durations)==n*100
    assert list(pub.screenshot_indices(n).values())==([0,132,264,395,527] if complete else [0,74,149,224,299])


@pytest.mark.parametrize("module",["torch","carla"])
def test_NO_model_or_simulator_process_before_payload(monkeypatch,module):
    monkeypatch.setattr(pub,"sys",SimpleNamespace(modules={module:object()}))
    monkeypatch.setattr(pub.archive.Pins,"file",lambda *a:pytest.fail("must reject before payload"))
    with pytest.raises(ValueError):pub.load_pair(Path("/synthetic/caller.json"),"a"*64)
