"""HH_261008 - Scalar/source/fake-filesystem clock-report checks only; no arrays, rendering, NN, ROS or actual journals."""
from __future__ import annotations

import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/"tools/e2e/report_recorded_ros_clock_comparison_20261008_v1.py"


@pytest.fixture
def report():
    spec=importlib.util.spec_from_file_location("_hh_clock_report_fake_test",SOURCE);value=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=value;spec.loader.exec_module(value);return value


def anchors():
    return [SimpleNamespace(ordinal=n,native_frame=4000+2*n,timestamp_ns=2**63+1+n*100000000,
        provenance=dict(schema="actual.source.bundle",ordinal=n,native_frame=4000+2*n,timestamp_ns=2**63+1+n*100000000,source_sha256="a"*64)) for n in range(918)]


def math_fake():
    def delta(a,b):
        if not(type(a) is int and type(b) is int and 0<a<=b):raise ValueError("actual integer chronology required")
        return (b-a)/1e6
    return dict(delta=delta,milliseconds=lambda s:s*1000,original_and_deferred_diagnostics=lambda row:row["policy_diagnostics"])


def dataset(report,observed=28,failed_last=True):
    index=anchors();direct=[];subscriber=[];published=[];printed=[];witness=[];calls=[];received=[]
    for anchor in index:
        identity=dict(ordinal=anchor.ordinal,native_frame=anchor.native_frame,timestamp_ns=anchor.timestamp_ns)
        direct.append(dict(identity,status="TRAJECTORY"))
        if anchor.ordinal>=observed:continue
        begin=1000000000+anchor.ordinal*1000000000
        marks={key:begin+i*10000000 for i,key in enumerate(report.MARKS[:-2])}
        marks["receipt_publication_entered_ns"]=begin+55000000
        marks["receipt_publication_finished_ns"]=begin+61000000
        status="FAILED_RETAINED" if failed_last and anchor.ordinal==observed-1 else "TRAJECTORY"
        value=dict(identity,status=status,stages=marks,source_selected_callback_provenance=anchor.provenance,
            policy_diagnostics=dict(total_seconds=.009,raw_nn_seconds=.007,owned_recorded_deferred_audit_timing=dict(audit_flush_seconds=.003)),
            actual_ACK_publish_completed_ns=begin+60000000)
        subscriber.append(value);published.append(deepcopy(value));raw=report.encode(value);printed.append((raw,deepcopy(value)))
        provenance=json.dumps(anchor.provenance,sort_keys=True,separators=(",",":"),allow_nan=False).encode()
        witness.append(dict(console_call=anchor.ordinal,binding="ACTUAL_RECEIPT_SOURCE_JOIN",ordinal=anchor.ordinal,native_frame=anchor.native_frame,
            timestamp_ns=anchor.timestamp_ns,publisher_bundle_started_ns=begin,original_receipt_stage_ns=marks["receipt_publication_finished_ns"],
            original_console_returned_ns=begin+70000000,receipt_arg_byte_sha256=hashlib.sha256(raw[:-1]).hexdigest(),
            source_provenance_sha256=hashlib.sha256(provenance).hexdigest(),actual_until_original_console_log_complete_ms=70.))
        if status=="TRAJECTORY":
            for suffix in ("shadow_trajectory","shadow_path"):
                calls.append(dict(ordinal=anchor.ordinal,topic="/planning/portable_e2e/"+suffix,actually_published=True))
                received.append(dict(ordinal=anchor.ordinal,topic="/planning/portable_e2e/"+suffix))
    rows=dict(direct_anchors=direct,subscriber_anchors=subscriber,publisher_anchors=published,subscriber_stdout=printed,
        publisher_stdout=[],subscriber_output_calls=calls,publisher_received_outputs=received,comparison=[dict(ordinal=n) for n in range(918)])
    closing=dict(expected_anchors=918,original_ACK_clock_unchanged=True,original_ACK_clock_includes_later_journal_or_console=False,rows=witness)
    return rows,closing,index


def test_import_has_no_numeric_framework_or_runtime_and_fixed_limits(report):
    assert not any(name in sys.modules for name in ("torch","numpy","matplotlib","rclpy","carla"))
    assert report.fixed()["NN_inferences"]==report.fixed()["training_steps"]==0
    assert report.TOTAL==16*1024**2 and report.CLOSING==2*1024**2
    assert report.SOURCE_LIMIT+2*report.PLAN_LIMIT+report.AUTH_LIMIT+2*report.PNG_LIMIT+report.CSV_LIMIT+report.MPL_CACHE_LIMIT<=report.TOTAL-report.CLOSING
    assert report.META_LIMIT<=report.CLOSING and report.fixed()["local_stage_SUM_is_not_wall"]
    assert report.FIXED_PAIRS==tuple(range(25))


def test_full918_both_observed_and_failed_tail_with_null_unexecuted(report):
    rows,witness,index=dataset(report,28)
    table,summary=report.summarize_arm("V14",rows,witness,index,math_fake())
    assert len(table)==918 and summary["status_counts"]==dict(TRAJECTORY=27,FAILED_RETAINED=1,UNEXECUTED=890)
    assert table[27]["status"]=="FAILED_RETAINED" and table[27]["full_ACK_ms"]==60 and table[27]["console_return_ms"]==70
    assert summary["complete_pairs"]==27 and summary["partial_trajectory"]==0
    assert all(table[28][key] is None for key in (*report.MARKS,*report.PHASES,"raw_NN_ms","full_ACK_ms","console_return_ms"))
    assert table[-1]["timestamp_ns"]>2**63 and table[-1]["source_provenance_sha256"]


def test_partial_trajectory_failed_anchor_not_complete_pair(report):
    rows,witness,index=dataset(report,28)
    rows["subscriber_output_calls"].append(dict(ordinal=27,topic="/planning/portable_e2e/shadow_trajectory",actually_published=True))
    rows["subscriber_output_calls"].append(dict(ordinal=27,topic="/planning/portable_e2e/shadow_path",actually_published=False))
    table,summary=report.summarize_arm("V14",rows,witness,index,math_fake())
    assert table[27]["partial_trajectory"] and not table[27]["complete_output_pair"]
    assert summary["partial_trajectory"]==1 and summary["all_output_calls"]==56


def test_missing_marks_and_unknown_console_stay_null_not_filled(report):
    rows,witness,index=dataset(report)
    del rows["subscriber_anchors"][27]["stages"]["conversion_finished_ns"]
    witness["rows"][27].update(binding="UNKNOWN",actual_until_original_console_log_complete_ms=None)
    table,summary=report.summarize_arm("V14",rows,witness,index,math_fake())
    assert table[27]["converter"] is table[27]["conversion_to_trajectory_calls_finish"] is None
    assert table[27]["console_return_ms"] is table[27]["ACK_to_console_return"] is None
    assert table[27]["full_ACK_ms"]==60 and summary["console_bound"]==27


@pytest.mark.parametrize("fault",["print_sha","source","frame","stamp","begin","console_chronology","unknown_fake_time","duplicate"])
def test_console_actual_byte_identity_provenance_and_chronology_denials(report,fault):
    rows,witness,index=dataset(report);row=witness["rows"][0]
    if fault=="print_sha":row["receipt_arg_byte_sha256"]="0"*64
    elif fault=="source":row["source_provenance_sha256"]="0"*64
    elif fault=="frame":row["native_frame"]+=1
    elif fault=="stamp":row["timestamp_ns"]+=1
    elif fault=="begin":row["publisher_bundle_started_ns"]+=1
    elif fault=="console_chronology":row["original_console_returned_ns"]=1
    elif fault=="unknown_fake_time":row["binding"]="UNKNOWN"
    else:rows["subscriber_stdout"].append(rows["subscriber_stdout"][0]);witness["rows"].append(dict(row,console_call=28))
    with pytest.raises(ValueError):report.console_map(witness,rows["subscriber_stdout"],index)


@pytest.mark.parametrize("role",["direct_anchors","subscriber_anchors","publisher_anchors","comparison"])
def test_all_role_ordinal_and_native_source_joins_not_only_subscriber(report,role):
    rows,witness,index=dataset(report)
    rows[role][0]["ordinal"]=1
    with pytest.raises(ValueError):report.summarize_arm("V14",rows,witness,index,math_fake())


def test_nested_NN_and_audit_not_added_to_phase_total(report):
    rows,witness,index=dataset(report)
    table,_=report.summarize_arm("V14",rows,witness,index,math_fake());row=table[0]
    assert row["policy"]==10 and row["raw_NN_ms"]==7 and row["audit_flush_ms"]==3
    assert sum(row[key] for key in report.PHASES)==row["console_return_ms"]==70
    tree=ast.parse(SOURCE.read_bytes());chart=next(p for p in tree.body if isinstance(p,ast.FunctionDef) and p.name=="chart")
    assert "raw_NN_ms" not in ast.unparse(chart) and "audit_flush_ms" not in ast.unparse(chart)


def test_fixed25_summary_preserves_all26_common_and_two_full918_tables(report):
    a,wa,index=dataset(report,26);b,wb,_=dataset(report,28)
    left,_=report.summarize_arm("V13",a,wa,index,math_fake());right,_=report.summarize_arm("V14",b,wb,index,math_fake())
    result=report.paired_summary(dict(V13=left,V14=right))
    assert len(result["fixed25"])==25 and result["all_common_observed_ordinals"]==list(range(26))
    assert result["all_failures_outside_fixed25_still_retained"] and not result["causal_or_sustained_performance_claimed"]
    right[917]["timestamp_ns"]+=1
    with pytest.raises(ValueError):report.paired_summary(dict(V13=left,V14=right))


def test_pair_denominator_cannot_silently_shrink_to_available_good_rows(report):
    a,wa,index=dataset(report,24);table,_=report.summarize_arm("V13",a,wa,index,math_fake())
    with pytest.raises(ValueError):report.paired_summary(dict(V13=table,V14=deepcopy(table)))


def test_all8_codec_iterators_fully_consumed_including_generator_final_checks(report,monkeypatch,tmp_path):
    calls=[]
    class Codec:
        def prospective_capacity(self):return dict(components={name:{} for name in report.LOG_NAMES})
        def iter_rows(self,path,**kw):
            calls.append((path.name,"start"));yield b'{"ordinal":0}\n';calls.append((path.name,"fully_consumed"))
        def decode_row(self,raw):return json.loads(raw)
    monkeypatch.setattr(report,"read_json",lambda *a,**kw:dict(codec_failure=False,ledgers=[]))
    rows,proof=report.read_ledgers(tmp_path,Codec())
    assert len(proof)==8 and len([c for c in calls if c[1]=="fully_consumed"])==8
    assert rows["subscriber_stdout"][0][0]==b'{"ordinal":0}\n'
    class Failing(Codec):
        def iter_rows(self,path,**kw):yield b'{"ordinal":0}\n';raise ValueError("unindexed failed gzip tail after yielded prefix")
    with pytest.raises(ValueError):report.read_ledgers(tmp_path,Failing())


def test_prior_codec_failure_denied_not_relabelled_prefix_PASS(report,monkeypatch,tmp_path):
    codec=SimpleNamespace(prospective_capacity=lambda:dict(components={n:{} for n in report.LOG_NAMES}),iter_rows=lambda *a,**kw:iter([]),decode_row=json.loads)
    monkeypatch.setattr(report,"read_json",lambda *a,**kw:dict(codec_failure=True,ledgers=[]))
    with pytest.raises(ValueError):report.read_ledgers(tmp_path,codec)


def test_combined_quota_prospective_closing_and_symlink_denial(report,monkeypatch,tmp_path):
    plan=tmp_path/"plan";out=tmp_path/"out";plan.mkdir();out.mkdir()
    monkeypatch.setattr(report.os,"statvfs",lambda _:SimpleNamespace(f_bavail=10*1024**3,f_frsize=1))
    (plan/"one").write_bytes(b"abc");(out/"two").write_bytes(b"defg")
    assert report.budget(roots=(plan,out))==7
    with pytest.raises(ValueError):report.budget(additional=report.TOTAL-report.CLOSING,roots=(plan,out))
    assert report.budget(additional=report.TOTAL-7,closing=True,roots=(plan,out))==7
    (out/"alias").symlink_to(plan/"one")
    with pytest.raises(ValueError):report.budget(roots=(plan,out))


def test_frozen_scalar_gallery_source_exact_function_set_and_no_render_call(report):
    maths=report.gallery_math();assert set(maths)&{"delta","milliseconds","original_and_deferred_diagnostics"}=={"delta","milliseconds","original_and_deferred_diagnostics"}
    assert maths["delta"](100,100000100)==100
    assert report.FIXED_SOURCE_PINS["tools/e2e/owned_lossless_jsonl_20261008_v2.py"]=="f867fc18ff6f85038e3ea4dab8fd4b7b50bba84be8655c4db68bd6eab8eff479"


def test_source_external_authority_before_numeric_renderer_no_runtime_or_replay(report):
    tree=ast.parse(SOURCE.read_bytes());execute=next(p for p in tree.body if isinstance(p,ast.FunctionDef) and p.name=="execute")
    text=ast.unparse(execute)
    assert text.index("external ROOT authority required")<text.index("flock(")<text.index("verify(plan)")<text.index("load_stdlib_source(")<text.index("import matplotlib")
    assert "actual_RViz_screenshot=False" in text and "NN_inferences=0" in text and "training_steps=0" in text
    calls={ast.unparse(p.func) for p in ast.walk(tree) if isinstance(p,ast.Call)}
    assert not any("forward" in name or "optimizer" in name or "torch" in name or "spawn" in name for name in calls)
    assert "matplotlib.use('Agg')" in text and "MPLCONFIGDIR" in text
