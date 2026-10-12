"""HH_261008 - Retain scalar provider/clock checks and cover full V16b/V18c subsets and native-call marks without arrays or rendering."""
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
SOURCE=ROOT/"tools/e2e/report_recorded_ros_clock_comparison_20261008_v3.py"


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
        CDR={"/fake/input/"+str(n):"a"*64 for n in range(19)}
        direct.append(dict(identity,status="TRAJECTORY",input_CDR_sha256=CDR))
        if anchor.ordinal>=observed:continue
        begin=1000000000+anchor.ordinal*1000000000
        marks={key:begin+i*10000000 for i,key in enumerate(report.MARKS[:-2])}
        marks["receipt_publication_entered_ns"]=begin+55000000
        marks["receipt_publication_finished_ns"]=begin+61000000
        status="FAILED_RETAINED" if failed_last and anchor.ordinal==observed-1 else "TRAJECTORY"
        value=dict(identity,status=status,stages=marks,source_selected_callback_provenance=anchor.provenance,
            actual_received_input_CDR_sha256=CDR,published_input_CDR_sha256=CDR,
            policy_diagnostics=dict(total_seconds=.009,raw_nn_seconds=.007,owned_recorded_deferred_audit_timing=dict(audit_flush_seconds=.003),
                owned_recorded_numerical_event=dict(tensor_sha256={"input"+str(n):"a"*64 for n in range(9)},
                    raw_tensor_sha256={"head"+str(n):"b"*64 for n in range(5)})),
            actual_ACK_publish_completed_ns=begin+60000000)
        subscriber.append(value);published.append(deepcopy(value));raw=report.encode(value);printed.append((raw,deepcopy(value)))
        provenance=json.dumps(anchor.provenance,sort_keys=True,separators=(",",":"),allow_nan=False).encode()
        witness.append(dict(console_call=anchor.ordinal,binding="ACTUAL_RECEIPT_SOURCE_JOIN",ordinal=anchor.ordinal,native_frame=anchor.native_frame,
            timestamp_ns=anchor.timestamp_ns,publisher_bundle_started_ns=begin,original_receipt_stage_ns=marks["receipt_publication_finished_ns"],
            original_console_returned_ns=begin+70000000,receipt_arg_byte_sha256=hashlib.sha256(raw[:-1]).hexdigest(),
            source_provenance_sha256=hashlib.sha256(provenance).hexdigest(),actual_until_original_console_log_complete_ms=70.))
        if status=="TRAJECTORY":
            for suffix in ("shadow_trajectory","shadow_path"):
                entered=begin+(45000000 if suffix=="shadow_trajectory" else 49000000)
                calls.append(dict(ordinal=anchor.ordinal,topic="/planning/portable_e2e/"+suffix,actually_published=True,CDR_sha256="c"*64,
                    publisher_bundle_started_ns=begin,actual_publish_entered_ns=entered,actual_publish_completed_ns=entered+1000000))
                received.append(dict(ordinal=anchor.ordinal,topic="/planning/portable_e2e/"+suffix,sha256="c"*64,timestamp_ns=anchor.timestamp_ns))
    rows=dict(direct_anchors=direct,subscriber_anchors=subscriber,publisher_anchors=published,subscriber_stdout=printed,
        publisher_stdout=[],subscriber_output_calls=calls,publisher_received_outputs=received,
        comparison=[dict(ordinal=n,tensor_equal=True,raw_equal=True,direct_status="TRAJECTORY",DDS_status=subscriber[n]["status"]) if n<observed else dict(ordinal=n,status="UNEXECUTED") for n in range(918)])
    closing=dict(expected_anchors=918,original_ACK_clock_unchanged=True,original_ACK_clock_includes_later_journal_or_console=False,rows=witness)
    return rows,closing,index


def test_import_has_no_numeric_framework_or_runtime_and_fixed_limits(report):
    assert not any(name in sys.modules for name in ("torch","numpy","matplotlib","rclpy","carla"))
    assert report.fixed()["NN_inferences"]==report.fixed()["training_steps"]==0
    assert report.TOTAL==16*1024**2 and report.CLOSING==2*1024**2
    assert report.SOURCE_LIMIT+2*report.PLAN_LIMIT+report.AUTH_LIMIT+2*report.PNG_LIMIT+report.CSV_LIMIT+report.MPL_CACHE_LIMIT<=report.TOTAL-report.CLOSING
    assert report.META_LIMIT<=report.CLOSING and report.fixed()["local_stage_SUM_is_not_wall"]
    assert report.fixed()["fixed_paired_ordinals"] is None
    assert list(report.ARMS)==["V16b","V18c"]


def test_full918_both_observed_and_failed_tail_with_null_unexecuted(report):
    rows,witness,index=dataset(report,28)
    table,summary=report.summarize_arm("V18c",rows,witness,index,math_fake())
    assert len(table)==918 and summary["status_counts"]==dict(TRAJECTORY=27,FAILED_RETAINED=1,UNEXECUTED=890)
    assert table[27]["status"]=="FAILED_RETAINED" and table[27]["full_ACK_ms"]==60 and table[27]["console_return_ms"]==70
    assert summary["complete_pairs"]==27 and summary["partial_trajectory"]==0
    assert all(table[28][key] is None for key in (*report.MARKS,*report.PHASES,"raw_NN_ms","full_ACK_ms","console_return_ms"))
    assert table[-1]["timestamp_ns"]>2**63 and table[-1]["source_provenance_sha256"]


def test_partial_trajectory_failed_anchor_not_complete_pair(report):
    rows,witness,index=dataset(report,28)
    begin=rows["subscriber_anchors"][27]["stages"]["publisher_bundle_started_ns"]
    rows["subscriber_output_calls"].append(dict(ordinal=27,topic="/planning/portable_e2e/shadow_trajectory",actually_published=True,
        CDR_sha256="c"*64,publisher_bundle_started_ns=begin,actual_publish_entered_ns=begin+93000000,actual_publish_completed_ns=begin+94000000))
    rows["subscriber_output_calls"].append(dict(ordinal=27,topic="/planning/portable_e2e/shadow_path",actually_published=False,
        CDR_sha256="d"*64,publisher_bundle_started_ns=begin,actual_publish_entered_ns=begin+101000000,actual_publish_completed_ns=None))
    rows["publisher_received_outputs"].append(dict(ordinal=27,topic="/planning/portable_e2e/shadow_trajectory",sha256="c"*64,timestamp_ns=index[27].timestamp_ns))
    rows["subscriber_anchors"][27]["actual_ACK_publish_completed_ns"]=begin+115000000
    rows["subscriber_anchors"][27]["stages"].pop("trajectory_publication_finished_ns")
    witness["rows"][27].update(binding="UNKNOWN",actual_until_original_console_log_complete_ms=None)
    table,summary=report.summarize_arm("V18c",rows,witness,index,math_fake())
    assert table[27]["partial_trajectory"] and not table[27]["complete_output_pair"]
    assert summary["partial_trajectory"]==1 and summary["all_output_calls"]==56
    assert table[27]["trajectory_call_finish_ms"]==94 and table[27]["path_call_entry_ms"]==101
    assert table[27]["path_call_finish_ms"] is table[27]["path_call_finish_ns"] is None


def test_missing_marks_and_unknown_console_stay_null_not_filled(report):
    rows,witness,index=dataset(report)
    del rows["subscriber_anchors"][27]["stages"]["conversion_finished_ns"]
    witness["rows"][27].update(binding="UNKNOWN",actual_until_original_console_log_complete_ms=None)
    table,summary=report.summarize_arm("V18c",rows,witness,index,math_fake())
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
    with pytest.raises(ValueError):report.summarize_arm("V18c",rows,witness,index,math_fake())


def test_nested_NN_and_audit_not_added_to_phase_total(report):
    rows,witness,index=dataset(report)
    table,_=report.summarize_arm("V18c",rows,witness,index,math_fake());row=table[0]
    assert row["policy"]==10 and row["raw_NN_ms"]==7 and row["audit_flush_ms"]==3
    assert sum(row[key] for key in report.PHASES)==row["console_return_ms"]==70
    tree=ast.parse(SOURCE.read_bytes());chart=next(p for p in tree.body if isinstance(p,ast.FunctionDef) and p.name=="chart")
    assert "raw_NN_ms" not in ast.unparse(chart) and "audit_flush_ms" not in ast.unparse(chart)


def test_actual_all_common_summary_derived_from_source_CDR_status_and_received_pairs(report):
    a,wa,index=dataset(report,26);b,wb,_=dataset(report,28)
    left,_=report.summarize_arm("V16b",a,wa,index,math_fake());right,_=report.summarize_arm("V18c",b,wb,index,math_fake())
    result=report.paired_summary(dict(V16b=left,V18c=right))
    assert len(result["all_common_observed"])==26 and result["all_common_observed_ordinals"]==list(range(26))
    assert result["same_source_CDR_status_complete_pair_ordinals"]==list(range(25))
    assert result["same_nine_tensor_complete_pair_ordinals"]==result["same_received_output_CDR_complete_pair_ordinals"]==list(range(25))
    assert result["complete_pair_statistics"]["V18c"]["full_ACK_ms"]["n"]==25
    assert result["all1836_status_rows_retained"] and not result["causal_or_sustained_performance_claimed"]
    right[917]["timestamp_ns"]+=1
    with pytest.raises(ValueError):report.paired_summary(dict(V16b=left,V18c=right))


def test_no_minimum25_or_good_row_filter_and_empty_actual_intersection_denied(report):
    a,wa,index=dataset(report,24);table,_=report.summarize_arm("V16b",a,wa,index,math_fake())
    paired=report.paired_summary(dict(V16b=table,V18c=deepcopy(table)))
    assert paired["all_common_observed_ordinals"]==list(range(24))
    assert paired["same_source_CDR_status_complete_pair_ordinals"]==list(range(23))
    missing=deepcopy(table)
    for row in missing:row["status"]="UNEXECUTED"
    with pytest.raises(ValueError):report.paired_summary(dict(V16b=table,V18c=missing))


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


def test_v3_tradeoff_keeps_whole_budget_and_original_clocks(report):
    assert report.SCHEMA.endswith(".v3")
    assert report.SOURCE_LIMIT==8*1024**2 and report.PNG_LIMIT==1024**2
    assert report.TOTAL==16*1024**2 and report.CLOSING==2*1024**2
    assert report.CSV_LIMIT==2*1024**2 and report.META_LIMIT==1024**2
    assert report.PLAN_LIMIT==256*1024 and report.AUTH_LIMIT==report.MPL_CACHE_LIMIT==512*1024
    assert report.fixed()["expected_anchors_per_arm"]==918 and report.fixed()["original_ACK_deadline_ms"]==100
    assert report.fixed()["sampled_wall_limit_seconds"]==600
    assert report.fixed()["selected_plotting_provider_entries_config_and_font_sealed"]
    assert not report.fixed()["complete_transitive_environment_equivalence_claimed"]
    assert all(report.FIXED_SOURCE_PINS[key]==pin for key,pin in report.OLD_V1_PINS.items())
    assert all(report.FIXED_SOURCE_PINS[key]==pin for key,pin in report.OLD_V2_PINS.items())


def provider_fixture(report,monkeypatch,tmp_path):
    origins={name:str(tmp_path/(name+"_init.py")) for name in ("numpy","matplotlib")}
    paths=[*origins.values(),str(tmp_path/"native_entry.so"),str(tmp_path/"matplotlibrc"),str(tmp_path/"DejaVuSans.ttf")]
    for number,path in enumerate(paths):Path(path).write_bytes(("fake provider bytes "+str(number)).encode())
    monkeypatch.setattr(report,"PROVIDER_ORIGINS",origins);monkeypatch.setattr(report,"PROVIDER_FILES",tuple(paths))
    monkeypatch.setattr(report,"PROVIDER_CONFIG",Path(paths[-2]));monkeypatch.setattr(report,"PROVIDER_FONT",Path(paths[-1]))
    monkeypatch.setattr(report.importlib.util,"find_spec",lambda name:SimpleNamespace(origin=origins[name]))
    monkeypatch.setattr(report.importlib.metadata,"version",lambda name:report.PROVIDER_VERSIONS[name])
    return origins,paths


def test_prepare_provider_hash_is_top_level_spec_and_selected_bytes_only(report,monkeypatch,tmp_path):
    origins,paths=provider_fixture(report,monkeypatch,tmp_path);called=[]
    monkeypatch.setattr(report.importlib.util,"find_spec",lambda name:called.append(name) or SimpleNamespace(origin=origins[name]))
    proof=report.provider_hash()
    assert called==["numpy","matplotlib"] and all("." not in name for name in called)
    assert proof["module_origins"]==origins and set(proof["selected_sha256"])==set(paths)
    assert all(proof["selected_sha256"][path]==hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in paths)
    assert "NOT_complete_transitive_environment_equivalence" in proof["scope"]
    assert not any(name in sys.modules for name in ("numpy","matplotlib","torch","rclpy","carla"))


@pytest.mark.parametrize("fault",["missing_spec","wrong_origin","wrong_version","symlink"])
def test_selected_provider_denials_without_actual_package_import(report,monkeypatch,tmp_path,fault):
    origins,paths=provider_fixture(report,monkeypatch,tmp_path)
    if fault=="missing_spec":monkeypatch.setattr(report.importlib.util,"find_spec",lambda name:None)
    elif fault=="wrong_origin":monkeypatch.setattr(report.importlib.util,"find_spec",lambda name:SimpleNamespace(origin=str(tmp_path/"shadow.py")))
    elif fault=="wrong_version":monkeypatch.setattr(report.importlib.metadata,"version",lambda name:"0")
    else:
        alias=tmp_path/"alias.so";alias.symlink_to(Path(paths[2]));monkeypatch.setattr(report,"PROVIDER_FILES",tuple([*paths,str(alias)]))
    with pytest.raises(ValueError):report.provider_hash()


def test_fake_provider_byte_drift_changes_receipt_not_version_or_origin(report,monkeypatch,tmp_path):
    origins,paths=provider_fixture(report,monkeypatch,tmp_path);before=report.provider_hash()
    Path(paths[2]).write_bytes(b"new fake native provider entry")
    after=report.provider_hash()
    assert after["module_origins"]==before["module_origins"] and after["versions"]==before["versions"]
    assert after["selected_sha256"]!=before["selected_sha256"]


def test_provider_table_in_plan_authority_preimport_and_closing_source_guards(report):
    tree=ast.parse(SOURCE.read_bytes());functions={node.name:node for node in tree.body if isinstance(node,ast.FunctionDef)}
    tables=ast.unparse(functions["tables"]);verify=ast.unparse(functions["verify"]);execute=ast.unparse(functions["execute"])
    assert "provider_hash=provider_hash()" in tables
    assert "'provider_hash'" in verify and "tables()" in verify
    assert "provider_hash=plan['provider_hash']" in execute
    assert execute.index("external ROOT authority required")<execute.index("verify(plan)")<execute.index("import matplotlib")
    assert execute.index("MATPLOTLIBRC")<execute.index("import matplotlib")
    assert execute.index("imported_provider_receipt(")<execute.index("chart(")
    assert execute.rindex("verify(plan)")>execute.index("chart(")
    assert "selected_plotting_provider_receipt=provider_receipt" in execute
    assert "font_manager.findfont" in ast.unparse(functions["imported_provider_receipt"])
    assert "import_module" not in ast.unparse(functions["provider_hash"])


def imported_fixture(report,monkeypatch):
    # HH_261008 - Provided scalar module stand-ins do not import or exercise a numeric package or font renderer.
    proof=dict(selected_sha256={path:"a"*64 for path in report.PROVIDER_FILES})
    monkeypatch.setattr(report,"provider_hash",lambda:deepcopy(proof))
    for name,path in report.PROVIDER_MODULE_FILES.items():
        module=SimpleNamespace(__file__=path)
        if name in report.PROVIDER_VERSIONS:module.__version__=report.PROVIDER_VERSIONS[name]
        monkeypatch.setitem(sys.modules,name,module)
    matplotlib=sys.modules["matplotlib"]
    matplotlib.get_data_path=lambda:str(report.PROVIDER_DATA)
    matplotlib.matplotlib_fname=lambda:str(report.PROVIDER_CONFIG)
    fonts=SimpleNamespace(FontProperties=lambda **kw:kw,findfont=lambda *a,**kw:str(report.PROVIDER_FONT))
    return dict(provider_hash=proof),SimpleNamespace(rcParams={}),fonts


def test_provided_loaded_origins_versions_config_font_and_hash_join(report,monkeypatch):
    plan,plt,fonts=imported_fixture(report,monkeypatch)
    receipt=report.imported_provider_receipt(plan,plt,fonts)
    assert receipt["module_origins"]==report.PROVIDER_MODULE_FILES
    assert receipt["selected_font"]==str(report.PROVIDER_FONT)
    assert not receipt["transitive_environment_equivalence_claimed"]
    assert plt.rcParams=={"font.family":["DejaVu Sans"],"text.usetex":False}


@pytest.mark.parametrize("name",[
    "numpy","matplotlib","numpy.core._multiarray_umath","matplotlib.pyplot",
    "matplotlib.backends.backend_agg","matplotlib.backends._backend_agg",
    "matplotlib.font_manager","matplotlib.ft2font","matplotlib.rcsetup",
])
def test_each_selected_loaded_origin_mismatch_denied_from_scalar_standins(report,monkeypatch,name):
    plan,plt,fonts=imported_fixture(report,monkeypatch)
    sys.modules[name].__file__="/unsealed/provider.py"
    with pytest.raises(ValueError):report.imported_provider_receipt(plan,plt,fonts)


@pytest.mark.parametrize("fault",["missing_loaded","version","data","config","font","hash"])
def test_provided_loaded_provider_contract_failures_not_complete_render(report,monkeypatch,fault):
    plan,plt,fonts=imported_fixture(report,monkeypatch)
    if fault=="missing_loaded":monkeypatch.delitem(sys.modules,"numpy")
    elif fault=="version":sys.modules["matplotlib"].__version__="0"
    elif fault=="data":sys.modules["matplotlib"].get_data_path=lambda:"/usr/share"
    elif fault=="config":sys.modules["matplotlib"].matplotlib_fname=lambda:"/etc/hosts"
    elif fault=="font":fonts.findfont=lambda *a,**kw:"/etc/hosts"
    else:plan["provider_hash"]={"wrong":"a"*64}
    with pytest.raises(ValueError):report.imported_provider_receipt(plan,plt,fonts)


def test_full31_and146_prefixes_keep_failed_context_and_all1836_rows(report):
    left,wl,index=dataset(report,31);right,wr,_=dataset(report,146)
    a,sa=report.summarize_arm("V16b",left,wl,index,math_fake());b,sb=report.summarize_arm("V18c",right,wr,index,math_fake())
    pair=report.paired_summary(dict(V16b=a,V18c=b))
    assert len(a)+len(b)==1836 and sa["UNEXECUTED"]==887 and sb["UNEXECUTED"]==772
    assert pair["all_common_observed_ordinals"]==list(range(31))
    assert pair["all_common_observed"][-1]["reference_status"]=="FAILED_RETAINED"
    assert a[917]["full_ACK_ms"] is b[917]["path_call_finish_ms"] is None
    assert a[0]["input_ready_ms"]==10 and a[0]["trajectory_call_entry_ms"]==45 and a[0]["path_call_finish_ms"]==50
    assert sa["complete_received_pairs"]==30 and sb["complete_received_pairs"]==145


@pytest.mark.parametrize("changed",["input_CDR_receipts_sha256","NN_input_receipts_sha256","received_output_CDR_receipts_sha256","status"])
def test_common_context_or_status_changes_are_classified_not_silently_excluded(report,changed):
    rows,witness,index=dataset(report,4);left,_=report.summarize_arm("V16b",rows,witness,index,math_fake());right=deepcopy(left)
    right[0][changed]="REJECTED" if changed=="status" else "0"*64
    result=report.paired_summary(dict(V16b=left,V18c=right))
    assert result["all_common_observed_ordinals"]==[0,1,2,3] and len(result["all_common_observed"])==4
    if changed in ("input_CDR_receipts_sha256","status"):assert result["same_source_CDR_status_complete_pair_ordinals"]==[1,2]
    elif changed=="NN_input_receipts_sha256":
        assert result["same_source_CDR_status_complete_pair_ordinals"]==[0,1,2]
        assert result["same_nine_tensor_complete_pair_ordinals"]==[1,2]
    else:
        assert result["same_source_CDR_status_complete_pair_ordinals"]==[0,1,2]
        assert result["same_received_output_CDR_complete_pair_ordinals"]==[1,2]


@pytest.mark.parametrize("fault",["incoming_CDR","published_CDR","nine_input_count","five_head_count","publisher_prefix","unobserved_call"])
def test_actual_full_input_and_role_receipt_guards_are_nonvacuous(report,fault):
    rows,witness,index=dataset(report,4)
    if fault=="incoming_CDR":rows["subscriber_anchors"][0]["actual_received_input_CDR_sha256"]={"fake":"z"*64}
    elif fault=="published_CDR":rows["publisher_anchors"][0]["published_input_CDR_sha256"]={"fake":"z"*64}
    elif fault=="nine_input_count":rows["subscriber_anchors"][0]["policy_diagnostics"]["owned_recorded_numerical_event"]["tensor_sha256"].pop("input0")
    elif fault=="five_head_count":rows["subscriber_anchors"][0]["policy_diagnostics"]["owned_recorded_numerical_event"]["raw_tensor_sha256"].pop("head0")
    elif fault=="publisher_prefix":rows["publisher_anchors"].pop()
    else:rows["subscriber_output_calls"][0]["ordinal"]=4
    with pytest.raises(ValueError):report.summarize_arm("V18c",rows,witness,index,math_fake())


@pytest.mark.parametrize("fault",["publisher_begin","float_mark","reverse_clock","after_ACK","received_stamp","received_CDR","duplicate_received","duplicate_call","unpublished_received"])
def test_native_call_clock_and_actual_received_byte_join_fail_closed(report,fault):
    rows,witness,index=dataset(report,4);call=rows["subscriber_output_calls"][0];rx=rows["publisher_received_outputs"][0]
    if fault=="publisher_begin":call["publisher_bundle_started_ns"]+=1
    elif fault=="float_mark":call["actual_publish_entered_ns"]=float(call["actual_publish_entered_ns"])
    elif fault=="reverse_clock":call["actual_publish_completed_ns"]=call["actual_publish_entered_ns"]-1
    elif fault=="after_ACK":call["actual_publish_completed_ns"]=rows["subscriber_anchors"][0]["actual_ACK_publish_completed_ns"]+1
    elif fault=="received_stamp":rx["timestamp_ns"]+=1
    elif fault=="received_CDR":rx["sha256"]="0"*64
    elif fault=="duplicate_received":rows["publisher_received_outputs"].append(deepcopy(rx))
    elif fault=="duplicate_call":rows["subscriber_output_calls"].append(deepcopy(call))
    else:call["actually_published"]=False
    with pytest.raises(ValueError):report.summarize_arm("V18c",rows,witness,index,math_fake())


def test_current_parents_and_frozen_v2_provider_budget_math_are_prospective(report):
    assert report.ARMS["V16b"]["result"]=="c3e3c0dc7618c5e481faa75e3b627003503ed5f87600aa3ba4397da426b0f528"
    assert report.ARMS["V18c"]["result"]=="951a27e7c95b65f0c9b031804d01b2917af78979fbd8a7d0342b7a52affbe355"
    assert report.parent_paths("V16b")[0].name=="temporal_recorded_ros_shadow_v16b"
    assert report.parent_paths("V18c")[1].name=="temporal_recorded_ros_shadow_v18c"
    assert report.SOURCE_LIMIT+2*report.PLAN_LIMIT+report.AUTH_LIMIT+2*report.PNG_LIMIT+report.CSV_LIMIT+report.MPL_CACHE_LIMIT+report.CLOSING<=report.TOTAL
    assert report.META_LIMIT<=report.CLOSING
    for name,pin in report.OLD_V2_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin


def test_dense_tick_labels_do_not_filter_any_measured_interval_or_add_nested_NN(report):
    tree=ast.parse(SOURCE.read_bytes());chart=next(p for p in tree.body if isinstance(p,ast.FunctionDef) and p.name=="chart");text=ast.unparse(chart)
    assert "for row in observed" in text and "set_yticks" in text and "labelled" in text
    assert "trajectory_call_finish_ms" in text and "path_call_finish_ms" in text
    assert "raw_NN_ms" not in text and "audit_flush_ms" not in text
    assert "r['status'] != 'UNEXECUTED'" in text and "FAILED_RETAINED" in text
