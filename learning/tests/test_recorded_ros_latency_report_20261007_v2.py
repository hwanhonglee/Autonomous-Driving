"""HH_261007 - Pure fake timing checks; no plots, NN or actual journals are read."""
import importlib.util
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('hh_latency_report_v2',Path(__file__).resolve().parents[2]/'tools/e2e/report_recorded_ros_latency_20261007_v2.py')
report=importlib.util.module_from_spec(spec);spec.loader.exec_module(report)


@pytest.mark.parametrize('value',[True,-1,float('nan'),float('inf'),'1',None])
def test_bad_duration_denied(value):
    with pytest.raises(ValueError):report.milliseconds(value)


def test_measured_duration_preserved():
    assert report.milliseconds(.1)==100 and report.milliseconds(0)==0
    assert report.delta(2**61+3,2**61+100000003)==100


@pytest.mark.parametrize('start,end',[(False,1),(0,1),(2,1),(1.,2),(1,2.)])
def test_noninteger_or_backwards_clock_denied(start,end):
    with pytest.raises(ValueError):report.delta(start,end)


@pytest.mark.parametrize('raw',['{"a":1,"a":2}','{"a":NaN}','{"a":Infinity}'])
def test_ambiguous_metadata_denied(raw):
    with pytest.raises(ValueError):report.metadata(raw)


def test_truncated_journals_not_full_denominator():
    with pytest.raises(ValueError):report.summarize([],[],[],[])


def test_no_unexecuted_zero_latency_or_screenshot_claim():
    text=Path(report.__file__).read_text()
    assert "status='UNEXECUTED'" in text and 'actual_RViz_screenshot=False' in text
    assert 'audit_flush_duration_separately_measured=True' in text and 'formal_main_approved=0' in text


def test_failed_ACK_recovers_only_same_anchor_original_timer():
    timing={'ACK_completion_includes_audit_cost':True,'audit_flush_seconds':.009}
    row={'timestamp_ns':123,'policy_diagnostics':{'owned_recorded_deferred_audit_timing':timing},
        'actual_policy_result_before_transport_failure':{'anchor_ns':123,'diagnostics':{'total_seconds':.05}}}
    actual=report.original_and_deferred_diagnostics(row)
    assert actual['total_seconds']==.05 and actual['owned_recorded_deferred_audit_timing']==timing
    assert 'total_seconds' not in row['policy_diagnostics']
    row['actual_policy_result_before_transport_failure']['anchor_ns']=124
    with pytest.raises(ValueError):report.original_and_deferred_diagnostics(row)


def test_missing_original_timer_or_excluded_audit_cost_denied():
    with pytest.raises(ValueError):report.original_and_deferred_diagnostics({'timestamp_ns':123})
    with pytest.raises(ValueError):report.original_and_deferred_diagnostics({'timestamp_ns':123,
        'policy_diagnostics':{'total_seconds':.05,'owned_recorded_deferred_audit_timing':{'ACK_completion_includes_audit_cost':False}}})
