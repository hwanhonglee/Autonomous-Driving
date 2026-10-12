"""HH_261002 - Exercise the exact large baseline boundary without changing frozen physical preview08 math."""

from contextlib import contextmanager,ExitStack
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import research_temporal_route_joint_preview08_signed45_v2_town07_20261002 as owner
from test_temporal_route_joint_preview08_signed45_20261002 import baseline_fixture,pins


@pytest.mark.parametrize("external,known",[(False,True),(True,True),(True,False)])
def test_exact32MiB_reader_full_census_and_only_authenticated_external_paths(tmp_path,external,known):
    local=tmp_path/"workspace";local.mkdir();path,report=baseline_fixture(local)
    source=tmp_path/"external_carla";source.write_bytes(b"known-native-source")
    if external:report["source_sha256"][str(source)]=owner.digest(source)
    report["notice"]="x"*(5*1024**2);path.write_text(json.dumps(report))
    with mock.patch.object(owner,"ROOT",local),mock.patch.object(owner,"BASELINE",path),mock.patch.object(owner,"BASELINE_SHA",owner.digest(path)), \
         mock.patch.object(owner.base,"external_sources",return_value={str(source):owner.digest(source)} if known else {}), \
         mock.patch.object(owner.base,"ASSETS",{}):
        if external and not known:
            with pytest.raises(Exception):owner.baseline_proof(pins())
        else:
            result=owner.baseline_proof(pins())
            assert result["all_failed_tail_retained"] and len(result["source_sha256"])==3+int(external)
            assert len(result["prospective_preview_acceleration_guard_applied_to_all_original_baseline_arms"])==3


@pytest.mark.parametrize("kind",["wrongpath","wrongsha","oversize"])
def test_reader_exception_is_only_exact_pinned_baseline(kind,tmp_path):
    path=tmp_path/"report";path.write_text("{}")
    with mock.patch.object(owner,"BASELINE",path),mock.patch.object(owner,"BASELINE_SHA",owner.digest(path)):
        if kind=="wrongpath":
            other=tmp_path/"other";other.write_text("{}")
            with pytest.raises(Exception):owner.exact_baseline_json(other)
        elif kind=="wrongsha":
            path.write_text("changed")
            with pytest.raises(Exception):owner.exact_baseline_json(path)
        else:
            with mock.patch.object(owner,"digest",return_value=owner.BASELINE_SHA), \
                 mock.patch.object(Path,"stat",return_value=SimpleNamespace(st_size=33*1024**2)),pytest.raises(Exception):
                owner.exact_baseline_json(path)


def test_mapping_census_is_exact_order_independent_and_no_pin_exclusion(tmp_path):
    report=dict(source_sha256={str(tmp_path/"a"):"a"*64,str(tmp_path/"b"):"b"*64})
    first=owner.source_census(report)
    second=owner.source_census(dict(source_sha256=dict(reversed(tuple(report["source_sha256"].items())))))
    assert first==second and first["pin_count"]==2
    assert first["full_mapping_archived_in_exact_report_asset"] and first["no_camera_source_pin_dropped"]


def test_generated_plan_no_large_inline_mapping_preserves_all_other_original_fields(tmp_path):
    report=tmp_path/"proof";report.write_bytes(b"x")
    fake=dict(schema="old",baseline_actual_output_source_sha256={str(tmp_path/str(i)):"a"*64 for i in range(31852)},
        controller_target_cap_mps=2.4,total_capture_seconds=45,guard=.1)
    # HH_261002 - Execute the AST-identical old function around a fixed contract stub, not a new physical algorithm.
    baseline=dict(source_sha256=fake["baseline_actual_output_source_sha256"],
        prospective_preview_acceleration_guard_applied_to_all_original_baseline_arms=[])
    with mock.patch.object(owner,"BASELINE",report),mock.patch.object(owner.previous,"_baseline",baseline), \
         mock.patch.object(owner.previous,"_fixed",return_value=dict(controller_target_cap_mps=2.4,total_capture_seconds=45,guard=.1)), \
         mock.patch.object(owner,"external_pins",return_value={}):
        result=owner.fixed_contract("v1",True)
    assert "baseline_actual_output_source_sha256" not in result
    assert result["baseline_complete_source_output_census"]["pin_count"]==31852
    assert len(json.dumps(result))<4*1024**2
    assert result["schema"]==owner.SCHEMA and result["controller_target_cap_mps"]==2.4 and result["guard"]==.1
    assert result["causal_selected_speed_preview_calibration"]["configured_speed_preview_s"]==.8
    assert result["collector_and_physical_preview_calibration_unchanged"]


@pytest.mark.parametrize("trial",[0,1,2])
def test_real_base_preflight_all_versioned_scopes_and_correct_worker_module(tmp_path,trial):
    root=owner.previous;thread=root.previous;joint=thread.previous
    report=tmp_path/"proof";report.write_bytes(b"x")
    baseline=dict(source_sha256={},final_endpoint_sha256=pins(),
        prospective_preview_acceleration_guard_applied_to_all_original_baseline_arms=[])
    model=SimpleNamespace(_research_temporal_checkpoint_sha256="c"*64)
    api=SimpleNamespace(stdout=str(owner.base.EGG)+"/__init__.py\n"+str(owner.base.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n1.8.0\n")
    original_digest=owner.digest
    patches=(mock.patch.object(root,"BASELINE",report),mock.patch.object(owner,"BASELINE",report),
        mock.patch.object(owner,"baseline_proof",return_value=baseline),mock.patch.object(owner,"digest",side_effect=lambda path:
            owner.FAILURE_SHA if Path(path)==owner.FAILURE else original_digest(path)),
        mock.patch.object(root,"digest",return_value=root.FROZEN_OWNER_SHA),mock.patch.object(thread,"evidence_assets",return_value={report:"f"*64}),
        mock.patch.object(thread,"configure_threads"),mock.patch.object(thread.torch,"get_num_threads",return_value=1),
        mock.patch.object(thread.torch,"set_num_threads"),mock.patch.object(joint,"verify_exports",return_value={}),
        mock.patch.object(owner,"runtime_sources",return_value={}),mock.patch.object(owner.base,"verify_plan",return_value=dict(longitudinal_pi=True)),
        mock.patch.object(owner.base.lifecycle,"_assert_no_other_carla"),mock.patch.object(owner.base,"ports_free"),
        mock.patch.object(owner.base.subprocess,"run",return_value=api),mock.patch.object(owner.base,"environment",return_value={}),
        mock.patch.object(owner.live.collector,"load_route",return_value=dict(town="Town07",scenario="straight")))
    with ExitStack() as stack:
        for patch in patches:stack.enter_context(patch)
        with owner.activated(pins()),owner.parent.scoped_trial(trial):
            # HH_261002 - The compatibility wrapper must reach the actual old worker/validator, not only a contract stub.
            with mock.patch.object(owner,"external_pins",return_value={}):
                contract=owner.base.fixed_contract("v1",True)
            assert contract["worker_command"][contract["worker_command"].index("-m")+1]==owner.MODULE
            assert contract["schema"]==owner.SCHEMA and owner.base.live is owner.live
            with mock.patch.object(owner.base,"paths",return_value=(tmp_path/"absent",None)), \
                 mock.patch.object(owner.base,"disk_check",return_value={}),mock.patch.object(owner.base,"load_npz",return_value=model):
                _,result=owner.base.preflight("v1","e"*64)
            assert result["status"]=="PREFLIGHT_PASS_NO_SIMULATOR" and not owner.live._active


@pytest.mark.parametrize("failure",[False,True])
def test_all_original_global_bindings_and_assets_restore(failure):
    names=("batch_paths","MODULE","SCHEMA","baseline_proof","fixed_contract","runtime_sources")
    before={name:getattr(owner.previous,name) for name in names};assets=owner.base.ASSETS
    @contextmanager
    def inherited(p):
        assert owner.previous.baseline_proof is owner.baseline_proof and owner.previous.SCHEMA==owner.SCHEMA
        if failure:raise OSError("original")
        yield
    with mock.patch.object(owner,"digest",return_value=owner.FAILURE_SHA),mock.patch.object(owner.previous,"activated",side_effect=inherited):
        try:
            with owner.activated(pins()):
                with pytest.raises(Exception):
                    with owner.activated(pins()):pass
        except OSError:assert failure
    assert all(getattr(owner.previous,k) is v for k,v in before.items()) and owner.base.ASSETS is assets
    assert not owner._active


def test_frozen_v1_source_tests_collector_remain_byte_exact():
    assert owner.digest(Path(owner.previous.__file__))==owner.FROZEN_OWNER_SHA
    assert owner.digest(owner.ROOT/"learning/tests/test_temporal_route_joint_preview08_signed45_20261002.py")=="6ba38fd375baf21cada915294099964b7e9b64216b479995592c9f9758b75b01"
    assert owner.digest(Path(owner.live.__file__))=="90a3788fa4be92136cb73b2f3413b4ddbbdd2c0a89f55f5998256e1abbba1498"
