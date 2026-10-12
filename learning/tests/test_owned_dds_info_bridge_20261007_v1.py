"""HH_261007 - Pure scalar/AST checks only; no native compilation, ROS graph or model."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/"tools/e2e/probe_owned_dds_info_bridge_20261007_v1.py"
C_SOURCE=SOURCE.with_name("owned_dds_info_bridge_20261007_v1.c")
spec=importlib.util.spec_from_file_location("hh_native_probe_pure",SOURCE)
helper=importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)
TREE=ast.parse(SOURCE.read_text())


def function(name):return next(node for node in TREE.body if isinstance(node,ast.FunctionDef) and node.name==name)


def text(name):return ast.get_source_segment(SOURCE.read_text(),function(name))


@pytest.mark.parametrize("token",[
    'exact_imported_type(subscription, "rclpy.subscription", "Subscription")',
    'exact_imported_type(handle, "rclpy._rclpy_pybind11", "Subscription")',
    'raw != Py_False','PyObject_CallMethod(handle, "__enter__"',
    'PyObject_CallMethod(handle, "__exit__"','PyLong_AsVoidPtr(pointer)',
    'rcl_subscription_is_valid(native)','rcl_take(native, ros_message, &info, NULL)',
    'RCL_RET_SUBSCRIPTION_TAKE_FAILED','info.publisher_gid.data',
    'RMW_GID_STORAGE_SIZE','info.publication_sequence_number','info.reception_sequence_number',
    'destroy(ros_message)','PyException_SetContext(cleanup_value, value)'])
def test_genuine_native_api_source_seams(token):
    assert token in C_SOURCE.read_text()


def test_no_unsafe_integer_api_or_gil_release():
    source=C_SOURCE.read_text()
    assert 'METH_O' in source and 'Py_BEGIN_ALLOW_THREADS' not in source
    assert 'PyArg_ParseTuple' not in source and 'get_publishers_info_by_topic' not in source
    assert '"{s:L,s:L,s:K,s:K,s:y#,s:s,s:O}"' in source


def test_fresh_cpu_contract():
    fixed=helper.contract()
    assert fixed['payload_messages']==6 and fixed['ROS_DOMAIN_ID']==93
    assert fixed['no_NN'] and fixed['no_training'] and fixed['no_actuation']
    assert fixed['command_lease_inherited'] and fixed['command_parent_death_signal']=='SIGTERM'
    assert fixed['quota_bytes']==64*1024**2 and fixed['closing_bytes']==8*1024**2


def graph(names=None,topics=None):
    return SimpleNamespace(get_node_names_and_namespaces=lambda:list(helper.NODES) if names is None else names,
        get_topic_names_and_types=lambda:[(name,[kind]) for name,kind in helper.TOPICS.items()] if topics is None else topics)


def test_actual_owned_graph_allowed():helper.graph_guard(graph())


@pytest.mark.parametrize('names',[[],[('foreign','/')],[('owned_dds_bridge_tx','/')]*2])
def test_missing_foreign_duplicate_graph_denied(names):
    with pytest.raises(ValueError):helper.graph_guard(graph(names))


@pytest.mark.parametrize('topics',[[('/control/command/control_cmd',['autoware_control_msgs/msg/Control'])],
    [('/portable_e2e/owned_native_info_probe/text',['sensor_msgs/msg/CompressedImage'])]])
def test_unknown_actuator_or_wrong_type_denied(topics):
    with pytest.raises(ValueError):helper.graph_guard(graph(topics=topics))


def test_discovery_subset_only():
    helper.graph_guard(graph([('owned_dds_bridge_tx','/')]),False)
    with pytest.raises(ValueError):helper.graph_guard(graph([('foreign','/')]),False)


def test_json_create_only_and_raw_pin(tmp_path):
    path=tmp_path/'receipt.json';pin=helper.write_new(path,{'actual_uint64':2**64-1,'actual_ns':2**61+7})
    assert helper.json_read(path,pin)['actual_uint64']==2**64-1
    with pytest.raises(FileExistsError):helper.write_new(path,{})
    with pytest.raises(ValueError):helper.json_read(path,'0'*64)


@pytest.mark.parametrize('value',['{"a":1,"a":2}','{"a":NaN}','{"a":Infinity}'])
def test_malformed_scalar_metadata_denied(tmp_path,value):
    path=tmp_path/'invalid.json';path.write_text(value)
    with pytest.raises(ValueError):helper.json_read(path)


def test_dependency_actual_source_required(tmp_path):
    path=tmp_path/'dep.d';path.write_text('target: /usr/include/stdio.h\n')
    with pytest.raises(ValueError):helper.parse_dependencies(path)


def test_dependency_source_parser_and_whole_fingerprint(tmp_path,monkeypatch):
    path=tmp_path/'dep.d';path.write_text('target: '+str(C_SOURCE)+' \\\n /usr/include/stdio.h\n')
    result=helper.parse_dependencies(path)
    assert str(C_SOURCE) in result and str(Path('/usr/include/stdio.h').resolve()) in result
    assert all(len(info['sha256'])==64 for info in result.values())


def test_before_actual_compile_same_flags():
    source=text('build')
    assert source.count('*plan["compiler_flags"]')==2
    assert 'after==before' in source and '"-MD"' in source and '"-M"' in source


def test_lease_pdeath_and_generation_guard_source():
    source=text('run_command')
    assert 'pass_fds=(lease_fd,)' in source and 'libc.prctl(1,int(signal.SIGTERM)' in source
    assert 'os.getppid()!=parent_pid' in source and 'proc_identity(parent_pid)==owner' in source
    assert source.index('try:\n            process=subprocess.Popen')<source.index('proc_identity(process.pid)')
    assert 'finally:' in source and 'cleanup_command(process,identity)' in source


def test_cleanup_only_fresh_owned_sessions():
    source=text('cleanup_command')
    assert 'leader==identity' in source and 'proc_identity(member["pid"])==member' in source
    assert 'os.killpg(identity["pgid"],sig)' in source and 'remaining' in source
    assert 'killall' not in source and 'pkill' not in source


def test_whole_failed_ledger_and_real_payload_hash_source():
    source=text('probe')
    assert 'for i in range(6)' in source and '"UNEXECUTED"' in source and 'status="FAILED"' in source
    assert 'actual_hash=hashlib.sha256(serialize_message(actual)).hexdigest()' in source
    assert 'actual_CDR_sha256=actual_hash' in source and 'verified==6' in source
    assert 'closing.extend(closing_seals' in source and 'closing_errors=closing' in source
    assert 'NN_inferences=0' in source and 'training_steps=0' in source


def test_clean_discovery_loader_override_denials():
    source=text('admit')
    for key in ('LD_PRELOAD','LD_AUDIT','ROS_DISCOVERY_SERVER','CYCLONEDDS_URI','FASTRTPS_DEFAULT_PROFILES_FILE'):
        assert key in source
    assert '"CUDA_VISIBLE_DEVICES")==""' in source and '"PYTHONNOUSERSITE")=="1"' in source


def test_full_loop_graph_checks_and_closing_failure_survival():
    source=text('probe')
    assert source.count('graph_guard(receiver')>=3
    assert 'except BaseException as failure:closing.append' in source
    assert source.index('finally:')<source.index('write_new(OUT/"result.json"')
    main=text('main')
    assert 'build_failure.json' in main and 'signal.alarm(0)' in main and 'lock.close()' in main


def test_closing_drift_is_failure_not_new_seal(monkeypatch):
    monkeypatch.setattr(helper,'source_pins',lambda:{'a':'changed'})
    monkeypatch.setattr(helper,'fingerprint',lambda path:{'pin':'stable'})
    plan=dict(source_sha256={'a':'sealed'},existing_native_binding='x',native_binding={'pin':'stable'},toolchain={},libraries={})
    assert helper.closing_seals(plan)==[{'stage':'sources','error':"ValueError('whole sealed source changed')"}]
