"""HH_261007 - Preserve prior pure evidence; validate public matched-GUID normalization and fail-closed guards.

No native compilation, ROS graph, model inference or historical-output mutation.
"""
import ast
import importlib.util
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/"tools/e2e/probe_owned_dds_info_bridge_20261007_v4.py"
C_SOURCE=SOURCE.with_name("owned_dds_info_bridge_20261007_v2.c")
spec=importlib.util.spec_from_file_location("hh_native_probe_pure_v4",SOURCE)
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
    plan=dict(source_sha256={'a':'sealed'},existing_native_binding='x',native_binding={'pin':'stable'},toolchain={},libraries={},preserved_evidence={})
    assert helper.closing_seals(plan)==[{'stage':'sources','error':"ValueError('whole sealed source changed')"}]


def test_json_roundtrip_contract_has_exact_native_types():
    fixed=helper.contract()
    assert json.loads(helper.encode(fixed))==fixed
    assert fixed['node_allowlist']==[list(node) for node in sorted(helper.NODES)]
    assert all(type(node) is list for node in fixed['node_allowlist'])


def test_frozen_v1_source_and_test_preserved():
    pins={ROOT/'tools/e2e/probe_owned_dds_info_bridge_20261007_v1.py':'5e63f89eb74c829478c35492f869912ba507445be8e71ad1836940c02d823fd3',
        ROOT/'learning/tests/test_owned_dds_info_bridge_20261007_v1.py':'4901f7b88b49472e244a216e982d27d536d9aa3ac3497c3e01db0f1bc01ee9a8',
        C_SOURCE.with_name('owned_dds_info_bridge_20261007_v1.c'):'20e8ba59d06c4326d47478f97a6c4c8bfe94cad05df8c579bb9d68d9c25315b3',
        ROOT/'tools/e2e/probe_owned_dds_info_bridge_20261007_v3.py':'758b2d420e413572449977a9f971dfae2ce0d292c12c97b4c64db88e75288de2',
        ROOT/'learning/tests/test_owned_dds_info_bridge_20261007_v3.py':'3b4a84935cfcdc08adcc483c9557242bf9fc66e545f475156453c7d892fce877'}
    assert all(hashlib.sha256(path.read_bytes()).hexdigest()==pin for path,pin in pins.items())
    assert all(str(path.relative_to(ROOT)) in helper.source_pins() for path in pins)


def test_actual_json_plan_shape_admitted_before_any_compiler(tmp_path,monkeypatch):
    # HH_261007 - Exercise the real admission function on file-encoded prospective metadata, not stub its positive branch.
    plan_dir=tmp_path/'plans';plan_path=plan_dir/'ROOT_plan.json'
    monkeypatch.setattr(helper,'PLAN_DIR',plan_dir);monkeypatch.setattr(helper,'PLAN',plan_path)
    fingerprints={'compiler':{'marker':'compiler'},'library':{'marker':'library'},'native':{'marker':'native'},
        'environment':{'sha256':'e'*64}}
    monkeypatch.setattr(helper,'fingerprint',lambda path:fingerprints[str(path)])
    monkeypatch.setattr(helper,'source_pins',lambda:{'five_sealed_source_entries':'s'*64})
    monkeypatch.setattr(helper,'sys',SimpleNamespace(executable='/usr/bin/python3',version_info=(3,10),modules={}))
    for key in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','ROS_LOG_DIR'):
        monkeypatch.setenv(key,str(helper.contract()[key]))
    monkeypatch.setenv('CUDA_VISIBLE_DEVICES','');monkeypatch.setenv('PYTHONNOUSERSITE','1')
    for key in ('LD_PRELOAD','LD_AUDIT','ROS_DISCOVERY_SERVER','CYCLONEDDS_URI','FASTRTPS_DEFAULT_PROFILES_FILE','RMW_FASTRTPS_USE_QOS_FROM_XML'):
        monkeypatch.delenv(key,raising=False)
    plan=dict(schema=helper.SCHEMA,fixed=helper.contract(),source_sha256=helper.source_pins(),
        toolchain={'compiler':fingerprints['compiler']},libraries={'library':fingerprints['library']},preserved_evidence={},
        existing_native_binding='native',native_binding=fingerprints['native'],
        environment_receipt={'path':'environment','sha256':'e'*64})
    pin=helper.write_new(plan_path,plan)
    authority=dict(schema=helper.SCHEMA+'.ROOT_authority',phase='build',plan_sha256=pin,source_sha256=plan['source_sha256'],
        private_existing_RCL_only=True,training_authorized=False,GPU_authorized=False,CARLA_authorized=False,actuation_authorized=False,native_binary_sha256=None)
    grant=helper.write_new(plan_dir/'ROOT_build_authority.json',authority)
    assert helper.admit('build',pin,grant)==plan
    with pytest.raises(ValueError):helper.admit('build','0'*64,grant)


def test_repeat_probe_denied_before_ros_import(tmp_path,monkeypatch):
    (tmp_path/'result.json').write_text('{}\n');monkeypatch.setattr(helper,'OUT',tmp_path)
    with pytest.raises(ValueError,match='fresh one-shot probe'):
        helper.probe({},123)
    source=text('probe')
    assert source.index('fresh one-shot probe')<source.index('import rclpy')


def test_original_lifecycle_and_graph_math_inverse_source():
    old=(ROOT/'tools/e2e/probe_owned_dds_info_bridge_20261007_v3.py').read_text()
    new=SOURCE.read_text()
    old_tree=ast.parse(old);new_tree=ast.parse(new)
    old_functions={node.name:node for node in old_tree.body if isinstance(node,ast.FunctionDef)}
    new_functions={node.name:node for node in new_tree.body if isinstance(node,ast.FunctionDef)}
    changed={'contract','source_pins','prepare','admit','shared_closure','build','closing_seals','probe'}
    assert old_functions.keys()<=new_functions.keys()
    for name in old_functions.keys()-changed:
        assert ast.dump(old_functions[name],include_attributes=False)==ast.dump(new_functions[name],include_attributes=False)
    assert new_functions.keys()-old_functions.keys()=={'source_paths','validate_normalization','json_native_info'}


def test_every_declared_include_exists_and_yaml_package_spelling_is_exact():
    # HH_261007 - Catch the genuine V2 dependency-scan typo before any future compiler invocation.
    assert Path('/opt/ros/humble/include/rcl_yaml_param_parser') in helper.INCLUDES
    assert all(path.is_dir() and path.resolve(strict=True)==path for path in helper.INCLUDES)
    assert 'all declared existing include directories required' in text('prepare')


def native_info():
    handle=2**63+17
    return dict(implementation_identifier='rmw_cyclonedds_cpp',publisher_gid=bytes(range(1,17))+bytes(8),
        raw_rmw_publisher_gid=handle.to_bytes(8,helper.sys.byteorder)+bytes(16),actual_publication_handle=handle,
        dds_reader_guid=bytes(range(16)),dds_reader=123,dds_participant=20,dds_local_entity_count=32,
        dds_topic_name='rt/portable_e2e/owned_native_info_probe/text',dds_type_name='std_msgs::msg::dds_::String_',
        publisher_gid_provenance=helper.PROVENANCE,source_timestamp=2**61+19,received_timestamp=2**61+22,
        publication_sequence_number=2**64-1,reception_sequence_number=2**64-1,from_intra_process=False)


def test_true_native_provenance_scalar_contract():
    info=native_info();helper.validate_normalization(info,'/portable_e2e/owned_native_info_probe/text')
    result=helper.json_native_info(info)
    assert result['raw_rmw_publisher_gid']==info['raw_rmw_publisher_gid'].hex()
    assert result['publisher_gid']==info['publisher_gid'].hex()
    assert result['publication_sequence_number']==2**64-1 and result['source_timestamp']==2**61+19
    assert json.loads(helper.encode(result))==result


@pytest.mark.parametrize('key,value',[
    ('implementation_identifier','rmw_fastrtps_cpp'),('publisher_gid',bytes(23)),
    ('raw_rmw_publisher_gid',bytes(8)),('raw_rmw_publisher_gid',bytes(24)),
    ('raw_rmw_publisher_gid',bytes(8)+b'x'+bytes(15)),('actual_publication_handle',0),
    ('actual_publication_handle',True),('actual_publication_handle',2**64),
    ('actual_publication_handle',17),('publisher_gid',bytes(16)+b'x'+bytes(7)),
    ('publisher_gid_provenance','graph_endpoint_substitution'),('dds_reader_guid',bytes(15)),
    ('dds_reader',True),('dds_participant',0),('dds_local_entity_count',0),
    ('dds_local_entity_count',513),('dds_local_entity_count',True),
    ('dds_topic_name','rt/foreign'),('dds_type_name','')])
def test_malformed_unresolved_or_ambiguous_native_abi_denied(key,value):
    info=native_info();info[key]=value
    with pytest.raises(ValueError):helper.validate_normalization(info,'/portable_e2e/owned_native_info_probe/text')


@pytest.mark.parametrize('bound',[1,512])
def test_declared_whole_entity_bounds_inclusive(bound):
    info=native_info();info['dds_local_entity_count']=bound
    helper.validate_normalization(info,'/portable_e2e/owned_native_info_probe/text')


@pytest.mark.parametrize('token',[
    'MAX_ENTITY_CHILDREN = 256','MAX_LOCAL_ENTITIES = 512','MAX_TREE_DEPTH = 8',
    'DDS_CYCLONEDDS_HANDLE','dds_get_children(entity, NULL, 0)',
    'count != before','depth[cursor] >= MAX_TREE_DEPTH','used >= MAX_LOCAL_ENTITIES',
    'dds_get_parent(children[i]) != entity','reader_count != 1','domain != 93',
    'dds_get_matched_publications(entity, NULL, 0)','dds_get_matched_publication_data(out->reader, out->publication_handle)',
    'dds_builtintopic_free_endpoint(endpoint)','strcmp(endpoint->type_name, out->type_name) == 0',
    'memcmp(&check_guid, &out->reader_guid','memcpy(out->gid, endpoint->key.v',
    '"raw_rmw_publisher_gid"','"actual_publication_handle"'])
def test_real_c_public_api_failclosed_source_seams(token):assert token in C_SOURCE.read_text()


def test_c_never_uses_private_rmw_data_or_python_graph():
    source=C_SOURCE.read_text()
    for forbidden in ('subscription->data','CddsSubscription','get_publishers_info_by_topic','dds_take('):
        assert forbidden not in source
    assert 'memcpy(&out->publication_handle, info->publisher_gid.data' in source
    assert 'Py_BEGIN_ALLOW_THREADS' not in source


def test_resolved_library_keys_and_canonical_direct_link():
    assert helper.DDS_LIBRARY.resolve(strict=True)==helper.DDS_LIBRARY
    assert helper.DDS_LIBRARY in helper.LIBRARIES and 'str(DDS_LIBRARY)' in text('build')
    assert 'result[str(actual)]=fingerprint(actual)' in text('shared_closure')
    assert Path('/opt/ros/humble/include') in helper.INCLUDES


def test_fresh_contract_truthful_normalization_json_roundtrip():
    rule=helper.contract()['publisher_gid_normalization']
    assert rule['public_DDS_only'] and rule['raw_RMW_gid_retained'] and rule['actual_publication_handle_retained']
    assert not rule['private_RMW_data_access'] and not rule['graph_identity_substitution']
    assert rule['matched_endpoint_GUID_bytes']==16 and rule['RMW_zero_padding_bytes']==8
    assert json.loads(helper.encode(helper.contract()))==helper.contract()


def test_failed_predecessor_evidence_pinned_not_modified():
    assert len(helper.PREDECESSOR_EVIDENCE)==4
    assert all(path.is_file() for path in helper.PREDECESSOR_EVIDENCE)
    assert len(helper.source_paths())==10 and len(set(helper.source_paths()))==10
    assert 'preserved_evidence' in text('prepare') and 'plan["preserved_evidence"]' in text('admit')


def test_probe_validates_native_before_graph_equality():
    source=text('probe')
    assert source.index('validate_normalization(info,topic)')<source.index('endpoints=receiver.get_publishers_info_by_topic(topic)')
    assert 'actual_info=json_native_info(info)' in source
    assert 'publisher_gid"]==bytes(endpoints[0].endpoint_gid)' in source
