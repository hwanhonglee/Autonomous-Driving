"""HH_261008 - Match the exact execute closing provider call; preserve V21c failure and reject undeclared AST changes.

The frozen V20 tests are preserved and source-pinned. These added fixtures use
only synthetic modules/parameters with scalar metadata; they are not evidence
that the installed Torch compiler or tensor aliases work numerically.
"""
from __future__ import annotations
import ast
from collections import OrderedDict
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import MethodType, ModuleType, SimpleNamespace
import pytest

ROOT=Path(__file__).resolve().parents[2]
NEW_FUNCTIONS={"read_v20_source_parent","v21_source_archive_mapping","verify_v21_source_archives","verify_torchscript_provider_sources",
    "script_feature_attribute_snapshot","script_original_module_structure","script_parameter_receipts","script_compiled_alias_receipts",
    "verify_script_features","_script_features_forward","restore_script_feature_binding","_install_script_features","install_script_features",
    "ScriptFeaturesBoundAdapter","close_script_features","restore_script_compiler_metadata","restore_script_compile_context"}
SCRIPT_EXPRESSIONS={
    "scripted_features_only":"True","script_calls_per_model":"1","scripted_feature_children":"12","scripted_feature_parameters":"12",
    "single_private_features_forward_binding":"True","compiler_instance_overloads_metadata_acknowledged":"True",
    "all101_original_state_storage_and12_compiled_aliases_verified":"True","script_trace_freeze_fuser_or_warmup_used":"False",
    "script_compile_and_cold_profiling_costs_retained":"True","script_report_limit_bytes":"SCRIPT_REPORT_LIMIT",
}
OLD_STRATEGY="147_exact_canonical_V19c_source_references_plus4_fresh_create_only_copies"
NEW_STRATEGY="151_exact_mixed_V20_V19c_refs_plus_stopped_V21_4_finding1_stopped_V21b_4_finding1_pure_failed_V21c4_failure1_fresh_V21d4_copied19"


def module(name,label):
    path=ROOT/name;spec=importlib.util.spec_from_file_location(label,path);value=importlib.util.module_from_spec(spec)
    sys.modules[label]=value;spec.loader.exec_module(value);return value


@pytest.fixture
def worker():return module("tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v21d.py","_hh_v21_pure_worker")


@pytest.fixture
def owner():return module("tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v21d.py","_hh_v21_pure_owner")


@pytest.fixture
def node():return module("autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v21d.py","_hh_v21_pure_node")


def normalize_revision(raw):
    return raw.replace("_20261008_v21d.py","_20261008_v20.py").replace("shadow_v21d","shadow_v20").replace(".20261008.v21d",".20261008.v20").replace("-DDS-v21d","-DDS-v20")


def exact_expression(part,text):
    assert ast.dump(part,include_attributes=False)==ast.dump(ast.parse(text).body[0],include_attributes=False)


def restore_v21_only(part):
    # HH_261008 - Undo exact admitted statements/keywords, never discard an old whole function or class.
    result=deepcopy(part)
    if result.name=="source_pins":
        stopped=ast.parse("""for name,pin in V21_STOPPED_SOURCE_PINS.items():need(digest(ROOT/name)==pin,"stopped V21 source changed")
names |= set(V21_STOPPED_SOURCE_PINS)
for path,pin in V21_SOURCE_FINDING.items():need(digest(path)==pin,"V21 SOURCE finding changed")
names |= {str(Path(path).relative_to(ROOT)) for path in V21_SOURCE_FINDING}
for name,pin in V21B_STOPPED_SOURCE_PINS.items():need(digest(ROOT/name)==pin,"stopped V21b source changed")
names |= set(V21B_STOPPED_SOURCE_PINS)
for path,pin in V21B_SOURCE_FINDING.items():need(digest(path)==pin,"V21b SOURCE finding changed")
names |= {str(Path(path).relative_to(ROOT)) for path in V21B_SOURCE_FINDING}
for name,pin in V21C_STOPPED_SOURCE_PINS.items():need(digest(ROOT/name)==pin,"pure-failed V21c source changed")
names |= set(V21C_STOPPED_SOURCE_PINS)
for path,pin in V21C_TEST_FAILURE.items():need(digest(path)==pin,"V21c scalar failure receipt changed")
names |= {str(Path(path).relative_to(ROOT)) for path in V21C_TEST_FAILURE}
""").body
        for item in stopped:
            indices=[i for i,actual in enumerate(result.body) if ast.dump(actual,include_attributes=False)==ast.dump(item,include_attributes=False)]
            assert len(indices)==1;result.body.pop(indices[0])
        expected=ast.parse('for name,pin in V20_PARENT_PINS.items():need(digest(ROOT/name)==pin,"frozen V20 source changed")\nnames |= set(V20_PARENT_PINS)').body
        for item in expected:
            indices=[i for i,actual in enumerate(result.body) if ast.dump(actual,include_attributes=False)==ast.dump(item,include_attributes=False)]
            assert len(indices)==1;result.body.pop(indices[0])
    if result.name=="fixed_contract":
        call=next(item for item in ast.walk(result) if isinstance(item,ast.Call) and ast.unparse(item.func)=="dict")
        added=[kw for kw in call.keywords if kw.arg in SCRIPT_EXPRESSIONS]
        assert len(added)==len(SCRIPT_EXPRESSIONS) and {kw.arg for kw in added}==set(SCRIPT_EXPRESSIONS)
        for kw in added:assert ast.dump(kw.value,include_attributes=False)==ast.dump(ast.parse(SCRIPT_EXPRESSIONS[kw.arg],mode="eval").body,include_attributes=False)
        strategy=next(kw for kw in call.keywords if kw.arg=="source_archive_strategy")
        assert ast.literal_eval(strategy.value)==NEW_STRATEGY;strategy.value=ast.Constant(OLD_STRATEGY)
        call.keywords=[kw for kw in call.keywords if kw.arg not in SCRIPT_EXPRESSIONS]
    function_name=result.name
    if function_name=="execute":
        # HH_261008 - Admit exactly one original added closing provider call, never a whole execute-function exemption.
        declared=ast.parse("worker.verify_torchscript_provider_sources()").body[0]
        admitted=[item for item in ast.walk(result) if isinstance(item,ast.Expr)
            and ast.dump(item,include_attributes=False)==ast.dump(declared,include_attributes=False)]
        assert len(admitted)==1
    class Restore(ast.NodeTransformer):
        def visit_Expr(self,item):
            if isinstance(item.value,ast.Call):
                text=ast.unparse(item.value)
                if function_name=="prepare" and text in ("worker.read_v20_source_parent()","worker.verify_torchscript_provider_sources()",
                    "input_pins.update(worker.V20_PARENT_EVIDENCE)","input_pins.update(worker.V21_SOURCE_FINDING)","input_pins.update(worker.V21B_SOURCE_FINDING)","input_pins.update(worker.V21C_TEST_FAILURE)","input_pins.update(worker.SCRIPT_FEASIBILITY_PINS)","input_pins.update(worker.TORCHSCRIPT_API_PINS)"):return None
                if function_name=="authorize" and text=="verify_torchscript_provider_sources()":return None
                if function_name=="main" and text=="worker.verify_torchscript_provider_sources()":return None
                if function_name=="execute" and ast.dump(item,include_attributes=False)==ast.dump(declared,include_attributes=False):return None
                if function_name=="run" and text=="scripted_adapters.append(adapter)":return None
                if text=="verify_v21_source_archives(plan)":
                    item.value.func=ast.Name(id="verify_source_archives",ctx=ast.Load())
                elif text=="worker.verify_v21_source_archives(plan)" or text=="worker.verify_v21_source_archives(view)":
                    item.value.func.attr="verify_source_archives"
            return self.generic_visit(item)
        def visit_Assign(self,item):
            target=ast.unparse(item.targets[0]) if len(item.targets)==1 else ""
            if function_name=="run" and target=="scripted_adapters":
                exact_expression(item,"scripted_adapters=[]");return None
            if function_name=="run" and target=="adapter" and isinstance(item.value,ast.Call) and ast.unparse(item.value.func)=="ScriptFeaturesBoundAdapter":
                exact_expression(item,"adapter=ScriptFeaturesBoundAdapter(adapter,worker,install_script_features(adapter,worker,mode))")
                return None
            if function_name=="prepare" and target=="source_archives":
                exact_expression(item,"source_archives=worker.v21_source_archive_mapping(sources)");item.value.func.attr="source_archive_mapping"
            if function_name=="prepare" and target=="prospective_metadata":
                expected="worker.budget()+2*len(worker.encode(plan))+128*1024+7*worker.STARTED_CACHE_REPORT_LIMIT+4*worker.CHANNELS_LAST_LAYOUT_REPORT_LIMIT+6*worker.SCRIPT_REPORT_LIMIT+2*worker.SCRIPT_COMPLETION_LIMIT"
                assert ast.dump(item.value,include_attributes=False)==ast.dump(ast.parse(expected,mode="eval").body,include_attributes=False)
                item.value=ast.parse("worker.budget()+2*len(worker.encode(plan))+128*1024+7*worker.STARTED_CACHE_REPORT_LIMIT+4*worker.CHANNELS_LAST_LAYOUT_REPORT_LIMIT",mode="eval").body
            if function_name=="prepare" and target=="plan":
                call=item.value;added=[kw for kw in call.keywords if kw.arg=="torchscript_API_source_sha256"]
                assert len(added)==1 and ast.unparse(added[0].value)=="worker.TORCHSCRIPT_API_PINS"
                call.keywords.remove(added[0]);kw=next(kw for kw in call.keywords if kw.arg=="source_archive_parent_RAW_sha256")
                assert ast.unparse(kw.value)=="worker.V20_PARENT_EVIDENCE";kw.value=ast.parse("worker.V19C_EVIDENCE",mode="eval").body
            return self.generic_visit(item)
        def visit_For(self,item):
            if function_name=="prepare" and ast.unparse(item.target)=="name" and ast.unparse(item.iter)=="sorted(worker.V21D_COPIED_SOURCE_NAMES)":
                expected=ast.parse("""for name in sorted(worker.V21D_COPIED_SOURCE_NAMES):
    pin=sources[name];archived=Path(source_archives[name]);archived.parent.mkdir(parents=True,exist_ok=True)
    raw=(ROOT/name).read_bytes();worker.need(hashlib.sha256(raw).hexdigest()==pin,"fresh source changed during archive")
    with archived.open("xb") as stream:worker.need(stream.write(raw)==len(raw),"partial fresh source copy");stream.flush()
    worker.source_archive_regular(archived,pin)
""").body[0]
                assert ast.dump(item,include_attributes=False)==ast.dump(expected,include_attributes=False)
                item.iter=ast.parse("sorted(worker.V20_FRESH_SOURCE_NAMES)",mode="eval").body
            if function_name=="run" and ast.unparse(item.target)=="scripted" and ast.unparse(item.iter)=="scripted_adapters":
                expected=ast.parse('for scripted in scripted_adapters:\n    try:close_script_features(scripted,worker,mode)\n    except BaseException as failure:error=(error+"; " if error else "")+"script features close: "+repr(failure)\n    if not any(audited.adapter is scripted for audited in audited_adapters):\n        try:close_channels_last_layout(scripted,worker,mode)\n        except BaseException as failure:error=(error+"; " if error else "")+"unaudited channels_last close: "+repr(failure)').body[0]
                assert ast.dump(item,include_attributes=False)==ast.dump(expected,include_attributes=False);return None
            return self.generic_visit(item)
    result=Restore().visit(result)
    if function_name=="authorize":
        expected=ast.parse('need(plan.get("torchscript_API_source_sha256")==TORCHSCRIPT_API_PINS,"selected script providers differ")').body[0]
        indices=[i for i,item in enumerate(result.body) if ast.dump(item,include_attributes=False)==ast.dump(expected,include_attributes=False)]
        assert len(indices)==1;result.body.pop(indices[0])
    return result


def test_imports_are_stdlib_only(worker,node,owner):
    assert not any(name in sys.modules for name in ("torch","numpy","scipy","rclpy","carla"))
    assert worker.SCHEMA.endswith(".v21d") and worker.HERE.stem.endswith("_v21d")
    for mode in ("direct","subscriber","publisher"):
        argv=worker.worker_command(mode,"a"*64,"b"*64)
        assert argv[4]=="scripts.e2e."+worker.HERE.stem
    assert owner.WORKER==worker.HERE and node.WORKER==worker.HERE


def test_every_original_V20_function_and_class_exact_after_narrow_inverse(worker,node,owner):
    for actual,oldpath in ((worker,"tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v20.py"),
        (owner,"tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v20.py"),
        (node,"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v20.py")):
        original={p.name:p for p in ast.parse((ROOT/oldpath).read_bytes()).body if isinstance(p,(ast.FunctionDef,ast.ClassDef))}
        current={p.name:p for p in ast.parse(normalize_revision(Path(actual.__file__).read_text())).body
            if isinstance(p,(ast.FunctionDef,ast.ClassDef)) and p.name not in NEW_FUNCTIONS}
        assert current.keys()==original.keys()
        for name in original:
            assert ast.dump(restore_v21_only(current[name]),include_attributes=False)==ast.dump(original[name],include_attributes=False),name


@pytest.mark.parametrize("fault",["keyword_value","keyword_missing","keyword_duplicate","strategy","deadline","side_effect","wrong_archive_argument"])
def test_inverse_rejects_or_exposes_all_undeclared_changes(worker,node,fault):
    source=normalize_revision(Path(worker.__file__).read_text())
    functions={p.name:p for p in ast.parse(source).body if isinstance(p,ast.FunctionDef)}
    name="fixed_contract" if fault not in ("side_effect","wrong_archive_argument") else "authorize"
    part=deepcopy(functions[name])
    if fault in ("keyword_value","keyword_missing","keyword_duplicate","strategy","deadline"):
        call=next(p for p in ast.walk(part) if isinstance(p,ast.Call) and ast.unparse(p.func)=="dict")
        key="script_calls_per_model" if fault.startswith("keyword") else ("source_archive_strategy" if fault=="strategy" else "policy_deadline_seconds")
        kw=next(kw for kw in call.keywords if kw.arg==key)
        if fault=="keyword_missing":call.keywords.remove(kw)
        elif fault=="keyword_duplicate":call.keywords.append(deepcopy(kw))
        else:kw.value=ast.Constant(value=2 if fault!="strategy" else "unapproved_strategy")
    elif fault=="wrong_archive_argument":
        call=next(p for p in ast.walk(part) if isinstance(p,ast.Call) and ast.unparse(p.func)=="verify_v21_source_archives")
        call.args[0]=ast.Name(id="unsealed_plan",ctx=ast.Load())
    else:part.body.append(ast.Expr(value=ast.Call(func=ast.Name(id="unapproved_extra_forward",ctx=ast.Load()),args=[],keywords=[])))
    old=next(p for p in ast.parse((ROOT/"tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v20.py").read_bytes()).body if isinstance(p,ast.FunctionDef) and p.name==name)
    if fault.startswith("keyword") or fault=="strategy":
        with pytest.raises((AssertionError,StopIteration)):restore_v21_only(part)
    else:assert ast.dump(restore_v21_only(part),include_attributes=False)!=ast.dump(old,include_attributes=False)


class ScalarDevice:
    type="cpu"
    def __str__(self):return "cpu"


class ScalarParameter:
    def __init__(self,name,pointer):
        self.sha=hashlib.sha256(name.encode()).hexdigest();self.shape=(1,);self.dtype="torch.float32";self.device=ScalarDevice()
        self.requires_grad=False;self._version=0;self.pointer=pointer;self.strides=(1,)
    def data_ptr(self):return self.pointer
    def stride(self):return self.strides


class ScalarModule:
    def __init__(self):
        self.training=False;self._modules=OrderedDict();self._parameters=OrderedDict();self._buffers=OrderedDict()
        self._forward_hooks=OrderedDict();self._forward_pre_hooks=OrderedDict();self._backward_hooks=OrderedDict()
    def forward(self,value):return value
    def __call__(self,value):return self.forward(value)
    def named_modules(self,prefix=""):
        yield prefix,self
        for name,child in self._modules.items():
            yield from child.named_modules(prefix+"."+name if prefix else name)
    def modules(self):return (value for _,value in self.named_modules())
    def children(self):return iter(self._modules.values())
    def named_parameters(self):
        for name,value in self.named_modules():
            for key,parameter in value._parameters.items():yield ((name+"." if name else "")+key),parameter
    def named_buffers(self):return iter(())
    def state_dict(self):return dict(self.named_parameters())


class ScalarConv(ScalarModule):pass
class ScalarNorm(ScalarModule):pass
class ScalarSiLU(ScalarModule):pass
class ScalarSequential(ScalarModule):pass
class ScalarScriptModule:pass


class ScalarCompiled(ScalarScriptModule):
    def __init__(self,features,events):
        self.parameters=dict(features.named_parameters());self.code="synthetic_source_not_numeric"
        class Forward:
            graph="synthetic_compiled_graph"
            def __call__(self,value):events.append(("compiled_forward",value));return value
        self.forward=Forward()
    def named_parameters(self):return list(self.parameters.items())
    def named_buffers(self):return iter(())
    def state_dict(self):return dict(self.parameters)


@pytest.fixture
def script_fixture(node,worker,tmp_path,monkeypatch):
    # HH_261008 - These are scalar symbolic aliases and fake modules, not real Torch parameters or a genuine compiler.
    events=[];features=ScalarSequential();pointer=1000
    for i in range(12):
        value=(ScalarConv,ScalarNorm,ScalarSiLU)[i%3]()
        names=("weight",) if i%3==0 else (("weight","bias") if i%3==1 else ())
        for name in names:value._parameters[name]=ScalarParameter(str(i)+name,pointer);pointer+=1
        features._modules[str(i)]=value
    encoder=ScalarModule();encoder.features=features;encoder._modules["features"]=features
    model=ScalarModule();model.image_encoder=encoder;model._modules["image_encoder"]=encoder
    for i in range(89):model._parameters["other_"+str(i)]=ScalarParameter("other_"+str(i),2000+i)
    model._hh_channels_last_layout_binding=object()
    class Core:
        _policy=SimpleNamespace(model=model)
        def accept(self,value,**kwargs):events.append(("original_accept",value,kwargs));return features(value)
        def reset(self,*args,**kwargs):events.append(("original_reset",args,kwargs));return "same_reset"
    core=Core();fake=ModuleType("torch");fake.float32="torch.float32"
    fake.nn=SimpleNamespace(Sequential=ScalarSequential,Conv2d=ScalarConv,GroupNorm=ScalarNorm,SiLU=ScalarSiLU,
        modules=SimpleNamespace(module=SimpleNamespace(_global_forward_hooks={},_global_forward_pre_hooks={},_global_backward_hooks={})))
    fake.cuda=SimpleNamespace(is_initialized=lambda:False);fake.get_num_threads=lambda:4;fake.get_num_interop_threads=lambda:1
    fake._C=SimpleNamespace(_jit_get_emit_hooks=lambda:(None,None))
    def script(value):
        events.append(("source_script",value))
        for child in value.modules():child.__dict__["__overloads__"]={}
        return ScalarCompiled(value,events)
    fake.jit=SimpleNamespace(script=script,ScriptModule=ScalarScriptModule)
    package=ModuleType("portable_e2e");cache=SimpleNamespace(_tensor_sha256=lambda value:value.sha)
    package.temporal_cached_encoder_v2=cache
    monkeypatch.setitem(sys.modules,"torch",fake);monkeypatch.setitem(sys.modules,"portable_e2e",package)
    monkeypatch.setattr(worker,"verify_torchscript_provider_sources",lambda:events.append(("sealed_provider_check",)))
    monkeypatch.setattr(worker,"OUTPUT",tmp_path);monkeypatch.setattr(worker,"budget",lambda **kwargs:0)
    monkeypatch.setattr(node,"close_channels_last_layout",lambda *args:events.append(("CL_failure_close",)))
    return SimpleNamespace(model=model,features=features,core=core,torch=fake,events=events,out=tmp_path)


def test_scalar_setup_one_source_compile_zero_forwards_complete101_and12_aliases_then_one_accept(node,worker,script_fixture):
    f=script_fixture;before=node.script_original_module_structure(f.model);attrs=node.script_feature_attribute_snapshot(f.features,worker)
    binding=node.install_script_features(f.core,worker,"direct")
    assert [x[0] for x in f.events]==["sealed_provider_check","source_script"]
    assert len(binding["original101"])==101 and len(binding["parameter_identity"])==12
    assert node.script_original_module_structure(f.model,f.features,binding)==before
    setup=worker.bounded_json(f.out/"direct_script_features_setup.json")
    assert setup["script_calls"]==1 and setup["additional_NN_warmups"]==setup["additional_driving_model_initializations"]==0
    assert setup["compiler_creates_RecursiveScriptModule_wrappers"] is True
    assert setup["logical_state_before"]==setup["logical_state_after"] and setup["compiled12_aliases_equal"] is True
    guarded=node.ScriptFeaturesBoundAdapter(f.core,worker,binding);value=object()
    assert guarded.accept(value,current_time_ns=123) is value
    assert [x[0] for x in f.events]==["sealed_provider_check","source_script","original_accept","compiled_forward"]
    assert guarded.reset("original")== "same_reset"
    node.close_script_features(guarded,worker,"direct")
    assert node.SCRIPT_FEATURE_BINDING not in f.model.__dict__ and "forward" not in f.features.__dict__
    assert all("__overloads__" not in child.__dict__ for child in f.features.modules())
    assert node.script_original_module_structure(f.model)==before and node.script_feature_attribute_snapshot(f.features,worker)==attrs
    closing=worker.bounded_json(f.out/"direct_script_features_closing.json")
    assert closing["full101_closing_equal"] and closing["compiled12_closing_equal"] and closing["forward_binding_restored"]


@pytest.mark.parametrize("fault",["original_parameter","compiled_parameter","pointer","version","stride","shape","dtype","device","requires_grad",
    "proxy","binding","module_forward","child","feature_attribute","feature_hook","global_hook","emit_hook","compiled_forward",
    "compiled_extra_parameter","compiled_buffer","original_buffer","compiler_metadata"])
def test_scalar_boundary_denies_every_declared_drift_without_fake_NN(node,worker,script_fixture,fault):
    f=script_fixture;b=node.install_script_features(f.core,worker,"direct");name=next(iter(b["parameter_identity"]));param=b["parameter_identity"][name]["original"]
    if fault=="original_parameter":f.features._modules["0"]._parameters["weight"]=ScalarParameter("replacement",4000)
    elif fault=="compiled_parameter":b["compiled"].parameters[name]=ScalarParameter("replacement",4000)
    elif fault=="pointer":param.pointer+=1
    elif fault=="version":param._version+=1
    elif fault=="stride":param.strides=(2,)
    elif fault=="shape":param.shape=(2,)
    elif fault=="dtype":param.dtype="torch.float64"
    elif fault=="device":param.device="cuda:0"
    elif fault=="requires_grad":param.requires_grad=True
    elif fault=="proxy":f.features.__dict__["forward"]=MethodType(lambda *_:None,f.features)
    elif fault=="binding":f.model.__dict__[node.SCRIPT_FEATURE_BINDING]={}
    elif fault=="module_forward":f.features._modules["1"].forward=lambda *_:None
    elif fault=="child":f.features._modules["0"]=ScalarConv()
    elif fault=="feature_attribute":f.features._modules["0"].unapproved_state="unexpected"
    elif fault=="feature_hook":f.features._modules["0"]._forward_hooks[1]=object()
    elif fault=="global_hook":f.torch.nn.modules.module._global_forward_hooks[1]=object()
    elif fault=="emit_hook":f.torch._C._jit_get_emit_hooks=lambda:(object(),None)
    elif fault=="compiled_extra_parameter":b["compiled"].parameters["foreign_extra"]=ScalarParameter("extra",6000)
    elif fault=="compiled_buffer":b["compiled"].named_buffers=lambda:iter((("foreign",object()),))
    elif fault=="original_buffer":f.features.named_buffers=lambda:iter((("foreign",object()),))
    elif fault=="compiler_metadata":f.features.__dict__["__overloads__"]={"unapproved":"metadata"}
    else:b["compiled"].forward=lambda *_:None
    with pytest.raises(ValueError):node.verify_script_features(f.model,worker,b)
    assert not any(x[0] in ("original_accept","compiled_forward") for x in f.events)


def test_scalar_double_accept_failure_preserves_original_and_post_guard_error(node,worker,script_fixture):
    f=script_fixture;b=node.install_script_features(f.core,worker,"direct");original=RuntimeError("original_policy_failed")
    def fail(*args,**kwargs):
        b["compiled"].forward=lambda *_:None
        raise original
    f.core.accept=fail;guarded=node.ScriptFeaturesBoundAdapter(f.core,worker,b)
    with pytest.raises(ValueError) as caught:guarded.accept(object())
    assert "original_policy_failed" in str(caught.value) and "script_post_guard_error" in str(caught.value)
    assert caught.value.__cause__ is original


@pytest.mark.parametrize("fault",["compile","compile_partial","noop","noop_after_metadata","alias","unapproved_attribute","unexpected_metadata","write"])
def test_scalar_setup_failure_keeps_actual_progress_and_zero_forward(node,worker,script_fixture,monkeypatch,fault):
    f=script_fixture
    if fault=="compile":f.torch.jit.script=lambda _:(_ for _ in ()).throw(RuntimeError("synthetic_compile_failure"))
    elif fault=="compile_partial":
        def partial(value):
            for child in list(value.modules())[:4]:child.__dict__["__overloads__"]={}
            raise RuntimeError("synthetic_partial_compile_failure")
        f.torch.jit.script=partial
    elif fault=="noop":f.torch.jit.script=lambda value:value
    elif fault=="noop_after_metadata":
        def noop(value):
            for child in value.modules():child.__dict__["__overloads__"]={}
            return value
        f.torch.jit.script=noop
    elif fault=="alias":
        original=f.torch.jit.script
        def bad(value):
            result=original(value);name=next(iter(result.parameters));result.parameters[name]=ScalarParameter("copy",9000);return result
        f.torch.jit.script=bad
    elif fault=="unapproved_attribute":
        original=f.torch.jit.script
        def bad(value):result=original(value);value.new_unapproved_attribute=True;return result
        f.torch.jit.script=bad
    elif fault=="unexpected_metadata":
        original=f.torch.jit.script
        def unexpected(value):
            result=original(value);value._modules["1"].__dict__["__overloads__"]={"foreign":"mutation"};return result
        f.torch.jit.script=unexpected
    else:
        original=worker.new_json
        def fail(path,*args,**kwargs):
            if path.name.endswith("_setup_completion.json"):raise OSError("actual_witness_write_failed")
            return original(path,*args,**kwargs)
        monkeypatch.setattr(worker,"new_json",fail)
    with pytest.raises((RuntimeError,ValueError,OSError)):node.install_script_features(f.core,worker,"direct")
    result=worker.bounded_json(f.out/"direct_script_features_setup_failure.json")
    assert result["setup_succeeded"] is False and result["additional_NN_warmups"]==0
    assert result["compile_started_ns"] is not None
    assert (result["compile_finished_ns"] is None) is (fault in ("compile","compile_partial"))
    assert not any(x[0] in ("original_accept","compiled_forward") for x in f.events)
    if fault=="write":assert result["our_forward_binding_restored"] is True and "forward" not in f.features.__dict__
    if fault=="unexpected_metadata":
        assert f.features._modules["1"].__dict__["__overloads__"]=={"foreign":"mutation"}
        assert all("__overloads__" not in child.__dict__ for child in f.features.modules() if child is not f.features._modules["1"])
    else:assert all("__overloads__" not in child.__dict__ for child in f.features.modules())
    if fault in ("compile","compile_partial","noop","noop_after_metadata","unapproved_attribute","unexpected_metadata"):
        assert result["logical_state_after"] is None
    else:assert result["logical_state_after"] is not None
    assert (result["restoration_error"] is not None) is (fault in ("unapproved_attribute","unexpected_metadata"))


def test_scalar_setup_failure_and_receipt_error_keep_both_errors(node,worker,script_fixture,monkeypatch):
    f=script_fixture;original=RuntimeError("first_compile_error");f.torch.jit.script=lambda _:(_ for _ in ()).throw(original)
    monkeypatch.setattr(worker,"new_json",lambda *_:(_ for _ in ()).throw(OSError("failure_receipt_write_error")))
    with pytest.raises(ValueError) as caught:node.install_script_features(f.core,worker,"direct")
    assert "first_compile_error" in str(caught.value) and "failure_receipt_write_error" in str(caught.value)
    assert caught.value.__cause__ is original


@pytest.mark.parametrize("fault",["value","restore","receipt"])
def test_scalar_closing_failure_no_inference_repair_and_double_errors(node,worker,script_fixture,monkeypatch,fault):
    f=script_fixture;b=node.install_script_features(f.core,worker,"direct")
    if fault=="value":b["parameter_identity"][next(iter(b["parameter_identity"]))]["original"].sha="f"*64
    elif fault=="restore":f.features.__dict__["forward"]=MethodType(lambda *_:None,f.features)
    else:
        b["parameter_identity"][next(iter(b["parameter_identity"]))]["original"].sha="f"*64
        monkeypatch.setattr(worker,"new_json",lambda *_:(_ for _ in ()).throw(OSError("closing_receipt_write_error")))
    with pytest.raises(ValueError) as caught:node.close_script_features(f.core,worker,"direct")
    assert not any(x[0] in ("original_accept","compiled_forward") for x in f.events)
    if fault=="receipt":assert "original complete101" in str(caught.value) and "closing_receipt_write_error" in str(caught.value)
    else:
        result=worker.bounded_json(f.out/"direct_script_features_closing.json")
        assert result["full101_closing_equal"] is False and (result["restoration_error"] is not None) is (fault=="restore")


def test_source_script_before_binding_no_trace_freeze_examples_warmup_and_closing_before_originalCL(node):
    tree=ast.parse(Path(node.__file__).read_bytes());parts={p.name:p for p in tree.body if isinstance(p,(ast.FunctionDef,ast.ClassDef))}
    setup=parts["_install_script_features"];calls=[p for p in ast.walk(setup) if isinstance(p,ast.Call)]
    equal=lambda left,right:ast.dump(left,include_attributes=False)==ast.dump(right,include_attributes=False)
    literal=lambda text:ast.parse(text).body[0]
    target_script=ast.parse("torch.jit.script(features)",mode="eval").body
    assert sum(equal(call,target_script) for call in calls)==1
    compile_index=next(i for i,p in enumerate(setup.body) if equal(p,literal("compiled=torch.jit.script(features)")))
    context_index=next(i for i,p in enumerate(setup.body) if isinstance(p,ast.Expr) and isinstance(p.value,ast.Call)
        and ast.unparse(p.value.func)=="cleanup_context.update")
    binding_index=next(i for i,p in enumerate(setup.body) if isinstance(p,ast.Assign) and len(p.targets)==1
        and equal(p.targets[0],ast.Name(id="binding",ctx=ast.Store())))
    proxy_index=next(i for i,p in enumerate(setup.body) if equal(p,literal('features.__dict__["forward"]=binding["proxy"]')))
    assert context_index<compile_index<binding_index<proxy_index
    assert not any(isinstance(call.func,ast.Attribute) and call.func.attr in ("trace","freeze","graph_for","optimize_for_inference") for call in calls)
    assert not any(isinstance(call.func,ast.Name) and call.func.id in ("compiled","features","model") for call in calls)
    assert equal(parts["_script_features_forward"].body[-1],literal('return binding["compiled_forward"](images)'))
    functions=[p for p in ast.walk(parts["run"]) if isinstance(p,ast.FunctionDef)]
    loading=next(p for p in functions if any(isinstance(s,ast.Assign) and isinstance(s.value,ast.Call)
        and isinstance(s.value.func,ast.Name) and s.value.func.id=="ScriptFeaturesBoundAdapter" for s in p.body))
    def call_statement_index(fn,text):return next(i for i,p in enumerate(fn.body) if equal(p,literal(text)))
    cl=call_statement_index(loading,"adapter=ChannelsLastBoundAdapter(adapter,worker,install_channels_last_layout(adapter,worker,mode,strict_load_started_ns))")
    script=call_statement_index(loading,"adapter=ScriptFeaturesBoundAdapter(adapter,worker,install_script_features(adapter,worker,mode))")
    registered=call_statement_index(loading,"scripted_adapters.append(adapter)")
    audit=next(i for i,p in enumerate(loading.body) if isinstance(p,ast.Assign) and any(isinstance(v,ast.Call)
        and isinstance(v.func,ast.Name) and v.func.id=="DeferredAuditedAdapter" for v in ast.walk(p.value)))
    assert cl<script<registered<audit
    final=next(p.finalbody for p in parts["run"].body if isinstance(p,ast.Try) and any(isinstance(v,ast.For) and ast.unparse(v.iter)=="scripted_adapters" for v in p.finalbody))
    script_close=next(i for i,p in enumerate(final) if isinstance(p,ast.For) and ast.unparse(p.iter)=="scripted_adapters")
    cl_close=next(i for i,p in enumerate(final) if isinstance(p,ast.For) and any(isinstance(v,ast.Call)
        and isinstance(v.func,ast.Name) and v.func.id=="close_channels_last_layout" for v in ast.walk(p))
        and ast.unparse(p.iter)=="audited_adapters")
    runtime_close=next(i for i,p in enumerate(final) if isinstance(p,ast.Expr) and isinstance(p.value,ast.Call)
        and isinstance(p.value.func,ast.Name) and p.value.func.id=="close_owned_runtime")
    assert script_close<cl_close<runtime_close


def test_full151_mixed_archive_paths_plus19_exact_not_fresh4_only_or_fake_V20_folder(worker,monkeypatch):
    old={f"old/{i}.py":hashlib.sha256(str(i).encode()).hexdigest() for i in range(151)}
    mapping={name:str(worker.V19C_PLAN_DIR/"sources"/name) for name in old}
    for name in list(old)[-4:]:mapping[name]=str(worker.V20_PARENT_PLAN_DIR/"sources"/name)
    fresh={name:hashlib.sha256(name.encode()).hexdigest() for name in worker.V21D_COPIED_SOURCE_NAMES}
    parent=dict(source_sha256=old,source_archive_path=mapping)
    monkeypatch.setattr(worker,"read_v20_source_parent",lambda:deepcopy(parent))
    all_sources={**old,**fresh};monkeypatch.setattr(worker,"digest",lambda p:all_sources[str(Path(p).relative_to(worker.ROOT))])
    actual=worker.v21_source_archive_mapping(all_sources)
    assert len(actual)==170 and all(actual[name]==path for name,path in mapping.items())
    assert all(actual[name]==str(worker.PLAN_DIR/"sources"/name) for name in fresh)
    # HH_261008 - Exercise the selected live mapper; the stale fresh-four admission must fail rather than only change declaration text.
    fresh4_only={**old,**{name:fresh[name] for name in worker.V20_FRESH_SOURCE_NAMES}}
    assert len(fresh4_only)==155 and len(worker.V21D_COPIED_SOURCE_NAMES)==19
    with pytest.raises(ValueError,match="plus19"):worker.v21_source_archive_mapping(fresh4_only)
    with pytest.raises(ValueError):worker.v21_source_archive_mapping({name:pin for name,pin in all_sources.items() if name!=next(iter(old))})
    with pytest.raises(ValueError):worker.v21_source_archive_mapping({**all_sources,"unapproved.py":"f"*64})


def test_actual_frozenV20_pins_and_provider_declaration_caps_are_preserved(worker,owner,node):
    assert len(worker.V20_PARENT_PINS)==4 and len(worker.V20_PARENT_EVIDENCE)==3
    for name,pin in worker.V20_PARENT_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    assert len(worker.TORCHSCRIPT_API_PINS)==12 and len(worker.SCRIPT_FEASIBILITY_PINS)==1
    assert worker.SCRIPT_REPORT_LIMIT==128*1024 and worker.SCRIPT_COMPLETION_LIMIT==4*1024
    source=Path(owner.__file__).read_text()
    assert "6*worker.SCRIPT_REPORT_LIMIT+2*worker.SCRIPT_COMPLETION_LIMIT" in source and "prospective_metadata<=10*1024**2" in source
    assert "source_archive_parent_RAW_sha256=worker.V20_PARENT_EVIDENCE" in source
    assert worker.QUOTA==128*1024**2 and worker.CLOSING==16*1024**2 and worker.FLOOR==2*1024**3
    fixed=worker.fixed_contract();assert fixed["camera_anchors"]==918 and fixed["policy_deadline_seconds"]==.1
    assert fixed["raw_atol"]==fixed["raw_rtol"]==1e-6 and fixed["CPU_threads"]==4 and fixed["interop_threads"]==1


@pytest.fixture
def synthetic_v20_raw_parent(worker,tmp_path,monkeypatch):
    # HH_261008 - These fake RAW files prove reader admission/denials, not a real ROOT grant or a successful918 experiment.
    old={f"ancestor/{i}.py":hashlib.sha256(str(i).encode()).hexdigest() for i in range(147)}
    added={f"v20_fresh/{i}.py":hashlib.sha256(("new"+str(i)).encode()).hexdigest() for i in range(4)}
    parent_dir=tmp_path/"v20_parent";ancestor_dir=tmp_path/"v19c_parent";parent_dir.mkdir();ancestor_dir.mkdir()
    paths=[parent_dir/"result.json",parent_dir/"ROOT_plan.json",parent_dir/"ROOT_execution_authority.json"]
    sources={**old,**added};mapping={name:str((parent_dir if name in added else ancestor_dir)/"sources"/name) for name in sources}
    parent=dict(schema="portable_e2e.owned_recorded_ROS_shadow.20261008.v20",source_sha256=sources,input_sha256={"scalar_metadata":"b"*64},
        source_archive_parent_RAW_sha256=deepcopy(worker.V19C_EVIDENCE),source_archive_path=mapping)
    result=dict(source_sha256_after=deepcopy(sources),input_sha256_after=deepcopy(parent["input_sha256"]),cleanup_errors=[],children=[{"exit":1}])
    grant=dict(plan_path=str(paths[1]),plan_sha256=None,source_sha256=deepcopy(sources))
    pins={}
    def persist():
        paths[0].write_bytes(worker.encode(result));paths[1].write_bytes(worker.encode(parent))
        grant["plan_sha256"]=hashlib.sha256(paths[1].read_bytes()).hexdigest();paths[2].write_bytes(worker.encode(grant))
        pins.clear();pins.update({str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths})
    persist();observed=[]
    monkeypatch.setattr(worker,"V20_PARENT_EVIDENCE",pins);monkeypatch.setattr(worker,"V20_PARENT_PINS",added)
    monkeypatch.setattr(worker,"V20_PARENT_PLAN_DIR",parent_dir);monkeypatch.setattr(worker,"V19C_PLAN_DIR",ancestor_dir)
    monkeypatch.setattr(worker,"read_v19c_source_parent",lambda:{"source_sha256":deepcopy(old)})
    monkeypatch.setattr(worker,"source_archive_regular",lambda path,pin:observed.append((str(path),pin)))
    return SimpleNamespace(old=old,added=added,parent=parent,result=result,grant=grant,paths=paths,pins=pins,persist=persist,observed=observed)


def test_complete151_parent_uses_exact147_old_and4_V20_paths_even_if_parent_failed(worker,synthetic_v20_raw_parent):
    f=synthetic_v20_raw_parent;actual=worker.read_v20_source_parent()
    assert actual==f.parent and len(f.observed)==151 and f.result["children"]==[{"exit":1}]
    assert dict(f.observed)=={path:f.parent["source_sha256"][name] for name,path in f.parent["source_archive_path"].items()}


@pytest.mark.parametrize("fault",["RAW_pin","schema","missing","source_after","input_after","unfinished","cleanup","grant_source",
    "nested_parent","old_membership","all151_in_V20_folder","wrong_fresh_folder"])
def test_parent_denies_missing_wrong_mixed_ownership_and_incomplete_proof(worker,synthetic_v20_raw_parent,fault):
    f=synthetic_v20_raw_parent;old_name=next(iter(f.old));new_name=next(iter(f.added))
    if fault=="RAW_pin":f.pins[str(f.paths[0])]="f"*64
    else:
        if fault=="schema":f.parent["schema"]="foreign.schema"
        elif fault=="missing":f.parent["source_sha256"].pop(old_name)
        elif fault=="source_after":f.result["source_sha256_after"]={}
        elif fault=="input_after":f.result["input_sha256_after"]={}
        elif fault=="unfinished":f.result["children"][0]["exit"]=None
        elif fault=="cleanup":f.result["cleanup_errors"]=["retained cleanup failure"]
        elif fault=="grant_source":f.grant["source_sha256"]={}
        elif fault=="nested_parent":f.parent["source_archive_parent_RAW_sha256"]={}
        elif fault=="old_membership":f.parent["source_sha256"][old_name]="f"*64;f.result["source_sha256_after"]=deepcopy(f.parent["source_sha256"]);f.grant["source_sha256"]=deepcopy(f.parent["source_sha256"])
        elif fault=="all151_in_V20_folder":f.parent["source_archive_path"]={name:str(worker.V20_PARENT_PLAN_DIR/"sources"/name) for name in f.parent["source_sha256"]}
        else:f.parent["source_archive_path"][new_name]=str(worker.V19C_PLAN_DIR/"sources"/new_name)
        f.persist()
    with pytest.raises(ValueError):worker.read_v20_source_parent()
    assert f.observed==[]

# HH_261008 - Cover the precise SOURCE-review gap: metadata mutations happen before binding/alias validation and must still be visible to cleanup.
@pytest.mark.parametrize("fault",["compile_partial","noop","alias"])
def test_prebinding_failure_preserves_exact_existing_empty_dict_object(node,worker,script_fixture,fault):
    f=script_fixture;original_empty={};existing=f.features._modules["0"];existing.__dict__["__overloads__"]=original_empty
    genuine=f.torch.jit.script
    def failing(value):
        result=genuine(value)
        if fault=="compile_partial":raise RuntimeError("compile_after_known_metadata")
        if fault=="noop":return value
        name=next(iter(result.parameters));result.parameters[name]=ScalarParameter("unshared_copy",9911);return result
    f.torch.jit.script=failing
    with pytest.raises((RuntimeError,ValueError)):node.install_script_features(f.core,worker,"direct")
    result=worker.bounded_json(f.out/"direct_script_features_setup_failure.json")
    assert node.SCRIPT_FEATURE_BINDING not in f.model.__dict__ and "forward" not in f.features.__dict__
    assert existing.__dict__["__overloads__"] is original_empty and original_empty=={}
    assert all("__overloads__" not in child.__dict__ for child in f.features.modules() if child is not existing)
    assert result["private_cleanup_context_available"] and result["scoped_compiler_metadata_restored"]
    assert len(result["compiler_metadata_cleanup_records"])==13 and result["restoration_error"] is None
    assert result["compiled12_alias_receipts"] is None
    assert (result["logical_state_after"] is None) is (fault!="alias")
    assert not any(x[0] in ("original_accept","compiled_forward") for x in f.events)


@pytest.mark.parametrize("foreign",["nonempty","wrong_type","deleted_original","mutated_original"])
def test_scoped_cleanup_retains_unknown_metadata_and_continues_all_other_known_modules(node,worker,script_fixture,foreign):
    f=script_fixture;existing=f.features._modules["0"];original_empty={}
    if foreign in ("deleted_original","mutated_original"):existing.__dict__["__overloads__"]=original_empty
    before=node.script_original_module_structure(f.model);attrs=node.script_feature_attribute_snapshot(f.features,worker)
    snapshot=tuple((name,value,"__overloads__" in value.__dict__,value.__dict__.get("__overloads__")) for name,value in f.features.named_modules())
    context=dict(model=f.model,features=f.features,module_structure=before,feature_attributes=attrs,overloads_before=snapshot,binding=None)
    for child in f.features.modules():child.__dict__["__overloads__"]={}
    if foreign=="nonempty":existing.__dict__["__overloads__"]={"unknown":1}
    elif foreign=="wrong_type":existing.__dict__["__overloads__"]="unknown"
    elif foreign=="deleted_original":del existing.__dict__["__overloads__"]
    else:original_empty["unexpected"]="preserved"
    with pytest.raises(ValueError):node.restore_script_compile_context(context,worker)
    assert len(context["metadata_cleanup_records"])==13 and context["cleanup_succeeded"] is False
    assert all("__overloads__" not in child.__dict__ for child in f.features.modules() if child is not existing)
    if foreign=="nonempty":assert existing.__dict__["__overloads__"]=={"unknown":1}
    elif foreign=="wrong_type":assert existing.__dict__["__overloads__"]=="unknown"
    elif foreign=="deleted_original":assert "__overloads__" not in existing.__dict__
    else:assert original_empty=={"unexpected":"preserved"} and existing.__dict__["__overloads__"] is not original_empty


def test_compile_failure_plus_scoped_cleanup_CL_close_and_receipt_errors_preserve_all(node,worker,script_fixture,monkeypatch):
    f=script_fixture;original=RuntimeError("original_compile_failure")
    def failing(value):
        value.__dict__["__overloads__"]={"foreign":"keep"}
        raise original
    f.torch.jit.script=failing
    monkeypatch.setattr(node,"close_channels_last_layout",lambda *_:(_ for _ in ()).throw(OSError("CL_closing_failure")))
    monkeypatch.setattr(worker,"new_json",lambda *_:(_ for _ in ()).throw(OSError("failure_receipt_unavailable")))
    with pytest.raises(ValueError) as caught:node.install_script_features(f.core,worker,"direct")
    assert caught.value.__cause__ is original
    assert all(text in str(caught.value) for text in ("original_compile_failure","scoped_script_cleanup_errors","CL_closing_failure","failure_receipt_unavailable"))
    assert f.features.__dict__["__overloads__"]=={"foreign":"keep"}


def test_preserved_source_findings_and_pure_failure_are_not_execution_parents_and_exact19_copies_are_declared(worker,owner,node):
    assert len(worker.V21_STOPPED_SOURCE_PINS)==4 and len(worker.V21_SOURCE_FINDING)==1
    assert len(worker.V21B_STOPPED_SOURCE_PINS)==4 and len(worker.V21B_SOURCE_FINDING)==1
    assert len(worker.V21D_COPIED_SOURCE_NAMES)==19
    assert set(worker.V20_FRESH_SOURCE_NAMES)<=worker.V21D_COPIED_SOURCE_NAMES
    assert set(worker.V21_STOPPED_SOURCE_PINS)<=worker.V21D_COPIED_SOURCE_NAMES
    assert set(worker.V21B_STOPPED_SOURCE_PINS)<=worker.V21D_COPIED_SOURCE_NAMES
    assert len(worker.V21C_STOPPED_SOURCE_PINS)==4 and len(worker.V21C_TEST_FAILURE)==1
    assert set(worker.V21C_STOPPED_SOURCE_PINS)<=worker.V21D_COPIED_SOURCE_NAMES
    for name,pin in {**worker.V21_STOPPED_SOURCE_PINS,**worker.V21B_STOPPED_SOURCE_PINS,**worker.V21C_STOPPED_SOURCE_PINS}.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    for name,pin in {**worker.V21_SOURCE_FINDING,**worker.V21B_SOURCE_FINDING}.items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==pin
        result=json.loads(Path(name).read_bytes())
        assert result["status"]=="STOPPED_SOURCE_ONLY_NOT_RUNTIME_OR_MODEL_FAILURE"
        assert all(result[key]==0 for key in ("actual_pure_tests","actual_prepare_attempts","actual_script_compilations","model_inferences","ROS_replays","training_steps"))
    source=Path(owner.__file__).read_text()
    assert "for name in sorted(worker.V21D_COPIED_SOURCE_NAMES)" in source
    assert all("input_pins.update(worker."+name+")" in source for name in ("V21_SOURCE_FINDING","V21B_SOURCE_FINDING","V21C_TEST_FAILURE"))
    assert not any(name in Path(worker.__file__).read_text() for name in ("V21_PARENT_PLAN_DIR","V21B_PARENT_PLAN_DIR","V21C_PARENT_PLAN_DIR"))
    for name,pin in worker.V21C_TEST_FAILURE.items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==pin
        failure=json.loads(Path(name).read_bytes())
        assert failure["status"]=="FAILED_SOURCE_TESTS_PREPARATION_NOT_RUN"
        assert failure["actual_pure_passed"]==71 and failure["actual_pure_failed"]==1
        assert failure["conditional_prepare_executed"] is False
        assert all(failure[key]==0 for key in ("actual_script_compilations","model_inferences","ROS_replays","training_steps"))


def test_every_constant_ast_parse_source_literal_is_syntactically_valid(worker,node,owner):
    # HH_261008 - Parse literal AST inputs themselves; parsing the surrounding test file alone misses escaped-newline errors.
    observed=[]
    for path in (Path(__file__),Path(worker.__file__),Path(node.__file__),Path(owner.__file__)):
        for call in ast.walk(ast.parse(path.read_bytes())):
            if (isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute)
                and isinstance(call.func.value,ast.Name) and call.func.value.id=="ast" and call.func.attr=="parse"
                and call.args and isinstance(call.args[0],ast.Constant) and type(call.args[0].value) is str):
                modes=[kw.value.value for kw in call.keywords if kw.arg=="mode" and isinstance(kw.value,ast.Constant)]
                assert len(modes)<=1
                ast.parse(call.args[0].value,mode=modes[0] if modes else "exec")
                observed.append((str(path),call.lineno))
    assert len(observed)==11


def test_unused_original_archive_mapper_matches_V20_exactly(worker):
    # HH_261008 - Keep the unused baseline function identical while fixing only the explicitly selected new mapper.
    old=next(p for p in ast.parse((ROOT/"tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v20.py").read_bytes()).body
        if isinstance(p,ast.FunctionDef) and p.name=="source_archive_mapping")
    current=next(p for p in ast.parse(Path(worker.__file__).read_bytes()).body
        if isinstance(p,ast.FunctionDef) and p.name=="source_archive_mapping")
    assert ast.dump(current,include_attributes=False)==ast.dump(old,include_attributes=False)


@pytest.mark.parametrize("fault",["none","extra_unapproved_call","duplicate_declared_call","changed_declared_argument"])
def test_execute_closing_exact_added_provider_call_inverse_and_undeclared_changes(owner,fault):
    # HH_261008 - Check the exact post-archive/pre-final-seal insertion and expose additional calls rather than waiving execute.
    current=next(p for p in ast.parse(normalize_revision(Path(owner.__file__).read_text())).body
        if isinstance(p,ast.FunctionDef) and p.name=="execute")
    baseline=next(p for p in ast.parse((ROOT/"tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v20.py").read_bytes()).body
        if isinstance(p,ast.FunctionDef) and p.name=="execute")
    matches=[]
    for node in ast.walk(current):
        for field,value in ast.iter_fields(node):
            if isinstance(value,list):
                for i,part in enumerate(value):
                    if isinstance(part,ast.Expr) and isinstance(part.value,ast.Call) and ast.unparse(part.value.func)=="worker.verify_torchscript_provider_sources":
                        matches.append((value,i,part))
    assert len(matches)==1
    block,ordinal,declared=matches[0]
    assert 0<ordinal<len(block)-1
    exact_expression(block[ordinal-1],"worker.verify_v21_source_archives(view)")
    exact_expression(block[ordinal+1],'worker.need(source_after==view["source_sha256"] and inputs_after==view["input_sha256"],"actual source/input post-seal failure")')
    if fault=="extra_unapproved_call":
        added=deepcopy(declared);added.value.func.attr="unapproved_extra_closing_call";block.insert(ordinal+1,added)
    elif fault=="duplicate_declared_call":block.insert(ordinal+1,deepcopy(declared))
    elif fault=="changed_declared_argument":declared.value.args=[ast.Name(id="unapproved_argument",ctx=ast.Load())]
    if fault in ("duplicate_declared_call","changed_declared_argument"):
        with pytest.raises(AssertionError):restore_v21_only(current)
    else:
        result=restore_v21_only(current)
        same=ast.dump(result,include_attributes=False)==ast.dump(baseline,include_attributes=False)
        assert same is (fault=="none")
