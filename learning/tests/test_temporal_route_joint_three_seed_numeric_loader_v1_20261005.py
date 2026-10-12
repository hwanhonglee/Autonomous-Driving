"""HH_261005 - Verify a default-denied future endpoint loader with scalar metadata, never weights or Torch.

These fixtures do not claim numeric model parity, endpoint completion, external
ROOT approval, evaluation, simulator timing, learned STOP, or vehicle readiness.
Authentic original function code is compiled without importing its dependencies.
"""

import ast
import builtins
import copy
import hashlib
from io import BytesIO
import json
import os
from pathlib import Path
import stat
from types import ModuleType, SimpleNamespace
import zipfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "learning/portable_e2e/temporal_route_joint_three_seed_numeric_loader_v1_20261005.py"
ORIGINAL = ROOT / "learning/portable_e2e/temporal_route_joint_export_research_20261002.py"
CONTRACT = ROOT / "learning/portable_e2e/contract.py"


def nodes(path, names):
    return [node for node in ast.parse(path.read_text()).body
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names]


def execute_nodes(body, scope, path):
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    tree = ast.fix_missing_locations(ast.Module(body=[future, *body], type_ignores=[]))
    exec(compile(tree, str(path), "exec"), scope)


def pure_module():
    # HH_261005 - Compile stdlib-only production AST and genuine bounded-file helpers; imports of old numeric code stay dormant.
    module = ModuleType("hh_reviewed_seed_scalar_fixture")
    scope = vars(module)
    scope.update(__file__=str(SOURCE), hashlib=hashlib, json=json, Path=Path,
        FunctionType=__import__("types").FunctionType, os=os, stat=stat, READ_CHUNK_BYTES=8 * 1024**2)
    execute_nodes(nodes(CONTRACT, {"ContractError", "_file_identity", "_open_regular_read_only",
        "_verify_open_file_identity", "_read_regular_file_bounded"}), scope, CONTRACT)
    body = [node for node in ast.parse(SOURCE.read_text()).body if isinstance(node, (ast.Assign, ast.FunctionDef))]
    execute_nodes(body, scope, SOURCE)
    return module


def original_functions(m):
    # HH_261005 - Preserve exact original code objects with explicit scalar/sentinel providers; the numeric function is not executed.
    scope = {"__builtins__": vars(builtins), "ContractError": m.ContractError, "ROOT": ROOT,
        "is_sha": m.is_sha, "ARCHITECTURE_SCHEMA": "scalar_V3_architecture",
        "MODEL_ID": "scalar_V3_model", "KERNEL_ID": "scalar_original_kernel",
        "json": json, "Path": Path, "zipfile": zipfile, "BytesIO": BytesIO,
        "np": object(), "torch": object(), "forbidden_fit": lambda: pytest.fail("fit must never be copied"),
        "forbidden_DEV_loader": lambda: pytest.fail("DEV loader must never be copied")}
    for node in ast.parse(ORIGINAL.read_text()).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            try:
                scope[node.targets[0].id] = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                pass
    for name in ("pinned", "unique_json_object", "load_joint_parent", "RouteRecoveryTemporalModelV3",
            "trainable_names", "frozen_fingerprint", "verify_owner"):
        scope[name] = lambda *args, **kwargs: pytest.fail("numeric callback execution forbidden in scalar tests")
    execute_nodes(nodes(ORIGINAL, {"require", "learning_rate_config", "verify_scope", "load_route_joint_research_npz"}), scope, ORIGINAL)
    return SimpleNamespace(**scope)


def sha(value):
    return hashlib.sha256(value.encode() if type(value) is str else value).hexdigest()


def metadata_fixture(m, endpoint=Path("/scalar/offline_endpoint")):
    files = {name: sha("scalar file " + name) for name in m.REQUIRED_FILES}
    binding = dict(schema=m.OWNER_SCHEMA, authorized=True, seed=m.SEEDS[0],
        purpose="offline_cpu_numeric_validation_only", endpoint_root=str(endpoint),
        fixed_endpoint_updates=6000, simulator_control_authorized=False,
        vehicle_control_approved=False, model_promoted=False, files=files,
        schedule_sha256=sha("scalar schedule"), loader_source_sha256=sha("scalar source"))
    binding_sha = m.json_sha(binding)
    review = dict(schema=m.REVIEW_SCHEMA, status="PASS_COMPLETED_SEED_ENDPOINT_OFFLINE_ONLY",
        endpoint_binding_sha256=binding_sha, seed=binding["seed"], endpoint_root=str(endpoint),
        all6000_updates_and_schedule_verified=True, all12_checkpoint_receipts_verified=True,
        all33_transferred_sources_and31_runtime_sources_verified=True, TRAIN_before_after_verified=True,
        CPU_GPU_authorities_verified=True, supervisor_full_raw_and_owned_session_closure_verified=True,
        model_promoted=False, vehicle_control_approved=False, learned_STOP_claim=False)
    minimum = {
        "tools/e2e/train_temporal_route_joint_20261002.py",
        "learning/portable_e2e/temporal_route_joint_export_research_20261002.py",
        "tools/e2e/train_temporal_route_recovery_moving_v4_20261001.py",
        "learning/portable_e2e/temporal_route_recovery_v3_20261001.py"}
    sources = {name: sha("scalar source " + name) for name in minimum}
    sources.update(m.SOURCE_PINS)
    for ordinal in range(31 - len(sources)):
        sources["learning/scalar_source_%02d.py" % ordinal] = sha(str(ordinal))
    declaration = dict(schema=m.TRAINING_SCHEMA, seed=binding["seed"],
        fixed_schedule_sha256=binding["schedule_sha256"],
        all_seed_schedules={str(seed): dict(schedule_sha256=binding["schedule_sha256"]) for seed in m.SEEDS},
        evaluation_only_payloads_opened=False, model_promoted=False, vehicle_control_approved=False,
        source_sha256=sources, common_initial101_sha256=sha("common101"),
        frozen47_sha256=sha("frozen47"), training_dataset_fingerprint_sha256=sha("TRAIN membership"))
    arm = dict(declaration, arm="control")
    checkpoints = [dict(optimizer_updates=step, optimizer_state_retained=True,
        sha256=files["control/checkpoints/step_%06d.pt" % step],
        export_npz_sha256=files["control/exports/step_%06d/model_state.npz" % step],
        export_metadata_sha256=files["control/exports/step_%06d/metadata.json" % step]) for step in m.CHECKPOINT_STEPS]
    result = dict(arm="control", optimizer_updates=6000, examples_seen=24000,
        completed_fixed_endpoint=True, DEV_or_HOLDOUT_read=False, model_promoted=False, checkpoints=checkpoints)
    summary = dict(schema=m.TRAINING_SCHEMA, seed=binding["seed"],
        status="FIXED6000_SEED_TRAIN_COMPLETE_NOT_PROMOTED", fit=result,
        TRAIN_receipts_before_after_exact=True, images_before={"TRAIN": sha("scalar retained inputs")},
        images_after={"TRAIN": sha("scalar retained inputs")}, evaluation_only_payloads_opened=False,
        common_pretrained_lineage_shared=True, three_seed_A_B_performed=False,
        model_promoted=False, vehicle_control_approved=False, formal_main_approved=0, formal_sub_approved=0,
        actual_batch_sha256=[sha("scalar batch")] * 6000,
        final_endpoint=dict(all101_numeric_finite=True, frozen47_exact=True, metadata_lineage_exact=True,
            seed=binding["seed"], fixed_endpoint_updates=6000, checkpoint_sha256=checkpoints[-1]["sha256"],
            npz_sha256=checkpoints[-1]["export_npz_sha256"], metadata_sha256=checkpoints[-1]["export_metadata_sha256"]))
    fresh = dict(fresh_optimizer_state_empty=True, optimizer_resume=False, optimizer_updates_before_fit=0,
        initial101_sha256=declaration["common_initial101_sha256"], common101_sha256=declaration["common_initial101_sha256"])
    return binding, review, binding_sha, declaration, arm, summary, result, fresh


def scope_fixture(m, original, binding, declaration):
    # HH_261005 - Derive numbers/flags from the actual frozen scalar scope rather than hand-copying its fixed lineage.
    function = nodes(ORIGINAL, {"verify_scope"})[0]
    description = {}
    for node in function.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and node.targets[0].id in {"numbers", "flags"}:
            description.update(eval(compile(ast.Expression(node.value), str(ORIGINAL), "eval"), {}))
    description.update(schema=m.EXPORT_SCHEMA, training_schema=m.TRAINING_SCHEMA,
        architecture_schema=original.ARCHITECTURE_SCHEMA, model_id=original.MODEL_ID,
        batch_domains=original.BATCH_DOMAINS, runtime_intent="explicit_cruise_GO_not_head",
        parent_checkpoint_sha256=original.PARENT_CHECKPOINT_SHA, parent_npz_sha256=original.PARENT_NPZ_SHA,
        parent_metadata_sha256=original.PARENT_METADATA_SHA, parent_declaration_sha256=original.PARENT_DECLARATION_SHA,
        learning_rate=original.learning_rate_config(),
        checkpoint_sha256=binding["files"]["control/checkpoints/step_006000.pt"],
        npz_sha256=binding["files"]["control/exports/step_006000/model_state.npz"],
        declaration_sha256=binding["files"]["control/declaration.json"], frozen_state_sha256=declaration["frozen47_sha256"],
        training_dataset_fingerprint_sha256=declaration["training_dataset_fingerprint_sha256"],
        holdout_dataset_fingerprint_sha256=sha("historical metadata only; no HOLDOUT loaded"),
        numerical_revision=dict(kernel_id=original.KERNEL_ID, strict_deterministic_algorithms=True,
            tf32_matrix_multiplication=False, finite_precision_bitwise_equivalence_claimed=False),
        source_sha256=declaration["source_sha256"], seed=binding["seed"],
        current_training_purpose="independent_seed_joint6000_TRAIN_only_reproduction",
        independently_random_pretraining_claimed=False, three_seed_model_A_B_performed=False,
        common_pretrained_lineage_shared=True, DEV_or_HOLDOUT_or_TEST_payload_loaded=False)
    return description


def mutate(document, path, value):
    for key in path[:-1]:
        document = document[key]
    document[path[-1]] = value


def test_positive_scalar_whole_endpoint_protocol():
    m = pure_module()
    binding, review, binding_sha, declaration, arm, summary, result, fresh = metadata_fixture(m)
    assert len(m.REQUIRED_FILES) == 42 and len(result["checkpoints"]) == 12
    assert m.validate_authorization(binding, review, binding_sha, Path(binding["endpoint_root"])) is binding
    assert m.validate_evidence(binding, declaration, arm, summary, result, fresh) is result["checkpoints"][-1]


@pytest.mark.parametrize("key,value", [
    ("schema", "old_export_schema"), ("authorized", False), ("authorized", 1),
    ("seed", 2026100504), ("seed", True), ("seed", 2026100501.0),
    ("purpose", "owned_carla_research_only"), ("endpoint_root", "/another/endpoint"),
    ("fixed_endpoint_updates", 5999), ("fixed_endpoint_updates", 6000.0),
    ("simulator_control_authorized", True), ("vehicle_control_approved", True), ("model_promoted", True),
    ("schedule_sha256", "A" * 64), ("loader_source_sha256", "missing")])
def test_binding_faults_are_not_authority(key, value):
    m = pure_module(); binding, review, binding_sha, *_ = metadata_fixture(m)
    binding[key] = value
    with pytest.raises(m.ContractError):
        m.validate_authorization(binding, review, binding_sha, Path("/scalar/offline_endpoint"))


@pytest.mark.parametrize("key,value", [
    ("schema", "old_review"), ("status", "UNKNOWN"), ("endpoint_binding_sha256", "0" * 64),
    ("seed", 2026100502), ("seed", 2026100501.0), ("endpoint_root", "/another/endpoint"),
    ("all6000_updates_and_schedule_verified", False), ("all12_checkpoint_receipts_verified", False),
    ("all33_transferred_sources_and31_runtime_sources_verified", "UNKNOWN"),
    ("TRAIN_before_after_verified", False), ("CPU_GPU_authorities_verified", False),
    ("supervisor_full_raw_and_owned_session_closure_verified", "UNKNOWN"),
    ("model_promoted", True), ("vehicle_control_approved", True), ("learned_STOP_claim", True)])
def test_review_failure_unknown_or_wrong_binding_denies(key, value):
    m = pure_module(); binding, review, binding_sha, *_ = metadata_fixture(m)
    review[key] = value
    with pytest.raises(m.ContractError):
        m.validate_authorization(binding, review, binding_sha, Path(binding["endpoint_root"]))


@pytest.mark.parametrize("mode", ["missing", "extra", "bad_sha"])
def test_all42_exact_receipts_are_required(mode):
    m = pure_module(); binding, review, binding_sha, *_ = metadata_fixture(m)
    name = "control/checkpoints/step_000500.pt"
    if mode == "missing": binding["files"].pop(name)
    elif mode == "extra": binding["files"]["extra_candidate.pt"] = "0" * 64
    else: binding["files"][name] = "bad"
    with pytest.raises(m.ContractError):
        m.validate_authorization(binding, review, binding_sha, Path(binding["endpoint_root"]))


@pytest.mark.parametrize("which,path,value", [
    ("declaration", ("schema",), "old_schema"), ("declaration", ("seed",), 2026100502),
    ("declaration", ("fixed_schedule_sha256",), "0" * 64),
    ("declaration", ("evaluation_only_payloads_opened",), True),
    ("declaration", ("model_promoted",), True), ("declaration", ("vehicle_control_approved",), True),
    ("summary", ("status",), "RUNNING"), ("summary", ("seed",), 2026100501.0),
    ("summary", ("TRAIN_receipts_before_after_exact",), False),
    ("summary", ("images_after",), {}), ("summary", ("common_pretrained_lineage_shared",), False),
    ("summary", ("evaluation_only_payloads_opened",), True), ("summary", ("three_seed_A_B_performed",), True),
    ("summary", ("model_promoted",), True), ("summary", ("vehicle_control_approved",), True),
    ("summary", ("formal_main_approved",), 1), ("summary", ("formal_sub_approved",), False),
    ("result", ("arm",), "candidate"), ("result", ("optimizer_updates",), 5999),
    ("result", ("optimizer_updates",), 6000.0), ("result", ("examples_seen",), 24001),
    ("result", ("completed_fixed_endpoint",), False), ("result", ("DEV_or_HOLDOUT_read",), True),
    ("fresh", ("fresh_optimizer_state_empty",), False), ("fresh", ("optimizer_resume",), True),
    ("fresh", ("optimizer_updates_before_fit",), True), ("fresh", ("initial101_sha256",), "1" * 64),
    ("summary", ("final_endpoint", "all101_numeric_finite"), False),
    ("summary", ("final_endpoint", "frozen47_exact"), False),
    ("summary", ("final_endpoint", "metadata_lineage_exact"), False),
    ("summary", ("final_endpoint", "fixed_endpoint_updates"), 6000.0),
    ("summary", ("final_endpoint", "checkpoint_sha256"), "0" * 64),
    ("summary", ("final_endpoint", "npz_sha256"), "0" * 64),
    ("summary", ("final_endpoint", "metadata_sha256"), "0" * 64)])
def test_whole_fit_and_fresh_common_initializer_faults(which, path, value):
    m = pure_module(); binding, _, _, declaration, arm, summary, result, fresh = metadata_fixture(m)
    documents = dict(declaration=declaration, summary=summary, result=result, fresh=fresh)
    mutate(documents[which], path, value)
    with pytest.raises(m.ContractError):
        m.validate_evidence(binding, declaration, arm, summary, result, fresh)


@pytest.mark.parametrize("mode", ["source30", "new_trainer_sha", "new_test_sha", "arm_not_control",
    "batch5999", "bad_batch_sha", "checkpoint11", "checkpoint_duplicate", "checkpoint_float",
    "optimizer_not_retained", "checkpoint_sha", "npz_sha", "metadata_sha"])
def test_full_sources_schedule_and_checkpoint_denominators(mode):
    m = pure_module(); binding, _, _, declaration, arm, summary, result, fresh = metadata_fixture(m)
    if mode == "source30": declaration["source_sha256"].pop(next(iter(declaration["source_sha256"])))
    elif mode in {"new_trainer_sha", "new_test_sha"}:
        prefix = "tools/" if mode == "new_trainer_sha" else "learning/tests/"
        name = next(name for name in m.SOURCE_PINS if name.startswith(prefix))
        declaration["source_sha256"][name] = "0" * 64
    elif mode == "arm_not_control": arm["arm"] = "candidate"
    elif mode == "batch5999": summary["actual_batch_sha256"].pop()
    elif mode == "bad_batch_sha": summary["actual_batch_sha256"][0] = "bad"
    elif mode == "checkpoint11": result["checkpoints"].pop(0)
    elif mode == "checkpoint_duplicate": result["checkpoints"][0]["optimizer_updates"] = 1000
    elif mode == "checkpoint_float": result["checkpoints"][0]["optimizer_updates"] = 500.0
    elif mode == "optimizer_not_retained": result["checkpoints"][0]["optimizer_state_retained"] = False
    else:
        key = {"checkpoint_sha": "sha256", "npz_sha": "export_npz_sha256", "metadata_sha": "export_metadata_sha256"}[mode]
        result["checkpoints"][0][key] = "0" * 64
    with pytest.raises(m.ContractError):
        m.validate_evidence(binding, declaration, arm, summary, result, fresh)


def test_private_function_code_and_callbacks_are_original_without_module_mutation():
    m = pure_module(); original = original_functions(m)
    binding, _, _, declaration, *_ = metadata_fixture(m)
    before = dict(original.verify_scope.__globals__)
    old_scope = original.verify_scope; old_numeric = original.load_route_joint_research_npz
    private_scope = m._scope_function(original)
    private_numeric = m._numeric_function(original, declaration, binding)
    assert private_scope.__code__ is old_scope.__code__
    assert private_numeric.__code__ is old_numeric.__code__
    assert dict(old_scope.__globals__) == before and old_numeric.__globals__ is old_scope.__globals__
    assert original.verify_scope is old_scope and original.load_route_joint_research_npz is old_numeric
    assert private_scope.__globals__["EXPORT_SCHEMA"] == m.EXPORT_SCHEMA
    assert private_scope.__globals__["TRAINING_SCHEMA"] == m.TRAINING_SCHEMA
    assert private_numeric.__globals__["SCHEMA"] == m.SCHEMA
    for name in ("load_joint_parent", "RouteRecoveryTemporalModelV3", "pinned", "np", "torch",
            "trainable_names", "frozen_fingerprint"):
        assert private_numeric.__globals__[name] is old_numeric.__globals__[name]
    assert set(private_numeric.__globals__) == {"__builtins__", "json", "pinned", "unique_json_object", "require",
        "Path", "load_joint_parent", "RouteRecoveryTemporalModelV3", "trainable_names", "zipfile", "BytesIO",
        "np", "torch", "frozen_fingerprint", "SCHEMA", "verify_scope", "verify_owner"}
    assert set(private_scope.__globals__) == {"__builtins__", "require", "is_sha", "ARCHITECTURE_SCHEMA", "MODEL_ID",
        "BATCH_DOMAINS", "PARENT_CHECKPOINT_SHA", "PARENT_NPZ_SHA", "PARENT_METADATA_SHA", "PARENT_DECLARATION_SHA",
        "learning_rate_config", "KERNEL_ID", "EXPORT_SCHEMA", "TRAINING_SCHEMA"}
    assert not any("forbidden" in name for name in private_numeric.__globals__ | private_scope.__globals__)


def test_truthful_new_scope_passes_without_old_schema_masquerade():
    m = pure_module(); original = original_functions(m)
    binding, _, _, declaration, *_ = metadata_fixture(m)
    description = scope_fixture(m, original, binding, declaration)
    before = copy.deepcopy(description)
    private = m._numeric_function(original, declaration, binding)
    private.__globals__["verify_scope"](description)
    private.__globals__["verify_owner"](binding, description,
        binding["files"]["control/exports/step_006000/model_state.npz"],
        binding["files"]["control/exports/step_006000/metadata.json"])
    assert description == before
    with pytest.raises(m.ContractError): original.verify_scope(description)


@pytest.mark.parametrize("key,value", [
    ("schema", "old_schema"), ("training_schema", "old_schema"), ("seed", 2026100502),
    ("seed", 2026100501.0), ("optimizer_updates", 6000.0), ("optimizer_updates", 5999),
    ("known_total_optimizer_updates", 6000), ("trainable_state_tensor_count", 53),
    ("frozen_state_tensor_count", 48), ("effective_batch_size", 8), ("examples_seen", 48000),
    ("visual_encoder_frozen", False), ("semantic_head_parameters_frozen", False),
    ("inference_loader_authorized", True), ("stop_intent_trained", True),
    ("independent_validation", True), ("checkpoint_selection_by_holdout", True),
    ("labels_clipped", True), ("labels_extrapolated", True),
    ("current_training_purpose", "owned_carla"), ("independently_random_pretraining_claimed", True),
    ("three_seed_model_A_B_performed", True), ("common_pretrained_lineage_shared", False),
    ("DEV_or_HOLDOUT_or_TEST_payload_loaded", True), ("declaration_sha256", "0" * 64)])
def test_original_and_added_scope_faults(key, value):
    m = pure_module(); original = original_functions(m)
    binding, _, _, declaration, *_ = metadata_fixture(m)
    description = scope_fixture(m, original, binding, declaration)
    description[key] = value
    with pytest.raises(m.ContractError):
        m._numeric_function(original, declaration, binding).__globals__["verify_scope"](description)


def test_root_declaration_cannot_replace_original_control_declaration_binding():
    m = pure_module(); original = original_functions(m)
    binding, _, _, declaration, *_ = metadata_fixture(m)
    description = scope_fixture(m, original, binding, declaration)
    description["declaration_sha256"] = binding["files"]["declaration.json"]
    with pytest.raises(m.ContractError):
        m._numeric_function(original, declaration, binding).__globals__["verify_scope"](description)


def test_original_numeric_guards_and_parent_code_remain_present_exactly():
    m = pure_module(); original = original_functions(m)
    binding, _, _, declaration, *_ = metadata_fixture(m)
    assert m._numeric_function(original, declaration, binding).__code__.co_code == original.load_route_joint_research_npz.__code__.co_code
    source = ast.unparse(nodes(ORIGINAL, {"load_route_joint_research_npz"})[0])
    for fragment in ("len(expected) == len(reference) == 101", "len(trainable_names(expected)) == 54",
        "all24 original actual parent training source pins", "len(entries) == 101", "e.flag_bits & 1",
        "sum((e.file_size for e in entries))", "allow_pickle=False", "array.dtype == np.float32",
        "np.isfinite(array).all()", "torch.equal(state[k], reference[k])", "frozen_fingerprint(reference)",
        "strict=True", "model.requires_grad_(False)"):
        assert fragment in source
    callbacks = {node.func.id for node in ast.walk(nodes(ORIGINAL, {"load_route_joint_research_npz"})[0])
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
    assert not callbacks & {"load_train", "fit", "train_arm", "evaluate", "load_DEV", "load_HOLDOUT", "torch_load"}


@pytest.mark.parametrize("authorization", [False, None, 0, 1, "True"])
def test_public_entry_default_denies_before_any_file_import_or_weight(authorization):
    m = pure_module()
    def forbidden(*args, **kwargs): pytest.fail("default DENY must precede input/dependency access")
    m.load_reviewed_seed_endpoint.__globals__.update(canonical=forbidden, document=forbidden, _dependencies=forbidden)
    with pytest.raises(m.ContractError, match="denied"):
        m.load_reviewed_seed_endpoint("/does/not/exist", endpoint_binding_path=None, endpoint_binding_sha256=None,
            ROOT_review_path=None, ROOT_review_sha256=None, parent_weights=None, parent_metadata=None,
            backbone_parent_weights=None, backbone_parent_metadata=None, authorize_reviewed_endpoint=authorization)


@pytest.mark.parametrize("payload", [b'{"duplicate":1,"duplicate":2}', b'{"notfinite":NaN}',
    b'{"notfinite":Infinity}', b'{"notfinite":-Infinity}', b'[]'])
def test_sealed_json_duplicate_nonfinite_or_nonobject_denied(tmp_path, payload):
    m = pure_module(); path = tmp_path / "scalar.json"; path.write_bytes(payload)
    with pytest.raises(m.ContractError): m.document(path, sha(payload))


def test_original_bounded_read_nonregular_symlink_wrong_sha_and_size(tmp_path):
    m = pure_module(); path = tmp_path / "scalar.json"; path.write_bytes(b"scalar")
    assert m.pinned(path, sha(b"scalar"), 6) == b"scalar"
    with pytest.raises(m.ContractError): m.pinned(path, sha(b"scalar"), 5)
    with pytest.raises(m.ContractError): m.pinned(path, "0" * 64, 6)
    with pytest.raises(m.ContractError): m.pinned(tmp_path, sha(b"scalar"), 6)
    link = tmp_path / "link.json"; link.symlink_to(path)
    with pytest.raises(m.ContractError): m.pinned(link, sha(b"scalar"), 6)
    with pytest.raises(m.ContractError): m.canonical(Path("relative"))


def test_authority_documents_precede_dependency_import_in_public_code():
    function = nodes(SOURCE, {"load_reviewed_seed_endpoint"})[0]
    statements = [ast.unparse(node) for node in function.body]
    default = next(i for i, text in enumerate(statements) if "authorize_reviewed_endpoint is True" in text)
    authority = next(i for i, text in enumerate(statements) if text.startswith("validate_authorization("))
    all_files = next(i for i, text in enumerate(statements) if "binding['files'].items()" in text)
    whole_fit = next(i for i, text in enumerate(statements) if text.startswith("validate_evidence("))
    sources = next(i for i, text in enumerate(statements) if "declaration['source_sha256'].items()" in text)
    dependencies = next(i for i, text in enumerate(statements) if "original = _dependencies(" in text)
    assert default < authority < all_files < whole_fit < sources < dependencies
    imports = [node for node in ast.parse(SOURCE.read_text()).body if isinstance(node, (ast.Import, ast.ImportFrom))]
    assert not any(any(name.name in {"torch", "numpy"} for name in node.names) for node in imports)
    assert "pickle" not in {node.func.id for node in ast.walk(function) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
