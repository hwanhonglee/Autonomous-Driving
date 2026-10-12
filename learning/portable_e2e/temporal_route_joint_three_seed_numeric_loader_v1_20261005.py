"""HH_261005 - Prepare a default-denied numeric loader for reviewed three-seed final6000 exports.

No endpoint SHA is guessed and no running/intermediate checkpoint is eligible.
ROOT must independently review a completed seed, seal its binding and review,
and explicitly authorize this OFFLINE-only entry. The original joint numeric
loader code is reused with private schema/validation globals: metadata is never
rewritten to masquerade as the old export, and no original module is mutated.
Its NPZ101/finite-float32/ZIP/shape/common-parent/frozen47 checks remain intact.
No pickle is read, no optimizer resumes, and no ROS/CARLA/vehicle actuation,
model promotion, learned STOP, paired evaluation or runtime gain is authorized.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import FunctionType

from .contract import ContractError, _read_regular_file_bounded


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.temporal_route_joint_three_seed_numeric_loader.20261005.v1"
TRAINING_SCHEMA = "portable_e2e.temporal_route_joint_three_seed_train_only.20261005.v2"
EXPORT_SCHEMA = TRAINING_SCHEMA + ".numeric_research_export"
OWNER_SCHEMA = SCHEMA + ".ROOT_endpoint_binding"
REVIEW_SCHEMA = SCHEMA + ".ROOT_endpoint_review"
SEEDS = (2026100501, 2026100502, 2026100503)
LOADER_NAME = "learning/portable_e2e/temporal_route_joint_three_seed_numeric_loader_v1_20261005.py"
ORIGINAL_NAME = "learning/portable_e2e/temporal_route_joint_export_research_20261002.py"
SOURCE_PINS = {
    ORIGINAL_NAME: "3245e5ee72b6a2bca030249b3075704eb579a0331d0b0c4d6292068e7c192bab",
    "tools/e2e/train_temporal_route_joint_three_seed_train_only_v2_20261005.py": "0f4c084305c44d6c994fddc416e523c23284234c4d4ec005d7eaddbd12a94cc9",
    "learning/tests/test_temporal_route_joint_three_seed_train_only_v2_20261005.py": "9a5ea5d1a7115bd69ba5c91b27f367c5b5000ad49826f2e9bdbdc2a9b201f82c",
}
CHECKPOINT_STEPS = tuple(range(500, 6001, 500))
REQUIRED_FILES = frozenset(("declaration.json", "summary.json", "control/declaration.json",
    "control/fit_result.json", "control/fresh_initialization.json", "control/history.jsonl")) | frozenset(
    name for step in CHECKPOINT_STEPS for name in (
        "control/checkpoints/step_%06d.pt" % step,
        "control/exports/step_%06d/model_state.npz" % step,
        "control/exports/step_%06d/metadata.json" % step))


def require(value, message):
    if not value:
        raise ContractError(message)


def is_sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def json_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def canonical(path):
    path = Path(path)
    require(path.is_absolute() and path.resolve(strict=True) == path
            and not any(p.is_symlink() for p in (path, *path.parents)), "canonical non-symlink readonly path required")
    return path


def pinned(path, expected, limit):
    require(is_sha(expected), "externally sealed SHA256 required")
    payload = _read_regular_file_bounded(canonical(path), limit, "reviewed seed numeric loader input")
    require(hashlib.sha256(payload).hexdigest() == expected, "reviewed seed input SHA256 differs")
    return payload


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate JSON key forbidden")
        result[key] = value
    return result


def reject_constant(value):
    raise ContractError("nonfinite JSON constant forbidden: " + value)


def document(path, expected):
    value = json.loads(pinned(path, expected, 8 * 1024**2), object_pairs_hook=unique_object, parse_constant=reject_constant)
    require(type(value) is dict, "sealed JSON object required")
    return value


def validate_authorization(binding, review, binding_sha, endpoint_root):
    require(type(binding) is dict and binding.get("schema") == OWNER_SCHEMA
        and binding.get("authorized") is True and type(binding.get("seed")) is int and binding["seed"] in SEEDS
        and binding.get("purpose") == "offline_cpu_numeric_validation_only"
        and binding.get("endpoint_root") == str(endpoint_root)
        and binding.get("fixed_endpoint_updates") == 6000 and type(binding["fixed_endpoint_updates"]) is int
        and all(binding.get(key) is False for key in ("simulator_control_authorized", "vehicle_control_approved", "model_promoted")),
        "ROOT final6000 offline-only endpoint binding required")
    require(type(review) is dict and review.get("schema") == REVIEW_SCHEMA
        and review.get("status") == "PASS_COMPLETED_SEED_ENDPOINT_OFFLINE_ONLY"
        and review.get("endpoint_binding_sha256") == binding_sha and review.get("seed") == binding["seed"]
        and type(review.get("seed")) is int and review.get("endpoint_root") == str(endpoint_root)
        and all(review.get(key) is True for key in (
            "all6000_updates_and_schedule_verified", "all12_checkpoint_receipts_verified",
            "all33_transferred_sources_and31_runtime_sources_verified", "TRAIN_before_after_verified",
            "CPU_GPU_authorities_verified", "supervisor_full_raw_and_owned_session_closure_verified"))
        and all(review.get(key) is False for key in ("model_promoted", "vehicle_control_approved", "learned_STOP_claim")),
        "independent completed endpoint review required; missing/UNKNOWN/failure is not authority")
    files = binding.get("files")
    require(type(files) is dict and set(files) == REQUIRED_FILES and all(is_sha(v) for v in files.values())
        and is_sha(binding.get("schedule_sha256")) and is_sha(binding.get("loader_source_sha256")),
        "all42 exact fixed endpoint/source receipts required")
    return binding


def validate_evidence(binding, declaration, arm_declaration, summary, result, fresh):
    seed = binding["seed"]
    require(declaration.get("schema") == TRAINING_SCHEMA and type(declaration.get("seed")) is int
        and declaration["seed"] == seed and arm_declaration == dict(declaration, arm="control")
        and declaration.get("fixed_schedule_sha256") == binding["schedule_sha256"]
        and declaration.get("all_seed_schedules", {}).get(str(seed), {}).get("schedule_sha256") == binding["schedule_sha256"]
        and declaration.get("evaluation_only_payloads_opened") is False
        and declaration.get("model_promoted") is False and declaration.get("vehicle_control_approved") is False,
        "exact original seed/control declaration and schedule required")
    sources = declaration.get("source_sha256")
    require(type(sources) is dict and len(sources) == 31 and all(is_sha(v) for v in sources.values())
        and all(sources.get(name) == pin for name, pin in SOURCE_PINS.items() if name != ORIGINAL_NAME),
        "actual sealed31 training sources including frozen new trainer/test required")
    require(summary.get("schema") == TRAINING_SCHEMA and type(summary.get("seed")) is int and summary["seed"] == seed
        and summary.get("status") == "FIXED6000_SEED_TRAIN_COMPLETE_NOT_PROMOTED" and summary.get("fit") == result
        and summary.get("TRAIN_receipts_before_after_exact") is True and summary.get("images_before") == summary.get("images_after")
        and type(summary.get("images_before")) is dict
        and summary.get("evaluation_only_payloads_opened") is False and summary.get("common_pretrained_lineage_shared") is True
        and summary.get("three_seed_A_B_performed") is False
        and summary.get("model_promoted") is False and summary.get("vehicle_control_approved") is False
        and all(type(summary.get(key)) is int and summary[key] == 0 for key in ("formal_main_approved", "formal_sub_approved")),
        "completed actual TRAIN-only summary required; no intermediate selection")
    require(result.get("arm") == "control" and type(result.get("optimizer_updates")) is int and result["optimizer_updates"] == 6000
        and type(result.get("examples_seen")) is int and result["examples_seen"] == 24000
        and result.get("completed_fixed_endpoint") is True and result.get("DEV_or_HOLDOUT_read") is False
        and result.get("model_promoted") is False, "fixed6000/B4 actual endpoint required")
    hashes = summary.get("actual_batch_sha256")
    require(type(hashes) is list and len(hashes) == 6000 and all(is_sha(value) for value in hashes), "all6000 actual batch receipts required")
    require(fresh.get("fresh_optimizer_state_empty") is True and fresh.get("optimizer_resume") is False
        and type(fresh.get("optimizer_updates_before_fit")) is int and fresh["optimizer_updates_before_fit"] == 0
        and is_sha(fresh.get("initial101_sha256"))
        and fresh["initial101_sha256"] == fresh.get("common101_sha256") == declaration.get("common_initial101_sha256"),
        "same common101/fresh optimizer receipt required")
    checkpoints = result.get("checkpoints")
    require(type(checkpoints) is list and len(checkpoints) == 12
        and all(type(row) is dict and type(row.get("optimizer_updates")) is int for row in checkpoints)
        and [row["optimizer_updates"] for row in checkpoints] == list(CHECKPOINT_STEPS), "all12 fixed checkpoint rows required")
    for row in checkpoints:
        step = row["optimizer_updates"]
        require(row.get("optimizer_state_retained") is True
            and row.get("sha256") == binding["files"]["control/checkpoints/step_%06d.pt" % step]
            and row.get("export_npz_sha256") == binding["files"]["control/exports/step_%06d/model_state.npz" % step]
            and row.get("export_metadata_sha256") == binding["files"]["control/exports/step_%06d/metadata.json" % step],
            "original checkpoint/export receipt differs")
    endpoint = summary.get("final_endpoint", {})
    require(endpoint.get("all101_numeric_finite") is True and endpoint.get("frozen47_exact") is True
        and endpoint.get("metadata_lineage_exact") is True and type(endpoint.get("seed")) is int and endpoint["seed"] == seed
        and type(endpoint.get("fixed_endpoint_updates")) is int and endpoint["fixed_endpoint_updates"] == 6000
        and endpoint.get("checkpoint_sha256") == checkpoints[-1]["sha256"]
        and endpoint.get("npz_sha256") == checkpoints[-1]["export_npz_sha256"]
        and endpoint.get("metadata_sha256") == checkpoints[-1]["export_metadata_sha256"],
        "authentic final101/frozen47 endpoint receipt required")
    return checkpoints[-1]


def _scope_function(original):
    # HH_261005 - New schema globals validate the truthful NEW metadata; no dictionary normalization or module mutation.
    function = original.verify_scope
    names = ("__builtins__", "require", "is_sha", "ARCHITECTURE_SCHEMA", "MODEL_ID", "BATCH_DOMAINS",
        "PARENT_CHECKPOINT_SHA", "PARENT_NPZ_SHA", "PARENT_METADATA_SHA", "PARENT_DECLARATION_SHA",
        "learning_rate_config", "KERNEL_ID")
    private = {name: function.__globals__[name] for name in names}
    private.update(EXPORT_SCHEMA=EXPORT_SCHEMA, TRAINING_SCHEMA=TRAINING_SCHEMA)
    copied = FunctionType(function.__code__, private, function.__name__, function.__defaults__, function.__closure__)
    copied.__kwdefaults__ = function.__kwdefaults__
    return copied


def verify_new_scope(description, original_scope, declaration, binding):
    original_scope(description)
    expected = dict(seed=binding["seed"], current_training_purpose="independent_seed_joint6000_TRAIN_only_reproduction",
        independently_random_pretraining_claimed=False, three_seed_model_A_B_performed=False,
        common_pretrained_lineage_shared=True, DEV_or_HOLDOUT_or_TEST_payload_loaded=False)
    require(all(type(description.get(key)) is type(value) and description[key] == value for key, value in expected.items())
        and description.get("source_sha256") == declaration["source_sha256"]
        and description.get("training_dataset_fingerprint_sha256") == declaration["training_dataset_fingerprint_sha256"]
        and description.get("frozen_state_sha256") == declaration["frozen47_sha256"]
        and description.get("declaration_sha256") == binding["files"]["control/declaration.json"],
        "new seed/common-lineage/TRAIN/source/control-declaration identities differ")


def _numeric_function(original, declaration, binding):
    inherited_scope = _scope_function(original)

    def check_scope(description):
        verify_new_scope(description, inherited_scope, declaration, binding)

    def check_owner(owner, description, weights_sha, metadata_sha):
        require(owner == binding and owner["purpose"] == "offline_cpu_numeric_validation_only"
            and owner["simulator_control_authorized"] is False
            and weights_sha == binding["files"]["control/exports/step_006000/model_state.npz"]
            and metadata_sha == binding["files"]["control/exports/step_006000/metadata.json"]
            and description["checkpoint_sha256"] == binding["files"]["control/checkpoints/step_006000.pt"],
            "one reviewed final endpoint and offline-only owner required")

    function = original.load_route_joint_research_npz
    # HH_261005 - Admit only original numeric callbacks; module-wide TRAIN/DEV/evaluator/fit globals are not copied.
    names = ("__builtins__", "json", "pinned", "unique_json_object", "require", "Path", "load_joint_parent",
        "RouteRecoveryTemporalModelV3", "trainable_names", "zipfile", "BytesIO", "np", "torch", "frozen_fingerprint")
    private = {name: function.__globals__[name] for name in names}
    private.update(SCHEMA=SCHEMA, verify_scope=check_scope, verify_owner=check_owner)
    copied = FunctionType(function.__code__, private, function.__name__, function.__defaults__, function.__closure__)
    copied.__kwdefaults__ = function.__kwdefaults__
    require(copied.__code__ is function.__code__, "exact original finite101/ZIP/common47 numeric code required")
    return copied


def _dependencies(source_root):
    for name, pin in SOURCE_PINS.items():
        pinned(source_root / name, pin, 1024**2)
    from . import temporal_route_joint_export_research_20261002 as original
    require(Path(original.__file__).resolve() == source_root / ORIGINAL_NAME, "actual imported frozen loader source root differs")
    return original


def load_reviewed_seed_endpoint(endpoint_root, *, endpoint_binding_path, endpoint_binding_sha256,
        ROOT_review_path, ROOT_review_sha256, parent_weights, parent_metadata,
        backbone_parent_weights, backbone_parent_metadata, source_root=ROOT, authorize_reviewed_endpoint=False):
    """HH_261005 - Default DENY before file access/import; ROOT must explicitly grant only reviewed offline loading."""
    require(authorize_reviewed_endpoint is True, "actual endpoint loading remains denied pending independent ROOT review")
    endpoint_root, source_root = canonical(endpoint_root), canonical(source_root)
    binding = document(endpoint_binding_path, endpoint_binding_sha256)
    review = document(ROOT_review_path, ROOT_review_sha256)
    validate_authorization(binding, review, endpoint_binding_sha256, endpoint_root)
    require(canonical(source_root / LOADER_NAME) == Path(__file__).resolve(), "exact current prepared loader source required")
    pinned(source_root / LOADER_NAME, binding["loader_source_sha256"], 1024**2)
    for name, pin in binding["files"].items():
        limit = 64 * 1024**2 if name.endswith((".pt", ".jsonl")) else 32 * 1024**2
        pinned(endpoint_root / name, pin, limit)
    docs = {name: document(endpoint_root / name, binding["files"][name]) for name in (
        "declaration.json", "control/declaration.json", "summary.json", "control/fit_result.json", "control/fresh_initialization.json")}
    declaration = docs["declaration.json"]
    validate_evidence(binding, declaration, docs["control/declaration.json"], docs["summary.json"],
                      docs["control/fit_result.json"], docs["control/fresh_initialization.json"])
    for name, pin in declaration["source_sha256"].items():
        require(type(name) is str and not Path(name).is_absolute() and str(Path(name)) == name
            and all(part not in ("", ".", "..") for part in name.split("/")), "safe canonical training source required")
        pinned(source_root / name, pin, 16 * 1024**2)
    original = _dependencies(source_root)
    function = _numeric_function(original, declaration, binding)
    result = function(endpoint_root / "control/exports/step_006000/model_state.npz",
        endpoint_root / "control/exports/step_006000/metadata.json",
        weights_sha256=binding["files"]["control/exports/step_006000/model_state.npz"],
        metadata_sha256=binding["files"]["control/exports/step_006000/metadata.json"],
        parent_weights=parent_weights, parent_metadata=parent_metadata,
        backbone_parent_weights=backbone_parent_weights, backbone_parent_metadata=backbone_parent_metadata,
        research_owner_binding=binding, source_root=source_root)
    # HH_261005 - Retain external review identities without granting any policy, evaluation dataset or release authority.
    result._research_three_seed_endpoint_binding_sha256 = endpoint_binding_sha256
    result._research_three_seed_ROOT_review_sha256 = ROOT_review_sha256
    return result
