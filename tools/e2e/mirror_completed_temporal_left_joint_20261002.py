"""HH_261002 - Mirror the completed fixed LEFT20k run; evidence is NOT deployment authority."""
from __future__ import annotations

import argparse
import ast
import hashlib
import inspect
import json
import math
from pathlib import Path
import shlex
import shutil
import subprocess
from types import SimpleNamespace

from scripts.e2e import mirror_completed_temporal_intrinsic_pair_20261002 as frozen

ROOT = Path(__file__).resolve().parents[2]
REMOTE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
RUN = REMOTE / "runs/training/temporal_left_joint_20261002_v1"
SOURCE = REMOTE / "source_worktrees/temporal_left_joint_20261002_v1"
PYTHON, SSH = frozen.PYTHON, frozen.SSH
LOCAL = ROOT / "storage/experiments/training/2026-10-02/temporal_left_joint_v1"
RECEIPT = ROOT / "storage/experiments/training/2026-10-02/temporal_left_joint_mirror_v1"
SCHEMA = "portable_e2e.temporal_left_joint_completed_mirror_evidence.20261002.v1"
TRAINING = "portable_e2e.temporal_actual_left_joint_continuation.20261002.v1"
EXPORT = "portable_e2e.temporal_actual_left_joint_export_research.20261002.v1"
TRAINER = "tools/e2e/train_temporal_left_joint_20261002.py"
TRAINER_SHA = "dc8963d4bd4941911b600f1409827f0961977ae9af75c7c146789cdba020aa3a"
LEFT_SHA = "f70350f3aeab5b88069759a259bf4b3059c775429eec6fabda38a70b966f2664"
PARENT_HISTORY = "3c1fe1e4f9e18017a2db31b5371001873dece4a2b8e78bb3cdb21f9aad5ebac3"
PARENT = dict(weights_sha256="b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625",
    metadata_sha256="a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
    checkpoint_sha256="43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88",
    declaration_sha256="0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5")
HELPERS = {"tools/e2e/mirror_completed_temporal_intrinsic_pair_20261002.py":
    "cd9b75d624a5c2b0cebd603657c11a37fa795e979d38bf768f7dc278c0b48ec1",
    "learning/portable_e2e/temporal_left_joint_numeric_loader_20261002.py":
    "7f95e61dd18f61b7f2a0b35908a004424e1f678f598cb46332ed5fc5c273ad0c"}
require, canonical, digest, inventory = frozen.require, frozen.canonical, frozen.digest, frozen.inventory


def is_sha(value):
    return type(value) is str and len(value) == 64 and set(value) <= set("0123456789abcdef")


def unique(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate JSON key"); result[key] = value
    return result


def finite_json_number(text):
    value = float(text); require(math.isfinite(value), "nonfinite JSON number"); return value


def invalid_json_constant(text):
    raise ValueError("nonfinite JSON constant: " + text)


def json_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def read(path, expected, maximum=16 * 1024**2):
    path = canonical(path); require(path.stat().st_size <= maximum, "bounded JSON artifact required")
    payload = path.read_bytes(); require(hashlib.sha256(payload).hexdigest() == expected, "JSON changed before parsing")
    value = json.loads(payload, object_pairs_hook=unique, parse_float=finite_json_number, parse_constant=invalid_json_constant)
    require(type(value) is dict, "JSON object required"); return value


def learning_rate_config():
    # HH_261002 - Exact frozen optimizer constants, with the actual fixed20k horizon.
    return dict(schedule="fixed20000_linear_warmup_cosine_two_groups", warmup_updates=300,
        shared_peak=1e-5, shared_final=1e-6, heads_peak=3e-5, heads_final=3e-6,
        gradient_clip_l2=5., weight_decay=1e-4, units="AdamW optimizer learning rate")


def journal_code():
    # HH_261002 - Extract ONLY two authenticated pure functions; never import the Torch loader.
    for name, sha in HELPERS.items(): require(digest(ROOT / name) == sha, "frozen mirror/loader helper changed")
    path = ROOT / "learning/portable_e2e/temporal_left_joint_numeric_loader_20261002.py"
    payload = path.read_text(); tree = ast.parse(payload)
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ("learning_rate", "verify_history")]
    require([n.name for n in nodes] == ["learning_rate", "verify_history"], "exact frozen journal function identities required")
    return "\n\n".join(ast.get_source_segment(payload, n) for n in nodes)


def journal_namespace(code):
    namespace = dict(require=require, json=json, math=math, is_sha=is_sha, json_sha=json_sha,
        learning_rate_config=learning_rate_config, finite_json_number=finite_json_number,
        invalid_json_constant=invalid_json_constant, parent_api=SimpleNamespace(unique_json_object=unique))
    exec(compile(code, "<authenticated_LEFT20k_journal_functions>", "exec"), namespace)
    return namespace


def inspect_run(run, source, summary_sha, declaration_sha, authorization_sha, code):
    # HH_261002 - The exact same stdlib inspection runs remotely and on synthetic local archives.
    run, source = canonical(run), canonical(source)
    files, source_files = inventory(run), inventory(source)
    def document(folder, census, relative, maximum=16 * 1024**2):
        return read(folder / relative, census[relative]["sha256"], maximum)
    require(files["summary.json"]["sha256"] == summary_sha and files["declaration.json"]["sha256"] == declaration_sha
        and source_files["authorization.json"]["sha256"] == authorization_sha, "external completion/ROOT authority pins differ")
    declaration = document(run, files, "declaration.json")
    summary, fit = document(run, files, "summary.json"), document(run, files, "fit_result.json")
    require(declaration["schema"] == summary["schema"] == TRAINING
        and summary["status"] == "FIXED20000_ACTUAL_LEFT_JOINT_COMPLETE_NOT_PROMOTED" and summary["fit"] == fit
        and summary["all71_original_screens_retained"] is True and summary["full_LEFT347_retained"] is True
        and summary["model_promoted"] is False and summary["vehicle_control_approved"] is False
        and "failure_receipt.json" not in files, "completed full evaluation required, not fit-only or failed partial")
    require(type(fit["optimizer_updates"]) is int and fit["optimizer_updates"] == 20000 and fit["examples_seen"] == 80000
        and fit["completed_fixed_endpoint"] is True and fit["DEV_or_HOLDOUT_read"] is False and fit["model_promoted"] is False
        and [r["optimizer_updates"] for r in fit["checkpoints"]] == list(range(1000, 20001, 1000)), "all20 fixed endpoint receipts required")
    sources, protocol = declaration["source_sha256"], declaration["protocol"]
    authorization = document(source, source_files, "authorization.json")
    require(type(sources) is dict and len(sources) == 30 and sources.get(TRAINER) == TRAINER_SHA
        and all(is_sha(pin) for pin in sources.values()) and protocol["sources"] == sources
        and declaration["authorization_sha256"] == authorization_sha
        and authorization["schema"] == TRAINING + ".prospective_authorization" and authorization["authorized"] is True
        and authorization["contract"] == protocol and authorization["contract_sha256"] == json_sha(protocol)
        and authorization["model_promoted"] is False and authorization["vehicle_control_approved"] is False,
        "exact30 sources and original externally sealed research-only training authority required")
    require(protocol["schema"] == TRAINING and protocol["actual_parent_final6000"] == PARENT
        and protocol["actual_parent_history_sha256"] == PARENT_HISTORY and protocol["new_LEFT_conversion_sha256"] == LEFT_SHA
        and protocol["fixed_new_updates"] == 20000 and protocol["examples_seen"] == 80000
        and protocol["training_episode_count"] == 19 and protocol["learning_rate"] == learning_rate_config()
        and declaration["training_episode_count"] == 19 and declaration["trainable_state_tensor_count"] == 54
        and declaration["frozen_state_tensor_count"] == 47 and declaration["parent_history_sha256"] == PARENT_HISTORY,
        "currentGOOD6000/19TRAIN/full347/54-47 fixed protocol required")
    cohorts = declaration["cohorts"]
    require(set(cohorts) == {"recovery", "v4", "legacy", "LEFT"} and all(v["split"] == "train" for v in cohorts.values())
        and cohorts["LEFT"]["sample_count"] == 347 and cohorts["LEFT"]["episode_count"] == 1
        and cohorts["LEFT"]["conversion_sha256"] == LEFT_SHA, "nineteen original TRAIN sources only")
    for name, pin in sources.items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts and str(relative) == name, "canonical source key required")
        require(source_files[name]["sha256"] == files["source/" + name]["sha256"] == pin, "live/archive frozen30 source differs")
    history = run / "history.jsonl"; require(history.stat().st_size <= 128 * 1024**2, "bounded full20k journal required")
    payload = history.read_bytes(); require(hashlib.sha256(payload).hexdigest() == files["history.jsonl"]["sha256"], "history changed before parsing")
    final_meta = document(run, files, "exports/step_020000/metadata.json")
    journal = journal_namespace(code)["verify_history"](payload, declaration, final_meta)
    prefix = hashlib.sha256(); history_prefix = {}
    for index, line in enumerate(payload.splitlines(keepends=True), 1):
        prefix.update(line)
        if index % 1000 == 0: history_prefix[index] = prefix.hexdigest()
    frozen_sha = final_meta["frozen_state_sha256"]
    for receipt in fit["checkpoints"]:
        step = receipt["optimizer_updates"]; export = "exports/step_%06d/" % step
        metadata = document(run, files, export + "metadata.json")
        require(files["checkpoints/step_%06d.pt" % step]["sha256"] == receipt["checkpoint_sha256"]
            and files[export + "model_state.npz"]["sha256"] == receipt["npz_sha256"]
            and files[export + "metadata.json"]["sha256"] == receipt["metadata_sha256"]
            and metadata["schema"] == EXPORT and metadata["training_schema"] == TRAINING
            and metadata["optimizer_updates"] == metadata["additional_optimizer_updates"] == step
            and metadata["maximum_optimizer_updates"] == 20000 and metadata["examples_seen"] == 4 * step
            and metadata["known_total_optimizer_updates"] == 12200 + step and metadata["source_sha256"] == sources
            and metadata["declaration_sha256"] == declaration_sha and metadata["history_sha256"] == history_prefix[step]
            and metadata["protocol"] == protocol and metadata["learning_rate"] == learning_rate_config()
            and metadata["frozen_state_sha256"] == frozen_sha and metadata["trainable_state_tensor_count"] == 54
            and metadata["frozen_state_tensor_count"] == 47 and metadata["runtime_loader_implemented"] is False
            and metadata["model_promoted"] is False and metadata["vehicle_control_approved"] is False,
            "all20 checkpoints/export hashes, actual journal prefixes and denied scopes must match")
    last = fit["checkpoints"][-1]
    endpoint = dict(weights_sha256=last["npz_sha256"], metadata_sha256=last["metadata_sha256"],
        checkpoint_sha256=last["checkpoint_sha256"], declaration_sha256=declaration_sha)
    opened = document(run, files, "evaluation_opened_after_final.json")
    require(opened["checkpoint_selection"] is False
        and all(opened["endpoint"].get(k) is True for k in ("all101_numeric_finite", "frozen47_byte_identical", "all20000_actual_journal_rows_verified"))
        and all(opened["endpoint"][k] == last[k] for k in ("npz_sha256", "metadata_sha256", "checkpoint_sha256")),
        "actual final endpoint/journal proof must precede evaluation")
    screens = summary["endpoint_diagnostic_screens"]
    require(screens["diagnostic_screen_count"] == len(screens["checks"]) == len({r["name"] for r in screens["checks"]}) == 74
        and all(type(r["passed"]) is bool for r in screens["checks"])
        and screens["all_diagnostic_screens_passed"] is all(r["passed"] for r in screens["checks"]), "all74 outcomes including failures required")
    for label in ("parent", "final"):
        for cohort in ("recovery_train", "recovery_holdout", "v4", "legacy", "known599", "LEFT"):
            document(run, files, label + "_" + cohort + ".json")
    require(inventory(run) == files and inventory(source) == source_files, "source/run changed during read-only inspection")
    return dict(run=str(run), source=str(source), files=files, source_files=source_files, sources=sources, endpoint=endpoint,
        protocol_sha256=json_sha(protocol), authorization_sha256=authorization_sha, summary_sha256=summary_sha,
        declaration_sha256=declaration_sha, fit_result_sha256=files["fit_result.json"]["sha256"],
        evaluation_boundary_sha256=files["evaluation_opened_after_final.json"]["sha256"],
        history_sha256=files["history.jsonl"]["sha256"], journal=journal,
        diagnostic_pass_count=sum(r["passed"] for r in screens["checks"]), diagnostic_screen_count=74)


def remote_command(args):
    code = journal_code()
    program = "import hashlib,json,math,os,stat\nfrom pathlib import Path\nfrom types import SimpleNamespace\n"
    for name in ("TRAINING", "EXPORT", "TRAINER", "TRAINER_SHA", "LEFT_SHA", "PARENT", "PARENT_HISTORY"):
        program += name + "=" + repr(globals()[name]) + "\n"
    names = (require, canonical, digest, inventory, is_sha, unique, finite_json_number, invalid_json_constant,
        json_sha, read, learning_rate_config, journal_namespace, inspect_run)
    program += "\n".join(inspect.getsource(fn) for fn in names)
    inputs = [str(RUN), str(SOURCE), args.expected_summary_sha256, args.expected_declaration_sha256, args.authorization_sha256, code]
    program += "\nprint(json.dumps(inspect_run(*json.loads(" + repr(json.dumps(inputs)) + ")),sort_keys=True,allow_nan=False))"
    return SSH + [shlex.join(["env", "-u", "PYTHONPATH", "-u", "PYTHONHOME", "-u", "PYTHONUSERBASE", "-u", "PYTHONSTARTUP",
        "PYTHONNOUSERSITE=1", "CUDA_VISIBLE_DEVICES=", str(PYTHON), "-I", "-B", "-c", program])]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("expected-summary-sha256", "expected-declaration-sha256", "authorization-sha256", "expected-tool-sha256"):
        parser.add_argument("--" + name, required=True)
    args = parser.parse_args(argv)
    require(all(is_sha(v) for v in vars(args).values()), "external SHA256 pins required")
    require(digest(__file__) == args.expected_tool_sha256 and all(not canonical(p).exists() for p in (LOCAL, RECEIPT)), "reviewed source/fresh destinations required")
    journal_code(); RECEIPT.mkdir(parents=True)
    frozen.write_new(RECEIPT / "started.json", dict(schema=SCHEMA, remote_run=str(RUN), remote_mutated=False, optimizer_started=False))
    with (RECEIPT / Path(__file__).name).open("xb") as stream: stream.write(Path(__file__).read_bytes())
    def invoke(command, label):
        try: result = subprocess.run(command, capture_output=True, timeout=1800)
        except subprocess.TimeoutExpired as error:
            # HH_261002 - Preserve partial raw streams even when a bounded remote read/transfer times out.
            result = SimpleNamespace(returncode=None, stdout=error.stdout or b"", stderr=error.stderr or b"")
        for suffix, payload in (("stdout", result.stdout), ("stderr", result.stderr)):
            with (RECEIPT / (label + "_" + suffix + ".txt")).open("xb") as stream: stream.write(payload)
        require(result.returncode == 0, "read-only mirror stage failed: " + label); return result.stdout
    try:
        before = json.loads(invoke(remote_command(args), "remote_before"))
        required = 5 * 1024**3 + 128 * 1024**2 + sum(v["size"] for v in before["files"].values())
        require(shutil.disk_usage(LOCAL.parent).free >= required, "5GiB floor/full mirror/write reserve required; no deletion")
        for name, pin in before["sources"].items(): require(digest(ROOT / name) == pin, "local frozen30 source differs")
        LOCAL.mkdir()
        invoke(["rsync", "--recursive", "--perms", "--protect-args", "--no-links", "--ignore-existing", "-e",
            "ssh -o BatchMode=yes -o ConnectTimeout=15", "--", "pro6000-training:" + str(RUN) + "/", str(LOCAL) + "/"], "rsync")
        after = json.loads(invoke(remote_command(args), "remote_after"))
        require(before == after and inventory(LOCAL) == before["files"], "remote-before/local/remote-after full census differs")
        require(shutil.disk_usage(LOCAL.parent).free >= 5 * 1024**3 + 128 * 1024**2, "free floor/write reserve changed; retain partial")
        for name, pin in before["sources"].items(): require(digest(ROOT / name) == pin, "local frozen30 source changed")
        journal_code(); require(digest(__file__) == digest(RECEIPT / Path(__file__).name) == args.expected_tool_sha256, "mirror source changed")
        frozen.write_new(RECEIPT / "completed_run_for_root_review.json", dict(before, schema=SCHEMA, local_run=str(LOCAL),
            status="MIRROR_COMPLETE_ROOT_SEAL_REQUIRED", numeric_state_validated=False, simulator_control_authorized=False,
            model_promoted=False, vehicle_control_approved=False, source_tool_sha256=args.expected_tool_sha256, helper_source_sha256=HELPERS))
    except Exception as error:
        frozen.write_new(RECEIPT / "failure_receipt.json", dict(schema=SCHEMA, error=type(error).__name__ + ": " + str(error),
            partial_mirror_retained=True, remote_mutated=False, model_promoted=False)); raise
    print(json.dumps(dict(status="MIRROR_COMPLETE_ROOT_SEAL_REQUIRED", run=str(LOCAL), receipt=str(RECEIPT))))
    return 0


if __name__ == "__main__": raise SystemExit(main())
