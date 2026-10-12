"""HH_261005 - Check full-seed postcheck metadata against private scalar fixtures only.

No production module, Torch, weights, real process, SSH, GPU, TRAIN dataset or
evaluation payload is imported/read. Remote admission is exercised only up to
the genuine pre-bootstrap boundary with tiny private filesystem documents.
"""

import ast
import copy
import hashlib
import json
import math
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/maintenance/postcheck_three_seed_joint6000_seed1_v1_20261005.py"
PURE_NAMES = {"require", "sha", "encode", "digest", "canonical", "verify_history"}
DOMAINS = ["recovery_noncentered_joint", "recovery_all_joint", "v4_joint", "legacy_raw_curvature_and_speed"]


def tree():
    return ast.parse(SOURCE.read_text())


def execute(body, scope):
    program = ast.fix_missing_locations(ast.Module(body=body, type_ignores=[]))
    exec(compile(program, str(SOURCE), "exec"), scope)


def pure_module():
    module = ModuleType("hh_whole_seed1_scalar_postcheck")
    scope = vars(module)
    scope.update(__file__=str(SOURCE), ast=ast, hashlib=hashlib, json=json, math=math, Path=Path)
    for node in tree().body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and node.targets[0].id == "REMOTE":
            scope["REMOTE"] = ast.literal_eval(node.value)
    execute([node for node in tree().body if isinstance(node, ast.FunctionDef)
        and node.name in PURE_NAMES | {"remote_program"}], scope)
    return module


def history_fixture():
    # HH_261005 - All6000 rows and all24k IDs remain in each fixture; no selected progress/checkpoint-only denominator.
    order = [[step, step + 10000, step + 20000, step + 30000] for step in range(6000)]
    tokens = [["scalar_TRAIN_%s" % ordinal for ordinal in row] for row in order]
    hashes = [hashlib.sha256(json.dumps(row).encode()).hexdigest() for row in order]
    rows = [dict(optimizer_updates=step + 1, examples_seen=4 * (step + 1),
        indices=indices.copy(), sample_ids=ids.copy(), batch_sha256=pin, legacy_curved=step % 2 == 0,
        intrinsic_coefficient=0.0, intrinsic_XY=0.0, loss=1.0, preclip_gradient_norm=2.0,
        recovery0={"total": 1.0}, recovery1={"total": 1.0}, v4={"total": 1.0},
        legacy={"raw_curvature": 1.0, "raw_preferred_speed": 1.0},
        learning_rates={"shared": 1e-5, "heads": 3e-5},
        per_domain_gradients={name: dict(all_present_branch_gradients_finite=True,
            gradient_l2_norm=1.0, unused_branch_tensors=0) for name in DOMAINS},
        elapsed_fit_seconds=(step + 1) / 2) for step, (indices, ids, pin) in enumerate(zip(order, tokens, hashes))]
    return rows, order, tokens, hashes, DOMAINS.copy()


def mutate(value, path, replacement):
    for key in path[:-1]:
        value = value[key]
    value[path[-1]] = replacement


def test_whole6000_history_and_four_domain_denominator():
    m = pure_module(); report = m.verify_history(*history_fixture())
    assert report == dict(optimizer_updates=6000, examples_seen=24000, domain_exposures=[6000] * 4,
        legacy_curved=3000, legacy_straight=3000, all_6000_schedule_rows_exact=True)


@pytest.mark.parametrize("index", [0, 1, 2, 3])
def test_any_truncated_whole_history_input_denied(index):
    m = pure_module(); values = list(history_fixture()); values[index].pop()
    with pytest.raises(RuntimeError, match="denominator"):
        m.verify_history(*values)


@pytest.mark.parametrize("path,value", [
    (("optimizer_updates",), 2), (("optimizer_updates",), True), (("optimizer_updates",), 1.0),
    (("examples_seen",), 5), (("examples_seen",), 4.0), (("indices",), [0, 10000, 20000]),
    (("indices", 0), False), (("indices", 0), 0.0), (("indices", 3), 99999),
    (("sample_ids",), ["only_one"]), (("sample_ids", 0), 0), (("sample_ids", 0), "not_original_token"),
    (("batch_sha256",), "0" * 64), (("legacy_curved",), False), (("legacy_curved",), 1),
    (("intrinsic_coefficient",), 0.25), (("intrinsic_XY",), 0.05),
    (("loss",), float("nan")), (("loss",), float("inf")), (("loss",), True),
    (("preclip_gradient_norm",), -1), (("preclip_gradient_norm",), float("nan")),
    (("intrinsic_coefficient",), False), (("intrinsic_XY",), False),
    (("recovery0", "total"), float("nan")), (("recovery1", "total"), float("inf")),
    (("v4", "total"), True), (("legacy", "raw_curvature"), float("nan")),
    (("learning_rates", "shared"), float("inf")), (("learning_rates", "heads"), True),
    (("recovery0",), {}), (("legacy",), []),
    (("per_domain_gradients", DOMAINS[0], "all_present_branch_gradients_finite"), False),
    (("per_domain_gradients", DOMAINS[0], "all_present_branch_gradients_finite"), 1),
    (("per_domain_gradients", DOMAINS[0], "gradient_l2_norm"), float("nan")),
    (("per_domain_gradients", DOMAINS[0], "gradient_l2_norm"), float("inf")),
    (("per_domain_gradients", DOMAINS[0], "gradient_l2_norm"), -1),
    (("per_domain_gradients", DOMAINS[0], "gradient_l2_norm"), True),
    (("per_domain_gradients", DOMAINS[0], "unused_branch_tensors"), -1),
    (("per_domain_gradients", DOMAINS[0], "unused_branch_tensors"), True),
    (("per_domain_gradients", DOMAINS[0], "unused_branch_tensors"), 0.0),
    (("elapsed_fit_seconds",), -0.5), (("elapsed_fit_seconds",), float("nan")),
    (("elapsed_fit_seconds",), float("inf")), (("elapsed_fit_seconds",), True),
    (("elapsed_fit_seconds",), 5400)])
def test_actual_history_numeric_membership_scope_or_wall_fault(path, value):
    m = pure_module(); values = history_fixture()
    mutate(values[0][0], path, value)
    with pytest.raises(RuntimeError):
        m.verify_history(*values)


@pytest.mark.parametrize("mode", ["elapsed_reverse", "wrong_last_update", "wrong_last_ids", "wrong_last_gradient",
    "domains3", "domains_duplicate", "missing_domain", "extra_domain", "bad_expected_sha"])
def test_tail_or_domain_denominator_fault_is_not_hidden(mode):
    m = pure_module(); rows, order, tokens, hashes, domains = history_fixture()
    if mode == "elapsed_reverse": rows[-1]["elapsed_fit_seconds"] = 0.0
    elif mode == "wrong_last_update": rows[-1]["optimizer_updates"] = 5999
    elif mode == "wrong_last_ids": rows[-1]["sample_ids"][-1] = "bad_tail"
    elif mode == "wrong_last_gradient": rows[-1]["per_domain_gradients"][DOMAINS[-1]]["all_present_branch_gradients_finite"] = False
    elif mode == "domains3": domains.pop()
    elif mode == "domains_duplicate": domains[-1] = domains[0]
    elif mode == "missing_domain": rows[-1]["per_domain_gradients"].pop(domains[-1])
    elif mode == "extra_domain": rows[-1]["per_domain_gradients"]["undeclared_domain"] = {}
    else: hashes[0] = rows[0]["batch_sha256"] = "A" * 64
    with pytest.raises(RuntimeError):
        m.verify_history(rows, order, tokens, hashes, domains)


def test_canonical_hash_and_symlink_path_refusal(tmp_path):
    m = pure_module(); raw = b"all private scalar evidence\x00tail"; path = tmp_path / "scalar.bin"
    path.write_bytes(raw)
    assert m.digest(m.canonical(path)) == hashlib.sha256(raw).hexdigest()
    assert m.sha(raw) == hashlib.sha256(raw).hexdigest()
    link = tmp_path / "alias.bin"; link.symlink_to(path)
    with pytest.raises(RuntimeError): m.canonical(link)
    other = tmp_path / "alias_dir"; other.symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(RuntimeError): m.canonical(other / "scalar.bin")


def test_private_remote_six_functions_have_exact_inverse_AST_and_no_local_imports():
    m = pure_module(); remote = ast.parse(m.remote_program())
    actual = {node.name: ast.dump(node, include_attributes=False) for node in remote.body
        if isinstance(node, ast.FunctionDef) and node.name in PURE_NAMES}
    original = {node.name: ast.dump(node, include_attributes=False) for node in tree().body
        if isinstance(node, ast.FunctionDef) and node.name in PURE_NAMES}
    assert actual == original and len(actual) == 6
    for node in tree().body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            assert not any(item.name in {"torch", "numpy", "carla"} for item in node.names)
    calls = {node.func.id for node in ast.walk(ast.parse(m.REMOTE))
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
    assert not calls & {"forward", "backward", "optimizer", "loss", "evaluate"}


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode())


def prefix_fixture(tmp_path):
    # HH_261005 - Execute the genuine pre-import statements against only tiny private files and a scalar closed-session provider.
    m = pure_module(); scope = vars(m)
    workspace = tmp_path / "private_workspace"; source = workspace / "source"
    plans = workspace / "plans"; seed = workspace / "seed"; receipt = plans / "supervisor"
    control = seed / "control"
    for path in (source, receipt, control): path.mkdir(parents=True, exist_ok=True)
    runtime = {}; entries = []
    for ordinal in range(33):
        name = "learning/scalar_source_%02d.py" % ordinal
        raw = ("scalar provenance %s" % ordinal).encode(); path = source / name
        path.parent.mkdir(exist_ok=True); path.write_bytes(raw)
        pin = m.sha(raw); entries.append(dict(path=name, bytes=len(raw), sha256=pin))
        if ordinal < 31: runtime[name] = pin
    contract = dict(sources=runtime)
    grant = dict(contract=contract)
    for name, raw in (("ROOT_CPU_TRAIN_only_authorization.json", b"{}"),
        ("CPU_admission.stdout", b"scalar CPU admission complete"), ("sealed_sources.tar", b"scalar archive, not a real tar")):
        (plans / name).write_bytes(raw)
    write_json(plans / "ROOT_GPU0_TRAIN_only_authorization.json", grant)
    leader = dict(pid=987654, ppid=987653, pgid=987654, session=987654, start_ticks=123456)
    started = dict(leader=leader, argv=["synthetic_private_python", "--seed", "2026100501"],
        environment_override={"CUDA_VISIBLE_DEVICES": "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"})
    first = dict(status="COMPLETE_SEED1_FIXED6000_NOT_PROMOTED", exit_code=0, leader=leader,
        complete_raw_stdout_stderr_retained=True, remaining_session=[], unknown_session_members=[],
        post_current_archive_CPU_GPU_seals_exact=True, current_archive_sources_CPU_GPU_grants_unchanged=True,
        artifacts={})
    for name, raw in (("fit.stdout", b"whole scalar raw stdout\n"), ("fit.stderr", b"whole scalar raw stderr\n")):
        (receipt / name).write_bytes(raw)
        first["artifacts"][name] = dict(bytes=len(raw), sha256=m.sha(raw))
    write_json(receipt / "result.json", first); write_json(receipt / "child_started.json", started)
    result = dict(arm="control", completed_fixed_endpoint=True, optimizer_updates=6000, examples_seen=24000)
    declaration = dict(protocol=contract, authorization_sha256=m.digest(plans / "ROOT_GPU0_TRAIN_only_authorization.json"),
        evaluation_only_payloads_opened=False, model_promoted=False, vehicle_control_approved=False)
    summary = dict(status="FIXED6000_SEED_TRAIN_COMPLETE_NOT_PROMOTED", seed=2026100501, fit=result,
        evaluation_only_payloads_opened=False, model_promoted=False, vehicle_control_approved=False,
        formal_main_approved=0, formal_sub_approved=0)
    for path, value in ((control / "fit_result.json", result), (control / "fresh_initialization.json", {}),
        (seed / "declaration.json", declaration), (control / "declaration.json", dict(declaration, arm="control")),
        (seed / "summary.json", summary)):
        write_json(path, value)
    package = dict(expected_remote_result_sha256=m.digest(receipt / "result.json"),
        expected_fit_stdout_sha256=m.digest(receipt / "fit.stdout"), expected_fit_stderr_sha256=m.digest(receipt / "fit.stderr"),
        expected_fit_result_sha256=m.digest(control / "fit_result.json"),
        CPU_authority_sha256=m.digest(plans / "ROOT_CPU_TRAIN_only_authorization.json"),
        GPU_authority_sha256=m.digest(plans / "ROOT_GPU0_TRAIN_only_authorization.json"),
        CPU_admission_sha256=m.digest(plans / "CPU_admission.stdout"), archive_sha256=m.digest(plans / "sealed_sources.tar"),
        contract_sha256=m.sha(m.encode(contract)), entries=entries)
    scope.update(package=package, workspace=workspace, source=source, plans=plans, seed=seed, receipt=receipt,
        control=control, checks={}, report={}, pathlib=SimpleNamespace(Path=Path),
        os=SimpleNamespace(environ={"CUDA_VISIBLE_DEVICES": ""}),
        sys=SimpleNamespace(prefix=str(workspace / "venvs/py312")), session_rows=lambda owned: [])
    remote = ast.parse(m.REMOTE)
    execute([node for node in remote.body if isinstance(node, ast.FunctionDef) and node.name == "read"], scope)
    whole_try = next(node for node in remote.body if isinstance(node, ast.Try))
    cutoff = next(index for index, node in enumerate(whole_try.body)
        if isinstance(node, ast.Assign) and ast.unparse(node).startswith("spec = importlib.util.spec_from_file_location"))
    prefix = ast.FunctionDef(name="admit_before_numeric_import", args=ast.arguments(posonlyargs=[], args=[],
        kwonlyargs=[], kw_defaults=[], defaults=[]), body=[*whole_try.body[:cutoff], ast.Return(value=ast.Constant(True))],
        decorator_list=[])
    execute([prefix], scope)
    return m, scope, dict(first=first, started=started, result=result, declaration=declaration, summary=summary,
        package=package, receipt=receipt, seed=seed, control=control, source=source)


def test_authentic_metadata_prefix_admits_only_closed_exact_seed_before_import(tmp_path):
    m, scope, fixture = prefix_fixture(tmp_path)
    assert scope["admit_before_numeric_import"]() is True
    assert scope["checks"]["owned_session_closed"] is True
    assert scope["checks"]["complete_raw_streams_exact"] is True
    assert "trainer" not in scope and "bootstrap" not in scope


@pytest.mark.parametrize("mode", ["CPU_device", "wrong_venv", "raw_stdout", "raw_stderr", "supervisor_raw",
    "fit_result_raw", "supervisor_fail", "supervisor_nonzero", "leader_mismatch", "incomplete_raw",
    "remaining_session", "unknown_session", "actual_session_live", "wrong_seed", "wrong_GPU",
    "CPU_grant", "GPU_grant", "CPU_admission", "archive", "source33", "source31_count",
    "entry32", "archive_after_false", "source_after_false", "artifact_size", "summary_running",
    "summary_seed2", "fit_partial", "control_declaration", "evaluation", "promotion", "formal_approval"])
def test_authentic_preimport_refusal_never_reaches_numeric_bootstrap(tmp_path, mode):
    m, scope, f = prefix_fixture(tmp_path)
    changed_result = False
    if mode == "CPU_device": scope["os"].environ["CUDA_VISIBLE_DEVICES"] = "0"
    elif mode == "wrong_venv": scope["sys"].prefix = str(tmp_path / "another_venv")
    elif mode in {"raw_stdout", "raw_stderr", "supervisor_raw", "fit_result_raw"}:
        key = {"raw_stdout": "expected_fit_stdout_sha256", "raw_stderr": "expected_fit_stderr_sha256",
            "supervisor_raw": "expected_remote_result_sha256", "fit_result_raw": "expected_fit_result_sha256"}[mode]
        f["package"][key] = "0" * 64
    elif mode in {"supervisor_fail", "supervisor_nonzero", "leader_mismatch", "incomplete_raw", "remaining_session",
            "unknown_session", "archive_after_false", "source_after_false", "artifact_size"}:
        key, value = {"supervisor_fail": ("status", "FAILED_RETAINED"), "supervisor_nonzero": ("exit_code", 2),
            "leader_mismatch": ("leader", {}), "incomplete_raw": ("complete_raw_stdout_stderr_retained", False),
            "remaining_session": ("remaining_session", [{}]), "unknown_session": ("unknown_session_members", [{}]),
            "archive_after_false": ("post_current_archive_CPU_GPU_seals_exact", False),
            "source_after_false": ("current_archive_sources_CPU_GPU_grants_unchanged", False),
            "artifact_size": ("artifacts", {"fit.stdout": dict(bytes=0, sha256="0" * 64)})}[mode]
        f["first"][key] = value; write_json(f["receipt"] / "result.json", f["first"])
        f["package"]["expected_remote_result_sha256"] = m.digest(f["receipt"] / "result.json")
    elif mode == "actual_session_live": scope["session_rows"] = lambda leader: [leader]
    elif mode in {"wrong_seed", "wrong_GPU"}:
        if mode == "wrong_seed": f["started"]["argv"][-1] = "2026100502"
        else: f["started"]["environment_override"]["CUDA_VISIBLE_DEVICES"] = "GPU-foreign"
        write_json(f["receipt"] / "child_started.json", f["started"])
    elif mode in {"CPU_grant", "GPU_grant", "CPU_admission", "archive"}:
        f["package"][{"CPU_grant": "CPU_authority_sha256", "GPU_grant": "GPU_authority_sha256",
            "CPU_admission": "CPU_admission_sha256", "archive": "archive_sha256"}[mode]] = "0" * 64
    elif mode == "source33": f["package"]["entries"][-1]["sha256"] = "0" * 64
    elif mode == "source31_count":
        grant_path = scope["plans"] / "ROOT_GPU0_TRAIN_only_authorization.json"
        grant = json.loads(grant_path.read_bytes()); grant["contract"]["sources"].pop(next(iter(grant["contract"]["sources"])))
        write_json(grant_path, grant); f["package"]["GPU_authority_sha256"] = m.digest(grant_path)
        f["package"]["contract_sha256"] = m.sha(m.encode(grant["contract"]))
    elif mode == "entry32": f["package"]["entries"].pop()
    else:
        if mode == "summary_running": f["summary"]["status"] = "RUNNING"
        elif mode == "summary_seed2": f["summary"]["seed"] = 2026100502
        elif mode == "fit_partial": f["result"]["optimizer_updates"] = 5999; changed_result = True
        elif mode == "control_declaration":
            write_json(f["control"] / "declaration.json", dict(f["declaration"], arm="candidate"))
        elif mode == "evaluation": f["summary"]["evaluation_only_payloads_opened"] = True
        elif mode == "promotion": f["summary"]["model_promoted"] = True
        elif mode == "formal_approval": f["summary"]["formal_main_approved"] = 1
        write_json(f["seed"] / "summary.json", f["summary"])
        if changed_result:
            write_json(f["control"] / "fit_result.json", f["result"])
            f["package"]["expected_fit_result_sha256"] = m.digest(f["control"] / "fit_result.json")
    with pytest.raises((RuntimeError, KeyError, ValueError)):
        scope["admit_before_numeric_import"]()
    assert "trainer" not in scope and "bootstrap" not in scope


def test_main_source_requires_external_review_and_manifest_before_any_transport_or_write():
    function = next(node for node in tree().body if isinstance(node, ast.FunctionDef) and node.name == "main")
    text = [ast.unparse(node) for node in function.body]
    review = next(index for index, item in enumerate(text) if "exact ROOT source/test-bound" in item)
    manifest = next(index for index, item in enumerate(text) if "manifest_raw =" in item)
    first_write = next(index for index, item in enumerate(text) if "with started.open('xb')" in item)
    transport = next(index for index, item in enumerate(text) if "subprocess.Popen(" in item)
    assert review < manifest < first_write < transport
    source = ast.unparse(function)
    assert "--review-sha256" in source and "start_new_session=True" in source
    assert "CUDA_VISIBLE_DEVICES=''" in source and "OPENBLAS_NUM_THREADS=1" in source
    assert "expected_fit_stdout_sha256" in source and "expected_fit_stderr_sha256" in source
    assert "identity(child.pid) == owned" in source and "automatic_retry=False" in source
    assert "stdout[:" not in source and "stderr[:" not in source

