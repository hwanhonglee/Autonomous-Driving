"""HH_261005 - Prepare a separately authorized fixed-final five-model research evaluator.

The common3000 and GOOD6000 baselines plus all three final6000 exports are
declared before evaluation. No intermediate/best checkpoint, optimizer, TEST,
new head, actuation, promotion, six-arm model A/B or latency claim is allowed.
The frozen metric functions and all71 screens retain their original math.
Private append-only scalar instrumentation preserves completed rows on errors;
remaining rows are explicitly UNEXECUTED, never silently excluded.
"""
from __future__ import annotations

import argparse
import ast
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import shutil
import signal
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.temporal_route_joint_three_seed_final_evaluation.20261005.v1"
AUTH_SCHEMA = SCHEMA + ".ROOT_prospective_evaluation_authority"
STUDY = "temporal_route_joint_three_seed_final_evaluation_20261005_v1"
WORKSPACE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
PERSONAL = WORKSPACE.parent
SEEDS = (2026100501, 2026100502, 2026100503)
ENDPOINTS = ("common3000", "GOOD6000", *("seed_%d" % seed for seed in SEEDS))
COUNTS = dict(recovery_train=2718, recovery_holdout=604, v4=327, legacy=2487,
              known599=599, intrinsic_legacy_TRAIN_only=328)
SOURCE_NAME = "tools/e2e/evaluate_temporal_route_joint_three_seed_final_v1_20261005.py"
TEST_NAME = "learning/tests/test_evaluate_temporal_route_joint_three_seed_final_v1_20261005.py"
LOADER_NAME = "learning/portable_e2e/temporal_route_joint_three_seed_numeric_loader_v1_20261005.py"
LOADER_TEST_NAME = "learning/tests/test_temporal_route_joint_three_seed_numeric_loader_v1_20261005.py"
TRAIN_CONTRACT_SHA = "cdde23e8f6265f49a4009e07be0e2e0cddc8b881ed4dca33307501e36ffd65c1"
FROZEN_PINS = {
    "tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py": "3da40c323c69b1d3e1a64b77f68955b4a8c169a895b5698db312f86d4b23c0c8",
    "tools/e2e/train_temporal_route_joint_20261002.py": "971eb207d79fe6ed7e015da50a5351e9e30e13ce9631f2cf39267214379ae673",
    "tools/e2e/train_temporal_route_recovery_v3_20261001.py": "6e1909f5574cf1449f56c4e0379507369f9a0f8841d05160a8e2cb861985452b",
    LOADER_NAME: "9be58fc9acb1b32f9830f3df18b48216e51834c5dbde11c0c011e82c7f2e9553",
    LOADER_TEST_NAME: "534b46d8ab6387f2e7436551ac2487f4e2e15ac103274f70b263876705d5a93e",
}
MAX_SECONDS, MAX_OUTPUT_BYTES = 7200, 512 * 1024**2
CLOSING_BYTES = 16 * 1024**2
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
BASELINE_EXPECTED = {
    "common3000": dict(weights="0bdd747dffa6e37196e0ee6afb25fa9930d6aba6ca6f787c779ccfbeae3d4dc9",
        metadata="4ace621d8cf1b8d3ce269bb77aabe9e89598f3f750e18ea707a0817086b3dc2d",
        checkpoint="a17544d99fd87467e4afe1c051dd598f72ab9cd0d79794f34d10291e1edce52f",
        declaration="34c8a55028ea7b4cb2678a73701b16425d811fb10dd77afb5554183dd79ef6c5",
        backbone_weights="4e1623e5156b2a19b546f1889d7a70e9c7b312f72fbf7a0e5a99ccda7d8e5735",
        backbone_metadata="88ffb8dcf9d21d0f756cad2c4ff478a74deee3b8e817f72a039a8885484bb4ec"),
    "GOOD6000": dict(weights="b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625",
        metadata="a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
        checkpoint="43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88",
        declaration="0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5"),
}


def require(value, message):
    if not value:
        raise RuntimeError(message)


def is_sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def encode(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def json_sha(value):
    return hashlib.sha256(encode(value)).hexdigest()


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            result.update(block)
    return result.hexdigest()


def canonical(path, *, boundary=None):
    path = Path(path)
    require(path.is_absolute() and path == path.resolve(strict=True)
        and not any(p.is_symlink() for p in (path, *path.parents)), "canonical non-symlink path required")
    require(boundary is None or path.is_relative_to(boundary), "path escaped its declared private boundary")
    return path


def document(path, expected_sha):
    path = canonical(path)
    require(path.is_file() and path.stat().st_size <= 32 * 1024**2 and is_sha(expected_sha)
        and digest(path) == expected_sha, "bounded externally sealed immutable document required")
    def unique(pairs):
        value = {}
        for key, item in pairs:
            require(key not in value, "duplicate JSON key forbidden")
            value[key] = item
        return value
    value = json.loads(path.read_bytes(), object_pairs_hook=unique,
        parse_constant=lambda value: require(False, "nonfinite JSON constant forbidden"))
    require(type(value) is dict, "sealed JSON object required")
    return value


def write_new(path, value):
    with Path(path).open("xb") as stream:
        stream.write(encode(value) + b"\n")


class OutputBudget:
    # HH_261005 - Account for ALL sources, metrics, declaration and journal bytes, reserving a separate failed closing receipt.
    def __init__(self, output):
        self.output, self.bytes_written = Path(output), 0

    def reserve(self, size, *, closing=False):
        require(type(size) is int and size >= 0 and self.bytes_written + size
            <= MAX_OUTPUT_BYTES - (0 if closing else CLOSING_BYTES), "full attempt output/closing reserve exceeded")
        require(shutil.disk_usage(self.output).free >= 5 * 1024**3 + 512 * 1024**2,
            "original private-output free floor/write reserve required")

    def write_raw(self, path, payload, *, closing=False):
        require(type(payload) is bytes and Path(path).is_relative_to(self.output), "exact owned output bytes/path required")
        self.reserve(len(payload), closing=closing)
        with Path(path).open("xb") as stream:
            require(stream.write(payload) == len(payload), "full create-only owned output write required")
        self.bytes_written += len(payload)

    def write(self, path, value, *, closing=False):
        self.write_raw(path, encode(value) + b"\n", closing=closing)


def source_pins(training_sources):
    require(type(training_sources) is dict and len(training_sources) == 31
        and all(is_sha(pin) for pin in training_sources.values()), "actual sealed31 training-source closure required")
    values = dict(training_sources)
    for name in (SOURCE_NAME, TEST_NAME, LOADER_NAME, LOADER_TEST_NAME):
        values[name] = digest(canonical(ROOT / name))
    require(len(values) == 35 and all(values.get(name) == pin for name, pin in FROZEN_PINS.items()),
        "all31 frozen training sources plus four exact evaluator/loader sources required")
    for name, pin in values.items():
        require(type(name) is str and not Path(name).is_absolute() and str(Path(name)) == name
            and all(part not in ("", ".", "..") for part in name.split("/"))
            and digest(canonical(ROOT / name)) == pin, "canonical source closure changed")
    return values


def fixed_contract(training_sources):
    return dict(schema=SCHEMA, source_sha256=source_pins(training_sources), endpoint_order=list(ENDPOINTS),
        training_contract_sha256=TRAIN_CONTRACT_SHA, fixed_final_updates=6000, common_baseline_updates=3000,
        full_rows_per_endpoint=dict(COUNTS), complete_record_denominator=sum(COUNTS.values()) * len(ENDPOINTS),
        screen_count_per_final=71, known_DEV_total_available=665, known_DEV_eligible=599,
        known_DEV_preexisting_exclusions=66, known_DEV_independent=False, recovery_holdout_episode_count=2,
        model_A_B_six_training_runs_performed=False, shared_pretrained_common_lineage=True,
        best_checkpoint_selection=False, automatic_model_selection=False, old_baseline_metrics_reused=False,
        baseline_forward_required=True, original_metric_math_unchanged=True, original71_thresholds_unchanged=True,
        scalar_record_instrumentation="private exact inverse-AST append seam; no global module changes",
        failed_partial_rows_retained=True, unexecuted_rows_explicit=True, GPU_physical_index=0,
        UNEXECUTED_record_notice="no retained completed scalar record; a failing batch may already have executed",
        private_venv=str(WORKSPACE / "venvs/py312"), torch_threads=4,
        OMP_NUM_THREADS="4", MKL_NUM_THREADS="4", OPENBLAS_NUM_THREADS="1",
        maximum_evaluation_seconds=MAX_SECONDS, maximum_output_bytes=MAX_OUTPUT_BYTES,
        closing_reserve_bytes=CLOSING_BYTES, GPU_UUID=GPU_UUID,
        wall_alarm_seconds=MAX_SECONDS, external_owned_process_supervisor_still_required=True,
        attention_backend="same TRAIN math SDP; flash and memory-efficient SDP disabled",
        minimum_free_floor_bytes=5 * 1024**3, write_reserve_bytes=512 * 1024**2,
        optimization_authorized=False, TEST_access_authorized=False, simulator_control_authorized=False,
        vehicle_control_approved=False, model_promoted=False, formal_main_approved=0, formal_sub_approved=0)


def validate_authority(value, proposed):
    require(type(value) is dict and value.get("schema") == AUTH_SCHEMA and value.get("authorized") is True
        and value.get("purpose") == "fixed_final_known_DEV_research_evaluation_only"
        and value.get("contract") == proposed and value.get("contract_sha256") == json_sha(proposed)
        and value.get("all_three_whole_training_postchecks_PASS") is True
        and type(value.get("GPU_physical_index")) is int and value["GPU_physical_index"] == 0
        and all(value.get(key) is False for key in ("model_promoted", "vehicle_control_approved", "TEST_access_authorized")),
        "new exact ROOT evaluation authority after all three whole postchecks required")
    require(type(value.get("training_contract")) is dict
        and json_sha(value["training_contract"]) == TRAIN_CONTRACT_SHA
        and value["training_contract"].get("sources") == {name: proposed["source_sha256"][name]
            for name in value["training_contract"].get("sources", {})}, "immutable original TRAIN contract required")
    require(set(value.get("completed_seeds", {})) == {str(seed) for seed in SEEDS}
        and set(value.get("baseline_assets", {})) == {"common3000", "GOOD6000"}, "all five fixed endpoints required")
    verify_baseline_membership(value["baseline_assets"])


def verify_baseline_membership(base):
    # HH_261005 - Both baseline declarations/checkpoints remain hash-only; omitted original files cannot support a whole-baseline seal claim.
    for name, expected in BASELINE_EXPECTED.items():
        entry = base[name]
        require(type(entry) is dict and all(type(entry.get(field)) is str
            and Path(entry[field]).is_absolute() and str(Path(entry[field])) == entry[field]
            and ".." not in Path(entry[field]).parts for field in expected), "all original six/four baseline paths required")
        mapping = {entry[field]: pin for field, pin in expected.items()}
        require(len(mapping) == len(expected) and entry.get("immutable_input_sha256") == mapping,
            "exact original common6/GOOD4 immutable baseline membership and known SHA values required")


def instrument_records(function, emit, *, stage, guard=None):
    # HH_261005 - Preserve genuine frozen numerical expressions; only journal a row AFTER its original append.
    original = ast.parse(inspect.getsource(function))
    require(len(original.body) == 1 and isinstance(original.body[0], ast.FunctionDef)
        and not original.body[0].decorator_list and function.__closure__ is None, "one genuine closure-free metric function required")
    modified = deepcopy(original)
    class Instrument(ast.NodeTransformer):
        count = 0
        loop_count = 0
        def visit_For(self, node):
            node = self.generic_visit(node)
            if isinstance(node.target, ast.Name) and node.target.id in ("offset", "index"):
                self.loop_count += 1
                node.body.insert(0, ast.copy_location(ast.Expr(ast.Call(ast.Name("__hh_guard", ast.Load()), [], [])), node))
            return node
        def visit_Expr(self, node):
            call = node.value
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute) and call.func.attr == "append" \
                    and isinstance(call.func.value, ast.Name) and call.func.value.id == "records":
                self.count += 1
                dataset = "legacy" if stage == "intrinsic_legacy_TRAIN_only" else "dataset"
                extra = ast.Expr(ast.Call(ast.Name("__hh_emit", ast.Load()), [ast.Name(dataset, ast.Load()),
                    ast.Subscript(ast.Name("records", ast.Load()), ast.UnaryOp(ast.USub(), ast.Constant(1)), ast.Load()),
                    ast.Constant(stage)], []))
                return [node, ast.copy_location(extra, node)]
            return node
    instrument = Instrument(); modified = instrument.visit(modified)
    require(instrument.count == 1, "exactly one original records.append seam required")
    inverse = deepcopy(modified)
    class Remove(ast.NodeTransformer):
        def visit_Expr(self, node):
            if isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name) and node.value.func.id in ("__hh_emit", "__hh_guard"):
                return None
            return node
    inverse = Remove().visit(inverse)
    require(ast.dump(inverse, include_attributes=False) == ast.dump(original, include_attributes=False),
        "scalar instrumentation must invert exactly to frozen numerical AST")
    private = dict(function.__globals__); private["__hh_emit"] = emit
    private["__hh_guard"] = guard if guard is not None else (lambda: None)
    exec(compile(ast.fix_missing_locations(modified), function.__code__.co_filename, "exec"), private)
    return private[function.__name__]


class Journal:
    def __init__(self, output, expected, clock=time.monotonic, budget_seconds=MAX_SECONDS, output_budget=None):
        self.output, self.expected, self.clock = Path(output), expected, clock
        self.started, self.budget_seconds = clock(), budget_seconds
        self.current, self.datasets, self.finished = None, {}, {}
        self.output_budget = output_budget
        self.last_unretained_candidate = None
        self.stream = (self.output / "all_completed_scalar_records.jsonl").open("xb")
        self.bytes_written = 0

    def bind_datasets(self, datasets):
        require(set(datasets) == set(COUNTS) - {"intrinsic_legacy_TRAIN_only"}, "all five exact admitted cohort datasets required")
        self.datasets = {id(value): key for key, value in datasets.items()}
        require(len(self.datasets) == 5, "cohort objects must be distinct")

    def select(self, endpoint):
        self.guard()
        require(endpoint in ENDPOINTS and endpoint not in self.finished, "each declared endpoint evaluated once in fixed order")
        require(endpoint == ENDPOINTS[len(self.finished)], "fixed five-endpoint order required")
        self.current = endpoint; self.finished[endpoint] = {key: 0 for key in COUNTS}

    def guard(self):
        # HH_261005 - Check before the next genuine batch/anchor; a completed late row is still written before rejection.
        require(self.clock() - self.started <= self.budget_seconds, "prospective evaluation wall limit exceeded before next batch")
        if self.output_budget is not None: self.output_budget.reserve(0)

    def emit(self, dataset, row, stage):
        self.last_unretained_candidate = dict(endpoint=self.current, stage=stage, actual_row_repr=repr(row))
        require(self.current is not None and type(row) is dict, "actual metric row after original append required")
        require(stage in ("dataset", "intrinsic_legacy_TRAIN_only"), "one original declared journal stage required")
        require(stage != "intrinsic_legacy_TRAIN_only" or self.datasets.get(id(dataset)) == "legacy",
            "intrinsic TRAIN rows must bind the same actual admitted legacy dataset")
        cohort = stage if stage == "intrinsic_legacy_TRAIN_only" else self.datasets.get(id(dataset))
        require(cohort in COUNTS, "actual admitted dataset/cohort identity required")
        offset = self.finished[self.current][cohort]
        expected = self.expected[self.current][cohort]
        require(type(row.get("index")) is int and type(row.get("token")) is str
            and offset < len(expected) and (row.get("index"), row.get("token")) == tuple(expected[offset]),
            "all original row indices/tokens/order required; no retry/drop")
        payload = encode(dict(endpoint=self.current, cohort=cohort, ordinal=offset, record=row)) + b"\n"
        require(self.bytes_written + len(payload) <= MAX_OUTPUT_BYTES // 2, "bounded full scalar journal quota required")
        if self.output_budget is not None: self.output_budget.reserve(len(payload))
        require(self.stream.write(payload) == len(payload), "full scalar record write required"); self.stream.flush()
        if self.output_budget is not None: self.output_budget.bytes_written += len(payload)
        self.bytes_written += len(payload); self.finished[self.current][cohort] += 1
        self.last_unretained_candidate = None
        # HH_261005 - Keep the actual completed row even if the declared deadline rejects its continuation.
        require(self.clock() - self.started <= self.budget_seconds, "prospective evaluation wall limit exceeded; failed tail retained")
        require(shutil.disk_usage(self.output).free >= 5 * 1024**3 + 512 * 1024**2,
            "original private-output free floor/write reserve required")

    def receipt(self):
        rows = {endpoint: {cohort: dict(expected=count,
            completed=self.finished.get(endpoint, {}).get(cohort, 0),
            UNEXECUTED=count-self.finished.get(endpoint, {}).get(cohort, 0)) for cohort, count in COUNTS.items()}
            for endpoint in ENDPOINTS}
        return dict(full_denominator=sum(COUNTS.values()) * len(ENDPOINTS), endpoints=rows,
            full_completed_scalar_journal_bytes=self.bytes_written, failed_partial_rows_retained=True,
            last_unretained_actual_candidate=self.last_unretained_candidate,
            UNEXECUTED_record_notice="no retained completed scalar record; a failing batch may already have executed",
            all_completed=all(row["UNEXECUTED"] == 0 for value in rows.values() for row in value.values()))

    def close(self):
        self.stream.close()


def instrumented_evaluator(pair, journal):
    previous = SimpleNamespace(**vars(pair.previous))
    previous.evaluate_joint = instrument_records(pair.previous.evaluate_joint, journal.emit, stage="dataset", guard=journal.guard)
    previous.evaluate_legacy_raw = instrument_records(pair.previous.evaluate_legacy_raw, journal.emit, stage="dataset", guard=journal.guard)
    function = instrument_records(pair.evaluate, journal.emit, stage="intrinsic_legacy_TRAIN_only", guard=journal.guard)
    function.__globals__["previous"] = previous
    require(previous.evaluate_joint.__globals__["__hh_emit"] == journal.emit
        and previous.evaluate_legacy_raw.__globals__["__hh_emit"] == journal.emit
        and function.__globals__["previous"] is previous, "actual nested cohort journal bindings required")
    return function


def verify_pins(pins):
    require(all(digest(canonical(path, boundary=PERSONAL)) == pin for path, pin in pins.items()),
        "one actual current immutable source/endpoint/evaluation input changed")


def run(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--authority", type=Path, required=True)
    parser.add_argument("--authority-sha256", required=True)
    args = parser.parse_args(argv)
    # HH_261005 - No Torch, model, evaluation-only payload or output is accessed before a distinct externally sealed ROOT grant.
    authority = document(args.authority, args.authority_sha256)
    proposed = fixed_contract(authority.get("training_contract", {}).get("sources"))
    validate_authority(authority, proposed)
    canonical(args.authority, boundary=PERSONAL)
    output = WORKSPACE / "runs/evaluation" / STUDY
    canonical(output.parent, boundary=PERSONAL)
    require(not output.exists() and not output.is_symlink(), "new create-only private evaluation output required")
    require(shutil.disk_usage(output.parent).free >= 5 * 1024**3 + 512 * 1024**2, "original private-output floor/reserve required")
    require(os.environ.get("OMP_NUM_THREADS") == "4" and os.environ.get("MKL_NUM_THREADS") == "4"
        and os.environ.get("OPENBLAS_NUM_THREADS") == "1" and os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8"
        and os.environ.get("CUDA_VISIBLE_DEVICES") == GPU_UUID, "exact prospectively declared CPU/GPU0 UUID environment required")
    import sys
    require(Path(sys.prefix).resolve() == WORKSPACE / "venvs/py312", "existing private venv only; no install")
    old_handler, old_timer = signal.getsignal(signal.SIGALRM), signal.getitimer(signal.ITIMER_REAL)
    require(old_timer == (0., 0.), "no unrelated alarm may be overwritten")
    output.mkdir(); budget = OutputBudget(output)
    budget.write(output / "declaration.json", dict(schema=SCHEMA, contract=proposed,
        authority_sha256=args.authority_sha256, UTC=datetime.now(timezone.utc).isoformat(), PID=os.getpid()))
    journal = Journal(output, expected={}, output_budget=budget); metrics, pins, models, summary = {}, {}, {}, {}
    def deadline(signum, frame):
        raise TimeoutError("whole evaluation7200-second alarm; all existing partial files retained")
    signal.signal(signal.SIGALRM, deadline); signal.setitimer(signal.ITIMER_REAL, MAX_SECONDS)
    try:
        pins.update({str(ROOT / name): pin for name, pin in proposed["source_sha256"].items()})
        pins[str(args.authority)] = args.authority_sha256
        for name, pin in proposed["source_sha256"].items():
            destination = output / "sources" / name; destination.parent.mkdir(parents=True, exist_ok=True)
            budget.write_raw(destination, canonical(ROOT / name).read_bytes())
            pins[str(destination)] = pin
        verify_pins(pins)
        base = authority["baseline_assets"]
        common = base["common3000"]
        for endpoint in base.values():
            for path, pin in endpoint["immutable_input_sha256"].items(): pins[str(canonical(path, boundary=PERSONAL))] = pin
        verify_pins(pins)
        import torch
        from scripts.e2e import train_temporal_intrinsic_curve_pair_20261002 as pair
        from scripts.e2e import train_temporal_intrinsic_curve_pair_v2_20261002 as gpu_revision
        from portable_e2e import temporal_route_joint_three_seed_numeric_loader_v1_20261005 as loader
        from portable_e2e import temporal_route_joint_export_research_20261002 as old_loader
        require(Path(pair.__file__).resolve() == ROOT / "tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py"
            and Path(loader.__file__).resolve() == ROOT / LOADER_NAME
            and Path(old_loader.__file__).resolve() == ROOT / "learning/portable_e2e/temporal_route_joint_export_research_20261002.py",
            "genuine sealed private evaluator and numeric-loader module paths required")
        torch.set_num_threads(4)
        require(not torch.cuda.is_initialized(), "all five numeric endpoint checks must precede CUDA/evaluation-only data")
        common_args = dict(parent_weights=Path(common["weights"]), parent_metadata=Path(common["metadata"]),
            backbone_parent_weights=Path(common["backbone_weights"]), backbone_parent_metadata=Path(common["backbone_metadata"]))
        models["common3000"] = old_loader.load_joint_parent(common_args["parent_weights"], common_args["parent_metadata"],
            backbone_parent_weights=common_args["backbone_parent_weights"], backbone_parent_metadata=common_args["backbone_parent_metadata"], source_root=ROOT)
        good = base["GOOD6000"]
        require({key: good[key] for key in pair.PARENT} == pair.PARENT, "original fixed GOOD6000 four asset pins required")
        good_owner = dict(schema=old_loader.OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id="HH_261005-fixed-GOOD-baseline",
            purpose="offline_cpu_numeric_validation_only", simulator_control_authorized=False,
            vehicle_control_approved=False, model_promoted=False, fixed_endpoint_updates=6000, **pair.PARENT)
        models["GOOD6000"] = old_loader.load_route_joint_research_npz(Path(good["weights"]), Path(good["metadata"]),
            weights_sha256=pair.PARENT["weights_sha256"], metadata_sha256=pair.PARENT["metadata_sha256"],
            research_owner_binding=good_owner, source_root=ROOT, **common_args)
        for seed in SEEDS:
            entry = authority["completed_seeds"][str(seed)]
            post = document(entry["whole_postcheck_path"], entry["whole_postcheck_sha256"])
            require(post.get("status") == "PASS_WHOLE_SEED%d_FIXED6000_NOT_PROMOTED" % (seed-SEEDS[0]+1)
                and type(post.get("seed")) is int and post["seed"] == seed, "all three actual completed whole postchecks required")
            pins[entry["whole_postcheck_path"]] = entry["whole_postcheck_sha256"]
            binding = document(entry["binding_path"], entry["binding_sha256"])
            pins[entry["binding_path"]] = entry["binding_sha256"]; pins[entry["review_path"]] = entry["review_sha256"]
            for name, pin in binding["files"].items(): pins[str(Path(entry["endpoint_root"]) / name)] = pin
            models["seed_%d" % seed] = loader.load_reviewed_seed_endpoint(Path(entry["endpoint_root"]),
                endpoint_binding_path=entry["binding_path"], endpoint_binding_sha256=entry["binding_sha256"],
                ROOT_review_path=entry["review_path"], ROOT_review_sha256=entry["review_sha256"],
                source_root=ROOT, authorize_reviewed_endpoint=True, **common_args)
        verify_pins(pins)
        require(list(models) == list(ENDPOINTS), "all five fixed CPU numeric endpoints admitted before any evaluation-only payload")
        data = SimpleNamespace(**{name: Path(path) for name, path in authority["dataset_paths"].items()})
        training, audits, low, legacy, strata, train_receipts = pair.load_train(data)
        holdout, holdout_audits, holdout_receipt, known, known_receipt = pair.load_evaluation_only(data, training, low, legacy)
        pins.update(pair.evaluation_artifact_pins(data, known_receipt, holdout, known))
        datasets = dict(recovery_train=training, recovery_holdout=holdout, v4=low, legacy=legacy, known599=known)
        before_images = pair.verify_training_images(tuple(datasets.values()))
        require(len(training) == 2718 and len(holdout) == 604 and len(low) == 327
            and len(strata["eligible"]) == 2487 and len(known) == 665, "original complete cohort denominators required")
        indices = {name: list(range(len(dataset))) for name, dataset in datasets.items()}
        indices["legacy"] = list(strata["eligible"]); indices["known599"] = list(pair.replay.legacy_strata(known)["eligible"])
        require(len(indices["known599"]) == 599, "all original599 knownDEV eligible anchors retained")
        rows = {name: [(i, datasets[name].examples[i].token) for i in chosen] for name, chosen in indices.items()}
        rows["intrinsic_legacy_TRAIN_only"] = [(i, legacy.examples[i].token) for i in pair.curved_indices(strata)]
        require({name: len(value) for name, value in rows.items()} == COUNTS, "all declared original scalar row denominators required")
        journal.expected = {name: deepcopy(rows) for name in ENDPOINTS}; journal.bind_datasets(datasets)
        evaluate = instrumented_evaluator(pair, journal)
        budget.write(output / "evaluation_opened_after_all_three_final.json", dict(whole_postchecks=authority["completed_seeds"],
            endpoint_order=list(models), HOLDOUT=holdout_receipt, known_DEV=known_receipt,
            TRAIN=train_receipts, source_images_before=before_images, independent_validation=False))
        with pair.raw_parent.gpu0_lease():
            gpu = gpu_revision.gpu_preflight(); budget.write(output / "GPU0_started.json", gpu)
            torch.use_deterministic_algorithms(True); torch.backends.cudnn.deterministic = True
            torch.backends.cudnn.benchmark = False; torch.backends.cuda.matmul.allow_tf32 = False
            torch.backends.cudnn.allow_tf32 = False
            if hasattr(torch.backends.cuda, "enable_flash_sdp"):
                torch.backends.cuda.enable_flash_sdp(False); torch.backends.cuda.enable_mem_efficient_sdp(False)
            for endpoint in ENDPOINTS:
                journal.select(endpoint)
                metrics[endpoint] = evaluate(models[endpoint], torch.device("cuda:0"), training, audits,
                    low, legacy, strata, holdout, holdout_audits, known)
                for cohort, value in metrics[endpoint].items(): budget.write(output / (endpoint + "_" + cohort + ".json"), value)
        screens = {name: pair.original.endpoint_screens(metrics["common3000"], metrics[name]) for name in ENDPOINTS[1:]}
        good_comparisons = {name: pair.original.endpoint_screens(metrics["GOOD6000"], metrics[name]) for name in ENDPOINTS[2:]}
        require(all(len(value["checks"]) == 71 for value in (*screens.values(), *good_comparisons.values())), "all71 original failures preserved in each fixed comparison")
        after_train = pair.load_train(data)
        after_holdout, _, after_ho_receipt, after_known, after_known_receipt = pair.load_evaluation_only(data, training, low, legacy)
        require(after_train[-1] == train_receipts and after_ho_receipt == holdout_receipt and after_known_receipt == known_receipt
            and after_holdout.fingerprint_sha256 == holdout.fingerprint_sha256
            and after_known.fingerprint_sha256 == known.fingerprint_sha256, "all evaluation/TRAIN loader fingerprints changed")
        require(pair.verify_training_images(tuple(datasets.values())) == before_images, "all source camera SHA checks before/after required")
        verify_pins(pins); require(journal.receipt()["all_completed"], "full five-endpoint denominator required")
        summary = dict(status="ALL_FIXED_FINAL_OFFLINE_EVALUATION_COMPLETE_NOT_PROMOTED", common3000_screens=screens,
            GOOD6000_reference_screens=good_comparisons, original71_thresholds_unchanged=True,
            all_source_endpoint_data_pins_before_after_exact=True, source_images_before_after=before_images)
        return_code = 0
    except BaseException as error:
        summary = dict(status="FAILED_FIXED_FINAL_EVALUATION_RETAIN_ALL_ROWS", error=repr(error),
            completed_metric_endpoints=list(metrics), remaining_rows_not_relabelled=True)
        return_code = 2
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0); signal.signal(signal.SIGALRM, old_handler)
        journal.close()
        summary.update(schema=SCHEMA, complete_denominator=journal.receipt(), model_promoted=False,
            vehicle_control_approved=False, formal_main_approved=0, formal_sub_approved=0,
            six_arm_model_A_B_claim=False, learned_STOP_claim=False, closed_loop_performance_claim=False,
            current_GOOD_replaced=False, existing_metric_files_never_deleted=True,
            output_bytes_before_closing=budget.bytes_written,
            CUDA_process_closure_requires_external_owned_supervisor=True)
        budget.write(output / "summary.json", summary, closing=True)
    print(json.dumps(dict(status=summary["status"], output=str(output), full_denominator=sum(COUNTS.values()) * len(ENDPOINTS),
        actual_completed=journal.receipt()["all_completed"], formal_main_approved=0, formal_sub_approved=0), allow_nan=False))
    return return_code


if __name__ == "__main__":
    raise SystemExit(run())
