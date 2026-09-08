"""HH_260906 - Test publication with small explicit synthetic fixtures, never by rerunning the 600-candidate primitive."""

import json
from pathlib import Path
import struct

import pytest

from scripts.e2e import curate_stop_primitive_probe as module


def encode(value):
    return (json.dumps(value, sort_keys=True, allow_nan=False) + "\n").encode()


def save_bundle(root, documents):
    for name, raw in documents.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)
    manifest = "".join(f"{module.digest(raw)}  {name}\n" for name, raw in sorted(documents.items()))
    (root / "SHA256SUMS").write_text(manifest)
    return module.digest(documents["summary.json"]), module.digest(manifest.encode())


@pytest.fixture
def fixture_factory(tmp_path, monkeypatch):
    def build(full_plot_grid=False):
        # HH_260906 - These small mocked records test authentication/curation, not independent physical correctness.
        monkeypatch.setattr(module, "SPEEDS", (.4, 4.) if full_plot_grid else (.4,))
        monkeypatch.setattr(module, "DURATION_LOGITS", (-20., -4., 0., 4., 20.) if full_plot_grid else (0.,))
        monkeypatch.setattr(module, "DTYPES", ("float32", "float64") if full_plot_grid else ("float32",))
        root = tmp_path / ("full_synthetic_fixture" if full_plot_grid else "small_synthetic_fixture")
        documents = {"source_archive/" + name: ("synthetic fixture: " + name).encode() for name in module.SOURCE_NAMES}
        pins = {name: {"sha256": module.digest(documents["source_archive/" + name]),
            "size_bytes": len(documents["source_archive/" + name])} for name in module.SOURCE_NAMES}
        monkeypatch.setattr(module, "RUNTIME_SHA", pins["portable_e2e/runtime_contract.py"]["sha256"])
        rows = []
        for index, (dtype, requested, logit) in enumerate(module.expected_cases()):
            epsilon = 2. ** (-23 if dtype == "float32" else -52)
            code = "f" if dtype == "float32" else "d"
            effective = struct.unpack(code, struct.pack(code, requested))[0]
            candidates = []
            for slot, name in enumerate(module.PROFILE_NAMES):
                candidates.append({"candidate_index": slot, "profile": name,
                    "runtime_selection_logits": [float(i == slot) for i in range(6)],
                    "stop_duration_s": 1., "speed_mps": [0.] * 64, "heading_rad": [0.] * 64,
                    "xy_base_m": [[0., 0.] for _ in range(64)],
                    "runtime_gate": {"status": "PASS", "reason": None, "selected_index": slot},
                    "independent_reference": {"status": "MATCH", "compared_scalar_count": 257,
                        "absolute_tolerance": 64 * epsilon, "relative_tolerance": 64 * epsilon,
                        "maximum_absolute_difference": dict.fromkeys(module.COMPONENTS, 0.),
                        "mismatch_scalar_count": 0, "mismatches": []},
                    "numerical_invariants": {"status": "PASS", "flags": dict.fromkeys(module.FLAG_NAMES, True),
                        "arithmetic_slack": 256 * epsilon, "minimum_speed_rate_mps2": 0.,
                        "maximum_speed_rate_mps2": 0., "discrete_path_distance_m": 0.,
                        "endpoint_speed_integral_m": 0., "maximum_step_integral_disagreement_m": 0.,
                        "continuous_linear_braking_distance_m_reference_only": .5 * effective,
                        "continuous_distance_is_not_the_discrete_output_contract": True}})
            rows.append({"case_index": index, "dtype": dtype, "dtype_epsilon": epsilon,
                "requested_current_speed_mps": requested, "effective_current_speed_mps": effective,
                "duration_logit": logit, "input_tensor_sha256": ["a" * 64] * 3,
                "input_tensors_unchanged": True, "synthetic_only": True, "candidates": candidates})
        profiles = [[0.] * 64, [20.] * 64, [-20.] * 64, [20. if i % 2 == 0 else -20. for i in range(64)],
            [2. * module.math.sin(2. * module.math.pi * i / 63.) for i in range(64)], [.2] * 64]
        count = len(rows)
        plan = {"schema": module.SCHEMA + ".plan", "status": "DECLARED_NOT_EXECUTED", "source_pins": pins,
            "research_constants": module.CONSTANTS, "runtime_config": module.RUNTIME_CONFIG,
            "runtime_gate_id": "portable_e2e.runtime_geometry_gate.v8", "scope": module.SCOPE,
            "current_speed_grid_mps": list(module.SPEEDS), "duration_logits_grid": list(module.DURATION_LOGITS),
            "dtypes": list(module.DTYPES), "case_order": "dtype, current speed, duration logit; all ascending as listed",
            "case_count": count, "candidate_count": count * 6, "future_point_count": count * 384,
            "batch_size": 1, "device": "cpu", "torch_threads": 4,
            "profiles": [{"candidate_index": i, "name": name, "curvature_logits": values}
                for i, (name, values) in enumerate(zip(module.PROFILE_NAMES, profiles))],
            "numerical_comparison": {"reference": "Independent stdlib float64 closed form on dtype-rounded input scalars.",
                "absolute_epsilon_factor": 64, "relative_epsilon_factor": 64,
                "invariant_epsilon_factor": 256, "runtime_tolerances_changed": False}}
        documents["plan.json"] = encode(plan)
        common = {"schema": module.SCHEMA, "scope": module.SCOPE, "source_pins": pins,
            "plan_sha256": module.digest(documents["plan.json"]), "started_utc": "2026-09-08T00:00:00+00:00",
            "device": "cpu", "cuda_visible_devices": "", "torch_threads": 4}
        initial = {**common, "status": "RUNNING"}
        initial.pop("torch_threads")
        documents["started.json"] = encode(initial)
        summary = {**common, "status": "COMPLETE_NOT_ADMITTED", "finished_utc": "2026-09-08T00:00:01+00:00",
            "source_and_plan_postcheck_pass": True, "interruption": {},
            "persisted_row_ledger": {"complete_record_count": count, "unparsed_tail_bytes": 0}, **module.recount(rows)}
        documents["summary.json"] = encode(summary)
        documents["rows.jsonl"] = b"".join(encode(row) for row in rows)
        pins = save_bundle(root, documents)
        return root, documents, rows, pins
    return build


def validate(bundle):
    root, _, _, pins = bundle
    return module.authenticate(root, *pins)


def test_small_fixture_complete(fixture_factory):
    bundle = fixture_factory()
    assert "torch_threads" not in module.read_json(bundle[1]["started.json"])
    report = validate(bundle)
    assert report["recount"]["reported_case_count"] == 1
    assert report["recount"]["reported_candidate_count"] == 6
    assert report["recount"]["compared_scalar_count"] == 1542


@pytest.mark.parametrize("field,value", [("expected_summary_sha256", "f" * 64),
    ("expected_checksums_sha256", "f" * 64), ("expected_summary_sha256", "not-a-sha")])
def test_explicit_authentication_pins_required(fixture_factory, field, value):
    root, _, _, pins = fixture_factory()
    args = dict(zip(("expected_summary_sha256", "expected_checksums_sha256"), pins))
    args[field] = value
    with pytest.raises(ValueError):
        module.authenticate(root, **args)


@pytest.mark.parametrize("name", ["rows.jsonl", "source_archive/portable_e2e/runtime_contract.py", "plan.json"])
def test_corrupt_input_without_new_manifest_rejected(fixture_factory, name):
    bundle = fixture_factory()
    path = bundle[0] / name
    path.write_bytes(path.read_bytes() + b" ")
    with pytest.raises(ValueError):
        validate(bundle)


@pytest.mark.parametrize("change", ["drop_case", "drop_slot", "slot_order", "wrong_case", "wrong_grid", "wrong_epsilon",
    "input_mutated", "nonfinite", "xy_shape", "gate_slot", "gate_reason", "one_hot", "tolerance", "flag_type", "flag_status"])
def test_structural_corruption_rejected_even_with_rehashed_inputs(fixture_factory, change):
    root, documents, rows, _ = fixture_factory()
    row, candidate = rows[0], rows[0]["candidates"][0]
    if change == "drop_case": rows = []
    elif change == "drop_slot": row["candidates"].pop()
    elif change == "slot_order": row["candidates"].reverse()
    elif change == "wrong_case": row["case_index"] = 1
    elif change == "wrong_grid": row["requested_current_speed_mps"] = 1.
    elif change == "wrong_epsilon": row["dtype_epsilon"] = 1.
    elif change == "input_mutated": row["input_tensors_unchanged"] = False
    elif change == "nonfinite": candidate["xy_base_m"][0][0] = "nan"
    elif change == "xy_shape": candidate["xy_base_m"].pop()
    elif change == "gate_slot": candidate["runtime_gate"]["selected_index"] = 1
    elif change == "gate_reason": candidate["runtime_gate"]["reason"] = "contradiction"
    elif change == "one_hot": candidate["runtime_selection_logits"] = [0.] * 6
    elif change == "tolerance": candidate["independent_reference"]["absolute_tolerance"] = 1.
    elif change == "flag_type": candidate["numerical_invariants"]["flags"]["nonnegative_speed"] = 1
    elif change == "flag_status": candidate["numerical_invariants"]["flags"]["nonnegative_speed"] = False
    documents["rows.jsonl"] = b"".join(encode(row) for row in rows)
    pins = save_bundle(root, documents)
    with pytest.raises(ValueError):
        module.authenticate(root, *pins)


def test_failures_are_retained_and_recounted(fixture_factory):
    root, documents, rows, _ = fixture_factory()
    candidate = rows[0]["candidates"][4]
    candidate["runtime_gate"].update(status="FAIL", reason="synthetic fixture failure")
    candidate["numerical_invariants"]["flags"]["terminal_speed_exact_zero"] = False
    candidate["numerical_invariants"]["status"] = "FAIL"
    reference = candidate["independent_reference"]
    reference.update(status="UNVERIFIED", mismatch_scalar_count=1,
        mismatches=[{"component": "speed_mps", "scalar_index": 0, "primitive_value": 0., "reference_value": 1., "absolute_difference": 1.}])
    summary = module.read_json(documents["summary.json"])
    summary.update(module.recount(rows))
    documents.update({"summary.json": encode(summary), "rows.jsonl": b"".join(encode(row) for row in rows)})
    pins = save_bundle(root, documents)
    result = module.authenticate(root, *pins)["recount"]
    assert result["runtime_gate_counts"] == {"PASS": 5, "FAIL": 1}
    assert result["numerical_reference_counts"] == {"MATCH": 5, "UNVERIFIED": 1}
    assert result["runtime_failures"][0]["candidate_index"] == 4


@pytest.mark.parametrize("change", ["incomplete", "denial_changed", "summary_count", "postcheck", "interrupted", "time_order", "unparsed_tail", "threads"])
def test_summary_failure_is_not_upgraded(fixture_factory, change):
    root, documents, _, _ = fixture_factory()
    summary = module.read_json(documents["summary.json"])
    if change == "incomplete": summary["status"] = "RUNNING"
    elif change == "denial_changed": summary["scope"]["training_data_approved"] = True
    elif change == "summary_count": summary["reported_candidate_count"] -= 1
    elif change == "postcheck": summary["source_and_plan_postcheck_pass"] = False
    elif change == "interrupted": summary["interruption"] = {"signal": 15}
    elif change == "time_order": summary["finished_utc"] = "2026-09-07T00:00:00+00:00"
    elif change == "unparsed_tail": summary["persisted_row_ledger"]["unparsed_tail_bytes"] = 2
    elif change == "threads": summary["torch_threads"] = 2
    documents["summary.json"] = encode(summary)
    pins = save_bundle(root, documents)
    with pytest.raises(ValueError):
        module.authenticate(root, *pins)


def test_source_archive_binding_not_live_source(fixture_factory, monkeypatch, tmp_path):
    bundle = fixture_factory()
    monkeypatch.setattr(module, "ROOT", tmp_path / "different_live_checkout")
    assert validate(bundle)["summary"]["status"] == "COMPLETE_NOT_ADMITTED"


def test_archive_change_cannot_hide_behind_rehashed_manifest(fixture_factory):
    root, documents, _, _ = fixture_factory()
    documents["source_archive/portable_e2e/contract.py"] += b" changed"
    pins = save_bundle(root, documents)
    with pytest.raises(ValueError, match="archived source"):
        module.authenticate(root, *pins)


@pytest.mark.parametrize("change", ["extra", "missing", "duplicate_manifest", "symlink", "partial_line"])
def test_inventory_and_rows_are_fail_closed(fixture_factory, change):
    bundle = fixture_factory()
    root, documents, _, pins = bundle
    if change == "extra": (root / "extra.json").write_text("{}")
    elif change == "missing": (root / "started.json").unlink()
    elif change == "duplicate_manifest":
        path = root / "SHA256SUMS"
        path.write_bytes(path.read_bytes() + path.read_bytes().splitlines(keepends=True)[0])
        pins = (pins[0], module.digest(path.read_bytes()))
    elif change == "symlink":
        path = root / "started.json"
        path.unlink(); path.symlink_to(root / "summary.json")
    elif change == "partial_line":
        documents["rows.jsonl"] = documents["rows.jsonl"].rstrip(b"\n")
        pins = save_bundle(root, documents)
    with pytest.raises(ValueError):
        module.authenticate(root, *pins)


@pytest.mark.parametrize("raw", [b'{"a":1,"a":2}', b'{"a":NaN}', b'{"a":1e999}'])
def test_json_rejects_ambiguous_numbers_and_duplicate_keys(raw):
    with pytest.raises(ValueError):
        module.read_json(raw)


def test_output_guard_protects_alias_dataset_target(tmp_path, monkeypatch):
    repository, data = tmp_path / "repo", tmp_path / "actual_dataset"
    repository.mkdir(); data.mkdir()
    (repository / "datasets").symlink_to(data, target_is_directory=True)
    monkeypatch.setattr(module, "ROOT", repository)
    with pytest.raises(ValueError, match="protected"):
        module.checked_output(data / "new_report", tmp_path / "input")
    with pytest.raises(ValueError, match="symlink"):
        module.checked_output(repository / "datasets" / "new_report", tmp_path / "input")


def test_output_guard_no_input_nested_existing_or_symlink(tmp_path):
    source = tmp_path / "input"
    source.mkdir()
    for target in (source, source / "nested"):
        with pytest.raises(ValueError):
            module.checked_output(target, source)
    alias = tmp_path / "alias"
    alias.symlink_to(source, target_is_directory=True)
    with pytest.raises(ValueError):
        module.checked_output(alias / "nested", source)


@pytest.mark.parametrize("raw", [b'/home/account/work', b'/tmp/private', b'ssh://machine',
    b'-----BEGIN PRIVATE KEY-----', b'192.0.2.123'])
def test_public_privacy_check(raw):
    with pytest.raises(ValueError):
        module.public_safe(raw)


def fake_render(rows, output):
    selected, xy = module.plot_selection(rows)
    for name in ("synthetic_stop_speed.png", "synthetic_stop_xy.png"):
        (output / name).write_bytes(b"synthetic test fixture, not actual PNG")
    return {"speed_plot_case_indices": [row["case_index"] for row in selected],
        "xy_plot_case_indices": [row["case_index"] for row in xy]}


def test_publication_copies_exact_full_rows_and_summary(fixture_factory, tmp_path, monkeypatch):
    root, documents, rows, pins = fixture_factory(full_plot_grid=True)
    monkeypatch.setattr(module, "render", fake_render)
    output = tmp_path / "published"
    result = module.publish(root, output, *pins)
    assert (output / "rows.jsonl").read_bytes() == documents["rows.jsonl"]
    assert (output / "summary.json").read_bytes() == documents["summary.json"]
    assert result["full_ledger_recount"]["reported_candidate_count"] == 120
    assert len(result["plots"]["speed_plot_case_indices"]) == 20
    assert len(result["plots"]["xy_plot_case_indices"]) == 2
    assert len((output / "SHA256SUMS").read_text().splitlines()) == 5
    assert len(list(output.iterdir())) == 6
    with pytest.raises(ValueError):
        module.publish(root, output, *pins)


def test_mutation_during_plot_has_no_completion_marker(fixture_factory, tmp_path, monkeypatch):
    root, _, _, pins = fixture_factory(full_plot_grid=True)
    def mutate(rows, output):
        result = fake_render(rows, output)
        (root / "rows.jsonl").write_bytes(b"changed")
        return result
    monkeypatch.setattr(module, "render", mutate)
    output = tmp_path / "incomplete_publication"
    with pytest.raises(ValueError):
        module.publish(root, output, *pins)
    assert not (output / "SHA256SUMS").exists()


def test_actual_renderer_small_synthetic_fixture(fixture_factory, tmp_path, monkeypatch):
    pytest.importorskip("matplotlib")
    from matplotlib.figure import Figure
    original_savefig = Figure.savefig

    def savefig_with_readability_check(figure, *args, **kwargs):
        # HH_260906 - Check the actual layout rather than accepting a PNG with overlapping axis and scope labels.
        figure.canvas.draw()
        renderer = figure.canvas.get_renderer()
        footer_top = figure.texts[-1].get_window_extent(renderer).y1
        for axis in figure.axes:
            assert axis.xaxis.label.get_window_extent(renderer).y0 > footer_top + 3
        return original_savefig(figure, *args, **kwargs)

    monkeypatch.setattr(Figure, "savefig", savefig_with_readability_check)
    _, _, rows, _ = fixture_factory(full_plot_grid=True)
    output = tmp_path / "fixture_charts"
    output.mkdir()
    result = module.render(rows, output)
    assert len(result["speed_plot_case_indices"]) == 20
    assert result["xy_plot_candidate_indices"] == list(range(6))
    for name in ("synthetic_stop_speed.png", "synthetic_stop_xy.png"):
        assert (output / name).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_no_torch_or_model_imports():
    import ast
    tree = ast.parse(Path(module.__file__).read_text())
    imports = [name.name for node in ast.walk(tree) if isinstance(node, ast.Import) for name in node.names]
    imports += [node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)]
    assert not any(name.startswith(("torch", "portable_e2e", "carla")) for name in imports)


def test_cli_requires_both_explicit_pins():
    with pytest.raises(SystemExit) as failure:
        module.main(["--input-root", "input", "--output-dir", "output"])
    assert failure.value.code == 2
