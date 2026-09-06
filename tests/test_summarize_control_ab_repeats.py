from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest
from PIL import Image


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts/e2e/summarize_control_ab_repeats.py"
SPEC = importlib.util.spec_from_file_location("summarize_control_ab_repeats", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _route_bytes() -> bytes:
    return b'{"scenario":"left","town":"C_track_1_0_7"}\n'


def _route_sha256() -> str:
    return hashlib.sha256(_route_bytes()).hexdigest()


def _write_arm(
    root: Path,
    *,
    success: bool,
    selected: bool,
    geometry_assessed: bool = True,
    health: str = "PASS",
    route_bytes: bytes | None = None,
    failure_reason: str | None = None,
) -> None:
    attempt = root / "attempts/attempt_001"
    attempt.mkdir(parents=True)
    source = _route_bytes() if route_bytes is None else route_bytes
    (attempt / "source_route.json").write_bytes(source)
    result = {
        "success": success,
        "reason": (
            "goal reached"
            if success
            else failure_reason
            or (
                "trajectory correction limit exceeded: 15.301 m"
                if geometry_assessed
                else "VAD route manager reported fault:candidate_timeout"
            )
        ),
        "assessment": {
            "trajectory_geometry": "hybrid_route_assisted"
            if geometry_assessed
            else "not_assessed"
        },
        "metrics": {"sim_elapsed_sec": 30.0 if geometry_assessed else 0.0},
    }
    (attempt / "result.json").write_text(json.dumps(result), encoding="utf-8")
    health_payload = {"status": "PASS" if health == "PASS" else "FAIL"}
    health_path = attempt / "runtime_health.json"
    health_path.write_text(json.dumps(health_payload), encoding="utf-8")
    health_sha256 = hashlib.sha256(health_path.read_bytes()).hexdigest()
    (attempt / "runtime.env").write_text(
        "RUNTIME_HEALTH_GATE_ENABLED=true\n"
        f"RUNTIME_HEALTH_GATE_STATUS={health}\n"
        f"RUNTIME_HEALTH_EVIDENCE_SHA256={health_sha256}\n",
        encoding="utf-8",
    )
    summary = {
        "schema_version": 1,
        "status": "PASS" if selected else "FAIL",
        "selected_attempt": "attempt_001" if selected else None,
        "route_sha256": hashlib.sha256(source).hexdigest(),
        "attempts": [
            {
                "attempt_id": "attempt_001",
                "path": str(attempt.resolve()),
                "runtime_health_status": health,
                "process_exit_status": 0 if selected else 1,
            }
        ],
    }
    (root / "owned_trial_summary.json").write_text(
        json.dumps(summary), encoding="utf-8"
    )


def _metrics(offset: float) -> dict[str, float]:
    return {
        metric: float(index + 1) + offset
        for index, metric in enumerate(MODULE.PAIRED_METRICS)
    }


def _make_valid_pairs(tmp_path: Path, count: int = 3) -> list[Path]:
    pairs = []
    for index in range(count):
        pair = tmp_path / f"pair_{index + 1:02d}"
        _write_arm(pair / "A_baseline", success=True, selected=True)
        _write_arm(pair / "B_turn_preview_10m", success=True, selected=True)
        pairs.append(pair)
    return pairs


def _fake_loader(expected_sha256: str):
    def load(path: Path, role: str) -> dict:
        pair_index = int(path.parent.name.rsplit("_", 1)[1])
        offset = float(pair_index)
        if role == "candidate":
            offset += float(pair_index)
        return {"route_sha256": expected_sha256, "metrics": _metrics(offset)}

    return load


def _fake_comparator(decision: str = "ACCEPT"):
    def compare(baseline: dict, candidate: dict, scenario: str, candidate_id: str):
        return {
            "decision": decision,
            "scenario": scenario,
            "candidate_id": candidate_id,
            "checks": {
                "candidate_goal": {"status": "PASS"},
                "turn_raw_gated_improvement": {
                    "status": "PASS" if decision == "ACCEPT" else "FAIL"
                },
            },
        }

    return compare


def test_three_fully_valid_accept_pairs_produce_only_paired_statistics(
    tmp_path: Path,
) -> None:
    pairs = _make_valid_pairs(tmp_path)
    route_sha256 = _route_sha256()

    payload = MODULE.summarize(
        pairs,
        "c_track_turn",
        "turn_preview_10m",
        "B_turn_preview_10m",
        route_sha256,
        loader=_fake_loader(route_sha256),
        comparator=_fake_comparator(),
    )

    assert payload["decision"] == "ACCEPT"
    assert payload["counts"]["valid_pairs"] == 3
    assert payload["counts"]["classifications"] == {"VALID_ACCEPT": 3}
    delta = payload["paired_delta_statistics"]["maximum_cte_m"]
    assert delta == {
        "definition": "candidate_minus_baseline",
        "count": 3,
        "minimum": 1.0,
        "median": 2.0,
        "maximum": 3.0,
    }
    assert all(
        gate["status"] == "PASS"
        for gate in payload["acceptance_gates"].values()
    )
    assert "every valid pair" in payload["acceptance_policy"][
        "aggregate_comparator_policy"
    ]


def test_preengagement_zero_result_is_invalid_and_never_loaded(
    tmp_path: Path,
) -> None:
    pairs = _make_valid_pairs(tmp_path)
    candidate = pairs[1] / "B_turn_preview_10m"
    for path in sorted(candidate.rglob("*"), reverse=True):
        if path.is_file():
            path.unlink()
        elif path.is_dir():
            path.rmdir()
    candidate.rmdir()
    _write_arm(
        candidate,
        success=False,
        selected=False,
        geometry_assessed=False,
    )
    route_sha256 = _route_sha256()
    loaded_paths: list[Path] = []
    base_loader = _fake_loader(route_sha256)

    def loader(path: Path, role: str) -> dict:
        loaded_paths.append(path)
        return base_loader(path, role)

    payload = MODULE.summarize(
        pairs,
        "c_track_turn",
        "turn_preview_10m",
        "B_turn_preview_10m",
        route_sha256,
        loader=loader,
        comparator=_fake_comparator(),
    )

    assert payload["decision"] == "HOLD"
    assert payload["pairs"][1]["classification"] == "INVALID_CANDIDATE_INFRA"
    assert payload["pairs"][1]["valid_for_performance_statistics"] is False
    assert candidate not in loaded_paths
    assert payload["counts"]["valid_pairs"] == 2
    assert all(statistic["count"] == 2 for statistic in payload["paired_delta_statistics"].values())
    candidate_route = payload["arm_gate_summary"]["candidate"]["route"]
    assert candidate_route["counts"] == {"PASS": 2, "FAIL": 1, "NOT_ASSESSED": 0}
    candidate_geometry = payload["arm_gate_summary"]["candidate"]["geometry"]
    assert candidate_geometry["counts"]["NOT_ASSESSED"] == 1


def test_assessed_candidate_route_failure_is_geometry_fail_and_regression(
    tmp_path: Path,
) -> None:
    pairs = _make_valid_pairs(tmp_path)
    candidate = pairs[2] / "B_turn_preview_10m"
    for path in sorted(candidate.rglob("*"), reverse=True):
        if path.is_file():
            path.unlink()
        elif path.is_dir():
            path.rmdir()
    candidate.rmdir()
    _write_arm(candidate, success=False, selected=False, geometry_assessed=True)
    route_sha256 = _route_sha256()

    payload = MODULE.summarize(
        pairs,
        "c_track_turn",
        "turn_preview_10m",
        "B_turn_preview_10m",
        route_sha256,
        loader=_fake_loader(route_sha256),
        comparator=_fake_comparator(),
    )

    pair = payload["pairs"][2]
    assert pair["classification"] == "GEOMETRY_FAIL_CANDIDATE"
    assert pair["candidate"]["infrastructure_status"] == "PASS"
    assert pair["candidate"]["geometry_status"] == "FAIL"
    geometry_gate = payload["acceptance_gates"][
        "candidate_geometry_failure_rate_nonregression"
    ]
    assert geometry_gate["status"] == "FAIL"
    assert geometry_gate["baseline_percent"] == 0.0
    assert geometry_gate["candidate_percent"] == pytest.approx(100.0 / 3.0)


def test_assessed_baseline_route_failure_is_preserved_outside_statistics(
    tmp_path: Path,
) -> None:
    pairs = _make_valid_pairs(tmp_path)
    baseline = pairs[0] / "A_baseline"
    for path in sorted(baseline.rglob("*"), reverse=True):
        if path.is_file():
            path.unlink()
        elif path.is_dir():
            path.rmdir()
    baseline.rmdir()
    _write_arm(baseline, success=False, selected=False, geometry_assessed=True)
    route_sha256 = _route_sha256()

    payload = MODULE.summarize(
        pairs,
        "c_track_turn",
        "turn_preview_10m",
        "B_turn_preview_10m",
        route_sha256,
        loader=_fake_loader(route_sha256),
        comparator=_fake_comparator(),
    )

    pair = payload["pairs"][0]
    assert pair["classification"] == "GEOMETRY_FAIL_BASELINE"
    assert pair["valid_for_performance_statistics"] is False
    assert payload["counts"]["valid_pairs"] == 2
    assert all(
        statistic["count"] == 2
        for statistic in payload["paired_delta_statistics"].values()
    )
    baseline_geometry = payload["arm_gate_summary"]["baseline"]["geometry"]
    assert baseline_geometry["counts"] == {
        "PASS": 2,
        "FAIL": 1,
        "NOT_ASSESSED": 0,
    }


@pytest.mark.parametrize(
    "reason",
    (
        "trajectory correction limit exceeded: 15.301 m",
        "cross-track error limit exceeded: 1.100 m",
        "VAD route manager reported fault:trajectory_correction:correction=15.301m>limit=15.000m:points=6:horizon=5.000m:command=0(LEFT)",
        "VAD route manager reported fault:route_deviation:abs(1.10m)>1.00m",
    ),
)
def test_only_explicit_geometry_reasons_are_geometry_failures(
    tmp_path: Path, reason: str
) -> None:
    pair = tmp_path / "pair_01"
    _write_arm(pair / "A_baseline", success=True, selected=True)
    _write_arm(
        pair / "B_turn_preview_10m",
        success=False,
        selected=False,
        geometry_assessed=True,
        failure_reason=reason,
    )

    arm = MODULE.inspect_arm(
        pair / "B_turn_preview_10m", "candidate", _route_sha256()
    )

    assert arm["disposition"] == "GEOMETRY_FAIL"
    assert arm["geometry_status"] == "FAIL"
    assert arm["infrastructure_status"] == "PASS"


@pytest.mark.parametrize(
    "reason",
    (
        "simulation-time timeout",
        "route progress stalled for 15.0 simulated seconds",
        "maximum lateral acceleration 1.900 m/s^2 exceeds 1.800 m/s^2",
    ),
)
def test_other_assessed_failures_are_route_failures_not_geometry(
    tmp_path: Path, reason: str
) -> None:
    pair = tmp_path / "pair_01"
    _write_arm(pair / "A_baseline", success=True, selected=True)
    _write_arm(
        pair / "B_turn_preview_10m",
        success=False,
        selected=False,
        geometry_assessed=True,
        failure_reason=reason,
    )
    route_sha256 = _route_sha256()

    payload = MODULE.summarize(
        [pair],
        "c_track_turn",
        "turn_preview_10m",
        "B_turn_preview_10m",
        route_sha256,
        minimum_valid_pairs=1,
        loader=_fake_loader(route_sha256),
        comparator=_fake_comparator(),
    )

    arm = payload["pairs"][0]["candidate"]
    assert payload["pairs"][0]["classification"] == "ROUTE_FAIL_CANDIDATE"
    assert arm["disposition"] == "ROUTE_FAIL"
    assert arm["geometry_status"] == "NOT_ASSESSED"
    assert arm["infrastructure_status"] == "PASS"
    assert payload["paired_delta_statistics"] == {}


def test_authoritative_hold_in_any_valid_pair_blocks_each_and_aggregate_policy(
    tmp_path: Path,
) -> None:
    pairs = _make_valid_pairs(tmp_path)
    route_sha256 = _route_sha256()
    calls = 0

    def comparator(baseline: dict, candidate: dict, scenario: str, candidate_id: str):
        nonlocal calls
        calls += 1
        decision = "HOLD" if calls == 2 else "ACCEPT"
        return {
            "decision": decision,
            "checks": {
                "candidate_goal": {"status": "PASS"},
                "turn_raw_gated_improvement": {
                    "status": "FAIL" if decision == "HOLD" else "PASS"
                },
            },
        }

    payload = MODULE.summarize(
        pairs,
        "c_track_turn",
        "turn_preview_10m",
        "B_turn_preview_10m",
        route_sha256,
        loader=_fake_loader(route_sha256),
        comparator=comparator,
    )

    assert payload["decision"] == "HOLD"
    assert payload["counts"]["classifications"] == {
        "VALID_ACCEPT": 2,
        "VALID_HOLD": 1,
    }
    assert payload["acceptance_gates"]["each_pair_comparator_acceptance"][
        "status"
    ] == "FAIL"
    assert payload["acceptance_gates"]["aggregate_comparator_acceptance"][
        "status"
    ] == "FAIL"
    aggregate = payload["aggregate_comparator_checks"][
        "turn_raw_gated_improvement"
    ]
    assert aggregate["pass_count"] == 2
    assert aggregate["valid_pair_count"] == 3


def test_expected_route_hash_mismatch_is_invalid_before_authoritative_load(
    tmp_path: Path,
) -> None:
    pair = _make_valid_pairs(tmp_path, count=1)[0]
    loader_called = False

    def loader(path: Path, role: str) -> dict:
        nonlocal loader_called
        loader_called = True
        raise AssertionError("route-mismatched arms must not be loaded")

    payload = MODULE.summarize(
        [pair],
        "c_track_turn",
        "turn_preview_10m",
        "B_turn_preview_10m",
        "0" * 64,
        minimum_valid_pairs=1,
        loader=loader,
        comparator=_fake_comparator(),
    )

    assert payload["decision"] == "HOLD"
    assert payload["pairs"][0]["classification"] == "INVALID_BOTH_INFRA"
    assert loader_called is False
    assert payload["acceptance_gates"]["all_route_contracts"]["status"] == "FAIL"


def test_candidate_directory_is_explicit_safe_basename(tmp_path: Path) -> None:
    pair = _make_valid_pairs(tmp_path, count=1)[0]
    with pytest.raises(MODULE.SummaryError, match="safe explicit basename"):
        MODULE.summarize(
            [pair],
            "c_track_turn",
            "turn_preview_10m",
            "../B_turn_preview_10m",
            _route_sha256(),
        )


def test_render_writes_readable_summary_png(tmp_path: Path) -> None:
    pairs = _make_valid_pairs(tmp_path)
    route_sha256 = _route_sha256()
    payload = MODULE.summarize(
        pairs,
        "c_track_turn",
        "turn_preview_10m",
        "B_turn_preview_10m",
        route_sha256,
        loader=_fake_loader(route_sha256),
        comparator=_fake_comparator(),
    )
    output = tmp_path / "summary.png"

    MODULE.render(payload, output)

    assert output.is_file()
    with Image.open(output) as image:
        assert image.width >= 2000
        assert image.height >= 800
