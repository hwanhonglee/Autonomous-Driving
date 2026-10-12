#!/usr/bin/env python3
"""HH_261004 - Prepare the same complete CPU counterfactual census for a new semantic mesh.

All 415 actual saved raw fields and four endpoint fixtures remain eligible;
the frozen V1 421 rejections are paired, never overwritten or removed.  This
benchmark changes only the decoder mesh.  It reuses the original geometry,
witness, runtime gate, exact preview fallback and PI math without importing an
active collector.  No actual endpoint driving, NN timing sum or approval claim.
"""

from __future__ import annotations

import argparse
from collections import Counter
import importlib.util
import json
import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning"))
from portable_e2e.navigation_terminal_retiming_v2_20261004 import NavigationTerminalRetimingV2


SCHEMA = "portable_e2e.navigation_terminal_cpu_counterfactual_benchmark.20261004.v2"
FROZEN_V1_SCRIPT = "tools/e2e/benchmark_navigation_terminal_retiming_v1_20261004.py"
V1_OUTPUT = "storage/experiments/diagnostics/2026-10-04/navigation_terminal_counterfactual_benchmark_v1"
REVISION_PINS = {
    FROZEN_V1_SCRIPT: "c10a35989a78ec2da42251cbcc1fc4e8bf47cd1769979f3f441ba518e977861f",
    "learning/tests/test_benchmark_navigation_terminal_retiming_v1_20261004.py": "485d0046cc38ac2e9e6ad4fa9fa569fd784d1275180e04516f6dc9e354d4696e",
    "learning/portable_e2e/terminaldecoder_v2_20261004.py": "596e247bc6c5f5574d5bbba0fc337d82111080a62c3e937d0f147c031d79c23e",
    "learning/portable_e2e/navigation_terminal_retiming_v2_20261004.py": "c51b4fc903a6cdaeda48a49fdef7474dfbbce1992628159e501de5434a9bb5e1",
    V1_OUTPUT + "/prospective_declaration.json": "b687aef7f1db9f2d61ee0cea3ad5dfd8e352b5a0f5e6be9cabffe90606879c97",
    V1_OUTPUT + "/summary.json": "b7e8afbee68d6b0f5785f0a3f4ab640b13f03a4de79c45ed558eac11cd1639fd",
    V1_OUTPUT + "/all_warmup_and_measured_cells.jsonl": "68dbb5a1c4864539c0b36bfc4d2c1de55ebe856047eb28cb33ebbbe4557f0863",
}

# HH_261004 - Load frozen numerical helpers only; the module's main benchmark is never called or patched.
spec = importlib.util.spec_from_file_location("frozen_navigation_terminal_benchmark_v1_numerical_helpers", ROOT / FROZEN_V1_SCRIPT)
frozen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(frozen)
SUPPORTS_M = frozen.SUPPORTS_M


def source_closure():
    actual = frozen.source_closure()
    revision = {name: frozen._sha(frozen.read_canonical_regular(ROOT / name)) for name in REVISION_PINS}
    frozen._require(revision == REVISION_PINS, "frozen V1 evidence or V2 revision source differs")
    actual.update(revision)
    for name in ("learning/tests/test_terminaldecoder_v2_20261004.py",
                 "learning/tests/test_navigation_terminal_retiming_v2_20261004.py",
                 "tools/e2e/benchmark_navigation_terminal_retiming_v2_20261004.py",
                 "learning/tests/test_benchmark_navigation_terminal_retiming_v2_20261004.py"):
        actual[name] = frozen._sha(frozen.read_canonical_regular(ROOT / name))
    return actual


def load_frozen_v1_outcomes():
    # HH_261004 - Every rejected prior slot is retained; no accepted-only selection changes either denominator.
    rows = frozen._lines(ROOT / V1_OUTPUT / "all_warmup_and_measured_cells.jsonl")
    measured = {(row["source_index"], row["declared_counterfactual_support_m"]): (row, digest)
                for row, digest, _ in rows if row["phase"] == "measured"}
    frozen._require(len(rows) == 1664 and len(measured) == 1660
                    and set(measured) == {(i, s) for i in range(415) for s in SUPPORTS_M},
                    "complete frozen V1 415x4 prior denominator required")
    frozen._require(sum(row["accepted"] for row, _ in measured.values()) == 1239,
                    "frozen V1 accepted/rejected census differs")
    return measured


def run_cell(case, support, geometry, decoder, *, phase, tracker=None, prior_outcome=None):
    # HH_261004 - Frozen helper already times the complete witness->decoder->gate->preview->PI scope without old NN sums.
    row = frozen.run_cell(case, support, geometry, decoder, phase=phase, tracker=tracker)
    row.update(schema=SCHEMA, semantic_grid_revision="V2 semantic partition; inherited V1 physical gates",
               original_raw_fields_changed=False, authentic_route_geometry_changed=False)
    if prior_outcome is not None:
        old, digest = prior_outcome
        frozen._require(old["source_index"] == case["source_index"] and old["declared_counterfactual_support_m"] == support
                        and old["source_line_sha256"] == case["source_line_sha256"], "prior same-slot source binding differs")
        row["prior_frozen_v1_slot"] = dict(accepted=old["accepted"], rejection_stage=old["rejection_stage"],
            rejection_code=old["rejection_code"], rejection_reason=old["rejection_reason"], result_line_sha256=digest)
    return row


def benchmark(output):
    execution = frozen.configure_cpu()
    closure = source_closure()
    cases, census, geometry = frozen.load_cases()
    prior = load_frozen_v1_outcomes()
    output = frozen.validate_output_path(output)
    output.mkdir(parents=False, exist_ok=False)
    declaration = dict(schema=SCHEMA, status="PROSPECTIVE_BEFORE_ANY_TIMED_CELL",
        provenance_comment="HH_261004 - Freeze a new whole1660 CPU census without hiding any old V1 rejection.",
        source_sha256=closure, execution=execution, source_row_census=census, supports_m=SUPPORTS_M,
        fresh_source_count=415, declared_measured_cell_count=1660, source_order=[dict(source_index=c["source_index"],
            source_line_index=c["source_line_index"], source_frame=c["record"]["inference_frame"], source_line_sha256=c["source_line_sha256"]) for c in cases],
        measured_order="source-major ascending original fresh line order; each17.9,10,5,2.5m; every cell retained including rejection",
        warmup_policy=dict(cell_count=4, source_index=0, supports_m=SUPPORTS_M, passes=1, failures_retained=True, excluded_from_measured_denominator=True),
        frozen_v1_prior=dict(measured_count=1660, accepted_count=1239, rejected_count=421,
            results_sha256=REVISION_PINS[V1_OUTPUT + "/all_warmup_and_measured_cells.jsonl"], every_same_slot_outcome_paired=True),
        terminal_intervention="grid generation only; exact0/s1/2m raw knots/D, uniform interval subdivisions",
        unchanged_scope="original raw NN values, actual D/v0, witness, navigation law, envelope/time/audits, original gate, exact preview fallback and PI",
        counterfactual_pose_only=True, saved_NN_not_inferred_at_counterfactual_pose=True, independent_sequence0_witness_per_cell=True,
        witness_clock_domain="replayed original simulation stamps; now_ns=replayed anchor; not live wall-clock source freshness",
        original_per_frame_PI_integral_before_restored=True, configured_preview_s=.5,
        original_whole_tracker03_preserved_when_STOP_or_non_GO=True, actual_PI_controller_target_cap_mps=2.4,
        measured_scope="geometry/payload/hashes/witness/tensorization/navigation decoder/original gate/preview05 tracker/PI and original diagnostic extraction",
        excluded_scope="source file loading; NN/image preprocessing; sensors/CARLA/control ACK/actuation; disk writes; prior-slot pairing metadata",
        model_weights_changed=False, raw_semantic_heads_used=False, training_data_approved=False,
        old_NN_timing_reported_separately_never_added=True, end_to_end100ms_proven=False, vehicle_control_approved=False,
        main_functions_approved=0, subfunctions_approved=0)
    declaration_sha = frozen.write_json_create(output / "prospective_declaration.json", declaration)
    decoder, tracker = NavigationTerminalRetimingV2(), frozen.PreparedPreview05Tracker()
    measured, warmups = [], []
    with (output / "all_warmup_and_measured_cells.jsonl").open("xb") as stream:
        for phase, inputs in (("warmup", [(cases[0], s) for s in SUPPORTS_M]),
                              ("measured", [(case, s) for case in cases for s in SUPPORTS_M])):
            for case, support in inputs:
                row = run_cell(case, support, geometry, decoder, phase=phase, tracker=tracker,
                    prior_outcome=prior[(case["source_index"], support)] if phase == "measured" else None)
                (warmups if phase == "warmup" else measured).append(row)
                stream.write(frozen._encoded(row)[0] + b"\n")
            stream.flush()
            os.fsync(stream.fileno())
    frozen._require(source_closure() == closure, "source closure changed during V2 benchmark")
    frozen._require(len(measured) == 1660, "complete1660 measured denominator required")
    transitions = Counter((r["prior_frozen_v1_slot"]["accepted"], r["accepted"]) for r in measured)
    report = dict(schema=SCHEMA, status="COMPLETED_CPU_COUNTERFACTUAL_NUMERICAL_OVERHEAD_ONLY",
        prospective_declaration_sha256=declaration_sha, execution=execution, source_sha256_unchanged=True,
        measured_count=1660, accepted_count=sum(r["accepted"] for r in measured), rejected_count=sum(not r["accepted"] for r in measured),
        warmup_count=4, warmup_rejected_count=sum(not r["accepted"] for r in warmups),
        frozen_preview_tracker_AST_proof=tracker.ast_proof,
        prior_frozen_v1_rejections_retained=421,
        paired_v1_to_v2=dict(accept_to_accept=transitions[(True, True)], accept_to_reject=transitions[(True, False)],
                            reject_to_accept=transitions[(False, True)], reject_to_reject=transitions[(False, False)]),
        preview_fallback_census=dict(Counter(r["preview05_tracker_receipt"]["fallback_reason"] or "GO_positive_effective05"
            for r in measured if r["accepted"])),
        rejection_census=dict(Counter(r["rejection_stage"] + ":" + r["rejection_code"] for r in measured if not r["accepted"])),
        measured_all_cell_overhead_seconds=frozen._summary(r["measured_overhead_seconds"] for r in measured),
        measured_accepted_only_overhead_seconds=frozen._summary(r["measured_overhead_seconds"] for r in measured if r["accepted"]),
        per_support={str(s): dict(count=415, accepted_count=sum(r["declared_counterfactual_support_m"] == s and r["accepted"] for r in measured),
            overhead_seconds=frozen._summary(r["measured_overhead_seconds"] for r in measured if r["declared_counterfactual_support_m"] == s)) for s in SUPPORTS_M},
        original_raw_NN_seconds_separate=frozen._summary(c["record"]["raw_nn_seconds"] for c in cases),
        original_policy_total_seconds_separate=frozen._summary(c["record"]["total_seconds"] for c in cases),
        old_and_new_timings_summed=False, actual_endpoint_driving_proven=False, sensor_to_plan100ms_proven=False,
        gates_unchanged=True, model_or_weights_changed=False, learned_STOP_claim=False, training_data_approved=False,
        vehicle_control_approved=False, main_functions_approved=0, subfunctions_approved=0,
        results_sha256=frozen._sha(frozen.read_canonical_regular(output / "all_warmup_and_measured_cells.jsonl")))
    frozen.write_json_create(output / "summary.json", report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="new canonical diagnostics directory; parent must exist")
    report = benchmark(parser.parse_args().output)
    print(json.dumps({key: report[key] for key in ("status", "measured_count", "accepted_count", "rejected_count",
        "paired_v1_to_v2", "measured_all_cell_overhead_seconds", "per_support", "sensor_to_plan100ms_proven")}, indent=2))


if __name__ == "__main__":
    main()
