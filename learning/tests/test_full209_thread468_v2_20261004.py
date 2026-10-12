"""HH_261004 - Synthetic proof of the single JSON admission repair and fresh scoped worker identity, never an actual replay."""

import json

import pytest

from scripts.e2e import benchmark_full209_thread468_v2_20261004 as b


def identities():
    return {name: getattr(b.original, name) for name in
        ("MODULE", "SCHEMA", "PLAN", "OUTPUT", "HERE", "TEST", "contract", "source_snapshot")}


def test_frozen_V1_failure_is_exactly_tuple_to_JSON_list_in_one_contract_field():
    first = b._ORIGINAL_CONTRACT()
    persisted = json.loads(json.dumps(first, allow_nan=False))
    assert first != persisted
    assert [name for name in first if first[name] != persisted[name]] == ["raw_heads"]
    assert all(type(shape) is tuple for shape in first["raw_heads"].values())
    assert all(type(shape) is list for shape in persisted["raw_heads"].values())


def test_V2_contract_JSON_roundtrip_and_inverse_changes_only_raw_head_representation():
    first, repaired = b._ORIGINAL_CONTRACT(), b.contract()
    assert repaired == json.loads(json.dumps(repaired, allow_nan=False))
    assert [name for name in first if first[name] != repaired[name]] == ["raw_heads"]
    inverse = dict(repaired, raw_heads={name: tuple(shape) for name, shape in repaired["raw_heads"].items()})
    assert inverse == first
    assert repaired["raw_heads"] == {name: list(shape) for name, shape in b.original.RAW_SHAPES.items()}


def test_scoped_activation_restores_every_original_identity_after_exception():
    before = identities()
    with pytest.raises(RuntimeError, match="synthetic retained failure"):
        with b.activated_revision():
            assert identities() == dict(MODULE=b.MODULE, SCHEMA=b.SCHEMA, PLAN=b.PLAN, OUTPUT=b.OUTPUT,
                HERE=b.HERE, TEST=b.TEST, contract=b.contract, source_snapshot=b.source_snapshot)
            raise RuntimeError("synthetic retained failure")
    assert identities() == before


def test_fresh_worker_entry_delegates_unchanged_V1_with_V2_module_and_paths(monkeypatch):
    before, calls = identities(), []
    neural_functions = {name: getattr(b.original, name) for name in
        ("worker", "run_parent", "raw_values", "raw_comparison", "ordinary_decoder_screen",
         "decoder_comparison", "timed_forward", "initialize_worker_runtime", "set_threads_after_load")}
    args = ["--mode", "worker", "--threads", "6", "--expected-declaration-sha256", "synthetic", "--runtime-lease-fd", "9"]
    def original_main(argv):
        calls.append(argv)
        assert b.original.MODULE == b.MODULE and b.original.SCHEMA == b.SCHEMA
        assert b.original.PLAN == b.PLAN and b.original.OUTPUT == b.OUTPUT
        assert b.PLAN != before["PLAN"] and b.OUTPUT != before["OUTPUT"]
        assert b.original.contract() == json.loads(json.dumps(b.original.contract(), allow_nan=False))
        assert neural_functions == {name: getattr(b.original, name) for name in neural_functions}
        return 17
    monkeypatch.setattr(b.original, "main", original_main)
    assert b.main(args) == 17 and calls == [args]
    assert identities() == before


def test_source_closure_keeps_frozen_V1_and_both_fresh_sources(monkeypatch):
    monkeypatch.setattr(b.original.previous, "source_snapshot", lambda: {"synthetic_prior_source": "unchanged"})
    with b.activated_revision():
        pins = b.original.source_snapshot()
    for path, expected in ((b.ORIGINAL_SOURCE, b.ORIGINAL_SOURCE_SHA), (b.ORIGINAL_TEST, b.ORIGINAL_TEST_SHA),
                           (b.HERE, b.original.digest(b.HERE)), (b.TEST, b.original.digest(b.TEST))):
        assert pins[str(path.relative_to(b.original.ROOT))] == expected
    assert pins["synthetic_prior_source"] == "unchanged"


def test_changed_frozen_original_source_is_rejected_without_replay(monkeypatch):
    actual_digest = b.original.digest
    monkeypatch.setattr(b.original, "digest", lambda path: "changed" if path == b.ORIGINAL_SOURCE else actual_digest(path))
    with pytest.raises(RuntimeError, match="frozen thread468 V1 source/test changed"):
        b.source_snapshot()
