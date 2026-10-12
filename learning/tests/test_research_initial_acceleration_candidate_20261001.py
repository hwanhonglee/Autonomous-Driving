"""HH_261001 - Keep candidate identities explicit and restore the original AH contract."""

from types import SimpleNamespace
import pytest

from scripts.e2e import research_initial_acceleration_candidate_20261001 as candidate


@pytest.mark.parametrize("sha", [None, "", "a" * 63, "G" * 64, candidate.PARENT_SHA256])
def test_reject_bad_or_parent_identity(sha):
    with pytest.raises(ValueError):
        candidate.validate_identity(sha)


def test_identity_restored_even_after_failure():
    original = candidate.research.AH_CHECKPOINT_SHA256, candidate.research.CHECKPOINT_SCHEMA
    collector = SimpleNamespace(AH_CHECKPOINT_SHA256=candidate.PARENT_SHA256)
    with pytest.raises(RuntimeError):
        with candidate.candidate_identity("a" * 64, collector):
            assert candidate.research.AH_CHECKPOINT_SHA256 == collector.AH_CHECKPOINT_SHA256 == "a" * 64
            assert candidate.research.CHECKPOINT_SCHEMA == candidate.CHECKPOINT_SCHEMA
            raise RuntimeError("deliberate")
    assert (candidate.research.AH_CHECKPOINT_SHA256, candidate.research.CHECKPOINT_SCHEMA) == original
    assert collector.AH_CHECKPOINT_SHA256 == candidate.PARENT_SHA256


def test_reject_incomplete_training_or_wrong_parent():
    payload = {"checkpoint_schema": candidate.CHECKPOINT_SCHEMA, "arm": candidate.ARMS[0],
               "parent_checkpoint_sha256": candidate.PARENT_SHA256, "fit": {"global_step": 3000}}
    candidate.validate_checkpoint(payload, candidate.ARMS[0])
    for changed in ({**payload, "fit": {"global_step": 2999}},
                    {**payload, "parent_checkpoint_sha256": "b" * 64},
                    {**payload, "checkpoint_schema": "unrelated"},
                    {**payload, "arm": candidate.ARMS[1]}):
        with pytest.raises(ValueError):
            candidate.validate_checkpoint(changed, candidate.ARMS[0])


def test_worker_preserves_arguments_and_actual_identity(monkeypatch):
    from scripts.e2e import collect_ah_simulation_research as collector
    original = collector.AH_CHECKPOINT_SHA256
    received = []
    def fake_main(argv):
        received.extend(argv)
        assert collector.AH_CHECKPOINT_SHA256 == candidate.research.AH_CHECKPOINT_SHA256 == "c" * 64
        return 0
    monkeypatch.setattr(collector, "main", fake_main)
    assert candidate.main(["--mode", "worker", "--arm", candidate.ARMS[1],
        "--checkpoint-sha256", "c" * 64, "--weights", "candidate.npz",
        "--research-weights", "candidate.npz", "episode", "route.json"]) == 0
    assert received == ["--research-weights", "candidate.npz", "episode", "route.json"]
    assert collector.AH_CHECKPOINT_SHA256 == original
