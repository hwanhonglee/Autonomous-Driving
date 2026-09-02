from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from scripts.e2e import organize_validation_campaign as organizer


def _artifact_path(repo_root: Path, index: int = 0) -> Path:
    return repo_root / organizer.ARTIFACT_CANDIDATES[index].relative_path


def _write_tree(path: Path, value: str = "evidence") -> None:
    path.mkdir(parents=True, exist_ok=True)
    (path / "evidence.txt").write_text(value, encoding="utf-8")


def _disable_runtime_activity(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        organizer,
        "inspect_project_processes",
        lambda: {"matches": [], "errors": []},
    )
    monkeypatch.setattr(
        organizer,
        "inspect_open_fds",
        lambda _sources: {"matches": [], "errors": []},
    )


def _write_stutter_fixture(repo_root: Path, campaign_root: Path) -> Path:
    source = (
        campaign_root
        / "20_30kph_control_ab/town07_straight/A_baseline_health_001"
    )
    attempts = []
    for number in range(1, 4):
        attempt_id = f"attempt_{number:03d}"
        attempt = source / "attempts" / attempt_id
        attempt.mkdir(parents=True)
        windows = []
        for index in range(3):
            windows.append(
                {
                    "clock": {"rtf": 0.24 + number * 0.001 + index * 0.002},
                    "minimum_observed_camera_wall_rate_hz": 1.25,
                    "cameras": {
                        organizer.CAM_FRONT_INFO_TOPIC: {"wall_rate_hz": 1.25}
                    },
                    "bundles": {
                        "receipt_span_seconds": {"p95": 0.64 + number * 0.001}
                    },
                }
            )
        health = {
            "status": "FAIL",
            "checked_at": f"2026-09-02T00:0{number}:00+00:00",
            "finished_at": f"2026-09-02T00:0{number}:45+00:00",
            "error": "runtime health gate timed out",
            "runtime": {"vehicle_engaged": False, "rosbag_started": False},
            "sequence": {
                "status": "FAIL",
                "timed_out": True,
                "required_consecutive_passes": 3,
                "maximum_consecutive_passes": 0,
                "elapsed_wall_seconds": 45.0,
            },
            "contract": {
                "thresholds": {
                    "minimum_rtf": 0.9,
                    "minimum_camera_wall_rate_hz": 4.0,
                    "maximum_bundle_receipt_p95_seconds": 0.04,
                }
            },
            "source": {
                "path": str(repo_root / "scripts/e2e/probe_runtime_health.py"),
                "sha256": "b" * 64,
            },
            "windows": windows,
        }
        runtime_health_path = attempt / "runtime_health.json"
        runtime_health_path.write_text(
            json.dumps(health), encoding="utf-8"
        )
        (attempt / "stack.log").write_text(
            "\n".join(
                (
                    "AutowareState: Planning => WaitingForEngage",
                    "[WARN] runtime_timing stage=camera_image_publish camera=CAM_FRONT "
                    f"duration_ms={600 + number:.3f} monotonic_ns={1000 + number} "
                    f"source_stamp_sec={2.0 + number:.1f}",
                    "[WARN] runtime_timing stage=camera_image_publish camera=CAM_FRONT "
                    f"duration_ms={620 + number:.3f} monotonic_ns={2000 + number} "
                    f"source_stamp_sec={3.0 + number:.1f} suppressed=6",
                )
            )
            + "\n",
            encoding="utf-8",
        )
        (attempt / "runtime.env").write_text(
            "\n".join(
                (
                    "CAMERA_SOURCE_5HZ=true",
                    "CAMERA_SOURCE_SENSOR_TICK_SEC=0.2",
                    "CAMERA_ROS_PUBLISH_HZ=5.0",
                    "RUNTIME_HEALTH_GATE_PHASE="
                    "after_optional_rviz_recorder_before_rosbag_and_engagement",
                    "RUNTIME_HEALTH_GATE_STATUS=FAIL",
                    "RUNTIME_HEALTH_GATE_EXIT_CODE=1",
                    "RUNTIME_HEALTH_EVIDENCE_SHA256="
                    f"{organizer.sha256_file(runtime_health_path)}",
                    "RUNTIME_HEALTH_EVALUATED_WINDOWS=3",
                    "RUNTIME_HEALTH_MAXIMUM_CONSECUTIVE_PASSES=0",
                    "RUNTIME_HEALTH_PROBE_FILE="
                    f"{repo_root / 'scripts/e2e/probe_runtime_health.py'}",
                    f"RUNTIME_HEALTH_PROBE_SHA256={'b' * 64}",
                )
            )
            + "\n",
            encoding="utf-8",
        )
        attempts.append(
            {
                "attempt_id": attempt_id,
                "path": str(attempt),
                "process_exit_status": 1,
                "runtime_health_status": "FAIL",
            }
        )
    (source / "owned_trial_summary.json").write_text(
        json.dumps(
            {
                "status": "FAIL",
                "selected_attempt": None,
                "attempts": attempts,
                "map": "Town07",
                "route": str(repo_root / "route.json"),
                "route_sha256": "a" * 64,
            }
        ),
        encoding="utf-8",
    )
    (source / "owned_trial_console.log").write_text(
        "Runtime health remained below threshold after 3 generations\n",
        encoding="utf-8",
    )
    return source


def test_initialize_campaign_creates_only_index_and_categories(
    tmp_path: Path,
) -> None:
    repo_root = tmp_path / "repo"
    source = (
        repo_root
        / "artifacts/validation/2026-09-02/"
        "autoware_vad_30kph_town06_long_straight_comparison_v1/result.json"
    )
    source.parent.mkdir(parents=True)
    source.write_text('{"preserve": true}\n', encoding="utf-8")
    before = organizer.sha256_file(source)
    campaign_root = (
        repo_root
        / "artifacts/validation/2026-09-02/autoware_vad_runtime_control_campaign_v1"
    )

    index = organizer.initialize_campaign(
        repo_root,
        campaign_root,
        generated_at="2026-09-02T00:00:00+00:00",
    )

    assert organizer.sha256_file(source) == before
    assert index["organization_mode"] == (
        "canonical_campaign_plus_absolute_legacy_pointers"
    )
    assert index["source_mutation"] == "NONE"
    assert index["source_contract"] == {
        "copies_created": False,
        "sources_moved": False,
        "sources_deleted": False,
        "symlinks_created": False,
        "note": (
            "This index does not mutate evidence. New campaign trials live directly "
            "under the campaign root; legacy and all-Town evidence remains at absolute "
            "paths because strict validators bind real paths and bag locations."
        ),
    }
    files = sorted(
        path.relative_to(campaign_root).as_posix()
        for path in campaign_root.rglob("*")
        if path.is_file()
    )
    assert files == ["00_index/INDEX.json", "00_index/README.md"]
    assert not any(path.is_symlink() for path in campaign_root.rglob("*"))
    for relative in organizer.CATEGORY_DIRECTORIES:
        assert (campaign_root / relative).is_dir()
    stored = json.loads((campaign_root / "00_index/INDEX.json").read_text())
    assert stored == index
    assert all(
        Path(item["path"]).is_absolute()
        for item in stored["pointers"]["30_60kph"]
    )
    assert "15_all_towns_30kph" in stored["pointers"]
    assert len(stored["pointers"]["20_30kph_control_ab"]["town07_straight"]) == 4
    assert any(
        item["role"] == "primary applied-cleanup and restore journal"
        for item in stored["pointers"]["99_integrity"]
    )
    assert any(
        item["role"] == "recoverable final tmp-audit quarantine"
        for item in stored["pointers"]["90_quarantine"]
    )


def test_default_main_is_pointer_only_index(tmp_path: Path, monkeypatch, capsys) -> None:
    repo_root = tmp_path / "repo"
    campaign_root = repo_root / "campaign"
    monkeypatch.setattr(organizer, "REPO_ROOT", repo_root)
    monkeypatch.setattr(organizer, "DEFAULT_CAMPAIGN_ROOT", campaign_root)

    assert organizer.main([]) == 0

    output = json.loads(capsys.readouterr().out)
    assert output["status"] == "POINTER_INDEX_READY"
    assert output["source_mutation"] == "NONE"
    assert sorted(
        path.relative_to(campaign_root).as_posix()
        for path in campaign_root.rglob("*")
        if path.is_file()
    ) == ["00_index/INDEX.json", "00_index/README.md"]


def test_plan_cleanup_hashes_exact_candidates_without_moving(tmp_path: Path) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    artifact = _artifact_path(repo_root)
    _write_tree(artifact, "artifact payload")
    exact_tmp = tmp_root / "autoware_longitudinal_30_final.agpq6U"
    _write_tree(exact_tmp, "analysis payload")
    audited_map_scratch = tmp_root / "c_track_autoware_system.osm"
    audited_map_scratch.write_text("temporary map copy", encoding="utf-8")
    superseded_launch_dump = (
        tmp_root
        / "town06_straight_60kph_pilot_best_effort_image_v2_trial_stack_launch.txt"
    )
    superseded_launch_dump.write_text("old launch dump", encoding="utf-8")
    launch_parameter = tmp_root / "launch_params_abc_123"
    launch_parameter.write_text("parameter", encoding="utf-8")
    excluded = tmp_root / "tmp.generic-session"
    excluded.write_text("must stay", encoding="utf-8")
    output = repo_root / "cleanup-plan.json"

    plan = organizer.write_cleanup_plan(
        repo_root,
        tmp_root,
        output,
        generated_at="2026-09-02T00:00:00+00:00",
    )

    organizer.verify_signed_payload(plan, "test plan")
    assert plan["applied"] is False
    assert artifact.is_dir()
    assert exact_tmp.is_dir()
    assert launch_parameter.is_file()
    assert excluded.is_file()
    assert all(entry["source_path"] != str(excluded) for entry in plan["entries"])
    by_source = {entry["source_path"]: entry for entry in plan["entries"]}
    for source in (
        artifact,
        exact_tmp,
        audited_map_scratch,
        superseded_launch_dump,
        launch_parameter,
    ):
        entry = by_source[str(source)]
        assert entry["status"] == "READY_TO_QUARANTINE"
        assert entry["size_bytes"] > 0
        assert len(entry["tree_sha256"]) == 64
    assert by_source[str(artifact)]["observed_status"] == "INCOMPLETE"
    assert plan["policy"]["ordinary_tmp_glob_excluded"] == "/tmp/tmp.*"
    assert json.loads(output.read_text())["manifest_sha256"] == plan["manifest_sha256"]


def test_cleanup_policy_includes_only_superseded_campaign_60kph_pilots(
    tmp_path: Path,
) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    campaign = (
        repo_root
        / "artifacts/validation/2026-09-02/"
        "autoware_vad_runtime_control_campaign_v1"
    )
    superseded = [
        campaign
        / "30_60kph/town06_straight_60kph_pilot_best_effort_image_v1",
        campaign
        / "30_60kph/town06_straight_60kph_pilot_best_effort_image_v2",
    ]
    protected = [
        campaign
        / "30_60kph/"
        "town06_straight_60kph_pilot_best_effort_image_depth1_v3",
        campaign
        / "20_30kph_control_ab/town07_straight/A_baseline_health_001",
        repo_root
        / "artifacts/validation/2026-09-02/"
        "autoware_vad_town_matrix_30kph_camera_source_5hz_v1",
    ]
    for path in [*superseded, *protected]:
        _write_tree(path, path.name)

    plan = organizer.build_cleanup_plan(repo_root, tmp_root)
    ready_sources = {
        Path(entry["source_path"])
        for entry in plan["entries"]
        if entry["status"] == "READY_TO_QUARANTINE"
    }

    assert set(superseded).issubset(ready_sources)
    assert set(protected).isdisjoint(ready_sources)


def test_superseded_campaign_pilot_can_be_quarantined_and_restored(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    source = (
        repo_root
        / "artifacts/validation/2026-09-02/"
        "autoware_vad_runtime_control_campaign_v1/30_60kph/"
        "town06_straight_60kph_pilot_best_effort_image_v1"
    )
    _write_tree(source, "superseded pilot")
    expected = organizer.inspect_path(source)
    manifest = repo_root / "cleanup-plan.json"
    organizer.write_cleanup_plan(repo_root, tmp_root, manifest)
    quarantine = (
        repo_root
        / "artifacts/validation/2026-09-02/"
        "autoware_vad_runtime_control_campaign_v1/90_quarantine"
    )
    _disable_runtime_activity(monkeypatch)

    journal = organizer.apply_cleanup(
        manifest,
        quarantine,
        expected_repo_root=repo_root,
        expected_tmp_root=tmp_root,
        confirmed=True,
    )

    moved = next(
        entry for entry in journal["entries"] if entry["source_path"] == str(source)
    )
    assert not source.exists()
    assert Path(moved["quarantine_path"]).is_dir()
    journal_path = next(quarantine.glob("cleanup-journal-*.json"))
    organizer.restore_cleanup(
        journal_path,
        expected_repo_root=repo_root,
        expected_tmp_root=tmp_root,
        confirmed=True,
    )
    assert organizer.inspect_path(source)["tree_sha256"] == expected["tree_sha256"]


def test_candidate_tree_hash_binds_internal_symlink_without_dereferencing(
    tmp_path: Path,
) -> None:
    candidate = tmp_path / "candidate"
    candidate.mkdir()
    target = candidate / "payload.txt"
    target.write_text("payload", encoding="utf-8")
    link = candidate / "latest"
    link.symlink_to("payload.txt")

    first = organizer.inspect_path(candidate)
    link.unlink()
    link.symlink_to("missing.txt")
    second = organizer.inspect_path(candidate)

    assert first["symlink_count"] == 1
    assert first["file_count"] == 1
    assert first["size_bytes"] == len("payload")
    assert first["tree_sha256"] != second["tree_sha256"]


def test_apply_cleanup_requires_explicit_confirmation(tmp_path: Path) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    source = _artifact_path(repo_root)
    _write_tree(source)
    manifest = repo_root / "plan.json"
    organizer.write_cleanup_plan(repo_root, tmp_root, manifest)

    with pytest.raises(organizer.CampaignError, match="disabled without --confirm"):
        organizer.apply_cleanup(
            manifest,
            repo_root / "quarantine",
            expected_repo_root=repo_root,
            expected_tmp_root=tmp_root,
        )
    assert source.exists()


def test_apply_cleanup_rejects_hash_change_before_any_rename(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    first = _artifact_path(repo_root, 0)
    second = _artifact_path(repo_root, 1)
    _write_tree(first, "first")
    _write_tree(second, "second")
    manifest = repo_root / "plan.json"
    organizer.write_cleanup_plan(repo_root, tmp_root, manifest)
    (second / "evidence.txt").write_text("changed", encoding="utf-8")
    _disable_runtime_activity(monkeypatch)

    with pytest.raises(organizer.CampaignError, match="hash/size changed"):
        organizer.apply_cleanup(
            manifest,
            repo_root / "quarantine",
            expected_repo_root=repo_root,
            expected_tmp_root=tmp_root,
            confirmed=True,
        )

    assert first.exists()
    assert second.exists()
    assert not (repo_root / "quarantine").exists()


def test_apply_cleanup_rejects_active_project_process(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    source = _artifact_path(repo_root)
    _write_tree(source)
    manifest = repo_root / "plan.json"
    organizer.write_cleanup_plan(repo_root, tmp_root, manifest)
    monkeypatch.setattr(
        organizer,
        "inspect_project_processes",
        lambda: {
            "matches": [{"pid": 42, "command": "CarlaUE4-Linux-Shipping"}],
            "errors": [],
        },
    )

    with pytest.raises(organizer.CampaignError, match="active project process"):
        organizer.apply_cleanup(
            manifest,
            repo_root / "quarantine",
            expected_repo_root=repo_root,
            expected_tmp_root=tmp_root,
            confirmed=True,
        )
    assert source.exists()


def test_apply_and_restore_use_hash_bound_rename_journal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    source = _artifact_path(repo_root)
    _write_tree(source, "recoverable")
    manifest = repo_root / "plan.json"
    plan = organizer.write_cleanup_plan(repo_root, tmp_root, manifest)
    quarantine = repo_root / "quarantine"
    _disable_runtime_activity(monkeypatch)

    journal = organizer.apply_cleanup(
        manifest,
        quarantine,
        expected_repo_root=repo_root,
        expected_tmp_root=tmp_root,
        confirmed=True,
    )

    assert journal["status"] == "APPLIED"
    assert journal["source_manifest_sha256"] == plan["manifest_sha256"]
    assert journal["delete_performed"] is False
    assert journal["copy_performed"] is False
    assert not source.exists()
    moved_entry = next(
        entry for entry in journal["entries"] if entry["source_path"] == str(source)
    )
    quarantined = Path(moved_entry["quarantine_path"])
    assert quarantined.exists()
    assert "restore-cleanup" in journal["restore_command"]
    journal_path = next(quarantine.glob("cleanup-journal-*.json"))

    restored = organizer.restore_cleanup(
        journal_path,
        expected_repo_root=repo_root,
        expected_tmp_root=tmp_root,
        confirmed=True,
    )

    assert restored["status"] == "RESTORED"
    assert source.exists()
    assert not quarantined.exists()
    assert organizer.inspect_path(source)["tree_sha256"] == moved_entry["tree_sha256"]


def test_partial_apply_stops_and_journal_restore_recovers_prior_moves(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    first = _artifact_path(repo_root, 0)
    second = _artifact_path(repo_root, 1)
    _write_tree(first, "first")
    _write_tree(second, "second")
    manifest = repo_root / "plan.json"
    organizer.write_cleanup_plan(repo_root, tmp_root, manifest)
    quarantine = repo_root / "quarantine"
    _disable_runtime_activity(monkeypatch)
    original_rename = os.rename
    original_write = organizer.atomic_write_json
    calls = 0
    journal_write_statuses = []

    def fail_second_source_move(source, destination):
        nonlocal calls
        if Path(source) in {first, second}:
            calls += 1
            if calls == 2:
                raise OSError("synthetic rename failure")
        return original_rename(source, destination)

    def capture_journal_write(path, value):
        if Path(path).name.startswith("cleanup-journal-"):
            journal_write_statuses.append(value.get("status"))
        return original_write(path, value)

    monkeypatch.setattr(organizer.os, "rename", fail_second_source_move)
    monkeypatch.setattr(organizer, "atomic_write_json", capture_journal_write)
    with pytest.raises(organizer.CampaignError, match="partial failure; use"):
        organizer.apply_cleanup(
            manifest,
            quarantine,
            expected_repo_root=repo_root,
            expected_tmp_root=tmp_root,
            confirmed=True,
        )

    assert not first.exists()
    assert second.exists()
    journal_path = next(quarantine.glob("cleanup-journal-*.json"))
    failed_journal = organizer.load_signed_json(journal_path, "test journal")
    assert failed_journal["status"] == "PARTIAL_FAILED"
    assert [entry["state"] for entry in failed_journal["entries"]].count("MOVED") == 1
    assert journal_write_statuses == ["APPLYING", "PARTIAL_FAILED"]

    restored = organizer.restore_cleanup(
        journal_path,
        expected_repo_root=repo_root,
        expected_tmp_root=tmp_root,
        confirmed=True,
    )
    assert restored["status"] == "RESTORED"
    assert first.exists()
    assert second.exists()


def test_large_apply_writes_only_begin_and_terminal_journal_snapshots(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    entry_count = 512
    for index in range(entry_count):
        (tmp_root / f"launch_params_bulk_{index:05d}").write_text(
            f"payload {index}\n", encoding="utf-8"
        )
    manifest = repo_root / "plan.json"
    organizer.write_cleanup_plan(repo_root, tmp_root, manifest)
    quarantine = repo_root / "quarantine"
    _disable_runtime_activity(monkeypatch)
    original_write = organizer.atomic_write_json
    journal_writes = []

    def count_journal_write(path, value):
        if Path(path).name.startswith("cleanup-journal-"):
            journal_writes.append(
                (value.get("status"), len(value.get("entries", [])))
            )
        return original_write(path, value)

    monkeypatch.setattr(organizer, "atomic_write_json", count_journal_write)

    journal = organizer.apply_cleanup(
        manifest,
        quarantine,
        expected_repo_root=repo_root,
        expected_tmp_root=tmp_root,
        confirmed=True,
    )

    assert journal["status"] == "APPLIED"
    assert journal_writes == [
        ("APPLYING", entry_count),
        ("APPLIED", entry_count),
    ]
    assert journal["journal_write_strategy"] == (
        "BEGIN_AND_TERMINAL_FULL_SNAPSHOTS_ONLY"
    )


def test_restore_reconciles_hard_interrupted_applying_journal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    first = _artifact_path(repo_root, 0)
    second = _artifact_path(repo_root, 1)
    _write_tree(first, "first")
    _write_tree(second, "second")
    manifest = repo_root / "plan.json"
    organizer.write_cleanup_plan(repo_root, tmp_root, manifest)
    quarantine = repo_root / "quarantine"
    _disable_runtime_activity(monkeypatch)
    original_rename = os.rename
    source_rename_count = 0

    class HardInterruption(BaseException):
        pass

    def stop_before_second_rename(source, destination):
        nonlocal source_rename_count
        if Path(source) in {first, second}:
            source_rename_count += 1
            if source_rename_count == 2:
                raise HardInterruption("synthetic hard interruption")
        return original_rename(source, destination)

    monkeypatch.setattr(organizer.os, "rename", stop_before_second_rename)
    with pytest.raises(HardInterruption):
        organizer.apply_cleanup(
            manifest,
            quarantine,
            expected_repo_root=repo_root,
            expected_tmp_root=tmp_root,
            confirmed=True,
        )

    journal_path = next(quarantine.glob("cleanup-journal-*.json"))
    applying = organizer.load_signed_json(journal_path, "test APPLYING journal")
    assert applying["status"] == "APPLYING"
    assert {entry["state"] for entry in applying["entries"]} == {"PENDING"}
    assert sum(path.exists() for path in (first, second)) == 1
    monkeypatch.setattr(organizer.os, "rename", original_rename)

    restored = organizer.restore_cleanup(
        journal_path,
        expected_repo_root=repo_root,
        expected_tmp_root=tmp_root,
        confirmed=True,
    )

    assert restored["status"] == "RESTORED"
    assert restored["restore_reconciliation"] == {
        "entry_count": 2,
        "moved_at_quarantine_count": 1,
        "already_at_source_count": 1,
        "basis": "exact source/quarantine locations plus manifest-bound content hash",
    }
    assert first.exists()
    assert second.exists()


@pytest.mark.parametrize("location_failure", ["both", "neither"])
def test_applying_restore_rejects_both_or_neither_exact_location(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    location_failure: str,
) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    source = _artifact_path(repo_root)
    _write_tree(source, "payload")
    manifest = repo_root / "plan.json"
    organizer.write_cleanup_plan(repo_root, tmp_root, manifest)
    quarantine = repo_root / "quarantine"
    _disable_runtime_activity(monkeypatch)
    original_rename = os.rename

    class HardInterruption(BaseException):
        pass

    def stop_after_first_rename(original, destination):
        original_rename(original, destination)
        if Path(original) == source:
            raise HardInterruption("synthetic hard interruption after rename")

    monkeypatch.setattr(organizer.os, "rename", stop_after_first_rename)
    with pytest.raises(HardInterruption):
        organizer.apply_cleanup(
            manifest,
            quarantine,
            expected_repo_root=repo_root,
            expected_tmp_root=tmp_root,
            confirmed=True,
        )
    journal_path = next(quarantine.glob("cleanup-journal-*.json"))
    applying = organizer.load_signed_json(journal_path, "test APPLYING journal")
    quarantined = Path(applying["entries"][0]["quarantine_path"])
    assert applying["status"] == "APPLYING"
    assert quarantined.exists()
    if location_failure == "both":
        _write_tree(source, "payload")
        expected_error = "both source and quarantine locations"
    else:
        holding = tmp_path / "outside-exact-locations"
        original_rename(quarantined, holding)
        expected_error = "neither source nor quarantine location"
    monkeypatch.setattr(organizer.os, "rename", original_rename)

    with pytest.raises(organizer.CampaignError, match=expected_error):
        organizer.restore_cleanup(
            journal_path,
            expected_repo_root=repo_root,
            expected_tmp_root=tmp_root,
            confirmed=True,
        )


def test_restore_rejects_tampered_quarantine_content(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    source = _artifact_path(repo_root)
    _write_tree(source, "original")
    manifest = repo_root / "plan.json"
    organizer.write_cleanup_plan(repo_root, tmp_root, manifest)
    quarantine = repo_root / "quarantine"
    _disable_runtime_activity(monkeypatch)

    journal = organizer.apply_cleanup(
        manifest,
        quarantine,
        expected_repo_root=repo_root,
        expected_tmp_root=tmp_root,
        confirmed=True,
    )
    journal_path = next(quarantine.glob("cleanup-journal-*.json"))
    quarantined = Path(journal["entries"][0]["quarantine_path"])
    (quarantined / "evidence.txt").write_text("tampered", encoding="utf-8")

    with pytest.raises(organizer.CampaignError, match="quarantined content hash changed"):
        organizer.restore_cleanup(
            journal_path,
            expected_repo_root=repo_root,
            expected_tmp_root=tmp_root,
            confirmed=True,
        )


def test_manifest_integrity_and_exact_policy_are_fail_closed(tmp_path: Path) -> None:
    repo_root = tmp_path / "repo"
    tmp_root = tmp_path / "tmp"
    tmp_root.mkdir()
    source = _artifact_path(repo_root)
    _write_tree(source)
    manifest_path = repo_root / "plan.json"
    plan = organizer.write_cleanup_plan(repo_root, tmp_root, manifest_path)
    ready = next(entry for entry in plan["entries"] if entry["status"] == "READY_TO_QUARANTINE")
    ready["source_path"] = str(repo_root / "unrelated-user-data")
    manifest_path.write_text(json.dumps(plan), encoding="utf-8")

    with pytest.raises(organizer.CampaignError, match="integrity mismatch"):
        organizer.apply_cleanup(
            manifest_path,
            repo_root / "quarantine",
            expected_repo_root=repo_root,
            expected_tmp_root=tmp_root,
            confirmed=True,
        )
    assert source.exists()


def test_open_fd_inspector_reports_an_open_candidate(tmp_path: Path) -> None:
    candidate = tmp_path / "candidate.txt"
    candidate.write_text("open", encoding="utf-8")
    with candidate.open("r", encoding="utf-8"):
        report = organizer.inspect_open_fds([candidate])
    assert any(
        item["pid"] == os.getpid() and item["target"] == str(candidate)
        for item in report["matches"]
    )


def test_open_fd_inspector_skips_only_verified_sd_pam_session_helper(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    proc_root = tmp_path / "proc"
    uid = os.getuid()
    cgroup = f"0::/user.slice/user-{uid}.slice/user@{uid}.service/init.scope\n"
    parent = proc_root / "41000"
    child = proc_root / "41001"
    for process_dir in (parent, child):
        (process_dir / "fd").mkdir(parents=True)

    (parent / "status").write_text(
        "Name:\tsystemd\n"
        "Tgid:\t41000\n"
        "Pid:\t41000\n"
        "PPid:\t1\n"
        f"Uid:\t{uid}\t{uid}\t{uid}\t{uid}\n",
        encoding="utf-8",
    )
    (parent / "comm").write_bytes(b"systemd\n")
    (parent / "cmdline").write_bytes(b"/usr/lib/systemd/systemd\0--user\0")
    (parent / "cgroup").write_text(cgroup, encoding="utf-8")
    trusted_systemd = tmp_path / "trusted-systemd"
    trusted_systemd.write_bytes(b"test fixture")
    (parent / "exe").symlink_to(trusted_systemd)

    (child / "status").write_text(
        "Name:\t(sd-pam)\n"
        "Tgid:\t41001\n"
        "Pid:\t41001\n"
        "PPid:\t41000\n"
        f"Uid:\t{uid}\t{uid}\t{uid}\t{uid}\n",
        encoding="utf-8",
    )
    (child / "comm").write_bytes(b"(sd-pam)\n")
    (child / "cmdline").write_bytes(b"(sd-pam)\0")
    (child / "cgroup").write_text(cgroup, encoding="utf-8")

    original_list = organizer._list_process_descriptors

    def deny_child_descriptors(process_dir: Path) -> list[Path]:
        if process_dir == child:
            raise PermissionError(13, "Permission denied", str(child / "fd"))
        return original_list(process_dir)

    monkeypatch.setattr(organizer, "_list_process_descriptors", deny_child_descriptors)
    monkeypatch.setattr(
        organizer,
        "_trusted_systemd_executables",
        lambda: {trusted_systemd.resolve()},
    )

    report = organizer.inspect_open_fds([tmp_path / "candidate"], proc_root=proc_root)

    assert report == {
        "matches": [],
        "errors": [],
        "skipped": [
            {
                "pid": 41001,
                "reason": "verified systemd --user (sd-pam) session helper",
            }
        ],
    }


def test_open_fd_inspector_rejects_unknown_inaccessible_same_uid_process(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    proc_root = tmp_path / "proc"
    process_dir = proc_root / "42000"
    (process_dir / "fd").mkdir(parents=True)
    candidate = tmp_path / "candidate"
    candidate.write_text("evidence", encoding="utf-8")

    def deny_descriptors(_process_dir: Path) -> list[Path]:
        raise PermissionError(13, "Permission denied", str(process_dir / "fd"))

    monkeypatch.setattr(organizer, "_list_process_descriptors", deny_descriptors)
    report = organizer.inspect_open_fds([candidate], proc_root=proc_root)

    assert report["matches"] == []
    assert report["skipped"] == []
    assert report["errors"] == [
        f"pid 42000: [Errno 13] Permission denied: '{process_dir / 'fd'}'"
    ]
    monkeypatch.setattr(
        organizer,
        "inspect_project_processes",
        lambda: {"matches": [], "errors": []},
    )
    monkeypatch.setattr(organizer, "inspect_open_fds", lambda _sources: report)
    with pytest.raises(organizer.CampaignError, match="cannot prove open-FD safety"):
        organizer._assert_runtime_safety([candidate])


def test_parse_front_image_publish_timing_preserves_suppression_metadata() -> None:
    samples = organizer.parse_front_image_publish_timing(
        "noise\n"
        "runtime_timing stage=camera_image_publish camera=CAM_FRONT "
        "duration_ms=623.861 monotonic_ns=437254836672621 "
        "source_stamp_sec=4.300000064074993 suppressed=6\n"
    )
    assert samples == [
        {
            "line_number": 2,
            "duration_ms": 623.861,
            "monotonic_ns": 437254836672621,
            "source_stamp_sec": 4.300000064074993,
            "suppressed_count_reported": 6,
        }
    ]


def test_write_stutter_evidence_is_small_hash_bound_and_pre_engagement(
    tmp_path: Path,
) -> None:
    repo_root = tmp_path / "repo"
    campaign_root = repo_root / "artifacts/validation/campaign"
    source = _write_stutter_fixture(repo_root, campaign_root)
    output = campaign_root / "10_runtime_stutter/town07_baseline"
    before = organizer.inspect_path(source)

    pointers, summary = organizer.write_stutter_evidence(
        repo_root,
        campaign_root,
        source,
        output,
        generated_at="2026-09-02T01:00:00+00:00",
    )

    assert organizer.inspect_path(source) == before
    assert sorted(path.name for path in output.iterdir()) == [
        "README.md",
        "runtime_stutter_summary.json",
        "source_pointers.json",
    ]
    organizer.verify_signed_payload(pointers, "pointer manifest")
    organizer.verify_signed_payload(summary, "stutter summary")
    assert summary["status"] == "PRE_ENGAGEMENT_RUNTIME_HEALTH_FAILED_ALL_ATTEMPTS"
    assert summary["vehicle_engaged_any_attempt"] is False
    assert summary["engagement_blocked_before_motion"] is True
    assert len(summary["attempts"]) == 3
    for attempt in summary["attempts"]:
        assert attempt["engagement"]["vehicle_engaged"] is False
        assert attempt["engagement"]["engaged_transition_stack_lines"] == []
        assert attempt["all_windows_below_rtf_threshold"] is True
        assert attempt["all_windows_below_camera_rate_threshold"] is True
        assert attempt["all_windows_above_bundle_receipt_p95_threshold"] is True
        timing = attempt["observed"]["runtime_timing_cam_front_image_publish"]
        assert timing["explicit_log_sample_count"] == 2
        assert timing["reported_suppressed_count_sum"] == 6
    authoritative = pointers["authoritative_files"]
    assert len(authoritative) == 11
    assert all(Path(item["absolute_path"]).is_absolute() for item in authoritative)
    assert all(item["repository_relative_path"] for item in authoritative)
    assert all(len(item["sha256"]) == 64 for item in authoritative)
    readme = (output / "README.md").read_text(encoding="utf-8")
    assert "failed the runtime-health gate before engage" in readme
    assert "no cleanup, move, delete, copy, or symlink" in readme


def test_stutter_summary_rejects_any_engaged_attempt(tmp_path: Path) -> None:
    repo_root = tmp_path / "repo"
    campaign_root = repo_root / "artifacts/validation/campaign"
    source = _write_stutter_fixture(repo_root, campaign_root)
    health_path = source / "attempts/attempt_002/runtime_health.json"
    health = json.loads(health_path.read_text())
    health["runtime"]["vehicle_engaged"] = True
    health_path.write_text(json.dumps(health), encoding="utf-8")

    with pytest.raises(organizer.CampaignError, match="vehicle_engaged=false"):
        organizer.build_stutter_evidence(
            repo_root,
            campaign_root,
            source,
            campaign_root / "10_runtime_stutter/output",
        )
