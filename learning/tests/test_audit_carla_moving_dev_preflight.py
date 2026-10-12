"""HH_260906 - Exercise DEV preflight geometry and fail-closed reference boundaries without CARLA."""
import hashlib
import json

import pytest

from scripts.e2e import audit_carla_moving_dev_preflight as audit


def write(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = json.dumps(document).encode()
    path.write_bytes(raw)
    return hashlib.sha256(raw).hexdigest()


@pytest.fixture
def references(tmp_path):
    root = tmp_path / "reference"
    refs = []
    for name, split, town in (("train_episode", "train", "Town07"), ("dev_episode", "val", "Town03")):
        folder = root / "episodes" / name
        geometry_sha = write(folder / "route_geometry.json", dict(route_id=name, frame_id="map", polyline_m=[[0, 10], [200, 10]]))
        episode = dict(schema_id="autoware-e2e.common10.episode.v1", episode_id=name, domain="carla",
            split=split, map_id=town, site_id=town, route_id=name, split_group_id="group:"+name,
            source_session_id="session:"+name, route_geometry_file="route_geometry.json", route_geometry_sha256=geometry_sha)
        ref = f"episodes/{name}/episode.json"
        refs.append(dict(episode_id=name, manifest=ref, sha256=write(root/ref, episode)))
    write(root/"dataset.json", dict(schema_id="autoware-e2e.common10.dataset.v1", contract_id="common_10hz_v1", episodes=refs))
    return root


def alter_episode(root, name, change):
    path = root/"episodes"/name/"episode.json"
    doc = json.loads(path.read_text())
    doc.update(change)
    digest = write(path, doc)
    manifest = json.loads((root/"dataset.json").read_text())
    next(row for row in manifest["episodes"] if row["episode_id"] == name)["sha256"] = digest
    write(root/"dataset.json", manifest)


@pytest.mark.parametrize("a,b,c,d,expected", [
    ((0, 0), (10, 0), (5, -5), (5, 5), 0),
    ((0, 0), (10, 0), (10, 0), (15, 0), 0),
    ((0, 0), (10, 0), (2, 0), (8, 0), 0),
    ((0, 0), (10, 0), (0, 6), (10, 6), 6),
    ((0, 0), (0, 0), (3, 4), (3, 4), 5),
])
def test_segment_distance_handles_crossing_touch_collinear_and_degenerate(a, b, c, d, expected):
    assert audit.segment_distance(a, b, c, d) == pytest.approx(expected)


def test_crossing_requires_review_even_when_endpoints_are_far():
    result = audit.compare_routes([(0, 0), (20, 0)], [(10, -20), (10, 20)])
    assert result["spatial_review_needed"] is True
    assert result["minimum_polyline_distance_m"] == 0
    assert result["candidate_vertices_within_review_distance"] == 0
    assert audit.compare_routes([(0, 0), (20, 0)], [(0, 5), (20, 5)])["spatial_review_needed"] is True
    assert audit.compare_routes([(0, 0), (20, 0)], [(0, 5.01), (20, 5.01)])["spatial_review_needed"] is False


@pytest.mark.parametrize("points", [[], [[1, 2]], [[0, 0], [0, 0]], [[True, 0], [1, 2]],
                                    [[0, 0], [float("nan"), 1]], [[0, 0, 0], [1, 2, 3]]])
def test_invalid_polyline_refused(points):
    with pytest.raises(ValueError):
        audit.polyline(points)


def test_metadata_does_not_read_geometry_samples_or_images(references):
    ledger = {}
    episodes = audit.reference_metadata([references], ledger)
    assert len(episodes) == 2
    assert len(ledger) == 3
    assert all(path.endswith(("dataset.json", "episode.json")) for path in ledger)


@pytest.mark.parametrize("change", [dict(split="test"), dict(map_id="Town04"), dict(map_id="Town15"),
    dict(split="train"), dict(site_id="different"), dict(route_geometry_file="../route_geometry.json"),
    dict(source_session_id="session:train_episode"), dict(split_group_id="group:train_episode")])
def test_bad_reference_fails_before_geometry(references, change):
    alter_episode(references, "dev_episode", change)
    ledger = {}
    with pytest.raises(ValueError):
        audit.reference_metadata([references], ledger)
    assert all(not path.endswith("route_geometry.json") for path in ledger)


@pytest.mark.parametrize("eid", ["carla_town04_secret", "carla_town04-secret", "carla_town15.backup"])
def test_protected_reference_name_refused_before_episode_read(references, eid):
    manifest = json.loads((references/"dataset.json").read_text())
    manifest["episodes"] = [dict(episode_id=eid, manifest="never-open", sha256="0"*64)]
    write(references/"dataset.json", manifest)
    ledger = {}
    with pytest.raises(ValueError, match="protected episode"):
        audit.reference_metadata([references], ledger)
    assert len(ledger) == 1


def test_manifest_hash_and_duplicate_reference_rejected(references):
    path = references/"episodes/dev_episode/episode.json"
    path.write_text(path.read_text()+" ")
    with pytest.raises(ValueError, match="SHA mismatch"):
        audit.reference_metadata([references], {})


def test_duplicate_roots_and_symlink_reference_refused(references, tmp_path):
    with pytest.raises(ValueError, match="unique safe"):
        audit.reference_metadata([references, references], {})
    alias = tmp_path/"alias"
    alias.symlink_to(references, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        audit.reference_metadata([alias], {})


@pytest.fixture
def synthetic_candidate(tmp_path, monkeypatch):
    path = tmp_path/"candidate.json"
    digest = write(path, dict(route=[dict(x=0, y=0), dict(x=210, y=0)]))
    candidate = dict(route_id=audit.ROUTE_ID, route_sha256=digest, route_path=str(path),
                     scenario="straight", canonical_name="Town03", recomputed_planar_length_m=210.)
    monkeypatch.setattr(audit, "ROUTE_SHA", digest)
    monkeypatch.setattr(audit.cohort, "load_candidates", lambda *args: ([candidate], []))
    native = tmp_path/"native.json"
    native_sha = write(native, dict(town="Town03", route=[dict(x=0, y=10), dict(x=200, y=10)],
        coordinate_alignment=dict(source_frame="carla_map", target_frame="map", map_bundle_profile="town03",
                                  carla_to_map_transform=dict(x_m=0., y_m=0., z_m=0., yaw_rad=0.))))
    monkeypatch.setattr(audit, "FRAME_REFERENCE_PATH", str(native))
    monkeypatch.setattr(audit, "FRAME_REFERENCE_SHA", native_sha)
    return candidate


def test_report_has_five_clear_nonexecution_boundaries(references, synthetic_candidate, tmp_path):
    result = audit.build_report(tmp_path, [references])
    assert result["geometry"]["preliminary_checks_clear"] is True
    assert result["geometry"]["nominal_window_seconds"] == pytest.approx(32.4)
    assert result["geometry"]["nominal_remaining_route_m"] == pytest.approx(18.3333333333)
    assert result["proposed_case"]["split"] == "val" and result["proposed_case"]["traffic_seed"] == 201
    assert result["proposed_case"]["source_session_id"] is None
    assert result["reference_split_counts"] == {"train": 1, "val": 1}
    for field in ("execution_allowed", "dataset_admission_approved", "training_executed", "world_rpc_performed", "test_payloads_read"):
        assert result[field] is False
    assert result["input_sha256_before"] == result["input_sha256_after"]
    assert result["comparisons"][0]["geometry_payload_read"] is False
    assert result["comparisons"][1]["minimum_polyline_distance_m"] == 10
    assert len(result["remaining_before_capture"]) == 3
    assert len(result["remaining_after_capture"]) == 2


def test_tampered_geometry_fails_and_near_route_is_not_green(references, synthetic_candidate, tmp_path, monkeypatch):
    folder = references/"episodes/dev_episode"
    digest = write(folder/"route_geometry.json", dict(route_id="dev_episode", frame_id="map", polyline_m=[[0, 1], [200, 1]]))
    with pytest.raises(ValueError, match="SHA mismatch"):
        audit.build_report(tmp_path, [references])
    alter_episode(references, "dev_episode", dict(route_geometry_sha256=digest))
    with pytest.raises(ValueError, match="prepared geometry does not match"):
        audit.build_report(tmp_path, [references])
    native = json.loads((tmp_path/"native.json").read_text())
    native["route"] = [dict(x=0, y=1), dict(x=200, y=1)]
    monkeypatch.setattr(audit, "FRAME_REFERENCE_SHA", write(tmp_path/"native.json", native))
    result = audit.build_report(tmp_path, [references])
    assert result["geometry"]["preliminary_checks_clear"] is False
    assert "PRIOR_DEV_ROUTE_PROXIMITY_REVIEW_REQUIRED" in result["geometry"]["blockers"]


@pytest.mark.parametrize("change", [dict(x_m=1.), dict(yaw_rad=.1), dict(z_m=True)])
def test_frame_name_does_not_replace_identity_alignment_proof(references, synthetic_candidate, tmp_path, monkeypatch, change):
    native = json.loads((tmp_path/"native.json").read_text())
    native["coordinate_alignment"]["carla_to_map_transform"].update(change)
    monkeypatch.setattr(audit, "FRAME_REFERENCE_SHA", write(tmp_path/"native.json", native))
    with pytest.raises(ValueError):
        audit.build_report(tmp_path, [references])


def test_cli_does_not_overwrite_existing_output(references, tmp_path):
    output = tmp_path/"report.json"
    output.write_text("keep")
    with pytest.raises(ValueError, match="fresh output"):
        audit.main(["--catalog-maps-root", str(tmp_path/"catalog"), "--reference-dataset", str(references),
                    "--output-json", str(output)])
    assert output.read_text() == "keep"
