"""HH_261005 - Exercise only synthetic bounded metadata, locked reservations and fail-closed authorization."""

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


SOURCE = Path(__file__).resolve().parents[2] / "tools/e2e/qualify_rte01_test_metadata_v1_20261005.py"
spec = importlib.util.spec_from_file_location("HH_rte01_metadata_only", SOURCE)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def encoded(document):
    return json.dumps(document, sort_keys=True, allow_nan=False).encode()


def sha(data):
    return hashlib.sha256(data).hexdigest()


def fixture_documents():
    # HH_261005 - These are synthetic lineage fields, not actual TEST data or an approval evidence substitution.
    train = dict(manifest_sha256="a" * 64, temporal_fingerprint_sha256="b" * 64)
    docs = {
        m.GUARD: dict(sites={site: dict(map=[], drive=[]) for site in m.SITES}),
        m.RAW: dict(initialization="fresh random weights, not adapted W1", steps=200, train=train),
        m.BALANCED: dict(parent_checkpoint_sha256="6c8bfceee5b3a2a0e332666b225769af236499baac9414294c18ccc6e9833ece",
            legacy_strata=dict(curved_by_map={"C_track_1_0_7": 260, "Town01": 68},
                straight_by_map={"C_track_1_0_7": 662, "Town01": 570, "Town07": 927}, eligible_count=2487),
            cohorts=dict(legacy=train)),
        m.BALANCED_META: dict(npz_sha256="c" * 64),
        m.RECOVERY_META: dict(npz_sha256="d" * 64),
    }
    bsha, rsha = sha(encoded(docs[m.BALANCED_META])), sha(encoded(docs[m.RECOVERY_META]))
    docs[m.RECOVERY] = dict(parent_npz_sha256="c" * 64, parent_metadata_sha256=bsha)
    recovery_sha = sha(encoded(docs[m.RECOVERY]))
    docs[m.JOINT] = dict(parent_npz_sha256="d" * 64, parent_metadata_sha256=rsha, parent_declaration_sha256=recovery_sha)
    docs[m.GOOD_META] = dict(npz_sha256=m.GOOD_SHA, parent_npz_sha256="d" * 64, parent_metadata_sha256=rsha)
    for site in m.SITES:
        docs[m.STATUS[site]] = dict(canonical_name=site,
            status="SOURCE_EDITOR_REQUIRED" if site == "Town15" else "SKIP_RESUME_VALIDATED",
            manifest_split="test" if site == "Town15" else "train",
            static_inventory=dict(server_profile="source_editor_0915_4ws" if site == "Town15" else "packaged_0915"))
    for name in m.METADATA:
        if name.endswith(".json") and name not in docs:
            docs[name] = dict(schema="synthetic_index_only", episodes=[])
    return docs


@pytest.fixture
def synthetic(tmp_path, monkeypatch):
    monkeypatch.setattr(m, "ROOT", tmp_path)
    docs = fixture_documents()
    pins = {name: sha(encoded(docs[name])) for name in m.PINS}
    monkeypatch.setattr(m, "PINS", pins)
    for name in m.METADATA:
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(encoded(docs[name]) if name.endswith(".json") else b"HH_261005 - Synthetic metadata only.\n")
    assets = {}
    for site in m.SITES:
        path = tmp_path / "synthetic_assets" / site
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"asset bytes are not read by qualifier")
        assets[site] = (path,)
    monkeypatch.setattr(m, "ASSETS", assets)
    auth = tmp_path / m.AUTHORIZATION
    auth.parent.mkdir(parents=True, exist_ok=True)
    auth.write_bytes(encoded(m.authorization_document()))
    return tmp_path, docs, sha(auth.read_bytes())


def test_complete_synthetic_metadata_keeps_all_approval_denials(synthetic):
    _, _, auth_sha = synthetic
    before = {name for name in sys.modules if name in ("torch", "carla")}
    value = m.qualify(auth_sha)
    assert value["status"] == "METADATA_PREPARED_NOT_INDEPENDENT_TEST_APPROVED"
    assert value["ancestry"]["declared_eligible_legacy_by_site"] == {"C_track_1_0_7": 922, "Town01": 638, "Town07": 927}
    assert not value["ancestry"]["reserved_sites_in_declared_TRAIN"]
    assert not value["ancestry"]["all_prior_current_model_inference_and_candidate_exposure_absence_verified"]
    assert not value["prospective_30_episode_matrix_declared"] and value["minimum_closed_loop_episodes_after_qualification"] == 30
    assert not value["TEST_payload_opened"] and not value["new_model_NN_or_training_executed"]
    assert not value["CARLA_GPU_SSH_or_install_executed"]
    assert value["main_functions_approved"] == value["subfunctions_approved"] == 0
    assert set(value["metadata_sha256"]) == set(m.METADATA)
    assert {name for name in sys.modules if name in ("torch", "carla")} == before
    for candidate in value["candidates"].values():
        assert candidate["all_named_assets_exist"] and not candidate["qualified_independent_TEST"]
        assert not candidate["current_owned_renderer_or_six_camera_quality_verified"]
        assert not candidate["current_model_exposure_complete"]


@pytest.mark.parametrize("name", ["TEST_training", "TEST_NN_or_loss_or_labels", "TEST_pixel_or_state_payload",
    "train_or_dev_role_change", "model_or_checkpoint_selection", "CARLA_GPU_SSH_or_install",
    "actual_capture_authorized_by_this_document", "pristine_or_independent_TEST_certified", "old_guard_modified"])
def test_authorization_cannot_widen_permissions(synthetic, name):
    root, _, _ = synthetic
    document = m.authorization_document()
    document[name] = True
    data = encoded(document)
    (root / m.AUTHORIZATION).write_bytes(data)
    with pytest.raises(RuntimeError, match="widened"):
        m.qualify(sha(data))


@pytest.mark.parametrize("mutation", ["extra_site", "other_weight", "extra_metadata", "extra_asset", "zero_not_false"])
def test_exact_scope_and_types_required(synthetic, mutation):
    root, _, _ = synthetic
    document = m.authorization_document()
    if mutation == "extra_site":
        document["candidate_sites"].append("Town07")
    elif mutation == "other_weight":
        document["model_weights_sha256"] = "f" * 64
    elif mutation == "extra_metadata":
        document["metadata_whitelist"].append("states.jsonl")
    elif mutation == "extra_asset":
        document["asset_lstat_whitelist"]["Town15"].append("/some/other/asset")
    else:
        document["TEST_training"] = 0
    data = encoded(document)
    (root / m.AUTHORIZATION).write_bytes(data)
    with pytest.raises(RuntimeError, match="widened"):
        m.qualify(sha(data))


@pytest.mark.parametrize("path", ["states.jsonl", "model_state.npz", "labels.json", "images/CAM_FRONT.jpg",
    "../metadata.json", "/etc/passwd", "storage/datasets/test/samples.jsonl"])
def test_payload_and_unlisted_paths_are_rejected_without_reads(path):
    with pytest.raises(RuntimeError, match="not authorized"):
        m.read_metadata(path)


@pytest.mark.parametrize("data", [b'{"same":1,"same":2}', b'{"value":NaN}', b'{"value":Infinity}'])
def test_duplicate_and_nonfinite_metadata_rejected(data):
    with pytest.raises(RuntimeError):
        m.unique_json(data)


@pytest.mark.parametrize("bad_sha", [None, True, 0, "0" * 63, "A" * 64, "g" * 64])
def test_bad_authorization_sha_rejected_before_read(bad_sha):
    with pytest.raises(RuntimeError, match="exact authorization"):
        m.qualify(bad_sha)


def test_changed_frozen_metadata_rejected(synthetic):
    root, _, auth_sha = synthetic
    (root / m.GUARD).write_bytes(b'{"sites":{}}')
    with pytest.raises(RuntimeError, match="seal changed"):
        m.qualify(auth_sha)


@pytest.mark.parametrize("site", m.SITES)
def test_old_reservations_cannot_be_reopened_even_with_resealed_fixture(synthetic, monkeypatch, site):
    root, docs, auth_sha = synthetic
    doc = copy.deepcopy(docs[m.GUARD])
    doc["sites"][site]["drive"] = ["reuse_existing_train"]
    payload = encoded(doc)
    (root / m.GUARD).write_bytes(payload)
    monkeypatch.setitem(m.PINS, m.GUARD, sha(payload))
    with pytest.raises(RuntimeError, match="reopened"):
        m.qualify(auth_sha)


@pytest.mark.parametrize("key", [m.RAW, m.BALANCED, m.RECOVERY, m.JOINT, m.GOOD_META])
def test_wrong_ancestry_fields_fail_closed(synthetic, monkeypatch, key):
    root, docs, auth_sha = synthetic
    doc = copy.deepcopy(docs[key])
    if key == m.RAW:
        doc["initialization"] = "adapted W1"
    elif key == m.BALANCED:
        doc["legacy_strata"]["straight_by_map"]["Town04"] = 1
    elif key == m.GOOD_META:
        doc["npz_sha256"] = "f" * 64
    else:
        doc["parent_npz_sha256"] = "f" * 64
    payload = encoded(doc)
    (root / key).write_bytes(payload)
    monkeypatch.setitem(m.PINS, key, sha(payload))
    with pytest.raises(RuntimeError):
        m.qualify(auth_sha)


def test_symlink_metadata_parent_and_final_are_rejected(tmp_path):
    real = tmp_path / "real"
    real.mkdir()
    (real / "metadata.json").write_bytes(b"{}")
    link = tmp_path / "link"
    link.symlink_to(real, target_is_directory=True)
    with pytest.raises(RuntimeError):
        m.bounded_bytes(link / "metadata.json")
    (tmp_path / "alias.json").symlink_to(real / "metadata.json")
    with pytest.raises(RuntimeError):
        m.bounded_bytes(tmp_path / "alias.json")


def test_oversize_metadata_rejected(tmp_path, monkeypatch):
    path = tmp_path / "metadata.json"
    path.write_bytes(b"12345")
    monkeypatch.setattr(m, "MAX_METADATA_BYTES", 4)
    with pytest.raises(RuntimeError, match="bounded"):
        m.bounded_bytes(path)


def test_asset_existence_is_not_runtime_or_independence_evidence(synthetic):
    root, _, auth_sha = synthetic
    m.ASSETS["Town15"][0].unlink()
    value = m.qualify(auth_sha)
    assert not value["candidates"]["Town15"]["all_named_assets_exist"]
    assert not value["candidates"]["Town15"]["qualified_independent_TEST"]
    assert not (root / m.REPORT).exists()


def test_create_only_report_and_symlink_race_preserve_existing_bytes(synthetic):
    root, _, auth_sha = synthetic
    value = m.qualify(auth_sha)
    m.save_report(value)
    before = (root / m.REPORT).read_bytes()
    with pytest.raises(FileExistsError):
        m.save_report(value)
    assert (root / m.REPORT).read_bytes() == before
    (root / m.REPORT).unlink()
    elsewhere = root / "existing.json"
    elsewhere.write_bytes(b"must preserve")
    (root / m.REPORT).symlink_to(elsewhere)
    with pytest.raises(RuntimeError):
        m.save_report(value)
    assert elsewhere.read_bytes() == b"must preserve"
