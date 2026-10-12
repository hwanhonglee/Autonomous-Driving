"""HH_260906 - Test capture boundaries without CARLA or GPU allocation."""
import hashlib
import json
from types import SimpleNamespace
from queue import Queue
import pytest
from scripts.e2e import collect_carla_mapping_probe as probe
from scripts.e2e.collect_carla_mapping_probe import receive, parse_args


def test_receive_discards_old_and_matches():
    q = Queue()
    q.put(SimpleNamespace(frame=1)); q.put(SimpleNamespace(frame=2))
    assert receive(q, 2).frame == 2


def test_receive_rejects_skipped():
    q = Queue(); q.put(SimpleNamespace(frame=3))
    with pytest.raises(ValueError):
        receive(q, 2)


@pytest.mark.parametrize("args", [["--frames", "0"], ["--frames", "61"],
                                     ["--host", "remote"], ["--port", "0"], ["--allow-map-load"]])
def test_probe_invalid_args(tmp_path, args):
    with pytest.raises(SystemExit):
        parse_args([str(tmp_path / "new"), "route.json", *args])


def test_existing_capture_not_overwritten(tmp_path):
    with pytest.raises(SystemExit):
        parse_args([str(tmp_path), "route.json"])


def test_town06_scout_requires_exact_route_bytes_and_world(tmp_path, monkeypatch):
    """HH_260906 - A copied or changed Town06 route must fail before any CARLA launch."""
    route = tmp_path / "p06.json"
    route.write_text(json.dumps({"town": "Town06", "coordinate_reference": "base_link",
                                 "route": [{"x": 0}, {"x": 1}]}))
    digest = hashlib.sha256(route.read_bytes()).hexdigest()
    monkeypatch.setattr(probe, "TOWN06_SCOUT_ROUTE_SHA256", digest)
    argv = [str(tmp_path / "capture"), str(route), "--motion-mode", "basic-agent",
            "--frames", "240", "--ignore-traffic-lights", "--town06-sign-scout"]
    args = parse_args(argv)
    loaded, observed = probe.load_bound_route(args, "Town06")
    assert loaded["town"] == "Town06" and observed == digest
    with pytest.raises(ValueError, match="world differs"):
        probe.load_bound_route(args, "Town01")
    route.write_text(route.read_text() + "\n")
    with pytest.raises(SystemExit):
        parse_args(argv)


def test_town06_scout_requires_explicit_physical_scope(tmp_path):
    """HH_260906 - The opt-in is not a general-purpose multi-town or kinematic switch."""
    with pytest.raises(SystemExit):
        parse_args([str(tmp_path / "capture"), str(tmp_path / "p06.json"), "--town06-sign-scout"])
    with pytest.raises(ValueError, match="route town differs"):
        probe.validate_route_binding({"town": "Town06", "coordinate_reference": "base_link",
                                      "route": [{}, {}]}, probe.TOWN06_SCOUT_ROUTE_SHA256,
                                     "Town06", False)
    probe.validate_route_binding({"town": "Town01", "coordinate_reference": "base_link",
                                  "route": [{}, {}]}, "unchanged Town01 route", "Town01", False)


@pytest.mark.parametrize("scope,town,hash_attr", [
    ("--town02-sign-train-scout", "Town02_Opt", "TOWN02_TRAIN_SCOUT_ROUTE_SHA256"),
    ("--town02-sign-train-scout-p02", "Town02_Opt", "TOWN02_P02_TRAIN_SCOUT_ROUTE_SHA256"),
    ("--town03-generalization-train", "Town03", "TOWN03_TRAIN_ROUTE_SHA256"),
    ("--town05-generalization-dev", "Town05_Opt", "TOWN05_DEV_ROUTE_SHA256"),
    ("--town05-sign-dev-scout", "Town05_Opt", "TOWN05_SIGN_DEV_SCOUT_ROUTE_SHA256"),
    ("--town07-generalization-test", "Town07", "TOWN07_TEST_ROUTE_SHA256"),
])
def test_cross_town_scope_requires_exact_route_bytes_and_world(tmp_path, monkeypatch,
                                                                  scope, town, hash_attr):
    """HH_260906 - TRAIN and sealed evaluation scopes cannot admit changed bytes or another CARLA world."""
    route = tmp_path / "route.json"
    route.write_text(json.dumps({"town": town, "coordinate_reference": "base_link",
                                 "route": [{"x": 0}, {"x": 1}]}))
    digest = hashlib.sha256(route.read_bytes()).hexdigest()
    monkeypatch.setattr(probe, hash_attr, frozenset({digest}))
    argv = [str(tmp_path / "capture"), str(route), "--motion-mode", "basic-agent",
            "--frames", "240", "--ignore-traffic-lights", scope]
    args = parse_args(argv)
    loaded, observed = probe.load_bound_route(args, town)
    assert loaded["town"] == town and observed == digest
    with pytest.raises(ValueError, match="world differs"):
        probe.load_bound_route(args, "Town01")
    route.write_text(route.read_text() + "\n")
    with pytest.raises(SystemExit):
        parse_args(argv)


@pytest.mark.parametrize("town,hashes,other_town", [
    ("Town03", probe.TOWN03_TRAIN_ROUTE_SHA256, "Town07"),
    ("Town07", probe.TOWN07_TEST_ROUTE_SHA256, "Town03"),
])
def test_cross_town_scope_accepts_only_two_predeclared_route_hashes(town, hashes, other_town):
    """HH_260906 - Each route catalog pair has exact SHA membership, not arbitrary Town access."""
    route = {"town": town, "coordinate_reference": "base_link", "route": [{}, {}]}
    flags = {"town03_generalization_train": town == "Town03",
             "town07_generalization_test": town == "Town07"}
    assert len(hashes) == 2
    for digest in hashes:
        probe.validate_route_binding(route, digest, town, False, **flags)
    with pytest.raises(ValueError, match="pinned route bytes"):
        probe.validate_route_binding(route, "0" * 64, town, False, **flags)
    with pytest.raises(ValueError, match="world differs"):
        probe.validate_route_binding(route, next(iter(hashes)), other_town, False, **flags)
    with pytest.raises(ValueError, match="route town differs"):
        probe.validate_route_binding(dict(route, town=other_town), next(iter(hashes)), town,
                                     False, **flags)


def test_town05_dev_scope_is_single_route_and_rejects_driving_test_site():
    """HH_260906 - Keep the map DEV on Town05 instead of the held-out driving Town04 route."""
    route = {"town": "Town05_Opt", "coordinate_reference": "base_link", "route": [{}, {}]}
    (digest,) = tuple(probe.TOWN05_DEV_ROUTE_SHA256)
    probe.validate_route_binding(route, digest, "Town05_Opt", False,
                                 town05_generalization_dev=True)
    with pytest.raises(ValueError, match="pinned route bytes"):
        probe.validate_route_binding(route, "0" * 64, "Town05_Opt", False,
                                     town05_generalization_dev=True)
    with pytest.raises(ValueError, match="route town differs"):
        probe.validate_route_binding(dict(route, town="Town04"), digest, "Town05_Opt", False,
                                     town05_generalization_dev=True)
    with pytest.raises(ValueError, match="world differs"):
        probe.validate_route_binding(route, digest, "Town04", False,
                                     town05_generalization_dev=True)
    with pytest.raises(ValueError, match="mutually exclusive"):
        probe.validate_route_binding(route, digest, "Town05_Opt", False,
                                     town03_generalization_train=True,
                                     town05_generalization_dev=True)


def test_town05_sign_dev_scout_is_not_straight_dev_or_train():
    """HH_260906 - Bind the sign DEV scout to right p00 without widening the original straight DEV scope."""
    route = {"town": "Town05_Opt", "coordinate_reference": "base_link", "route": [{}, {}]}
    (right_digest,) = tuple(probe.TOWN05_SIGN_DEV_SCOUT_ROUTE_SHA256)
    (straight_digest,) = tuple(probe.TOWN05_DEV_ROUTE_SHA256)
    assert right_digest != straight_digest
    probe.validate_route_binding(route, right_digest, "Town05_Opt", False,
                                 town05_sign_dev_scout=True)
    with pytest.raises(ValueError, match="pinned route bytes"):
        probe.validate_route_binding(route, straight_digest, "Town05_Opt", False,
                                     town05_sign_dev_scout=True)
    with pytest.raises(ValueError, match="pinned route bytes"):
        probe.validate_route_binding(route, right_digest, "Town05_Opt", False,
                                     town05_generalization_dev=True)
    with pytest.raises(ValueError, match="route town differs"):
        probe.validate_route_binding(dict(route, town="Town02_Opt"), right_digest, "Town05_Opt",
                                     False, town05_sign_dev_scout=True)
    with pytest.raises(ValueError, match="world differs"):
        probe.validate_route_binding(route, right_digest, "Town02_Opt", False,
                                     town05_sign_dev_scout=True)
    with pytest.raises(ValueError, match="mutually exclusive"):
        probe.validate_route_binding(route, right_digest, "Town05_Opt", False,
                                     town05_generalization_dev=True,
                                     town05_sign_dev_scout=True)


def test_town02_train_scout_does_not_open_other_sites():
    """HH_260906 - A TRAIN-site scout cannot become a generic Town or DEV capture."""
    route = {"town": "Town02_Opt", "coordinate_reference": "base_link", "route": [{}, {}]}
    (digest,) = tuple(probe.TOWN02_TRAIN_SCOUT_ROUTE_SHA256)
    probe.validate_route_binding(route, digest, "Town02_Opt", False,
                                 town02_sign_train_scout=True)
    with pytest.raises(ValueError, match="pinned route bytes"):
        probe.validate_route_binding(route, "0" * 64, "Town02_Opt", False,
                                     town02_sign_train_scout=True)
    with pytest.raises(ValueError, match="route town differs"):
        probe.validate_route_binding(dict(route, town="Town07"), digest, "Town02_Opt", False,
                                     town02_sign_train_scout=True)
    with pytest.raises(ValueError, match="mutually exclusive"):
        probe.validate_route_binding(route, digest, "Town02_Opt", False,
                                     town02_sign_train_scout=True,
                                     town05_generalization_dev=True)


def test_town02_p02_train_scout_cannot_reuse_p00_or_other_scopes():
    """HH_260906 - The planned second Town02 corridor is not an alias for p00 or an unrestricted Town."""
    route = {"town": "Town02_Opt", "coordinate_reference": "base_link", "route": [{}, {}]}
    (p02_digest,) = tuple(probe.TOWN02_P02_TRAIN_SCOUT_ROUTE_SHA256)
    (p00_digest,) = tuple(probe.TOWN02_TRAIN_SCOUT_ROUTE_SHA256)
    assert p02_digest != p00_digest
    probe.validate_route_binding(route, p02_digest, "Town02_Opt", False,
                                 town02_sign_train_scout_p02=True)
    with pytest.raises(ValueError, match="pinned route bytes"):
        probe.validate_route_binding(route, p00_digest, "Town02_Opt", False,
                                     town02_sign_train_scout_p02=True)
    with pytest.raises(ValueError, match="pinned route bytes"):
        probe.validate_route_binding(route, p02_digest, "Town02_Opt", False,
                                     town02_sign_train_scout=True)
    with pytest.raises(ValueError, match="route town differs"):
        probe.validate_route_binding(dict(route, town="Town07"), p02_digest, "Town02_Opt",
                                     False, town02_sign_train_scout_p02=True)
    with pytest.raises(ValueError, match="world differs"):
        probe.validate_route_binding(route, p02_digest, "Town05_Opt", False,
                                     town02_sign_train_scout_p02=True)
    with pytest.raises(ValueError, match="mutually exclusive"):
        probe.validate_route_binding(route, p02_digest, "Town02_Opt", False,
                                     town02_sign_train_scout=True,
                                     town02_sign_train_scout_p02=True)


def test_cross_town_scopes_require_physical_run_and_are_exclusive(tmp_path):
    """HH_260906 - A sealed TEST candidate needs the same explicit 240-frame owner contract as TRAIN."""
    base = [str(tmp_path / "capture"), str(tmp_path / "route.json")]
    for scope in ("--town02-sign-train-scout", "--town02-sign-train-scout-p02",
                  "--town03-generalization-train",
                  "--town05-generalization-dev",
                  "--town05-sign-dev-scout",
                  "--town07-generalization-test"):
        with pytest.raises(SystemExit):
            parse_args([*base, scope])
    with pytest.raises(SystemExit):
        parse_args([*base, "--town03-generalization-train", "--town07-generalization-test"])
    with pytest.raises(SystemExit):
        parse_args([*base, "--town02-sign-train-scout", "--town02-sign-train-scout-p02"])
    with pytest.raises(SystemExit):
        parse_args([*base, "--town05-generalization-dev", "--town05-sign-dev-scout"])
    route = {"town": "Town03", "coordinate_reference": "base_link", "route": [{}, {}]}
    with pytest.raises(ValueError, match="mutually exclusive"):
        probe.validate_route_binding(route, "unused", "Town03", False, True, True)


def test_town13_render_sensor_scout_requires_exact_route_and_full_source_map(tmp_path, monkeypatch):
    """HH_260928 - Authenticate one diagnostic route and the full Town13 editor asset."""
    route_file = tmp_path / "town13_right_s0001_p00.json"
    route_file.write_text(json.dumps({"town": "Town13", "coordinate_reference": "base_link",
                                      "route": [{"x": 0}, {"x": 1}]}))
    digest = hashlib.sha256(route_file.read_bytes()).hexdigest()
    monkeypatch.setattr(probe, "TOWN13_RENDER_SENSOR_SCOUT_ROUTE_SHA256", frozenset({digest}))
    argv = [str(tmp_path / "capture"), str(route_file), "--motion-mode", "basic-agent",
            "--frames", "240", "--ignore-traffic-lights", "--town13-render-sensor-scout"]
    args = parse_args(argv)
    for full_name in sorted(probe.TOWN13_LIVE_MAP_NAMES):
        loaded, observed = probe.load_bound_route(args, full_name)
        assert loaded["town"] == "Town13" and observed == digest
    for wrong_name in ("Town13", "Other/Maps/Town13/Town13", "Carla/Maps/Town13/Town13_copy"):
        with pytest.raises(ValueError, match="source asset"):
            probe.load_bound_route(args, wrong_name)
    route_file.write_text(route_file.read_text() + "\n")
    with pytest.raises(SystemExit):
        parse_args(argv)


def test_town13_render_sensor_scout_cannot_become_generic_dev_or_kinematic(tmp_path, monkeypatch):
    """HH_260928 - Keep the render scout separate from dataset admission and other scopes."""
    route = {"town": "Town13", "coordinate_reference": "base_link", "route": [{}, {}]}
    (digest,) = tuple(probe.TOWN13_RENDER_SENSOR_SCOUT_ROUTE_SHA256)
    assert digest == "6927c0de706e5ffb89afce25a75a892d56f7ee212b577002787605802839520b"
    probe.validate_route_binding(route, digest, "Carla/Maps/Town13/Town13", False,
                                 town13_render_sensor_scout=True)
    with pytest.raises(ValueError, match="pinned route bytes"):
        probe.validate_route_binding(route, "0" * 64, "Carla/Maps/Town13/Town13", False,
                                     town13_render_sensor_scout=True)
    with pytest.raises(ValueError, match="route town differs"):
        probe.validate_route_binding(dict(route, town="Town02_Opt"), digest,
                                     "Carla/Maps/Town13/Town13", False,
                                     town13_render_sensor_scout=True)
    with pytest.raises(ValueError, match="mutually exclusive"):
        probe.validate_route_binding(route, digest, "Carla/Maps/Town13/Town13", False,
                                     town13_render_sensor_scout=True,
                                     town05_sign_dev_scout=True)
    route_file = tmp_path / "route.json"
    route_file.write_text(json.dumps(route))
    monkeypatch.setattr(probe, "TOWN13_RENDER_SENSOR_SCOUT_ROUTE_SHA256",
                        frozenset({hashlib.sha256(route_file.read_bytes()).hexdigest()}))
    base = [str(tmp_path / "capture"), str(route_file)]
    for extras in ([], ["--motion-mode", "basic-agent", "--frames", "239",
                        "--ignore-traffic-lights"],
                   ["--motion-mode", "basic-agent", "--frames", "240"]):
        with pytest.raises(SystemExit):
            parse_args([*base, *extras, "--town13-render-sensor-scout"])
    with pytest.raises(SystemExit):
        parse_args([*base, "--town13-render-sensor-scout", "--town05-sign-dev-scout"])
