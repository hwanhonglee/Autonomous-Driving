"""HH_261001 - Prospective two-attempt W1 brake-only contract; declaration is explicit and create-only."""

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[2]
RUN_ROOT = ROOT / "storage/experiments/validation/2026-10-01/town07_w1_moving_brake_exploratory_v2"
PLAN = RUN_ROOT.with_name("town07_w1_moving_brake_exploratory_plan_v2.json")
FLOOR = 5 * 1024**3
ARM_RESERVE, SHARED_RESERVE = 256 * 1024**2, 64 * 1024**2
SAMPLE_LIMIT, CLOSING_RESERVE = 128 * 1024**2, 16 * 1024**2
CHECKPOINT = "b9ee946ada6824e7d79f51bc1693463dc506db3faaa9038ddaa05816cf7ecf76"
CHECKPOINT_SCHEMA = "portable_e2e.warmup_supervision_ab_20261001.v1"
MODEL_STATE = "ffcb58cd319bb6d1db412da71f8f9ef8697613482cdd2d760c64b7abbf6f110f"
WEIGHTS = ROOT / "storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1/exports/W1_warmup_anchor_off.npz"
METADATA = WEIGHTS.with_suffix(".json")
WEIGHTS_SHA = "4176ebe38bccd85394de480946614bfb306c571eb9a677282dd840d78c5354cf"
METADATA_SHA = "4b559b17387138faf2570c28f22974bacc9aa67b59a8c810ba5962a946128e1f"
ROUTE = ROOT / "storage/datasets/raw/carla/common10_v1/2026-09-04/30kph/town07/straight/ClearNoon/seed_0000/run_001/episode/route.json"
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
EGG = CARLA_ROOT / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
XODR = CARLA_ROOT / "CarlaUE4/Content/Carla/Maps/OpenDrive/Town07.xodr"
MAPPING = ROOT / "autonomy/autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml"
CALIBRATION = ROOT / "autonomy/src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml"
FREEZE = ROOT / "docs/assets/validation/2026-10-01/selector_phase_ab/DIRECTIONAL_REPLAY_SOURCE_FREEZE.json"
FREEZE_SHA = "d2a8fae7537b45a18d8bc4e29c6a8ae7da9f2d9d61dc535b35a3480b2d071ed2"
SCHEMA = "portable_e2e.town07_w1_moving_brake_exploratory.v2"
ARMS = ("A_original", "B_suppress_normal_moving_brake")
NEW_SOURCES = (
    "tools/e2e/town07_moving_brake_ab_control_v2_20261001.py",
    "tools/e2e/town07_moving_brake_ab_contract_v2_20261001.py",
    "tools/e2e/research_town07_w1_moving_brake_ab_v2_20261001.py",
    "learning/tests/test_town07_moving_brake_ab_control_v2_20261001.py",
    "learning/tests/test_town07_moving_brake_ab_contract_v2_20261001.py",
    "learning/tests/test_town07_w1_moving_brake_ab_v2_20261001.py",
)
HELPER_SOURCES = (
    "tools/e2e/research_warmup_supervision_candidate_20261001.py",
    "tools/e2e/research_warmup_duration_w2_replay_20261001.py",
    "tools/e2e/collect_w1_town07_long_20261001.py",
)
AGENT_SOURCES = tuple(CARLA_ROOT / "PythonAPI/carla/agents" / name for name in (
    "navigation/basic_agent.py", "navigation/local_planner.py", "navigation/controller.py",
    "navigation/global_route_planner.py", "tools/misc.py"))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "regular non-symlink file required: " + str(path))
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            result.update(block)
    return result.hexdigest()


def read_json(path):
    digest(path)
    require(Path(path).stat().st_size <= 4 * 1024**2, "bounded JSON required")
    return json.loads(Path(path).read_text())


def write_new(path, value):
    require(not Path(path).is_symlink(), "create-only non-symlink JSON required")
    with Path(path).open("x") as stream:
        json.dump(value, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write("\n")


def trial_specs():
    return tuple(dict(index=index, trial_id=name, arm=arm,
                      output=str(RUN_ROOT / name)) for index, (name, arm) in enumerate((
                          ("A1_original", ARMS[0]), ("B1_suppress_normal_moving_brake", ARMS[1]))))


def trial(index):
    require(type(index) is int and index in (0, 1), "one fixed exploratory trial index required")
    return trial_specs()[index]


def output(index):
    return Path(trial(index)["output"])


def assets():
    return {str(WEIGHTS): WEIGHTS_SHA, str(METADATA): METADATA_SHA,
        str(ROUTE): "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9",
        str(XODR): "be730652769e993e27f748ba5793dba9e49d299205146a7a8db0bcd17af877bf",
        str(EGG): "6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a",
        str(MAPPING): "9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136",
        str(CALIBRATION): "5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea"}


def fixed_contract():
    return dict(schema=SCHEMA, trial_order=list(trial_specs()), attempts_per_arm=1,
        prospective=True, exploratory=True, ABBA=False, independent_replication=False,
        automatic_retry=False, outcome_dependent_selection=False, model_optimizer_updates=0,
        checkpoint_sha256=CHECKPOINT, checkpoint_schema=CHECKPOINT_SCHEMA,
        model_state_sha256=MODEL_STATE, parent_runtime_freeze_sha256=FREEZE_SHA,
        intervention="B changes only original normal tracker brake to zero when original desired_speed_mps > 0.05",
        original_stop_command_unchanged=True, original_emergency_authority_unchanged=True,
        feedforward_added=False, integral_added=False, throttle_floor_added=False,
        speed_preview_s=.3, throttle_gain=.7, brake_gain=.7, maximum_throttle=.5,
        town="Town07", vehicle="vehicle.toyota.prius", weather="ClearNoon", seed=0,
        port_triplet=[2440, 2441, 2442], physics_hz=20, capture_hz=10,
        total_capture_seconds=20, stationary_warmup_seconds=1.2,
        teacher_target_kph=9, learned_speed_cap_mps=2.5, maximum_route_cte_m=1.,
        maximum_measured_actor_planar_speed_mps=2.7,
        speed_guard_source="immutable native actor snapshot world XY velocity norm",
        common_speed_guard="retain violating native observation, then original full-brake exception cleanup; no resumption",
        native_control_disk_floor_guard=True,
        spawn_z_offset_m=.5, goal_stop_profile="disabled", control_transport="acknowledged_batch",
        raw_argmax_and_runtime_gates_unchanged=True, held_control_between_camera_frames=True,
        sparse_exact_input_policy_call_indices="0,10,20,...", all_exact_inputs_retained=False,
        all_original_JPEG_native_ACK_predictions_retained=True,
        per_arm_reserved_bytes=ARM_RESERVE, sample_directory_maximum_bytes=SAMPLE_LIMIT,
        shared_safety_reserved_bytes=SHARED_RESERVE, free_disk_floor_bytes=FLOOR,
        closing_reserved_bytes=CLOSING_RESERVE,
        failed_attempts_retained=True, full_feature_approval=False,
        learned_driving_success_claimed=False, training_data_approved=False,
        vehicle_control_approved=False, model_promoted=False)


def runtime_sources():
    require(digest(FREEZE) == FREEZE_SHA, "existing runtime freeze bytes differ")
    inherited = read_json(FREEZE)
    require(inherited.get("schema") == "portable_e2e.directional_replay_runtime_source_freeze.v1",
            "existing runtime freeze schema differs")
    pins = dict(inherited["source_sha256"])
    for name, expected in pins.items():
        require(digest(ROOT / name) == expected, "existing frozen runtime source changed: " + name)
    for name in NEW_SOURCES + HELPER_SOURCES + (str(FREEZE.relative_to(ROOT)),):
        pins[name] = digest(ROOT / name)
    return pins


def build_plan():
    require(not any(output(index).exists() or output(index).is_symlink() for index in (0, 1)),
            "declaration must precede either physical attempt")
    asset_pins = assets()
    require(all(digest(name) == expected for name, expected in asset_pins.items()),
            "fixed W1 or packaged assets differ")
    return dict(fixed_contract(), declared_at_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=runtime_sources(), external_agent_sha256={str(path): digest(path) for path in AGENT_SOURCES},
        asset_sha256=asset_pins)


def verify_plan(plan, *, expected_sha256):
    require(isinstance(expected_sha256, str) and len(expected_sha256) == 64
            and all(c in "0123456789abcdef" for c in expected_sha256), "exact plan SHA256 required")
    require(digest(PLAN) == expected_sha256, "create-only declaration bytes differ")
    fixed = fixed_contract()
    require(isinstance(plan, dict) and set(plan) == set(fixed) | {
        "declared_at_utc", "source_sha256", "external_agent_sha256", "asset_sha256"},
        "declaration keys differ")
    require(all(plan[key] == value for key, value in fixed.items()), "fixed prospective contract differs")
    stamp = datetime.fromisoformat(plan["declared_at_utc"])
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds() == 0
            and stamp <= datetime.now(timezone.utc), "prior UTC declaration required")
    require(plan["source_sha256"] == runtime_sources(), "declared local source closure changed")
    require(plan["asset_sha256"] == assets(), "declared asset identities differ")
    require(set(plan["external_agent_sha256"]) == {str(path) for path in AGENT_SOURCES},
            "declared agent source closure differs")
    require(all(digest(name) == expected for name, expected in dict(
        plan["asset_sha256"], **plan["external_agent_sha256"]).items()),
        "declared assets or external agent sources changed")


def disk_check(arms=1, free=None):
    require(type(arms) is int and arms in (1, 2), "one arm or initial exploratory pair reserve required")
    available = shutil.disk_usage(ROOT).free if free is None else free
    needed = FLOOR + arms * ARM_RESERVE + SHARED_RESERVE
    require(type(available) is int and available >= needed,
            "projected exploratory disk headroom unavailable: need " + str(needed) + ", have " + str(available))
    return dict(free_bytes=available, required_bytes=needed, floor_bytes=FLOOR,
                reserved_remaining_arms=arms, per_arm_reserved_bytes=ARM_RESERVE,
                sample_directory_maximum_bytes=SAMPLE_LIMIT, shared_safety_reserved_bytes=SHARED_RESERVE)
