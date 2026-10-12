"""HH_261007 - Record a clean canonical CPU/ROS environment without model loading, nodes, training or CUDA initialization."""
from __future__ import annotations

import argparse
import hashlib
import importlib
import inspect
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / 'storage/experiments/diagnostics/2026-10-07/nontraining_cpu_ros_environment_v1'


def require(value, message):
    if not value:
        raise RuntimeError(message)


def module_origin(module):
    raw = Path(module.__file__)
    return dict(imported_path=str(raw), resolved_path=str(raw.resolve()),
                source_sha256=hashlib.sha256(raw.read_bytes()).hexdigest())


def check():
    # HH_261007 - Refuse the inherited mixed workspace and record only selected nonsecret environment keys.
    require(sys.executable == '/usr/bin/python3', 'existing system Python CPU probe only')
    require(os.environ.get('CUDA_VISIBLE_DEVICES') == '' and os.environ.get('PYTHONNOUSERSITE') == '1',
            'CPU-only and disabled user site required')
    require(os.environ.get('ROS_DOMAIN_ID') == '93' and os.environ.get('ROS_LOCALHOST_ONLY') == '1',
            'prospective private localhost ROS domain required, without joining it')
    require(os.environ.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp', 'fixed existing Cyclone RMW required')
    require('/home/a/carla-autoware-universe/autoware' not in os.environ.get('AMENT_PREFIX_PATH', '')
            and '/home/a/carla-autoware-universe/autoware' not in os.environ.get('PYTHONPATH', ''),
            'unrelated Autoware overlay still inherited')
    source_before = Path(__file__).read_bytes()
    names = ('torch', 'numpy', 'PIL', 'rclpy', 'tf2_ros', 'autoware_planning_msgs',
             'autoware_vehicle_msgs', 'sensor_msgs', 'nav_msgs', 'geometry_msgs', 'rosgraph_msgs', 'tf2_msgs')
    modules = {name: importlib.import_module(name) for name in names}
    for name in ('autoware_planning_msgs', 'autoware_vehicle_msgs'):
        require(Path(modules[name].__file__).is_relative_to(ROOT / 'autonomy/install'),
                'Autoware message import not from canonical install')
        require(Path(modules[name].__file__).resolve().is_relative_to(ROOT / 'autonomy'),
                'canonical message build link escapes autonomy')
    from rclpy.serialization import deserialize_message, serialize_message
    from autoware_planning_msgs.msg import Trajectory
    from autoware_vehicle_msgs.msg import SteeringReport
    from sensor_msgs.msg import CameraInfo, CompressedImage
    from nav_msgs.msg import Odometry, Path as PathMessage
    from geometry_msgs.msg import AccelWithCovarianceStamped, TransformStamped
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    serialized = []
    for cls in (Trajectory, SteeringReport, CameraInfo, CompressedImage, Odometry, PathMessage,
                AccelWithCovarianceStamped, TransformStamped, Clock, TFMessage):
        first = cls()
        raw = serialize_message(first)
        second = deserialize_message(raw, cls)
        require(serialize_message(second) == raw, 'default ROS message serialization mismatch')
        serialized.append(dict(type=cls.__module__ + '.' + cls.__name__, bytes=len(raw),
                               sha256=hashlib.sha256(raw).hexdigest(), roundtrip=True))
    torch = modules['torch']
    require(not torch.cuda.is_initialized(), 'unexpected CUDA initialization')
    require(Path(__file__).read_bytes() == source_before, 'probe source changed')
    return dict(schema='portable_e2e.nontraining_cpu_ros_environment.20261007.v1',
                provenance='HH_261007 - Real existing CPU imports and default-message serialization only; no model, ROS graph, sensor, GPU or training.',
                observed_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                python_executable=sys.executable, python_version=sys.version,
                versions={name: getattr(modules[name], '__version__', 'not_exported') for name in names},
                origins={name: module_origin(module) for name, module in modules.items()},
                selected_environment={key: os.environ.get(key) for key in ('CUDA_VISIBLE_DEVICES', 'PYTHONNOUSERSITE',
                    'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION', 'AMENT_PREFIX_PATH', 'PYTHONPATH')},
                transformer_encoder_layer_signature=str(inspect.signature(torch.nn.TransformerEncoderLayer)),
                source_sha256=hashlib.sha256(source_before).hexdigest(), message_checks=serialized,
                message_checks_passed=len(serialized), CUDA_initialized=False, ROS_nodes_started=0,
                model_weights_loaded=0, model_forwards=0, optimizer_steps=0,
                historical_V5_package_version_or_numerical_parity_verified=False,
                new_direct_and_ROS_must_use_the_same_environment=True,
                installations_deletions_or_system_changes=0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--write-report', action='store_true')
    args = parser.parse_args(argv)
    require(args.write_report, 'explicit fresh report write required')
    require(not OUTPUT.exists() and OUTPUT.resolve() == OUTPUT, 'fresh canonical output required')
    capacity = os.statvfs(ROOT)
    require(capacity.f_bavail * capacity.f_frsize >= 64 * 1024**2, 'small metadata reserve unavailable')
    report = check()
    payload = (json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()
    require(len(payload) <= 64 * 1024, 'bounded report exceeded')
    OUTPUT.mkdir(parents=True, mode=0o700)
    with (OUTPUT / 'result.json').open('xb') as output:
        output.write(payload)
        output.flush()
        os.fsync(output.fileno())
    print(json.dumps(dict(output=str(OUTPUT), sha256=hashlib.sha256(payload).hexdigest(),
                          message_checks_passed=report['message_checks_passed'], CUDA_initialized=False,
                          ROS_nodes_started=0, model_forwards=0)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
