"""HH_261007 - Inspect one genuine native DDS take without models, training or system changes."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / 'storage/experiments/plans/2026-10-07/native_dds_info_probe_v1/ROOT_authority.json'
OUT = ROOT / 'storage/experiments/diagnostics/2026-10-07/native_dds_info_probe_v1'
TOPIC = '/portable_e2e/native_dds_info_probe'
NAMES = {'hh_native_info_publisher', 'hh_native_info_receiver'}


def require(value, why):
    if not value:
        raise ValueError(why)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def graph(node):
    names = node.get_node_names_and_namespaces()
    require(len(names) == len(set(names)) and all(n in NAMES and ns == '/' for n, ns in names), 'foreign/duplicate ROS graph denied')
    require({t for t, _ in node.get_topic_names_and_types()} <= {TOPIC, '/parameter_events'}, 'foreign graph topic denied')


def scalar(value):
    # HH_261007 - Preserve native fields and types; never derive a missing sender GID from graph names.
    if type(value) in (str, int, bool, float) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): scalar(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, bytes, bytearray)):
        return [scalar(v) for v in value]
    return {'unsupported_type': type(value).__module__ + '.' + type(value).__name__}


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument('--authority-sha256', required=True)
    args = parser.parse_args()
    require(sha(PLAN) == args.authority_sha256, 'external exact authority required')
    authority = json.loads(PLAN.read_bytes())
    require(authority['source_sha256'] == sha(__file__) and authority['authorized_native_take_only'] is True,
            'source/native-only scope differs')
    require(os.environ.get('CUDA_VISIBLE_DEVICES') == '' and os.environ.get('PYTHONNOUSERSITE') == '1'
            and os.environ.get('ROS_DOMAIN_ID') == '93' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
            and os.environ.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp'
            and os.environ.get('ROS_LOG_DIR') == str(OUT / 'ros_logs'), 'clean private CPU environment required')
    for name, pin in authority['input_sha256'].items():
        require(sha(name) == pin, 'input seal differs')
    require(not OUT.exists() and OUT.resolve() == OUT and shutil.disk_usage(ROOT).free >= 68 * 1024**2, 'fresh bounded diagnostic required')
    resource.setrlimit(resource.RLIMIT_FSIZE, (1024**2, 1024**2))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    lock = (ROOT / 'storage/assets/locks/autoware_e2e_runtime.lock').open('a+')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    OUT.mkdir(parents=True)
    result = dict(provenance='HH_261007 - Actual single owned native DDS message; no NN or training.',
                  status='FAILED_RETAINED', training_steps=0, NN_inferences=0, GPU_used=False,
                  CARLA_started=False, actuation=False, source_identity_approved=False)
    pub = receiver = None
    rclpy = None
    started = time.monotonic()
    def timeout(signum, frame):
        raise TimeoutError('fixed15s native diagnostic expired')
    old = signal.signal(signal.SIGALRM, timeout)
    signal.setitimer(signal.ITIMER_REAL, 15)
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        require('torch' not in sys.modules, 'native diagnostic must not import Torch')
        rclpy.init(args=[])
        pub = Node('hh_native_info_publisher', enable_rosout=False, start_parameter_services=False)
        receiver = Node('hh_native_info_receiver', enable_rosout=False, start_parameter_services=False)
        publisher = pub.create_publisher(String, TOPIC, 10)
        subscription = receiver.create_subscription(String, TOPIC, lambda msg: None, 10)
        while publisher.get_subscription_count() != 1:
            graph(pub)
            rclpy.spin_once(pub, timeout_sec=.02)
        graph(receiver)
        payload = 'HH_261007-native-only-' + args.authority_sha256
        publisher.publish(String(data=payload))
        native = None
        while native is None:
            graph(receiver)
            # HH_261007 - Take directly from our owned subscription, bypassing ONLY the old executor's metadata discard.
            with subscription.handle:
                native = subscription.handle.take_message(String, False)
            if native is None:
                time.sleep(.005)
        require(isinstance(native, tuple) and len(native) == 2 and native[0].data == payload, 'actual own payload/tuple differs')
        info = native[1]
        native_gid = info.get('publisher_gid') if isinstance(info, dict) else getattr(info, 'publisher_gid', None)
        endpoints = receiver.get_publishers_info_by_topic(TOPIC)
        require(len(endpoints) == 1 and endpoints[0].node_name == 'hh_native_info_publisher', 'one owned graph endpoint required')
        result.update(status='COMPLETE_NATIVE_INFO_DIAGNOSTIC', native_info_type=type(info).__name__, native_info=scalar(info),
                      native_publisher_gid_available=native_gid is not None, graph_endpoint_gid=list(endpoints[0].endpoint_gid),
                      native_gid_equals_graph=None if native_gid is None else bytes(native_gid) == bytes(endpoints[0].endpoint_gid),
                      observed_graph=pub.get_node_names_and_namespaces(), payload_received_exactly=True)
    except BaseException as error:
        result['error'] = repr(error)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old)
        for node in (receiver, pub):
            if node is not None:
                node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        result.update(wall_seconds=time.monotonic() - started, source_unchanged=sha(__file__) == authority['source_sha256'],
                      input_seals_unchanged=all(sha(name) == pin for name, pin in authority['input_sha256'].items()))
        with (OUT / 'result.json').open('x') as stream:
            json.dump(result, stream, sort_keys=True, allow_nan=False)
            stream.write('\n')
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()
    print(json.dumps(result, sort_keys=True))
    return 0 if result['status'] == 'COMPLETE_NATIVE_INFO_DIAGNOSTIC' else 1


if __name__ == '__main__':
    raise SystemExit(main())
