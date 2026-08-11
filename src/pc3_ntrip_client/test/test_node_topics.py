import os
import time

# HH_260811 - Verify both correction topics retain the historical rtcm_msgs injection type.

from pc3_ntrip_client.node import NtripClientNode
from pc3_ntrip_client.rtcm import crc24q
import rclpy
from rtcm_msgs.msg import Message
from test_config import CONFIG


class WaitingStream:
    """Block without using a live caster until the node requests shutdown."""

    def __init__(self, _config):
        pass

    def stream(self, stop_event, _on_data):
        stop_event.wait(2.0)

    def close(self):
        pass


def test_topic_message_types(tmp_path, monkeypatch):
    path = tmp_path / "ntrip_config.yaml"
    path.write_text(CONFIG.replace("enabled: true", "enabled: false"))
    os.chmod(path, 0o600)
    monkeypatch.setattr("pc3_ntrip_client.node.NtripStream", WaitingStream)
    rclpy.init(args=["--ros-args", "-p", f"config_file:={path}"])
    node = NtripClientNode()
    try:
        assert [publisher.msg_type for publisher in node._rtcm_message_publishers] == [
            Message,
            Message,
        ]
        assert [publisher.topic_name for publisher in node._rtcm_message_publishers] == [
            "/sensing/gnss/novatel/rtcm",
            "/rtcm",
        ]
        # rclpy owns node._publishers and appends the diagnostics publisher there.
        # A valid frame must use only our dedicated fields and never hit that list.
        assert node._diagnostics in node._publishers
        payload = bytes((0x43, 0x20, 1, 2))
        packet = bytes((0xD3, 0, len(payload))) + payload
        frame = packet + crc24q(packet).to_bytes(3, "big")
        node._handle_network_data(frame)
        assert node._parser.frames_valid == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


class FailingStream:
    """Raise an unexpected callback-style failure for worker recovery testing."""

    def __init__(self, _config):
        pass

    def stream(self, _stop_event, _on_data):
        raise TypeError("simulated publisher type mismatch")

    def close(self):
        pass


def test_worker_survives_unexpected_exception(tmp_path, monkeypatch):
    path = tmp_path / "ntrip_config.yaml"
    path.write_text(CONFIG.replace("enabled: true", "enabled: false"))
    os.chmod(path, 0o600)
    monkeypatch.setattr("pc3_ntrip_client.node.NtripStream", FailingStream)
    rclpy.init(args=["--ros-args", "-p", f"config_file:={path}"])
    node = NtripClientNode()
    try:
        deadline = time.monotonic() + 2.0
        while node._worker_errors == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert node._worker_errors >= 1
        assert node._worker.is_alive()
        assert node._last_error == "internal stream processing error"
    finally:
        node.destroy_node()
        rclpy.shutdown()
