"""ROS 2 node joining NTRIP, validated RTCM topics, diagnostics, and OEM7 serial."""

# HH_260811 - Fan validated RTCM frames out to NovAtel, /rtcm, and diagnostics.

import threading
import time
from typing import Dict, List, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rtcm_msgs.msg import Message

from .config import AppConfig, ConfigError, DEFAULT_CONFIG_FILE, load_config
from .ntrip import NtripError, NtripStream
from .rtcm import message_type, Rtcm3Parser
from .serial_sink import SerialError, SerialSink


def _values(items: Dict[str, object]) -> List[KeyValue]:
    return [KeyValue(key=key, value=str(value)) for key, value in items.items()]


class NtripClientNode(Node):
    """Bridge one private NTRIP stream to ROS diagnostics and GNSS receivers."""

    def __init__(self) -> None:
        super().__init__("pc3_ntrip_client")
        self.declare_parameter("config_file", DEFAULT_CONFIG_FILE)
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        try:
            self._config: AppConfig = load_config(config_file)
        except ConfigError as exc:
            # ConfigError messages contain field names/reasons only, never values or paths.
            raise RuntimeError(str(exc)) from exc

        # HH_260811 - Preserve the historical PC3 rtcm_msgs contract used for u-blox injection.
        rtcm_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self._rtcm_message_publishers = [
            self.create_publisher(Message, topic, rtcm_qos)
            for topic in self._config.ros.rtcm_topics
        ]
        self._diagnostics = self.create_publisher(
            DiagnosticArray, self._config.ros.diagnostics_topic, 10
        )
        self._parser = Rtcm3Parser()
        self._serial = (
            SerialSink(
                self._config.serial.port,
                self._config.serial.baud,
                self._config.serial.reopen_interval_sec,
                self._config.serial.configure_novatel_rtcm_input,
            )
            if self._config.serial.enabled
            else None
        )
        self._stop_event = threading.Event()
        self._state_lock = threading.Lock()
        self._active_stream: Optional[NtripStream] = None
        self._state = "starting"
        self._last_error = "none"
        self._reconnects = 0
        self._network_bytes = 0
        self._worker_errors = 0
        self._last_frame_time = 0.0
        self._last_message_type = 0
        self._serial_error_log_after = 0.0

        self._timer = self.create_timer(
            self._config.ros.diagnostic_period_sec, self._publish_diagnostics
        )
        self._worker = threading.Thread(target=self._run, name="ntrip-stream", daemon=False)
        self._worker.start()
        self.get_logger().info(
            "NTRIP client started; credentials are loaded only from the private config file"
        )

    def _set_state(self, state: str, error: str = "none") -> None:
        with self._state_lock:
            self._state = state
            self._last_error = error

    def _handle_network_data(self, data: bytes) -> None:
        with self._state_lock:
            self._state = "streaming"
            self._last_error = "none"
            self._network_bytes += len(data)
        for frame in self._parser.feed(data):
            now = self.get_clock().now()
            message = Message()
            message.header.stamp = now.to_msg()
            message.header.frame_id = self._config.ros.frame_id
            message.message = list(frame)
            for publisher in self._rtcm_message_publishers:
                publisher.publish(message)

            self._last_frame_time = time.monotonic()
            self._last_message_type = message_type(frame)
            if self._serial is not None:
                try:
                    self._serial.write(frame)
                except SerialError:
                    # Throttle and never include paths, config values, or exception causes.
                    current = time.monotonic()
                    if current >= self._serial_error_log_after:
                        self.get_logger().error("RTCM serial output is unavailable")
                        self._serial_error_log_after = current + 10.0

    def _run(self) -> None:
        delay = self._config.ntrip.reconnect_initial_sec
        while not self._stop_event.is_set():
            self._set_state("connecting")
            stream = NtripStream(self._config.ntrip)
            with self._state_lock:
                self._active_stream = stream
            try:
                stream.stream(self._stop_event, self._handle_network_data)
                delay = self._config.ntrip.reconnect_initial_sec
            except NtripError as exc:
                if self._stop_event.is_set():
                    break
                # NtripError is intentionally sanitized in ntrip.py.
                self._set_state("disconnected", str(exc))
                with self._state_lock:
                    self._reconnects += 1
                self.get_logger().warning("NTRIP stream unavailable; reconnect scheduled")
                if self._stop_event.wait(delay):
                    break
                delay = min(delay * 2.0, self._config.ntrip.reconnect_max_sec)
            except Exception:
                # Keep the correction worker alive on publisher/runtime defects while
                # ensuring exception values can never leak configuration or credentials.
                if self._stop_event.is_set():
                    break
                self._set_state("disconnected", "internal stream processing error")
                with self._state_lock:
                    self._reconnects += 1
                    self._worker_errors += 1
                self.get_logger().error(
                    "NTRIP worker recovered from an internal processing error"
                )
                if self._stop_event.wait(delay):
                    break
                delay = min(delay * 2.0, self._config.ntrip.reconnect_max_sec)
            finally:
                stream.close()
                with self._state_lock:
                    if self._active_stream is stream:
                        self._active_stream = None
        self._set_state("stopped")

    def _publish_diagnostics(self) -> None:
        with self._state_lock:
            state = self._state
            error = self._last_error
            reconnects = self._reconnects
            network_bytes = self._network_bytes
            worker_errors = self._worker_errors
        frame_age = (
            time.monotonic() - self._last_frame_time if self._last_frame_time else float("inf")
        )
        recent_limit = max(5.0, self._config.ros.diagnostic_period_sec * 3)
        if state == "streaming" and frame_age <= recent_limit:
            stream_level = DiagnosticStatus.OK
            stream_message = "valid RTCM3 streaming"
        elif state in ("starting", "connecting"):
            stream_level = DiagnosticStatus.WARN
            stream_message = "waiting for NTRIP stream"
        else:
            stream_level = DiagnosticStatus.ERROR
            stream_message = "NTRIP stream unavailable"

        statuses = [
            DiagnosticStatus(
                level=stream_level,
                name="pc3_ntrip_client: stream",
                message=stream_message,
                hardware_id="pc3_gnss_corrections",
                values=_values(
                    {
                        "state": state,
                        "last_error": error,
                        "network_bytes": network_bytes,
                        "valid_frames": self._parser.frames_valid,
                        "crc_errors": self._parser.crc_errors,
                        "discarded_bytes": self._parser.bytes_discarded,
                        "reconnects": reconnects,
                        "worker_errors": worker_errors,
                        "last_rtcm_type": self._last_message_type,
                    }
                ),
            )
        ]
        if self._serial is not None:
            serial_ok = self._serial.is_open and self._serial.write_errors == 0
            statuses.append(
                DiagnosticStatus(
                    level=DiagnosticStatus.OK if serial_ok else DiagnosticStatus.WARN,
                    name="pc3_ntrip_client: NovAtel RTCM serial output",
                    message="serial output active" if serial_ok else "serial output not active",
                    hardware_id="novatel_oem7_rtcm_port",
                    values=_values(
                        {
                            "open": self._serial.is_open,
                            "novatel_setup_enabled": (
                                self._serial.configure_novatel_rtcm_input
                            ),
                            "novatel_setup_attempts": self._serial.setup_attempts,
                            "novatel_setup_successes": self._serial.setup_successes,
                            "novatel_setup_ok": self._serial.setup_succeeded,
                            "bytes_written": self._serial.bytes_written,
                            "write_errors": self._serial.write_errors,
                        }
                    ),
                )
            )

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = statuses
        self._diagnostics.publish(array)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        with self._state_lock:
            stream = self._active_stream
        if stream is not None:
            stream.close()
        if hasattr(self, "_worker") and self._worker.is_alive():
            self._worker.join(timeout=self._config.ntrip.connect_timeout_sec + 1.0)
        if self._serial is not None:
            self._serial.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = NtripClientNode()
        rclpy.spin(node)
    except (RuntimeError, KeyboardInterrupt) as exc:
        if not isinstance(exc, KeyboardInterrupt):
            # RuntimeError text originates from sanitized configuration errors.
            print(f"pc3_ntrip_client: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
