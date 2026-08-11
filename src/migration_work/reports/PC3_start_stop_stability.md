# PC3 Start/Stop Stability Report

Audit date: 2026-08-10 KST
Tested workflow: exact `run_autoware` alias, stationary PC3, sensor hardware disabled, localization disabled, vehicle interface disabled, MRM disabled, and RViz disabled.

## Result

| Boundary | Result | Meaning |
|---|---|---|
| Stable hardware-disabled startup/shutdown | **PASS, 5/5** | Every accepted cycle reached the readiness barrier, stopped by Ctrl+C, returned launcher exit code 0, shut down the Hesai sensor container and gyro-bias estimator cleanly, produced no bad shutdown signature, and left no orphan process. |
| Pre-fix Hesai shutdown | **FAILED deterministically** | Destroying a joinable decoder `std::thread` invoked `std::terminate`, producing SIGABRT and container exit code `-6`. |
| 0.56-second startup interruption | **EXCLUDED** | Ctrl+C arrived while components and Python processes were still constructing/importing; it is not a valid post-start stability cycle. |
| `launch_hw=true` | **NOT ACCEPTED** | Physical LiDAR startup, callback teardown, UDP shutdown, data validity, and repeated hardware-on shutdown have not been tested. |

The PASS applies only to the exact hardware-disabled launch boundary. It is not evidence that live Hesai hardware, GNSS hardware, localization, or the four-PC graph is ready.

## Pre-fix failure and deterministic cause

The initial hardware-disabled test constructed the Hesai wrapper and explicitly entered packet-subscription mode:

```text
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:116
```

On Ctrl+C, the sensor container reported:

```text
terminate called without an active exception
SIGABRT
std::terminate()
std::_Sp_counted_ptr_inplace<>::_M_dispose()
rclcpp_components::ComponentManager::~ComponentManager()
exit code -6
```

at:

```text
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:228
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:231
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:239
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:240
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:242
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:252
```

The source-level cause is exact:

1. The pre-fix constructor starts `decoder_thread_` before branching on `launch_hw`.
2. The thread loops forever and waits in `packet_queue_.pop()`.
3. The queue has no close/stop state, so a no-packet hardware-disabled run leaves the worker blocked.
4. The pre-fix wrapper destructor is defaulted and neither stops nor joins the worker.
5. Destruction of the still-joinable `std::thread` therefore calls `std::terminate`.

Source evidence:

```text
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper (copy_org).cpp:72
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper (copy_org).cpp:78
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/include/nebula_ros/hesai/hesai_ros_wrapper (copy_org).hpp:56
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/include/nebula_ros/common/mt_queue.hpp:54
```

This failure is independent of receiving physical packets: the worker is created in both modes, while the failing run proves `launch_hw=false` was active. The periodic “Missed pointcloud output deadline” warning is expected when no packets arrive and was not the SIGABRT source.

## Lifecycle fix

The rebuilt wrapper now has an explicit shutdown protocol:

1. A null queue item is the decoder-thread stop sentinel.
2. The decoder loop returns when it receives that sentinel.
3. The destructor resets parameter/subscription and producer ownership.
4. It pushes the sentinel to unblock the queue.
5. It joins the worker if joinable before member destruction continues.

Active source evidence:

```text
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/include/nebula_ros/hesai/hesai_ros_wrapper.hpp:56
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp:72
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp:75
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp:102
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp:105
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp:109
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp:110
```

`nebula_ros` rebuilt successfully before the accepted cycles. The five-cycle result below is runtime confirmation that the sentinel/join lifecycle resolves the hardware-disabled SIGABRT.

## Five accepted post-fix cycles

An accepted cycle had to reach a stable readiness barrier before Ctrl+C: the sensor wrapper had loaded in hardware-disabled mode and the map/vector-TF path had completed. The external launch harness recorded exit code 0 and verified no bad signatures or orphan processes after each stop; the table cites the corresponding in-log evidence.

| Cycle | Launch log | Hardware disabled | Readiness (`map -> viewer`) | Ctrl+C | Gyro clean | Sensor container clean |
|---|---|---:|---:|---:|---:|---:|
| 1 | `/home/a/.ros/log/2026-08-10-19-56-09-947244-a-37623/launch.log` | `:113` | `:174` | `:176` | `:187` | `:220` |
| 2 | `/home/a/.ros/log/2026-08-10-20-00-04-431231-a-39099/launch.log` | `:116` | `:174` | `:175` | `:189` | `:208` |
| 3 | `/home/a/.ros/log/2026-08-10-20-00-16-066567-a-39672/launch.log` | `:116` | `:174` | `:175` | `:189` | `:207` |
| 4 | `/home/a/.ros/log/2026-08-10-20-00-58-921599-a-40260/launch.log` | `:112` | `:174` | `:175` | `:178` | `:219` |
| 5 | `/home/a/.ros/log/2026-08-10-20-01-09-770034-a-40803/launch.log` | `:111` | `:174` | `:175` | `:198` | `:218` |

Across the five accepted logs there is no `terminate called`, SIGABRT, exit code `-6`, or gyro exit code `-9`. Post-stop harness checks found no residual role-owned process. This satisfies the requested five-cycle stability gate for `launch_hw=false`.

## Excluded startup-interrupt run

The run below is deliberately excluded from the denominator:

```text
/home/a/.ros/log/2026-08-10-19-58-34-798961-a-38418/launch.log
```

The child processes began spawning at `:3` around timestamp `1786359518.156`; Ctrl+C arrived at `:86` around `1786359518.738`, only about 0.56 seconds later. At that instant:

- the Hesai component was still constructing and later failed because the ROS context was already invalid (`:87`, `:192`);
- system-monitor component loads were still pending (`:85`, `:88`);
- Python processes were interrupted during imports (`:90`, `:91`, `:119-164`);
- the gyro-bias estimator received SIGINT (`:154`) but was later escalated through SIGTERM and SIGKILL (`:194-199`).

The sensor container itself finished cleanly at `:92`. More importantly, all five later runs waited for stable readiness and both the gyro-bias estimator and sensor container then exited cleanly. The excluded run therefore demonstrates that the launcher is not guaranteed to tolerate an interruption during parallel construction; it does not justify a gyro source patch and does not invalidate the stable-shutdown result.

## Hardware-on boundary remains unaccepted

The sentinel/join change fixes wrapper-thread lifetime, but it does not establish safe live-device teardown. The current hardware interface still has these gaps:

```text
/home/a/autoware/src/sensor_component/external/nebula/nebula_hw_interfaces/src/nebula_hesai_hw_interfaces/hesai_hw_interface.cpp:33
/home/a/autoware/src/sensor_component/external/nebula/nebula_hw_interfaces/src/nebula_hesai_hw_interfaces/hesai_hw_interface.cpp:35
/home/a/autoware/src/sensor_component/external/nebula/nebula_hw_interfaces/src/nebula_hesai_hw_interfaces/hesai_hw_interface.cpp:208
/home/a/autoware/src/sensor_component/external/nebula/nebula_hw_interfaces/src/nebula_hesai_hw_interfaces/hesai_hw_interface.cpp:210
```

`HesaiHwInterface::~HesaiHwInterface()` only calls `FinalizeTcpDriver()`, and `SensorInterfaceStop()` is a stub returning `ERROR_1`. No accepted cycle exercised the UDP receive callback, a live sensor stream, callback quiescence, or hardware-on destruction order.

Do not set `launch_sensing_driver:=true` or otherwise cause `launch_hw=true` on the strength of this report. A separately authorized hardware-on test must first verify the current sensor endpoint and network isolation, then prove bounded callback shutdown, clean UDP/TCP teardown, launcher exit 0, and no orphan over five stable cycles.

## Operational readiness rule

- Accepted now: exact PC3 alias with the patched defaults and `launch_hw=false`, stopped only after stable readiness.
- Not accepted: immediate interruption during parallel startup as a required operating mode.
- Not accepted: `launch_hw=true`, manual `hesai_ros_driver/start.py`, simultaneous driver entry points, localization enablement, or four-PC uniqueness.
- If any later boundary produces SIGABRT, `std::terminate`, exit code `-6`, forced `-9`, a hung callback, or an orphan, stop that exact launch and return to the accepted hardware-disabled configuration.
