<!-- HH_260812 - Preserve the complete PC1 change, experiment, incident, and VILS review record. -->

# VILS PC1 work history

Status: **AUDITED WORK RECORD — not a driving approval**

Period covered: 2026-08-10 through 2026-08-12 KST

Applies to: IONIQ EV 308 PC1 planning/control, vehicle status/CAN bridge, DDS deployment,
three-PC integration, release preparation, and proposed four-PC VILS integration

## 1. Purpose and evidence rules

This file records what was requested and investigated on PC1, what was actually changed, what was
only tried and later restored, what was merely observed at runtime, why each action was taken, and
what remains unresolved. It exists so a later operator does not confuse an intermediate safety
experiment, a live diagnosis, or a documentation-only comment with the final vehicle configuration.

The following labels are used throughout:

- **PERSISTENT**: present in the current PC1 source or deployment configuration.
- **RESTORED**: temporarily changed during diagnosis, then returned to the historical PC1 behavior.
- **RUNTIME-ONLY**: command, relay, or test performed during one session with no permanent source
  change on PC1.
- **READ-ONLY FINDING**: measured or established from source/log inspection; no fix was applied.
- **EXTERNAL**: root cause or permanent owner is PC2, PC3, PC4, or the cross-PC deployment.
- **BLOCKER**: must be resolved before the corresponding real-vehicle stage is accepted.

The final four-file documentation set is:

1. `docs/vils/VILS_SHARED_ARCHITECTURE.md` — shared PC1/PC2/PC3/PC4 contract and opinions;
2. `docs/vils/VILS_PC1_WORK_HISTORY.md` — this PC1 record;
3. `docs/vils/VILS_PC2_WORK_HISTORY.md` — maintained by PC2;
4. `docs/vils/VILS_PC3_WORK_HISTORY.md` — maintained by PC3.

The shared contract is authoritative for the intended architecture. This worklog is authoritative
for the PC1 history and observed evidence.

The integration documents are developed on GitHub branch
`agent/pc4-digital-twin-integration` in Draft PR #8. At PC1 review start that branch contained only
the shared PC4 document. PC2 and PC3 then contributed their owner-authored records while PC1 review
was in progress. This change adds the final PC1 record and PC1 edits to the shared contract, so the
requested set now contains exactly the shared document plus three owner records.

## 2. PC1 environment and role

### 2.1 Host and middleware

| Item | Audited value |
|---|---|
| Host | `a` |
| OS | Ubuntu 22.04.5 LTS, x86-64 |
| Kernel at final audit | `6.8.0-136-generic` |
| Computer | Neousys Nuvo-8108GC |
| CPU | Intel Xeon E-2176G, 6 cores / 12 threads |
| RAM | 31 GiB |
| GPU | NVIDIA GeForce RTX 3070 8 GiB, driver 570.211.01 |
| ROS | ROS 2 Humble |
| System Python | 3.10.12 |
| Workspace | `/home/a/autoware` |
| Vehicle ROS domain | `ROS_DOMAIN_ID=10` |
| RMW | `rmw_cyclonedds_cpp` |
| Vehicle LAN NIC | `enp0s31f6`, `192.168.9.2/24` |
| Other wired NIC | `enp8s0`, `192.168.1.11/24` |
| Physical CAN | Kvaser USBcan Light 4xHS; `can0` and `can1` are set to 500 kbit/s by the active setup script |

The final shell aliases are direct historical commands:

```bash
alias run_autoware='ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/Downloads/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit'
alias run_planning_universe='ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/Downloads/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit'
alias run_bridge='ros2 launch ros2_socketcan can_brdige.launch.xml'
```

PC1 owns planning, control, Autoware API/RViz in the historical split, vehicle reports decoded from
CAN, and the physical CAN command path. PC2 owns perception. PC3 owns the real map, sensing,
localization, TF, and system/fail-safe foundation. PC4 is proposed as an isolated simulator and
virtual-actor source; it must not become a second PC1/PC3 authority.

The active PC1 top launch declares legacy `launch_vehicle`, `launch_map`, `launch_sensing`,
`launch_localization`, `launch_perception`, and `launch_system` arguments, but the historical PC1
file does not contain those component include groups. Its effective local groups are global
parameters, Planning, Control, API, and RViz. Setting the legacy arguments to `true` does not turn
PC1 into a monolithic PC2/PC3 stack.

### 2.2 Current publication references

The baseline and final PC1 snapshots were published as:

| Scope | Remote branch | Tag | Commit |
|---|---|---|---|
| Historical 307 Autoware baseline | `IONIQ_EV/PC1/autoware_universe/v1.0.0` | `IONIQ_EV_307_PC1_a` | `5b06fff6a713001f5bbb23aac2b34adb03e126f6` |
| Full Autoware workspace | `IONIQ_EV/PC1/autoware_universe/v2.0.0` | `IONIQ_EV_308_PC1_a` | `c8c6a0b795d91ccf9f9efe95e54084dd8c0481d8` |
| Independent PC1 ros2_ws payload | `IONIQ_EV/PC1/ros2_ws/v2.0.0` | `IONIQ_EV_308_PC1_r` | `8658adf6512fd3374ebcd2b4a3a0ba2656a0ce34` |

These tags preserve the audited workspace and evidence. They do not certify real-vehicle driving.

Publication provenance matters because the repository was reconstructed from a workspace whose
top-level Git metadata was empty. The historical PC1 v1.0 commit was fetched for byte-level
comparison; because the new snapshot was initialized independently, it is not a Git ancestor. The
first 308 Autoware snapshot was the parentless
root commit `2523b6620d4c6e657e06983e3d7c1e5340dd4611`, with 8,553 tracked files and 537,279,158
content bytes. The final documentation/incident commit `c8c6a0b...` has that root as its parent.
The internal evidence directory still says `PC1_v1.1` because it records the migration effort;
the final public branch names were later normalized to `IONIQ_EV/PC1/.../v2.0.0`. The independent
ros2_ws snapshot is `8658adf...`. Its local checkout retained an earlier long v1.1 branch label,
while the authoritative remote branch and tag in the table point to this same final commit.
Release-update PR #1 merged as `c8c6a0b...`; the current VILS integration remains Draft PR #8.

The full snapshot flattened the visible nested `autoware_tools` checkout as ordinary files. It
preserved the 625 files present at the time and did not restore 238 upstream-tracked files that
were already absent. This is a faithful PC1 worktree snapshot, not a complete upstream checkout.
The earlier pre-prepare inventory counted 8,546 intended files and 537,254,945 bytes; the committed
root later counted 8,553 tracked files and 537,279,158 bytes because release helpers/evidence were
added during preparation. The two largest files were 48,000,176-byte test-map PCDs, and no tracked
file exceeded GitHub's 100 MB hard limit.

## 3. Request and work chronology

### 3.0 Operator request trace

This is the normalized request sequence. It preserves the technical intent rather than chat
wording, and separates a request from the change that ultimately remained.

| Order | Operator request | PC1 response/disposition |
|---:|---|---|
| 1 | Run `~/scripts/start.sh`, bring up `run_bridge`, keep testing CAN input/output, and send the requested literal `a` into the interactive test rather than stopping at the prompt | Inspected and exercised the existing stack; later made CAN setup repeatable and fixed receiver shutdown |
| 2 | Keep the interactive test alive, retry inputs, and verify data from other PCs; PC3 uses Domain 10 | Audited processes, topics, DDS/NIC selection, peer data, and launch logs |
| 3 | Explain missing `control_cmd`/goal and make cruise/display plus zero longitudinal/lateral hold work | Traced the route/planning/control prerequisites and restored the full historical sender+adapter path after receive-only experiments |
| 4 | Bring the bridge down so the operator could restart it; then monitor remote data and local `run_autoware` for anomalies | Stopped treating endpoint discovery as proof and monitored fresh samples, rates, logs, MRM, TF, and process flaps |
| 5 | Diagnose missing goal/data, CAN write, and “MRM Stop” with all three PCs on | Found pointcloud contract, occupancy/planning cascade, duplicate nodes, PC3 MRM heartbeat loss, and downstream gate overrides |
| 6 | Prevent recurrence rather than only clear the current symptom | Identified durable PC2/PC3 owners and documented why relays/diagnostic bypasses were not permanent fixes |
| 7 | Explain why AUTO did not depart and what `start_planner` does by reading the code | Traced pull-out candidate retention, safe-path failure, zero-speed path, and hidden debug reasons; no bypass applied |
| 8 | Produce a detailed PC1 release, comments, v1.1 history, `_a`/`_r` tags, Git initialization, flattened workspace, push commands, and PR flow | Built/published full Autoware and independent ros2_ws snapshots, manifests, security redaction, changelogs, tags, and release update |
| 9 | Leave the system running and continuously find additional problems | Performed read-only live monitoring and recorded the 2026-08-12 localization, PC2, MRM, CAN, Conda, RViz, and time-source incident evidence |
| 10 | Consider raising the lateral-velocity stop criterion and continue measuring | Verified the exceeded threshold but did not relax it; kept DBC/signal validation as the first action |
| 11 | Make sure all latest changes reached the current branches/tags and clarify the v1.1 PR | Audited and published the final refs while keeping release status blocked for driving |
| 12 | Add PC4 virtual obstacles and decide whether PC2 physical obstacles should be removed for a controlled test | Designed hybrid, controlled virtual-only, AEB/pointcloud, source-selector, time/frame, bridge, and staged safety contracts |
| 13 | On the new PC4 branch, create one shared opinion file plus detailed per-PC histories, including all PC1 work/reasons/problems | Reviewed and corrected the shared document and created this exhaustive PC1 worklog; PC2/PC3 files remain owner-authored |
| 14 | Resume after loss of power and use a supplied credential once | Resumed from a separate worktree; did not use/store the exposed credential because an existing authenticated CLI session was available |

### 3.1 Initial physical bridge and cross-PC request — 2026-08-10

The operator first requested the existing operational sequence rather than a static source audit:

```bash
cd ~/scripts
./start.sh
run_bridge
```

The operator also asked that the interactive test continue beyond one input, that both bridge
directions be exercised, and that PC1 verify communication with the other hosts. PC3 was specified
as ROS Domain 10. The intended vehicle behavior was the existing one: pressing the cruise/control
button should make the display show cruise and let the longitudinal/lateral controller hold zero
until the later drive command.

An early safety interpretation treated the work as a stationary no-actuation validation. That led
to a sequence of fail-closed launch, vcan, receive-only bridge, process-lock, and guarded-alias
experiments. Those experiments were defensible as isolated tests but did not match the requested
operational vehicle sequence. Their effects and complete rollback are recorded in Section 5.

The live vehicle-LAN peers later verified from PC1 were PC3 `192.168.9.7` and PC2
`192.168.9.110`; the previously assumed `192.168.9.11` was not the active PC2 address in that
window.

### 3.2 CAN topic and goal-chain clarification

The operator corrected two important assumptions:

- vehicle-facing CAN results are not only raw `/from_can_bus` frames; the custom receiver decodes
  `/vehicle/status/steering_status`, `velocity_status`, `gear_status`, and `control_mode`;
- `control_cmd` is not expected before localization, a route/goal, trajectory generation, and the
  controller readiness chain are established.

Source inspection confirmed the customized receiver and command adapter. Relevant receive IDs are
`0x371`, `0x372`, `0x381`, `0x386`, `0x394`, and `0x4F1`. The command adapter emits standard CAN ID
`0x630` at approximately 33.3 Hz. This established that a raw-only or receive-only test could not
validate the operator's display/control sequence.

| CAN ID | Audited custom role |
|---|---|
| `0x371` | brake status decode |
| `0x372` | gear status decode |
| `0x381` | steering status decode |
| `0x386` | wheel/longitudinal-speed decode |
| `0x394` | acceleration decode and the branch's pseudo lateral-velocity path |
| `0x4F1` | cruise/cancel button and vehicle-control-mode state |
| `0x630` | PC1 acceleration, steering, control-request, and alive-counter transmit frame |

### 3.3 Domain 10 discovery diagnosis

With two wired interfaces available, CycloneDDS initially selected the wrong interface
(`192.168.1.11`) and PC1 saw no remote Domain 10 nodes. Forcing `enp0s31f6` exposed PC3 map,
sensing, and system endpoints and allowed actual map samples to be received. The modern CycloneDDS
interface form was selected:

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="enp0s31f6" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
```

This became a persistent PC1 deployment improvement. It fixed DDS interface selection only; it did
not guarantee that remote applications were alive or publishing fresh samples.

### 3.4 Bridge shutdown failure and receiver fix

Repeated bridge logs showed the CAN receiver exiting with code `-6` on Ctrl+C. The receive worker
was created during lifecycle configure, but a joinable `std::thread` could reach destruction
without a deterministic stop/join path. A cleanup-time unconditional join could also wait forever
while the ROS context remained alive and the inactive worker kept sleeping.

The receiver was changed to use an atomic worker-running flag and one stop/join helper from the
destructor, cleanup, and shutdown callbacks. Classic CAN and CAN-FD loops now honor both ROS shutdown
and that lifecycle flag. `SocketCanTimeout` is treated as normal idle polling instead of a warning.
The worker is joined before publisher reset to reduce a publish/reset race.

This is the principal persistent functional source change made during this work.

The first recorded abort is in
`/home/a/.ros/log/2026-08-10-20-55-25-165240-a-32479/launch.log`. Later Release-build evidence
includes `/home/a/autoware/log/build_2026-08-11_19-05-42` and the final evidence-comment rebuild
`/home/a/autoware/log/build_2026-08-12_14-51-08`.

### 3.5 Planning-simulator detour and return to real `run_autoware`

An isolated Domain 110 planning-simulator experiment was investigated to prove map, route,
trajectory, and internal controller flow without physical CAN. Temporary launch options disabled
engagement and RViz and attempted to make map/control explicit. A missing simulator pointcloud
container path was also diagnosed.

The operator clarified that the target was the real distributed `run_autoware` stack, not the
planning simulator. The simulator changes were therefore restored to the v1.0 file and were not
carried into the final release.

### 3.6 Restoration of the historical PC1 operating profile — 2026-08-11

The operator reported that cruise/display and longitudinal/lateral hold no longer worked and that
the guarded aliases made the normal sequence unnecessarily complex. The direct causes were the
receive-only bridge and temporary wrappers, not a newly discovered vehicle CAN decode failure.

The following historical behavior was restored:

- direct `run_autoware`, `run_planning_universe`, and `run_bridge` aliases;
- original `autoware.launch.xml` defaults and Planning/Control/API/RViz profile;
- original `planning_simulator.launch.xml`;
- full bridge receiver + sender + `twistController2VCU2EPS2ACC_node`;
- original global CAN topics and physical `can0` path;
- removal of the active flock/process guard and safe-wrapper dependency.

After the full bridge returned, the operator reported that cruise/display input finally appeared.
No cruise decoder change is credited for that recovery: the final `0x4F1` decode was restored to
the pre-experiment source.

### 3.7 Three-PC readiness and planning-chain diagnosis

Once PC1, PC2, and PC3 were running, PC1 monitoring established several different blockers over
time:

1. PC3 localization, TF, and system endpoints disappeared and later returned even while basic LAN
   reachability remained available; map samples were independently observed in healthy windows.
2. PC2 perception nodes could exist while their pipeline produced no samples.
3. PC2 expected `/sensing/lidar/top/pointcloud_before_sync`, while PC3's active output was
   `/sensing/lidar/concatenated/pointcloud`.
4. Missing pointcloud samples caused no obstacle pointcloud and no occupancy grid, so the behavior
   path planner remained at `waiting for occupancy_grid`; trajectory and control were consequently
   absent.
5. A temporary runtime compatibility relay made `before_sync`, crop, single-frame, occupancy-grid,
   obstacle, trajectory, and control streams recover. That relay was proof of the topic-contract
   mismatch, not a PC1 permanent fix.
6. Duplicate `/pointcloud_container/glog_component` and repeated `/rviz2` FQNs were observed in the
   shared Domain 10 graph.

The first simultaneous three-PC snapshot was only a transport/availability milestone. PC3 maps,
GNSS, IMU, LiDAR, and PC1 vehicle status arrived, while the official PC2 predicted-object sample
was an empty array. Localization initially had no kinematic/acceleration publisher or connected
`map -> base_link`, route state was unset/unknown, and bounded trajectory/control checks had no
samples. One API goal attempt returned `status=false`, code `1`, and `route already set` despite
that inconsistent route state; it was not treated as a valid route. After localization appeared,
a nominally stationary 9.982-second window moved 1.539 m in
3D while decoded longitudinal speed remained 0.0 m/s. The run therefore proved connectivity but
not localization stability or perception quality.

PC3 MRM handler and operator nodes also disappeared or were loaded into the wrong component
container. When `/system/fail_safe/mrm_state` was missing longer than the PC1 command gate timeout,
PC1's vehicle command gate replaced a positive follower command with the configured emergency
acceleration, observed as `-2.4 m/s²`. This configured fail-safe override was observed in response
to the missing heartbeat; it is not by itself an end-to-end safety approval. The permanent owner of
the missing MRM stack is PC3.

### 3.8 Start planner investigation

`start_planner` was inspected because it remained visible and the vehicle did not start. Source
analysis established:

- it is the pull-out module used to move from a shoulder/road edge into the main lane;
- its presence in the plugin/RTC list is normal and does not itself mean vehicle operation mode;
- once retained as a candidate, a `WAITING_APPROVAL` module can stay in the manager;
- when no safe shift/geometric/freespace pull-out path is found, it publishes a two-point zero-speed
  stop path and remains safe=false rather than transitioning to failure;
- the live state showed `Not found safe pull out path, publish stop path` and zero velocity points.

No start-planner source or parameter was changed. The proposed next diagnostic was to enable only
its debug reason output temporarily, collect the per-candidate failure reason, then restore it.
The audit also found that several freespace/A* values present in the launcher YAML were not loaded
by this branch's manager code; some corresponding scalar fields lacked explicit defaults. This is
an unconfirmed fallback-planner defect candidate, not the proven reason that shift/geometric paths
failed, and it was not modified during this work.

### 3.9 Release documentation, repository preparation, and publication

The operator requested a complete, reviewable v1.1 record and later chose a full current-workspace
snapshot rather than a small curated patch. Work performed included:

- separate final changes from reverted experiments and unresolved findings;
- flatten nested Git metadata so no mode-160000 gitlinks or `.gitmodules` remained;
- exclude generated top-level `build/`, `install/`, and `log/` from publication;
- preserve the visible source tree, reports, backups, and package-local historical files as the
  requested snapshot while marking them as unverified where appropriate;
- redact two physical MAC addresses and two NetworkManager UUIDs from a public report;
- scan for high-confidence credentials and GitHub hard-limit files;
- construct an independent PC1 ros2_ws payload containing ros2_socketcan, CAN setup, DDS config,
  release evidence, and checksums;
- publish Autoware and ros2_ws branch/tag pairs and document rollback.

The snapshot policy preserves evidence; it does not make every included file an approved change.

### 3.10 Real three-PC incident observation — 2026-08-12

The vehicle stack was monitored read-only while the operator and other PCs performed the test.
Codex did not send a route, engage command, operation-mode service request, control message, or CAN
frame. The observed sequence was:

| Approximate KST | Observation | Meaning |
|---|---|---|
| 11:50 | ADAPI web server exited because Conda Python 3.13 resolved a Humble Python 3.10 `rclpy` extension; `/rviz2` appeared three times | PC1 environment and duplicate-FQN failure |
| 11:54–12:59 | Raw GNSS/IMU/LiDAR and vehicle reports were live, but localization stayed uninitialized | Downstream planning could not be ready |
| before 13:00 | Stationary CAN-derived lateral velocity was about `0.00110999995 m/s`, above the pose initializer's `0.001 m/s` stop criterion for 3 s | This independently verified one deterministic rejection condition; it does not exclude concurrent localization faults |
| about 13:00 | Localization initialized; `map -> base_link` and kinematic state recovered around 40–50 Hz | Temporary recovery |
| about 13:01:52 | Localization regressed to initializing for roughly 12–13 s, then recovered | Continuity failure |
| about 13:02:23 | Route, trajectory, controller, AUTO/DRIVE, and physical CAN control request became active | End-to-end actuation path was live; not a passive test |
| about 13:04:49 | PC2 perception stack and objects disappeared, then returned | Remote perception/DDS/process dropout |
| about 13:05:32 | NDT/trajectory translation and rotation deviation faults coincided with the transition to MRM emergency stop while reported speed reached about `2.47 m/s` | Critical same-window correlation; event attribution requires synchronized PC2/PC3/PC1 logs |
| about 13:06:32 | Speed returned to zero and PARK | Vehicle stopped, but release remained blocked |

During the same run, the command adapter was proven to retransmit its cached last acceleration and
steering every 30 ms even when `/control/command/control_cmd` was no longer fresh. A cached
`-2.4 m/s²`, zero-steering command remained encoded in CAN ID `0x630` at about 33.3 Hz. The control
request byte was zero while cruise was false, but enabling cruise could attach the request bit to a
stale command. This is an unresolved PC1 blocker, not a completed watchdog fix.

This live stale-command case is separate from the adapter's startup defaults: before any valid
`control_cmd`, source initialization can already transmit the cached default acceleration
`-1.0 m/s²`, steering zero, and control request false on the same 30 ms timer. Both cases require a
freshness/arming contract; neither is made safe merely by leaving the request bit false at startup.

Chrony also selected a public NTP server instead of the expected PC3 vehicle-LAN source during this
run. Consequently, the run is incident evidence, not acceptable latency/timestamp paper data.

### 3.11 Four-PC VILS design request

The operator then asked how to use PC4 virtual obstacles with the real vehicle and whether PC2 real
obstacles should be removed for a controlled experiment. PC1 source review concluded:

- hybrid VILS should keep PC2 as the sole canonical final object owner and add validated PC4 actors
  before PC2 map-based prediction;
- a controlled virtual-only profile may replace the PC2 object source, but the selection must be
  explicit and mutually exclusive;
- two publishers on `/perception/object_recognition/objects` do not merge arrays; depth-one
  subscribers receive alternating nondeterministic scenes;
- PC1 also consumes obstacle pointcloud and occupancy-grid inputs, so replacing PredictedObjects
  alone does not remove every physical-obstacle influence;
- current AEB uses pointcloud and does not use predicted objects, so a validated PC4 object accepted
  into the canonical stream would be eligible for selected object-based planners but not the
  default AEB path;
- retaining the real pointcloud as an independent AEB safety overlay is preferable for the first
  controlled real-vehicle stages, while it prevents a claim of mathematically pure virtual-only
  causality.

No VILS runtime source or bridge was implemented on PC1 in this phase. The shared architecture
document contains the proposed contract and staged gates.

### 3.12 Resume after the PC1 power interruption

Work resumed on 2026-08-12 after the host lost power. The post-restart documentation snapshot had
both wired NICs up at their recorded addresses, `can0` through `can3` down, and no local Autoware,
RViz, SocketCAN sender/receiver, or command-adapter process. Chrony was synchronized but still
selected public source `211.108.117.211`; PC3 `192.168.9.7` was reachable only as a non-selected
candidate. Therefore no live driving result was inferred from the post-restart host.

The shared VILS branch was edited in the separate worktree
`/home/a/autoware-vils-pc1-review` so the active published PC1 Autoware branch and its clean source
tree were not checked out or altered. The credential pasted during the resume request was not used
or stored; the already configured GitHub CLI login was used for repository access.

## 4. Persistent final changes

### 4.1 SocketCAN receiver lifecycle and idle handling

| File | Final behavior |
|---|---|
| `ros2_socketcan/include/ros2_socketcan/socket_can_receiver_node.hpp` | Adds `<atomic>`, destructor, shared stop helper, and `receiver_running_` |
| `ros2_socketcan/src/socket_can_receiver_node.cpp` | Starts flag before worker creation; stops/joins on destructor, cleanup, and shutdown; joins before publisher reset; applies flag to CAN and CAN-FD loops; handles `SocketCanTimeout` as normal idle polling |

Reason: prevent Ctrl+C `std::terminate`/exit `-6`, lifecycle cleanup deadlock, and idle warning spam.

Observed result: most subsequent receiver launches finished cleanly. Historical aggregation found
23 clean finishes after the first died instance, one SIGTERM escalation, and one log without a
terminal record. Therefore the fix is improved and build-validated, but not claimed as a perfect
long-duration shutdown acceptance.

### 4.2 Repeatable physical CAN setup

The active `/home/a/scripts/start.sh` and repository payload version perform:

```text
can0 down -> bitrate 500000 -> can0 up
can1 down -> bitrate 500000 -> can1 up
```

They use a Bash shebang, strict error handling, `nullglob`, and existence checks so missing tty,
video, or Docker socket classes do not abort the setup. The historical broad `chmod 777` policy was
retained and remains security debt; this was not redesigned.

### 4.3 CycloneDDS vehicle-LAN binding

`migration_work/config/cyclonedds_pc1.xml` pins CycloneDDS to `enp0s31f6`. `.bashrc` exports Domain
10, CycloneDDS RMW, and the URI. NetworkManager was also aligned to the intended NIC/IP topology.

The live home `.bashrc` and NetworkManager database are deployment state, not portable source.
Only the minimal `pc1_ros_env.sh`, CycloneDDS XML, and documented NIC contract were published for
reproduction.

Reason: without explicit selection, CycloneDDS selected the other NIC and remote PC2/PC3 discovery
was empty even though the Domain ID was correct.

### 4.4 Evidence comments and release documentation

The 2026-08-11 note beside the dual raw gear mappings and the 2026-08-12 notes beside CAN `0x394`
and the 30 ms adapter timer are comments only. They record pre-existing unverified mappings, the
lateral stop-gate, and stale-command findings. They do not change the gear mapping, signal, safety
threshold, timer, payload, or control behavior.

Likewise, `tier4_localization_component.launch.xml`, the global vehicle-stop criterion,
start-planner source/config, and AEB behavior were not modified. They were read to establish causes
and dependencies; diagnoses must not be listed as functional improvements.

The release documentation under `migration_work/releases/PC1_v1.1/` records code ownership,
validation, known issues, rollback, checksums, and publication steps.

### 4.5 Final file identity for critical paths

The paths below are relative to `/home/a/autoware/src` unless labeled as ros2_ws payload. These
hashes identify files in the published PC1 snapshots; the source files themselves are not copied
into this VILS architecture branch.

| File | SHA-256 | Note |
|---|---|---|
| `sensor_component/ros2_socketcan/ros2_socketcan/include/ros2_socketcan/socket_can_receiver_node.hpp` | `a5ce2266d381f4d34ff482e75b4eaeae5989d5e17cd85fcc485b739b46aadff8` | lifecycle fix |
| `sensor_component/ros2_socketcan/ros2_socketcan/src/socket_can_receiver_node.cpp` | `410f16f9116923cebe3cd1ee209ac7c04444784cac0982370c8f1ee57f166c8b` | lifecycle fix + evidence comment |
| `sensor_component/ros2_socketcan/ros2_socketcan/src/twistController2VCU2EPS2ACC_node.cpp` | `e762af9cccad3e3dc32f3a5f78d311d7e5d4721690d28b80908163878c3f695b` | behavior unchanged; watchdog warning comments |
| `migration_work/config/cyclonedds_pc1.xml` | `0e4f30b76e868f5477dc8252736d66502466efbd5fd6bb7d1cde8cddbd49f880` | DDS NIC pin |
| ros2_ws payload `config/cyclonedds_pc1.xml` | `5e5638be5de778f53a25fb26c580b2bba1844729b672321dca86b448c97f74f3` | portable-path DDS copy |
| `/home/a/scripts/start.sh` and ros2_ws payload `scripts/start.sh` | `a7ed55451104e4b162f01a8e8cf1a2e254241cdadc478fe684836dd43d214b62` | active and payload copies are identical |
| `launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml` | `b9129744fc216fd3d79918aa8c425e1299a51283c31cf7ccecdfb7753cb7f774` | restored historical file |
| `launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml` | `a82e5f26ec0e3cf62be53dcfa3f05a2d9357c26a3e285fca62d069311a9684a8` | restored historical file |
| `sensor_component/ros2_socketcan/ros2_socketcan/launch/can_brdige.launch.xml` | `c577b7eff1f5817cbec0e4ebefb327208dbfaee93e9d78e1c12cfcaf65222242` | restored full bridge |
| `sensor_component/ros2_socketcan/ros2_socketcan/launch/socket_can_bridge.launch.xml` | `b52011c74897e2528911b4642adc22c55083c4f702c44d86b3e40407750a5eeb` | receiver/sender child launch; historical `interface1` issue remains |
| ros2_ws payload `config/pc1_ros_env.sh` | `f149431d5fc1080496a58692c236822efb31e5731bc3f9eafff8fc42bd055f4e` | portable Domain/RMW/DDS/alias environment |
| `launcher/autoware_launch/autoware_launch/config/control/autoware_autonomous_emergency_braking/autonomous_emergency_braking.param.yaml` | `3cec365fc36e5b687f4cb0c50e66ca9edd3d54b032f4d68e7fc2ff4da0191a02` | unchanged audited baseline: pointcloud on, predicted objects off |

## 5. Tried and restored changes

None of the following is the active production profile.

| Experiment | Reason attempted | Problem produced or discovered | Final disposition |
|---|---|---|---|
| Fail-closed top launch | Minimize actuation during stationary audit | Removed Control/RViz/parking/API adaptors needed for the requested distributed test | Restored `autoware.launch.xml` and direct alias |
| Guarded `run_autoware` | Refuse launch in unexpected CAN/process state | Alias once failed with `/home/a/autoware/src/migration_work/scripts/run_autoware_safe.sh: No such file or directory`; cached shells retained obsolete aliases | Direct historical alias restored; compatibility helper is not active |
| vcan0 bridge | Test decode and ROS↔CAN without physical bus | Could not validate actual cruise display or zero hold; namespaced raw topics differed from historical global contract | vcan/remaps/wrapper removed from active path |
| Physical receive-only bridge | Observe real CAN without writing | Removed `0x630`, so cruise display and longitudinal/lateral hold did not engage | Full sender + adapter bridge restored |
| flock/process guard | Prevent duplicate bridge | Existing lock caused `another run_bridge instance holds ...` and complicated operator flow | Active lock wrapper removed |
| Cruise debounce change | Investigate button behavior | Was not the actual cause; final source equals pre-experiment snapshot | Fully restored |
| Domain 110 planning simulator | Isolated route/controller smoke test | Diverged from real three-PC architecture and hit simulator container/map issues | Launch restored; evidence kept only |
| Fail-closed five-cycle launch test | Measure isolated start/stop | 47-node graph did not represent final full profile | Kept as historical evidence only |

Important operational lesson: a receive-only/shadow profile and the full physical bridge must have
different explicit launch names. Quietly changing the implementation behind `run_bridge` made a
safe experiment look like a vehicle failure.

The five isolated fail-closed cycles each had 47 nodes, zero duplicate FQNs, zero forbidden
control/CAN nodes, and clean Ctrl+C in approximately 0.45 seconds. Their recorded RSS values were
570932, 570468, 569676, 574268, and 571208 KiB. Every cycle started and ended with no residual
process/node/lock and with `can0` through `can3` down. These are lifecycle measurements of a
reverted profile, not acceptance evidence for the restored full bridge or the four-PC system.

## 6. Read-only findings and unresolved problems

### 6.1 PC1 blockers

1. **No command freshness watchdog.** The adapter republishes cached acceleration/steering every
   30 ms. Required fix: timestamp the last valid `control_cmd`, clear the control request and emit a
   tested safe fallback on timeout, then prove this on bench and physical CAN.
2. **Global `/to_can_bus` authority.** The sender accepts the global topic without a PC1 namespace,
   publisher identity policy, CAN-ID allowlist, or source freshness contract.
3. **CAN `0x394`/lateral velocity semantics.** A derived lateral velocity around 0.00111 m/s while
   stationary blocked localization initialization. The DBC indices, units, integration semantics,
   and stop-check input must be validated before changing the 0.001 m/s safety criterion.
4. **Cruise/ACC state logic.** The ACC branch updates `previous_cancel_pressed` rather than
   `previous_acc_pressed`; exact-byte button matching may also fail when unrelated bits are set.
5. **Launch argument typo/default dependency.** The parent bridge historically passes
   `interface1`, while both receiver and sender child launches declare `interface`; operation
   relies on their default `can0`.
6. **Vehicle geometry.** The launch still uses `sample_vehicle`; real IONIQ EV geometry and control
   calibration have not received final acceptance.
7. **ADAPI environment contamination.** `#!/usr/bin/env python3` selected Conda Python 3.13 and
   failed to load Humble's Python 3.10 `rclpy` extension. ROS launch environments must exclude the
   incompatible Conda prefix/PATH.
8. **Duplicate RViz FQN.** Multiple PCs launch `/rviz2` with the same name. Use one designated RViz
   owner or host-unique names.
9. **Pre-existing unverified parameters.** `max_vel: 36.0`, the local RViz configuration, dual raw
   gear mappings, and backup/copy files were preserved but not newly validated by this work.
10. **Start-planner observability/config mapping.** Detailed path-attempt reasons are disabled by
    default, and several freespace/A* YAML values appear unmapped in this source branch. Capture the
    debug reason table and initialize/map every planner parameter before attributing a no-path
    result to vehicle geometry or obstacles.

### 6.2 External and cross-PC blockers

1. PC2 perception/DDS stack disappeared during active AUTO/DRIVE and later recovered.
2. PC2's historical pointcloud input did not match PC3's active concatenated pointcloud topic.
3. PC3 localization/NDT dropped and jumped, causing trajectory deviation and MRM emergency stop.
4. PC3 MRM handler/operators/heartbeat disappeared or loaded into an unintended component
   container during earlier runs.
5. Chrony did not consistently select the approved common vehicle time source.
6. Duplicate component helper FQNs and TF/node ownership require a cross-PC naming policy.
7. During the audited window PC1 had no approved remote-management path to PC2/PC3, so a durable
   remote launch fix was not applied from PC1; each owner must commit and validate its local change.

### 6.3 Diagnoses that were not source changes

- No localization safety threshold was raised.
- No start-planner parameter or plugin code was changed.
- No AEB predicted-object mode was enabled.
- No MRM timeout or availability rule was bypassed.
- No permanent PC2 pointcloud relay was committed on PC1.
- No PC4 object bridge, fuser, selector, actor adapter, or command path was implemented.
- No CARLA `/clock`, TF, localization, vehicle status, or control topic was authorized into Domain
  10.

### 6.4 Complete carried issue register

This table carries all 26 IDs from the PC1 release audit so that the VILS record does not silently
drop an unresolved item. `INCLUDED_UNVERIFIED` means preserved in the snapshot, not approved.

| ID | Status | Issue and impact | Required owner/action |
|---|---|---|---|
| PC1-001 | BLOCKER | Global `/to_can_bus` has no publisher ownership or CAN-ID allowlist; another Domain 10 writer can reach physical CAN | PC1 ros2_socketcan: namespace/ownership/ID allowlist |
| PC1-002 | BLOCKER | Converter has no command-age watchdog and can repeat cached accel/steer | PC1: timeout, request-bit clear, safe fallback, bench/CAN tests |
| PC1-003 | INCLUDED_UNVERIFIED | Planning `max_vel: 36.0 m/s` may exceed the approved vehicle/course envelope | Planning/vehicle owner approval or replacement; never use as Stage 5 limit |
| PC1-004 | BLOCKER | `sample_vehicle` geometry may not match IONIQ EV | Vehicle owner: calibrated geometry/limits package |
| PC1-005 | EXTERNAL | PC3 MRM nodes/heartbeat disappeared; PC1 gate applied `-2.4 m/s²` emergency override | PC3: durable handler/operators and heartbeat supervision |
| PC1-006 | OPEN | start_planner found no safe pull-out and emitted a two-point zero-speed path | Planning owner: capture reason table, verify route/pose/lane/geometry |
| PC1-007 | EXTERNAL | PC2 expected `pointcloud_before_sync` while PC3 published concatenated pointcloud | PC2/PC3: one permanent input contract, no ad-hoc ownership ambiguity |
| PC1-008 | OPEN | Parent launch passes `interface1`; receiver/sender children declare `interface` | PC1: correct argument and test non-default interface |
| PC1-009 | OPEN | Cruise `0x4F1` exact-byte toggle can miss frames with unrelated bits | Vehicle/CAN owner: bitmask/state-machine trace test |
| PC1-010 | OPEN | ACC edge branch updates the cancel-history variable | PC1: correct and unit-test ACC/cancel edges |
| PC1-011 | OPEN | Dual `0x372` gear mapping has no attached raw-frame proof | Vehicle/CAN owner: lever/raw-frame truth table |
| PC1-012 | OPEN | `0x386` velocity comment and `/3.6` conversion disagree on units | Vehicle/CAN owner: DBC and independent speed comparison |
| PC1-013 | OPEN | `0x394` indices/units and pseudo lateral velocity are unverified | Vehicle/CAN owner: DBC vectors and stationary/motion measurement |
| PC1-014 | OPEN | Conda Python 3.13 broke Humble Python 3.10 ADAPI `rclpy` loading | PC1 deployment: isolate ROS PATH/Python |
| PC1-015 | OPEN | `/rviz2` appeared three times with the same FQN | Cross-PC launch owner: one RViz or host-unique names |
| PC1-016 | OPEN | Legacy setup retains broad `chmod 777` | PC1 operations: udev/groups/minimum permissions |
| PC1-017 | OPEN | Backup/pycache files in the full snapshot can expose old launch files in install/share | Release owner: active/archive policy and clean install audit |
| PC1-018 | OPEN | Full ros2_socketcan test suite still has legacy lint/style failures and disabled tests | Package owner: clean complete test gate |
| PC1-019 | OPEN | PC4 bridge, source contract, timestamp, and duplicate-publisher behavior are unimplemented | PC4/integration owners: allowlisted bridge and runtime proof |
| PC1-020 | INCLUDED_UNVERIFIED | Local RViz differences and missing/linked YOLOX launch state were preserved | File owners: explicit per-file validation |
| PC1-021 | OPEN | Full snapshot is about 537 MB/8,553 tracked files and includes historical evidence | Release owner: hard-limit gate now; source/history separation later |
| PC1-022 | OPEN | Nested `autoware_tools` was flattened with 625 present and 238 already-deleted files | Vendor owner: verified import if full upstream completeness is required |
| PC1-023 | BLOCKER | Stationary lateral velocity about 0.00111 m/s exceeded the 0.001 m/s pose-init gate | PC1/PC3: validate signal semantics before changing threshold |
| PC1-024 | EXTERNAL | PC2 perception/DDS stack repeatedly disappeared, including during AUTO/DRIVE | PC2: process/network/DDS supervision and long-window recovery proof |
| PC1-025 | EXTERNAL | PC3 NDT jump/reinitialization coincided with trajectory deviation and MRM event | PC3: map/TF/calibration/timing and synchronized reproduction |
| PC1-026 | OPEN | Chrony selected public NTP rather than the proposed common vehicle source | All PCs: one selected source and recorded offset/jitter gate |

## 7. Validation evidence

### 7.1 Build and static checks

- `ros2_socketcan` Release builds succeeded on 2026-08-10 and 2026-08-11, with final evidence-comment
  rebuild on 2026-08-12.
- Enabled functional CTest target `ros2_socketcan_test` passed 1/1; three enabled assertions passed.
- Seven CAN tests are disabled and therefore are not coverage evidence.
- The full package test suite still reports legacy copyright, cpplint, flake8, lint_cmake, pep257,
  and uncrustify failures. “All tests pass” must not be claimed.
- Launch/DDS XML, shell syntax, checksum manifests, nested Git/gitlink checks, and public-secret scan
  were performed for release preparation.
- At the final documentation audit, `can0` through `can3` were down and no claim was made that the
  physical bridge was safe to arm merely because its source had been published.

Representative build/test commands were:

```bash
cd /home/a/autoware
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select ros2_socketcan \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select ros2_socketcan
colcon test-result --test-result-base build/ros2_socketcan --verbose
```

### 7.2 Runtime checks

Runtime monitoring included node/publisher ownership, QoS and actual sample checks, map/TF,
localization, perception, route/trajectory/controller rates, diagnostic graph/hazard/MRM state,
vehicle reports, CAN interface statistics, process ancestry, launch logs, network routes, peer
reachability, and Chrony source/offset.

Key positive evidence:

- fixed DDS NIC allowed actual PC3 vector and pointcloud map samples;
- later three-PC windows delivered GNSS, IMU, LiDAR, localization, and PC2 predicted objects;
- the pointcloud contract diagnosis correctly predicted the downstream occupancy/planning recovery;
- the full planning/controller/CAN path became active during the operator's test;
- receiver lifecycle changes greatly reduced exit `-6` behavior.

Key negative evidence:

- remote endpoints could remain discoverable without fresh samples;
- PC2 whole-stack dropout and PC3 localization/TF/system endpoint flaps occurred;
- localization initialized, regressed, and recovered;
- cached CAN retransmission continued without fresh control input;
- MRM emergency stop occurred during real motion;
- the time source was not deterministic.

## 8. PC1 VILS owner review

### 8.1 Accepted architecture

PC1 accepts the shared architecture under these constraints:

- PC1 remains the only planning/control and physical-CAN authority.
- PC3 remains the only real localization and `map -> base_link` authority.
- In every armed injection mode, PC2 remains the only canonical final
  `/perception/object_recognition/objects` publisher.
- PC4 runs in a separate simulator domain and can send only validated, namespaced virtual actors
  and diagnostics through a deny-by-default bridge.
- PC4 cannot send `/clock`, TF, localization, vehicle status, planning, control, system commands,
  services, actions, parameters, or CAN topics into the vehicle domain.

### 8.2 Current PC1 object and AEB behavior

PC1 consumes `autoware_perception_msgs/msg/PredictedObjects` on
`/perception/object_recognition/objects`. The monitored contract is reliable, volatile, depth one,
and the measured PC2 transport window was about 9.09 Hz. That sample contained an empty object array,
so it verifies transport/rate only, not detection quality.

The current AEB configuration is different:

```yaml
use_pointcloud_data: true
use_predicted_object_data: false
aeb_hz: 10.0
```

Therefore PC4 virtual objects, if validated and accepted into the canonical stream, would be
eligible to influence selected object-based planning modules but do not automatically become AEB
obstacles. A claim that “PC4 obstacle was detected by AEB” is false unless AEB is deliberately
redesigned and independently validated.

### 8.3 Hybrid versus controlled virtual-only mode

For hybrid VILS, use PC2 physical-main fail-open fusion, physical priority for associated objects,
a TTL/session gate for PC4, one PC2 predictor, and one canonical publisher.

For a controlled virtual-only experiment, do not delete the PC2 source. Use the shared contract's
`real_only`, `shadow`, `hybrid_optional`, `hybrid_required`, and
`virtual_only_required` state machine. If PC4 supplies stable `TrackedObjects`, select at the
tracking boundary before PC2 prediction; if it supplies `DetectedObjects`, select before the PC2
tracker. Preserve physical objects under a shadow topic. PC2 map-based prediction remains the sole
canonical publisher and PC4 never publishes the canonical topic directly. In
`virtual_only_required`, accepted PC4 snapshots drive the selected path; missing or stale PC4 data
is a fault and never triggers automatic physical-source fallback.

Physical pointcloud and occupancy-grid paths must be named in the test definition:

- retaining real pointcloud for AEB is the safer first real-vehicle overlay, but the experiment is
  not purely virtual-only;
- removing all real obstacle inputs also removes an independent physical safety channel and is
  limited to stationary/no-actuation until a separate AEB/sensor-in-the-loop safety design is
  approved. Low speed alone does not make removal of that physical safety channel acceptable.

### 8.4 PC4-required loss behavior is not yet implemented

Current PC1 does not have a proven PC4-session/TTL availability input wired into operation-mode
availability and MRM. A missing virtual-object stream must not be represented by repeatedly cached
objects or silently treated as a clear road in a required scenario.

Before a PC4-required moving test, implement and verify:

1. PC4 session/sequence/health diagnostics;
2. stale/future/replay/frame/map-hash rejection;
3. an explicit armed state that resets to shadow on restart or writer/session change;
4. engagement inhibition when required PC4 data is unavailable;
5. a controlled-stop/MRM request that is separately implemented and validated before any moving
   required-source test;
6. stationary manual re-arm after recovery.

### 8.5 Shadow testing must truly have no physical command sink

The current historical `run_bridge` starts receiver, sender, and command adapter together. It is
actuation-capable and cannot be described as shadow mode. Until a separately named receive-only
launch is created and verified, PC1 VILS shadow tests must have the converter and sender absent,
`/to_can_bus` absent, and physical CAN TX count unchanged.

No closed-loop PC4 actuation stage is allowed while the command freshness watchdog, PC2/PC3
continuity, NDT jump, MRM heartbeat, geometry, and common-clock blockers remain open.

## 9. Required next work

### PC1

- implement and bench-test command-age timeout, safe fallback, and request-bit clear;
- isolate physical CAN writer ownership and CAN-ID allowlist;
- validate `0x394` DBC/lateral velocity and the stop-check signal before changing thresholds;
- separate ROS Humble from Conda Python 3.13;
- use a host-unique RViz naming/ownership policy;
- implement PC4-required diagnostics-to-availability/MRM wiring only after the cross-PC contract is
  finalized;
- add a clearly named receive-only shadow launch rather than replacing `run_bridge` again.

### PC2/PC3/PC4 dependencies

- PC2 must provide physical-main fail-open fusion or explicit source selection and prove continuity
  through PC4 loss;
- PC3 must prove stable localization, TF, MRM heartbeat, approved map hashes, and Chrony service;
- PC4 must provide actor identity/lifetime, real-map alignment, wall-clock stamps, health/session
  diagnostics, and a deny-by-default bridge;
- all four PCs must record same-window rates, timestamps, ownership, diagnostics, process logs, CAN
  statistics, and exact Git/map/config hashes.

### Staged acceptance

1. Offline message/TTL/map/transform/unit tests.
2. PC4 private-domain replay with no vehicle bridge.
3. Four-PC shadow in MANUAL/PARK with no CAN TX.
4. Candidate fusion with no canonical planning input.
5. Canonical planning response with no physical command sink.
6. Only after every blocker closes: closed-course, safety driver/E-stop/spotters, one static virtual
   obstacle, and initial speed no greater than 3 km/h.

## 10. Final assessment

The work successfully restored the operator's historical full PC1 behavior after unsuitable
intermediate safety profiles, fixed an actual SocketCAN receiver lifecycle defect, made CAN setup
repeatable, repaired deterministic DDS interface selection, and produced detailed cross-PC evidence.

It also exposed serious unresolved integration risks: cached physical CAN commands, ambiguous CAN
signals, remote perception/localization dropouts, MRM heartbeat loss, NDT discontinuity, duplicate
nodes, Python environment contamination, and nondeterministic time-source selection. The 2026-08-12
vehicle movement and emergency stop are incident evidence, not a release pass.

PC1 can participate in PC4 VILS only after the shared ownership, health, loss, map/time, and staged
safety contracts are implemented and the existing PC1/PC2/PC3 blockers are closed with measured
evidence.
