# PC3 Chrony audit

Audit snapshot: **2026-08-10 19:45-19:47 KST**
Post-audit staged-run note: **2026-08-10**
Host: **`a` / `192.168.9.7`**
Audit mode: read-only and unprivileged. No clock, service, source, firewall, or configuration state was changed.

## Evidence labels

- **VERIFIED** — directly reported by systemd, `chronyc`, `timedatectl`, `ss`, or readable configuration.
- **INFERRED** — behavior implied by configuration/protocol semantics but not tested from another PC.
- **UNVERIFIED** — blocked by permissions or outside this PC3-only audit.

## Current status

| Item | Status | Evidence |
|---|---|---|
| Service | VERIFIED | `chrony.service` is **enabled** and **active (running)** since 2026-08-10 16:47:31 KST; main process `/usr/sbin/chronyd -F 1`. |
| OS synchronization | VERIFIED | `timedatectl`: `System clock synchronized: yes`, `NTP service: active`, RTC is UTC, timezone Asia/Seoul. |
| Source activity | VERIFIED | `chronyc activity`: 8 sources online, 0 offline, 0 unknown. |
| Selected source | VERIFIED | Reference `3.39.176.65` (`1.ubuntu.pool.ntp.org` / AWS ap-northeast-2); reference stratum 2, making PC3 stratum 3. |
| Tracking quality at snapshot | VERIFIED | System time 0.000000061 s fast; last offset -0.000355063 s; RMS offset 0.000158103 s; frequency 19.502 ppm fast; skew 0.197 ppm; root delay 0.008427541 s; root dispersion 0.002842755 s; leap status Normal. |
| NTP server socket | VERIFIED | UDP `0.0.0.0:123` was listening. Process details were hidden from the unprivileged `ss` output, but service/config evidence identifies Chrony. |
| Four-PC client use | INFERRED | `allow` directives and the UDP/123 listener configure PC3 to serve NTP, but no query from PC1/PC2/PC4 was performed. `chronyc clients` returned `501 Not authorised`. |
| GNSS/PPS time input | VERIFIED absent from readable config; hardware use UNVERIFIED | No `refclock`, PPS, PHC, or GNSS source directive appeared in readable Chrony configuration. PC3 currently selects a public pool source. |

## Post-audit staged-run context

- **VERIFIED scope:** The reported post-audit work restored/built the NovAtel packages, configured the stable serial by-id path, and performed gated staged runs. No Chrony configuration change was part of that work. The detailed tracking values in this report remain the original point-in-time snapshot rather than a new post-run sample.
- **VERIFIED safe hardware-disabled result:** No NovAtel process, `/dev/ttyUSB1` holder, or UDP 2368/9347 listener existed in the first hardware-disabled run. Those are sensor-driver resources and do not contradict the separately observed Chrony UDP/123 listener.
- **VERIFIED resolved non-time cause:** The sensor-container `SIGABRT` was traced deterministically to a Nebula decoder thread left joinable during teardown, fixed, and rebuilt. Five valid hardware-disabled exact-alias cycles then shut down cleanly with no orphan processes (`19-56-09`, `20-00-04`, `20-00-16`, `20-00-58`, `20-01-09` log directories under `/home/a/.ros/log/2026-08-10-*`). This resolves the container lifecycle failure; it does not provide cross-PC clock evidence.
- **UNVERIFIED/unaccepted time gate:** PC1/PC2/PC4 source selection, offset, stability, and behavior through a complete live run remain unaccepted. The clean hardware-disabled cycles must not be used as four-PC time acceptance.

## Readable configuration evidence

From `/etc/chrony/chrony.conf`:

```text
pool ntp.ubuntu.com        iburst maxsources 4
pool 0.ubuntu.pool.ntp.org iburst maxsources 1
pool 1.ubuntu.pool.ntp.org iburst maxsources 1
pool 2.ubuntu.pool.ntp.org iburst maxsources 2
sourcedir /run/chrony-dhcp
sourcedir /etc/chrony/sources.d
rtcsync
makestep 1 3
local stratum 10
allow 192.168.9.2/24
allow 192.168.9.110/24
bindaddress 0.0.0.0
bindaddress ::
```

- **VERIFIED:** `/etc/chrony/conf.d` and `/etc/chrony/sources.d` contain only packaged README files; `/run/chrony-dhcp` did not exist at the snapshot.
- **VERIFIED:** The configured upstream topology is public Ubuntu pools, not another vehicle PC and not the attached GNSS.
- **INFERRED:** CIDR `/24` entries are subnet permissions, so the two host-looking `allow` lines likely both authorize the same `192.168.9.0/24` network rather than only `.2` and `.110`. They are therefore likely redundant. Confirm intended client scope before changing them.
- **INFERRED:** `local stratum 10` allows PC3 to remain an NTP source for clients when upstream synchronization is unavailable. This may be intentional for isolated-vehicle operation, but clients must distinguish a fallback clock from a GNSS-disciplined clock.
- **VERIFIED configured exposure:** `bindaddress 0.0.0.0` and `bindaddress ::` request all-address binding. The observed IPv4 socket was `0.0.0.0:123`; reachability is still constrained by Chrony access rules and any firewall.

## Boot-time clock event

Systemd journal evidence for this boot:

```text
17:18:58 Selected source 158.247.202.103 (2.ubuntu.pool.ntp.org)
17:18:58 System clock wrong by 111.803682 seconds
17:20:49 System clock was stepped by 111.803682 seconds
17:28:56 Selected source 3.39.176.65 (1.ubuntu.pool.ntp.org)
```

**VERIFIED risk:** PC3 experienced a 111.803682-second wall-clock step about 33 minutes after service start. A launch that records sensor data before Chrony becomes synchronized can therefore contain a major timestamp discontinuity. Gate multi-PC Autoware startup/recording on confirmed synchronization, not merely on `chrony.service` being active.

## Four-PC topology conclusion

Current evidence supports this topology:

```text
Public NTP pools -> PC3 chronyd (stratum 3 while synchronized)
                         |
                         +-> intended clients on 192.168.9.0/24
```

- **VERIFIED:** PC3 is presently an internet-pool NTP client and has an IPv4 NTP server socket.
- **INFERRED:** PC3 is intended to be the vehicle LAN time server.
- **UNVERIFIED:** PC1, PC2, and PC4 source selection, offset, reachability, and whether they actually use `192.168.9.7`.
- **UNVERIFIED:** Behavior when public NTP is unavailable, including whether all four PCs remain mutually close enough for sensor fusion while PC3 serves local stratum 10.

## Permission limits

| Check | Result | Consequence |
|---|---|---|
| `/etc/chrony/chrony.keys` | Permission denied | Authentication key content was not inspected. |
| `chronyc clients` | `501 Not authorised` | Current client list/request counts are UNVERIFIED. |
| `ufw status verbose` | Must be root | UFW policy and UDP/123 allowance are UNVERIFIED. |
| `nft list ruleset` | `Operation not permitted` | nftables policy is UNVERIFIED. |
| Remote-PC checks | Out of PC3-only scope | Actual PC1/PC2/PC4 Chrony sources and offsets are UNVERIFIED. |

## Safe acceptance checks for the later bring-up

Run read-only checks on every PC after its intended Chrony configuration is in place:

```bash
systemctl is-active chrony
chronyc activity
chronyc tracking
chronyc sources -v
timedatectl status
```

Acceptance should require all four PCs to report synchronization, PC1/PC2/PC4 to select the intended PC3 source (if PC3 remains the chosen authority), stable offsets suitable for the sensing stack, and no post-launch wall-clock step. The resolved hardware-disabled sensor-container lifecycle does not clear this cross-PC time gate. Firewall validation requires an authorized administrator because it was not visible to this audit.
