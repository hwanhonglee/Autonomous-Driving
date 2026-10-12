# Every physical-profile PASS route: nominal body audit

<!-- HH_260929 - Resolve endpoint-variant uncertainty without changing the physical gate. -->

Status before execution: `NOT_RUN`. The pinned [v3 static scout](report_v3.json) contains 62 routes that passed the unchanged physical-turn profile: Town03 20, Town02_Opt 4, Town05_Opt 38. Its one-representative-per-connector Prius body screen does not prove that every endpoint variant fails. This follow-up replays **all 62 exact route hashes** with the same pinned packaged XODR, original 30–120 m window, archived Prius dimensions, 0.10 m sample spacing, raw lane and 10 cm inset body/sweep rule. It writes new `body_all_report.json` only. The report will distinguish `PASS`, `FAIL`, and `UNSUPPORTED` and preserve every route's status; no failing sample is removed or threshold altered.

This remains a static nominal screen. A candidate with no signal record and nominal PASS would still need a locked TRAIN/DEV role, live XODR/bbox match, actual continuous `>0.5 m/s` movement for 30 s, complete 64 future points/6.4 s, the original nine motion bounds, actual full body/sweep, collision and ≤2.5 m goal QA. No CARLA server, TEST, SSH, GPU, installation, raw capture, or model run is authorized here.
