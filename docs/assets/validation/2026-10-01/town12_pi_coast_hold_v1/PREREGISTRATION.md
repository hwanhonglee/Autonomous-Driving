# Town12 bounded PI / coast / hold candidate v1

<!-- HH_261001 - Declare one combined controller candidate before execution, without upgrading failed measurements into TRAIN. -->

Status at preparation: NOT EXECUTED. One prospective, explicitly NON-TRAIN candidate; no full route, six-camera collection, future labels, automatic retry or training admission. Existing v1-v4 measurements and their source files remain unchanged. The parent owner alone schedules the live simulator.

## Evidence and scope

V4 completed four measured tiny-brake cases, not smooth teacher data. Its zero-brake pulse covered 2.00000003 seconds from actual 1.464669 m/s to 0.726980 m/s, mean deceleration 0.368845 m/s²; it did not stop. Brakes 0.0001 / 0.001 / 0.005 each produced original native rate-limit failures. All four launches shared a +3.614599 m/s² maximum at the observed gear 0-to-1 transition, while the original normal PID was saturated and the timed cap moved from 0.1460 to 0.1465.

The raw 20 Hz acceleration alternation existed at constant throttle 0.20 and continued under zero pedals. After excluding the first two zero-pedal observations, acceleration lag-1 correlation was -0.99721; after ten, -0.99813. This is not evidence that PID creates all oscillation or that any proposed controller will remove it. A controller-only two-sample mean leaves the known saturated launch prefix unchanged.

Local PhysX source distinguishes nonzero brake torque in tire-slip normalization and wheel-speed sign-crossing locking. CARLA source disables the generic reverse-as-brake behavior. Source availability is not verified loaded-binary, wheel-speed or internal-substep equivalence; no engine/config/asset changes are authorized.

This calibration occupies the same verified Town12 road 588 / section 0 / lane 1 uphill starting area. Neither its coast response nor a PASS transfers to the later signal approach, downhill route endpoint, flatter Town07, other speeds or full-route stopping. Prior 5 ms substep, hold-brake ramp and terminal creep-feedback experiments already failed full data admission and are not repeated here.

Pinned evidence:

- V4 owner result SHA256: `b7d79fb91362f6fcdf04f585408bdf6e1a16684b11ae7f4b4d6d3fee6c8c2de5`.
- V4 zero-coast level result SHA256: `ded00452ca969359e9782364895cb3768a366e16529a10f0e681396fe2b97190`.
- V4 zero-coast raw native frames SHA256: `bbc9a59a20ced2c32d32f7245361512aaf1ad71b52a83fa9ea94574e5266a954`.

## Declared intervention

The experimental unit is ONE combined normal-longitudinal controller replacement, not an isolated Kp or pure-speed causal A/B. Persistent SI PI, anti-windup, from-zero throttle slew and removal of the empirical 1.58 m/s target compensation are deliberately combined. The nominal target is 1.50 m/s. Gains are an unvalidated engineering candidate, not fitted plant parameters or a smoothness guarantee.

At the native source observation, use unfiltered actor-snapshot planar speed. Error is `e = 1.50 - speed` in m/s. Store the integral as a throttle contribution. With `dt=0.05 s`, candidate integral is `clip(I + 0.04*e*dt, 0, 0.20)`. Proportional contribution is `0.12*e`. The actuator interval is `[max(0, previous_throttle-0.05*dt), min(0.20, previous_throttle+0.02*dt)]`.

Reject the candidate integral only when its raw PI output exceeds the current upper bound with positive error, or falls below the lower bound with negative error. Otherwise accept it. Recompute PI output with the retained/accepted integral and clip it to the actuator interval. The first command has `dt=0`, throttle exactly 0 and integral 0. The integral persists across native PI ticks; it is neither a ten-sample window nor silently reset. No filtered measurements, future values or rewritten labels are used.

Only the normal longitudinal output is replaced. Original BasicAgent steering, route plan, base minimum distance 1.5 m, distance ratio 0.5 s, vehicle hazards and Red emergency remain. The original longitudinal proposal remains logged but is not the driving throttle source. The normal replacement runs BEFORE the original emergency return. If that return overrides it, apply the emergency unchanged, preserve ACK and physical response, then end the calibration without resuming PI.

## Fixed state sequence and limits

1. One verified +0.5 m spawn attempt; the same XY/lane/clear-space checks. Exactly one fresh owned Prius plus its owned event sensors. Settle under brake 1 for 40 native observations; require actual speed at most 0.1 m/s.
2. PI launch/stabilize for strictly less than 20 s. Require ten consecutive raw source observations in `[1.45,1.55]` m/s. A bad sample clears the counter. Log all ten witnesses. No target-speed substitution.
3. At that source observation, immediately command throttle 0 / brake 0. Coast for at most 10 s. Two consecutive raw source observations at or below 0.05 m/s authorize hold; a larger sample clears this counter. No low-speed feedback throttle or moving service-brake pulse.
4. Hold throttle 0 / brake 1 for 40 command-bound physical observations, each at most 0.05 m/s. Verify the actual observations; commands or predicted future stops do not count. The first complete hold observation is one native tick after the qualifying source observation.
5. Separate cleanup always retains its full-service-brake native observations. Destroy and verify absence of exactly created actors, then restore owned runtime settings/weather. Never destroy foreign actors.

Coast, hold and original emergency bypass normal throttle slew. No new state is allowed after 40 successful hold observations. The complete normal sequence has an outer cap of 640 native control ticks (20 s launch + 10 s coast + 2 s hold); there is no automatic extension or retry.

Unchanged operational guards: exact source world/map/XODR/route/signal identity, all three owned TCP/UDP ports and process-generation ownership, native 20 Hz ACK/frame/readback alignment, speed at most 2.55 m/s, route progress at most 30 m, every body corner at least 40 m upstream of the controlling trigger boundary, all eight corners in lane 588/0/1 with 0.10 m inset, no collision/lane events. Original 0.01 s maximum substep and max count 10 must be read back unchanged. Instantaneous corners do not prove swept-body containment or physical painted-stop-line alignment.

Additional storage guard requested before freeze: require at least 5 GiB free at owner preflight and worker entry. Record free bytes in every native observation; abort after preserving an observation below that floor. Do not delete any old evidence to make room. Bounded actor/process cleanup remains active and retains its small raw receipt even if the floor is crossed.

## Raw evidence and verdict

Retain raw pose, world velocity/acceleration/angular velocity, virtual-base state, raw actor speed, frame/time, gear, proposed/applied controls, PI before/candidate/after state, saturation/slew bounds, policy stage, source witnesses, control receipts, geometry and events. Retain rejected policy decisions and all abort/cleanup rows separately. No row is deleted, clipped, smoothed, shifted in time or relabeled.

Operational guard failure aborts after the violating observation is written. Pure motion-rate QA failure does not truncate the planned bounded measurement; the complete raw response is retained and the final candidate result is FAIL. Assess every normal PI/coast/hold native interval against the existing physical-decoder acceleration/deceleration limit 2.9 m/s² and runtime limits +3/-6 m/s², sourced from the current frozen model/contract. Cleanup is explicitly excluded from candidate-quality counts, not removed from raw evidence. Independent offline audit remains required. Native-rate PASS alone does not establish all-nine future metrics, continuous-moving duration, body/swept geometry, image ABI or TRAIN eligibility.

Exactly one candidate is authorized here. Failure means preserve it and diagnose; no next controller value is automatically tried. Only a separately authorized clean repeat and later Town07 stratum can follow success. Full right-turn capture is not authorized by this preregistration.

Prepared command (parent only, after tests and final freeze acknowledgement):

```bash
python3 tools/workspace_python.py tools/e2e/run_owned_town12_j15824_pi_coast_hold_20261001.py --execute
```

Output: `storage/experiments/validation/2026-10-01/town12_j15824_pi_coast_hold_v1`. Omitting `--execute` performs only static owner preflight; the worker without `--execute` emits its contract without CARLA contact.
