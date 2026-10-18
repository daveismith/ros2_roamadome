# Dome calibration: findings and history

The measurements behind the calibration procedure in the README ("Re-tuning the velocity
mapping when the dome changes"). The README keeps the instructions; this file keeps what was
found on the bench robot, when, and the numbers that are configured today. Every run's own
report lives in `docs/bench_results/`.

## Current calibration state (2026-10-17)

Dome installed on the new sensor plate, realigned on 2026-10-17. Firmware settings (persisted
on the controller, read back through `device.*`):

| Setting | Value | Why |
|---|---|---|
| `MaxSpeed` | 80 | Unchanged from the original setup; does not scale `:DPR` (see below) |
| `MinSpeed` | 20 | MinSpeed study of 2026-10-17: 20 clean, 15 stops the position approach short |
| `SpeedTarget` | 100 | Caps `:DPR`; 60 limited the velocity controller to 3.5 rad/s |
| `DecelerationScale` | 250 | Position-move tuning of 2026-10-16 |
| `Fudge` | 5 | Fudge study of 2026-10-16 (user limit: at most 5) |
| `Scaling` | off | |
| WiFi | off | Controller restart avoided during runs |

Velocity map, from the operating-settings speed sweep at MinSpeed 20
([2026-10-17-realigned-dome.md](bench_results/2026-10-17-realigned-dome.md), MinSpeed study),
configured in `r2_bringup/urdf/ros2_control.xacro`:

| Parameter | Value |
|---|---|
| `max_speed_rad_per_sec` | 6.62 (the lower measured 100 % rate; the fit extrapolates 6.75) |
| `velocity_map.positive.k_percent_per_rad_s` / `p0_percent` / `min_percent` / `percent_sign` | 12.75 / 13.9 / 20 / -1 |
| `velocity_map.negative.k_percent_per_rad_s` / `p0_percent` / `min_percent` / `percent_sign` | 12.36 / 14.5 / 22 / +1 |
| `velocity_map.below_minimum` | zero (default) |
| `position_move_speed_percent` | 0 (not used; see the per-move speed finding) |

With these the plugin sends `:DPR-33` for +1.5 rad/s, `:DPR33` for -1.5, `:DPR-52` for +3.0
and `:DPR52` for -3.0. A single shared set would be k 12.6, p0 14.2, `min_percent` 22.

Sign convention: on the bench robot (firmware `Inverted` true) a **positive** percent moves
the reported angle **down**, so a positive joint velocity needs a negative percent. Since
phase 7 the plugin carries that sign (`percent_sign`), and the teleop scales in `r2_bringup`
are positive. Every speed sweep measures the sign (`angle_sign_for_positive_percent`); if a
run reports the opposite, something in the wiring or `Inverted` changed.

## Timeline of the bench work

- **No dome (2026-10-14).** The first sweeps
  ([2026-10-14-no-dome-baseline.md](bench_results/2026-10-14-no-dome-baseline.md)) were taken
  with no dome installed, the motor driving only the bare base platform. Baseline only.
- **Dome mounted, old plate (2026-10-14 and 2026-10-15).** Repeated with the dome
  ([2026-10-14-with-dome.md](bench_results/2026-10-14-with-dome.md)) and after re-seating it
  ([2026-10-15-with-dome-reseated.md](bench_results/2026-10-15-with-dome-reseated.md)). Both
  failed the sensor sweep with position spikes (6, then 102 jumps).
- **The cause** ([2026-10-15-sensor-ring-raw-debug.md](bench_results/2026-10-15-sensor-ring-raw-debug.md)):
  single-bit misreads of two sensors at three spots on the old sensor pattern plate, made worse
  by the dome's load. The ring has one track read by nine sensors 40 deg apart.
- **New plate (2026-10-16).** A stiffer plate; sweeps without the dome
  ([2026-10-16-no-dome-new-plate.md](bench_results/2026-10-16-no-dome-new-plate.md)) and with it
  ([2026-10-16-with-dome-new-plate.md](bench_results/2026-10-16-with-dome-new-plate.md)): no
  spike or excursion in about 194 revolutions; the firmware's sensor error counter rose by 3 in
  about 309 revolutions, against 2.2 to 2.9 per revolution on the old plate. MinSpeed raised to
  32 and the position-move settings tuned the same day.
- **Position sweep service, wrap allowance, plausibility gate (2026-10-16 evening)**
  ([2026-10-16-position-sweep-service.md](bench_results/2026-10-16-position-sweep-service.md)):
  the bench node's position sweep validated on the dome; the per-move speed probe; SpeedTarget
  back to 100 with the operating-settings speed sweep that became the first phase 7 input.
- **Phase 7 acceptance (2026-10-17)**
  ([2026-10-17-phase7-acceptance.md](bench_results/2026-10-17-phase7-acceptance.md)).
- **Realignment and MinSpeed study (2026-10-17 evening)**
  ([2026-10-17-realigned-dome.md](bench_results/2026-10-17-realigned-dome.md)): after the user's
  adjustment, no skips, same top speed and floor, less friction at the low end; MinSpeed lowered
  to 20.

Superseded values, kept in their reports as history: old plate with dome
(`max_speed_rad_per_sec` 6.56; k 11.76 / 11.72; p0 20.0 / 20.7; `min_percent` 32), new plate
before the realignment at MinSpeed 32 (6.58; 11.94 / 11.75; 18.3 / 18.2; 32), realigned at
MinSpeed 32 (6.55; 13.00 / 13.12; 13.0 / 12.8; 32). The no-dome reports are for comparison only.

## Findings

### The drive and the firmware settings

- **MaxSpeed 80 does not scale `:DPR` to 80 %.** In every pair of raw-motor and
  operating-settings runs, with and without the dome and on both plates, the speeds from 50 %
  up agreed within 6 %, and within 5 % at 100 %. (Firmware: the serial console's emulated
  throttle is uninitialised, which forces the speed modifier to 1.0.)
- **SpeedTarget caps `:DPR`** (the firmware clamps the rotate percent to it) as well as
  position moves. At 60, set during the position tuning, the velocity controller topped out at
  3.5 rad/s (200 deg/s). It went back to 100 on 2026-10-16 once the per-move speed probe
  showed it does not change the moves.
- **A per-move speed changes nothing with a long taper.** The firmware accepts
  `:DPA<deg>,<percent>` (it sets that move's target speed exactly as SpeedTarget would) and the
  plugin can send it (`position_move_speed_percent`, `DomeBenchMove.speed_percent`,
  `position_sweep.move_speed_percent`), but 150 deg moves at 0.40, 0.60 and 1.00 gave the same
  peak (76 to 112 deg/s) and rest error. The Reeltwo drive commands
  `speed x distance / DecelerationScale x MaxSpeed`, floored at `MinSpeed`, so with
  `DecelerationScale` 250 and moves of at most 180 deg the taper and the floor decide the speed.
  The knob only matters with a short taper (around 100 deg), where the earlier runs overshot.
- **The dome barely changes the steady-state speeds** (100 %: 6.70 / 6.70 rad/s with it against
  6.69 / 6.65 without) but slows the step response about 2.6 times (1.58 s against 0.60 s to
  90 % of the step speed).
- **The first sweep of a session runs about 6 % slower at 100 %** than one twelve minutes later
  (6.21 / 6.31 against 6.55 / 6.56 on 2026-10-17): the dome warms up.
- **The SyRen 10 has no ramping, deadband or serial timeout** (packet serial commands 0 to 3
  only); the firmware's Sabertooth ramp and timeout calls are no-ops on it, and the stopping
  distance is pure coasting.

### The low end: the floor and the fit

- Below about 40 % the speed is set by the dome's position-dependent friction rather than by
  the command. The speed sweep's lowest steps scatter by up to 22 % around the fit; from 40 %
  up every step is within 5 % (R squared 0.993 to 0.999 in every run).
- The raw-motor and operating-settings runs find slightly different stall boundaries, because
  where a slow step stalls depends on where it starts (2026-10-14: 32 % operating against 27 %
  raw; 2026-10-16: 32 / 25 against 30 / 27). Hence the README rule: use the larger
  `min_percent` of the two per direction.
- **MinSpeed study (2026-10-17, after the realignment).** Position sweep rest errors and
  speed-sweep floors per MinSpeed:

  | MinSpeed | rest errors (deg) | mean / max | floor (+ / -) and speed there |
  |---|---|---|---|
  | 32 | +6 -10 +11 -5 +9 -8 +9 -4 +8 -9 +13 -7 | 8.2 / 13 | 32 / 32 %, 1.30 / 1.30 rad/s |
  | 28 | +4 -15 +5 -9 +1 -9 +4 -5 +2 -10 +6 -9 | 6.6 / 15 | (speed sweep aborted by a 10 s report gap) |
  | 25 | +1 -9 +3 -5 -2 -5 +4 -3 0 -8 +2 -4 | 3.8 / 9 | 25 / 25 %, 0.77 / 0.59 rad/s |
  | **20** | **0 +4 +8 +2 +5 +3 +5 +3 0 +2 +7 +2** | **3.4 / 8** | 22 / 20 %, 0.55 / 0.30 rad/s |
  | 15 | -38 -3 -22 0 -43 +27 -3 +5 -39 +24 -16 +28 | 20.7 / 43 | 15 % does not move |

  15 is where it breaks: seven of twelve moves stopped 16 to 43 deg short with no
  `REACHED TARGET`. The user's rule, the multiple of 5 above the break, gives 20. Before the
  realignment (2026-10-16) 25 stalled 2 of 18 moves and 28 stalled 1 of 18, which is why 32 was
  chosen then.

### Acceptance of the velocity map

Points chosen 2026-10-17: +-1.5 and +-3.0 rad/s, telemetry within 10 % after settling (the
plan's 0.5 rad/s is unreachable: it maps below the floor and the plugin sends 0 by design).

- At MinSpeed 32 ([2026-10-17-phase7-acceptance.md](bench_results/2026-10-17-phase7-acceptance.md)):
  3.0 rad/s within 4 % in every run; 1.5 rad/s marginal, 36 % running 1.04 to 1.56 rad/s over
  eight windows (four within 10 %), the slow ones mid-session after an hour of use.
- Realigned, MinSpeed 32 map: 3.0 within 3 %; 1.5 mapped to the floor itself and ran 1.16 to
  1.30 (14 to 23 % slow).
- Realigned, MinSpeed 20 map: 3.0 within 6 % (a little fast after an hour of running); 1.5 at
  the edge of the band, 1.31 to 1.65 rad/s over four windows, three within 10 %; 1.0 rad/s
  (27 %) commandable but 0.68 to 0.81.

The mapping is reliable from about 2 rad/s (39 %) upward and indicative below.

### Position moves

The firmware's position move tapers the commanded percent from `SpeedTarget` down to the
`MinSpeed` floor over the last `DecelerationScale` degrees, then cuts the motor at the `Fudge`
boundary; the dome coasts from there (no driver ramp). The rest error is the stopping distance
at the arrival speed minus the fudge.

- Stopping-distance curve (travel from the stop to rest against the speed just before it):
  `d = 0.47 s x v - 18 deg` on 2026-10-16 (direct serial, SpeedTarget 100, 73 to 392 deg/s);
  `0.41 s x v - 14` by the sweep service the same evening; `0.50 s x v - 29` after the
  realignment (less friction, longer coast); `0.37 s x v - 4` at MinSpeed 20. Steps whose speed
  window or coast crosses 0 deg are inflated by the dead zone there (about 9 deg).
- Tuning table (new plate, MinSpeed 32, 2026-10-16):

  | SpeedTarget | DecelerationScale | Fudge | rest error over 6 moves |
  |---|---|---|---|
  | 100 | 50 | 5 (original) | +47, -30, +3, -13, +49, -22 |
  | 100 | 150 | 5 | +18, -3, -20 (stalled short), -1, +21, 0 |
  | 45 | 200 | 5 | +10, 0, +12, -1, +7, -1 |
  | 45 | 250 | 8 | +6, -6, 0, -8, +2, -10 |
  | 60 | 250 | 10 | -2, -5, -2, -1, -5, -2 |
  | 60 | 250 | 5 | +2, -6, +2, -5, +3, -5, +2, -4, -3, +2, +4, -4 (12 moves, 20 to 150 deg) |
  | 60 | 250 | 5 | 0, -5, +9, -2, +5, -7, +9, -3, +4, -3, +7, -5 (sweep service, same evening; arrival 52 to 79 deg/s) |
  | 60 | 250 | 5 | +10, -3, +16, -3, +10, -4, +11, -2, +6, -4, +9, -6 (re-run from 257 deg: the angle-increasing moves ended in the fast 299 to 354 deg sector) |
  | 100 | 250 | 5 | +7, -10, +7, -8, +1, -10, +4, -5, +2, -11, +3, -7 (SpeedTarget back to 100) |

- `DecelerationScale` 250 (the maximum) is needed because the dome's response time is 1.5 s;
  50 deg is far too short. `SpeedTarget` does not change the accuracy once the taper is long
  enough. Fudge 3 adds about 3 deg of overshoot on short moves, which arrive at 60 to 75 deg/s
  regardless of the floor. The floor speed varies with the end angle (position-dependent
  friction: 40 to 100 deg/s at 32 %, 13 to 55 at 20).
- The rest error depends strongly on the end sector: angle-increasing moves ending between
  about 300 and 355 deg arrive faster and overshoot about 10 deg at MinSpeed 32. Lowering
  MinSpeed to 20 after the realignment halved the rest errors (mean 3.4 deg, max 8).
- Getting below this needs a shorter stopping distance (active braking in the firmware, or a
  host-side final nudge), not more fudge tuning.

### The sensor ring and the plausibility gate

- **The 355 to 4 deg dead zone.** The controller never reports 359 to 2 deg, so every crossing
  of 0 has one step about 9 deg longer than the dome moved. At 100 % that step (20 to 22 deg)
  reached the jump threshold of 1.5 x the measured maximum and scored as the one jump of the
  2026-10-16 with-dome sensor sweep. It is present on both plates and is not a sensor fault.
  `sensor_sweep.wrap_allowance_deg` (10 deg) now absorbs it; re-analysed with the allowance that
  sweep has 0 jumps, and older runs reproduce unchanged because a `run.json` without the setting
  gets an allowance of 0.
- **The plausibility gate** (`max_plausible_deg_s`, 600 in the robot URDF, 1.5 x the measured
  381 deg/s top speed) has held, rejected and resynced nothing across every sweep since it went
  in (tens of thousands of reports at up to 6.9 rad/s). Insurance against a repeat of the
  old-plate misreads, which shifted the unwrapped angle by a full turn.
- A benign one-sample edge race exists at dome angle 264 deg (sensors 1 and 3); the ring's
  median filter removes it.

### Telemetry dropouts

- The 1 s dropouts of 2026-10-15/16 were host stalls (CPU load from analysis run during a
  sweep; unattended-upgrades), not the serial link; mitigated by running the stack at nice 0,
  disabling the apt timers, and the plugin's `report_gap_warn_ms` warning and `report_gaps`
  counter. A 4.5 min soak showed none afterwards.
- One unexplained 10.5 s gap in position reports on 2026-10-17 during the MinSpeed 28 speed
  sweep, with the dome at 70 % and no USB error in the kernel log; reports resumed and the stop
  went through. The velocity path has no watchdog, so a stalled process cannot stop the dome:
  keep a human at the robot.
- A firmware position move cannot be cancelled from the host: `:DPR0` does not clear the
  firmware's target mode, a non-zero `:DPR` only overrides it while non-zero. The plugin tracks
  an outstanding move and refuses bench drives and settings writes until it ends.
