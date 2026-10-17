# 2026-10-16 evening: position sweep service, wrap allowance and plausibility gate on hardware

Validation run for the bench node's new `~/start_position_sweep`, the 0-crossing wrap
allowance in the jump scoring, the plugin's position-report plausibility gate, the bench move
service and the position-mode activation fix. Dome installed, new sensor plate, firmware
settings as left by the fudge study: MaxSpeed 80, MinSpeed 32, SpeedTarget 60,
DecelerationScale 250, Fudge 5, Scaling off, WiFi off. Standalone stack
(`~/dome_bench/stack`, every process at nice 0) with `max_plausible_deg_s` 600 in the URDF.
Run directories: `~/dome_bench/position_sweep/20261016-202611-position` and
`20261016-202200-sensor`; the result JSONs are next to this file
(`2026-10-16-position-sweep-service-position.json` and `-sensor.json`).

## Position sweep (`position_sweep.json`)

Default parameters: stop percents 35, 50, 70, 100 (5 s run), twelve moves +150, -150, +40,
-40, +90, -90, +20, -20, +120, -120, +60, -60 deg, settle 1.2 s. Status `complete`, 3037
samples, 0 jumps, largest report gap 67 ms (one controller cycle), no firmware position
timeouts.

**The firmware clamps `:DPR` to `SpeedTarget`.** With SpeedTarget 60 the 70 and 100 % steps
ran at the same 200 deg/s (3.5 rad/s) as 60 % would. The node now warns when a stop percent
exceeds `device.speed_target`. This also means the velocity controller's top speed is
3.5 rad/s at the current settings, not the 6.56 rad/s of the calibration reference, which was
measured at SpeedTarget 100 (see the README's step (e)).

Stopping-distance curve (speed is the slope over the 0.3 s before the first report stamped
`commanded_percent` 0; distance is the unwrapped travel from that report to rest):

| percent | speed before stop (deg/s) | stop distance (deg) | stop time (s) |
|---|---|---|---|
| +35 | 88.5 | 20 | 0.60 |
| -35 | 107.0 | 36 | 0.77 |
| +50 | 175.3 | 41 | 0.73 |
| -50 | 173.4 | 54 | 0.90 |
| +70 | 198.0 | 74 | 0.97 |
| -70 | 183.0 | 74 | 0.93 |
| +100 | 203.5 | 72 | 0.87 |
| -100 | 218.9 | 74 | 0.97 |

Fit `d = 0.41 s x v - 14 deg` (R squared 0.82; the direct-serial `pos_tune.py` curve of the
afternoon, at SpeedTarget 100 and up to 392 deg/s, was `0.47 s x v - 18 deg`). Only 90 to
220 deg/s is covered at SpeedTarget 60; set `device.speed_target` 100 for the full curve.

Moves (all completed, all `REACHED TARGET` from the firmware at the fudge boundary):

| move | rest error (deg) | furthest past target | peak (deg/s) | arrival at fudge (deg/s) | settle (s) |
|---|---|---|---|---|---|
| 321 -> 111 (+150) | 0 | 0 | 90 | 54 | 3.39 |
| 111 -> 321 (-150) | -5 | 5 | 103 | 55 | 2.98 |
| 316 -> 356 (+40) | +9 | 9 | 66 | 66 | 1.83 |
| 5 -> 325 (-40) | -2 | 2 | 70 | 64 | 1.65 |
| 323 -> 53 (+90) | +5 | 5 | 92 | 66 | 2.26 |
| 58 -> 328 (-90) | -7 | 7 | 88 | 79 | 2.29 |
| 321 -> 341 (+20) | +9 | 9 | 65 | 65 | 1.35 |
| 350 -> 330 (-20) | -3 | 3 | 60 | 58 | 1.40 |
| 327 -> 87 (+120) | +4 | 4 | 89 | 53 | 2.76 |
| 91 -> 331 (-120) | -3 | 3 | 92 | 72 | 2.52 |
| 328 -> 28 (+60) | +7 | 7 | 99 | 70 | 1.98 |
| 35 -> 335 (-60) | -5 | 5 | 90 | 67 | 1.89 |

Rest error: mean |e| 4.9 deg, max 9 (angle increasing 5.7 mean, decreasing 4.2); every move
overshoots, none falls short. Arrival speed 52 to 79 deg/s, and the stop fit predicts 8 to
18 deg of coast from those speeds, consistent with the 0 to 9 deg rest errors after the 5 deg
fudge. The two 9 deg results were short angle-increasing moves ending at 356 and 341 deg.
Peak speed stayed at 60 to 103 deg/s: with DecelerationScale 250 every move up to 150 deg
lies inside the taper, so SpeedTarget's cruise speed is never reached (as in the afternoon's
tuning: about 3 s per 150 deg move).

Compared with the afternoon's twelve-move `pos_tune.py` run at the same settings (+2, -6, +2,
-5, +3, -5, +2, -4, -3, +2, +4, -4) this run is about 2 deg worse in the angle-increasing
direction; the dome had just completed eight full-speed stop steps, so bearing temperature
and the end angles differ. Both are within the fudge-5 expectation (rest within about 6 deg
typical, outliers to 9 to 12).

## Sensor sweep at 100 % with the wrap allowance (`sensor_sweep.json`)

`sensor_sweep.percents` [100], 15 s per direction, raw motor settings, threshold the default
720 deg/s (no speed sweep in the run directory), `wrap_allowance_deg` 10.

| step | speed (rad/s) | displacement (deg) | jumps | largest gap (s) |
|---|---|---|---|---|
| +100 % | 6.87 | -6441 | 0 | 0.067 |
| -100 % | 6.64 | +6226 | 0 | 0.067 |

0 jumps, 0 reversals, `suggested_pass` true over 35 revolutions, about 35 crossings of 0
deg. Re-analysed offline with the allowance, the three saved sensor sweeps of 2026-10-16
(`~/dome_bench/phase6-newplate*/…-sensor`) also score 0 jumps; without it they reproduce
their saved JSONs exactly (`--check`), so older results are unchanged.

## Plugin: plausibility gate, reporting flag, position-mode activation

`/diagnostics` after both sweeps (`roamadome: sensor` and `link`): `reports_held` 0,
`reports_rejected` 0, `jumps_confirmed` 0, `last_rejection_age_s` never, `report_gaps` 0,
`position_reporting_enabled` true (it read false for whole sessions before whenever the
startup `#DPREPORT` ack was missed). The gate at 600 deg/s held nothing over 4800 reports at
up to 6.9 rad/s, including the 0-crossings (the +10 deg allowance covers the 355 to 4 deg
dead zone, which at 100 % is a 22 deg step against an allowed 20 + 2 + 10).

Spawning `dome_position_controller` with the dome at 330 deg logged
`Position mode: holding the current angle (330.0 deg) until commanded`, the dome did not
move and `last_rotary_target_deg` stayed at the previous bench move's 335: no `:DPA0` on
activation any more.

Bench move service: twelve `:DPA` moves sent once each (`Bench move: sent :DPA<n>`),
keep-alive calls every 0.5 s re-sent nothing, each claim released with percent 0 at rest.

## Lessons

- A bench node left running from an earlier script (the pid file held the `ros2 run`
  wrapper, not the node) answered the next start call too and drove the dome alongside the
  new node. Two contaminated run directories were deleted. Kill bench nodes by the node's own
  pid (`pgrep -f lib/ros2_roamadome/roamadome_bench`) and check `pgrep` before a run.
- `SpeedTarget` caps the velocity path as well as position moves. Decide whether the robot
  runs at SpeedTarget 100 (full velocity range, position moves tuned with `:DPA<deg>,<speed>`
  per move, which the firmware supports but the plugin does not send yet) or at 60 (current;
  velocity mapping to be re-measured at these settings for phase 7).
