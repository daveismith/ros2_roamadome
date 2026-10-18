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

## Per-move speed probe (21:08, `:DPA<deg>,<percent>`)

After the plugin learned to send a per-move speed, a short position sweep with
`position_sweep.move_speed_percent` 100 (log lines `Bench move: sent :DPA82,100` and so on,
SpeedTarget still 60) gave the same moves as before: +150 rest +2 peak 101 deg/s, -150 rest
-5 peak 114, +90 rest +11 peak 81, -90 rest -6 peak 84. A direct serial probe then sent four
150 deg moves straight to the controller:

| command | firmware echo | peak (deg/s) | rest error |
|---|---|---|---|
| `:DPA84` | `ROTARY DEGREE: 84 speed: 0.60` | 88 | +1 |
| `:DPA235,100` | `ROTARY DEGREE: 235 speed: 1.00` | 76 | +6 |
| `:DPA31,40` | `ROTARY DEGREE: 31 speed: 0.40` | 112 | +7 |
| `:DPA188,100+` | `ROTARY DEGREE: 188 speed: 1.00` | 92 | +4 |

The firmware parses and applies the suffix (it sets the move's target speed, exactly what
`SpeedTarget` does), but the motion does not follow it. The Reeltwo drive's
`moveDomeToTarget` commands `speed x |distance| / DecelerationScale`, then
`max(MaxSpeed x that, MinSpeed)`: with DecelerationScale 250 a 150 deg move starts at
0.6 x speed x 0.8 and is at the 32 % floor after about 50 deg, so the first half-second is
the only difference and the slow-responding dome never shows it. Consequences:

- `device.speed_target` can go back to 100 for the velocity controller without changing
  position-move accuracy; the position moves behave as they do now at 60.
- The per-move speed only matters with a short `DecelerationScale`, where the earlier
  tuning overshot. It stays available (`position_move_speed_percent`,
  `DomeBenchMove.speed_percent`, `position_sweep.move_speed_percent`) for that case.
- The firmware's echoed speed is now a diagnostic value (`last_rotary_speed` on
  `roamadome: motion`), so a sweep shows which speed each move ran with.

## Lessons

- A bench node left running from an earlier script (the pid file held the `ros2 run`
  wrapper, not the node) answered the next start call too and drove the dome alongside the
  new node. Two contaminated run directories were deleted. Kill bench nodes by the node's own
  pid (`pgrep -f lib/ros2_roamadome/roamadome_bench`) and check `pgrep` before a run.
- `SpeedTarget` caps the velocity path as well as position moves. Decide whether the robot
  runs at SpeedTarget 100 (full velocity range, position moves tuned with `:DPA<deg>,<speed>`
  per move, which the firmware supports but the plugin does not send yet) or at 60 (current;
  velocity mapping to be re-measured at these settings for phase 7).

## Re-run after review (20:47, `2026-10-16-position-sweep-service-position-rerun.json`)

The PR review added an outstanding-firmware-move flag in the plugin (no bench drive or
settings write until `REACHED TARGET`), a gate resync, stricter standstill detection for moves
(the dome must first leave its start angle) and the firmware verdict tied to the move's own
time window. Same settings, same defaults, run directory
`~/dome_bench/position_sweep/20261016-204753-position`.

- Status `complete`, 12/12 moves completed and `REACHED TARGET` for each, 0 firmware timeouts,
  0 jumps, 3029 samples, largest gap 67 ms, gate counters all 0.
- The startup `#DPREPORT28` probe timed out once and succeeded on its retry (the known
  once-per-session miss); `position_reporting_enabled` nevertheless read true.
- Rest errors: +10, -3, +16, -3, +10, -4, +11, -2, +6, -4, +9, -6 (mean |e| 7.0; angle
  increasing 10.3, decreasing 3.7). Worse than the 20:26 run (4.9) in the angle-increasing
  direction only. The dome started this run at 257 deg, so every angle-increasing move ended
  between 299 and 354 deg (the 20:26 run's ended at 28 to 111), and the arrival speeds there
  were 69 to 88 deg/s against 53 to 70 before. This is the position-dependent friction the
  afternoon's tuning noted (floor speed 40 to 100 deg/s with end angle): the 280 to 355 deg
  sector is a fast, low-friction one for angle-increasing travel. The lubrication the user
  plans is the fix; until then expect +10 deg there with Fudge 5.
- Stop curve at SpeedTarget 60 again (speeds 85 to 249 deg/s, all clamped runs); two steps
  flagged for a 0-crossing in the speed window or coast. The fit over all eight,
  `0.22 s x v + 17 deg` (R squared 0.66), differs from the 20:26 run's `0.41 s x v - 14`
  because the clamped steps all sit at one speed with 62 to 74 deg of scatter; the curve
  needs SpeedTarget 100 to be meaningful, as noted above.

## SpeedTarget back to 100 (21:39, `2026-10-16-speedtarget-100-position.json` and `-speed-operating.json`)

`ros2 param set /roamadome device.speed_target 100` (readback 100, one settings ack, no
timeout), then the default position sweep and an operating-settings speed sweep
(`speed_sweep.raw_motor` false: MaxSpeed 80, MinSpeed 32, SpeedTarget 100, Scaling off), run
directories `~/dome_bench/speedtarget100/20261016-213926-position` and `20261016-214116-speed`.

**Position sweep** (status complete, 12/12 moves reached, 0 jumps, 3064 samples, largest gap
67 ms). The stop curve now covers the full range, 57 to 394 deg/s: `d = 0.41 s x v - 14 deg`
with R squared 0.93 (17 deg from 57 deg/s, 50 from 183, 74 to 90 from 258, 143 to 162 from
367 to 394; five of the eight steps crossed 0 deg somewhere, which inflates them by up to
9 deg). The afternoon's direct-serial curve was `0.47 s x v - 18 deg`. Moves: +7, -10, +7, -8,
+1, -10, +4, -5, +2, -11, +3, -7 (mean |e| 6.3, max 11; this time the angle-decreasing moves,
ending at 0 to 31 deg, were the worse ones), peak 55 to 106 deg/s, arrival 33 to 93 deg/s.
**Position-move behaviour is unchanged by SpeedTarget**, as the per-move speed probe predicted:
the three runs at 60 gave mean |e| 4.9, 7.0 and (per-move 100) 6.0, this one 6.3.

**Speed sweep at the operating settings** (status complete, 0 jumps, 4223 samples):

| direction | `min_percent` | 100 % (rad/s) | fit k (rad/s per %) | p0 (%) | R squared |
|---|---|---|---|---|---|
| positive percent (angle decreasing) | 32 (bracketed) | 6.79 | 0.0851 | 18.2 | 0.998 |
| negative percent (angle increasing) | 32 (bracketed) | 6.58 | 0.0837 | 18.3 | 0.994 |

Phase 7 inputs from this run (`phase7` in the JSON): `max_speed_rad_per_sec` 6.58 (the lower
measured 100 % rate; the fit value is 6.84), `positive_velocity` percent_sign -1, k 11.94 % per
rad/s, p0 18.3, min_percent 32; `negative_velocity` percent_sign +1, k 11.75, p0 18.2,
min_percent 32. Compared with the calibration reference (SpeedTarget 100 then too, before the
MinSpeed change): the same top speed, k within 4 %, p0 lower by 2 to 4 % because the
32 % floor now zeroes everything below it. Asymmetry 1.018.

**Velocity controller.** `dome_controller` with a 2.5 rad/s command (80 % through the plugin's
default 3.14 rad/s linear mapping) ran the dome at 5.06 rad/s: the full range is back
(3.5 rad/s was the ceiling at SpeedTarget 60).

**Incident (operator error, not the robot).** The validation script killed "the bench node"
by a `pgrep -f` pattern that also matched the shell the script itself had been started from;
that shell's cleanup took the zenoh router down twenty seconds into the velocity-controller
check. Every `ros2` CLI call afterwards hung and the zero command never reached the
controller: the dome kept turning at 80 % for about two minutes until `stop_all.sh` sent
SIGINT to the controller manager, whose deactivation `:DPR0` stopped it (confirmed over direct
serial: angle steady). Lessons: the velocity path has no watchdog of its own, so a human at the
robot and a stop that does not depend on the ROS graph (SIGINT to the controller manager, or
`:DPR0` over the serial port) are the safety net; never run a velocity command from a script
whose stop relies on the CLI; and kill bench nodes by an exact pid, never by a pattern.
