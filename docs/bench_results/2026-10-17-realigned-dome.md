# 2026-10-17 evening: sweeps after the dome realignment

The user adjusted the dome setup (seating / alignment, "it may have re-aligned sticky or skip
locations"). All four bench sweeps were re-run on the standalone stack (`~/dome_bench/stack`,
every process at nice 0, plausibility gate at 600 deg/s), firmware MaxSpeed 80, MinSpeed 32,
SpeedTarget 100, DecelerationScale 250, Fudge 5, Scaling off. Run directories under
`~/dome_bench/2026-10-17-realigned/`; the JSONs are next to this file
(`2026-10-17-realigned-{speed,sensor,speed-operating,position}.json`). Script:
`~/dome_bench/scripts/run_full_sweeps.sh`.

## Sensor sweep (skips): pass

Raw motor settings, 20 / 35 / 50 / 75 / 100 % both ways, 30 s each, threshold 542 deg/s
(1.5 x the raw sweep's maximum). **0 jumps, 0 reversals, 0 stalls**, 8137 samples, largest
gap 67 ms, `suggested_pass` true; the plugin gate held, rejected and resynced nothing. No
skip anywhere on the ring after the realignment.

## Speed sweeps: same top speed and floor, less friction at the low end

| | raw motor, 2026-10-16 ref | raw motor, today | operating, 2026-10-16 ref | operating, today |
|---|---|---|---|---|
| `min_percent` (+ / -) | 30 / 27 | 32 / 32 | 32 / 32 | 32 / 32 |
| 100 % (rad/s, + / -) | 6.70 / 6.70 | 6.21 / 6.31 | 6.79 / 6.58 | 6.55 / 6.56 |
| k (rad/s per %, + / -) | 0.0883 / 0.0872 | 0.0731 / 0.0740 | 0.0851 / 0.0837 | 0.0762 / 0.0769 |
| p0 (%, + / -) | 22.1 / 20.5 | 13.4 / 14.0 | 18.2 / 18.3 | 12.8 / 13.0 |
| R squared | 0.994 / 0.997 | 0.999 / 0.999 | 0.998 / 0.994 | 0.998 / 0.998 |

- The fit is straighter than ever (R squared 0.998 to 0.999) with a **lower p0 and a lower k**:
  the dome now runs faster at low percents (32 %: 1.30 rad/s both ways, against 1.05 / 0.89 on
  the 16th; 40 %: 2.1 against 1.9) and the same at the top (6.55 / 6.56 rad/s). Less friction,
  as the adjustment intended. The floor stays at 32 % (30 % did not move in either run).
- The raw-motor run, the first of the session, came out 6 % slower at 100 % than the
  operating run twelve minutes later (6.21 / 6.31 against 6.55 / 6.56) with the same floor.
  MaxSpeed does not scale `:DPR` (known), so this is the dome warming up, as the acceptance
  runs on the velocity map also suggested; take the operating run as the calibration.

**Phase 7 values from the operating run** (`phase7` in the JSON; these replace the 16 October
values in the README and the r2_bringup URDF):

- `max_speed_rad_per_sec` **6.55** (the lower measured 100 % rate; fit 6.65).
- `velocity_map.positive`: `percent_sign` -1, `k_percent_per_rad_s` **13.00**, `p0_percent`
  **13.0**, `min_percent` 32.
- `velocity_map.negative`: `percent_sign` +1, `k_percent_per_rad_s` **13.12**, `p0_percent`
  **12.8**, `min_percent` 32.

With these, +1.5 rad/s sends -33 %, +3.0 sends -52 %; -1.5 sends +32 % (at the floor) and
-3.0 sends +52 %. The slowest commandable speed is now about 1.46 rad/s (32 %), up from 1.15.

## Position sweep (accuracy): a little more coasting

Stop curve now `d = 0.50 s x v - 29 deg` (R squared 0.95; 26 to 33 deg from 86 to 114 deg/s,
47 to 57 from 173 to 181, 87 to 90 from 239 to 270, 160 to 164 from 355 to 375), against
`0.41 s x v - 14` on the 16th: the dome coasts further for the same speed, again less friction.

Twelve moves, all completed with `REACHED TARGET`, no firmware timeouts: rest errors +6, -10,
+11, -5, +9, -8, +9, -4, +8, -9, +13, -7 (mean |e| 8.2 deg, max 13; angle increasing 9.3,
decreasing 7.2), arrival at the fudge band 50 to 91 deg/s, every move overshooting. On the 16th
the same settings gave mean 4.9 to 7.0. Expected: the floor speed is higher now (less friction
at 32 %) and the coast longer, so with Fudge 5 the overshoot grows. The tuning levers are the
ones from the fudge study, and the realignment moves them in the right direction: `MinSpeed`
below 32 stalled at the tight spots on the 16th (25: 2 of 18, 28: 1 of 18); with the friction
reduced a re-run of the position sweep at MinSpeed 28 and 25 is the next step, and a lower
floor also lowers the velocity map's minimum. Not done tonight.

## Phase 7 acceptance with the new values

Velocity controller on the standalone stack, the new map in the URDF, each command for 5 s,
telemetry averaged over the last 2.3 s of the window:

| command (rad/s) | percent sent | telemetry (rad/s) | error |
|---|---|---|---|
| +1.5 | -33 | +1.296 | -14 % |
| -1.5 | +32 | -1.205 | -20 % |
| +3.0 | -52 | +3.060 | +2 % |
| -3.0 | +52 | -3.075 | +3 % |
| +1.5 | -33 | +1.282 | -15 % |
| -1.5 | +32 | -1.158 | -23 % |

**3.0 rad/s passes** (within 3 % both ways). **1.5 rad/s fails** consistently now: with the
lower p0 it maps to 32.5 %, the floor itself, and the floor runs 1.16 to 1.30 rad/s (the speed
sweep's 32 % steps measured 1.30 both ways while the fit says 1.46: the lowest points sit below
the line). So the slowest commandable speed is about 1.3 rad/s, and the fit is trustworthy from
about 2 rad/s (39 %) upward, as on the 16th. The way to bring 1.5 rad/s inside the band is a
lower floor: with the friction reduced, `MinSpeed` 28 or 25 may no longer stall at the tight
spots (they did on the 16th), which would also let the velocity map's `min_percent` drop; run
the position sweep and the operating-settings speed sweep at the candidate MinSpeed to find out.

## MinSpeed study: 28, 25, 20, 15 (then set to 20)

With the friction reduced, `MinSpeed` was lowered in steps, each with a position sweep (the
approach floors at MinSpeed, so a floor too low stalls or stops short at the tight spots) and
an operating-settings speed sweep (the velocity floor). JSONs `2026-10-17-minspeed-{25,20}-*`
and `-15-position` next to this file; run directories `~/dome_bench/2026-10-17-minspeed*`.
The rule agreed with the user: go down until it breaks, then set the multiple of 5 above.

| MinSpeed | position sweep: rest errors (deg) | mean / max | speed sweep floor (+ / -), speed there |
|---|---|---|---|
| 32 (before) | +6 -10 +11 -5 +9 -8 +9 -4 +8 -9 +13 -7 | 8.2 / 13 | 32 / 32 %, 1.30 / 1.30 rad/s |
| 28 | +4 -15 +5 -9 +1 -9 +4 -5 +2 -10 +6 -9 | 6.6 / 15 | (sweep aborted by a 10 s report gap, see below) |
| 25 | +1 -9 +3 -5 -2 -5 +4 -3 0 -8 +2 -4 | 3.8 / 9 | 25 / 25 %, 0.77 / 0.59 rad/s |
| **20** | **0 +4 +8 +2 +5 +3 +5 +3 0 +2 +7 +2** | **3.4 / 8** | 22 / 20 %, 0.55 / 0.30 rad/s |
| 15 | -38 -3 -22 0 -43 +27 -3 +5 -39 +24 -16 +28 | 20.7 / 43 | 25 / 20 % (15 % does not move) |

- Every sweep down to 20 completed all twelve moves with `REACHED TARGET` and no firmware
  timeout; the accuracy improves as the floor comes down because the dome arrives at the fudge
  band more slowly (arrival 13 to 55 deg/s at 20, 50 to 91 at 32) and coasts less (stop fit at
  20: `0.37 s x v - 4 deg`).
- **15 is where it breaks**: seven of twelve moves stopped 16 to 43 deg short with no
  `REACHED TARGET` (the approach at 15 % cannot pass the tight spots; the bench's standstill
  detector ended them), and the speed sweep's 15 % steps did not move at all.
- At 20 the positive direction is right at the edge: the speed sweep's +20 % step crawled at
  0.08 rad/s and the floor was found at 22 %, while -20 % ran 0.30 rad/s. The position moves
  are fine because the approach taper keeps them above that until the fudge band.
- **MinSpeed is now 20** (persisted; acked). The velocity map from the MinSpeed-20 operating
  sweep (now in the README and the r2_bringup URDF): `max_speed_rad_per_sec` 6.62 (lower
  measured 100 %; fit 6.75); positive velocity sign -1, k 12.75, p0 13.9, `min_percent` 20;
  negative velocity sign +1, k 12.36, p0 14.5, `min_percent` 22. The slowest commandable speed
  is about 0.5 rad/s now, down from 1.3.
- The MinSpeed 28 speed sweep aborted on a telemetry dropout: the plugin logged a 10.5 s gap
  in position reports while the dome ran at 70 %, with no USB error in the kernel log; reports
  then resumed and the bench's stop went through. Unexplained; the dropout class seen before
  was host stalls of about 1 s. Worth watching: the velocity path has no watchdog, and a
  stalled process cannot stop the dome.

## Phase 7 acceptance at MinSpeed 20 with the MinSpeed-20 map

Same procedure as above, after `device.min_speed` 20 was set and acked (readback 20):

| command (rad/s) | percent sent | telemetry (rad/s) | error |
|---|---|---|---|
| +1.5 | -33 | +1.654 | +10 % |
| -1.5 | +33 | -1.495 | 0 % |
| +3.0 | -52 | +3.141 | +5 % |
| -3.0 | +52 | -3.183 | +6 % |
| +1.0 | -27 | +0.811 | -19 % |
| -1.0 | +27 | -0.677 | -32 % |
| +1.5 | -33 | +1.374 | -8 % |
| -1.5 | +33 | -1.313 | -12 % |

**3.0 rad/s passes** (within 6 %, a little fast this time after an hour of running). **1.5 rad/s
is now at the edge of the band**: three of four windows within 10 %, the spread 1.31 to
1.65 rad/s for the same 33 %. **1.0 rad/s is commandable** (27 %, above the new floor) but runs
0.68 to 0.81 rad/s: the fit's p0 overstates the speed within a few points of the floor, where
the dome's sector-to-sector friction dominates. Compared with MinSpeed 32 the floor has moved
from 1.3 to about 0.5 rad/s and the position moves are twice as accurate; the mapping is
reliable from about 2 rad/s (39 %) upward and indicative below that.
