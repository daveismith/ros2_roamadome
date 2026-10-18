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
