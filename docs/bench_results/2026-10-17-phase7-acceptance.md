# 2026-10-17: phase 7 velocity mapping, acceptance on the dome

The measured velocity map (`velocity_map.*` from the 2026-10-16 operating-settings speed sweep:
positive joint velocity -> negative percent, k 11.94 % per rad/s, p0 18.3 %, min 32 %; negative
velocity -> positive percent, k 11.75, p0 18.2, min 32; `below_minimum` zero) driven through
`dome_controller` (forward velocity controller) on the standalone dome stack, dome installed,
firmware MaxSpeed 80, MinSpeed 32, SpeedTarget 100, DecelerationScale 250, Fudge 5. Each command
was published at 10 Hz for about 5 s, then 0 for 4 s; the telemetry velocity was averaged over
the last 2.3 s of each command window. Scripts: `~/dome_bench/scripts/phase7_validate*.sh` and
`phase7_analyze.py`.

## Run 1: the acceptance points plus the floor and the top

| command (rad/s) | percent sent | telemetry (rad/s) | fit (rad/s) | error |
|---|---|---|---|---|
| +1.5 | -36 | +1.443 | 1.500 | -3.8 % |
| -1.5 | +36 | -1.225 | -1.500 | -18.4 % |
| +3.0 | -54 | +3.049 | 3.000 | +1.6 % |
| -3.0 | +53 | -3.112 | -2.996 | +3.9 % |
| +1.0 | 0 | 0.000 | 0 (below the 32 % floor: 30.2 %) | by design |
| +7.0 | -100 | +6.565 | 6.84 (fit), 6.58 measured at 100 % | -0.2 % of the measured top |

The sign convention is now the joint's: a positive command turns the joint positive (the
reported angle increases), and the percent sent carries the measured sign (-36 for +1.5).

## Run 2: the 1.5 rad/s point repeated

| command | percent | telemetry | error |
|---|---|---|---|
| -1.5 | +36 | -1.287 | -14 % |
| +1.5 | -36 | +1.190 | -21 % |
| -1.5 | +36 | -1.290 | -14 % |
| +1.5 | -36 | +1.037 | -31 % |

## Run 3: the four points again on the final build, after a ten-minute rest

| command | percent | telemetry | error |
|---|---|---|---|
| +1.5 | -36 | +1.425 | -5.0 % |
| -1.5 | +36 | -1.556 | +3.7 % |
| +3.0 | -54 | +3.020 | +0.7 % |
| -3.0 | +53 | -3.078 | +2.6 % |

## Verdict

- **3.0 rad/s: pass** in both directions in every run, within 4 %.
- **1.5 rad/s: passes in some runs, not in others.** Eight windows at 36 % gave 1.04 to
  1.56 rad/s; four were within the 10 % band, four were 14 to 31 % slow, and the slow ones
  were the middle runs after an hour of continuous dome use. 36 % is only four points above the
  32 % floor, where the speed is set by the dome's position-dependent friction (and, it seems,
  its thermal state) rather than by the command. The speed sweep itself shows the same scatter
  at its low end (32 % measured 1.05 and 0.89 rad/s against a fit of 1.15, 35 % measured 1.35
  and 1.43 against 1.40; from 40 % up every step is within 5 % of the fit).
- The mapping is therefore validated from about 2 rad/s (42 %) upward, and marginal at
  1.5 rad/s. Below that the command is honoured in sign and ordering but the speed is
  friction-limited; the planned lubrication is the lever, after which the operating-settings
  speed sweep should be re-run and 1.5 rad/s re-checked.
- 1.0 rad/s maps to 30 %, below the floor, and sends 0 by design (`velocity_map.below_minimum`
  zero); `minimum` would send 32 % and run the dome at about 1 rad/s instead.
