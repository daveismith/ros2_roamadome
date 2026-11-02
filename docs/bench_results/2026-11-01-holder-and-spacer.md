# 2026-11-01: dome spacer, the Inverted fault, and the reprinted holder

The user added a spacer inside the dome to cure the sensor 1 skips of 2026-10-17 (reported on
PR #25, `2026-10-17-ring-teleop-sensor1.md`), then reprinted the dome holder with a permanent
spacing change. This report covers both setups. **The reprinted holder is the current state**,
and its numbers are the calibration reference. Run directories are under `~/dome_bench/`.

Firmware throughout: MaxSpeed 80, MinSpeed 20, SpeedTarget 100, DecelerationScale 250, Fudge 5,
Scaling off, HomePos 128 (the user re-homed after 2026-10-17). HomePos means controller angles
are offset from ring angles; in this report "ring angle" is the raw ring reading and "reported
angle" is the home-relative one.

## Result

| | Sensor 1 baseline (10-17) | Temporary spacer | Reprinted holder |
|---|---|---|---|
| Filtered angle jumps over 25 deg, slow pass | 140 | 0 | 0 |
| Controller reported jumps, slow pass | 12 | 0 | 0 |
| Gate held / rejected / confirmed | 25 / 10 / 15 | 0 / 0 / 0 | 0 / 0 / 0 |
| Longest raw misread (samples) | 4 | 1 | 2 |
| Sensor sweep | passed, but blind to this fault | 186 revs, 0 jumps | 164 revs, 0 jumps |
| Position rest error, mean / max (deg) | 3.4 / 8 | 5.6 / 17 | 2.4 / 8 |
| Top speed measured (rad/s) | 6.75 / 6.62 | 7.28 / 7.02 | 6.79 / 6.70 |
| Firmware sensor errors | 6 in 10 min of teleop | 6 to 31 | 0 |

The skips are gone on both setups. The firmware sensor error counter may reset with a launch
restart, so the 0 is not strictly comparable with the spacer's count.

## How the skips were tested

The sensor sweep cannot see this fault: at sweep speeds a sensor crosses a weak spot in
milliseconds, and the 2026-10-17 sweep passed while slow teleop was skipping. The test is slow
travel with the ring's raw stream recorded on `/dev/ttyUSB0` (`#DPDEBUG1`):

- `~/dome_bench/scripts/record_ring_raw.py` records the stream.
- `~/dome_bench/scripts/analyze_ring_teleop.py` counts faults. On the 2026-10-17 recording it
  reproduces that report's 94 raw steps, 25 invalid codes, 140 filtered jumps and 12 controller
  jumps exactly, so the comparison is like for like.
- `~/dome_bench/scripts/slow_pass_supervise.py` runs a driver with a backstop that does not
  depend on ROS: if the driver ends abnormally and the ring still shows motion, it sends SIGINT
  to `ros2_control_node`. A script drove these passes rather than a person, so a teleop session
  is still the last check.

Under `dome_bench:=true` the passes use `~/bench/drive`, with its 1.5 s heartbeat. On a normal
launch they publish to `/cmd_vel_dome_foxglove`, whose converter times out after 0.5 s. On a
normal launch the personality must be paused: claim `/expression/claim/dome` (priority 200, lease
60 s, renewed), then release with lease 0. In bench mode the converter is not running, so the
personality cannot reach the dome, but the claim was held anyway.

## Reprinted holder

Raw CSVs: `2026-11-01-holder-ring-{slow,crawl}-raw.csv`. Sweep JSONs:
`2026-11-01-holder-{speed,speed-operating,sensor,position}.json`.

| Test | Revolutions | Filtered jumps | Controller jumps | Notes |
|---|---|---|---|---|
| Slow pass: 20, 25, 35 % and 60 stop-and-go steps | 21.6 | 0 | 0 | 20 % stalled after 83 and 140 deg |
| Crawl: 22 and 20 % with a 30 % kick after each stall | 17.2 | 0 | 0 | 0.38 to 0.62 rad/s, all 36 bins |
| Sensor sweep, threshold 586 deg/s | 164 | | 0 | `suggested_pass` true; two stalls at 20 % |

- **Raw glitches.** All implausible raw steps but one are the one-sample races at ring angle
  264 (sensors 1 and 3, intermediate code 154) and 225 (sensors 0 and 2, intermediate 114),
  nearly always with the angle falling. One excursion lasted 2 samples. The ring's filter
  removes them all.
- **Parked on an invalid code.** Three runs of 12 to 19 samples (1 to 2 s) of invalid code while
  parked near ring angles 341 and 126. The filter held the angle.
- **Stalls.** At 20 to 22 % the dome stalled 15 times in about 3.5 min of crawl, at ring angles
  92 to 124 and 231 to 233. `min_percent` is marginal there.

### Velocity map

Operating-settings sweep (`2026-11-01-holder-speed-operating.json`). `min_percent` is the same in
the raw and operating runs. `max_speed_rad_per_sec` is the lower measured 100 % rate as the README
says, not `phase7.max_speed_rad_per_sec` (6.87), which is a fit extrapolation.

| Parameter | Previous positive / negative | New positive / negative |
|---|---|---|
| `k_percent_per_rad_s` | 12.75 / 12.36 | 12.90 / 12.46 |
| `p0_percent` | 13.9 / 14.5 | 11.38 / 12.89 |
| `min_percent` | 20 / 22 | 20 / 22 |
| `max_speed_rad_per_sec` | 6.62 | 6.70 |

The new values have **not** had an acceptance run on the hardware: they were derived from the
sweep, and the acceptance check at 1.5 and 3.0 rad/s needs a normal launch with them installed.

**The map raises the speed below which the plugin sends 0.** The plugin rounds the percent, and a
result below `min_percent` becomes 0 (`below_minimum` zero). That threshold moves from 0.44 / 0.57
rad/s to **0.63 / 0.69 rad/s**. The personality's minimum, `min_speed_rad_s` in r2_expression and
`personality_min_speed_rad_s` in r2_bringup, is 0.6 rad/s, so a personality command at its floor
would now send 0 and the dome would not move. Raise it above 0.69, or reconsider
`below_minimum`, before running the personality on this map. Its measured `floor` and `coast`
values (2026-10-23) also predate both hardware changes.

### Position sweep

`2026-11-01-holder-position.json`: 12 of 12 moves, no firmware timeouts. Rest errors -5 -1 +8 -2 +2
+1 +5 +1 -1 +1 +1 -1; mean 2.4, max 8 (3.7 with the angle increasing, 1.2 decreasing), the best
measured. Stop curve `d = 0.54 s x v - 23 deg` (R squared 0.75).

### Unexplained report gap

Position reports stopped for 17.9 s right after the raw sweep restored MaxSpeed
(`Updated` at 1793591392.96, then the gap warning), with the dome at rest at ring angle 283. The
next sensor sweep was refused ("no telemetry"), and it was re-run afterwards with the threshold
set by parameter. No cause found. It is not a parked-on-invalid-code case. A 10.5 s gap also
stopped a sweep on 2026-10-17.

## Temporary spacer (superseded)

Runs: `2026-11-01-spacer`, `-spacer-crawl-fast`, `-spacer-sweeps`, `-inverted-position-2`. No skips, but
much lower friction: floor 0.51 to 0.81 rad/s at 20 %, 100 % = 7.28 / 7.02 rad/s, map values k 12.49 /
12.04, p0 10.24 / 11.50, and rest error mean 5.6, max 17 with overshoot growing with move length
when the angle increases. JSONs: `2026-11-01-spacer-{speed-operating,position}.json`. The holder
brought the friction back, so these values were never configured.

## The Inverted fault

The first position sweep on the spacer failed: 0 of 12 moves, rest errors 161 to 179 deg. Firmware
`Inverted` was false; it was true on 2026-10-17. `#DPSETUP` re-detects it from observed motion and
writes it, and the user may have triggered that. Inverted applies to the position loop but not to
`:DPR` (stop-curve speeds at 35 to 100 % match before and after, in direction and within about
2 %), so velocity driving looked normal and only position moves showed it.

- With Inverted wrong the loop drives away from the target and settles rocking near target + 180.
- The firmware's position timeout needs the angle to stop changing, so a hunting move never ends.
  The dome kept rocking about 3 min after the sweep finished, with `commanded_percent` 0.
- `:DPR0` does not end it. Commanding the dome's own current position does, because the move is
  reached at once: `~/dome_bench/scripts/end_hunting_move.py`.
- Fixed with `#DPINVERT1` over `/dev/ttyACM0` with the launch stopped; `device.inverted` is
  read-only. **Check `device.inverted` before any `:DPA`, and after any `#DPSETUP`.**

A related stall: a move that stopped 7 deg past its target, outside Fudge, stayed outstanding for
over a minute with neither `REACHED TARGET` nor a timeout, and blocked bench drives until cleared
the same way. Another short move stopped 25 deg short, though the next command may have cut it off.
Neither recurred in the sweeps; the cause is unknown.
