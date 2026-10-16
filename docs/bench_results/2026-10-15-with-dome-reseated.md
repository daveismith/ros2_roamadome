# Dome bench results, 2026-10-15: DOME INSTALLED, re-seated (repeatability run)

**Purpose.** On 2026-10-14 the [with-dome run](2026-10-14-with-dome.md) failed the sensor sweep:
position spikes at fixed angles that the [no-dome baseline](2026-10-14-no-dome-baseline.md) never
showed. The human then re-seated the dome. This run repeats the same three sweeps with the same
stack, commands and parameters, to answer one question: **were the spikes a one-off seating or
alignment problem, or are they a constant with the dome on?** It also checks how well the speed
calibration repeats.

**Short answer.** The spikes came back, and more often. Every spike from 2026-10-14 and almost every
spike today is a single sensor bit reading **set (1) where the decode table says it should be
clear (0)**. Both days show the same bit (bit 0, `SENSORS[0]`) at the same true angles (198-201 deg
and 232-238 deg). Today a second sensor, bit 1 (`SENSORS[1]`), also failed, at 239 deg and
277 deg. That fault did not appear yesterday. Re-seating did not remove the bit 0 fault, and the
spike rate went up, not down. See [Conclusion](#conclusion).

Phase 6 of [docs/dome_diagnostics_plan.md](../dome_diagnostics_plan.md), run with the human at the
robot. The robot was rebooted a few minutes before the session.

| | |
|---|---|
| Date | 2026-10-15, 20:31 to 20:47 local (-0400) |
| Stack | Same as 2026-10-14: `robot_state_publisher` with the dome-only URDF (`enable_bench_mode`, `publish_telemetry`), `ros2_control_node` with only the RoamADome hardware at 30 Hz, no controllers; `rmw_zenoh_cpp` with a router started for the session |
| Code | `ros2_roamadome` at `33f7ce8` (`diag/phase-6-with-dome`). The bench node and analysis are unchanged since `fa97b3b`, the code used for both 2026-10-14 runs. `#DPREPORT28` |
| Bench node | `ros2 run ros2_roamadome roamadome_bench --ros-args -p output_dir:=/home/pi/dome_bench/phase6-dome-2 -p hardware_node:=roamadome` with plan 5.4 defaults; the third run adds `-p speed_sweep.raw_motor:=false`. Identical to 2026-10-14 apart from `output_dir` |
| Results | [2026-10-15-with-dome-reseated-speed.json](2026-10-15-with-dome-reseated-speed.json), [2026-10-15-with-dome-reseated-sensor.json](2026-10-15-with-dome-reseated-sensor.json), [2026-10-15-with-dome-reseated-speed-operating.json](2026-10-15-with-dome-reseated-speed-operating.json) (copies of the bench node's result files, unmodified; the operating run **aborted**, see below) |
| Raw data (not committed) | `/home/pi/dome_bench/phase6-dome-2/`: `20261015-203255-speed/`, `20261015-203716-sensor/`, `20261015-204300-speed/` (each with `samples.csv`, `run.json`, `settings_before.json`, `settings_after.json`); `bench_*.log`, `cm.log`, `baseline_params.yaml`, `/diagnostics` captures, `check_<run>.txt`, `notes.txt`; `analysis/` holds the offline spike and bit analysis scripts and their output (see [Method](#method-of-the-spike-and-bit-analysis)) |

## Runs

| Run | Sweep | Settings during the run | Status | Settings restored |
|-----|-------|-------------------------|--------|-------------------|
| `20261015-203255-speed` | Speed, plan defaults (12 percents, `raw_motor` true) | MaxSpeed 100, SpeedTarget 100, Scaling off | complete, 27 steps, 3 min 0 s | yes, firmware acked (`#DPMAXSPEED80`), `settings_restored: true` |
| `20261015-203716-sensor` | Sensor, plan defaults (20/35/50/75/100, 30 s, both directions, `raw_motor` true) | MaxSpeed 100, SpeedTarget 100, Scaling off | complete, 10 steps, 4 min 34 s | yes, firmware acked, `settings_restored: true` |
| `20261015-204300-speed` | Speed, `raw_motor` false (operating settings) | MaxSpeed 80, SpeedTarget 100, Scaling off | **aborted** after 2.5 steps (15 s): `telemetry dropout: no message for 1.03 s` | nothing written, `settings_restored: true`, `firmware_acked: true` (0 writes) |

The sensor sweep took its threshold and minimum percentages from today's speed run:
`max_plausible_deg_s` 579.67 = 1.5 x 6.7447 rad/s, `max_plausible_source` `1.5 x measured maximum
in /home/pi/dome_bench/phase6-dome-2/20261015-203255-speed/speed_sweep.json`, `min_percent_known`
27 / 25 from the same file. `roamadome_bench_report <run_dir> --check` reproduces all three result
files (exit 0, "result matches"). After every run `ros2 param get` returned `device.max_speed` 80,
`device.speed_target` 100, `device.scaling` False. At the end, `min_speed` was 15, `inverted` True,
`home_pos` 0 and `auto_safety` False (never written).

**The operating-settings run aborted.** During the +15 % step at 20:43:09 the bench node got no
telemetry message for 1.03 s and aborted, as designed. It commanded 0 and found nothing to restore.
`settings_after.json` has `firmware_acked: true`. The dome stopped: it was idle at 35 deg with
`commanded_percent` 0. Telemetry was back at 33.8 Hz when checked a minute later. The controller
manager log has no warning, the kernel log has nothing for that minute and `position_timeouts`
stayed 0. The cause was not found. The dropout coincided with offline analysis running on the Pi
at the same time: a CPU-heavy re-analysis of yesterday's CSVs and a recursive `grep` over
`/home/pi`. The 1-minute load average was 5.6 just afterwards. The bench node and the controller
manager were running at nice 19, inherited from the shell that started them. Following the
session's safety rule for a telemetry stop, a final `percent: 0` was sent
(`bench drive already stopped`), the controller manager was stopped, and the run was **not
repeated**. The operating-settings `min_percent` (32 on 2026-10-14) is therefore **not
re-measured today**. Before the abort, ±10 % did not move (at 151 deg), and +15 % turned 89 deg
(151 to 62 deg) in 3.9 s. Yesterday +15 % did not move.

## Firmware configuration (`device.*` at the start of the session)

`ros2 param dump /roamadome`, **identical to both 2026-10-14 sessions** and unchanged at the end:

```yaml
acceleration_scale: 20
auto_left: 80
auto_max_delay: 8
auto_min_delay: 6
auto_mode: false
auto_restart: true
auto_right: 80
auto_safety: false
deceleration_scale: 50
digital_out: 0
fudge: 5
home_max_delay: 8
home_min_delay: 6
home_mode: false
home_pos: 0
input_speed: 100
inverted: true
max_speed: 80
min_speed: 15
pwm_arc_mode: false
pwm_deadband: 5
pwm_in: false
pwm_max_pulse: 2000
pwm_min_pulse: 1000
pwm_neutral_pulse: 1500
pwm_out: false
scaling: false
sensor_baud: 115200
serial_baud: 9600
serial_in: true
serial_out: true
setup_angular_velocity: 100
speed_auto: 30
speed_home: 40
speed_target: 100
syren_address_in: 129
syren_address_out: 129
syren_baud: 9600
target_max_delay: 1
target_min_delay: 0
timeout: 5
```

## Speed sweep (`20261015-203255-speed`, raw motor)

Columns as in the 2026-10-14 report. The last column is today's offline re-analysis of the same
step after the spike samples are removed (see [Method](#method-of-the-spike-and-bit-analysis)); a
value in parentheses belongs to a step that did not move. Velocities are signed in the
reported-angle sense.

| # | Kind | Percent | Samples | Moving | Stalled (after s) | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Jumps | Max gap s | Slope, spikes removed |
|---|------|--------:|--------:|:------:|------------------:|----------:|-----:|------------:|-----------:|----------:|------:|----------:|------:|
| 0 | step | +10 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0 | 0.033 | (0.000) |
| 1 | step | -10 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0 | 0.233 | (0.000) |
| 2 | step | +15 | 106 | no | yes (3.2) | 1.2 | 0.00 | 0.000 | 0.000 | 0.000 | 0 | 0.033 | (0.000) |
| 3 | step | -15 | 70 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0 | 0.033 | (0.000) |
| 4 | step | +20 | 98 | no | yes (2.9) | 0.9 | 0.00 | 0.000 | 0.000 | 0.000 | 0 | 0.033 | (0.000) |
| 5 | step | -20 | 181 | no | yes (5.5) | 3.5 | 0.06 | 0.096 | 0.115 | 0.101 | 0 | 0.067 | (0.096) |
| 6 | step | +25 | 281 | no | yes (9.0) | 6.9 | 0.30 | -0.363 | -0.359 | 0.104 | 3 | 0.067 | (-0.369) |
| 7 | step | -25 | 266 | yes | no | 6.8 | 0.61 | 0.559 | 0.569 | 0.173 | 1 | 0.067 | 0.559 |
| 8 | step | +30 | 186 | yes | no | 4.1 | 0.56 | -0.820 | -0.919 | 0.345 | 2 | 0.067 | -0.836 |
| 9 | step | -30 | 191 | yes | no | 4.3 | 0.69 | 1.034 | 1.031 | 0.203 | 2 | 0.067 | 1.034 |
| 10 | step | +40 | 187 | yes | no | 4.0 | 1.14 | -1.651 | -1.760 | 0.975 | 4 | 0.067 | -1.669 |
| 11 | step | -40 | 188 | yes | no | 4.0 | 0.82 | 0.398 | -0.710 | 5.834 | 6 | 0.233 | 1.924 |
| 12 | step | +50 | 187 | yes | no | 4.0 | 1.75 | -2.772 | -2.751 | 0.740 | 6 | 0.067 | -2.787 |
| 13 | step | -50 | 188 | yes | no | 4.0 | 0.21 | 0.277 | 1.037 | 4.844 | 4 | 0.067 | 2.837 |
| 14 | step | +60 | 189 | yes | no | 4.0 | 1.33 | -2.252 | -2.204 | 4.361 | 4 | 0.067 | -3.676 |
| 15 | step | -60 | 72 | no | yes (2.3) | 0.3 | 0.15 | 3.481 | 3.442 | 0.105 | 2 | 0.033 | 3.481 |
| 16 | step | +70 | 187 | yes | no | 4.0 | 2.82 | -4.436 | -4.413 | 0.245 | 0 | 0.033 | -4.436 |
| 17 | step | -70 | 186 | yes | no | 4.0 | 2.82 | 4.426 | 2.965 | 4.540 | 4 | 0.033 | 4.426 |
| 18 | step | +80 | 184 | yes | no | 4.0 | 3.31 | -5.186 | -5.167 | 0.154 | 2 | 0.067 | -5.186 |
| 19 | step | -80 | 185 | yes | no | 4.0 | 3.33 | 5.253 | 5.264 | 0.180 | 0 | 0.067 | 5.253 |
| 20 | step | +90 | 182 | yes | no | 4.0 | 3.87 | -6.074 | -6.030 | 1.099 | 1 | 0.233 | -6.081 |
| 21 | step | -90 | 183 | yes | no | 4.0 | 3.78 | 6.017 | 5.982 | 0.224 | 0 | 0.067 | 6.017 |
| 22 | step | +100 | 183 | yes | no | 4.0 | 4.30 | -6.745 | -6.702 | 0.406 | 1 | 0.067 | -6.743 |
| 23 | step | -100 | 182 | yes | no | 4.0 | 4.21 | 6.639 | 6.593 | 0.163 | 0 | 0.033 | 6.639 |
| 24 | search | +27 | 351 | yes | no | 9.2 | 0.99 | -0.770 | -0.668 | 0.637 | 6 | 0.067 | -0.762 |
| 25 | search | -22 | 267 | no | yes (8.5) | 6.5 | 0.39 | 0.402 | 0.382 | 0.253 | 0 | 0.067 | (0.402) |
| 26 | search | -23 | 227 | no | yes (6.9) | 4.9 | 0.19 | 0.251 | 0.256 | 1.934 | 2 | 0.067 | (0.208) |

The bench node's own result is distorted by the spikes. It reports 50 jumps against 2 on
2026-10-14 and `angle_sign_consistent` **false**: one step voted the wrong way (positive 1,
negative 22), because a spike shifted its unwrapped angle by a full turn. The R squared values are
0.9649 / 0.8959. In the negative direction `stalled_above_min_percent` is `[60]`, and that fit
(k 0.0923, p0 27.3) excludes the -60 % step and includes the spike-distorted -40 % and -50 % steps
(slopes 0.398 and 0.277 rad/s, std 5.8 and 4.8). Two of its stall decisions are spike artefacts:

- **-60 % (step 15)** is not a real stall. During settle, the reading went 189 -> 13 -> 201 deg
  (bit 0 fault, true ~197 deg). The shortest-distance unwrap counted that as -360 deg, which
  cancelled the turn the dome had made. The 2 s stall window therefore saw no net forward motion,
  the same mechanism as step 17 (-70 %) of the 2026-10-14 operating run.
- **+25 % (step 6) is a real stall at true 238 deg.** The dome stopped there, and at rest the
  sensor read 276 deg (bit 0 set) **continuously for about 25 reports (0.83 s)**, across the step
  6/7 boundary, until the -25 % step moved the dome. At this angle the fault holds whenever the
  dome stands still, so it is not a passing glitch.

The bench node's `phase7` block in the committed JSON (positive velocity k 10.84, p0 27.30, min 25;
negative velocity k 11.78, p0 20.58, min 27) **must not be used**. For the calibration comparison
see [Speed repeatability](#speed-repeatability), which uses the re-analysis with spikes removed and
the live stall decisions.

## Sensor sweep (`20261015-203716-sensor`, raw motor)

Jump threshold 579.67 deg/s. "Slope, spikes removed" and "Revs, spikes removed" are the offline
re-analysis.

| # | Percent | Samples | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Jumps | Reversals | Stalls | Scored | Same-cycle pairs | Max gap s | End | Slope, spikes removed | Revs, spikes removed |
|---|--------:|--------:|----------:|-----:|------------:|-----------:|----------:|------:|----------:|-------:|:------:|-----:|------:|-----|------:|------:|
| 0 | +20 | 274 | 6.6 | 0.44 | -0.483 | -0.410 | 0.297 | 4 | 0 | 0 | no | 21 | 0.067 | stalled | -0.483 | 0.44 |
| 1 | -20 | 190 | 3.9 | 0.13 | 0.186 | 0.237 | 0.196 | 0 | 0 | 0 | no | 18 | 0.067 | stalled | 0.186 | 0.13 |
| 2 | +35 | 984 | 30.0 | 6.80 | -1.435 | -1.425 | 0.526 | 17 | 1 | 0 | yes | 63 | 0.233 | completed | -1.435 | 6.80 |
| 3 | -35 | 666 | 19.5 | 4.71 | 1.511 | 1.506 | 0.654 | 16 | 0 | 1 | yes | 34 | 0.067 | stalled | 1.513 | 4.71 |
| 4 | +50 | 996 | 30.0 | 13.60 | -2.864 | -2.860 | 0.563 | 30 | 0 | 0 | yes | 59 | 0.233 | completed | -2.864 | 13.60 |
| 5 | -50 | 1000 | 30.0 | 6.36 | 1.352 | 1.391 | 4.262 | 28 | 0 | 0 | yes | 50 | 0.067 | completed | 2.797 | 13.36 |
| 6 | +75 | 979 | 30.0 | 22.77 | -4.773 | -4.769 | 0.331 | 2 | 0 | 0 | yes | 34 | 0.233 | completed | -4.773 | 22.77 |
| 7 | -75 | 983 | 30.0 | 22.88 | 4.803 | 4.796 | 0.147 | 0 | 0 | 0 | yes | 34 | 0.067 | completed | 4.803 | 22.88 |
| 8 | +100 | 963 | 30.0 | 32.17 | -6.743 | -6.732 | 0.426 | 5 | 0 | 0 | yes | 17 | 0.233 | completed | -6.743 | 32.17 |
| 9 | -100 | 964 | 30.0 | 31.41 | 6.592 | 6.580 | 0.277 | 0 | 0 | 0 | yes | 29 | 0.067 | completed | 6.592 | 31.41 |

Totals: 7999 samples, **102 jumps** (2026-10-14: 6), 1 reversal, 1 stall (scored),
**`suggested_pass` false**. The bench node's jump start histogram (10 deg bins, by the angle
*before* the jump, so each spike shows twice: once where it starts and once at the wrong angle it
returns from): 10-20 deg 22, 30-40 deg 1, 50-60 deg 11, 190-200 deg 11, 200-210 deg 12, 230-240
deg 18, 240-250 deg 3, 250-260 deg 2, 270-280 deg 12, 280-290 deg 5, 310-320 deg 4, 350-360 deg 1.

The scored reversal and stall are both spike artefacts:

- **Reversal, +35 % (step 2), 20:37:49.4**: inside the two-report bit 0 spike 206 -> 11 -> 13 ->
  196 deg, the pairs 206 -> 11 (same controller cycle, so it is not scored as a jump) and 11 -> 13
  counted as two backward pairs.
- **Stall, -35 % (step 3), after 19.5 s**: the dome was turning at about 100 deg/s. The stall
  window's reference sample, exactly 2 s back, was the bit 0 spike 193 -> **12** -> 200 deg. The
  unwrap took 193 -> 12 as +179 deg, so the window saw only 4 deg of net motion.
- **-50 % (step 5)** completed, but its 6.36 revolutions and 1.35 rad/s are wrong. The spikes cost
  it seven full turns of unwrapped angle: 13.36 revolutions and 2.80 rad/s once they are removed.

Both 20 % steps moved and stalled, as on 2026-10-14: +20 % from 202 to 8 deg, -20 % from 8 to 99
deg. The +20 % step started at rest at true 200-202 deg, and two bit 0 spikes appeared in its
first 0.4 s.

## Spike analysis

### Method of the spike and bit analysis

Offline, with the scripts in `/home/pi/dome_bench/phase6-dome-2/analysis/` (`spikes.py`,
`codes.py`, `tables.py`, `speedcmp.py`), applied the same way to every 2026-10-14 and 2026-10-15
run:

- **Excursion detection.** Rows are consecutive firmware reports (`seq` +1 within a step). Reports
  delayed by the ~233 ms config refresh arrive in a burst; none are lost. The detector therefore
  works per report, not per host interval. For each pair it takes the shortest-distance change and
  subtracts the local motion: the median change over the neighbouring ±7 pairs. An excursion starts
  at a residual of **≥ 10 deg** and ends when the residuals sum back to near zero (within
  max(8 deg, 30 % of the outward step)). One or two reports count as a *spike*; three or more as
  *held*. This also catches out-and-back spikes **below the jump threshold**, which the bench node
  does not score. Per step only: reports between steps are not recorded. The one excursion that
  crossed a step boundary is joined by hand.
- **True angle.** Linear interpolation between the reports before and after the excursion.
- **Bit classification.** From the Reeltwo decode table `sDomeAngle[]` in `DomeSensorRing.h`
  (index = 9-bit mask, bit i = `digitalRead(SENSORS[i])`; every angle 0..359 has exactly one code).
  For each wrong reading, every angle on the path from the report before to the report after, ±2
  deg, is tested as the true angle. The script reports the candidate whose code is closest in
  Hamming distance to the code of the reading, plus the differing bits and whether they read set
  or cleared. The ±2 deg margin is there because the firmware's median-of-5 output lags the decoded
  angle by up to two ring samples. **Caveat:** with about 10 candidates there is roughly an 18 %
  chance per spike that some candidate is one bit away from an arbitrary reading. A single-bit
  match on one spike proves little. The evidence is that the same true -> read pairs repeat dozens
  of times.
- **Band passes.** After the excursion samples are removed, the unwrapped trajectory is scanned for
  full traversals of 230-280 deg and 190-210 deg, entering at one edge and leaving at the other,
  per direction.
- **Validation.** On the 2026-10-14 data the detector gives the counts of that report: raw speed 1
  pattern A spike, sensor 4 (all angle-decreasing), operating 5 A and 10 B. The report gave 9 B: it
  counted the two B excursions 66 ms apart at 22:37:09 as one. The no-dome baseline gives zero.
  Each detector "unreturned" item (one per baseline and per operating run) is stick-slip motion or
  a stuck report, not an out-and-back.

### Bit-level result

Spike faults by bit and true angle. A and B are 2026-10-14's two patterns; C and D are new today.

| Fault | Sensor | True angle (code) | Reads as (code) | 2026-10-14 runs (raw / sensor / operating) | 2026-10-15 runs (raw / sensor) |
|-------|--------|-------------------|-----------------|------------------------------------------|----------------------------|
| A | bit 0, `SENSORS[0]`, set | 235 `111111000`, 236 `111110000`, 237 `111100000`, 238 `011100000`, 232 `111110110` | 274 `111111001`, 275 `111110001`, 276 `111100001`, 283 `011100001`, 353 `111110111` | 1 / 4 / 5 | 13 / 15 |
| B | bit 0, `SENSORS[0]`, set | 198 `001110000`, 199 `000110000`, 200 `000100000`, 201 `010100000` | 13 `001110001`, 12 `000110001`, 11 `000100001`, 284 `010100001` | 0 / 0 / 10 | 13 / 24 |
| C | bit 1, `SENSORS[1]`, set | 239 `001100000` | 52 `001100010` | 0 / 0 / 0 | 0 / 11 |
| D | bit 1, `SENSORS[1]`, set | 277 `111000001` | 316 `111000011` | 0 / 0 / 0 | 0 / 4 |
| other | 3 bits: 4, 7 cleared, 8 set | 193 `011111111` (closest) | 32 `101101111` | 0 / 0 / 0 | 0 / 1 |
| **Total** | | | | **1 / 4 / 15 (20)** | **26 / 55 (81)** |

Codes are printed bit 8 ... bit 0. Every spike from 2026-10-14 is a **single-bit error on bit 0,
reading set**. On 2026-10-15, 80 of 81 are single-bit errors reading set: 65 on bit 0 and 15 on
bit 1. The 81st (20:39:03, -50 %, true ~197 deg reading 32 deg) is three bits from any true
candidate (bits 4 and 7 read clear, bit 8 set). It may be a median-filter mix of two wrong
readings. Apart from that one outlier, no spike on either day has a bit reading **clear** where it
should be set.

**Where the two tracks should read clear** (from the decode table): bit 0 (`SENSORS[0]`) is clear
at 194-211 deg and 227-258 deg, among other ranges. Bit 1 (`SENSORS[1]`) is clear at 234-251 deg
and 267-298 deg, among others. All faults sit **inside** these clear runs, not at their edges:
bit 0 at 198-201 deg of 194-211 deg and at 232-238 deg of 227-258 deg; bit 1 at 239 deg of
234-251 deg and at 277 deg of 267-298 deg. An early or late code transition would show at a run
edge. These are spots in the middle of a run where the sensor intermittently reads 1. The sensors
use `INPUT_PULLUP`, so 1 is also what an undriven input reads. Whether 0 means "marker present"
depends on the sensor board, which the header does not describe. **The physical sensors are
`SENSORS[0]`** (pin 27 on the RP2040 sensor-ring variant; pin 18 on the plain ESP32 variant and
pin 27 on the ESP32 variant with `PIN_NEOPIXEL`, per the Reeltwo header) **and, new today,
`SENSORS[1]`** (pin 28 on RP2040; pin 19 or 25 on the ESP32 variants). The reported angle is
home-relative, and with `home_pos` 0 it equals the sensor angle.

The firmware's median-of-5 filter passes a wrong value only when it fills at least three of the
five buffered samples. The filter appends a sample only when the 9-bit mask changes. So every
spike seen here means the faulty bit misread over several consecutive ring samples, not one
electrical glitch. At rest at 238 deg the misread lasted 0.83 s.

### Rate per pass, compared with 2026-10-14

Passes through the band, by direction of the true angle. Spikes at rest or in stick-slip (stalled
low-percent steps) are listed separately because they have no pass. "Decr." is angle decreasing
(positive percent).

| Run | A spikes / passes 230-280, decr. | A, incr. | B spikes / passes 190-210, decr. | B, incr. | C (bit 1 @239), incr. / decr. | D (bit 1 @277), decr. / incr. | At rest |
|-----|------:|------:|------:|------:|------:|------:|------:|
| No dome, 3 runs | 0 / 140 | 0 / 136 | 0 / 142 | 0 / 138 | 0 | 0 | 0 |
| With dome 1, speed raw | 0 / 23 | 0 / 25 | 0 / 26 | 0 / 24 | 0 | 0 | 1 A (+27 %) |
| With dome 1, sensor | 4 / 76 | 0 / 74 | 0 / 76 | 0 / 75 | 0 | 0 | 0 |
| With dome 1, speed operating | 4 / 27 | 1 / 22 | 5 / 26 | 3 / 23 | 0 | 0 | 2 B |
| **With dome 2, speed raw** | **8 / 25** | **3 / 24** | **5 / 29** | **6 / 24** | 0 / 24, 0 / 25 | 0 / 25, 0 / 24 | 2 A, 2 B |
| **With dome 2, sensor** | **15 / 79** | **0 / 75** | **11 / 78** | **11 / 76** | **11 / 75**, 0 / 79 | **4 / 79**, 0 / 75 | 2 B (+20 %) |

The sensor sweeps repeat the same steps, so they compare directly by speed:

| Sensor sweep steps | Fault | 2026-10-14 | 2026-10-15 |
|--------------------|-------|-----------:|-----------:|
| ±35 % and ±50 % | A, decr. | 4 / 20 passes (20 %) | 11 / 21 (52 %) |
| | A, incr. | 0 / 18 | 0 / 18 |
| | B, decr. | 0 / 20 | 11 / 21 (52 %) |
| | B, incr. | 0 / 18 | 11 / 19 (58 %) |
| | C (bit 1 @239), incr. | 0 / 18 | 11 / 18 (61 %) |
| | D (bit 1 @277), decr. | 0 / 20 | 4 / 21 (all 4 at +50 %: 4 / 14) |
| ±75 % and ±100 % | A, decr. | 0 / 56 | 4 / 58 (1 at +75 %, 3 at +100 %) |
| | B, C, D, A incr. | 0 (B band 113 passes, A band 112) | 0 (B band 114, A band 115) |

Spikes are much more frequent at low and middle speed. They are rare at 75-100 %, where the dome
crosses a 3-7 deg faulty spot in a few ring samples.

### Spike tables: every jump and spike, 2026-10-15

Every excursion of at least 10 deg, scored as a jump or not, with its bit classification. "Before"
is the reported angle before the excursion, with the telemetry `unwrapped_rad` in degrees in
parentheses; "Read" lists the wrong reports; "Scored jumps" is how many of the bench node's jumps
this excursion produced (all 50 and 102 jumps are accounted for). Near misses are pairs below the
jump threshold but above 50 % of it and at least 10 deg off the local motion. Every near miss is one
leg of a spike in the table; at 75-100 % normal motion alone is about 60 % of the threshold, so a
plain |change| criterion would list every fast pair.

#### 20261015-203255-speed (26 excursions, 50 scored jumps; the 50th is the return of the 20:33:23.15 reading, held at rest across the step 6/7 boundary for 0.83 s)

| Time | Step | % | Dir | Before deg (unwrapped) | Read | After | Reports | True code angle -> read: masks (bit 8..0) | Ham. | Bit | Sense | Scored jumps | Fault |
|---|---:|---:|---|---|---|---:|---:|---|---:|---|---|---:|---|
| 20:33:23.09 | 6 | +25 | rest (stalled) | 238 (-122) | 283 | 238 | 1 | 238 011100000 -> 283 011100001 | 1 | 0 | set | 2 | A |
| 20:33:23.15 | 6 | +25 | rest (stalled) | 238 (-122) | 276 | 238 (step 7) | 1 + 8 in step 7 | 237 111100000 -> 276 111100001 | 1 | 0 | set | 1 + 1 in step 7 | A |
| 20:33:38.62 | 8 | +30 | decr. | 238 (-122) | 276, 274, 274 | 235 | 3 | 235 111111000 -> 274 111111001; 237 111100000 -> 276 111100001 | 1/1 | 0/0 | set/set | 2 | A |
| 20:33:40.82 | 9 | -30 | incr. | 235 (-125) | 274 | 239 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:33:48.05 | 10 | +40 | decr. | 200 (200) | 13 | 196 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:33:51.72 | 10 | +40 | decr. | 203 (-157) | 12, 13 | 195 | 2 | 199 000110000 -> 12 000110001; 198 001110000 -> 13 001110001 | 1/1 | 0/0 | set/set | 2 | B |
| 20:33:56.62 | 11 | -40 | incr. | 193 (-167) | 13 | 201 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:33:57.02 | 11 | -40 | incr. | 228 (-492) | 274 | 238 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:33:59.89 | 11 | -40 | incr. | 192 (-168) | 13 | 198 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:34:01.99 | 12 | +50 | decr. | 239 (-481) | 274 | 236 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:34:04.79 | 12 | +50 | decr. | 207 (-873) | 13 | 196 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:34:06.99 | 12 | +50 | decr. | 201 (-1239) | 13 | 195 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:34:12.25 | 13 | -50 | incr. | 191 (-889) | 13 | 201 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:34:14.45 | 13 | -50 | incr. | 193 (-887) | 13 | 203 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:34:18.55 | 14 | +60 | decr. | 207 (-1593) | 13 | 192 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:34:21.75 | 14 | +60 | decr. | 246 (-1914) | 274 | 232 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:34:23.69 | 15 | -60 | incr. | 193 (-1967) | 13 | 201 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:34:34.15 | 17 | -70 | incr. | 230 (-3730) | 274 | 241 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:34:35.62 | 17 | -70 | incr. | 193 (-3407) | 13 | 210 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:34:41.35 | 18 | +80 | decr. | 239 (-2641) | 274 | 234 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:35:01.79 | 20 | +90 | decr. | 251 (-4429) | 274 | 228 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 1 | A |
| 20:35:14.49 | 22 | +100 | decr. | 253 (-3707) | 274 | 227 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 1 | A |
| 20:35:28.02 | 24 | +27 | rest | 238 (-2642) | 276 | 237 | 1 | 237 111100000 -> 276 111100001 | 1 | 0 | set | 2 | A |
| 20:35:28.12 | 24 | +27 | rest | 237 (-2643) | 274 | 235 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:35:29.85 | 24 | +27 | rest | 206 (-2674) | 11, 13 | 197 | 2 | 200 000100000 -> 11 000100001; 198 001110000 -> 13 001110001 | 1/1 | 0/0 | set/set | 2 | B |
| 20:35:51.59 | 26 | -23 | rest | 198 (-2682) | 13, 13, 13, 13, 13, 13 | 200 | 6 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |

Near misses (not scored, > 50 % of the threshold and >= 10 deg off the local motion):
- 20:35:01.79 step 20 +90% 251 -> 274 dt 0.033 ratio 0.88
- 20:35:14.49 step 22 +100% 253 -> 274 dt 0.033 ratio 0.81
#### 20261015-203716-sensor (55 excursions, 102 scored jumps, 0 jumps not inside an excursion)

| Time | Step | % | Dir | Before deg (unwrapped) | Read | After | Reports | True code angle -> read: masks (bit 8..0) | Ham. | Bit | Sense | Scored jumps | Fault |
|---|---:|---:|---|---|---|---:|---:|---|---:|---|---|---:|---|
| 20:37:18.02 | 0 | +20 | rest | 202 (-2678) | 284 | 200 | 1 | 201 010100000 -> 284 010100001 | 1 | 0 | set | 2 | B |
| 20:37:18.12 | 0 | +20 | rest | 200 (-2680) | 11, 12, 12, 12 | 198 | 4 | 200 000100000 -> 11 000100001; 199 000110000 -> 12 000110001 | 1/1 | 0/0 | set/set | 2 | B |
| 20:37:35.62 | 2 | +35 | decr. | 239 (-3001) | 276, 274, 274 | 235 | 3 | 235 111111000 -> 274 111111001; 237 111100000 -> 276 111100001 | 1/1 | 0/0 | set/set | 2 | A |
| 20:37:40.12 | 2 | +35 | decr. | 239 (-3361) | 276, 274 | 237 | 2 | 235 111111000 -> 274 111111001; 237 111100000 -> 276 111100001 | 1/1 | 0/0 | set/set | 2 | A |
| 20:37:40.65 | 2 | +35 | decr. | 206 (-3394) | 13 | 196 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:37:44.42 | 2 | +35 | decr. | 239 (-3721) | 274 | 236 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 1 | A |
| 20:37:48.79 | 2 | +35 | decr. | 239 (-4081) | 276, 274 | 236 | 2 | 235 111111000 -> 274 111111001; 237 111100000 -> 276 111100001 | 1/1 | 0/0 | set/set | 1 | A |
| 20:37:49.39 | 2 | +35 | decr. | 206 (-4114) | 11, 13 | 196 | 2 | 200 000100000 -> 11 000100001; 198 001110000 -> 13 001110001 | 1/1 | 0/0 | set/set | 1 | B |
| 20:37:53.22 | 2 | +35 | decr. | 239 (-4441) | 275, 274 | 235 | 2 | 235 111111000 -> 274 111111001; 236 111110000 -> 275 111110001 | 1/1 | 0/0 | set/set | 2 | A |
| 20:37:53.75 | 2 | +35 | decr. | 206 (-4474) | 13 | 197 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:37:57.55 | 2 | +35 | decr. | 239 (-4801) | 274 | 235 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:37:58.09 | 2 | +35 | decr. | 205 (-4835) | 13 | 196 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:38:01.95 | 2 | +35 | decr. | 239 (-5161) | 276, 274, 274 | 234 | 3 | 235 111111000 -> 274 111111001; 237 111100000 -> 276 111100001 | 1/1 | 0/0 | set/set | 0 | A |
| 20:38:08.32 | 3 | -35 | incr. | 193 (-5207) | 12 | 200 | 1 | 199 000110000 -> 12 000110001 | 1 | 0 | set | 2 | B |
| 20:38:08.89 | 3 | -35 | incr. | 236 (-5164) | 52 | 240 | 1 | 239 001100000 -> 52 001100010 | 1 | 1 | set | 2 | C |
| 20:38:12.42 | 3 | -35 | incr. | 197 (-4843) | 13 | 201 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:38:12.99 | 3 | -35 | incr. | 237 (-4803) | 52 | 239 | 1 | 239 001100000 -> 52 001100010 | 1 | 1 | set | 2 | C |
| 20:38:17.12 | 3 | -35 | incr. | 237 (-4443) | 52 | 240 | 1 | 239 001100000 -> 52 001100010 | 1 | 1 | set | 2 | C |
| 20:38:20.72 | 3 | -35 | incr. | 193 (-4127) | 12 | 200 | 1 | 199 000110000 -> 12 000110001 | 1 | 0 | set | 2 | B |
| 20:38:21.35 | 3 | -35 | incr. | 237 (-4083) | 52 | 240 | 1 | 239 001100000 -> 52 001100010 | 1 | 1 | set | 2 | C |
| 20:38:24.89 | 3 | -35 | incr. | 193 (-3767) | 12 | 200 | 1 | 199 000110000 -> 12 000110001 | 1 | 0 | set | 2 | B |
| 20:38:29.82 | 4 | +50 | decr. | 207 (-3753) | 13 | 197 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:38:33.89 | 4 | +50 | decr. | 287 (-4393) | 316 | 277 | 1 | 277 111000001 -> 316 111000011 | 1 | 1 | set | 2 | D |
| 20:38:34.45 | 4 | +50 | decr. | 200 (-4480) | 13 | 194 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:38:36.09 | 4 | +50 | decr. | 286 (-4754) | 316 | 276 | 1 | 277 111000001 -> 316 111000011 | 1 | 1 | set | 2 | D |
| 20:38:38.79 | 4 | +50 | decr. | 207 (-5193) | 13 | 197 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:38:40.95 | 4 | +50 | decr. | 208 (-5552) | 13 | 197 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:38:42.62 | 4 | +50 | decr. | 286 (-5834) | 316 | 275 | 1 | 277 111000001 -> 316 111000011 | 1 | 1 | set | 2 | D |
| 20:38:49.72 | 4 | +50 | decr. | 207 (-6993) | 13 | 196 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:38:51.65 | 4 | +50 | decr. | 243 (-7317) | 274 | 232 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:38:51.92 | 4 | +50 | decr. | 207 (-7353) | 13 | 195 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:38:53.89 | 4 | +50 | decr. | 245 (-7675) | 274 | 235 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:38:55.82 | 4 | +50 | decr. | 287 (-7993) | 316 | 276 | 1 | 277 111000001 -> 316 111000011 | 1 | 1 | set | 2 | D |
| 20:38:56.12 | 4 | +50 | decr. | 239 (-8041) | 353 | 231 | 1 | 232 111110110 -> 353 111110111 | 1 | 0 | set | 2 | A |
| 20:38:56.32 | 4 | +50 | decr. | 207 (-8073) | 13 | 195 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:38:58.29 | 4 | +50 | decr. | 239 (-8401) | 274 | 235 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:39:03.45 | 5 | -50 | incr. | 193 (-8447) | 32 | 201 | 1 | 193 011111111 -> 32 101101111 | 3 | 4,7,8 | cleared,cleared,set | 2 | multi-bit |
| 20:39:05.69 | 5 | -50 | incr. | 191 (-8089) | 13 | 200 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 1 | B |
| 20:39:05.95 | 5 | -50 | incr. | 233 (-8407) | 52 | 241 | 1 | 239 001100000 -> 52 001100010 | 1 | 1 | set | 2 | C |
| 20:39:12.35 | 5 | -50 | incr. | 193 (-7367) | 13 | 202 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:39:12.62 | 5 | -50 | incr. | 233 (-7687) | 52 | 240 | 1 | 239 001100000 -> 52 001100010 | 1 | 1 | set | 2 | C |
| 20:39:14.89 | 5 | -50 | incr. | 236 (-7324) | 52 | 241 | 1 | 239 001100000 -> 52 001100010 | 1 | 1 | set | 2 | C |
| 20:39:17.12 | 5 | -50 | incr. | 230 (-6970) | 52 | 241 | 1 | 239 001100000 -> 52 001100010 | 1 | 1 | set | 2 | C |
| 20:39:19.09 | 5 | -50 | incr. | 196 (-7004) | 13 | 202 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:39:19.35 | 5 | -50 | incr. | 234 (-6966) | 52 | 241 | 1 | 239 001100000 -> 52 001100010 | 1 | 1 | set | 2 | C |
| 20:39:21.35 | 5 | -50 | incr. | 191 (-6649) | 13 | 201 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:39:21.65 | 5 | -50 | incr. | 236 (-6964) | 52 | 241 | 1 | 239 001100000 -> 52 001100010 | 1 | 1 | set | 2 | C |
| 20:39:23.62 | 5 | -50 | incr. | 193 (-6647) | 13 | 201 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:39:28.12 | 5 | -50 | incr. | 191 (-6289) | 13 | 200 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 20:39:32.62 | 5 | -50 | incr. | 192 (-5928) | 13 | 201 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 1 | B |
| 20:39:32.89 | 5 | -50 | incr. | 234 (-6246) | 52 | 242 | 1 | 239 001100000 -> 52 001100010 | 1 | 1 | set | 2 | C |
| 20:39:35.12 | 6 | +75 | decr. | 246 (-6234) | 274 | 232 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:40:52.32 | 8 | +100 | decr. | 252 (-10188) | 274 | 228 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 20:40:54.19 | 8 | +100 | decr. | 253 (-10907) | 274 | 227 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 1 | A |
| 20:41:05.35 | 8 | +100 | decr. | 252 (-15228) | 274 | 226 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |

Near misses (not scored, > 50 % of the threshold and >= 10 deg off the local motion):
- 20:37:44.42 step 2 +35% 239 -> 274 dt 0.067 ratio 0.86
- 20:38:02.02 step 2 +35% 274 -> 234 dt 0.067 ratio 0.98
- 20:40:54.19 step 8 +100% 253 -> 274 dt 0.033 ratio 0.98

### Spike tables, 2026-10-14 (same method, for comparison)

#### 20261014-222436-speed (1 excursion, 2 scored jumps, 0 jumps not inside an excursion)

| Time | Step | % | Dir | Before deg (unwrapped) | Read | After | Reports | True code angle -> read: masks (bit 8..0) | Ham. | Bit | Sense | Scored jumps | Fault |
|---|---:|---:|---|---|---|---:|---:|---|---:|---|---|---:|---|
| 22:27:04.09 | 24 | +27 | rest | 239 (239) | 276 | 237 | 1 | 237 111100000 -> 276 111100001 | 1 | 0 | set | 2 | A |
#### 20261014-222816-sensor (4 excursions, 6 scored jumps, 0 jumps not inside an excursion)

| Time | Step | % | Dir | Before deg (unwrapped) | Read | After | Reports | True code angle -> read: masks (bit 8..0) | Ham. | Bit | Sense | Scored jumps | Fault |
|---|---:|---:|---|---|---|---:|---:|---|---:|---|---|---:|---|
| 22:28:49.13 | 2 | +35 | decr. | 239 (-1561) | 274 | 236 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 0 | A |
| 22:28:53.49 | 2 | +35 | decr. | 243 (-1917) | 274 | 237 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 22:28:57.89 | 2 | +35 | decr. | 238 (-2282) | 274 | 237 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 22:29:59.56 | 4 | +50 | decr. | 247 (-4433) | 276 | 237 | 1 | 237 111100000 -> 276 111100001 | 1 | 0 | set | 2 | A |

Near misses (not scored, > 50 % of the threshold and >= 10 deg off the local motion):
- 22:28:49.13 step 2 +35% 239 -> 274 dt 0.067 ratio 0.88
#### 20261014-223427-speed (15 excursions, 28 scored jumps, 0 jumps not inside an excursion)

| Time | Step | % | Dir | Before deg (unwrapped) | Read | After | Reports | True code angle -> read: masks (bit 8..0) | Ham. | Bit | Sense | Scored jumps | Fault |
|---|---:|---:|---|---|---|---:|---:|---|---:|---|---|---:|---|
| 22:35:11.46 | 10 | +40 | decr. | 207 (-873) | 13 | 196 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 22:35:16.46 | 11 | -40 | incr. | 195 (-885) | 12, 12 | 200 | 2 | 199 000110000 -> 12 000110001 | 1 | 0 | set | 2 | B |
| 22:35:25.99 | 12 | +50 | decr. | 245 (-1195) | 275 | 235 | 1 | 236 111110000 -> 275 111110001 | 1 | 0 | set | 2 | A |
| 22:35:32.43 | 13 | -50 | incr. | 230 (-850) | 274 | 240 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 1 | A |
| 22:35:40.96 | 14 | +60 | decr. | 208 (-1592) | 13 | 195 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 22:35:58.89 | 17 | -70 | incr. | 191 (-1609) | 13 | 206 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 22:36:08.53 | 19 | -80 | incr. | 192 (-3408) | 12 | 206 | 1 | 199 000110000 -> 12 000110001 | 1 | 0 | set | 2 | B |
| 22:36:49.06 | 24 | +35 | decr. | 242 (-2278) | 274 | 237 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 22:36:49.53 | 24 | +35 | decr. | 207 (-2313) | 13 | 196 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 22:36:55.39 | 25 | +32 | decr. | 242 (-2638) | 274 | 237 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 22:36:55.96 | 25 | +32 | decr. | 199 (-2681) | 13 | 196 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 1 | B |
| 22:37:01.93 | 25 | +32 | decr. | 239 (-3001) | 274 | 237 | 1 | 235 111111000 -> 274 111111001 | 1 | 0 | set | 2 | A |
| 22:37:02.43 | 25 | +32 | decr. | 207 (-3033) | 13 | 197 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 22:37:09.43 | 26 | -35 | rest | 198 (-3042) | 13 | 199 | 1 | 198 001110000 -> 13 001110001 | 1 | 0 | set | 2 | B |
| 22:37:09.49 | 26 | -35 | rest | 199 (-3041) | 12, 12, 11 | 200 | 3 | 200 000100000 -> 11 000100001; 199 000110000 -> 12 000110001 | 1/1 | 0/0 | set/set | 2 | B |

Near misses (not scored, > 50 % of the threshold and >= 10 deg off the local motion):
- 22:35:32.46 step 13 -50% 274 -> 240 dt 0.067 ratio 0.68
- 22:37:13.89 step 27 -32% 285 -> 300 dt 0.033 ratio 0.58

## Speed repeatability

The raw-motor speed sweeps, compared on the same footing. For the with-dome runs the bench node's
analysis is re-run on the CSV with the spike samples removed. The live stall decisions below 40 %
are kept, because a stall is decided on the spot and the run then stops the step. Today's -60 %
step was stopped by a spike artefact after only 0.3 s of measurement; the re-analysis counts it as
moving at 3.48 rad/s. Removing the one spike from yesterday's raw run changes nothing. "Time to
90 %" is recomputed the same way for every run: from the command to the first 0.5 s regression
velocity at 90 % of the step slope, median over the ±40 to ±100 % steps. This method gives 1.37 s
where the 2026-10-14 report quoted 1.27 s, and 0.60 s where the baseline report quoted 0.63 s.

"+%" is positive percent (angle decreasing, negative joint velocity); "-%" is negative percent.

| Quantity | No dome (raw) | With dome 1 (raw) | With dome 2 (raw, spikes removed) | Dome 2 vs dome 1 |
|----------|--------:|--------:|--------:|--------:|
| omega at 100 % measured, +% / -% | 6.347 / 6.836 rad/s | 6.609 / 6.558 | 6.743 / 6.639 | +2.0 % / +1.2 % |
| Lower measured 100 % (`max_speed_rad_per_sec`) | 6.35 | 6.56 | 6.64 | +1.2 % |
| omega at 100 %, fit k (100 - p0), +% / -% | 6.504 / 6.882 | 6.766 / 6.800 | 6.892 / 6.803 | +1.9 % / 0.0 % |
| k, +% / -% (rad/s per %) | 0.07875 / 0.08505 | 0.08536 / 0.08502 | 0.08457 / 0.08184 | -0.9 % / **-3.7 %** |
| p0, +% / -% | 17.42 / 19.09 | 20.73 / 20.01 | 18.50 / 16.88 | **-2.2 / -3.1 points** |
| R squared, +% / -% (9 points each) | 0.9942 / 0.9969 | 0.9911 / 0.9911 | 0.9971 / 0.9980 | higher |
| min_percent raw, +% / -% | 27 / 25 | 27 / 27 | 27 / 25 | 0 / -2 |
| Velocity at min_percent (raw), +% / -% | 0.631 / 0.553 rad/s | 0.414 / 0.186 | 0.762 / 0.559 | **higher** (stick-slip region) |
| min_percent at operating settings, +% / -% | 27 / 25 | 32 / 32 | **not measured (run aborted)** | |
| Asymmetry, fit / measured 100 % | 0.945 / 0.929 | 0.995 / 1.008 | 1.013 / 1.016 | +1.8 % / +0.8 % |
| Time to 90 % of step speed, median (recomputed) | 0.60 s | 1.37 s | 1.43 s | +4 % |
| Stall end angles, +% (angle decreasing) | 47-59 (and 241) | 37-57 (one at 7) | 53, 43, **238** (+10 no motion at 72) | new stop on fault A |
| Stall end angles, -% (angle increasing) | 168-192 (and 345, 354) | 105-188 | 93, 105, 202 (-10, -15 no motion) | same region |
| Sensor errors per revolution (session) | 2.7 | 2.8 | 2.3 | unchanged |

As recorded by the bench node, today's raw JSON gives k 0.08488 / 0.09227, p0 20.58 / 27.30 and R
squared 0.9649 / 0.8959. The differences from the table above are spike damage.

**Flagged (outside a few percent):**

- **p0** is 2.2 and 3.1 points lower, and the negative-percent k is 3.7 % lower. In phase 7 terms:
  positive joint velocity (negative percent) gives k 12.22 %/(rad/s), p0 16.88 (yesterday 11.76,
  20.01); negative joint velocity gives k 11.82, p0 18.50 (yesterday 11.72, 20.73). The predicted
  percent for 1 rad/s drops from 31.8 / 32.4 % to 29.1 / 30.3 %. For 3 rad/s it drops from 55.3 /
  55.9 % to 53.5 / 54.0 %. At 6 rad/s they differ by 0.4 and 1.6 points. The difference
  shrinks with speed.
- **Velocity at the raw min_percent** (27 % / 25 %) is 0.76 / 0.56 rad/s against 0.41 / 0.19 rad/s.
  Both days are in the stick-slip region below 30 %. The velocities there vary with start angle:
  -25 % moved today and stalled yesterday. This is the scatter that 2026-10-14 observation 3
  already described, not a calibration change.
- **Stall end angles.** The angle-decreasing direction still stops at 43-56 deg. A new stop at true
  238 deg (+25 %) falls exactly on fault A. The angle-increasing direction stops at 93-202 deg,
  the same rising-friction region as yesterday (105-188 deg).

Everything else repeats within 2 %: the top speed, the fit extrapolation at 100 %, the symmetry,
the positive-percent k and the response time.

## Telemetry timing

| Run | Pairs | Same-cycle (< 5 ms) | Median | > 100 ms | Max |
|-----|------:|--------------------:|-------:|---------:|----:|
| speed (raw) | 4818 | 293 (6.1 %) | 33.3 ms | 3 | 233 ms |
| sensor | 7989 | 359 (4.5 %) | 33.3 ms | 4 | 233 ms |
| speed (operating, partial) | 259 | 23 | 33.3 ms | 0 | 67 ms |

As on 2026-10-14, every gap above 100 ms is a ~233 ms config-refresh gap. The 1.03 s dropout that
aborted the operating run came after its last recorded row, so the CSV does not show it.

## Firmware sensor error counter

The counter restarted at the reboot.

| Point | Time | `sensor_errors` |
|-------|------|----------------:|
| Session start, at rest | 20:32:29 | 0 |
| After the speed sweep (raw), at rest | 20:37:06 | 143 (+143 over 56.1 revolutions = 2.5 / rev) |
| After the sensor sweep, at rest | 20:42:24 | 483 (+340 over 155.6 revolutions = 2.2 / rev) |
| After the aborted run / session end | 20:45:36 | 483 |

The session rate is 2.3 per revolution, against 2.8 on 2026-10-14 with the dome and 2.7 without.
`packet_serial_errors` stayed 0. The parse-error rate on the sensor link did not rise with the
spikes. It counts malformed lines, and the spikes are valid lines carrying a wrong angle (plan 2.1).

## Conclusion

**The spikes recurred at the same angles, with the same bit, after re-seating.** The bit 0
(`SENSORS[0]`) fault reads set where the track should read clear. It appeared at true 232-238 deg
(reading 274-276, 283 and 353 deg) and at true 198-201 deg (reading 11-13 and 284 deg) on both
days, with identical true -> read code pairs. That rules out a one-off: the fault is tied to those
dome angles and survived re-seating. It is **constant with the dome on, in location**, and it has
never appeared without the dome (0 in about 280 revolutions).

**Its rate is not constant.** In the sensor sweep at 35-50 %, bit 0 spikes went from 4 (all
pattern A, angle decreasing) to 33 (A 11; B 22 in both directions) over the same steps and about
the same number of passes. Pattern B, absent from yesterday's first two runs and seen 10 times in
the third, is present from the first minutes of today's session. **And a second sensor now
fails:** bit 1 (`SENSORS[1]`) reads set at true 239 deg (reading 52 deg, 11 of 18
angle-increasing passes at 35-50 %) and at 277 deg (reading 316 deg, 4 of 14 angle-decreasing
passes at +50 %). Each of these four spots fails in only one direction, except pattern B, which
fails in both. Since mounting, the problem has grown from one bit at one angle in one direction,
to two angles, and now to two bits at four angles. It got worse within yesterday's session and is
worse again after re-seating.

**What the evidence supports:** a mechanical relationship between the dome and the sensor ring
that changes with the dome's angle. At fixed dome angles a sensor intermittently fails to read 0
on a track that should read 0. The relationship is sensitive to how the dome sits: the rate and
the set of failing bits changed with the re-seating, while the bit 0 angles did not. Examples are
the sensor-to-track gap or the alignment over a stretch of the ring, or the dome loading the ring
or its mounting at those angles. All but one of 101 misreads go the same way (a bit reading set),
always inside a clear run and never at a code edge, and each lasts several ring samples. That fits
a sensor losing its signal over part of its track better than electrical noise. The rates depend
on speed and direction, which also fits a marginal signal (more ring samples per degree at low
speed).

**What it cannot distinguish:**

- whether the fixed angles come from the ring itself (a defect or weak spot in the bit 0 track at
  198-201 and 232-238 deg, made marginal by the dome's presence) or from the dome's geometry
  (runout or tilt that opens the gap at the same dome angles whatever the seating);
- whether the bit 1 fault is new because of the re-seating or because of a progressive change.
  The B pattern was already growing during yesterday's session, before anyone touched the dome;
- the electrical side: with `INPUT_PULLUP`, a sensor that does not pull low reads 1, and the
  header does not say which level the sensor drives on a mark. Probing `SENSORS[0]` and
  `SENSORS[1]` at those angles with the dome turned slowly by hand would settle it.

**Recommended next steps** (the human decides): check the sensor ring's gap and alignment for
`SENSORS[0]` and `SENSORS[1]`, at true 195-205 deg and 230-240 deg (bit 0) and around 239 deg and
277 deg (bit 1). Also check whether the dome presses on the ring or its mounting there. Then re-run
the sensor sweep (and the operating-settings speed sweep that did not complete today). A firmware
or plugin guard that rejects implausible single-sample steps (plan section 9) would contain the
symptom but not fix the cause.

## Values to configure

**Confirmed, unchanged from 2026-10-14** (see [that report, "What to
configure"](2026-10-14-with-dome.md#what-to-configure)):

| Parameter | Value | Today |
|-----------|------:|-------|
| `max_speed_rad_per_sec` | 6.56 | Today's lower measured 100 % rate is 6.64 (+1.2 %); keep the conservative 6.56 |
| `velocity_map.k_percent_per_rad_s` | 11.76 (positive velocity), 11.72 (negative velocity) | 12.22 / 11.82 today (+3.9 % / +0.9 %) |
| `velocity_map.p0_percent` | 20.01 (positive velocity), 20.73 (negative velocity) | 16.88 / 18.50 today; the two calibrations differ by 2-3 points at 1 rad/s and by 0.4-1.6 points at 6 rad/s |
| `velocity_map.min_percent` | 32 both directions | Not re-measured: the operating-settings run aborted. The raw run again needed 27 / 25 % |
| Sign | `percent_sign` -1 for positive velocity | Confirmed (24 of 24 votes on the spike-free data) |

None of these changed materially. Phase 7 should still treat the low end as uncertain by a few
percent. The dome position will not be trustworthy for position control until the sensor fault is
fixed: a bit 0 B spike still shifts the unwrapped angle by a full turn.
