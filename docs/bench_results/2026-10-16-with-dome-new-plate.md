# Dome bench results, 2026-10-16: new sensor pattern plate, DOME INSTALLED (calibration reference)

**New sensor pattern plate, dome installed; this is the calibration reference, superseding
[2026-10-14-with-dome.md](2026-10-14-with-dome.md) and
[2026-10-15-with-dome-reseated.md](2026-10-15-with-dome-reseated.md).** The dome shell was mounted
for every measurement in this report. The runs repeat the three sweeps of this morning's
[no-dome run on the same plate](2026-10-16-no-dome-new-plate.md) with the same stack, commands and
parameters. Configure the velocity mapping from
[THE VALUES TO CONFIGURE](#the-values-to-configure). The two old-plate with-dome reports are kept as
history.

**Short answer.** The new plate fixes the with-dome sensor fault.

- **Sensor.** About 194 revolutions of sensor sweeps (one aborted run and one complete run) showed
  no position spikes. On the old plate the same sweeps showed 6 and 102 jumps. `spikes.py` finds
  **no excursion** in any of the four runs.
- **The one scored jump is not a sensor spike.** The complete sensor sweep scored **one** jump, at
  -100 % from 355 deg: +22 deg in one 33 ms interval, against a threshold of 21.2 deg. The reported
  angle never shows 359 to 2 deg, on either plate, so every crossing of 0 deg makes one long step.
  At -100 % that step reached the threshold. See
  [The one scored jump](#the-one-scored-jump-the-355-to-4-deg-dead-zone).
- **Drive calibration.** It is close to the old plate with the dome:
  - 100 %: 6.70 / 6.70 rad/s (raw motor) and 6.71 / 6.56 rad/s (operating settings).
  - k 0.088 / 0.087 rad/s per %, p0 22.1 / 20.5 %.
  - `min_percent` 30 / 27 (raw) and 32 / 25 (operating settings).
  - The step response is about 2.6 times slower than without the dome.

**One incident.** The first sensor sweep aborted on a telemetry dropout: at least 1.05 s without a
message, at +75 %. The dome stopped and the settings were restored. The session was restarted with
the main threads of the stack processes at nice 0 and a timestamp recorder running. Both remaining
sweeps completed with no gap over 233 ms. See
[Telemetry dropout](#telemetry-dropout-in-the-first-sensor-sweep).

Phase 6 of [docs/dome_diagnostics_plan.md](../dome_diagnostics_plan.md), run with the human at the
robot. The human mounted the dome and asked for the run.

| | |
|---|---|
| Date | 2026-10-16, 12:58 to 13:28 local (-0400) |
| Stack | Same as the morning: `robot_state_publisher` with the dome-only URDF (`enable_bench_mode`, `publish_telemetry`), `ros2_control_node` with only the RoamADome hardware at 30 Hz, no controllers; `rmw_zenoh_cpp` with a router started for the session (restarted after the abort) |
| Code | `ros2_roamadome` at `82c038d` (`diag/phase-6-new-plate-no-dome`). The bench node and analysis are unchanged since `fa97b3b` (only docs changed since). `#DPREPORT28` |
| Bench node | `ros2 run ros2_roamadome roamadome_bench --ros-args -p output_dir:=/home/pi/dome_bench/phase6-newplate-dome -p hardware_node:=roamadome`, plan 5.4 defaults; the operating run adds `-p speed_sweep.raw_motor:=false`. Identical to the morning apart from `output_dir` |
| Results | [2026-10-16-with-dome-new-plate-speed.json](2026-10-16-with-dome-new-plate-speed.json), [2026-10-16-with-dome-new-plate-sensor.json](2026-10-16-with-dome-new-plate-sensor.json), [2026-10-16-with-dome-new-plate-speed-operating.json](2026-10-16-with-dome-new-plate-speed-operating.json), and the aborted first sensor sweep [2026-10-16-with-dome-new-plate-sensor-aborted.json](2026-10-16-with-dome-new-plate-sensor-aborted.json) (unmodified copies of the bench node's result files) |
| Raw data (not committed) | `/home/pi/dome_bench/phase6-newplate-dome/`: `20261016-125950-speed/`, `20261016-130416-sensor/` (aborted), `20261016-131650-sensor/`, `20261016-132359-speed/` (each with `samples.csv`, `run.json`, `settings_before.json`, `settings_after.json`); `bench_speed.log`, `bench_sensor.log` (aborted run), `bench_sensor_2.log`, `bench_speed_operating.log`; `cm.log` (first stack), `cm_2.log` (after the restart); `baseline_params.yaml`, `final_params.yaml`; `/diagnostics` captures (`baseline_`, `pre_sensor_`, `post_abort_`, `resume_`, `post_sensor_`, `pre_operating_`, `final_diagnostics.txt`); `telemetry_stamps.txt`; `nice_values.txt`; `check_<run>.txt`; `notes.txt`; `analysis/` (`tables.py`, `speedcmp.py`, `spikes.py`, `codes.py` copied from the morning, and their output) |

`notes.txt` records that the dome was installed on the new plate. The result JSON files have no
free-text field for this.

## Runs

| Run | Sweep | Settings during the run | Status | Settings restored |
|-----|-------|-------------------------|--------|-------------------|
| `20261016-125950-speed` | Speed, plan defaults (12 percents, `raw_motor` true) | MaxSpeed 100, SpeedTarget 100, Scaling off | complete, 27 steps, 3 min 29 s | yes: `#DPMAXSPEED80` acked, `settings_restored: true`, `firmware_acked: true` |
| `20261016-130416-sensor` | Sensor, plan defaults | MaxSpeed 100, SpeedTarget 100, Scaling off | **aborted** in step 6 (+75 %) after 2 min 29 s: `telemetry dropout: no message for 1.05 s` | yes, as above |
| `20261016-131650-sensor` | Sensor, plan defaults (20/35/50/75/100, 30 s, both directions, `raw_motor` true) | MaxSpeed 100, SpeedTarget 100, Scaling off | complete, 10 steps, 4 min 48 s | yes, as above |
| `20261016-132359-speed` | Speed, plan defaults but `raw_motor` false | Operating settings: MaxSpeed 80, SpeedTarget 100, Scaling off | complete, 28 steps, 3 min 19 s | nothing written; `settings_restored: true`, `firmware_acked: true` (0 writes) |

**Threshold and minimum percentages.** Both sensor sweeps took them from this session's with-dome
speed run:

- `max_plausible_deg_s` 576.02 = 1.5 x 6.702 rad/s.
- `max_plausible_source`: `1.5 x measured maximum in
  /home/pi/dome_bench/phase6-newplate-dome/20261016-125950-speed/speed_sweep.json`.
- `min_percent_known` 30 / 27, from the same file.

**Checks.**

- `roamadome_bench_report <run_dir> --check` reproduces all four result files (exit 0, "result
  matches").
- After every run `ros2 param get` returned `device.max_speed` 80, `device.speed_target` 100 and
  `device.scaling` False.
- At the end `min_speed` was 15, `inverted` True, `home_pos` 0 and `auto_safety` False. None of
  these was written.
- `settings_acks` 2 and `settings_ack_timeouts` 0 on the second stack.
- `final_params.yaml` is identical to `baseline_params.yaml`.
- Neither stack start had a startup command timeout this time.

## Firmware configuration (`device.*` at the start of the session)

From `ros2 param dump /roamadome`. It is unchanged at the end and **identical to this morning and
to 2026-10-14**:

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

## Speed sweep (`20261016-125950-speed`, raw motor)

Positive percent moves the reported angle negative (`angle_sign_for_positive_percent` -1, 25 of 25
votes). Columns as in the earlier reports. "n/a" is a stalled step with fewer than two measure
samples, where the JSON has `null`.

| # | Kind | Percent | Samples | Moving | Stalled (after s) | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Max gap s |
| 0 | step | +10 | 76 | no | yes (2.0) | 0.1 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 1 | step | -10 | 69 | no | yes (2.0) | 0.0 | 0.00 | n/a | n/a | n/a | 0.033 |
| 2 | step | +15 | 111 | no | yes (3.3) | 1.3 | 0.01 | -0.033 | -0.041 | 0.055 | 0.067 |
| 3 | step | -15 | 159 | no | yes (4.7) | 2.7 | 0.03 | 0.080 | 0.081 | 0.059 | 0.033 |
| 4 | step | +20 | 147 | no | yes (4.5) | 2.5 | 0.04 | -0.090 | -0.144 | 0.149 | 0.233 |
| 5 | step | -20 | 234 | no | yes (7.2) | 5.2 | 0.14 | 0.200 | 0.178 | 0.113 | 0.067 |
| 6 | step | +25 | 168 | no | yes (5.2) | 3.2 | 0.09 | -0.142 | -0.214 | 0.206 | 0.067 |
| 7 | step | -25 | 402 | no | yes (12.4) | 10.5 | 0.45 | 0.270 | 0.268 | 0.171 | 0.067 |
| 8 | step | +30 | 227 | yes | no | 5.3 | 0.58 | -0.620 | -0.716 | 0.309 | 0.067 |
| 9 | step | -30 | 187 | yes | no | 4.3 | 0.52 | 0.732 | 0.799 | 0.189 | 0.067 |
| 10 | step | +40 | 186 | yes | no | 4.0 | 0.90 | -1.328 | -1.437 | 0.264 | 0.067 |
| 11 | step | -40 | 197 | yes | no | 4.4 | 1.14 | 1.704 | 1.628 | 0.309 | 0.067 |
| 12 | step | +50 | 190 | yes | no | 4.0 | 1.61 | -2.585 | -2.552 | 0.243 | 0.233 |
| 13 | step | -50 | 190 | yes | no | 4.0 | 1.70 | 2.647 | 2.684 | 0.217 | 0.067 |
| 14 | step | +60 | 188 | yes | no | 4.0 | 2.23 | -3.561 | -3.510 | 0.275 | 0.067 |
| 15 | step | -60 | 189 | yes | no | 4.0 | 2.26 | 3.613 | 3.566 | 0.261 | 0.033 |
| 16 | step | +70 | 187 | yes | no | 4.0 | 2.81 | -4.416 | -4.382 | 0.216 | 0.067 |
| 17 | step | -70 | 188 | yes | no | 4.0 | 2.82 | 4.467 | 4.433 | 0.234 | 0.067 |
| 18 | step | +80 | 186 | yes | no | 4.0 | 3.29 | -5.189 | -5.164 | 0.158 | 0.067 |
| 19 | step | -80 | 186 | yes | no | 4.0 | 3.31 | 5.246 | 5.206 | 0.186 | 0.067 |
| 20 | step | +90 | 185 | yes | no | 4.0 | 3.74 | -5.909 | -5.817 | 0.705 | 0.233 |
| 21 | step | -90 | 185 | yes | no | 4.0 | 3.84 | 6.084 | 6.018 | 0.267 | 0.067 |
| 22 | step | +100 | 184 | yes | no | 4.0 | 4.25 | -6.702 | -6.641 | 0.285 | 0.067 |
| 23 | step | -100 | 184 | yes | no | 4.0 | 4.24 | 6.696 | 6.648 | 0.251 | 0.067 |
| 24 | search | +27 | 210 | no | yes (6.6) | 4.6 | 0.26 | -0.328 | -0.372 | 0.306 | 0.067 |
| 25 | search | +28 | 938 | no | yes (30.5) | 28.5 | 2.94 | -0.690 | -0.634 | 0.324 | 0.233 |
| 26 | search | -27 | 215 | yes | no | 4.9 | 0.34 | 0.449 | 0.474 | 0.248 | 0.067 |

Step 25 (+28 %, binary search) turned 2.9 revolutions (1070 deg) at about 0.69 rad/s for 30 s, then
stalled at 245 deg. The search therefore placed the positive-percent boundary at 30 %. Step 7
(-25 %) turned 204 deg and stalled at 190 deg; -27 % then moved at 0.45 rad/s.

### Per-direction fit, `omega = k * (p - p0)` over the moving steps

| | Positive percent (angle decreasing) | Negative percent (angle increasing) |
|---|---:|---:|
| k (rad/s per %) | 0.08828 | 0.08717 |
| p0 (%) | 22.08 | 20.45 |
| R squared | 0.9936 (8 points) | 0.9967 (9 points) |
| omega at 100 %, measured | 6.702 rad/s | 6.696 rad/s |
| omega at 100 %, from the fit, k (100 - p0) | 6.878 rad/s | 6.934 rad/s |
| min_percent (bracketed to 2 %) | 30 (28 stalled) | 27 (25 stalled) |
| Velocity at min_percent | 0.620 rad/s | 0.449 rad/s |
| Stalled above min_percent | none | none |
| Wrong-direction steps | none | none |

Asymmetry (positive / negative `omega_max_rad_s`, fit values): 0.992. From the measured 100 %
steps: 6.702 / 6.696 = 1.001.

### Phase 7 parameter block (copied from the JSON)

```json
"phase7": {
  "positive_velocity": {
    "percent_sign": -1,
    "k_percent_per_rad_s": 11.471487852554455,
    "p0_percent": 20.453840262080796,
    "min_percent": 27
  },
  "negative_velocity": {
    "percent_sign": 1,
    "k_percent_per_rad_s": 11.327744897359382,
    "p0_percent": 22.083429155831187,
    "min_percent": 30
  },
  "max_speed_rad_per_sec": 6.878383257229953
}
```

`positive_velocity` is the joint's positive direction (reported angle increasing). On this
platform it needs a **negative** `:DPR` percent, as before. The JSON's `max_speed_rad_per_sec` is
the lower of the two fit extrapolations. The table below gives the values to use.

### THE VALUES TO CONFIGURE

> **New plate, dome installed. These are the values to configure for phase 7.** They replace the
> old-plate with-dome values (2026-10-14 / 2026-10-15) and the provisional no-dome block.

| Parameter | Value | Source |
|-----------|------:|--------|
| `max_speed_rad_per_sec` | **6.56** | The lower measured 100 % rate: -100 % (positive joint velocity) in the operating-settings run, the settings the plugin drives at. The raw run measured 6.70 / 6.70 (2 % higher). The complete sensor sweep sustained 6.57 / 6.65 rad/s over 30 s at ±100 %, which agrees. The JSON's 6.88 is the fit extrapolation, 5 % above the measurement |
| `velocity_map.k_percent_per_rad_s` | **11.47** (positive velocity), **11.33** (negative velocity) | `phase7` block of the raw run above |
| `velocity_map.p0_percent` | **20.45** (positive velocity), **22.08** (negative velocity) | `phase7` block of the raw run above |
| `velocity_map.min_percent` | **27** (positive velocity), **32** (negative velocity) | Larger of the two runs per direction. Negative velocity (positive percent): the operating run needed 32 (+30 % stalled), the raw run 30. Positive velocity (negative percent): the operating run's boundary is 25, but -25 % moved there at only 0.35 rad/s and stalled in the raw run after 204 deg, so the raw run's 27 is used. See the note below |
| Sign | `percent_sign` -1 for positive velocity, +1 for negative | Measured, 25 of 25 votes in both speed runs, unchanged |

If only one parameter set is supported: k 11.47, p0 22.08, min_percent 32. The values hold for
MaxSpeed 80, MinSpeed 15, SpeedTarget 100 and Scaling off (the settings above).

**`min_percent` note.** The README rule was: where the two runs differ, use the operating-settings
run's `min_percent`.

- The rule assumed the operating run is the more conservative one. That was true on 2026-10-14
  (32 against 27).
- Today it is not true for negative percent. The operating search bracketed 25 (23 stalled), and
  -25 % moved at 0.35 rad/s over 0.29 revolutions, which is stick-slip. In the raw run -25 %
  stalled.
- The low end depends on where each step starts (see
  [Low end and stall angles](#low-end-and-stall-angles)), so each run finds a different boundary
  within a few percent.
- Taking the larger value per direction is the safe reading of both runs. The README now says this.

## Speed sweep at the operating settings (`20261016-132359-speed`, raw_motor false)

| # | Kind | Percent | Samples | Moving | Stalled (after s) | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Max gap s |
| 0 | step | +10 | 69 | no | yes (2.0) | 0.0 | 0.00 | n/a | n/a | n/a | 0.033 |
| 1 | step | -10 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 2 | step | +15 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 3 | step | -15 | 249 | no | yes (7.6) | 5.6 | 0.12 | 0.116 | 0.157 | 0.148 | 0.233 |
| 4 | step | +20 | 290 | no | yes (8.8) | 6.8 | 0.18 | -0.181 | -0.171 | 0.101 | 0.067 |
| 5 | step | -20 | 299 | no | yes (9.2) | 7.2 | 0.23 | 0.217 | 0.215 | 0.136 | 0.033 |
| 6 | step | +25 | 408 | no | yes (12.6) | 10.6 | 0.33 | -0.208 | -0.210 | 0.126 | 0.067 |
| 7 | step | -25 | 228 | yes | no | 5.3 | 0.29 | 0.350 | 0.385 | 0.219 | 0.067 |
| 8 | step | +30 | 240 | no | yes (7.6) | 5.6 | 0.36 | -0.428 | -0.421 | 0.288 | 0.067 |
| 9 | step | -30 | 184 | yes | no | 4.1 | 0.45 | 0.630 | 0.728 | 0.276 | 0.067 |
| 10 | step | +40 | 187 | yes | no | 4.0 | 1.03 | -1.537 | -1.607 | 0.280 | 0.233 |
| 11 | step | -40 | 190 | yes | no | 4.2 | 1.04 | 1.648 | 1.560 | 0.342 | 0.067 |
| 12 | step | +50 | 191 | yes | no | 4.0 | 1.66 | -2.596 | -2.540 | 0.244 | 0.033 |
| 13 | step | -50 | 189 | yes | no | 4.0 | 1.66 | 2.636 | 2.578 | 0.305 | 0.033 |
| 14 | step | +60 | 189 | yes | no | 4.0 | 2.20 | -3.494 | -3.462 | 0.221 | 0.067 |
| 15 | step | -60 | 190 | yes | no | 4.0 | 2.27 | 3.496 | 3.504 | 0.183 | 0.067 |
| 16 | step | +70 | 187 | yes | no | 4.0 | 2.69 | -4.235 | -4.252 | 0.213 | 0.067 |
| 17 | step | -70 | 186 | yes | no | 4.0 | 2.76 | 4.401 | 4.360 | 0.228 | 0.067 |
| 18 | step | +80 | 186 | yes | no | 4.0 | 3.23 | -5.102 | -5.050 | 0.634 | 0.233 |
| 19 | step | -80 | 186 | yes | no | 4.0 | 3.33 | 5.253 | 5.211 | 0.184 | 0.067 |
| 20 | step | +90 | 185 | yes | no | 4.0 | 3.72 | -5.864 | -5.825 | 0.177 | 0.033 |
| 21 | step | -90 | 186 | yes | no | 4.0 | 3.84 | 6.028 | 5.986 | 0.177 | 0.067 |
| 22 | step | +100 | 183 | yes | no | 4.0 | 4.27 | -6.711 | -6.664 | 0.324 | 0.067 |
| 23 | step | -100 | 183 | yes | no | 4.0 | 4.19 | 6.559 | 6.532 | 0.210 | 0.067 |
| 24 | search | +35 | 184 | yes | no | 4.0 | 0.79 | -1.153 | -1.249 | 0.233 | 0.067 |
| 25 | search | +32 | 246 | yes | no | 6.1 | 1.05 | -1.078 | -1.065 | 0.279 | 0.067 |
| 26 | search | -22 | 250 | no | yes (7.9) | 5.9 | 0.25 | 0.286 | 0.295 | 0.200 | 0.200 |
| 27 | search | -23 | 187 | no | yes (5.6) | 3.6 | 0.05 | 0.101 | 0.097 | 0.063 | 0.067 |

| | Positive percent | Negative percent |
|---|---:|---:|
| k (rad/s per %) | 0.08469 | 0.08619 |
| p0 (%) | 20.21 | 20.59 |
| R squared (9 points) | 0.9982 | 0.9958 |
| omega at 100 %, measured | 6.711 rad/s | 6.559 rad/s |
| omega at 100 %, from the fit | 6.758 rad/s | 6.844 rad/s |
| min_percent | **32** (30 stalled) | 25 (23 stalled) |
| Velocity at min_percent | 1.078 rad/s | 0.350 rad/s |
| Stalled above min_percent | none | none |

Asymmetry (fit): 0.987. No jumps; the largest step was 81 % of the default 720 deg/s threshold.

Phase 7 block of this run:

- Positive velocity: percent_sign -1, k 11.60, p0 20.59, min 25.
- Negative velocity: percent_sign +1, k 11.81, p0 20.21, min 32.
- `max_speed_rad_per_sec` 6.76 (fit).

**Raw against operating: the speeds agree.** Operating / raw slope per step:

- Median 0.99 from 40 % up.
- From 50 % up the ratios lie between 0.96 and 1.00:

  | Percent | 50 | 60 | 70 | 80 | 90 | 100 |
  |---|---:|---:|---:|---:|---:|---:|
  | + % | 1.00 | 0.98 | 0.96 | 0.98 | 0.99 | 1.00 |
  | - % | 1.00 | 0.97 | 0.99 | 1.00 | 0.99 | 0.98 |

- Off the line: +40 % at 1.16 and -30 % at 0.86.

As on 2026-10-14 and this morning, MaxSpeed 80 does not scale `:DPR` output measurably under the
dome's load. The fits differ by up to 2 points in p0 and 4 % in k. This is within the run-to-run
scatter at the low end, which carries most of the weight of p0. The two runs predict the same
percent within 1.5 points from 2 rad/s up.

## Sensor sweep (`20261016-131650-sensor`, raw motor)

Jump threshold 576.02 deg/s (from this session's speed sweep). Reversals and stalls are scored
only at or above 30 % (positive) / 27 % (negative), so the two 20 % steps are not scored.

| # | Percent | Samples | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Jumps | Reversals | Stalls | Scored | Same-cycle pairs | Max gap s | End |
| 0 | +20 | 304 | 7.3 | 0.26 | -0.253 | -0.223 | 0.138 | 0 | 0 | 0 | no | 29 | 0.067 | stalled (9.3 s) |
| 1 | -20 | 287 | 7.0 | 0.28 | 0.244 | 0.263 | 0.181 | 0 | 0 | 0 | no | 28 | 0.233 | stalled (8.9 s) |
| 2 | +35 | 961 | 30.0 | 6.60 | -1.382 | -1.388 | 0.257 | 0 | 0 | 0 | yes | 38 | 0.067 | completed |
| 3 | -35 | 954 | 30.0 | 5.87 | 1.234 | 1.236 | 0.372 | 0 | 0 | 0 | yes | 39 | 0.233 | completed |
| 4 | +50 | 1007 | 30.0 | 12.77 | -2.679 | -2.670 | 0.223 | 0 | 0 | 0 | yes | 53 | 0.067 | completed |
| 5 | -50 | 1010 | 30.0 | 12.30 | 2.596 | 2.580 | 0.288 | 0 | 0 | 0 | yes | 58 | 0.233 | completed |
| 6 | +75 | 986 | 30.0 | 22.85 | -4.793 | -4.781 | 0.174 | 0 | 0 | 0 | yes | 37 | 0.067 | completed |
| 7 | -75 | 988 | 30.0 | 23.25 | 4.869 | 4.864 | 0.291 | 0 | 0 | 0 | yes | 41 | 0.233 | completed |
| 8 | +100 | 969 | 30.0 | 31.29 | -6.567 | -6.551 | 0.233 | 0 | 0 | 0 | yes | 21 | 0.067 | completed |
| 9 | -100 | 968 | 30.0 | 31.70 | 6.647 | 6.626 | 0.352 | 1 | 0 | 0 | yes | 32 | 0.233 | completed |

- **Totals:** 8434 samples, 154.2 revolutions, **1 jump, 0 reversals, 0 stalls** (scored).
- **`suggested_pass` is false** because of the one jump.
- **Jump histogram:** a single entry, in the 350 deg bin.
- **`spikes.py`:** **no excursion**.
- **20 % steps:** +20 % moved 111 deg and stalled at 261 deg; -20 % moved 176 deg and stalled at
  77 deg.
- **Sustained top speed:** at ±100 % the dome held 6.57 / 6.65 rad/s for 30 s, within 2 % of the
  4 s speed steps. Without the dome this morning the -100 % sensor step sagged to 5.95 rad/s. That
  sag did not recur.

### The one scored jump: the 355 to 4 deg dead zone

**The event.** Step 9 (-100 %), 13:21:09.37.

- Reports: 335, 349, **355, 17**, 29, 41.
- Steps: 14, 6, **22**, 12, 12 deg, all at 33 ms intervals with consecutive sequence numbers.
- The 22 deg step is just over the threshold of 576.02 x 0.0333 + 2 = 21.2 deg.
- The short step before it means two consecutive reports average 14 deg, the normal rate at
  6.65 rad/s.
- The position stays on its track on both sides: it does not leave the line and return.

**The cause** is a property of the angle the ring reports near 0, not a dome or plate fault:

- **The controller skips 359 to 2 deg.** In every sensor and speed run, on both plates, the
  controller essentially never reports 359, 0, 1 or 2 deg. 356 to 358 and 3 appear rarely. 355 and
  4 are over-represented: at ≤ 50 % in this run, 355 appeared 77 times and 4 appeared 48 times,
  while 359 to 2 appeared 0 times. This morning's no-dome run and the 2026-10-14 old-plate
  with-dome run show the same pattern.
- **The raw ring stream shows why.** In the coordinator's runs (`ring_newplate_dome_run.csv` and
  the no-dome `ring_newplate_run.csv`), the ring's median-filtered angle never takes the values 358,
  359, 0 or 1. Raw angles 351, 355 and 0 (and 6 and 14) are hardly ever seen, so the pattern edges near there
  switch together. The decode table has a code for every angle, so the gap is in the pattern or its
  alignment, not in the table.
- **The effect.** Crossing 0 deg always includes about 9 deg that the reported angle skips. Across
  the 32 wraps at -100 % in this run, the step across the wrap was 11 to 22 deg (median 15), while
  the median step overall was 13 deg.

**It was predicted.** This morning's no-dome sweep had the same event at the same place: +20 deg at
355 deg at -100 %, 95 % of its threshold. That report predicted that a slightly faster or
worse-timed crossing would score a false jump. Today the -100 % step ran at 6.65 rad/s (this
morning 5.95), and it did.

**Verdict: the sensor result is a pass for the plate.** There is no spike and no excursion, and the
one scored jump is the known wrap dead zone. For future sweeps, the bench node's jump threshold
needs an allowance of about 10 deg for this, or a factor larger than 1.5 (2.0 would allow 28 deg at
100 %).

### The angle-264 edge race (ring debug addendum 3)

The angle-264 edge race does not appear as a jump or an excursion in any of the four runs.

| Run | Crossings of 264 deg, decreasing / increasing |
|-----|----------------------------------------------:|
| Complete sensor sweep | 78 / 87 |
| Raw speed run | 31 / 28 |
| Operating speed run | 28 / 35 |
| Aborted sensor sweep | 22 / 24 |

The largest residual of any of those pairs against the local motion was 3 deg, which is normal
quantisation and timing noise. The ring's median filter removes the one-sample race, as in the raw
ring run below.

## First sensor sweep (`20261016-130416-sensor`, aborted)

| # | Percent | Samples | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Jumps | Reversals | Stalls | Scored | Same-cycle pairs | Max gap s | End |
| 0 | +20 | 290 | 6.9 | 0.16 | -0.138 | -0.155 | 0.105 | 0 | 0 | 0 | no | 24 | 0.067 | stalled (8.9 s) |
| 1 | -20 | 111 | 1.3 | 0.01 | 0.032 | 0.052 | 0.048 | 0 | 0 | 0 | no | 11 | 0.033 | stalled (3.3 s) |
| 2 | +35 | 959 | 30.0 | 5.85 | -1.217 | -1.229 | 0.294 | 0 | 0 | 0 | yes | 44 | 0.067 | completed |
| 3 | -35 | 954 | 30.0 | 5.72 | 1.208 | 1.197 | 0.364 | 0 | 0 | 0 | yes | 51 | 0.233 | completed |
| 4 | +50 | 1009 | 30.0 | 12.01 | -2.510 | -2.514 | 0.269 | 0 | 0 | 0 | yes | 58 | 0.233 | completed |
| 5 | -50 | 1009 | 30.0 | 12.77 | 2.681 | 2.677 | 0.229 | 0 | 0 | 0 | yes | 51 | 0.067 | completed |
| 6 | +75 | 70 | 0.2 | 0.14 | -4.749 | -4.399 | 0.084 | 0 | 0 | 0 | yes | 4 | 0.067 | incomplete |

- **Up to the abort:** 4402 samples, about 36 revolutions at 35 and 50 %, **0 jumps, 0 reversals,
  0 stalls**. `spikes.py` found no excursion.
- **This is the speed range where the old plate with the dome failed most.**
- **The largest step was 66 % of the threshold.**
- **The per-step speeds agree with the complete run:** within 2 to 12 % at 35 % and 3 to 6 % at
  50 %, typical of the scatter at those speeds.

## Telemetry dropout in the first sensor sweep

- **The abort.** At 13:06:45.65 the bench node aborted with `telemetry dropout: no message for
  1.05 s`. The last sample was at 13:06:44.60, 3.3 s into the +75 % step: 243 deg, about
  -4.5 rad/s, sequence 14612.
- **The node then logged "No standstill within 5 s of the stop".** Standstill needs a fresh
  telemetry message showing 0 % and no motion. The node reached standstill within 5 s after every
  stop in the speed sweep, including from ±100 %. So telemetry was probably missing for longer than
  1.05 s, though the length cannot be measured from this run.
- **The node restored `max_speed` 80**, which the firmware acked; `settings_restored` and
  `firmware_acked` are true.
- **Afterwards.** Telemetry was flowing again a few seconds later: dome at rest at 12 deg, 0 %.
  Following the session's safety rule, a final `DomeBenchDrive {percent: 0}` was sent ("bench drive
  already stopped") and the controller manager was stopped (SIGINT; clean deactivate).
- **Cause not found.**
  - No CPU-heavy work was running on the Pi.
  - `cm.log` has no warning and the router log is clean.
  - The journal has nothing at that time apart from the usual `cyphal_service` messages, and there
    was no USB event. `vcgencmd get_throttled` read 0x0.
  - `position_timeouts` stayed 0. That counter counts the firmware's move timeout, not a pause in
    reports.
  - The firmware prints `DOME POSITION` unconditionally in its main loop, and its `#DPSTATUS`
    handler does nothing that blocks. A periodic status poll was due about 0.4 s before the gap.
- **Similar to 2026-10-15.** That abort (1.03 s, at +15 %) also ended a sweep with the whole stack
  at nice 19, inherited from the shell that started it. That one happened under heavy CPU load and
  this one without.
- **The firmware's sensor error counter went from 0 to 3 during this sweep.** It was 0 at its start
  and 3 at the end. It had been 0 for about 337 revolutions earlier today (279 in this morning's
  session, 58 in this session's speed sweep), and it stayed at 3 for the rest of the session (211
  more revolutions). The counter is polled every 10 s, so it cannot show whether the 3 errors
  coincide with the dropout.

**Restart.** With the coordinator's approval the stack was restarted (new router,
`robot_state_publisher`, controller manager, bench node).

- **Priority.** The main thread of each process was raised from nice 19 to 0 with
  `sudo -n renice -n 0 -p <pid>`. A per-thread renice was refused by the session's permission
  system, so the other threads (zenoh transport, the plugin's serial and executor threads) stayed at
  19 (`nice_values.txt`).
- **Recorder.** `ros2 topic echo /roamadome/telemetry --field header.stamp` ran from 13:16:12 to
  13:28:43:

| Messages | Span | Rate | Median | p99 | p99.9 | < 5 ms | > 50 ms | > 100 ms | > 200 ms | > 300 ms | Max |
|---------:|-----:|-----:|-------:|----:|------:|-------:|--------:|---------:|---------:|---------:|----:|
| 24044 | 751.3 s | 32.0 /s | 33.3 ms | 66.6 ms | 66.7 ms | 1879 | 304 | 15 | 14 | 0 | 233.4 ms |

- **Gaps over 100 ms (15).** Twelve are the 233 ms gap of the plugin's 60 s config refresh, at :06
  past each minute. The other three came with a settings write or restore: 233 ms at 13:16:51 and
  167 ms and 200 ms at 13:21:36.
- **The dropout did not recur.** This run cannot tell whether the restart, the priority change or
  chance made the difference. The stamps file stays in the run directory for comparison if it
  happens again.

## Telemetry timing (from `samples.csv`)

Intervals between consecutive reports within a step (host stamps):

| Run | Pairs | Same-cycle (< 5 ms) | Median | p99 | > 50 ms | > 100 ms | > 200 ms | Max |
|-----|------:|--------------------:|-------:|----:|--------:|---------:|---------:|----:|
| speed (raw) | 5741 | 384 (6.7 %) | 33.3 ms | 66.7 ms | 129 | 4 | 4 | 233.3 ms |
| sensor (aborted) | 4395 | 243 (5.5 %) | 33.3 ms | 66.7 ms | 111 | 2 | 2 | 233.3 ms (the dropout came after the last row) |
| sensor | 8424 | 376 (4.5 %) | 33.3 ms | 66.6 ms | 155 | 5 | 5 | 233.3 ms |
| speed (operating) | 5562 | 397 (7.1 %) | 33.3 ms | 66.6 ms | 128 | 4 | 4 | 233.4 ms |

## Firmware sensor error counter (`#DPSTATUS`, from `/diagnostics` and `run.json`)

| Point | Time | `sensor_errors` |
|-------|------|----------------:|
| Session start, at rest | 12:59 | 0 |
| Speed sweep (raw) start / end | 12:59:50 / 13:03:19 | 0 / 0 |
| Sensor sweep (aborted) start / end | 13:04:16 / 13:06:55 | 0 / **3** |
| After the restart, at rest | 13:15 | 3 |
| Sensor sweep start / end | 13:16:50 / 13:21:38 | 3 / 3 |
| Speed sweep (operating) start / end | 13:23:59 / 13:27:18 | 3 / 3 |
| Session end, at rest | 13:27 | 3 |

- **+3 over about 309 revolutions** (58.4 + 39.5 + 154.2 + 57.4), or 0.01 per revolution. On the
  old plate the rate was 2.2 to 2.9 per revolution, with and without the dome.
- All 3 came in the aborted sweep (see above).
- `packet_serial_errors` and `position_timeouts` stayed 0, and 76 `#DPSTATUS` polls were answered on
  the second stack.

## Raw sensor ring check with the dome on the new plate

The coordinator took this run minutes before the bench session, with the dome mounted on the new
plate, using the same scripts as the [ring debug report](2026-10-15-sensor-ring-raw-debug.md)
(addendum 4 there).

- **Coverage:** 7493 ring samples over about 22 revolutions at 35 % and 50 %, both directions,
  plus parked holds at 238 and 200.
- **Files:** `/home/pi/dome_bench/ring_debug/ring_newplate_dome_run.csv` and
  `ctl_newplate_dome_run.csv`.

| Spurious "B" by track position | Old plate, dome (runs 1+2) | New plate, dome |
|---|---|---|
| sensor 0 at 198-201 / 234-237 / 252 | 18 / 6 / 6 | 0 / 0 / 0 |
| sensor 1 at 198-201 / 237 / 252 | 5 / 9 / 5 | 0 / 0 / 0 |
| raw glitches per 1000 samples | 5.0 | 1.5 (11 in 7493) |

- Sensors 0 and 1 show **no spurious reading anywhere on the track**. The parked holds at 238 and
  200 were steady (no bit flicker).
- The only raw events are 11 one-sample occurrences of the known angle-264 edge race: sensor 3 sets
  one sample before sensor 1 clears, angle-decreasing direction only. The ring's median filter
  removes them.
- On the controller side, position steps over 8 deg were only the 359/0 wraparound.
- The bench sweeps above agree: no spike or excursion anywhere.

## Low end and stall angles

Stall end angles: the last report of each stalled step that moved. "No motion" means the step did
not leave its start angle.

| Run | Positive percent (angle decreasing) | Negative percent (angle increasing) |
|-----|-------------------------------------|-------------------------------------|
| Speed, raw | +10 no motion at 50; +15 at 36; +20 at 4; +25 at 346; +27 at 235; +28 at 245 (after 2.9 rev) | -10 no motion at 50; -15 at 55; -20 at 77; -25 at 190 |
| Sensor (aborted) | +20 at 12 | -20 at 28 (moved 16 deg) |
| Sensor | +20 at 261 | -20 at 77 |
| Speed, operating | +10 / +15 no motion at 276; +20 at 288; +25 at 253; +30 at 237 | -10 no motion at 276; -15 at 15; -20 at 67; -22 at 74; -23 at 102 (moved 28 deg) |

- **Angle decreasing:** the stops cluster at 235 to 261 deg (235, 237, 245, 253, 261). This
  morning's no-dome run on the new plate stopped at 230 to 253, the same spot. With the dome on the
  old plate the stops were at 37 to 57 deg.
- **Angle increasing:** the stops cluster at 55 to 102 deg (55, 67, 74, 77, 77, 102), with single
  stops at 190, 15 and 28. With the dome on the old plate this region was at 105 to 188. Without
  the dome on the new plate it was at 339 to 348.
- **Interpretation.** The plate swap meant taking the ring side apart, and the tight spots in the
  drive have moved with it. The low end still depends on position, as on every earlier run.
- **No step below 25 % ran continuously in any run.** The slowest dependable speed is still about
  1 rad/s: +32 % ran at 1.08 rad/s, and ±35 % at 1.2 to 1.4 rad/s for 30 s.

## Comparison: old plate with dome, new plate without and with dome

- **Notation.** "+ %" is positive percent (reported angle decreasing, negative joint velocity),
  "- %" negative percent. Raw = `raw_motor` true, op = operating settings.
- **Time to 90 %** uses `speedcmp.py` for every column: the median, over the moving steps at 40 %
  and above, of the time from the command until the 0.5 s windowed velocity reaches 90 % of the
  step's measured speed.
- **2026-10-15 column.** The spike-damaged raw run was re-analysed with the spikes removed (see that
  report). The operating run aborted.

| Quantity | Old plate, dome (2026-10-14) | Old plate, re-seated (2026-10-15) | New plate, no dome (2026-10-16 AM) | **New plate, dome (this run)** |
|----------|---:|---:|---:|---:|
| omega at 100 %, measured, raw (+ % / - %) | 6.609 / 6.558 | 6.743 / 6.639 | 6.685 / 6.651 | **6.702 / 6.696** |
| omega at 100 %, measured, op (+ % / - %) | 6.643 / 6.596 | not measured | 6.998 / 6.845 | **6.711 / 6.559** |
| omega at ±100 % sustained 30 s, sensor sweep (+ % / - %) | 6.622 / 6.536 | 6.743 / 6.592 | 6.460 / 5.945 | **6.567 / 6.647** |
| `max_speed_rad_per_sec` to configure (lower measured 100 %) | 6.56 | 6.56 (kept) | 6.65 (provisional) | **6.56** |
| k raw (+ % / - %, rad/s per %) | 0.08536 / 0.08502 | 0.08457 / 0.08184 | 0.08330 / 0.08249 | **0.08828 / 0.08717** |
| p0 raw (+ % / - %) | 20.73 / 20.01 | 18.50 / 16.88 | 17.20 / 17.07 | **22.08 / 20.45** |
| R squared raw (+ % / - %) | 0.9911 / 0.9911 | 0.9971 / 0.9980 | 0.9949 / 0.9968 | **0.9936 / 0.9967** |
| k op (+ % / - %) | 0.08205 / 0.09224 (glitches) | not measured | 0.08711 / 0.08650 | **0.08469 / 0.08619** |
| p0 op (+ % / - %) | 18.82 / 25.13 (glitches) | not measured | 19.01 / 18.31 | **20.21 / 20.59** |
| min_percent raw (+ % / - %) | 27 / 27 | 27 / 25 | 27 / 25 | **30 / 27** |
| min_percent op (+ % / - %) | 32 / 32 | not measured | 32 / 25 | **32 / 25** |
| Velocity at min_percent, raw (+ % / - %) | 0.414 / 0.186 | 0.762 / 0.559 | 0.757 / 0.675 | **0.620 / 0.449** |
| Velocity at min_percent, op (+ % / - %) | 1.000 / 0.743 | not measured | 1.092 / 0.501 | **1.078 / 0.350** |
| Asymmetry, fit, raw (op) | 0.995 (0.965) | 1.013 | 1.008 (0.998) | **0.992 (0.987)** |
| Asymmetry, measured 100 %, raw | 1.008 | 1.016 | 1.005 | **1.001** |
| Operating / raw speed, median from 40-50 % up | 0.98 | not measured | 1.02 | **0.99** |
| Time to 90 % of step speed, median (range), raw | 1.37 s | 1.43 s | 0.60 s (0.57 to 1.00) | **1.58 s (0.80 to 2.17)** |
| Time to 90 %, median (range), op | not recomputed | not measured | 0.65 s (0.57 to 1.10) | **1.65 s (0.80 to 2.83)** |
| Sensor errors per revolution (session) | 2.8 | 2.3 | 0 | **0.01** (3 in 309 rev, all in the aborted sweep) |
| Scored jumps: speed raw / sensor / speed op | 2 / 6 / 28 | 50 / 102 / aborted | 0 / 0 / 0 | **0 / 1 / 0** (sensor: 355/4 wrap, not a spike; aborted sweep 0) |
| Excursions >= 10 deg off local motion (`spikes.py`), speed + sensor runs | 20 | 81 | 0 | **0** (one spin-up artefact at the start of -100 % in the raw run, not a sensor event) |
| Reversals (sensor sweep, scored) | 0 | 1 | 0 | **0** |
| `suggested_pass` (sensor) | false | false | true | **false** (the one wrap jump; aborted run true) |
| Largest step / jump threshold (sensor sweep) | jumps | jumps | 95 % | **104 %** (the wrap jump; aborted run 66 %) |
| Sensor sweep revolutions | 152.2 | 155.6 | 152.5 | **154.2** (+ 39.5 aborted) |
| Stall end angles, + % | 37-57 (one at 7) | 43-56, 238 | 230-253 | **235-261** (and 346, 4, 36, 288) |
| Stall end angles, - % | 105-188 | 93-202 | 339-348 | **55-102** (and 190, 15, 28) |
| Telemetry gaps (max, median) | 233 ms, 33.3 ms | 233 ms, 33.3 ms (one 1.03 s dropout, abort) | 233 ms, 33.3 ms | **233 ms, 33.3 ms** (one dropout ≥ 1.05 s, abort; none in 751 s of stamps after the restart) |

The 2026-10-14 operating-run time to 90 % is not in the table: the 2026-10-15 re-analysis covered
the raw runs only.

## Interpretation

- **The new plate fixes the with-dome position fault.**
  - On the old plate the dome turned single-bit misreads on sensors 0 and 1 into spikes that passed
    the median filter: 6, then 102 jumps in the sensor sweep, at the same three track positions.
  - On the new plate, with the dome on, the raw ring stream has no spurious reading on those sensors.
    The bench sweeps show no spike or excursion in about 309 revolutions.
  - The firmware's sensor link error counter, which rose by 2.2 to 2.9 per revolution on the old
    plate, rose by 3 in total.
  - The one scored jump is the dead zone at 355 to 4 deg. It is present on both plates and with and
    without the dome. It scores only when a fast crossing lines up with the report timing.
- **The dome barely changes the steady-state drive.**
  - Against this morning's no-dome run on the same plate, the 100 % rates differ by 0.3 to 4 %, k
    by 6 % and p0 by 3 to 5 points. The low end is 3 points higher in the raw run (30 / 27 against
    27 / 25).
  - The velocity at the minimum percent is lower, which is stick-slip.
  - Against the old plate with the dome, k and p0 are within 4 % and 1.5 points, `max_speed` is the
    same 6.56, and the operating-settings `min_percent` is the same 32 for positive percent.
- **The dome slows the response.**
  - Time to 90 % of the step speed is 1.58 s (raw) and 1.65 s (operating), against 0.60 s and
    0.65 s without the dome. That is a factor of 2.6, slightly more than the factor of 2.3 seen on
    the old plate (1.37 s).
  - With Scaling off, this is the Syren ramp and current limit acting on the dome's inertia. Any
    controller tuning should use the with-dome response.
- **Sustained top speed held** at ±100 % for 30 s (6.57 / 6.65 rad/s). This morning's no-dome sag at
  -100 % (5.95 rad/s) did not repeat, so it was not a load effect.

## Observations

1. **Phase 7 acceptance at 0.5 rad/s is not reachable with the dome**, as on 2026-10-14.
   - +0.5 rad/s (negative percent): the fit gives 26.2 %, which clamps to 27. In the raw run 27 %
     moved at 0.45 rad/s, just outside 10 %.
   - -0.5 rad/s (positive percent): the fit gives 27.7 %, which clamps to 32. That runs at about
     1.08 rad/s.
   - **1.0 rad/s works in both directions:** 31.9 % and 33.4 % from the fit, at or above both
     `min_percent` values. The README's acceptance check now uses 1.0 rad/s with the dome (plan
     section 10 decision for the human).
2. **Telemetry dropout.** It is the second in three with-dome sessions; the cause is still not found
   (see the dropout section). The restart with the main threads at nice 0 and a stamps recorder ran
   12.5 minutes with nothing over 233 ms. Keep the recorder running in future sweeps.
3. **Jump threshold margin at 100 %.** The 355 to 4 deg dead zone adds about 9 deg to one step per
   revolution. With the threshold at 1.5 x the measured maximum, a 100 % crossing sits within 5 % of
   the threshold or just over it. This needs a bench-node change (a larger factor or a fixed allowance)
   before the sensor sweep's `suggested_pass` can be relied on at 100 %. The same dead zone means
   the reported angle cannot resolve 356 to 3 deg at any speed. A `:DPA` target there will settle
   at 355 or 4.
4. **Sign convention unchanged.** Positive percent moves the reported angle down
   (`angle_sign_for_positive_percent` -1, 25 of 25 votes in both speed runs, 7 and 10 in the sensor
   sweeps).
5. **Startup.** Neither stack start had a startup command timeout this time. `/diagnostics` again
   showed `position_reporting_enabled: false` while reports flowed.
6. **Shutdown.** `ros2_control_node` stopped cleanly on SIGINT both times (hardware deactivated),
   with the same `pal_statistics` "context cannot be slept with" errors at exit as before.

## MinSpeed follow-up (2026-10-16, after the run)

The firmware's `MinSpeed` was 15 while the measured minimum sustainable
percent is 27-32, so `:DPR` requests between 15 and 26 were accepted and
stalled, and position moves floored their final approach at 15 %. `MinSpeed`
was set to 32 through `device.min_speed` (acked: `#DPMINSPEED32`, Write
Settings, Updated; persisted). It is now part of the calibration state.

Check, same session, dome installed (`/home/pi/dome_bench/minspeed_check/`):

- `:DPR20` after the change: the firmware answered `ROTATE SPEED: 0` and the
  dome did not move. `:DPR35`: accepted, 1.35 rad/s. The sub-stall creep is gone.
- Position moves (90, 270, 180, 0 degrees, full `SpeedTarget`) before and after:

| MinSpeed | target | REACHED TARGET reported | rest position | overshoot |
|---|---|---|---|---|
| 15 | 90 / 270 / 180 / 0 | 85 / 265 / 184 / 5 | 114 / 315 / 159 / 329 | +24 / +45 / -21 / -31 |
| 32 | 90 / 270 / 180 / 0 | 85 / 265 / 185 / 5 | 118 / 317 / 155 / 327 | +28 / +47 / -25 / -33 |

No position timeouts either way. `MinSpeed` makes no difference to the
overshoot, which is 25-47 degrees with the dome: the firmware declares the
target reached at the 5 degree fudge boundary while the dome is still turning
at about 3.5 rad/s (peak 3.5-4.2 rad/s on a 135-180 degree move), then the
Sabertooth ramp (`setRamping(80)`, fixed in `DomeDriveSabertooth::setup`) and
the dome's inertia carry it on. Settling takes 3.1-3.7 s per move. The 50
degree `DecelerationScale` is too short for a dome whose response time is
1.5 s; raising `device.deceleration_scale` (for example to 150) and lowering
`device.speed_target` for position moves are the firmware-side knobs to try.
Without the dome (phase 2 check) the same moves overshot by only a few degrees.

## Position-move tuning (2026-10-16, after the run)

Method and data: `/home/pi/dome_bench/pos_tune/` (`run1.csv` to `run3.csv`,
`pos_tune.py`, `pos_tune_analyze.py`); procedure written up in the README,
re-tuning section (g).

Stopping distance after `:DPR0` from steady rotation (dome installed), symmetric:

| percent | steady deg/s | stop distance deg | stop time s |
|---|---|---|---|
| 32 / -32 | 43 / 37 | 12 / 8 | 0.65 / 0.60 |
| 36 / -36 | 57 / 81 | 16 / 17 | 0.70 / 0.75 |
| 35 / -35 | 73 / 58 | 19 / 18 | 0.65 / 0.70 |
| 50 / -50 | 141 / 155 | 46 / 45 | 0.80 / 0.90 |
| 70 / -70 | 229 / 257 | 90 / 89 | 1.05 / 1.00 |
| 100 / -100 | 392 / 369 | 161 / 171 | 1.10 / 1.20 |

Fit: `d_stop = 0.469 s x v - 18.2 deg`. Position moves (150 and 40 deg, both
directions) under candidate settings, rest error per move:

| SpeedTarget | DecelerationScale | Fudge | rest errors | arrival speed at fudge |
|---|---|---|---|---|
| 100 | 50 | 5 (original) | +47 -30 +3 -13 +49 -22 | 80-210 deg/s |
| 100 | 150 | 5 | +18 -3 -20* -1 +21 0 | 44-126 |
| 60 | 150 | 5 | +14 -2 +5 -2 +11 -4 | 41-102 |
| 45 | 200 | 5 | +10 0 +12 -1 +7 -1 | 25-84 |
| 35 | 250 | 5 | +12 -4 +10 -5 +10 +2 | 38-96 |
| 45 | 250 | 8 | +6 -6 0 -8 +2 -10 | 52-89 |
| 45 | 250 | 12 | -6 -8 -7 -1 -8 -6 | 30-98 |
| 60 | 250 | 10 | -2 -5 -2 -1 -5 -2 | 52-98 |

`*` the 40 degree move at the 32 % floor stalled 20 degrees short at a tight spot
near 200 degrees (no REACHED TARGET).

The arrival speed at the floor command varies by a factor of two with the end
angle (position-dependent friction, the same tight spots seen in the sweeps),
not consistently by direction, which is why a fudge in the middle of the
stopping-distance range (10 deg) balances the moves. With the 250 degree taper
the firmware's own deceleration ramp divisor (the same setting) is also at its
slowest, which did not hurt here because the ramp jumps on direction changes.

Applied: `SpeedTarget` 60, `DecelerationScale` 250, `Fudge` 10 (all acked and
persisted). Originals were 100 / 50 / 5. Follow-up for the bench node: a
`start_position_sweep` service that automates these two measurements.

## Next

Phase 7 can proceed with [THE VALUES TO CONFIGURE](#the-values-to-configure). Set the acceptance
point to 1.0 rad/s with the dome, or record the decision. The bench node's jump threshold needs an
allowance for the 355 to 4 deg dead zone before the sensor sweep is used as an automatic gate at
100 %.
