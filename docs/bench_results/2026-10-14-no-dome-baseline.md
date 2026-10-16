# Dome bench baseline, 2026-10-14: bare base platform, NO DOME INSTALLED

**The dome shell was not installed for any of these measurements.** The motor drove only the
bare base platform (the rotating ring without the dome shell on it). The figures below are a
baseline for the bare base platform, **not** a calibration for the assembled robot. The same
applies to the phase 1 and phase 5 numbers taken earlier on the same day. The sweeps must be
re-run with the dome mounted before any of these values configure the velocity mapping; see
"Re-tuning the velocity mapping when the dome changes" in the [README](../../README.md).

Phase 6 of [docs/dome_diagnostics_plan.md](../dome_diagnostics_plan.md), run with a human at
the robot and the platform clear.

| | |
|---|---|
| Date | 2026-10-14, 21:37 to 21:51 local (-0400) |
| Stack | Bench stack: `robot_state_publisher` with a dome-only URDF (`enable_bench_mode`, `publish_telemetry`), `ros2_control_node` with only the RoamADome hardware at 30 Hz, no controllers; `rmw_zenoh_cpp` |
| Code | `ros2_roamadome` `diag/phase-5-bench-node` at `fa97b3b` (bench node and analysis), `#DPREPORT28` |
| Bench node | `ros2 run ros2_roamadome roamadome_bench --ros-args -p output_dir:=/home/pi/dome_bench/phase6 -p hardware_node:=roamadome`, plan 5.4 defaults |
| Results | [2026-10-14-no-dome-baseline-speed.json](2026-10-14-no-dome-baseline-speed.json), [2026-10-14-no-dome-baseline-sensor.json](2026-10-14-no-dome-baseline-sensor.json), [2026-10-14-no-dome-baseline-speed-operating.json](2026-10-14-no-dome-baseline-speed-operating.json) (copies of the bench node's result files, unmodified) |
| Raw data (not committed) | `/home/pi/dome_bench/phase6/20261014-213751-speed/`, `20261014-214159-sensor/`, `20261014-214737-speed/` (each with `samples.csv`, `run.json`, `settings_before.json`, `settings_after.json`); bench logs `bench_speed.log`, `bench_sensor.log`, `bench_speed_operating.log`; `cm.log`; baseline `baseline_params.yaml` and `/diagnostics` captures; `notes.txt` records that no dome was installed |

The bench node's result and `run.json` files have no free-text field, so the "no dome installed"
note lives in this report and in `/home/pi/dome_bench/phase6/notes.txt`, not in the JSON.

## Runs

| Run | Sweep | Settings during the run | Status | Settings restored |
|-----|-------|-------------------------|--------|-------------------|
| `20261014-213751-speed` | Speed, plan defaults (12 percents, `raw_motor` true) | MaxSpeed 100, SpeedTarget 100, Scaling off | complete, 27 steps, 2 min 53 s | yes, firmware acked (`#DPMAXSPEED80`), `settings_restored: true` |
| `20261014-214159-sensor` | Sensor, plan defaults (20/35/50/75/100, 30 s, both directions, `raw_motor` true) | MaxSpeed 100, SpeedTarget 100, Scaling off | complete, 10 steps, 4 min 41 s | yes, firmware acked, `settings_restored: true` |
| `20261014-214737-speed` | Speed, plan defaults but `raw_motor` false (supplementary, see observations) | Operating settings: MaxSpeed 80, SpeedTarget 100, Scaling off | complete, 27 steps, 3 min 6 s | nothing written, nothing to restore |

The speed sweep ran first so the sensor sweep could take its jump threshold from a measured
maximum (plan 7.2). The sensor result's config confirms it: `max_plausible_deg_s` 587.47 =
1.5 x 6.836 rad/s (391.6 deg/s), source `1.5 x measured maximum in
.../20261014-213751-speed/speed_sweep.json`, and `min_percent_known` 27 / 25 from the same file.
No run aborted; no telemetry dropout; no settings-ack timeout. `roamadome_bench_report <run_dir>
--check` reproduces all three result files (exit 0, "result matches"). After every run
`ros2 param get` returned `device.max_speed` 80, `device.speed_target` 100, `device.scaling`
False (and `min_speed` 15, `inverted` True, `home_pos` 0, `auto_safety` False, never written).

## Firmware configuration (`device.*` at the start of the session)

From `ros2 param dump /roamadome` (the plugin's `#DPCONFIG` parse), unchanged at the end:

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

## Speed sweep (`20261014-213751-speed`, raw motor)

Positive percent moves the reported angle negative (`angle_sign_for_positive_percent` -1,
20 of 20 moving steps agree). Velocities are signed in the reported-angle sense; the fit uses
the speed (|slope|). "Stalled (after s)" is the time from the command to the stall decision
(average speed below 3 deg/s over 2 s).

| # | Kind | Percent | Samples | Moving | Stalled (after s) | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Max gap s |
|---|------|--------:|--------:|:------:|------------------:|----------:|-----:|------------:|-----------:|----------:|----------:|
| 0 | step | +10 | 70 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 1 | step | -10 | 70 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 2 | step | +15 | 70 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 3 | step | -15 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 4 | step | +20 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 5 | step | -20 | 70 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 6 | step | +25 | 186 | no | yes (5.8) | 3.8 | 0.28 | -0.465 | -0.452 | 0.480 | 0.233 |
| 7 | step | -25 | 235 | yes | no | 5.7 | 0.50 | 0.553 | 0.558 | 0.169 | 0.067 |
| 8 | step | +30 | 213 | yes | no | 5.0 | 0.64 | -0.872 | -0.808 | 0.370 | 0.067 |
| 9 | step | -30 | 496 | yes | no | 14.3 | 1.95 | 0.861 | 0.852 | 0.357 | 0.067 |
| 10 | step | +40 | 188 | yes | no | 4.0 | 1.06 | -1.693 | -1.651 | 0.397 | 0.067 |
| 11 | step | -40 | 191 | yes | no | 4.1 | 1.07 | 1.633 | 1.640 | 0.306 | 0.067 |
| 12 | step | +50 | 190 | yes | no | 4.0 | 1.73 | -2.732 | -2.698 | 0.251 | 0.067 |
| 13 | step | -50 | 189 | yes | no | 4.0 | 1.72 | 2.775 | 2.680 | 0.459 | 0.233 |
| 14 | step | +60 | 187 | yes | no | 4.0 | 2.25 | -3.578 | -3.557 | 0.214 | 0.067 |
| 15 | step | -60 | 188 | yes | no | 4.0 | 2.35 | 3.667 | 3.687 | 0.179 | 0.033 |
| 16 | step | +70 | 186 | yes | no | 4.0 | 2.78 | -4.323 | -4.296 | 0.127 | 0.067 |
| 17 | step | -70 | 187 | yes | no | 4.0 | 2.65 | 4.188 | 4.222 | 0.293 | 0.067 |
| 18 | step | +80 | 185 | yes | no | 4.0 | 3.14 | -4.988 | -4.968 | 0.169 | 0.067 |
| 19 | step | -80 | 185 | yes | no | 4.0 | 3.26 | 5.089 | 5.108 | 0.218 | 0.067 |
| 20 | step | +90 | 183 | yes | no | 4.0 | 3.56 | -5.568 | -5.610 | 0.171 | 0.067 |
| 21 | step | -90 | 182 | yes | no | 4.0 | 3.91 | 6.143 | 6.134 | 0.127 | 0.067 |
| 22 | step | +100 | 183 | yes | no | 4.0 | 4.05 | -6.347 | -6.314 | 0.242 | 0.233 |
| 23 | step | -100 | 181 | yes | no | 4.0 | 4.35 | 6.836 | 6.808 | 0.119 | 0.067 |
| 24 | search | +27 | 389 | yes | no | 10.6 | 1.18 | -0.631 | -0.679 | 0.393 | 0.067 |
| 25 | search | -22 | 140 | no | yes (4.3) | 2.3 | 0.03 | 0.057 | 0.099 | 0.110 | 0.067 |
| 26 | search | -23 | 88 | no | yes (2.7) | 0.7 | 0.00 | -0.023 | -0.018 | 0.017 | 0.067 |

### Per-direction fit, `omega = k * (p - p0)` over the moving steps

| | Positive percent (angle decreasing) | Negative percent (angle increasing) |
|---|---:|---:|
| k (rad/s per %) | 0.07875 | 0.08505 |
| p0 (%) | 17.42 | 19.09 |
| R squared (9 points) | 0.9942 | 0.9969 |
| omega at 100 %, measured | 6.347 rad/s | 6.836 rad/s |
| omega at 100 %, from the fit, k (100 - p0) | 6.504 rad/s | 6.882 rad/s |
| min_percent (bracketed to 2 %) | 27 (25 stalled) | 25 (23 stalled) |
| Velocity at min_percent | 0.631 rad/s | 0.553 rad/s |
| Stalled above min_percent | none | none |
| Wrong-direction steps | none | none |

Asymmetry (positive / negative `omega_max_rad_s`, fit values): 0.945. From the measured 100 %
steps: 6.347 / 6.836 = 0.929.

### Phase 7 parameter block (copied from the JSON)

> **Provisional, no dome installed; do not use for the velocity mapping until re-measured with
> the dome on.**

```json
"phase7": {
  "positive_velocity": {
    "percent_sign": -1,
    "k_percent_per_rad_s": 11.757603497921103,
    "p0_percent": 19.08645690290372,
    "min_percent": 25
  },
  "negative_velocity": {
    "percent_sign": 1,
    "k_percent_per_rad_s": 12.697981830705626,
    "p0_percent": 17.418172158171892,
    "min_percent": 27
  },
  "max_speed_rad_per_sec": 6.503539613053537
}
```

`positive_velocity` is the joint's positive direction (reported angle increasing, the sign of
`position_` and of the telemetry `velocity_rad_s`); on this platform it needs a **negative**
`:DPR` percent. The JSON's `max_speed_rad_per_sec` (6.504) is the lower of the two **fit**
values k (100 - p0). Plan phase 7 asks for the **measured** rate at 100 %; that is the lower of
the two measured 100 % steps, **6.35 rad/s** (positive percent / negative velocity). The
measured 100 % sits 2.4 % below the fit in that direction, a sign that the response flattens at
the top. Recommendation for this platform: `max_speed_rad_per_sec` 6.35 (provisional, no dome).
The current URDF uses the plugin default 3.14.

## Supplementary speed sweep at the operating settings (`20261014-214737-speed`, raw_motor false)

| # | Kind | Percent | Samples | Moving | Stalled (after s) | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Max gap s |
|---|------|--------:|--------:|:------:|------------------:|----------:|-----:|------------:|-----------:|----------:|----------:|
| 0 | step | +10 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 1 | step | -10 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 2 | step | +15 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 3 | step | -15 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 4 | step | +20 | 304 | no | yes (9.4) | 7.4 | 0.36 | -0.367 | -0.300 | 0.204 | 0.067 |
| 5 | step | -20 | 245 | no | yes (7.5) | 5.6 | 0.19 | 0.174 | 0.229 | 0.228 | 0.067 |
| 6 | step | +25 | 178 | no | yes (5.6) | 3.6 | 0.19 | -0.267 | -0.357 | 0.400 | 0.233 |
| 7 | step | -25 | 266 | yes | no | 6.4 | 0.35 | 0.247 | 0.337 | 0.254 | 0.067 |
| 8 | step | +30 | 290 | yes | no | 7.7 | 1.17 | -0.921 | -0.947 | 0.324 | 0.067 |
| 9 | step | -30 | 298 | yes | no | 7.8 | 1.36 | 1.062 | 1.082 | 0.331 | 0.067 |
| 10 | step | +40 | 195 | yes | no | 4.2 | 1.29 | -1.900 | -1.889 | 0.296 | 0.067 |
| 11 | step | -40 | 194 | yes | no | 4.1 | 1.25 | 1.853 | 1.837 | 0.353 | 0.067 |
| 12 | step | +50 | 189 | yes | no | 4.0 | 1.82 | -2.881 | -2.886 | 0.304 | 0.067 |
| 13 | step | -50 | 190 | yes | no | 4.0 | 1.86 | 2.912 | 2.907 | 0.193 | 0.033 |
| 14 | step | +60 | 188 | yes | no | 4.0 | 2.41 | -3.726 | -3.761 | 0.263 | 0.067 |
| 15 | step | -60 | 186 | yes | no | 4.0 | 2.45 | 3.883 | 3.850 | 0.187 | 0.067 |
| 16 | step | +70 | 186 | yes | no | 4.0 | 2.87 | -4.498 | -4.493 | 0.172 | 0.067 |
| 17 | step | -70 | 188 | yes | no | 4.0 | 2.76 | 4.354 | 4.365 | 0.217 | 0.033 |
| 18 | step | +80 | 185 | yes | no | 4.0 | 3.43 | -5.415 | -5.385 | 0.203 | 0.067 |
| 19 | step | -80 | 185 | yes | no | 4.0 | 3.38 | 5.299 | 5.293 | 0.220 | 0.067 |
| 20 | step | +90 | 182 | yes | no | 4.0 | 3.96 | -6.227 | -6.211 | 0.157 | 0.067 |
| 21 | step | -90 | 182 | yes | no | 4.0 | 3.99 | 6.260 | 6.269 | 0.115 | 0.067 |
| 22 | step | +100 | 181 | yes | no | 4.0 | 4.39 | -6.889 | -6.868 | 0.960 | 0.233 |
| 23 | step | -100 | 181 | yes | no | 4.0 | 4.34 | 6.810 | 6.785 | 0.123 | 0.033 |
| 24 | search | +27 | 565 | yes | no | 16.5 | 1.85 | -0.667 | -0.690 | 0.328 | 0.067 |
| 25 | search | -22 | 133 | no | yes (4.1) | 2.1 | 0.03 | 0.050 | 0.158 | 0.234 | 0.067 |
| 26 | search | -23 | 91 | no | yes (2.7) | 0.7 | 0.00 | 0.000 | 0.022 | 0.028 | 0.033 |

| | Positive percent | Negative percent |
|---|---:|---:|
| k (rad/s per %) | 0.08634 | 0.08625 |
| p0 (%) | 18.15 | 18.46 |
| R squared | 0.9978 | 0.9929 |
| omega at 100 %, measured | 6.889 rad/s | 6.810 rad/s |
| min_percent | 27 (25 stalled) | 25 (23 stalled) |
| Velocity at min_percent | 0.667 rad/s | 0.247 rad/s |

Asymmetry (fit): 1.005. Phase 7 block of this run: positive velocity percent_sign -1, k 11.59,
p0 18.46, min 25; negative velocity percent_sign +1, k 11.58, p0 18.15, min 27;
`max_speed_rad_per_sec` 7.03 (fit). See observation 1 for why this run was added.

## Sensor sweep (`20261014-214159-sensor`, raw motor)

Jump threshold 587.47 deg/s (from the speed sweep, above); reversals and stalls scored only at
or above 27 % (positive) / 25 % (negative), so the two 20 % steps are not scored ("Scored" no).
"Same-cycle pairs" are report pairs stamped less than 5 ms apart (two lines read in one
controller cycle), which are not scored for jumps.

| # | Percent | Samples | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Jumps | Reversals | Stalls | Scored | Same-cycle pairs | Max gap s | End |
|---|--------:|--------:|----------:|-----:|------------:|-----------:|----------:|------:|----------:|-------:|:------:|-----:|------:|-----|
| 0 | +20 | 252 | 5.7 | 0.33 | -0.424 | -0.346 | 0.254 | 0 | 0 | 0 | no | 30 | 0.200 | stalled (7.8 s) |
| 1 | -20 | 159 | 3.1 | 0.10 | 0.161 | 0.261 | 0.287 | 0 | 0 | 0 | no | 11 | 0.067 | stalled (5.1 s) |
| 2 | +35 | 985 | 30.0 | 7.06 | -1.490 | -1.483 | 0.338 | 0 | 0 | 0 | yes | 51 | 0.067 | completed |
| 3 | -35 | 971 | 30.0 | 5.79 | 1.213 | 1.219 | 0.372 | 0 | 0 | 0 | yes | 55 | 0.233 | completed |
| 4 | +50 | 1001 | 30.0 | 13.80 | -2.893 | -2.898 | 0.250 | 0 | 0 | 0 | yes | 45 | 0.067 | completed |
| 5 | -50 | 1008 | 30.0 | 13.33 | 2.811 | 2.788 | 0.360 | 0 | 0 | 0 | yes | 57 | 0.233 | completed |
| 6 | +75 | 978 | 30.0 | 23.30 | -4.895 | -4.876 | 0.178 | 0 | 0 | 0 | yes | 25 | 0.067 | completed |
| 7 | -75 | 986 | 30.0 | 22.69 | 4.756 | 4.748 | 0.314 | 0 | 0 | 0 | yes | 39 | 0.233 | completed |
| 8 | +100 | 964 | 30.0 | 30.95 | -6.492 | -6.479 | 0.233 | 0 | 0 | 0 | yes | 19 | 0.067 | completed |
| 9 | -100 | 961 | 30.0 | 32.41 | 6.786 | 6.788 | 0.490 | 0 | 0 | 0 | yes | 29 | 0.233 | completed |

Totals: 8265 samples, **0 jumps, 0 reversals, 0 stalls** (scored), `suggested_pass` true.
Both 20 % steps moved and then stalled (+20 % after 136 deg, -20 % after 111 deg), as in phase 5.

### Jump start histogram (10 degree bins)

All 36 bins are zero: no jump at any start angle, so there is no sensor-track misalignment to
correct and no mechanical sensor adjustment to request. The largest scored step relative to
the threshold was 79 % of it (-17 deg in a 33 ms stamped interval at +100 %; the host stamp is
quantised to the 33 ms read cycle, so a pair can carry more than one 28 ms report interval of
motion). The speed sweeps, scored with the default 720 deg/s, had 0 jumps as well (largest
69 % and 73 % of the threshold).

## Telemetry timing

Intervals between consecutive reports within a step (host stamps):

| Run | Pairs | Same-cycle (< 5 ms) | Median | p99 | > 50 ms | > 100 ms | > 200 ms | Max |
|-----|------:|--------------------:|-------:|----:|--------:|---------:|---------:|----:|
| speed (raw) | 4743 | 291 (6.1 %) | 33.3 ms | 66.7 ms | 116 | 3 | 3 | 233.3 ms |
| sensor | 8255 | 361 (4.4 %) | 33.3 ms | 66.7 ms | 149 | 5 | 5 | 233.4 ms |
| speed (operating) | 5231 | 336 (6.4 %) | 33.3 ms | 66.7 ms | 143 | 2 | 2 | 233.3 ms |

Every gap above 100 ms is a single ~233 ms gap, the plugin's periodic config refresh seen in
phase 5 (about one per 60 s of sweep). The 67 ms intervals are the partner of a same-cycle
pair. No gap came near the bench node's 1 s telemetry dropout limit.

## Firmware sensor error counter (`#DPSTATUS`, from `/diagnostics` and `run.json`)

| Point | Time | `sensor_errors` |
|-------|------|----------------:|
| Session baseline | 21:37:27 | 76 |
| Speed sweep (raw) start / end | 21:37:51 / 21:40:44 | 76 / 249 (+173) |
| Sensor sweep start / end | 21:41:59 / 21:46:40 | 262 / 663 (+401) |
| Speed sweep (operating) start / end | 21:47:37 / 21:50:43 | 663 / 830 (+167) |
| Session end, at rest | 21:53:41 | 832 |

Delta over the session: **+756**. `packet_serial_errors` stayed 0 and `position_timeouts` 0.
The count grows with rotation, not time: 173 errors over 60 revolutions (2.9 per rev), 401
over 159 revolutions (2.5 per rev), 167 over 65 revolutions (2.6 per rev), and about zero at
rest (663 to 663 over 30 s between runs, 830 to 832 over 3 min at the end; phase 5 saw roughly
one per 15 s at rest). The counter counts parse errors on the sensor ring's serial link only
(plan 2.1), and none of them turned into a position jump.

## Observations

1. **MaxSpeed made no measurable difference to `:DPR`.** Plan 2.3 says `:DPR` output is
   multiplied by `MaxSpeed/100`, so the operating setting (MaxSpeed 80) should give about 80 %
   of the raw-motor speed and move the minimum percentages up to about 34 / 32. To check, a
   third speed sweep was run with `raw_motor` false (operating settings, gentler, same human
   consent). Result: the same min_percent (27 / 25) and the same speeds within run-to-run
   scatter (ratio operating / raw 1.00 to 1.23 from 30 % up, median 1.06, so if anything slightly faster, perhaps from the platform warming up; 6.89 / 6.81 rad/s at 100 %). The
   plugin logged `#DPMAXSPEED100` with `Write Settings` and `Updated` at the start of the raw
   runs and `#DPMAXSPEED80` acked at the end, so the setting did change in the firmware. A
   reading of upstream `Reeltwo` `DomeDrive::domeStick` (`drive_mod =
   throttleSpeed(fSpeedModifier)`, which adds `throttle * (1 - modifier)` from the serial
   joystick emulation) suggests the serial controller may report full throttle and cancel the
   MaxSpeed modifier, but that is unverified; it needs a look at the flashed firmware. For
   phase 7 it means the raw-motor fit applies at the operating settings on this firmware.
2. **The low end is position-dependent.** Steps near the minimum often move a fraction of a
   turn and then stall: +25 % moved 123 deg then stalled in the raw speed sweep while +27 %
   ran for 1.2 turns; ±20 % stalled at once from 2 to 3 deg but moved 110 to 160 deg from
   other start angles. The stall end angles cluster: positive percent (angle decreasing)
   stopped at 47, 57 and 59 deg (and once at 241 deg); negative percent (angle increasing)
   stopped at 168, 183, 191 and 192 deg (and at 345 and 354 deg). These are tight spots in the
   platform's rotation (friction), not sensor faults. The measured min_percent (27 / 25) is
   therefore the percent that gets through the tight spots on these runs, with no margin, and
   `velocity_at_min_percent` varies a lot between runs (-25 %: 0.55 then 0.25 rad/s).
3. **Run-to-run scatter at low percent is large.** ±30 %: 0.87 / 0.86 rad/s (raw) vs 0.92 /
   1.06 (operating); ±35 % in the sensor sweep 1.49 / 1.21. From 50 % up the runs agree within
   about 5 to 10 %.
4. **Phase 7 acceptance at 0.5 rad/s will be marginal on this platform.** The fit gives 25.0 %
   for +0.5 rad/s (negative percent, min 25) and 23.8 % for -0.5 rad/s, clamped up to the
   positive-percent min of 27, which moves at about 0.63 rad/s. The slowest reliable speed is
   about 0.55 to 0.65 rad/s, so 0.5 rad/s is at or below the stall boundary. The acceptance
   point may need to move above the measured minimum velocity, or `min_percent` take a margin;
   decide after the dome re-run.
5. **Sign convention.** A positive percent turns the platform toward decreasing reported
   angle. The current linear `velocityToPercentage` passes the sign straight through, so a
   positive velocity command moves the joint negative (the dome teleop config uses a negative
   scale, `scale_linear.x: -1.5`). Phase 7's `percent_sign` per direction fixes this.
6. **Response time.** Time from a step's command to the windowed velocity (0.5 s regression)
   reaching 90 % of the step's measured speed: median about 0.63 s at every percent (0.47 to
   1.0 s, one 1.63 s outlier at -40 %), including about 0.25 to 0.5 s of estimator lag. Speed
   does not depend on ramp settings here (Scaling off; the Syren's own ramp applies).
7. **Startup.** The plugin's `#DPREPORT28` startup command timed out once and was retried
   (`Startup command '#DPREPORT28' timed out, retry 1/1`), and `/diagnostics` showed
   `position_reporting_enabled: false` although reports flowed normally all session. The
   reporting ack seems to be missed or arrive late; harmless here, worth a look.
8. Compared with the phase 5 short runs (same platform, no dome): phase 5's fit over 20 to 50 %
   gave k 0.105 / 0.087 and p0 24 / 21 and extrapolated 7 to 8 rad/s at 100 %; the full sweep
   measured 6.35 / 6.84 rad/s at 100 % with k 0.079 / 0.085 and p0 17 / 19. The low-percent
   points sit above a straight line through the high-percent points, so short low-range fits
   overestimate k. Minimum percent matches phase 5 (about 30 / 25 then, 27 / 25 now).

## Expected effect of mounting the dome, and how to compare

Adding the dome shell adds mass, inertia and probably bearing friction. Expect, on the re-run:

- a higher `min_percent` in both directions, and possibly new or larger tight spots
  (observation 2);
- a lower `omega_max_measured_rad_s` at 100 % and a smaller `k`, with `p0` moving up;
- a slower ramp: a longer time to reach 90 % of the step speed (observation 6) and, at the top,
  possibly steps where the measure window starts before the speed has settled (larger
  `std_velocity_rad_s`);
- possibly a different asymmetry ratio if the load is off centre;
- the sensor error rate per revolution (2.5 to 2.9 here) should stay about the same if it comes
  from the sensor link; a change would point at mechanics or wiring that moved with the dome.

How to compare: run the same two sweeps in the same order with the dome mounted (the commands
are in the README section "Re-tuning the velocity mapping when the dome changes"), write the
report as `docs/bench_results/<date>-dome.md`, and diff the per-direction numbers and the
`phase7` block against this file's JSON, for example:

```bash
for f in docs/bench_results/2026-10-14-no-dome-baseline-speed.json NEW_RUN_DIR/speed_sweep.json; do
  jq -c '.directions | map_values({min_percent, omega_max_measured_rad_s,
         k: .fit.k_rad_s_per_percent, p0: .fit.p0_percent, r2: .fit.r_squared,
         v_min: .velocity_at_min_percent_rad_s})' "$f"
done
jq -c '.totals' docs/bench_results/2026-10-14-no-dome-baseline-sensor.json NEW_RUN_DIR/sensor_sweep.json
```

Also compare the sensor error delta per revolution (read `sensor_diagnostics_before` /
`_after` in each `run.json`), the stall end angles (last position of each stalled step in
`samples.csv`) and the telemetry gap table. Keep `MaxSpeed`, `MinSpeed`, `SpeedTarget` and
`Scaling` at the values above for the re-run, or note any change, since they are part of the
calibration state.
