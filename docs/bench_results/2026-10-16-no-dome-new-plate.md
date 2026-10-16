# Dome bench results, 2026-10-16: new sensor pattern plate, dome not installed

**New sensor pattern plate, dome not installed; supersedes the 2026-10-14 no-dome baseline as the
no-dome reference.** The user replaced the plate that carries the sensor ring's pattern with a
stiffer one (see addendum 3 of the [ring debug report](2026-10-15-sensor-ring-raw-debug.md)). This
run repeats the three sweeps of the [2026-10-14 no-dome baseline](2026-10-14-no-dome-baseline.md)
with the same stack, commands and parameters, so the two can be compared directly. The motor drove
only the bare base platform. As before, these figures are a no-dome reference, **not** a
calibration for the assembled robot. The with-dome sweeps must be repeated on the new plate before
any value configures the velocity mapping. See "Re-tuning the velocity mapping when the dome
changes" in the [README](../../README.md).

**Short answer.** All three runs completed with 0 jumps, 0 reversals and no aborts. The firmware's
sensor error counter stayed at **0** for the whole session, over about 279 revolutions. On the old
plate it rose by 2.5 to 2.9 per revolution. The speed calibration matches the old baseline within
the earlier run-to-run scatter: 100 % gives 6.69 / 6.65 rad/s (old 6.35 / 6.84), k and p0 are
within 6 %, the raw-motor `min_percent` is unchanged at 27 / 25, and the asymmetry is now close to 1.
In the operating-settings run +30 % stalled once, so that run's `min_percent` for positive percent
is 32 (old 27). The angle-264 edge race from addendum 3 does not show up in the controller's
position stream.

Phase 6 of [docs/dome_diagnostics_plan.md](../dome_diagnostics_plan.md), run with the human at the
robot and the platform clear.

| | |
|---|---|
| Date | 2026-10-16, 12:26 to 12:40 local (-0400) |
| Stack | Same as 2026-10-14: `robot_state_publisher` with the dome-only URDF (`enable_bench_mode`, `publish_telemetry`), `ros2_control_node` with only the RoamADome hardware at 30 Hz, no controllers; `rmw_zenoh_cpp` with a router started for the session |
| Code | `ros2_roamadome` at `ca0c632` (`diag/phase-6-with-dome-reseated`). The bench node and analysis are unchanged since `fa97b3b`, the code used on 2026-10-14 (only docs changed since). `#DPREPORT28` |
| Bench node | `ros2 run ros2_roamadome roamadome_bench --ros-args -p output_dir:=/home/pi/dome_bench/phase6-newplate -p hardware_node:=roamadome`, plan 5.4 defaults; the third run adds `-p speed_sweep.raw_motor:=false`. Identical to 2026-10-14 apart from `output_dir` |
| Results | [2026-10-16-no-dome-new-plate-speed.json](2026-10-16-no-dome-new-plate-speed.json), [2026-10-16-no-dome-new-plate-sensor.json](2026-10-16-no-dome-new-plate-sensor.json), [2026-10-16-no-dome-new-plate-speed-operating.json](2026-10-16-no-dome-new-plate-speed-operating.json) (unmodified copies of the bench node's result files) |
| Raw data (not committed) | `/home/pi/dome_bench/phase6-newplate/`: `20261016-122703-speed/`, `20261016-123051-sensor/`, `20261016-123618-speed/` (each with `samples.csv`, `run.json`, `settings_before.json`, `settings_after.json`); `bench_speed.log`, `bench_sensor.log`, `bench_speed_operating.log`, `cm.log`, `baseline_params.yaml`, `/diagnostics` captures (`baseline_`, `pre_sensor_`, `post_sensor_`, `final_diagnostics.txt`), `check_<run>.txt`, `notes.txt`; `analysis/` holds the offline scripts and their output (`tables.py`, plus `spikes.py` and `speedcmp.py` copied from the 2026-10-15 analysis) |

`notes.txt` records that the dome was not installed and the plate was new. The result JSON files
have no free-text field for this.

## Runs

| Run | Sweep | Settings during the run | Status | Settings restored |
|-----|-------|-------------------------|--------|-------------------|
| `20261016-122703-speed` | Speed, plan defaults (12 percents, `raw_motor` true) | MaxSpeed 100, SpeedTarget 100, Scaling off | complete, 27 steps, 2 min 56 s | yes: `#DPMAXSPEED80` acked by the firmware, `settings_restored: true`, `firmware_acked: true` |
| `20261016-123051-sensor` | Sensor, plan defaults (20/35/50/75/100, 30 s, both directions, `raw_motor` true) | MaxSpeed 100, SpeedTarget 100, Scaling off | complete, 10 steps, 4 min 36 s | yes, as above |
| `20261016-123618-speed` | Speed, plan defaults but `raw_motor` false | Operating settings: MaxSpeed 80, SpeedTarget 100, Scaling off | complete, 28 steps, 2 min 50 s | nothing written; `settings_restored: true`, `firmware_acked: true` (0 writes) |

The sensor sweep took its jump threshold from today's speed run: `max_plausible_deg_s` 574.56 =
1.5 x 6.685 rad/s, source `1.5 x measured maximum in
/home/pi/dome_bench/phase6-newplate/20261016-122703-speed/speed_sweep.json`. Its
`min_percent_known` of 27 / 25 comes from the same file. No run aborted, there was no telemetry
dropout and no settings-ack timeout (`settings_acks` 4, `settings_ack_timeouts` 0 at the end).
`roamadome_bench_report <run_dir> --check` reproduces all three result files (exit 0, "result
matches"). After every run `ros2 param get` returned `device.max_speed` 80, `device.speed_target`
100 and `device.scaling` False. At the end `min_speed` was 15, `inverted` True, `home_pos` 0 and
`auto_safety` False; none of these was written.

At start-up the plugin's `#DPCONFIG` startup command timed out once and its retry succeeded
(`Startup command '#DPCONFIG' timed out, retry 1/1`). On earlier days the same thing happened to
`#DPREPORT28`. `/diagnostics` again showed `position_reporting_enabled: false`, although reports
flowed all session (25493 at the end).

## Firmware configuration (`device.*` at the start of the session)

From `ros2 param dump /roamadome`, unchanged at the end and **identical to 2026-10-14**:

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

## Speed sweep (`20261016-122703-speed`, raw motor)

Positive percent moves the reported angle negative (`angle_sign_for_positive_percent` -1, 24 of
24 votes). Columns as in the 2026-10-14 report. "n/a" is a stalled step with fewer than two measure
samples, where the JSON has `null`.

| # | Kind | Percent | Samples | Moving | Stalled (after s) | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Max gap s |
|---|------|--------:|--------:|:------:|------------------:|----------:|-----:|------------:|-----------:|----------:|----------:|
| 0 | step | +10 | 74 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 1 | step | -10 | 69 | no | yes (2.0) | 0.0 | 0.00 | n/a | n/a | n/a | 0.033 |
| 2 | step | +15 | 208 | no | yes (6.3) | 4.3 | 0.14 | -0.212 | -0.226 | 0.162 | 0.067 |
| 3 | step | -15 | 183 | no | yes (5.6) | 3.6 | 0.12 | 0.185 | 0.255 | 0.234 | 0.233 |
| 4 | step | +20 | 134 | no | yes (4.2) | 2.2 | 0.03 | -0.040 | -0.161 | 0.252 | 0.067 |
| 5 | step | -20 | 148 | no | yes (4.6) | 2.6 | 0.05 | 0.084 | 0.184 | 0.235 | 0.067 |
| 6 | step | +25 | 127 | no | yes (4.0) | 2.0 | 0.01 | -0.011 | -0.115 | 0.211 | 0.067 |
| 7 | step | -25 | 236 | yes | no | 5.7 | 0.57 | 0.675 | 0.626 | 0.173 | 0.067 |
| 8 | step | +30 | 226 | yes | no | 5.5 | 0.75 | -0.872 | -0.876 | 0.344 | 0.067 |
| 9 | step | -30 | 317 | yes | no | 8.5 | 1.43 | 0.998 | 1.046 | 0.313 | 0.067 |
| 10 | step | +40 | 188 | yes | no | 4.0 | 1.15 | -1.846 | -1.794 | 0.286 | 0.067 |
| 11 | step | -40 | 186 | yes | no | 4.0 | 1.12 | 1.697 | 1.807 | 0.317 | 0.067 |
| 12 | step | +50 | 194 | yes | no | 4.0 | 1.83 | -2.892 | -2.891 | 0.251 | 0.067 |
| 13 | step | -50 | 189 | yes | no | 4.0 | 1.79 | 2.812 | 2.798 | 0.274 | 0.033 |
| 14 | step | +60 | 187 | yes | no | 4.0 | 2.37 | -3.733 | -3.770 | 0.233 | 0.067 |
| 15 | step | -60 | 188 | yes | no | 4.0 | 2.28 | 3.678 | 3.668 | 0.281 | 0.067 |
| 16 | step | +70 | 185 | yes | no | 4.0 | 2.89 | -4.574 | -4.544 | 0.164 | 0.067 |
| 17 | step | -70 | 186 | yes | no | 4.0 | 2.83 | 4.475 | 4.465 | 0.254 | 0.067 |
| 18 | step | +80 | 186 | yes | no | 4.0 | 3.39 | -5.342 | -5.284 | 0.180 | 0.033 |
| 19 | step | -80 | 186 | yes | no | 4.0 | 3.32 | 5.248 | 5.219 | 0.197 | 0.067 |
| 20 | step | +90 | 184 | yes | no | 4.0 | 3.82 | -5.974 | -5.974 | 0.920 | 0.233 |
| 21 | step | -90 | 184 | yes | no | 4.0 | 3.88 | 6.054 | 6.094 | 0.216 | 0.067 |
| 22 | step | +100 | 183 | yes | no | 4.0 | 4.28 | -6.685 | -6.664 | 0.259 | 0.067 |
| 23 | step | -100 | 183 | yes | no | 4.0 | 4.21 | 6.651 | 6.661 | 0.297 | 0.067 |
| 24 | search | +27 | 278 | yes | no | 7.1 | 0.81 | -0.757 | -0.722 | 0.334 | 0.067 |
| 25 | search | -22 | 172 | no | yes (5.5) | 3.5 | 0.22 | 0.369 | 0.436 | 0.432 | 0.067 |
| 26 | search | -23 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.067 |

### Per-direction fit, `omega = k * (p - p0)` over the moving steps

| | Positive percent (angle decreasing) | Negative percent (angle increasing) |
|---|---:|---:|
| k (rad/s per %) | 0.08330 | 0.08249 |
| p0 (%) | 17.20 | 17.07 |
| R squared (9 points) | 0.9949 | 0.9968 |
| omega at 100 %, measured | 6.685 rad/s | 6.651 rad/s |
| omega at 100 %, from the fit, k (100 - p0) | 6.898 rad/s | 6.842 rad/s |
| min_percent (bracketed to 2 %) | 27 (25 stalled) | 25 (23 stalled) |
| Velocity at min_percent | 0.757 rad/s | 0.675 rad/s |
| Stalled above min_percent | none | none |
| Wrong-direction steps | none | none |

Asymmetry (positive / negative `omega_max_rad_s`, fit values): 1.008. From the measured 100 %
steps: 6.685 / 6.651 = 1.005.

### Phase 7 parameter block (copied from the JSON)

> **Provisional, no dome installed; do not use for the velocity mapping. The with-dome values
> must be re-measured on the new plate first.**

```json
"phase7": {
  "positive_velocity": {
    "percent_sign": -1,
    "k_percent_per_rad_s": 12.12217423012676,
    "p0_percent": 17.066027407065725,
    "min_percent": 25
  },
  "negative_velocity": {
    "percent_sign": 1,
    "k_percent_per_rad_s": 12.004090769980595,
    "p0_percent": 17.195274677534364,
    "min_percent": 27
  },
  "max_speed_rad_per_sec": 6.8415097010255606
}
```

`positive_velocity` is the joint's positive direction (reported angle increasing). On this
platform it needs a **negative** `:DPR` percent, as before. The JSON's `max_speed_rad_per_sec`
(6.84) is the lower of the two fit values. The lower **measured** 100 % rate is **6.65 rad/s**
(negative percent, positive joint velocity), and that is the figure plan phase 7 asks for. Today the
measured 100 % sits 3 % below the fit in both directions, so the response flattens at the top, as
it did on 2026-10-14. Following the README rule (use the operating-settings run's `min_percent`
where the two runs differ), the no-dome `negative_velocity.min_percent` is 32, not 27; see the
next section.

## Supplementary speed sweep at the operating settings (`20261016-123618-speed`, raw_motor false)

| # | Kind | Percent | Samples | Moving | Stalled (after s) | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Max gap s |
|---|------|--------:|--------:|:------:|------------------:|----------:|-----:|------------:|-----------:|----------:|----------:|
| 0 | step | +10 | 68 | no | yes (2.0) | 0.0 | 0.00 | n/a | n/a | n/a | 0.033 |
| 1 | step | -10 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |
| 2 | step | +15 | 147 | no | yes (4.4) | 2.4 | 0.03 | -0.067 | -0.082 | 0.069 | 0.033 |
| 3 | step | -15 | 96 | no | yes (2.8) | 0.8 | 0.00 | 0.000 | 0.001 | 0.004 | 0.033 |
| 4 | step | +20 | 186 | no | yes (5.8) | 3.8 | 0.19 | -0.298 | -0.350 | 0.299 | 0.067 |
| 5 | step | -20 | 142 | no | yes (4.4) | 2.4 | 0.05 | 0.074 | 0.204 | 0.293 | 0.067 |
| 6 | step | +25 | 153 | no | yes (4.7) | 2.7 | 0.04 | -0.083 | -0.154 | 0.187 | 0.067 |
| 7 | step | -25 | 227 | yes | no | 5.3 | 0.42 | 0.501 | 0.520 | 0.156 | 0.067 |
| 8 | step | +30 | 210 | no | yes (6.8) | 4.8 | 0.44 | -0.658 | -0.586 | 0.506 | 0.067 |
| 9 | step | -30 | 188 | yes | no | 4.3 | 0.66 | 1.008 | 0.945 | 0.206 | 0.067 |
| 10 | step | +40 | 185 | yes | no | 4.0 | 1.04 | -1.638 | -1.633 | 0.392 | 0.067 |
| 11 | step | -40 | 185 | yes | no | 4.0 | 1.06 | 1.719 | 1.684 | 0.253 | 0.233 |
| 12 | step | +50 | 189 | yes | no | 4.0 | 1.75 | -2.784 | -2.707 | 0.288 | 0.067 |
| 13 | step | -50 | 190 | yes | no | 4.0 | 1.75 | 2.788 | 2.781 | 0.174 | 0.067 |
| 14 | step | +60 | 188 | yes | no | 4.0 | 2.32 | -3.711 | -3.717 | 0.282 | 0.067 |
| 15 | step | -60 | 189 | yes | no | 4.0 | 2.38 | 3.791 | 3.774 | 0.241 | 0.067 |
| 16 | step | +70 | 187 | yes | no | 4.0 | 2.91 | -4.558 | -4.545 | 0.210 | 0.033 |
| 17 | step | -70 | 187 | yes | no | 4.0 | 2.94 | 4.635 | 4.636 | 0.183 | 0.067 |
| 18 | step | +80 | 184 | yes | no | 4.0 | 3.38 | -5.349 | -5.325 | 0.207 | 0.067 |
| 19 | step | -80 | 185 | yes | no | 4.0 | 3.43 | 5.364 | 5.288 | 0.663 | 0.233 |
| 20 | step | +90 | 183 | yes | no | 4.0 | 3.85 | -6.078 | -6.045 | 0.173 | 0.067 |
| 21 | step | -90 | 183 | yes | no | 4.0 | 3.95 | 6.240 | 6.240 | 0.232 | 0.067 |
| 22 | step | +100 | 183 | yes | no | 4.0 | 4.47 | -6.998 | -6.994 | 0.243 | 0.067 |
| 23 | step | -100 | 181 | yes | no | 4.0 | 4.31 | 6.845 | 6.846 | 0.280 | 0.067 |
| 24 | search | +35 | 227 | yes | no | 5.4 | 1.25 | -1.411 | -1.449 | 0.247 | 0.067 |
| 25 | search | +32 | 291 | yes | no | 7.6 | 1.33 | -1.092 | -1.099 | 0.339 | 0.067 |
| 26 | search | -22 | 68 | no | yes (2.0) | 0.0 | 0.00 | n/a | n/a | n/a | 0.067 |
| 27 | search | -23 | 69 | no | yes (2.0) | 0.0 | 0.00 | 0.000 | 0.000 | 0.000 | 0.033 |

| | Positive percent | Negative percent |
|---|---:|---:|
| k (rad/s per %) | 0.08711 | 0.08650 |
| p0 (%) | 19.01 | 18.31 |
| R squared (9 points) | 0.9975 | 0.9966 |
| omega at 100 %, measured | 6.998 rad/s | 6.845 rad/s |
| min_percent | **32** (30 stalled) | 25 (23 stalled) |
| Velocity at min_percent | 1.092 rad/s | 0.501 rad/s |

Asymmetry (fit): 0.998. Phase 7 block of this run: positive velocity percent_sign -1, k 11.56, p0
18.31, min 25; negative velocity percent_sign +1, k 11.48, p0 19.01, min 32;
`max_speed_rad_per_sec` 7.06 (fit).

Raw against operating: the speeds agree. The ratio operating / raw per step from 40 % up has a median
of 1.02 (0.96 to 1.05 from 50 % up; 0.89 at +40 %). As on 2026-10-14, MaxSpeed 80 does not scale
`:DPR` output measurably. The one difference is at the low end. At +30 % the platform moved 267 deg
from 137 deg and stalled at 230 deg, so the search went 35 % then 32 % (both moved) and placed
the boundary at 32 %. In the raw run +30 % and +27 % moved. The 2026-10-14 operating run moved
at +27 %. The 2026-10-14 with-dome operating run also needed 32 %. This is the position-dependent low end
(see the stall angles below), not a property of the operating settings that this run can separate.

## Sensor sweep (`20261016-123051-sensor`, raw motor)

Jump threshold 574.56 deg/s (from today's speed sweep). Reversals and stalls are scored only at or
above 27 % (positive) / 25 % (negative), so the two 20 % steps are not scored.

| # | Percent | Samples | Measure s | Revs | Slope rad/s | Mean rad/s | Std rad/s | Jumps | Reversals | Stalls | Scored | Same-cycle pairs | Max gap s | End |
|---|--------:|--------:|----------:|-----:|------------:|-----------:|----------:|------:|----------:|-------:|:------:|-----:|------:|-----|
| 0 | +20 | 151 | 2.7 | 0.08 | -0.138 | -0.247 | 0.292 | 0 | 0 | 0 | no | 15 | 0.067 | stalled (4.7 s) |
| 1 | -20 | 132 | 2.1 | 0.03 | 0.043 | 0.170 | 0.254 | 0 | 0 | 0 | no | 9 | 0.067 | stalled (4.1 s) |
| 2 | +35 | 953 | 30.0 | 6.41 | -1.343 | -1.350 | 0.297 | 0 | 0 | 0 | yes | 44 | 0.233 | completed |
| 3 | -35 | 962 | 30.0 | 6.16 | 1.305 | 1.299 | 0.314 | 0 | 0 | 0 | yes | 44 | 0.067 | completed |
| 4 | +50 | 1006 | 30.0 | 12.95 | -2.711 | -2.708 | 0.304 | 0 | 0 | 0 | yes | 53 | 0.233 | completed |
| 5 | -50 | 1007 | 30.0 | 13.23 | 2.774 | 2.763 | 0.233 | 0 | 0 | 0 | yes | 50 | 0.067 | completed |
| 6 | +75 | 986 | 30.0 | 22.87 | -4.780 | -4.787 | 0.354 | 0 | 0 | 0 | yes | 43 | 0.233 | completed |
| 7 | -75 | 991 | 30.0 | 22.54 | 4.725 | 4.727 | 0.369 | 0 | 0 | 0 | yes | 39 | 0.233 | completed |
| 8 | +100 | 973 | 30.0 | 30.95 | -6.460 | -6.464 | 0.303 | 0 | 0 | 0 | yes | 28 | 0.067 | completed |
| 9 | -100 | 973 | 30.0 | 28.37 | 5.945 | 5.951 | 0.464 | 0 | 0 | 0 | yes | 25 | 0.233 | completed |

Totals: 8134 samples, **0 jumps, 0 reversals, 0 stalls** (scored), `suggested_pass` true. The
jump start histogram is zero in all 36 bins. The +20 % step moved 96 deg and stalled at 252 deg;
the -20 % step moved 88 deg and stalled at 340 deg.

The largest scored step relative to the threshold was 95 % of it, at -100 %: +20 deg in a 33 ms
stamped interval at 355 deg. The sequence around it (322, 336, 349, 355, 15, 28, 38 deg) is steady
motion of about 13 deg per report. A short 6 deg interval is followed by a long 20 deg one, which is
report timing against the 33 ms read cycle, not a sensor event. On 2026-10-14 the largest was 79 %.
The margin is smaller because the threshold is 1.5 x a 4 s measured speed (6.685 rad/s), while
during the 30 s sensor steps the speed varies by up to about 1 rad/s around the mean. At -100 % a
1.75-interval stamp gap now comes within 5 % of a false jump. If this recurs, a larger factor or a
threshold taken from the highest speed seen in the sensor sweep itself would remove it.

### The angle-264 edge race (ring debug addendum 3)

On the new plate the raw ring data showed one benign event: going in the angle-decreasing direction
through dome angle 264, the ring's raw code reads 154 for one ring sample. It does **not** show up
in the controller's position stream. In the sensor sweep the reported angle crossed 264 deg 77
times decreasing and 81 times increasing. The largest residual of any of those pairs against the
local motion (median of the neighbouring ±7 pairs) was 4 deg, normal 1-degree quantisation and
report-timing noise. No jump was scored, and the offline excursion scan (`spikes.py`, any report
10 deg or more off the local motion) found **no excursion anywhere** in any of the three runs.
The ring's median-of-5 filter removes the one-sample race, as addendum 3 expected. The two speed
runs agree: 35 + 38 decreasing crossings of 264, residual at most 3 deg.

## Telemetry timing

Intervals between consecutive reports within a step (host stamps):

| Run | Pairs | Same-cycle (< 5 ms) | Median | p99 | > 50 ms | > 100 ms | > 200 ms | Max |
|-----|------:|--------------------:|-------:|----:|--------:|---------:|---------:|----:|
| speed (raw) | 4823 | 287 (6.0 %) | 33.3 ms | 66.7 ms | 102 | 2 | 2 | 233.4 ms |
| sensor | 8124 | 350 (4.3 %) | 33.3 ms | 66.7 ms | 145 | 5 | 5 | 233.4 ms |
| speed (operating) | 4712 | 283 (6.0 %) | 33.3 ms | 66.7 ms | 115 | 2 | 2 | 233.3 ms |

As before, every gap above 100 ms is a single ~233 ms gap from the plugin's periodic config refresh.
No gap came near the 1 s dropout limit, and no CPU-heavy work ran on the Pi during the sweeps
(analysis was done afterwards).

## Firmware sensor error counter (`#DPSTATUS`, from `/diagnostics` and `run.json`)

| Point | Time | `sensor_errors` |
|-------|------|----------------:|
| Session start, at rest | 12:26:40 | 0 |
| Speed sweep (raw) start / end | 12:27:03 / 12:29:59 | 0 / 0 |
| Sensor sweep start / end | 12:30:51 / 12:35:27 | 0 / 0 |
| Speed sweep (operating) start / end | 12:36:18 / 12:39:08 | 0 / 0 |
| Session end, at rest | 12:39:55 | 0 |

The counter read **0 throughout**, over about 279 revolutions (62.9 + 152.5 + 63.8, summed from the
reported position changes). The plugin polled `#DPSTATUS` every 10 s and 78 polls were answered, so
the value is current, not stale. `packet_serial_errors` and `position_timeouts` stayed 0. On the
old plate the counter grew with rotation at 2.5 to 2.9 per revolution without the dome on
2026-10-14 (+756 in the session), and at 2.2 to 2.5 per revolution after a reboot on 2026-10-15
with the dome. The robot was powered up this morning, so today's counter also started from 0.

This counter counts parse errors on the sensor ring's serial link (plan 2.1), not wrong angles, so
there is no obvious mechanism linking it to the pattern plate. The plate swap meant taking the ring
side apart, which may also have re-seated the ring's connector or cable. This run cannot separate
the two. The observation stands: no sensor link errors at all on the new plate.

## Comparison with the 2026-10-14 no-dome baseline (old plate)

"+ %" is positive percent (reported angle decreasing), "- %" negative percent. Raw = `raw_motor`
true, op = operating settings. Time to 90 % was recomputed for both days with the same script
(`speedcmp.py`: median over the moving steps at 40 % and above of the time from the command until
the 0.5 s windowed velocity reaches 90 % of the step's measured speed). The 2026-10-14 report gave
"about 0.63 s" from a different count.

| Quantity | 2026-10-14, old plate | 2026-10-16, new plate | Change |
|----------|----------------------:|----------------------:|--------|
| omega at 100 %, measured, raw (+ % / - %) | 6.347 / 6.836 rad/s | 6.685 / 6.651 rad/s | +5.3 % / -2.7 % |
| omega at 100 %, measured, op (+ % / - %) | 6.889 / 6.810 rad/s | 6.998 / 6.845 rad/s | +1.6 % / +0.5 % |
| k raw (+ % / - %) | 0.07875 / 0.08505 rad/s per % | 0.08330 / 0.08249 | +5.8 % / -3.0 % |
| p0 raw (+ % / - %) | 17.42 / 19.09 % | 17.20 / 17.07 % | -0.2 / -2.0 points |
| R squared raw (+ % / - %) | 0.9942 / 0.9969 | 0.9949 / 0.9968 | same |
| k op (+ % / - %) | 0.08634 / 0.08625 | 0.08711 / 0.08650 | +0.9 % / +0.3 % |
| p0 op (+ % / - %) | 18.15 / 18.46 % | 19.01 / 18.31 % | +0.9 / -0.2 points |
| min_percent raw (+ % / - %) | 27 / 25 | 27 / 25 | same |
| min_percent op (+ % / - %) | 27 / 25 | **32** / 25 | +30 % stalled once (see above) |
| Velocity at min_percent, raw (+ % / - %) | 0.631 / 0.553 rad/s | 0.757 / 0.675 rad/s | scatter |
| Velocity at min_percent, op (+ % / - %) | 0.667 / 0.247 rad/s | 1.092 (at 32 %) / 0.501 rad/s | scatter |
| Asymmetry, fit, raw (op) | 0.945 (1.005) | 1.008 (0.998) | closer to 1 |
| Asymmetry, measured 100 %, raw | 0.929 | 1.005 | closer to 1 |
| Recommended `max_speed_rad_per_sec` (lower measured 100 %, raw) | 6.35 | 6.65 | +4.7 % |
| Time to 90 % of step speed, median (range), raw | 0.60 s (0.53 to 1.60) | 0.60 s (0.57 to 1.00) | same |
| Time to 90 %, median (range), op | 0.60 s (0.57 to 0.97) | 0.65 s (0.57 to 1.10) | same within one 33 ms cycle |
| Sensor errors per revolution (session total) | 2.5 to 2.9 (+756) | **0** (0) | gone |
| Jumps: speed raw / sensor / speed op | 0 / 0 / 0 | 0 / 0 / 0 | same |
| Reversals (sensor sweep) | 0 | 0 | same |
| Excursions >= 10 deg off local motion (`spikes.py`), all runs | 0 (one unreturned end-of-step artefact in the raw speed run, not a jump) | 0 | same |
| Largest step / jump threshold (sensor sweep) | 79 % | 95 % | see sensor sweep section |
| Sensor sweep revolutions | 158.7 | 152.5 | |
| Stall end angles, + % (angle decreasing) | 47, 57, 59, 241 | 230, 234, 245, 249, 249, 252, 253 | different spots reached |
| Stall end angles, - % (angle increasing) | 168, 183, 191, 345 (192, 354 after moves under 20 deg) | 339, 340, 344, 345, 347 (348, 201 after moves under 20 deg) | different spots reached |

Stall angles are the last reported position of each stalled step that moved more than 20 deg
before stopping (steps that never left their start angle are not listed). On both days the
low-percent steps that never moved started on a tight spot: 2 to 4 deg and 217 deg on
2026-10-14, 337 deg and 19 deg today. Where a low step stops depends on where it starts, because it
stalls at the first tight spot it reaches. Today's low steps happened to start at 337 to 348 deg and
245 to 253 deg and shuttle between them, so the two clusters partly reflect the start angles. The
stalls are mechanical: no jump, no excursion, and after every stalled step the next step started at
the angle where it had stopped. 241 deg (decreasing) and 345 to 354 deg (increasing) were already
secondary stall spots on 2026-10-14. The operating -22 % / -23 % searches started at 195 and 201 deg and
moved 6 and 0 deg, close to the old 183 to 192 deg spot. The old 47 to 59 deg spot was passed
once at +30 % (operating run, from 137 deg down to 230 deg) without stopping, and no low step
started above it in the decreasing direction otherwise. So the evidence does not show that the
plate change moved or removed any tight spot. It shows that the same kind of position-dependent
friction is still there.

### What the new plate changed

- **Sensor side: clean.** The firmware sensor error counter no longer grows with rotation (0 against
  about 2.7 per revolution). Jumps and reversals stay at zero, as they already were without the
  dome on the old plate. The real test of the plate is with the dome on, where the old plate failed
  (6 and 102 jumps on 2026-10-14 and 2026-10-15). The raw ring run (addendum 3) and this sweep
  remove the no-dome part of the doubt only.
- **Drive side: unchanged within the scatter.** The 100 % rates, k and p0 moved by 0.5 to 6 %,
  within the run-to-run scatter seen between the two 2026-10-14 runs, which differed by up to 8.5 %
  at +100 %. The raw-motor `min_percent` is the same. The asymmetry is now about 1.0 in both runs.
  On 2026-10-14 the raw run's 0.93 to 0.95 came from a slow +100 % step (6.35 rad/s), which the
  operating run that day did not repeat (6.89), so it was probably scatter too.
- **Low end: still position dependent.** The operating-settings `min_percent` for positive percent
  came out at 32 because +30 % hit a tight spot. With the dome (old plate, 2026-10-14) the
  operating run also gave 32.
  The README's rule (use the operating run's `min_percent`) gives 32 / 25 here.

## Observations

1. **Sustained top speed is lower than the 4 s speed step.** In the sensor sweep the 30 s -100 %
   step averaged 5.95 rad/s against 6.65 rad/s in the speed sweep (+100 %: 6.46 against 6.69).
   On 2026-10-14 the two agreed within 1 to 2 %. -100 % was the last step of the sensor sweep,
   after about 4 minutes of continuous running, and its standard deviation was the highest of the
   sweep (0.46 rad/s). A warm motor or a sagging supply would explain it. This run cannot tell
   which; it matters for `max_speed_rad_per_sec`, which is taken from the 4 s speed step.
2. **Phase 7 acceptance at 0.5 rad/s without the dome.** The fit gives 23.1 % for +0.5 rad/s
   (negative percent, clamped to min 25, which moved at 0.50 to 0.68 rad/s) and 23.2 % for -0.5
   rad/s (positive percent, clamped to 27 raw / 32 operating, 0.76 / 1.09 rad/s). As on
   2026-10-14, 0.5 rad/s is at or below the stall boundary on the bare platform. The with-dome
   figure (about 1 rad/s slowest dependable) still has to be re-measured on the new plate.
3. **Sign convention unchanged.** Positive percent moves the reported angle down
   (`angle_sign_for_positive_percent` -1 in all three runs, 24 / 10 / 23 of 24 / 10 / 23 votes).
4. **Startup.** The `#DPCONFIG` startup command (the first of three) timed out once and was
   retried successfully. In the three earlier bench sessions it was `#DPREPORT28` (the last of the
   three) each time. One startup timeout per session, retry succeeds, the command differs: still a
   start-up timing nit for later work. `position_reporting_enabled` reads false again while reports
   flow.
5. **Shutdown.** `ros2_control_node` stopped cleanly on SIGINT (hardware deactivated, `:DPR0`
   path), with the same `pal_statistics` "context cannot be slept with" errors at exit as on
   2026-10-14.

## Next

Mount the dome and repeat the three sweeps on the new plate (same commands, a new `output_dir`).
That run decides whether the with-dome spikes are gone. Its speed numbers replace the old-plate
with-dome values for phase 7. Compare against this file's JSON with the `jq` commands in the
[2026-10-14 report](2026-10-14-no-dome-baseline.md#expected-effect-of-mounting-the-dome-and-how-to-compare).
