# Dome Diagnostics and Telemetry Implementation Plan

Status: proposed, 2026-10-13. Not yet started.

This document is written for an agent implementing the work directly on the
robot, with a human available for the hardware-in-the-loop phases. It records
the facts established from reading the firmware and this package, the design
decisions already made, the interfaces to build, and acceptance criteria per
phase. Read all of it before starting any phase.

## 0. How to use this document

Work one phase at a time, in order. Each phase ends with a pull request to
`main` of the repository it touches. Do not start a phase that depends on an
unmerged one unless you branch from that phase's branch.

Suggested model and effort per phase. These are the minimum expected to be
sufficient; go up a level if a phase stalls, not down.

| Phase | Work | Model | Effort | Why |
|-------|------|-------|--------|-----|
| 1 | Velocity estimate and telemetry publisher | Sonnet 5.5 | medium | Follows existing patterns, pure-function math, gtest |
| 2 | Classify dropped firmware lines, diagnostics | Sonnet 5.5 | medium | String matching plus a publisher |
| 3 | Writable speed settings, `Unchanged` ack, set-home service | Sonnet 5.5 | low to medium | Table entries in the parameter registry |
| 4 | Bench gate, raw drive service, launch and xacro plumbing | Sonnet 5.5 | medium | Touches two repositories, needs care with gating |
| 5 | Bench node: sensor sweep, speed sweep, analysis, CSV | Opus 5.5 | high | Concurrency, safety watchdogs, numerical analysis |
| 6 | Hardware validation on the robot | Opus 5.5 | high | Interprets unexpected firmware behaviour live; human present |
| 7 | Replace linear velocity mapping with measured model | Sonnet 5.5 | medium | Small change with a clear spec from phase 6 output |
| any | Code review before merge | Opus 5.5 | high | Use `/code-review high` or an independent review agent |

Use Fable 5.1 only if a phase is blocked on firmware behaviour that the facts
in section 2 do not explain, for example an unexpected serial response.
Haiku 5.5 is not recommended for any phase that edits C++ in this package.

## 1. Goals

1. Evaluate sensor reliability. A bench diagnostic that spins the dome
   continuously at several speeds in both directions, records every position
   report with a host timestamp, and scores jumps, reversals and stalls.
   Hidden behind a launch option. Not run during normal operation.
2. Measure minimum and maximum dome speed. Using the same bench path, sweep
   the motor percentage, measure steady-state angular velocity from the
   position stream, find the stall threshold, and fit a model that the
   hardware interface then uses to map rad/s to a motor percentage.
3. Along the way, expose firmware feedback the interface currently drops, and
   compute the joint velocity state that is exported but never filled.

Out of scope: changes to the firmware. Section 9 lists optional firmware
improvements; they are not prerequisites.

## 2. Facts established from source review

Sources reviewed on 2026-10-13: this package at commit `5d1fd14`,
`reeltwo/DomeControlFirmware` at `f9b1b83` (the merged position-reporting PR),
and `reeltwo/Reeltwo` at `a4e7292`, specifically `src/drive/DomeDrive.h`,
`DomePosition.h`, `DomeSensorRingSerialListener.h`, `DomeSensorRing.h`,
`DomeDriveSabertooth.h`. To consult them, clone into a scratch directory
outside the workspace; do not add them to `~/r2_ws/src`.

### 2.1 Position sensing

- The sensor ring is a separate board. It sends `#DP@<degrees>` lines to the
  controller over a dedicated serial port. The controller's
  `DomeSensorRingSerialListener` applies a median-of-5 filter before any other
  code sees the angle. The first six samples pass unfiltered.
- The 9-bit decode table in `DomeSensorRing::getDomeAngle` is not a Gray code
  with respect to single-bit faults. Codes 1, 2 and 4 decode to 0, 40 and 80
  degrees. A bit that transitions early or late produces a large jump to a
  specific wrong angle. Because of the median filter, a glitch lasting one or
  two samples is suppressed and one lasting three or more passes through.
  Expect jumps to cluster at particular start angles if a track is misaligned.
- `DomePosition::getDomePosition` accumulates relative degrees via shortest
  distance between successive readings. A jump out and back nets to zero, but
  disturbs any relative move in progress.
- The firmware's sensor error counter, reported by `#DPSTATUS`, counts only
  parse errors on the sensor serial link. It does not count implausible angles.

### 2.2 Position reporting to the host

- `#DPREPORT<ms>` enables `DOME POSITION: <n>` on the console at that interval.
  `n` is the home-relative integer angle 0..359. No timestamp, mode or motor
  value is included. `#DPREPORT0` disables it.
- The firmware main loop yields one FreeRTOS tick per iteration, so report
  jitter on the ESP32-S3 is on the order of 1 ms. Host receive time over native
  USB CDC is adequate as the sample timestamp.
- The hardware interface requests an interval of `1000/rw_rate - 5` ms. With
  the controller manager at 30 Hz that is 28 ms.

### 2.3 How a rotate command becomes motor output

For `:DPR<p>` with `p` in -100..100, in order:

1. If `|p| < MinSpeed` the firmware sets `p = 0`.
2. `p` is clamped to `±SpeedTarget`.
3. The firmware prints `ROTATE SPEED: <p>` with the clamped value.
4. In `DomeDrive::domeStick` the value is multiplied by `MaxSpeed/100`.
5. If `Scaling` is on, the acceleration and deceleration scale ramps apply,
   and a result below `MinSpeed/100` is forced to zero.
6. The result times 127 goes to the Sabertooth or Syren, which applies its own
   ramping (`setRamping(80)` in `DomeDriveSabertooth::setup`).

With factory defaults (MinSpeed 15, SpeedTarget 100, MaxSpeed 50) a `:DPR100`
is 50% motor output and anything under 15% stops the dome. `MaxSpeed` and
`SpeedTarget` on this robot are whatever `#DPCONFIG` reports; read them, do
not assume.

Continuous rotation runs in mode `kOff`, so the firmware's movement watchdog
does not apply. A stalled dome under `:DPR` stays commanded until told
otherwise. The bench code must implement its own stall detection and stop.

Changing `MaxSpeed` (`#DPMAXSPEED`) calls `DomeDrive::setMaxSpeed`, which
calls `stop()` and zeroes the autonomous drive value. Never change speed
settings while a rotation is in progress; set them first, then rotate.

### 2.4 Settings write acknowledgements

Every `#DP...` setting command that changes a value prints, in order,
`Restore Dome Settings`, `Write Settings`, `Updated`, and persists to
EEPROM or Preferences. If the value is unchanged it prints `Unchanged` and
nothing else. The current ack-tracked flow in
`RoamadomeControl::handleSendQueueFeedback` waits for `Write Settings` then
`Updated` and does not recognise `Unchanged`; an unchanged write therefore
times out after 3 s with a warning. Phase 3 fixes this.

### 2.5 Firmware lines the interface currently drops

All of these arrive via `ISerialObserver::onUnhandledLine` and are logged at
debug level only:

| Line | Meaning |
|------|---------|
| `ROTATE SPEED: <p>` | Accepted and clamped rotate percentage |
| `ROTARY DEGREE: <d> speed: <f>` | Accepted absolute or relative move |
| `REACHED TARGET: <n>` | Position move finished at home-relative `n` |
| `REACHED HOME: <n>` | Home move finished |
| `ERROR: DOME POSITION TIMEOUT` | Firmware aborted a move; no position change within `Timeout` seconds |
| `Dome Sensor Errors: <n>` / `No Dome Sensor Errors` | In `#DPSTATUS` output |
| `Packet Serial Errors: <n>` | In `#DPSTATUS` output |
| `Position Reporting Enabled` / `Disabled` | Ack for `#DPREPORT` |
| `SPEED: <n>`, `Angular velocity: <f> cm/s`, `GOOD MAX SPEED: <n>`, `AUTO SETUP FAILED` | Emitted during `#DPSETUP` |
| `Out of range.`, `Invalid`, `Unchanged` | Command rejections and no-ops |
| `READY` | Firmware finished boot |

### 2.6 Current state of this package

- Joint velocity state is exported but never written. `onVelocityUpdate` in
  `include/ros2_roamadome/roamadome_control.hpp` is a no-op.
- `velocityToPercentage` in `src/roamadome_control.cpp` maps rad/s linearly to
  a percentage using `max_speed_rad_per_sec`, ignoring everything in 2.3.
- `write()` sends `:DPR` only when the commanded velocity changes by more than
  0.01 rad/s, and `:DPA<deg>` only when the commanded position changes.
- A `~/setup` service already exists on the hardware node, created in
  `on_activate`. Follow that pattern for new services.
- `device.*` parameters are declared from `src/device_parameter_specs.cpp`.
  Writable specs carry a command prefix; read-only specs do not. The speed and
  ramp settings are currently read-only.
- The serial port is opened with exclusive access. Nothing else on the host
  can talk to the controller while the hardware interface is configured, so
  all bench traffic must go through the plugin.
- Unit tests in `test/test_roamadome_control.cpp` drive the plugin through a
  pseudo-terminal fixture that scripts firmware responses. New serial
  behaviour gets tests in that style.

### 2.7 Workspace facts

- ROS 2 Jazzy, `rmw_zenoh_cpp`. Python 3 with numpy 1.26 is available on the
  development host; treat numpy as optional on the robot and keep the analysis
  code working without it.
- The workspace is composed from separate repositories listed in
  `src/r2_bringup/r2d2.repos`. This plan touches `ros2_roamadome`,
  `r2_bringup` (launch, xacro) and `astromech_msgs` (new interfaces). Each
  gets its own branch and PR.
- Controller manager runs at 30 Hz. The dome uses
  `forward_command_controller` on the velocity interface
  (`dome_controller`) and a `dome_state_broadcaster` at 10 Hz.
- The dome `ros2_control` block is in `src/r2_bringup/urdf/ros2_control.xacro`.
  Xacro arguments are declared in `src/r2_bringup/urdf/robot.urdf.xacro`.

## 3. Working rules for the implementing agent

Environment and build:

- Always `source ~/r2_ws/setup.bash` first. Build from `~/r2_ws` with
  `colcon build --symlink-install --packages-select <pkg>`. Never build the
  whole workspace for a change to one package.
- Run `colcon test --packages-select ros2_roamadome --event-handlers console_direct+`
  and `colcon test-result --verbose` before opening a PR.
- Use `python3 -B` for any ad-hoc import of workspace Python sources. Check
  `git status --ignored` in each touched repository before committing and
  remove any `__pycache__` you created under `src/`.
- Before creating a file, check it does not already exist with `ls` or
  `git ls-files`. Merge into existing files; never clobber.

Git:

- One branch per phase, named `diag/phase-<n>-<slug>`. One PR per repository
  per phase. After merge and push, delete the branch on the remote and
  locally.
- Keep `main` buildable at every merge. A phase that adds an interface to
  `astromech_msgs` must merge there first.

Hardware safety:

- The hardware interface sends `:DPR0` on deactivate. Any new code path that
  can leave the dome rotating must also stop it on error, cancel, timeout and
  process exit. Test this deliberately before any sweep.
- Never run a sweep without a human present and the dome clear of cables,
  tools and hands. Confirm with the human before the first rotation of each
  session.
- Do not change `Inverted`, `HomePos` or AutoSafety from bench code.
- Every setting the bench changes must be read back from `#DPCONFIG` first
  and restored at the end, including on abort. Settings writes persist to
  flash, so write once per sweep, not once per step.
- Start every sweep at the lowest speed in the list, not the highest, so a
  wiring or sign problem shows up slowly.

## 4. Architecture

Two parts.

Hardware interface (`ros2_roamadome`, C++ plugin, always loaded):

- Computes joint velocity from the position stream.
- Publishes stamped raw telemetry on `~/telemetry` when enabled.
- Recognises the firmware lines in 2.5 and publishes diagnostics.
- Makes speed and ramp settings writable through `device.*` parameters.
- In bench mode only, offers `~/bench/drive`, a raw rotate service with a
  heartbeat watchdog, and refuses it while any controller holds a command
  interface.

Bench node (`ros2_roamadome`, Python executable `roamadome_bench`, launched
only with `dome_bench:=true`):

- Subscribes to `~/telemetry`, drives through `~/bench/drive`, changes
  settings through `device.*` parameters.
- Runs the sensor sweep and the speed sweep as services, publishes progress,
  writes CSV and JSON results to a configurable directory.
- Contains the analysis as pure functions importable without ROS so they can
  be unit tested and re-run offline on a saved CSV.

Why not a separate node talking to the serial port: the plugin holds the port
exclusively. Why not implement the sweeps inside the plugin: the analysis is
easier to iterate in Python and must not run in the controller manager's
real-time loop. Why services and a status topic rather than actions: fewer
moving parts for a bench tool; actions remain an optional upgrade.

## 5. Interface specification

### 5.1 New hardware parameters (URDF `<param>`)

| Name | Type | Default | Meaning |
|------|------|---------|---------|
| `publish_telemetry` | bool | false | Publish `~/telemetry` |
| `velocity_window_s` | double | 0.5 | Regression window for velocity estimate |
| `enable_bench_mode` | bool | false | Create `~/bench/drive`; set by the `dome_bench` xacro arg |
| `bench_heartbeat_timeout_ms` | uint32 | 1500 | Send `:DPR0` if no bench drive call arrives within this time while a bench drive is active |
| `status_poll_interval_ms` | uint32 | 10000 | Interval for `#DPSTATUS` polling used by diagnostics; 0 disables |

### 5.2 Messages and services (`astromech_msgs`)

`msg/DomeTelemetry.msg`

```
std_msgs/Header header        # stamp is host receive time of the report line
uint16 position_deg           # home-relative integer from firmware, 0..359
float64 position_rad          # wrapped to [0, 2*pi)
float64 unwrapped_rad         # continuous since activation
float64 velocity_rad_s        # NaN until the regression window is populated
int8 commanded_percent        # last :DPR value sent in any mode; 0 when none
uint8 command_mode            # 0 idle, 1 position, 2 velocity, 3 bench
uint32 sequence               # increments per report line; wraps
```

`srv/DomeBenchDrive.srv`

```
int8 percent                  # -100..100, 0 stops
---
bool success
string message
```

Everything else uses `std_srvs/Trigger`, `std_msgs/String` and
`diagnostic_msgs`.

### 5.3 Hardware node topics and services

| Name | Type | Notes |
|------|------|-------|
| `~/telemetry` | `astromech_msgs/DomeTelemetry` | One message per `DOME POSITION:` line |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Status `roamadome: link`, `roamadome: sensor`, `roamadome: motion` |
| `~/set_home_here` | `std_srvs/Trigger` | Sends `#DPHOMEPOS`, ack-tracked |
| `~/bench/drive` | `astromech_msgs/DomeBenchDrive` | Bench mode only |

### 5.4 Bench node parameters, topics and services

Node name `roamadome_bench`, namespace same as the hardware node.

| Name | Kind | Default | Notes |
|------|------|---------|-------|
| `output_dir` | param | `~/dome_bench` | CSV and JSON written here, one subdirectory per run with a timestamp |
| `heartbeat_period_s` | param | 0.5 | Re-sends the current percent to keep the watchdog alive |
| `sensor_sweep.percents` | param | `[20, 35, 50, 75, 100]` | Each run in both directions, lowest first |
| `sensor_sweep.duration_s` | param | 30.0 | Per percent per direction |
| `sensor_sweep.max_plausible_deg_s` | param | 720.0 | Jump threshold input; replaced by the speed sweep result once known |
| `speed_sweep.percents` | param | `[10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100]` | Lowest first |
| `speed_sweep.settle_s` | param | 2.0 | Wait after each change before measuring |
| `speed_sweep.min_measure_s` | param | 4.0 | Measurement window lower bound; extended to one full revolution at the observed speed |
| `speed_sweep.stall_deg_s` | param | 3.0 | Below this for `stall_s` counts as stalled |
| `speed_sweep.stall_s` | param | 2.0 | |
| `speed_sweep.raw_motor` | param | true | Temporarily set max speed 100, target speed 100, scaling off |
| `~/start_sensor_sweep` | service | `std_srvs/Trigger` | Rejects if a sweep is running |
| `~/start_speed_sweep` | service | `std_srvs/Trigger` | Rejects if a sweep is running |
| `~/abort` | service | `std_srvs/Trigger` | Stops the dome, restores settings, writes partial results |
| `~/status` | topic | `std_msgs/String` | JSON: phase, percent, direction, elapsed, counts so far |

### 5.5 CSV schema

One row per telemetry message during a sweep:

```
t_host,seq,position_deg,unwrapped_rad,velocity_rad_s,commanded_percent,direction,step_index,phase
```

`phase` is `settle` or `measure`. Results JSON per sweep is specified in
sections 7.3 and 7.4.

## 6. Phases

### Phase 1: velocity estimate and telemetry publisher

Repositories: `astromech_msgs` (message), `ros2_roamadome`.

Steps:

1. Add `DomeTelemetry.msg` to `astromech_msgs`, register it in
   `CMakeLists.txt`, build, merge.
2. Add `astromech_msgs` as a dependency in `package.xml` and `CMakeLists.txt`.
3. Add a small header-only `PositionTracker` class under
   `include/ros2_roamadome/` with no ROS dependency: takes
   `(steady_clock time, uint16 degrees)`, maintains unwrapped radians and a
   ring buffer over `velocity_window_s`, returns the regression slope or NaN.
   Unwrap with `delta = ((new - last + 180) mod 360) - 180`.
4. In `SerialEventHandler::onPositionUpdate`, feed the tracker, write
   `position_` and `velocity_`, and publish telemetry if enabled. Publishing
   from the read path is acceptable at 30 Hz; use a `rclcpp::Publisher`
   created in `on_activate` and reset in `on_deactivate`. Record
   `commanded_percent` and `command_mode` from the write path under a mutex or
   atomics.
5. Parse the three new hardware parameters in `on_init` using the helpers in
   `parameter_parser.hpp`.
6. Update the README interface tables and the "velocity not computed" notes.

Acceptance:

- `ros2 topic echo /RoamADome/telemetry` shows plausible unwrapped position
  and velocity during a manual teleop move. Confirm the node name and
  namespace in use rather than assuming.
- Velocity is NaN for the first window after activation, then finite.
- Velocity sign matches the sign of the unwrapped position change.

Tests (gtest, no hardware):

- Unwrap across the 359 to 0 boundary in both directions.
- Slope of a synthetic constant-velocity stream within 2% after the window
  fills; NaN before.
- A 1 degree quantised stream at 30 deg/s and 28 ms spacing gives a
  velocity within 5% once the window is full.

### Phase 2: classify dropped firmware lines and publish diagnostics

Repository: `ros2_roamadome`.

Steps:

1. Add a `FirmwareLineClassifier` free function that maps a line to a tagged
   variant covering every row in 2.5, with parsed numbers where present.
   Unit-test it against literal firmware strings.
2. Call it from `onUnhandledLine` before the existing handlers. Keep the
   existing startup and ack handlers unchanged.
3. Maintain counters and last-seen values: position timeouts, sensor error
   count, packet serial error count, last accepted rotate percent, last
   target-reached position, last `READY` time.
4. Publish `diagnostic_msgs/DiagnosticArray` on `/diagnostics` at 1 Hz from a
   wall timer created in `on_activate`. `ERROR: DOME POSITION TIMEOUT` sets
   `motion` to ERROR for 5 s then WARN until a successful move. A sensor error
   count that increased since the last poll sets `sensor` to WARN.
5. Queue `#DPSTATUS` plus `#DPINVALID` on a timer every
   `status_poll_interval_ms` using `enqueueConfigDumpWithInvalidLocked` as the
   model. The status section parser already exists; extend `onStatusUpdate`
   to feed the counters.
6. A `READY` line seen after activation means the controller rebooted. Log
   at ERROR and set `link` to ERROR; the human decides what to do.

Acceptance: `ros2 topic echo /diagnostics` shows the three statuses; a
deliberate position timeout (hold the dome, command a move) flips `motion`.

### Phase 3: writable speed settings, `Unchanged` ack, set-home service

Repository: `ros2_roamadome`.

Steps:

1. Convert these specs in `src/device_parameter_specs.cpp` from read-only to
   writable with the given prefixes and ranges:

   | Field | Prefix | Range |
   |-------|--------|-------|
   | `max_speed` | `#DPMAXSPEED` | 0..100 |
   | `min_speed` | `#DPMINSPEED` | 0..100 |
   | `speed_target` | `#DPTARGETSPEED` | 0..100 |
   | `speed_home` | `#DPHOMESPEED` | 0..100 |
   | `scaling` | `#DPSCALE` | bool |
   | `acceleration_scale` | `#DPASCALE` | 0..255 |
   | `deceleration_scale` | `#DPDSCALE` | 0..255 |
   | `timeout` | `#DPTIMEOUT` | 1..255 |

   Leave `inverted`, `home_pos`, `auto_safety`, baud rates and addresses
   read-only.
2. In `handleSendQueueFeedback`, treat `Unchanged` while in
   `WAIT_WRITE_SETTINGS` as a completed ack with no config refresh.
3. Add `~/set_home_here` next to `~/setup`. It queues `#DPHOMEPOS` as an
   ack-tracked command with a post-ack config dump so `device.home_pos`
   refreshes.
4. Update the README writable-parameter table and the parameter test files.

Acceptance: `ros2 param set <node> device.max_speed 60` logs `Write Settings`
and `Updated`, and `ros2 param get` returns 60 after the refresh. Setting the
same value again returns promptly without a timeout warning.

### Phase 4: bench gate, raw drive service, launch and xacro plumbing

Repositories: `astromech_msgs` (service), `ros2_roamadome`, `r2_bringup`.

Steps:

1. Add `DomeBenchDrive.srv` to `astromech_msgs`, merge.
2. In the plugin, parse `enable_bench_mode` and
   `bench_heartbeat_timeout_ms`. When enabled, create `~/bench/drive` in
   `on_activate`. The handler:
   - rejects with a message if `currentMode_` is not `IDLE`, so it can never
     fight a controller;
   - clamps to -100..100, stores the request as the pending bench percent,
     records the time, and returns success;
   - does not touch the serial port directly. The `write()` loop sends
     `:DPR<p>` when the pending bench percent differs from the last sent, and
     sends `:DPR0` and clears bench state if the heartbeat has expired while
     the last sent percent is non-zero.
3. `perform_command_mode_switch` into POSITION or VELOCITY while a bench
   drive is active must send `:DPR0` first and clear bench state.
   `on_deactivate` already stops the dome; make sure bench state is cleared
   there too.
4. In `r2_bringup`: add a `dome_bench` xacro arg defaulting to `false` in
   `robot.urdf.xacro`, pass it through to `ros2_control.xacro` as the
   `enable_bench_mode` and `publish_telemetry` params on the RoamADome block,
   and add a `dome_bench` launch argument to `launch_robot.launch.py` that
   forwards the arg to the robot state publisher include, skips the
   `dome_controller` spawner and the dome teleop include when true, and
   starts the `roamadome_bench` node when true. Read `rsp.launch.py` to see
   how xacro arguments are passed today and follow it.

Acceptance:

- With `dome_bench:=false` nothing changes: the service does not exist and
  `ros2 control list_controllers` is as before.
- With `dome_bench:=true` the service exists, `dome_controller` is not
  spawned, and a call with percent 20 followed by silence stops the dome
  within the heartbeat timeout. Test the heartbeat stop before anything else.

Tests: pseudo-terminal test that a bench drive call produces `:DPR20` on the
wire, a second identical call produces nothing, and a simulated clock advance
past the timeout produces `:DPR0`.

### Phase 5: bench node

Repository: `ros2_roamadome`. Install with `ament_cmake_python` or
`install(PROGRAMS ...)` so the executable is `ros2 run ros2_roamadome
roamadome_bench`. Put the analysis in a module that imports without `rclpy`.

Structure:

- `ros2_roamadome/bench/analysis.py`: unwrap, velocity regression, jump
  scoring, stall detection, affine fit. Pure functions over lists of tuples.
- `ros2_roamadome/bench/node.py`: the ROS node. A single worker thread runs
  the active sweep; service callbacks only start, abort and report.
- `scripts/roamadome_bench`: entry point.
- `scripts/roamadome_bench_report`: offline CLI that re-runs the analysis on
  a CSV directory and prints the JSON. Useful for the human and for tests.

Sweep lifecycle, common to both sweeps:

1. Read `device.max_speed`, `device.min_speed`, `device.speed_target`,
   `device.scaling` and save them to `settings_before.json` in the run
   directory.
2. If `raw_motor` is set, write max speed 100, target speed 100, scaling
   false, and wait for each parameter to read back. Settings must be written
   before any rotation; see 2.3.
3. Run steps. A heartbeat timer re-sends the current percent every
   `heartbeat_period_s`.
4. On completion, abort, exception or SIGINT: send percent 0, wait until
   telemetry velocity is below the stall threshold or 5 s pass, restore the
   saved settings, write results.

Acceptance for the node as a whole:

- `ros2 service call ~/abort` during a sweep stops the dome within one
  heartbeat period and the restored settings match `settings_before.json`.
- Killing the node with SIGINT during a sweep does the same.
- Killing the node with SIGKILL during a sweep stops the dome via the
  plugin's heartbeat timeout, and the human restores settings by hand from
  `settings_before.json`. Document this in the README.

### Phase 6: hardware validation

Run with a human present. Produce a short report in `docs/bench_results/`
with the date, firmware config dump, both sweeps' JSON, and observations.
Expected outcomes that feed phase 7: maximum angular velocity per direction,
minimum sustainable percent per direction, the affine fit, and the jump
histogram. If jumps cluster at particular angles, record them for the
mechanical adjustment the human will make, then re-run the sensor sweep.

### Phase 7: replace the linear velocity mapping

Repository: `ros2_roamadome`.

Steps:

1. Add hardware parameters `velocity_map.k_percent_per_rad_s`,
   `velocity_map.p0_percent`, `velocity_map.min_percent`, each optionally per
   direction, defaulting to values that reproduce today's linear behaviour so
   the change is opt-in.
2. `velocityToPercentage` becomes: zero if `|v|` maps below `min_percent`;
   otherwise `sign(v) * clamp(p0 + |v| * k, min_percent, 100)`.
3. Set `max_speed_rad_per_sec` from the measured maximum and document that
   it now means the measured rate at 100%, with the firmware's `MaxSpeed`
   and `SpeedTarget` noted as part of the calibration state.
4. Add the measured numbers to `ros2_control.xacro` in `r2_bringup`.

Acceptance: a commanded 0.5 rad/s produces a telemetry velocity within 10% of
0.5 rad/s in both directions after settling.

## 7. Algorithms

### 7.1 Unwrap and velocity

Unwrapped degrees accumulate `((new - last + 180) mod 360) - 180` using the
integer report values. Velocity is the least-squares slope of unwrapped angle
against host time over the trailing `velocity_window_s`, requiring at least 4
samples spanning at least 40% of the window; otherwise NaN. Do not
differentiate adjacent samples: 1 degree quantisation at 28 ms is 36 deg/s of
noise per sample.

### 7.2 Jump, reversal and stall scoring

Per consecutive sample pair with interval `dt` and change `d` in degrees:

- Jump: `|d| > max_plausible_deg_s * dt + 2`. Record start angle, `d`, `dt`,
  commanded percent.
- Reversal: `sign(d)` opposite to the commanded direction with `|d| >= 2`
  for two consecutive pairs while `|percent| >= min_percent_known`.
- Stall: no change in position for `stall_s` while `|percent| >=
  min_percent_known`.

`max_plausible_deg_s` defaults to the parameter and is replaced by 1.5 times
the measured maximum once a speed sweep result exists in `output_dir`.

Sensor sweep result JSON: per percent and direction, sample count, mean and
standard deviation of velocity over the measure phase, jump count, reversal
count, stall count, and a histogram of jump start angles in 10 degree bins
aggregated across all steps. Pass criterion suggested as zero jumps and zero
reversals at every step; the human decides.

### 7.3 Speed sweep

For each percent in ascending order, for each direction:

1. Command the percent, wait `settle_s`.
2. Measure for `max(min_measure_s, 360 / |velocity estimate|)` seconds,
   capped at 30 s. Record mean velocity, standard deviation, and the number
   of full revolutions covered.
3. If the stall criterion triggers during settle or measure, mark the step
   stalled, command 0, wait for standstill, and continue to the next percent.
4. After the ascending pass, binary-search between the highest stalled
   percent and the lowest moving percent to 2% resolution, each probe using
   the same settle and stall logic. Record the boundary as `min_percent`.
5. Command 0 between directions and wait for standstill.

Fit, per direction, over the moving steps: `omega = k * (p - p0)` by least
squares. Report `k`, `p0`, R squared, `omega_max` at 100%, `min_percent`,
and the velocity at `min_percent`. Also report the ratio of the two
directions' `omega_max` as an asymmetry figure.

Speed sweep result JSON: the per-step table, the per-direction fit, the
recommended `max_speed_rad_per_sec`, and the phase 7 parameters.

## 8. Protocol features not yet exposed

Candidates for later work, in rough priority order. None are needed for the
two diagnostics.

1. `:DPA<deg>,<speed>,<maxspeed>`: absolute move with a start speed and a
   ramp-to speed. The interface sends the bare form, so every move uses the
   stored target speed. Could become a third command interface or a
   per-move parameter.
2. `:DPD<deg>` relative move, `:DPH[speed]` home, `:DPW` waits, `:DPS<n>`
   stored sequences and `#DPS<n>:...` to store them. Natural services; home
   and relative moves help calibration.
3. `#DPDEBUG1` on the ESP32 prints `ST: <motor>` per motor update and the
   ramp trace. A second, firmware-side measurement to correlate with the
   speed sweep. Noisy; bench only.
4. `:DPT<pin>`, `:DPP<pin><val>` digital outputs and `:DPQ` servo moves.
5. `#DPRESTART`, `#DPZERO`, `#DPFACTORY`. Guarded services for bench
   recovery only.
6. `#DPSETUPVELOCITY`, `#DPAUTORESTART`, `#DPINPUTSPEED`, `#DPPWMARC`:
   settings that exist in firmware but are not in the firmware README; the
   last three are already parsed read-only.

## 9. Optional firmware changes

Out of scope here, but each would make the diagnostics materially better.
The project already has a merged upstream PR, so these are realistic.

1. Report format with the controller's millisecond clock, mode character and
   motor fraction on each position line, or a new `#DPREPORT` variant.
2. A flag to include the unfiltered sensor angle beside the median-filtered
   one. The median filter hides exactly the glitches the sensor sweep looks
   for.
3. Count implausible angle changes in the sensor error counter, not just
   parse errors.
4. `calculateSpeed` converts inches with 0.0256 rather than 0.0254, and the
   dome diameter is hard-coded. Minor, since this plan does not rely on the
   firmware's speed estimate.

## 10. Decisions for the human

1. Interface package: this plan puts new messages in `astromech_msgs`
   because it already holds the project's dome messages. If you would rather
   keep `ros2_roamadome` self-contained, the package can generate its own
   interfaces instead; say so before phase 1.
2. Topic namespace for `/diagnostics`: standard global topic, or under the
   hardware node namespace.
3. Whether phase 6 results should be committed to the repository or kept
   outside it.
4. Whether to pursue the firmware changes in section 9 in parallel.
