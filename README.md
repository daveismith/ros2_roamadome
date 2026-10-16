# ros2_roamadome

A ROS2 hardware interface for the [Roam-A-Dome](https://github.com/reeltwo/DomeControlFirmware) controller, which manages dome positioning for R2-D2 models and similar projects.

## Overview

This package provides a ROS2 control hardware interface that integrates with the Roam-A-Dome controller firmware. It exposes the dome joint through standard ROS2 hardware interfaces, allowing controllers and motion planning nodes to command and monitor dome position and velocity.

### Key Features

- **Hardware Interface**: Implements `ActuatorInterface` for ROS2 control framework
- **Startup State Machine**: Configure lifecycle uses a read-driven command/probe sequence
- **State Interfaces**: Exposes dome joint position and velocity feedback
- **Command Interfaces**: Accepts position and velocity commands for dome movement
- **Pluginlib-based**: Loadable as a ROS2 controller plugin

## Hardware Background

The [Roam-A-Dome](https://github.com/reeltwo/DomeControlFirmware) (RDH) is an Arduino-based controller board for dome positioning systems. It supports:

- **Multiple Board Variants**: Arduino Mega, ESP32, ESP32S2, ESP32S3 with optional LCD displays
- **Controls**: Motor speed, acceleration/deceleration ramping, homing, and autonomous movement modes
- **Communication**: 
  - Serial packet protocol for command/control
  - PWM input/output support
  - WiFi support (ESP32 variants)
  - Droid Remote support for app-based control
- **Positioning**: Absolute and relative dome rotation with configurable speed profiles

For complete hardware documentation, see the [DomeControlFirmware repository](https://github.com/reeltwo/DomeControlFirmware).

## Package Structure

```
ros2_roamadome/
├── src/
│   ├── roamadome_control.cpp      # Hardware interface implementation
│   └── roamadome_control_node.cpp # Standalone node (if applicable)
├── ros2_roamadome/bench/
│   ├── analysis.py                # Bench analysis, pure Python (no ROS imports)
│   ├── control.py                 # Node safety logic: sweep claim, drive gate, validation
│   └── node.py                    # roamadome_bench node
├── scripts/
│   ├── roamadome_bench            # ros2 run entry point for the node
│   └── roamadome_bench_report     # Offline analysis of a run directory
├── include/ros2_roamadome/
│   ├── roamadome_control.hpp      # Hardware interface header
│   └── visibility_control.h       # Symbol visibility macros
├── robot_hardware.xml             # Hardware interface plugin description
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Interfaces

### State Interfaces

The controller exports the following state interfaces for the `dome_joint`:

| Interface | Description |
|-----------|-------------|
| `dome_joint/position` | Current dome position in radians (0-2π) |
| `dome_joint/velocity` | Dome angular velocity in rad/s, estimated from the firmware position stream (see [Velocity estimate and telemetry](#velocity-estimate-and-telemetry)). NaN until the regression window is populated after activation. |

### Command Interfaces

The controller accepts the following command interfaces for the `dome_joint`:

| Interface | Description |
|-----------|-------------|
| `dome_joint/position` | Desired dome position in radians |
| `dome_joint/velocity` | Desired dome angular velocity in rad/s |

### Velocity estimate and telemetry

The firmware reports only an integer, home-relative angle (`DOME POSITION: <n>`)
at the interval requested by `#DPREPORT`. The hardware interface unwraps that
stream into a continuous angle (shortest signed step between successive
readings) and estimates velocity as the least-squares slope of unwrapped angle
against host receive time over the trailing `velocity_window_s` seconds. The
estimate is NaN until the window holds at least four samples spanning at least
40% of the window. Adjacent samples are never differentiated directly: one
degree of quantisation at a 28 ms report interval would be 36°/s of noise per
sample. The implementation is the header-only `PositionTracker` in
`include/ros2_roamadome/position_tracker.hpp`.

With `publish_telemetry` set to `true`, every position report is also published
as `astromech_msgs/msg/DomeTelemetry` on `~/telemetry` of the hardware
component's node. The node is named after the `ros2_control` block in lower
case, so the `RoamADome` block publishes on `/roamadome/telemetry`:

| Field | Meaning |
|-------|---------|
| `header.stamp` | Host time when the read cycle consumed the report line, so quantised to the controller manager period |
| `position_deg` | Integer angle from the firmware, 0..359 |
| `position_rad` | The same angle in radians, [0, 2π) |
| `unwrapped_rad` | Continuous angle since activation |
| `velocity_rad_s` | Regression velocity estimate, NaN until the window is populated |
| `commanded_percent` | Last `:DPR` percentage sent in any mode, 0 when none |
| `command_mode` | 0 idle, 1 position, 2 velocity, 3 bench |
| `sequence` | Increments per report line since activation |

The topic is intended for the bench diagnostics described in
[docs/dome_diagnostics_plan.md](docs/dome_diagnostics_plan.md) and is off by default.
Because the serial port is polled from `read()`, the stamp resolution is one
controller manager cycle (33 ms at 30 Hz) and two report lines that arrive in
the same cycle get stamps only microseconds apart; the regression velocity
tolerates this, but a per-sample timing analysis should not assume evenly
spaced samples.

### Bench mode

With the hardware parameter `enable_bench_mode` set to `true` the hardware node offers
`~/bench/drive` (`astromech_msgs/srv/DomeBenchDrive`, request `int8 percent`, -100..100,
0 stops), a raw rotate service for bench diagnostics (docs/dome_diagnostics_plan.md). With
the parameter `false` (the default) the service does not exist.

- **Raw drive.** The percentage is sent as `:DPR<p>` and is subject to the firmware's own
  `MinSpeed` (below it the dome stops), `SpeedTarget` (clamp) and `MaxSpeed` (scale), exactly
  as for the velocity controller (plan section 2.3). Read the values from `device.*`.
- **Heartbeat.** A drive is kept alive by calling the service again. The service only records
  the request; the `write()` loop sends `:DPR<p>` when the percent differs from the last one
  sent (an identical repeat call sends nothing but refreshes the heartbeat). If no call
  arrives within `bench_heartbeat_timeout_ms` while the last sent percent is non-zero, the
  interface logs a WARN and sends `:DPR0`.
- **Gating.** The call is rejected (`success: false` with a message) unless the interface is
  active, the serial port is open and no controller holds the command interfaces
  (command mode IDLE), so it can never fight `dome_controller`. Starting a controller while a
  bench drive is active sends `:DPR0` first and clears the bench state, as does deactivation.
- **Telemetry.** While a bench drive is active `~/telemetry` reports `command_mode` 3
  (`COMMAND_MODE_BENCH`) and the sent percent in `commanded_percent`; it returns to 0 when
  the drive stops. Settings writes are refused while `commanded_percent` is non-zero.

Safety: the firmware has no watchdog in continuous rotation. The heartbeat only protects
while this process is alive. If the process is killed (SIGKILL, crash, power loss to the
host) the dome keeps turning at the last `:DPR` value; only a normal `on_deactivate` (which
sends `:DPR0`) or a manual stop will halt it. Keep a human at the robot and the dome clear.

### Bench node

`roamadome_bench` (Python, `ros2_roamadome/bench/`) runs the two bench diagnostics of
[docs/dome_diagnostics_plan.md](docs/dome_diagnostics_plan.md) through a bench-enabled
hardware node: a **sensor sweep** that spins the dome at several percentages in both
directions and scores every position report for jumps, reversals and stalls (plan 7.2), and a
**speed sweep** that measures steady-state velocity per percentage, finds the minimum moving
percentage and fits `omega = k * (p - p0)` per direction (plan 7.3).

Launch it with the robot (`dome_bench:=true` also skips `dome_controller` and the dome teleop):

```bash
ros2 launch r2_bringup launch_robot.launch.py dome_bench:=true
```

or on its own against any hardware node that has `enable_bench_mode` and `publish_telemetry`
set and no controller claiming the dome:

```bash
ros2 run ros2_roamadome roamadome_bench --ros-args -p output_dir:=~/dome_bench
ros2 service call /roamadome_bench/start_sensor_sweep std_srvs/srv/Trigger
ros2 topic echo /roamadome_bench/status
ros2 service call /roamadome_bench/abort std_srvs/srv/Trigger
```

The node finds the hardware node by name through the `hardware_node` parameter (default
`roamadome`, the lower-cased `ros2_control` block name), not by sharing its namespace as plan
5.4 suggests: the hardware node's name comes from the URDF, so a parameter is simpler than
pushing the bench node into a matching namespace. It uses `<hardware_node>/telemetry`,
`<hardware_node>/bench/drive` and `<hardware_node>/{get,set}_parameters`.

Parameters (read when a sweep starts, except `hardware_node` and `heartbeat_period_s`, which
are read at startup and cannot be changed while the node runs). Invalid values are rejected,
not corrected: at startup the node exits with an error listing them, and a later
`ros2 param set` with an invalid value fails. Percent lists must be integers in 1..100,
strictly ascending without duplicates; settle times must be >= 0; durations and thresholds
> 0; `speed_sweep.min_measure_s` <= `speed_sweep.max_measure_s`.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `hardware_node` | `roamadome` | Hardware node name (a leading `/` is added if missing) |
| `output_dir` | `~/dome_bench` | One subdirectory per run, named `<YYYYmmdd-HHMMSS>-<sweep>` |
| `heartbeat_period_s` | `0.5` | Re-send the current percent this often; must be in (0, 1.0] s (the plugin stops after `bench_heartbeat_timeout_ms`, 1.5 s by default) |
| `velocity_window_s` | `0.5` | Regression window for the per-sample velocity in the analysis (same rule as the plugin) |
| `sensor_sweep.percents` | `[20, 35, 50, 75, 100]` | Run lowest first, each `+p` then `-p` |
| `sensor_sweep.settle_s` | `2.0` | Settle before the measure phase (addition to plan 5.4) |
| `sensor_sweep.duration_s` | `30.0` | Measure phase per percent per direction |
| `sensor_sweep.max_plausible_deg_s` | `720.0` | Jump threshold; replaced by 1.5 x the measured maximum of a completed speed sweep in `output_dir` that used the same `raw_motor` and moved at or above the sensor sweep's highest percent |
| `speed_sweep.percents` | `[10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100]` | Ascending, each `+p` then `-p` |
| `speed_sweep.settle_s` | `2.0` | Wait after each command before measuring |
| `speed_sweep.min_measure_s` | `4.0` | Measure for max(this, one revolution at the observed speed) ... |
| `speed_sweep.max_measure_s` | `30.0` | ... capped here (addition to plan 5.4, which fixes 30 s) |
| `speed_sweep.stall_deg_s` | `3.0` | Stalled when the average speed over the last `stall_s` is below this; used by both sweeps |
| `speed_sweep.stall_s` | `2.0` | |
| `speed_sweep.raw_motor` | `true` | For both sweeps, temporarily set `device.max_speed` 100, `device.speed_target` 100, `device.scaling` false |

Services and topics on the bench node:

| Name | Type | Notes |
|------|------|-------|
| `~/start_sensor_sweep` | `std_srvs/Trigger` | Rejected if a sweep runs or is starting (the slot is claimed atomically), telemetry is not flowing, the hardware is not idle (`command_mode` 0, `commanded_percent` 0) or a hardware service is missing |
| `~/start_speed_sweep` | `std_srvs/Trigger` | Same checks |
| `~/abort` | `std_srvs/Trigger` | Returns at once; the worker stops the dome, restores settings and writes partial results |
| `~/status` | `std_msgs/String` | JSON at 2 Hz: state, sweep, run directory, phase, step, percent, direction, elapsed, counts so far, last result |

Every sweep follows the same lifecycle: save `device.max_speed`, `min_speed`, `speed_target`
and `scaling` to `settings_before.json` (with the `ros2 param set` commands that restore them);
with `raw_motor`, write the raw settings at standstill and read each back (a value already set
is not written again, since every write goes to flash). The plugin accepts a parameter write
before the firmware acks it, so the read-back only confirms the ROS value. The node therefore
also waits (up to 10 s) for the firmware acks the plugin reports on `/diagnostics`
(`roamadome: motion` values `settings_writes_pending`, `settings_acks`,
`settings_ack_timeouts`): nothing pending, the ack count advanced by the number of writes
issued (`Unchanged` counts as an ack) and no ack timeout. If that fails the sweep ends with an
error before the dome moves. The first drive is also retried for up to 12 s while the plugin
rejects it because a settings write is in flight; acceptance only means no write is queued or
waiting any more (an ack timeout clears it too), not that the firmware acked. Then the steps
run. Each step commands its
percent, settles, measures, then commands 0 and waits for standstill (telemetry
`commanded_percent` 0 and |velocity| below `stall_deg_s`, or 5 s) before the next one, so
every step starts from rest. The bench node does its own stall detection, because the
firmware has no watchdog in continuous rotation: a stalled step is stopped at once and the
sweep moves on. The speed sweep then binary-searches each direction between the highest
stalled and the lowest moving percent to 2 % for `min_percent`. On completion, `~/abort`, an
exception, SIGINT or SIGTERM the node commands 0 (from then on it refuses to send any non-zero
percent and the heartbeat only sends 0), waits for standstill, restores the saved settings
that differ, waits for the firmware acks as above (status phase `restoring settings`), reads
the settings back into `settings_after.json` and writes the results. If the restore is not
confirmed it logs `settings restore NOT confirmed by firmware; check device.* and
settings_before.json` at ERROR and records `settings_restored: false`. A drive-service
rejection, a heartbeat call failure, a controller taking the dome or more than 1 s without
telemetry aborts the sweep. A second SIGINT exits without waiting for the restore (it still
sends 0).

Output files in each run directory:

| File | Contents |
|------|----------|
| `settings_before.json` | Saved settings, context (`inverted`, `home_pos`, ...), restore commands |
| `settings_after.json` | Settings read back after the restore, the restore writes issued, `ros_matches_before` (the ROS parameters equal the saved values), `firmware_acked` (true only if the firmware acked every restore write with no ack timeout; `null` if the plugin does not publish the counters, false otherwise) and `ack_timeouts_during_restore` |
| `samples.csv` | One row per telemetry message during a step: `t_host,seq,position_deg,unwrapped_rad,velocity_rad_s,commanded_percent,direction,step_index,phase` (plan 5.5); `direction` is the sign of the commanded percent, `phase` is `settle` or `measure`. Written as it arrives |
| `run.json` | Sweep type, status (`running`, `complete`, `aborted`, `error`) and reason, the analysis config, the step list, settings and counts |
| `sensor_sweep.json` | Per step: samples, mean and standard deviation of velocity over the measure phase, jumps (with start angle, change, interval, percent), reversals, stalls; the 10 degree jump start histogram; totals and `suggested_pass` (no jumps and no reversals) |
| `speed_sweep.json` | Per step table; per direction (`positive_percent`, `negative_percent`): fit `k`, `p0`, R squared, `omega_max_rad_s` at 100 %, `min_percent`, velocity at `min_percent`; asymmetry ratio; recommended `max_speed_rad_per_sec` (the lower of the two maxima); the phase 7 `velocity_map` parameters keyed by velocity sign |

Notes on the analysis (`ros2_roamadome/bench/analysis.py`, no ROS imports):

- The direction of angle change for a positive percent is measured, not assumed
  (`angle_sign_for_positive_percent`); on the bench robot a positive percent moves the reported
  angle negative. Reversals are scored against this measured sign, and the phase 7 parameters
  give the `percent_sign` that produces a positive (angle-increasing) velocity.
- Two report lines read in the same controller cycle are stamped only tens of microseconds
  apart (about 7 % of pairs on the bench robot), although the firmware sent them 28 ms apart;
  scored with that interval every normal move would look like a jump. Pairs closer than 5 ms
  (`same_cycle_dt_s` in the config) are therefore not scored for jumps; their count is
  reported as `same_cycle_pairs`, and the largest gap between reports (the ~230 ms periodic
  config refresh) as `max_gap_s`.
- Reversals and stalls count only at percentages at or above the known minimum: the speed
  sweep's `min_percent` when its result is used, otherwise the firmware `MinSpeed`.
- Velocity per step is the least-squares slope over the measure phase; the mean and standard
  deviation are of the windowed per-sample estimates. The fit uses the slope.

Re-run the analysis offline (plain Python, no ROS needed beyond the installed package):

```bash
ros2 run ros2_roamadome roamadome_bench_report ~/dome_bench/20261014-210000-speed
ros2 run ros2_roamadome roamadome_bench_report RUN_DIR --check          # exit 1 if it differs from the saved JSON
ros2 run ros2_roamadome roamadome_bench_report RUN_DIR --set max_plausible_deg_s=400
```

**If the bench node is killed with SIGKILL** (or crashes) during a sweep, the dome stops
through the plugin's heartbeat: no drive call arrives, so the hardware interface logs
`Bench drive stopped (heartbeat expired); sent :DPR0` within `bench_heartbeat_timeout_ms`.
The settings are **not** restored. Restore them by hand from the run's `settings_before.json`
(it lists the commands), for example:

```bash
ros2 param set /roamadome device.max_speed 80
ros2 param set /roamadome device.speed_target 100
ros2 param set /roamadome device.scaling false
ros2 param get /roamadome device.max_speed
```

The same applies when the whole launch is stopped with Ctrl-C during a sweep: the controller
manager deactivates the hardware (which sends `:DPR0`) at the same time, so the bench node
cannot write settings any more. Abort the sweep first and wait for `~/status` to report `idle`.

### Diagnostics

While active, the hardware interface publishes a `diagnostic_msgs/DiagnosticArray`
on the global `/diagnostics` topic at 1 Hz with three statuses, all carrying the
serial port path as `hardware_id`:

| Status | OK | WARN | ERROR |
|--------|----|------|-------|
| `roamadome: link` | Position reports flowing | No report yet, or none for 0.5 s | Serial port closed, or `READY` seen while active (the controller rebooted) |
| `roamadome: sensor` | Sensor error count steady | Count grew between the last two `#DPSTATUS` polls | |
| `roamadome: motion` | No timeout outstanding | `ERROR: DOME POSITION TIMEOUT` more than 5 s ago and no `REACHED TARGET` / `REACHED HOME` since | `ERROR: DOME POSITION TIMEOUT` within the last 5 s |

Values include the report and poll counters, the firmware's sensor and packet
serial error counts, the last accepted `:DPR` percent, the last target and home
reached, position timeouts, out-of-range rejections and the current command mode.
`roamadome: motion` also reports the settings-write pipeline: `settings_writes_pending`
(ack-tracked `#DP...` writes queued, plus one waiting for its ack), `settings_acks`
(completed by `Write Settings` + `Updated` or by `Unchanged`) and `settings_ack_timeouts`
(abandoned after the 3 s ack timeout; such a write may not have reached flash). The counters
reset on activation; `on_deactivate` drops queued writes.

The firmware's own counters come from `#DPSTATUS`, which the interface queues
(followed by `#DPINVALID` to close the section) once on activation and then every
`status_poll_interval_ms`; set it to `0` to disable polling. The console lines the
firmware prints for accepted commands, finished moves, rejections and boot
(`ROTATE SPEED`, `ROTARY DEGREE`, `REACHED TARGET`, `REACHED HOME`,
`ERROR: DOME POSITION TIMEOUT`, `Out of range.`, `Unchanged`, `READY`, the
`#DPSETUP` output and the reporting acks) are recognised by
`classifyFirmwareLine()` in `include/ros2_roamadome/firmware_line_classifier.hpp`
and folded into the diagnostics state; a position timeout and a reboot are also
logged at WARN and ERROR respectively.

## Dependencies

- **ROS2 Dependencies**:
  - `hardware_interface`: Hardware abstraction layer
  - `controller_manager`: Manages lifecycle and plugins
  - `pluginlib`: Dynamic plugin loading
  - `rclcpp`: C++ ROS2 client library

## Building

Build the package using `colcon`:

```bash
cd ~/r2_ws
colcon build --packages-select ros2_roamadome
```

## Usage

### Hardware Configuration

To use this hardware interface, include it in your ROS2 hardware configuration URDF or YAML file. The hardware interface will be loaded by the controller manager as a pluginlib plugin.

Example configuration:

```yaml
hardware:
  - name: roamadome_control
    type: ros2_roamadome/RoamadomeControl
```

### Launching Controllers

Once the hardware interface is loaded, you can load controllers (e.g., a position controller) that interact with the dome joint:

```bash
ros2 control load_controller dome_position_controller
ros2 control set_controller_state dome_position_controller active
```

### Commanding the Dome

Commands can be published to controller topics or issued via the controller manager interface.

## Setting up the Hardware Device

Before using this interface:

1. **Program the Roam-A-Dome**: Program the controller board with the DomeControlFirmware
2. **Serial Configuration**: Configure via serial commands (see firmware documentation)
3. **Homing**: Perform initial homing calibration either via serial, LCD menu, or hardware setup
4. **Connection**: Establish serial communication from ROS2 host to the controller

For detailed hardware setup instructions, see the [DomeControlFirmware README](https://github.com/reeltwo/DomeControlFirmware#readme).

## Serial Protocol

This interface communicates with the hardware via the Roam-A-Dome serial protocol.

### Configuration Parameters

The following parameters can be specified in your URDF/XACRO hardware description:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | *required* | Serial port device path (e.g., `/dev/ttyACM0`) |
| `serial_baud` | uint32 | `115200` | Serial baud rate |
| `max_speed_rad_per_sec` | double | `3.14` | Maximum dome angular velocity in rad/s (~180°/s) |
| `publish_telemetry` | bool | `false` | Publish `astromech_msgs/DomeTelemetry` on `~/telemetry` for every position report |
| `velocity_window_s` | double | `0.5` | Trailing regression window for the velocity estimate (s), must be > 0 |
| `enable_bench_mode` | bool | `false` | Create the raw drive service `~/bench/drive` (see [Bench mode](#bench-mode)); normally set by the `dome_bench` launch argument |
| `bench_heartbeat_timeout_ms` | uint32 | `1500` | Send `:DPR0` if no bench drive call arrives within this time while a bench drive is active, must be > 0 |
| `status_poll_interval_ms` | uint32 | `10000` | Interval for the `#DPSTATUS` poll that feeds `/diagnostics`; `0` disables |
| `auto_mode` | bool | `false` | Enable AutoMode on device during startup |
| `home_mode` | bool | `false` | Enable HomeMode on device during startup |
| `startup_log_level` | string | `debug` | Log level for startup transitions (`debug` or `info`) |
| `startup_default_timeout_ms` | uint32 | `1000` | Default timeout for startup commands (ms) |
| `startup_setup_timeout_ms` | uint32 | `10000` | Timeout for `#DPSETUP` command (ms) |
| `startup_report_timeout_ms` | uint32 | `1000` | Timeout for `#DPREPORT` command (ms) |
| `startup_retries` | uint32 | `1` | Number of retries for timed-out startup commands |
| `startup_config_stale_warning_ms` | uint32 | `30000` | Warning threshold for stale configuration data (ms) |
| `serial_section_flush_timeout_ms` | uint32 | `500` | Idle timeout before flushing partial CONFIG/STATUS sections during serial parsing (ms) |

**Example URDF Configuration:**

```xml
<ros2_control name="roamadome_actuator" type="actuator">
  <hardware>
    <plugin>ros2_roamadome/RoamadomeControl</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="serial_baud">115200</param>
    <param name="max_speed_rad_per_sec">3.14</param>
    <param name="auto_mode">true</param>
    <param name="home_mode">false</param>
    <param name="startup_log_level">info</param>
  </hardware>
  <joint name="dome_joint">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

**Note on Logging Levels:**
- `startup_log_level=debug`: Startup transition logs only appear when running with `--ros-args --log-level debug`
- `startup_log_level=info`: Startup transition logs always appear at default log level

### Runtime ROS Parameters (device.*)

After `on_configure()`, the hardware interface declares a `device.*` parameter set and keeps it synchronized
with controller configuration snapshots from the serial device.

Configuration metadata is centralized in `DeviceParameterSpec` definitions (`device_parameter_specs.cpp`).
The same per-field definitions now drive:

- device config parsing (`#DPCONFIG` key/value -> typed parse + validation)
- per-device value storage inside the instance-owned parameter spec registry
- device-to-ROS synchronization (`#DPCONFIG` key/value -> `device.*` parameter updates)
- writable parameter validation and command generation (`device.*` -> serial command)

This keeps field names, ranges, and type expectations in one place when adding or changing parameters.

When adding or modifying spec definitions in `device_parameter_specs.cpp`, use this constructor order:

- `ReadOnlyParameterSpec`: `field_name`, `firmware_config_key`, `description`, `default_value`,
  `min_value`, `max_value`
- `WritableParameterSpec`: `field_name`, `firmware_config_key`, `description`, `command_prefix`,
  `default_value`, `min_value`, `max_value`

`firmware_config_key` is required for all specs and is used for `#DPCONFIG` key mapping.

Writable parameters (ROS -> device):

| Parameter | Type | Valid Range | Device Command |
|-----------|------|-------------|----------------|
| `device.auto_mode` | bool | `true/false` | `#DPAUTO0/1` |
| `device.home_mode` | bool | `true/false` | `#DPHOME0/1` |
| `device.auto_left` | int | `0..180` | `#DPAUTOLEFT<n>` |
| `device.auto_right` | int | `0..180` | `#DPAUTORIGHT<n>` |
| `device.auto_min_delay` | int | `0..255` | `#DPAUTOMIN<n>` |
| `device.auto_max_delay` | int | `0..255` | `#DPAUTOMAX<n>` |
| `device.speed_auto` | int | `0..100` | `#DPAUTOSPEED<n>` |
| `device.fudge` | int | `0..20` | `#DPFUDGE<n>` |
| `device.max_speed` | int | `0..100` | `#DPMAXSPEED<n>` |
| `device.min_speed` | int | `0..100` | `#DPMINSPEED<n>` |
| `device.speed_target` | int | `0..100` | `#DPTARGETSPEED<n>` |
| `device.speed_home` | int | `0..100` | `#DPHOMESPEED<n>` |
| `device.scaling` | bool | `true/false` | `#DPSCALE0/1` |
| `device.acceleration_scale` | int | `0..255` | `#DPASCALE<n>` |
| `device.deceleration_scale` | int | `0..255` | `#DPDSCALE<n>` |
| `device.timeout` | int | `1..30` | `#DPTIMEOUT<n>` |

The firmware clamps `#DPTIMEOUT` to 30 seconds, so larger values are rejected here rather than
silently clamped. The firmware can report `Timeout=0`, which is accepted when reading the config
but rejected on write (`1..30`). Every write persists to flash.

Writing a setting makes the firmware stop the drive (for example `setMaxSpeed` calls `stop()`), and
a position move in flight is aborted by the firmware when any setting is written. The interface
does not block the latter. It does reject `device.*` writes and `~/set_home_here` while a
continuous rotation is commanded (last `:DPR` percent is non-zero), because `write()` only resends
`:DPR` when the command changes and the dome would stay stopped while the controller believes it
is driving. Stop the rotation first.

Read-only parameters (from config snapshot parsing):

- `device.home_pos`
- `device.input_speed`
- `device.inverted`
- `device.auto_safety`
- `device.auto_restart`
- `device.home_min_delay`
- `device.home_max_delay`
- `device.target_min_delay`
- `device.target_max_delay`
- `device.setup_angular_velocity`
- `device.syren_address_in`
- `device.syren_address_out`
- `device.sensor_baud`
- `device.syren_baud`
- `device.serial_baud`
- `device.serial_in`
- `device.serial_out`
- `device.pwm_in`
- `device.pwm_out`
- `device.pwm_min_pulse`
- `device.pwm_max_pulse`
- `device.pwm_neutral_pulse`
- `device.pwm_deadband`
- `device.pwm_arc_mode`
- `device.digital_out`

Parameter update behavior:

- Non-writable `device.*` fields are declared with `read_only=false` at the ROS descriptor layer so
  internal Device->ROS snapshot sync (`node->set_parameters(...)`) can update them.
- The device parameter callback is registered during `on_configure()` when parameters are declared,
  so writes between configure and activate are still validated deterministically.
- External writes to non-writable `device.*` fields are rejected in
  `RoamadomeControl::onParameterChange(...)`.
- External writes to writable `device.*` fields are rejected while hardware is inactive
  (`on_configure` completed but not active, or deactivated).
- Writable changes are validated in the parameter callback before queueing serial commands.
- Non-writable `device.*` parameters are updated from device config reads and rejected if a user tries to set them directly.
- Ack-tracked updates wait for `Write Settings` then `Updated` feedback from firmware. If the
  value is already set the firmware prints only `Unchanged`; that completes the ack immediately
  and no config refresh is queued. `Write Settings`, `Updated` and `Unchanged` acks are logged at
  INFO.
- After update ack, a config refresh (`#DPCONFIG` + `#DPINVALID`) is queued to synchronize all `device.*` values.
- Device-to-ROS synchronization uses a thread-local callback bypass guard to avoid callback loops
  without dropping concurrent external writes from other threads.
- Firmware config parsing accepts `SetupAngularVelocity` values as plain numbers (for example,
  `100`) or with an optional `cm/s` suffix (for example, `100 cm/s`).

### Parameter Mutability Test Plan

- Verify internal sync path can update non-writable `device.*` fields through `set_parameters(...)`.
- Verify external writes to non-writable `device.*` fields are rejected with a clear reason.
- Verify writable `device.*` fields still go through callback validation and active-hardware checks.
- Verify non-`device.*` parameters are ignored by the device callback policy.
- Verify the speed and ramp settings (`device.max_speed` etc.) build the documented `#DP...` command
  and reject out-of-range values, while `inverted`, `home_pos`, `auto_safety`, baud rates and
  addresses stay read-only.
- Verify a firmware `Unchanged` reply completes an ack-tracked write without a `#DPCONFIG` refresh
  or an ack timeout.
- Verify settings writes and `~/set_home_here` are rejected while a rotation is commanded and
  accepted after `:DPR0`.
- Verify `~/set_home_here` queues `#DPHOMEPOS` and that `Write Settings` + `Updated` triggers a
  config refresh.

Examples:

```bash
# Toggle auto mode at runtime
ros2 param set /roamadome device.auto_mode true

# Update auto sweep limits
ros2 param set /roamadome device.auto_left 90
ros2 param set /roamadome device.auto_right 90

# Change a speed setting (persists to flash)
ros2 param set /roamadome device.max_speed 60

# Store the current dome position as home (refreshes device.home_pos)
ros2 service call /roamadome/set_home_here std_srvs/srv/Trigger

# Read back synchronized values
ros2 param get /roamadome device.auto_mode
ros2 param get /roamadome device.home_pos
```

Services on the hardware node:

| Service | Type | Effect |
|---------|------|--------|
| `~/setup` | `std_srvs/Trigger` | Sends `#DPSETUP` (one-shot) |
| `~/set_home_here` | `std_srvs/Trigger` | Sends `#DPHOMEPOS`, ack-tracked; the current position becomes home and `device.home_pos` refreshes after the ack. `success=true` means only that the command was queued, not that the firmware acked it. Rejected while inactive, with the port closed, or while a rotation is commanded |

### Startup Configure Sequence

During `on_configure()`, the hardware interface performs a deterministic startup sequence to ensure the device is properly configured:

```mermaid
flowchart TD
    Start([Start Configure]) --> Baud[Baud Rate Sweep]
    Baud --> Config[CONFIG_INITIAL: Read Config]
    Config --> CheckAutoSafety{AutoSafety == 0?}
    
    CheckAutoSafety -->|Yes| Status[STATUS: Verify Status]
    CheckAutoSafety -->|No| Setup[SETUP: Run Setup]
    
    Setup --> AutoSafety0[AUTOSAFETY0: Disable AutoSafety]
    AutoSafety0 --> Verify[VERIFY_AUTOSAFETY: Re-read Config]
    Verify --> CheckVerify{AutoSafety == 0?}
    CheckVerify -->|Yes| Status
    CheckVerify -->|No| Failed([FAILED])
    
    Status --> CheckConfigAge{Config > 30s old?}
    CheckConfigAge -->|Yes| WarnStale[Warn: Config Stale]
    CheckConfigAge -->|No| CheckAuto
    WarnStale --> CheckAuto{AutoMode Match?}
    
    CheckAuto -->|No| SetAuto[SET_AUTOMODE: #DPAUTO]
    CheckAuto -->|Yes| CheckHome{HomeMode Match?}
    
    SetAuto --> CheckHome
    CheckHome -->|No| SetHome[SET_HOMEMODE: #DPHOME]
    CheckHome -->|Yes| Report[REPORT: Enable Reporting]
    
    SetHome --> Report
    Report --> Complete([COMPLETE])
    
    style Start fill:#e1f5e1
    style Complete fill:#e1f5e1
    style Failed fill:#ffe1e1
    style CheckAutoSafety fill:#fff4e1
    style CheckVerify fill:#fff4e1
    style CheckConfigAge fill:#fff4e1
    style CheckAuto fill:#fff4e1
    style CheckHome fill:#fff4e1
```

**Startup Phases:**

1. **Baud Rate Sweep**: Iterate through supported baud rates (`2400`, `9600`, `19200`, `38400`, `115200`) and send `#DPSERIALBAUD<target>` at each rate. This is a best-effort step with no explicit completion check.

2. **CONFIG_INITIAL**: Read the device configuration using `#DPCONFIG`. If `AutoSafety` is already `0`, skip to STATUS. Otherwise, proceed to SETUP.

3. **SETUP → AUTOSAFETY0 → VERIFY_AUTOSAFETY** (conditional): If AutoSafety was enabled:
   - Run `#DPSETUP` (10-second timeout for mechanical homing)
   - Send `#DPAUTOSAFETY0` to disable auto safety
   - Re-read config with `#DPCONFIG` to verify `AutoSafety == 0`
   - If verification fails, startup fails with error

4. **STATUS**: Run `#DPSTATUS` to verify auto safety is disabled in device status

5. **Configuration Staleness Check**: If the configuration data is older than the threshold (default 30 seconds, configurable via `startup_config_stale_warning_ms`), log a warning. This can catch firmware communication issues.

6. **AutoMode/HomeMode Configuration**: Check if device AutoMode and HomeMode match desired states from URDF parameters:
   - If `auto_mode=true` but device `AutoMode != 1`, send `#DPAUTO1`
   - If `auto_mode=false` but device `AutoMode != 0`, send `#DPAUTO0`
   - If `home_mode=true` but device `HomeMode != 1`, send `#DPHOME1`
   - If `home_mode=false` but device `HomeMode != 0`, send `#DPHOME0`

7. **REPORT**: Enable periodic position reporting with `#DPREPORT<ms>`, where `<ms>` is calculated from the controller read/write rate

**Command Probe Pattern:**

Each startup command is followed by `#DPINVALID` as a completion probe. The device processes commands sequentially, so probe completion requires reading both:
- `PROCESS: "#DPINVALID"`
- `Invalid`

This ensures the previous command has been fully processed before proceeding.

**Timeouts and Retries:**

Each wait state performs serial reads in a loop with configurable timeouts:
- `startup_default_timeout_ms`: Default timeout for most commands (default: 1000ms)
- `startup_setup_timeout_ms`: Extended timeout for `#DPSETUP` due to mechanical homing (default: 10000ms)
- `startup_report_timeout_ms`: Timeout for `#DPREPORT` command (default: 1000ms)
- `startup_retries`: Number of retry attempts for timed-out commands (default: 1)

**Special Behaviors:**

- `#DPSETUP` output may include `GOOD MAX SPEED: <n>` from the firmware; this value is currently treated as informational and is not parsed or stored by the ROS2 interface
- Transitions are data-driven from a startup command table with conditional branching via lambda functions
- Configuration data includes a timestamp to detect stale data (warns if older than `startup_config_stale_warning_ms`, default 30 seconds)

## Integration Testing And Validation

Use this sequence for full validation on a real robot or test bench:

1. Build and run package tests:

```bash
cd ~/r2_ws
source setup.bash
colcon build --packages-select ros2_roamadome --symlink-install
colcon test --packages-select ros2_roamadome --event-handlers console_direct+
```

2. Launch hardware stack and verify startup:

```bash
source setup.bash
ros2 launch r2_bringup launch_robot.launch.py
```

3. Trigger setup service and verify success:

```bash
ros2 service call /roamadome/setup std_srvs/srv/Trigger
```

4. Validate parameter sync path end-to-end:

```bash
ros2 param set /roamadome device.auto_mode false
ros2 param get /roamadome device.auto_mode
```

5. Check logs for expected ack and sync flow:

- `Queued parameter update: device.*`
- `Received 'Write Settings'`
- `Received 'Updated'`
- `Device->ROS parameter sync complete`

### Command Terminators

`RoamadomeSerialPort::sendCommand()` appends a newline terminator by default when missing. Pass `append_terminator=false` for raw writes.

### Common Runtime Commands

Common commands:

- `:DPA<degrees>` - Rotate dome to absolute position (0-359°)
- `:DPH` - Return dome to home position
- `:DPR<speed>` - Rotate dome continuously
- `:DPS<number>` - Play stored sequence

For a complete list of commands and configuration options, refer to the [hardware documentation](https://github.com/reeltwo/DomeControlFirmware#serial-configuration-commands).

## Development Notes

- Startup configuration uses a deterministic state machine to sequence status/config/report setup.
- Position wraps around at 2π radians (360°).
- Velocity state feedback is estimated on the host from the position stream; the firmware sends no velocity.
- Read/write cycle timing is managed by the ROS2 controller manager.

## Testing

Run package tests with:

```bash
cd ~/r2_ws
colcon test --packages-select ros2_roamadome
colcon test-result --verbose
```

Current tests include serial parser behavior, command terminator behavior, and configure-state-machine startup coverage.
`test/test_bench_analysis.py` (pytest) covers the bench analysis on synthetic telemetry
streams without ROS, including the offline report tool; `test/test_bench_control.py` covers
the node's safety logic without a ROS graph (sweep claim, drive gate, parameter validation,
settings-ack check).

## License

This package is part of the R2 robotics project. See your repository's LICENSE file for details.

## Additional Resources

- [Dome diagnostics and telemetry plan](docs/dome_diagnostics_plan.md): bench sensor sweep, speed range measurement, and protocol features to expose
- [ROS2 Hardware Interfaces](https://docs.ros.org/en/rolling/Concepts/Advanced/Hardware-Acceleration/Overview-of-hardware-interfaces.html)
- [Roam-A-Dome Firmware (GitHub)](https://github.com/reeltwo/DomeControlFirmware)
- [ROS2 Control Framework](https://control.ros.org/)

## Support

For issues related to:
- **ROS2 Integration**: Check ROS2 control framework documentation
- **Hardware Firmware**: See [DomeControlFirmware Issues](https://github.com/reeltwo/DomeControlFirmware/issues)
- **This Package**: Check the main R2 project repository
