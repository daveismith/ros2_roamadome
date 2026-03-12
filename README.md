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
| `dome_joint/velocity` | Current dome angular velocity in rad/s (currently 0.0 unless device velocity feedback is added) |

### Command Interfaces

The controller accepts the following command interfaces for the `dome_joint`:

| Interface | Description |
|-----------|-------------|
| `dome_joint/position` | Desired dome position in radians |
| `dome_joint/velocity` | Desired dome angular velocity in rad/s |

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

Read-only parameters (from config snapshot parsing):

- `device.home_pos`
- `device.max_speed`
- `device.min_speed`
- `device.input_speed`
- `device.scaling`
- `device.inverted`
- `device.timeout`
- `device.auto_safety`
- `device.auto_restart`
- `device.acceleration_scale`
- `device.deceleration_scale`
- `device.home_min_delay`
- `device.home_max_delay`
- `device.target_min_delay`
- `device.target_max_delay`
- `device.setup_angular_velocity`
- `device.speed_home`
- `device.speed_target`
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
- Ack-tracked updates wait for `Write Settings` then `Updated` feedback from firmware.
- After update ack, a config refresh (`#DPCONFIG` + `#DPINVALID`) is queued to synchronize all `device.*` values.
- Device-to-ROS synchronization uses a thread-local callback bypass guard to avoid callback loops
  without dropping concurrent external writes from other threads.

### Parameter Mutability Test Plan

- Verify internal sync path can update non-writable `device.*` fields through `set_parameters(...)`.
- Verify external writes to non-writable `device.*` fields are rejected with a clear reason.
- Verify writable `device.*` fields still go through callback validation and active-hardware checks.
- Verify non-`device.*` parameters are ignored by the device callback policy.

Examples:

```bash
# Toggle auto mode at runtime
ros2 param set /roamadome device.auto_mode true

# Update auto sweep limits
ros2 param set /roamadome device.auto_left 90
ros2 param set /roamadome device.auto_right 90

# Read back synchronized values
ros2 param get /roamadome device.auto_mode
ros2 param get /roamadome device.home_pos
```

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
- Velocity state feedback is currently exported but not computed from hardware feedback yet.
- Read/write cycle timing is managed by the ROS2 controller manager.

## Testing

Run package tests with:

```bash
cd ~/r2_ws
colcon test --packages-select ros2_roamadome
colcon test-result --verbose
```

Current tests include serial parser behavior, command terminator behavior, and configure-state-machine startup coverage.

## License

This package is part of the R2 robotics project. See your repository's LICENSE file for details.

## Additional Resources

- [ROS2 Hardware Interfaces](https://docs.ros.org/en/rolling/Concepts/Advanced/Hardware-Acceleration/Overview-of-hardware-interfaces.html)
- [Roam-A-Dome Firmware (GitHub)](https://github.com/reeltwo/DomeControlFirmware)
- [ROS2 Control Framework](https://control.ros.org/)

## Support

For issues related to:
- **ROS2 Integration**: Check ROS2 control framework documentation
- **Hardware Firmware**: See [DomeControlFirmware Issues](https://github.com/reeltwo/DomeControlFirmware/issues)
- **This Package**: Check the main R2 project repository
