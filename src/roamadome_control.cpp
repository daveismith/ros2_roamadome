#include "ros2_roamadome/roamadome_control.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <format>
#include <iostream>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>

namespace ros2_roamadome
{

RoamadomeControl::RoamadomeControl()
: logger_(rclcpp::get_logger("RoamadomeControl")), position_(0.0)
{
  std::cout << "RoamadomeControl initialized." << std::endl;
}

RoamadomeControl::~RoamadomeControl()
{
  std::cout << "RoamadomeControl destroyed." << std::endl;
}

hardware_interface::CallbackReturn RoamadomeControl::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::ActuatorInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Note: info_ is inherited from ActuatorInterface and already populated by parent on_init()
  // Validate that exactly one joint is configured
  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(
      logger_,
      "RoamadomeControl supports exactly 1 joint, but URDF defines %zu joints",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Extract joint name from URDF configuration
  joint_name_ = info_.joints[0].name;
  RCLCPP_INFO(
    logger_,
    "RoamadomeControl configured for joint: '%s'", joint_name_.c_str());

    //RCLCPP_INFO(logger_, "Initializing RoamadomeControl with params: %s", params.name.c_str());
  RCLCPP_INFO(logger_, "Initializing RoamadomeControl with params: %s", info_.name.c_str());

  try {
    auto it = info_.hardware_parameters.find("serial_port");
    if (it != info_.hardware_parameters.end()) {
      serialPort_ = it->second;
    } else {
      RCLCPP_ERROR(logger_, "serial_port Parameter Not Found");
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (...) {
    RCLCPP_ERROR(logger_, "Failed to find serial_port parameter");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    auto it = info_.hardware_parameters.find("serial_baud");
    if (it != info_.hardware_parameters.end()) {
      serialBaud_ = static_cast<uint32_t>(std::stoul(it->second));
    } else {
      RCLCPP_WARN(logger_, "serial_baud not found, defaulting to 115200");
      serialBaud_ = 115200;
    }
  } catch (const std::out_of_range & oor) {
    RCLCPP_ERROR(logger_, "serial_baud out of range");
    return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::invalid_argument & ia) {
    RCLCPP_ERROR(logger_, "serial_baud has invalid format");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse optional max_speed_rad_per_sec parameter
  try {
    auto it = info_.hardware_parameters.find("max_speed_rad_per_sec");
    if (it != info_.hardware_parameters.end()) {
      maxSpeedRadPerSec_ = std::stod(it->second);
      if (maxSpeedRadPerSec_ <= 0.0) {
        RCLCPP_WARN(logger_, "max_speed_rad_per_sec must be positive, using default 3.14");
        maxSpeedRadPerSec_ = 3.14;
      }
    } else {
      RCLCPP_INFO(logger_, "max_speed_rad_per_sec not found, using default 3.14 rad/s");
      maxSpeedRadPerSec_ = 3.14;
    }
  } catch (const std::invalid_argument & ia) {
    RCLCPP_WARN(logger_, "max_speed_rad_per_sec has invalid format, using default 3.14");
    maxSpeedRadPerSec_ = 3.14;
  }

  try {
    auto timeout_it = info_.hardware_parameters.find("startup_default_timeout_ms");
    if (timeout_it != info_.hardware_parameters.end()) {
      startupDefaultTimeoutMs_ = static_cast<uint32_t>(std::stoul(timeout_it->second));
      if (0 == startupDefaultTimeoutMs_) {
        RCLCPP_WARN(logger_, "startup_default_timeout_ms must be > 0, using 1000");
        startupDefaultTimeoutMs_ = 1000;
      }
    }

    auto report_timeout_it = info_.hardware_parameters.find("startup_report_timeout_ms");
    if (report_timeout_it != info_.hardware_parameters.end()) {
      startupReportTimeoutMs_ = static_cast<uint32_t>(std::stoul(report_timeout_it->second));
      if (0 == startupReportTimeoutMs_) {
        RCLCPP_WARN(logger_, "startup_report_timeout_ms must be > 0, using default timeout");
        startupReportTimeoutMs_ = startupDefaultTimeoutMs_;
      }
    } else {
      startupReportTimeoutMs_ = startupDefaultTimeoutMs_;
    }

    auto setup_timeout_it = info_.hardware_parameters.find("startup_setup_timeout_ms");
    if (setup_timeout_it != info_.hardware_parameters.end()) {
      startupSetupTimeoutMs_ = static_cast<uint32_t>(std::stoul(setup_timeout_it->second));
      if (0 == startupSetupTimeoutMs_) {
        RCLCPP_WARN(logger_, "startup_setup_timeout_ms must be > 0, using 10000");
        startupSetupTimeoutMs_ = 10000;
      }
    }

    auto retries_it = info_.hardware_parameters.find("startup_retries");
    if (retries_it != info_.hardware_parameters.end()) {
      startupMaxRetries_ = static_cast<uint32_t>(std::stoul(retries_it->second));
    }

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Invalid startup timing parameters: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Maximum speed configured: %.4f rad/s (%.2f°/s)",
              maxSpeedRadPerSec_, radiansToDegrees(maxSpeedRadPerSec_));
  RCLCPP_INFO(logger_, "Startup timeouts: default=%u ms setup=%u ms report=%u ms retries=%u",
              startupDefaultTimeoutMs_, startupSetupTimeoutMs_, startupReportTimeoutMs_,
              startupMaxRetries_);

  initializeStartupCommandTable();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoamadomeControl::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  const struct timespec ts = {.tv_sec = 0, .tv_nsec = 500000000};
  const uint32_t report_ms = (1000 / info_.rw_rate) - 5;

  RCLCPP_INFO(logger_, "Configuring RoamadomeControl from state: %s",
      previous_state.label().c_str());
  RCLCPP_INFO(logger_, "Read Rate in Hz: %u (#DPREPORT%u)", info_.rw_rate, report_ms);

  // Create and setup the serial handler
  serialHandler_ = std::make_unique<RoamadomeSerialPort>(serialPort_);

  // Create and register the serial event handler (observer pattern)
  serialEventHandler_ = std::make_unique<SerialEventHandler>(this);
  serialHandler_->registerObserver(serialEventHandler_.get());

  // Open the serial port
  if (!serialHandler_->open()) {
    RCLCPP_ERROR(logger_, "Failed to open serial port: %s", serialPort_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::string baud_command = std::format("#DPSERIALBAUD{}", serialBaud_);
  RCLCPP_INFO(logger_, "Baud Command is %s", baud_command.c_str());

  // Keep baud sweep separate from startup state machine. It may produce responses,
  // but no explicit completion check is required for this step.
  for (size_t idx = 0; idx < sizeof(mSupportedBaudRates) / sizeof(mSupportedBaudRates[0]); idx++) {
    uint32_t target_baud = mSupportedBaudRates[idx];

    if (!serialHandler_->configurePort(target_baud)) {
      RCLCPP_ERROR(logger_, "Failed to configure serial port baud rate: %u", target_baud);
      serialHandler_->close();
      serialHandler_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!serialHandler_->sendCommand(baud_command)) {
      RCLCPP_ERROR(logger_, "Failed to send baud configuration command at baud %u", target_baud);
      serialHandler_->close();
      serialHandler_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read any opportunistic feedback while sweeping baud rates.
    serialHandler_->read();
    nanosleep(&ts, NULL);
  }

  if (!serialHandler_->configurePort(serialBaud_)) {
    RCLCPP_ERROR(logger_, "Failed to configure serial port baud rate: %u", serialBaud_);
    serialHandler_->close();
    serialHandler_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  nanosleep(&ts, NULL);

  if (!runStartupStateMachine(report_ms)) {
    RCLCPP_ERROR(logger_, "Startup state machine failed: %s",
      startupContext_.failure_reason.c_str());
    serialHandler_->close();
    serialHandler_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "RoamadomeControl configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void RoamadomeControl::resetStartupContext(StartupContext * context, uint32_t timeout_ms)
{
  context->state = StartupState::SEND_COMMAND;
  context->current_command.reset();
  context->formatted_command.clear();
  context->state_label.clear();
  context->state_start = std::chrono::steady_clock::now();
  context->timeout_ms = timeout_ms;
  context->retries_used = 0;
  context->probe_process_seen = false;
  context->probe_invalid_seen = false;
  context->status_received = false;
  context->config_received = false;
  context->auto_safety_engaged = true;
  context->failure_reason.clear();
  setupGoodMaxSpeed_.reset();
}

void RoamadomeControl::initializeStartupCommandTable()
{
  startupCommandTable_[0] = StartupCommandInfo{
    .id = StartupCommandId::STATUS,
    .command_template = "#DPSTATUS",
    .timeout_ms = startupDefaultTimeoutMs_,
    .transition_rule = StartupTransitionRule::STATUS_AUTO_SAFETY_BRANCH,
    .linear_next = std::nullopt
  };

  startupCommandTable_[1] = StartupCommandInfo{
    .id = StartupCommandId::SETUP,
    .command_template = "#DPSETUP",
    .timeout_ms = startupSetupTimeoutMs_,
    .transition_rule = StartupTransitionRule::LINEAR_NEXT,
    .linear_next = StartupCommandId::CONFIG
  };

  startupCommandTable_[2] = StartupCommandInfo{
    .id = StartupCommandId::CONFIG,
    .command_template = "#DPCONFIG",
    .timeout_ms = startupDefaultTimeoutMs_,
    .transition_rule = StartupTransitionRule::LINEAR_NEXT,
    .linear_next = StartupCommandId::REPORT
  };

  startupCommandTable_[3] = StartupCommandInfo{
    .id = StartupCommandId::REPORT,
    .command_template = "#DPREPORT{}",
    .timeout_ms = startupReportTimeoutMs_,
    .transition_rule = StartupTransitionRule::COMPLETE,
    .linear_next = std::nullopt
  };

  startupCommandTableInitialized_ = true;
}

const RoamadomeControl::StartupCommandInfo * RoamadomeControl::findStartupCommandInfo(
  StartupCommandId command_id) const
{
  for (const auto & command_info : startupCommandTable_) {
    if (command_info.id == command_id) {
      return &command_info;
    }
  }

  return nullptr;
}

std::string RoamadomeControl::formatStartupCommand(StartupCommandId command_id, uint32_t report_ms)
const
{
  const StartupCommandInfo * command_info = findStartupCommandInfo(command_id);
  if (nullptr == command_info) {
    return "";
  }

  if (StartupCommandId::REPORT == command_id) {
    return std::format("#DPREPORT{}", report_ms);
  }

  return command_info->command_template;
}

bool RoamadomeControl::sendStartupCommandWithProbe(
  StartupContext * context,
  StartupCommandId command_id,
  uint32_t report_ms)
{
  const StartupCommandInfo * command_info = findStartupCommandInfo(command_id);
  if (nullptr == command_info) {
    context->failure_reason = "startup command metadata missing";
    context->state = StartupState::FAILED;
    return false;
  }

  context->current_command = command_id;
  context->formatted_command = formatStartupCommand(command_id, report_ms);
  context->state_label = context->formatted_command;
  context->probe_process_seen = false;
  context->probe_invalid_seen = false;

  if (!serialHandler_->sendCommand(context->formatted_command)) {
    context->failure_reason = std::format("failed to send {}", context->state_label);
    context->state = StartupState::FAILED;
    return false;
  }

  if (!serialHandler_->sendCommand("#DPINVALID")) {
    context->failure_reason = std::format(
      "failed to send completion probe for {}", context->state_label);
    context->state = StartupState::FAILED;
    return false;
  }

  transitionStartupState(
    context,
    StartupState::WAIT_PROBE,
    context->state_label,
    command_info->timeout_ms);
  RCLCPP_INFO(logger_, "Startup command sent: %s", context->state_label.c_str());
  return true;
}

std::optional<RoamadomeControl::StartupCommandId> RoamadomeControl::resolveStartupNextCommand(
  const StartupContext & context) const
{
  if (!context.current_command.has_value()) {
    return StartupCommandId::STATUS;
  }

  const StartupCommandInfo * command_info = findStartupCommandInfo(*context.current_command);
  if (nullptr == command_info) {
    return std::nullopt;
  }

  switch (command_info->transition_rule) {
    case StartupTransitionRule::LINEAR_NEXT:
      return command_info->linear_next;

    case StartupTransitionRule::STATUS_AUTO_SAFETY_BRANCH:
      if (context.auto_safety_engaged) {
        return StartupCommandId::SETUP;
      }
      return StartupCommandId::CONFIG;

    case StartupTransitionRule::COMPLETE:
      return std::nullopt;
  }

  return std::nullopt;
}

bool RoamadomeControl::startupProbeComplete(const StartupContext & context) const
{
  return context.probe_process_seen && context.probe_invalid_seen;
}

bool RoamadomeControl::startupStateTimedOut(const StartupContext & context) const
{
  const auto elapsed = std::chrono::steady_clock::now() - context.state_start;
  const auto elapsed_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  return elapsed_ms > context.timeout_ms;
}

void RoamadomeControl::transitionStartupState(
  StartupContext * context,
  StartupState next_state,
  const std::string & label,
  uint32_t timeout_ms)
{
  context->state = next_state;
  context->state_label = label;
  context->timeout_ms = timeout_ms;
  context->state_start = std::chrono::steady_clock::now();
}

bool RoamadomeControl::retryCurrentStartupCommand(StartupContext * context)
{
  if (context->retries_used >= startupMaxRetries_) {
    context->failure_reason = std::format(
      "startup command '{}' timed out after {} retries",
      context->state_label,
      context->retries_used);
    context->state = StartupState::FAILED;
    return false;
  }

  context->retries_used++;
  context->probe_process_seen = false;
  context->probe_invalid_seen = false;
  context->state_start = std::chrono::steady_clock::now();

  RCLCPP_WARN(
    logger_,
    "Startup command '%s' timed out, retry %u/%u",
    context->state_label.c_str(),
    context->retries_used,
    startupMaxRetries_);

  if (!serialHandler_->sendCommand(context->formatted_command)) {
    context->failure_reason = std::format("retry send failed for '{}'", context->state_label);
    context->state = StartupState::FAILED;
    return false;
  }

  if (!serialHandler_->sendCommand("#DPINVALID")) {
    context->failure_reason = std::format("retry probe failed for '{}'", context->state_label);
    context->state = StartupState::FAILED;
    return false;
  }

  return true;
}

bool RoamadomeControl::runStartupStateMachine(uint32_t report_ms)
{
  if (!startupCommandTableInitialized_) {
    initializeStartupCommandTable();
  }

  resetStartupContext(&startupContext_, startupDefaultTimeoutMs_);

  while (startupContext_.state != StartupState::COMPLETE &&
    startupContext_.state != StartupState::FAILED)
  {
    if (!serialHandler_->read()) {
      startupContext_.failure_reason = "serial read failed during startup";
      startupContext_.state = StartupState::FAILED;
      break;
    }

    switch (startupContext_.state) {
      case StartupState::SEND_COMMAND: {
          std::optional<StartupCommandId> next_command = resolveStartupNextCommand(startupContext_);
          if (!next_command.has_value()) {
            if (startupContext_.current_command.has_value() &&
              *startupContext_.current_command == StartupCommandId::REPORT)
            {
              startupContext_.state = StartupState::COMPLETE;
              break;
            }

            startupContext_.failure_reason = "unable to resolve next startup command";
            startupContext_.state = StartupState::FAILED;
            break;
          }

          startupContext_.retries_used = 0;
          if (!sendStartupCommandWithProbe(&startupContext_, *next_command, report_ms)) {
            break;
          }
          break;
        }

      case StartupState::WAIT_PROBE:
        if (startupProbeComplete(startupContext_)) {
          transitionStartupState(
            &startupContext_,
            StartupState::SEND_COMMAND,
            "",
            startupDefaultTimeoutMs_);
        } else if (startupStateTimedOut(startupContext_)) {
          retryCurrentStartupCommand(&startupContext_);
        }
        break;

      case StartupState::COMPLETE:
      case StartupState::FAILED:
        break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(startupLoopSleepMs_));
  }

  return startupContext_.state == StartupState::COMPLETE;
}

void RoamadomeControl::handleStartupUnhandledLine(const std::string & line)
{
  if (line.find("PROCESS: \"#DPINVALID\"") != std::string::npos) {
    startupContext_.probe_process_seen = true;
    return;
  }

  if ("Invalid" == line) {
    startupContext_.probe_invalid_seen = true;
    return;
  }

  std::string lowered_line = line;
  std::transform(lowered_line.begin(), lowered_line.end(), lowered_line.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (lowered_line.find("auto safety engaged") != std::string::npos) {
    startupContext_.auto_safety_engaged = true;
    return;
  }

  if (lowered_line.find("auto safety disengaged") != std::string::npos) {
    startupContext_.auto_safety_engaged = false;
    return;
  }

  const std::string speed_key = "good max speed:";
  size_t key_pos = lowered_line.find(speed_key);
  if (key_pos != std::string::npos) {
    std::string speed_value = line.substr(key_pos + speed_key.size());
    const size_t first_non_space = speed_value.find_first_not_of(" \t");
    if (first_non_space == std::string::npos) {
      return;
    }

    speed_value.erase(0, first_non_space);
    const size_t last_non_space = speed_value.find_last_not_of(" \t");
    speed_value.erase(last_non_space + 1);

    try {
      int parsed = std::stoi(speed_value);
      if (parsed >= 0) {
        setupGoodMaxSpeed_ = static_cast<uint32_t>(parsed);
        RCLCPP_INFO(logger_, "Startup setup GOOD MAX SPEED captured: %u", *setupGoodMaxSpeed_);
      }
    } catch (const std::exception &) {
      RCLCPP_WARN(logger_, "Failed to parse GOOD MAX SPEED from line: %s", line.c_str());
    }
  }
}

hardware_interface::CallbackReturn RoamadomeControl::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "Activating RoamadomeControl from state: %s",
      previous_state.label().c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoamadomeControl::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  const struct timespec ts = {.tv_sec = 1, .tv_nsec = 0};

  RCLCPP_INFO(logger_, "Deactivating RoamadomeControl from state: %s",
      previous_state.label().c_str());

  if (serialHandler_) {
    serialHandler_->sendCommand("#DPREPORT0");
    nanosleep(&ts, NULL);
    serialHandler_->close();
    serialHandler_.reset();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoamadomeControl::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(joint_name_, "position", &position_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(joint_name_, "velocity", &velocity_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoamadomeControl::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(joint_name_, "position", &cmd_position_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(joint_name_, "velocity", &cmd_velocity_));
  return command_interfaces;
}

hardware_interface::return_type RoamadomeControl::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  (void) time;
  (void) period;

  if (serialHandler_) {
    serialHandler_->read();
  }

  return return_type::OK;
}

hardware_interface::return_type RoamadomeControl::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  (void) time;
  (void) period;

  if (!serialHandler_ || !serialHandler_->isOpen()) {
    return return_type::OK;
  }

  // Determine which command mode is active and send appropriate command
  if (currentMode_ == CommandMode::POSITION) {
    // Position command: convert radians to degrees and normalize to [0, 359]
    uint32_t target_degrees = normalizeAngleDegrees(cmd_position_);

    // Only send if position changed significantly (avoid redundant commands)
    if (lastSentPosition_ < 0 || std::fabs(cmd_position_ - lastSentPosition_) > 0.01) {
      if (sendPositionCommand(target_degrees)) {
        lastSentPosition_ = cmd_position_;
        RCLCPP_DEBUG(logger_, "Sent position command: %u°", target_degrees);
      }
    }
  } else if (currentMode_ == CommandMode::VELOCITY) {
    // Velocity command: convert rad/s to percentage [-100, 100]
    int32_t target_percentage = velocityToPercentage(cmd_velocity_);

    // Only send if velocity changed significantly (avoid redundant commands)
    if (std::fabs(cmd_velocity_ - lastSentVelocity_) > 0.01) {
      if (sendVelocityCommand(target_percentage)) {
        lastSentVelocity_ = cmd_velocity_;
        RCLCPP_DEBUG(logger_, "Sent velocity command: %d%%", target_percentage);
      }
    }
  }
  // IDLE mode: don't send any commands

  return return_type::OK;
}

hardware_interface::return_type RoamadomeControl::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  (void) stop_interfaces;  // May be used for validation in future

  // Validate that we're not trying to start both position and velocity
  bool position_starting = false;
  bool velocity_starting = false;

  for (const auto & interface : start_interfaces) {
    if (interface.find("position") != std::string::npos) {
      position_starting = true;
    } else if (interface.find("velocity") != std::string::npos) {
      velocity_starting = true;
    }
  }

  // Ensure mutual exclusivity: can't have both position and velocity starting
  if (position_starting && velocity_starting) {
    RCLCPP_ERROR(logger_, "Cannot switch to both position and velocity modes simultaneously");
    return return_type::ERROR;
  }

  RCLCPP_DEBUG(logger_, "Preparing command mode switch - Position: %d, Velocity: %d",
               position_starting, velocity_starting);

  return return_type::OK;
}

hardware_interface::return_type RoamadomeControl::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  (void) stop_interfaces;  // May be used for validation in future

  // Determine the new mode based on starting interfaces
  CommandMode new_mode = CommandMode::IDLE;

  for (const auto & interface : start_interfaces) {
    if (interface.find("position") != std::string::npos) {
      new_mode = CommandMode::POSITION;
      break;
    } else if (interface.find("velocity") != std::string::npos) {
      new_mode = CommandMode::VELOCITY;
      break;
    }
  }

  // If switching away from an active mode, send stop command
  if (currentMode_ != CommandMode::IDLE && new_mode != currentMode_) {
    RCLCPP_INFO(logger_, "Mode switch detected, sending stop command");
    sendStopCommand();
  }

  previousMode_ = currentMode_;
  currentMode_ = new_mode;

  RCLCPP_INFO(logger_, "Command mode switched to: %s",
              currentMode_ == CommandMode::POSITION ? "POSITION" :
              currentMode_ == CommandMode::VELOCITY ? "VELOCITY" : "IDLE");

  return return_type::OK;
}

uint32_t RoamadomeControl::normalizeAngleDegrees(double radians) const
{
  // Convert radians to degrees
  double degrees = radiansToDegrees(radians);

  // Normalize to [0, 359] using modulo arithmetic
  // First convert to integer to avoid floating point issues
  int32_t deg_int = static_cast<int32_t>(degrees);

  // Apply modulo to handle negative angles and > 359
  deg_int = deg_int % 360;
  if (deg_int < 0) {
    deg_int += 360;
  }

  return static_cast<uint32_t>(deg_int);
}

double RoamadomeControl::radiansToDegrees(double radians) const
{
  return radians * (180.0 / M_PI);
}

int32_t RoamadomeControl::velocityToPercentage(double rad_per_sec) const
{
  // Scale rad/s to [-100, 100] percentage based on maxSpeedRadPerSec_
  double percentage = (rad_per_sec / maxSpeedRadPerSec_) * 100.0;

  // Clamp to valid range
  if (percentage > 100.0) {
    percentage = 100.0;
  } else if (percentage < -100.0) {
    percentage = -100.0;
  }

  return static_cast<int32_t>(std::round(percentage));
}

bool RoamadomeControl::sendPositionCommand(uint32_t degrees)
{
  if (!serialHandler_ || !serialHandler_->isOpen()) {
    RCLCPP_WARN(logger_, "Serial handler not available for position command");
    return false;
  }

  // Format: :DPA<degrees> where degrees is 0-359
  std::string command = std::format(":DPA{}", degrees);
  return serialHandler_->sendCommand(command);
}

bool RoamadomeControl::sendVelocityCommand(int32_t percentage)
{
  if (!serialHandler_ || !serialHandler_->isOpen()) {
    RCLCPP_WARN(logger_, "Serial handler not available for velocity command");
    return false;
  }

  // Format: :DPR<speed> where speed is -100 to 100
  std::string command = std::format(":DPR{}", percentage);
  return serialHandler_->sendCommand(command);
}

bool RoamadomeControl::sendStopCommand()
{
  if (!serialHandler_ || !serialHandler_->isOpen()) {
    RCLCPP_WARN(logger_, "Serial handler not available for stop command");
    return false;
  }

  // Send stop command: :DPR0 (0% velocity)
  return serialHandler_->sendCommand(":DPR0");
}

}  // namespace roamadome_control

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_roamadome::RoamadomeControl,
  hardware_interface::ActuatorInterface)
