#include "ros2_roamadome/roamadome_control.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>  // C++17 replacement for std::format (C++20)
#include <string>
#include <string_view>
#include <thread>
#include <time.h>

namespace ros2_roamadome
{

namespace
{

bool matchesInterfaceName(const std::string & interface_name, std::string_view expected_name)
{
  const size_t separator_pos = interface_name.rfind('/');
  const std::string_view candidate =
    (std::string::npos == separator_pos) ?
    std::string_view(interface_name) :
    std::string_view(interface_name).substr(separator_pos + 1);
  return candidate == expected_name;
}

/// @brief Constructs a command string by concatenating prefix and value
/// @tparam TValue Value type that supports stream insertion (uint32_t, int32_t, etc.)
/// @param prefix Command prefix string (e.g., "#DPSERIALBAUD", ":DPA")
/// @param value Numeric value to append
/// @return Formatted command string
template<typename TValue>
std::string buildCommand(const char * prefix, TValue value)
{
  std::ostringstream stream;
  stream << prefix << value;
  return stream.str();
}

/// @brief Builds a failure reason message with required prefix, label, and optional suffix
/// @param prefix Text before label (e.g., "failed to send ")
/// @param label Context label (e.g., command name)
/// @param suffix Text after label (e.g., "")
/// @return Formatted failure message
std::string buildSingleLabelFailure(
  const char * prefix,
  const std::string & label,
  const char * suffix)
{
  std::ostringstream stream;
  stream << prefix << label << suffix;
  return stream.str();
}

/// @brief Builds a retry limit exceeded failure message
/// @param label Command label that timed out
/// @param retries Number of retries attempted
/// @return Formatted failure message
std::string buildRetryLimitFailure(const std::string & label, uint32_t retries)
{
  std::ostringstream stream;
  stream << "startup command '" << label << "' timed out after " << retries << " retries";
  return stream.str();
}

/// @brief Builds an exception message from a startup transition lambda
/// @param reason Exception message from the lambda
/// @return Formatted failure message
std::string buildTransitionLambdaFailure(const std::string & reason)
{
  std::ostringstream stream;
  stream << "startup transition lambda failed: " << reason;
  return stream.str();
}

}  // namespace

RoamadomeControl::RoamadomeControl()
: logger_(rclcpp::get_logger("RoamadomeControl")), position_(0.0), velocity_(0.0),
  cmd_position_(0.0), cmd_velocity_(0.0),
  currentMode_(CommandMode::IDLE), previousMode_(CommandMode::IDLE),
  lastSentPosition_(-1.0), lastSentVelocity_(0.0)
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

  // Parse optional auto_mode parameter (default: false)
  try {
    auto it = info_.hardware_parameters.find("auto_mode");
    if (it != info_.hardware_parameters.end()) {
      std::string value_lower = it->second;
      std::transform(value_lower.begin(), value_lower.end(), value_lower.begin(),
        [](unsigned char c) {return static_cast<char>(std::tolower(c));});
      autoModeEnabled_ = (value_lower == "true" || value_lower == "1" || value_lower == "yes");
      RCLCPP_INFO(logger_, "auto_mode configured: %s", autoModeEnabled_ ? "enabled" : "disabled");
    } else {
      RCLCPP_INFO(logger_, "auto_mode not specified, defaulting to disabled");
      autoModeEnabled_ = false;
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger_, "Failed to parse auto_mode parameter, using default (false): %s",
      ex.what());
    autoModeEnabled_ = false;
  }

  // Parse optional home_mode parameter (default: false)
  try {
    auto it = info_.hardware_parameters.find("home_mode");
    if (it != info_.hardware_parameters.end()) {
      std::string value_lower = it->second;
      std::transform(value_lower.begin(), value_lower.end(), value_lower.begin(),
        [](unsigned char c) {return static_cast<char>(std::tolower(c));});
      homeModeEnabled_ = (value_lower == "true" || value_lower == "1" || value_lower == "yes");
      RCLCPP_INFO(logger_, "home_mode configured: %s", homeModeEnabled_ ? "enabled" : "disabled");
    } else {
      RCLCPP_INFO(logger_, "home_mode not specified, defaulting to disabled");
      homeModeEnabled_ = false;
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger_, "Failed to parse home_mode parameter, using default (false): %s",
      ex.what());
    homeModeEnabled_ = false;
  }

  // Parse optional startup_log_level parameter (default: "debug")
  try {
    auto it = info_.hardware_parameters.find("startup_log_level");
    if (it != info_.hardware_parameters.end()) {
      std::string value_lower = it->second;
      std::transform(value_lower.begin(), value_lower.end(), value_lower.begin(),
        [](unsigned char c) {return static_cast<char>(std::tolower(c));});
      if (value_lower == "debug" || value_lower == "info") {
        startupLogLevel_ = value_lower;
        RCLCPP_INFO(logger_, "startup_log_level configured: %s", startupLogLevel_.c_str());
      } else {
        RCLCPP_WARN(logger_,
            "Invalid startup_log_level '%s', must be 'debug' or 'info'. Using default 'debug'",
          it->second.c_str());
        startupLogLevel_ = "debug";
      }
    } else {
      RCLCPP_INFO(logger_, "startup_log_level not specified, defaulting to 'debug'");
      startupLogLevel_ = "debug";
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger_, "Failed to parse startup_log_level parameter, using default (debug): %s",
      ex.what());
    startupLogLevel_ = "debug";
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

    auto stale_warning_it = info_.hardware_parameters.find("startup_config_stale_warning_ms");
    if (stale_warning_it != info_.hardware_parameters.end()) {
      configStaleWarningMs_ = static_cast<uint32_t>(std::stoul(stale_warning_it->second));
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
  struct timespec ts = {};
  ts.tv_sec = 0;
  ts.tv_nsec = 500000000;
  if (0 == info_.rw_rate) {
    RCLCPP_ERROR(logger_, "Invalid rw_rate: 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const uint32_t cycle_ms = std::max<uint32_t>(1U, 1000U / info_.rw_rate);
  const uint32_t report_ms = (cycle_ms > 5U) ? (cycle_ms - 5U) : 1U;

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

  const std::string baud_command = buildCommand("#DPSERIALBAUD", serialBaud_);
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
  context->report_ms = 0;
  context->probe_process_seen = false;
  context->probe_invalid_seen = false;
  context->status_received = false;
  context->config_received = false;
  context->auto_safety_engaged = true;
  context->failure_reason.clear();
  context->config_map.clear();
  context->config_timestamp = std::chrono::steady_clock::time_point();
  context->initial_config_read = false;
  context->config_verify_read = false;
  context->autosafety_valid = false;
  setupGoodMaxSpeed_.reset();
}

void RoamadomeControl::initializeStartupCommandTable()
{
  // Compile-time safety check: ensure array size matches command enum cardinality
  static_assert(
    static_cast<int>(StartupCommandId::REPORT) + 1 == 8,
    "startupCommandTable_ size (8) must match highest StartupCommandId + 1; "
    "update array size if adding new StartupCommandId entries"
  );

  // Command 0: CONFIG_INITIAL - Read initial configuration
  StartupCommandInfo config_initial_command;
  config_initial_command.id = StartupCommandId::CONFIG_INITIAL;
  config_initial_command.command_template = "#DPCONFIG";
  config_initial_command.timeout_ms = startupDefaultTimeoutMs_;
  config_initial_command.is_terminal_command = false;
  config_initial_command.linear_next = std::nullopt;
  config_initial_command.command_formatter = std::nullopt;
  config_initial_command.transition_lambda = [](const StartupContext & ctx,
    RoamadomeControl * ctrl) -> std::optional<StartupCommandId> {
      std::lock_guard<std::mutex> lock(ctrl->startupContext_mutex_);
      // Check if AutoSafety is disabled (value should be "0")
      auto it = ctx.config_map.find("AutoSafety");
      if (it != ctx.config_map.end() && it->second == "0") {
        return StartupCommandId::STATUS;
      }
      // AutoSafety is enabled or not found, need to run SETUP
      return StartupCommandId::SETUP;
    };
  startupCommandTable_[0] = config_initial_command;

  // Command 1: SETUP - Run setup procedure
  StartupCommandInfo setup_command;
  setup_command.id = StartupCommandId::SETUP;
  setup_command.command_template = "#DPSETUP";
  setup_command.timeout_ms = startupSetupTimeoutMs_;
  setup_command.is_terminal_command = false;
  setup_command.linear_next = StartupCommandId::AUTOSAFETY0;
  setup_command.command_formatter = std::nullopt;
  setup_command.transition_lambda = std::nullopt;
  startupCommandTable_[1] = setup_command;

  // Command 2: AUTOSAFETY0 - Disable auto safety
  StartupCommandInfo autosafety0_command;
  autosafety0_command.id = StartupCommandId::AUTOSAFETY0;
  autosafety0_command.command_template = "#DPAUTOSAFETY0";
  autosafety0_command.timeout_ms = startupDefaultTimeoutMs_;
  autosafety0_command.is_terminal_command = false;
  autosafety0_command.linear_next = StartupCommandId::VERIFY_AUTOSAFETY;
  autosafety0_command.command_formatter = std::nullopt;
  autosafety0_command.transition_lambda = std::nullopt;
  startupCommandTable_[2] = autosafety0_command;

  // Command 3: VERIFY_AUTOSAFETY - Re-read config to verify AutoSafety is now 0
  StartupCommandInfo verify_autosafety_command;
  verify_autosafety_command.id = StartupCommandId::VERIFY_AUTOSAFETY;
  verify_autosafety_command.command_template = "#DPCONFIG";
  verify_autosafety_command.timeout_ms = startupDefaultTimeoutMs_;
  verify_autosafety_command.is_terminal_command = false;
  verify_autosafety_command.linear_next = std::nullopt;
  verify_autosafety_command.command_formatter = std::nullopt;
  verify_autosafety_command.transition_lambda = [](const StartupContext & ctx,
    RoamadomeControl * ctrl) -> std::optional<StartupCommandId> {
      std::lock_guard<std::mutex> lock(ctrl->startupContext_mutex_);
      // Verify AutoSafety is now disabled
      auto it = ctx.config_map.find("AutoSafety");
      if (it != ctx.config_map.end() && it->second == "0") {
        return StartupCommandId::STATUS;
      }
      // AutoSafety is still enabled - this is a failure
      throw std::runtime_error(
        "AutoSafety verification failed: expected 0, got " +
              (it != ctx.config_map.end() ? it->second : "not found"));
    };
  startupCommandTable_[3] = verify_autosafety_command;

  // Command 4: STATUS - Verify status shows auto safety disabled
  StartupCommandInfo status_command;
  status_command.id = StartupCommandId::STATUS;
  status_command.command_template = "#DPSTATUS";
  status_command.timeout_ms = startupDefaultTimeoutMs_;
  status_command.is_terminal_command = false;
  status_command.linear_next = std::nullopt;
  status_command.command_formatter = std::nullopt;
  status_command.transition_lambda = [](const StartupContext & ctx,
    RoamadomeControl * ctrl) -> std::optional<StartupCommandId> {
      // Acquire lock for reading config snapshot
      std::lock_guard<std::mutex> lock(ctrl->startupContext_mutex_);

      // Check config age and warn if stale
      auto config_age = std::chrono::steady_clock::now() - ctx.config_timestamp;
      auto config_age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(config_age).count();
      if (config_age_ms > ctrl->configStaleWarningMs_) {
        RCLCPP_WARN(ctrl->logger_,
          "Configuration data is %lld ms old (threshold: %u ms)",
          static_cast<long long>(config_age_ms), ctrl->configStaleWarningMs_);
      }

      // Check if AutoMode matches desired state
      auto auto_mode_it = ctx.config_map.find("AutoMode");
      std::string expected_auto_mode = ctrl->autoModeEnabled_ ? "1" : "0";
      if (auto_mode_it == ctx.config_map.end() ||
        auto_mode_it->second != expected_auto_mode)
      {
        return StartupCommandId::SET_AUTOMODE;
      }

      // Check if HomeMode matches desired state
      auto home_mode_it = ctx.config_map.find("HomeMode");
      std::string expected_home_mode = ctrl->homeModeEnabled_ ? "1" : "0";
      if (home_mode_it == ctx.config_map.end() ||
        home_mode_it->second != expected_home_mode)
      {
        return StartupCommandId::SET_HOMEMODE;
      }

      // Both modes match, proceed to REPORT
      return StartupCommandId::REPORT;
    };
  startupCommandTable_[4] = status_command;

  // Command 5: SET_AUTOMODE - Configure AutoMode
  StartupCommandInfo set_automode_command;
  set_automode_command.id = StartupCommandId::SET_AUTOMODE;
  set_automode_command.command_template = "#DPAUTO{}";
  set_automode_command.timeout_ms = startupDefaultTimeoutMs_;
  set_automode_command.is_terminal_command = false;
  set_automode_command.linear_next = std::nullopt;
  set_automode_command.command_formatter =
    [](const StartupContext & ctx, RoamadomeControl * ctrl) -> std::string {
      (void)ctx;
      return buildCommand("#DPAUTO", ctrl->autoModeEnabled_ ? 1 : 0);
    };
  set_automode_command.transition_lambda = [](const StartupContext & ctx,
    RoamadomeControl * ctrl) -> std::optional<StartupCommandId> {
      std::lock_guard<std::mutex> lock(ctrl->startupContext_mutex_);
      // Check if HomeMode matches desired state
      auto home_mode_it = ctx.config_map.find("HomeMode");
      std::string expected_home_mode = ctrl->homeModeEnabled_ ? "1" : "0";
      if (home_mode_it == ctx.config_map.end() ||
        home_mode_it->second != expected_home_mode)
      {
        return StartupCommandId::SET_HOMEMODE;
      }
      // HomeMode matches, proceed to REPORT
      return StartupCommandId::REPORT;
    };
  startupCommandTable_[5] = set_automode_command;

  // Command 6: SET_HOMEMODE - Configure HomeMode
  StartupCommandInfo set_homemode_command;
  set_homemode_command.id = StartupCommandId::SET_HOMEMODE;
  set_homemode_command.command_template = "#DPHOME{}";
  set_homemode_command.timeout_ms = startupDefaultTimeoutMs_;
  set_homemode_command.is_terminal_command = false;
  set_homemode_command.linear_next = StartupCommandId::REPORT;
  set_homemode_command.command_formatter =
    [](const StartupContext & ctx, RoamadomeControl * ctrl) -> std::string {
      (void)ctx;
      return buildCommand("#DPHOME", ctrl->homeModeEnabled_ ? 1 : 0);
    };
  set_homemode_command.transition_lambda = std::nullopt;
  startupCommandTable_[6] = set_homemode_command;

  // Command 7: REPORT - Enable periodic reporting (terminal command)
  StartupCommandInfo report_command;
  report_command.id = StartupCommandId::REPORT;
  report_command.command_template = "#DPREPORT{}";
  report_command.timeout_ms = startupReportTimeoutMs_;
  report_command.is_terminal_command = true;
  report_command.linear_next = std::nullopt;
  report_command.command_formatter =
    [](const StartupContext & ctx, RoamadomeControl * ctrl) -> std::string {
      (void)ctrl;
      return buildCommand("#DPREPORT", ctx.report_ms);
    };
  report_command.transition_lambda = std::nullopt;
  startupCommandTable_[7] = report_command;

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

std::string RoamadomeControl::formatStartupCommand(
  StartupCommandId command_id,
  const StartupContext & context) const
{
  const StartupCommandInfo * command_info = findStartupCommandInfo(command_id);
  if (nullptr == command_info) {
    return "";
  }

  if (command_info->command_formatter) {
    return (*command_info->command_formatter)(context, const_cast<RoamadomeControl *>(this));
  }

  return command_info->command_template;
}

bool RoamadomeControl::sendStartupCommandWithProbe(
  StartupContext * context,
  StartupCommandId command_id)
{
  const StartupCommandInfo * command_info = findStartupCommandInfo(command_id);
  if (nullptr == command_info) {
    context->failure_reason = "startup command metadata missing";
    context->state = StartupState::FAILED;
    return false;
  }

  context->current_command = command_id;
  context->formatted_command = formatStartupCommand(command_id, *context);
  context->state_label = context->formatted_command;
  context->probe_process_seen = false;
  context->probe_invalid_seen = false;

  if (!serialHandler_->sendCommand(context->formatted_command)) {
    context->failure_reason = buildSingleLabelFailure("failed to send ", context->state_label, "");
    context->state = StartupState::FAILED;
    return false;
  }

  if (!serialHandler_->sendCommand("#DPINVALID")) {
    context->failure_reason = buildSingleLabelFailure(
      "failed to send completion probe for ", context->state_label, "");
    context->state = StartupState::FAILED;
    return false;
  }

  transitionStartupState(
    context,
    StartupState::WAIT_PROBE,
    context->state_label,
    command_info->timeout_ms);

  std::ostringstream log_msg;
  log_msg << "Startup command sent: " << context->state_label;
  logStartupTransition(log_msg.str());
  return true;
}

std::optional<RoamadomeControl::StartupCommandId> RoamadomeControl::resolveStartupNextCommand(
  const StartupContext & context) const
{
  if (!context.current_command.has_value()) {
    return StartupCommandId::CONFIG_INITIAL;
  }

  const StartupCommandInfo * command_info = findStartupCommandInfo(*context.current_command);
  if (nullptr == command_info) {
    return std::nullopt;
  }

  // Try transition lambda first if available
  if (command_info->transition_lambda) {
    return (*command_info->transition_lambda)(context, const_cast<RoamadomeControl *>(this));
  }

  // Fall back to linear_next
  return command_info->linear_next;
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
    context->failure_reason = buildRetryLimitFailure(context->state_label, context->retries_used);
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
    context->failure_reason = buildSingleLabelFailure(
      "retry send failed for '", context->state_label, "'");
    context->state = StartupState::FAILED;
    return false;
  }

  if (!serialHandler_->sendCommand("#DPINVALID")) {
    context->failure_reason = buildSingleLabelFailure(
      "retry probe failed for '", context->state_label, "'");
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
  startupContext_.report_ms = report_ms;

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
          std::optional<StartupCommandId> next_command;

          try {
            next_command = resolveStartupNextCommand(startupContext_);
          } catch (const std::exception & ex) {
            startupContext_.failure_reason = buildTransitionLambdaFailure(ex.what());
            startupContext_.state = StartupState::FAILED;
            RCLCPP_ERROR(logger_, "Startup transition lambda exception: %s", ex.what());
            break;
          }

          if (!next_command.has_value()) {
            // If the current command is terminal, transition to COMPLETE
            if (startupContext_.current_command.has_value()) {
              const StartupCommandInfo * current_cmd_info =
                findStartupCommandInfo(*startupContext_.current_command);
              if (nullptr != current_cmd_info && current_cmd_info->is_terminal_command) {
                startupContext_.state = StartupState::COMPLETE;
                break;
              }
            }

            startupContext_.failure_reason = "unable to resolve next startup command";
            startupContext_.state = StartupState::FAILED;
            break;
          }

          startupContext_.retries_used = 0;
          if (!sendStartupCommandWithProbe(&startupContext_, *next_command)) {
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

void RoamadomeControl::logStartupTransition(const std::string & message)
{
  if ("info" == startupLogLevel_) {
    RCLCPP_INFO(logger_, "%s", message.c_str());
  } else {
    RCLCPP_DEBUG(logger_, "%s", message.c_str());
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
  struct timespec ts = {};
  ts.tv_sec = 1;
  ts.tv_nsec = 0;

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
    if (true != serialHandler_->read()) {
      // Handle read failure if necessary (e.g., log, set error state, etc.)
      RCLCPP_ERROR(logger_, "Serial read failed during read()");
      return return_type::ERROR;
    }
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
    if (matchesInterfaceName(interface, "position")) {
      position_starting = true;
    } else if (matchesInterfaceName(interface, "velocity")) {
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
    if (matchesInterfaceName(interface, "position")) {
      new_mode = CommandMode::POSITION;
      break;
    } else if (matchesInterfaceName(interface, "velocity")) {
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

  // Normalize to [0, 359] using modulo arithmetic with floor for correct negative handling
  // std::floor ensures negative angles wrap correctly (e.g., -0.5° → -1 → 359)
  int32_t deg_int = static_cast<int32_t>(std::floor(degrees));

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
  std::string command = buildCommand(":DPA", degrees);
  return serialHandler_->sendCommand(command);
}

bool RoamadomeControl::sendVelocityCommand(int32_t percentage)
{
  if (!serialHandler_ || !serialHandler_->isOpen()) {
    RCLCPP_WARN(logger_, "Serial handler not available for velocity command");
    return false;
  }

  // Format: :DPR<speed> where speed is -100 to 100
  std::string command = buildCommand(":DPR", percentage);
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

}  // namespace ros2_roamadome

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_roamadome::RoamadomeControl,
  hardware_interface::ActuatorInterface)
