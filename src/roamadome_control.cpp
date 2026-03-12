#include "ros2_roamadome/roamadome_control.hpp"
#include "ros2_roamadome/device_parameter_specs.hpp"
#include "ros2_roamadome/parameter_parser.hpp"
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
#include <limits>
#include <unordered_map>
#include <vector>

namespace ros2_roamadome
{

namespace
{

bool parameterValuesMatch(
  const rclcpp::Parameter & current_parameter,
  const rclcpp::Parameter & desired_parameter)
{
  return current_parameter.get_type() == desired_parameter.get_type() &&
         current_parameter.get_parameter_value() == desired_parameter.get_parameter_value();
}

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

bool & parameterSyncCallbackBypass()
{
  static thread_local bool bypass = false;
  return bypass;
}

class ScopedParameterSyncCallbackBypass
{
public:
  ScopedParameterSyncCallbackBypass()
  {
    parameterSyncCallbackBypass() = true;
  }

  ~ScopedParameterSyncCallbackBypass()
  {
    parameterSyncCallbackBypass() = false;
  }
};

}  // namespace

RoamadomeControl::RoamadomeControl()
: logger_(rclcpp::get_logger("RoamadomeControl")), position_(0.0), velocity_(0.0),
  cmd_position_(0.0), cmd_velocity_(0.0),
  currentMode_(CommandMode::IDLE),
  lastSentPosition_(-1.0), lastSentVelocity_(0.0),
  configRefreshIntervalMs_(60000)
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

  const auto & parameters = info_.hardware_parameters;

  if (!parseRequiredStringParameter(parameters, logger_, "serial_port", &serialPort_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto baud_validator = [this](uint32_t baud) {
      return std::any_of(
        std::begin(mSupportedBaudRates),
        std::end(mSupportedBaudRates),
        [baud](uint32_t supported_baud) {return baud == supported_baud;});
    };
  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "serial_baud",
      115200,
      baud_validator,
      "one of {2400, 9600, 19200, 38400, 115200}",
      &serialBaud_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalDoubleParameter(
      parameters,
      logger_,
      "max_speed_rad_per_sec",
      3.14,
      [](double value) {return value > 0.0;},
      "> 0",
      &maxSpeedRadPerSec_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalBoolParameter(parameters, logger_, "auto_mode", false, &autoModeEnabled_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalBoolParameter(parameters, logger_, "home_mode", false, &homeModeEnabled_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalEnumParameter(
      parameters,
      logger_,
      "startup_log_level",
      "debug",
      {"debug", "info"},
      &startupLogLevel_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "startup_default_timeout_ms",
      1000,
      [](uint32_t value) {return value > 0;},
      "> 0",
      &startupDefaultTimeoutMs_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "startup_report_timeout_ms",
      startupDefaultTimeoutMs_,
      [](uint32_t value) {return value > 0;},
      "> 0",
      &startupReportTimeoutMs_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "startup_setup_timeout_ms",
      10000,
      [](uint32_t value) {return value > 0;},
      "> 0",
      &startupSetupTimeoutMs_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "startup_retries",
      1,
      nullptr,
      "",
      &startupMaxRetries_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "baud_sweep_sleep_ms",
      500,
      nullptr,
      "",
      &baudSweepSleepMs_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "startup_config_stale_warning_ms",
      30000,
      nullptr,
      "",
      &configStaleWarningMs_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "serial_section_flush_timeout_ms",
      500,
      [](uint32_t value) {return value > 0;},
      "> 0",
      &serialSectionFlushTimeoutMs_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Maximum speed configured: %.4f rad/s (%.2f°/s)",
              maxSpeedRadPerSec_, radiansToDegrees(maxSpeedRadPerSec_));
  RCLCPP_INFO(logger_, "Startup timeouts: default=%u ms setup=%u ms report=%u ms retries=%u",
              startupDefaultTimeoutMs_, startupSetupTimeoutMs_, startupReportTimeoutMs_,
              startupMaxRetries_);
  RCLCPP_INFO(logger_, "Startup baud sweep settle sleep: %u ms", baudSweepSleepMs_);

  initializeStartupCommandTable();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoamadomeControl::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  if (0 == info_.rw_rate) {
    RCLCPP_ERROR(logger_, "Invalid rw_rate: 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::chrono::milliseconds baud_settle_sleep(baudSweepSleepMs_);

  const uint32_t cycle_ms = std::max<uint32_t>(1U, 1000U / info_.rw_rate);
  const uint32_t report_ms = (cycle_ms > 5U) ? (cycle_ms - 5U) : 1U;

  RCLCPP_INFO(logger_, "Configuring RoamadomeControl from state: %s",
      previous_state.label().c_str());
  RCLCPP_INFO(logger_, "Read Rate in Hz: %u (#DPREPORT%u)", info_.rw_rate, report_ms);

  // Declare parameters before startup reads so config callbacks can safely update values.
  declareDeviceParameters();

  // Create and setup the serial handler
  serialHandler_ = std::make_unique<RoamadomeSerialPort>(serialPort_);
  serialHandler_->setSectionFlushTimeoutMs(serialSectionFlushTimeoutMs_);

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
    std::this_thread::sleep_for(baud_settle_sleep);
  }

  if (!serialHandler_->configurePort(serialBaud_)) {
    RCLCPP_ERROR(logger_, "Failed to configure serial port baud rate: %u", serialBaud_);
    serialHandler_->close();
    serialHandler_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(baud_settle_sleep);

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
  context->auto_safety_engaged = true;
  context->failure_reason.clear();
  context->config_map.clear();
  context->config_timestamp = std::chrono::steady_clock::time_point();
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

      if (ctx.auto_safety_engaged) {
        RCLCPP_ERROR(ctrl->logger_, "AutoSafety is still engaged according to status response");
        throw std::runtime_error("AutoSafety is still engaged according to status response");
      }

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

}

void RoamadomeControl::enqueueConfigDumpWithInvalidLocked(const std::string & reason)
{
  sendQueue_.push({reason + " #DPCONFIG", "#DPCONFIG", SendQueueCommand::Flow::ONE_SHOT, false});
  sendQueue_.push({reason + " #DPINVALID", "#DPINVALID", SendQueueCommand::Flow::ONE_SHOT, false});
}

void RoamadomeControl::handleSendQueueFeedback(const std::string & line)
{
  std::lock_guard<std::mutex> lock(sendQueue_mutex_);
  if (!sendQueueCommandActive_) {
    return;
  }

  if (SendQueueCommand::Flow::PARAMETER_UPDATE_ACK != activeSendQueueCommand_.flow) {
    return;
  }

  if (SendQueueStage::WAIT_WRITE_SETTINGS == sendQueueStage_) {
    if ("Write Settings" == line) {
      sendQueueStage_ = SendQueueStage::WAIT_UPDATED;
      sendQueueStageStartTime_ = std::chrono::steady_clock::now();
      RCLCPP_DEBUG(logger_, "Received 'Write Settings' for %s",
        activeSendQueueCommand_.label.c_str());
      return;
    }

    if ("Updated" == line) {
      RCLCPP_WARN(logger_, "Received 'Updated' before 'Write Settings' for %s",
        activeSendQueueCommand_.label.c_str());
      return;
    }
  }

  if (SendQueueStage::WAIT_UPDATED == sendQueueStage_ && "Updated" == line) {
    RCLCPP_DEBUG(logger_, "Received 'Updated' for %s", activeSendQueueCommand_.label.c_str());
    if (activeSendQueueCommand_.post_ack_config_dump) {
      enqueueConfigDumpWithInvalidLocked("post-update");
    }
    sendQueueCommandActive_ = false;
    sendQueueStage_ = SendQueueStage::IDLE;
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

void RoamadomeControl::ensureParameterCallbackRegistered(const rclcpp::Node::SharedPtr & node)
{
  if (!node || paramCallbackHandle_) {
    return;
  }

  // Register callback as soon as device parameters exist. This guarantees
  // configure->activate writes are validated/rejected deterministically.
  paramCallbackHandle_ = node->add_on_set_parameters_callback(
    std::bind(&RoamadomeControl::onParameterChange, this, std::placeholders::_1));
}

hardware_interface::CallbackReturn RoamadomeControl::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "Activating RoamadomeControl from state: %s",
      previous_state.label().c_str());

  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(logger_, "Cannot activate RoamadomeControl: lifecycle node is null");
    return hardware_interface::CallbackReturn::ERROR;
  }

  ensureParameterCallbackRegistered(node);

  if (!setupService_) {
    setupService_ = node->create_service<std_srvs::srv::Trigger>(
      "~/setup",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>/* request */,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!serialHandler_ || !serialHandler_->isOpen()) {
          response->success = false;
          response->message = "hardware interface is unavailable; command not queued";
          RCLCPP_WARN(logger_, "Setup service called while serial interface is unavailable");
          return;
        }
        std::lock_guard<std::mutex> lock(sendQueue_mutex_);
        sendQueue_.push({"setup", "#DPSETUP", RoamadomeControl::SendQueueCommand::Flow::ONE_SHOT,
          false});
        response->success = true;
        response->message = "Setup command queued";
        RCLCPP_INFO(logger_, "Setup service called, #DPSETUP will be sent in next write cycle");
      });
  }

  if (!periodicConfigTimer_) {
    periodicConfigTimer_ = node->create_wall_timer(
      std::chrono::milliseconds(configRefreshIntervalMs_),
      [this]() {
        std::lock_guard<std::mutex> lock(sendQueue_mutex_);
        enqueueConfigDumpWithInvalidLocked("periodic refresh");
      });
  }

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

  setupService_.reset();
  // Keep the callback registered across deactivate so device.* writes continue
  // to be validated/rejected deterministically while hardware is inactive.

  if (periodicConfigTimer_) {
    periodicConfigTimer_->cancel();
    periodicConfigTimer_.reset();
  }

  // Reset the send queue and related state to ensure a clean slate on next activation.
  {
    std::lock_guard<std::mutex> lock(sendQueue_mutex_);
    while (!sendQueue_.empty()) {
      sendQueue_.pop();
    }
    sendQueueCommandActive_ = false;
    sendQueueStage_ = SendQueueStage::IDLE;
  }

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

  std::optional<SendQueueCommand> queued_command;

  // Process queue bookkeeping under lock.
  {
    std::lock_guard<std::mutex> lock(sendQueue_mutex_);
    const auto now = std::chrono::steady_clock::now();

    if (sendQueueCommandActive_) {
      const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - sendQueueStageStartTime_).count();

      if ((SendQueueStage::WAIT_WRITE_SETTINGS == sendQueueStage_ ||
        SendQueueStage::WAIT_UPDATED == sendQueueStage_) &&
        elapsed_ms > sendQueueAckTimeoutMs_)
      {
        RCLCPP_WARN(logger_,
          "Timed out waiting for ack stage (%d) for %s after %lld ms",
          static_cast<int>(sendQueueStage_), activeSendQueueCommand_.label.c_str(),
          static_cast<long long>(elapsed_ms));
        sendQueueCommandActive_ = false;
        sendQueueStage_ = SendQueueStage::IDLE;
      }
    }

    if (!sendQueueCommandActive_ && !sendQueue_.empty()) {
      queued_command = sendQueue_.front();
      sendQueue_.pop();
    }
  }

  // Execute serial I/O outside sendQueue_mutex_ to avoid stalling queue producers/consumers.
  if (queued_command.has_value()) {
    const SendQueueCommand & command = *queued_command;

    if (SendQueueCommand::Flow::ONE_SHOT == command.flow) {
      if (!serialHandler_->sendCommand(command.command)) {
        RCLCPP_WARN(logger_, "Failed one-shot command '%s' (%s)",
          command.command.c_str(), command.label.c_str());
      } else {
        RCLCPP_DEBUG(logger_, "Sent one-shot command '%s' (%s)",
          command.command.c_str(), command.label.c_str());
      }
    } else if (serialHandler_->sendCommand(command.command)) {
      {
        std::lock_guard<std::mutex> lock(sendQueue_mutex_);
        activeSendQueueCommand_ = command;
        sendQueueCommandActive_ = true;
        sendQueueStage_ = SendQueueStage::WAIT_WRITE_SETTINGS;
        sendQueueStageStartTime_ = std::chrono::steady_clock::now();
      }

      RCLCPP_INFO(logger_, "Sent ack-tracked command '%s' (%s)",
        command.command.c_str(), command.label.c_str());
    } else {
      RCLCPP_WARN(logger_, "Failed ack-tracked command '%s' (%s)",
        command.command.c_str(), command.label.c_str());
    }
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

void RoamadomeControl::declareDeviceParameters()
{
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(logger_, "Failed to get node for parameter declaration");
    return;
  }

  // All device parameters — skip already-declared ones to survive re-configure
  for (const auto * spec : deviceParameterRegistry_.allSpecs()) {
    if (!node->has_parameter(spec->fullName())) {
      spec->declareParameter(node);
    }
  }

  // Register mutability policy callback at parameter declaration time so
  // configure->activate writes are not accepted without validation.
  ensureParameterCallbackRegistered(node);

  RCLCPP_INFO(logger_, "Device parameters declared");
}

std::vector<rclcpp::Parameter> RoamadomeControl::filterChangedParameters(
  const rclcpp::Node::SharedPtr & node,
  const std::vector<rclcpp::Parameter> & desired_parameters)
{
  std::vector<rclcpp::Parameter> changed_parameters;
  changed_parameters.reserve(desired_parameters.size());
  for (const auto & desired_parameter : desired_parameters) {
    rclcpp::Parameter current_parameter;
    if (!node->get_parameter(desired_parameter.get_name(), current_parameter) ||
      !parameterValuesMatch(current_parameter, desired_parameter))
    {
      changed_parameters.push_back(desired_parameter);
    }
  }

  return changed_parameters;
}

void RoamadomeControl::updateParametersFromDevice()
{
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(logger_, "Failed to get node for parameter update");
    return;
  }

  if (!node->has_parameter("device.home_pos")) {
    RCLCPP_DEBUG(logger_, "Device parameters not declared yet; skipping config-to-parameter sync");
    return;
  }

  std::vector<rclcpp::Parameter> params_to_set;
  {
    std::lock_guard<std::mutex> lock(deviceParameterRegistry_mutex_);
    params_to_set = deviceParameterRegistry_.currentParameters();
  }

  const std::vector<rclcpp::Parameter> changed_params = filterChangedParameters(node,
      params_to_set);

  if (changed_params.empty()) {
    RCLCPP_DEBUG(logger_, "Device->ROS parameter sync skipped: no changed values");
    return;
  }

  ScopedParameterSyncCallbackBypass sync_guard;

  auto results = node->set_parameters(changed_params);

  size_t success_count = 0;
  for (const auto & result : results) {
    if (result.successful) {
      success_count++;
    }
  }

  RCLCPP_DEBUG(logger_,
      "Device->ROS parameter sync complete: updated %zu/%zu parameters", success_count,
      results.size());
}

rcl_interfaces::msg::SetParametersResult RoamadomeControl::onParameterChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  if (parameterSyncCallbackBypass()) {
    RCLCPP_DEBUG(logger_,
      "Ignoring %zu parameter callback entries triggered by device->ROS sync",
      parameters.size());
    return result;
  }

  RCLCPP_DEBUG(logger_, "Processing %zu parameter callback entries", parameters.size());

  for (const auto & param : parameters) {
    const std::string & name = param.get_name();

    // Only handle device.* parameters; user writes to non-writable fields are
    // rejected below. Internal sync updates bypass this callback with a
    // thread-local guard around updateParametersFromDevice().
    if (name.find("device.") != 0) {
      continue;  // Not a device parameter, ignore
    }

    // Extract field name
    std::string field_name = name.substr(7);  // Remove "device." prefix
    const WritableParameterSpecBase * spec = deviceParameterRegistry_.findWritableParameterSpec(
      field_name);
    if (nullptr == spec) {
      result.successful = false;
      result.reason = "device parameter " + name + " is read-only and updated from device config";
      return result;
    }

    if (!serialHandler_ || !serialHandler_->isOpen()) {
      result.successful = false;
      result.reason = "cannot update device parameter " + name +
        ": hardware interface is not active";
      return result;
    }

    std::string command;
    std::string reason;
    if (!spec->validateAndBuildCommand(param, &command, &reason)) {
      result.successful = false;
      result.reason = reason;
      return result;
    }

    RCLCPP_DEBUG(logger_, "Validated %s, command=%s", name.c_str(), command.c_str());

    {
      std::lock_guard<std::mutex> lock(sendQueue_mutex_);
      sendQueue_.push({
          name,
          command,
          SendQueueCommand::Flow::PARAMETER_UPDATE_ACK,
          true
        });
      RCLCPP_INFO(logger_, "Queued parameter update: %s (queue depth=%zu)",
        name.c_str(), sendQueue_.size());
    }
  }

  return result;
}

}  // namespace ros2_roamadome

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_roamadome::RoamadomeControl,
  hardware_interface::ActuatorInterface)
