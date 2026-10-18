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
  lastSentPosition_(std::numeric_limits<double>::quiet_NaN()),
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
      [](double value) {return std::isfinite(value) && value > 0.0;},
      "finite and > 0",
      &maxSpeedRadPerSec_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseVelocityMapParameters(parameters)) {
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

  if (!parseOptionalBoolParameter(
      parameters, logger_, "publish_telemetry", false, &publishTelemetry_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalDoubleParameter(
      parameters,
      logger_,
      "velocity_window_s",
      0.5,
      [](double value) {return value > 0.0;},
      "> 0",
      &velocityWindowS_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  positionTracker_ = PositionTracker(velocityWindowS_);

  if (!parseOptionalDoubleParameter(
      parameters,
      logger_,
      "max_plausible_deg_s",
      0.0,
      [](double value) {return value >= 0.0;},
      ">= 0",
      &maxPlausibleDegS_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalBoolParameter(
      parameters, logger_, "serial_baud_sweep", false, &serialBaudSweep_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "config_refresh_interval_ms",
      60000,
      nullptr,
      "",
      &configRefreshIntervalMs_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "report_gap_warn_ms",
      250,
      nullptr,
      "",
      &reportGapWarnMs_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "status_poll_interval_ms",
      10000,
      nullptr,
      "",
      &statusPollIntervalMs_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalBoolParameter(
      parameters, logger_, "enable_bench_mode", false, &enableBenchMode_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "bench_heartbeat_timeout_ms",
      1500,
      [](uint32_t value) {return value > 0;},
      "> 0",
      &benchHeartbeatTimeoutMs_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseOptionalUInt32Parameter(
      parameters,
      logger_,
      "position_move_speed_percent",
      0,
      [](uint32_t value) {return value <= 100;},
      "0..100",
      &positionMoveSpeedPercent_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Bench mode %s; bench heartbeat timeout: %u ms",
              enableBenchMode_ ? "ENABLED (raw ~/bench/drive service)" : "disabled",
              benchHeartbeatTimeoutMs_);
  RCLCPP_INFO(logger_, "Maximum speed configured: %.4f rad/s (%.2f°/s)",
              maxSpeedRadPerSec_, radiansToDegrees(maxSpeedRadPerSec_));
  RCLCPP_INFO(logger_, "Velocity estimate window: %.3f s; telemetry publishing %s",
              velocityWindowS_, publishTelemetry_ ? "enabled" : "disabled");
  RCLCPP_INFO(logger_, "Serial baud sweep at startup: %s; periodic config refresh: %u ms%s",
              serialBaudSweep_ ? "enabled" : "disabled (USB CDC link)",
              configRefreshIntervalMs_, configRefreshIntervalMs_ == 0 ? " (disabled)" : "");
  RCLCPP_INFO(logger_, "Diagnostics status poll interval: %u ms%s",
              statusPollIntervalMs_, statusPollIntervalMs_ == 0 ? " (disabled)" : "");
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
  lifecycleActive_.store(false);

  if (0 == info_.rw_rate) {
    RCLCPP_ERROR(logger_, "Invalid rw_rate: 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring RoamadomeControl from state: %s",
      previous_state.label().c_str());

  // Declare parameters before startup reads so config callbacks can safely update values.
  declareDeviceParameters();

  if (!openDeviceAndRunStartup()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "RoamadomeControl configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RoamadomeControl::openDeviceAndRunStartup()
{
  if (0 == info_.rw_rate) {
    RCLCPP_ERROR(logger_, "Invalid rw_rate: 0");
    return false;
  }

  const std::chrono::milliseconds baud_settle_sleep(baudSweepSleepMs_);

  const uint32_t cycle_ms = std::max<uint32_t>(1U, 1000U / info_.rw_rate);
  const uint32_t report_ms = (cycle_ms > 5U) ? (cycle_ms - 5U) : 1U;

  RCLCPP_INFO(logger_, "Read Rate in Hz: %u (#DPREPORT%u)", info_.rw_rate, report_ms);

  // Reports arrive at the report interval; two read in one cycle are stamped
  // microseconds apart, so the gate never judges a pair over less than that.
  plausibilityGate_ = ReportPlausibilityGate(maxPlausibleDegS_, report_ms / 1000.0);
  if (plausibilityGate_.enabled()) {
    RCLCPP_INFO(logger_,
      "Position report plausibility gate on: a step over %.0f deg/s x dt + 2 deg "
      "(+10 deg across 0) is held until the next report confirms it", maxPlausibleDegS_);
  }

  // Create and setup the serial handler
  serialHandler_ = std::make_unique<RoamadomeSerialPort>(serialPort_);
  serialHandler_->setSectionFlushTimeoutMs(serialSectionFlushTimeoutMs_);

  // Create and register the serial event handler (observer pattern)
  serialEventHandler_ = std::make_unique<SerialEventHandler>(this);
  serialHandler_->registerObserver(serialEventHandler_.get());

  // Open the serial port
  if (!serialHandler_->open()) {
    RCLCPP_ERROR(logger_, "Failed to open serial port: %s", serialPort_.c_str());
    serialHandler_.reset();
    return false;
  }

  if (serialBaudSweep_) {
    // UART link: the firmware may be at any of its supported rates, so tell it
    // the wanted rate at each candidate before settling on it. A USB CDC link
    // ignores baud entirely, so this is off by default (serial_baud_sweep).
    const std::string baud_command = buildCommand("#DPSERIALBAUD", serialBaud_);
    RCLCPP_INFO(logger_, "Baud Command is %s", baud_command.c_str());

    // Keep baud sweep separate from startup state machine. It may produce responses,
    // but no explicit completion check is required for this step.
    const size_t baud_count = sizeof(mSupportedBaudRates) / sizeof(mSupportedBaudRates[0]);
    for (size_t idx = 0; idx < baud_count; idx++) {
      uint32_t target_baud = mSupportedBaudRates[idx];

      if (!serialHandler_->configurePort(target_baud)) {
        RCLCPP_ERROR(logger_, "Failed to configure serial port baud rate: %u", target_baud);
        serialHandler_->close();
        serialHandler_.reset();
        return false;
      }

      for (size_t blankLineIndex = 0; blankLineIndex < 5; blankLineIndex++) {
        // Send out a blank commond to flush out any partial lines in the buffer and elicit responses from the device at the current baud.
        serialHandler_->sendCommand("");
      }

      if (!serialHandler_->sendCommand(baud_command)) {
        RCLCPP_ERROR(logger_, "Failed to send baud configuration command at baud %u", target_baud);
        serialHandler_->close();
        serialHandler_.reset();
        return false;
      }

      // Read any opportunistic feedback while sweeping baud rates.
      serialHandler_->read();
      std::this_thread::sleep_for(baud_settle_sleep);
    }
  } else {
    RCLCPP_INFO(logger_, "Skipping the serial baud sweep (serial_baud_sweep=false)");
  }

  if (!serialHandler_->configurePort(serialBaud_)) {
    RCLCPP_ERROR(logger_, "Failed to configure serial port baud rate: %u", serialBaud_);
    serialHandler_->close();
    serialHandler_.reset();
    return false;
  }
  std::this_thread::sleep_for(baud_settle_sleep);

  if (!runStartupStateMachine(report_ms)) {
    RCLCPP_ERROR(logger_, "Startup state machine failed: %s",
      startupContext_.failure_reason.c_str());
    serialHandler_->close();
    serialHandler_.reset();
    return false;
  }

  return true;
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

bool RoamadomeControl::rotationInProgress(std::string * reason) const
{
  const int percent = lastCommandedPercent_.load();
  if (0 != percent) {
    *reason = "dome is rotating (commanded " + std::to_string(percent) +
      "%); stop it before changing settings";
    return true;
  }
  // A bench drive that has been accepted but not yet sent counts as rotating.
  std::lock_guard<std::mutex> lock(bench_mutex_);
  if (benchActive_ && (benchPendingPercent_ != 0 || benchLastSentPercent_ != 0)) {
    *reason = "a bench drive is active (pending " + std::to_string(benchPendingPercent_) +
      "%); stop it before changing settings";
    return true;
  }
  if (benchActive_ && benchMoveTarget_.has_value()) {
    *reason = "a bench move to " + std::to_string(*benchMoveTarget_) +
      " deg is active; release it (bench drive 0) before changing settings";
    return true;
  }
  if (firmwareMoveOutstanding_.load()) {
    *reason = "a firmware position move is still outstanding; wait for REACHED TARGET "
      "before changing settings";
    return true;
  }
  return false;
}

bool RoamadomeControl::domeMoving() const
{
  std::string reason;
  if (rotationInProgress(&reason)) {
    return true;
  }
  const double v = velocity_;
  return !std::isnan(v) && std::fabs(v) > 0.1;
}

void RoamadomeControl::requestConfigRefresh(const std::string & reason)
{
  const bool moving = domeMoving();
  std::lock_guard<std::mutex> lock(sendQueue_mutex_);
  if (moving) {
    if (!configRefreshPending_) {
      RCLCPP_DEBUG(logger_, "Deferring %s #DPCONFIG until the dome is still", reason.c_str());
    }
    configRefreshPending_ = true;
    configRefreshDeferredSince_ = std::chrono::steady_clock::now();
    return;
  }
  enqueueConfigDumpWithInvalidLocked(reason);
}

void RoamadomeControl::serviceDeferredConfigRefresh()
{
  {
    std::lock_guard<std::mutex> lock(sendQueue_mutex_);
    if (!configRefreshPending_) {
      return;
    }
  }
  const auto now = std::chrono::steady_clock::now();
  const bool moving = domeMoving();
  std::lock_guard<std::mutex> lock(sendQueue_mutex_);
  if (moving) {
    configRefreshDeferredSince_ = now;
    return;
  }
  if (now - configRefreshDeferredSince_ >= std::chrono::milliseconds(configRefreshSettleMs_)) {
    enqueueConfigDumpWithInvalidLocked("deferred periodic refresh");
    configRefreshPending_ = false;
  }
}

bool RoamadomeControl::queueSetHomeHere(std::string * message)
{
  if (!lifecycleActive_.load()) {
    *message = "hardware interface is inactive; command not queued";
    RCLCPP_WARN(logger_, "set_home_here called while hardware interface is inactive");
    return false;
  }

  if (!serialHandler_ || !serialHandler_->isOpen()) {
    *message = "hardware interface is unavailable; command not queued";
    RCLCPP_WARN(logger_, "set_home_here called while serial interface is unavailable");
    return false;
  }

  std::string rotating_reason;
  if (rotationInProgress(&rotating_reason)) {
    *message = rotating_reason;
    RCLCPP_WARN(logger_, "set_home_here rejected: %s", rotating_reason.c_str());
    return false;
  }

  // #DPHOMEPOS with no argument stores the current dome position as home. The firmware
  // answers Write Settings / Updated, or Unchanged if home is already there.
  {
    std::lock_guard<std::mutex> lock(sendQueue_mutex_);
    sendQueue_.push({"set_home_here", "#DPHOMEPOS",
        SendQueueCommand::Flow::PARAMETER_UPDATE_ACK, true});
  }
  *message = "Set home command queued";  // queued, not yet acked by the firmware
  RCLCPP_INFO(logger_, "set_home_here called, #DPHOMEPOS will be sent in next write cycle");
  return true;
}

void RoamadomeControl::enqueueStatusPollWithInvalidLocked(const std::string & reason)
{
  // #DPINVALID ends the STATUS section promptly instead of waiting for the
  // section flush timeout, exactly as the config dump does.
  sendQueue_.push(
    {reason + " status poll #DPSTATUS", "#DPSTATUS", SendQueueCommand::Flow::ONE_SHOT, false});
  sendQueue_.push(
    {reason + " status poll #DPINVALID", "#DPINVALID", SendQueueCommand::Flow::ONE_SHOT, false});
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
      RCLCPP_INFO(logger_, "Received 'Write Settings' for %s",
        activeSendQueueCommand_.label.c_str());
      return;
    }

    if ("Updated" == line) {
      RCLCPP_WARN(logger_, "Received 'Updated' before 'Write Settings' for %s",
        activeSendQueueCommand_.label.c_str());
      return;
    }
  }

  if (SendQueueStage::WAIT_WRITE_SETTINGS == sendQueueStage_ && "Unchanged" == line) {
    // The firmware prints only "Unchanged" when the value is already set: nothing was
    // written, so there is nothing to refresh.
    RCLCPP_INFO(logger_, "Received 'Unchanged' for %s; ack complete, no config refresh",
      activeSendQueueCommand_.label.c_str());
    recordSettingsAckLocked(false);
    sendQueueCommandActive_ = false;
    sendQueueStage_ = SendQueueStage::IDLE;
    return;
  }

  if (SendQueueStage::WAIT_UPDATED == sendQueueStage_ && "Updated" == line) {
    RCLCPP_INFO(logger_, "Received 'Updated' for %s", activeSendQueueCommand_.label.c_str());
    if (activeSendQueueCommand_.post_ack_config_dump) {
      enqueueConfigDumpWithInvalidLocked("post-update");
    }
    recordSettingsAckLocked(false);
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

  // Drop anything queued while inactive (for example a set_home_here racing a
  // deactivate) so it is not sent at the start of this activation.
  {
    std::lock_guard<std::mutex> lock(sendQueue_mutex_);
    while (!sendQueue_.empty()) {
      sendQueue_.pop();
    }
    sendQueueCommandActive_ = false;
    sendQueueStage_ = SendQueueStage::IDLE;
  }
  resetBenchState();

  // on_deactivate closes the serial port, and an inactive -> active transition
  // does not pass through on_configure again, so reopen the device here.
  if (!serialHandler_) {
    RCLCPP_INFO(logger_, "Serial port is closed; reopening device and re-running startup");
    if (!openDeviceAndRunStartup()) {
      RCLCPP_ERROR(logger_, "Cannot activate RoamadomeControl: device bring-up failed");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

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

  if (!setHomeService_) {
    setHomeService_ = node->create_service<std_srvs::srv::Trigger>(
      "~/set_home_here",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>/* request */,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = queueSetHomeHere(&response->message);
      });
  }

  if (enableBenchMode_ && !benchDriveService_) {
    benchDriveService_ = node->create_service<astromech_msgs::srv::DomeBenchDrive>(
      "~/bench/drive",
      [this](const std::shared_ptr<astromech_msgs::srv::DomeBenchDrive::Request> request,
      std::shared_ptr<astromech_msgs::srv::DomeBenchDrive::Response> response) {
        handleBenchDrive(request, response);
      });
    RCLCPP_WARN(logger_, "Bench mode enabled: %s raw drive service is available",
      benchDriveService_->get_service_name());
  }
  if (enableBenchMode_ && !benchMoveService_) {
    benchMoveService_ = node->create_service<astromech_msgs::srv::DomeBenchMove>(
      "~/bench/move",
      [this](const std::shared_ptr<astromech_msgs::srv::DomeBenchMove::Request> request,
      std::shared_ptr<astromech_msgs::srv::DomeBenchMove::Response> response) {
        handleBenchMove(request, response);
      });
  }

  if (!periodicConfigTimer_ && configRefreshIntervalMs_ > 0) {
    periodicConfigTimer_ = node->create_wall_timer(
      std::chrono::milliseconds(configRefreshIntervalMs_),
      [this]() {requestConfigRefresh("periodic refresh");});
  }

  // The unwrapped angle and sequence counter are "since activation".
  positionTracker_.reset();
  plausibilityGate_.reset();
  plausibilityGate_.resetCounters();
  firmwareMoveOutstanding_.store(false);
  telemetrySequence_ = 0;
  velocity_ = std::numeric_limits<double>::quiet_NaN();

  // Diagnostics: fresh counters per activation, published at 1 Hz on the
  // global /diagnostics topic, with a periodic #DPSTATUS poll for the
  // firmware's own error counters.
  {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    diagnosticsState_ = DomeDiagnosticsState();
  }
  if (!diagnosticsPublisher_) {
    diagnosticsPublisher_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)));
  }
  if (!diagnosticsTimer_) {
    diagnosticsTimer_ = node->create_wall_timer(
      std::chrono::seconds(1), [this]() {publishDiagnostics();});
  }
  if (statusPollIntervalMs_ > 0) {
    {
      std::lock_guard<std::mutex> lock(sendQueue_mutex_);
      enqueueStatusPollWithInvalidLocked("activation");
    }
    if (!statusPollTimer_) {
      statusPollTimer_ = node->create_wall_timer(
        std::chrono::milliseconds(statusPollIntervalMs_),
        [this]() {
          std::lock_guard<std::mutex> lock(sendQueue_mutex_);
          enqueueStatusPollWithInvalidLocked("periodic");
        });
    }
  }

  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    telemetryClock_ = node->get_clock();
    if (publishTelemetry_ && !telemetryPublisher_) {
      telemetryPublisher_ = node->create_publisher<astromech_msgs::msg::DomeTelemetry>(
        "~/telemetry", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
      RCLCPP_INFO(logger_, "Publishing dome telemetry on %s",
        telemetryPublisher_->get_topic_name());
    }
  }

  lifecycleActive_.store(true);

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

  lifecycleActive_.store(false);

  setupService_.reset();
  setHomeService_.reset();
  benchDriveService_.reset();
  benchMoveService_.reset();
  // Keep the callback registered across deactivate so device.* writes continue
  // to be validated/rejected deterministically while hardware is inactive.

  if (periodicConfigTimer_) {
    periodicConfigTimer_->cancel();
    periodicConfigTimer_.reset();
  }

  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    telemetryPublisher_.reset();
    telemetryClock_.reset();
  }

  if (diagnosticsTimer_) {
    diagnosticsTimer_->cancel();
    diagnosticsTimer_.reset();
  }
  if (statusPollTimer_) {
    statusPollTimer_->cancel();
    statusPollTimer_.reset();
  }
  diagnosticsPublisher_.reset();

  // Reset the send queue and related state to ensure a clean slate on next activation.
  {
    std::lock_guard<std::mutex> lock(sendQueue_mutex_);
    while (!sendQueue_.empty()) {
      sendQueue_.pop();
    }
    sendQueueCommandActive_ = false;
    sendQueueStage_ = SendQueueStage::IDLE;
    configRefreshPending_ = false;
  }

  if (serialHandler_) {
    // Continuous rotation has no firmware watchdog, so never leave the dome
    // turning when the interface goes inactive.
    sendStopCommand();
    currentMode_ = CommandMode::IDLE;
    resetBenchState();
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
  bool benchBusy = false;
  if (enableBenchMode_) {
    std::lock_guard<std::mutex> lock(bench_mutex_);
    benchBusy = benchActive_;
  }

  serviceDeferredConfigRefresh();

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
        if (SendQueueCommand::Flow::PARAMETER_UPDATE_ACK == activeSendQueueCommand_.flow) {
          recordSettingsAckLocked(true);
        }
        sendQueueCommandActive_ = false;
        sendQueueStage_ = SendQueueStage::IDLE;
      }
    }

    if (!sendQueueCommandActive_ && !sendQueue_.empty()) {
      // While a bench drive is active, settings writes must wait (the firmware
      // stops the motor on them); one-shot queries may still go. Rotate through
      // the queue once to find the first sendable command.
      for (size_t n = sendQueue_.size(); n > 0; --n) {
        if (benchBusy && sendQueue_.front().flow == SendQueueCommand::Flow::PARAMETER_UPDATE_ACK) {
          sendQueue_.push(sendQueue_.front());
          sendQueue_.pop();
          continue;
        }
        queued_command = sendQueue_.front();
        sendQueue_.pop();
        if (queued_command->flow == SendQueueCommand::Flow::PARAMETER_UPDATE_ACK) {
          // Mark it in flight before the serial write, so a bench request that
          // arrives during the write cannot start motion behind a settings write.
          activeSendQueueCommand_ = *queued_command;
          sendQueueCommandActive_ = true;
          sendQueueStage_ = SendQueueStage::WAIT_WRITE_SETTINGS;
          sendQueueStageStartTime_ = std::chrono::steady_clock::now();
        }
        break;
      }
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
        sendQueueStageStartTime_ = std::chrono::steady_clock::now();  // ack timeout from the send
      }
      RCLCPP_INFO(logger_, "Sent ack-tracked command '%s' (%s)",
        command.command.c_str(), command.label.c_str());
    } else {
      {
        std::lock_guard<std::mutex> lock(sendQueue_mutex_);
        sendQueueCommandActive_ = false;
        sendQueueStage_ = SendQueueStage::IDLE;
      }
      RCLCPP_WARN(logger_, "Failed ack-tracked command '%s' (%s)",
        command.command.c_str(), command.label.c_str());
    }
  }

  // Determine which command mode is active and send appropriate command
  if (currentMode_ == CommandMode::POSITION && std::isfinite(cmd_position_)) {
    // Position command: convert radians to degrees and normalize to [0, 359]
    uint32_t target_degrees = normalizeAngleDegrees(cmd_position_);

    // Only send if position changed significantly (avoid redundant commands)
    if (std::isnan(lastSentPosition_) || std::fabs(cmd_position_ - lastSentPosition_) > 0.01) {
      if (sendPositionCommand(target_degrees)) {
        lastSentPosition_ = cmd_position_;
        RCLCPP_DEBUG(logger_, "Sent position command: %u°", target_degrees);
      }
    }
  } else if (currentMode_ == CommandMode::VELOCITY && std::isfinite(cmd_velocity_)) {
    // Velocity command: convert rad/s to percentage [-100, 100]
    int32_t target_percentage = velocityToPercentage(cmd_velocity_);

    // Only send when the percent actually changes (avoid redundant commands)
    if (target_percentage != lastSentPercent_.load()) {
      if (sendVelocityCommand(target_percentage)) {
        lastSentPercent_.store(target_percentage);
        RCLCPP_DEBUG(logger_, "Sent velocity command: %d%%", target_percentage);
      }
    }
  }
  // IDLE mode: no controller commands; only the bench drive may act.
  if (enableBenchMode_) {
    serviceBenchDrive();
  }

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

  // A bench drive must never run under a controller: stop it first.
  if (new_mode != CommandMode::IDLE && !stopBenchDrive("controller started")) {
    RCLCPP_ERROR(logger_,
      "Refusing controller start: the bench drive could not be stopped (:DPR0 failed)");
    return return_type::ERROR;
  }

  // If switching away from an active mode, send stop command
  if (currentMode_ != CommandMode::IDLE && new_mode != currentMode_) {
    RCLCPP_INFO(logger_, "Mode switch detected, sending stop command");
    sendStopCommand();
  }

  if (new_mode == CommandMode::VELOCITY && currentMode_ != CommandMode::VELOCITY) {
    // :DPR0 has been sent (or nothing was commanded): the first non-zero velocity
    // command is sent, a command of 0 is not re-sent.
    cmd_velocity_ = 0.0;
    lastSentPercent_.store(0);
  }
  if (new_mode != CommandMode::IDLE && firmwareMoveOutstanding_.load()) {
    RCLCPP_WARN(logger_,
      "A firmware position move is still outstanding; it cannot be cancelled from the host and "
      "resumes after any :DPR0 until the firmware reports REACHED TARGET or a timeout");
  }
  if (new_mode == CommandMode::POSITION && currentMode_ != CommandMode::POSITION) {
    // A position controller starts with whatever the command interface holds
    // (0 unless it wrote first), which used to send the dome to 0 degrees on
    // activation. Hold the current angle instead until the controller commands.
    cmd_position_ = position_;
    lastSentPosition_ = cmd_position_;
    RCLCPP_INFO(logger_, "Position mode: holding the current angle (%.1f deg) until commanded",
      radiansToDegrees(position_));
  }

  {
    std::lock_guard<std::mutex> lock(bench_mutex_);
    currentMode_ = new_mode;
    updateTelemetryModeLocked();
  }

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
  return velocityToPercent(rad_per_sec, velocityMap_);
}

bool RoamadomeControl::parseVelocityMapParameters(
  const std::unordered_map<std::string, std::string> & parameters)
{
  static const std::vector<std::string> kKnown = {
    "k_percent_per_rad_s", "p0_percent", "min_percent", "percent_sign"};
  bool unknown_key = false;
  for (const auto & entry : parameters) {
    if (entry.first.rfind("velocity_map.", 0) != 0) {
      continue;
    }
    bool known = entry.first == "velocity_map.below_minimum";
    for (const char * dir : {"positive", "negative"}) {
      for (const auto & key : kKnown) {
        known = known || entry.first == std::string("velocity_map.") + dir + "." + key;
      }
    }
    if (!known) {
      RCLCPP_ERROR(logger_, "Unknown velocity map parameter '%s'", entry.first.c_str());
      unknown_key = true;
    }
  }
  if (unknown_key) {
    return false;
  }
  for (const char * dir : {"positive", "negative"}) {
    size_t given = 0;
    for (const auto & key : kKnown) {
      given += parameters.count(std::string("velocity_map.") + dir + "." + key);
    }
    if (given != 0 && given != kKnown.size()) {
      RCLCPP_WARN(
        logger_,
        "velocity_map.%s is only partly specified (%zu of 4 keys); the others use defaults",
        dir, given);
    }
  }

  // k defaults to 100 / max_speed_rad_per_sec, so max_speed_rad_per_sec must be parsed first.
  const double default_k = 100.0 / maxSpeedRadPerSec_;
  VelocityMap map = linearVelocityMap(maxSpeedRadPerSec_);
  const auto parse_direction =
    [&](const std::string & prefix, VelocityMapDirection * dir) {
      if (!parseOptionalDoubleParameter(
          parameters, logger_, prefix + ".k_percent_per_rad_s", default_k,
          [](double value) {return std::isfinite(value) && value > 0.0;},
          "finite and > 0", &dir->k_percent_per_rad_s))
      {
        return false;
      }
      if (!parseOptionalDoubleParameter(
          parameters, logger_, prefix + ".p0_percent", 0.0,
          [](double value) {return std::isfinite(value);},
          "finite", &dir->p0_percent))
      {
        return false;
      }
      if (!parseOptionalDoubleParameter(
          parameters, logger_, prefix + ".min_percent", 0.0,
          [](double value) {
            return value >= 0.0 && value <= 100.0 && value == std::floor(value);
          },
          "an integer in 0..100", &dir->min_percent))
      {
        return false;
      }
      double sign = dir->percent_sign;
      if (!parseOptionalDoubleParameter(
          parameters, logger_, prefix + ".percent_sign", sign,
          [](double value) {return value == 1.0 || value == -1.0;},
          "1 or -1", &sign))
      {
        return false;
      }
      dir->percent_sign = static_cast<int>(sign);
      return true;
    };

  if (!parse_direction("velocity_map.positive", &map.positive) ||
    !parse_direction("velocity_map.negative", &map.negative))
  {
    return false;
  }

  std::string below_minimum;
  if (!parseOptionalEnumParameter(
      parameters, logger_, "velocity_map.below_minimum", "zero", {"zero", "minimum"},
      &below_minimum))
  {
    return false;
  }
  if (map.positive.percent_sign == map.negative.percent_sign) {
    RCLCPP_ERROR(
      logger_,
      "velocity_map.positive.percent_sign and velocity_map.negative.percent_sign must differ "
      "(both are %+d): the motor sign is one physical fact",
      map.positive.percent_sign);
    return false;
  }
  map.below_minimum = below_minimum == "minimum" ? BelowMinimum::MINIMUM : BelowMinimum::ZERO;
  velocityMap_ = map;

  RCLCPP_INFO(
    logger_,
    "Velocity map: positive v -> percent_sign %+d, k %.4f %%/(rad/s), p0 %.3f %%, min %.1f %%; "
    "negative v -> percent_sign %+d, k %.4f %%/(rad/s), p0 %.3f %%, min %.1f %%; "
    "below minimum: %s",
    map.positive.percent_sign, map.positive.k_percent_per_rad_s, map.positive.p0_percent,
    map.positive.min_percent,
    map.negative.percent_sign, map.negative.k_percent_per_rad_s, map.negative.p0_percent,
    map.negative.min_percent,
    below_minimum.c_str());
  return true;
}

std::string RoamadomeControl::positionCommandString(uint32_t degrees, uint32_t speed_percent) const
{
  const uint32_t speed = speed_percent != 0 ? speed_percent : positionMoveSpeedPercent_;
  std::string command = buildCommand(":DPA", degrees);
  if (speed != 0) {
    command += "," + std::to_string(speed);
  }
  return command;
}

bool RoamadomeControl::sendPositionCommand(uint32_t degrees, uint32_t speed_percent)
{
  if (!serialHandler_ || !serialHandler_->isOpen()) {
    RCLCPP_WARN(logger_, "Serial handler not available for position command");
    return false;
  }

  // Format: :DPA<degrees>[,<percent>] where degrees is 0-359; the optional percent is a
  // per-move speed the firmware applies to this move only.
  const std::string command = positionCommandString(degrees, speed_percent);
  if (!serialHandler_->sendCommand(command)) {
    return false;
  }
  firmwareMoveOutstanding_.store(true);
  return true;
}

bool RoamadomeControl::sendVelocityCommand(int32_t percentage)
{
  if (!serialHandler_ || !serialHandler_->isOpen()) {
    RCLCPP_WARN_THROTTLE(logger_, throttleClock_, 1000,
      "Serial handler not available for velocity command");
    return false;
  }

  // Format: :DPR<speed> where speed is -100 to 100
  std::string command = buildCommand(":DPR", percentage);
  if (!serialHandler_->sendCommand(command)) {
    return false;
  }
  recordCommandedPercent(percentage);
  return true;
}

void RoamadomeControl::updateTelemetryModeLocked()
{
  telemetryCommandMode_.store(
    benchActive_ ? astromech_msgs::msg::DomeTelemetry::COMMAND_MODE_BENCH :
    telemetryCommandModeValue(currentMode_));
}

bool RoamadomeControl::benchRequestAllowed(std::string * reason, bool starts_motion) const
{
  if (!lifecycleActive_.load()) {
    *reason = "hardware interface is inactive";
    return false;
  }
  if (!serialHandler_ || !serialHandler_->isOpen()) {
    *reason = "serial port is closed";
    return false;
  }
  if (starts_motion) {
    // A settings write that reaches the firmware stops the motor (plan 2.3) and
    // would never be resent, so do not start motion while one is in flight.
    bool settings_in_flight = false;
    {
      std::lock_guard<std::mutex> lock(sendQueue_mutex_);
      settings_in_flight = sendQueueCommandActive_;
      auto queue_copy = sendQueue_;
      while (!settings_in_flight && !queue_copy.empty()) {
        settings_in_flight =
          queue_copy.front().flow == SendQueueCommand::Flow::PARAMETER_UPDATE_ACK;
        queue_copy.pop();
      }
    }
    if (settings_in_flight) {
      *reason = "a settings write is in flight";
      return false;
    }
  }
  return true;
}

void RoamadomeControl::handleBenchDrive(
  const std::shared_ptr<astromech_msgs::srv::DomeBenchDrive::Request> request,
  std::shared_ptr<astromech_msgs::srv::DomeBenchDrive::Response> response)
{
  // Executor thread. Records the request only; write() does the serial I/O.
  response->success = false;
  const int32_t percent = std::clamp<int32_t>(request->percent, -100, 100);
  std::string reason;
  if (!benchRequestAllowed(&reason, percent != 0)) {
    response->message = reason + "; bench drive rejected";
    return;
  }
  std::optional<uint16_t> released_move;
  {
    std::lock_guard<std::mutex> lock(bench_mutex_);
    if (currentMode_ != CommandMode::IDLE) {
      response->message = "a controller is commanding the dome (mode is not IDLE); "
        "bench drive rejected";
      return;
    }
    if (percent != 0 && benchActive_ && benchMoveTarget_.has_value()) {
      // :DPR overrides a firmware move only while non-zero; at :DPR0 the firmware
      // resumes the move. Never mix the two.
      response->message = "a bench move to " + std::to_string(*benchMoveTarget_) +
        " deg is active; release it with percent 0 once the dome is at rest; bench drive rejected";
      return;
    }
    if (percent != 0 && firmwareMoveOutstanding_.load()) {
      // A released or controller move the firmware has not finished: a drive would
      // override it and the stop would restart it.
      response->message = "a firmware position move is still outstanding (no REACHED TARGET or "
        "timeout yet); wait for it to finish; bench drive rejected";
      return;
    }
    if (!benchActive_ && percent == 0 && benchLastSentPercent_ == 0 &&
      lastCommandedPercent_.load() == 0)
    {
      response->success = true;
      response->message = "bench drive already stopped";
      return;
    }
    benchActive_ = true;
    benchPendingPercent_ = percent;
    benchLastCallTime_ = std::chrono::steady_clock::now();
    if (percent == 0 && benchMoveTarget_.has_value()) {
      released_move = benchMoveTarget_;
      benchMoveTarget_.reset();
      benchMoveSpeed_ = 0;
      benchMoveSent_ = false;
    }
    updateTelemetryModeLocked();
  }
  response->success = true;
  if (released_move.has_value()) {
    response->message = "bench move to " + std::to_string(*released_move) + " deg released";
    return;
  }
  response->message = "bench drive " + std::to_string(percent) + "% accepted; call again within " +
    std::to_string(benchHeartbeatTimeoutMs_) + " ms to keep the dome turning";
}

void RoamadomeControl::handleBenchMove(
  const std::shared_ptr<astromech_msgs::srv::DomeBenchMove::Request> request,
  std::shared_ptr<astromech_msgs::srv::DomeBenchMove::Response> response)
{
  // Executor thread. Records the request only; write() sends :DPA<target>.
  response->success = false;
  if (request->target_deg > 359) {
    response->message = "target_deg must be 0..359; bench move rejected";
    return;
  }
  if (request->speed_percent > 100) {
    response->message = "speed_percent must be 0..100; bench move rejected";
    return;
  }
  const uint16_t target = request->target_deg;
  const uint8_t speed = request->speed_percent;
  std::string reason;
  if (!benchRequestAllowed(&reason, true)) {
    response->message = reason + "; bench move rejected";
    return;
  }
  bool kept_alive = false;
  {
    std::lock_guard<std::mutex> lock(bench_mutex_);
    if (currentMode_ != CommandMode::IDLE) {
      response->message = "a controller is commanding the dome (mode is not IDLE); "
        "bench move rejected";
      return;
    }
    if (benchActive_ && (benchPendingPercent_ != 0 || benchLastSentPercent_ != 0)) {
      response->message = "a bench drive is active (pending " +
        std::to_string(benchPendingPercent_) + "%); stop it (percent 0) first; bench move rejected";
      return;
    }
    if (lastCommandedPercent_.load() != 0) {
      response->message = "the dome is still commanded at " +
        std::to_string(lastCommandedPercent_.load()) + "%; stop it first; bench move rejected";
      return;
    }
    kept_alive = benchMoveTarget_.has_value() && *benchMoveTarget_ == target &&
      benchMoveSpeed_ == speed && benchMoveSent_;
    if (!kept_alive) {
      benchMoveTarget_ = target;
      benchMoveSpeed_ = speed;
      benchMoveSent_ = false;
    }
    benchActive_ = true;
    benchPendingPercent_ = 0;
    benchLastCallTime_ = std::chrono::steady_clock::now();
    updateTelemetryModeLocked();
  }
  response->success = true;
  if (kept_alive) {
    response->message = "bench move to " + std::to_string(target) + " deg kept alive";
    return;
  }
  response->message = "bench move to " + std::to_string(target) +
    " deg accepted; the firmware runs it to completion (it cannot be cancelled); call again "
    "within " + std::to_string(benchHeartbeatTimeoutMs_) +
    " ms to keep the bench claim, then release it with bench drive 0 at rest";
}

void RoamadomeControl::resetBenchState()
{
  std::lock_guard<std::mutex> lock(bench_mutex_);
  benchActive_ = false;
  benchPendingPercent_ = 0;
  benchLastSentPercent_ = 0;
  benchMoveTarget_.reset();
  benchMoveSpeed_ = 0;
  benchMoveSent_ = false;
  updateTelemetryModeLocked();
}

bool RoamadomeControl::stopBenchDrive(const char * reason)
{
  // Controller manager thread. Sends :DPR0 if a non-zero percent was sent and
  // clears the bench state only once that succeeded. On failure the state is
  // kept (pending 0) so write() retries every cycle.
  bool need_stop = false;
  std::optional<uint16_t> move;
  {
    std::lock_guard<std::mutex> lock(bench_mutex_);
    need_stop = benchActive_ && benchLastSentPercent_ != 0;
    if (benchActive_) {
      move = benchMoveTarget_;
    }
  }
  if (need_stop) {
    if (!sendStopCommand()) {
      RCLCPP_ERROR_THROTTLE(logger_, throttleClock_, 1000,
        "Bench drive stop (%s) failed: could not send :DPR0; will retry", reason);
      std::lock_guard<std::mutex> lock(bench_mutex_);
      benchPendingPercent_ = 0;
      return false;
    }
    RCLCPP_WARN(logger_, "Bench drive stopped (%s); sent :DPR0", reason);
  }
  if (move.has_value()) {
    RCLCPP_WARN(logger_,
      "Bench move to %u deg released (%s); the firmware finishes the move on its own",
      *move, reason);
  }
  resetBenchState();
  return true;
}

void RoamadomeControl::serviceBenchDrive()
{
  enum class Action {NONE, SEND, MOVE, EXPIRE, CONTROLLER};
  Action action = Action::NONE;
  int32_t percent = 0;
  uint16_t target = 0;
  uint8_t move_speed = 0;
  {
    std::lock_guard<std::mutex> lock(bench_mutex_);
    if (!benchActive_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (currentMode_ != CommandMode::IDLE) {
      action = Action::CONTROLLER;
    } else if (now - benchLastCallTime_ > std::chrono::milliseconds(benchHeartbeatTimeoutMs_)) {
      action = Action::EXPIRE;
    } else if (benchMoveTarget_.has_value() && !benchMoveSent_) {
      action = Action::MOVE;
      target = *benchMoveTarget_;
      move_speed = benchMoveSpeed_;
    } else if (benchPendingPercent_ != benchLastSentPercent_ ||
      (benchPendingPercent_ == 0 && lastCommandedPercent_.load() != 0))
    {
      action = Action::SEND;
      percent = benchPendingPercent_;
    } else if (benchPendingPercent_ == 0 && !benchMoveTarget_.has_value()) {
      // Pending and last-sent are both 0 and no move is claimed: go back to idle now.
      benchActive_ = false;
      updateTelemetryModeLocked();
    }
  }

  if (action == Action::CONTROLLER) {
    stopBenchDrive("a controller took over");
  } else if (action == Action::EXPIRE) {
    stopBenchDrive("heartbeat expired");
  } else if (action == Action::MOVE) {
    const std::string command = positionCommandString(target, move_speed);
    if (!sendPositionCommand(target, move_speed)) {
      RCLCPP_ERROR_THROTTLE(logger_, throttleClock_, 1000,
        "Bench move: failed to send %s; will retry", command.c_str());
    } else {
      RCLCPP_INFO(logger_, "Bench move: sent %s", command.c_str());
      std::lock_guard<std::mutex> lock(bench_mutex_);
      if (benchMoveTarget_.has_value() && *benchMoveTarget_ == target &&
        benchMoveSpeed_ == move_speed)
      {
        benchMoveSent_ = true;  // a newer target would be sent on the next cycle
      }
    }
  } else if (action == Action::SEND) {
    const bool sent = percent == 0 ? sendStopCommand() : sendVelocityCommand(percent);
    if (!sent) {
      RCLCPP_ERROR_THROTTLE(logger_, throttleClock_, 1000,
        "Bench drive: failed to send :DPR%d; will retry", percent);
    } else {
      std::lock_guard<std::mutex> lock(bench_mutex_);
      benchLastSentPercent_ = percent;
      if (percent == 0 && benchPendingPercent_ == 0) {
        benchActive_ = false;  // explicitly stopped
        updateTelemetryModeLocked();
      }
    }
  }
}

bool RoamadomeControl::sendStopCommand()
{
  if (!serialHandler_ || !serialHandler_->isOpen()) {
    RCLCPP_WARN_THROTTLE(logger_, throttleClock_, 1000,
      "Serial handler not available for stop command");
    return false;
  }

  // Send stop command: :DPR0 (0% velocity)
  if (!serialHandler_->sendCommand(":DPR0")) {
    return false;
  }
  recordCommandedPercent(0);
  lastSentPercent_.store(0);
  return true;
}

void RoamadomeControl::recordCommandedPercent(int32_t percentage)
{
  const int32_t clamped = std::clamp<int32_t>(percentage, -100, 100);
  lastCommandedPercent_.store(static_cast<int8_t>(clamped));
}

uint8_t RoamadomeControl::telemetryCommandModeValue(CommandMode mode)
{
  switch (mode) {
    case CommandMode::POSITION:
      return astromech_msgs::msg::DomeTelemetry::COMMAND_MODE_POSITION;
    case CommandMode::VELOCITY:
      return astromech_msgs::msg::DomeTelemetry::COMMAND_MODE_VELOCITY;
    case CommandMode::IDLE:
    default:
      return astromech_msgs::msg::DomeTelemetry::COMMAND_MODE_IDLE;
  }
}

void RoamadomeControl::handlePositionReport(uint32_t degrees, double radians)
{
  // Runs on the read path for every "DOME POSITION: <n>" line. Host receive
  // time is the sample timestamp (firmware report jitter is ~1 ms).
  (void)radians;  // recomputed from the accepted integer angle
  const auto now_steady = std::chrono::steady_clock::now();

  // Gap accounting counts raw arrivals, before the plausibility gate.
  double gap_s = 0.0;
  uint32_t gaps = 0;
  {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    gap_s = recordPositionReport(&diagnosticsState_, now_steady, reportGapWarnMs_ / 1000.0);
    gaps = diagnosticsState_.report_gaps;
  }
  if (reportGapWarnMs_ > 0 && gap_s * 1000.0 > reportGapWarnMs_) {
    // Reports normally arrive every cycle; a long gap means the firmware, the
    // USB link or this process stalled. Log it with its length so the next
    // dropout is self-documenting.
    RCLCPP_WARN(logger_, "Position report gap of %.0f ms (gap #%u since activation)",
      gap_s * 1000.0, gaps);
  }

  const ReportPlausibilityGate::Decision decision =
    plausibilityGate_.update(now_steady, static_cast<uint16_t>(degrees % 360));
  if (decision.held || decision.rejected || decision.confirmed || decision.resynced) {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    diagnosticsState_.reports_held = plausibilityGate_.heldCount();
    diagnosticsState_.reports_rejected = plausibilityGate_.rejectedCount();
    diagnosticsState_.jumps_confirmed = plausibilityGate_.confirmedCount();
    diagnosticsState_.gate_resyncs = plausibilityGate_.resyncCount();
    if (decision.rejected) {
      diagnosticsState_.last_report_rejected = now_steady;
    }
  }
  if (decision.rejected) {
    RCLCPP_WARN_THROTTLE(logger_, throttleClock_, 1000,
      "Rejected implausible position report %u deg (the next report, %u deg, disagreed with it; "
      "%u rejected since activation)", decision.dropped.degrees, degrees % 360,
      plausibilityGate_.rejectedCount());
  }
  if (decision.resynced) {
    RCLCPP_WARN(logger_,
      "Position reports disagreed with the accepted history %u times in a row: accepting %u deg "
      "as the new position (is max_plausible_deg_s below the dome's real speed?)",
      ReportPlausibilityGate::kResyncAfter, degrees % 360);
  }
  if (decision.held) {
    RCLCPP_DEBUG(logger_, "Holding position report %u deg until the next one confirms it",
      degrees % 360);
  }
  if (decision.confirmed) {
    RCLCPP_INFO(logger_, "Position jump to %u deg confirmed by the next report (%u deg)",
      decision.accepted[0].degrees, degrees % 360);
  }
  for (std::size_t i = 0; i < decision.accepted_count; ++i) {
    applyAcceptedReport(decision.accepted[i]);
  }
}

void RoamadomeControl::applyAcceptedReport(const ReportPlausibilityGate::Report & report)
{
  const PositionTracker::Estimate estimate = positionTracker_.update(report.time, report.degrees);

  position_ = estimate.position_rad;
  velocity_ = estimate.velocity_rad_s;
  ++telemetrySequence_;

  RCLCPP_DEBUG(logger_, "Position updated: %u° = %.4f rad, unwrapped %.4f rad, velocity %.4f rad/s",
    report.degrees, estimate.position_rad, estimate.unwrapped_rad, estimate.velocity_rad_s);

  rclcpp::Publisher<astromech_msgs::msg::DomeTelemetry>::SharedPtr publisher;
  rclcpp::Clock::SharedPtr clock;
  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    publisher = telemetryPublisher_;
    clock = telemetryClock_;
  }
  if (!publisher || !clock) {
    return;
  }

  astromech_msgs::msg::DomeTelemetry msg;
  msg.header.stamp = clock->now();  // a confirmed held report is published one report late
  msg.header.frame_id = joint_name_;
  msg.position_deg = report.degrees;
  msg.position_rad = estimate.position_rad;
  msg.unwrapped_rad = estimate.unwrapped_rad;
  msg.velocity_rad_s = estimate.velocity_rad_s;
  msg.commanded_percent = lastCommandedPercent_.load();
  msg.command_mode = telemetryCommandMode_.load();
  msg.sequence = telemetrySequence_;
  publisher->publish(msg);
}

void RoamadomeControl::handleClassifiedLine(const std::string & raw_line)
{
  const FirmwareLine line = classifyFirmwareLine(raw_line);
  if (FirmwareLineKind::UNKNOWN == line.kind) {
    return;
  }

  const bool active = lifecycleActive_.load();
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    applyFirmwareLine(&diagnosticsState_, line, now, active);
  }

  switch (line.kind) {
    case FirmwareLineKind::POSITION_TIMEOUT:
      firmwareMoveOutstanding_.store(false);
      RCLCPP_WARN(logger_, "Firmware aborted the dome move: no position change within its timeout");
      break;
    case FirmwareLineKind::REACHED_TARGET:
    case FirmwareLineKind::REACHED_HOME:
      firmwareMoveOutstanding_.store(false);
      break;
    case FirmwareLineKind::READY:
      if (active) {
        RCLCPP_ERROR(logger_,
          "Dome controller rebooted (READY seen while active); settings and reporting may be reset");
      }
      break;
    case FirmwareLineKind::OUT_OF_RANGE:
      RCLCPP_WARN(logger_, "Firmware rejected a value as out of range");
      break;
    case FirmwareLineKind::SETUP_FAILED:
      RCLCPP_WARN(logger_, "Firmware reported AUTO SETUP FAILED");
      break;
    case FirmwareLineKind::INVALID:
      // Routine: every #DPINVALID probe produces one.
      break;
    default:
      RCLCPP_DEBUG(logger_, "Firmware line %s: %s", firmwareLineKindName(line.kind),
        raw_line.c_str());
      break;
  }
}

void RoamadomeControl::handleStatusSection(const std::vector<std::string> & status)
{
  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  applyStatusSection(&diagnosticsState_, status, std::chrono::steady_clock::now());
}

uint32_t RoamadomeControl::settingsWritesPendingLocked() const
{
  uint32_t pending = 0;
  auto queue_copy = sendQueue_;
  while (!queue_copy.empty()) {
    if (SendQueueCommand::Flow::PARAMETER_UPDATE_ACK == queue_copy.front().flow) {
      ++pending;
    }
    queue_copy.pop();
  }
  if (sendQueueCommandActive_ &&
    SendQueueCommand::Flow::PARAMETER_UPDATE_ACK == activeSendQueueCommand_.flow)
  {
    ++pending;
  }
  return pending;
}

void RoamadomeControl::recordSettingsAckLocked(bool timed_out)
{
  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  if (timed_out) {
    ++diagnosticsState_.settings_ack_timeouts;
  } else {
    ++diagnosticsState_.settings_acks;
  }
}

void RoamadomeControl::publishDiagnostics()
{
  auto publisher = diagnosticsPublisher_;
  auto node = get_node();
  if (!publisher || !node) {
    return;
  }

  DomeDiagnosticsReadings readings;
  readings.hardware_id = serialPort_;
  readings.serial_open = serialHandler_ && serialHandler_->isOpen();
  readings.position_rad = position_;
  readings.velocity_rad_s = velocity_;
  readings.commanded_percent = lastCommandedPercent_.load();
  readings.command_mode = telemetryCommandMode_.load();
  {
    std::lock_guard<std::mutex> lock(sendQueue_mutex_);
    readings.settings_writes_pending = settingsWritesPendingLocked();
  }

  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = node->now();
  {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    msg.status = buildDomeDiagnostics(
      diagnosticsState_, readings, DomeDiagnosticsOptions(), std::chrono::steady_clock::now());
  }
  publisher->publish(msg);
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

  std::vector<SendQueueCommand> validated_commands;
  validated_commands.reserve(parameters.size());

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

    if (!lifecycleActive_.load()) {
      result.successful = false;
      result.reason = "cannot update device parameter " + name +
        ": hardware interface is inactive";
      return result;
    }

    if (!serialHandler_ || !serialHandler_->isOpen()) {
      result.successful = false;
      result.reason = "cannot update device parameter " + name +
        ": hardware interface is not active";
      return result;
    }

    std::string rotating_reason;
    if (rotationInProgress(&rotating_reason)) {
      result.successful = false;
      result.reason = "cannot update device parameter " + name + ": " + rotating_reason;
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

    validated_commands.push_back({
        name,
        command,
        SendQueueCommand::Flow::PARAMETER_UPDATE_ACK,
        true
    });
  }

  if (!validated_commands.empty()) {
    std::lock_guard<std::mutex> lock(sendQueue_mutex_);
    for (const auto & send_command : validated_commands) {
      sendQueue_.push(send_command);
      RCLCPP_INFO(logger_, "Queued parameter update: %s (queue depth=%zu)",
        send_command.label.c_str(), sendQueue_.size());
    }
  }

  return result;
}

}  // namespace ros2_roamadome

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_roamadome::RoamadomeControl,
  hardware_interface::ActuatorInterface)
