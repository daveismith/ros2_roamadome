#ifndef roamadome_control__ROAMADOME_CONTROL_HPP_
#define roamadome_control__ROAMADOME_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "astromech_msgs/srv/dome_bench_drive.hpp"
#include "astromech_msgs/msg/dome_telemetry.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "ros2_roamadome/device_parameter_specs.hpp"
#include "ros2_roamadome/dome_diagnostics.hpp"
#include "ros2_roamadome/firmware_line_classifier.hpp"
#include "ros2_roamadome/position_tracker.hpp"
#include "ros2_roamadome/visibility_control.h"
#include "ros2_roamadome/roamadome_serial_port.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>

using hardware_interface::return_type;

namespace ros2_roamadome
{

class RoamadomeControl : public hardware_interface::ActuatorInterface
{
public:
  friend class RoamadomeControlTest;

  RoamadomeControl();
  virtual ~RoamadomeControl();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  // on_cleanup
  // on_shutdown

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  // Command mode enumeration
  enum class CommandMode
  {
    IDLE,
    POSITION,
    VELOCITY
  };

  // Startup state machine for on_configure()
  enum class StartupState
  {
    SEND_COMMAND,
    WAIT_PROBE,
    COMPLETE,
    FAILED
  };

  enum class StartupCommandId
  {
    CONFIG_INITIAL = 0,
    SETUP,
    AUTOSAFETY0,
    VERIFY_AUTOSAFETY,
    STATUS,
    SET_AUTOMODE,
    SET_HOMEMODE,
    REPORT
  };

  // Forward declare for type aliases
  struct StartupContext;

  using CommandFormatterFunc = std::function<std::string(const StartupContext &,
      RoamadomeControl *)>;
  using TransitionLambdaFunc = std::function<std::optional<StartupCommandId>(const StartupContext &,
      RoamadomeControl *)>;

  struct StartupCommandInfo
  {
    StartupCommandId id;
    std::string command_template;
    uint32_t timeout_ms;
    bool is_terminal_command = false;
    std::optional<StartupCommandId> linear_next;
    std::optional<CommandFormatterFunc> command_formatter;
    std::optional<TransitionLambdaFunc> transition_lambda;
  };

  struct StartupContext
  {
    /// @note: startupContext_ is accessed from both the serial reader callback
    /// (which runs during serialHandler_->read() via SerialEventHandler::onConfigUpdate)
    /// and from the startup state machine (which runs synchronously during on_configure).
    /// Access to config_map and config_timestamp fields are protected by
    /// startupContext_mutex_ to ensure thread-safe delivery of configuration snapshots.

    StartupState state = StartupState::SEND_COMMAND;
    std::optional<StartupCommandId> current_command;
    std::string formatted_command;
    std::string state_label;
    std::chrono::steady_clock::time_point state_start;
    uint32_t timeout_ms = 1000;
    uint32_t retries_used = 0;
    uint32_t report_ms = 0;
    bool probe_process_seen = false;
    bool probe_invalid_seen = false;
    bool auto_safety_engaged = true;
    std::string failure_reason;

    // Configuration storage (protected by startupContext_mutex_)
    std::map<std::string, std::string> config_map;
    std::chrono::steady_clock::time_point config_timestamp;
  };

  // Helper methods for startup state machine
  void resetStartupContext(StartupContext * context, uint32_t timeout_ms);
  void initializeStartupCommandTable();
  const StartupCommandInfo * findStartupCommandInfo(StartupCommandId command_id) const;
  std::string formatStartupCommand(
    StartupCommandId command_id,
    const StartupContext & context) const;
  bool sendStartupCommandWithProbe(
    StartupContext * context,
    StartupCommandId command_id);
  std::optional<StartupCommandId> resolveStartupNextCommand(
    const StartupContext & context) const;
  bool startupProbeComplete(const StartupContext & context) const;
  bool startupStateTimedOut(const StartupContext & context) const;
  void transitionStartupState(
    StartupContext * context,
    StartupState next_state,
    const std::string & label,
    uint32_t timeout_ms);
  bool retryCurrentStartupCommand(StartupContext * context);
  bool runStartupStateMachine(uint32_t report_ms);
  bool openDeviceAndRunStartup();
  void handleStartupUnhandledLine(const std::string & line);
  void handleSendQueueFeedback(const std::string & line);
  void logStartupTransition(const std::string & message);
  bool queueSetHomeHere(std::string * message);
  bool rotationInProgress(std::string * reason) const;
  void enqueueConfigDumpWithInvalidLocked(const std::string & reason);
  void enqueueStatusPollWithInvalidLocked(const std::string & reason);

  // Helper methods for command handling
  uint32_t normalizeAngleDegrees(double radians) const;
  double radiansToDegrees(double radians) const;
  int32_t velocityToPercentage(double rad_per_sec) const;
  bool sendPositionCommand(uint32_t degrees);
  bool sendVelocityCommand(int32_t percentage);
  bool sendStopCommand();

  // Bench mode (docs/dome_diagnostics_plan.md, phase 4)
  void handleBenchDrive(
    const std::shared_ptr<astromech_msgs::srv::DomeBenchDrive::Request> request,
    std::shared_ptr<astromech_msgs::srv::DomeBenchDrive::Response> response);
  void serviceBenchDrive();
  bool stopBenchDrive(const char * reason);  // false if :DPR0 could not be sent; state kept
  void resetBenchState();
  void updateTelemetryModeLocked();  // requires bench_mutex_

  // Telemetry and velocity estimation
  void handlePositionReport(uint32_t degrees, double radians);

  // Diagnostics
  void handleClassifiedLine(const std::string & line);
  void handleStatusSection(const std::vector<std::string> & status);
  void publishDiagnostics();
  /// Ack-tracked settings writes queued plus one if active. Caller holds sendQueue_mutex_.
  uint32_t settingsWritesPendingLocked() const;
  /// Count a completed or timed-out settings ack in diagnosticsState_. Caller holds
  /// sendQueue_mutex_; lock order is sendQueue_mutex_ then diagnostics_mutex_.
  void recordSettingsAckLocked(bool timed_out);
  void recordCommandedPercent(int32_t percentage);
  static uint8_t telemetryCommandModeValue(CommandMode mode);

  // Parameter management methods
  void ensureParameterCallbackRegistered(const rclcpp::Node::SharedPtr & node);
  void declareDeviceParameters();
  static std::vector<rclcpp::Parameter> filterChangedParameters(
    const rclcpp::Node::SharedPtr & node,
    const std::vector<rclcpp::Parameter> & desired_parameters);
  void updateParametersFromDevice();
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters);

  // Logging and hardware connection
  rclcpp::Logger logger_;

  double position_;
  double velocity_;

  double cmd_position_;
  double cmd_velocity_;

  // Command mode state tracking
  // Atomic: written by the controller manager thread (mode switch), read by the
  // bench drive service callback on the executor thread.
  std::atomic<CommandMode> currentMode_{CommandMode::IDLE};
  double lastSentPosition_ = -1.0;  // Track last sent position to avoid redundant commands
  double lastSentVelocity_ = 0.0;   // Track last sent velocity to avoid redundant commands
  double maxSpeedRadPerSec_ = 3.14;  // Default: ~180°/s (configurable via parameter)
  uint32_t serialSectionFlushTimeoutMs_ = 500;

  std::unique_ptr<RoamadomeSerialPort> serialHandler_;
  std::string serialPort_;
  uint32_t serialBaud_;

  uint32_t startupDefaultTimeoutMs_ = 1000;
  uint32_t startupReportTimeoutMs_ = 1000;
  uint32_t startupSetupTimeoutMs_ = 10000;
  uint32_t startupMaxRetries_ = 1;
  uint32_t baudSweepSleepMs_ = 500;
  uint32_t startupLoopSleepMs_ = 10;
  uint32_t configStaleWarningMs_ = 30000;
  std::array<StartupCommandInfo, 8> startupCommandTable_{};
  bool startupCommandTableInitialized_ = false;

  // Configuration flags
  bool autoModeEnabled_ = false;
  bool homeModeEnabled_ = false;
  std::string startupLogLevel_ = "debug";

  StartupContext startupContext_;
  mutable std::mutex startupContext_mutex_;  // Protects startupContext_.config_map and config_timestamp

  const uint32_t mSupportedBaudRates[5] = {
    2400,
    9600,
    19200,
    38400,
    115200
  };

  // Joint naming - read from URDF during initialization
  std::string joint_name_;

  // Device configuration structure

  struct SendQueueCommand
  {
    enum class Flow
    {
      ONE_SHOT,
      PARAMETER_UPDATE_ACK
    };

    std::string label;
    std::string command;
    Flow flow = Flow::ONE_SHOT;
    bool post_ack_config_dump = false;
  };

  enum class SendQueueStage
  {
    IDLE,
    WAIT_WRITE_SETTINGS,
    WAIT_UPDATED
  };

  DeviceParameterRegistry deviceParameterRegistry_;
  mutable std::mutex deviceParameterRegistry_mutex_;

  // Command send queue
  std::queue<SendQueueCommand> sendQueue_;
  mutable std::mutex sendQueue_mutex_;
  bool sendQueueCommandActive_ = false;
  SendQueueCommand activeSendQueueCommand_;
  SendQueueStage sendQueueStage_ = SendQueueStage::IDLE;
  std::atomic<bool> lifecycleActive_{false};
  std::chrono::steady_clock::time_point sendQueueStageStartTime_;
  uint32_t sendQueueAckTimeoutMs_ = 3000;
  uint32_t configRefreshIntervalMs_ = 60000;  // Default 60 seconds
  rclcpp::TimerBase::SharedPtr periodicConfigTimer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCallbackHandle_;

  // Setup service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr setupService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr setHomeService_;

  // Bench mode. The service callback (executor thread) only records the request
  // under bench_mutex_; write() (controller manager thread) does all serial I/O
  // and never holds bench_mutex_ across it.
  bool enableBenchMode_ = false;
  uint32_t benchHeartbeatTimeoutMs_ = 1500;
  rclcpp::Service<astromech_msgs::srv::DomeBenchDrive>::SharedPtr benchDriveService_;
  mutable std::mutex bench_mutex_;
  rclcpp::Clock throttleClock_{RCL_STEADY_TIME};
  bool benchActive_ = false;
  int32_t benchPendingPercent_ = 0;
  int32_t benchLastSentPercent_ = 0;
  std::chrono::steady_clock::time_point benchLastCallTime_;

  // Telemetry and velocity estimation (docs/dome_diagnostics_plan.md, phase 1).
  // positionTracker_ and telemetrySequence_ are only touched from the read path.
  // lastCommandedPercent_ and telemetryCommandMode_ are written from the write
  // path and the mode switch, read from the read path, hence atomics.
  bool publishTelemetry_ = false;
  double velocityWindowS_ = 0.5;
  PositionTracker positionTracker_;
  uint32_t telemetrySequence_ = 0;
  std::atomic<int8_t> lastCommandedPercent_{0};
  std::atomic<uint8_t> telemetryCommandMode_{0};
  rclcpp::Publisher<astromech_msgs::msg::DomeTelemetry>::SharedPtr telemetryPublisher_;
  rclcpp::Clock::SharedPtr telemetryClock_;
  mutable std::mutex telemetry_mutex_;  // Protects telemetryPublisher_ and telemetryClock_

  // Diagnostics (docs/dome_diagnostics_plan.md, phase 2). diagnosticsState_ is
  // mutated from the serial read path and read by the 1 Hz timer.
  uint32_t statusPollIntervalMs_ = 10000;
  DomeDiagnosticsState diagnosticsState_;
  mutable std::mutex diagnostics_mutex_;  // Protects diagnosticsState_
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticsPublisher_;
  rclcpp::TimerBase::SharedPtr diagnosticsTimer_;
  rclcpp::TimerBase::SharedPtr statusPollTimer_;

  // Inner class: Observer for serial port events
  class SerialEventHandler : public ISerialObserver
  {
public:
    SerialEventHandler(RoamadomeControl * controller)
    : controller_(controller)
    {
    }

    virtual ~SerialEventHandler() = default;

    void onPositionUpdate(uint32_t degrees, double radians) override
    {
      controller_->handlePositionReport(degrees, radians);
    }

    void onVelocityUpdate(double rad_per_sec) override
    {
      (void)rad_per_sec;
      // Device does not send velocity updates; velocity is estimated from the
      // position stream in handlePositionReport().
    }

    void onUnhandledLine(const std::string & line) override
    {
      controller_->handleClassifiedLine(line);
      controller_->handleSendQueueFeedback(line);
      controller_->handleStartupUnhandledLine(line);
      RCLCPP_DEBUG(controller_->logger_, "Unhandled serial line: %s", line.c_str());
    }

    void onConfigUpdate(const std::map<std::string, std::string> & config) override
    {
      bool parsed_cleanly = false;
      {
        std::lock_guard<std::mutex> lock(controller_->deviceParameterRegistry_mutex_);
        parsed_cleanly = controller_->deviceParameterRegistry_.parseConfigMap(
          config, controller_->logger_);
        if (!parsed_cleanly) {
          RCLCPP_WARN(controller_->logger_,
            "Device configuration parsed with one or more field errors");
        }
      }

      if (parsed_cleanly) {
        controller_->updateParametersFromDevice();
      } else {
        RCLCPP_WARN(controller_->logger_,
          "Skipping device->ROS parameter sync because configuration parse was not clean");
      }

      // Also update startup context for startup state machine
      {
        std::lock_guard<std::mutex> lock(controller_->startupContext_mutex_);
        controller_->startupContext_.config_map = config;
        controller_->startupContext_.config_timestamp = std::chrono::steady_clock::now();
      }
    }

    void onStatusUpdate(const std::vector<std::string> & status) override
    {
      for (const auto & line : status) {
        std::string lowered_line = line;
        std::transform(lowered_line.begin(), lowered_line.end(), lowered_line.begin(),
          [](unsigned char c) {return static_cast<char>(std::tolower(c));});
        if (lowered_line.find("auto safety engaged") != std::string::npos) {
          controller_->startupContext_.auto_safety_engaged = true;
        } else if (lowered_line.find("auto safety disengaged") != std::string::npos) {
          controller_->startupContext_.auto_safety_engaged = false;
        }
      }
      controller_->handleStatusSection(status);
    }

private:
    RoamadomeControl * controller_;
  };

  std::unique_ptr<SerialEventHandler> serialEventHandler_;

};

}  // namespace ros2_roamadome

#endif  // roamadome_control__ROAMADOME_CONTROL_HPP_
