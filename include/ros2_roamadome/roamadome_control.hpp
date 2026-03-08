#ifndef roamadome_control__ROAMADOME_CONTROL_HPP_
#define roamadome_control__ROAMADOME_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"

#include "ros2_roamadome/visibility_control.h"
#include "ros2_roamadome/roamadome_serial_port.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include <chrono>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

using hardware_interface::return_type;

namespace ros2_roamadome
{

class RoamadomeControl : public hardware_interface::ActuatorInterface
{
public:
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
    STATUS = 0,
    SETUP,
    CONFIG,
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
    bool status_received = false;
    bool config_received = false;
    bool auto_safety_engaged = true;
    std::string failure_reason;
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
  void handleStartupUnhandledLine(const std::string & line);

  // Helper methods for command handling
  uint32_t normalizeAngleDegrees(double radians) const;
  double radiansToDegrees(double radians) const;
  int32_t velocityToPercentage(double rad_per_sec) const;
  bool sendPositionCommand(uint32_t degrees);
  bool sendVelocityCommand(int32_t percentage);
  bool sendStopCommand();

  // Logging and hardware connection
  rclcpp::Logger logger_;

  std::vector<std::string> exported_state_interface_names_;
  //std::vector<hardware_interface::StateInterface::SharedPtr> ordered_exported_state_interfaces_;
  //std::unordered_map<std::string, hardware_interface::StateInterface::SharedPtr>
  //  exported_state_interfaces_;
  std::vector<double> state_interfaces_values_;

  double position_;
  double velocity_;

  double cmd_position_;
  double cmd_velocity_;

  // Command mode state tracking
  CommandMode currentMode_ = CommandMode::IDLE;
  CommandMode previousMode_ = CommandMode::IDLE;
  double lastSentPosition_ = -1.0;  // Track last sent position to avoid redundant commands
  double lastSentVelocity_ = 0.0;   // Track last sent velocity to avoid redundant commands
  double maxSpeedRadPerSec_ = 3.14;  // Default: ~180°/s (configurable via parameter)

  std::unique_ptr<RoamadomeSerialPort> serialHandler_;
  std::string serialPort_;
  uint32_t serialBaud_;

  uint32_t startupDefaultTimeoutMs_ = 1000;
  uint32_t startupReportTimeoutMs_ = 1000;
  uint32_t startupSetupTimeoutMs_ = 10000;
  uint32_t startupMaxRetries_ = 1;
  uint32_t startupLoopSleepMs_ = 10;
  std::optional<uint32_t> setupGoodMaxSpeed_;
  std::array<StartupCommandInfo, 4> startupCommandTable_{};
  bool startupCommandTableInitialized_ = false;

  StartupContext startupContext_;

  const uint32_t mSupportedBaudRates[5] = {
    2400,
    9600,
    19200,
    38400,
    115200
  };

  // Joint naming - read from URDF during initialization
  std::string joint_name_;

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
      controller_->position_ = radians;
      RCLCPP_DEBUG(
        controller_->logger_,
        "Position updated: %u° = %.4f rad", degrees, radians);
    }

    void onVelocityUpdate(double rad_per_sec) override
    {
      (void)rad_per_sec;
      // Device does not send velocity updates, this is a no-op
    }

    void onUnhandledLine(const std::string & line) override
    {
      controller_->handleStartupUnhandledLine(line);
      RCLCPP_DEBUG(controller_->logger_, "Unhandled serial line: %s", line.c_str());
    }

    void onConfigUpdate(const std::map<std::string, std::string> & config) override
    {
      controller_->startupContext_.config_received = true;
      RCLCPP_INFO(controller_->logger_, "Config received:");
      for (const auto & [key, value] : config) {
        RCLCPP_INFO(controller_->logger_, "  %s = %s", key.c_str(), value.c_str());
      }
    }

    void onStatusUpdate(const std::vector<std::string> & status) override
    {
      controller_->startupContext_.status_received = true;
      RCLCPP_INFO(controller_->logger_, "Status received:");
      for (const auto & line : status) {
        std::string lowered_line = line;
        std::transform(lowered_line.begin(), lowered_line.end(), lowered_line.begin(),
          [](unsigned char c) {return static_cast<char>(std::tolower(c));});
        if (lowered_line.find("auto safety engaged") != std::string::npos) {
          controller_->startupContext_.auto_safety_engaged = true;
        } else if (lowered_line.find("auto safety disengaged") != std::string::npos) {
          controller_->startupContext_.auto_safety_engaged = false;
        }
        RCLCPP_INFO(controller_->logger_, "  %s", line.c_str());
      }
    }

private:
    RoamadomeControl * controller_;
  };

  std::unique_ptr<SerialEventHandler> serialEventHandler_;

};

}  // namespace ros2_roamadome

#endif  // roamadome_control__ROAMADOME_CONTROL_HPP_
