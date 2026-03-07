#ifndef roamadome_control__ROAMADOME_CONTROL_HPP_
#define roamadome_control__ROAMADOME_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"

#include "ros2_roamadome/visibility_control.h"
#include "ros2_roamadome/roamadome_serial_port.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include <memory>

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
      RCLCPP_DEBUG(controller_->logger_, "Unhandled serial line: %s", line.c_str());
    }

    void onConfigUpdate(const std::map<std::string, std::string> & config) override
    {
      RCLCPP_INFO(controller_->logger_, "Config received:");
      for (const auto & [key, value] : config) {
        RCLCPP_INFO(controller_->logger_, "  %s = %s", key.c_str(), value.c_str());
      }
    }

    void onStatusUpdate(const std::vector<std::string> & status) override
    {
      RCLCPP_INFO(controller_->logger_, "Status received:");
      for (const auto & line : status) {
        RCLCPP_INFO(controller_->logger_, "  %s", line.c_str());
      }
    }

private:
    RoamadomeControl * controller_;
  };

  std::unique_ptr<SerialEventHandler> serialEventHandler_;

};

}  // namespace roamadome_control

#endif  // roamadome_control__ROAMADOME_CONTROL_HPP_
