#ifndef roamadome_control__ROAMADOME_CONTROL_HPP_
#define roamadome_control__ROAMADOME_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"

#include "ros2_roamadome/visibility_control.h"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

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

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
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

  std::string serialPort_;
  uint32_t serialBaud_;

  int serialFd_;

  int openSerialPort(const char* aPortName);
  void closeSerialPort(int aFd);
  bool configureSerialPort(int aFd, int aBaudRate);

};

}  // namespace roamadome_control

#endif  // roamadome_control__ROAMADOME_CONTROL_HPP_
