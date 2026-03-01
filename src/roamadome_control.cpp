#include "ros2_roamadome/roamadome_control.hpp"
#include <iostream>

namespace ros2_roamadome
{

RoamadomeControl::RoamadomeControl() : logger_(rclcpp::get_logger("RoamadomeControl")), position_(0.0)
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

    //RCLCPP_INFO(logger_, "Initializing RoamadomeControl with params: %s", params.name.c_str());
    RCLCPP_INFO(logger_, "Initializing RoamadomeControl with params: %s", info_.name.c_str());
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoamadomeControl::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(logger_, "Configuring RoamadomeControl from state: %s", previous_state.label().c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoamadomeControl::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(logger_, "Activating RoamadomeControl from state: %s", previous_state.label().c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoamadomeControl::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(logger_, "Deactivating RoamadomeControl from state: %s", previous_state.label().c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

/*return_type RoamadomeControl::configure(const hardware_interface::HardwareInfo & info)
{
    std::cout << "Configuring RoamadomeControl with hardware info: " << info.name << std::endl;
    return return_type::OK;
}
*/

/*return_type RoamadomeControl::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}


return_type RoamadomeControl::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}*/

std::vector<hardware_interface::StateInterface> RoamadomeControl::export_state_interfaces()
{
  /*std::vector<StateInterface> state_interfaces;
  for (size_t i = 0; i < exported_state_interface_names_.size(); ++i)
  {
    state_interfaces.emplace_back(
      get_node()->get_name(), exported_state_interface_names_[i], &state_interfaces_values_[i]);
  }
  return state_interfaces;*/
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface("dome_joint", "position", &position_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("dome_joint", "velocity", &velocity_)); // Placeholder for velocity state interface
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoamadomeControl::export_command_interfaces()
{
  /*std::vector<CommandInterface> command_interfaces;
  for (size_t i = 0; i < exported_command_interface_names_.size(); ++i)
  {
    command_interfaces.emplace_back(
      get_node()->get_name(), exported_command_interface_names_[i], &command_interfaces_values_[i]);
  }
  return command_interfaces;*/

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface("dome_joint", "position", &cmd_position_)); // Placeholder for position command interface
  command_interfaces.emplace_back(hardware_interface::CommandInterface("dome_joint", "velocity", &cmd_velocity_)); // Placeholder for velocity command interface
  return command_interfaces;
}

hardware_interface::return_type RoamadomeControl::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void) time;
    (void) period;

    /*position_ += 0.01745; // Simulate some movement for testing
    if (position_ > 3.14159) {
        position_ = -3.14159; // Wrap around for testing
    }*/
   position_ += (cmd_velocity_ * period.seconds()); // Simulate some movement for testing

    RCLCPP_INFO(rclcpp::get_logger("RoamadomeControl"), "Read called. Current position: %f", position_);

  //RCLCPP_INFO(logger_, "read called with time: %f and period: %f", time.seconds(), period.seconds());
  return return_type::OK; 
}

hardware_interface::return_type RoamadomeControl::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void) time;
  (void) period;

  //RCLCPP_INFO(logger_, "write called with time: %f and period: %f", time.seconds(), period.seconds());
  RCLCPP_INFO(logger_, "write called. Command position: %f, Command velocity: %f", cmd_position_, cmd_velocity_);

  return return_type::OK;
}

}  // namespace roamadome_control

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_roamadome::RoamadomeControl,
  hardware_interface::ActuatorInterface)
