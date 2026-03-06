#include "ros2_roamadome/roamadome_control.hpp"
#include <format>
#include <iostream>
#include <string>
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

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoamadomeControl::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  char report_cmd[20] = {0};
  const struct timespec ts = {.tv_sec = 0, .tv_nsec = 500000000};
  uint32_t reportMs = (1000 / info_.rw_rate) - 5;

  RCLCPP_INFO(logger_, "Configuring RoamadomeControl from state: %s",
      previous_state.label().c_str());
  snprintf(report_cmd, sizeof(report_cmd), "#DPREPORT%u\n", reportMs);
  RCLCPP_INFO(logger_, "Read Rate in Hz: %u (%s)", info_.rw_rate, report_cmd);

    // Create and setup the serial handler
  serialHandler_ = std::make_unique<RoamadomeSerialPort>(serialPort_);

    // Setup callbacks for position updates from device
  serialHandler_->setPositionCallback(
    [this](uint32_t degrees, double radians) {
      position_ = radians;
      RCLCPP_DEBUG(logger_, "Position updated: %u° = %.4f rad", degrees, radians);
        });

    // Setup callback for unhandled lines (for debugging)
  serialHandler_->setUnhandledLineCallback(
    [this](const std::string & line) {
      RCLCPP_DEBUG(logger_, "Unhandled serial line: %s", line.c_str());
        });

    // Setup callback for config updates
  serialHandler_->setConfigCallback(
    [this](const std::map<std::string, std::string> & config) {
      RCLCPP_INFO(logger_, "Config received:");
      for (const auto & [key, value] : config) {
        RCLCPP_INFO(logger_, "  %s = %s", key.c_str(), value.c_str());
      }
        });

    // Setup callback for status updates
  serialHandler_->setStatusCallback(
    [this](const std::vector<std::string> & status) {
      RCLCPP_INFO(logger_, "Status received:");
      for (const auto & line : status) {
        RCLCPP_INFO(logger_, "  %s", line.c_str());
      }
        });

    // Open the serial port
  if (!serialHandler_->open()) {
    RCLCPP_ERROR(logger_, "Failed to open serial port: %s", serialPort_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string aBaudCommand = std::format("#DPSERIALBAUD{}\n", serialBaud_);
  RCLCPP_INFO(logger_, "Baud Command is %s", aBaudCommand.c_str());
  for (size_t idx = 0; idx < sizeof(mSupportedBaudRates) / sizeof(mSupportedBaudRates[0]); idx++) {
    uint32_t targetBaud = mSupportedBaudRates[idx];

    // Configure baud rate and port settings
    if (!serialHandler_->configurePort(targetBaud)) {
      RCLCPP_ERROR(logger_, "Failed to configure serial port baud rate: %u", serialBaud_);
      serialHandler_->close();
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!serialHandler_->sendCommand(aBaudCommand)) {
      RCLCPP_ERROR(logger_, "Failed to send baud configuration command");
      serialHandler_->close();
      return hardware_interface::CallbackReturn::ERROR;
    }
    nanosleep(&ts, NULL);
  }

  if (!serialHandler_->configurePort(serialBaud_)) {
    RCLCPP_ERROR(logger_, "Failed to configure serial port baud rate: %u", serialBaud_);
    serialHandler_->close();
    return hardware_interface::CallbackReturn::ERROR;
  }
  nanosleep(&ts, NULL);

  // Send initialization commands
  serialHandler_->sendCommand(std::string("#DPSTATUS\n"));
  nanosleep(&ts, NULL);

  serialHandler_->sendCommand(std::string("#DPCONFIG\n"));
  nanosleep(&ts, NULL);

    // Configure reporting
  serialHandler_->sendCommand(report_cmd);

  RCLCPP_INFO(logger_, "RoamadomeControl configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
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
    serialHandler_->sendCommand("#DPREPORT0\n");
    nanosleep(&ts, NULL);
    serialHandler_->close();
    serialHandler_ = NULL;
  }

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
  state_interfaces.emplace_back(hardware_interface::StateInterface("dome_joint", "position",
      &position_));
  state_interfaces.emplace_back(hardware_interface::StateInterface("dome_joint", "velocity",
      &velocity_));                                                                                        // Placeholder for velocity state interface
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
  command_interfaces.emplace_back(hardware_interface::CommandInterface("dome_joint", "position",
      &cmd_position_));                                                                                            // Placeholder for position command interface
  command_interfaces.emplace_back(hardware_interface::CommandInterface("dome_joint", "velocity",
      &cmd_velocity_));                                                                                            // Placeholder for velocity command interface
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

  //RCLCPP_INFO(logger_, "write called with time: %f and period: %f", time.seconds(), period.seconds());
  //RCLCPP_INFO(logger_, "write called. Command position: %f, Command velocity: %f", cmd_position_, cmd_velocity_);

  return return_type::OK;
}

}  // namespace roamadome_control

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_roamadome::RoamadomeControl,
  hardware_interface::ActuatorInterface)
