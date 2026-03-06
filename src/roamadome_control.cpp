#include "ros2_roamadome/roamadome_control.hpp"
#include <format>
#include <iostream>
#include <string>
#include <time.h>
#include <unistd.h>
#include <cmath>

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

  RCLCPP_INFO(logger_, "Maximum speed configured: %.4f rad/s (%.2f°/s)",
              maxSpeedRadPerSec_, radiansToDegrees(maxSpeedRadPerSec_));

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
  std::string command = std::format(":DPA{}\n", degrees);
  return serialHandler_->sendCommand(command);
}

bool RoamadomeControl::sendVelocityCommand(int32_t percentage)
{
  if (!serialHandler_ || !serialHandler_->isOpen()) {
    RCLCPP_WARN(logger_, "Serial handler not available for velocity command");
    return false;
  }

  // Format: :DPR<speed> where speed is -100 to 100
  std::string command = std::format(":DPR{}\n", percentage);
  return serialHandler_->sendCommand(command);
}

bool RoamadomeControl::sendStopCommand()
{
  if (!serialHandler_ || !serialHandler_->isOpen()) {
    RCLCPP_WARN(logger_, "Serial handler not available for stop command");
    return false;
  }

  // Send stop command: :DPR0 (0% velocity)
  return serialHandler_->sendCommand(":DPR0\n");
}

}  // namespace roamadome_control

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_roamadome::RoamadomeControl,
  hardware_interface::ActuatorInterface)
