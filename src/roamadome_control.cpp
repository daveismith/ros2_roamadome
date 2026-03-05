#include "ros2_roamadome/roamadome_control.hpp"
#include <iostream>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static uint32_t sSerialBaudRates[] = {
    2400,
    9600,
    19200,
    38400
};

namespace ros2_roamadome
{

RoamadomeControl::RoamadomeControl() : logger_(rclcpp::get_logger("RoamadomeControl")), position_(0.0), serialFd_(-1)
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
            RCLCPP_WARN(logger_, "serial_baud not found, defaulting to 38400");
            serialBaud_ = 38400;
        }
    } catch (const std::out_of_range& oor) {
        RCLCPP_ERROR(logger_, "serial_baud out of range");
        return hardware_interface::CallbackReturn::ERROR;
    } catch (const std::invalid_argument &ia) {
        RCLCPP_ERROR(logger_, "serial_baud has invalid format");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoamadomeControl::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    const char *status = "#DPSTATUS\n";
    //const char *report = "#DPREPORT50\n";   // Report Every 50 ms
    char baud_cmd[20] = { 0 }; 
    char report_cmd[20] = { 0 };    // report config
    char buffer[1000] = { 0 };
    int n = 0;
    const struct timespec ts = { .tv_sec = 0, .tv_nsec = 500000000 };
    uint32_t reportMs = (1000 / info_.rw_rate) - 5; // report at the required rate, go a bit faster to ensure at least one report per read cycle

    RCLCPP_INFO(logger_, "Configuring RoamadomeControl from state: %s", previous_state.label().c_str());
    snprintf(report_cmd, sizeof(report_cmd), "#DPREPORT%u\n", reportMs);
    RCLCPP_INFO(logger_, "Read Rate in Hz: %u (%s)", info_.rw_rate, report_cmd);
    

    // This Is Where We Open The Serial Port
    if (-1 == serialFd_) {
        int fd = openSerialPort(serialPort_.c_str());
        if (fd < 0) {
            goto fail;
        }
        serialFd_ = fd;
    }

    n = snprintf(baud_cmd, sizeof(baud_cmd), "#DPSERIALBAUD%u\n", serialBaud_);
    for (size_t idx = 0; idx < sizeof(sSerialBaudRates) / sizeof(sSerialBaudRates[0]); idx++) {
        // Now configure the baud rate
        int baudRate = sSerialBaudRates[idx];

        if (configureSerialPort(serialFd_, baudRate) != true) {
            // failed to configure the baud rate
            goto fail;
        }

        ::write(serialFd_, baud_cmd, n);
        nanosleep(&ts, NULL);
    }

    // Switch To Final Baud
    configureSerialPort(serialFd_, serialBaud_);
        
    ::write(serialFd_, status, strlen(status));

    nanosleep(&ts, NULL);

    // Read Back Some Data
    n = ::read(serialFd_, buffer, sizeof(buffer) / sizeof(buffer[0]));
    if (n < 0) {
        RCLCPP_ERROR(logger_, "Error reading from serial port: %s", strerror(errno));
    } else {
        RCLCPP_INFO(logger_, "Serial: %d, %s", n, buffer);
    }

    // Configure Reporting
    ::write(serialFd_, report_cmd, strlen(report_cmd));

    return hardware_interface::CallbackReturn::SUCCESS;

fail:
    if (serialFd_ >= 0) {
        closeSerialPort(serialFd_);
        serialFd_ = -1;
    }
    return hardware_interface::CallbackReturn::ERROR;
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
    const char *report = "#DPREPORT0\n";   // Disable Reporting
    const struct timespec ts = { .tv_sec = 0, .tv_nsec = 500000000 };    
    
    RCLCPP_INFO(logger_, "Deactivating RoamadomeControl from state: %s", previous_state.label().c_str());

    // Disable Reporting & Disconnect
    ::write(serialFd_, report, strlen(report));
    nanosleep(&ts, NULL);
    closeSerialPort(serialFd_);
    serialFd_ = -1;

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

    static uint32_t counter = 0;

    char buffer[100] = { 0 };
    int n = 0;

    /*position_ += 0.01745; // Simulate some movement for testing
    if (position_ > 3.14159) {
        position_ = -3.14159; // Wrap around for testing
    }*/
    position_ += (cmd_velocity_ * period.seconds()); // Simulate some movement for testing

    n = ::read(serialFd_, buffer, sizeof(buffer) / sizeof(buffer[0]));
    if (n < 0) {
        RCLCPP_ERROR(logger_, "Error reading from serial port: %s", strerror(errno));
    } else if (n > 0) {
        RCLCPP_INFO(logger_, "counter: %u, data: %s", counter, buffer);
    }

    counter++;

    //RCLCPP_INFO(rclcpp::get_logger("RoamadomeControl"), "Read called. Current position: %f", position_);

    //RCLCPP_INFO(logger_, "read called with time: %f and period: %f", time.seconds(), period.seconds());
    return return_type::OK; 
}

hardware_interface::return_type RoamadomeControl::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void) time;
  (void) period;

  //RCLCPP_INFO(logger_, "write called with time: %f and period: %f", time.seconds(), period.seconds());
  //RCLCPP_INFO(logger_, "write called. Command position: %f, Command velocity: %f", cmd_position_, cmd_velocity_);

  return return_type::OK;
}

int RoamadomeControl::openSerialPort(const char* aPortName)
{
    int fd = open(aPortName, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        RCLCPP_ERROR(logger_, "Error opening %s: %s", aPortName, strerror(errno));
        return -1;
    }
    return fd;
}

void RoamadomeControl::closeSerialPort(int aFd)
{
    close(aFd);
}

bool RoamadomeControl::configureSerialPort(int aFd, int aBaudRate)
{
    struct termios tty;

    if (aFd < 0) {
        RCLCPP_ERROR(logger_, "Invalid file descriptor in configureSerialPort");
        return false;
    }

    if (tcgetattr(aFd, &tty) != 0) {
        RCLCPP_ERROR(logger_, "Error from tcgetattr: %s", strerror(errno));
        return false;
    }

    cfsetospeed(&tty, aBaudRate);
    cfsetispeed(&tty, aBaudRate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0;    // no signalling characters, no echo, no canonical processing
    tty.c_oflag = 0;    // no remapping, no delays
    tty.c_cc[VMIN] = 0; // read doesn't block
    tty.c_cc[VTIME] = 0;    // 0.1 second read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff control
    tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls & enable reading
    tty.c_cflag &= ~(PARENB | PARODD);  // disable partity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(aFd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(logger_, "Error from tcsetattr: %s", strerror(errno));
        return false;
    }

    return true;
}

}  // namespace roamadome_control

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_roamadome::RoamadomeControl,
  hardware_interface::ActuatorInterface)
