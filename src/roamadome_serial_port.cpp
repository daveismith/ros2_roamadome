#include "ros2_roamadome/roamadome_serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <sstream>
#include <iostream>
#include <sys/ioctl.h>

namespace ros2_roamadome
{

RoamadomeSerialPort::RoamadomeSerialPort(const std::string & port_name, bool exclusive_access)
: portName_(port_name), serialFd_(-1), parseState_(ParseState::NONE),
  exclusiveAccess_(exclusive_access)
{
}

RoamadomeSerialPort::~RoamadomeSerialPort()
{
  if (isOpen()) {
    close();
  }
}

bool RoamadomeSerialPort::open()
{
  if (isOpen()) {
    std::cerr << "Serial port already open: " << portName_ << std::endl;
    return false;
  }

  int fd = ::open(portName_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    std::cerr << "Error opening " << portName_ << ": " << strerror(errno) << std::endl;
    return false;
  }

  serialFd_ = fd;
  lineBuffer_.clear();

  // Set exclusive access if requested
  if (exclusiveAccess_) {
    if (ioctl(serialFd_, TIOCEXCL) == -1) {
      std::cerr << "Error setting exclusive access on " << portName_ << ": " << strerror(errno) <<
        std::endl;
      ::close(serialFd_);
      serialFd_ = -1;
      return false;
    }
  }

  return true;
}

void RoamadomeSerialPort::close()
{
  if (isOpen()) {
    ::close(serialFd_);
    serialFd_ = -1;
    lineBuffer_.clear();
  }
}

bool RoamadomeSerialPort::configurePort(uint32_t baud_rate)
{
  if (!isOpen()) {
    std::cerr << "Cannot configure closed port" << std::endl;
    return false;
  }

  struct termios tty;

  if (tcgetattr(serialFd_, &tty) != 0) {
    std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
    return false;
  }

  // Set baud rate
  speed_t baud;
  switch (baud_rate) {
    case 2400:
      baud = B2400;
      break;
    case 9600:
      baud = B9600;
      break;
    case 19200:
      baud = B19200;
      break;
    case 38400:
      baud = B38400;
      break;
    case 115200:
      baud = B115200;
      break;
    default:
      std::cerr << "Unsupported baud rate: " << baud_rate << std::endl;
      return false;
  }

  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  // 8-bit characters
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

  // Disable break processing
  tty.c_iflag &= ~IGNBRK;

  // No signalling characters, no echo, no canonical processing
  tty.c_lflag = 0;

  // No remapping, no delays
  tty.c_oflag = 0;

  // Non-blocking read
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  // Shut off xon/xoff control
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  // Ignore modem controls & enable reading
  tty.c_cflag |= (CLOCAL | CREAD);

  // Disable parity
  tty.c_cflag &= ~(PARENB | PARODD);

  // 1 stop bit
  tty.c_cflag &= ~CSTOPB;

  // No hardware flow control
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serialFd_, TCSANOW, &tty) != 0) {
    std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool RoamadomeSerialPort::sendCommand(const std::string & command, bool append_terminator)
{
  if (!isOpen()) {
    std::cerr << "Cannot send command: port not open" << std::endl;
    return false;
  }

  std::string wire_command = command;
  if (append_terminator && (wire_command.empty() || wire_command.back() != '\n')) {
    wire_command.push_back('\n');
  }

  size_t total_written = 0;
  while (total_written < wire_command.size()) {
    const ssize_t n = ::write(
      serialFd_,
      wire_command.c_str() + total_written,
      wire_command.size() - total_written);
    if (n < 0) {
      std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
      return false;
    }
    if (0 == n) {
      std::cerr << "Error writing to serial port: short write (0 bytes written)" << std::endl;
      return false;
    }
    total_written += static_cast<size_t>(n);
  }

  return true;
}

bool RoamadomeSerialPort::read()
{
  if (!isOpen()) {
    std::cerr << "Cannot read: port not open" << std::endl;
    return false;
  }

  char buffer[100] = {0};
  ssize_t n = ::read(serialFd_, buffer, sizeof(buffer) - 1);

  if (n < 0) {
    std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
    return false;
  }

  if (n == 0) {
    // No data available (non-blocking), continue
    return true;
  }

  // Append new data to buffer
  lineBuffer_.append(buffer, n);

  // Check for buffer overflow
  if (lineBuffer_.size() > 10000) {
    std::cerr << "Serial line buffer overflow (>10KB), truncating oldest data" << std::endl;
    lineBuffer_ = lineBuffer_.substr(lineBuffer_.size() - 5000);
  }

  // Split into complete lines and remainder
  auto [lines, remainder] = splitLines(lineBuffer_);
  lineBuffer_ = remainder;

  // Process each complete line
  for (const auto & line : lines) {
    // Skip empty lines
    if (line.empty()) {
      continue;
    }

    if (line.starts_with("PROCESS: \"")) {
        // Notify any pending data at end of read
      if (parseState_ == ParseState::CONFIG && !currentConfig_.empty()) {
        notifyConfigObservers(currentConfig_);
        currentConfig_.clear();
      } else if (parseState_ == ParseState::STATUS && !currentStatus_.empty()) {
        notifyStatusObservers(currentStatus_);
        currentStatus_.clear();
      }

        // clear out any existing state
      parseState_ = ParseState::NONE;
    }

    // Check if this line starts a new PROCESS command
    // If so, flush any pending data from the previous state and transition
    if (line.find("PROCESS: \"#DPCONFIG\"") != std::string::npos) {
      // Flush previous state if needed
      if (parseState_ == ParseState::STATUS && !currentStatus_.empty()) {
        notifyStatusObservers(currentStatus_);
        currentStatus_.clear();
      }
      parseState_ = ParseState::CONFIG;
      currentConfig_.clear();
      continue;  // Skip further processing of this PROCESS line
    }

    if (line.find("PROCESS: \"#DPSTATUS\"") != std::string::npos) {
      // Flush previous state if needed
      if (parseState_ == ParseState::CONFIG && !currentConfig_.empty()) {
        notifyConfigObservers(currentConfig_);
        currentConfig_.clear();
      }
      parseState_ = ParseState::STATUS;
      currentStatus_.clear();
      continue;  // Skip further processing of this PROCESS line
    }

    // Always try to parse position line first (regardless of state)
    auto positionData = parsePositionLine(line);
    if (positionData) {
      auto [degrees, radians] = positionData.value();
      notifyPositionObservers(degrees, radians);
      continue;  // Successfully parsed position, skip remaining processing
    }

    // Process based on current state for non-position lines
    if (parseState_ == ParseState::CONFIG) {
      auto kv = parseConfigLine(line);
      if (kv) {
        currentConfig_[kv->first] = kv->second;
      }
    } else if (parseState_ == ParseState::STATUS) {
      currentStatus_.push_back(line);
    } else {
      // In NONE state, line doesn't match known patterns
      notifyUnhandledLineObservers(line);
    }
  }

  // Notify any pending data at end of read
  if (parseState_ == ParseState::CONFIG && !currentConfig_.empty()) {
    notifyConfigObservers(currentConfig_);
    currentConfig_.clear();
  } else if (parseState_ == ParseState::STATUS && !currentStatus_.empty()) {
    notifyStatusObservers(currentStatus_);
    currentStatus_.clear();
  }

  return true;
}

std::optional<std::pair<std::string, std::string>> RoamadomeSerialPort::parseConfigLine(
  const std::string & line)
{
  size_t eqPos = line.find('=');
  if (eqPos != std::string::npos) {
    std::string key = trim(line.substr(0, eqPos));
    std::string value = trim(line.substr(eqPos + 1));
    return std::make_pair(key, value);
  }
  return std::nullopt;
}

std::optional<std::pair<uint32_t, double>> RoamadomeSerialPort::parsePositionLine(
  const std::string & line)
{
  const std::string searchStr = "DOME POSITION: ";
  auto pos = line.find(searchStr);

  if (pos == std::string::npos) {
    return std::nullopt;
  }

  // Extract substring after "DOME POSITION: "
  std::string posStr = line.substr(pos + searchStr.length());

  // Trim the position string
  posStr = trim(posStr);

  // Try to parse as integer (degrees)
  try {
    // Find the end of the number (space, comma, newline, etc.)
    size_t endIdx = 0;
    for (size_t i = 0; i < posStr.length(); ++i) {
      if (0 == std::isdigit(static_cast<unsigned char>(posStr[i]))) {
        endIdx = i;
        break;
      }
      endIdx = i + 1;
    }

    std::string numStr = posStr.substr(0, endIdx);
    if (numStr.empty()) {
      std::cerr << "Warning: Failed to extract number from position line: " << line << std::endl;
      return std::nullopt;
    }

    uint32_t degrees = std::stoul(numStr);

    // Validate range [0, 359]
    if (degrees > 359) {
      std::cerr << "Warning: Position out of range [0-359]: " << degrees << std::endl;
      return std::nullopt;
    }

    // Convert to radians
    double radians = degrees * (M_PI / 180.0);

    return std::make_pair(degrees, radians);

  } catch (const std::exception & e) {
    std::cerr << "Warning: Failed to parse position from line: " << line << " (" << e.what()
              << ")" << std::endl;
    return std::nullopt;
  }
}

std::pair<std::vector<std::string>, std::string> RoamadomeSerialPort::splitLines(
  const std::string & data)
{
  std::vector<std::string> lines;
  std::string remainder;

  size_t start = 0;
  size_t end = data.find('\n');

  while (end != std::string::npos) {
    // Extract line (excluding the newline)
    std::string line = data.substr(start, end - start);

    // Remove carriage return if present
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    lines.push_back(line);

    start = end + 1;
    end = data.find('\n', start);
  }

  // Remaining data (incomplete line)
  if (start < data.length()) {
    remainder = data.substr(start);
  }

  return {lines, remainder};
}

std::string RoamadomeSerialPort::trim(const std::string & str)
{
  // Find first non-whitespace
  size_t start = str.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }

  // Find last non-whitespace
  size_t end = str.find_last_not_of(" \t\r\n");

  return str.substr(start, end - start + 1);
}

void RoamadomeSerialPort::notifyPositionObservers(uint32_t degrees, double radians)
{
  // Call registered callback
  if (positionCallback_) {
    positionCallback_(degrees, radians);
  }

  // Call observer interface methods
  for (auto observer : observers_) {
    observer->onPositionUpdate(degrees, radians);
  }
}

void RoamadomeSerialPort::notifyVelocityObservers(double rad_per_sec)
{
  // Call registered callback
  if (velocityCallback_) {
    velocityCallback_(rad_per_sec);
  }

  // Call observer interface methods
  for (auto observer : observers_) {
    observer->onVelocityUpdate(rad_per_sec);
  }
}

void RoamadomeSerialPort::notifyUnhandledLineObservers(const std::string & line)
{
  // Call registered callback
  if (unhandledLineCallback_) {
    unhandledLineCallback_(line);
  }

  // Call observer interface methods
  for (auto observer : observers_) {
    observer->onUnhandledLine(line);
  }
}

void RoamadomeSerialPort::notifyConfigObservers(const std::map<std::string, std::string> & config)
{
  // Call registered callback
  if (configCallback_) {
    configCallback_(config);
  }

  // Call observer interface methods
  for (auto observer : observers_) {
    observer->onConfigUpdate(config);
  }
}

void RoamadomeSerialPort::notifyStatusObservers(const std::vector<std::string> & status)
{
  // Call registered callback
  if (statusCallback_) {
    statusCallback_(status);
  }

  // Call observer interface methods
  for (auto observer : observers_) {
    observer->onStatusUpdate(status);
  }
}

}  // namespace ros2_roamadome
