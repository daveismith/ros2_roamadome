#ifndef roamadome_serial_port__ROAMADOME_SERIAL_PORT_HPP_
#define roamadome_serial_port__ROAMADOME_SERIAL_PORT_HPP_

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <optional>
#include <map>
#include <cstdint>

namespace ros2_roamadome
{

/**
 * @brief Observer interface for serial port events
 *
 * Implement this interface to receive notifications of parsed serial data.
 */
class ISerialObserver
{
public:
  virtual ~ISerialObserver() = default;

  /**
   * @brief Called when a position update is received from the device
   * @param degrees Position in degrees (0-359)
   * @param radians Position in radians (converted from degrees)
   */
  virtual void onPositionUpdate(uint32_t degrees, double radians) = 0;

  /**
   * @brief Called when a velocity update is received from the device
   * @param rad_per_sec Velocity in radians per second
   */
  virtual void onVelocityUpdate(double rad_per_sec) = 0;

  /**
   * @brief Called when a line is received that doesn't match known patterns
   * @param line The unhandled line content
   */
  virtual void onUnhandledLine(const std::string & line) = 0;

  /**
   * @brief Called when config data is received from the device
   * @param config Map of config key-value pairs
   */
  virtual void onConfigUpdate(const std::map<std::string, std::string> & config) = 0;

  /**
   * @brief Called when status data is received from the device
   * @param status Vector of status lines
   */
  virtual void onStatusUpdate(const std::vector<std::string> & status) = 0;
};

/**
 * @brief Encapsulates all serial communication with the Roam-A-Dome controller
 *
 * This class handles:
 * - Opening/closing serial connection
 * - Configuring serial port parameters (baud rate, termios settings)
 * - Reading data with line-based buffering
 * - Parsing position updates from device responses
 * - Notifying observers via callbacks and/or observer interface
 *
 * Zero ROS dependencies - uses only POSIX APIs and C++ standard library.
 * Can be tested independently or run outside ROS context.
 */
class RoamadomeSerialPort
{
public:
  /**
   * @brief Construct a serial port handler
   * @param port_name Device path (e.g., "/dev/ttyACM1")
   * @param exclusive_access Whether to request exclusive access to the port (default: true)
   */
  explicit RoamadomeSerialPort(const std::string & port_name, bool exclusive_access = true);

  /**
   * @brief Destructor - closes port if open
   */
  ~RoamadomeSerialPort();

  /**
   * @brief Opens the serial port device
   * @return true if successfully opened, false on error
   */
  bool open();

  /**
   * @brief Closes the serial port device
   */
  void close();

  /**
   * @brief Checks if the port is currently open
   * @return true if open, false otherwise
   */
  bool isOpen() const {return serialFd_ >= 0;}

  /**
   * @brief Configures serial port parameters (baud rate, termios settings)
   * @param baud_rate Baud rate (e.g., 38400)
   * @return true if successfully configured, false on error
   */
  bool configurePort(uint32_t baud_rate);

  /**
   * @brief Sends a command to the device
   * @param command Command string to send
   * @param append_terminator If true, append '\n' when missing (default: true)
   * @return true if successfully written, false on error
   */
  bool sendCommand(const std::string & command, bool append_terminator = true);

  /**
   * @brief Main read loop - reads from serial and processes lines
   *
   * This should be called periodically (e.g., in RoamadomeControl::read()).
   * Accumulates partial lines across calls and parses complete lines.
   * Invokes registered callbacks/observers when data is parsed or unhandled.
   *
   * @return true if read was successful (may have read 0 bytes), false on error
   */
  bool read();

  /**
   * @brief Register a callback for position updates
   * @param callback Function called as callback(degrees, radians)
   */
  void setPositionCallback(std::function<void(uint32_t, double)> callback)
  {
    positionCallback_ = callback;
  }

  /**
   * @brief Register a callback for velocity updates
   * @param callback Function called as callback(rad_per_sec)
   */
  void setVelocityCallback(std::function<void(double)> callback)
  {
    velocityCallback_ = callback;
  }

  /**
   * @brief Register a callback for unhandled lines
   * @param callback Function called as callback(line)
   */
  void setUnhandledLineCallback(std::function<void(const std::string &)> callback)
  {
    unhandledLineCallback_ = callback;
  }

  /**
   * @brief Register a callback for config updates
   * @param callback Function called as callback(config_map)
   */
  void setConfigCallback(std::function<void(const std::map<std::string, std::string> &)> callback)
  {
    configCallback_ = callback;
  }

  /**
   * @brief Register a callback for status updates
   * @param callback Function called as callback(status_lines)
   */
  void setStatusCallback(std::function<void(const std::vector<std::string> &)> callback)
  {
    statusCallback_ = callback;
  }

  /**
   * @brief Register an observer interface for notifications
   * @param observer Pointer to observer (lifetime must outlive this object)
   */
  void registerObserver(ISerialObserver * observer)
  {
    if (observer) {
      observers_.push_back(observer);
    }
  }

private:
  std::string portName_;
  int serialFd_;
  std::string lineBuffer_;

  // Parsing state
  enum class ParseState { NONE, CONFIG, STATUS };
  ParseState parseState_;
  bool exclusiveAccess_;
  std::map<std::string, std::string> currentConfig_;
  std::vector<std::string> currentStatus_;

  // Callbacks
  std::function<void(uint32_t, double)> positionCallback_;
  std::function<void(double)> velocityCallback_;
  std::function<void(const std::string &)> unhandledLineCallback_;
  std::function<void(const std::map<std::string, std::string> &)> configCallback_;
  std::function<void(const std::vector<std::string> &)> statusCallback_;

  // Observers
  std::vector<ISerialObserver *> observers_;

  /**
   * @brief Parse a line looking for "DOME POSITION: <degrees>"
   * @param line The line to parse
   * @return optional pair of (degrees, radians) if found and valid, empty if not found or invalid
   */
  std::optional<std::pair<uint32_t, double>> parsePositionLine(const std::string & line);

  /**
   * @brief Parse a config line for key=value
   * @param line The line to parse
   * @return optional pair of (key, value) if found, empty otherwise
   */
  std::optional<std::pair<std::string, std::string>> parseConfigLine(const std::string & line);

  /**
   * @brief Split data into lines and extract remainder
   * @param data New data from serial read
   * @return pair of (complete_lines, remaining_incomplete_line)
   */
  std::pair<std::vector<std::string>, std::string> splitLines(const std::string & data);

  /**
   * @brief Trim whitespace from both ends of a string
   * @param str Input string
   * @return Trimmed string
   */
  std::string trim(const std::string & str);

  /**
   * @brief Notify all registered observers and callbacks of a position update
   * @param degrees Position in degrees
   * @param radians Position in radians
   */
  void notifyPositionObservers(uint32_t degrees, double radians);

  /**
   * @brief Notify all registered observers and callbacks of a velocity update
   * @param rad_per_sec Velocity in radians per second
   */
  void notifyVelocityObservers(double rad_per_sec);

  /**
   * @brief Notify all registered observers and callbacks of an unhandled line
   * @param line The unhandled line
   */
  void notifyUnhandledLineObservers(const std::string & line);

  /**
   * @brief Notify all registered observers and callbacks of a config update
   * @param config The config map
   */
  void notifyConfigObservers(const std::map<std::string, std::string> & config);

  /**
   * @brief Notify all registered observers and callbacks of a status update
   * @param status The status lines
   */
  void notifyStatusObservers(const std::vector<std::string> & status);
};

}  // namespace ros2_roamadome

#endif  // roamadome_serial_port__ROAMADOME_SERIAL_PORT_HPP_
