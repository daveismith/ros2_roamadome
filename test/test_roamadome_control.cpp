#include <gtest/gtest.h>
#include "ros2_roamadome/roamadome_control.hpp"
#include <fcntl.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <sys/select.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ros2_roamadome
{

/**
 * @brief Test fixture for RoamadomeControl tests
 *
 * Provides utilities to set up mock HardwareInfo structures and
 * create RoamadomeControl instances for testing.
 */
class RoamadomeControlTest : public ::testing::Test
{
protected:
  RoamadomeControlTest()
  : controller_(std::make_unique<RoamadomeControl>())
  {
  }

  /**
   * @brief Create a HardwareInfo structure with specified number of joints
   * @param num_joints Number of joints to configure
   * @return HardwareInfo with the specified joint count
   */
  hardware_interface::HardwareInfo createMockHardwareInfo(size_t num_joints)
  {
    hardware_interface::HardwareInfo info;
    info.name = "RoamADome";
    info.type = "actuator";
    info.rw_rate = 50;  // 50 Hz read/write rate

    // Add hardware parameters
    info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
    info.hardware_parameters["serial_baud"] = "115200";

    // Create the specified number of joints
    for (size_t i = 0; i < num_joints; ++i) {
      // ComponentInfo represents a hardware component (joint in this case)
      hardware_interface::ComponentInfo component;
      component.name = (i == 0) ? "dome_joint" : ("dome_joint_" + std::to_string(i));
      component.type = "joint";

      // Add command interfaces
      hardware_interface::InterfaceInfo cmd_pos;
      cmd_pos.name = "position";
      component.command_interfaces.push_back(cmd_pos);

      hardware_interface::InterfaceInfo cmd_vel;
      cmd_vel.name = "velocity";
      component.command_interfaces.push_back(cmd_vel);

      // Add state interfaces
      hardware_interface::InterfaceInfo state_pos;
      state_pos.name = "position";
      component.state_interfaces.push_back(state_pos);

      hardware_interface::InterfaceInfo state_vel;
      state_vel.name = "velocity";
      component.state_interfaces.push_back(state_vel);

      info.joints.push_back(component);
    }

    return info;
  }

  std::unique_ptr<RoamadomeControl> controller_;
};

class PseudoTerminal
{
public:
  PseudoTerminal()
  {
    master_fd_ = posix_openpt(O_RDWR | O_NOCTTY);
    if (master_fd_ < 0) {
      return;
    }

    if (0 != grantpt(master_fd_) || 0 != unlockpt(master_fd_)) {
      ::close(master_fd_);
      master_fd_ = -1;
      return;
    }

    char * path = ptsname(master_fd_);
    if (nullptr == path) {
      ::close(master_fd_);
      master_fd_ = -1;
      return;
    }

    slave_path_ = path;
  }

  ~PseudoTerminal()
  {
    if (master_fd_ >= 0) {
      ::close(master_fd_);
      master_fd_ = -1;
    }
  }

  bool valid() const
  {
    return master_fd_ >= 0 && !slave_path_.empty();
  }

  const std::string & slavePath() const
  {
    return slave_path_;
  }

  bool readLine(std::string * line, int timeout_ms)
  {
    line->clear();
    const auto start = std::chrono::steady_clock::now();

    while (true) {
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(master_fd_, &readfds);

      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 100000;

      int ret = select(master_fd_ + 1, &readfds, nullptr, nullptr, &tv);
      if (ret > 0 && FD_ISSET(master_fd_, &readfds)) {
        char ch = 0;
        ssize_t n = ::read(master_fd_, &ch, 1);
        if (n > 0) {
          if ('\r' == ch) {
            continue;
          }
          if ('\n' == ch) {
            return true;
          }
          line->push_back(ch);
        }
      }

      const auto elapsed = std::chrono::steady_clock::now() - start;
      if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > timeout_ms) {
        return false;
      }
    }
  }

  void writeText(const std::string & text)
  {
    ssize_t written = ::write(master_fd_, text.c_str(), text.size());
    (void)written;
  }

private:
  int master_fd_ = -1;
  std::string slave_path_;
};

/**
 * ============================================================================
 * JOINT VALIDATION TESTS
 * ============================================================================
 * These tests verify that RoamadomeControl correctly validates joint
 * configuration during on_init()
 */

TEST_F(RoamadomeControlTest, JointValidation_SingleJoint_Success)
{
  // Test: on_init() should succeed with exactly 1 joint
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(RoamadomeControlTest, JointValidation_NoJoints_Fails)
{
  // Test: on_init() should fail with 0 joints
  hardware_interface::HardwareInfo info = createMockHardwareInfo(0);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(RoamadomeControlTest, JointValidation_MultipleJoints_Fails)
{
  // Test: on_init() should fail with 2+ joints
  hardware_interface::HardwareInfo info = createMockHardwareInfo(2);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

/**
 * ============================================================================
 * EXPORT FUNCTIONS TESTS
 * ============================================================================
 * These tests verify that export_state_interfaces() and
 * export_command_interfaces() use dynamic joint naming
 */

TEST_F(RoamadomeControlTest, ExportStateInterfaces_UsesJointName)
{
  // Test: export_state_interfaces() should use joint_name_ from URDF
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  auto state_interfaces = controller_->export_state_interfaces();

  ASSERT_EQ(state_interfaces.size(), 2);  // position and velocity
  EXPECT_EQ(state_interfaces[0].get_interface_name(), "position");
  EXPECT_EQ(state_interfaces[1].get_interface_name(), "velocity");
}

TEST_F(RoamadomeControlTest, ExportStateInterfaces_DynamicJointName)
{
  // Test: export_state_interfaces() should respect alternate joint name
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  // Override the joint name
  info.joints[0].name = "rotation_actuator";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  auto state_interfaces = controller_->export_state_interfaces();

  ASSERT_EQ(state_interfaces.size(), 2);
  EXPECT_EQ(state_interfaces[0].get_interface_name(), "position");
  EXPECT_EQ(state_interfaces[1].get_interface_name(), "velocity");
  EXPECT_EQ(state_interfaces[0].get_prefix_name(), info.joints[0].name);
  EXPECT_EQ(state_interfaces[1].get_prefix_name(), info.joints[0].name);
}

TEST_F(RoamadomeControlTest, ExportCommandInterfaces_UsesJointName)
{
  // Test: export_command_interfaces() should use joint_name_ from URDF
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  auto command_interfaces = controller_->export_command_interfaces();

  ASSERT_EQ(command_interfaces.size(), 2);  // position and velocity
  EXPECT_EQ(command_interfaces[0].get_interface_name(), "position");
  EXPECT_EQ(command_interfaces[1].get_interface_name(), "velocity");
}

TEST_F(RoamadomeControlTest, ExportCommandInterfaces_DynamicJointName)
{
  // Test: export_command_interfaces() should respect alternate joint name
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  // Override the joint name
  info.joints[0].name = "rotation_actuator";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  auto command_interfaces = controller_->export_command_interfaces();

  ASSERT_EQ(command_interfaces.size(), 2);
  EXPECT_EQ(command_interfaces[0].get_interface_name(), "position");
  EXPECT_EQ(command_interfaces[1].get_interface_name(), "velocity");
  EXPECT_EQ(command_interfaces[0].get_prefix_name(), info.joints[0].name);
  EXPECT_EQ(command_interfaces[1].get_prefix_name(), info.joints[0].name);
}

/**
 * ============================================================================
 * PARAMETER PARSING TESTS
 * ============================================================================
 * These tests verify that on_init() correctly parses hardware parameters
 */

TEST_F(RoamadomeControlTest, ParameterParsing_SerialPort_Success)
{
  // Test: on_init() should successfully parse serial_port parameter
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(RoamadomeControlTest, ParameterParsing_MissingSerialPort_Fails)
{
  // Test: on_init() should fail if serial_port parameter is missing
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters.erase("serial_port");

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(RoamadomeControlTest, ParameterParsing_InvalidBaudFormat_Fails)
{
  // Test: on_init() should fail if serial_baud has invalid format
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_baud"] = "invalid_baud";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);  // Invalid format should error
}

/**
 * ============================================================================
 * NON-HARDWARE TESTS FOR OBSERVER PATTERN
 * ============================================================================
 * These tests focus on the SerialEventHandler implementation without
 * requiring actual serial hardware
 */

TEST_F(RoamadomeControlTest, ExportsCorrectInterfaceCount)
{
  // Test: Both export functions return exactly 2 interfaces (position + velocity)
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  auto state_interfaces = controller_->export_state_interfaces();
  auto command_interfaces = controller_->export_command_interfaces();

  EXPECT_EQ(state_interfaces.size(), 2);
  EXPECT_EQ(command_interfaces.size(), 2);
}

TEST_F(RoamadomeControlTest, ExportsCorrectInterfaceTypes)
{
  // Test: State and command interfaces have correct types
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  auto state_interfaces = controller_->export_state_interfaces();
  auto command_interfaces = controller_->export_command_interfaces();

  // State interfaces
  EXPECT_EQ(state_interfaces[0].get_interface_name(), "position");
  EXPECT_EQ(state_interfaces[1].get_interface_name(), "velocity");

  // Command interfaces
  EXPECT_EQ(command_interfaces[0].get_interface_name(), "position");
  EXPECT_EQ(command_interfaces[1].get_interface_name(), "velocity");
}

TEST_F(RoamadomeControlTest, PrepareCommandModeSwitch_IgnoresSubstringInterfaceNames)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  // "position_limit" should not be treated as the position interface.
  const std::vector<std::string> start_interfaces = {
    "dome_joint/position_limit",
    "dome_joint/velocity"
  };

  EXPECT_EQ(
    controller_->prepare_command_mode_switch(start_interfaces, {}),
    hardware_interface::return_type::OK);
}

TEST_F(RoamadomeControlTest, ConfigureFailsWhenReadWriteRateIsZero)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.rw_rate = 0;

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  rclcpp_lifecycle::State previous_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    "inactive");
  EXPECT_EQ(
    controller_->on_configure(previous_state),
    hardware_interface::CallbackReturn::ERROR);
}

TEST_F(RoamadomeControlTest, ConfigureStateMachine_SkipsSetupWhenAutoSafetyNotEngaged)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["startup_default_timeout_ms"] = "200";
  info.hardware_parameters["startup_report_timeout_ms"] = "200";
  info.hardware_parameters["startup_retries"] = "1";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::vector<std::string> seen_commands;
  std::mutex commands_mutex;
  bool pending_status_payload = true;
  std::thread firmware([&]() {
      std::string line;
      while (running.load()) {
        if (!pty.readLine(&line, 100)) {
          continue;
        }

        if (!line.empty() && '#' == line.front()) {
          std::lock_guard<std::mutex> lock(commands_mutex);
          seen_commands.push_back(line);
        }

        if (line.rfind("#DPSTATUS", 0) == 0) {
          pending_status_payload = true;
        } else if (line.rfind("#DPSETUP", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPSETUP\"\n"
            "SPEED: 50\n"
            "GOOD MAX SPEED: 80\n"
            "Restore Dome Settings\n"
            "Write Settings\n"
            "Updated\n");
        } else if (line.rfind("#DPCONFIG", 0) == 0) {
          pty.writeText("PROCESS: \"#DPCONFIG\"\nfoo=bar\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
          if (pending_status_payload) {
            pty.writeText(
              "PROCESS: \"#DPSTATUS\"\n"
              "Auto Safety Disengaged\n"
              "Dome Sensor Errors: 10\n"
              "WiFi Enabled\n"
              "Remote Enabled\n");
            pending_status_payload = false;
          }
          pty.writeText("PROCESS: \"#DPINVALID\"\nInvalid\n");
        }
      }
    });

  rclcpp_lifecycle::State previous_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    "inactive");
  auto result = controller_->on_configure(previous_state);

  running.store(false);
  firmware.join();

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  bool setup_seen = false;
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    for (const std::string & command : seen_commands) {
      if (command.rfind("#DPSETUP", 0) == 0) {
        setup_seen = true;
        break;
      }
    }
  }

  EXPECT_FALSE(setup_seen);
}

TEST_F(RoamadomeControlTest, ConfigureStateMachine_RunsSetupWhenAutoSafetyEngaged)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["startup_default_timeout_ms"] = "200";
  info.hardware_parameters["startup_report_timeout_ms"] = "200";
  info.hardware_parameters["startup_retries"] = "1";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::vector<std::string> seen_commands;
  std::mutex commands_mutex;
  bool pending_status_payload = true;
  std::thread firmware([&]() {
      std::string line;
      while (running.load()) {
        if (!pty.readLine(&line, 100)) {
          continue;
        }

        if (!line.empty() && '#' == line.front()) {
          std::lock_guard<std::mutex> lock(commands_mutex);
          seen_commands.push_back(line);
        }

        if (line.rfind("#DPSTATUS", 0) == 0) {
          pending_status_payload = true;
        } else if (line.rfind("#DPSETUP", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPSETUP\"\n"
            "SPEED: 50\n"
            "NEW DOME MODE\n"
            "INVERTED prev:14 current:15\n"
            "REACHED TARGET: 14\n"
            "Angular velocity: 65.26 cm/s\n"
            "SPEED: 60\n"
            "INVERTED prev:16 current:17\n"
            "[DOME SENSOR] ERROR READING POSITION\n"
            "[DOME SENSOR] ERROR READING POSITION\n"
            "[DOME SENSOR] ERROR READING POSITION\n"
            "REACHED TARGET: 14\n"
            "Angular velocity: 84.89 cm/s\n"
            "SPEED: 70\n"
            "INVERTED prev:17 current:19\n"
            "[DOME SENSOR] ERROR READING POSITION\n"
            "[DOME SENSOR] ERROR READING POSITION\n"
            "REACHED TARGET: 14\n"
            "Angular velocity: 99.98 cm/s\n"
            "SPEED: 80\n"
            "INVERTED prev:17 current:18\n"
            "REACHED TARGET: 14\n"
            "Angular velocity: 114.58 cm/s\n"
            "GOOD MAX SPEED: 80\n"
            "Restore Dome Settings\n"
            "Write Settings\n"
            "Updated\n");
        } else if (line.rfind("#DPCONFIG", 0) == 0) {
          pty.writeText("PROCESS: \"#DPCONFIG\"\nfoo=bar\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
          if (pending_status_payload) {
            pty.writeText(
              "PROCESS: \"#DPSTATUS\"\n"
              "Auto Safety Engaged\n"
              "No Dome Sensor Errors\n"
              "WiFi Enabled\n"
              "Remote Enabled\n");
            pending_status_payload = false;
          }
          pty.writeText("PROCESS: \"#DPINVALID\"\nInvalid\n");
        }
      }
    });

  rclcpp_lifecycle::State previous_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    "inactive");
  auto result = controller_->on_configure(previous_state);

  running.store(false);
  firmware.join();

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  int setup_index = -1;
  int config_index = -1;
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    for (size_t idx = 0; idx < seen_commands.size(); ++idx) {
      if (setup_index < 0 && seen_commands[idx].rfind("#DPSETUP", 0) == 0) {
        setup_index = static_cast<int>(idx);
      }
      if (config_index < 0 && seen_commands[idx].rfind("#DPCONFIG", 0) == 0) {
        config_index = static_cast<int>(idx);
      }
    }
  }

  EXPECT_GE(setup_index, 0);
  EXPECT_GE(config_index, 0);
  EXPECT_LT(setup_index, config_index);
}

TEST_F(RoamadomeControlTest, ConfigureStateMachine_FailsOnProbeTimeout)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["startup_default_timeout_ms"] = "75";
  info.hardware_parameters["startup_report_timeout_ms"] = "75";
  info.hardware_parameters["startup_retries"] = "0";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::thread firmware([&]() {
      std::string line;
      while (running.load()) {
        if (!pty.readLine(&line, 100)) {
          continue;
        }

        if (line.rfind("#DPSTATUS", 0) == 0) {
          pty.writeText("PROCESS: \"#DPSTATUS\"\nready\n");
        }
      }
    });

  rclcpp_lifecycle::State previous_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    "inactive");
  auto result = controller_->on_configure(previous_state);

  running.store(false);
  firmware.join();

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(RoamadomeControlTest, ConfigureStateMachine_UsesConfigurableSetupTimeout)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["startup_default_timeout_ms"] = "1000";
  info.hardware_parameters["startup_setup_timeout_ms"] = "75";
  info.hardware_parameters["startup_report_timeout_ms"] = "1000";
  info.hardware_parameters["startup_retries"] = "0";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::vector<std::string> seen_commands;
  std::mutex commands_mutex;
  bool pending_status_payload = true;
  bool setup_started = false;
  std::thread firmware([&]() {
      std::string line;
      while (running.load()) {
        if (!pty.readLine(&line, 100)) {
          continue;
        }

        if (!line.empty() && '#' == line.front()) {
          std::lock_guard<std::mutex> lock(commands_mutex);
          seen_commands.push_back(line);
        }

        if (line.rfind("#DPSTATUS", 0) == 0) {
          pending_status_payload = true;
        } else if (line.rfind("#DPSETUP", 0) == 0) {
          setup_started = true;
        } else if (line.rfind("#DPINVALID", 0) == 0) {
          if (pending_status_payload) {
            pty.writeText(
              "PROCESS: \"#DPSTATUS\"\n"
              "Auto Safety Engaged\n"
              "No Dome Sensor Errors\n"
              "WiFi Enabled\n"
              "Remote Enabled\n");
            pending_status_payload = false;
          }

          // Simulate a setup-probe hang: status probe succeeds, setup probe gets no response.
          if (!setup_started) {
            pty.writeText("PROCESS: \"#DPINVALID\"\nInvalid\n");
          }
        }
      }
    });

  rclcpp_lifecycle::State previous_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    "inactive");
  auto result = controller_->on_configure(previous_state);

  running.store(false);
  firmware.join();

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);

  int setup_index = -1;
  int config_index = -1;
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    for (size_t idx = 0; idx < seen_commands.size(); ++idx) {
      if (setup_index < 0 && seen_commands[idx].rfind("#DPSETUP", 0) == 0) {
        setup_index = static_cast<int>(idx);
      }
      if (config_index < 0 && seen_commands[idx].rfind("#DPCONFIG", 0) == 0) {
        config_index = static_cast<int>(idx);
      }
    }
  }

  EXPECT_GE(setup_index, 0);
  EXPECT_EQ(config_index, -1);
}

}  // namespace ros2_roamadome
