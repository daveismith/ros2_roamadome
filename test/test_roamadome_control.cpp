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
    info.hardware_parameters["baud_sweep_sleep_ms"] = "0";

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

  rcl_interfaces::msg::SetParametersResult setDeviceParametersForTest(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    return controller_->onParameterChange(parameters);
  }

  std::vector<rclcpp::Parameter> filterChangedParametersForTest(
    const rclcpp::Node::SharedPtr & node,
    const std::vector<rclcpp::Parameter> & desired_parameters)
  {
    return RoamadomeControl::filterChangedParameters(node, desired_parameters);
  }

  void setLifecycleActiveForTest(bool active)
  {
    controller_->lifecycleActive_.store(active);
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
      tv.tv_usec = 5000;

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

class FirmwareThreadGuard
{
public:
  FirmwareThreadGuard(std::atomic<bool> * running, std::thread * worker)
  : running_(running), worker_(worker)
  {
  }

  ~FirmwareThreadGuard()
  {
    if (running_) {
      running_->store(false);
    }

    if (worker_ && worker_->joinable()) {
      worker_->join();
    }
  }

private:
  std::atomic<bool> * running_;
  std::thread * worker_;
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

TEST_F(RoamadomeControlTest, DeviceParameterSync_SkipsUnchangedSnapshots)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  auto node = std::make_shared<rclcpp::Node>("device_parameter_sync_test");
  node->declare_parameter("device.auto_mode", false);
  node->declare_parameter("device.home_mode", false);

  const std::vector<rclcpp::Parameter> initial_snapshot = {
    rclcpp::Parameter("device.auto_mode", true),
    rclcpp::Parameter("device.home_mode", false)
  };
  const auto first_changed = filterChangedParametersForTest(node, initial_snapshot);
  ASSERT_EQ(first_changed.size(), 1u);
  EXPECT_EQ(first_changed[0].get_name(), "device.auto_mode");

  auto first_results = node->set_parameters(first_changed);
  ASSERT_EQ(first_results.size(), 1u);
  EXPECT_TRUE(first_results[0].successful);

  const auto unchanged = filterChangedParametersForTest(node, initial_snapshot);
  EXPECT_TRUE(unchanged.empty());

  const std::vector<rclcpp::Parameter> updated_snapshot = {
    rclcpp::Parameter("device.auto_mode", true),
    rclcpp::Parameter("device.home_mode", true)
  };
  const auto second_changed = filterChangedParametersForTest(node, updated_snapshot);
  ASSERT_EQ(second_changed.size(), 1u);
  EXPECT_EQ(second_changed[0].get_name(), "device.home_mode");

  auto second_results = node->set_parameters(second_changed);
  ASSERT_EQ(second_results.size(), 1u);
  EXPECT_TRUE(second_results[0].successful);

  bool auto_mode = false;
  bool home_mode = false;
  ASSERT_TRUE(node->get_parameter("device.auto_mode", auto_mode));
  ASSERT_TRUE(node->get_parameter("device.home_mode", home_mode));
  EXPECT_TRUE(auto_mode);
  EXPECT_TRUE(home_mode);
}

TEST_F(RoamadomeControlTest, OnParameterChange_RejectsReadOnlyDeviceParameter)
{
  const auto result = setDeviceParametersForTest(
    {rclcpp::Parameter("device.home_pos", static_cast<int64_t>(90))});

  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("read-only"), std::string::npos);
}

TEST_F(RoamadomeControlTest, OnParameterChange_WritableParameterRequiresActiveHardware)
{
  const auto result = setDeviceParametersForTest(
    {rclcpp::Parameter("device.auto_mode", true)});

  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("hardware interface is inactive"), std::string::npos);
}

TEST_F(RoamadomeControlTest, OnParameterChange_IgnoresNonDeviceParameter)
{
  const auto result = setDeviceParametersForTest(
    {rclcpp::Parameter("some_other_param", 123)});

  EXPECT_TRUE(result.successful);
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
  std::thread firmware([&]() {
      std::string line;
      while (running.load()) {
        if (!pty.readLine(&line, 100)) {
          continue;
        }

        if (!line.empty() && '#' == line.front()) {
          std::lock_guard<std::mutex> lock(commands_mutex);
          seen_commands.push_back(line);
        } else if (line.rfind("DPCONFIG", 0) == 0) {
          // PTY can occasionally drop the leading '#'; normalize for assertions.
          std::lock_guard<std::mutex> lock(commands_mutex);
          seen_commands.push_back("#" + line);
        }

        if (line.rfind("#DPCONFIG", 0) == 0 || line.rfind("DPCONFIG", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPCONFIG\"\n"
            "AutoSafety=0\n"
            "AutoMode=0\n"
            "HomeMode=0\n");
        } else if (line.rfind("#DPSTATUS", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPSTATUS\"\n"
            "Auto Safety Disengaged\n"
            "ready\n");
        } else if (line.rfind("#DPREPORT", 0) == 0) {
          pty.writeText("PROCESS: \"" + line + "\"\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
          pty.writeText("PROCESS: \"#DPINVALID\"\nInvalid\n");
        }
      }
    });
  FirmwareThreadGuard firmware_guard(&running, &firmware);

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
  info.hardware_parameters["startup_setup_timeout_ms"] = "200";
  info.hardware_parameters["startup_report_timeout_ms"] = "200";
  info.hardware_parameters["startup_retries"] = "1";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::vector<std::string> seen_commands;
  std::mutex commands_mutex;
  bool first_config = true;
  std::thread firmware([&]() {
      std::string line;
      while (running.load()) {
        if (!pty.readLine(&line, 100)) {
          continue;
        }

        if (!line.empty() && '#' == line.front()) {
          std::lock_guard<std::mutex> lock(commands_mutex);
          seen_commands.push_back(line);
        } else if (line.rfind("DPCONFIG", 0) == 0) {
          std::lock_guard<std::mutex> lock(commands_mutex);
          seen_commands.push_back("#" + line);
        }

        if (line.rfind("#DPCONFIG", 0) == 0 || line.rfind("DPCONFIG", 0) == 0) {
          if (first_config) {
            pty.writeText(
              "PROCESS: \"#DPCONFIG\"\n"
              "AutoSafety=1\n"
              "AutoMode=0\n"
              "HomeMode=0\n");
            first_config = false;
          } else {
            pty.writeText(
              "PROCESS: \"#DPCONFIG\"\n"
              "AutoSafety=0\n"
              "AutoMode=0\n"
              "HomeMode=0\n");
          }
        } else if (line.rfind("#DPSETUP", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPSETUP\"\n"
            "SPEED: 50\n"
            "GOOD MAX SPEED: 80\n"
            "Restore Dome Settings\n"
            "Write Settings\n"
            "Updated\n");
        } else if (line.rfind("#DPAUTOSAFETY0", 0) == 0) {
          pty.writeText("PROCESS: \"#DPAUTOSAFETY0\"\n");
        } else if (line.rfind("#DPSTATUS", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPSTATUS\"\n"
            "Auto Safety Disengaged\n"
            "ready\n");
        } else if (line.rfind("#DPREPORT", 0) == 0) {
          pty.writeText("PROCESS: \"" + line + "\"\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
          pty.writeText("PROCESS: \"#DPINVALID\"\nInvalid\n");
        }
      }
    });
  FirmwareThreadGuard firmware_guard(&running, &firmware);

  rclcpp_lifecycle::State previous_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    "inactive");
  auto result = controller_->on_configure(previous_state);

  running.store(false);
  firmware.join();

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  // Verify full sequence: SETUP → AUTOSAFETY0 → verify CONFIG
  int setup_index = -1;
  int autosafety0_index = -1;
  int second_config_index = -1;
  bool found_first_config = false;
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    for (size_t idx = 0; idx < seen_commands.size(); ++idx) {
      if (seen_commands[idx].rfind("#DPCONFIG", 0) == 0) {
        if (!found_first_config) {
          found_first_config = true;  // Skip first CONFIG_INITIAL
        } else if (second_config_index < 0) {
          second_config_index = static_cast<int>(idx);  // This is VERIFY_AUTOSAFETY
        }
      }
      if (setup_index < 0 && seen_commands[idx].rfind("#DPSETUP", 0) == 0) {
        setup_index = static_cast<int>(idx);
      }
      if (autosafety0_index < 0 && seen_commands[idx].rfind("#DPAUTOSAFETY0", 0) == 0) {
        autosafety0_index = static_cast<int>(idx);
      }
    }
  }

  EXPECT_GE(setup_index, 0);
  EXPECT_GE(autosafety0_index, 0);
  EXPECT_GE(second_config_index, 0);
  EXPECT_LT(setup_index, autosafety0_index);
  EXPECT_LT(autosafety0_index, second_config_index);
}

TEST_F(RoamadomeControlTest, ConfigureStateMachine_FailsOnProbeTimeout)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["startup_default_timeout_ms"] = "75";
  info.hardware_parameters["startup_setup_timeout_ms"] = "200";
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

        if (line.rfind("#DPCONFIG", 0) == 0 || line.rfind("DPCONFIG", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPCONFIG\"\n"
            "AutoSafety=1\n"
            "AutoMode=0\n"
            "HomeMode=0\n");
        } else if (line.rfind("#DPSETUP", 0) == 0) {
          setup_started = true;
        } else if (line.rfind("#DPINVALID", 0) == 0) {
          // Simulate a setup-probe hang: config probe succeeds, setup probe gets no response.
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
  int first_config_index = -1;
  int config_after_setup_index = -1;
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    for (size_t idx = 0; idx < seen_commands.size(); ++idx) {
      const bool is_config = seen_commands[idx].rfind("#DPCONFIG", 0) == 0;
      if (first_config_index < 0 && is_config) {
        first_config_index = static_cast<int>(idx);
      }
      if (setup_index < 0 && seen_commands[idx].rfind("#DPSETUP", 0) == 0) {
        setup_index = static_cast<int>(idx);
      }
      if (setup_index >= 0 && config_after_setup_index < 0 && is_config &&
        static_cast<int>(idx) > setup_index)
      {
        config_after_setup_index = static_cast<int>(idx);
      }
    }
  }

  EXPECT_GE(first_config_index, 0);
  EXPECT_GE(setup_index, 0);
  EXPECT_LT(first_config_index, setup_index);
  EXPECT_EQ(config_after_setup_index, -1);
}

/**
 * ============================================================================
 * NEW PARAMETER PARSING TESTS
 * ============================================================================
 */

TEST_F(RoamadomeControlTest, ParameterParsing_AutoMode_True)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["auto_mode"] = "true";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(RoamadomeControlTest, ParameterParsing_AutoMode_False)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["auto_mode"] = "false";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(RoamadomeControlTest, ParameterParsing_AutoMode_DefaultsToFalse)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  // Don't set auto_mode parameter

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(RoamadomeControlTest, ParameterParsing_HomeMode_True)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["home_mode"] = "true";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(RoamadomeControlTest, ParameterParsing_HomeMode_False)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["home_mode"] = "false";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(RoamadomeControlTest, ParameterParsing_BothModesTrue)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["auto_mode"] = "true";
  info.hardware_parameters["home_mode"] = "true";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(RoamadomeControlTest, ParameterParsing_StartupLogLevel_Debug)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["startup_log_level"] = "debug";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(RoamadomeControlTest, ParameterParsing_StartupLogLevel_Info)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["startup_log_level"] = "info";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(RoamadomeControlTest, ParameterParsing_StartupLogLevel_InvalidFails)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["startup_log_level"] = "invalid_level";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(RoamadomeControlTest, ParameterParsing_AutoMode_InvalidFails)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["auto_mode"] = "tru";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(RoamadomeControlTest, ParameterParsing_HomeMode_InvalidFails)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["home_mode"] = "fals";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(RoamadomeControlTest, ParameterParsing_UnsupportedSerialBaud_Fails)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_baud"] = "57600";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(RoamadomeControlTest, ParameterParsing_InvalidStartupTimeout_Fails)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["startup_default_timeout_ms"] = "0";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(RoamadomeControlTest, ParameterParsing_InvalidSerialSectionFlushTimeout_Fails)
{
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_section_flush_timeout_ms"] = "0";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  auto result = controller_->on_init(params);

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

/**
 * ============================================================================
 * NEW STARTUP SEQUENCE TESTS - AutoSafety Already Disabled
 * ============================================================================
 */

TEST_F(RoamadomeControlTest, StartupSequence_AutoSafetyAlreadyDisabled_SkipsSetup)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["startup_default_timeout_ms"] = "200";
  info.hardware_parameters["startup_report_timeout_ms"] = "200";
  info.hardware_parameters["startup_setup_timeout_ms"] = "200";
  info.hardware_parameters["startup_retries"] = "1";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::vector<std::string> seen_commands;
  std::mutex commands_mutex;

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

        if (line.rfind("#DPCONFIG", 0) == 0 || line.rfind("DPCONFIG", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPCONFIG\"\n"
            "AutoSafety=0\n"
            "AutoMode=0\n"
            "HomeMode=0\n");
        } else if (line.rfind("#DPSTATUS", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPSTATUS\"\n"
            "Auto Safety Disengaged\n"
            "ready\n");
        } else if (line.rfind("#DPREPORT", 0) == 0) {
          pty.writeText("PROCESS: \"" + line + "\"\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
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

  // Verify SETUP was not called
  bool setup_seen = false;
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    for (const std::string & cmd : seen_commands) {
      if (cmd.rfind("#DPSETUP", 0) == 0) {
        setup_seen = true;
        break;
      }
    }
  }
  EXPECT_FALSE(setup_seen);
}

/**
 * ============================================================================
 * NEW STARTUP SEQUENCE TESTS - AutoSafety Needs Disabling
 * ============================================================================
 */

TEST_F(RoamadomeControlTest, StartupSequence_AutoSafetyEnabled_RunsFullSequence)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["startup_default_timeout_ms"] = "200";
  info.hardware_parameters["startup_setup_timeout_ms"] = "500";
  info.hardware_parameters["startup_report_timeout_ms"] = "200";
  info.hardware_parameters["startup_retries"] = "1";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::vector<std::string> seen_commands;
  std::mutex commands_mutex;
  bool first_config = true;

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

        if (line.rfind("#DPCONFIG", 0) == 0 || line.rfind("DPCONFIG", 0) == 0) {
          if (first_config) {
            pty.writeText(
              "PROCESS: \"#DPCONFIG\"\n"
              "AutoSafety=1\n"
              "AutoMode=0\n"
              "HomeMode=0\n");
            first_config = false;
          } else {
            pty.writeText(
              "PROCESS: \"#DPCONFIG\"\n"
              "AutoSafety=0\n"
              "AutoMode=0\n"
              "HomeMode=0\n");
          }
        } else if (line.rfind("#DPSETUP", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPSETUP\"\n"
            "SPEED: 50\n"
            "GOOD MAX SPEED: 80\n"
            "Restore Dome Settings\n"
            "Write Settings\n"
            "Updated\n");
        } else if (line.rfind("#DPAUTOSAFETY0", 0) == 0) {
          pty.writeText("PROCESS: \"#DPAUTOSAFETY0\"\n");
        } else if (line.rfind("#DPSTATUS", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPSTATUS\"\n"
            "Auto Safety Disengaged\n"
            "ready\n");
        } else if (line.rfind("#DPREPORT", 0) == 0) {
          pty.writeText("PROCESS: \"" + line + "\"\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
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

  // Verify full sequence was called in correct order
  int setup_index = -1;
  int autosafety0_index = -1;
  int verify_config_index = -1;
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    for (size_t idx = 0; idx < seen_commands.size(); ++idx) {
      if (setup_index < 0 && seen_commands[idx].rfind("#DPSETUP", 0) == 0) {
        setup_index = static_cast<int>(idx);
      }
      if (autosafety0_index < 0 && seen_commands[idx].rfind("#DPAUTOSAFETY0", 0) == 0) {
        autosafety0_index = static_cast<int>(idx);
      }
      if (verify_config_index < 0 && autosafety0_index >= 0 &&
        seen_commands[idx].rfind("#DPCONFIG", 0) == 0)
      {
        verify_config_index = static_cast<int>(idx);
      }
    }
  }

  EXPECT_GE(setup_index, 0);
  EXPECT_GE(autosafety0_index, 0);
  EXPECT_GE(verify_config_index, 0);
  EXPECT_LT(setup_index, autosafety0_index);
  EXPECT_LT(autosafety0_index, verify_config_index);
}

TEST_F(RoamadomeControlTest, StartupSequence_AutoSafetyVerificationFails)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["startup_default_timeout_ms"] = "200";
  info.hardware_parameters["startup_setup_timeout_ms"] = "500";
  info.hardware_parameters["startup_report_timeout_ms"] = "200";
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

        if (line.rfind("#DPCONFIG", 0) == 0 || line.rfind("DPCONFIG", 0) == 0) {
          // Always return AutoSafety=1 (failed to disable)
          pty.writeText(
            "PROCESS: \"#DPCONFIG\"\n"
            "AutoSafety=1\n"
            "AutoMode=0\n"
            "HomeMode=0\n");
        } else if (line.rfind("#DPSETUP", 0) == 0) {
          pty.writeText("PROCESS: \"#DPSETUP\"\nUpdated\n");
        } else if (line.rfind("#DPAUTOSAFETY0", 0) == 0) {
          pty.writeText("PROCESS: \"#DPAUTOSAFETY0\"\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
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

  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

/**
 * ============================================================================
 * NEW AUTOMODE CONFIGURATION TESTS
 * ============================================================================
 */

TEST_F(RoamadomeControlTest, StartupSequence_AutoMode_NeedsEnabling)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["auto_mode"] = "true";  // Want it enabled
  info.hardware_parameters["startup_default_timeout_ms"] = "200";
  info.hardware_parameters["startup_setup_timeout_ms"] = "200";
  info.hardware_parameters["startup_report_timeout_ms"] = "200";
  info.hardware_parameters["startup_retries"] = "1";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::vector<std::string> seen_commands;
  std::mutex commands_mutex;

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

        if (line.rfind("#DPCONFIG", 0) == 0 || line.rfind("DPCONFIG", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPCONFIG\"\n"
            "AutoSafety=0\n"
            "AutoMode=0\n"  // Currently disabled
            "HomeMode=0\n");
        } else if (line.rfind("#DPSTATUS", 0) == 0) {
          pty.writeText("PROCESS: \"#DPSTATUS\"\nAuto Safety Disengaged\nready\n");
        } else if (line.rfind("#DPAUTO", 0) == 0) {
          pty.writeText("PROCESS: \"" + line + "\"\n");
        } else if (line.rfind("#DPREPORT", 0) == 0) {
          pty.writeText("PROCESS: \"" + line + "\"\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
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

  // Verify #DPAUTO1 was sent
  bool auto1_seen = false;
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    for (const std::string & cmd : seen_commands) {
      if (cmd == "#DPAUTO1") {
        auto1_seen = true;
        break;
      }
    }
  }
  EXPECT_TRUE(auto1_seen);
}

TEST_F(RoamadomeControlTest, StartupSequence_AutoMode_AlreadyMatches_SkipsSetCommand)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["auto_mode"] = "true";  // Want it enabled
  info.hardware_parameters["startup_default_timeout_ms"] = "200";
  info.hardware_parameters["startup_setup_timeout_ms"] = "200";
  info.hardware_parameters["startup_report_timeout_ms"] = "200";
  info.hardware_parameters["startup_retries"] = "1";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::vector<std::string> seen_commands;
  std::mutex commands_mutex;

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

        if (line.rfind("#DPCONFIG", 0) == 0 || line.rfind("DPCONFIG", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPCONFIG\"\n"
            "AutoSafety=0\n"
            "AutoMode=1\n"  // Already enabled
            "HomeMode=0\n");
        } else if (line.rfind("#DPSTATUS", 0) == 0) {
          pty.writeText("PROCESS: \"#DPSTATUS\"\nAuto Safety Disengaged\nready\n");
        } else if (line.rfind("#DPREPORT", 0) == 0) {
          pty.writeText("PROCESS: \"" + line + "\"\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
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

  // Verify #DPAUTO was NOT sent
  bool auto_seen = false;
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    for (const std::string & cmd : seen_commands) {
      if (cmd.rfind("#DPAUTO", 0) == 0) {
        auto_seen = true;
        break;
      }
    }
  }
  EXPECT_FALSE(auto_seen);
}

/**
 * ============================================================================
 * NEW HOMEMODE CONFIGURATION TESTS
 * ============================================================================
 */

TEST_F(RoamadomeControlTest, StartupSequence_HomeMode_NeedsEnabling)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["home_mode"] = "true";  // Want it enabled
  info.hardware_parameters["startup_default_timeout_ms"] = "200";
  info.hardware_parameters["startup_setup_timeout_ms"] = "200";
  info.hardware_parameters["startup_report_timeout_ms"] = "200";
  info.hardware_parameters["startup_retries"] = "1";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::vector<std::string> seen_commands;
  std::mutex commands_mutex;

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

        if (line.rfind("#DPCONFIG", 0) == 0 || line.rfind("DPCONFIG", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPCONFIG\"\n"
            "AutoSafety=0\n"
            "AutoMode=0\n"
            "HomeMode=0\n");  // Currently disabled
        } else if (line.rfind("#DPSTATUS", 0) == 0) {
          pty.writeText("PROCESS: \"#DPSTATUS\"\nAuto Safety Disengaged\nready\n");
        } else if (line.rfind("#DPHOME", 0) == 0) {
          pty.writeText("PROCESS: \"" + line + "\"\n");
        } else if (line.rfind("#DPREPORT", 0) == 0) {
          pty.writeText("PROCESS: \"" + line + "\"\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
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

  // Verify #DPHOME1 was sent
  bool home1_seen = false;
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    for (const std::string & cmd : seen_commands) {
      if (cmd == "#DPHOME1") {
        home1_seen = true;
        break;
      }
    }
  }
  EXPECT_TRUE(home1_seen);
}

TEST_F(RoamadomeControlTest, StartupSequence_HomeMode_AlreadyMatches_SkipsSetCommand)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["home_mode"] = "false";  // Want it disabled (default)
  info.hardware_parameters["startup_default_timeout_ms"] = "200";
  info.hardware_parameters["startup_setup_timeout_ms"] = "200";
  info.hardware_parameters["startup_report_timeout_ms"] = "200";
  info.hardware_parameters["startup_retries"] = "1";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::vector<std::string> seen_commands;
  std::mutex commands_mutex;

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

        if (line.rfind("#DPCONFIG", 0) == 0 || line.rfind("DPCONFIG", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPCONFIG\"\n"
            "AutoSafety=0\n"
            "AutoMode=0\n"
            "HomeMode=0\n");  // Already disabled
        } else if (line.rfind("#DPSTATUS", 0) == 0) {
          pty.writeText("PROCESS: \"#DPSTATUS\"\nAuto Safety Disengaged\nready\n");
        } else if (line.rfind("#DPREPORT", 0) == 0) {
          pty.writeText("PROCESS: \"" + line + "\"\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
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

  // Verify #DPHOME was NOT sent
  bool home_seen = false;
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    for (const std::string & cmd : seen_commands) {
      if (cmd.rfind("#DPHOME", 0) == 0) {
        home_seen = true;
        break;
      }
    }
  }
  EXPECT_FALSE(home_seen);
}

/**
 * ============================================================================
 * RUNTIME SEND-QUEUE ACK FLOW TESTS
 * ============================================================================
 */

TEST_F(RoamadomeControlTest, RuntimeQueue_ParameterUpdateAck_TriggersConfigRefreshAfterUpdated)
{
  PseudoTerminal pty;
  ASSERT_TRUE(pty.valid());

  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  info.hardware_parameters["serial_port"] = pty.slavePath();
  info.hardware_parameters["startup_default_timeout_ms"] = "200";
  info.hardware_parameters["startup_report_timeout_ms"] = "200";
  info.hardware_parameters["startup_setup_timeout_ms"] = "200";
  info.hardware_parameters["startup_retries"] = "1";

  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  std::atomic<bool> running{true};
  std::vector<std::string> seen_commands;
  std::mutex commands_mutex;

  std::thread firmware([&]() {
      std::string line;
      while (running.load()) {
        if (!pty.readLine(&line, 100)) {
          continue;
        }

        if (!line.empty() && '#' == line.front()) {
          std::lock_guard<std::mutex> lock(commands_mutex);
          seen_commands.push_back(line);
        } else if (line.rfind("DP", 0) == 0) {
          // PTY can occasionally drop the leading '#'; normalize for assertions.
          std::lock_guard<std::mutex> lock(commands_mutex);
          seen_commands.push_back("#" + line);
        }

        if (line.rfind("#DPCONFIG", 0) == 0 || line.rfind("DPCONFIG", 0) == 0) {
          pty.writeText(
            "PROCESS: \"#DPCONFIG\"\n"
            "AutoSafety=0\n"
            "AutoMode=0\n"
            "HomeMode=0\n");
        } else if (line.rfind("#DPSTATUS", 0) == 0) {
          pty.writeText("PROCESS: \"#DPSTATUS\"\nAuto Safety Disengaged\nready\n");
        } else if (line.rfind("#DPREPORT", 0) == 0) {
          pty.writeText("PROCESS: \"" + line + "\"\n");
        } else if (line.rfind("#DPINVALID", 0) == 0) {
          pty.writeText("PROCESS: \"#DPINVALID\"\nInvalid\n");
        }
      }
    });
  FirmwareThreadGuard firmware_guard(&running, &firmware);

  rclcpp_lifecycle::State previous_state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    "inactive");
  ASSERT_EQ(
    controller_->on_configure(previous_state),
    hardware_interface::CallbackReturn::SUCCESS);

  // Unit tests for this class run without a lifecycle node, so mark the
  // controller active via the fixture helper to exercise writable updates.
  setLifecycleActiveForTest(true);

  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    seen_commands.clear();
  }

  auto set_result = setDeviceParametersForTest({rclcpp::Parameter("device.auto_mode", true)});
  ASSERT_TRUE(set_result.successful) << set_result.reason;

  const rclcpp::Time now(0, 0, RCL_SYSTEM_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.02);

  ASSERT_EQ(controller_->write(now, period), hardware_interface::return_type::OK);

  auto wait_for_command = [&](const std::string & exact, int timeout_ms) -> bool {
      const auto start = std::chrono::steady_clock::now();
      while (true) {
        {
          std::lock_guard<std::mutex> lock(commands_mutex);
          for (const auto & cmd : seen_commands) {
            if (cmd == exact) {
              return true;
            }
          }
        }

        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > timeout_ms) {
          return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    };

  ASSERT_TRUE(wait_for_command("#DPAUTO1", 500));

  pty.writeText("Write Settings\n");
  ASSERT_EQ(controller_->read(now, period), hardware_interface::return_type::OK);

  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    EXPECT_EQ(
      std::count(seen_commands.begin(), seen_commands.end(), "#DPCONFIG"),
      0);
    EXPECT_EQ(
      std::count(seen_commands.begin(), seen_commands.end(), "#DPINVALID"),
      0);
  }

  pty.writeText("Updated\n");
  ASSERT_EQ(controller_->read(now, period), hardware_interface::return_type::OK);
  ASSERT_EQ(controller_->write(now, period), hardware_interface::return_type::OK);

  ASSERT_TRUE(wait_for_command("#DPCONFIG", 500));

  ASSERT_EQ(controller_->write(now, period), hardware_interface::return_type::OK);
  ASSERT_TRUE(wait_for_command("#DPINVALID", 500));

}

}  // namespace ros2_roamadome
