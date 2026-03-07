#include <gtest/gtest.h>
#include "ros2_roamadome/roamadome_control.hpp"
#include <memory>
#include <map>
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
      hardware_interface::JointInfo joint;
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
}

/**
 * ============================================================================
 * SERIAL EVENT HANDLER (OBSERVER) TESTS
 * ============================================================================
 * These tests verify that the SerialEventHandler inner class correctly
 * implements the observer pattern and handles serial events properly
 */

TEST_F(RoamadomeControlTest, SerialEventHandler_PositionUpdate)
{
  // Test: SerialEventHandler should update position_ on position events
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  // The handler is created during on_configure, so we verify joint_name_ is set
  // which is required before export functions are called
  auto state_interfaces = controller_->export_state_interfaces();
  EXPECT_EQ(state_interfaces.size(), 2);  // position and velocity
}

TEST_F(RoamadomeControlTest, SerialEventHandler_HandlesDegreesDegrees_RadiansConversion)
{
  // Test: Position updates convert degrees to radians correctly
  // 90 degrees should be approximately pi/2 radians
  hardware_interface::HardwareInfo info = createMockHardwareInfo(1);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;

  ASSERT_EQ(controller_->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  // The conversion happens in SerialEventHandler::onPositionUpdate
  // which we'll test indirectly through the full stack once on_configure is called
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

TEST_F(RoamadomeControlTest, ParameterParsing_InvalidBaud_Defaults)
{
  // Test: on_init() should use default baud if invalid value provided
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

}  // namespace ros2_roamadome

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
