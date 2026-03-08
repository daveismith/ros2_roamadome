cmake_minimum_required(VERSION 3.8)
project(ros2_roamadome)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# build library
add_library(ros2_roamadome
  src/roamadome_control.cpp
  src/roamadome_serial_port.cpp
  src/parameter_parser.cpp
)
add_library(ros2_roamadome::roamadome_control ALIAS ros2_roamadome)
target_compile_features(ros2_roamadome PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ros2_roamadome PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ros2_roamadome PRIVATE "ros2_roamadome_BUILDING_LIBRARY")

ament_target_dependencies(ros2_roamadome PUBLIC
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)
pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ros2_roamadome
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(roamadome_control_node src/roamadome_control_node.cpp)
target_include_directories(roamadome_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(roamadome_control_node ros2_roamadome)

install(TARGETS roamadome_control_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit tests for RoamadomeSerialPort
  ament_add_gtest(test_roamadome_serial_port test/test_roamadome_serial_port.cpp)
  target_include_directories(test_roamadome_serial_port PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_roamadome_serial_port ros2_roamadome)

  # Unit tests for RoamadomeControl
  ament_add_gtest(test_roamadome_control test/test_roamadome_control.cpp)
  target_include_directories(test_roamadome_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_roamadome_control ros2_roamadome)
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  ros2_roamadome
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

ament_package()
