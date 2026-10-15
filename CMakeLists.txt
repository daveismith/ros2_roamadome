cmake_minimum_required(VERSION 3.8)
project(ros2_roamadome)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(astromech_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# build library
add_library(ros2_roamadome
  src/roamadome_control.cpp
  src/device_parameter_specs.cpp
  src/roamadome_serial_port.cpp
  src/parameter_parser.cpp
  src/firmware_line_classifier.cpp
  src/dome_diagnostics.cpp
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
  std_srvs
  astromech_msgs
  diagnostic_msgs
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

# Bench diagnostics (docs/dome_diagnostics_plan.md, phase 5): the Python package
# ros2_roamadome.bench (analysis without ROS, the roamadome_bench node) and its
# executables, so `ros2 run ros2_roamadome roamadome_bench` works.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/roamadome_bench
  scripts/roamadome_bench_report
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

  # Unit tests for PositionTracker (header-only, no ROS dependency)
  ament_add_gtest(test_position_tracker test/test_position_tracker.cpp)
  target_include_directories(test_position_tracker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # Unit tests for the firmware console line classifier
  ament_add_gtest(test_firmware_line_classifier test/test_firmware_line_classifier.cpp)
  target_include_directories(test_firmware_line_classifier PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_firmware_line_classifier ros2_roamadome)

  # Unit tests for the diagnostics state and status builder
  ament_add_gtest(test_dome_diagnostics test/test_dome_diagnostics.cpp)
  target_include_directories(test_dome_diagnostics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_dome_diagnostics ros2_roamadome)

  # Unit tests for DeviceParameterSpecs
  ament_add_gtest(test_device_parameter_specs test/test_device_parameter_specs.cpp)
  target_include_directories(test_device_parameter_specs PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_device_parameter_specs ros2_roamadome)

  # Unit tests for DeviceParameterRegistry config parsing
  ament_add_gtest(
    test_device_parameter_registry_parse
    test/test_device_parameter_registry_parse.cpp)
  target_include_directories(test_device_parameter_registry_parse PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_device_parameter_registry_parse ros2_roamadome)

  # Bench analysis (pure Python, must not need rclpy)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bench_analysis test/test_bench_analysis.py)
  ament_add_pytest_test(test_bench_control test/test_bench_control.py)
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
  std_srvs
  astromech_msgs
  diagnostic_msgs
)

ament_package()
