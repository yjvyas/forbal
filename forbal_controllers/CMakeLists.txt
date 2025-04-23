cmake_minimum_required(VERSION 3.8)
project(forbal_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(forbal_interfaces REQUIRED)

include_directories(include)

add_executable(forbal2 src/forbal2.cpp)
add_executable(forbal5 src/forbal5.cpp)

target_include_directories(forbal2 PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_include_directories(forbal5 PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_compile_features(forbal2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(forbal5 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(forbal2 
rclcpp
rclcpp_action
control_msgs
geometry_msgs
sensor_msgs
trajectory_msgs
visualization_msgs
forbal_interfaces
)

ament_target_dependencies(forbal5 
rclcpp
rclcpp_action
control_msgs
geometry_msgs
sensor_msgs
trajectory_msgs
visualization_msgs
forbal_interfaces
orocos_kdl_vendor
kdl_parser
urdf
)

install(TARGETS 
    forbal2
    forbal5
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY 
  config
  include/
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
