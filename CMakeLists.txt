cmake_minimum_required(VERSION 3.8)
project(waypoint_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Specify the source files for both nodes
set(SOURCE_FILES
  src/waypoint_nav_node.cpp
  src/takeoff_node.cpp
)

# Create the executable for waypoint_nav_node
add_executable(waypoint_nav_node src/waypoint_nav_node.cpp)

# Link the libraries to the waypoint_nav_node executable
ament_target_dependencies(waypoint_nav_node
  rclcpp
  sensor_msgs
  nav_msgs
)

# Create the executable for takeoff_node
add_executable(takeoff_node src/takeoff_node.cpp)

# Link the libraries to the takeoff_node executable
ament_target_dependencies(takeoff_node
  rclcpp
  sensor_msgs
  nav_msgs
)

# Install the executables
install(TARGETS
  waypoint_nav_node
  takeoff_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

