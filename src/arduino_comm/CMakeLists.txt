cmake_minimum_required(VERSION 3.8)
project(arduino_comm)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # For ROS 2 Node
find_package(std_msgs REQUIRED)  # For std_msgs message types
find_package(serial REQUIRED)  # Add serial package (from the serial library)

# Include directories for both the Arduino communication and the serial library
include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}  # Include std_msgs headers
  ${serial_INCLUDE_DIRS}     # Include serial headers
)

# Add executable
add_executable(arduinocomm_node
  src/arduinocomm.cpp
  src/arduinocomm_node.cpp
)

# Link dependencies (ROS 2 packages and serial)
target_link_libraries(arduinocomm_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${serial_LIBRARY}  # Link to serial library
)

ament_target_dependencies(arduinocomm_node
  rclcpp
  std_msgs
  serial  # Add serial as a dependency
)

# Install target
install(TARGETS arduinocomm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

