cmake_minimum_required(VERSION 3.5)
project(diffdrive_arduino)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)


# Include directories
include_directories(
  include
)

# Source files
set(SRC_FILES
  src/diffdrive_arduino.cpp
  src/arduino_comms.cpp
  src/wheel.cpp
)

# Add library
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install shared library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export the plugin
pluginlib_export_plugin_description_file(hardware_interface diffdrive_arduino_plugin.xml)

# Install plugin XML
install(
  FILES diffdrive_arduino_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Ament export
ament_export_dependencies(rclcpp hardware_interface pluginlib )
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()

