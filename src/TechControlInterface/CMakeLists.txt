cmake_minimum_required(VERSION 3.8)
project(TechControlInterface)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
  sensor_msgs
  std_msgs
)
# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_library(soem soem REQUIRED PATHS /home/jintao/Projects/SOEM/install/lib)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# ros2_control plugin library
add_library(${PROJECT_NAME}
  SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/hardware/TechHardware.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/controller/TechController.cpp
)


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies( ${PROJECT_NAME}  ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface TechControlInterface.xml)
pluginlib_export_plugin_description_file(controller_interface TechControlInterface.xml)


target_link_libraries(${PROJECT_NAME} ${soem})
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  "/home/jintao/Projects/SOEM/install/include/soem"
)

install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)
install(DIRECTORY rviz/ 
  DESTINATION share/${PROJECT_NAME}/rviz
)
install(DIRECTORY urdf/ 
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()