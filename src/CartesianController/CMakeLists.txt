cmake_minimum_required(VERSION 3.22)
project(CartesianController)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Boost
  control_msgs
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_trajectory_cache
  moveit_servo
  moveit_visual_tools
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rviz_visual_tools
  tf2_geometry_msgs
  warehouse_ros_sqlite
)

# foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()


# add_executable(${PROJECT_NAME} src/test.cpp)

# # target_include_directories(${PROJECT_NAME}
# #   PUBLIC
# #   src/include
# # )
# ament_target_dependencies(
#   ${PROJECT_NAME}
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()