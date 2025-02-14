cmake_minimum_required(VERSION 3.8)
project(fr3_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(PROJECT_DEPENDS
  rclcpp
  pluginlib
  rcpputils
  hardware_interface
  controller_interface
  realtime_tools
  Eigen3
  franka_semantic_components
  trajectory_msgs
  franka_msgs
  geometry_msgs
)

# # Specify the required version of ros2_control
# find_package(controller_manager 4.0.0)
# # Handle the case where the required version is not found
# if(NOT controller_manager_FOUND)
#   message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
#   "Are you using the correct branch of the ros2_control_demos repository?")
# endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${PROJECT_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# from franka_ros2_example
# find_package(ament_index_cpp REQUIRED)
# find_package(controller_interface REQUIRED)
# find_package(franka_semantic_components REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(moveit_core REQUIRED)
# find_package(moveit_msgs REQUIRED)
# find_package(pinocchio REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
# find_package(std_srvs REQUIRED)

#! COMPILE
# add_executable(send_trajectory reference_generator/send_trajectory.cpp)

# ament_target_dependencies(
#   send_trajectory PUBLIC
#   ${REF_GEN_INCLUDE_DEPENDS}
# )

add_library(
  fr3_controllers
  SHARED
  src/test_controller.cpp
  src/joint_velocity_controller.cpp
  src/cartesian_pose_controller.cpp
  src/cartesian_vel_controller.cpp
)

target_compile_features(fr3_controllers PUBLIC cxx_std_17)
target_include_directories(fr3_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/fr3_controllers>
  ${EIGEN3_INCLUDE_DIRS}
)

# target_include_directories(
#         ${PROJECT_NAME}
#         PUBLIC
#         include
#         ${EIGEN3_INCLUDE_DIRS}
# )

ament_target_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_DEPENDS}
)

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface fr3_controllers.xml)


# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/fr3_controllers
)
# install(
#   DIRECTORY description/launch description/ros2_control description/urdf
#   DESTINATION share/fr3_controllers
# )
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/ros2_control_demo_example_7
# )
# install(
#     TARGETS send_trajectory
#     RUNTIME DESTINATION lib/ros2_control_demo_example_7
# )

install(
  TARGETS 
  ${PROJECT_NAME}
  EXPORT export_fr3_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

## EXPORTS
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)

ament_export_targets(export_fr3_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${PROJECT_DEPENDS} 
)
ament_package()