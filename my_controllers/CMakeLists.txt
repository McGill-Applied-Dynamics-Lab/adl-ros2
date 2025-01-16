cmake_minimum_required(VERSION 3.8)
project(my_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  # kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
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
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#! COMPILE
# add_executable(send_trajectory reference_generator/send_trajectory.cpp)

# ament_target_dependencies(
#   send_trajectory PUBLIC
#   ${REF_GEN_INCLUDE_DEPENDS}
# )

add_library(
  my_controllers
  SHARED
  src/test_controller.cpp
)

target_compile_features(my_controllers PUBLIC cxx_std_17)
target_include_directories(my_controllers PUBLIC
  # $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/my_controllers>
)

ament_target_dependencies(
  my_controllers PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface my_controllers.xml)


# INSTALL
# install(
#   DIRECTORY hardware/include/
#   DESTINATION include/my_controllers
# )
# install(
#   DIRECTORY description/launch description/ros2_control description/urdf
#   DESTINATION share/my_controllers
# )
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/ros2_control_demo_example_7
# )
# install(
#     TARGETS send_trajectory
#     RUNTIME DESTINATION lib/ros2_control_demo_example_7
# )

install(TARGETS my_controllers
  EXPORT export_my_controllers
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
ament_export_targets(export_my_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()