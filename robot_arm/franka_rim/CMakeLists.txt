cmake_minimum_required(VERSION 3.8)
project(franka_rim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(PROJECT_DEPENDS
  rclcpp
  pluginlib
  rcpputils
  Eigen3
  Franka
  sensor_msgs
  std_msgs
  franka_msgs
  franka_hardware
  hardware_interface
)

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${PROJECT_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  franka_rim SHARED
  src/franka_robot_model.cpp
)

target_include_directories(franka_rim PUBLIC include)
ament_target_dependencies(franka_rim ${PROJECT_DEPENDS})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Add executable
add_executable(franka_rim_node src/franka_rim_node.cpp)

# Include directories
ament_target_dependencies(franka_rim_node ${PROJECT_DEPENDS})
target_include_directories(franka_rim_node PRIVATE include)
target_link_libraries(franka_rim_node franka_rim)

# Install targets
install(TARGETS franka_rim_node
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(franka_rim)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
