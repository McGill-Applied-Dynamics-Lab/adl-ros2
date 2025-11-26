cmake_minimum_required(VERSION 3.8)
project(acoustic_sensing)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AcousticPacket.msg"
  DEPENDENCIES std_msgs
)

add_executable(acoustic_receiver_node src/acoustic_receiver_node.cpp)

ament_target_dependencies(acoustic_receiver_node
  rclcpp
  std_msgs
)

rosidl_target_interfaces(acoustic_receiver_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS acoustic_receiver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_dependencies(rosidl_default_runtime)



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

ament_package()

install(
  PROGRAMS
    scripts/acoustic_plot_receiver.py
    scripts/acoustic_plot_2receiver.py
  DESTINATION lib/${PROJECT_NAME}
)