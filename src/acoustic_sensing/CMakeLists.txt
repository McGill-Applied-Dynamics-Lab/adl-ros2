cmake_minimum_required(VERSION 3.8)
project(acoustic_sensing)

# --------------------------------------------------------------------------- #
# Compiler settings
# --------------------------------------------------------------------------- #
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------- #
# Dependencies
# --------------------------------------------------------------------------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --------------------------------------------------------------------------- #
# Interfaces
# --------------------------------------------------------------------------- #
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AcousticPacket.msg"
  DEPENDENCIES std_msgs
)

# --------------------------------------------------------------------------- #
# Node
# --------------------------------------------------------------------------- #
add_executable(acoustic_receiver_node src/acoustic_receiver_node.cpp)

ament_target_dependencies(acoustic_receiver_node
  rclcpp
  std_msgs
)

# New way: link typesupport target instead of rosidl_target_interfaces()
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(acoustic_receiver_node
  ${cpp_typesupport_target}
)

# --------------------------------------------------------------------------- #
# Installation
# --------------------------------------------------------------------------- #
install(TARGETS acoustic_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/acoustic_plot_receiver.py
    scripts/acoustic_plot_2receiver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------------------------------- #
# Finalize package
# --------------------------------------------------------------------------- #
ament_package()
