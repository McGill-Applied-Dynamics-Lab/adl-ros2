cmake_minimum_required(VERSION 3.8)
project(teleop_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#! VARIABLES
# Dependencies
set(INTERFACE_DEPENDS
  std_msgs
  geometry_msgs
  builtin_interfaces
)

set(INTERFACES
  "msg/Inverse3State.msg"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(Dependency IN ITEMS ${INTERFACE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


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

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACES}
  DEPENDENCIES 
  ${INTERFACE_DEPENDS}
)

ament_package()
