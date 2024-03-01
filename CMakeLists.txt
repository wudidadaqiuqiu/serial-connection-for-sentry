cmake_minimum_required(VERSION 3.8)
project(serial_connection)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(easy_robot_commands REQUIRED)
find_package(geometry_msgs REQUIRED)

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

set(connector_dependencies
  rclcpp
  easy_robot_commands
  geometry_msgs
)

add_executable(test_connection
  src/test.cpp)
target_include_directories(test_connection PRIVATE include)
ament_target_dependencies(test_connection ${connector_dependencies})

add_executable(connector src/connector_node.cpp)
target_include_directories(connector PRIVATE include)
ament_target_dependencies(connector ${connector_dependencies})


install (TARGETS
  test_connection
  connector
DESTINATION lib/${PROJECT_NAME})


ament_package()
