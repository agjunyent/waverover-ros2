cmake_minimum_required(VERSION 3.8)
project(waverover_controller)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_cpp REQUIRED)
find_package(waverover_helpers REQUIRED)

include_directories(
  include
)


add_executable(waverover_controller src/waverover_controller_node.cpp
                                    src/waverover_controller.cpp
                                    src/uart_serial_port.cpp)

target_link_libraries(waverover_controller serial_cpp::serial_cpp)

ament_target_dependencies(waverover_controller rclcpp std_msgs geometry_msgs sensor_msgs serial_cpp waverover_helpers)

install(TARGETS
  waverover_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
