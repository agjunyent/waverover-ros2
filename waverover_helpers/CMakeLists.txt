cmake_minimum_required(VERSION 3.8)
project(waverover_helpers)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

install(
  DIRECTORY include/
  DESTINATION include
)

### Helpers ###
## Declare a C++ library
set(HEADER_FILES
  include/waverover_helpers/helpers.hpp
  src/helpers.cpp)
add_library(helpers src/helpers.cpp ${HEADER_FILES})
ament_target_dependencies(helpers rclcpp geometry_msgs)
ament_export_targets(helpers HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs)

install(
  TARGETS helpers
  EXPORT helpers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
