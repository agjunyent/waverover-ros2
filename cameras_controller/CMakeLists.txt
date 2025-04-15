cmake_minimum_required(VERSION 3.8)
project(cameras_controller)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Python3)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(waverover_helpers REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  #MSG
  "msg/CameraFrame.msg"
  "msg/CompressedFrame.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} rosidl_typesupport_cpp)

### cameras_controller ###
## Declare a C++ library
set(HEADER_FILES
  include/cameras_controller/camera.hpp
  include/cameras_controller/camera_rgb.hpp
  include/cameras_controller/cameras_controller.hpp
  src/camera.cpp
  src/camera_rgb.cpp
  src/camera_thm.cpp
  src/cameras_controller.cpp)

add_executable(cameras_controller_node src/cameras_controller_node.cpp
                                       ${HEADER_FILES})
target_link_libraries(cameras_controller_node ${OpenCV_LIBS} "${cpp_typesupport_target}")
ament_target_dependencies(cameras_controller_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV waverover_helpers rosidl_default_runtime)

install(TARGETS
  cameras_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()