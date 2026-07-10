cmake_minimum_required(VERSION 3.8)
project(navi_bot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate C / C++ / Python type-support for our messages.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose2D.msg"
)

ament_package()
