cmake_minimum_required(VERSION 3.5)
project(control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(control_interfaces
                           "msg/PositionCommand.msg"
                           "msg/VelocityCommand.msg")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
