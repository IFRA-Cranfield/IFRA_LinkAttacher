cmake_minimum_required(VERSION 3.16)
project(linkattacher_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AttachLink.srv"
  "srv/DetachLink.srv"
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
