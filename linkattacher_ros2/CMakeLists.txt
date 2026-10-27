cmake_minimum_required(VERSION 3.16)
project(linkattacher_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(linkattacher_msgs REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(gz_link_attacher_plugin SHARED src/gz_link_attacher.cpp)
target_compile_features(gz_link_attacher_plugin PUBLIC cxx_std_17)
target_include_directories(gz_link_attacher_plugin PRIVATE include)
target_link_libraries(gz_link_attacher_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2-register)
ament_target_dependencies(gz_link_attacher_plugin rclcpp linkattacher_msgs)

install(TARGETS gz_link_attacher_plugin LIBRARY DESTINATION lib)
ament_environment_hooks(env-hooks/gz_sim_system_plugin_path.dsv.in)

ament_package()
