cmake_minimum_required(VERSION 3.16)
project(linkattacher_test)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes urdf sdf worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(linkattacher_msgs REQUIRED)
  find_package(linkattacher_ros2 REQUIRED)
  find_package(gz_sim_vendor REQUIRED)
  find_package(gz-sim8 REQUIRED)

  ament_add_gtest(test_link_attacher test/test_link_attacher.cpp TIMEOUT 90)
  target_compile_features(test_link_attacher PRIVATE cxx_std_17)
  target_link_libraries(test_link_attacher gz-sim8::gz-sim8)
  ament_target_dependencies(test_link_attacher rclcpp linkattacher_msgs)
  target_compile_definitions(test_link_attacher PRIVATE
    TEST_WORLD_PATH="${CMAKE_CURRENT_SOURCE_DIR}/worlds/link_attacher.sdf")
  # Find the installed plugin dependency, also when running tests immediately
  # after a build without sourcing the new workspace first.
  ament_index_has_resource(LINKATTACHER_PREFIX packages linkattacher_ros2)
  if(NOT LINKATTACHER_PREFIX)
    message(FATAL_ERROR "The installed linkattacher_ros2 package was not found")
  endif()
  set_tests_properties(test_link_attacher PROPERTIES ENVIRONMENT
    "GZ_SIM_SYSTEM_PLUGIN_PATH=${LINKATTACHER_PREFIX}/lib:$ENV{GZ_SIM_SYSTEM_PLUGIN_PATH};ROS_DOMAIN_ID=97;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log;GZ_PARTITION=ifra_linkattacher_test")
endif()

ament_package()
