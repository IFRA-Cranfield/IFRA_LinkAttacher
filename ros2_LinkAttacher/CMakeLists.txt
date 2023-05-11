cmake_minimum_required(VERSION 3.5)
project(ros2_linkattacher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

############## COMPILE the LinkAttacher Gazebo Plugin ##############

# Find required packages:
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(linkattacher_msgs REQUIRED)

# /include:
include_directories(include)

# GAZEBO LINK ATTACHER:

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gazebo_link_attacher SHARED
  src/gazebo_link_attacher.cpp
)
target_link_libraries(gazebo_link_attacher ${GAZEBO_LIBRARIES})
ament_target_dependencies(gazebo_link_attacher
  "rclcpp"
  "gazebo_dev"
  "gazebo_msgs"
  "gazebo_ros"
  "linkattacher_msgs"
)
ament_export_libraries(gazebo_link_attacher)
install(
  TARGETS
    gazebo_link_attacher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()