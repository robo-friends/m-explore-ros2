cmake_minimum_required(VERSION 3.5)
project(explore_lite)

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

# Set flag depending on distro
if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined." )
endif()
if("$ENV{ROS_DISTRO}" STREQUAL "eloquent")
  message(STATUS "Build for ROS2 eloquent")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DELOQUENT")
elseif("$ENV{ROS_DISTRO}" STREQUAL "dashing")
  message(STATUS "Build for ROS2 dashing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDASHING")
else()
  message(STATUS "BuilD for ROS2: " "$ENV{ROS_DISTRO}")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)


set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav_msgs
  map_msgs
  nav2_costmap_2d
  visualization_msgs
)

include_directories(
    include
)

install(
  DIRECTORY include/explore/
  DESTINATION include/explore/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


add_executable(explore
  src/costmap_client.cpp
  src/explore.cpp
  src/frontier_search.cpp
)

target_include_directories(explore PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


target_link_libraries(explore ${rclcpp_LIBRARIES})

ament_target_dependencies(explore ${DEPENDENCIES})

install(TARGETS explore
  DESTINATION lib/${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_explore test/test_explore.cpp)
  target_link_libraries(test_explore ${catkin_LIBRARIES})
  ament_target_dependencies(test_explore ${DEPENDENCIES})

  
endif()


ament_export_include_directories(include)
ament_package()