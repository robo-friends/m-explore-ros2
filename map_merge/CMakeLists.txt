cmake_minimum_required(VERSION 3.5)
project(multirobot_map_merge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

# OpenCV is required for merging without initial positions
find_package(OpenCV REQUIRED)
if("${OpenCV_VERSION}" VERSION_LESS "3.0")
message(FATAL_ERROR "This package needs OpenCV >= 3.0")
endif()
if("${OpenCV_VERSION}" VERSION_LESS "4.0")
message(WARNING "This package supports OpenCV 3, but some features may not be "
"available. Upgrade to OpenCV 4 to take advantage of all features.")
endif()

set(DEPENDENCIES
  rclcpp
  geometry_msgs
  image_geometry
  map_msgs
  nav_msgs
  tf2_geometry_msgs
  OpenCV
)

## Specify additional locations of header files
include_directories(
  # ${Boost_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  include
)

install(
  DIRECTORY include/map_merge/
  DESTINATION include/map_merge/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# we want static linking for now
add_library(combine_grids STATIC
src/combine_grids/grid_compositor.cpp
src/combine_grids/grid_warper.cpp
src/combine_grids/merging_pipeline.cpp
)

add_executable(map_merge
  src/map_merge.cpp
)

#############
## Install ##
#############
target_include_directories(map_merge PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_include_directories(combine_grids PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(combine_grids ${rclcpp_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries(combine_grids ${OpenCV_LIBRARIES})

target_link_libraries(map_merge combine_grids)
# target_link_libraries(map_merge)

ament_target_dependencies(map_merge ${DEPENDENCIES})

install(
  TARGETS combine_grids
  EXPORT export_combine_grids
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS map_merge
  DESTINATION lib/${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

ament_export_include_directories(include)
ament_export_libraries(combine_grids)
ament_package()


#############
## Testing ##
#############
# if(CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)

#   # download test data
#   set(base_url https://raw.githubusercontent.com/hrnr/m-explore-extra/master/map_merge)
#   catkin_download_test_data(${PROJECT_NAME}_map00.pgm ${base_url}/hector_maps/map00.pgm MD5 915609a85793ec1375f310d44f2daf87)
#   catkin_download_test_data(${PROJECT_NAME}_map05.pgm ${base_url}/hector_maps/map05.pgm MD5 cb9154c9fa3d97e5e992592daca9853a)
#   catkin_download_test_data(${PROJECT_NAME}_2011-08-09-12-22-52.pgm ${base_url}/gmapping_maps/2011-08-09-12-22-52.pgm MD5 3c2c38e7dec2b7a67f41069ab58badaa)
#   catkin_download_test_data(${PROJECT_NAME}_2012-01-28-11-12-01.pgm ${base_url}/gmapping_maps/2012-01-28-11-12-01.pgm MD5 681e704044889c95e47b0c3aadd81f1e)

#   catkin_add_gtest(test_merging_pipeline test/test_merging_pipeline.cpp)
#   # ensure that test data are downloaded before we run tests
#   add_dependencies(test_merging_pipeline ${PROJECT_NAME}_map00.pgm ${PROJECT_NAME}_map05.pgm ${PROJECT_NAME}_2011-08-09-12-22-52.pgm ${PROJECT_NAME}_2012-01-28-11-12-01.pgm)
#   target_link_libraries(test_merging_pipeline combine_grids ${catkin_LIBRARIES})

#   # test all launch files
#   # do not test from_map_server.launch as we don't want to add dependency on map_server and this
#   # launchfile is not critical
#   roslaunch_add_file_check(launch/map_merge.launch)
#   roslaunch_add_file_check(launch/experiments)
# endif()



