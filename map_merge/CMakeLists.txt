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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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
  tf2
  tf2_ros
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
ament_target_dependencies(combine_grids ${DEPENDENCIES})

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

  # download test data: TODO: ROS2 alternative? For now you'll need to download them manually
  set(base_url https://raw.githubusercontent.com/hrnr/m-explore-extra/master/map_merge)
  # ament_download(${base_url}/hector_maps/map00.pgm
  #   MD5 915609a85793ec1375f310d44f2daf87
  #   FILENAME ${PROJECT_NAME}_map00.pgm
  # )
  execute_process(
    COMMAND bash download.sh ${CMAKE_BINARY_DIR}/map00.pgm ${base_url}/hector_maps/map00.pgm 915609a85793ec1375f310d44f2daf87
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test
  )
  execute_process(
    COMMAND bash download.sh ${CMAKE_BINARY_DIR}/map05.pgm ${base_url}/hector_maps/map05.pgm cb9154c9fa3d97e5e992592daca9853a
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test
  )
  execute_process(
    COMMAND bash download.sh ${CMAKE_BINARY_DIR}/2011-08-09-12-22-52.pgm ${base_url}/gmapping_maps/2011-08-09-12-22-52.pgm 3c2c38e7dec2b7a67f41069ab58badaa
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test
  )
  execute_process(
    COMMAND bash download.sh ${CMAKE_BINARY_DIR}/2012-01-28-11-12-01.pgm ${base_url}/gmapping_maps/2012-01-28-11-12-01.pgm 681e704044889c95e47b0c3aadd81f1e
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test
  )

  ament_add_gtest(test_merging_pipeline test/test_merging_pipeline.cpp)
  # ensure that test data are downloaded before we run tests
  # add_dependencies(test_merging_pipeline ${PROJECT_NAME}_map00.pgm ${PROJECT_NAME}_map05.pgm ${PROJECT_NAME}_2011-08-09-12-22-52.pgm ${PROJECT_NAME}_2012-01-28-11-12-01.pgm)
  target_link_libraries(test_merging_pipeline combine_grids ${catkin_LIBRARIES})
  
endif()


ament_export_include_directories(include)
ament_export_libraries(combine_grids)
ament_package()