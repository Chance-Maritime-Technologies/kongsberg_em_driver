cmake_minimum_required(VERSION 3.8)
project(kongsberg_em_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_options(-Wall -Werror=return-type)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ds_core_msgs REQUIRED)
find_package(ds_multibeam_msgs REQUIRED)
find_package(ds_kongsberg_msgs REQUIRED)

# kongsberg_em2040_util.h builds a pcl::PointCloud<pcl::PointXYZI> directly, so PCL's own
# headers are needed and not just the pcl_conversions/pcl_ros wrappers.
find_package(PCL REQUIRED COMPONENTS common)

find_package(Boost REQUIRED COMPONENTS system)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  ds_core_msgs
  ds_multibeam_msgs
  ds_kongsberg_msgs
)

###########
## Build ##
###########

file(GLOB_RECURSE LIB_SOURCES src/em_driver_library/*.cpp src/em_driver_library/*.h)
add_library(lib_kongsberg_em ${LIB_SOURCES})

target_include_directories(lib_kongsberg_em PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(lib_kongsberg_em SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
ament_target_dependencies(lib_kongsberg_em ${DEPENDENCIES})
target_link_libraries(lib_kongsberg_em Boost::system ${PCL_COMMON_LIBRARIES})

file(GLOB_RECURSE NODE_SOURCES src/em_driver_node/*.cpp src/em_driver_node/*.h)
add_executable(kongsberg_em_node ${NODE_SOURCES})

target_include_directories(kongsberg_em_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(kongsberg_em_node ${DEPENDENCIES})
target_link_libraries(kongsberg_em_node lib_kongsberg_em)

#############
## Install ##
#############

install(
  TARGETS lib_kongsberg_em
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS kongsberg_em_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(lib_kongsberg_em)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Both tests read fixtures out of test/ via relative paths, so they run from the package
  # source dir. This replaces ROS 1's add_rostest_gtest + em2040_parse_test.test, which only
  # existed to inject those paths as rosparams; there is no ROS 2 equivalent of that macro.
  ament_add_gtest(test_em2040_parse src/test/test_kongsberg_em2040_parsing.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  target_link_libraries(test_em2040_parse lib_kongsberg_em)

  ament_add_gtest(test_em2040_utils src/test/test_kongsberg_em2040_util.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  target_link_libraries(test_em2040_utils lib_kongsberg_em)
endif()

ament_package()
