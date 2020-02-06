cmake_minimum_required(VERSION 2.8.3)
project(lidar_distance_filter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  sensor_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lidar_filter
  CATKIN_DEPENDS pcl_ros roscpp
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/lidar_filter.cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
