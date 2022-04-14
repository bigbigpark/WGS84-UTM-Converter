cmake_minimum_required(VERSION 3.0.2)
project(wgs84_to_utm)

add_compile_options(-std=c++14)
#set(GeographicLib_DIR /home/scpark/tools/geographic/lib/cmake/GeographicLib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package (GeographicLib REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wgs84_to_utm_converter src/wgs84_to_utm.cpp src/gps_converter.cpp)
target_link_libraries(wgs84_to_utm_converter ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
