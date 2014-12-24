cmake_minimum_required(VERSION 2.8.3)
project(touch_evaluated_point)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED
  roscpp
  pcl_ros
  std_msgs
  sensor_msgs 
  geometry_msgs  
  pcl_conversions
  eigen_conversions
  tf_conversions
  tf
)
#find_package(PCL 1.2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(CATKIN_DEPENDS 
    roscpp
    std_msgs
    pcl_conversions
    eigen_conversions
    tf_conversions
    geometry_msgs 
    pcl_ros
    sensor_msgs
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
#  ${CMAKE_CURRENT_LIST_DIR}/include/
)


link_directories(
  ${Boost_LIBRARY_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_executable (point src/point.cpp)
target_link_libraries (point ${catkin_LIBRARIES} ${PCL_LIBRARIES})
