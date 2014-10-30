cmake_minimum_required(VERSION 2.8.3)
project(segmentation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  std_msgs
  sensor_msgs 
  geometry_msgs 
  message_generation 
  object_recognition_msgs 
  pcl_conversions 
  tf
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

add_message_files(
   DIRECTORY msg
   FILES
   Table.msg
   TableRegion.msg
)
generate_messages(DEPENDENCIES sensor_msgs shape_msgs)
catkin_package(CATKIN_DEPENDS 
    message_runtime
    shape_msgs
    roscpp
    sensor_msgs
    std_msgs
    pcl_conversions
    pcl_ros
    sensor_msgs
    tf
)
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES segmentation
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
#)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


link_directories(
  ${Boost_LIBRARY_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_executable(plane_segmentation src/plane_segmentation.cpp)
add_dependencies(plane_segmentation ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(plane_segmentation ${PCL_INCLUDE_DIRS}
${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
