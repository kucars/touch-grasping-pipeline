cmake_minimum_required(VERSION 2.8.3)
project(moveit_api_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  tf
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  cmake_modules
  eigen_conversions
  )

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES opencv_calibration
# CATKIN_DEPENDS opencv2 roscpp
# DEPENDS system_lib
)

# include_directories(include)
include_directories(${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )
  
add_executable(moveit_api_test src/moveit_api_test.cpp)
target_link_libraries(moveit_api_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_moveit src/test_moveit.cpp)
target_link_libraries(test_moveit ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(api_moveit_test src/api_moveit_test.cpp)
target_link_libraries(api_moveit_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(mitsubishi_barrett_moveit src/mitsubishi_barrett_moveit.cpp)
target_link_libraries(mitsubishi_barrett_moveit ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(barrett_mitsubishi_moveit src/barrett_mitsubishi_moveit.cpp)
target_link_libraries(barrett_mitsubishi_moveit ${catkin_LIBRARIES} ${PCL_LIBRARIES})

