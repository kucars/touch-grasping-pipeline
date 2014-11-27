cmake_minimum_required(VERSION 2.8.3)
project(perception_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        actionlib_msgs
        std_msgs
	sensor_msgs
        segmentation
        ist_msgs
	bottomup_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   TableRegion.msg
   Object.msg
   Region.msg
   Task.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   PoseEstimation.srv
   SetObjectCoordinateFrame.srv
   GetObjectDetails.srv
   GetRefinedPointCloud.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  DetectObjects.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs 
   actionlib_msgs
   ist_msgs
   segmentation
   bottomup_msgs # Or other packages containing msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_msgs
  CATKIN_DEPENDS std_msgs sensor_msgs ist_msgs segmentation bottomup_msgs
#  DEPENDS system_lib
)