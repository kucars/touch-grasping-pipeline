cmake_minimum_required(VERSION 2.8.3)
project(ist_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  manipulation_msgs
  trajectory_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ActionablePartData.msg
   ActionablePartStatus.msg
   ActionableType.msg
   ActionAffordance.msg
   Action.msg
   ActionType.msg
   ActionVector.msg
   BoundingBox.msg
   Graspability.msg
   GraspParams.msg
   GripDirection.msg
   GripDistance.msg
   GripList.msg
   GripPose.msg
   GripState.msg	
   GripTrial.msg
   HandPose.msg
   HandPosture.msg
   HandState.msg
   MaterialType.msg
   ObjectCategory.msg	
   ObjectData.msg
   ObjectList.msg
   Object.msg
   ObjectPart.msg
   ObjectState.msg
   ObjectType.msg
   RefinedObjectPointCloud.msg
   SizeType.msg
   ShapeType.msg
   SymmetryType.msg
   TaskType.msg
   Task.msg
)


generate_messages(DEPENDENCIES   
  std_msgs
  geometry_msgs
  sensor_msgs
  manipulation_msgs
  trajectory_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_msgs
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs manipulation_msgs trajectory_msgs
#  DEPENDS system_lib
)
