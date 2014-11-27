# varying part of the object message

# ROS data (potential models with ROS ID) + cluster + pose + collision name)
manipulation_msgs/GraspableObject graspable_object

# Optional actionable parts. Must be in the same order as the actionable_parts_data
ist_msgs/ActionablePartStatus[] actionable_parts_status

# weight may vary if liquid is poured inside etc...
float64 weight

# Discrete pose
int8 discrete_pose
int8 ANY_POSE=0
int8 VERTICAL=1
int8 HORIZONTAL=2
