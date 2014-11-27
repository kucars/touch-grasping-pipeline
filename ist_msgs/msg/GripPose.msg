# pose relatively to the object frame
geometry_msgs/PoseStamped pose

# part where the thumb touches in a pinch grasp
ist_msgs/ObjectPart part

# closest distance between object center and palm plane (not to the palm frame)
ist_msgs/GripDistance distance

# direction of approach relatively to the object frame
ist_msgs/GripDirection direction


bool perturb
# pose perturb relatively to the object frame
geometry_msgs/PoseStamped perturb_pose

