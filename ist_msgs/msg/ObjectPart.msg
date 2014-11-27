#discrete
int8 id
int8 ANY_PART=0
int8 CENTER=1
int8 MIDDLE=2
int8 TOP=3
int8 HANDLE=4
int8 USABLE=5

#continuous
# part pose relatively to the object frame
geometry_msgs/PoseStamped pose

#part dimensions
geometry_msgs/Vector3 bounding_box

#Part confidence
float64 confidence

#Part discrete confidence
bool use
