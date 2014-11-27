# defines the grip considered
#int32 grip_type_id
# defines the object on which this grip was tried
int32 object_id

int32 grip_index
# number of trials for this grip_type and object_id
#int32 nb_trials
# number of success grips
#int32 nb_success
# value of the gaussian feature that was used for babbling"
#float64[] gaussian_features
# name of the feature (ex: translation x)
#string[] feature_types

bool grasped
trajectory_msgs/JointTrajectory lift_trajectory
bool lift_trajectory_success
bool lifted

float64 grasp_probability
