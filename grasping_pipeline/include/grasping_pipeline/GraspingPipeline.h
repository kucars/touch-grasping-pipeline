#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <perception_msgs/DetectObjectsAction.h>
#include <ist_grasp_generation_msgs/GenerateGraspsAction.h>

#include <ist_grasp_generation_msgs/GraspingAction.h>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Empty.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_listener.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <map>
class GraspingPipelineAction
{
protected:

    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<ist_grasp_generation_msgs::GraspingAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    ist_grasp_generation_msgs::GraspingFeedback feedback_;
    ist_grasp_generation_msgs::GraspingResult result_;

public:

    GraspingPipelineAction(std::string name) :
        as_(nh_, name, boost::bind(&GraspingPipelineAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~GraspingPipelineAction(void)
    {}

    void executeCB(const ist_grasp_generation_msgs::GraspingGoalConstPtr &goal);


};
