#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ist_grasp_generation_msgs/GraspingAction.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "grasping_pipeline_client");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<ist_grasp_generation_msgs::GraspingAction> ac("grasping_pipeline_server", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    ist_grasp_generation_msgs::GraspingGoal goal;

    goal.table_region.x_filter_min = 0.2;
    goal.table_region.x_filter_max = 1.0;
    goal.table_region.y_filter_min =-0.4;
    goal.table_region.y_filter_max = 0.4;
    goal.table_region.z_filter_min =-0.1;
    goal.table_region.z_filter_max = 0.3;

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //exit
    return 0;
}
