#include <grasping_pipeline/GraspingPipeline.h>

void GraspingPipelineAction::executeCB(const ist_grasp_generation_msgs::GraspingGoalConstPtr &goal)
{
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    //feedback_.sequence.clear();
    //feedback_.sequence.push_back(0);
    //feedback_.sequence.push_back(1);

    // publish info to the console for the user
    //ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    ////////////////////
    // DETECT OBJECTS //
    ////////////////////


    actionlib::SimpleActionClient<perception_msgs::DetectObjectsAction> detect_objects("detect_objects_server", true);

    ROS_INFO("Waiting for object detection action server to start.");
    // wait for the action server to start
    detect_objects.waitForServer(); //will wait for infinite time

    // send a goal to the action
    perception_msgs::DetectObjectsGoal object_detection_goal;

    object_detection_goal.table_region.x_filter_min = 0.4;
    object_detection_goal.table_region.x_filter_max = 1.0;
    object_detection_goal.table_region.y_filter_min =-0.4;
    object_detection_goal.table_region.y_filter_max = 0.4;
    object_detection_goal.table_region.z_filter_min =-0.1;
    object_detection_goal.table_region.z_filter_max = 0.3;

    detect_objects.sendGoal(object_detection_goal);

    //wait for the action to return
    bool finished_before_timeout = detect_objects.waitForResult(ros::Duration(60.0));

    if(!finished_before_timeout)
    {
        ROS_INFO("Action did not finish before the time out.");
        as_.setAborted();
        success = false;
        return;
    }
    else if(detect_objects.getState()!=detect_objects.getState().SUCCEEDED)
    {
        ROS_INFO("Action did not succeed.");
        as_.setAborted();
        success = false;
        return;
    }
    else
    {
        ROS_INFO("Action finished: %s",detect_objects.getState().toString().c_str());
    }

    ist_grasp_generation_msgs::GenerateGraspsGoal objects;
    objects.object_to_grasp_id=0;
    objects.object_list=detect_objects.getResult()->object_list;
    //objectsToCollisionEnvironment(objects.object_list);

    /////////////////////
    // GENERATE GRASPS //
    /////////////////////

    actionlib::SimpleActionClient<ist_grasp_generation_msgs::GenerateGraspsAction> generate_grasps("generate_grasps_server", true);
    ROS_INFO("Waiting for generate grasps action server to start.");

    // wait for the action server to start
    generate_grasps.waitForServer(); //will wait for infinite time

    generate_grasps.sendGoal(objects);

    //wait for the action to return
    finished_before_timeout = detect_objects.waitForResult(ros::Duration(30.0));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = generate_grasps.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        as_.setPreempted();
        success = false;
    }

    if(success)
    {
        //result_.sequence = feedback_.state;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasping_pipeline_server");

    GraspingPipelineAction grasping_pipeline(ros::this_node::getName());
    ros::spin();

    return 0;
}



