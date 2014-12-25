#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <visualization_msgs/Marker.h>
int readpose=0;
void pointCallback(const visualization_msgs::Marker::ConstPtr& point)
{
    readpose=1;
    move_group_interface::MoveGroup group3("chain3");
    group3.setPoseReferenceFrame("base_link");
    group3.setEndEffectorLink("finger_3_dist_link");
    ROS_INFO("Reference frame3: %s", group3.getPlanningFrame().c_str());
    ROS_INFO("Reference end effector3: %s", group3.getEndEffectorLink().c_str());
    group3.setPlanningTime(30.0);
    group3.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);

    geometry_msgs::Pose fixedPose;
    moveit::planning_interface::MoveGroup::Plan plan;
    fixedPose.position = point->pose.position;
    fixedPose.orientation = point->pose.orientation;
    group3.setPoseTarget(fixedPose);
    //group3.setApproximateJointValueTarget(fixedPose);
    //group3.setJointValueTarget(fixedPose);


    bool success = group3.plan(plan);
    if(success)
    {
        ROS_INFO("Moving to my Destination");
        //        group3.move();
        ROS_INFO("Finished Moving");
    }
    else
        ROS_INFO("Failure. No movement");
    sleep(1);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_api_test", ros::init_options::AnonymousName);
    ros::NodeHandle n;


    // start a ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("loading the model");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();

    // this connecs to a running instance of the move_group node
    move_group_interface::MoveGroup group("chain1");
    group.setPoseReferenceFrame("base_link");
    group.setEndEffectorLink("finger_1_dist_link");
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference end effector: %s", group.getEndEffectorLink().c_str());
    group.setPlanningTime(30.0);
    group.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);
    planning_scene->setCurrentState(*group.getCurrentState());
    const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("chain1");
    robot_state::RobotState goal_state(robot_model);
    std::vector<double> joint_values(9, 0.0);
    joint_values[0] = 0.0;
    joint_values[1] = 0.0;
    joint_values[2] = 0.0;
    joint_values[3] = 0.0;
    joint_values[4] = 0.0;
    joint_values[5] = 0.0;
    joint_values[6] = 0.0;
    joint_values[7] = 2.4;
    joint_values[8] = 2.4;
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    group.setJointValueTarget(goal_state);
    group.move();
    sleep(2);


    robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(*group.getCurrentState());


    move_group_interface::MoveGroup group1("chain2");
    group1.setPoseReferenceFrame("base_link");
    group1.setEndEffectorLink("finger_2_dist_link");
    ROS_INFO("Reference frame2: %s", group1.getPlanningFrame().c_str());
    ROS_INFO("Reference end effector2: %s", group1.getEndEffectorLink().c_str());
    group1.setPlanningTime(30.0);
    group1.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);
    planning_scene->setCurrentState(*group1.getCurrentState());
    joint_model_group = robot_state.getJointModelGroup("chain2");
    joint_values[0] = 0.0;
    joint_values[1] = 0.0;
    joint_values[2] = 0.0;
    joint_values[3] = 0.0;
    joint_values[4] = 0.0;
    joint_values[5] = 0.0;
    joint_values[6] = 0.0;
    joint_values[7] = 2.4;
    joint_values[8] = 2.4;
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    group1.setJointValueTarget(goal_state);
    group1.move();
    sleep(2);

    robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(*group1.getCurrentState());

    ros::Subscriber pointsub = n.subscribe<visualization_msgs::Marker>("visualization_marker", 10, pointCallback);

    while(ros::ok())
    {

        if(readpose==1)
            ros::waitForShutdown();

        ros::spinOnce();
    }

}

