// code for the mitsubishi_barrett different chains "main" "finger1" "finger2" finger3" "arm1" "arm2" "arm3" "arm"

#include <ros/ros.h>
#include <iostream>
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
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();

    move_group_interface::MoveGroup group3("main");
    group3.setPoseReferenceFrame("base_link");
    group3.setEndEffectorLink("end_effector");
    ROS_INFO("Reference frame3: %s", group3.getPlanningFrame().c_str());
    ROS_INFO("Reference end effector3: %s", group3.getEndEffectorLink().c_str());
    group3.setPlanningTime(30.0);
    group3.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);

    //start state specified in cartisian space
//    robot_state::RobotState start_state(*group3.getCurrentState());
//    geometry_msgs::Pose start_pose2;
//    start_pose2.orientation.w = 1.0;
//    start_pose2.position.x = 0.0;
//    start_pose2.position.y = 0.0;
//    start_pose2.position.z = 1.32;
//    const robot_state::JointModelGroup *joint_model_group =
//    start_state.getJointModelGroup(group3.getName());
//    start_state.setFromIK(joint_model_group, start_pose2);
//    group3.setStartState(start_state);
    group3.setPlannerId("RRTkConfigDefault");
    group3.setGoalPositionTolerance(0.05);
    group3.setGoalOrientationTolerance(0.05);
    geometry_msgs::Pose fixedPose;
    moveit::planning_interface::MoveGroup::Plan plan;
    fixedPose.position = point->pose.position;
//    fixedPose.orientation = point->pose.orientation;
    //working orientation
    fixedPose.orientation.x = 0.693981;
    fixedPose.orientation.y = -0.69391;
    fixedPose.orientation.z = 0.135814;
    fixedPose.orientation.w = -0.135773;


    //*************************
//    robot_state::RobotState start_state(*group3.getCurrentState());
//    const robot_state::JointModelGroup *joint_model_group =
//            start_state.getJointModelGroup(group3.getName());
//    start_state.setFromIK(joint_model_group, fixedPose);
//    group3.setJointValueTarget(start_state);
    //*************************
    std::cout << "Position x " <<point->pose.position.x<<std::endl;
    std::cout << "Position y " <<point->pose.position.y<<std::endl;
    std::cout << "Position z " <<point->pose.position.z<<std::endl;
    std::cout << "orientation x " <<point->pose.orientation.x<<std::endl;
    std::cout << "orientation y " <<point->pose.orientation.y<<std::endl;
    std::cout << "orientation z " <<point->pose.orientation.z<<std::endl;
    std::cout << "orientation w " <<point->pose.orientation.w<<std::endl;


    // fixed known position for the mitsubishi end_effector
//    fixedPose.position.x = 0.460301;
//    fixedPose.position.y = 0.0863982;
//    fixedPose.position.z = 0.322652;
//    fixedPose.orientation.x = -0.751631;
//    fixedPose.orientation.y = -0.240872;
//    fixedPose.orientation.z = -0.00833184;
//    fixedPose.orientation.w = 0.613973;

    group3.setPoseTarget(fixedPose);
    //group3.setApproximateJointValueTarget(fixedPose);
   // group3.setJointValueTarget(fixedPose);


    bool success = group3.plan(plan);
    if(success)
    {
        ROS_INFO("Moving to my Destination");
//                group3.move();
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
    move_group_interface::MoveGroup group("finger1");
    group.setPoseReferenceFrame("barrett_base_link");
    group.setEndEffectorLink("finger_1_dist_link");
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference end effector: %s", group.getEndEffectorLink().c_str());
    group.setPlanningTime(30.0);
    group.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);
    planning_scene->setCurrentState(*group.getCurrentState());
    const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("finger1");
    robot_state::RobotState goal_state(robot_model);
    std::vector<double> joint_values(3, 0.0);
    joint_values[0] = 0.0;
    joint_values[1] = 2.4;
    joint_values[2] = 2.4;
//    joint_values[3] = 0.0;
//    joint_values[4] = 0.0;
//    joint_values[5] = 0.0;
//    joint_values[6] = 0.0;
//    joint_values[7] = 2.4;
//    joint_values[8] = 2.4;
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    group.setJointValueTarget(goal_state);
        group.move();
    sleep(2);


    robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(*group.getCurrentState());


    move_group_interface::MoveGroup group1("finger2");
    group1.setPoseReferenceFrame("barrett_base_link");
    group1.setEndEffectorLink("finger_2_dist_link");
    ROS_INFO("Reference frame2: %s", group1.getPlanningFrame().c_str());
    ROS_INFO("Reference end effector2: %s", group1.getEndEffectorLink().c_str());
    group1.setPlanningTime(30.0);
    group1.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);
    planning_scene->setCurrentState(*group1.getCurrentState());
    joint_model_group = robot_state.getJointModelGroup("finger2");
    joint_values[0] = 0.0;
    joint_values[1] = 2.4;
    joint_values[2] = 2.4;
//    joint_values[3] = 0.0;
//    joint_values[4] = 0.0;
//    joint_values[5] = 0.0;
//    joint_values[6] = 0.0;
//    joint_values[7] = 2.4;
//    joint_values[8] = 2.4;
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    group1.setJointValueTarget(goal_state);
        group1.move();
    sleep(2);

    robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(*group1.getCurrentState());


    move_group_interface::MoveGroup group2("finger3");
    group2.setPoseReferenceFrame("barrett_base_link");
    group2.setEndEffectorLink("finger_3_dist_link");
    ROS_INFO("Reference frame3: %s", group2.getPlanningFrame().c_str());
    ROS_INFO("Reference end effector3: %s", group2.getEndEffectorLink().c_str());
    group2.setPlanningTime(30.0);
    group2.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);
    planning_scene->setCurrentState(*group2.getCurrentState());
    joint_model_group = robot_state.getJointModelGroup("finger3");
    joint_values[0] = 0.85;
    joint_values[1] = 0.85;
    joint_values[2] = 0.85;
//    joint_values[3] = 0.0;
//    joint_values[4] = 0.0;
//    joint_values[5] = 0.0;
//    joint_values[6] = 0.0;
//    joint_values[7] = 0.85;
//    joint_values[8] = 0.85;
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    group2.setJointValueTarget(goal_state);
        group2.move();
    sleep(2);

    robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(*group1.getCurrentState());

    ros::Subscriber pointsub = n.subscribe<visualization_msgs::Marker>("visualization_marker", 10, pointCallback);

    while(ros::ok())
    {

        if(readpose==1)
            ros::waitForShutdown();
//            return 0;

        ros::spinOnce();
    }

    //testing the main arm *************************
//    move_group_interface::MoveGroup group3("main");
//    group3.setPoseReferenceFrame("base_link");
//    group3.setEndEffectorLink("end_effector");
//    ROS_INFO("Reference frame main: %s", group3.getPlanningFrame().c_str());
//    ROS_INFO("Reference end effector main: %s", group3.getEndEffectorLink().c_str());
//    group3.setPlanningTime(30.0);
//    group3.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);

//    geometry_msgs::Pose fixedPose;
//    moveit::planning_interface::MoveGroup::Plan plan;

//    fixedPose.position.x = 0.460301;
//    fixedPose.position.y = 0.0863982;
//    fixedPose.position.z = 0.322652;
//    fixedPose.orientation.x = -0.751631;
//    fixedPose.orientation.y = -0.240872;
//    fixedPose.orientation.z = -0.00833184;
//    fixedPose.orientation.w = 0.613973;

//    group3.setPoseTarget(fixedPose);
//    //group3.setApproximateJointValueTarget(fixedPose);
//    //group3.setJointValueTarget(fixedPose);


//    bool success = group3.plan(plan);
//    if(success)
//    {
//        ROS_INFO("Moving to my Destination");
//                group3.move();
//        ROS_INFO("Finished Moving");
//    }
//    else
//        ROS_INFO("Failure. No movement");
//    sleep(1);

//    ros::waitForShutdown();



}

