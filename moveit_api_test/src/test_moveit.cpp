
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <visualization_msgs/Marker.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_api_test", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // start a ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // this connecs to a running instance of the move_group node
    move_group_interface::MoveGroup group("barrett_hand");
    group.setEndEffectorLink("/barrett_hand/finger_3_med_link");
    geometry_msgs::Pose fixedPose;


    ROS_INFO("loading the model");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();


    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference end effector: %s", group.getEndEffectorLink().c_str());
    group.setPlanningTime(40.0);
    group.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);
    moveit::planning_interface::MoveGroup::Plan plan;

    //        ROS_INFO("Attempting to move to position: %d", poseIndex);
    //        fixedPose.position.x = -0.025825;
    //        fixedPose.position.y = 0.00124727;
    //        fixedPose.position.z = 0.306;
    //        fixedPose.orientation.x = 0;
    //        fixedPose.orientation.y = 0;
    //        fixedPose.orientation.z = 0;
    //        fixedPose.orientation.w = 1;

    planning_scene->setCurrentState(*group.getCurrentState());
    const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("barrett_hand");
    //    robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

    // Now, setup a joint space goal
    robot_state::RobotState goal_state(robot_model);
    std::vector<double> joint_values(8, 0.0);
    joint_values[0] = 1.04;
    joint_values[1] = 1.04;
    joint_values[2] = 1.04;
    joint_values[3] = 1.04;
    joint_values[4] = 1.0;
    joint_values[5] = 0.7;
    joint_values[6] = 0.7;
    joint_values[7] = 0.7;
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    group.setJointValueTarget(goal_state);


    //        group.setPoseTarget(fixedPose);
    //group.setApproximateJointValueTarget(fixed_pose);
    //group.setJointValueTarget(fixed_pose);
    //-------------------------
    //group.setGoalPositionTolerance(0.5);
    //group.setGoalOrientationTolerance(0.5);
    //group.setApproximateJointValueTarget(fixed_pose);
    //group.setRandomTarget();
    bool success = group.plan(plan);
    if(success)
    {
        ROS_INFO("Moving to my Destination");
        group.move();
        ROS_INFO("Finished Moving");
    }
    else
        ROS_INFO("Failure. No movement");
    ros::waitForShutdown();

}
