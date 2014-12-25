
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
    move_group_interface::MoveGroup group("main");
    group.setPoseReferenceFrame("base_link");
    group.setEndEffectorLink("end_effector");
    geometry_msgs::Pose fixedPose;

    ROS_INFO("loading the model");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference end effector: %s", group.getEndEffectorLink().c_str());
    group.setPlanningTime(30.0);
    group.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);
    moveit::planning_interface::MoveGroup::Plan plan;
    planning_scene->setCurrentState(*group.getCurrentState());
    //finger 1
//    fixedPose.position.x = 0.0794881;
//    fixedPose.position.y = -0.0297269;
//    fixedPose.position.z = 0.144352;
//    fixedPose.orientation.x = 0.705589;
//    fixedPose.orientation.y = 0.0463296;
//    fixedPose.orientation.z = -0.378593;
//    fixedPose.orientation.w = 0.597215;
//    fixedPose.position.x = 0.036773;
//    fixedPose.position.y = 0.0367728;
//    fixedPose.position.z = 0.104537;
//    fixedPose.orientation.x = 0.0341714;
//    fixedPose.orientation.y = 0.706282;
//    fixedPose.orientation.z = -0.0450827;
//    fixedPose.orientation.w = 0.705667;

    //finger 3
//    fixedPose.position.x = 0.0;
//    fixedPose.position.y = -0.0529071;
//    fixedPose.position.z = 0.14534;
//    fixedPose.orientation.x = 0.600692;
//    fixedPose.orientation.y = -0.373057;
//    fixedPose.orientation.z = -0.600688;
//    fixedPose.orientation.w = 0.373057;

//    group.setPoseTarget(fixedPose);
//     group.setApproximateJointValueTarget(fixedPose);
//    group.setJointValueTarget(fixedPose);

//    // group.setRandomTarget();
//    bool success = group.plan(plan);
//    if(success)
//    {
//        ROS_INFO("Moving to my Destination");
//        group.move();
//        ROS_INFO("Finished Moving");
//    }
//    else
//        ROS_INFO("Failure. No movement");

    robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(*group.getCurrentState());
//    joint_values=group.getCurrentJointValues();
//    moveit::core::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
//    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("main");

//    robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

//   moveit::core::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
//    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("finger_1_dist_link");

//    std::vector<double> joint_values;
//    std::vector<std::string> joint_names;

//   kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
//   bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

//    if(found_ik)
//    {
//        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
//        joint_names=joint_model_group->getJointModelNames();
//        for(std::size_t i=0; i < joint_names.size(); ++i)
//        {
//            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
//        }
//    }
//    else
//    {
//        ROS_INFO("Did not find IK solution");
//    }


    // Now, setup a joint space goal
    robot_state::RobotState goal_state(robot_model);
    std::vector<double> joint_values(9, 0.0);
    joint_values[0] = 0.0;
    joint_values[1] = 0.5;
    joint_values[2] = 0.0;
    joint_values[3] = 0.0;
    joint_values[4] = 0.0;
    joint_values[5] = 0.0;
//    joint_values[6] = 0.0;
//    joint_values[7] = 2.4;
//    joint_values[8] = 2.4;
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    group.setJointValueTarget(goal_state);
    group.move();

    sleep(3);

    move_group_interface::MoveGroup group1("finger2");
    group1.setPoseReferenceFrame("barrett_base_link");
    group1.setEndEffectorLink("finger_2_dist_link");
    joint_values[6] = 0.0;
    joint_values[7] = 2.4;
    joint_values[8] = 2.4;
    joint_model_group = robot_state.getJointModelGroup("finger2");
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
//    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    group1.setJointValueTarget(goal_state);
    group1.move();

    sleep(3);

    move_group_interface::MoveGroup group2("finger3");
    group2.setPoseReferenceFrame("barrett_base_link");
    group2.setEndEffectorLink("finger_3_dist_link");
    joint_values[6] = 0.85;
    joint_values[7] = 0.85;
    joint_values[8] = 0.85;
    joint_model_group = robot_state.getJointModelGroup("finger3");
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
//    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    group2.setJointValueTarget(goal_state);
    group2.move();

//        group.setPoseTarget(fixedPose);
        //group.setApproximateJointValueTarget(fixed_pose);
        //group.setJointValueTarget(fixed_pose);
        //-------------------------
//        group.setGoalPositionTolerance(0.5);
//        group.setGoalOrientationTolerance(0.5);
//        group.setApproximateJointValueTarget(fixedPose);
//        //group.setRandomTarget();
//        bool success = group.plan(plan);
//        if(success)
//        {
//            ROS_INFO("Moving to my Destination");
//            group.move();
//            ROS_INFO("Finished Moving");
//        }
//        else
//            ROS_INFO("Failure. No movement");
        ros::waitForShutdown();
       // r.sleep();
//    }
}

