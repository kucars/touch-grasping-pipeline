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
    ros::Rate r(10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    // start a ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // this connecs to a running instance of the move_group node
    move_group_interface::MoveGroup group("barrett_hand");
    group.setEndEffectorLink("finger_3_med_link");
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference end effector: %s", group.getEndEffectorLink().c_str());
    group.setPlanningTime(30.0);
    group.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);
    /*
    geometry_msgs::Pose fixed_pose;
    geometry_msgs::Quaternion quat_msg;
    fixed_pose.position.x = 0.72959;
    fixed_pose.position.y = -0.49258;
    fixed_pose.position.z = 0.24919;
    quat_msg.x = 0.78338;
    quat_msg.y = 0.10916;
    quat_msg.z = 0.30131;
    quat_msg.w = 0.53255;
    fixed_pose.orientation = quat_msg;
    */
//    float positionx[16]={0.76601,1.0618, 0.72959,0.82633,0.81688,0.79482,0.82561,0.85233,0.85823, 0.84924,0.84145,0.80744, 0.83643,0.91167,0.76203,0.92631};
//    float positiony[16]={-0.16509,0.089273,-0.49258,-5.4058e-06,-0.09412, -0.16984,-0.18168,-0.084054,0.033804,0.10065,0.1336,0.12433,0.16757,-0.0374,-0.12664,0.1364};
//    float positionz[16]={0.26517,0.48622,0.24919,0.24769,0.24767,0.24769,0.29213,0.29221,0.2922,0.29219, 0.29219,0.26394,0.28019,0.33795,0.29291, 0.35577};
//    float quatx[16]={0.4225,0.4225,0.78338,-0.071466, -0.00091693, 0.057936,0.10371,0.03338, 0.049068,0.095922,0.1192,0.22109,-0.095911, -0.14986,0.02758,-0.047177};
//    float quaty[16]={0.88506,0.88506,0.10916,0.70349,0.70711, 0.70473,0.69946,0.70632,-0.7054,-0.70057,-0.69699,-0.67165,0.70057,-0.69104,0.70657,0.70553};
//    float quatz[16]={-0.10551,-0.10551,0.30131,-0.70349,-0.69283,-0.67866,-0.69015,-0.70207,0.7071,0.70567, 0.70376,0.70618,-0.69339,0.6995,-0.68318,-0.69732};
//    float quatw[16]={-0.16441,-0.16441,0.53255,-0.071474,-0.14136,-0.19853,-0.15393,-0.084216,0.001914,-0.045118,-0.068728,0.036208,0.13859,-0.10341,-0.1824,0.11726};
    geometry_msgs::Pose fixedPose;
    moveit::planning_interface::MoveGroup::Plan plan;
    visualization_msgs::Marker marker;
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::ARROW;
//    uint poseIndex = 0;
    // Set the marker type. Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
    marker.scale.x = 0.30;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    // Set the namespace and id for this marker. This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;
//    while (ros::ok())
//    {
//        ROS_INFO("Attempting to move to position: %d", poseIndex);
        fixedPose.position.x = -0.025825;
        fixedPose.position.y = 0.00124727;
        fixedPose.position.z = 1.306;
        fixedPose.orientation.x = -0.000931395;
        fixedPose.orientation.y = 0.000679287;
        fixedPose.orientation.z = -0.721889;
        fixedPose.orientation.w = 0.692008;
//        if (++poseIndex>15)
//            poseIndex= 0;
        ROS_INFO("Publishing Marker");
        // Set the frame ID and timestamp. See the TF tutorials for information on these.
//        marker.pose = fixedPose;
//        marker.header.frame_id = "odom";
//        marker.header.stamp = ros::Time::now();
//        marker.lifetime = ros::Duration();
        // Publish the marker
//        marker_pub.publish(marker);
        /*
        robot_state::RobotState start_state(*group.getCurrentState());
        geometry_msgs::Pose start_pose2;
        start_pose2.orientation.w = 1.0;
        start_pose2.position.x = 0.55;
        start_pose2.position.y = -0.05;
        start_pose2.position.z = 0.8;
        const robot_state::JointModelGroup *joint_model_group =
        start_state.getJointModelGroup(group.getName());
        start_state.setFromIK(joint_model_group, start_pose2);
        group.setStartState(start_state);
        */
        group.setPoseTarget(fixedPose);
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
           // group.move();
            ROS_INFO("Finished Moving");
        }
        else
            ROS_INFO("Failure. No movement");
        ros::waitForShutdown();
       // r.sleep();
//    }
}
