#include <ros/ros.h>
#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/PointStamped.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "panda_goal_controller");
    ros::NodeHandle nh("/panda_goal_controller");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";
    static const std::string LOGNAME = "panda_goal_controller";

    ros::Duration(1.0).sleep();
    ROS_INFO_STREAM_NAMED(LOGNAME, "Starting MoveIt Panda Controller ...");
    
    moveit::planning_interface::MoveGroupInterface panda_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    panda_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    panda_group.setPlannerId("PTP");
    panda_group.setMaxVelocityScalingFactor(0.2);
    panda_group.setMaxAccelerationScalingFactor(0.1);

    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    const double pi = 3.141592653589793238;

    geometry_msgs::PoseStamped target;
    target.header.frame_id = "panda_link0";
    target.header.stamp = ros::Time::now();
    target.pose.position.x = 0.45;
    target.pose.position.y = 0;
    target.pose.position.z = 0.5;
    target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(180 * (pi / 180), 0 * (pi / 180), - 45 * (pi / 180));

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose p1 = target.pose;
    waypoints.push_back(p1);
    p1.position.z -= 0.2;
    waypoints.push_back(p1);
    p1.position.y -= 0.2;
    waypoints.push_back(p1);
    p1.position.z += 0.2;
    p1.position.y += 0.2;
    p1.position.x -= 0.2;
    waypoints.push_back(p1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = panda_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    panda_group.execute(trajectory);
    return 0;
}