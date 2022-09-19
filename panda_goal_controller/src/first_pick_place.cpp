#include <ros/ros.h>
#include <memory>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>
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

    const double pi = 3.141592653589793238;

    geometry_msgs::PoseStamped target;
    target.header.frame_id = "panda_link0";
    target.header.stamp = ros::Time::now();
    target.pose.position.x = 0.25;
    target.pose.position.y = 0;
    target.pose.position.z = 0.5;
    target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(180 * (pi / 180), 0 * (pi / 180), - 45 * (pi / 180));

    panda_group.setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode eCode = panda_group.plan(plan);
    ROS_INFO("Motion Planning is: %s", eCode?"Success":"Failed");

    if(eCode)
    {
        panda_group.execute(plan);
    }
    return 0;
}