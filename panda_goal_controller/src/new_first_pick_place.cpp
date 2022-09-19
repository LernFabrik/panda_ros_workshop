#include <ros/ros.h>
#include <memory>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

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
    
    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(nh);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto robot_start_state = planning_components->getStartState();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0", rvt::RVIZ_MARKER_TOPIC,
                                                      moveit_cpp_ptr->getPlanningSceneMonitor());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveItCpp Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    const double pi = 3.141592653589793238;

    geometry_msgs::PoseStamped target;
    target.header.frame_id = "panda_link0";
    target.header.stamp = ros::Time::now();
    target.pose.position.x = 0.25;
    target.pose.position.y = 0;
    target.pose.position.z = 0.5;
    target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(180 * (pi / 180), 0 * (pi / 180), - 45 * (pi / 180));

    planning_components->setStartStateToCurrentState();
    planning_components->setGoal(target, "panda_link8");

    auto plan = planning_components->plan();
    if(plan)
    {
        // Visualize the start pose in Rviz
        visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("panda_link8"), "start_pose");
        visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
        // Visualize the goal pose in Rviz
        visual_tools.publishAxisLabeled(target.pose, "target_pose");
        visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
        // Visualize the trajectory in Rviz
        visual_tools.publishTrajectoryLine(plan.trajectory, joint_model_group_ptr);
        visual_tools.trigger();

        planning_components->execute();
    }

    return 0;
}