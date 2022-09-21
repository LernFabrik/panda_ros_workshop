#!/usr/bin/env python3

import imp
import sys
import math
from tokenize import group
import rospy
import moveit_commander
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose

import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


# Robot Positions
orientation = [180, 0.0, -45]
home_pose = [0.25, 0.0, 0.5]

pre_pick = [0.5, -0.2, 0.3]
pick = [0.5, -0.2, 0.2]

pre_place = [0.5, 0.2, 0.3]
place = [0.5, 0.2, 0.2]

# Gripper Position
open_gripper = GripperCommandGoal()
open_gripper.command.position = 0.02
open_gripper.command.max_effort = 0

close_gripper = GripperCommandGoal()
close_gripper.command.position = 0.01
close_gripper.command.max_effort = 5

def set_pose(xyz = [0.0, 0.0, 0.0], rpy=[0.0, 0.0, 0.0]):
    target = Pose()

    target.position.x = xyz[0]
    target.position.y = xyz[1]
    target.position.z = xyz[2]

    quad = quaternion_from_euler(math.radians(rpy[0]), math.radians(rpy[1]), math.radians(rpy[2]))
    
    target.orientation.x = quad[0]
    target.orientation.y = quad[1]
    target.orientation.z = quad[2]
    target.orientation.w = quad[3]

    return target


def target_execute(group, target):
    group.set_pose_target(target, end_effector_link="panda_link8")
    group.go()
    group.stop()
    group.clear_pose_targets()


def main():
    moveit_commander.roscpp_initialize(sys.argv)

    group = moveit_commander.MoveGroupCommander("panda_arm")
    group.set_max_velocity_scaling_factor(0.5)
    group.set_max_acceleration_scaling_factor(0.3)
    group.set_planning_pipeline_id('pilz_industrial_motion_planner')
    group.set_planner_id('PTP')

    gripper = actionlib.SimpleActionClient('franka_gripper/gripper_action', GripperCommandAction)
    gripper.wait_for_server()

    # Home Pose
    target_execute(group, set_pose(home_pose, orientation))
    # Pre -Pick Pose
    target_execute(group, set_pose(pre_pick, orientation))
    # Open Gripper
    gripper.send_goal(open_gripper)
    gripper.wait_for_result()
    # pick POse
    group.set_max_velocity_scaling_factor(0.5)
    group.set_planner_id('LIN')
    target_execute(group, set_pose(pick, orientation))
    # Close Gripper
    gripper.send_goal(close_gripper)
    gripper.wait_for_result()
    # Pre Pick Pose
    target_execute(group, set_pose(pre_pick, orientation))
    group.set_max_velocity_scaling_factor(0.5)
    group.set_planner_id('PTP')
    # Pre Place Pose
    target_execute(group, set_pose(pre_place, orientation))
    # Place Pose 
    target_execute(group, set_pose(place, orientation))
    # Open Gripper
    gripper.send_goal(open_gripper)
    gripper.wait_for_result()
    # Pre Place POse
    target_execute(group, set_pose(pre_place, orientation))
    # Home Pose
    target_execute(group, set_pose(home_pose, orientation))


if __name__ == '__main__':
    rospy.init_node('go_to_goal', anonymous=True)
    
    main()