#!/usr/bin/env python3

from http import client
import sys
from multiprocessing.connection import wait
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import MotionPlanRequest
from pilz_robot_programming import *
import tf
from geometry_msgs.msg import Pose

import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

world_frame = "world"
vel_scaling =  .3
home_position = [.5, -0.4, .4]
home_orientation = [.0, .0, .0, .1]

def set_pose(xyz = [0, 0, 0], q = [0, 0, 0, 1]):
	pose = Pose() 
	pose.position.x = xyz[0]
	pose.position.y = xyz[1]
	pose.position.z = xyz[2]
	pose.orientation.x = q[0]
	pose.orientation.y = q[1]
	pose.orientation.z = q[2]
	pose.orientation.w = q[3]
	return pose

def plan_and_execute(group, pose):
    group.set_pose_target(pose)

    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def main():
    moveit_commander.roscpp_initialize(sys.argv)

    group = moveit_commander.MoveGroupCommander("panda_arm")
    group.set_planning_pipeline_id('pilz_industrial_motion_planner')
    group.set_planner_id('PTP')
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.1)

    plan_and_execute(group, set_pose(home_position, home_orientation))

    client = actionlib.SimpleActionClient('franka_gripper/gripper_action', GripperCommandAction)
    client.wait_for_server()
    print('Connected to gripper servers')
    goal = GripperCommandGoal()
    goal.command.position = 0.02

    client.send_goal(goal=goal)
    client.wait_for_result()


if __name__ == '__main__':
	rospy.init_node('panda_goal', anonymous=True)

	main()

