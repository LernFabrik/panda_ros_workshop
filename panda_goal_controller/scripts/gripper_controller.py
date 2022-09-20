#!/usr/bin/env python3

import sys
import math
import sys
import rospy
import moveit_commander

from actionlib import SimpleActionClient
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


def main():

    client = SimpleActionClient("franka_gripper/gripper_action", GripperCommandAction)
    client.wait_for_server()

    print("Server is up!")

    goal = GripperCommandGoal()

    goal.command.position = 0.04
    goal.command.max_effort = 0

    client.send_goal(goal)
    client.wait_for_result()

    goal.command.position = 0.00
    goal.command.max_effort = 0

    client.send_goal(goal)
    client.wait_for_result()


if __name__ == '__main__':
	rospy.init_node('panda_goal', anonymous=True)

	main()