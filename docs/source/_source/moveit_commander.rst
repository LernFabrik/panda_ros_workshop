Move Group Python interface
===========================

Follow MoveIt tutorial on Python programming `here <https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html>`_.

.. code-block:: python
    :linenos:

    #!/usr/bin/env python3

    import sys
    import math
    import sys
    import rospy
    import moveit_commander
    from tf.transformations import quaternion_from_euler
    from geometry_msgs.msg import Pose

    import actionlib
    from control_msgs.msg import GripperCommandAction, GripperCommandGoal

    world_frame = "world"
    vel_scaling =  .3
    home_position = [.25, 0, .5]
    home_orientation = [180, .0, -45]

    # Gripper
    gripper_open = GripperCommandGoal()
    gripper_open.command.position = 0.0212
    gripper_open.command.max_effort = 0

    gripper_close = GripperCommandGoal()
    gripper_close.command.position = 0.02
    gripper_close.command.max_effort = 10

    def set_pose(xyz = [0, 0, 0], q = [0, 0, 0]):
        pose = Pose() 
        pose.position.x = xyz[0]
        pose.position.y = xyz[1]
        pose.position.z = xyz[2]
        quad = quaternion_from_euler(math.radians(q[0]), math.radians(q[1]), math.radians(q[2]))
        pose.orientation.x = quad[0]
        pose.orientation.y = quad[1]
        pose.orientation.z = quad[2]
        pose.orientation.w = quad[3]
        return pose

    def plan_and_execute(group, pose):
        group.set_pose_target(pose)

        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

    def main():
        moveit_commander.roscpp_initialize(sys.argv)

        #Setup robot arm planner
        group = moveit_commander.MoveGroupCommander("panda_arm")
        group.set_planning_pipeline_id('pilz_industrial_motion_planner')
        group.set_planner_id('PTP')
        group.set_max_velocity_scaling_factor(0.2)
        group.set_max_acceleration_scaling_factor(0.1)

        #Gripper Client
        gripper = actionlib.SimpleActionClient('franka_gripper/gripper_action', GripperCommandAction)
        gripper.wait_for_server()

        plan_and_execute(group, set_pose(home_position, home_orientation))
        gripper.send_goal(goal=gripper_open)
        gripper.wait_for_result()
        gripper.send_goal(goal=gripper_close)
        gripper.wait_for_result()

    if __name__ == '__main__':
        rospy.init_node('panda_goal', anonymous=True)

        main()