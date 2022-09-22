Robot Tasks
===========

Move to Marker
--------------

To recognize the marker we need package called `ar_track_alvar`. ::

    cd ~/catkin_ws/src
    git clone https://github.com/ROSinTraining/ar_track_alvar.git
    cd .. 
    catkin_make

Create launch file called `ar.launch` inside `franka_gazebo/launch`.

.. code-block:: xml
    :linenos:

    <?xml version="1.0"?>
    <launch>
        <arg name="marker_size" default="11.5" />
        <arg name="max_new_marker_error" default="0.08" />
        <arg name="max_track_error" default="0.4" />

        <arg name="cam_image_topic" default="/camera/rgb/image_raw" />
        <arg name="cam_info_topic" default="/camera/rgb/camera_info" />
        <arg name="output_frame" default="/camera_link" />

        <node name="ar_track_alvar" pkg="ar_track_alvar" type="individualMarkersNoKinect" respawn="false" output="screen">
            <param name="marker_size"           type="double" value="$(arg marker_size)" />
            <param name="max_new_marker_error"  type="double" value="$(arg max_new_marker_error)" />
            <param name="max_track_error"       type="double" value="$(arg max_track_error)" />
            <param name="output_frame"          type="string" value="$(arg output_frame)" />

            <remap from="camera_image"  to="$(arg cam_image_topic)" />
            <remap from="camera_info"   to="$(arg cam_info_topic)" />
        </node>
    </launch>

Finally code for move to marker. ::

    cd catkin_ws/src
    catkin_create_pkg panda_controllers rospy
    cd .. 
    catkin_make
    mkdir -p src/panda_controllers/scripts
    touch src/panda_controllers/scripts/move_to_marker.py
    chmod +x src/panda_controllers/scripts/move_to_marker.py

Add the code to `move_to_marker.py`.

.. code-block:: python
    :linenos:

    #!/usr/bin/env python3

    import sys
    import rospy
    import moveit_commander
    import tf
    import tf2_ros
    from geometry_msgs.msg import Pose
    from ar_track_alvar_msgs.msg import AlvarMarkers

    #declare global variables
    world_frame = "panda_link0"
    vel_scaling =  .3
    home_position = [.4, .0, .3]
    home_orientation = [.1, .0, .0, .0]
    marker_pose = None

    #convert marker from camera frame to robot's base frame
    def transform_pose(pose, target_frame):
        if tf_listener.canTransform(target_frame, pose.header.frame_id, rospy.Time(0)):
            #transform pose
            transform = tf_listener.transformPose(target_frame, pose)

            # try:
            #     tran = tf2_ros.Buffer.lookup_transform("world", "ar_maker_1", rospy.Time(), rospy.Duration(10))

            return transform.pose

    #callback function to receive marker messages
    def marker_cb(msg):
        global marker_pose
        if len(msg.markers) == 0:
            return
        marker = msg.markers[0]
        marker.pose.header.frame_id = marker.header.frame_id
        marker_pose = transform_pose(marker.pose, "panda_link0")

    #set Pose message through lists
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

    #confirm if plan should be executed
    def plan_accepted():
        return input("Do you want to execute the plan [y] or replan [n]? ") == "y"

    #plan and execute to given pose; If plan is not confirmed plan again
    def plan_and_execute(group, pose):
        group.set_pose_target(pose)
        if plan_accepted():
            group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
        else:
            exit()

    #main function of application
    def main():
        #initialize moveit
        moveit_commander.roscpp_initialize(sys.argv)

        # Your Code For Task 1 #
        group = moveit_commander.MoveGroupCommander("panda_arm")
        group.set_max_velocity_scaling_factor(vel_scaling)


        #while loop to move the robot to the found AR marker
        while not rospy.is_shutdown():
            plan_and_execute(group, set_pose(home_position, home_orientation))
            if marker_pose:
                marker = [marker_pose.position.x, marker_pose.position.y, home_position[2]]
                plan_and_execute(group, set_pose(marker))
            else:
                rospy.logwarn("No marker detected.")

    if __name__ == '__main__':
        rospy.init_node('move_to_marker', anonymous=True)
        tf_listener = tf.TransformListener()

        # Your Code for Task 2 #
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, marker_cb)

        main()

Run the task. Please use new terminals for the every command.

.. warning:: 

    Do not forget to `source` your catkin workspace.

.. code-block:: bash

    roslaunch franka_gazebo panda.launch 
    roslaunch panda_moveit_config move_group.launch
    roslaunch franka_gazebo ar.launch
    rosrun panda_controllers move_to_marker.py


Go to Goal (Real Robot)
-----------------------

.. hint:: 

    Please remember if your using different system for controlling robot, add `export ROS_MASTER_URI=http://<ip>:11311` and `export ROS_IP=<ip>`.

Refere to `Getting Started <https://frankaemika.github.io/docs/getting_started.html>`_ for setting up your robot. ::
    
    roslaunch franka_control f^Cnka_control.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda
    roslaunch panda_moveit_config move_group.launch

Create `go_to_goal.py` inside `panda_controllers` package.

.. code-block:: python
    :linenos:

    #!/usr/bin/env python3

    import sys
    from tokenize import group
    import rospy
    import moveit_commander
    import tf
    import tf2_ros
    from geometry_msgs.msg import Pose


    def main():
        moveit_commander.roscpp_initialize(sys.argv)

        group = moveit_commander.MoveGroupCommander("panda_arm")
        group.set_max_velocity_scaling_factor(0.5)
        group.set_max_acceleration_scaling_factor(0.3)
        group.set_planning_pipeline_id('pilz_industrial_motion_planner')
        group.set_planner_id('PTP')


        target = Pose()
        target.position.x = 0.4
        target.position.y = 0.2
        target.position.z = 0.5
        target.orientation.x = 1
        target.orientation.y = 0
        target.orientation.z = 0
        target.orientation.w = 0

        target1 = Pose()
        target1.position.x = 0.4
        target1.position.y = - 0.2
        target1.position.z = 0.5
        target1.orientation.x = 1
        target1.orientation.y = 0
        target1.orientation.z = 0
        target1.orientation.w = 0

        while not rospy.is_shutdown():
            group.set_pose_target(target, end_effector_link="panda_link8")
            group.go()
            group.stop()
            group.clear_pose_targets()

            group.set_pose_target(target1, end_effector_link="panda_link8")
            group.go()
            group.stop()
            group.clear_pose_targets()


    if __name__ == '__main__':
        rospy.init_node('go_to_goal', anonymous=True)
        
        main()


.. warning:: 

    Carefully test the robot position so that it do not collide with its environment. Also set ::

        group.set_max_velocity_scaling_factor(0.5)
        group.set_max_acceleration_scaling_factor(0.3)
    
    low for the testing.
