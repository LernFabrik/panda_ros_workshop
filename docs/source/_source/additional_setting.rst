Additional MoveIt Configuration
===============================

Change the controller in `catkin_ws/src/panda_moveit_config/config/simple_moveit_controllers.yaml` file. 
You can follow the step given `here <https://github.com/ros-planning/panda_moveit_config/blob/noetic-devel/config/simple_moveit_controllers.yaml>`_.

.. code-block:: yaml
   :linenos:

   controller_list:
    - name: position_joint_trajectory_controller
      action_ns: follow_joint_trajectory
      type: FollowJointTrajectory
      default: True
      joints:
        - panda_joint1
        - panda_joint2
        - panda_joint3
        - panda_joint4
        - panda_joint5
        - panda_joint6
        - panda_joint7
    - name: franka_gripper
      action_ns: gripper_action
      type: GripperCommand
      default: True
      joints:
        - panda_finger_joint1


The controller name should match the controller spawers when you start your robot (Real or Simulation)

For Real Hardware: https://github.com/frankaemika/franka_ros/blob/a58d3052a241304392847df6464234c83d728a38/franka_control/launch/franka_control.launch#L22

Change from ::

   <node name="state_controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" args="franka_state_controller"/>

to ::

   <node name="state_controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" args="position_joint_trajectory_controller franka_state_controller"/>

For Simulation: https://github.com/frankaemika/franka_ros/blob/a58d3052a241304392847df6464234c83d728a38/franka_gazebo/launch/panda.launch#L14

Change from ::

    <arg name="controller"  default=" "     doc="Which example controller should be started? (One of {cartesian_impedance,model,force,joint_position,joint_velocity}_example_controller)" />

to ::

    <arg name="controller"  default="position_joint_trajectory_controller"     doc="Which example controller should be started? (One of {cartesian_impedance,model,force,joint_position,joint_velocity}_example_controller)" />

Add acceleration limits
------------------------

To use `pilz_industrial_motion` planners we need to modify `catkin_ws/src/panda_moveit_config/config/joint_limits.yaml` file as shown bellow. 

https://github.com/ros-planning/panda_moveit_config/blob/noetic-devel/config/joint_limits.yaml 

.. code-block:: yaml
    :linenos:

    joint_limits:
        panda_finger_joint1:
            has_velocity_limits: true
            max_velocity: 0.2
            has_acceleration_limits: true
            max_acceleration: 0
        panda_finger_joint2:
            has_velocity_limits: true
            max_velocity: 0.2
            has_acceleration_limits: true
            max_acceleration: 0
        panda_joint1:
            has_velocity_limits: true
            max_velocity: 2.175
            has_acceleration_limits: true
            max_acceleration: 3.75
        panda_joint2:
            has_velocity_limits: true
            max_velocity: 2.175
            has_acceleration_limits: true
            max_acceleration: 1.875
        panda_joint3:
            has_velocity_limits: true
            max_velocity: 2.175
            has_acceleration_limits: true
            max_acceleration: 2.5
        panda_joint4:
            has_velocity_limits: true
            max_velocity: 2.175
            has_acceleration_limits: true
            max_acceleration: 3.125
        panda_joint5:
            has_velocity_limits: true
            max_velocity: 2.61
            has_acceleration_limits: true
            max_acceleration: 3.75
        panda_joint6:
            has_velocity_limits: true
            max_velocity: 2.61
            has_acceleration_limits: true
            max_acceleration: 5
        panda_joint7:
            has_velocity_limits: true
            max_velocity: 2.61
            has_acceleration_limits: true
            max_acceleration: 5

Additional reference to URDF modifications
-------------------------------------------

Panda with table and camera: `Link <https://github.com/LernFabrik/franka_ros/blob/665a846bf45f7a8111d8f020ca3d6a32dcc07bbc/franka_description/robots/panda/panda.urdf.xacro>`_.
Change to gazebo launch: `Link <https://github.com/LernFabrik/franka_ros/blob/665a846bf45f7a8111d8f020ca3d6a32dcc07bbc/franka_gazebo/launch/panda.launch>`_
