#!/bin/bash

# Build ROS dependency
echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

sudo apt-get update
rosdep update

$@