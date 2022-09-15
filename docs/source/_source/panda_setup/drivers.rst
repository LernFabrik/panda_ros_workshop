Install Panda ROS Drivers
=========================

This chapter describes how to install `libfranka`` and `franka_ros`, either as binary packages or by building from source. 
franka_ros is only required if you want to control your robot using ROS.

For more detail follow this `link <https://frankaemika.github.io/docs/installation_linux.html#installation-on-linux>`_.

.. note:: 

    While libfranka and the franka_ros packages should work on different Linux distributions, official support is currently only provided for:
    
    Ubuntu 18.04 LTS Bionic Beaver and ROS Melodic Morenia (requires at least libfranka 0.6.0)
    Ubuntu 20.04 LTS Focal Fossa and ROS Noetic Ninjemys (requires at least libfranka 0.8.0)
    
    The following instructions are exemplary for Ubuntu 20.04 LTS system and ROS Noetic Ninjemys. They only work in the supported environments.


Installing from the ROS repositories
------------------------------------

Binary packages for libfranka and franka_ros are available from the ROS repositories. After setting up ROS Noetic, execute:

.. code-block:: bash

    sudo apt install ros-noetic-libfranka ros-noetic-franka-ros

Building from source (recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before building from source, please uninstall existing installations of libfranka and franka_ros to avoid conflicts:

.. code-block:: bash
    
    sudo apt remove "*libfranka*"

Building libfranka
___________________

To build **libfranka**, install the following dependencies from Ubuntu's package manager:

.. code-block:: bash

    sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

Then, download the source code by cloning libfranka from GitHub (https://github.com/frankaemika/libfranka).

For Panda you need to clone:

.. code-block:: bash

    git clone --recursive https://github.com/frankaemika/libfranka # only for panda
    cd libfranka

By default, this will check out the newest release of **libfranka**. If you want to build a particular version of **libfranka** instead, check out the 
corresponding Git tag:

.. code-block:: bash
    
    git checkout 0.9.2
    git submodule update

The above instructions for cloning libfranka only work for Panda. For Franka Research 3 you have to clone:

In the source directory, create a build directory and run CMake:

.. code-block:: bash 
    
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
    cmake --build .

Building the ROS packages
_________________________

After setting up ROS Noetic, create a Catkin workspace in a directory of your choice:


.. code-block:: bash
    
    cd /path/to/desired/folder
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    source /opt/ros/noetic/setup.sh
    catkin_init_workspace src

Then clone the franka_ros repository from GitHub:

.. code-block:: bash
    
    git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros

By default, this will check out the newest release of franka_ros. If you want to build a particular version of franka_ros instead, 
check out the corresponding Git tag:

.. code-block:: bash
    
    cd src/franka_ros
    git checkout noetic-devel

Install any missing dependencies and build the packages:

.. code-block:: bash

    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
    catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
    source devel/setup.sh

.. warning:: 

    If you also installed ros-noetic-libfranka, libfranka might be picked up from /opt/ros/noetic instead of from your custom libfranka build!