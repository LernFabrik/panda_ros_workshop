ARG ROS_DISTRO=foxy

FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_FRONTEND=noninteractive
COPY bashrc_ros /etc/bash.bashrc
RUN chmod a+rwx /etc/bash.bashrc
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# upgrade current version
RUN apt-get update && apt-get -y upgrade && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    libpoco-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/frankaemika/libfranka.git --recursive && \
    cd libfranka && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF  .. && \
    cmake --build . -j$(nproc) && \
    cpack -G DEB && \
    dpkg -i libfranka-*.deb

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-control-msgs \
    ros-foxy-xacro \
    ros-foxy-angles \
    ros-foxy-ros2-control \
    ros-foxy-realtime-tools \
    ros-foxy-control-toolbox \
    ros-foxy-moveit \
    ros-foxy-ros2-controllers \
    ros-foxy-joint-state-publisher \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-ament-cmake-clang-format \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Final Steps
ARG USERNAME=admin
ARG USER_UID=1000
ARG USER_GID=1000

# Create the 'admin' user if not already exists
RUN if [ ! $(getent passwd ${USERNAME}) ]; then \
        groupadd --gid ${USER_GID} ${USERNAME} ; \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} ; \
    fi

# Update 'admin' user
RUN echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && adduser ${USERNAME} video && adduser ${USERNAME} sudo

# Copy scripts
RUN mkdir -p /usr/local/bin/scripts
COPY *entrypoint.sh /usr/local/bin/scripts/
RUN  chmod +x /usr/local/bin/scripts/*.sh

WORKDIR /home/${USERNAME}

ENV USERNAME=${USERNAME}
ENV USER_GID=${USER_GID}
ENV USER_UID=${USER_UID}

# Panda Packages
RUN apt update && mkdir -p colcon_panda_ws/src && cd colcon_panda_ws && \
    source ${ROS_ROOT}/setup.bash && \
    git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2 && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 