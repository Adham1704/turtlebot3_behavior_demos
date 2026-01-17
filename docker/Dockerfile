# ROS distribution to use
ARG ROS_DISTRO=jazzy
ARG INSTALL_TB3=0

#######################################
# Base Image for TurtleBot Simulation #
#######################################
FROM ros:${ROS_DISTRO} AS base
ARG ROS_DISTRO
ARG INSTALL_TB3
ENV ROS_DISTRO=${ROS_DISTRO}

# ✅ GLOBAL ENVIRONMENT SETTINGS
ENV ROS_DOMAIN_ID=30
ENV TURTLEBOT3_MODEL=burger
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SHELL ["/bin/bash", "-c"]

# ------------------------------------
# System packages + tooling
# ------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl git lsb-release gnupg sudo \
    libcanberra-gtk-module libcanberra-gtk3-module \
    fuse3 libfuse2 \
    libqt5svg5-dev \
    python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine \
 && rm -rf /var/lib/apt/lists/*

# Additional Python modules
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-matplotlib \
    python3-transforms3d \
 && rm -rf /var/lib/apt/lists/*

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------
# Gazebo Harmonic (OSRF repo)
# ------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl lsb-release gnupg \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
    gz-harmonic \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------
# Nav2 + Cartographer
# ------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*


# ------------------------------------
# GUI Tools (RQT + RViz)
# ------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-rqt-robot-monitor \
    ros-${ROS_DISTRO}-rviz2 \
 && rm -rf /var/lib/apt/lists/*
# ------------------------------------
# Auto-source ROS
# ------------------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# ------------------------------------
# Create Colcon workspace
# ------------------------------------
RUN mkdir -p /turtlebot_ws/src
WORKDIR /turtlebot_ws/src
COPY dependencies.repos .
RUN vcs import < dependencies.repos

WORKDIR /turtlebot_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------
# OPTIONAL: Bake TurtleBot3 repos
# ------------------------------------
RUN if [ "${INSTALL_TB3}" = "1" ]; then \
      set -e; \
      mkdir -p /turtlebot3_ws/src; \
      cd /turtlebot3_ws/src; \
      git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git; \
      git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git; \
      git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git; \
      cd /turtlebot3_ws; \
      source /opt/ros/${ROS_DISTRO}/setup.bash; \
      colcon build --symlink-install; \
    fi

# ------------------------------------
# OPTIONAL: Download Groot2 AppImage
# Set INSTALL_GROOT=1 to enable. Default is 0 (skip).
# ------------------------------------
ARG INSTALL_GROOT=0
WORKDIR /root/
RUN if [ "${INSTALL_GROOT}" = "1" ]; then \
      echo "Installing Groot2..."; \
      curl -L --retry 10 --retry-delay 5 --connect-timeout 60 \
        -o Groot2.AppImage \
        https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage \
      && chmod a+x Groot2.AppImage; \
    else \
      echo "Skipping Groot2 installation (INSTALL_GROOT=0)"; \
    fi

# Set up the entrypoint
COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh  # ✅ VERIFY PERMISSION

ENTRYPOINT ["/entrypoint.sh"]

##########################################
# Overlay Image for TurtleBot Simulation #
##########################################
FROM base AS overlay
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws
COPY ./tb_autonomy/ ./src/tb_autonomy/
COPY ./tb_worlds/ ./src/tb_worlds/
RUN source /turtlebot_ws/install/setup.bash \
 && apt-get update -y \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install \
 && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh"]

#####################
# Development Image #
#####################
FROM overlay AS dev
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

RUN apt-get update && apt-get install -y --no-install-recommends \
    gdb gdbserver nano \
 && rm -rf /var/lib/apt/lists/*

# Handle existing ubuntu user
RUN if id -u ubuntu >/dev/null 2>&1; then \
      userdel -r ubuntu; \
    fi

# Create devuser
RUN groupadd --gid ${GID} ${USERNAME} \
 && useradd --uid ${UID} --gid ${GID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# ✅ FIX PERMISSIONS FOR DEVUSER
RUN chown -R ${USERNAME}:${USERNAME} /overlay_ws/ /turtlebot_ws/
RUN if [ -d "/turtlebot3_ws" ]; then chown -R ${USERNAME}:${USERNAME} /turtlebot3_ws/; fi

# Move Groot2 to new user's home directory ONLY if it exists
RUN groupadd -f fuse \
 && usermod -aG fuse ${USERNAME} \
 && if [ -f /root/Groot2.AppImage ]; then \
      mv /root/Groot2.AppImage /home/${USERNAME}/Groot2.AppImage \
      && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/Groot2.AppImage \
      && chmod +x /home/${USERNAME}/Groot2.AppImage; \
    fi

USER ${USERNAME}
# Automatically source the entrypoint for interactive shells
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc

WORKDIR /overlay_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]