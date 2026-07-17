FROM ghcr.io/cyberbotics/webots-docker/webots:r2025a-ubuntu22.04-ubuntu

ENV DEBIAN_FRONTEND=noninteractive

#################################################
# Add ROS 2 Humble repository & Enable Universe
#################################################

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    apt-transport-https \
    && add-apt-repository -y universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list


#################################################
# Install basic tools + ROS dependencies
#################################################

RUN apt-get update && apt-get install -y \
    sudo \
    wget \
    git \
    xterm \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pandas \
    python3-numpy \
    python3-matplotlib \
    python3-psutil \
    python3-pyqt5 \
    mesa-utils \
    \
    ros-humble-ros-base \
    ros-humble-webots-ros2 \
    ros-humble-webots-ros2-driver \
    ros-humble-robot-localization \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rclpy \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-rcl-interfaces \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*


#################################################
# Initialize rosdep
#################################################

RUN rosdep init || true


#################################################
# Create non-root user
#################################################

ARG USERNAME=robot
ARG UID=1000
ARG GID=1000

RUN groupadd -g ${GID} ${USERNAME} && \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers


#################################################
# Switch user
#################################################

USER ${USERNAME}


#################################################
# Workspace
#################################################

WORKDIR /home/${USERNAME}/ros2_ws

RUN mkdir -p src


#################################################
# Copy ROS package
#################################################

COPY --chown=${USERNAME}:${USERNAME} ros2_ws/src ./src


#################################################
# Install package dependencies
#################################################

# Sourced keys are updated, apt lists refreshed, and redundant python3-pyqt5 resolution skipped
RUN bash -c "source /opt/ros/humble/setup.bash && \
    rosdep update && \
    sudo apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys='python3-pyqt5 pyqt5-dev python3-pyqt5.qtsvg python3-sip-dev'"


#################################################
# Build workspace
#################################################

RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"


#################################################
# Auto source ROS
#################################################

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc


#################################################
# Default directory
#################################################

WORKDIR /home/${USERNAME}/ros2_ws

CMD ["/bin/bash"]
