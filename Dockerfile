FROM autodriveecosystem/autodrive_roboracer_api:2026-icra-practice

# 1. Stay as root for installations
USER root

# 2. Define your IDs
ARG USER_ID=1000
ARG GROUP_ID=1000

# 3. Create the user (but don't switch yet)
RUN groupadd -g $GROUP_ID novauser && \
    useradd -u $USER_ID -g $GROUP_ID -m -s /bin/bash novauser && \
    echo "novauser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update && apt-get install -y \
    ros-humble-ackermann-msgs \
    ros-humble-rqt-gui \
    ros-humble-rqt-common-plugins \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    wget \
    jq x11-apps \
    python3-pip \
    python3-colcon-clean \
    apt-utils \
    net-tools \
    joystick \
    ros-humble-asio-cmake-module \
    ros-humble-diagnostic-updater \
    ros-humble-robot-localization \
    ros-humble-tf-transformations \
    ros-humble-cartographer-ros \
    ros-humble-urg-node \
    ros-humble-ackermann-msgs \
    ros-humble-control-msgs \
    ros-humble-rosbridge-server \
    ros-humble-serial-driver \
    ros-humble-test-msgs \
    ros-humble-rqt \
    ros-humble-xacro \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-rmw-zenoh-cpp \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# 5. Set up the workspace
WORKDIR /home/ros2_ws
COPY ./src ./src

# 6. Build the code (root can build everything)
RUN . /opt/ros/humble/setup.sh && colcon build

# 7. NOW switch to your user for the final environment
RUN chown -R novauser:novauser /home/ros2_ws
USER novauser

# Automatically source ROS 2 for novauser
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# If the devkit has its own setup, source that too
# Check if this path exists in your specific image
RUN echo "source /home/autodrive_devkit/install/setup.bash" >> ~/.bashrc

# Source your own Nova workspace
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /home/novauser
RUN mkdir -p /home/novauser/ws/src
RUN git clone https://github.com/AbhayAysola/race_stack -b ros2-humble --recurse-submodules ./ws/src/race_stack
RUN /bin/bash /home/novauser/ws/src/race_stack/.install_utils/post_create_command.sh
