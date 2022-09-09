FROM ros:noetic

RUN apt-get update && apt-get -y install \
    build-essential \
    wget \
    git \
    cmake\
    gazebo11 \
    python3-catkin-tools

# Setup user
RUN adduser --disabled-password --gecos '' docker && \
    groupadd -r -g 110 render && \
    adduser docker sudo && \
    adduser docker render && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Create ROS workspace
WORKDIR /home/docker
RUN mkdir -p ws/src
WORKDIR ws/src
ENV ROS_WORKSPACE=/home/docker/ws
RUN catkin init

# Add PX4
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx

# Install ROS dependencies
RUN rosdep update && apt-get update && \
    rosdep install --from-paths ${ROS_WORKSPACE} -r -y --ignore-src && \
    apt-get install -y ros-noetic-mavros

# RUN apt-get install -y python3-catkin-pkg ros-noetic-cmake-modules

SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/docker/.bashrc && \
    echo "source /home/docker/ws/devel/setup.bash" >> /home/docker/.bashrc

# Build
WORKDIR $ROS_WORKSPACE
RUN source /opt/ros/noetic/setup.bash && \
    catkin build

USER docker